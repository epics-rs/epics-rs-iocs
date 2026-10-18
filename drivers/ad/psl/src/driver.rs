//! The PSL areaDetector driver (C `PSL`).
//!
//! Ownership: the PSLViewer socket lives behind [`PslServer`], which serialises
//! every exchange; the choice sets and the camera count are plain fields of the
//! driver, so only the port actor ever reads or writes them. The acquisition
//! task reaches the parameter library through the actor and the server through
//! its own mutex — see the invariant in [`crate::connection`].

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::ndarray_driver::check_path_str;
use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;

use crate::connection::{PslServer, ServerResult};
use crate::params::PslParams;
use crate::protocol::{self, Choices};
use crate::task::{AcquisitionContext, start_acquisition_task};

/// The acquisition task's command channel.
pub(crate) enum TaskCommand {
    Start,
}

/// State shared between the port actor and the acquisition task.
pub struct SharedState {
    /// Set when Acquire goes to 0 during an acquisition; the task clears it
    /// when it starts the next one. Only the actor sets it, only the task
    /// clears it (C used an `epicsEvent` both threads could consume).
    pub stop_requested: AtomicBool,
}

/// What the server told us about itself. Only the port actor touches this.
#[derive(Default)]
struct DeviceState {
    /// How many sub-cameras the open configuration drives; >1 wraps every
    /// reply in a Python list.
    n_cameras: i32,
    valid_options: Choices,
    camera_names: Choices,
    trigger_modes: Choices,
    record_formats: Choices,
}

impl DeviceState {
    fn multi_camera(&self) -> bool {
        self.n_cameras > 1
    }
}

pub struct PslDetector {
    pub ad: ADDriverBase,
    pub params: PslParams,
    server: PslServer,
    state: DeviceState,
    shared: Arc<SharedState>,
    commands: rt::CommandSender<TaskCommand>,
}

impl PslDetector {
    fn new(
        port_name: &str,
        server: PslServer,
        max_memory: usize,
        shared: Arc<SharedState>,
        commands: rt::CommandSender<TaskCommand>,
    ) -> AsynResult<(Self, bool)> {
        let mut ad = ADDriverBase::new(port_name, 1, 0, 0, max_memory, ASYN_CANBLOCK)?;
        // C PSL never sets NDDriverVersion: it keeps asynNDArrayDriver's
        // ADCore version string.
        let params = PslParams::create(&mut ad.port_base)?;

        let mut det = Self {
            ad,
            params,
            server,
            state: DeviceState {
                n_cameras: 1,
                ..Default::default()
            },
            shared,
            commands,
        };
        let booted = det.connect_to_server()?;
        Ok((det, booted))
    }

    /// The rest of C's constructor, from getVersion on. False when the
    /// server version is wrong: C returns there (PSL.cpp:168-173), leaving
    /// the defaults unset and PSLTask not started, and the port stays up.
    fn connect_to_server(&mut self) -> AsynResult<bool> {
        let reply = self.command("GetVersion")?;
        if protocol::parse_version(&reply).is_err() {
            self.server.trace_error(&format!(
                "PSL:getVersion, ERROR, unexpected version string = {reply}.\n"
            ));
            self.trace_error("PSL:PSL: ERROR, incorrect server version\n");
            return Ok(false);
        }

        self.state.camera_names = self.get_choices("GetCamList")?;
        // The multi-camera configuration is not in GetCamList.
        self.state.camera_names.insert("multiconf".into());
        // C's constructor ignores what openCamera returns (PSL.cpp:180).
        let _ = self.open_camera(0);

        let ad = &self.ad.params;
        let base = &mut self.ad.port_base;
        base.set_string_param(ad.base.manufacturer, 0, "PSL")?;
        base.set_string_param(ad.base.model, 0, "CCD")?;
        base.set_int32_param(ad.base.data_type, 0, NDDataType::UInt16 as u8 as i32)?;
        base.set_int32_param(ad.image_mode, 0, ImageMode::Single as i32)?;
        base.set_float64_param(ad.acquire_period, 0, 0.0)?;
        base.set_float64_param(ad.acquire_time, 0, 1.0)?;
        base.set_int32_param(ad.num_images, 0, 1)?;
        Ok(true)
    }

    /// Send one command and publish both sides of the exchange to the
    /// StringToServer / StringFromServer records (C `writeReadServer`).
    fn command(&mut self, command: &str) -> AsynResult<String> {
        Ok(self.command_status(command)?.0)
    }

    /// [`Self::command`] for a caller that returns C's `writeReadServer` status.
    fn send(&mut self, command: &str) -> AsynResult<()> {
        self.command_status(command)?.1.map_err(AsynError::from)
    }

    /// [`Self::command`] with the outcome C's `writeReadServer` returned.
    fn command_status(&mut self, command: &str) -> AsynResult<(String, ServerResult<()>)> {
        let (reply, status) = match self.server.command(command) {
            Ok(reply) => (reply, Ok(())),
            Err(e) => (String::new(), Err(e)),
        };
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.string_to_server, 0, command)?;
        base.set_string_param(self.ad.params.string_from_server, 0, reply.clone())?;
        Ok((reply, status))
    }

    /// C `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, ...)`.
    fn trace_error(&self, message: &str) {
        self.ad
            .port_base
            .trace_print(TraceMask::ERROR, format_args!("{}", message));
    }

    fn get_choices(&mut self, command: &str) -> AsynResult<Choices> {
        let multi = self.state.multi_camera();
        let (reply, status) = self.command_status(command)?;
        if let Err(e) = &status {
            self.trace_error(&format!(
                "PSL::getChoices error in write/read to server {}\n",
                e.status()
            ));
        }
        let choices = protocol::parse_choices(&reply, multi);
        if choices.is_empty() {
            self.trace_error("PSL::getChoices Options are NULL\n");
        }
        Ok(choices)
    }

    /// Open one of the cameras `GetCamList` offered (C `openCamera`).
    fn open_camera(&mut self, index: i32) -> AsynResult<()> {
        let name = match protocol::choice_from_index(&self.state.camera_names, index) {
            Ok(name) => name.to_string(),
            // C bug not copied: getChoiceFromIndex (PSL.cpp:275-282)
            // dereferences end() for an out-of-range index, as the init-time
            // CameraName=0 write does on a port whose GetCamList never ran;
            // C then publishes an uninitialised ADModel.
            Err(_) => return Ok(()),
        };

        self.command("Close")?;
        self.command(&protocol::open(&name))?;
        self.command(&protocol::select(&name))?;

        let reply = self.command("GetCamNum")?;
        self.state.n_cameras = protocol::parse_ints(&reply).first().copied().unwrap_or(1);

        let model = self.command("GetCam")?;
        let model = model.split_whitespace().next().unwrap_or("").to_string();
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.base.model, 0, model)?;
        base.set_string_param(self.ad.params.base.manufacturer, 0, "PSL")?;

        // C returns this exchange's status (PSL.cpp:230).
        let (reply, status) = self.command_status("GetMaximumSize")?;
        let size = protocol::parse_ints(&reply);
        if size.len() >= 2 {
            let base = &mut self.ad.port_base;
            base.set_int32_param(self.ad.params.max_size_x, 0, size[0])?;
            base.set_int32_param(self.ad.params.max_size_y, 0, size[1])?;
        }

        self.state.valid_options = self.get_choices("GetOptions")?;
        self.state.record_formats = self.get_choices("GetOptionRange;RecordFormat")?;
        self.state.trigger_modes = self.get_choices("GetOptionRange;TriggerMode")?;

        let _ = self.get_config();
        self.ad.port_base.call_param_callbacks(0)?;
        status.map_err(AsynError::from)
    }

    fn supports(&self, option: &str) -> bool {
        self.state.valid_options.contains(option)
    }

    /// Read the whole configuration back from the server (C `getConfig`).
    fn get_config(&mut self) -> AsynResult<()> {
        let multi = self.state.multi_camera();

        let reply = self.command("GetSize")?;
        let size = protocol::parse_ints(protocol::numeric_field(&reply, multi));
        if size.len() >= 2 {
            let base = &mut self.ad.port_base;
            base.set_int32_param(self.ad.params.base.array_size_x, 0, size[0])?;
            base.set_int32_param(self.ad.params.base.array_size_y, 0, size[1])?;
        }

        // GetMode is always supported.
        let (reply, status) = self.command_status("GetMode")?;
        match protocol::parse_mode(protocol::quoted_field(&reply, multi)) {
            Ok((data_type, color_mode)) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(self.ad.params.base.data_type, 0, data_type as u8 as i32)?;
                base.set_int32_param(self.ad.params.base.color_mode, 0, color_mode as i32)?;
            }
            Err(_) if status.is_ok() => {
                self.trace_error(&format!("PSL:getConfig: unknown mode={reply}\n"));
            }
            Err(_) => {}
        }

        if self.supports("TriggerMode") {
            let (reply, status) = self.command_status("GetTriggerMode")?;
            let mode = protocol::quoted_field(&reply, multi).to_string();
            match protocol::index_of_choice(&self.state.trigger_modes, &mode) {
                Some(index) => {
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.trigger_mode, 0, index)?;
                }
                None if status.is_ok() => {
                    self.trace_error(&format!("PSL:getConfig: unknown trigger mode={mode}\n"));
                }
                None => {}
            }
        }

        if self.supports("Exposure") {
            let reply = self.command("GetExposure")?;
            if let Some(seconds) = protocol::parse_exposure(protocol::numeric_field(&reply, multi))
            {
                self.ad
                    .port_base
                    .set_float64_param(self.ad.params.acquire_time, 0, seconds)?;
            }
        }

        if self.supports("ChipGain") {
            let reply = self.command("GetChipGain")?;
            if let Some(gain) = protocol::parse_first_f64(protocol::numeric_field(&reply, multi)) {
                self.ad
                    .port_base
                    .set_float64_param(self.ad.params.gain, 0, gain)?;
            }
        }

        let base = &mut self.ad.port_base;
        base.set_int32_param(self.ad.params.bin_x, 0, 1)?;
        base.set_int32_param(self.ad.params.bin_y, 0, 1)?;
        if self.supports("Binning") {
            let reply = self.command("GetBinning")?;
            let bin = protocol::parse_ints(protocol::numeric_field(&reply, multi));
            if bin.len() >= 2 {
                let base = &mut self.ad.port_base;
                base.set_int32_param(self.ad.params.bin_x, 0, bin[0])?;
                base.set_int32_param(self.ad.params.bin_y, 0, bin[1])?;
            }
        }

        if self.supports("SubArea") {
            let reply = self.command("GetSubArea")?;
            let area = protocol::parse_ints(protocol::numeric_field(&reply, multi));
            if area.len() >= 4 {
                let (min_x, min_y, right, bottom) = (area[0], area[1], area[2], area[3]);
                let base = &mut self.ad.port_base;
                base.set_int32_param(self.ad.params.min_x, 0, min_x)?;
                base.set_int32_param(self.ad.params.min_y, 0, min_y)?;
                base.set_int32_param(self.ad.params.size_x, 0, right - min_x + 1)?;
                base.set_int32_param(self.ad.params.size_y, 0, bottom - min_y + 1)?;
            }
        }

        if self.supports("Fliplr") {
            let reply = self.command("GetFliplr")?;
            if let Some(v) = protocol::parse_ints(protocol::numeric_field(&reply, multi)).first() {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.reverse_x, 0, *v)?;
            }
        }

        if self.supports("Flipud") {
            let reply = self.command("GetFlipud")?;
            if let Some(v) = protocol::parse_ints(protocol::numeric_field(&reply, multi)).first() {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.reverse_y, 0, *v)?;
            }
        }

        if self.supports("RecordPath") {
            let reply = self.command("GetRecordPath")?;
            let path = protocol::quoted_field(&reply, multi).to_string();
            self.ad
                .port_base
                .set_string_param(self.ad.params.base.file_path, 0, path)?;
        }

        if self.supports("RecordName") {
            let reply = self.command("GetRecordName")?;
            let name = protocol::quoted_field(&reply, multi).to_string();
            self.ad
                .port_base
                .set_string_param(self.ad.params.base.file_name, 0, name)?;
        }

        if self.supports("RecordNumber") {
            let reply = self.command("GetRecordNumber")?;
            if let Some(n) = protocol::parse_ints(protocol::numeric_field(&reply, multi)).first() {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.base.file_number, 0, *n)?;
            }
        }

        if self.supports("RecordFormat") {
            let (reply, status) = self.command_status("GetRecordFormat")?;
            let format = protocol::quoted_field(&reply, multi).to_string();
            match protocol::index_of_choice(&self.state.record_formats, &format) {
                Some(index) => {
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.base.file_format, 0, index)?;
                }
                None if status.is_ok() => {
                    self.trace_error(&format!(
                        "PSL:getConfig: error, unknown file format string = {format}\n"
                    ));
                }
                None => {}
            }
        }

        if self.supports("RecordTag") {
            let reply = self.command("GetRecordTag")?;
            let tag = protocol::quoted_field(&reply, multi).to_string();
            self.ad
                .port_base
                .set_string_param(self.params.tiff_comment, 0, tag)?;
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    /// Report whether the directory the server saves into exists here as well
    /// (C `asynNDArrayDriver::checkPath`, asynNDArrayDriver.cpp:98-111): an
    /// empty path is left alone, any other is stored with its separator.
    fn check_path(&mut self) -> AsynResult<()> {
        let p = self.ad.params.base;
        let mut path = String::from_utf8_lossy(self.ad.port_base.get_string_param(p.file_path, 0)?)
            .into_owned();
        if path.is_empty() {
            return Ok(());
        }
        let exists = check_path_str(&mut path);
        self.ad.port_base.set_string_param(p.file_path, 0, path)?;
        self.ad
            .port_base
            .set_int32_param(p.file_path_exists, 0, exists as i32)?;
        Ok(())
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    /// Tell the server to expose, then wake the acquisition task
    /// (C `startAcquire`).
    fn start_acquire(&mut self) -> AsynResult<()> {
        let image_mode = ImageMode::from_i32(self.get_i32(self.ad.params.image_mode));
        match image_mode {
            ImageMode::Single => {
                self.command(&protocol::set_frame_number(1))?;
                self.command("Snap")?;
            }
            ImageMode::Multiple => {
                let num_images = self.get_i32(self.ad.params.num_images).max(1);
                self.command(&protocol::set_frame_number(num_images))?;
                self.command("Snap")?;
            }
            ImageMode::Continuous => {
                self.command("Start")?;
            }
        }
        let _ = self.commands.try_send(TaskCommand::Start);
        Ok(())
    }

    /// The choices a record binds to at init (C `readEnum` / `doEnumCallbacks`).
    fn enum_entries(choices: &Choices) -> Arc<[EnumEntry]> {
        choices
            .iter()
            .enumerate()
            .map(|(i, s)| EnumEntry {
                string: s.clone(),
                value: i as i32,
                severity: 0,
            })
            .collect()
    }
}

impl PortDriver for PslDetector {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let was_acquiring = self.get_i32(self.ad.params.acquire) != 0;
        self.ad
            .port_base
            .set_int32_param(reason, user.addr, value)?;

        // C keeps the server status only on the branches below that assign it
        // (PSL.cpp:744-797); the others report success whatever the server said.
        let p = self.ad.params;
        let status = if reason == p.acquire {
            if value != 0 && !was_acquiring {
                let _ = self.start_acquire();
                Ok(())
            } else if value == 0 && was_acquiring {
                let status = self.send("Stop");
                self.shared.stop_requested.store(true, Ordering::Release);
                status
            } else {
                Ok(())
            }
        } else if reason == p.bin_x || reason == p.bin_y {
            let bin_x = self.get_i32(p.bin_x);
            let bin_y = self.get_i32(p.bin_y);
            self.command(&protocol::set_binning(bin_x, bin_y)).map(drop)
        } else if reason == p.min_x || reason == p.min_y || reason == p.size_x || reason == p.size_y
        {
            let min_x = self.get_i32(p.min_x);
            let min_y = self.get_i32(p.min_y);
            let size_x = self.get_i32(p.size_x);
            let size_y = self.get_i32(p.size_y);
            self.send(&protocol::set_sub_area(min_x, min_y, size_x, size_y))
        } else if reason == p.reverse_x {
            self.command(&protocol::set_fliplr(value)).map(drop)
        } else if reason == p.reverse_y {
            self.command(&protocol::set_flipud(value)).map(drop)
        } else if reason == p.trigger_mode {
            match protocol::choice_from_index(&self.state.trigger_modes, value) {
                Ok(mode) => {
                    let command = protocol::set_trigger_mode(mode);
                    self.command(&command).map(drop)
                }
                Err(_) => Ok(()),
            }
        } else if reason == p.base.auto_save {
            self.send(&protocol::set_auto_save(value))
        } else if reason == p.base.file_format {
            match protocol::choice_from_index(&self.state.record_formats, value) {
                Ok(format) => {
                    let command = protocol::set_record_format(format);
                    self.send(&command)
                }
                Err(_) => Ok(()),
            }
        } else if reason == self.params.camera_name {
            self.open_camera(value)
        } else if reason == p.base.file_number {
            self.send(&protocol::set_record_number(value))
        } else if reason < self.params.camera_name {
            ADDriver::ad_write_int32(self, user, value)
        } else {
            Ok(())
        };

        let _ = self.get_config();
        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "PSL:writeInt32: error, status={} function={reason}, value={value}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("PSL:writeInt32: function={reason}, value={value}\n"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let mut status = self
            .ad
            .port_base
            .set_float64_param(reason, user.addr, value);

        let p = self.ad.params;
        if reason == p.acquire_time {
            status = self.send(&protocol::set_exposure(value));
        } else if reason == p.acquire_period {
            status = self.send(&protocol::set_frame_time(value));
        } else if reason == p.gain {
            status = self.send(&protocol::set_chip_gain(value));
        } else if reason < self.params.camera_name {
            // ADDriver::writeFloat64 is asynPortDriver's (PSL.cpp:848).
            status = self.ad.port_base.write_float64_param(user, value);
        }

        let _ = self.get_config();
        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "PSL:writeFloat64: error, status={} function={reason}, value={value:.6}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("PSL:writeFloat64: function={reason}, value={value:.6}\n"),
            ),
        }
        status
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let text = String::from_utf8_lossy(value)
            .trim_end_matches('\0')
            .to_string();
        self.ad
            .port_base
            .set_string_param(reason, user.addr, text.clone())?;

        let p = self.ad.params;
        let status = if reason == p.base.file_path {
            let status = self.send(&protocol::set_record_path(&text));
            let _ = self.check_path();
            status
        } else if reason == p.base.file_name {
            self.send(&protocol::set_record_name(&text))
        } else if reason == self.params.tiff_comment {
            self.send(&protocol::set_record_tag(&text))
        } else {
            self.ad.write_octet(user, value).map(drop)
        };

        let _ = self.get_config();
        let _ = self.ad.port_base.call_param_callbacks(0);
        // A failure only fills errorMessage; success is a TRACEIO line.
        match status {
            Err(e) => Err(AsynError::Status {
                status: e.status(),
                message: format!(
                    "PSL:writeOctet: status={}, function={reason}, value={text}",
                    e.status() as i32
                ),
            }),
            Ok(()) => {
                user.print(
                    TraceMask::IO_DRIVER,
                    file!(),
                    line!(),
                    format_args!("PSL:writeOctet: function={reason}, value={text}\n"),
                );
                Ok(value.len())
            }
        }
    }

    fn read_enum(&mut self, user: &AsynUser) -> AsynResult<(usize, Arc<[EnumEntry]>)> {
        let choices = if user.reason == self.params.camera_name {
            &self.state.camera_names
        } else if user.reason == self.ad.params.trigger_mode {
            &self.state.trigger_modes
        } else if user.reason == self.ad.params.base.file_format {
            &self.state.record_formats
        } else {
            return self.ad.port_base.params.get_enum(user.reason, user.addr);
        };
        let entries = Self::enum_entries(choices);
        Ok((entries.len(), entries))
    }
}

impl ADDriver for PslDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// What the IOC layer holds on to.
pub struct PslRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: PslParams,
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    /// None when the constructor returned early, as C never starts PSLTask.
    #[allow(dead_code)]
    task: Option<std::thread::JoinHandle<()>>,
}

impl PslRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its acquisition task (C `PSLConfig`).
///
/// `server_handle` is the user of the `drvAsynIPPort` that reaches PSLViewer —
/// one with no port when that port does not exist, since C ignores the
/// connect status (PSL.cpp:166-167).
pub fn create_psl_detector(
    claim: &PortClaim,
    server_handle: SyncIOHandle,
    max_memory: usize,
) -> AsynResult<PslRuntime> {
    let port_name = claim.name();
    let shared = Arc::new(SharedState {
        stop_requested: AtomicBool::new(false),
    });
    let server = PslServer::new(server_handle);
    let (tx, rx) = rt::command_channel::<TaskCommand>(4);

    let (det, booted) =
        PslDetector::new(port_name, server.clone(), max_memory, shared.clone(), tx)?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let task = booted.then(|| {
        start_acquisition_task(AcquisitionContext {
            server,
            handle: runtime_handle.port_handle().clone(),
            output: ArrayPublisher::new(array_output.clone()),
            pool: pool.clone(),
            ad_params,
            shared,
            commands: rx,
        })
    });

    Ok(PslRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        task,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::{ADStatus, ShutterMode};
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::registry::claim_port;
    use std::time::Duration;

    /// A detector whose PSLViewer socket never connects: the writes under
    /// test must not need the server.
    fn detector(name: &str) -> PslDetector {
        let ip = format!("{name}_IP");
        let driver = DrvAsynIPPort::new(&ip, "127.0.0.1:1").unwrap();
        let (runtime, _) =
            create_port_runtime(&claim_port(&ip).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let shared = Arc::new(SharedState {
            stop_requested: AtomicBool::new(false),
        });
        let (tx, _rx) = rt::command_channel::<TaskCommand>(4);
        PslDetector::new(
            name,
            PslServer::new(SyncIOHandle::from_handle(handle, 0, Duration::ZERO)),
            1 << 20,
            shared,
            tx,
        )
        .unwrap()
        .0
    }

    /// C ignores the `pasynOctetSyncIO->connect` status (PSL.cpp:166): with no
    /// server port configured, getVersion fails and the port stays up, but the
    /// constructor returns there (PSL.cpp:168-173): ADStatus keeps the base
    /// class Idle, the defaults are unset and PSLTask never runs.
    #[test]
    fn a_detector_whose_server_port_is_missing_still_boots() {
        let (server, status) =
            SyncIOHandle::connect_port("PSL_NONE_IP", 0, Duration::ZERO, InterfaceType::Octet);
        assert!(status.is_err());
        let rt = create_psl_detector(&claim_port("PSL_NONE").unwrap(), server, 1 << 20)
            .expect("the detector port");
        assert!(rt.task.is_none());
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        assert_eq!(
            sync.read_int32(rt.ad_params.status).unwrap(),
            ADStatus::Idle as i32
        );
        let octet = |reason| {
            let mut v = sync.read_octet(reason, 64).unwrap();
            v.retain(|&b| b != 0);
            String::from_utf8(v).unwrap()
        };
        assert_eq!(octet(rt.ad_params.status_message), "");
        assert!(sync.read_octet(rt.ad_params.base.model, 64).is_err());
        assert_eq!(
            octet(rt.ad_params.base.driver_version),
            octet(rt.ad_params.base.ad_core_version)
        );
    }

    /// ADDriver::writeInt32 and asynNDArrayDriver::writeInt32 each close with
    /// an ASYN_TRACEIO_DRIVER line (ADDriver.cpp:111-113,
    /// asynNDArrayDriver.cpp:703-705).
    #[test]
    fn a_base_class_write_runs_both_base_class_write_int32s() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "PSL_BASE_TRACE";
        let det = detector(name);
        let p = det.ad.params.base;
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_int32_blocking(p.pool_poll_stats, 0, 1).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for driver in ["asynNDArrayDriver", "ADDriver"] {
            assert!(
                text.contains(&format!(
                    "{driver}:writeInt32: function={}",
                    p.pool_poll_stats
                )),
                "{driver}: {text:?}"
            );
        }
        std::mem::forget(runtime);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (PSL.cpp:848), whose
    /// IO_DRIVER line precedes PSL's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "PSL_BASE_F64";
        let det = detector(name);
        let delay = det.ad.params.shutter_open_delay;
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        let path = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_float64_blocking(delay, 0, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("PSL:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
        std::mem::forget(runtime);
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let mut det = detector("PSL_BASE_WRITE");
        let p = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        let mut user = AsynUser::new(p.shutter_control);
        det.write_int32(&mut user, 1).unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    #[test]
    fn writes_report_the_server_status_only_where_c_keeps_it() {
        let mut det = detector("PSL_STATUS");
        let p = det.ad.params;
        // Kept by C: the dead server's failure is the write's status.
        assert!(
            det.write_int32(&mut AsynUser::new(p.base.file_number), 3)
                .is_err()
        );
        assert!(
            det.write_float64(&mut AsynUser::new(p.acquire_time), 0.5)
                .is_err()
        );
        assert!(
            det.write_octet(&mut AsynUser::new(p.base.file_name), b"scan")
                .is_err()
        );
        // Dropped by C: SetBinning's status is never assigned.
        det.write_int32(&mut AsynUser::new(p.bin_x), 2).unwrap();
        // The parameter is stored either way.
        assert_eq!(det.get_i32(p.base.file_number), 3);
    }

    /// A FilePath write runs C `checkPath` (PSL.cpp:886), which stores the
    /// path with its separator and leaves FilePathExists alone for an empty
    /// one (asynNDArrayDriver.cpp:104-109).
    #[test]
    fn a_file_path_write_runs_check_path() {
        let mut det = detector("PSL_CHECK_PATH");
        let p = det.ad.params.base;
        let dir = std::env::temp_dir().display().to_string();
        let dir = dir.trim_end_matches('/').to_string();
        let _ = det.write_octet(&mut AsynUser::new(p.file_path), dir.as_bytes());
        let stored = |det: &PslDetector| {
            String::from_utf8_lossy(det.ad.port_base.get_string_param(p.file_path, 0).unwrap())
                .into_owned()
        };
        assert_eq!(stored(&det), format!("{dir}/"));
        assert_eq!(det.get_i32(p.file_path_exists), 1);
        let _ = det.write_octet(&mut AsynUser::new(p.file_path), b"");
        assert_eq!(det.get_i32(p.file_path_exists), 1);
    }

    /// A PSLViewer that answers every command with `reply`.
    struct FakeServer {
        base: PortDriverBase,
        reply: Vec<u8>,
    }

    impl PortDriver for FakeServer {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let n = self.reply.len().min(buf.len());
            buf[..n].copy_from_slice(&self.reply[..n]);
            Ok(n)
        }
    }

    /// getConfig stores GetRecordPath's answer as it came, without
    /// `checkPath` (PSL.cpp:453-459).
    #[test]
    fn get_config_stores_the_record_path_unchecked() {
        let name = "PSL_RECORD_PATH";
        let dir = std::env::temp_dir().display().to_string();
        let dir = dir.trim_end_matches('/').to_string();
        let ip = format!("{name}_IP");
        let mut base = PortDriverBase::new(&ip, 1, Default::default());
        base.init_connected(true);
        let server = FakeServer {
            base,
            reply: dir.clone().into_bytes(),
        };
        let (runtime, _) =
            create_port_runtime(&claim_port(&ip).unwrap(), server, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let shared = Arc::new(SharedState {
            stop_requested: AtomicBool::new(false),
        });
        let (tx, _rx) = rt::command_channel::<TaskCommand>(4);
        let mut det = PslDetector::new(
            name,
            PslServer::new(SyncIOHandle::from_handle(handle, 0, Duration::ZERO)),
            1 << 20,
            shared,
            tx,
        )
        .unwrap()
        .0;
        det.state.valid_options = ["RecordPath".to_string()].into();
        let p = det.ad.params.base;
        det.get_config().unwrap();
        let stored =
            String::from_utf8_lossy(det.ad.port_base.get_string_param(p.file_path, 0).unwrap())
                .into_owned();
        assert_eq!(stored, dir);
        assert_eq!(det.get_i32(p.file_path_exists), 0);
    }
}
