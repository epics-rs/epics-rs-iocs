//! The PSL areaDetector driver (C `PSL`).
//!
//! Ownership: the PSLViewer socket lives behind [`PslServer`], which serialises
//! every exchange; the choice sets and the camera count are plain fields of the
//! driver, so only the port actor ever reads or writes them. The acquisition
//! task reaches the parameter library through the actor and the server through
//! its own mutex — see the invariant in [`crate::connection`].

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;

use crate::connection::PslServer;
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
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(port_name, 0, 0, max_memory)?;
        let params = PslParams::create(&mut ad.port_base)?;

        let base = &mut ad.port_base;
        base.set_string_param(ad.params.base.manufacturer, 0, "PSL".into())?;
        base.set_string_param(ad.params.base.model, 0, "CCD".into())?;
        base.set_string_param(
            ad.params.base.driver_version,
            0,
            env!("CARGO_PKG_VERSION").into(),
        )?;
        base.set_int32_param(ad.params.base.data_type, 0, NDDataType::UInt16 as u8 as i32)?;
        base.set_int32_param(ad.params.image_mode, 0, ImageMode::Single as i32)?;
        base.set_float64_param(ad.params.acquire_period, 0, 0.0)?;
        base.set_float64_param(ad.params.acquire_time, 0, 1.0)?;
        base.set_int32_param(ad.params.num_images, 0, 1)?;
        base.set_int32_param(ad.params.status, 0, ADStatus::Idle as i32)?;

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
        det.connect_to_server()?;
        Ok(det)
    }

    /// Greet the server and open its first camera (C's constructor).
    ///
    /// A server that is not running must not take the IOC down with it: the
    /// port comes up with no choices, and the next `CameraName` write retries.
    fn connect_to_server(&mut self) -> AsynResult<()> {
        let reply = self.command("GetVersion")?;
        match protocol::parse_version(&reply) {
            Ok(version) => log::info!("psl: PSLViewer {version}"),
            Err(e) => {
                log::error!("psl: {e}");
                self.ad.port_base.set_string_param(
                    self.ad.params.status_message,
                    0,
                    format!("PSL server: {e}"),
                )?;
                self.ad.port_base.set_int32_param(
                    self.ad.params.status,
                    0,
                    ADStatus::Error as i32,
                )?;
                return Ok(());
            }
        }

        self.state.camera_names = self.get_choices("GetCamList")?;
        // The multi-camera configuration is not in GetCamList.
        self.state.camera_names.insert("multiconf".into());
        self.open_camera(0)
    }

    /// Send one command and publish both sides of the exchange to the
    /// StringToServer / StringFromServer records (C `writeReadServer`).
    fn command(&mut self, command: &str) -> AsynResult<String> {
        let reply = match self.server.command(command) {
            Ok(reply) => reply,
            Err(e) => {
                log::error!("psl: '{command}' failed: {e}");
                String::new()
            }
        };
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.string_to_server, 0, command.into())?;
        base.set_string_param(self.ad.params.string_from_server, 0, reply.clone())?;
        Ok(reply)
    }

    fn get_choices(&mut self, command: &str) -> AsynResult<Choices> {
        let multi = self.state.multi_camera();
        let reply = self.command(command)?;
        Ok(protocol::parse_choices(&reply, multi))
    }

    /// Open one of the cameras `GetCamList` offered (C `openCamera`).
    fn open_camera(&mut self, index: i32) -> AsynResult<()> {
        let name = match protocol::choice_from_index(&self.state.camera_names, index) {
            Ok(name) => name.to_string(),
            Err(e) => {
                // C walked its std::set past the end and dereferenced the end
                // iterator, so an out-of-range CameraName was undefined
                // behaviour.
                log::error!("psl: cannot open camera {index}: {e}");
                return Ok(());
            }
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
        base.set_string_param(self.ad.params.base.manufacturer, 0, "PSL".into())?;

        let reply = self.command("GetMaximumSize")?;
        let size = protocol::parse_ints(&reply);
        if size.len() >= 2 {
            let base = &mut self.ad.port_base;
            base.set_int32_param(self.ad.params.max_size_x, 0, size[0])?;
            base.set_int32_param(self.ad.params.max_size_y, 0, size[1])?;
        } else {
            log::error!("psl: GetMaximumSize answered '{reply}'");
        }

        self.state.valid_options = self.get_choices("GetOptions")?;
        self.state.record_formats = self.get_choices("GetOptionRange;RecordFormat")?;
        self.state.trigger_modes = self.get_choices("GetOptionRange;TriggerMode")?;

        self.get_config()?;
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
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
        let reply = self.command("GetMode")?;
        match protocol::parse_mode(protocol::quoted_field(&reply, multi)) {
            Ok((data_type, color_mode)) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(self.ad.params.base.data_type, 0, data_type as u8 as i32)?;
                base.set_int32_param(self.ad.params.base.color_mode, 0, color_mode as i32)?;
            }
            Err(e) => log::error!("psl: GetMode: {e}"),
        }

        if self.supports("TriggerMode") {
            let reply = self.command("GetTriggerMode")?;
            let mode = protocol::quoted_field(&reply, multi).to_string();
            match protocol::index_of_choice(&self.state.trigger_modes, &mode) {
                Some(index) => {
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.trigger_mode, 0, index)?;
                }
                None => log::error!("psl: unknown trigger mode '{mode}'"),
            }
        }

        if self.supports("Exposure") {
            let reply = self.command("GetExposure")?;
            match protocol::parse_exposure(protocol::numeric_field(&reply, multi)) {
                Some(seconds) => {
                    self.ad
                        .port_base
                        .set_float64_param(self.ad.params.acquire_time, 0, seconds)?;
                }
                None => log::error!("psl: GetExposure answered '{reply}'"),
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
            self.check_path()?;
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
            let reply = self.command("GetRecordFormat")?;
            let format = protocol::quoted_field(&reply, multi).to_string();
            match protocol::index_of_choice(&self.state.record_formats, &format) {
                Some(index) => {
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.base.file_format, 0, index)?;
                }
                None => log::error!("psl: unknown file format '{format}'"),
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
    /// (C `checkPath`).
    fn check_path(&mut self) -> AsynResult<()> {
        let path = self
            .ad
            .port_base
            .get_string_param(self.ad.params.base.file_path, 0)?
            .to_string();
        let exists = !path.is_empty() && Path::new(&path).is_dir();
        self.ad.port_base.set_int32_param(
            self.ad.params.base.file_path_exists,
            0,
            exists as i32,
        )?;
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
        if self.commands.try_send(TaskCommand::Start).is_err() {
            log::error!("psl: the acquisition task is not accepting commands");
        }
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
            .params
            .set_int32(reason, user.addr, value)?;

        let p = self.ad.params;
        if reason == p.acquire {
            if value != 0 && !was_acquiring {
                self.start_acquire()?;
            } else if value == 0 && was_acquiring {
                self.command("Stop")?;
                self.shared.stop_requested.store(true, Ordering::Release);
            }
        } else if reason == p.bin_x || reason == p.bin_y {
            let bin_x = self.get_i32(p.bin_x);
            let bin_y = self.get_i32(p.bin_y);
            self.command(&protocol::set_binning(bin_x, bin_y))?;
        } else if reason == p.min_x || reason == p.min_y || reason == p.size_x || reason == p.size_y
        {
            let min_x = self.get_i32(p.min_x);
            let min_y = self.get_i32(p.min_y);
            let size_x = self.get_i32(p.size_x);
            let size_y = self.get_i32(p.size_y);
            self.command(&protocol::set_sub_area(min_x, min_y, size_x, size_y))?;
        } else if reason == p.reverse_x {
            self.command(&protocol::set_fliplr(value))?;
        } else if reason == p.reverse_y {
            self.command(&protocol::set_flipud(value))?;
        } else if reason == p.trigger_mode {
            match protocol::choice_from_index(&self.state.trigger_modes, value) {
                Ok(mode) => {
                    let command = protocol::set_trigger_mode(mode);
                    self.command(&command)?;
                }
                Err(e) => log::error!("psl: TriggerMode {value}: {e}"),
            }
        } else if reason == p.base.auto_save {
            self.command(&protocol::set_auto_save(value))?;
        } else if reason == p.base.file_format {
            match protocol::choice_from_index(&self.state.record_formats, value) {
                Ok(format) => {
                    let command = protocol::set_record_format(format);
                    self.command(&command)?;
                }
                Err(e) => log::error!("psl: FileFormat {value}: {e}"),
            }
        } else if reason == self.params.camera_name {
            self.open_camera(value)?;
        } else if reason == p.base.file_number {
            self.command(&protocol::set_record_number(value))?;
        }

        self.get_config()?;
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .params
            .set_float64(reason, user.addr, value)?;

        let p = self.ad.params;
        if reason == p.acquire_time {
            self.command(&protocol::set_exposure(value))?;
        } else if reason == p.acquire_period {
            self.command(&protocol::set_frame_time(value))?;
        } else if reason == p.gain {
            self.command(&protocol::set_chip_gain(value))?;
        }

        self.get_config()?;
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let text = String::from_utf8_lossy(value)
            .trim_end_matches('\0')
            .to_string();
        self.ad
            .port_base
            .params
            .set_string(reason, user.addr, text.clone())?;

        let p = self.ad.params;
        if reason == p.base.file_path {
            self.command(&protocol::set_record_path(&text))?;
            self.check_path()?;
        } else if reason == p.base.file_name {
            self.command(&protocol::set_record_name(&text))?;
        } else if reason == self.params.tiff_comment {
            self.command(&protocol::set_record_tag(&text))?;
        }

        self.get_config()?;
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(value.len())
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
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    #[allow(dead_code)]
    task: std::thread::JoinHandle<()>,
}

impl PslRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.array_output.lock().add(sender);
    }
}

/// Create the detector port and start its acquisition task (C `PSLConfig`).
///
/// `server_handle` is the `drvAsynIPPort` that reaches PSLViewer.
pub fn create_psl_detector(
    port_name: &str,
    server_handle: PortHandle,
    max_memory: usize,
) -> AsynResult<PslRuntime> {
    let shared = Arc::new(SharedState {
        stop_requested: AtomicBool::new(false),
    });
    let server = PslServer::new(server_handle);
    let (tx, rx) = rt::command_channel::<TaskCommand>(4);

    let det = PslDetector::new(port_name, server.clone(), max_memory, shared.clone(), tx)?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(det, RuntimeConfig::default());
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let queued_counter = Arc::new(QueuedArrayCounter::new());

    let task = start_acquisition_task(AcquisitionContext {
        server,
        handle: runtime_handle.port_handle().clone(),
        output: ArrayPublisher::new(array_output.clone()),
        queued: queued_counter.clone(),
        ad_params,
        shared,
        commands: rx,
    });

    Ok(PslRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        queued_counter,
        task,
    })
}
