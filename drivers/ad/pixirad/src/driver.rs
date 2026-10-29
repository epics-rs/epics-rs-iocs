//! The Pixirad areaDetector driver (C `pixirad.cpp`).
//!
//! Ownership: the driver owns the [`PixiradServer`], so every command to the
//! box runs on the port actor and goes through [`PixiradDetector::exchange`].
//! The three background threads — UDP data listener, data task, status task —
//! reach the driver only through the actor.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, Ordering};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};

use crate::connection::{Exchange, PixiradServer, ServerError};
use crate::params::PixiradParams;
use crate::protocol;
use crate::task::{self, DataContext, StatusContext};
use crate::thresholds;
use crate::types::*;
use crate::udp::Frame;

/// State the port actor shares with the background threads.
pub struct SharedState {
    /// Frames handed to the data task and not yet taken (C
    /// `epicsMessageQueuePending`).
    pub queued_frames: AtomicI32,
    /// The size of the frame queue (C `maxDataPortBuffers`).
    pub max_buffers: i32,
}

pub struct PixiradDetector {
    pub ad: ADDriverBase,
    pub params: PixiradParams,
    server: PixiradServer,
    /// `None` for a `maxSizeX` C does not know.
    asic: Option<Asic>,
    /// `None` for a size C does not know; it then has no frames to read.
    sensor: Option<Sensor>,
    /// Set by `pixiradAutoCal`; only the Pixie-III sends it.
    vbg_mcal_dac: i32,
    shared: Arc<SharedState>,
}

impl PixiradDetector {
    fn new(
        port_name: &str,
        server: PixiradServer,
        max_size_x: i32,
        max_size_y: i32,
        max_memory: usize,
        shared: Arc<SharedState>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            max_memory,
            ASYN_CANBLOCK,
            InterfaceMask::NONE,
            InterfaceMask::NONE,
        )?;
        let params = PixiradParams::create(&mut ad.port_base)?;
        let p = ad.params;

        let base = &mut ad.port_base;
        base.set_string_param(p.base.manufacturer, 0, "Pixirad")?;
        // pixirad.cpp:40-42.
        base.set_string_param(p.base.driver_version, 0, "2.2.0")?;
        base.set_int32_param(p.max_size_x, 0, max_size_x)?;
        base.set_int32_param(p.max_size_y, 0, max_size_y)?;
        base.set_int32_param(p.size_x, 0, max_size_x)?;
        base.set_int32_param(p.size_y, 0, max_size_y)?;
        base.set_int32_param(p.base.array_size_x, 0, max_size_x)?;
        base.set_int32_param(p.base.array_size_y, 0, max_size_y)?;
        base.set_int32_param(p.base.array_size, 0, 0)?;
        base.set_int32_param(p.base.data_type, 0, NDDataType::UInt16 as u8 as i32)?;
        base.set_int32_param(p.num_images, 0, 1)?;
        base.set_int32_param(p.image_mode, 0, ImageMode::Continuous as i32)?;
        base.set_int32_param(p.trigger_mode, 0, TriggerMode::Internal as i32)?;
        base.set_float64_param(p.acquire_time, 0, 1.0)?;
        base.set_float64_param(p.acquire_period, 0, 1.0)?;
        base.set_float64_param(p.temperature, 0, INITIAL_COOLING_VALUE)?;
        base.set_int32_param(p.frame_type, 0, FrameType::OneColorLow as i32)?;

        base.set_int32_param(params.cooling_state, 0, 1)?;
        base.set_float64_param(params.hv_value, 0, INITIAL_HV_VALUE)?;
        base.set_int32_param(params.hv_state, 0, 1)?;
        base.set_int32_param(params.hv_mode, 0, HVMode::Auto as i32)?;
        base.set_int32_param(params.colors_collected, 0, 0)?;
        base.set_int32_param(params.udp_buffers_read, 0, 0)?;
        base.set_int32_param(params.udp_buffers_max, 0, shared.max_buffers)?;
        base.set_int32_param(params.udp_buffers_free, 0, shared.max_buffers)?;
        base.set_float64_param(params.threshold[0], 0, 10.0)?;
        base.set_float64_param(params.threshold[1], 0, 15.0)?;
        base.set_float64_param(params.threshold[2], 0, 20.0)?;
        base.set_float64_param(params.threshold[3], 0, 25.0)?;
        base.set_int32_param(params.sync_in_polarity, 0, SyncPolarity::Pos as i32)?;
        base.set_int32_param(params.sync_out_polarity, 0, SyncPolarity::Pos as i32)?;
        base.set_int32_param(params.sync_out_function, 0, SyncOutFunction::Shutter as i32)?;
        base.set_int32_param(params.system_reset, 0, 0)?;
        base.set_int32_param(params.cooling_status, 0, CoolingStatus::Ok as i32)?;

        let mut det = Self {
            ad,
            params,
            server,
            asic: Asic::from_size_x(max_size_x),
            sensor: Sensor::from_size(max_size_x, max_size_y),
            vbg_mcal_dac: 0,
            shared,
        };

        // The firmware will not take a high-voltage setting unless it differs
        // from the last one it saw, so the initial value is sent twice, once
        // one volt off. C ignores both statuses: a box that is not there yet
        // still gets a driver.
        det.set_float64(det.params.hv_value, INITIAL_HV_VALUE - 1.0)?;
        let _ = det.set_cooling_and_hv();
        det.set_float64(det.params.hv_value, INITIAL_HV_VALUE)?;
        let _ = det.set_cooling_and_hv();

        det.identify()?;
        match det.sensor {
            Some(sensor) => {
                let model = det.ad.params.base.model;
                det.ad
                    .port_base
                    .set_string_param(model, 0, sensor.build.model_name())?;
            }
            // C then names the model from an uninitialised build; it stays
            // empty here.
            None if det.asic.is_none() => {
                println!("pixirad::pixirad Illegal maxSizeX={max_size_x}")
            }
            None => println!("pixirad::pixirad Illegal maxSizeY={max_size_y}"),
        }
        det.ad.port_base.call_param_callbacks(0)?;
        Ok(det)
    }

    /// Ask the box who it is (C's constructor).
    fn identify(&mut self) -> AsynResult<()> {
        let reply = self.exchange(&protocol::get_firmware_version())?.reply;
        // C bug, pixirad.cpp:448-449, :564-566: both are set whatever the
        // reply, and a part `sscanf` did not match publishes the uninitialised
        // `serialNumber` and a `tempString` still holding the driver version.
        // Here such a part is empty.
        let (serial, version) = protocol::parse_firmware_version(&reply);
        let base = &mut self.ad.port_base;
        let p = self.ad.params.base;
        base.set_string_param(p.serial_number, 0, serial.unwrap_or_default())?;
        base.set_string_param(p.firmware_version, 0, version.unwrap_or_default())?;

        let reply = self.exchange(&protocol::get_additional_info())?.reply;
        // C added the length of the marker to a null `strstr` result and
        // published whatever was at that address.
        if let Some(info) = protocol::parse_additional_info(&reply) {
            self.ad
                .port_base
                .set_string_param(self.params.system_info, 0, info)?;
        }
        Ok(())
    }

    /// Send one command and return its status (C `writeReadServer`).
    fn command(&mut self, command: &str) -> AsynResult<()> {
        self.exchange(command)?.result.map_err(AsynError::from)
    }

    /// Send one command; publish both sides of it and whether the box took it.
    fn exchange(&mut self, command: &str) -> AsynResult<Exchange> {
        let exchange = self.server.command(command);
        // C sets the message only on a reply (pixirad.cpp:910-920).
        let message = match &exchange.result {
            Ok(()) => Some("Server returned OK"),
            Err(ServerError::Rejected(_)) => Some("Error from server"),
            Err(_) => None,
        };
        let base = &mut self.ad.port_base;
        if let Some(message) = message {
            base.set_string_param(self.ad.params.status_message, 0, message)?;
        }
        base.set_string_param(self.ad.params.string_to_server, 0, command)?;
        base.set_string_param(self.ad.params.string_from_server, 0, exchange.reply.clone())?;
        Ok(exchange)
    }

    fn get_i32(&self, reason: usize) -> i32 {
        self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0)
    }

    fn get_f64(&self, reason: usize) -> f64 {
        self.ad
            .port_base
            .get_float64_param(reason, 0)
            .unwrap_or(0.0)
    }

    fn set_int32(&mut self, reason: usize, value: i32) -> AsynResult<()> {
        self.ad.port_base.set_int32_param(reason, 0, value)
    }

    fn set_float64(&mut self, reason: usize, value: f64) -> AsynResult<()> {
        self.ad.port_base.set_float64_param(reason, 0, value)
    }

    /// `DAQ:! INIT` — cooling set point and high voltage go together
    /// (C `setCoolingAndHV`).
    fn set_cooling_and_hv(&mut self) -> AsynResult<()> {
        let command = protocol::init(
            self.get_f64(self.ad.params.temperature),
            self.get_i32(self.params.cooling_state),
            self.get_f64(self.params.hv_value),
            self.get_i32(self.params.hv_state),
        );
        self.command(&command)
    }

    /// `DAQ:! SET_SYNC` (C `setSync`).
    fn set_sync(&mut self) -> AsynResult<()> {
        let sync_in = SyncPolarity::from_i32(self.get_i32(self.params.sync_in_polarity))
            .unwrap_or(SyncPolarity::Pos);
        let sync_out = SyncPolarity::from_i32(self.get_i32(self.params.sync_out_polarity))
            .unwrap_or(SyncPolarity::Pos);
        let function = SyncOutFunction::from_i32(self.get_i32(self.params.sync_out_function))
            .unwrap_or(SyncOutFunction::Shutter);
        let command = protocol::set_sync(sync_in, sync_out, function);
        self.command(&command)
    }

    /// One pass of C statusTask under the port lock: the broadcast readings,
    /// then the dew point and cooling status when every input arrived, then on
    /// an error CoolingState off and `setCoolingAndHV` (its status ignored),
    /// and one `callParamCallbacks`, which posts in that order.
    /// The UDP listener's pass after it queues a frame (C
    /// `udpDataListenerTask`, pixirad.cpp:1234-1245).
    pub(crate) fn post_udp_frame(&mut self, speed: f64) -> AsynResult<()> {
        let read = self.get_i32(self.params.udp_buffers_read) + 1;
        self.set_int32(self.params.udp_buffers_read, read)?;
        let free = self.shared.max_buffers - self.shared.queued_frames.load(Ordering::Acquire);
        self.set_int32(self.params.udp_buffers_free, free)?;
        self.set_float64(self.params.udp_speed, speed)?;
        self.ad.port_base.call_param_callbacks(0)
    }

    pub(crate) fn post_status(
        &mut self,
        readings: &[(usize, f64)],
        cooling: Option<(f64, CoolingStatus)>,
    ) -> AsynResult<()> {
        for &(reason, value) in readings {
            self.set_float64(reason, value)?;
        }
        if let Some((dew_point, status)) = cooling {
            self.set_float64(self.params.dew_point, dew_point)?;
            self.set_int32(self.params.cooling_status, status as i32)?;
            if status.is_error() {
                self.set_int32(self.params.cooling_state, 0)?;
                let _ = self.set_cooling_and_hv();
            }
        }
        self.ad.port_base.call_param_callbacks(0)
    }

    fn frame_type(&self) -> FrameType {
        FrameType::from_i32(self.get_i32(self.ad.params.frame_type))
            .unwrap_or(FrameType::OneColorLow)
    }

    /// Work out the threshold registers and send them (C `setThresholds`).
    fn set_thresholds(&mut self, reference: i32) -> AsynResult<()> {
        let energies = [
            self.get_f64(self.params.threshold[0]),
            self.get_f64(self.params.threshold[1]),
            self.get_f64(self.params.threshold[2]),
            self.get_f64(self.params.threshold[3]),
            self.get_f64(self.params.hit_threshold),
        ];
        // C picks a register layout from an uninitialised ASIC.
        let asic = self.asic.ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "pixirad: no ASIC is known for this maxSizeX".into(),
        })?;
        let t = thresholds::calculate(asic, &energies);

        for i in 0..4 {
            self.set_float64(self.params.threshold_actual[i], t.actual_energy[i])?;
        }
        self.set_float64(self.params.hit_threshold_actual, t.actual_energy[4])?;

        let readout_mode = self.frame_type().readout_mode();
        let command = match asic {
            Asic::PIII => {
                let count_mode = CountMode::from_i32(self.get_i32(self.params.count_mode))
                    .unwrap_or(CountMode::Normal);
                protocol::set_sensor_operatings_piii(
                    &t.registers,
                    readout_mode,
                    count_mode.as_str(),
                    self.vbg_mcal_dac,
                )
            }
            // The Pixie-II has no count mode and a fixed auto-full-scale.
            Asic::PII => protocol::set_sensor_operatings_pii(
                &t.registers,
                t.vth_max,
                reference,
                7,
                readout_mode,
            ),
        };
        self.command(&command)
    }

    /// `DAQ:! AUTOCAL` between two threshold settings (C `doAutoCalibrate`).
    fn do_auto_calibrate(&mut self) -> AsynResult<()> {
        self.reset_frame_counters()?;
        // The reference has to be off while the chip calibrates itself. C
        // returns the AUTOCAL status alone.
        let _ = self.set_thresholds(0);
        let status = self.command(&protocol::autocal());
        let _ = self.set_thresholds(1);
        status
    }

    fn reset_frame_counters(&mut self) -> AsynResult<()> {
        self.set_int32(self.ad.params.num_images_counter, 0)?;
        self.set_int32(self.params.colors_collected, 0)?;
        self.set_int32(self.params.udp_buffers_read, 0)
    }

    /// `DAQ:! LOOP` (C `startAcquire`).
    fn start_acquire(&mut self) -> AsynResult<()> {
        let acquire_time = self.get_f64(self.ad.params.acquire_time);
        let acquire_period = self.get_f64(self.ad.params.acquire_period);
        let shutter_pause = (acquire_period - acquire_time).max(0.0);
        let trigger_mode = TriggerMode::from_i32(self.get_i32(self.ad.params.trigger_mode))
            .unwrap_or(TriggerMode::Internal);
        let hv_mode = HVMode::from_i32(self.get_i32(self.params.hv_mode)).unwrap_or(HVMode::Manual);
        let command = protocol::loop_acquire(
            self.get_i32(self.ad.params.num_images),
            acquire_time,
            shutter_pause,
            self.frame_type(),
            trigger_mode,
            hv_mode,
        );
        self.reset_frame_counters()?;
        self.command(&command)
    }

    /// Reboot the box and put back everything it forgot (C `systemReset`).
    fn system_reset(&mut self) -> AsynResult<()> {
        self.ad.port_base.call_param_callbacks(0)?;
        // C carries on through every step and returns the last one's status.
        let _ = self.command(&protocol::system_reset());
        self.server.reconnect_after(DETECTOR_RESET_TIME);

        let _ = self.set_sync();
        let _ = self.set_thresholds(1);

        // Same firmware quirk as at startup: two different high-voltage values.
        let hv_value = self.get_f64(self.params.hv_value);
        let hv_state = self.get_i32(self.params.hv_state);
        self.set_float64(self.params.hv_value, hv_value - 1.0)?;
        self.set_int32(self.params.hv_state, 1)?;
        let _ = self.set_cooling_and_hv();
        self.set_float64(self.params.hv_value, hv_value)?;
        self.set_int32(self.params.hv_state, hv_state)?;
        let status = self.set_cooling_and_hv();

        self.set_int32(self.params.system_reset, 0)?;
        status
    }
}

impl PortDriver for PixiradDetector {
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

        let p = self.ad.params;
        let q = self.params;
        let mut status = Ok(());
        if reason == p.acquire {
            if value != 0 && !was_acquiring {
                status = self.start_acquire();
            } else if value == 0 && was_acquiring {
                status = self.command(&protocol::acquisition_break());
            }
        } else if reason == q.sync_in_polarity
            || reason == q.sync_out_polarity
            || reason == q.sync_out_function
        {
            status = self.set_sync();
        } else if reason == p.frame_type || reason == q.count_mode {
            status = self.set_thresholds(1);
        } else if reason == q.hv_state || reason == q.cooling_state {
            status = self.set_cooling_and_hv();
        } else if reason == q.system_reset {
            if value != 0 {
                status = self.system_reset();
            }
        } else if reason == q.auto_calibrate && value != 0 {
            status = self.do_auto_calibrate();
        } else if reason < q.system_reset {
            // FIRST_PIXIRAD_PARAM is PixiradSystemReset (pixirad.cpp:238).
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "pixirad:writeInt32: error, status={} function={reason}, value={value}\n",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("pixirad:writeInt32: function={reason}, value={value}\n"),
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
        let q = self.params;
        if reason == p.temperature || reason == q.hv_value {
            status = self.set_cooling_and_hv();
        } else if reason == q.hit_threshold || q.threshold.contains(&reason) {
            status = self.set_thresholds(1);
        } else if reason < q.system_reset {
            // ADDriver::writeFloat64 is asynPortDriver's (pixirad.cpp:1469).
            status = self.ad.port_base.write_float64_param(user, value);
        }

        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "pixirad:writeFloat64 error, status={} function={reason}, value={value:.6}\n",
                    e.status() as i32,
                    value = CFloat(value)
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "pixirad:writeFloat64: function={reason}, value={value:.6}\n",
                    value = CFloat(value)
                ),
            ),
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
        status
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        if reason != self.params.autocal_conf {
            // C has no writeOctet and inherits `ADDriver::writeOctet`.
            return self.ad.write_octet(user, value);
        }
        let text = String::from_utf8_lossy(value)
            .trim_end_matches('\0')
            .to_string();
        self.ad
            .port_base
            .set_string_param(reason, user.addr, text.clone())?;

        if reason == self.params.autocal_conf {
            // `pixiradAutoCal ofs0 fs0 ofs2 fs1 fs2 Ibias vbgMcalDAC`
            // (C `setAutoCalParams`, which the iocsh command reached by
            // looking the driver up by port name).
            let values: Vec<i32> = text
                .split_whitespace()
                .filter_map(|t| t.parse().ok())
                .collect();
            if values.len() == 7 {
                self.vbg_mcal_dac = values[6];
                let command = protocol::set_piii_conf(
                    values[0], values[1], values[2], values[3], values[4], values[5],
                );
                self.command(&command)?;
            }
        }

        self.ad.port_base.call_param_callbacks(0)?;
        Ok(value.len())
    }
}

impl ADDriver for PixiradDetector {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

/// What the IOC layer holds on to.
pub struct PixiradRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub params: PixiradParams,
    pool: DriverPool,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)]
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl PixiradRuntime {
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

/// Create the detector port and start its three threads (C `pixiradConfig`).
///
/// `command_handle` is the user of the `drvAsynIPPort` that reaches the box on
/// TCP 2222 — one with no port when the startup script named none, since C
/// ignores the connect status (pixirad.cpp:471-472);
/// `data_port` and `status_port` are the UDP ports it broadcasts on.
#[allow(clippy::too_many_arguments)]
pub fn create_pixirad_detector(
    claim: &PortClaim,
    command_handle: SyncIOHandle,
    data_port: u16,
    status_port: u16,
    max_data_port_buffers: usize,
    max_size_x: i32,
    max_size_y: i32,
    max_memory: usize,
) -> AsynResult<PixiradRuntime> {
    let port_name = claim.name();
    let max_buffers = max_data_port_buffers.max(1);
    let shared = Arc::new(SharedState {
        queued_frames: AtomicI32::new(0),
        max_buffers: max_buffers as i32,
    });

    let det = PixiradDetector::new(
        port_name,
        PixiradServer::new(command_handle),
        max_size_x,
        max_size_y,
        max_memory,
        shared.clone(),
    )?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();
    let sensor = det.sensor;
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let mut threads = vec![task::start_status_task(
        status_port,
        StatusContext {
            handle: runtime_handle.port_handle().clone(),
            ad_params,
            params,
        },
    )];
    // C starts the data tasks on an uninitialised sensor; without one there is
    // no frame to read.
    if let Some(sensor) = sensor {
        let (frame_tx, frame_rx) = std::sync::mpsc::sync_channel::<Frame>(max_buffers);
        threads.push(task::start_udp_listener(
            data_port,
            runtime_handle.port_handle().clone(),
            sensor,
            frame_tx,
            shared.clone(),
        ));
        threads.push(task::start_data_task(DataContext {
            handle: runtime_handle.port_handle().clone(),
            output: ArrayPublisher::new(array_output.clone()),
            pool: pool.clone(),
            ad_params,
            params,
            sensor,
            shared: shared.clone(),
            frames: frame_rx,
        }));
    }

    Ok(PixiradRuntime {
        runtime_handle,
        ad_params,
        params,
        pool,
        array_output,
        threads,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::time::Duration;

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;

    /// A box that answers `GOT:` to every command except those containing
    /// `reject`; `None` is a box that never accepts the connection.
    fn detector(name: &str, reject: Option<&'static str>) -> PixiradDetector {
        detector_sized(name, reject, 1 << 20)
    }

    fn detector_sized(
        name: &str,
        reject: Option<&'static str>,
        max_memory: usize,
    ) -> PixiradDetector {
        let answer = reject.map(|reject| {
            move |command: &str| -> Vec<u8> {
                if command.contains(reject) {
                    b"NAK".to_vec()
                } else {
                    b"GOT: OK".to_vec()
                }
            }
        });
        detector_answering(name, answer, max_memory)
    }

    /// A box that sends `answer(command)` back for each command; `None` is a
    /// box that never accepts the connection.
    fn detector_answering(
        name: &str,
        answer: Option<impl Fn(&str) -> Vec<u8> + Send + 'static>,
        max_memory: usize,
    ) -> PixiradDetector {
        let addr = match answer {
            None => "127.0.0.1:1".to_string(),
            Some(answer) => {
                let listener = TcpListener::bind("127.0.0.1:0").unwrap();
                let addr = listener.local_addr().unwrap().to_string();
                std::thread::spawn(move || {
                    let (mut stream, _) = listener.accept().unwrap();
                    let mut buf = [0u8; 512];
                    while let Ok(n) = stream.read(&mut buf) {
                        if n == 0 {
                            break;
                        }
                        let reply = answer(&String::from_utf8_lossy(&buf[..n]));
                        if stream.write_all(&reply).is_err() {
                            break;
                        }
                    }
                });
                addr
            }
        };
        let ip = format!("{name}_IP");
        let driver = DrvAsynIPPort::new(&ip, &addr).unwrap();
        let (runtime, _) =
            create_port_runtime(&claim_port(&ip).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let shared = Arc::new(SharedState {
            queued_frames: AtomicI32::new(0),
            max_buffers: 1,
        });
        PixiradDetector::new(
            name,
            PixiradServer::new(
                SyncIOHandle::from_handle(handle, 0, Duration::ZERO, InterfaceType::Octet).unwrap(),
            ),
            476,
            512,
            max_memory,
            shared,
        )
        .expect("C builds the driver whatever the box answers")
    }

    /// C ignores the command port's connect status (pixirad.cpp:471-472): the
    /// driver is built on a user with no port and each command fails.
    #[test]
    fn a_missing_command_port_fails_each_command() {
        let (user, status) =
            SyncIOHandle::connect_port("PIXI_NONE_IP", 0, Duration::ZERO, InterfaceType::Octet);
        assert!(status.is_err());
        let shared = Arc::new(SharedState {
            queued_frames: AtomicI32::new(0),
            max_buffers: 1,
        });
        let mut det = PixiradDetector::new(
            "PIXI_NONE",
            PixiradServer::new(user),
            476,
            512,
            1 << 20,
            shared,
        )
        .expect("C builds the driver whatever the box answers");
        let q = det.params;
        assert!(det.write_int32(&mut AsynUser::new(q.hv_state), 0).is_err());
    }

    /// pixirad.cpp:557-559 formats DRIVER_VERSION.REVISION.MODIFICATION
    /// (pixirad.cpp:40-42).
    #[test]
    fn the_driver_version_is_the_c_driver_release() {
        let det = detector("PIXI_VERSION", None);
        assert_eq!(
            det.ad
                .port_base
                .get_string_param(det.ad.params.base.driver_version, 0)
                .unwrap(),
            b"2.2.0"
        );
    }

    /// C never sets PixiradHitThresh (pixirad.cpp:510-544), so the PINI ao
    /// keeps its database VAL instead of reading a driver default back.
    #[test]
    fn the_hit_threshold_starts_undefined() {
        let det = detector("PIXI_HIT_UNSET", None);
        assert!(
            det.ad
                .port_base
                .get_float64_param(det.params.hit_threshold, 0)
                .is_err()
        );
    }

    #[test]
    fn a_dead_box_still_sets_serial_number_and_firmware_version() {
        let det = detector("PIXI_DEAD_ID", None);
        let p = det.ad.params.base;
        for reason in [p.serial_number, p.firmware_version] {
            assert_eq!(det.ad.port_base.get_string_param(reason, 0).unwrap(), b"");
        }
    }

    /// pixirad.cpp:569-570 publishes the bytes after the marker as they came.
    #[test]
    fn system_info_keeps_the_raw_reply() {
        let det = detector_answering(
            "PIXI_SYSINFO",
            Some(|command: &str| -> Vec<u8> {
                if command.contains("GET_ADDITIONAL_INFO") {
                    b"ADDITIONAL INFO:  box 3 \t".to_vec()
                } else {
                    b"GOT: OK".to_vec()
                }
            }),
            1 << 20,
        );
        assert_eq!(
            det.ad
                .port_base
                .get_string_param(det.params.system_info, 0)
                .unwrap(),
            b"  box 3 \t"
        );
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        use epics_rs::ad_core::driver::ShutterMode;
        let mut det = detector("PIXI_BASE_WRITE", None);
        let ad = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(ad.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        det.write_int32(&mut AsynUser::new(ad.shutter_control), 1)
            .unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(ad.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }

    #[test]
    fn a_dead_box_is_the_status_of_a_write_that_talks_to_it() {
        let mut det = detector("PIXI_DEAD", None);
        let q = det.params;
        let e = det
            .write_int32(&mut AsynUser::new(q.hv_state), 0)
            .unwrap_err();
        assert_ne!(e.status(), AsynStatus::Success);
        assert_eq!(e.message(), "", "C returns writeReadServer's bare status");
        assert!(
            det.write_float64(&mut AsynUser::new(q.hit_threshold), 1.0)
                .is_err()
        );
    }

    #[test]
    fn a_failed_exchange_leaves_the_status_message() {
        let mut det = detector("PIXI_DEAD_MSG", None);
        let q = det.params;
        let _ = det.write_int32(&mut AsynUser::new(q.hv_state), 0);
        assert_eq!(
            det.ad
                .port_base
                .get_string_param(det.ad.params.status_message, 0)
                .unwrap(),
            b""
        );
    }

    #[test]
    fn a_reply_without_got_is_an_error() {
        let mut det = detector("PIXI_NAK", Some("DAQ:! INIT"));
        let q = det.params;
        let e = det
            .write_int32(&mut AsynUser::new(q.cooling_state), 1)
            .unwrap_err();
        assert_eq!(e.status(), AsynStatus::Error);
        assert_eq!(e.message(), "");
        // A command the box takes is a success.
        det.write_int32(&mut AsynUser::new(q.sync_in_polarity), 1)
            .unwrap();
    }

    /// A status task on a box that takes every command, fed broadcasts by the
    /// test; its port is open, as after iocInit.
    struct StatusTask {
        handle: PortHandle,
        ad: ADBaseParams,
        params: PixiradParams,
        port: u16,
    }

    impl StatusTask {
        fn start(name: &str) -> Self {
            let det = detector(name, Some("NEVER"));
            let (ad, params) = (det.ad.params, det.params);
            let claim = claim_port(name).unwrap();
            let (runtime, _) = create_port_runtime(&claim, det, RuntimeConfig::default()).unwrap();
            let handle = runtime.port_handle().clone();
            claim
                .publish(handle.clone(), &epics_rs::asyn::registry::open_owner())
                .unwrap();
            std::mem::forget(runtime);
            let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            let port = socket.local_addr().unwrap().port();
            task::start_status_task_on(
                socket,
                StatusContext {
                    handle: handle.clone(),
                    ad_params: ad,
                    params,
                },
            );
            Self {
                handle,
                ad,
                params,
                port,
            }
        }

        fn published(&self, reason: usize) -> Vec<u8> {
            self.handle
                .with_driver_blocking(move |d: &mut PixiradDetector| {
                    d.ad.port_base.get_string_param(reason, 0).unwrap().to_vec()
                })
                .unwrap()
        }

        /// Broadcast a cold side at 45 C (TColdError) until the cooling-off
        /// exchange is published.
        fn overheat(&self) {
            let message = "READ_TCOLD 45.0 READ_THOT 25.0 READ_BOX_TEMP 20.0 READ_BOX_HUM 10.0 \
                           READ_PELTIER_PWR 1.0 READ_HV 350.0 READ_HV_CURRENT 0.1";
            let sender = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(20);
            while !self
                .published(self.ad.string_to_server)
                .starts_with(b"DAQ:! INIT")
            {
                assert!(
                    std::time::Instant::now() < deadline,
                    "the cooling-off exchange never reached ADStringToServer"
                );
                sender
                    .send_to(message.as_bytes(), ("127.0.0.1", self.port))
                    .unwrap();
                std::thread::sleep(Duration::from_millis(200));
            }
        }
    }

    /// C statusTask switches the cooling off through `setCoolingAndHV`, whose
    /// `writeReadServer` publishes the exchange (pixirad.cpp:1363-1369, :916-924).
    #[test]
    fn the_status_task_publishes_its_cooling_off_exchange() {
        let st = StatusTask::start("PIXI_STATUS_OFF");
        st.overheat();
        let command = protocol::init(INITIAL_COOLING_VALUE, 0, INITIAL_HV_VALUE, 1);
        assert_eq!(st.published(st.ad.string_to_server), command.as_bytes());
        assert_eq!(st.published(st.ad.string_from_server), b"GOT: OK");
        assert_eq!(st.published(st.ad.status_message), b"Server returned OK");
    }

    /// C statusTask posts the readings, CoolingState and the exchange in one
    /// `callParamCallbacks` (pixirad.cpp:1325-1370), which posts in the order
    /// the parameters were set (asynPortDriver.cpp:841-843).
    #[test]
    fn the_status_task_posts_its_cooling_off_in_one_pass() {
        let st = StatusTask::start("PIXI_STATUS_PASS");
        let mut posts = st.handle.interrupts().subscribe_async(256, 0);
        st.overheat();
        let mut reasons = Vec::new();
        while let Ok(v) = posts.try_recv() {
            reasons.push(v.reason);
        }
        // The open flush can still be delivering when the subscription starts;
        // the status pass is what follows its last reading.
        let pass = reasons
            .iter()
            .rposition(|&r| r == st.ad.temperature_actual)
            .map_or(&[][..], |at| &reasons[at..]);
        let p = st.params;
        assert_eq!(
            pass,
            [
                st.ad.temperature_actual,
                p.hot_temperature,
                p.box_temperature,
                p.box_humidity,
                p.peltier_power,
                p.hv_actual,
                p.hv_current,
                p.dew_point,
                p.cooling_status,
                p.cooling_state,
                st.ad.string_to_server,
            ]
        );
    }

    /// C udpDataListenerTask posts UDPBuffersRead, UDPBuffersFree and
    /// UDPSpeed once it has queued a frame, whether or not dataTask has taken
    /// it (pixirad.cpp:1234-1245).
    #[test]
    fn the_udp_listener_posts_each_queued_frame() {
        let det = detector("PIXI_UDP", Some("NEVER"));
        let (params, sensor, shared) = (det.params, det.sensor.unwrap(), det.shared.clone());
        let (runtime, _) = create_port_runtime(
            &claim_port("PIXI_UDP").unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let port = std::net::UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        // No dataTask: the frame stays queued.
        let (frame_tx, _frame_rx) = std::sync::mpsc::sync_channel(1);
        task::start_udp_listener(port, handle.clone(), sensor, frame_tx, shared);

        let sender = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let read = || {
            handle
                .read_int32_blocking(params.udp_buffers_read, 0)
                .unwrap()
        };
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        // Until the listener is bound, a frame goes nowhere: send it again.
        while read() == 0 {
            assert!(
                std::time::Instant::now() < deadline,
                "the listener posted no UDPBuffersRead"
            );
            for id in 0..sensor.num_udp_packets {
                let mut packet = vec![0u8; MAX_UDP_PACKET_LEN];
                packet[PACKET_ID_OFFSET..PACKET_ID_OFFSET + 2]
                    .copy_from_slice(&((id % DAQ_PACKET_FRAGMENT) as u16).to_be_bytes());
                sender.send_to(&packet, ("127.0.0.1", port)).unwrap();
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        assert_eq!(read(), 1);
        assert_eq!(
            handle
                .read_int32_blocking(params.udp_buffers_free, 0)
                .unwrap(),
            0
        );
        let speed = handle.read_float64_blocking(params.udp_speed, 0).unwrap();
        assert!(speed > 0.0, "UDPSpeed {speed}");
    }

    #[test]
    fn auto_calibrate_reports_the_autocal_status_alone() {
        let mut det = detector("PIXI_AUTOCAL_NAK", Some("AUTOCAL"));
        let q = det.params;
        assert!(
            det.write_int32(&mut AsynUser::new(q.auto_calibrate), 1)
                .is_err()
        );

        let mut det = detector("PIXI_THRESH_NAK", Some("SET_SENSOR_OPERATINGS"));
        det.write_int32(&mut AsynUser::new(q.auto_calibrate), 1)
            .unwrap();
    }

    /// The data task of a detector on a box that takes every command, fed
    /// frames by the test instead of the UDP listener.
    struct DataTask {
        handle: PortHandle,
        ad: ADBaseParams,
        params: PixiradParams,
        frames: std::sync::mpsc::SyncSender<Frame>,
        arrays: epics_rs::ad_core::plugin::channel::NDArrayReceiver,
        frame_bytes: usize,
        pool: DriverPool,
    }

    impl DataTask {
        fn start(name: &str, max_memory: usize) -> Self {
            let det = detector_sized(name, Some("NEVER"), max_memory);
            let (ad, params, pool, sensor) = (
                det.ad.params,
                det.params,
                det.ad.pool.clone(),
                det.sensor.unwrap(),
            );
            let shared = det.shared.clone();
            let (runtime, _) =
                create_port_runtime(&claim_port(name).unwrap(), det, RuntimeConfig::default())
                    .unwrap();
            let handle = runtime.port_handle().clone();
            std::mem::forget(runtime);
            let (sink, arrays) =
                epics_rs::ad_core::plugin::channel::ndarray_channel("PIXI_SINK", 8);
            let mut output = NDArrayOutput::new();
            output.add(sink);
            // A rendezvous: a send returns once the task is back for the next
            // frame, so the frame before it is done.
            let (frames, frame_rx) = std::sync::mpsc::sync_channel(0);
            task::start_data_task(DataContext {
                handle: handle.clone(),
                output: ArrayPublisher::new(Arc::new(parking_lot::Mutex::new(output))),
                pool: pool.clone(),
                ad_params: ad,
                params,
                sensor,
                shared,
                frames: frame_rx,
            });
            Self {
                handle,
                ad,
                params,
                frames,
                arrays,
                frame_bytes: crate::decode::Decoder::new(sensor).frame_bytes(false),
                pool,
            }
        }

        fn send_frame(&self) {
            self.frames
                .send(Frame {
                    is_autocal: false,
                    align_errors: false,
                    payload: vec![0; self.frame_bytes],
                })
                .unwrap();
        }

        fn counter(&self) -> i32 {
            self.handle
                .read_int32_blocking(self.ad.base.array_counter, 0)
                .unwrap()
        }
    }

    #[test]
    fn every_image_carries_its_own_start_time() {
        let mut t = DataTask::start("PIXI_TS", 1 << 20);
        t.handle
            .write_int32_blocking(t.ad.base.array_callbacks, 0, 1)
            .unwrap();
        t.send_frame();
        let first = t.arrays.blocking_recv().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(5));
        t.send_frame();
        let second = t.arrays.blocking_recv().unwrap();
        assert!(second.timestamp.as_f64() > first.timestamp.as_f64());
    }

    /// dataTask stores no array size, dimension or time-stamp param per
    /// image; the constructor's values stand (pixirad.cpp:1051-1079).
    #[test]
    fn an_image_leaves_the_array_params_alone() {
        let mut t = DataTask::start("PIXI_PARAMS", 1 << 20);
        t.handle
            .write_int32_blocking(t.ad.base.array_callbacks, 0, 1)
            .unwrap();
        t.send_frame();
        t.arrays.blocking_recv().unwrap();
        let int = |reason| t.handle.read_int32_blocking(reason, 0).unwrap();
        assert_eq!(int(t.ad.base.array_size), 0);
        assert_eq!(int(t.ad.base.n_dimensions), 0);
        assert_eq!(int(t.ad.base.epics_ts_sec), 0);
        assert_eq!(int(t.ad.base.epics_ts_nsec), 0);
        assert_eq!(
            t.handle
                .read_float64_blocking(t.ad.base.timestamp_rbv, 0)
                .unwrap(),
            0.0
        );
    }

    /// C queues the last image on the plugins before it stores Acquire=0
    /// (pixirad.cpp:1066-1077), so under WaitForPlugins the queued image
    /// holds AcquireBusy (asynNDArrayDriver.cpp:640-647).
    #[test]
    fn the_last_image_is_queued_before_acquire_drops() {
        let t = DataTask::start("PIXI_BUSY", 1 << 20);
        for (reason, value) in [
            (t.ad.base.array_callbacks, 1),
            (t.ad.wait_for_plugins, 1),
            (t.ad.num_images, 1),
            (t.ad.acquire, 1),
        ] {
            t.handle.write_int32_blocking(reason, 0, value).unwrap();
        }
        t.send_frame();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while t.counter() != 1 {
            assert!(std::time::Instant::now() < deadline, "no image");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        let int = |reason| t.handle.read_int32_blocking(reason, 0).unwrap();
        assert_eq!(int(t.ad.acquire), 0);
        assert_eq!(
            int(t.ad.acquire_busy),
            1,
            "AcquireBusy cleared with the image unqueued"
        );
        assert_eq!(t.arrays.pending(), 1);
    }

    /// C bumps NDArrayCounter only after the colour-0 alloc
    /// (pixirad.cpp:1033, :1055-1057).
    #[test]
    fn a_frame_the_pool_refuses_leaves_the_counter_alone() {
        // Room for one 476 x 512 NDUInt16 image.
        let mut t = DataTask::start("PIXI_REFUSE", 600_000);
        t.handle
            .write_int32_blocking(t.ad.base.array_callbacks, 0, 1)
            .unwrap();
        t.send_frame();
        let held = t.arrays.blocking_recv().unwrap();
        assert_eq!(held.unique_id, 1);
        t.send_frame();
        t.send_frame();
        assert_eq!(t.counter(), 1);
        drop(held);
        t.send_frame();
        assert_eq!(t.arrays.blocking_recv().unwrap().unique_id, 2);
    }

    /// C fills colour `colorsCollected` of the image the colour-0 frame
    /// allocated (pixirad.cpp:1023-1044): after a refused colour-0 alloc the
    /// next frame is still colour 1, and a new image starts at colour 0.
    #[test]
    fn a_refused_colour_zero_loses_its_image_not_the_colour_order() {
        // Room for one 476 x 512 x 2 NDUInt16 image.
        let mut t = DataTask::start("PIXI_COLOUR", 1_000_000);
        t.handle
            .write_int32_blocking(t.ad.base.array_callbacks, 0, 1)
            .unwrap();
        t.handle
            .write_int32_blocking(t.ad.frame_type, 0, FrameType::TwoColors as i32)
            .unwrap();
        t.send_frame();
        t.send_frame();
        let held = t.arrays.blocking_recv().unwrap();
        assert_eq!(held.unique_id, 1);

        t.send_frame(); // colour 0, refused
        // The rendezvous returns once the refused frame is done.
        t.send_frame(); // colour 1 of the refused image
        let colors_collected = t
            .handle
            .read_int32_blocking(t.params.colors_collected, 0)
            .unwrap();
        assert_eq!(colors_collected, 1, "the refused frame took no colour slot");
        drop(held);
        t.send_frame(); // colour 0 of the next
        assert_eq!(t.arrays.pending(), 0);
        t.send_frame();
        assert_eq!(t.arrays.blocking_recv().unwrap().unique_id, 2);
    }

    /// The colour-0 alloc does not depend on NDArrayCallbacks.
    #[test]
    fn an_image_is_allocated_from_the_pool_with_callbacks_off() {
        let t = DataTask::start("PIXI_NOCB", 1 << 20);
        t.handle
            .write_int32_blocking(t.ad.base.array_callbacks, 0, 0)
            .unwrap();
        t.send_frame();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while t.counter() != 1 {
            assert!(std::time::Instant::now() < deadline, "no image");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(t.pool.num_alloc_buffers(), 1);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (pixirad.cpp:1469), whose
    /// IO_DRIVER line precedes the driver's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "PIXI_BASE_F64";
        let mut det = detector(port, None);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let delay = det.ad.params.shutter_open_delay;
        let mut user = AsynUser::new(delay).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        let _ = det.write_float64(&mut user, 2.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("pixirad:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }

    /// pixirad.cpp:1106-1121, :1289-1303: the UDP sockets are bound inside the
    /// tasks, so a port already in use stops that task and leaves the driver.
    #[test]
    fn a_udp_port_in_use_leaves_the_driver() {
        let data = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
        let status = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
        let (command, _) =
            SyncIOHandle::connect_port("PIXI_BUSY_IP", 0, Duration::ZERO, InterfaceType::Octet);
        let rt = create_pixirad_detector(
            &claim_port("PIXI_BUSY").unwrap(),
            command,
            data.local_addr().unwrap().port(),
            status.local_addr().unwrap().port(),
            1,
            476,
            512,
            1 << 20,
        )
        .expect("C builds the driver before its tasks bind");
        assert_eq!(rt.port_handle().port_name(), "PIXI_BUSY");
    }

    /// pixirad.cpp:510-517, :574-631: C sets the size parameters before it
    /// looks the size up, prints an illegal one and goes on.
    #[test]
    fn an_unknown_size_still_makes_the_port() {
        for (name, x, y) in [("PIXI_ODD_X", 1024, 1024), ("PIXI_ODD_Y", 476, 640)] {
            let (command, _) = SyncIOHandle::connect_port(
                &format!("{name}_IP"),
                0,
                Duration::ZERO,
                InterfaceType::Octet,
            );
            let rt = create_pixirad_detector(
                &claim_port(name).unwrap(),
                command,
                0,
                0,
                1,
                x,
                y,
                1 << 20,
            )
            .expect("C makes the port whatever the size");
            let read = |reason| rt.port_handle().read_int32_blocking(reason, 0).unwrap();
            assert_eq!(read(rt.ad_params.max_size_x), x);
            assert_eq!(read(rt.ad_params.max_size_y), y);
            assert_eq!(read(rt.ad_params.size_x), x);
            assert_eq!(read(rt.ad_params.base.array_size_y), y);
        }
    }
}
