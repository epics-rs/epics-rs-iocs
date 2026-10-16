//! The Pixirad areaDetector driver (C `pixirad.cpp`).
//!
//! Ownership: every command to the box goes through [`PixiradServer`]; the port
//! actor owns the parameter library and is the only thing that reacts to a
//! record write. The three background threads — UDP data listener, data task,
//! status task — reach the parameter library through the actor and never call
//! into the driver directly (see the invariant in [`crate::connection`]).

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};

use crate::connection::{Exchange, PixiradServer};
use crate::params::PixiradParams;
use crate::protocol;
use crate::task::{self, DataContext, StatusContext};
use crate::thresholds;
use crate::types::*;
use crate::udp::Frame;

/// State the port actor shares with the background threads.
pub struct SharedState {
    /// Frames the UDP listener has completed (C `PixiradUDPBuffersRead`).
    pub udp_buffers_read: AtomicI32,
    /// Frames handed to the data task and not yet taken.
    pub queued_frames: AtomicI32,
    /// The size of the frame queue (C `maxDataPortBuffers`).
    pub max_buffers: i32,
    /// The last measured data rate, in MB/s, as `f64::to_bits`.
    pub udp_speed: AtomicU64,
}

impl SharedState {
    pub fn set_speed(&self, mb_per_second: f64) {
        self.udp_speed
            .store(mb_per_second.to_bits(), Ordering::Relaxed);
    }

    pub fn speed(&self) -> f64 {
        f64::from_bits(self.udp_speed.load(Ordering::Relaxed))
    }
}

pub struct PixiradDetector {
    pub ad: ADDriverBase,
    pub params: PixiradParams,
    server: PixiradServer,
    sensor: Sensor,
    /// Set by `pixiradAutoCal`; only the Pixie-III sends it.
    vbg_mcal_dac: i32,
    shared: Arc<SharedState>,
}

impl PixiradDetector {
    fn new(
        port_name: &str,
        server: PixiradServer,
        sensor: Sensor,
        max_size_x: i32,
        max_size_y: i32,
        max_memory: usize,
        shared: Arc<SharedState>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            max_size_x,
            max_size_y,
            max_memory,
            ASYN_CANBLOCK,
        )?;
        let params = PixiradParams::create(&mut ad.port_base)?;
        let p = ad.params;

        let base = &mut ad.port_base;
        base.set_string_param(p.base.manufacturer, 0, "Pixirad")?;
        base.set_string_param(p.base.model, 0, sensor.build.model_name())?;
        base.set_string_param(p.base.driver_version, 0, env!("CARGO_PKG_VERSION"))?;
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
        base.set_float64_param(params.hit_threshold, 0, 0.0)?;
        base.set_int32_param(params.sync_in_polarity, 0, SyncPolarity::Pos as i32)?;
        base.set_int32_param(params.sync_out_polarity, 0, SyncPolarity::Pos as i32)?;
        base.set_int32_param(params.sync_out_function, 0, SyncOutFunction::Shutter as i32)?;
        base.set_int32_param(params.system_reset, 0, 0)?;
        base.set_int32_param(params.cooling_status, 0, CoolingStatus::Ok as i32)?;

        let mut det = Self {
            ad,
            params,
            server,
            sensor,
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
        det.ad.port_base.call_param_callbacks(0)?;
        Ok(det)
    }

    /// Ask the box who it is (C's constructor).
    fn identify(&mut self) -> AsynResult<()> {
        let reply = self.exchange(&protocol::get_firmware_version())?.reply;
        if let Some((serial, version)) = protocol::parse_firmware_version(&reply) {
            let base = &mut self.ad.port_base;
            base.set_string_param(self.ad.params.base.serial_number, 0, serial)?;
            base.set_string_param(self.ad.params.base.firmware_version, 0, version)?;
        }

        let reply = self.exchange(&protocol::get_additional_info())?.reply;
        // C added the length of the marker to a null `strstr` result and
        // published whatever was at that address.
        if let Some(info) = protocol::parse_additional_info(&reply) {
            let info = info.trim().to_string();
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
        let message = match &exchange.result {
            Ok(()) => "Server returned OK",
            Err(_) => "Error from server",
        };
        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.status_message, 0, message)?;
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
        let t = thresholds::calculate(self.sensor.asic, &energies);

        for i in 0..4 {
            self.set_float64(self.params.threshold_actual[i], t.actual_energy[i])?;
        }
        self.set_float64(self.params.hit_threshold_actual, t.actual_energy[4])?;

        let readout_mode = self.frame_type().readout_mode();
        let command = match self.sensor.asic {
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
        self.set_int32(self.params.udp_buffers_read, 0)?;
        self.shared.udp_buffers_read.store(0, Ordering::Release);
        Ok(())
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
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "pixirad:writeInt32: error, status={} function={reason}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("pixirad:writeInt32: function={reason}, value={value}"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        self.ad
            .port_base
            .set_float64_param(reason, user.addr, value)?;

        let p = self.ad.params;
        let q = self.params;
        let mut status = Ok(());
        if reason == p.temperature || reason == q.hv_value {
            status = self.set_cooling_and_hv();
        } else if reason == q.hit_threshold || q.threshold.contains(&reason) {
            status = self.set_thresholds(1);
        }

        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "pixirad:writeFloat64 error, status={} function={reason}, value={value:.6}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("pixirad:writeFloat64: function={reason}, value={value:.6}"),
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
    pool: Arc<NDArrayPool>,
    array_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    #[allow(dead_code)]
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl PixiradRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
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
/// `command_handle` is the `drvAsynIPPort` that reaches the box on TCP 2222;
/// `data_port` and `status_port` are the UDP ports it broadcasts on.
#[allow(clippy::too_many_arguments)]
pub fn create_pixirad_detector(
    claim: &PortClaim,
    command_handle: PortHandle,
    data_port: u16,
    status_port: u16,
    max_data_port_buffers: usize,
    max_size_x: i32,
    max_size_y: i32,
    max_memory: usize,
) -> AsynResult<PixiradRuntime> {
    let port_name = claim.name();
    // C printed a message for a size it did not know and then decoded frames
    // with an uninitialised sensor description.
    let sensor = Sensor::from_size(max_size_x, max_size_y).ok_or_else(|| AsynError::Status {
        status: AsynStatus::Error,
        message: format!("pixirad: no Pixirad detector is {max_size_x} x {max_size_y}"),
    })?;

    let max_buffers = max_data_port_buffers.max(1);
    let shared = Arc::new(SharedState {
        udp_buffers_read: AtomicI32::new(0),
        queued_frames: AtomicI32::new(0),
        max_buffers: max_buffers as i32,
        udp_speed: AtomicU64::new(0),
    });

    let (data_socket, data_rcvbuf) =
        crate::udp::bind_data_socket(data_port).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("pixirad: cannot listen on UDP data port {data_port}: {e}"),
        })?;
    let status_socket =
        crate::udp::bind_status_socket(status_port).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("pixirad: cannot listen on UDP status port {status_port}: {e}"),
        })?;

    let server = PixiradServer::new(command_handle);
    let det = PixiradDetector::new(
        port_name,
        server.clone(),
        sensor,
        max_size_x,
        max_size_y,
        max_memory,
        shared.clone(),
    )?;
    let ad_params = det.ad.params;
    let params = det.params;
    let pool = det.ad.pool.clone();
    let (runtime_handle, _) = create_port_runtime(claim, det, RuntimeConfig::default())?;
    let array_output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));

    let (frame_tx, frame_rx) = std::sync::mpsc::sync_channel::<Frame>(max_buffers);

    let threads = vec![
        task::start_udp_listener(
            data_socket,
            data_rcvbuf,
            runtime_handle.port_handle().clone(),
            sensor,
            frame_tx,
            shared.clone(),
        ),
        task::start_data_task(DataContext {
            handle: runtime_handle.port_handle().clone(),
            output: ArrayPublisher::new(array_output.clone()),
            pool: pool.clone(),
            ad_params,
            params,
            sensor,
            shared: shared.clone(),
            frames: frame_rx,
        }),
        task::start_status_task(StatusContext {
            socket: status_socket,
            server,
            handle: runtime_handle.port_handle().clone(),
            ad_params,
            params,
        }),
    ];

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
    use std::io::{Read, Write};
    use std::net::TcpListener;

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;

    /// A box that answers `GOT:` to every command except those containing
    /// `reject`; `None` is a box that never accepts the connection.
    fn detector(name: &str, reject: Option<&'static str>) -> PixiradDetector {
        let addr = match reject {
            None => "127.0.0.1:1".to_string(),
            Some(reject) => {
                let listener = TcpListener::bind("127.0.0.1:0").unwrap();
                let addr = listener.local_addr().unwrap().to_string();
                std::thread::spawn(move || {
                    let (mut stream, _) = listener.accept().unwrap();
                    let mut buf = [0u8; 512];
                    while let Ok(n) = stream.read(&mut buf) {
                        if n == 0 {
                            break;
                        }
                        let command = String::from_utf8_lossy(&buf[..n]);
                        let reply: &[u8] = if command.contains(reject) {
                            b"NAK"
                        } else {
                            b"GOT: OK"
                        };
                        if stream.write_all(reply).is_err() {
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
            udp_buffers_read: AtomicI32::new(0),
            queued_frames: AtomicI32::new(0),
            max_buffers: 1,
            udp_speed: AtomicU64::new(0),
        });
        let sensor = Sensor::from_size(476, 512).unwrap();
        PixiradDetector::new(
            name,
            PixiradServer::new(handle),
            sensor,
            476,
            512,
            1 << 20,
            shared,
        )
        .expect("C builds the driver whatever the box answers")
    }

    #[test]
    fn a_dead_box_is_the_status_of_a_write_that_talks_to_it() {
        let mut det = detector("PIXI_DEAD", None);
        let q = det.params;
        let e = det
            .write_int32(&mut AsynUser::new(q.hv_state), 0)
            .unwrap_err();
        assert_ne!(e.status(), AsynStatus::Success);
        assert!(
            det.write_float64(&mut AsynUser::new(q.hit_threshold), 1.0)
                .is_err()
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
        // A command the box takes is a success.
        det.write_int32(&mut AsynUser::new(q.sync_in_polarity), 1)
            .unwrap();
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
}
