//! The `PortDriver` half of the Pilatus port: parameter writes and the
//! bookkeeping C does inline in `writeInt32` / `writeFloat64` / `writeOctet`.
//!
//! Camserver I/O is not done here — a `PortDriver` method runs inside the port
//! actor, whose runtime cannot block on a second port — so each branch enqueues
//! the corresponding [`Cmd`] for `PilatusCmdTask`.

use std::sync::Arc;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverPool, PersistentArray};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;
use parking_lot::Mutex;

use crate::camserver::Ctx;
use crate::params::PilatusParams;
use crate::protocol::{
    cmd_cbf_template_file, cmd_mx_f64, cmd_mx_i32, cmd_mx_pair, cmd_oscill_axis,
};
use crate::task::{Cmd, Shared, Worker, start_acq_task, start_cmd_task};
use crate::types::{Event, TriggerMode};

/// C `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION`.
const DRIVER_VERSION: &str = "2.9.0";

pub struct PilatusDriver {
    pub ad: ADDriverBase,
    pub p: PilatusParams,
    cmd_tx: rt::CommandSender<Cmd>,
    start: Arc<Event>,
    stop: Arc<Event>,
    shared: Arc<Mutex<Shared>>,
}

/// The handles the driver hands to (and shares with) its worker threads.
pub(crate) struct DriverLinks {
    pub(crate) cmd_tx: rt::CommandSender<Cmd>,
    pub(crate) start: Arc<Event>,
    pub(crate) stop: Arc<Event>,
    pub(crate) shared: Arc<Mutex<Shared>>,
}

impl PilatusDriver {
    pub(crate) fn new(
        port_name: &str,
        max_size_x: i32,
        max_size_y: i32,
        max_memory: usize,
        links: DriverLinks,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            max_size_x,
            max_size_y,
            max_memory,
            ASYN_CANBLOCK,
        )?;
        let p = PilatusParams::create(&mut ad.port_base)?;

        let params = ad.params;
        let base = &mut ad.port_base;
        base.set_string_param(params.base.manufacturer, 0, "Dectris")?;
        base.set_string_param(params.base.model, 0, "Pilatus")?;
        base.set_string_param(params.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_int32_param(params.max_size_x, 0, max_size_x)?;
        base.set_int32_param(params.max_size_y, 0, max_size_y)?;
        base.set_int32_param(params.size_x, 0, max_size_x)?;
        base.set_int32_param(params.size_y, 0, max_size_y)?;
        base.set_int32_param(params.base.array_size_x, 0, max_size_x)?;
        base.set_int32_param(params.base.array_size_y, 0, max_size_y)?;
        base.set_int32_param(params.base.array_size, 0, 0)?;
        base.set_int32_param(params.base.data_type, 0, NDDataType::UInt32 as i32)?;
        base.set_int32_param(params.image_mode, 0, ImageMode::Continuous as i32)?;
        base.set_int32_param(params.trigger_mode, 0, TriggerMode::Internal as i32)?;

        base.set_int32_param(p.armed, 0, 0)?;
        base.set_int32_param(p.reset_power, 0, 0)?;
        base.set_int32_param(p.reset_power_time, 0, 1)?;
        base.set_string_param(p.bad_pixel_file, 0, String::new())?;
        base.set_int32_param(p.num_bad_pixels, 0, 0)?;
        base.set_string_param(p.flat_field_file, 0, String::new())?;
        base.set_int32_param(p.flat_field_valid, 0, 0)?;

        base.set_float64_param(p.th_temp_0, 0, 0.0)?;
        base.set_float64_param(p.th_temp_1, 0, 0.0)?;
        base.set_float64_param(p.th_temp_2, 0, 0.0)?;
        base.set_float64_param(p.th_humid_0, 0, 0.0)?;
        base.set_float64_param(p.th_humid_1, 0, 0.0)?;
        base.set_float64_param(p.th_humid_2, 0, 0.0)?;
        base.set_string_param(p.tvx_version, 0, "Unknown")?;
        base.set_string_param(p.header_string, 0, String::new())?;

        Ok(Self {
            ad,
            p,
            cmd_tx: links.cmd_tx,
            start: links.start,
            stop: links.stop,
            shared: links.shared,
        })
    }

    fn f64_or_zero(&self, reason: usize) -> f64 {
        self.ad
            .port_base
            .get_float64_param(reason, 0)
            .unwrap_or(0.0)
    }

    fn send(&self, cmd: Cmd) {
        let _ = self.cmd_tx.try_send(cmd);
    }
}

impl PortDriver for PilatusDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // Ensure that ADStatus is set correctly before we set ADAcquire.
        let adstatus = self
            .ad
            .port_base
            .get_int32_param(params.status, 0)
            .unwrap_or(ADStatus::Idle as i32);
        let can_start = adstatus == ADStatus::Idle as i32
            || adstatus == ADStatus::Error as i32
            || adstatus == ADStatus::Aborted as i32;
        let acquiring = adstatus == ADStatus::Acquire as i32;

        if function == params.acquire {
            if value != 0 && can_start {
                self.ad
                    .port_base
                    .set_string_param(params.status_message, 0, "Acquiring data")?;
                self.ad
                    .port_base
                    .set_int32_param(params.status, 0, ADStatus::Acquire as i32)?;
            }
            if value == 0 && acquiring {
                self.ad.port_base.set_string_param(
                    params.status_message,
                    0,
                    "Acquisition aborted",
                )?;
                self.ad
                    .port_base
                    .set_int32_param(params.status, 0, ADStatus::Aborted as i32)?;
            }
        }
        self.ad.port_base.call_param_callbacks(0)?;

        let mut status = Ok(());
        if function == params.acquire {
            self.ad.set_acquire(value)?;
        } else {
            self.ad
                .port_base
                .set_int32_param(function, user.addr, value)?;
        }

        if function == params.acquire {
            if value != 0 && can_start {
                // Wake up the Pilatus task.
                self.start.signal();
            }
            if value == 0 && acquiring {
                self.stop.signal();
                self.send(Cmd::AbortAcquire);
            }
        } else if function == params.trigger_mode
            || function == params.num_images
            || function == params.num_exposures
            || function == p.gap_fill
        {
            self.send(Cmd::SetAcquireParams);
        } else if function == p.threshold_apply {
            self.send(Cmd::SetThreshold);
        } else if function == p.reset_power {
            self.send(Cmd::ResetModulePower);
        } else if function == p.num_oscill {
            self.send(Cmd::Raw(cmd_mx_i32("N_oscillations", value)));
        } else if function == params.read_status {
            if !acquiring {
                self.send(Cmd::ReadStatus);
            }
        } else if function < p.first() {
            status = ADDriver::ad_write_int32(self, user, value);
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "pilatusDetector:writeInt32: error, status={} function={function}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(()) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("pilatusDetector:writeInt32: function={function}, value={value}"),
            ),
        }
        status
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // Set the parameter and readback. This may be overwritten below.
        let old_value = self
            .ad
            .port_base
            .get_float64_param(function, 0)
            .unwrap_or(0.0);
        self.ad
            .port_base
            .set_float64_param(function, user.addr, value)?;

        if function == params.gain || function == p.energy || function == p.threshold {
            let auto_apply = self
                .ad
                .port_base
                .get_int32_param(p.threshold_auto_apply, 0)
                .unwrap_or(0);
            {
                let mut shared = self.shared.lock();
                if function == p.threshold {
                    shared.demanded_threshold = value;
                }
                if function == p.energy {
                    shared.demanded_energy = value;
                }
            }
            if auto_apply != 0 {
                let demanded = {
                    let shared = self.shared.lock();
                    if function == p.threshold {
                        Some(shared.demanded_threshold)
                    } else if function == p.energy {
                        Some(shared.demanded_energy)
                    } else {
                        None
                    }
                };
                if let Some(v) = demanded {
                    self.ad
                        .port_base
                        .set_float64_param(function, user.addr, v)?;
                }
                self.send(Cmd::SetThreshold);
            } else {
                // Put the old value back if we are deferring the threshold.
                if function == p.threshold || function == p.energy {
                    self.ad
                        .port_base
                        .set_float64_param(function, user.addr, old_value)?;
                }
            }
        } else if function == params.acquire_time
            || function == params.acquire_period
            || function == p.delay_time
        {
            self.send(Cmd::SetAcquireParams);
        } else if function == p.wavelength {
            self.send(Cmd::Raw(cmd_mx_f64("Wavelength", value)));
        } else if function == p.energy_low || function == p.energy_high {
            // C ignores both statuses and gets 0 (pilatusDetector.cpp:1511-1512).
            let low = self.f64_or_zero(p.energy_low);
            let high = self.f64_or_zero(p.energy_high);
            self.send(Cmd::Raw(cmd_mx_pair("Energy_range", low, high)));
        } else if function == p.det_dist {
            self.send(Cmd::Raw(cmd_mx_f64("Detector_distance", value / 1000.0)));
        } else if function == p.det_voffset {
            self.send(Cmd::Raw(cmd_mx_f64("Detector_Voffset", value / 1000.0)));
        } else if function == p.beam_x || function == p.beam_y {
            // As above (pilatusDetector.cpp:1523-1524).
            let x = self.f64_or_zero(p.beam_x);
            let y = self.f64_or_zero(p.beam_y);
            self.send(Cmd::Raw(cmd_mx_pair("Beam_xy", x, y)));
        } else if function == p.flux {
            self.send(Cmd::Raw(cmd_mx_f64("Flux", value)));
        } else if function == p.filter_transm {
            self.send(Cmd::Raw(cmd_mx_f64("Filter_transmission", value)));
        } else if function == p.start_angle {
            self.send(Cmd::Raw(cmd_mx_f64("Start_angle", value)));
        } else if function == p.angle_incr {
            self.send(Cmd::Raw(cmd_mx_f64("Angle_increment", value)));
        } else if function == p.det_2theta {
            self.send(Cmd::Raw(cmd_mx_f64("Detector_2theta", value)));
        } else if function == p.polarization {
            self.send(Cmd::Raw(cmd_mx_f64("Polarization", value)));
        } else if function == p.alpha {
            self.send(Cmd::Raw(cmd_mx_f64("Alpha", value)));
        } else if function == p.kappa {
            self.send(Cmd::Raw(cmd_mx_f64("Kappa", value)));
        } else if function == p.phi {
            self.send(Cmd::Raw(cmd_mx_f64("Phi", value)));
        } else if function == p.phi_incr {
            self.send(Cmd::Raw(cmd_mx_f64("Phi_increment", value)));
        } else if function == p.chi {
            self.send(Cmd::Raw(cmd_mx_f64("Chi", value)));
        } else if function == p.chi_incr {
            self.send(Cmd::Raw(cmd_mx_f64("Chi_increment", value)));
        } else if function == p.omega {
            self.send(Cmd::Raw(cmd_mx_f64("Omega", value)));
        } else if function == p.omega_incr {
            self.send(Cmd::Raw(cmd_mx_f64("Omega_increment", value)));
        }
        // Parameters below FIRST_PILATUS_PARAM need no extra work beyond the
        // `set_float64` above, which is all `ADDriver::writeFloat64` does.

        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("pilatusDetector:writeFloat64: function={function}, value={value:.6}"),
        );
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // C receives a NUL-terminated `char *`; waveform records pad with NULs.
        let payload = data.split(|&b| b == 0).next().unwrap_or(&[]);
        let value = String::from_utf8_lossy(payload).into_owned();

        self.ad
            .port_base
            .set_string_param(function, user.addr, value.clone())?;

        let mut status = Ok(());
        if function == p.bad_pixel_file {
            self.send(Cmd::ReadBadPixelFile(value.clone()));
        } else if function == p.flat_field_file {
            self.send(Cmd::ReadFlatFieldFile(value.clone()));
        } else if function == params.base.file_path {
            self.send(Cmd::ImgPath(value.clone()));
        } else if function == p.oscill_axis {
            self.send(Cmd::Raw(cmd_oscill_axis(&value)));
        } else if function == p.cbf_template_file {
            self.send(Cmd::Raw(cmd_cbf_template_file(&value)));
        } else {
            status = self.ad.write_octet(user, data).map(|_| ());
        }

        let _ = self.ad.port_base.call_param_callbacks(0);
        if let Err(e) = status {
            return Err(AsynError::Status {
                status: e.status(),
                message: format!(
                    "pilatusDetector:writeOctet: status={}, function={function}, value={value}",
                    e.status() as i32
                ),
            });
        }
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("pilatusDetector:writeOctet: function={function}, value={value}"),
        );
        Ok(data.len())
    }
}

impl ADDriver for PilatusDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

pub struct PilatusRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub ad_params: ADBaseParams,
    pub pilatus_params: PilatusParams,
    pool: DriverPool,
    array_output: Arc<Mutex<NDArrayOutput>>,
    /// Worker threads; kept alive for the IOC's lifetime.
    pub tasks: Vec<std::thread::JoinHandle<()>>,
}

impl PilatusRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &DriverPool {
        &self.pool
    }

    pub fn array_output(&self) -> &Arc<Mutex<NDArrayOutput>> {
        &self.array_output
    }

    pub fn connect_downstream(&self, sender: NDArraySender) {
        self.array_output.lock().add(sender);
    }
}

/// C `pilatusDetectorConfig`.
///
/// `camserver_port` must already exist (`drvAsynIPPortConfigure`), because C
/// connects to it with `pasynOctetSyncIO->connect` in the constructor.
pub fn create_pilatus_detector(
    claim: &PortClaim,
    camserver_port: &str,
    max_size_x: i32,
    max_size_y: i32,
    max_memory: usize,
) -> Result<PilatusRuntime, String> {
    let port_name = claim.name();
    let cam_entry = get_port(camserver_port).ok_or_else(|| {
        format!("camserver port '{camserver_port}' not found (call drvAsynIPPortConfigure first)")
    })?;
    let cam_handle = cam_entry.handle.clone();

    let (cmd_tx, cmd_rx) = rt::command_channel::<Cmd>(64);
    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());

    let shared = Arc::new(Mutex::new(Shared::default()));

    let driver = PilatusDriver::new(
        port_name,
        max_size_x,
        max_size_y,
        max_memory,
        DriverLinks {
            cmd_tx: cmd_tx.clone(),
            start: start.clone(),
            stop: stop.clone(),
            shared: shared.clone(),
        },
    )
    .map_err(|e| format!("failed to create Pilatus driver: {e}"))?;
    // C allocates pFlatField once, here (pilatusDetector.cpp:1726-1728).
    shared.lock().flat_field = driver
        .ad
        .pool
        .alloc(
            vec![
                NDDimension::new(max_size_x.max(0) as usize),
                NDDimension::new(max_size_y.max(0) as usize),
            ],
            NDDataType::Int32,
        )
        .ok()
        .map(PersistentArray::new);

    let ad_params = driver.ad.params;
    let pilatus_params = driver.p;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the Pilatus port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

    let cmd_worker = Worker {
        ctx: Ctx::new(
            handle.clone(),
            cam_handle.clone(),
            ad_params,
            pilatus_params,
            stop.clone(),
        ),
        shared: shared.clone(),
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
    };
    let acq_worker = Worker {
        ctx: Ctx::new(handle, cam_handle, ad_params, pilatus_params, stop),
        shared,
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
    };

    let tasks = vec![
        start_cmd_task(cmd_worker, cmd_rx),
        start_acq_task(acq_worker, start),
    ];

    // C always calls pilatusStatus() once at the end of the constructor to get
    // the TVX version.
    if cmd_tx.try_send(Cmd::ReadStatus).is_err() {
        return Err("pilatus: command task did not start".to_string());
    }

    Ok(PilatusRuntime {
        runtime_handle,
        ad_params,
        pilatus_params,
        pool,
        array_output,
        tasks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_base_class_write_reaches_addriver() {
        use epics_rs::ad_core::driver::ShutterMode;
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Cmd>(4);
        let links = DriverLinks {
            cmd_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
        };
        let mut det = PilatusDriver::new("PILATUS_BASE_WRITE", 8, 8, 0, links).unwrap();
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
    fn a_pair_write_sends_zero_for_its_unset_partner() {
        let (cmd_tx, mut cmd_rx) = rt::command_channel::<Cmd>(4);
        let links = DriverLinks {
            cmd_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
        };
        let mut det = PilatusDriver::new("PILATUS_PAIR", 8, 8, 0, links).unwrap();
        let p = det.p;

        for (reason, name) in [(p.energy_low, "Energy_range"), (p.beam_x, "Beam_xy")] {
            let mut user = AsynUser::new(reason);
            det.write_float64(&mut user, 2.5).unwrap();
            match cmd_rx.try_recv() {
                Ok(Cmd::Raw(cmd)) => {
                    assert_eq!(cmd, format!("mxsettings {name} 2.500000,0.000000"))
                }
                other => panic!("{name}: {other:?}"),
            }
        }
    }
}
