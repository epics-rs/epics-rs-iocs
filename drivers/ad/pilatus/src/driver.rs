//! The `PortDriver` half of the Pilatus port: parameter writes and what C does
//! inline in `writeInt32` / `writeFloat64` / `writeOctet`, camserver exchanges
//! included (see [`crate::port_thread`]).

use std::sync::Arc;
use std::time::Duration;

use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::write_tail::{OwnList, WriteTail};
use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverPool, PersistentArray};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;
use parking_lot::Mutex;

use crate::camserver::{Camserver, Ctx};
use crate::params::PilatusParams;
use crate::protocol::{
    cmd_cbf_template_file, cmd_mx_f64, cmd_mx_i32, cmd_mx_pair, cmd_oscill_axis,
};
use crate::task::{FlatFieldJob, Shared, Worker, start_acq_task, start_flat_field_task};
use crate::types::{CAMSERVER_DEFAULT_TIMEOUT, Event, TriggerMode};

/// C `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION`.
const DRIVER_VERSION: &str = "2.9.0";

pub struct PilatusDriver {
    pub ad: ADDriverBase,
    pub p: PilatusParams,
    /// Hands `readFlatFieldFile`'s array to `PilatusFlatField` to publish.
    pub(crate) flat_field_tx: rt::CommandSender<FlatFieldJob>,
    start: Arc<Event>,
    pub(crate) stop: Arc<Event>,
    pub(crate) shared: Arc<Mutex<Shared>>,
    /// C `pasynUserCamserver` with its buffers, for the port-thread exchanges.
    pub(crate) cam: Camserver,
    /// C `firstStatusCall`.
    pub(crate) first_status_call: bool,
    /// The camserver version `pilatusStatus` parsed, for `resetModulePower`.
    pub(crate) version: (i32, i32, i32),
}

/// The handles the driver hands to (and shares with) its worker threads.
pub(crate) struct DriverLinks {
    pub(crate) flat_field_tx: rt::CommandSender<FlatFieldJob>,
    pub(crate) start: Arc<Event>,
    pub(crate) stop: Arc<Event>,
    pub(crate) shared: Arc<Mutex<Shared>>,
    pub(crate) cam: Arc<SyncIOHandle>,
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
            max_memory,
            ASYN_CANBLOCK,
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
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
        base.set_int32_param(params.base.data_type, 0, NDDataType::Int32 as i32)?;
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
            cam: Camserver::new(links.cam, ad.params, links.stop.clone()),
            ad,
            p,
            flat_field_tx: links.flat_field_tx,
            start: links.start,
            stop: links.stop,
            shared: links.shared,
            first_status_call: true,
            version: (0, 0, 0),
        })
    }

    fn f64_or_zero(&self, reason: usize) -> f64 {
        self.ad
            .port_base
            .get_float64_param(reason, 0)
            .unwrap_or(0.0)
    }
}

const INT32_TAIL: WriteTail = WriteTail {
    error: "pilatusDetector:writeInt32: error, ",
    done: "pilatusDetector:writeInt32: ",
    list: OwnList::Zero,
};

const FLOAT64_TAIL: WriteTail = WriteTail {
    error: "pilatusDetector:writeFloat64 error, ",
    done: "pilatusDetector:writeFloat64: ",
    list: OwnList::Zero,
};

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

        INT32_TAIL.write_int32(self, user, value, |det, user, mut status| {
            if function == params.acquire {
                if value != 0 && can_start {
                    // Wake up the Pilatus task.
                    det.start.signal();
                }
                if value == 0 && acquiring {
                    det.stop.signal();
                    det.abort_acquire();
                }
            } else if function == params.trigger_mode
                || function == params.num_images
                || function == params.num_exposures
                || function == p.gap_fill
            {
                det.set_acquire_params();
            } else if function == p.threshold_apply {
                det.set_threshold();
            } else if function == p.reset_power {
                det.reset_module_power();
            } else if function == p.num_oscill {
                det.cam_write_read(
                    &cmd_mx_i32("N_oscillations", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == params.read_status {
                if !acquiring {
                    // C returns pilatusStatus()'s status (pilatusDetector.cpp:1429-1431).
                    status = match det.pilatus_status() {
                        AsynStatus::Success => Ok(()),
                        s => Err(AsynError::Status {
                            status: s,
                            message: "pilatusStatus".into(),
                        }),
                    };
                }
            } else if function < p.first() {
                status = ADDriver::ad_write_int32(det, user, value);
            }
            status
        })
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
        FLOAT64_TAIL.write_float64(self, user, value, |det, user, mut status| {
            if function == params.gain || function == p.energy || function == p.threshold {
                let auto_apply = det
                    .ad
                    .port_base
                    .get_int32_param(p.threshold_auto_apply, 0)
                    .unwrap_or(0);
                {
                    let mut shared = det.shared.lock();
                    if function == p.threshold {
                        shared.demanded_threshold = value;
                    }
                    if function == p.energy {
                        shared.demanded_energy = value;
                    }
                }
                if auto_apply != 0 {
                    let demanded = {
                        let shared = det.shared.lock();
                        if function == p.threshold {
                            Some(shared.demanded_threshold)
                        } else if function == p.energy {
                            Some(shared.demanded_energy)
                        } else {
                            None
                        }
                    };
                    if let Some(v) = demanded {
                        status = det.ad.port_base.set_float64_param(function, 0, v);
                    }
                    det.set_threshold();
                } else {
                    // Put the old value back if we are deferring the threshold.
                    if function == p.threshold || function == p.energy {
                        status = det.ad.port_base.set_float64_param(function, 0, old_value);
                    }
                }
            } else if function == params.acquire_time
                || function == params.acquire_period
                || function == p.delay_time
            {
                det.set_acquire_params();
            } else if function == p.wavelength {
                det.cam_write_read(&cmd_mx_f64("Wavelength", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.energy_low || function == p.energy_high {
                // C ignores both statuses and gets 0 (pilatusDetector.cpp:1511-1512).
                let low = det.f64_or_zero(p.energy_low);
                let high = det.f64_or_zero(p.energy_high);
                det.cam_write_read(
                    &cmd_mx_pair("Energy_range", low, high),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.det_dist {
                det.cam_write_read(
                    &cmd_mx_f64("Detector_distance", value / 1000.0),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.det_voffset {
                det.cam_write_read(
                    &cmd_mx_f64("Detector_Voffset", value / 1000.0),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.beam_x || function == p.beam_y {
                // As above (pilatusDetector.cpp:1523-1524).
                let x = det.f64_or_zero(p.beam_x);
                let y = det.f64_or_zero(p.beam_y);
                det.cam_write_read(&cmd_mx_pair("Beam_xy", x, y), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.flux {
                det.cam_write_read(&cmd_mx_f64("Flux", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.filter_transm {
                det.cam_write_read(
                    &cmd_mx_f64("Filter_transmission", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.start_angle {
                det.cam_write_read(&cmd_mx_f64("Start_angle", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.angle_incr {
                det.cam_write_read(
                    &cmd_mx_f64("Angle_increment", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.det_2theta {
                det.cam_write_read(
                    &cmd_mx_f64("Detector_2theta", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.polarization {
                det.cam_write_read(
                    &cmd_mx_f64("Polarization", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.alpha {
                det.cam_write_read(&cmd_mx_f64("Alpha", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.kappa {
                det.cam_write_read(&cmd_mx_f64("Kappa", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.phi {
                det.cam_write_read(&cmd_mx_f64("Phi", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.phi_incr {
                det.cam_write_read(
                    &cmd_mx_f64("Phi_increment", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.chi {
                det.cam_write_read(&cmd_mx_f64("Chi", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.chi_incr {
                det.cam_write_read(
                    &cmd_mx_f64("Chi_increment", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function == p.omega {
                det.cam_write_read(&cmd_mx_f64("Omega", value), CAMSERVER_DEFAULT_TIMEOUT);
            } else if function == p.omega_incr {
                det.cam_write_read(
                    &cmd_mx_f64("Omega_increment", value),
                    CAMSERVER_DEFAULT_TIMEOUT,
                );
            } else if function < p.delay_time {
                // ADDriver::writeFloat64 is asynPortDriver's (pilatusDetector.cpp:1571).
                status = det.ad.port_base.write_float64_param(user, value);
            }
            if status.is_err() {
                // C puts the old value back (pilatusDetector.cpp:1574-1580).
                let _ = det.ad.port_base.set_float64_param(function, 0, old_value);
            }
            status
        })
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let function = user.reason;
        let params = self.ad.params;
        let p = self.p;

        // C receives a NUL-terminated `char *`; waveform records pad with NULs.
        let payload = data.split(|&b| b == 0).next().unwrap_or(&[]);
        let value = String::from_utf8_lossy(payload).into_owned();

        // C's setStringParam and ADDriver::writeOctet statuses are both
        // overwritten by callParamCallbacks() (pilatusDetector.cpp:1630).
        // C also drops checkPath's (:1615), so a FilePath that does not
        // exist is written without error; that defect is not copied.
        let mut checked = Ok(());
        let _ = self
            .ad
            .port_base
            .set_string_param(function, 0, value.clone());
        if function == p.bad_pixel_file {
            self.read_bad_pixel_file(&value);
        } else if function == p.flat_field_file {
            self.read_flat_field_file(&value);
        } else if function == params.base.file_path {
            checked = self.img_path(&value);
        } else if function == p.oscill_axis {
            self.cam_write_read(&cmd_oscill_axis(&value), CAMSERVER_DEFAULT_TIMEOUT);
        } else if function == p.cbf_template_file {
            self.cam_write_read(&cmd_cbf_template_file(&value), CAMSERVER_DEFAULT_TIMEOUT);
        } else if function < p.first() {
            let _ = self.ad.write_octet(user, data);
        }

        let status = self.ad.port_base.call_param_callbacks(0).and(checked);
        if let Err(e) = status {
            return Err(AsynError::Status {
                status: e.status(),
                message: format!(
                    "pilatusDetector:writeOctet: status={}, function={function}, value={value}",
                    e.c_status_code()
                ),
            });
        }
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!("pilatusDetector:writeOctet: function={function}, value={value}\n"),
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
/// C connects to `camserver_port` with `pasynOctetSyncIO->connect` in the
/// constructor and ignores the status (pilatusDetector.cpp:1731): a port that
/// does not exist leaves a user whose every camserver exchange fails.
pub fn create_pilatus_detector(
    claim: &PortClaim,
    camserver_port: &str,
    max_size_x: i32,
    max_size_y: i32,
    max_memory: usize,
) -> Result<PilatusRuntime, String> {
    let port_name = claim.name();
    let (cam_handle, _) =
        SyncIOHandle::connect_port(camserver_port, 0, Duration::ZERO, InterfaceType::Octet);
    let cam_handle = Arc::new(cam_handle);

    // The port thread waits for each ack, so one job is ever queued.
    let (flat_field_tx, flat_field_rx) = rt::command_channel::<FlatFieldJob>(1);
    let start = Arc::new(Event::new());
    let stop = Arc::new(Event::new());

    let shared = Arc::new(Mutex::new(Shared::default()));

    let mut driver = PilatusDriver::new(
        port_name,
        max_size_x,
        max_size_y,
        max_memory,
        DriverLinks {
            flat_field_tx,
            start: start.clone(),
            stop: stop.clone(),
            shared: shared.clone(),
            cam: cam_handle.clone(),
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
    // C always calls pilatusStatus() once at the end of the constructor to get
    // the TVX version.
    let _ = driver.pilatus_status();

    let ad_params = driver.ad.params;
    let pilatus_params = driver.p;
    let pool = driver.ad.pool.clone();

    let (runtime_handle, _) = create_port_runtime(claim, driver, RuntimeConfig::default())
        .map_err(|e| format!("failed to start the Pilatus port runtime: {e}"))?;
    let handle = runtime_handle.port_handle().clone();
    let array_output = Arc::new(Mutex::new(NDArrayOutput::new()));

    let acq_worker = Worker {
        ctx: Ctx::new(handle, cam_handle, ad_params, pilatus_params, stop),
        shared,
        output: ArrayPublisher::new(array_output.clone()),
        pool: pool.clone(),
    };

    let tasks = vec![
        start_flat_field_task(ArrayPublisher::new(array_output.clone()), flat_field_rx),
        start_acq_task(acq_worker, start),
    ];

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

    /// A camserver user with no port, as C's when the startup script named
    /// none (pilatusDetector.cpp:1731).
    fn no_camserver() -> Arc<SyncIOHandle> {
        Arc::new(
            SyncIOHandle::connect_port("PILATUS_NO_CAM", 0, Duration::ZERO, InterfaceType::Octet).0,
        )
    }

    /// pilatusDetector.cpp:1796 seeds NDDataType NDInt32, the type of
    /// every array the driver reads.
    #[test]
    fn data_type_starts_at_int32() {
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
        };
        let det = PilatusDriver::new("PILATUS_DATA_TYPE", 8, 8, 0, links).unwrap();
        let data_type = det.ad.params.base.data_type;
        assert_eq!(
            det.ad.port_base.get_int32_param(data_type, 0).unwrap(),
            NDDataType::Int32 as i32
        );
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        use epics_rs::ad_core::driver::ShutterMode;
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
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

    /// pilatusDetector.cpp:1399, :1471, :1606: the param sets are list 0
    /// whatever the address.
    #[test]
    fn a_write_at_another_address_sets_list_zero() {
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
        };
        let mut det = PilatusDriver::new("PILATUS_LIST0", 8, 8, 0, links).unwrap();
        let p = det.p;
        det.write_int32(&mut AsynUser::new(p.pixel_cutoff).with_addr(1), 3)
            .unwrap();
        det.write_float64(&mut AsynUser::new(p.delay_time).with_addr(1), 2.5)
            .unwrap();
        det.write_octet(&mut AsynUser::new(p.header_string).with_addr(1), b"h")
            .unwrap();
        let base = &det.ad.port_base;
        assert_eq!(base.get_int32_param(p.pixel_cutoff, 0).unwrap(), 3);
        assert_eq!(base.get_float64_param(p.delay_time, 0).unwrap(), 2.5);
        assert_eq!(base.get_string_param(p.header_string, 0).unwrap(), b"h");
    }

    /// pilatusDetector.cpp:1441, :1578 carry the param library's
    /// asynParamWrongType for a write of the other type; writeOctet's is
    /// overwritten by callParamCallbacks() (:1630), so it succeeds.
    #[test]
    fn a_refused_write_prints_the_param_librarys_status() {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "PILATUS_STATUS";
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
        };
        let mut det = PilatusDriver::new(port, 8, 8, 0, links).unwrap();
        let manager = PortServices::global().trace().clone();
        let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let user = |reason| {
            let mut user = AsynUser::new(reason);
            user.trace = Some(UserTrace {
                manager: manager.clone(),
                port: port.into(),
            });
            user
        };
        let (float, int) = (det.p.delay_time, det.p.pixel_cutoff);
        assert!(det.write_int32(&mut user(float), 1).is_err());
        assert!(det.write_float64(&mut user(int), 1.0).is_err());
        assert_eq!(det.write_octet(&mut user(int), b"x").unwrap(), 1);
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        for line in [
            format!("pilatusDetector:writeInt32: error, status=8 function={float}, value=1"),
            format!("pilatusDetector:writeFloat64 error, status=8 function={int}, value=1.000000"),
        ] {
            assert!(text.contains(&line), "{line}: {text}");
        }
    }

    #[test]
    fn a_pair_write_sends_zero_for_its_unset_partner() {
        let sent = recording_camserver("PILATUS_PAIR_CAM");
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: camserver("PILATUS_PAIR_CAM"),
        };
        let mut det = PilatusDriver::new("PILATUS_PAIR", 8, 8, 0, links).unwrap();
        let p = det.p;

        for (reason, name) in [(p.energy_low, "Energy_range"), (p.beam_x, "Beam_xy")] {
            let mut user = AsynUser::new(reason);
            det.write_float64(&mut user, 2.5).unwrap();
            assert_eq!(
                sent.lock().last().map(String::as_str),
                Some(format!("mxsettings {name} 2.500000,0.000000").as_str())
            );
        }
    }

    /// A FilePath write runs C's `imgpath` then `asynNDArrayDriver::checkPath()`
    /// (pilatusDetector.cpp:1612-1615, asynNDArrayDriver.cpp:98-111): an empty
    /// path leaves FilePathExists undefined, any other is written back with its
    /// separator.
    #[test]
    fn a_file_path_write_runs_ndarray_driver_check_path() {
        let sent = recording_camserver("PILATUS_CHECK_PATH_CAM");
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: camserver("PILATUS_CHECK_PATH_CAM"),
        };
        let mut det = PilatusDriver::new("PILATUS_CHECK_PATH", 8, 8, 0, links).unwrap();
        let (file_path, exists) = (
            det.ad.params.base.file_path,
            det.ad.params.base.file_path_exists,
        );

        det.write_octet(&mut AsynUser::new(file_path), b"").unwrap();
        assert_eq!(sent.lock().last().map(String::as_str), Some("imgpath "));
        assert!(
            det.ad.port_base.get_int32_param(exists, 0).is_err(),
            "an empty FilePath leaves FilePathExists undefined"
        );

        let dir = tempfile::tempdir().unwrap();
        let dir_path = dir.path().to_str().unwrap().to_string();
        det.write_octet(&mut AsynUser::new(file_path), dir_path.as_bytes())
            .unwrap();
        assert_eq!(sent.lock().last(), Some(&format!("imgpath {dir_path}")));
        assert_eq!(
            det.ad.port_base.get_string_param(file_path, 0).unwrap(),
            format!("{dir_path}/").as_bytes(),
            "checkPath writes the path back with its separator"
        );
        assert_eq!(det.ad.port_base.get_int32_param(exists, 0).unwrap(), 1);
    }

    /// A FilePath that does not exist still goes to camserver as `imgpath`,
    /// but checkPath's error fails the write, which C drops
    /// (pilatusDetector.cpp:1615).
    #[test]
    fn a_missing_file_path_fails_the_write() {
        let sent = recording_camserver("PILATUS_MISSING_PATH_CAM");
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: camserver("PILATUS_MISSING_PATH_CAM"),
        };
        let mut det = PilatusDriver::new("PILATUS_MISSING_PATH", 8, 8, 0, links).unwrap();
        let (file_path, exists) = (
            det.ad.params.base.file_path,
            det.ad.params.base.file_path_exists,
        );

        let dir = tempfile::tempdir().unwrap();
        let missing = format!("{}/missing", dir.path().to_str().unwrap());
        let Err(AsynError::Status { status, message }) =
            det.write_octet(&mut AsynUser::new(file_path), missing.as_bytes())
        else {
            panic!("a missing FilePath must fail the write");
        };
        assert_eq!(status, AsynStatus::Error);
        assert_eq!(
            message,
            format!("pilatusDetector:writeOctet: status=3, function={file_path}, value={missing}")
        );
        assert_eq!(sent.lock().last(), Some(&format!("imgpath {missing}")));
        assert_eq!(det.ad.port_base.get_int32_param(exists, 0).unwrap(), 0);
    }

    /// ADDriver::writeFloat64 is asynPortDriver's (pilatusDetector.cpp:1571),
    /// whose IO_DRIVER line precedes pilatusDetector's.
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "PILATUS_BASE_F64";
        let (flat_field_tx, _flat_field_rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
        };
        let mut det = PilatusDriver::new(port, 8, 8, 0, links).unwrap();
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("pilatus-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let delay = det.ad.params.shutter_open_delay;
        let mut user = AsynUser::new(delay).with_addr(0);
        user.trace = Some(UserTrace {
            manager,
            port: port.into(),
        });
        det.write_float64(&mut user, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = format!("pilatusDetector:writeFloat64: function={delay}, value=2.000000");
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
    }

    /// C ignores the camserver `pasynOctetSyncIO->connect` status
    /// (pilatusDetector.cpp:1731): with no camserver port configured the
    /// detector port still exists.
    #[test]
    fn a_detector_whose_camserver_port_is_missing_still_boots() {
        use epics_rs::asyn::registry::claim_port;

        let rt = create_pilatus_detector(
            &claim_port("PILATUS_NONE").unwrap(),
            "PILATUS_NONE_CAM",
            8,
            8,
            0,
        )
        .expect("the detector port");
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        sync.write_int32(rt.ad_params.num_images, 3).unwrap();
        assert_eq!(sync.read_int32(rt.ad_params.num_images).unwrap(), 3);
    }

    /// pilatusDetector.cpp:1788: MaxSize is pilatusDetectorConfig's maxSizeX/Y.
    #[test]
    fn max_size_is_the_configured_size() {
        use epics_rs::asyn::registry::claim_port;

        let rt = create_pilatus_detector(
            &claim_port("PILATUS_MAX_SIZE").unwrap(),
            "PILATUS_MAX_SIZE_CAM",
            487,
            195,
            0,
        )
        .expect("the detector port");
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        assert_eq!(sync.read_int32(rt.ad_params.max_size_x).unwrap(), 487);
        assert_eq!(sync.read_int32(rt.ad_params.max_size_y).unwrap(), 195);
    }

    /// A camserver that records every command and answers `15 OK`; the first
    /// `nimages` waits for `gate`.
    struct GatedCamserver {
        base: epics_rs::asyn::port::PortDriverBase,
        sent: Arc<Mutex<Vec<String>>>,
        gate: Option<Mutex<std::sync::mpsc::Receiver<()>>>,
    }

    impl PortDriver for GatedCamserver {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            if cmd.starts_with("nimages")
                && let Some(gate) = self.gate.take()
            {
                let _ = gate.lock().recv();
            }
            self.sent.lock().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
            let reply = b"15 OK";
            buf[..reply.len()].copy_from_slice(reply);
            OctetReadResult::ok(reply.len(), EomReason::EOS)
        }
    }

    /// A camserver user on `port`.
    fn camserver(port: &str) -> Arc<SyncIOHandle> {
        Arc::new(SyncIOHandle::connect_port(port, 0, Duration::ZERO, InterfaceType::Octet).0)
    }

    /// Registers an ungated [`GatedCamserver`] as `port` and returns what it
    /// is sent.
    fn recording_camserver(port: &str) -> Arc<Mutex<Vec<String>>> {
        use epics_rs::asyn::manager::{PortManager, claim_port_driver};
        use epics_rs::asyn::port::PortFlags;
        use epics_rs::asyn::registry::open_owner;

        let sent = Arc::new(Mutex::new(Vec::new()));
        let rt = PortManager::new()
            .register_port(
                claim_port_driver(port).unwrap(),
                GatedCamserver {
                    base: PortDriverBase::new(
                        port,
                        1,
                        PortFlags::default(),
                        InterfaceMask::OCTET_TRANSPORT,
                        InterfaceMask::OCTET,
                    ),
                    sent: sent.clone(),
                    gate: None,
                },
                &open_owner(),
            )
            .unwrap();
        std::mem::forget(rt);
        sent
    }

    /// C publishes the flat field before setting FlatFieldValid, both on the
    /// port thread (pilatusDetector.cpp:315-316): when the write returns, a
    /// plugin already holds the array.
    #[test]
    fn a_flat_field_write_returns_after_the_array_is_published() {
        use epics_rs::ad_core::plugin::channel::ndarray_channel;
        use epics_rs::asyn::registry::claim_port;

        let rt = create_pilatus_detector(
            &claim_port("PILATUS_FF_ORDER").unwrap(),
            "PILATUS_FF_ORDER_CAM",
            2,
            1,
            0,
        )
        .expect("the detector port");
        let (sender, mut receiver) = ndarray_channel("PILATUS_FF_SINK", 4);
        rt.connect_downstream(sender);
        let path = std::env::temp_dir().join(format!("pil-ff-order-{}.tif", std::process::id()));
        let mut enc =
            tiff::encoder::TiffEncoder::new(std::fs::File::create(&path).unwrap()).unwrap();
        enc.write_image::<tiff::encoder::colortype::Gray32>(2, 1, &[7, 9])
            .unwrap();
        let sync = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(5));
        let mut late = 0;
        for _ in 0..200 {
            sync.write_octet(
                rt.pilatus_params.flat_field_file,
                path.to_string_lossy().as_bytes(),
            )
            .unwrap();
            assert_eq!(
                sync.read_int32(rt.pilatus_params.flat_field_valid).unwrap(),
                1
            );
            if receiver.pending() == 0 {
                late += 1;
            }
            receiver.blocking_recv().unwrap();
        }
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            late, 0,
            "FlatFieldValid read 1 before the plugin had the array"
        );
        std::mem::forget(rt);
    }

    /// C writeInt32 returns pilatusStatus()'s status for ReadStatus
    /// (pilatusDetector.cpp:1429-1431), the `thread` exchange's.
    #[test]
    fn a_read_status_whose_thread_exchange_fails_fails_the_write() {
        let (flat_field_tx, _rx) = rt::command_channel::<FlatFieldJob>(4);
        let links = DriverLinks {
            flat_field_tx,
            start: Arc::new(Event::new()),
            stop: Arc::new(Event::new()),
            shared: Arc::new(Mutex::new(Shared::default())),
            cam: no_camserver(),
        };
        let mut det = PilatusDriver::new("PILATUS_READ_STATUS", 8, 8, 0, links).unwrap();
        let r = det.ad.params.read_status;
        assert!(det.write_int32(&mut AsynUser::new(r), 1).is_err());
    }

    /// C runs setAcquireParams inside writeFloat64 on the port thread
    /// (pilatusDetector.cpp:1502-1505), so a PINI AcquirePeriod write queued
    /// behind AcquireTime's lands only after setAcquireParams has read the
    /// still undefined period and sent its default 2 (:790-795).
    #[test]
    fn a_queued_acquire_period_lands_after_set_acquire_params_reads_it() {
        use epics_rs::asyn::manager::{PortManager, claim_port_driver};
        use epics_rs::asyn::port::PortFlags;
        use epics_rs::asyn::registry::{claim_port, open_owner};
        use epics_rs::asyn::request::RequestOp;

        let cam = "PILATUS_PINI_CAM";
        let (open, gate) = std::sync::mpsc::channel();
        let sent = Arc::new(Mutex::new(Vec::new()));
        let cam_rt = PortManager::new()
            .register_port(
                claim_port_driver(cam).unwrap(),
                GatedCamserver {
                    base: PortDriverBase::new(
                        cam,
                        1,
                        PortFlags::default(),
                        InterfaceMask::OCTET_TRANSPORT,
                        InterfaceMask::OCTET,
                    ),
                    sent: sent.clone(),
                    gate: Some(Mutex::new(gate)),
                },
                &open_owner(),
            )
            .unwrap();
        let rt = create_pilatus_detector(&claim_port("PILATUS_PINI").unwrap(), cam, 8, 8, 0)
            .expect("the detector port");
        let ad = rt.ad_params;
        let port = rt.port_handle();
        let time = port
            .try_submit(
                RequestOp::Float64Write { value: 1.0 },
                AsynUser::new(ad.acquire_time),
            )
            .unwrap();
        let period = port
            .try_submit(
                RequestOp::Float64Write { value: 0.0 },
                AsynUser::new(ad.acquire_period),
            )
            .unwrap();
        open.send(()).unwrap();
        time.wait_blocking(Duration::from_secs(10)).unwrap();
        period.wait_blocking(Duration::from_secs(10)).unwrap();

        let periods = || -> Vec<String> {
            sent.lock()
                .iter()
                .filter(|c| c.starts_with("expperiod"))
                .cloned()
                .collect()
        };
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        while periods().len() < 2 && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(
            periods(),
            ["expperiod  2.00000000", "expperiod  0.00000000"]
        );
        std::mem::forget(cam_rt);
        std::mem::forget(rt);
    }
}
