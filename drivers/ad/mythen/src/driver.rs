//! The Mythen asyn port driver (port of the `mythen` class, mythen.cpp).

use std::sync::Arc;
use std::sync::atomic::Ordering;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::plugin::runtime::ASYN_CANBLOCK;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::records::printf::format_g;

use crate::detector::{Detector, Settings};
use crate::protocol::CHANNELS_PER_MODULE;

/// The detector is one row of channels, however many modules it has
/// (C `MAX_DIMS` / `dims[1]`, mythen.cpp:1345).
const SENSOR_SIZE_X: i32 = CHANNELS_PER_MODULE as i32;
const SENSOR_SIZE_Y: i32 = 1;

/// The parameters this driver adds to `ADDriver` (C `SD*String`).
#[derive(Debug, Clone, Copy)]
pub struct MythenParams {
    pub setting: usize,
    pub delay_time: usize,
    pub threshold: usize,
    pub energy: usize,
    pub use_flat_field: usize,
    pub use_count_rate: usize,
    pub use_bad_chan_intrpl: usize,
    pub bit_depth: usize,
    pub use_gates: usize,
    pub num_gates: usize,
    pub num_frames: usize,
    pub trigger: usize,
    pub reset: usize,
    pub tau: usize,
    pub nmodules: usize,
    pub firmware_version: usize,
    pub read_mode: usize,
}

impl MythenParams {
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        use ParamType::{Float64, Int32, Octet};
        Ok(Self {
            setting: base.create_param("SD_SETTING", Int32)?,
            delay_time: base.create_param("SD_DELAY_TIME", Float64)?,
            threshold: base.create_param("SD_THRESHOLD", Float64)?,
            energy: base.create_param("SD_ENERGY", Float64)?,
            use_flat_field: base.create_param("SD_USE_FLATFIELD", Int32)?,
            use_count_rate: base.create_param("SD_USE_COUNTRATE", Int32)?,
            use_bad_chan_intrpl: base.create_param("SD_USE_BADCHANNEL_INTRPL", Int32)?,
            bit_depth: base.create_param("SD_BIT_DEPTH", Int32)?,
            use_gates: base.create_param("SD_USE_GATES", Int32)?,
            num_gates: base.create_param("SD_NUM_GATES", Int32)?,
            num_frames: base.create_param("SD_NUM_FRAMES", Int32)?,
            trigger: base.create_param("SD_TRIGGER", Int32)?,
            reset: base.create_param("SD_RESET", Int32)?,
            tau: base.create_param("SD_TAU", Float64)?,
            nmodules: base.create_param("SD_NMODULES", Int32)?,
            firmware_version: base.create_param("SD_FIRMWARE_VERSION", Octet)?,
            read_mode: base.create_param("SD_READ_MODE", Int32)?,
        })
    }
}

/// The outcome of the constructor's detector queries.
///
/// The single gate that keeps a constructor-time detector query from failing
/// port creation: it records that a query failed, and hands the caller a `None` to fall back from. C gets the same property from
/// `status |= ...` over a C++ constructor, which cannot fail — an unreachable
/// detector must never stop the IOC from reaching `iocInit`.
#[derive(Default)]
struct BootQueries {
    failed: bool,
}

impl BootQueries {
    /// Run one constructor-time query, recording a failure.
    fn run<T>(&mut self, result: AsynResult<T>) -> Option<T> {
        match result {
            Ok(value) => Some(value),
            Err(_) => {
                self.fail();
                None
            }
        }
    }

    /// Record a query that answered, but answered something unusable.
    fn fail(&mut self) {
        self.failed = true;
    }
}

pub struct MythenDriver {
    pub ad: ADDriverBase,
    pub p: MythenParams,
    pub det: Arc<Detector>,
    /// Tells the acquisition task that an acquisition has started
    /// (C `startEventId_`).
    start_tx: rt::CommandSender<()>,
}

impl MythenDriver {
    pub fn new(
        port_name: &str,
        det: Arc<Detector>,
        max_memory: usize,
        start_tx: rt::CommandSender<()>,
    ) -> AsynResult<Self> {
        let mut ad = ADDriverBase::new(
            port_name,
            1,
            SENSOR_SIZE_X,
            SENSOR_SIZE_Y,
            max_memory,
            PortFlags {
                multi_device: true,
                ..ASYN_CANBLOCK
            },
        )?;
        let p = MythenParams::create(&mut ad.port_base)?;

        let mut driver = Self {
            ad,
            p,
            det,
            start_tx,
        };
        driver.init_params()?;
        Ok(driver)
    }

    /// Probe a detector that is marked disconnected and, if it answers, re-read
    /// everything the constructor reads.
    ///
    /// The only way back to connected. C has none: its constructor is the only
    /// place that reads the module count (mythen.cpp:1363), so a detector that
    /// was off when the IOC booted stays unusable until the IOC restarts —
    /// every readout after that would be sized from a count that was never
    /// read. Re-running the constructor's queries here is what closes that.
    pub fn reconnect(&mut self) -> AsynResult<()> {
        self.det.probe()?;
        self.init_params()
    }

    /// C's constructor body, mythen.cpp:1326-1377.
    ///
    /// Every detector query on this path goes through [`BootQueries`]. C runs
    /// the same queries with `status |= ...` and carries on when one fails, so
    /// a detector that is powered off when the IOC boots leaves the parameters
    /// at their defaults but still gets a port — and the IOC still reaches
    /// `iocInit`. Nothing here may use `?` on the detector directly.
    fn init_params(&mut self) -> AsynResult<()> {
        let mut q = BootQueries::default();

        let firmware = q.run(self.det.get_firmware());
        // C ORs only these two into the constructor's `status`
        // (mythen.cpp:1329, :1364).
        let mut camera_params_failed = firmware.is_none();

        let base = &mut self.ad.port_base;
        base.set_string_param(self.ad.params.base.manufacturer, 0, "Dectris")?;
        base.set_string_param(self.ad.params.base.model, 0, "Mythen")?;
        base.set_string_param(self.p.firmware_version, 0, firmware.unwrap_or_default())?;

        base.set_int32_param(self.ad.params.max_size_x, 0, SENSOR_SIZE_X)?;
        base.set_int32_param(self.ad.params.max_size_y, 0, SENSOR_SIZE_Y)?;
        base.set_int32_param(self.ad.params.min_x, 0, 1)?;
        base.set_int32_param(self.ad.params.min_y, 0, 1)?;
        base.set_int32_param(self.ad.params.size_x, 0, SENSOR_SIZE_X)?;
        base.set_int32_param(self.ad.params.size_y, 0, SENSOR_SIZE_Y)?;
        base.set_int32_param(self.ad.params.base.array_size, 0, 0)?;
        base.set_int32_param(self.ad.params.base.data_type, 0, NDDataType::UInt32 as i32)?;
        base.set_int32_param(self.ad.params.image_mode, 0, ImageMode::Single as i32)?;

        let status = q.run(self.det.get_status());
        self.ad.port_base.set_int32_param(
            self.ad.params.status,
            0,
            status.unwrap_or(ADStatus::Disconnected) as i32,
        )?;

        if let Some(settings) = q.run(self.det.get_settings()) {
            self.apply_settings(&settings)?;
        }

        let nmodules = match q.run(self.det.read_nmodules()) {
            Some(n) if n >= 0 => Some(n),
            // C prints the same message and abandons the rest of the
            // constructor (mythen.cpp:1367-1372); the port survives either way.
            Some(n) => {
                self.det.trace_error(&format!(
                    "mythen:mythen: error, outString=-get nmodules, inString={n}"
                ));
                q.fail();
                camera_params_failed = false;
                None
            }
            None => {
                camera_params_failed = true;
                None
            }
        };
        if camera_params_failed {
            println!("mythen: unable to read camera parameters");
        }
        if let Some(nmodules) = nmodules {
            self.ad
                .port_base
                .set_int32_param(self.p.nmodules, 0, nmodules)?;
            // One module is 1280 channels; the array the driver publishes is as
            // wide as all the modules together.
            self.ad.port_base.set_int32_param(
                self.ad.params.base.array_size_x,
                0,
                nmodules * CHANNELS_PER_MODULE as i32,
            )?;
        }

        // The message is set and cleared here because this runs again on every
        // reconnect: a detector that came back must not leave the operator
        // reading "FAILED TO CONNECT" off a working port.
        let message = if q.failed {
            self.ad.port_base.set_int32_param(
                self.ad.params.status,
                0,
                ADStatus::Disconnected as i32,
            )?;
            "Mythen FAILED TO CONNECT"
        } else {
            ""
        };
        self.ad
            .port_base
            .set_string_param(self.ad.params.status_message, 0, message)?;

        self.ad.port_base.call_param_callbacks(0)
    }

    /// Push a `getSettings` result into the parameter library
    /// (C `getSettings`'s `setIntegerParam`/`setDoubleParam` calls).
    fn apply_settings(&mut self, s: &Settings) -> AsynResult<()> {
        let base = &mut self.ad.port_base;
        base.set_int32_param(self.p.use_flat_field, 0, s.use_flat_field)?;
        base.set_int32_param(self.p.use_bad_chan_intrpl, 0, s.use_bad_chan_intrpl)?;
        base.set_int32_param(self.p.use_count_rate, 0, s.use_count_rate)?;
        // C stores the *bit count* here, which is what BitDepth_RBV (a longin)
        // displays; the BitDepth mbbo writes the menu index into the same
        // parameter.
        base.set_int32_param(self.p.bit_depth, 0, s.nbits)?;
        base.set_float64_param(self.ad.params.acquire_time, 0, s.acquire_time)?;
        if let Some(frames) = s.frames {
            base.set_int32_param(self.p.num_frames, 0, frames)?;
        }
        base.set_float64_param(self.p.tau, 0, s.tau)?;
        base.set_float64_param(self.p.threshold, 0, s.threshold)?;
        if let Some(energy) = s.energy {
            base.set_float64_param(self.p.energy, 0, energy)?;
        }
        if let Some(delay) = s.delay_time {
            base.set_float64_param(self.p.delay_time, 0, delay)?;
        }
        if let Some(trigger) = s.trigger {
            base.set_int32_param(self.p.trigger, 0, trigger)?;
        }
        Ok(())
    }

    /// C `getSettings` as called at the end of every write.
    fn refresh_settings(&mut self) -> AsynResult<()> {
        if self.det.state.acquiring.load(Ordering::Acquire) {
            // C refuses to talk to a running detector (mythen.cpp:689).
            return Ok(());
        }
        // C's getSettings returns asynError whatever failed (mythen.cpp:795).
        let settings = self.det.get_settings().map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: e.to_string(),
        })?;
        self.apply_settings(&settings)
    }

    fn int_param(&self, index: usize) -> i32 {
        self.ad.port_base.get_int32_param(index, 0).unwrap_or(0)
    }

    /// C `setAcquire`, mythen.cpp:268.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 0 {
            self.det.stop()?;
            let status = self.det.get_status()?;
            self.ad
                .port_base
                .set_int32_param(self.ad.params.status, 0, status as i32)?;
            return Ok(());
        }
        // Starting an acquisition is the one moment worth spending a real
        // command on a detector that is not answering: the operator asked for
        // data, and the module count the readout is sized from is only ever
        // read by the constructor's queries, which `reconnect` re-runs.
        if !self.det.is_connected() {
            self.reconnect()?;
        }
        if self.det.start()? {
            let _ = self.start_tx.try_send(());
        }
        Ok(())
    }
}

/// C's `int status` in `writeInt32` / `writeFloat64`: `status |= step()` keeps
/// going after a failed step, and the write returns the OR of every status.
#[derive(Default)]
struct StatusOr {
    bits: i32,
    err: Option<AsynError>,
}

impl StatusOr {
    /// `status |= step`.
    fn or(&mut self, step: AsynResult<()>) {
        if let Err(e) = step {
            self.bits |= e.status() as i32;
            self.err = Some(e);
        }
    }

    /// `status = step`.
    fn set(&mut self, step: AsynResult<()>) {
        *self = Self::default();
        self.or(step);
    }

    fn into_result(self) -> AsynResult<()> {
        self.err.map_or(Ok(()), Err)
    }
}

impl ADDriver for MythenDriver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }
    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

impl PortDriver for MythenDriver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquire = self.ad.params.acquire;

        // C refuses every write except ADAcquire while an acquisition runs
        // (mythen.cpp:1111).
        if reason != acquire && self.det.state.acquiring.load(Ordering::Acquire) {
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!("mythen:writeInt32: detector is busy"),
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "mythen: detector is busy".into(),
            });
        }

        let mut status = StatusOr::default();
        status.or(self.ad.port_base.set_int32_param(reason, addr, value));

        if reason == acquire {
            let read_mode = self.int_param(self.p.read_mode);
            self.det.state.read_mode.store(read_mode, Ordering::Release);
            status.or(self.set_acquire(value));
        } else {
            let image_mode = self.int_param(self.ad.params.image_mode);
            if reason == self.p.setting {
                status.or(self.det.load_settings(value));
            } else if reason == self.p.use_flat_field {
                status.or(self.det.set_flat_field_correction(value));
            } else if reason == self.p.use_count_rate {
                status.or(self.det.set_rate_correction(value));
            } else if reason == self.p.use_bad_chan_intrpl {
                status.or(self.det.set_bad_chan_intrpl(value));
            } else if reason == self.p.bit_depth {
                status.or(self.det.set_bit_depth(value));
            } else if reason == self.p.num_gates {
                status.set(self.det.set_num_gates(value));
            } else if reason == self.p.use_gates {
                status.set(self.det.set_use_gates(value));
            } else if reason == self.p.num_frames {
                status.or(self.det.set_frames(value, image_mode));
            } else if reason == self.p.trigger {
                status.or(self.det.set_trigger(value));
            } else if reason == self.p.reset {
                // C flips SD_RESET on for the duration of the reset
                // (mythen.cpp:665-673).
                let _ = self.ad.port_base.set_int32_param(self.p.reset, 0, 1);
                let _ = self.ad.port_base.call_param_callbacks(0);
                status.or(self.det.reset());
                let _ = self.ad.port_base.set_int32_param(self.p.reset, 0, 0);
            } else if reason == self.ad.params.image_mode {
                // The frame count the detector needs depends on the image mode.
                let frames = self.det.state.frames.load(Ordering::Acquire);
                status.or(self.det.set_frames(frames, value));
            } else if reason < self.p.setting {
                // `FIRST_SD_PARAM` is SDSetting (mythen.cpp:100, :1153).
                status.set(ADDriver::ad_write_int32(self, user, value));
            }
            status.or(self.refresh_settings());
        }

        status.or(self.ad.port_base.call_param_callbacks(addr));
        if status.bits != 0 {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "mythen:writeInt32: error, status={} function={reason}, value={value}",
                    status.bits
                ),
            );
        } else {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("mythen:writeInt32: function={reason}, value={value}"),
            );
        }
        status.into_result()
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        if self.det.state.acquiring.load(Ordering::Acquire) {
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!("mythen:writeFloat64: detector is busy"),
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "mythen: detector is busy".into(),
            });
        }

        let mut status = StatusOr::default();
        status.set(self.ad.port_base.set_float64_param(reason, addr, value));

        if reason == self.ad.params.acquire_time {
            status.or(self.det.set_exposure_time(value));
        } else if reason == self.p.delay_time {
            status.or(self.det.set_delay_after_trigger(value));
        } else if reason == self.p.threshold {
            status.or(self.det.set_kthresh(value));
        } else if reason == self.p.energy {
            status.or(self.det.set_energy(value));
        } else if reason == self.p.tau {
            // C zeroes the parameter and errors out on a value that is neither
            // -1 nor positive (mythen.cpp:361-368).
            status.or(match self.det.set_tau(value) {
                Ok(true) => Ok(()),
                Ok(false) => {
                    let _ = self.ad.port_base.set_float64_param(self.p.tau, 0, 0.0);
                    let _ = self.ad.port_base.call_param_callbacks(addr);
                    Err(AsynError::Status {
                        status: AsynStatus::Error,
                        message: format!("mythen: tau must be -1 or > 0, got {value}"),
                    })
                }
                Err(e) => Err(e),
            });
        } else if reason < self.p.setting {
            // ADDriver::writeFloat64 is asynPortDriver's (mythen.cpp:1210).
            status.set(self.ad.port_base.write_float64_param(user, value));
        }

        status.or(self.refresh_settings());
        status.or(self.ad.port_base.call_param_callbacks(addr));
        let g = format_g(value, 6);
        if status.bits != 0 {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "mythen:writeFloat64: error, status={} function={reason}, value={g}",
                    status.bits
                ),
            );
        } else {
            user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("mythen:writeFloat64: function={reason}, value={g}"),
            );
        }
        status.into_result()
    }

    /// `mythen::writeOctet` (mythen.cpp:1062-1097) handles no octet of its
    /// own and calls `ADDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let status = self.ad.write_octet(user, data);
        // C prints the value as the `char *` it was handed: up to the NUL.
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        let value = String::from_utf8_lossy(&data[..end]);
        match &status {
            Err(e) => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "mythen:writeOctet: error, status={} function={reason}, value={value}",
                    e.status() as i32
                ),
            ),
            Ok(_) => user.print(
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!("mythen:writeOctet: function={reason}, value={value}"),
            ),
        }
        status
    }
}
