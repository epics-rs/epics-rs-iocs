//! The camserver exchanges C runs on the asyn port thread, inside
//! `writeInt32` / `writeFloat64` / `writeOctet`. They run inline here too, on
//! the port's own thread, so a write queued behind one lands only after it.

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::driver::ndarray_driver::check_path_str;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::trace::TraceMask;

use crate::camserver::CamParams;
use crate::driver::PilatusDriver;
use crate::image::{apply_flat_field_floor, flat_field_average, parse_bad_pixel_file};
use crate::protocol::{
    cmd_delay, cmd_expperiod, cmd_exptime, cmd_gapfill, cmd_imgpath, cmd_nexpframe, cmd_nimages,
    cmd_reset_module_power, cmd_set_threshold, gain_index, parse_energy_setting, parse_tau_cutoff,
    parse_threshold_ev, parse_version, thread_channel_values,
};
use crate::task::{ImageReader, refill_flat_field};
use crate::types::{CAMSERVER_DEFAULT_TIMEOUT, CAMSERVER_RESET_POWER_TIMEOUT, TriggerMode, secs};

impl PilatusDriver {
    /// C `getIntegerParam`; `None` while the parameter is undefined.
    fn get_i32(&self, reason: usize) -> Option<i32> {
        self.ad.port_base.get_int32_param(reason, 0).ok()
    }

    /// C `getDoubleParam`; `None` while the parameter is undefined.
    fn get_f64(&self, reason: usize) -> Option<f64> {
        self.ad.port_base.get_float64_param(reason, 0).ok()
    }

    fn set_i32(&mut self, reason: usize, value: i32) {
        self.ad.port_base.set_i32(reason, value);
    }

    fn set_f64(&mut self, reason: usize, value: f64) {
        let _ = self.ad.port_base.set_float64_param(reason, 0, value);
    }

    /// C's `if ((status != asynSuccess) || <invalid>) { ival = fallback;
    /// setIntegerParam(...); }`: an undefined parameter is replaced and
    /// stored exactly as an invalid one is.
    fn i32_or_store(&mut self, reason: usize, invalid: impl Fn(i32) -> bool, fallback: i32) -> i32 {
        match self.get_i32(reason) {
            Some(v) if !invalid(v) => v,
            _ => {
                self.set_i32(reason, fallback);
                fallback
            }
        }
    }

    /// [`Self::i32_or_store`] for a `getDoubleParam`.
    fn f64_or_store(&mut self, reason: usize, invalid: impl Fn(f64) -> bool, fallback: f64) -> f64 {
        match self.get_f64(reason) {
            Some(v) if !invalid(v) => v,
            _ => {
                self.set_f64(reason, fallback);
                fallback
            }
        }
    }

    fn set_str(&mut self, reason: usize, value: impl Into<String>) {
        self.ad.port_base.set_str(reason, value.into());
    }

    /// C `writeCamserver`.
    fn cam_write(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.cam.write(&mut self.ad.port_base, cmd, timeout)
    }

    /// C `writeReadCamserver`.
    pub(crate) fn cam_write_read(&mut self, cmd: &str, timeout: f64) -> AsynStatus {
        self.cam.write_read(&mut self.ad.port_base, cmd, timeout)
    }

    /// C's `ADAcquire = 0` branch while acquiring (pilatusDetector.cpp:1406-1416).
    pub(crate) fn abort_acquire(&mut self) {
        self.cam_write_read("camcmd k", CAMSERVER_DEFAULT_TIMEOUT);
        self.cam_write("K", CAMSERVER_DEFAULT_TIMEOUT);
        // Sleep for two seconds to allow acquisition to stop in camserver.
        std::thread::sleep(secs(2.0));
        let reason = self.ad.params.status_message;
        self.set_str(reason, "Acquisition aborted");
    }

    /// C's `NDFilePath` branch of `writeOctet` (pilatusDetector.cpp:1612-1615).
    pub(crate) fn img_path(&mut self, path: &str) -> AsynResult<()> {
        self.cam_write_read(&cmd_imgpath(path), CAMSERVER_DEFAULT_TIMEOUT);
        self.check_path()
    }

    /// C `asynNDArrayDriver::checkPath()` (asynNDArrayDriver.cpp:98-111),
    /// which fails a path that does not exist.
    fn check_path(&mut self) -> AsynResult<()> {
        let base = self.ad.params.base;
        let Ok(path) = self.ad.port_base.get_string_param(base.file_path, 0) else {
            return Ok(());
        };
        let mut path = String::from_utf8_lossy(path).into_owned();
        if path.is_empty() {
            return Ok(());
        }
        let exists = check_path_str(&mut path);
        self.set_str(base.file_path, path);
        self.set_i32(base.file_path_exists, i32::from(exists));
        if exists {
            Ok(())
        } else {
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "checkPath: the directory does not exist".into(),
            })
        }
    }

    /// C `setAcquireParams()`.
    pub(crate) fn set_acquire_params(&mut self) {
        let (ad, p) = (self.ad.params, self.p);
        let trigger_mode = self
            .get_i32(ad.trigger_mode)
            .unwrap_or(TriggerMode::Internal as i32);
        // When we change modes download all exposure parameters, since some modes
        // replace values with new parameters.
        if trigger_mode == TriggerMode::Alignment as i32 {
            self.set_i32(ad.num_images, 1);
        }

        let ival = self.i32_or_store(ad.num_images, |v| v < 1, 1);
        self.cam_write_read(&cmd_nimages(ival), CAMSERVER_DEFAULT_TIMEOUT);

        let ival = self.i32_or_store(ad.num_exposures, |v| v < 1, 1);
        self.cam_write_read(&cmd_nexpframe(ival), CAMSERVER_DEFAULT_TIMEOUT);

        let dval = self.f64_or_store(ad.acquire_time, |v| v < 0.0, 1.0);
        self.cam_write_read(&cmd_exptime(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let dval = self.f64_or_store(ad.acquire_period, |v| v < 0.0, 2.0);
        self.cam_write_read(&cmd_expperiod(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let dval = self.f64_or_store(p.delay_time, |v| v < 0.0, 0.0);
        self.cam_write_read(&cmd_delay(dval), CAMSERVER_DEFAULT_TIMEOUT);

        let ival = self.i32_or_store(p.gap_fill, |v| !(-2..=0).contains(&v), -2);
        // -2 means GapFill is not supported (single-element detector).
        if ival != -2 {
            self.cam_write_read(&cmd_gapfill(ival), CAMSERVER_DEFAULT_TIMEOUT);
        }

        // Read back the pixel count-rate cut-off.
        let status = self.cam_write_read("Tau", 5.0);
        if status == AsynStatus::Success
            && let Some(cutoff) = parse_tau_cutoff(self.cam.from())
        {
            self.set_i32(p.pixel_cutoff, cutoff);
        }
    }

    /// C `setThreshold()`.
    pub(crate) fn set_threshold(&mut self) {
        let (ad, p) = (self.ad.params, self.p);
        let dgain = self.get_f64(ad.gain).unwrap_or(0.0);
        let igain = gain_index(dgain);
        let (threshold, mut energy) = {
            let s = self.shared.lock();
            (s.demanded_threshold, s.demanded_energy)
        };
        if energy == 0.0 {
            energy = threshold * 2.0;
        }
        let cmd = cmd_set_threshold(energy, igain, threshold);

        // Set the status to waiting so we can be notified when it has finished.
        self.set_i32(ad.status, ADStatus::Waiting as i32);
        self.set_str(ad.status_message, "Setting threshold");
        let _ = self.ad.port_base.call_param_callbacks(0);

        // This command can take 96 seconds on a 6M.
        let status = self.cam_write_read(&cmd, 110.0);
        let new_status = if status != AsynStatus::Success {
            ADStatus::Error
        } else {
            ADStatus::Idle
        };
        self.set_i32(ad.status, new_status as i32);
        self.set_i32(p.threshold_apply, 0);

        // Read back the actual threshold setting, in case we are out of bounds.
        let status = self.cam_write_read("SetThreshold", 5.0);
        if status == AsynStatus::Success
            && let Some(ev) = parse_threshold_ev(self.cam.from())
        {
            self.set_f64(p.threshold, ev as f64 / 1000.0);
        }

        // Read back the actual energy setting.
        let status = self.cam_write_read("SetEnergy", 5.0);
        if status == AsynStatus::Success {
            let ev = parse_energy_setting(self.cam.from());
            self.set_f64(p.energy, ev as f64 / 1000.0);
        }

        // SetThreshold resets nimages and gapfill, so re-send the acquisition
        // parameters.
        self.set_acquire_params();
        let _ = self.ad.port_base.call_param_callbacks(0);
    }

    /// C `resetModulePower()`.
    pub(crate) fn reset_module_power(&mut self) {
        let (major, minor, patch) = self.version;
        // This command only exists on camserver 7.9.0 and higher.
        if major < 7 || (major == 7 && minor < 9) {
            self.ad.port_base.trace(
                TraceMask::ERROR,
                &format!(
                    "pilatusDetector::resetModulePower ResetModulePower not supported on \
                     version {major}.{minor}.{patch} of camserver\n"
                ),
            );
            return;
        }
        let reason = self.ad.params.status_message;
        self.set_str(reason, "Resetting module power");
        let _ = self.ad.port_base.call_param_callbacks(0);
        let reset_time = self.get_i32(self.p.reset_power_time).unwrap_or(0);
        self.cam_write_read(
            &cmd_reset_module_power(reset_time),
            CAMSERVER_RESET_POWER_TIMEOUT + reset_time as f64,
        );
        // The threshold must be set after resetting module power.
        self.set_threshold();
    }

    /// C `pilatusStatus()`; the status is the `thread` exchange's.
    pub(crate) fn pilatus_status(&mut self) -> AsynStatus {
        let (ad, p) = (self.ad.params, self.p);
        if self.first_status_call {
            let status = self.cam_write_read("version", 1.0);
            if status == AsynStatus::Success {
                if let Some(v) = parse_version(self.cam.from()) {
                    self.set_str(p.tvx_version, v.version_string.clone());
                    self.set_str(ad.base.sdk_version, v.version_string);
                    if let Some(x) = v.major {
                        self.version.0 = x;
                    }
                    if let Some(x) = v.minor {
                        self.version.1 = x;
                    }
                    if let Some(x) = v.patch {
                        self.version.2 = x;
                    }
                }
                self.set_i32(ad.status, ADStatus::Idle as i32);
            } else {
                self.set_i32(ad.status, ADStatus::Error as i32);
            }
            self.first_status_call = false;
        }

        let status = self.cam_write_read("thread", 1.0);
        if status == AsynStatus::Success {
            // Fixed (upstream-c-defects.md #12): resolve each channel independently
            // so a partial reply for one channel never leaks another channel's
            // value (C shared one `temp` / `humid` pair across all channels).
            let reply = self.cam.from().to_string();

            if let Some((temp, humid)) = thread_channel_values(&reply, 0) {
                self.set_f64(p.th_temp_0, temp);
                self.set_f64(p.th_humid_0, humid);
                self.set_f64(ad.temperature, temp);
            }
            if let Some((temp, humid)) = thread_channel_values(&reply, 1) {
                self.set_f64(p.th_temp_1, temp);
                self.set_f64(p.th_humid_1, humid);
            }
            if let Some((temp, humid)) = thread_channel_values(&reply, 2) {
                self.set_f64(p.th_temp_2, temp);
                self.set_f64(p.th_humid_2, humid);
            }
            // Channel 3 handling removed as fixed defect #10 — the `thread` reply
            // defines only channels 0-2 and there is no channel-3 parameter.
        } else {
            self.set_i32(ad.status, ADStatus::Error as i32);
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
        status
    }
    /// C `readBadPixelFile()`.
    pub(crate) fn read_bad_pixel_file(&mut self, file: &str) {
        let nx = self.get_i32(self.ad.params.base.array_size_x).unwrap_or(0);
        let reason = self.p.num_bad_pixels;
        self.set_i32(reason, 0);
        self.shared.lock().bad_pixels.clear();
        if file.is_empty() {
            return;
        }
        let Ok(text) = std::fs::read_to_string(file) else {
            self.ad.port_base.trace(
                TraceMask::ERROR,
                &format!("pilatusDetector::readBadPixelFile, cannot open file {file}\n"),
            );
            return;
        };
        match parse_bad_pixel_file(&text, nx) {
            Ok(map) => {
                let n = map.len() as i32;
                self.shared.lock().bad_pixels = map;
                self.set_i32(reason, n);
            }
            // C returns without updating NUM_BAD_PIXELS, which it zeroed above.
            Err(n) => self.ad.port_base.trace(
                TraceMask::ERROR,
                &format!("pilatusDetector::readBadPixelFile, too few items ={n}, should be 4\n"),
            ),
        }
    }

    /// C `readFlatFieldFile()`.
    pub(crate) fn read_flat_field_file(&mut self, file: &str) {
        let valid = self.p.flat_field_valid;
        self.set_i32(valid, 0);
        let min_flat_field = self.get_i32(self.p.min_flat_field).unwrap_or(0);
        if file.is_empty() {
            return;
        }

        // readTiff checks the file against pFlatField's own dims (:678, :685).
        let (dims, bad_pixels) = {
            let s = self.shared.lock();
            let dims = s
                .flat_field
                .as_ref()
                .map(|f| (f.dims[0].size, f.dims[1].size));
            (dims, s.bad_pixels.clone())
        };
        let Some((nx, ny)) = dims else {
            return;
        };
        // C reads into pFlatField itself (:290), so a failed read leaves the
        // flat field plugins already hold half overwritten; this stages it.
        let mut data = vec![0; nx * ny];
        let read = ImageReader {
            ad: self.ad.params,
            cx: &mut self.ad.port_base,
            stop: &self.stop,
        }
        .read_image_file(file, None, 0.0, [nx, ny], &mut data, &bad_pixels);
        let Ok(description) = read else {
            self.ad.port_base.trace(
                TraceMask::ERROR,
                &format!(
                    "pilatusDetector::readFlatFieldFile, error reading flat field file {file}\n"
                ),
            );
            return;
        };

        let Some(average) = flat_field_average(&data, min_flat_field) else {
            // Fixed (upstream-c-defects.md #11): no pixel reached MinFlatField, so
            // the average would be 0/0 = NaN and would poison every corrected
            // pixel. Skip normalization: leave FlatFieldValid = 0 (set above) and
            // do not publish a NaN-based flat field.
            return;
        };
        apply_flat_field_floor(&mut data, min_flat_field, average);
        let Some(array) = refill_flat_field(&mut self.shared.lock(), &data, description, average)
        else {
            return;
        };
        // C publishes before FlatFieldValid = 1 (pilatusDetector.cpp:315-316);
        // a flat field that never reached the publisher is not valid.
        let (done, published) = std::sync::mpsc::sync_channel(1);
        if self.flat_field_tx.try_send((array, done)).is_ok() && published.recv().is_ok() {
            self.set_i32(valid, 1);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::time::Duration;

    use epics_rs::ad_core::runtime as rt;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::interrupt::InterruptFilter;
    use epics_rs::asyn::manager::{PortManager, claim_port_driver};
    use epics_rs::asyn::param::ParamValue;
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::sync_io::SyncIOHandle;
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;
    use parking_lot::Mutex;

    use super::*;
    use crate::driver::DriverLinks;
    use crate::task::{FlatFieldJob, Shared};
    use crate::types::Event;

    type Log = Arc<Mutex<Vec<String>>>;

    /// A camserver that logs `<flush>`, each command and `<read>`, and
    /// answers a read with `reply(last command)`.
    struct ScriptedCamserver {
        base: PortDriverBase,
        log: Log,
        reply: fn(&str) -> String,
        last: String,
    }

    impl PortDriver for ScriptedCamserver {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_flush(&mut self, _user: &mut AsynUser) -> AsynResult<()> {
            self.log.lock().push("<flush>".into());
            Ok(())
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.last = String::from_utf8_lossy(data).into_owned();
            self.log.lock().push(self.last.clone());
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            self.log.lock().push("<read>".into());
            let reply = (self.reply)(&self.last);
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            OctetReadResult::ok(reply.len(), EomReason::EOS)
        }
    }

    /// Registers a [`ScriptedCamserver`] as `cam` and returns its log.
    fn scripted_camserver(cam: &str, reply: fn(&str) -> String) -> Log {
        let log = Log::default();
        std::mem::forget(
            PortManager::new()
                .register_port(
                    claim_port_driver(cam).unwrap(),
                    ScriptedCamserver {
                        base: PortDriverBase::new(
                            cam,
                            1,
                            PortFlags::default(),
                            InterfaceMask::UNMIGRATED_ALL,
                            InterfaceMask::UNMIGRATED_INTERRUPTS,
                        ),
                        log: log.clone(),
                        reply,
                        last: String::new(),
                    },
                    &open_owner(),
                )
                .unwrap(),
        );
        log
    }

    /// A detector on a fresh [`ScriptedCamserver`]; its ADStatus posts land
    /// in the same log as `ADStatus=<v>`.
    fn rig(name: &str, reply: fn(&str) -> String) -> (PilatusDriver, Log) {
        let cam = format!("{name}_CAM");
        let log = scripted_camserver(&cam, reply);
        let (flat_field_tx, flat_field_rx) = rt::command_channel::<FlatFieldJob>(1);
        std::mem::forget(flat_field_rx);
        let mut det = PilatusDriver::new(
            name,
            8,
            8,
            0,
            DriverLinks {
                flat_field_tx,
                start: Arc::new(Event::new()),
                stop: Arc::new(Event::new()),
                shared: Arc::new(Mutex::new(Shared::default())),
                cam: Arc::new(
                    SyncIOHandle::connect_port(&cam, 0, Duration::ZERO, InterfaceType::Octet).0,
                ),
            },
        )
        .unwrap();
        det.ad.port_base.own_interrupts(InterruptGate::opened());
        let posts = log.clone();
        let status = det.ad.params.status;
        std::mem::forget(
            det.ad
                .port_base
                .interrupts
                .register_sync_callback(
                    InterruptFilter {
                        reason: Some(status),
                        addr: Some(0),
                        ..Default::default()
                    },
                    move |iv| {
                        if let ParamValue::Int32(v) = &iv.value {
                            posts.lock().push(format!("ADStatus={v}"));
                        }
                    },
                )
                .unwrap(),
        );
        (det, log)
    }

    /// The commands alone, in order.
    fn commands(log: &Log) -> Vec<String> {
        log.lock()
            .iter()
            .filter(|e| !e.starts_with('<') && !e.starts_with("ADStatus"))
            .cloned()
            .collect()
    }

    /// pilatusDetector.cpp:748-829 on a fresh detector: the acquisition
    /// parameters with their defaults, then `Tau`, whose cutoff lands in
    /// PixelCutOff.
    #[test]
    fn set_acquire_params_sends_the_c_commands_and_reads_back_the_cutoff() {
        let (mut det, log) = rig("PILATUS_SAP", |cmd| match cmd {
            "Tau" => {
                "15 OK Rate correction is on; tau = 1.200000e-07 s, cutoff = 1221026 counts".into()
            }
            _ => "15 OK".into(),
        });
        let ad = det.ad.params;
        det.write_float64(&mut AsynUser::new(ad.acquire_time), 1.0)
            .unwrap();
        assert_eq!(
            commands(&log),
            [
                "nimages 1",
                "nexpframe 1",
                "exptime  1.00000000",
                "expperiod  2.00000000",
                "delay 0.000000",
                "Tau"
            ]
        );
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(det.p.pixel_cutoff, 0)
                .unwrap(),
            1221026
        );
    }

    /// pilatusDetector.cpp:766-810: a parameter no record has written yet
    /// fails its get, and C stores the fallback as it does for an invalid
    /// value. GapFill's fallback, -2, is also its one "not sent" value.
    #[test]
    fn set_acquire_params_stores_each_fallback_for_an_undefined_parameter() {
        let (mut det, log) = rig("PILATUS_SAP_UNDEF", ok);
        det.set_acquire_params();
        assert_eq!(commands(&log), FRESH_ACQUIRE_PARAMS);
        let (ad, p) = (det.ad.params, det.p);
        let base = &det.ad.port_base;
        assert_eq!(base.get_int32_param(ad.num_images, 0).unwrap(), 1);
        assert_eq!(base.get_int32_param(ad.num_exposures, 0).unwrap(), 1);
        assert_eq!(base.get_float64_param(ad.acquire_time, 0).unwrap(), 1.0);
        assert_eq!(base.get_float64_param(ad.acquire_period, 0).unwrap(), 2.0);
        assert_eq!(base.get_float64_param(p.delay_time, 0).unwrap(), 0.0);
        assert_eq!(base.get_int32_param(p.gap_fill, 0).unwrap(), -2);
    }

    /// pilatusDetector.cpp:806-814 at each edge of `-2..=0`: inside it the
    /// value is kept and only -2 is not sent; outside it -2 is stored.
    #[test]
    fn set_acquire_params_gap_fill_edges() {
        for (set, kept, sent) in [
            (-3, -2, None),
            (-2, -2, None),
            (-1, -1, Some("gapfill -1")),
            (0, 0, Some("gapfill 0")),
            (1, -2, None),
        ] {
            let (mut det, log) = rig(&format!("PILATUS_GAP_{}", set + 3), ok);
            let p = det.p;
            det.ad.port_base.set_i32(p.gap_fill, set);
            det.set_acquire_params();
            let cmds = commands(&log);
            let gap = cmds.iter().find(|c| c.starts_with("gapfill"));
            assert_eq!(gap.map(String::as_str), sent, "set {set}");
            let stored = det.ad.port_base.get_int32_param(p.gap_fill, 0).unwrap();
            assert_eq!(stored, kept, "set {set}");
        }
    }

    /// The acquisition parameters setAcquireParams sends on a fresh
    /// detector (pilatusDetector.cpp:748-829).
    const FRESH_ACQUIRE_PARAMS: [&str; 6] = [
        "nimages 1",
        "nexpframe 1",
        "exptime  1.00000000",
        "expperiod  2.00000000",
        "delay 0.000000",
        "Tau",
    ];

    /// pilatusDetector.cpp:833-889: Waiting is posted before `SetThreshold
    /// energy`, the two readbacks follow, then setAcquireParams re-sends.
    #[test]
    fn set_threshold_posts_waiting_then_sends_the_c_commands() {
        let (mut det, log) = rig("PILATUS_THR", |cmd| match cmd {
            "SetThreshold" => "15 OK  Settings: lowG gain; threshold: 10000 eV; vcmp: 0.6 V".into(),
            "SetEnergy" => "15 OK Energy setting: 20000 eV".into(),
            _ => "15 OK".into(),
        });
        det.shared.lock().demanded_threshold = 10.0;
        let p = det.p;
        det.write_int32(&mut AsynUser::new(p.threshold_apply), 1)
            .unwrap();

        let events = log.lock().clone();
        let waiting = format!("ADStatus={}", ADStatus::Waiting as i32);
        let posted = events.iter().position(|e| *e == waiting);
        let sent = events
            .iter()
            .position(|e| e.starts_with("SetThreshold energy"));
        assert!(
            matches!((posted, sent), (Some(w), Some(c)) if w < c),
            "{events:?}"
        );
        let mut expected = vec![
            "SetThreshold energy 20000 lowG 10000",
            "SetThreshold",
            "SetEnergy",
        ];
        expected.extend(FRESH_ACQUIRE_PARAMS);
        assert_eq!(commands(&log), expected);
        let base = &det.ad.port_base;
        assert_eq!(base.get_float64_param(p.threshold, 0).unwrap(), 10.0);
        assert_eq!(base.get_float64_param(p.energy, 0).unwrap(), 20.0);
    }

    fn ok(_: &str) -> String {
        "15 OK".into()
    }

    /// pilatusDetector.cpp:893-912: camserver before 7.9.0 has no
    /// `ResetModulePower`; from 7.9.0 it gets `ResetModulePower <ResetPowerTime>`
    /// and then setThreshold's commands.
    #[test]
    fn reset_module_power_needs_camserver_7_9() {
        for (version, supported) in [
            ((6, 10, 0), false),
            ((7, 8, 9), false),
            ((7, 9, 0), true),
            ((8, 0, 0), true),
        ] {
            let name = format!("PILATUS_RMP_{}_{}_{}", version.0, version.1, version.2);
            let (mut det, log) = rig(&name, ok);
            det.version = version;
            let reason = det.p.reset_power;
            det.write_int32(&mut AsynUser::new(reason), 1).unwrap();
            let cmds = commands(&log);
            if supported {
                assert_eq!(
                    cmds.get(..2),
                    Some(
                        &[
                            "ResetModulePower 1".to_string(),
                            "SetThreshold energy 0 lowG 0".to_string()
                        ][..]
                    ),
                    "{version:?}"
                );
            } else {
                assert!(cmds.is_empty(), "{version:?}: {cmds:?}");
            }
        }
    }

    /// pilatusDetector.cpp:1406-1416: an Acquire 0 while acquiring sends
    /// `camcmd k` and reads, sends `K` without reading, then sleeps 2 s.
    #[test]
    fn an_abort_sends_camcmd_k_then_k_and_sleeps_two_seconds() {
        let (mut det, log) = rig("PILATUS_ABORT", ok);
        let ad = det.ad.params;
        det.ad
            .port_base
            .set_i32(ad.status, ADStatus::Acquire as i32);
        let start = std::time::Instant::now();
        det.write_int32(&mut AsynUser::new(ad.acquire), 0).unwrap();
        let elapsed = start.elapsed();
        let io: Vec<String> = log
            .lock()
            .iter()
            .filter(|e| !e.starts_with("ADStatus"))
            .cloned()
            .collect();
        assert_eq!(io, ["<flush>", "camcmd k", "<read>", "<flush>", "K"]);
        assert!(elapsed >= Duration::from_secs(2), "{elapsed:?}");
        assert_eq!(
            det.ad
                .port_base
                .get_string_param(ad.status_message, 0)
                .unwrap(),
            b"Acquisition aborted"
        );
    }

    /// pilatusDetector.cpp:1304-1330: only the first pilatusStatus reads
    /// `version`; every one reads `thread`.
    #[test]
    fn pilatus_status_reads_the_version_once() {
        let (mut det, log) = rig("PILATUS_STATUS_VERSION", |cmd| match cmd {
            "version" => "15 OK Code release: 7.9.0".into(),
            _ => "15 OK".into(),
        });
        let reason = det.ad.params.read_status;
        det.write_int32(&mut AsynUser::new(reason), 1).unwrap();
        det.write_int32(&mut AsynUser::new(reason), 1).unwrap();
        assert_eq!(commands(&log), ["version", "thread", "thread"]);
        assert_eq!(det.version, (7, 9, 0));
        assert_eq!(
            det.ad
                .port_base
                .get_string_param(det.p.tvx_version, 0)
                .unwrap(),
            b"7.9.0"
        );
    }

    /// pilatusDetector.cpp:1429-1431: ReadStatus is ignored while acquiring.
    #[test]
    fn read_status_while_acquiring_sends_nothing() {
        let (mut det, log) = rig("PILATUS_STATUS_ACQUIRING", ok);
        let ad = det.ad.params;
        det.ad
            .port_base
            .set_i32(ad.status, ADStatus::Acquire as i32);
        det.write_int32(&mut AsynUser::new(ad.read_status), 1)
            .unwrap();
        assert!(commands(&log).is_empty(), "{:?}", log.lock());
    }

    /// pilatusDetector.cpp:1834-1837: the constructor calls pilatusStatus once.
    #[test]
    fn the_constructor_reads_the_status_once() {
        use epics_rs::asyn::registry::claim_port;

        let log = scripted_camserver("PILATUS_BOOT_CAM", ok);
        let rt = crate::driver::create_pilatus_detector(
            &claim_port("PILATUS_BOOT").unwrap(),
            "PILATUS_BOOT_CAM",
            8,
            8,
            0,
        )
        .expect("the detector port");
        assert_eq!(commands(&log), ["version", "thread"]);
        std::mem::forget(rt);
    }

    /// pilatusDetector.cpp:249-250: an empty name zeroes NumBadPixels, so no
    /// pixel of the previous file is replaced any more.
    #[test]
    fn an_empty_bad_pixel_file_name_clears_the_list() {
        let (mut det, log) = rig("PILATUS_BAD_PIXEL_CLEAR", ok);
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("bad.txt");
        std::fs::write(&file, "1,2 3,4\n").unwrap();
        let reason = det.p.num_bad_pixels;

        det.read_bad_pixel_file(file.to_str().unwrap());
        assert_eq!(det.shared.lock().bad_pixels.len(), 1);
        assert_eq!(det.get_i32(reason).unwrap(), 1);

        det.read_bad_pixel_file("");
        assert!(det.shared.lock().bad_pixels.is_empty());
        assert_eq!(det.get_i32(reason).unwrap(), 0);
        assert!(commands(&log).is_empty());
    }

    /// pilatusDetector.cpp:1426-1428: NumOscill is one mxsettings exchange.
    #[test]
    fn num_oscill_sends_mxsettings_n_oscillations() {
        let (mut det, log) = rig("PILATUS_NUM_OSCILL", ok);
        det.write_int32(&mut AsynUser::new(det.p.num_oscill), 3)
            .unwrap();
        let io: Vec<String> = log
            .lock()
            .iter()
            .filter(|e| !e.starts_with("ADStatus"))
            .cloned()
            .collect();
        assert_eq!(io, ["<flush>", "mxsettings N_oscillations 3", "<read>"]);
    }

    /// pilatusDetector.cpp:922: writeCamserver flushes stale input before
    /// every write.
    #[test]
    fn every_camserver_write_flushes_first() {
        let (mut det, log) = rig("PILATUS_CAM_FLUSH", ok);
        det.cam_write("Exposure x.tif", 1.0);
        det.cam_write("K", 1.0);
        assert_eq!(*log.lock(), ["<flush>", "Exposure x.tif", "<flush>", "K"]);
    }
}
