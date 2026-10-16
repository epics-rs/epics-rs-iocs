//! The ADTimePix3 asyn port driver (port of the `ADTimePix` class,
//! `ADTimePix.cpp` + `serval_http.cpp` + `acquire.cpp` + `mask_io.cpp`).

use std::sync::Arc;

use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use serde_json::json;

use crate::adlog;
use crate::calibration::PathPolicy;
use crate::coordinator::{Phase, Resource};
use crate::destination;
use crate::family::DetectorFamily;
use crate::http::{HttpError, ServalHttp, TIMEOUT_CONFIG, TIMEOUT_POLL, decode_base64};
use crate::mask::{self, Geometry, PIXEL_CONFIG_BYTES};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Shared};
use crate::tasks;

/// The line C prints when a Serval request fails: `logHttpFailure` or
/// `logHttpWarning` with its context, method and path, or nothing.
#[derive(Clone, Copy)]
enum HttpPrint<'a> {
    None,
    Failure(&'a str, &'a str, &'a str),
    Warning(&'a str, &'a str, &'a str),
}

/// C's `maxAddr` (ADTimePix.cpp:1069): 0 = PrvImg, 1 = Img, 2 = Img running
/// sum, 3 = Img sum-of-N, 4 = PrvHst sum-of-N, 5 = PrvHst running sum,
/// 6 = PrvHst frame, 7 = PrvHst ToF axis. The chip DACs bind addresses 0-7 and
/// the voltage rails 0-5 on the same port.
pub const MAX_ADDR: usize = 8;

/// ADTIMEPIX_VERSION.REVISION.MODIFICATION (ADTimePix.h:20-22).
const DRIVER_VERSION: &str = "1.6.4";

/// C `calculateImgMemoryUsageMB()` before the first frame (ADTimePix.cpp:1545):
/// the 512x512 UINT16 current frame, the 512x512 default for the two Int64 and
/// one Int32 waveforms, plus the fixed 0.1 MB overhead.
const INITIAL_IMG_MEMORY_MB: f64 =
    (512.0 * 512.0 * (2.0 + 8.0 + 8.0 + 4.0)) / (1024.0 * 1024.0) + 0.1;

/// The per-chip DAC names Serval takes on `/detector/chips/<n>/dacs/`
/// (ADTimePix.cpp:1123-1147). These are on-wire names, not display names.
type DacParam = fn(&TimePixParams) -> usize;

const DAC_NAMES: [(&str, DacParam); 18] = [
    ("Ibias_Preamp_ON", |p| p.preamp_on),
    ("Ibias_Preamp_OFF", |p| p.preamp_of_f),
    ("VPreamp_NCAS", |p| p.v_preamp_nc_as),
    ("Ibias_Ikrum", |p| p.ikrum),
    ("Vfbk", |p| p.vfbk),
    ("Vthreshold_fine", |p| p.vthreshold_fine),
    ("Vthreshold_coarse", |p| p.vthreshold_coarse),
    ("Ibias_DiscS1_ON", |p| p.disc_s1_on),
    ("Ibias_DiscS1_OFF", |p| p.disc_s1_of_f),
    ("Ibias_DiscS2_ON", |p| p.disc_s2_on),
    ("Ibias_DiscS2_OFF", |p| p.disc_s2_of_f),
    ("Ibias_PixelDAC", |p| p.pixel_da_c),
    ("Ibias_TPbufferIn", |p| p.t_pbuffer_in),
    ("Ibias_TPbufferOut", |p| p.t_pbuffer_out),
    ("VTP_coarse", |p| p.vt_pcoarse),
    ("VTP_fine", |p| p.vt_pfine),
    ("Ibias_CP_PLL", |p| p.cp_pl_l),
    ("PLL_Vcntrl", |p| p.pl_l_vcntrl),
];

/// The DAC table as `(Serval name, parameter index)` pairs.
pub fn dac_params(p: &TimePixParams) -> Vec<(&'static str, usize)> {
    DAC_NAMES.iter().map(|(name, f)| (*name, f(p))).collect()
}

type ParamOf = fn(&TimePixParams) -> usize;

/// The accumulation settings and the range writeInt32 clamps each to
/// (ADTimePix.cpp:758, :789, :805-806, :816, :833, :851-852;
/// `MAX_RETENTION_LIMIT_MB`, ADTimePix.h:988).
const ACCUMULATION_LIMITS: [(ParamOf, i32, i32); 6] = [
    (|p| p.img_frames_to_sum, 1, 100_000),
    (|p| p.img_retention_limit_mb, 1, 4096),
    (|p| p.img_sum_update_interval_frames, 1, 10_000),
    (|p| p.prv_hst_frames_to_sum, 1, 100_000),
    (|p| p.prv_hst_retention_limit_mb, 1, 4096),
    (|p| p.prv_hst_sum_update_interval, 1, 10_000),
];

/// One channel's `Base` PV check (C `checkRawPath` .. `checkPrvHstPath`,
/// ADTimePix.cpp:282-398).
struct ChannelPath {
    base: fn(&TimePixParams) -> usize,
    stream: Option<fn(&TimePixParams) -> usize>,
    exists: fn(&TimePixParams) -> usize,
    error_message: &'static str,
    /// The channels whose `tcp://` path C parses into the stream target, and
    /// the function name its lines carry.
    tcp: Option<(&'static str, &'static str)>,
}

const CHANNEL_PATHS: [ChannelPath; 7] = [
    ChannelPath {
        base: |p| p.raw_base,
        stream: Some(|p| p.raw_stream),
        exists: |p| p.raw_file_path_exists,
        error_message: "Raw file path must be file:/path_to_raw_folder, http://localhost:8081, or tcp://localhost:8085",
        tcp: None,
    },
    ChannelPath {
        base: |p| p.raw1_base,
        stream: Some(|p| p.raw1_stream),
        exists: |p| p.raw1_file_path_exists,
        error_message: "Raw1 file path must be file:/path_to_raw_folder, http://localhost:8081, or tcp://localhost:8085",
        tcp: None,
    },
    ChannelPath {
        base: |p| p.img_base,
        stream: None,
        exists: |p| p.img_file_path_exists,
        error_message: "Img file path must be file:/path_to_img_folder, http://localhost:8081, or tcp://listen@hostname:port",
        tcp: Some(("Img", adlog::CHECK_IMG_PATH)),
    },
    ChannelPath {
        base: |p| p.img1_base,
        stream: None,
        exists: |p| p.img1_file_path_exists,
        error_message: "Img1 file path must be file:/path_to_img_folder, http://localhost:8081, or tcp://localhost:8085",
        tcp: None,
    },
    ChannelPath {
        base: |p| p.prv_img_base,
        stream: None,
        exists: |p| p.prv_img_file_path_exists,
        error_message: "PrvImg file path must be file:/path_to_img_folder, http://localhost:8081, or tcp://listen@hostname:port",
        tcp: Some(("PrvImg", adlog::CHECK_PRV_IMG_PATH)),
    },
    ChannelPath {
        base: |p| p.prv_img1_base,
        stream: None,
        exists: |p| p.prv_img1_file_path_exists,
        error_message: "PrvImg1 file path must be file:/path_to_img_folder, http://localhost:8081, or tcp://listen@hostname:port",
        tcp: Some(("PrvImg1", adlog::CHECK_PRV_IMG1_PATH)),
    },
    ChannelPath {
        base: |p| p.prv_hst_base,
        stream: Some(|p| p.prv_hst_stream),
        exists: |p| p.prv_hst_file_path_exists,
        error_message: "Prv Histogram path must be file:/path_to_folder, http://localhost:8081, or tcp://localhost:8085",
        tcp: None,
    },
];

/// C `checkPath` (ADTimePix.cpp:115): one trailing '/' is dropped for the
/// directory test and the path always comes back ending in '/'.
fn check_path(path: &str) -> (String, bool) {
    if path.is_empty() {
        return (String::new(), false);
    }
    let trimmed = path.strip_suffix('/').unwrap_or(path);
    (
        format!("{trimmed}/"),
        std::path::Path::new(trimmed).is_dir(),
    )
}

/// C `ADTimePix3Action::oneShotDecision` (one_shot_action.h): an action
/// parameter runs on 1 only and always reads back 0.
fn one_shot(value: i32) -> bool {
    value == 1
}

/// C `kMpx3BothCountersTriggerMsg` (ADTimePix.cpp:2129-2130).
const MPX3_TRIGGER_MSG: &str =
    "MPX3: BothCounters requires non-Continuous TriggerMode (use AutoTrgSt_TmrSp=4)";
/// C `kMpx3BothCountersPixelDepthMsg` (ADTimePix.cpp:2132-2133).
const MPX3_PIXEL_DEPTH_MSG: &str =
    "MPX3: PixelDepth 24 incompatible with BothCounters (use 12-bit)";

pub struct TimePix3Driver {
    pub ad: ADDriverBase,
    pub p: TimePixParams,
    pub http: Arc<ServalHttp>,
    pub shared: Arc<Shared>,
    cmd_tx: rt::CommandSender<Command>,
    calibration: PathPolicy,
    destinations: destination::Policy,
}

impl TimePix3Driver {
    pub fn new(
        port_name: &str,
        http: Arc<ServalHttp>,
        shared: Arc<Shared>,
        max_memory: usize,
        cmd_tx: rt::CommandSender<Command>,
    ) -> AsynResult<Self> {
        // ADDriver(portName, 8, ..., ASYN_MULTIDEVICE | ASYN_CANBLOCK, ...),
        // ADTimePix.cpp:1072-1076.
        let mut ad = ADDriverBase::new(
            port_name,
            MAX_ADDR,
            0,
            0,
            max_memory,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
        )?;
        let p = TimePixParams::create(&mut ad.port_base)?;

        let mut driver = Self {
            ad,
            p,
            http,
            shared,
            cmd_tx,
            calibration: PathPolicy::default(),
            destinations: destination::Policy::default(),
        };
        driver.init_params()?;
        driver.configure_calibration(
            &std::env::var("ADTIMEPIX_CALIBRATION_ROOT").unwrap_or_default(),
        )?;
        driver.configure_destinations(
            &std::env::var("ADTIMEPIX_DESTINATION_ALLOWLIST").unwrap_or_default(),
        )?;
        Ok(driver)
    }

    /// C `ADTimePixConfig`'s ADTIMEPIX_DESTINATION_ALLOWLIST, permissive when
    /// unset (ADTimePix.cpp:76-78, :1612-1625).
    fn configure_destinations(&mut self, allowlist: &str) -> AsynResult<()> {
        let status = self.destinations.configure(if allowlist.is_empty() {
            destination::PERMISSIVE
        } else {
            allowlist
        });
        let policy = match status {
            Ok(()) if self.destinations.permissive() => "Permissive".to_string(),
            Ok(()) => "Enforced".to_string(),
            Err(e) => format!("Blocked: {}", e.message()),
        };
        let description = self.destinations.description().to_string();
        self.set_text(self.p.destination_allowlist, &description)?;
        self.set_text(self.p.destination_policy, &policy)
    }

    /// C `validateDestination` (ADTimePix.cpp:171-182).
    fn validate_destination(&mut self, destination: &str) -> AsynResult<()> {
        if self.destinations.allows(destination) {
            return Ok(());
        }
        let message = format!("Destination blocked by startup allowlist: {destination}");
        self.set_text(self.p.write_msg, &message)?;
        self.ad.port_base.trace_print(
            TraceMask::ERROR,
            format_args!("{}", adlog::err(adlog::VALIDATE_DESTINATION, &message)),
        );
        self.ad.port_base.call_param_callbacks(0)?;
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message,
        })
    }

    /// C `checkChannelPath` (ADTimePix.cpp:217-280), plus the `tcp://` parse
    /// `checkImgPath` and the two PrvImg checks add.
    fn check_channel_path(&mut self, channel: &ChannelPath) -> AsynResult<()> {
        let p = self.p;
        let path = self.text((channel.base)(&p), 0);
        if path.is_empty() {
            return Ok(());
        }
        let bytes = path.as_bytes();
        let (stream, found) = if bytes.len() <= 6 {
            (None, false)
        } else if let Some(dir) = path.strip_prefix("file:") {
            if bytes[5] == b'/' && bytes[6] != b'/' {
                (Some(0), check_path(dir).1)
            } else {
                println!("Invalid file path format: '{path}'. Expected format: 'file:/path'.");
                println!(
                    "Debug: length={}, char[5]='{}', char[6]='{}'",
                    bytes.len(),
                    char::from(bytes[5]),
                    char::from(bytes[6])
                );
                (Some(3), false)
            }
        } else if path.starts_with("http://") {
            (Some(1), true)
        } else if path.starts_with("tcp://") {
            (Some(2), true)
        } else {
            println!("{}", channel.error_message);
            (Some(3), false)
        };
        if let (Some(param), Some(value)) = (channel.stream, stream) {
            self.ad.port_base.set_int32_param(param(&p), 0, value)?;
        }
        let exists = (channel.exists)(&p);
        self.ad
            .port_base
            .set_int32_param(exists, 0, i32::from(found))?;
        let refused = || AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: {}", channel.error_message),
        };
        if !found {
            return Err(refused());
        }
        let Some((name, func)) = channel.tcp.filter(|_| path.starts_with("tcp://")) else {
            return Ok(());
        };
        let (mask, line) = match crate::stream::parse_tcp_path(&path) {
            Some((host, port)) => (
                TraceMask::IO_DRIVER,
                adlog::log(
                    func,
                    &format!("Parsed {name} TCP path: host={host}, port={port}"),
                ),
            ),
            None => {
                self.ad.port_base.set_int32_param(exists, 0, 0)?;
                (
                    TraceMask::ERROR,
                    adlog::err(func, &format!("Failed to parse {name} TCP path: {path}")),
                )
            }
        };
        self.ad
            .port_base
            .trace_print(mask, format_args!("{}", line));
        if mask == TraceMask::ERROR {
            return Err(refused());
        }
        Ok(())
    }

    /// C `checkBPCPath` (mask_io.cpp:502) and `checkDACSPath`
    /// (ADTimePix.cpp:185): the directory must exist inside the calibration
    /// root, and is stored back with a trailing '/'.
    fn check_calibration_dir(&mut self, path: usize, exists: usize, func: &str) -> AsynResult<()> {
        let dir = self.text(path, 0);
        if dir.is_empty() {
            return Ok(());
        }
        let (normalized, found) = check_path(&dir);
        let verdict = self.calibration.validate_directory(&normalized);
        let permitted = found && verdict.is_ok();
        self.set_text(path, &normalized)?;
        self.ad
            .port_base
            .set_int32_param(exists, 0, i32::from(permitted))?;
        if permitted {
            return Ok(());
        }
        let message = format!(
            "Calibration path blocked: {}",
            verdict.err().map_or("ok", |e| e.message())
        );
        self.set_text(self.p.write_msg, &message)?;
        self.ad.port_base.trace_print(
            TraceMask::ERROR,
            format_args!(
                "{}",
                adlog::err(func, &format!("{message} (directory=\"{normalized}\")"))
            ),
        );
        self.ad.port_base.call_param_callbacks(0)?;
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message,
        })
    }

    /// C `ADTimePixConfig`'s ADTIMEPIX_CALIBRATION_ROOT, "/" when unset
    /// (ADTimePix.cpp:74-75, :1597-1609).
    fn configure_calibration(&mut self, root: &str) -> AsynResult<()> {
        let status = self
            .calibration
            .configure(if root.is_empty() { "/" } else { root });
        let policy = match status {
            Ok(()) if self.calibration.permissive() => "Permissive: /".to_string(),
            Ok(()) => format!("Enforced: {}", self.calibration.root()),
            Err(e) => format!("Blocked: {}", e.message()),
        };
        let root = self.calibration.root();
        self.set_text(self.p.calibration_root, &root)?;
        self.set_text(self.p.calibration_policy, &policy)
    }

    /// C's constructor seeds (ADTimePix.cpp:1414-1553, ADTimePix3 R1-6-4).
    /// Manufacturer and Model come from the dashboard, and the PrvHst sum
    /// parameters from the first histogram frame or write, as in C.
    fn init_params(&mut self) -> AsynResult<()> {
        let server = self.http.base_url().to_string();
        let p = self.p;
        let ad = &self.ad.params;
        let base = &mut self.ad.port_base;
        base.set_string_param(ad.base.driver_version, 0, DRIVER_VERSION)?;
        base.set_string_param(p.server_name, 0, server)?;
        base.set_int32_param(p.img_accumulation_enable, 0, 1)?;
        base.set_int32_param(p.prv_hst_accumulation_enable, 0, 1)?;
        base.set_int32_param(p.img_frames_to_sum, 0, 10)?;
        base.set_int32_param(p.img_sum_update_interval_frames, 0, 1)?;
        base.set_int64_param(p.img_total_counts, 0, 0)?;
        base.set_float64_param(p.img_processing_time, 0, 0.0)?;
        base.set_float64_param(p.img_memory_usage, 0, INITIAL_IMG_MEMORY_MB)?;
        base.set_int32_param(p.write_processed_img, 0, 0)?;
        base.set_int32_param(p.processed_img_output_type, 0, 0)?;
        base.set_int32_param(p.write_processed_hst, 0, 0)?;
        base.set_int32_param(p.processed_hst_output_type, 0, 0)?;
        base.set_int32_param(ad.num_images, 0, 0)?;
        Ok(())
    }

    fn int(&self, index: usize, addr: i32) -> i32 {
        self.ad.port_base.get_int32_param(index, addr).unwrap_or(0)
    }

    fn dbl(&self, index: usize, addr: i32) -> f64 {
        self.ad
            .port_base
            .get_float64_param(index, addr)
            .unwrap_or(0.0)
    }

    fn text(&self, index: usize, addr: i32) -> String {
        String::from_utf8_lossy(
            self.ad
                .port_base
                .get_string_param(index, addr)
                .unwrap_or_default(),
        )
        .into_owned()
    }

    /// Publish the HTTP status of the last request, as C's `TPX3_HTTP_CODE`
    /// does, print the line C prints for it, and turn the failure into an
    /// asyn error.
    fn http_failed(&mut self, what: &str, e: &HttpError, print: HttpPrint) -> AsynError {
        let _ = self
            .ad
            .port_base
            .set_int32_param(self.p.http_code, 0, e.code());
        self.log_http(e, print);
        AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: {what}: {e}"),
        }
    }

    /// C `logHttpFailure`/`logHttpWarning`, which leave HttpCode alone.
    fn log_http(&mut self, e: &HttpError, print: HttpPrint) {
        let url = |path: &str| format!("{}{path}", self.http.base_url());
        let line = match print {
            HttpPrint::None => None,
            HttpPrint::Failure(context, method, path) => {
                adlog::http_failure(context, method, &url(path), e).map(|l| (TraceMask::ERROR, l))
            }
            HttpPrint::Warning(context, method, path) => {
                adlog::http_warning(context, method, &url(path), e).map(|l| (TraceMask::WARNING, l))
            }
        };
        if let Some((mask, line)) = line {
            self.ad
                .port_base
                .trace_print(mask, format_args!("{}", line));
        }
    }

    fn http_ok(&mut self) {
        let _ = self.ad.port_base.set_int32_param(self.p.http_code, 0, 200);
    }

    /// C `mpx3BothCountersTriggerConflict` (ADTimePix.cpp:2135-2143).
    fn mpx3_trigger_conflict(&self, trigger_mode: i32) -> bool {
        self.shared.family() == DetectorFamily::Mpx3
            && self.int(self.p.both_counters, 0) != 0
            && trigger_mode == 5
    }

    /// C `mpx3BothCountersPixelDepthConflict` (ADTimePix.cpp:2145-2153).
    fn mpx3_pixel_depth_conflict(&self, pixel_depth: i32) -> bool {
        self.shared.family() == DetectorFamily::Mpx3
            && self.int(self.p.both_counters, 0) != 0
            && pixel_depth == 24
    }

    /// C `LOG`/`LOG_ARGS` in `writeInt32`.
    fn log_write_int32(&self, msg: &str) {
        self.ad.port_base.trace_print(
            TraceMask::IO_DRIVER,
            format_args!("{}", adlog::log(adlog::WRITE_INT32, msg)),
        );
    }

    fn set_text(&mut self, index: usize, text: &str) -> AsynResult<()> {
        self.ad.port_base.set_string_param(index, 0, text)
    }

    // -- Serval requests -----------------------------------------------------

    /// C `initAcquisition` (serval_http.cpp:2254): read `/detector/config`,
    /// merge the driver's settings into it, PUT it back.
    fn init_acquisition(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::DETECTOR_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /detector/config", &e, HttpPrint::None))?;

        let cfg = serval::DetectorConfig {
            trigger_mode: self.int(self.ad.params.trigger_mode, 0),
            exposure_time: self.dbl(self.ad.params.acquire_time, 0),
            trigger_period: self.dbl(self.ad.params.acquire_period, 0),
            trigger_delay: self.dbl(self.p.trigger_delay, 0),
            global_timestamp_interval: self.dbl(self.p.global_timestamp_interval, 0),
            n_triggers: self.int(self.ad.params.num_images, 0),
            bias_voltage: self.int(self.p.bias_volt, 0),
            bias_enabled: self.int(self.p.bias_enable, 0) != 0,
            chain_mode: self.int(self.p.chain_mode, 0),
            polarity: self.int(self.p.polarity, 0),
            trigger_in: self.int(self.p.trigger_in, 0),
            trigger_out: self.int(self.p.trigger_out, 0),
            log_level: self.int(self.p.log_level, 0),
            external_reference_clock: self.int(self.p.external_reference_clock, 0) != 0,
            periph_clk80: self.int(self.p.periph_clk80, 0) != 0,
            tdc0: self.int(self.p.tdc0, 0),
            tdc1: self.int(self.p.tdc1, 0),
        };
        let body = serval::detector_config_body(&current, &cfg).map_err(|m| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: /detector/config: {m}"),
        })?;
        self.http
            .put_json(serval::DETECTOR_CONFIG, &body, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("PUT /detector/config", &e, HttpPrint::None))?;
        self.http_ok();
        Ok(())
    }

    /// C `sendMeasurementConfig` (serval_http.cpp:2033).
    fn send_measurement_config(&mut self) -> AsynResult<()> {
        let current = self
            .http
            .get_json(serval::MEASUREMENT_CONFIG, TIMEOUT_CONFIG)
            .map_err(|e| self.http_failed("GET /measurement/config", &e, HttpPrint::None))?;
        let cfg = serval::MeasurementConfig {
            scan_width: self.int(self.p.stem_scan_width, 0),
            scan_height: self.int(self.p.stem_scan_height, 0),
            dwell_time: self.dbl(self.p.stem_dwell_time, 0),
            radius_outer: self.int(self.p.stem_radius_outer, 0),
            radius_inner: self.int(self.p.stem_radius_inner, 0),
            tdc_reference: self.text(self.p.tof_tdc_reference, 0),
            tof_min: self.dbl(self.p.tof_min, 0),
            tof_max: self.dbl(self.p.tof_max, 0),
        };
        let body = serval::measurement_config_body(&current, &cfg);
        self.http
            .put_json(serval::MEASUREMENT_CONFIG, &body, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    "PUT /measurement/config",
                    &e,
                    HttpPrint::Failure(
                        "sendMeasurementConfig PUT /measurement/config",
                        "PUT",
                        serval::MEASUREMENT_CONFIG,
                    ),
                )
            })?;
        self.http_ok();
        Ok(())
    }

    /// The destination the enabled channels describe (C `fileWriter`,
    /// serval_http.cpp:2095).
    fn destination(&self) -> serval::Destination {
        let mut d = serval::Destination::default();

        for (enable, base, pat, split, queue) in [
            (
                self.p.write_raw,
                self.p.raw_base,
                self.p.raw_file_pat,
                self.p.raw_split_strategy,
                self.p.raw_queue_size,
            ),
            (
                self.p.write_raw1,
                self.p.raw1_base,
                self.p.raw1_file_pat,
                self.p.raw1_split_strategy,
                self.p.raw1_queue_size,
            ),
        ] {
            if self.int(enable, 0) != 0 {
                d.raw.push(serval::RawChannel {
                    base: self.text(base, 0),
                    file_pattern: self.text(pat, 0),
                    split_strategy: self.int(split, 0),
                    queue_size: self.int(queue, 0),
                });
            }
        }

        let image = |this: &Self, base, pat, format, mode, int_size, int_mode, stop, queue| {
            serval::ImageChannel {
                base: this.text(base, 0),
                file_pattern: this.text(pat, 0),
                format: this.int(format, 0),
                mode: this.int(mode, 0),
                integration_size: this.int(int_size, 0),
                integration_mode: this.int(int_mode, 0),
                stop_on_disk_limit: this.int(stop, 0) != 0,
                queue_size: this.int(queue, 0),
            }
        };

        if self.int(self.p.write_img, 0) != 0 {
            d.image.push(image(
                self,
                self.p.img_base,
                self.p.img_file_pat,
                self.p.img_format,
                self.p.img_mode,
                self.p.img_int_size,
                self.p.img_int_mode,
                self.p.img_stp_on_dsk_lim,
                self.p.img_queue_size,
            ));
        }
        if self.int(self.p.write_img1, 0) != 0 {
            d.image.push(image(
                self,
                self.p.img1_base,
                self.p.img1_file_pat,
                self.p.img1_format,
                self.p.img1_mode,
                self.p.img1_int_size,
                self.p.img1_int_mode,
                self.p.img1_stp_on_dsk_lim,
                self.p.img1_queue_size,
            ));
        }
        if self.int(self.p.write_prv_img, 0) != 0 {
            d.preview_image.push(image(
                self,
                self.p.prv_img_base,
                self.p.prv_img_file_pat,
                self.p.prv_img_format,
                self.p.prv_img_mode,
                self.p.prv_img_int_size,
                self.p.prv_img_int_mode,
                self.p.prv_img_stp_on_dsk_lim,
                self.p.prv_img_queue_size,
            ));
        }
        if self.int(self.p.write_prv_img1, 0) != 0 {
            d.preview_image.push(image(
                self,
                self.p.prv_img1_base,
                self.p.prv_img1_file_pat,
                self.p.prv_img1_format,
                self.p.prv_img1_mode,
                self.p.prv_img1_int_size,
                self.p.prv_img1_int_mode,
                self.p.prv_img1_stp_on_dsk_lim,
                self.p.prv_img1_queue_size,
            ));
        }
        if self.int(self.p.write_prv_hst, 0) != 0 {
            d.preview_histogram = Some(serval::HistogramChannel {
                image: image(
                    self,
                    self.p.prv_hst_base,
                    self.p.prv_hst_file_pat,
                    self.p.prv_hst_format,
                    self.p.prv_hst_mode,
                    self.p.prv_hst_int_size,
                    self.p.prv_hst_int_mode,
                    self.p.prv_hst_stp_on_dsk_lim,
                    self.p.prv_hst_queue_size,
                ),
                number_of_bins: self.int(self.p.prv_hst_num_bins, 0),
                bin_width: self.dbl(self.p.prv_hst_bin_width, 0),
                offset: self.dbl(self.p.prv_hst_offset, 0),
            });
        }
        d.preview = Some(serval::PreviewSettings {
            period: self.dbl(self.p.prv_period, 0),
            sampling_mode: self.int(self.p.prv_sampling_mode, 0),
        });
        d
    }

    /// C `fileWriter` (serval_http.cpp:2095).
    fn file_writer(&mut self) -> AsynResult<()> {
        let dest = self.destination();
        // Every enabled channel's Base passes the allowlist before anything
        // is sent (serval_http.cpp:1782, :1866, :2096).
        let bases: Vec<String> = dest
            .raw
            .iter()
            .map(|c| c.base.clone())
            .chain(dest.image.iter().map(|c| c.base.clone()))
            .chain(dest.preview_image.iter().map(|c| c.base.clone()))
            .chain(dest.preview_histogram.iter().map(|h| h.image.base.clone()))
            .collect();
        for base in &bases {
            self.validate_destination(base)?;
        }
        let body = serval::destination_body(&dest).map_err(|m| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: /server/destination: {m}"),
        })?;
        self.http
            .put_json(serval::SERVER_DESTINATION, &body, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    "PUT /server/destination",
                    &e,
                    HttpPrint::Failure(
                        "sendConfiguration PUT /server/destination",
                        "PUT",
                        serval::SERVER_DESTINATION,
                    ),
                )
            })?;
        self.http_ok();
        Ok(())
    }

    /// C `getServer` (serval_http.cpp:1420): read back the configured channels.
    fn get_server(&mut self) -> AsynResult<()> {
        let dest = self
            .http
            .get_json(serval::SERVER_DESTINATION, TIMEOUT_POLL)
            .map_err(|e| self.http_failed("GET /server/destination", &e, HttpPrint::None))?;
        self.http_ok();

        let raw = dest.get("Raw").and_then(|v| v.as_array());
        for (n, param) in [(0, self.p.write_raw_read), (1, self.p.write_raw1_read)] {
            let on = raw.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let img = dest.get("Image").and_then(|v| v.as_array());
        for (n, param) in [(0, self.p.write_img_read), (1, self.p.write_img1_read)] {
            let on = img.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let prv = dest
            .pointer("/Preview/ImageChannels")
            .and_then(|v| v.as_array());
        for (n, param) in [
            (0, self.p.write_prv_img_read),
            (1, self.p.write_prv_img1_read),
        ] {
            let on = prv.is_some_and(|a| a.len() > n);
            self.ad.port_base.set_int32_param(param, 0, i32::from(on))?;
        }
        let hst = dest
            .pointer("/Preview/HistogramChannels")
            .and_then(|v| v.as_array());
        self.ad.port_base.set_int32_param(
            self.p.write_prv_hst_read,
            0,
            i32::from(hst.is_some_and(|a| !a.is_empty())),
        )?;
        Ok(())
    }

    /// C `writeDac` (serval_http.cpp:2199).
    fn write_dac(&mut self, chip: i32, name: &str, value: i32) -> AsynResult<()> {
        if !(0..MAX_ADDR as i32).contains(&chip) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("timepix3: chip {chip} is outside 0..{}", MAX_ADDR - 1),
            });
        }
        let path = serval::chip_dacs(chip);
        let mut dacs = self.http.get_json(&path, TIMEOUT_CONFIG).map_err(|e| {
            let context = format!("writeDac GET chip {chip}");
            self.http_failed(
                &format!("GET {path}"),
                &e,
                HttpPrint::Failure(&context, "GET", &path),
            )
        })?;
        dacs[name] = json!(value);
        self.http
            .put_json(&path, &dacs, TIMEOUT_CONFIG)
            .map_err(|e| {
                self.http_failed(
                    &format!("PUT {path}"),
                    &e,
                    HttpPrint::Failure("writeDac PUT", "PUT", &path),
                )
            })?;
        self.http_ok();
        Ok(())
    }

    /// C `rotateLayout` (serval_http.cpp:476).
    fn rotate_layout(&mut self, orientation: i32) -> AsynResult<()> {
        let path = serval::layout_rotate(orientation).ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: orientation {orientation} is not one of the eight"),
        })?;
        self.http.get(&path, TIMEOUT_CONFIG).map_err(|e| {
            self.http_failed(
                "GET /detector/layout/rotate",
                &e,
                HttpPrint::Failure("rotateLayout", "GET", &path),
            )
        })?;
        self.http_ok();
        self.shared.set_orientation(orientation);
        Ok(())
    }

    /// C `uploadBPC` / `uploadDACS` (mask_io.cpp:339): Serval loads the file
    /// from its own filesystem, so only the path travels.
    fn config_load(&mut self, format: &str, path: usize, name: usize) -> AsynResult<()> {
        let file = format!("{}{}", self.text(path, 0), self.text(name, 0));
        let url = serval::config_load(format, &file);
        self.http.get(&url, TIMEOUT_CONFIG).map_err(|e| {
            let context = if format == "dacs" {
                "uploadDACS GET /config/load dacs"
            } else {
                "uploadBPC GET /config/load pixelconfig"
            };
            self.http_failed(
                &format!("GET /config/load ({format})"),
                &e,
                HttpPrint::Failure(context, "GET", &url),
            )
        })?;
        self.http_ok();
        Ok(())
    }

    // -- Mask ----------------------------------------------------------------

    /// The layout Serval reported, as the mask code needs it.
    fn geometry(&self) -> AsynResult<Geometry> {
        Geometry::new(
            self.int(self.p.row_len, 0),
            self.int(self.p.number_of_chips, 0),
            self.int(self.p.number_of_rows, 0),
            self.int(self.p.detector_orientation, 0),
        )
        .ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: "timepix3: the detector layout is not known yet (no /detector reply)".into(),
        })
    }

    fn bpc_path(&self) -> std::path::PathBuf {
        std::path::PathBuf::from(format!(
            "{}{}",
            self.text(self.p.bp_c_file_path, 0),
            self.text(self.p.bp_c_file_name, 0)
        ))
    }

    /// The one mask drawing entry point (C's `readInt32Array` mask branch,
    /// mask_io.cpp:88-170, which mutates the *record's* buffer in place).
    ///
    /// The driver owns the mask here, so a re-read of the waveform cannot
    /// re-apply an operation and the record's buffer is never the source of
    /// truth.
    fn mask_op(&mut self) -> AsynResult<()> {
        let geom = self.geometry()?;
        let on = self.int(self.p.mask_on_off_pel, 0) != 0;
        let mut mask = self.shared.take_mask(geom.pixel_count());

        if self.int(self.p.mask_reset, 0) == 1 {
            mask::mask_reset(&geom, &mut mask, on);
        } else if self.int(self.p.mask_rectangle, 0) == 1 {
            mask::mask_rectangle(
                &geom,
                &mut mask,
                self.int(self.p.mask_min_x, 0),
                self.int(self.p.mask_size_x, 0),
                self.int(self.p.mask_min_y, 0),
                self.int(self.p.mask_size_y, 0),
                on,
            );
        } else if self.int(self.p.mask_circle, 0) == 1 {
            mask::mask_circle(
                &geom,
                &mut mask,
                self.int(self.p.mask_min_x, 0),
                self.int(self.p.mask_min_y, 0),
                self.int(self.p.mask_radius, 0),
                on,
            );
        } else if self.int(self.p.mask_pel, 0) == 1 {
            let bpc = self.read_bpc()?;
            mask::mask_from_bpc(&geom, &bpc, &mut mask);
        } else if self.int(self.p.mask_write, 0) == 1 {
            let mut bpc = self.read_bpc()?;
            mask::apply_mask_to_bpc(&geom, &mask, &mut bpc);
            self.write_mask_file(&bpc)?;
        }

        let n = mask.iter().filter(|&&v| v & 1 != 0).count();
        self.shared.put_mask(mask);
        self.ad.port_base.set_int32_param(
            self.p.bp_cmasked,
            0,
            i32::try_from(n).unwrap_or(i32::MAX),
        )?;
        Ok(())
    }

    /// C `readBPCfile` (mask_io.cpp:363).
    fn read_bpc(&mut self) -> AsynResult<Vec<u8>> {
        let path = self.bpc_path();
        let bpc = std::fs::read(&path).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: reading {}: {e}", path.display()),
        })?;
        let n = bpc.iter().filter(|&&b| b & 1 != 0).count();
        self.ad
            .port_base
            .set_int32_param(self.p.bp_cn, 0, i32::try_from(n).unwrap_or(i32::MAX))?;
        Ok(bpc)
    }

    /// C `writeBPCfile` (mask_io.cpp:430): write the edited BPC to the mask
    /// file name, then have Serval load *that* file.
    fn write_mask_file(&mut self, bpc: &[u8]) -> AsynResult<()> {
        let dir = self.text(self.p.bp_c_file_path, 0);
        let mut name = self.text(self.p.mask_file_name, 0);
        if name.is_empty() {
            name = "mask.bpc".to_string();
            self.ad
                .port_base
                .set_string_param(self.p.mask_file_name, 0, name.clone())?;
        }
        let path = std::path::PathBuf::from(format!("{dir}{name}"));
        std::fs::write(&path, bpc).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: writing {}: {e}", path.display()),
        })?;

        let url = serval::config_load("pixelconfig", &format!("{dir}{name}"));
        self.http.get(&url, TIMEOUT_CONFIG).map_err(|e| {
            let err = self.http_failed(
                "GET /config/load (pixelconfig)",
                &e,
                HttpPrint::Failure("uploadBPC GET /config/load pixelconfig", "GET", &url),
            );
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_BPC_FILE,
                        "WriteBPC: uploadBPC to SERVAL failed",
                    )
                ),
            );
            err
        })?;
        self.http_ok();
        Ok(())
    }

    /// C `refreshPixelConfigFromServal` (serval_http.cpp:1000).
    fn refresh_pixel_config(&mut self) -> AsynResult<()> {
        let geom = self.geometry()?;
        let bpc = self.read_bpc()?;
        let mut serval_linear = vec![0u8; geom.num_chips * PIXEL_CONFIG_BYTES];

        for chip in 0..geom.num_chips {
            let path = serval::chip_pixel_config(i32::try_from(chip).unwrap_or(0));
            let reply = self.http.get(&path, TIMEOUT_POLL).map_err(|e| {
                let err = self.http_failed(&format!("GET {path}"), &e, HttpPrint::None);
                self.ad.port_base.trace_print(
                    TraceMask::ERROR,
                    format_args!(
                        "{}",
                        adlog::err(
                            adlog::REFRESH_PIXEL_CONFIG,
                            &format!("chip {chip} PixelConfig GET failed: HTTP {}", e.code()),
                        )
                    ),
                );
                err
            })?;
            // Serval answers with a JSON string, or with the bare base64.
            let encoded = match serde_json::from_str::<serde_json::Value>(&reply) {
                Ok(v) => serval::json_to_string(&v),
                Err(_) => reply.trim().to_string(),
            };
            let addr = i32::try_from(chip).unwrap_or(0);
            let Some(decoded) = decode_base64(&encoded) else {
                self.ad
                    .port_base
                    .set_int32_param(self.p.pixel_config_match_bp_c, addr, -1)?;
                self.ad.port_base.set_string_param(
                    self.p.pixel_config_status,
                    addr,
                    "base64 decode failed",
                )?;
                continue;
            };

            let offset = chip * PIXEL_CONFIG_BYTES;
            let len = decoded.len();
            self.ad.port_base.set_int32_param(
                self.p.pixel_config_len,
                addr,
                i32::try_from(len).unwrap_or(i32::MAX),
            )?;

            let file_len = bpc.len().saturating_sub(offset).min(PIXEL_CONFIG_BYTES);
            let (match_code, mismatch, status) = if file_len == 0 {
                (
                    3,
                    0i64,
                    format!("BPC too small for chip (need offset {offset})"),
                )
            } else {
                let n = decoded
                    .iter()
                    .zip(&bpc[offset..offset + file_len])
                    .filter(|(a, b)| a != b)
                    .count() as i64;
                if n > 0 {
                    (0, n, format!("Mismatch {n} bytes"))
                } else if len != file_len {
                    (
                        3,
                        0,
                        format!("Length mismatch (decoded {len}, chip file {file_len})"),
                    )
                } else {
                    (1, 0, "OK, matches BPC".to_string())
                }
            };
            for (i, &b) in decoded.iter().take(PIXEL_CONFIG_BYTES).enumerate() {
                if let Some(slot) = serval_linear.get_mut(offset + i) {
                    *slot = b;
                }
            }
            self.ad
                .port_base
                .set_int32_param(self.p.pixel_config_match_bp_c, addr, match_code)?;
            self.ad.port_base.set_int64_param(
                self.p.pixel_config_mismatch_bytes,
                addr,
                mismatch,
            )?;
            self.ad
                .port_base
                .set_string_param(self.p.pixel_config_status, addr, status)?;
            self.ad.port_base.call_param_callbacks(addr)?;
        }

        let mut diff = vec![0i32; geom.pixel_count()];
        mask::pixel_config_diff(&geom, &serval_linear, &bpc, &mut diff);
        self.ad
            .port_base
            .params
            .set_int32_array(self.p.pixel_config_diff, 0, diff)?;

        let masked = mask::masked_pixels(&geom, &bpc);
        self.export_masked_pixels(&masked)?;
        Ok(())
    }

    /// C `exportMaskedPelsJsonFromBpcBuffer` (serval_http.cpp:900).
    fn export_masked_pixels(&mut self, masked: &[(usize, usize)]) -> AsynResult<()> {
        let path = self.text(self.p.masked_pels_json_path, 0);
        self.ad.port_base.set_int32_param(
            self.p.masked_pels_count,
            0,
            i32::try_from(masked.len()).unwrap_or(i32::MAX),
        )?;
        if path.is_empty() {
            self.ad
                .port_base
                .set_string_param(self.p.masked_pels_export_status, 0, "no path")?;
            return Ok(());
        }
        let body = json!({
            "MaskedPixels": masked
                .iter()
                .map(|&(x, y)| json!({ "x": x, "y": y }))
                .collect::<Vec<_>>(),
        });
        let status =
            match std::fs::write(&path, serde_json::to_vec_pretty(&body).unwrap_or_default()) {
                Ok(()) => format!("wrote {} pixels", masked.len()),
                Err(e) => format!("write failed: {e}"),
            };
        self.ad
            .port_base
            .set_string_param(self.p.masked_pels_export_status, 0, status)?;
        Ok(())
    }

    // -- Acquisition ---------------------------------------------------------

    /// C `publishAcquisitionPhase` (acquisition_runtime.cpp:35-64).
    fn publish_phase(&mut self, phase: Phase, message: &str) {
        let (acquire, status) = match phase {
            Phase::Starting | Phase::Running => (1, ADStatus::Acquire),
            Phase::Stopping => (0, ADStatus::Acquire),
            Phase::Fault => (0, ADStatus::Error),
            Phase::Idle => (0, ADStatus::Idle),
        };
        let ad = self.ad.params;
        let _ = self.ad.port_base.set_int32_param(ad.acquire, 0, acquire);
        let _ = self
            .ad
            .port_base
            .set_int32_param(ad.status, 0, status as i32);
        if !message.is_empty() {
            let _ = self.set_text(ad.status_message, message);
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
    }

    /// C `acquireStart` (acquire.cpp:313-535): every failure after the
    /// coordinator admits the start rolls back through [`Self::stop_acquisition`].
    fn acquire_start(&mut self) -> AsynResult<()> {
        let coordinator = &self.shared.acquisition;
        if !coordinator.begin_start() {
            let phase = coordinator.snapshot().phase;
            let message = format!("Acquisition start rejected while state is {}", phase.name());
            self.publish_phase(phase, &message);
            return Err(acquisition_error(message));
        }
        self.publish_phase(Phase::Starting, "Starting acquisition...");
        match self.start_measurement() {
            Ok(()) => {
                self.publish_phase(Phase::Running, "Acquisition running");
                Ok(())
            }
            Err(message) => {
                let _ = self.stop_acquisition(true, message);
                Err(acquisition_error(message))
            }
        }
    }

    /// The part of `acquireStart` that can fail; the error is the rollback
    /// reason.
    fn start_measurement(&mut self) -> Result<(), &'static str> {
        const FAILED: &str = "Failed to start acquisition";
        if self.mpx3_trigger_conflict(self.int(self.ad.params.trigger_mode, 0)) {
            self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!("{}", adlog::err(adlog::ACQUIRE_START, MPX3_TRIGGER_MSG)),
            );
            return Err(MPX3_TRIGGER_MSG);
        }

        // Serval re-binds the preview ports only once a prior measurement is
        // stopped; a 404 means there was none (acquire.cpp:348-381).
        let prior_stop = HttpPrint::Warning(
            "acquireStart stop prior measurement",
            "GET",
            serval::MEASUREMENT_STOP,
        );
        if let Err(e) = self.http.get(serval::MEASUREMENT_STOP, TIMEOUT_POLL)
            && e.code() != 404
        {
            self.log_http(&e, prior_stop);
        }
        if let Ok(measurement) = self.http.get_json(serval::MEASUREMENT, TIMEOUT_POLL)
            && serval::measurement_is_running(&measurement)
        {
            let status = measurement
                .pointer("/Info/Status")
                .and_then(serde_json::Value::as_str)
                .unwrap_or_default();
            self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::ACQUIRE_START,
                        &format!("Measurement is running (status: {status}), stopping it first"),
                    )
                ),
            );
            if let Err(e) = self.http.get(serval::MEASUREMENT_STOP, TIMEOUT_POLL) {
                self.log_http(&e, prior_stop);
            }
        }

        if !self
            .shared
            .acquisition
            .note_resource(Resource::RemoteMeasurement)
        {
            return Err("Acquisition start was cancelled");
        }
        if let Err(e) = self.http.get(serval::MEASUREMENT_START, TIMEOUT_CONFIG) {
            self.log_http(
                &e,
                HttpPrint::Failure(
                    "acquireStart GET /measurement/start",
                    "GET",
                    serval::MEASUREMENT_START,
                ),
            );
            return Err(FAILED);
        }
        self.sync_stream_endpoints();

        if self.cmd_tx.try_send(Command::AcquisitionStarted).is_err() {
            return Err("Failed to create measurement monitor thread");
        }
        let coordinator = &self.shared.acquisition;
        if !coordinator.note_resource(Resource::MonitorThread) || !coordinator.commit_start() {
            return Err("Acquisition start was cancelled before commit");
        }
        Ok(())
    }

    /// C `syncTcpStreamEndpoints` (acquire.cpp:201-223): the workers stream
    /// from the enabled channels' Base PVs as they stand at the start.
    fn sync_stream_endpoints(&mut self) {
        let p = self.p;
        let enabled = |this: &Self, write: usize, base: usize| {
            (this.int(write, 0) != 0).then(|| this.text(base, 0))
        };
        let prv_img = enabled(self, p.write_prv_img, p.prv_img_base);
        let img = enabled(self, p.write_img, p.img_base);
        let prv_hst = enabled(self, p.write_prv_hst, p.prv_hst_base);
        let mut checked = vec![];
        if prv_img.is_some() {
            checked.push(p.prv_img_base);
        }
        if self.int(p.write_prv_img1, 0) != 0 {
            checked.push(p.prv_img1_base);
        }
        if img.as_deref().is_some_and(|b| b.starts_with("tcp://")) {
            checked.push(p.img_base);
        }
        for base in checked {
            if let Some(channel) = CHANNEL_PATHS.iter().find(|c| (c.base)(&p) == base) {
                let _ = self.check_channel_path(channel);
            }
        }
        self.shared.set_stream_paths(prv_img, img, prv_hst);
    }

    /// C `acquireStop` (acquire.cpp:696-698).
    fn acquire_stop(&mut self) -> AsynResult<()> {
        self.stop_acquisition(false, "")
    }

    /// C `stopAcquisition` (acquisition_runtime.cpp:155-265): only the
    /// caller the coordinator hands the cleanup to talks to Serval.
    pub(crate) fn stop_acquisition(&mut self, fault: bool, reason: &str) -> AsynResult<()> {
        let claim = self.shared.acquisition.begin_stop(fault);
        if !claim.owner {
            return match self.shared.acquisition.snapshot().phase {
                Phase::Idle => {
                    self.publish_phase(Phase::Idle, "Acquisition stopped");
                    Ok(())
                }
                Phase::Stopping => Ok(()),
                phase => {
                    if phase == Phase::Fault && !reason.is_empty() {
                        self.publish_phase(Phase::Fault, reason);
                    }
                    Err(acquisition_error(format!(
                        "acquisition is {}",
                        phase.name()
                    )))
                }
            };
        }

        self.publish_phase(
            Phase::Stopping,
            if fault {
                "Rolling back acquisition..."
            } else {
                "Stopping acquisition..."
            },
        );

        // The monitor and the stream workers leave on the phase change.
        let p = self.p;
        let had_remote = claim.snapshot.has(Resource::RemoteMeasurement);
        let mut stop_succeeded = !had_remote;
        let mut remote_stopped = !had_remote;
        let mut measurement_valid = false;
        if had_remote {
            match self.http.get(serval::MEASUREMENT_STOP, TIMEOUT_POLL) {
                Ok(_) => stop_succeeded = true,
                Err(e) => {
                    self.log_http(
                        &e,
                        HttpPrint::Failure(
                            "transactional stop GET /measurement/stop",
                            "GET",
                            serval::MEASUREMENT_STOP,
                        ),
                    );
                }
            }
            match self.http.get_json(serval::MEASUREMENT, TIMEOUT_POLL) {
                Ok(m) => {
                    measurement_valid = true;
                    remote_stopped = !serval::measurement_is_running(&m);
                    for (reason, value) in tasks::measurement_updates(&self.p, &m) {
                        let _ = self.ad.port_base.set_param_value(reason, 0, value);
                    }
                }
                Err(e) => {
                    self.log_http(
                        &e,
                        HttpPrint::Failure(
                            "transactional stop GET /measurement",
                            "GET",
                            serval::MEASUREMENT,
                        ),
                    );
                }
            }
        }

        // C `resetAcquisitionRuntimeState` (acquisition_runtime.cpp:108-153).
        for rate in [p.prv_img_acq_rate, p.img_acq_rate, p.prv_hst_acq_rate] {
            let _ = self.ad.port_base.set_float64_param(rate, 0, 0.0);
        }
        self.shared.img.lock().reset();

        if self.shared.acquisition.finish_stop(remote_stopped, fault) == Phase::Fault {
            let mut message = if reason.is_empty() {
                "Acquisition cleanup failed".to_string()
            } else {
                reason.to_string()
            };
            if had_remote && !remote_stopped {
                message.push_str("; Serval measurement state not confirmed stopped");
            }
            self.publish_phase(Phase::Fault, &message);
            return Err(acquisition_error(message));
        }

        if !measurement_valid {
            // C `updateMeasurementStatusFromJson("DA_IDLE")`.
            let _ = self.set_text(self.p.status, "DA_IDLE");
            let _ = self
                .ad
                .port_base
                .set_int32_param(self.p.pipeline_state, 0, 3);
        }
        self.publish_phase(
            Phase::Idle,
            if stop_succeeded {
                "Acquisition stopped"
            } else {
                "Acquisition stopped (remote state reconciled)"
            },
        );
        self.ad.port_base.trace_print(
            TraceMask::FLOW,
            format_args!(
                "{}",
                adlog::log(adlog::STOP_ACQUISITION, "Stopping Image Acquisition")
            ),
        );
        Ok(())
    }
}

fn acquisition_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

impl ADDriver for TimePix3Driver {
    fn ad_base(&self) -> &ADDriverBase {
        &self.ad
    }

    fn ad_base_mut(&mut self) -> &mut ADDriverBase {
        &mut self.ad
    }
}

impl PortDriver for TimePix3Driver {
    fn base(&self) -> &PortDriverBase {
        &self.ad.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.ad.port_base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;

        if reason == self.p.bias_enable && !(value == 0 || value == 1) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("BiasEnabled must be 0 or 1 (received {value})"),
            });
        }
        // Acquire is published once the coordinator accepts the transition.
        if reason != self.ad.params.acquire {
            self.ad.port_base.set_int32_param(reason, addr, value)?;
        }

        let p = self.p;
        let result = if reason == self.ad.params.acquire {
            let snapshot = self.shared.acquisition.snapshot();
            let active = self.shared.acquisition.monitoring();
            let needs_stop = active || snapshot.resources != 0 || snapshot.phase == Phase::Stopping;
            println!(
                "ACQUIRE CHANGE: ADAcquire={value} (active={}), current ADStatus={}",
                i32::from(active),
                self.int(self.ad.params.status, 0)
            );
            let flow = |d: &mut Self, msg: &str| {
                d.ad.port_base.trace_print(
                    TraceMask::FLOW,
                    format_args!("{}", adlog::log(adlog::WRITE_INT32, msg)),
                )
            };
            if value != 0 && !active {
                flow(self, "Entering acquire start");
                self.acquire_start()
            } else if value == 0 && needs_stop {
                flow(self, "Entering acquire stop");
                self.acquire_stop()
            } else {
                Ok(())
            }
        } else if reason == p.vthreshold_fine {
            // Only the two thresholds are written; the other DACs are
            // readbacks (ADTimePix.cpp:540-546).
            self.write_dac(addr, "Vthreshold_fine", value)
        } else if reason == p.vthreshold_coarse {
            self.write_dac(addr, "Vthreshold_coarse", value)
        } else if reason == p.write_raw
            || reason == p.write_raw1
            || reason == p.write_img
            || reason == p.write_img1
            || reason == p.write_prv_img
            || reason == p.write_prv_img1
            || reason == p.write_prv_hst
        {
            self.get_server()
        } else if reason == p.refresh_connection {
            self.ad
                .port_base
                .set_int32_param(p.refresh_connection, 0, 0)?;
            if one_shot(value) {
                self.cmd_tx
                    .try_send(Command::RefreshConnection)
                    .map_err(|_| AsynError::Status {
                        status: AsynStatus::Error,
                        message: "timepix3: the connection task is not running".into(),
                    })
            } else {
                Ok(())
            }
        } else if reason == p.refresh_pixel_config {
            self.ad
                .port_base
                .set_int32_param(p.refresh_pixel_config, 0, 0)?;
            if addr == 0 && one_shot(value) {
                self.refresh_pixel_config()
            } else {
                Ok(())
            }
        } else if reason == p.apply_config {
            self.ad.port_base.set_int32_param(p.apply_config, 0, 0)?;
            if one_shot(value) {
                self.file_writer().and_then(|()| self.get_server())
            } else {
                Ok(())
            }
        } else if reason == p.write_bp_c_file || reason == p.write_da_cs_file {
            self.ad.port_base.set_int32_param(reason, addr, 0)?;
            if !one_shot(value) {
                Ok(())
            } else if reason == p.write_bp_c_file {
                self.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name)
            } else {
                self.config_load("dacs", p.da_cs_file_path, p.da_cs_file_name)
            }
        } else if reason == p.write_data {
            self.file_writer().and_then(|()| self.get_server())
        } else if reason == p.detector_orientation {
            let value = if (0..8).contains(&value) { value } else { 0 };
            self.ad
                .port_base
                .set_int32_param(p.detector_orientation, 0, value)?;
            self.rotate_layout(value)?;
            // The rotation changes Layout.Rotated.Chips (ADTimePix.cpp:635-640).
            self.cmd_tx
                .try_send(Command::RefreshDetector)
                .map_err(|_| AsynError::Status {
                    status: AsynStatus::Error,
                    message: "timepix3: the connection task is not running".into(),
                })
        } else if reason == p.bias_volt
            || reason == p.bias_enable
            || reason == p.trigger_in
            || reason == p.trigger_out
            || reason == p.log_level
            || reason == p.external_reference_clock
            || reason == p.chain_mode
            || reason == p.polarity
            || reason == p.periph_clk80
            || reason == p.tdc0
            || reason == p.tdc1
            || reason == p.gain_mode
            || reason == p.charge_summing
            || reason == p.colour
            || reason == p.pixel_depth
            || reason == p.counter_select_in
            || reason == p.counter_select_out
            || reason == p.i_delay0
            || reason == p.i_delay1
            || reason == p.i_delay2
            || reason == p.i_delay3
        {
            if reason == p.gain_mode && !(0..=3).contains(&value) {
                self.ad.port_base.set_int32_param(p.gain_mode, 0, 0)?;
            }
            let mut value = value;
            if reason == p.pixel_depth && ![1, 6, 12, 24].contains(&value) {
                value = 12;
                self.ad.port_base.set_int32_param(p.pixel_depth, 0, value)?;
            }
            if reason == p.pixel_depth && self.mpx3_pixel_depth_conflict(value) {
                // C returns before its closing callbacks and line; its
                // callParamCallbacks(ADTimePixPixelDepth) calls pass a
                // parameter index as the address and post nothing (UPSTREAM
                // DEFECT, ADTimePix.cpp:664-666), so the readbacks are posted
                // here as on every other path.
                self.log_write_int32(MPX3_PIXEL_DEPTH_MSG);
                self.ad.port_base.set_int32_param(p.pixel_depth, 0, 12)?;
                self.set_text(self.ad.params.status_message, MPX3_PIXEL_DEPTH_MSG)?;
                self.set_text(p.write_msg, MPX3_PIXEL_DEPTH_MSG)?;
                self.ad.port_base.set_int32_param(p.http_code, 0, 400)?;
                let _ = self.ad.port_base.call_param_callbacks(addr);
                self.ad.port_base.trace_print(
                    TraceMask::ERROR,
                    format_args!("{}", adlog::err(adlog::WRITE_INT32, MPX3_PIXEL_DEPTH_MSG)),
                );
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: MPX3_PIXEL_DEPTH_MSG.into(),
                });
            }
            self.init_acquisition()
        } else if reason == p.both_counters {
            if value != 0 {
                if self.mpx3_trigger_conflict(self.int(self.ad.params.trigger_mode, 0)) {
                    self.log_write_int32(
                        "MPX3 BothCounters: auto-switching TriggerMode 5 (CONTINUOUS) -> 4 (AUTOTRIGSTART_TIMERSTOP)",
                    );
                    self.ad
                        .port_base
                        .set_int32_param(self.ad.params.trigger_mode, 0, 4)?;
                }
                for ths in [p.prv_img_ths, p.prv_img1_ths, p.img_ths, p.img1_ths] {
                    self.set_text(ths, "0,1")?;
                }
                if self.mpx3_pixel_depth_conflict(self.int(p.pixel_depth, 0)) {
                    self.log_write_int32(MPX3_PIXEL_DEPTH_MSG);
                    self.ad.port_base.set_int32_param(p.pixel_depth, 0, 12)?;
                    self.set_text(self.ad.params.status_message, MPX3_PIXEL_DEPTH_MSG)?;
                }
            }
            self.init_acquisition()
        } else if reason == self.ad.params.num_images || reason == self.ad.params.trigger_mode {
            if reason == self.ad.params.trigger_mode
                && !(0..serval::TRIGGER_MODES.len() as i32).contains(&value)
            {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.trigger_mode, 0, 0)?;
            }
            let trigger_mode = self.int(self.ad.params.trigger_mode, 0);
            if reason == self.ad.params.trigger_mode && self.mpx3_trigger_conflict(trigger_mode) {
                self.log_write_int32(MPX3_TRIGGER_MSG);
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.trigger_mode, 0, 4)?;
                self.set_text(self.ad.params.status_message, MPX3_TRIGGER_MSG)?;
            }
            // Single image mode carries exactly one image (C ADTimePix.cpp:716-724).
            if reason == self.ad.params.num_images
                && self.int(self.ad.params.image_mode, 0) == 0
                && value != 1
            {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.num_images, 0, 1)?;
                self.log_write_int32(&format!(
                    "Ignoring NumImages={value} in single mode, forcing NumImages=1"
                ));
            }
            self.init_acquisition()
        } else if reason == self.ad.params.image_mode {
            if acquiring {
                let _ = self.acquire_stop();
            }
            if value == 0 {
                self.ad
                    .port_base
                    .set_int32_param(self.ad.params.num_images, 0, 1)?;
                self.init_acquisition()
            } else {
                Ok(())
            }
        } else if reason == p.stem_scan_width
            || reason == p.stem_scan_height
            || reason == p.stem_radius_outer
            || reason == p.stem_radius_inner
        {
            self.send_measurement_config()
        } else if let Some(&(_, low, high)) =
            ACCUMULATION_LIMITS.iter().find(|(f, ..)| f(&p) == reason)
        {
            // Each setting reads back clamped (ADTimePix.cpp:755-857).
            let value = value.clamp(low, high);
            self.ad.port_base.set_int32_param(reason, 0, value)?;
            if reason == p.img_frames_to_sum {
                self.shared.img.lock().set_frames_to_sum(value);
            } else if reason == p.img_sum_update_interval_frames {
                self.shared.img.lock().set_update_interval(value);
            } else if reason == p.prv_hst_frames_to_sum {
                self.shared.hst.lock().set_frames_to_sum(value);
            } else if reason == p.prv_hst_sum_update_interval {
                self.shared.hst.lock().set_update_interval(value);
            }
            Ok(())
        } else if reason == p.processed_img_output_type || reason == p.processed_hst_output_type {
            // ADTimePix.cpp:593-597, :605-609.
            if value != 0 && value != 1 {
                self.ad.port_base.set_int32_param(reason, 0, 0)?;
            }
            Ok(())
        } else if reason == p.img_image_data_reset {
            if value == 1 {
                self.shared.img.lock().reset();
                self.ad
                    .port_base
                    .set_int32_param(p.img_image_data_reset, 0, 0)?;
            }
            Ok(())
        } else if reason == p.prv_hst_data_reset {
            if value == 1 {
                self.shared.hst.lock().reset();
                self.ad
                    .port_base
                    .set_int32_param(p.prv_hst_data_reset, 0, 0)?;
            }
            Ok(())
        } else if reason == p.mask_reset
            || reason == p.mask_rectangle
            || reason == p.mask_circle
            || reason == p.mask_pel
            || reason == p.mask_write
        {
            let r = self.mask_op();
            // These are one-shot triggers, exactly as the db's autosave-free
            // bo records expect.
            self.ad.port_base.set_int32_param(reason, 0, 0)?;
            r
        } else if reason == p.health {
            self.cmd_tx
                .try_send(Command::RefreshStatus)
                .map_err(|_| AsynError::Status {
                    status: AsynStatus::Error,
                    message: "timepix3: the connection task is not running".into(),
                })
        } else if reason < p.http_code {
            // ADTIMEPIX_FIRST_PARAM is ADTimePixHttpCode (ADTimePix.h:467,
            // ADTimePix.cpp:860-863).
            ADDriver::ad_write_int32(self, user, value)
        } else {
            Ok(())
        };

        // The write's outcome is the record's, and it must survive the
        // callbacks (ADTimePix.cpp:866-873).
        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(e) => self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_INT32,
                        &format!(
                            "ERROR status={} function={reason}, value={value}",
                            e.status() as i32
                        ),
                    )
                ),
            ),
            Ok(()) => self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::WRITE_INT32,
                        &format!("function={reason} value={value}"),
                    )
                ),
            ),
        }
        result
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        // UPSTREAM DEFECT (ADTimePix.cpp:791): C's writeFloat64 calls
        // `setDoubleParam(function, value)` — the two-argument form, which
        // always writes address 0 — on a maxAddr=8 multi-device port, so every
        // per-chip float write lands on chip 0. The address is honoured here.
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;

        self.ad.port_base.set_float64_param(reason, addr, value)?;

        let p = self.p;
        let result = if reason == self.ad.params.acquire_time
            || reason == self.ad.params.acquire_period
            || reason == p.trigger_delay
            || reason == p.global_timestamp_interval
        {
            if acquiring {
                let _ = self.acquire_stop();
            }
            self.init_acquisition()
        } else if reason == p.stem_dwell_time || reason == p.tof_min || reason == p.tof_max {
            self.send_measurement_config()
        } else if reason < p.http_code {
            // ADDriver::writeFloat64 is asynPortDriver's (ADTimePix.cpp:1100-1104).
            self.ad.port_base.write_float64_param(user, value)
        } else {
            Ok(())
        };

        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(e) => self.ad.port_base.trace_print(
                TraceMask::ERROR,
                format_args!(
                    "{}",
                    adlog::err(
                        adlog::WRITE_FLOAT64,
                        &format!(
                            "ERROR status = {}, function ={reason}, value = {value:.6}",
                            e.status() as i32
                        ),
                    )
                ),
            ),
            Ok(()) => self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::WRITE_FLOAT64,
                        &format!("function={reason} value={value:.6}"),
                    )
                ),
            ),
        }
        result
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        let addr = user.addr;
        let p = self.p;
        // C stores the string up to its first NUL (ADTimePix.cpp:428-432).
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        let value = String::from_utf8_lossy(&data[..end]).into_owned();
        let name = self
            .ad
            .port_base
            .params
            .param_name(reason)
            .unwrap_or("")
            .to_owned();
        if [p.img_ths, p.img1_ths, p.prv_img_ths, p.prv_img1_ths].contains(&reason)
            && value.trim_start_matches([' ', '\t']).is_empty()
        {
            self.ad.port_base.trace_print(
                TraceMask::IO_DRIVER,
                format_args!(
                    "{}",
                    adlog::log(
                        adlog::WRITE_OCTET,
                        &format!("Ignoring empty threshold list write for paramName={name}"),
                    )
                ),
            );
            return Ok(data.len());
        }
        self.ad
            .port_base
            .set_string_param(reason, addr, value.clone())?;

        let status = if reason == p.bp_c_file_path {
            self.check_calibration_dir(
                p.bp_c_file_path,
                p.bp_c_file_path_exists,
                adlog::CHECK_BPC_PATH,
            )
        } else if reason == p.da_cs_file_path {
            self.check_calibration_dir(
                p.da_cs_file_path,
                p.da_cs_file_path_exists,
                adlog::CHECK_DACS_PATH,
            )
        } else if let Some(channel) = CHANNEL_PATHS.iter().find(|c| (c.base)(&p) == reason) {
            self.check_channel_path(channel)
        } else if reason == p.tof_tdc_reference {
            self.send_measurement_config()
        } else {
            Ok(())
        };
        // The callbacks run on failure too (ADTimePix.cpp:463-465).
        let callbacks = self.ad.port_base.call_param_callbacks(addr);
        if let Err(e) = status.and(callbacks) {
            let status = match e {
                AsynError::Status { status, .. } => status,
                _ => AsynStatus::Error,
            };
            return Err(AsynError::Status {
                status,
                message: format!(
                    "ADTimePix:{}: status={}, function={reason}, paramName={name}, value={value}",
                    adlog::WRITE_OCTET,
                    status as i32
                ),
            });
        }
        self.ad.port_base.trace_print(
            TraceMask::IO_DRIVER,
            format_args!(
                "{}",
                adlog::log(
                    adlog::WRITE_OCTET,
                    &format!("function={reason}, paramName={name}, value={value}"),
                )
            ),
        );
        Ok(data.len())
    }

    /// C `readEnum` (ADTimePix.cpp:979-1023): the orientation, trigger-mode
    /// and pipeline-state menus come from the driver, not the template.
    fn read_enum(&mut self, user: &AsynUser) -> AsynResult<(usize, Arc<[EnumEntry]>)> {
        let entry = |(value, s): (i32, &str)| EnumEntry {
            string: s.to_string(),
            value,
            severity: 0,
        };
        let entries: Arc<[EnumEntry]> = if user.reason == self.p.detector_orientation {
            (0..).zip(serval::ORIENTATIONS).map(entry).collect()
        } else if user.reason == self.ad.params.trigger_mode {
            (0..).zip(serval::TRIGGER_MODE_LABELS).map(entry).collect()
        } else if user.reason == self.p.pipeline_state {
            serval::PIPELINE_STATES.into_iter().map(entry).collect()
        } else {
            return self.ad.port_base.params.get_enum(user.reason, user.addr);
        };
        let index = usize::try_from(self.int(user.reason, user.addr)).unwrap_or(0);
        Ok((index, entries))
    }

    /// The mask waveform reads back what the driver drew (C `readInt32Array`,
    /// mask_io.cpp:18).
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let reason = user.reason;
        if reason == self.p.mask_bp_c {
            let mask = self.shared.mask.lock();
            let n = buf.len().min(mask.len());
            buf[..n].copy_from_slice(&mask[..n]);
            buf[n..].fill(0);
            return Ok(buf.len());
        }
        if reason == self.p.bp_c {
            let bpc = self.read_bpc()?;
            for (slot, &b) in buf.iter_mut().zip(bpc.iter()) {
                // Bit 8 marks a masked pixel for the calibration view
                // (mask_io.cpp:186).
                *slot = i32::from(b) | if b & 1 != 0 { 1 << 8 } else { 0 };
            }
            if bpc.len() < buf.len() {
                buf[bpc.len()..].fill(0);
            }
            return Ok(buf.len());
        }
        // C answers every other reason with nElements, zero-filled past the
        // data, including before the first value (mask_io.cpp:23-74, :205).
        let data = self
            .ad
            .port_base
            .params
            .get_int32_array(reason, user.addr)
            .unwrap_or_default();
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        buf[n..].fill(0);
        Ok(buf.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::driver::ShutterMode;

    /// The calibration root and its policy read back what the constructor
    /// configured (ADTimePix.cpp:1597-1609).
    #[test]
    fn the_calibration_policy_reads_back_its_root() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_CALIB_ROOT",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.configure_calibration("").unwrap();
        assert_eq!(det.text(p.calibration_root, 0), "/");
        assert_eq!(det.text(p.calibration_policy, 0), "Permissive: /");
        let tmp = std::env::temp_dir().canonicalize().unwrap();
        det.configure_calibration(tmp.to_str().unwrap()).unwrap();
        assert_eq!(
            det.text(p.calibration_policy, 0),
            format!("Enforced: {}", tmp.display())
        );
        det.configure_calibration("relative").unwrap();
        assert_eq!(det.text(p.calibration_root, 0), "");
        assert_eq!(
            det.text(p.calibration_policy, 0),
            "Blocked: calibration root is not absolute"
        );
    }

    /// The destination allowlist reads back at startup and refuses a
    /// channel Base before WriteData sends anything (ADTimePix.cpp:171-182,
    /// :1612-1625).
    #[test]
    fn the_destination_allowlist_refuses_a_channel_base() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_DEST_POLICY",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        assert_eq!(
            det.text(p.destination_allowlist, 0),
            "file:/*,tcp://*,http://*"
        );
        assert_eq!(det.text(p.destination_policy, 0), "Permissive");
        det.configure_destinations("tcp://localhost:8085").unwrap();
        assert_eq!(det.text(p.destination_policy, 0), "Enforced");
        det.ad
            .port_base
            .set_string_param(p.raw_base, 0, "file:/data")
            .unwrap();
        det.ad.port_base.set_int32_param(p.write_raw, 0, 1).unwrap();
        let e = det
            .write_int32(&mut AsynUser::new(p.write_data), 1)
            .unwrap_err();
        assert!(e.to_string().contains("Destination blocked"), "{e}");
        assert_eq!(
            det.text(p.write_msg, 0),
            "Destination blocked by startup allowlist: file:/data"
        );
        det.configure_destinations("tcp://*x").unwrap();
        assert_eq!(
            det.text(p.destination_policy, 0),
            "Blocked: destination allowlist contains an invalid entry"
        );
    }

    /// A channel Base PV reads back whether it names a reachable folder or
    /// a stream, and which (ADTimePix.cpp:217-398).
    #[test]
    fn a_channel_base_is_classified_and_checked() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_CHANNEL_PATH",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let tmp = std::env::temp_dir().canonicalize().unwrap();
        let mut write = |reason, value: String| {
            det.write_octet(&mut AsynUser::new(reason), value.as_bytes())
                .is_ok()
        };
        assert!(write(p.raw_base, format!("file:{}", tmp.display())));
        assert!(!write(p.raw1_base, "file://tmp".into()));
        assert!(write(p.prv_hst_base, "tcp://localhost:8085".into()));
        assert!(!write(p.img1_base, "ftp://somewhere".into()));
        assert!(!write(p.prv_img_base, "tcp://listen@host".into()));
        assert!(write(p.img_base, "http://localhost:8081".into()));
        assert_eq!(
            [
                (det.int(p.raw_stream, 0), det.int(p.raw_file_path_exists, 0)),
                (
                    det.int(p.raw1_stream, 0),
                    det.int(p.raw1_file_path_exists, 0)
                ),
                (
                    det.int(p.prv_hst_stream, 0),
                    det.int(p.prv_hst_file_path_exists, 0)
                ),
            ],
            [(0, 1), (3, 0), (2, 1)]
        );
        assert_eq!(det.int(p.img1_file_path_exists, 0), 0);
        assert_eq!(det.int(p.prv_img_file_path_exists, 0), 0);
        assert_eq!(det.int(p.img_file_path_exists, 0), 1);
        assert_eq!(det.text(p.raw_base, 0), format!("file:{}", tmp.display()));
    }

    /// A calibration directory gains its trailing '/' and is refused outside
    /// the calibration root (ADTimePix.cpp:185-212, mask_io.cpp:502-526).
    #[test]
    fn a_calibration_directory_is_checked_against_the_root() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_CALIB_DIR",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let tmp = std::env::temp_dir()
            .canonicalize()
            .unwrap()
            .join(format!("tpx-calib-dir-{}", std::process::id()));
        std::fs::create_dir_all(tmp.join("root/sub")).unwrap();
        std::fs::create_dir_all(tmp.join("outside")).unwrap();
        let sub = tmp.join("root/sub");
        for (path, exists) in [
            (p.bp_c_file_path, p.bp_c_file_path_exists),
            (p.da_cs_file_path, p.da_cs_file_path_exists),
        ] {
            det.write_octet(&mut AsynUser::new(path), sub.to_str().unwrap().as_bytes())
                .unwrap();
            assert_eq!(det.text(path, 0), format!("{}/", sub.display()));
            assert_eq!(det.int(exists, 0), 1);
        }
        det.calibration
            .configure(tmp.join("root").to_str().unwrap())
            .unwrap();
        let outside = format!("{}/", tmp.join("outside").display());
        let e = det
            .write_octet(&mut AsynUser::new(p.da_cs_file_path), outside.as_bytes())
            .unwrap_err();
        assert!(e.to_string().contains("status=3"), "{e}");
        assert_eq!(det.int(p.da_cs_file_path_exists, 0), 0);
        assert_eq!(
            det.text(p.write_msg, 0),
            "Calibration path blocked: calibration directory is outside the approved root"
        );
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// A failed writeOctet action reports C's errorMessage, not the
    /// action's own (ADTimePix.cpp:467-470).
    #[test]
    fn a_failed_write_octet_reports_the_write_octet_message() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_OCTET_FAIL",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let e = det
            .write_octet(&mut AsynUser::new(p.tof_tdc_reference), b"PN\0x")
            .unwrap_err();
        assert_eq!(
            e.to_string(),
            format!(
                "asyn: Error - ADTimePix:{}: status=3, function={}, paramName=TPX3_TOF_TDC_REFERENCE, value=PN",
                adlog::WRITE_OCTET,
                p.tof_tdc_reference
            )
        );
        assert_eq!(det.text(p.tof_tdc_reference, 0), "PN");
    }

    /// writeOctet stores up to the first NUL and ignores a blank threshold
    /// list (ADTimePix.cpp:424-440).
    #[test]
    fn a_blank_threshold_list_is_ignored_and_a_nul_ends_the_string() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_WRITE_OCTET",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.write_octet(&mut AsynUser::new(p.img_ths), b"1,2\0junk")
            .unwrap();
        assert_eq!(det.text(p.img_ths, 0), "1,2");
        let n = det
            .write_octet(&mut AsynUser::new(p.img_ths), b" \t")
            .unwrap();
        assert_eq!(n, 2);
        assert_eq!(det.text(p.img_ths, 0), "1,2");
    }

    /// A processed-output type other than 0 or 1 reads back 0
    /// (ADTimePix.cpp:593-597, :605-609).
    #[test]
    fn a_processed_output_type_outside_zero_and_one_reads_back_zero() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_OUTPUT_TYPE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for reason in [p.processed_img_output_type, p.processed_hst_output_type] {
            det.write_int32(&mut AsynUser::new(reason), 1).unwrap();
            assert_eq!(det.int(reason, 0), 1);
            det.write_int32(&mut AsynUser::new(reason), 7).unwrap();
            assert_eq!(det.int(reason, 0), 0);
        }
    }

    /// A Serval that answers `{}` to everything but a request line holding
    /// `fail`, which gets a 500.
    fn fake_serval(fail: &'static str) -> String {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let (mut request, mut length) = (String::new(), 0);
                let _ = reader.read_line(&mut request);
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                    let lower = line.to_ascii_lowercase();
                    if let Some(v) = lower.strip_prefix("content-length:") {
                        length = v.trim().parse().unwrap_or(0);
                    }
                }
                let mut body = vec![0; length];
                let _ = reader.read_exact(&mut body);
                let status = if request.contains(fail) {
                    "500 Internal Server Error"
                } else {
                    "200 OK"
                };
                let _ = reader.get_mut().write_all(
                    format!(
                        "HTTP/1.1 {status}\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{{}}"
                    )
                    .as_bytes(),
                );
            }
        });
        url
    }

    /// An acquisition that will not stop does not keep a timing write from
    /// reaching Serval (ADTimePix.cpp:533, :1095-1097).
    #[test]
    fn a_failed_stop_does_not_block_the_timing_write() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_STOP_FAIL",
            Arc::new(ServalHttp::new(&fake_serval("/measurement/stop"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let ad = det.ad.params;
        det.ad.port_base.set_int32_param(ad.acquire, 0, 1).unwrap();
        det.write_float64(&mut AsynUser::new(ad.acquire_time), 0.5)
            .unwrap();
        det.ad
            .port_base
            .set_int32_param(ad.num_images, 0, 7)
            .unwrap();
        det.write_int32(&mut AsynUser::new(ad.image_mode), 0)
            .unwrap();
        assert_eq!(det.int(ad.num_images, 0), 1);
    }

    fn status_of(det: &TimePix3Driver) -> (i32, i32, String) {
        let ad = det.ad.params;
        (
            det.int(ad.acquire, 0),
            det.int(ad.status, 0),
            det.text(ad.status_message, 0),
        )
    }

    /// A start Serval refuses rolls back to Fault with Acquire=0, and a stop
    /// returns to Idle (acquire.cpp:402-410, acquisition_runtime.cpp:35-64,
    /// :245-262).
    #[test]
    fn a_refused_start_rolls_back_and_a_stop_settles_idle() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_START_FAIL",
            Arc::new(ServalHttp::new(&fake_serval("/measurement/start"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let acquire = det.ad.params.acquire;
        assert!(det.write_int32(&mut AsynUser::new(acquire), 1).is_err());
        assert_eq!(
            status_of(&det),
            (
                0,
                ADStatus::Error as i32,
                "Failed to start acquisition".into()
            )
        );

        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_START_OK",
            Arc::new(ServalHttp::new(&fake_serval("\0"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        det.write_int32(&mut AsynUser::new(acquire), 1).unwrap();
        assert_eq!(
            status_of(&det),
            (1, ADStatus::Acquire as i32, "Acquisition running".into())
        );
        det.write_int32(&mut AsynUser::new(acquire), 0).unwrap();
        assert_eq!(
            status_of(&det),
            (0, ADStatus::Idle as i32, "Acquisition stopped".into())
        );
    }

    /// The monitor stops an acquisition Serval reports idle through the
    /// driver's stop transaction (acquire.cpp:680-683).
    #[test]
    fn the_monitor_stops_an_idle_measurement() {
        let runtime = crate::create_timepix3_detector(
            &epics_rs::asyn::registry::claim_port("TPX_MONITOR").unwrap(),
            &fake_serval("\0"),
            0,
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        let acquire = handle
            .with_driver_blocking::<TimePix3Driver, _>(|d| d.ad.params.acquire)
            .unwrap();
        handle.write_int32_blocking(acquire, 0, 1).unwrap();
        let idle = (0, ADStatus::Idle as i32, "Acquisition stopped".to_string());
        let mut seen = None;
        for _ in 0..500 {
            seen = handle
                .with_driver_blocking::<TimePix3Driver, _>(|d| status_of(d))
                .ok();
            if seen.as_ref() == Some(&idle) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(seen, Some(idle));
    }

    /// A successful rotation re-reads `/detector` (ADTimePix.cpp:630-641).
    #[test]
    fn a_rotation_rereads_the_detector() {
        let url = fake_serval("\0");
        let (cmd_tx, mut cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_ROTATE",
            Arc::new(ServalHttp::new(&url)),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.write_int32(&mut AsynUser::new(p.detector_orientation), 3)
            .unwrap();
        assert!(matches!(cmd_rx.try_recv(), Ok(Command::RefreshDetector)));
    }

    /// A DAC other than the two thresholds is stored without a Serval
    /// round trip (ADTimePix.cpp:540-546).
    #[test]
    fn only_the_threshold_dacs_reach_serval() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_DAC_DISPATCH",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.write_int32(&mut AsynUser::new(p.preamp_on), 77)
            .unwrap();
        assert_eq!(det.int(p.preamp_on, 0), 77);
        assert!(
            det.write_int32(&mut AsynUser::new(p.vthreshold_fine), 5)
                .is_err()
        );
    }

    /// BiasEnable takes 0 or 1 only; anything else is refused before the
    /// parameter is stored (ADTimePix.cpp:500-505).
    #[test]
    fn a_bias_enable_outside_zero_and_one_is_refused() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_BIAS_ENABLE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let e = det
            .write_int32(&mut AsynUser::new(p.bias_enable), 2)
            .unwrap_err();
        assert!(
            e.to_string()
                .contains("BiasEnabled must be 0 or 1 (received 2)"),
            "{e}"
        );
        assert_eq!(det.int(p.bias_enable, 0), 0);
    }

    /// A base-class float write runs asynPortDriver::writeFloat64, whose
    /// line precedes ADTimePix's (ADTimePix.cpp:1100-1112).
    #[test]
    fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_BASE_F64";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
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
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_float64_blocking(delay, 0, 2.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let base = format!(
            "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
        );
        let own = adlog::log(
            adlog::WRITE_FLOAT64,
            &format!("function={delay} value=2.000000"),
        );
        let at = |line: &str| {
            text.find(line)
                .unwrap_or_else(|| panic!("{line}: {text:?}"))
        };
        assert!(at(&base) < at(&own));
        std::mem::forget(runtime);
    }

    /// An Acquire write says which transition it enters (ADTimePix.cpp:519-526).
    #[test]
    fn an_acquire_write_traces_the_transition() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_ACQ_FLOW";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let acquire = det.ad.params.acquire;
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
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace().set_trace_mask(Some(name), TraceMask::FLOW);
        let _ = h.write_int32_blocking(acquire, 0, 1);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let line = adlog::log(adlog::WRITE_INT32, "Entering acquire start");
        assert!(text.contains(&line), "{text:?}");
        std::mem::forget(runtime);
    }

    /// A start stops any prior measurement first and warns when Serval
    /// refuses with anything but 404 (acquire.cpp:348-357).
    #[test]
    fn a_start_stops_a_prior_measurement_first() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_PRIOR_STOP";
        let url = fake_serval("/measurement/stop");
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&url)),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let acquire = det.ad.params.acquire;
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
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace().set_trace_mask(Some(name), TraceMask::WARNING);
        let _ = h.write_int32_blocking(acquire, 0, 1);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let line =
            format!("acquireStart stop prior measurement: GET {url}/measurement/stop HTTP 500");
        assert!(text.contains(&line), "{text:?}");
        std::mem::forget(runtime);
    }

    /// The stop transaction logs a refused stop without touching HttpCode
    /// (acquisition_runtime.cpp:198-208, serval_http.cpp:186-201).
    #[test]
    fn a_refused_stop_leaves_the_http_code() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_STOP_CODE",
            Arc::new(ServalHttp::new(&fake_serval("/measurement/stop"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let acquire = det.ad.params.acquire;
        det.write_int32(&mut AsynUser::new(acquire), 1).unwrap();
        det.ad
            .port_base
            .set_int32_param(det.p.http_code, 0, 7)
            .unwrap();
        det.write_int32(&mut AsynUser::new(acquire), 0).unwrap();
        assert_eq!(det.int(det.p.http_code, 0), 7);
        assert_eq!(
            det.text(det.ad.params.status_message, 0),
            "Acquisition stopped (remote state reconciled)"
        );
    }

    /// A start sends no configuration; the workers take the enabled
    /// channels' Base PVs (acquire.cpp:313-535, :201-223).
    #[test]
    fn a_start_sends_no_configuration_and_syncs_the_streams() {
        let shared = Arc::new(Shared::new());
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_START_SYNC",
            Arc::new(ServalHttp::new(&fake_serval("/server/destination"))),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.ad
            .port_base
            .set_int32_param(p.write_prv_img, 0, 1)
            .unwrap();
        det.set_text(p.prv_img_base, "tcp://listen@localhost:8089")
            .unwrap();
        det.write_int32(&mut AsynUser::new(det.ad.params.acquire), 1)
            .unwrap();
        let paths = shared.stream_paths();
        assert_eq!(
            paths.prv_img.as_deref(),
            Some("tcp://listen@localhost:8089")
        );
        assert_eq!(paths.img, None);
    }

    /// A stop zeroes the stream rates and the Img accumulation
    /// (acquisition_runtime.cpp:108-153, :211-212).
    #[test]
    fn a_stop_resets_the_acquisition_runtime_state() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_STOP_RESET",
            Arc::new(ServalHttp::new(&fake_serval("\0"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let (p, acquire) = (det.p, det.ad.params.acquire);
        det.write_int32(&mut AsynUser::new(acquire), 1).unwrap();
        for rate in [p.prv_img_acq_rate, p.img_acq_rate, p.prv_hst_acq_rate] {
            det.ad.port_base.set_float64_param(rate, 0, 5.0).unwrap();
        }
        det.shared.img.lock().add(&[1, 2, 3]);
        det.write_int32(&mut AsynUser::new(acquire), 0).unwrap();
        for rate in [p.prv_img_acq_rate, p.img_acq_rate, p.prv_hst_acq_rate] {
            assert_eq!(det.dbl(rate, 0), 0.0);
        }
        assert_eq!(det.shared.img.lock().frame_count(), 0);
    }

    /// An accumulation setting reads back clamped to C's range
    /// (ADTimePix.cpp:755-857).
    #[test]
    fn an_accumulation_setting_reads_back_clamped() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_ACCUM_CLAMP",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for (reason, write, read) in [
            (p.img_frames_to_sum, 0, 1),
            (p.img_frames_to_sum, 200_000, 100_000),
            (p.img_retention_limit_mb, 5000, 4096),
            (p.img_sum_update_interval_frames, 20_000, 10_000),
            (p.prv_hst_frames_to_sum, -3, 1),
            (p.prv_hst_retention_limit_mb, 0, 1),
            (p.prv_hst_sum_update_interval, 0, 1),
        ] {
            det.write_int32(&mut AsynUser::new(reason), write).unwrap();
            assert_eq!(det.int(reason, 0), read, "{reason} <- {write}");
        }
        assert_eq!(det.shared.img.lock().frames_to_sum(), 100_000);
    }

    /// A NumImages write in single mode is clamped to 1 and says so on
    /// ASYN_TRACEIO_DRIVER (ADTimePix.cpp:716-724).
    #[test]
    fn a_single_mode_num_images_write_logs_the_clamp() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_NUM_IMAGES_LOG";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let ad = det.ad.params;
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
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::IO_DRIVER);
        h.write_int32_blocking(ad.image_mode, 0, 0).ok();
        h.write_int32_blocking(ad.num_images, 0, 5).ok();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(h.read_int32_blocking(ad.num_images, 0).unwrap(), 1);
        assert!(
            text.contains("Ignoring NumImages=5 in single mode, forcing NumImages=1"),
            "{text:?}"
        );
        std::mem::forget(runtime);
    }

    /// An image frame read before the first frame, and a pixel-config diff
    /// shorter than the waveform, both come back as nElements zero-filled
    /// (mask_io.cpp:23-74).
    #[test]
    fn an_int32_array_read_fills_the_waveform() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_READ_ARRAY",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let mut buf = [7; 4];
        let n = det
            .read_int32_array(&AsynUser::new(p.img_image_frame), &mut buf)
            .unwrap();
        assert_eq!((n, buf), (4, [0; 4]));
        det.ad
            .port_base
            .params
            .set_int32_array(p.pixel_config_diff, 0, vec![5, 6])
            .unwrap();
        let mut buf = [7; 4];
        let n = det
            .read_int32_array(&AsynUser::new(p.pixel_config_diff), &mut buf)
            .unwrap();
        assert_eq!((n, buf), (4, [5, 6, 0, 0]));
    }

    /// A Medipix3 counting both counters refuses 24-bit depth and CONTINUOUS
    /// triggering (ADTimePix.cpp:658-671, :675-702, :709-714).
    #[test]
    fn medipix3_both_counters_refuses_24_bit_and_continuous() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Mpx3);
        let mut det = TimePix3Driver::new(
            "TPX_MPX3_BOTH",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let (trigger_mode, status_message) =
            (det.ad.params.trigger_mode, det.ad.params.status_message);
        det.ad
            .port_base
            .set_int32_param(trigger_mode, 0, 5)
            .unwrap();
        det.ad
            .port_base
            .set_int32_param(p.pixel_depth, 0, 24)
            .unwrap();
        let _ = det.write_int32(&mut AsynUser::new(p.both_counters), 1);
        assert_eq!(det.int(trigger_mode, 0), 4);
        assert_eq!(det.int(p.pixel_depth, 0), 12);
        assert_eq!(det.text(p.img1_ths, 0), "0,1");
        assert_eq!(det.text(status_message, 0), MPX3_PIXEL_DEPTH_MSG);

        let e = det
            .write_int32(&mut AsynUser::new(p.pixel_depth), 24)
            .unwrap_err();
        assert!(e.to_string().contains(MPX3_PIXEL_DEPTH_MSG));
        assert_eq!(det.int(p.pixel_depth, 0), 12);
        assert_eq!(det.int(p.http_code, 0), 400);
        assert_eq!(det.text(p.write_msg, 0), MPX3_PIXEL_DEPTH_MSG);

        let _ = det.write_int32(&mut AsynUser::new(trigger_mode), 5);
        assert_eq!(det.int(trigger_mode, 0), 4);
        assert_eq!(det.text(status_message, 0), MPX3_TRIGGER_MSG);
    }

    /// The trigger-mode menu is C's nine labels, FOLLOWING included, and the
    /// pipeline state carries -1 for Unknown (ADTimePix.cpp:900-926).
    #[test]
    fn the_driver_supplies_the_trigger_mode_and_pipeline_menus() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_READ_ENUM",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let (_, menu) = det
            .read_enum(&AsynUser::new(det.ad.params.trigger_mode))
            .unwrap();
        assert_eq!(menu.len(), 9);
        assert_eq!(menu[7].string, "SwReSt_SwReSp");
        assert_eq!((menu[8].string.as_str(), menu[8].value), ("FOLLOWING", 8));
        let (_, menu) = det.read_enum(&AsynUser::new(det.p.pipeline_state)).unwrap();
        assert_eq!((menu[4].string.as_str(), menu[4].value), ("Unknown", -1));
        let (_, menu) = det
            .read_enum(&AsynUser::new(det.p.detector_orientation))
            .unwrap();
        assert_eq!(menu[7].string, "LEFT_MIRRORED");
        assert!(det.read_enum(&AsynUser::new(det.p.gain_mode)).is_err());
    }

    /// An out-of-range GainMode, PixelDepth, TriggerMode or orientation is
    /// stored as C's default (ADTimePix.cpp:631-634, :649-657, :705-708).
    #[test]
    fn an_out_of_range_setting_reads_back_the_default() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_CLAMP",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let trigger_mode = det.ad.params.trigger_mode;
        for (reason, written, stored) in [
            (p.gain_mode, 4, 0),
            (p.pixel_depth, 8, 12),
            (trigger_mode, 9, 0),
            (p.detector_orientation, 8, 0),
        ] {
            let _ = det.write_int32(&mut AsynUser::new(reason), written);
            assert_eq!(det.int(reason, 0), stored, "reason {reason}");
        }
    }

    /// The Medipix3 detector settings go to Serval like the Timepix3 ones
    /// (ADTimePix.cpp:643-673); with no Serval behind the driver the write
    /// fails.
    #[test]
    fn a_medipix3_setting_reaches_init_acquisition() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MPX3_SETTING",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for reason in [p.gain_mode, p.pixel_depth, p.colour, p.i_delay3] {
            assert!(det.write_int32(&mut AsynUser::new(reason), 1).is_err());
        }
    }

    /// The one-shot actions run on 1 only and read back 0 whatever was
    /// written (one_shot_action.h, ADTimePix.cpp:563-584, :614-621); with no
    /// Serval behind the driver, running one fails the write.
    #[test]
    fn a_one_shot_action_runs_on_one_only() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_ONE_SHOT",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for reason in [
            p.apply_config,
            p.write_bp_c_file,
            p.write_da_cs_file,
            p.refresh_pixel_config,
        ] {
            det.write_int32(&mut AsynUser::new(reason), 2).unwrap();
            assert_eq!(det.int(reason, 0), 0);
        }
        assert!(
            det.write_int32(&mut AsynUser::new(p.apply_config), 1)
                .is_err()
        );
        assert_eq!(det.int(p.apply_config, 0), 0);
        assert!(
            det.write_int32(&mut AsynUser::new(p.write_bp_c_file), 1)
                .is_err()
        );
        assert_eq!(det.int(p.write_bp_c_file, 0), 0);
    }

    #[test]
    fn a_base_class_write_reaches_addriver() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_BASE_WRITE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.ad.params;
        det.ad
            .port_base
            .set_int32_param(p.shutter_mode, 0, ShutterMode::EpicsOnly as i32)
            .unwrap();
        det.write_int32(&mut AsynUser::new(p.shutter_control), 1)
            .unwrap();
        assert_eq!(
            det.ad
                .port_base
                .get_int32_param(p.shutter_control_epics, 0)
                .unwrap(),
            1,
            "ADShutterControl did not reach ADDriver::setShutter"
        );
    }
}
