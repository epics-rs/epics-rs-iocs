//! The ADTimePix3 asyn port driver (port of the `ADTimePix` class,
//! `ADTimePix.cpp` + `serval_http.cpp` + `acquire.cpp` + `mask_io.cpp`).

use std::path::Path;
use std::sync::Arc;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::driver::{ADDriver, ADDriverBase, ADStatus};
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::{EnumEntry, ParamValue};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use serde_json::{Value, json};

use crate::accum::{Accumulator, RangeState};
use crate::adlog;
use crate::bpc_file;
use crate::calibration::{PathAccess, PathPolicy};
use crate::coordinator::{Phase, Resource};
use crate::destination;
use crate::family::DetectorFamily;
use crate::http::{DEFAULT_TIMEOUT, HttpError, HttpResult, ServalHttp, TIMEOUT_5000_MS};
use crate::mask::{self, Geometry, PixelConfigError};
use crate::params::TimePixParams;
use crate::serval;
use crate::state::{Command, Mirrored, Shared, StreamEndpoints};
use crate::tasks::{self, Accum};
use crate::window::{Configuration, RollingSumStatus};

/// The line C prints when a Serval request fails: `logHttpFailure` or
/// `logHttpWarning` with its context, method and path, or nothing.
#[derive(Clone, Copy)]
enum HttpPrint<'a> {
    None,
    Failure(&'a str, &'a str, &'a str),
    Warning(&'a str, &'a str, &'a str),
}

/// C's `NDARRAY_MAX_ADDR` (ADTimePix.h:923, ADTimePix.cpp:1424-1429): 0 =
/// PrvImg, 1 = Img, 2 = Img running sum, 3 = Img sum-of-N, 4 = PrvHst
/// sum-of-N, 5 = PrvHst running sum, 6 = PrvHst frame, 7 = PrvHst ToF axis,
/// 8-9 = PrvImg threshold 1 and T0-T1 band, 10-12 = PrvImg1 threshold 0, 1
/// and band, 13 = Img threshold 1. The chip DACs and voltage rails bind the
/// low addresses of the same port.
pub const MAX_ADDR: usize = 14;

/// ADTIMEPIX_VERSION.REVISION.MODIFICATION (ADTimePix.h:20-22).
const DRIVER_VERSION: &str = "1.6.4";

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
    /// The channels whose `tcp://` path C parses into the stream target: the
    /// name and function its lines carry, and the target.
    tcp: Option<(&'static str, &'static str, EndpointOf)>,
}

type EndpointOf = fn(&mut StreamEndpoints) -> &mut Option<(String, u16)>;

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
        tcp: Some(("Img", adlog::CHECK_IMG_PATH, |e| &mut e.img)),
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
        tcp: Some(("PrvImg", adlog::CHECK_PRV_IMG_PATH, |e| &mut e.prv_img)),
    },
    ChannelPath {
        base: |p| p.prv_img1_base,
        stream: None,
        exists: |p| p.prv_img1_file_path_exists,
        error_message: "PrvImg1 file path must be file:/path_to_img_folder, http://localhost:8081, or tcp://listen@hostname:port",
        tcp: Some(("PrvImg1", adlog::CHECK_PRV_IMG1_PATH, |e| &mut e.prv_img1)),
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
/// C `NetworkClient::isTcpPortInUse` (network_client.cpp:177-198): a bind
/// with SO_REUSEADDR to the host's IPv4 address fails with EADDRINUSE.
fn tcp_port_in_use(host: &str, port: u32) -> bool {
    use std::net::ToSocketAddrs;
    let Ok(port) = u16::try_from(port) else {
        return false;
    };
    let Some(addr) = (host, port)
        .to_socket_addrs()
        .ok()
        .and_then(|mut addrs| addrs.find(|a| a.is_ipv4()))
    else {
        return false;
    };
    // std sets SO_REUSEADDR before binding on Unix.
    matches!(
        std::net::TcpListener::bind(addr),
        Err(e) if e.kind() == std::io::ErrorKind::AddrInUse
    )
}

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

/// C `kStreamCloseTimeout` (acquisition_runtime.cpp:24).
const STREAM_CLOSE_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(2000);

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

/// nlohmann 3.12's `json::parse("")` message, as the vendored json.hpp
/// prints it.
const NLOHMANN_EMPTY_INPUT: &str = "[json.exception.parse_error.101] parse error at line 1, \
     column 1: attempting to parse an empty input; check that your input string or stream \
     contains the expected JSON";

/// C `stripBpcExtForMaskedJson`: drop a trailing `.bpc`, any case.
fn strip_bpc_ext(name: &str) -> &str {
    match name.len().checked_sub(4) {
        Some(n) if name.is_char_boundary(n) && name[n..].eq_ignore_ascii_case(".bpc") => &name[..n],
        _ => name,
    }
}

/// C `utcIso8601Now`'s `%Y-%m-%dT%H:%M:%SZ` of `now`.
fn utc_iso8601(now: std::time::SystemTime) -> String {
    let secs = match now.duration_since(std::time::UNIX_EPOCH) {
        Ok(d) => d.as_secs() as i64,
        Err(e) => -(e.duration().as_secs() as i64),
    };
    let (days, rem) = (secs.div_euclid(86400), secs.rem_euclid(86400));
    // Howard Hinnant's civil_from_days.
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem / 60 % 60,
        rem % 60
    )
}

/// C `pixelConfigDiff_`'s fixed length (ADTimePix.cpp:1911).
const PIXEL_CONFIG_DIFF_LEN: usize = 262144;

enum LayoutEntryError {
    /// The nlohmann parse message.
    Json(String),
    Incomplete,
}

/// One Serval Layout chip entry: integer `Chip` equal to `chip`, integer
/// `X` and `Y`, and a string `Orientation`.
fn mpx3_layout_entry(layout: &str, chip: i32) -> Result<(i32, i32, String), LayoutEntryError> {
    let entry: Value = if layout.is_empty() {
        return Err(LayoutEntryError::Json(NLOHMANN_EMPTY_INPUT.to_string()));
    } else {
        crate::serval::parse_json(layout).map_err(|e| LayoutEntryError::Json(e.to_string()))?
    };
    let int = |key| match entry.get(key) {
        Some(Value::Number(n)) if !n.is_f64() => n
            .as_i64()
            .or_else(|| n.as_u64().map(|v| v as i64))
            .map(|v| v as i32),
        _ => None,
    };
    match (int("Chip"), int("X"), int("Y"), entry.get("Orientation")) {
        (Some(id), Some(x), Some(y), Some(Value::String(orientation))) if id == chip => {
            Ok((x, y, orientation.clone()))
        }
        _ => Err(LayoutEntryError::Incomplete),
    }
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
            max_memory,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
            InterfaceMask::UNMIGRATED_SUBCLASS,
            InterfaceMask::UNMIGRATED_SUBCLASS_INTERRUPTS,
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
        driver.mirror_worker_params();
        driver.configure_calibration(
            &std::env::var("ADTIMEPIX_CALIBRATION_ROOT").unwrap_or_default(),
        )?;
        driver.configure_destinations(
            &std::env::var("ADTIMEPIX_DESTINATION_ALLOWLIST").unwrap_or_default(),
        )?;
        Ok(driver)
    }

    /// Keep [`Shared::mirror`] equal to the address-0 values of what the
    /// stream workers read, from now on and from what is already stored.
    fn mirror_worker_params(&mut self) {
        let ad = self.ad.params;
        for which in Mirrored::ALL {
            let reason = match which {
                Mirrored::ArrayCallbacks => ad.base.array_callbacks,
                Mirrored::ArrayCounter => ad.base.array_counter,
                Mirrored::MaxSizeX => ad.max_size_x,
                Mirrored::MaxSizeY => ad.max_size_y,
                Mirrored::PixCount => self.p.pix_count,
                Mirrored::ImgAccumulationEnable => self.p.img_accumulation_enable,
                Mirrored::PrvHstAccumulationEnable => self.p.prv_hst_accumulation_enable,
            };
            let shared = self.shared.clone();
            self.ad.port_base.set_int32_rule(
                reason,
                Arc::new(move |_: &mut PortDriverBase, addr, value| {
                    if addr == 0 {
                        shared.mirror.store(which, value);
                    }
                    Ok(())
                }),
            );
            let value = self.ad.port_base.get_int32_param(reason, 0).unwrap_or(0);
            self.shared.mirror.store(which, value);
        }
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
        let message = adlog::snprintf(
            384,
            format!("Destination blocked by startup allowlist: {destination}"),
        );
        self.set_text(self.p.write_msg, &message)?;
        adlog::print(
            &self.ad.port_base,
            TraceMask::ERROR,
            &adlog::err(adlog::VALIDATE_DESTINATION, &message),
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
        let Some((name, func, endpoint)) = channel.tcp.filter(|_| path.starts_with("tcp://"))
        else {
            return Ok(());
        };
        let (mask, line) = match self.parse_tcp_path(&path) {
            Some((host, port)) => {
                let line = format!("Parsed {name} TCP path: host={host}, port={port}");
                *endpoint(&mut self.shared.endpoints.lock()) = Some((host, port));
                (TraceMask::IO_DRIVER, adlog::log(func, &line))
            }
            None => {
                self.ad.port_base.set_int32_param(exists, 0, 0)?;
                (
                    TraceMask::ERROR,
                    adlog::err(func, &format!("Failed to parse {name} TCP path: {path}")),
                )
            }
        };
        adlog::print(&self.ad.port_base, mask, &line);
        if mask == TraceMask::ERROR {
            return Err(refused());
        }
        Ok(())
    }

    /// C `parseTcpPath`, printing its ERR line for a refused port.
    fn parse_tcp_path(&mut self, path: &str) -> Option<(String, u16)> {
        match crate::stream::parse_tcp_path(path) {
            Ok(endpoint) => Some(endpoint),
            Err(line) => {
                if let Some(line) = line {
                    adlog::print(
                        &self.ad.port_base,
                        TraceMask::ERROR,
                        &adlog::err(adlog::PARSE_TCP_PATH, &line),
                    );
                }
                None
            }
        }
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
        adlog::print(
            &self.ad.port_base,
            TraceMask::ERROR,
            &adlog::err(func, &format!("{message} (directory=\"{normalized}\")")),
        );
        self.ad.port_base.call_param_callbacks(0)?;
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message,
        })
    }

    /// C `resolveCalibrationFile` (ADTimePix.cpp:153-169): a refused path
    /// is WriteMsg, an ERR line and no file access.
    fn resolve_calibration_file(
        &mut self,
        directory: &str,
        file_name: &str,
        access: PathAccess,
    ) -> AsynResult<std::path::PathBuf> {
        let e = match self.calibration.resolve(directory, file_name, access) {
            Ok(path) => return Ok(path),
            Err(e) => e,
        };
        let message = format!("Calibration path blocked: {}", e.message());
        self.set_text(self.p.write_msg, &message)?;
        adlog::print(
            &self.ad.port_base,
            TraceMask::ERROR,
            &adlog::err(
                adlog::RESOLVE_CALIBRATION_FILE,
                &format!("{message} (directory=\"{directory}\", name=\"{file_name}\")"),
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
        // C `calculateImgMemoryUsageMB()` before the first frame (ADTimePix.cpp:1545).
        base.set_float64_param(
            p.img_memory_usage,
            0,
            crate::tasks::img_memory_mb(&crate::accum::Accumulator::new()),
        )?;
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

    /// The parameter half of C `updatePrvHstRangeState` /
    /// `updateImgRangeState` (ADTimePix.cpp:1329-1332, :1344-1347).
    fn set_range(&mut self, saturated: usize, status: usize, range: RangeState) -> AsynResult<()> {
        self.ad
            .port_base
            .set_int32_param(saturated, 0, i32::from(range != RangeState::Ok))?;
        self.ad.port_base.set_string_param(status, 0, range.name())
    }

    /// The FramesToSum / RetentionLimitMB branches of C writeInt32
    /// (ADTimePix.cpp:755-845): reconfigure a window that has a geometry
    /// under the accumulator lock and set what `configure*RollingWindow`
    /// sets, or "Waiting for ... geometry" when none has arrived; an Img
    /// FramesToSum also hands out the new window sum.
    fn reconfigure_window(
        &mut self,
        which: Accum,
        publish_sum: bool,
        apply: impl FnOnce(&mut Accumulator) -> Option<Configuration>,
    ) -> AsynResult<()> {
        let p = self.p;
        let rate_samples = self.shared.rates.lock().prv_hst.sample_count();
        let (config, summed, memory, sum) = {
            let mut acc = which.get(&self.shared).lock();
            let config = apply(&mut acc);
            let ok = config.is_some_and(|c| c.status == RollingSumStatus::Ok);
            let summed = if ok { acc.buffered_frames() } else { 0 };
            let sum = (publish_sum && summed > 0).then(|| {
                acc.update_sum_on_next_frame();
                tasks::int64_array(acc.window().sum())
            });
            let memory = match which {
                Accum::Img => tasks::img_memory_mb(&acc),
                Accum::PrvHst => {
                    tasks::prv_hst_memory_mb(&acc, rate_samples, tasks::PRV_HST_LINE_BUFFER)
                }
            };
            (config, summed, memory, sum)
        };
        let (effective_param, summed_param, status_param, memory_param, sum_param, waiting) =
            match which {
                Accum::Img => (
                    p.img_effective_frames,
                    p.img_frames_summed,
                    p.img_retention_status,
                    p.img_memory_usage,
                    p.img_image_sum_n_frames,
                    "Waiting for image geometry",
                ),
                Accum::PrvHst => (
                    p.prv_hst_effective_frames,
                    p.prv_hst_frames_summed,
                    p.prv_hst_retention_status,
                    p.prv_hst_memory_usage,
                    p.prv_hst_histogram_sum_n_frames,
                    "Waiting for histogram geometry",
                ),
            };
        let (effective, status) = match config {
            None => (0, waiting.to_string()),
            Some(c) => {
                let report = which.window_report(&c);
                if let Some(warning) = &report.warning {
                    adlog::print(&self.ad.port_base, TraceMask::WARNING, warning);
                }
                (report.effective, report.status)
            }
        };
        let base = &mut self.ad.port_base;
        base.set_int32_param(
            effective_param,
            0,
            i32::try_from(effective).unwrap_or(i32::MAX),
        )?;
        base.set_int32_param(summed_param, 0, i32::try_from(summed).unwrap_or(i32::MAX))?;
        base.set_string_param(status_param, 0, &status)?;
        base.set_float64_param(memory_param, 0, memory)?;
        if let Some(sum) = sum {
            // C posts FramesToSum and MemoryUsage before the array (:779-783).
            base.call_param_callbacks(0)?;
            base.do_callbacks_array(sum_param, 0, ParamValue::Int64Array(sum.into()))?;
        }
        Ok(())
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
            adlog::print(&self.ad.port_base, mask, &line);
        }
    }

    fn http_ok(&mut self) {
        let _ = self.ad.port_base.set_int32_param(self.p.http_code, 0, 200);
    }

    /// C's `setStringParam(ADTimePixWriteMsg, r.text)` after a request whose
    /// reply the operator sees, accepted or not: empty when Serval never
    /// answered, as cpr's text is.
    fn write_msg_from(&mut self, reply: &HttpResult<String>) {
        let text = match reply {
            Ok(text) => text.as_str(),
            Err(e) => e.body(),
        };
        let _ = self
            .ad
            .port_base
            .set_string_param(self.p.write_msg, 0, text);
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
        adlog::print(
            &self.ad.port_base,
            TraceMask::IO_DRIVER,
            &adlog::log(adlog::WRITE_INT32, msg),
        );
    }

    fn set_text(&mut self, index: usize, text: &str) -> AsynResult<()> {
        self.ad.port_base.set_string_param(index, 0, text)
    }

    // -- Serval requests -----------------------------------------------------

    /// C `initAcquisition` (serval_http.cpp:2438-2660): read
    /// `/detector/config`, merge the driver's settings into it, PUT it back.
    fn init_acquisition(&mut self) -> AsynResult<()> {
        let p = self.p;
        let ad = self.ad.params;
        let refused = |what: &str, message: &str| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: {what} /detector/config: {message}"),
        };
        let current = match self.http.get(serval::DETECTOR_CONFIG, DEFAULT_TIMEOUT) {
            Ok(body) => body,
            Err(e) => {
                // serval_http.cpp:2449-2458.
                let base = &mut self.ad.port_base;
                base.set_int32_param(p.det_connected, 0, 0)?;
                base.set_int32_param(p.http_code, 0, e.code())?;
                base.set_string_param(p.write_msg, 0, e.body())?;
                base.set_string_param(
                    ad.status_message,
                    0,
                    "Failed to read detector configuration",
                )?;
                self.log_http(
                    &e,
                    HttpPrint::Failure(
                        "initAcquisition GET /detector/config",
                        "GET",
                        serval::DETECTOR_CONFIG,
                    ),
                );
                self.ad.port_base.call_param_callbacks(0)?;
                return Err(refused("GET", &e.to_string()));
            }
        };
        self.ad.port_base.set_int32_param(p.det_connected, 0, 1)?;
        let current = match serval::parse_detector_config_response(&current) {
            Ok(current) => current,
            Err(e) => {
                // serval_http.cpp:2466-2477.
                let message = format!("Invalid detector configuration response: {}", e.message());
                let base = &mut self.ad.port_base;
                adlog::print(
                    base,
                    TraceMask::ERROR,
                    &adlog::err(
                        adlog::INIT_ACQUISITION,
                        &format!("initAcquisition: {message}"),
                    ),
                );
                base.set_int32_param(p.http_code, 0, 200)?;
                base.set_string_param(p.write_msg, 0, &message)?;
                base.set_string_param(ad.status_message, 0, &message)?;
                base.call_param_callbacks(0)?;
                return Err(refused("GET", &message));
            }
        };
        // serval_http.cpp:2477-2479: the trigger and depth checks below need
        // the family.
        if self.shared.family() == DetectorFamily::Unknown {
            let _ = self.get_detector(true);
        }

        let mut trigger_mode = self.int(ad.trigger_mode, 0);
        if !(0..serval::TRIGGER_MODES.len() as i32).contains(&trigger_mode) {
            trigger_mode = 0;
            self.ad.port_base.set_int32_param(ad.trigger_mode, 0, 0)?;
        }
        if self.mpx3_trigger_conflict(trigger_mode) {
            // serval_http.cpp:2488-2493.
            let base = &mut self.ad.port_base;
            adlog::print(
                base,
                TraceMask::ERROR,
                &adlog::err(adlog::INIT_ACQUISITION, MPX3_TRIGGER_MSG),
            );
            base.set_string_param(p.write_msg, 0, MPX3_TRIGGER_MSG)?;
            base.set_int32_param(p.http_code, 0, 400)?;
            return Err(refused("PUT", MPX3_TRIGGER_MSG));
        }
        // serval_http.cpp:2596-2602: the depth sent is normalised, and 12
        // when counting both counters rules 24 out.
        let mpx3 = (self.shared.family() == DetectorFamily::Mpx3).then(|| {
            let depth = serval::pixel_depth_normalize(self.int(p.pixel_depth, 0));
            let conflict = self.mpx3_pixel_depth_conflict(depth);
            let config = serval::Mpx3Config {
                both_counters: self.int(p.both_counters, 0) != 0,
                charge_summing: self.int(p.charge_summing, 0) != 0,
                colour: self.int(p.colour, 0) != 0,
                gain_mode: self.int(p.gain_mode, 0),
                pixel_depth: if conflict { 12 } else { depth },
                counter_select_in: self.int(p.counter_select_in, 0),
                counter_select_out: self.int(p.counter_select_out, 0),
                idelay: [p.i_delay0, p.i_delay1, p.i_delay2, p.i_delay3].map(|r| self.int(r, 0)),
            };
            (conflict, config)
        });
        let cfg = serval::DetectorConfig {
            trigger_mode,
            exposure_time: self.dbl(ad.acquire_time, 0),
            trigger_period: self.dbl(ad.acquire_period, 0),
            trigger_delay: self.dbl(p.trigger_delay, 0),
            global_timestamp_interval: self.dbl(p.global_timestamp_interval, 0),
            n_triggers: self.int(ad.num_images, 0),
            bias_voltage: self.int(p.bias_volt, 0),
            bias_enabled: self.int(p.bias_enable, 0),
            chain_mode: self.int(p.chain_mode, 0),
            polarity: self.int(p.polarity, 0),
            trigger_in: self.int(p.trigger_in, 0),
            trigger_out: self.int(p.trigger_out, 0),
            log_level: self.int(p.log_level, 0),
            external_reference_clock: self.int(p.external_reference_clock, 0),
            periph_clk80: self.int(p.periph_clk80, 0),
            tdc0: self.int(p.tdc0, 0),
            tdc1: self.int(p.tdc1, 0),
            mpx3: mpx3.as_ref().map(|(_, m)| m.clone()),
        };
        let body = match serval::detector_config_body(&current, &cfg) {
            Ok(body) => body,
            Err(message) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(p.http_code, 0, 400)?;
                base.set_string_param(p.write_msg, 0, message)?;
                return Err(refused("PUT", message));
            }
        };
        if let Some((conflict, m)) = &mpx3 {
            self.ad
                .port_base
                .set_int32_param(p.pixel_depth, 0, m.pixel_depth)?;
            if *conflict {
                self.set_text(ad.status_message, MPX3_PIXEL_DEPTH_MSG)?;
            }
        }

        let reply = self.http.put_json(
            serval::DETECTOR_CONFIG,
            &serval::dump(&body, None, false),
            DEFAULT_TIMEOUT,
        );
        // serval_http.cpp:2626-2648.
        let code = reply.as_ref().map_or_else(HttpError::code, |_| 200);
        self.ad.port_base.set_int32_param(p.http_code, 0, code)?;
        self.write_msg_from(&reply);
        let result = match reply {
            Ok(_) => Ok(()),
            Err(e) => {
                self.set_text(ad.status_message, "Failed to upload detector configuration")?;
                self.log_http(
                    &e,
                    HttpPrint::Failure(
                        "initAcquisition PUT /detector/config",
                        "PUT",
                        serval::DETECTOR_CONFIG,
                    ),
                );
                if self.shared.family() == DetectorFamily::Mpx3 {
                    self.resync_mpx3_setpoints()?;
                }
                Err(refused("PUT", &e.to_string()))
            }
        };
        self.ad.port_base.call_param_callbacks(0)?;
        result
    }

    /// C re-reads PixelDepth and GainMode after a rejected Medipix3 PUT so the
    /// setpoints match Serval (serval_http.cpp:2632-2647).
    fn resync_mpx3_setpoints(&mut self) -> AsynResult<()> {
        let Ok(body) = self.http.get(serval::DETECTOR_CONFIG, DEFAULT_TIMEOUT) else {
            return Ok(());
        };
        let Ok(cfg) = crate::serval::parse_json(&body) else {
            return Ok(());
        };
        if let Some(depth) = cfg.get("PixelDepth") {
            self.ad.port_base.set_int32_param(
                self.p.pixel_depth,
                0,
                serval::pixel_depth_from_json(depth),
            )?;
        }
        if let Some(gain) = cfg.get("GainMode") {
            self.ad.port_base.set_int32_param(
                self.p.gain_mode,
                0,
                serval::gain_mode_from_string(&serval::json_string_or(gain)),
            )?;
        }
        Ok(())
    }

    /// C `sendMeasurementConfig` (serval_http.cpp:2234-2322): the PUT merges
    /// into a readback that must validate, and a rejected PUT re-reads it.
    fn send_measurement_config(&mut self) -> AsynResult<()> {
        let refused = |message: &str| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: GET /measurement/config: {message}"),
        };
        let body = match self
            .http
            .get_json(serval::MEASUREMENT_CONFIG, TIMEOUT_5000_MS)
        {
            Ok(body) => body,
            Err(e) => {
                let message = serval::ConfigResponseError::HttpFailure.message();
                self.set_text(self.p.write_msg, message)?;
                return Err(self.http_failed(
                    "GET /measurement/config",
                    &e,
                    HttpPrint::Failure(
                        "sendMeasurementConfig GET /measurement/config",
                        "GET",
                        serval::MEASUREMENT_CONFIG,
                    ),
                ));
            }
        };
        let current = match serval::parse_config_response(&body) {
            Ok(current) => serval::parse_config_readback(&body)
                .map(|_| current)
                .map_err(|e| e.message()),
            Err(e) => Err(e.message()),
        };
        let current = match current {
            Ok(current) => current,
            Err(message) => {
                self.http_ok();
                self.set_text(self.p.write_msg, message)?;
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(
                        adlog::SEND_MEASUREMENT_CONFIG,
                        &format!("sendMeasurementConfig GET /measurement/config: {message}"),
                    ),
                );
                return Err(refused(message));
            }
        };
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
        // WriteMsg is the PUT's reply body, accepted or not (serval_http.cpp:2308-2309).
        let e = match self.http.put_json(
            serval::MEASUREMENT_CONFIG,
            &serval::dump(&body, None, false),
            DEFAULT_TIMEOUT,
        ) {
            Ok(reply) => {
                self.http_ok();
                return self.set_text(self.p.write_msg, &reply);
            }
            Err(e) => e,
        };
        self.log_http(
            &e,
            HttpPrint::Failure(
                "sendMeasurementConfig PUT /measurement/config",
                "PUT",
                serval::MEASUREMENT_CONFIG,
            ),
        );
        if self.get_measurement_config().is_err() {
            adlog::print(
                &self.ad.port_base,
                TraceMask::WARNING,
                &adlog::warn(
                    adlog::SEND_MEASUREMENT_CONFIG,
                    "Failed to restore measurement configuration after rejected PUT",
                ),
            );
        }
        // The rejected PUT stays the operator-visible result.
        let refused = self.http_failed("PUT /measurement/config", &e, HttpPrint::None);
        self.set_text(self.p.write_msg, e.body())?;
        Err(refused)
    }

    fn reader(&self) -> crate::tasks::Reader<'_> {
        crate::tasks::Reader {
            http: &self.http,
            shared: &self.shared,
            p: self.p,
            ad: self.ad.params,
        }
    }

    /// A Serval read's steps on the port thread, in C order; `what` names the
    /// C function whose asynError a failed read is.
    fn apply_read(&mut self, read: crate::tasks::DetectorRead, what: &str) -> AsynResult<()> {
        let base = &mut self.ad.port_base;
        for step in &read.steps {
            match step {
                crate::tasks::DetectorStep::Trace(mask, line) => adlog::print(base, *mask, line),
                crate::tasks::DetectorStep::Post(addr, updates) => {
                    crate::tasks::apply_updates(base, *addr, updates);
                }
            }
        }
        if read.ok {
            Ok(())
        } else {
            Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("timepix3: {what} failed"),
            })
        }
    }

    /// C `getDetector` (serval_http.cpp:1293-1576) on the port thread.
    fn get_detector(&mut self, publish_http_status: bool) -> AsynResult<()> {
        let read = crate::tasks::read_detector(
            &self.http,
            &self.shared,
            self.p,
            self.ad.params,
            publish_http_status,
        );
        self.apply_read(read, "getDetector")
    }

    /// C `getMeasurementConfig` (serval_http.cpp:2198-2232).
    pub(crate) fn get_measurement_config(&mut self) -> AsynResult<()> {
        let reply = self
            .http
            .get_json(serval::MEASUREMENT_CONFIG, TIMEOUT_5000_MS);
        let snapshot = match &reply {
            Ok(body) => serval::parse_config_readback(body),
            Err(_) => Err(serval::ConfigReadbackError::HttpFailure),
        };
        let s = match snapshot {
            Ok(s) => s,
            Err(e) => {
                let (code, body) = match &reply {
                    Ok(body) => (200, body.as_str()),
                    Err(e) => (e.code(), e.body()),
                };
                let url = format!("{}{}", self.http.base_url(), serval::MEASUREMENT_CONFIG);
                for (mask, line) in [
                    (
                        TraceMask::IO_DRIVER,
                        adlog::log(
                            adlog::GET_MEASUREMENT_CONFIG,
                            &format!(
                                "GET {url} failed: {code} (Measurement.Config may not be supported): {}",
                                adlog::trim_body(body)
                            ),
                        ),
                    ),
                    (
                        TraceMask::ERROR,
                        adlog::err(
                            adlog::GET_MEASUREMENT_CONFIG,
                            &format!("getMeasurementConfig: {}", e.message()),
                        ),
                    ),
                ] {
                    adlog::print(&self.ad.port_base, mask, &line);
                }
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!("timepix3: GET /measurement/config: {}", e.message()),
                });
            }
        };
        let p = self.p;
        for (reason, v) in [
            (p.stem_scan_width, s.stem_scan_width),
            (p.stem_scan_height, s.stem_scan_height),
            (p.stem_radius_outer, s.stem_radius_outer),
            (p.stem_radius_inner, s.stem_radius_inner),
        ] {
            if let Some(v) = v {
                self.ad.port_base.set_int32_param(reason, 0, v)?;
            }
        }
        for (reason, v) in [
            (p.stem_dwell_time, s.stem_dwell_time),
            (p.tof_min, s.tof_min),
            (p.tof_max, s.tof_max),
        ] {
            if let Some(v) = v {
                self.ad.port_base.set_float64_param(reason, 0, v)?;
            }
        }
        if let Some(v) = &s.tof_tdc_reference {
            self.set_text(p.tof_tdc_reference, v)?;
        }
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    /// A builder's refusal of a channel setting.
    fn destination_refused(message: String) -> AsynError {
        AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: /server/destination: {message}"),
        }
    }

    /// A channel builder's `ERR_ARGS` refusal.
    fn configure_refused(&mut self, func: &str, message: String) -> AsynError {
        adlog::print(
            &self.ad.port_base,
            TraceMask::ERROR,
            &adlog::err(func, &message),
        );
        Self::destination_refused(message)
    }

    /// C `configureRawChannel` (serval_http.cpp:1755-1812).
    fn configure_raw_channel(&mut self, index: usize, server: &mut Value) -> AsynResult<()> {
        let p = self.p;
        let [enable, base, pat, stream, split, queue] = if index == 0 {
            [
                p.write_raw,
                p.raw_base,
                p.raw_file_pat,
                p.raw_stream,
                p.raw_split_strategy,
                p.raw_queue_size,
            ]
        } else {
            [
                p.write_raw1,
                p.raw1_base,
                p.raw1_file_pat,
                p.raw1_stream,
                p.raw1_split_strategy,
                p.raw1_queue_size,
            ]
        };
        if self.int(enable, 0) == 0 {
            return Ok(());
        }
        let c = serval::RawChannel {
            base: self.text(base, 0),
            file_pattern: self.text(pat, 0),
            stream: self.int(stream, 0),
            split_strategy: self.int(split, 0),
            queue_size: self.int(queue, 0),
        };
        self.validate_destination(&c.base)?;
        let body = serval::raw_body(&c)
            .map_err(|m| self.configure_refused(adlog::CONFIGURE_RAW_CHANNEL, m))?;
        *serval::channel_slot(&mut server["Raw"], index) = body;
        Ok(())
    }

    /// One image channel's settings from its parameters: Base, FilePattern,
    /// Format, Mode, IntegrationSize, IntegrationMode, StopOnDiskLimit,
    /// QueueSize.
    fn image_channel(
        &self,
        [base, pat, format, mode, size, int_mode, stop, queue]: [usize; 8],
    ) -> serval::ImageChannel {
        serval::ImageChannel {
            base: self.text(base, 0),
            file_pattern: self.text(pat, 0),
            format: self.int(format, 0),
            mode: self.int(mode, 0),
            integration_size: self.int(size, 0),
            integration_mode: self.int(int_mode, 0),
            stop_on_disk_limit: self.int(stop, 0),
            queue_size: self.int(queue, 0),
        }
    }

    /// C `configureImageChannel` (serval_http.cpp:1817-2058): `preview`
    /// selects `Preview.ImageChannels` over `Image`.
    fn configure_image_channel(
        &mut self,
        preview: bool,
        index: usize,
        server: &mut Value,
    ) -> AsynResult<()> {
        let p = self.p;
        let (enable, params) = match (preview, index) {
            (false, 0) => (
                p.write_img,
                [
                    p.img_base,
                    p.img_file_pat,
                    p.img_format,
                    p.img_mode,
                    p.img_int_size,
                    p.img_int_mode,
                    p.img_stp_on_dsk_lim,
                    p.img_queue_size,
                ],
            ),
            (false, _) => (
                p.write_img1,
                [
                    p.img1_base,
                    p.img1_file_pat,
                    p.img1_format,
                    p.img1_mode,
                    p.img1_int_size,
                    p.img1_int_mode,
                    p.img1_stp_on_dsk_lim,
                    p.img1_queue_size,
                ],
            ),
            (true, 0) => (
                p.write_prv_img,
                [
                    p.prv_img_base,
                    p.prv_img_file_pat,
                    p.prv_img_format,
                    p.prv_img_mode,
                    p.prv_img_int_size,
                    p.prv_img_int_mode,
                    p.prv_img_stp_on_dsk_lim,
                    p.prv_img_queue_size,
                ],
            ),
            (true, _) => (
                p.write_prv_img1,
                [
                    p.prv_img1_base,
                    p.prv_img1_file_pat,
                    p.prv_img1_format,
                    p.prv_img1_mode,
                    p.prv_img1_int_size,
                    p.prv_img1_int_mode,
                    p.prv_img1_stp_on_dsk_lim,
                    p.prv_img1_queue_size,
                ],
            ),
        };
        if self.int(enable, 0) == 0 {
            return Ok(());
        }
        let mut c = self.image_channel(params);
        self.validate_destination(&c.base)?;
        if let Some(format) = serval::tcp_format_override(&c.base, c.format) {
            let path = match (preview, index) {
                (false, 0) => "Image",
                (false, _) => "Image[1]",
                (true, 0) => "Preview",
                (true, _) => "Preview[1]",
            };
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::CONFIGURE_IMAGE_CHANNEL,
                    &format!(
                        "TCP channel {path}: format index {} invalid for tcp; using jsonimage (3)",
                        c.format
                    ),
                ),
            );
            self.ad.port_base.set_int32_param(params[2], 0, format)?;
            c.format = format;
        }
        let ths = match (preview, index) {
            (false, 0) => p.img_ths,
            (false, _) => p.img1_ths,
            (true, 0) => p.prv_img_ths,
            (true, _) => p.prv_img1_ths,
        };
        let ths_text = self.text(ths, 0);
        let thresholds = self
            .shared
            .family()
            .capabilities()
            .supports_image_thresholds
            .then_some((ths, ths_text.as_str()));
        let body = serval::image_body(&c, thresholds)
            .map_err(|m| self.configure_refused(adlog::CONFIGURE_IMAGE_CHANNEL, m))?;
        let channels = if preview {
            &mut server["Preview"]["ImageChannels"]
        } else {
            &mut server["Image"]
        };
        *serval::channel_slot(channels, index) = body;
        Ok(())
    }

    /// C `configureHistogramChannel` (serval_http.cpp:2085-2168).
    fn configure_histogram_channel(&mut self, server: &mut Value) -> AsynResult<()> {
        let p = self.p;
        if self.int(p.write_prv_hst, 0) == 0 {
            return Ok(());
        }
        let h = serval::HistogramChannel {
            image: self.image_channel([
                p.prv_hst_base,
                p.prv_hst_file_pat,
                p.prv_hst_format,
                p.prv_hst_mode,
                p.prv_hst_int_size,
                p.prv_hst_int_mode,
                p.prv_hst_stp_on_dsk_lim,
                p.prv_hst_queue_size,
            ]),
            number_of_bins: self.int(p.prv_hst_num_bins, 0),
            bin_width: self.dbl(p.prv_hst_bin_width, 0),
            offset: self.dbl(p.prv_hst_offset, 0),
        };
        self.validate_destination(&h.image.base)?;
        let body = serval::histogram_body(&h)
            .map_err(|m| self.configure_refused(adlog::CONFIGURE_HISTOGRAM_CHANNEL, m))?;
        *serval::channel_slot(&mut server["Preview"]["HistogramChannels"], 0) = body;
        Ok(())
    }

    /// C `configurePreviewSettings` (serval_http.cpp:2063-2080).
    fn configure_preview_settings(&mut self, server: &mut Value) -> AsynResult<()> {
        let p = self.p;
        server["Preview"]["Period"] = json!(self.dbl(p.prv_period, 0));
        let sampling_mode = self.int(p.prv_sampling_mode, 0);
        let Some(sampling) = serval::enum_name(&serval::SAMPLING_MODES, sampling_mode) else {
            return Err(self.configure_refused(
                adlog::CONFIGURE_PREVIEW_SETTINGS,
                format!("Invalid sampling mode index: {sampling_mode}"),
            ));
        };
        server["Preview"]["SamplingMode"] = json!(sampling);
        Ok(())
    }

    /// C `fileWriter` (serval_http.cpp:2333-2425): every channel in C's
    /// order, each at its own index, and nothing sent when none is enabled.
    fn file_writer(&mut self) -> AsynResult<()> {
        adlog::print(
            &self.ad.port_base,
            TraceMask::FLOW,
            &adlog::log(adlog::FILE_WRITER, "Configuring file writer channels"),
        );
        type Step = fn(&mut TimePix3Driver, &mut Value) -> AsynResult<()>;
        let steps: [(&str, Step); 7] = [
            ("Failed to configure raw channel 0", |d, s| {
                d.configure_raw_channel(0, s)
            }),
            ("Failed to configure raw channel 1", |d, s| {
                d.configure_raw_channel(1, s)
            }),
            ("Failed to configure image channel", |d, s| {
                d.configure_image_channel(false, 0, s)
            }),
            ("Failed to configure image channel 1", |d, s| {
                d.configure_image_channel(false, 1, s)
            }),
            ("Failed to configure preview image channel 0", |d, s| {
                d.configure_image_channel(true, 0, s)
            }),
            ("Failed to configure preview image channel 1", |d, s| {
                d.configure_image_channel(true, 1, s)
            }),
            ("Failed to configure histogram channel", |d, s| {
                d.configure_histogram_channel(s)
            }),
        ];
        let mut server = json!({});
        let preview: Step = |d, s| {
            if s.get("Preview").is_some() {
                d.configure_preview_settings(s)
            } else {
                Ok(())
            }
        };
        for (failed, step) in steps
            .into_iter()
            .chain([("Failed to configure preview settings", preview)])
        {
            if let Err(e) = step(self, &mut server) {
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(adlog::FILE_WRITER, failed),
                );
                return Err(e);
            }
        }
        if server.as_object().is_none_or(|o| o.is_empty()) {
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::FILE_WRITER,
                    "No channels are enabled. Skipping file writer configuration.",
                ),
            );
            return Ok(());
        }
        // C `sendConfiguration` (serval_http.cpp:2172-2190).
        println!("{}", send_configuration_line(&server));
        let reply = self.http.put_json(
            serval::SERVER_DESTINATION,
            &serval::dump(&server, None, false),
            DEFAULT_TIMEOUT,
        );
        // serval_http.cpp:2182-2183.
        self.write_msg_from(&reply);
        reply.map_err(|e| {
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

    /// C `getServer` (serval_http.cpp:1583-1653): an unset destination reads
    /// as no channels; any other refused reply leaves the readbacks alone.
    pub(crate) fn get_server(&mut self) -> AsynResult<()> {
        let reply = self.http.get(serval::SERVER_DESTINATION, TIMEOUT_5000_MS);
        let (code, body) = match &reply {
            Ok(body) => (200, body.as_str()),
            Err(e) => (e.code(), e.body()),
        };
        let snapshot = match serval::parse_destination_response(code, body) {
            Ok(s) => s,
            Err(serval::DestinationError::NotConfigured) => serval::DestinationSnapshot::default(),
            Err(e) => {
                let http_failure = e == serval::DestinationError::HttpFailure;
                let msg = if http_failure && !body.is_empty() {
                    body
                } else {
                    e.message()
                };
                let base = &mut self.ad.port_base;
                let _ = base.set_int32_param(self.p.http_code, 0, code);
                let _ = base.set_string_param(self.p.write_msg, 0, msg);
                let line = if http_failure {
                    adlog::http_failure_reply(
                        "getServer GET /server/destination",
                        "GET",
                        &format!("{}{}", self.http.base_url(), serval::SERVER_DESTINATION),
                        code,
                        body,
                    )
                } else {
                    adlog::err(adlog::GET_SERVER, &format!("getServer: {}", e.message()))
                };
                let base = &mut self.ad.port_base;
                adlog::print(base, TraceMask::ERROR, &line);
                base.call_param_callbacks(0)?;
                return Err(AsynError::Status {
                    status: AsynStatus::Error,
                    message: format!("timepix3: GET /server/destination: {}", e.message()),
                });
            }
        };
        let p = self.p;
        for (param, channels, n) in [
            (p.write_raw_read, snapshot.raw_channels, 1),
            (p.write_raw1_read, snapshot.raw_channels, 2),
            (p.write_img_read, snapshot.image_channels, 1),
            (p.write_img1_read, snapshot.image_channels, 2),
            (p.write_prv_img_read, snapshot.preview_image_channels, 1),
            (p.write_prv_img1_read, snapshot.preview_image_channels, 2),
            (p.write_prv_hst_read, snapshot.preview_histogram_channels, 1),
        ] {
            self.ad
                .port_base
                .set_int32_param(param, 0, i32::from(channels >= n))?;
        }
        self.ad.port_base.call_param_callbacks(0)
    }

    /// C `writeDac` (serval_http.cpp:589-660): a refused write puts the
    /// record back to the value Serval holds.
    fn write_dac(
        &mut self,
        chip: i32,
        param: usize,
        name: &str,
        value: i32,
        previous: i32,
    ) -> AsynResult<()> {
        if !(0..MAX_ADDR as i32).contains(&chip) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("timepix3: chip {chip} is outside 0..{}", MAX_ADDR - 1),
            });
        }
        let p = self.p;
        let path = serval::chip_dacs(chip);
        let refused = |what: &str, message: &str| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: writeDac {what} {path}: {message}"),
        };
        let body = match self.http.get(&path, DEFAULT_TIMEOUT) {
            Ok(body) => body,
            Err(e) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(param, chip, previous)?;
                base.set_int32_param(p.http_code, 0, e.code())?;
                let msg = match e.body() {
                    "" => "Failed to read DAC configuration",
                    body => body,
                };
                base.set_string_param(p.write_msg, 0, msg)?;
                let context = format!("writeDac GET chip {chip}");
                self.log_http(&e, HttpPrint::Failure(&context, "GET", &path));
                self.ad.port_base.call_param_callbacks(0)?;
                return Err(refused("GET", &e.to_string()));
            }
        };
        let (dacs, accepted) = match serval::prepare_dac_update(&body, name, value) {
            Ok(update) => update,
            Err(e) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(param, chip, previous)?;
                base.set_int32_param(p.http_code, 0, 200)?;
                base.set_string_param(p.write_msg, 0, e.message())?;
                adlog::print(
                    base,
                    TraceMask::ERROR,
                    &adlog::err(
                        adlog::WRITE_DAC,
                        &format!("writeDac GET chip {chip}: {}", e.message()),
                    ),
                );
                base.call_param_callbacks(0)?;
                return Err(refused("GET", e.message()));
            }
        };

        let reply = self
            .http
            .put_json(&path, &serval::dump(&dacs, Some(3), true), DEFAULT_TIMEOUT);
        let e = match reply {
            Ok(text) => {
                let base = &mut self.ad.port_base;
                base.set_int32_param(p.http_code, 0, 200)?;
                let msg = if text.is_empty() {
                    "Uploaded DAC configuration."
                } else {
                    &text
                };
                base.set_string_param(p.write_msg, 0, msg)?;
                return base.call_param_callbacks(0);
            }
            Err(e) => e,
        };
        let failed = match e.body() {
            "" => "Failed to upload DAC configuration".to_owned(),
            body => body.to_owned(),
        };
        self.log_http(&e, HttpPrint::Failure("writeDac PUT", "PUT", &path));
        // A rejected PUT leaves Serval's value; read it back, else keep the
        // value validated before the PUT.
        let restored = match self
            .http
            .get(&path, DEFAULT_TIMEOUT)
            .ok()
            .and_then(|body| serval::prepare_dac_update(&body, name, value).ok())
        {
            Some((_, refreshed)) => refreshed,
            None => {
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::WARNING,
                    &adlog::warn(
                        adlog::WRITE_DAC,
                        &format!(
                            "writeDac: failed to refresh {name} after rejected PUT; \
                                 using pre-PUT value"
                        ),
                    ),
                );
                accepted
            }
        };
        let base = &mut self.ad.port_base;
        base.set_int32_param(param, chip, restored)?;
        base.set_int32_param(p.http_code, 0, e.code())?;
        base.set_string_param(p.write_msg, 0, &failed)?;
        base.call_param_callbacks(0)?;
        Err(refused("PUT", &e.to_string()))
    }

    /// C `rotateLayout` (serval_http.cpp:476).
    fn rotate_layout(&mut self, orientation: i32) -> AsynResult<()> {
        let path = serval::layout_rotate(orientation).ok_or_else(|| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: orientation {orientation} is not one of the eight"),
        })?;
        self.http
            .get_auth_only(&path, DEFAULT_TIMEOUT)
            .map_err(|e| {
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

    /// C `uploadBPC` / `uploadDACS` (mask_io.cpp:534-563,
    /// serval_http.cpp:1668-1695): Serval loads the file from its own
    /// filesystem, so only the resolved path travels.
    fn config_load(&mut self, format: &str, path: usize, name: usize) -> AsynResult<()> {
        // mask_io.cpp:535-539: the BPC is checked against the geometry first.
        if format != "dacs" && self.read_bpc().is_none() {
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::UPLOAD_BPC,
                    "uploadBPC: local BPC validation failed; request not sent",
                ),
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "uploadBPC: local BPC validation failed; request not sent".into(),
            });
        }
        let (dir, file_name) = (self.text(path, 0), self.text(name, 0));
        let file = match self.resolve_calibration_file(&dir, &file_name, PathAccess::Read) {
            Ok(file) => file,
            Err(e) => {
                let (func, msg) = if format == "dacs" {
                    (
                        adlog::UPLOAD_DACS,
                        "uploadDACS: calibration path validation failed; request not sent",
                    )
                } else {
                    (
                        adlog::UPLOAD_BPC,
                        "uploadBPC: calibration path validation failed; request not sent",
                    )
                };
                adlog::print(&self.ad.port_base, TraceMask::ERROR, &adlog::err(func, msg));
                return Err(e);
            }
        };
        let url = serval::config_load(format, &file.to_string_lossy());
        let reply = self.http.get_auth_only(&url, DEFAULT_TIMEOUT);
        let (code, text) = match &reply {
            Ok(text) => (200, text.as_str()),
            Err(e) => (e.code(), e.body()),
        };
        if let Err(e) = &reply {
            let context = if format == "dacs" {
                "uploadDACS GET /config/load dacs"
            } else {
                "uploadBPC GET /config/load pixelconfig"
            };
            self.log_http(e, HttpPrint::Failure(context, "GET", &url));
        }
        if format == "dacs" {
            // serval_http.cpp:1688-1689.
            println!("\nuploadDACS: http_code = {code}");
            println!("Text dacs_file: {text}");
        } else {
            // mask_io.cpp:557.
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::UPLOAD_BPC,
                    &format!("uploadBPC: http_code={code} file={}", file.display()),
                ),
            );
        }
        // serval_http.cpp:1690-1691, mask_io.cpp:559-560.
        self.ad
            .port_base
            .set_int32_param(self.p.http_code, 0, code)?;
        self.write_msg_from(&reply);
        reply.map(drop).map_err(|e| AsynError::Status {
            status: AsynStatus::Error,
            message: format!("timepix3: GET /config/load ({format}): {e}"),
        })
    }

    // -- Mask ----------------------------------------------------------------

    /// C `rowsCols` (mask_io.cpp:251-313), with its WriteMsg and WARN lines.
    fn rows_cols(&mut self) -> Option<Geometry> {
        let p = self.p;
        let boards = [self.text(p.boards_id, 0), self.text(p.boards2_id, 0)];
        let family = self.shared.family();
        let layout = mask::Layout {
            pix_count: self.int(p.pix_count, 0),
            row_len: self.int(p.row_len, 0),
            num_chips: self.int(p.number_of_chips, 0),
            num_rows: self.int(p.number_of_rows, 0),
            tpx3: family == DetectorFamily::Tpx3,
            boards: [&boards[0], &boards[1]],
        };
        let (write_msg, warning) = match mask::rows_cols(&layout) {
            Ok(geom) => return Some(geom),
            Err(mask::RowsColsError::NoRowLength) => return None,
            Err(mask::RowsColsError::EightChipLayout) => (
                "Unsupported eight-chip mask layout: require two TPX3 quad boards".to_string(),
                format!(
                    "rowsCols: unsupported eight-chip mask layout (family={} board0={} board1={})",
                    family.name(),
                    boards[0],
                    boards[1]
                ),
            ),
            Err(mask::RowsColsError::Geometry(e)) => (
                e.message().to_string(),
                format!(
                    "rowsCols: {} (pixels={} rowLength={} chips={} rows={})",
                    e.message(),
                    layout.pix_count,
                    layout.row_len,
                    layout.num_chips,
                    layout.num_rows
                ),
            ),
        };
        let _ = self.set_text(p.write_msg, &write_msg);
        adlog::print(
            &self.ad.port_base,
            TraceMask::WARNING,
            &adlog::warn(adlog::ROWS_COLS, &warning),
        );
        None
    }

    /// C `pelIndex` for every image pixel, row-major, printing its WARN
    /// lines as C's per-pixel calls do.
    fn pel_table(&mut self, geom: &Geometry) -> Vec<i32> {
        let chips = self.int(self.p.number_of_chips, 0);
        let orientation = self.int(self.p.detector_orientation, 0);
        let mut table = Vec::with_capacity(geom.pixel_count());
        for j in 0..geom.rows {
            for i in 0..geom.cols {
                let (index, warning) = mask::pel_index(geom, chips, orientation, i, j);
                if let Some(warning) = warning {
                    adlog::print(
                        &self.ad.port_base,
                        TraceMask::WARNING,
                        &adlog::warn(adlog::PEL_INDEX, &warning),
                    );
                }
                table.push(index);
            }
        }
        table
    }

    /// C `bpcImageByteOffsets` (mask_io.cpp:318-422): the slice-0 BPC byte
    /// of every image pixel, through `pelIndex`, or through the per-chip
    /// Layout on an MPX3.
    fn bpc_image_byte_offsets(&mut self) -> Option<Vec<Option<usize>>> {
        let p = self.p;
        let g = self.rows_cols().unwrap_or_default();
        let refuse = |d: &mut Self, msg: &str| {
            let _ = d.set_text(p.write_msg, msg);
            let _ = d.ad.port_base.call_param_callbacks(0);
            None
        };
        if g.rows <= 0 || g.cols <= 0 || g.pel_width <= 0 {
            return refuse(self, "Invalid BPC image geometry");
        }
        let (rows, cols, width) = (g.rows as usize, g.cols as usize, g.pel_width as usize);
        let mut offsets = vec![None; rows * cols];
        let per_chip = width * width;
        let family = self.shared.family();
        let caps = family.capabilities();
        let store = |offsets: &mut [Option<usize>], image: usize, logical: usize| {
            let byte = mask::selected_slice_index(
                logical,
                per_chip,
                caps.bpc_bytes_per_pel,
                caps.bpc_threshold_slices,
                0,
            );
            match (offsets.get_mut(image), byte) {
                (Some(slot @ None), Some(byte)) => {
                    *slot = Some(byte);
                    true
                }
                _ => false,
            }
        };

        if family != DetectorFamily::Mpx3 {
            let chips = self.int(p.number_of_chips, 0);
            let orientation = self.int(p.detector_orientation, 0);
            for y in 0..g.rows {
                for x in 0..g.cols {
                    let (logical, warning) = mask::pel_index(&g, chips, orientation, x, y);
                    if let Some(warning) = warning {
                        adlog::print(
                            &self.ad.port_base,
                            TraceMask::WARNING,
                            &adlog::warn(adlog::PEL_INDEX, &warning),
                        );
                    }
                    let image = y as usize * cols + x as usize;
                    if !usize::try_from(logical).is_ok_and(|l| store(&mut offsets, image, l)) {
                        return refuse(self, "BPC image mapping is incomplete");
                    }
                }
            }
            return Some(offsets);
        }

        for chip in 0..self.int(p.number_of_chips, 0) {
            let layout = self.text(p.layout, chip);
            let mapped = (|| {
                let (ox, oy, orientation) =
                    mpx3_layout_entry(&layout, chip).map_err(|e| match e {
                        LayoutEntryError::Json(e) => e,
                        LayoutEntryError::Incomplete => "incomplete chip layout".to_string(),
                    })?;
                for ly in 0..g.pel_width {
                    for lx in 0..g.pel_width {
                        let (ix, iy) = mask::mpx3_layout_coordinates(
                            (lx, ly),
                            g.pel_width,
                            (ox, oy),
                            g.rows,
                            &orientation,
                        )
                        .filter(|&(ix, iy)| (0..g.cols).contains(&ix) && (0..g.rows).contains(&iy))
                        .ok_or("unsupported or out-of-range chip layout")?;
                        let logical = chip as usize * per_chip + ly as usize * width + lx as usize;
                        if !store(&mut offsets, iy as usize * cols + ix as usize, logical) {
                            return Err("duplicate chip layout coordinate".to_string());
                        }
                    }
                }
                Ok(())
            })();
            if let Err(e) = mapped {
                let msg = format!("MPX3 chip {chip} layout unavailable: {e}");
                let _ = self.set_text(p.write_msg, &msg);
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(adlog::BPC_IMAGE_BYTE_OFFSETS, &msg),
                );
                let _ = self.ad.port_base.call_param_callbacks(0);
                return None;
            }
        }
        if offsets.contains(&None) {
            return refuse(self, "MPX3 chip layout does not cover the full image");
        }
        Some(offsets)
    }

    /// C `readInt32Array`'s MaskBPC branch (mask_io.cpp:95-207, :240-242): the
    /// db sets one operation flag and processes the waveform, so the operation
    /// runs on the waveform read, over its `nElements`.
    fn mask_bp_c(&mut self, mask: &mut [i32]) -> AsynResult<()> {
        let p = self.p;
        let flag = |s: &Self, reason| s.int(reason, 0) == 1;
        let on_off = self.int(p.mask_on_off_pel, 0);
        let (x, y) = (self.int(p.mask_min_x, 0), self.int(p.mask_min_y, 0));
        let n = mask.len();
        let drawn = if flag(self, p.mask_reset) {
            self.mask_flow(&format!("MaskBPC: reset (waveform nElements={n})"));
            let image = self.mask_image();
            mask::reset(mask, image, on_off)
        } else if flag(self, p.mask_rectangle) {
            self.mask_flow(&format!("MaskBPC: rectangle (waveform nElements={n})"));
            let (sx, sy) = (self.int(p.mask_size_x, 0), self.int(p.mask_size_y, 0));
            let image = self.mask_image();
            mask::rectangle(mask, image, x, sx, y, sy, on_off != 0)
        } else if flag(self, p.mask_circle) {
            self.mask_flow(&format!("MaskBPC: circle (waveform nElements={n})"));
            let radius = self.int(p.mask_radius, 0);
            let image = self.mask_image();
            mask::circle(mask, image, x, y, radius, on_off != 0)
        } else if flag(self, p.mask_pel) || flag(self, p.mask_write) {
            let write = !flag(self, p.mask_pel);
            let family = self.shared.family();
            if !mask::operator_mask_supported(family) {
                let (what, op) = if write {
                    ("write blocked", "write")
                } else {
                    ("read unavailable", "read")
                };
                let msg = format!(
                    "{} mask {what}: pixel mask encoding is not documented",
                    family.name()
                );
                self.set_text(p.write_msg, &msg)?;
                self.ad
                    .port_base
                    .set_param_status(p.mask_bp_c, 0, AsynStatus::Error)?;
                self.ad.port_base.call_param_callbacks(0)?;
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(
                        adlog::READ_INT32_ARRAY,
                        &format!("MaskBPC: refusing unsupported detector-family mask {op}"),
                    ),
                );
                return Err(Self::read_array_refused());
            }
            let mut bpc = self.read_bpc().ok_or_else(Self::read_array_refused)?;
            let offsets = self
                .bpc_image_byte_offsets()
                .ok_or_else(Self::read_array_refused)?;
            if write {
                mask::apply_mask_to_bpc(family, &offsets, mask, &mut bpc);
                self.write_bpc_file(&bpc)
                    .ok_or_else(Self::read_array_refused)?;
            } else {
                mask::mask_from_bpc(family, &offsets, &bpc, mask);
            }
            Ok(())
        } else {
            self.mask_flow(&format!("MaskBPC: no draw op (nElements={n})"));
            Ok(())
        };
        let status = match drawn {
            Ok(()) => AsynStatus::Success,
            Err(e) => {
                self.set_text(p.write_msg, e.message())?;
                AsynStatus::Error
            }
        };
        self.ad.port_base.set_param_status(p.mask_bp_c, 0, status)?;
        self.ad.port_base.call_param_callbacks(0)?;
        match status {
            AsynStatus::Success => Ok(()),
            _ => Err(Self::read_array_refused()),
        }
    }

    /// A refused readInt32Array carries the message ADDriver::readInt32Array
    /// left in the asynUser (asynPortDriver.cpp:1815-1816).
    fn read_array_refused() -> AsynError {
        AsynError::Status {
            status: AsynStatus::Error,
            message: "asynPortDriver:readArray not implemented".into(),
        }
    }

    /// The (COLS, ROWS) C's mask helpers draw in; zeros when `rowsCols`
    /// fails, whose status C ignores (mask_io.cpp:424-466).
    fn mask_image(&mut self) -> (i32, i32) {
        let g = self.rows_cols().unwrap_or_default();
        (g.cols, g.rows)
    }

    fn mask_flow(&self, msg: &str) {
        adlog::print(
            &self.ad.port_base,
            TraceMask::FLOW,
            &adlog::log(adlog::READ_INT32_ARRAY, msg),
        );
    }

    /// C `expectedBPCSize` (mask_io.cpp:565-584).
    fn expected_bpc_size(&mut self) -> Option<usize> {
        let pixels = self.int(self.p.pix_count, 0);
        let caps = self.shared.family().capabilities();
        let size =
            bpc_file::expected_size(pixels, caps.bpc_bytes_per_pel, caps.bpc_threshold_slices);
        if size.is_none() {
            let message = format!(
                "Invalid BPC geometry: pixels={pixels} bytes/pixel={} slices={}",
                caps.bpc_bytes_per_pel, caps.bpc_threshold_slices
            );
            self.refuse_bpc(adlog::EXPECTED_BPC_SIZE, adlog::snprintf(192, message));
        }
        size
    }

    /// C's WriteMsg, `ERR_ARGS("%s")` and `callParamCallbacks` on a refused
    /// BPC file.
    fn refuse_bpc(&mut self, func: &str, message: String) {
        let _ = self.set_text(self.p.write_msg, &message);
        adlog::print(
            &self.ad.port_base,
            TraceMask::ERROR,
            &adlog::err(func, &message),
        );
        let _ = self.ad.port_base.call_param_callbacks(0);
    }

    /// C `readBPCfile` (mask_io.cpp:586-634): the file at the geometry's
    /// size, and BPCn its masked count (-1 for a family without a mask).
    fn read_bpc(&mut self) -> Option<Vec<u8>> {
        let (dir, name) = (
            self.text(self.p.bp_c_file_path, 0),
            self.text(self.p.bp_c_file_name, 0),
        );
        let path = self
            .resolve_calibration_file(&dir, &name, PathAccess::Read)
            .ok()?;
        let expected = self.expected_bpc_size()?;
        let data = match bpc_file::read_exact(&path, expected) {
            Ok(data) => data,
            Err(status) => {
                let message = format!("ReadBPC: {}: {}", status.message(), path.display());
                self.refuse_bpc(adlog::READ_BPC_FILE, adlog::snprintf(256, message));
                return None;
            }
        };
        let family = self.shared.family();
        let supported = mask::operator_mask_supported(family);
        let masked = if supported {
            i32::try_from(mask::count_masked(family, &data)).unwrap_or(i32::MAX)
        } else {
            -1
        };
        let _ = self.ad.port_base.set_int32_param(self.p.bp_cn, 0, masked);
        let line = if supported {
            format!(
                "ReadBPC: file=\"{}\" bytes={} masked {} pixels={masked}",
                path.display(),
                data.len(),
                family.name()
            )
        } else {
            format!(
                "ReadBPC: file=\"{}\" bytes={}; mask count unavailable for family {}",
                path.display(),
                data.len(),
                family.name()
            )
        };
        adlog::print(
            &self.ad.port_base,
            TraceMask::IO_DRIVER,
            &adlog::log(adlog::READ_BPC_FILE, &line),
        );
        let _ = self.ad.port_base.call_param_callbacks(0);
        Some(data)
    }

    /// C `writeBPCfile` (mask_io.cpp:639-720): write the edited BPC to the
    /// mask file name at the geometry's size, then have Serval load *that*
    /// file.
    fn write_bpc_file(&mut self, bpc: &[u8]) -> Option<()> {
        let p = self.p;
        let family = self.shared.family();
        if !mask::operator_mask_supported(family) {
            let _ = self.set_text(
                p.write_msg,
                &format!(
                    "{} mask write blocked: pixel mask encoding is not documented",
                    family.name()
                ),
            );
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::WRITE_BPC_FILE,
                    "WriteBPC: refusing unsupported detector-family mask write",
                ),
            );
            let _ = self.ad.port_base.call_param_callbacks(0);
            return None;
        }
        let dir = self.text(p.bp_c_file_path, 0);
        let mut name = self.text(p.mask_file_name, 0);
        let path_kind = check_file(Path::new(&dir));
        self.write_bpc_log(&format!(
            "Mask write: BPCFilePath type={path_kind} (1=dir) path=\"{dir}\" maskName=\"{name}\""
        ));
        if name.is_empty() {
            let _ = self.set_text(p.mask_file_name, "mask.bpc");
            name = self.text(p.mask_file_name, 0);
        }
        let path = self
            .resolve_calibration_file(&dir, &name, PathAccess::Write)
            .ok()?;
        let mask_kind = check_file(&path);
        if path_kind != 1 {
            let _ = self.set_text(
                p.write_msg,
                &adlog::snprintf(
                    256,
                    format!("Mask: BPC file path is not a directory: {dir}"),
                ),
            );
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::WRITE_BPC_FILE,
                    &format!(
                        "Mask: BPCFilePath is not a directory (pathExists={path_kind}), \
                             aborting write to \"{dir}\""
                    ),
                ),
            );
            let _ = self.ad.port_base.call_param_callbacks(0);
            return None;
        }
        self.write_bpc_log(&if mask_kind == 2 {
            format!(
                "Mask: overwriting file \"{}\" (pathExists={path_kind}, maskFileExistsAsFile={mask_kind})",
                path.display()
            )
        } else {
            format!(
                "Mask: writing new file \"{}\" (pathExists={path_kind}, maskStat={mask_kind})",
                path.display()
            )
        });

        let expected = self.expected_bpc_size()?;
        let status = bpc_file::write_exact(&path, bpc, expected);
        if status != bpc_file::Status::Ok {
            let message = format!("WriteBPC: {}: {}", status.message(), path.display());
            self.refuse_bpc(adlog::WRITE_BPC_FILE, adlog::snprintf(256, message));
            return None;
        }

        // mask_io.cpp:707-718: uploadBPC loads the mask under BPCFileName,
        // which then reads back the operator's file again.
        let bpc_name = self.text(p.bp_c_file_name, 0);
        let _ = self.set_text(p.bp_c_file_name, &name);
        let status = self.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name);
        let _ = self.set_text(p.bp_c_file_name, &bpc_name);
        let _ = self.ad.port_base.call_param_callbacks(0);
        if status.is_err() {
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::WRITE_BPC_FILE,
                    "WriteBPC: uploadBPC to SERVAL failed",
                ),
            );
            return None;
        }
        Some(())
    }

    fn write_bpc_log(&self, line: &str) {
        adlog::print(
            &self.ad.port_base,
            TraceMask::IO_DRIVER,
            &adlog::log(adlog::WRITE_BPC_FILE, line),
        );
    }

    /// C `refreshPixelConfigFromServal` (serval_http.cpp:1074-1290).
    fn refresh_pixel_config(&mut self) -> AsynResult<()> {
        let p = self.p;
        adlog::print(
            &self.ad.port_base,
            TraceMask::FLOW,
            &adlog::log(
                adlog::REFRESH_PIXEL_CONFIG,
                "fetch per chip, compare to BPC on disk",
            ),
        );
        let bpc = self.read_bpc();
        let num_chips = self.int(p.number_of_chips, 0).max(1);
        let g = self.rows_cols().unwrap_or_default();
        let per_chip = usize::try_from(g.pel_width).map_or(0, |w| w * w);
        let family = self.shared.family();
        let caps = family.capabilities();
        let (bpp, slices) = (caps.bpc_bytes_per_pel, caps.bpc_threshold_slices);
        let chip_bytes = mask::bytes_per_chip(per_chip, bpp, slices);
        let Some(total) = (num_chips as usize)
            .checked_mul(chip_bytes)
            .filter(|_| chip_bytes != 0)
        else {
            let msg = "Invalid PixelConfig detector geometry";
            self.set_text(p.write_msg, msg)?;
            self.ad.port_base.call_param_callbacks(0)?;
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: msg.into(),
            });
        };
        let mut serval_linear = vec![0u8; total];
        let mut chip_decoded = vec![false; num_chips as usize];
        let mut failed = false;

        for chip in 0..num_chips {
            let reply = self
                .http
                .get(&serval::chip_pixel_config(chip), TIMEOUT_5000_MS);
            let (code, body) = match &reply {
                Ok(body) => (200, body.as_str()),
                Err(e) => (e.code(), e.body()),
            };
            let decoded = match mask::parse_pixel_config_response(code, body, chip_bytes) {
                Ok(decoded) => decoded,
                Err(e) => {
                    failed = true;
                    let status = match e {
                        PixelConfigError::HttpFailure => format!("HTTP {code}"),
                        PixelConfigError::LengthMismatch => {
                            format!("Length mismatch (expected {chip_bytes} bytes)")
                        }
                        _ => e.message().to_string(),
                    };
                    let match_code = if e == PixelConfigError::LengthMismatch {
                        3
                    } else {
                        -1
                    };
                    self.set_pixel_config_status(chip, 0, match_code, 0, &status)?;
                    adlog::print(
                        &self.ad.port_base,
                        TraceMask::ERROR,
                        &adlog::err(
                            adlog::REFRESH_PIXEL_CONFIG,
                            &format!("chip {chip} PixelConfig failed: {status}"),
                        ),
                    );
                    continue;
                }
            };
            let len = i32::try_from(decoded.len()).unwrap_or(i32::MAX);
            let Some(bpc) = &bpc else {
                self.set_pixel_config_status(chip, len, 2, 0, "OK, no BPC file")?;
                continue;
            };
            let offset = chip as usize * chip_bytes;
            let Some(file) = bpc.get(offset..).and_then(|b| b.get(..chip_bytes)) else {
                failed = true;
                let status = format!(
                    "BPC too small for chip (need {chip_bytes} bytes at {offset}, have {})",
                    bpc.len()
                );
                self.set_pixel_config_status(chip, len, 3, 0, &status)?;
                continue;
            };
            let mismatch = decoded.iter().zip(file).filter(|(a, b)| a != b).count() as i64;
            chip_decoded[chip as usize] = true;
            serval_linear[offset..offset + chip_bytes].copy_from_slice(&decoded);
            if mismatch > 0 {
                let status = format!("Mismatch {mismatch} bytes");
                self.set_pixel_config_status(chip, len, 0, mismatch, &status)?;
            } else {
                self.set_pixel_config_status(chip, len, 1, 0, "OK, matches BPC")?;
            }
        }

        let mut diff = self
            .ad
            .port_base
            .params
            .get_int32_array(p.pixel_config_diff, 0)
            .map(|d| d.to_vec())
            .unwrap_or_default();
        diff.resize(PIXEL_CONFIG_DIFF_LEN, 0);
        if let Some(bpc) = &bpc {
            let mut selected = self.int(p.counter_select_in, 0);
            if !(0..slices).contains(&selected) {
                selected = 0;
            }
            diff.fill(0);
            let store = |diff: &mut [i32], image: usize, logical: usize| {
                let Some(slot) = diff.get_mut(image) else {
                    return;
                };
                let Some(k) = mask::selected_slice_index(logical, per_chip, bpp, slices, selected)
                    .filter(|&k| k < bpc.len() && k < serval_linear.len())
                else {
                    return;
                };
                if chip_decoded.get(logical / per_chip) != Some(&true) {
                    return;
                }
                if let Some(d) = mask::absolute_packed_difference(&serval_linear, bpc, k, bpp) {
                    *slot = d as i32;
                }
            };
            let cols = usize::try_from(g.cols).unwrap_or(0);
            if family == DetectorFamily::Mpx3 {
                for chip in 0..num_chips {
                    let layout = self.text(p.layout, chip);
                    let warning = match mpx3_layout_entry(&layout, chip) {
                        Err(LayoutEntryError::Incomplete) => {
                            format!("chip {chip} MPX3 layout entry is incomplete")
                        }
                        Err(LayoutEntryError::Json(e)) => {
                            format!("chip {chip} MPX3 layout JSON error: {e}")
                        }
                        Ok((ox, oy, orientation)) => {
                            let map = |lx, ly| {
                                mask::mpx3_layout_coordinates(
                                    (lx, ly),
                                    g.pel_width,
                                    (ox, oy),
                                    g.rows,
                                    &orientation,
                                )
                            };
                            if map(0, 0).is_some() {
                                for ly in 0..g.pel_width {
                                    for lx in 0..g.pel_width {
                                        let Some((ix, iy)) = map(lx, ly).filter(|&(ix, iy)| {
                                            (0..g.cols).contains(&ix) && (0..g.rows).contains(&iy)
                                        }) else {
                                            continue;
                                        };
                                        let local =
                                            ly as usize * g.pel_width as usize + lx as usize;
                                        store(
                                            &mut diff,
                                            iy as usize * cols + ix as usize,
                                            chip as usize * per_chip + local,
                                        );
                                    }
                                }
                                continue;
                            }
                            format!(
                                "chip {chip} MPX3 layout orientation is unsupported: {orientation}"
                            )
                        }
                    };
                    adlog::print(
                        &self.ad.port_base,
                        TraceMask::WARNING,
                        &adlog::warn(adlog::REFRESH_PIXEL_CONFIG, &warning),
                    );
                }
            } else {
                for (image, &k) in self.pel_table(&g).iter().enumerate() {
                    if let Ok(k) = usize::try_from(k) {
                        store(&mut diff, image, k);
                    }
                }
            }
            self.export_masked_pels_json(bpc)?;
        }
        self.ad
            .port_base
            .set_param_status(p.pixel_config_diff, 0, AsynStatus::Success)?;
        // Stored as C's pixelConfigDiff_, which readInt32Array serves.
        let diff = ParamValue::Int32Array(diff.into());
        self.ad
            .port_base
            .set_param_value(p.pixel_config_diff, 0, diff.clone())?;
        self.ad
            .port_base
            .do_callbacks_array(p.pixel_config_diff, 0, diff)?;

        let msg = match (failed, bpc.is_some()) {
            (true, _) => "PixelConfig refresh completed with one or more chip errors.",
            (false, true) => "PixelConfig refresh complete.",
            (false, false) => "PixelConfig refresh complete; BPC comparison skipped.",
        };
        self.set_text(p.write_msg, msg)?;
        self.ad.port_base.call_param_callbacks(0)?;
        if failed {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: msg.into(),
            });
        }
        Ok(())
    }

    /// One chip's PixelConfig readbacks, then `callParamCallbacks(chip)`.
    fn set_pixel_config_status(
        &mut self,
        chip: i32,
        len: i32,
        match_code: i32,
        mismatch: i64,
        status: &str,
    ) -> AsynResult<()> {
        let (p, base) = (self.p, &mut self.ad.port_base);
        base.set_int32_param(p.pixel_config_len, chip, len)?;
        base.set_int32_param(p.pixel_config_match_bp_c, chip, match_code)?;
        base.set_int64_param(p.pixel_config_mismatch_bytes, chip, mismatch)?;
        base.set_string_param(p.pixel_config_status, chip, status)?;
        base.call_param_callbacks(chip)
    }

    /// C `exportMaskedPelsJsonFromBpcBuffer` (serval_http.cpp:911-1072).
    fn export_masked_pels_json(&mut self, bpc: &[u8]) -> AsynResult<()> {
        let p = self.p;
        let family = self.shared.family();
        let refuse = |d: &mut Self, count: i32, status: &str| -> AsynResult<()> {
            d.set_text(p.masked_pels_json_path, "")?;
            d.ad.port_base
                .set_int32_param(p.masked_pels_count, 0, count)?;
            d.set_text(p.masked_pels_export_status, status)?;
            d.ad.port_base.call_param_callbacks(0)
        };
        if bpc.is_empty() {
            return refuse(self, 0, "Skipped: no BPC buffer");
        }
        if !mask::operator_mask_supported(family) {
            let status = format!(
                "Unavailable: {} pixel mask encoding is not documented",
                family.name()
            );
            return refuse(self, -1, &status);
        }
        let (dir, name) = (
            self.text(p.bp_c_file_path, 0),
            self.text(p.bp_c_file_name, 0),
        );
        if name.is_empty() {
            return refuse(self, 0, "Skipped: BPCFileName empty");
        }
        let out_name = format!("{}_masked_pels.json", strip_bpc_ext(&name));
        let Ok(out_path) = self.resolve_calibration_file(&dir, &out_name, PathAccess::Write) else {
            return refuse(self, 0, "Blocked: outside calibration root");
        };
        let g = self.rows_cols().unwrap_or_default();
        let num_chips = self.int(p.number_of_chips, 0);
        let orientation = self.int(p.detector_orientation, 0);
        let width = usize::try_from(g.pel_width).unwrap_or(0);
        let chip_pels = width * width;
        let pixel_bytes = mask::bytes_per_pixel(family);
        let Some(offsets) = self.bpc_image_byte_offsets() else {
            return refuse(self, -1, "Unavailable: detector BPC layout is incomplete");
        };
        let source = self
            .resolve_calibration_file(&dir, &name, PathAccess::Read)
            .map_or_else(|_| name.clone(), |s| s.to_string_lossy().into_owned());
        let det_type = self.text(p.det_type, 0);
        let cols = usize::try_from(g.cols).unwrap_or(0);

        // Keys in nlohmann's sorted order, whatever serde_json's map is.
        let (mut masked, mut bad) = (Vec::new(), Vec::new());
        for (image, offset) in offsets.iter().enumerate() {
            let Some(byte) = *offset else { continue };
            let Some(value) = mask::pixel_value(family, bpc, byte).filter(|v| v & 1 != 0) else {
                continue;
            };
            let logical = byte / pixel_bytes;
            let local = logical.checked_rem(chip_pels).unwrap_or(0);
            let (i, j) = (
                image.checked_rem(cols).unwrap_or(0),
                image.checked_div(cols).unwrap_or(0),
            );
            masked.push(json!({
                "bpc_index": byte,
                "bpc_pixel_index": logical,
                "chip": logical.checked_div(chip_pels).unwrap_or(0) as i32,
                "i": i as i32,
                "index": masked.len() + 1,
                "j": j as i32,
                "lx": local.checked_rem(width).unwrap_or(0) as i32,
                "ly": local.checked_div(width).unwrap_or(0) as i32,
                "value": value,
            }));
            bad.push(json!({ "Median": [1, 1], "Pixel": [i as i32, j as i32] }));
        }
        let counter = i32::try_from(masked.len()).unwrap_or(i32::MAX);
        let mut root = json!({
            "Bad pixels": bad,
            "acquisition": {
                "BPCFileName": name,
                "BPCFilePath": dir,
                "detector_orientation": orientation,
            },
            "counts": { "masked_pels": counter },
        });
        if !det_type.is_empty() {
            root["detector"] = json!({ "type": det_type });
        }
        root["format_version"] = json!(1);
        root["masked_pels"] = Value::Array(masked);
        root["source"] = json!({
            "bpc_bytes_per_pixel": pixel_bytes,
            "bpc_path": source,
            "chip_pel_width": g.pel_width,
            "detector_orientation": orientation,
            "exported_at_utc": utc_iso8601(std::time::SystemTime::now()),
            "num_chips": num_chips,
            "tool": "ADTimePix RefreshPixelConfig mask export",
        });

        let serialized = serde_json::to_string_pretty(&root).unwrap_or_default();
        let status = bpc_file::write_atomic(&out_path, serialized.as_bytes());
        if status != bpc_file::Status::Ok {
            let message = adlog::snprintf(
                256,
                format!("Write failed: {}: {}", status.message(), out_path.display()),
            );
            self.set_text(p.masked_pels_json_path, "")?;
            self.ad
                .port_base
                .set_int32_param(p.masked_pels_count, 0, 0)?;
            self.set_text(p.masked_pels_export_status, &message)?;
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::EXPORT_MASKED_PELS_JSON,
                    &format!("exportMaskedPelsJson: {message}"),
                ),
            );
            return self.ad.port_base.call_param_callbacks(0);
        }
        let out = out_path.to_string_lossy();
        self.set_text(p.masked_pels_json_path, &out)?;
        self.ad
            .port_base
            .set_int32_param(p.masked_pels_count, 0, counter)?;
        self.set_text(
            p.masked_pels_export_status,
            &format!("OK: wrote {counter} masked pels"),
        )?;
        adlog::print(
            &self.ad.port_base,
            TraceMask::FLOW,
            &adlog::log(
                adlog::EXPORT_MASKED_PELS_JSON,
                &format!("masked pels JSON {out} ({counter} entries)"),
            ),
        );
        self.ad.port_base.call_param_callbacks(0)
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
        // acquire.cpp:333-336: start from one known worker generation.
        self.shared
            .stop_and_join_stream_workers(&mut self.ad.port_base);
        if self.mpx3_trigger_conflict(self.int(self.ad.params.trigger_mode, 0)) {
            adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(adlog::ACQUIRE_START, MPX3_TRIGGER_MSG),
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
        if let Err(e) = self.http.get(serval::MEASUREMENT_STOP, DEFAULT_TIMEOUT)
            && e.code() != 404
        {
            self.log_http(&e, prior_stop);
        }
        let measurement = match self.http.get(serval::MEASUREMENT, DEFAULT_TIMEOUT) {
            Ok(body) if !body.is_empty() => match serval::parse_status_response(&body) {
                Ok(m) => Some(m),
                Err(e) => {
                    adlog::print(
                        &self.ad.port_base,
                        TraceMask::WARNING,
                        &adlog::warn(
                            adlog::ACQUIRE_START,
                            &format!(
                                "Failed to validate measurement status: {}, continuing after prior stop",
                                e.message()
                            ),
                        ),
                    );
                    None
                }
            },
            _ => None,
        };
        if let Some(measurement) = measurement
            && measurement.is_running()
        {
            let status = measurement.status.unwrap_or_default();
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::ACQUIRE_START,
                    &format!("Measurement is running (status: {status}), stopping it first"),
                ),
            );
            if let Err(e) = self.http.get(serval::MEASUREMENT_STOP, DEFAULT_TIMEOUT) {
                self.log_http(&e, prior_stop);
            }
        }

        if !self.ensure_preview_tcp_ports_free(false) {
            return Err("Failed to reassign occupied preview TCP ports");
        }
        if !self
            .shared
            .acquisition
            .note_resource(Resource::RemoteMeasurement)
        {
            return Err("Acquisition start was cancelled");
        }
        const PORT_CONFLICT: &str = "Address already in use";
        let mut started = self.http.get(serval::MEASUREMENT_START, DEFAULT_TIMEOUT);
        if let Err(e) = &started
            && e.body().contains(PORT_CONFLICT)
        {
            adlog::print(
                &self.ad.port_base,
                TraceMask::WARNING,
                &adlog::warn(
                    adlog::ACQUIRE_START,
                    "measurement/start failed with port conflict; rotating preview ports and retrying once",
                ),
            );
            if self.ensure_preview_tcp_ports_free(true) {
                started = self.http.get(serval::MEASUREMENT_START, DEFAULT_TIMEOUT);
            }
        }
        if let Err(e) = started {
            self.log_http(
                &e,
                HttpPrint::Failure(
                    "acquireStart GET /measurement/start",
                    "GET",
                    serval::MEASUREMENT_START,
                ),
            );
            return Err(if e.body().contains(PORT_CONFLICT) {
                "Preview TCP port in use (Serval did not release a prior listener)"
            } else {
                FAILED
            });
        }
        // acquire.cpp:411-428.
        let log_headers = self.int(self.p.prv_img_log_headers, 0).max(0);
        for preview in &self.shared.preview {
            let mut preview = preview.lock();
            preview.json_headers = log_headers;
            preview.reset_pairing();
            // C `releasePreviewBandArrays` (serval_stream.cpp:470-481).
            preview.diff = None;
        }
        self.shared.rates.lock().prv_img.restart();
        self.sync_stream_endpoints();
        self.start_stream_workers()?;

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
    /// from the enabled channels' Base PVs as they stand at the start. Img
    /// needs ImgAccumulationEnable, PrvHst needs Format 4 and
    /// PrvHstAccumulationEnable too (acquire.cpp:461-475, :478-500).
    fn sync_stream_endpoints(&mut self) {
        let p = self.p;
        let enabled = |this: &Self, write: usize, base: usize| {
            (this.int(write, 0) != 0).then(|| this.text(base, 0))
        };
        let prv_img = enabled(self, p.write_prv_img, p.prv_img_base);
        let prv_img1 = enabled(self, p.write_prv_img1, p.prv_img1_base);
        let img = enabled(self, p.write_img, p.img_base)
            .filter(|_| self.int(p.img_accumulation_enable, 0) != 0);
        let prv_hst = enabled(self, p.write_prv_hst, p.prv_hst_base).filter(|path| {
            path.starts_with("tcp://")
                && self.int(p.prv_hst_format, 0) == 4
                && self.int(p.prv_hst_accumulation_enable, 0) != 0
        });
        self.sync_tcp_stream_endpoints();
        self.shared.set_stream_paths(crate::state::StreamPaths {
            prv_img,
            prv_img1,
            img,
            prv_hst,
        });
    }

    /// C `syncTcpStreamEndpoints` (acquire.cpp:201-223): re-check each
    /// enabled preview path, and the image path when it is tcp://.
    fn sync_tcp_stream_endpoints(&mut self) {
        let p = self.p;
        let mut checked = vec![];
        if self.int(p.write_prv_img, 0) != 0 {
            checked.push(p.prv_img_base);
        }
        if self.int(p.write_prv_img1, 0) != 0 {
            checked.push(p.prv_img1_base);
        }
        if self.int(p.write_img, 0) != 0 && self.text(p.img_base, 0).starts_with("tcp://") {
            checked.push(p.img_base);
        }
        for base in checked {
            if let Some(channel) = CHANNEL_PATHS.iter().find(|c| (c.base)(&p) == base) {
                let _ = self.check_channel_path(channel);
            }
        }
    }

    /// C `ensurePreviewTcpPortsFree` (acquire.cpp:229-293): move each enabled
    /// tcp:// channel whose port is bound locally (Serval keeps listeners
    /// after a stop), or every one when `force_rotate`, to the next free port,
    /// then re-send the destination. False is C's asynError.
    fn ensure_preview_tcp_ports_free(&mut self, force_rotate: bool) -> bool {
        let p = self.p;
        let mut reserved = std::collections::BTreeSet::new();
        let mut changed = false;
        for (write, base) in [
            (p.write_prv_img, p.prv_img_base),
            (p.write_prv_img1, p.prv_img1_base),
            (p.write_img, p.img_base),
            (p.write_prv_hst, p.prv_hst_base),
        ] {
            if self.int(write, 0) == 0 {
                continue;
            }
            let Some((host, port)) = self.parse_tcp_path(&self.text(base, 0)) else {
                continue;
            };
            let port = u32::from(port);
            reserved.insert(port);
            let busy = tcp_port_in_use(&host, port);
            if !force_rotate && !busy {
                continue;
            }
            let search_from = port + 1;
            let Some(candidate) = (search_from..=65535)
                .find(|c| !reserved.contains(c) && !tcp_port_in_use(&host, *c))
            else {
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(
                        adlog::ENSURE_PREVIEW_TCP_PORTS_FREE,
                        &format!(
                            "No free TCP port found for preview channel (starting at {search_from})"
                        ),
                    ),
                );
                let _ = self.ad.port_base.set_string_param(
                    self.ad.params.status_message,
                    0,
                    "Preview TCP ports in use; restart Serval or choose different preview ports",
                );
                return false;
            };
            reserved.insert(candidate);
            let _ = self.ad.port_base.set_string_param(
                base,
                0,
                format!("tcp://listen@{host}:{candidate}"),
            );
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::ENSURE_PREVIEW_TCP_PORTS_FREE,
                    &format!(
                        "Preview TCP port {port} -> {candidate} ({})",
                        if force_rotate {
                            "forced rotate"
                        } else {
                            "port in use"
                        }
                    ),
                ),
            );
            changed = true;
        }
        self.sync_tcp_stream_endpoints();
        if !changed {
            return true;
        }
        let _ = self.ad.port_base.call_param_callbacks(0);
        self.file_writer().is_ok()
    }

    /// acquire.cpp:428-503: every enabled TCP channel's worker must connect
    /// within C `startRequiredStreamWorker`'s readiness deadline
    /// (acquisition_runtime.cpp:66-82) before the start commits.
    fn start_stream_workers(&mut self) -> Result<(), &'static str> {
        const READY: std::time::Duration = std::time::Duration::from_millis(5000);
        let paths = self.shared.stream_paths();
        let s = &self.shared;
        for (path, worker, resource, failure) in [
            (
                paths.prv_img,
                &s.prv_img_worker,
                Resource::PreviewWorker,
                "Preview stream did not become ready",
            ),
            (
                paths.prv_img1,
                &s.prv_img1_worker,
                Resource::IntegratedPreviewWorker,
                "Integrated preview stream did not become ready",
            ),
            (
                paths.img,
                &s.img_worker,
                Resource::ImageWorker,
                "Image stream did not become ready",
            ),
        ] {
            if !path.is_some_and(|p| p.starts_with("tcp://")) {
                continue;
            }
            if !worker.start()
                || !s.acquisition.note_resource(resource)
                || !worker.wait_for_connected_for(READY)
            {
                return Err(failure);
            }
        }
        // acquire.cpp:477-516: the histogram path is parsed only after the
        // image workers have connected.
        let Some(path) = paths.prv_hst else {
            return Ok(());
        };
        let Some(target) = self.parse_tcp_path(&path) else {
            return Err("Histogram TCP path is invalid");
        };
        let s = &self.shared;
        s.endpoints.lock().prv_hst = Some(target);
        if !s.prv_hst_worker.start()
            || !s.acquisition.note_resource(Resource::HistogramWorker)
            || !s.prv_hst_worker.wait_for_connected_for(READY)
        {
            return Err("Histogram stream did not become ready");
        }
        Ok(())
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

        // The monitor leaves on the phase change.
        let p = self.p;
        let had_remote = claim.snapshot.has(Resource::RemoteMeasurement);
        let mut stop_succeeded = !had_remote;
        let mut remote_stopped = !had_remote;
        let mut measurement_valid = false;
        if had_remote {
            match self.http.get(serval::MEASUREMENT_STOP, DEFAULT_TIMEOUT) {
                Ok(_) => {
                    stop_succeeded = true;
                    // acquisition_runtime.cpp:207: Serval closes the streams
                    // once it has sent the last frames; the workers drain
                    // them while the port stays held.
                    self.shared.wait_for_stream_workers_to_stop(
                        &mut self.ad.port_base,
                        STREAM_CLOSE_TIMEOUT,
                    );
                }
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
        }
        // acquisition_runtime.cpp:211.
        self.shared
            .stop_and_join_stream_workers(&mut self.ad.port_base);
        if had_remote {
            match self.http.get(serval::MEASUREMENT, DEFAULT_TIMEOUT) {
                Ok(body) => match serval::parse_status_response(&body) {
                    Ok(m) => {
                        measurement_valid = true;
                        remote_stopped = m.is_remote_idle();
                        for (reason, value) in tasks::measurement_updates(&self.p, &m) {
                            let _ = self.ad.port_base.set_param_value(reason, 0, value);
                        }
                    }
                    Err(e) => adlog::print(
                        &self.ad.port_base,
                        TraceMask::ERROR,
                        &adlog::err(
                            adlog::STOP_ACQUISITION,
                            &format!(
                                "transactional stop: invalid measurement response: {}",
                                e.message()
                            ),
                        ),
                    ),
                },
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
        self.shared.rates.lock().reset();
        for preview in &self.shared.preview {
            preview.lock().reset_pairing();
        }

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
        adlog::print(
            &self.ad.port_base,
            TraceMask::FLOW,
            &adlog::log(adlog::STOP_ACQUISITION, "Stopping Image Acquisition"),
        );
        Ok(())
    }
}

/// The stdout line C `sendConfiguration` prints before the PUT:
/// `config.dump(3, ' ', true)` (serval_http.cpp:2176).
fn send_configuration_line(server: &Value) -> String {
    format!("server={}", serval::dump(server, Some(3), true))
}

fn acquisition_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// How a failed write reaches the record. C returns asynError for every
/// failure, whatever status its ERR line prints (ADTimePix.cpp:870-873,
/// :1108-1113), and writes `errorMessage` only at its BiasEnabled and MPX3
/// refusals and inside ADDriver's own writes (ADTimePix.cpp:502, :667).
enum Refusal {
    /// The status alone; the closing callbacks and ERR line follow.
    Bare(AsynError),
    /// ADDriver's failure, its message kept; the closing callbacks follow.
    Base(AsynError),
    /// C returns at once with its message.
    Told(AsynError),
}

impl From<AsynError> for Refusal {
    fn from(e: AsynError) -> Self {
        Refusal::Bare(e)
    }
}

impl Refusal {
    /// The status C's ERR line prints.
    fn c_status_code(&self) -> i32 {
        match self {
            Refusal::Bare(e) | Refusal::Base(e) | Refusal::Told(e) => e.c_status_code(),
        }
    }

    fn into_record(self) -> AsynError {
        let message = match self {
            Refusal::Told(e) => return e,
            Refusal::Bare(_) => String::new(),
            Refusal::Base(e) => e.message(),
        };
        AsynError::Status {
            status: AsynStatus::Error,
            message,
        }
    }
}

impl TimePix3Driver {
    /// writeInt32 from its parameter store through its dispatch, before the
    /// closing callbacks (ADTimePix.cpp:505-863).
    fn write_int32_effect(&mut self, user: &mut AsynUser, value: i32) -> Result<(), Refusal> {
        let reason = user.reason;
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;
        let previous = self.int(reason, addr);
        // Acquire is published once the coordinator accepts the transition.
        // The set's status stands wherever C assigns no other
        // (ADTimePix.cpp:507-509).
        let set = if reason != self.ad.params.acquire {
            self.ad.port_base.set_int32_param(reason, addr, value)
        } else {
            Ok(())
        };

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
                adlog::print(
                    &d.ad.port_base,
                    TraceMask::FLOW,
                    &adlog::log(adlog::WRITE_INT32, msg),
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
            self.write_dac(addr, reason, "Vthreshold_fine", value, previous)
        } else if reason == p.vthreshold_coarse {
            self.write_dac(addr, reason, "Vthreshold_coarse", value, previous)
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
                let read = crate::tasks::check_connection(self.reader());
                self.apply_read(read, "checkConnection")
            } else {
                set
            }
        } else if reason == p.refresh_pixel_config {
            self.ad
                .port_base
                .set_int32_param(p.refresh_pixel_config, 0, 0)?;
            if addr == 0 && one_shot(value) {
                self.refresh_pixel_config()
            } else {
                set
            }
        } else if reason == p.apply_config {
            self.ad.port_base.set_int32_param(p.apply_config, 0, 0)?;
            if one_shot(value) {
                self.file_writer().and_then(|()| self.get_server())
            } else {
                set
            }
        } else if reason == p.write_bp_c_file || reason == p.write_da_cs_file {
            // A refused store does not stop the upload (ADTimePix.cpp:614-621).
            let _ = self.ad.port_base.set_int32_param(reason, addr, 0);
            if !one_shot(value) {
                set
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
            self.get_detector(false)
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
                adlog::print(
                    &self.ad.port_base,
                    TraceMask::ERROR,
                    &adlog::err(adlog::WRITE_INT32, MPX3_PIXEL_DEPTH_MSG),
                );
                return Err(Refusal::Told(AsynError::Status {
                    status: AsynStatus::Error,
                    message: MPX3_PIXEL_DEPTH_MSG.into(),
                }));
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
            // ADTimePix.cpp:726.
            set.and_then(|()| self.init_acquisition())
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
                set
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
                self.reconfigure_window(Accum::Img, true, |a| a.set_frames_to_sum(value))?;
            } else if reason == p.img_retention_limit_mb {
                self.reconfigure_window(Accum::Img, false, |a| a.set_retention_limit_mb(value))?;
            } else if reason == p.prv_hst_frames_to_sum {
                self.reconfigure_window(Accum::PrvHst, false, |a| a.set_frames_to_sum(value))?;
            } else if reason == p.prv_hst_retention_limit_mb {
                self.reconfigure_window(Accum::PrvHst, false, |a| a.set_retention_limit_mb(value))?;
            } else if reason == p.img_sum_update_interval_frames {
                self.shared.img.lock().set_update_interval(value);
            } else if reason == p.prv_hst_sum_update_interval {
                self.shared.hst.lock().set_update_interval(value);
            }
            set
        } else if reason == p.processed_img_output_type || reason == p.processed_hst_output_type {
            // The addressed set, then always list 0 (ADTimePix.cpp:593-597,
            // :605-609).
            let output_type = if value == 0 || value == 1 { value } else { 0 };
            let _ = self.ad.port_base.set_int32_param(reason, 0, output_type);
            set
        } else if reason == p.img_image_data_reset {
            if value == 1 {
                // C resetImgAccumulation (serval_stream.cpp:1421).
                let (range, memory) = {
                    let mut img = self.shared.img.lock();
                    img.reset();
                    (
                        img.report_range(RangeState::Ok),
                        crate::tasks::img_memory_mb(&img),
                    )
                };
                let base = &mut self.ad.port_base;
                base.set_int32_param(p.img_frames_summed, 0, 0)?;
                if let Some(range) = range {
                    self.set_range(p.img_range_saturated, p.img_range_status, range)?;
                }
                let base = &mut self.ad.port_base;
                base.set_int64_param(p.img_total_counts, 0, 0)?;
                base.set_float64_param(p.img_processing_time, 0, 0.0)?;
                base.set_float64_param(p.img_memory_usage, 0, memory)?;
                self.ad
                    .port_base
                    .set_int32_param(p.img_image_data_reset, 0, 0)?;
            }
            set
        } else if reason == p.write_processed_img {
            // ADTimePix.cpp:586-592; the task stores 0 once the arrays are out.
            if value == 1 {
                self.cmd_tx
                    .try_send(Command::PushProcessedImg)
                    .map_err(|_| AsynError::Status {
                        status: AsynStatus::Error,
                        message: "timepix3: the connection task is not running".into(),
                    })?;
            }
            set
        } else if reason == p.write_processed_hst {
            // ADTimePix.cpp:598-604; the task stores 0 once the arrays are out.
            if value == 1 {
                self.cmd_tx
                    .try_send(Command::PushProcessedHst)
                    .map_err(|_| AsynError::Status {
                        status: AsynStatus::Error,
                        message: "timepix3: the connection task is not running".into(),
                    })?;
            }
            set
        } else if reason == p.prv_hst_data_reset {
            if value == 1 {
                // C resetPrvHstAccumulation (ADTimePix.cpp:1365-1416).
                let rate_samples = self.shared.rates.lock().prv_hst.sample_count();
                let (range, memory) = {
                    let mut hst = self.shared.hst.lock();
                    hst.reset();
                    (
                        hst.report_range(RangeState::Ok),
                        tasks::prv_hst_memory_mb(&hst, rate_samples, tasks::PRV_HST_LINE_BUFFER),
                    )
                };
                self.ad
                    .port_base
                    .set_int32_param(p.prv_hst_frames_summed, 0, 0)?;
                if let Some(range) = range {
                    self.set_range(p.prv_hst_range_saturated, p.prv_hst_range_status, range)?;
                }
                let base = &mut self.ad.port_base;
                base.set_int32_param(p.prv_hst_frame_count, 0, 0)?;
                base.set_int64_param(p.prv_hst_total_counts, 0, 0)?;
                base.set_float64_param(p.prv_hst_processing_time, 0, 0.0)?;
                base.set_float64_param(p.prv_hst_memory_usage, 0, memory)?;
                // Zero arrays clear the waveforms at once, sized to the last
                // frame's bins or C's default 16000.
                let bins = usize::try_from(self.int(p.prv_hst_frame_bin_size, 0))
                    .ok()
                    .filter(|&n| n > 0)
                    .unwrap_or(16000);
                for reason in [p.prv_hst_histogram_data, p.prv_hst_histogram_sum_n_frames] {
                    let zeros = ParamValue::Int64Array(vec![0; bins].into());
                    self.ad.port_base.do_callbacks_array(reason, 0, zeros)?;
                }
                self.ad
                    .port_base
                    .set_int32_param(p.prv_hst_data_reset, 0, 0)?;
            }
            set
        } else if reason == p.health {
            // ADTimePix.cpp:553-561: only getDetector's status stands.
            let read = crate::tasks::get_dashboard(self.reader());
            let _ = self.apply_read(read, "getDashboard");
            let status = self.get_detector(false);
            let _ = self.get_measurement_config();
            status
        } else if reason < p.http_code {
            // ADTIMEPIX_FIRST_PARAM is ADTimePixHttpCode (ADTimePix.h:467,
            // ADTimePix.cpp:860-863).
            return ADDriver::ad_write_int32(self, user, value).map_err(Refusal::Base);
        } else {
            set
        };
        Ok(result?)
    }

    /// writeFloat64 from its parameter store through its dispatch, before
    /// the closing callbacks (ADTimePix.cpp:1090-1105).
    fn write_float64_effect(&mut self, user: &mut AsynUser, value: f64) -> Result<(), Refusal> {
        let reason = user.reason;
        // UPSTREAM DEFECT (ADTimePix.cpp:1093): C's writeFloat64 calls
        // `setDoubleParam(function, value)` — the two-argument form, which
        // always writes address 0 — on a multi-device port, so every
        // per-address float write lands on address 0. The address is honoured
        // here.
        let addr = user.addr;
        let acquiring = self.int(self.ad.params.acquire, 0) != 0;

        // The set's status stands unless a branch assigns another
        // (ADTimePix.cpp:1093-1106).
        let set = self.ad.port_base.set_float64_param(reason, addr, value);

        let p = self.p;
        if reason == self.ad.params.acquire_time
            || reason == self.ad.params.acquire_period
            || reason == p.trigger_delay
            || reason == p.global_timestamp_interval
        {
            if acquiring {
                let _ = self.acquire_stop();
            }
            Ok(self.init_acquisition()?)
        } else if reason == p.stem_dwell_time || reason == p.tof_min || reason == p.tof_max {
            Ok(self.send_measurement_config()?)
        } else if reason < p.http_code {
            // ADDriver::writeFloat64 is asynPortDriver's (ADTimePix.cpp:1100-1104).
            self.ad
                .port_base
                .write_float64_param(user, value)
                .map_err(Refusal::Base)
        } else {
            Ok(set?)
        }
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

        if reason == self.p.bias_enable && !(value == 0 || value == 1) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("BiasEnabled must be 0 or 1 (received {value})"),
            });
        }
        let result = match self.write_int32_effect(user, value) {
            Err(Refusal::Told(e)) => return Err(e),
            r => r,
        };

        // The write's outcome is the record's, and it must survive the
        // callbacks (ADTimePix.cpp:866-873).
        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(refusal) => adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::WRITE_INT32,
                    &format!(
                        "ERROR status={} function={reason}, value={value}",
                        refusal.c_status_code()
                    ),
                ),
            ),
            Ok(()) => adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::WRITE_INT32,
                    &format!("function={reason} value={value}"),
                ),
            ),
        }
        result.map_err(Refusal::into_record)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let result = self.write_float64_effect(user, value);

        let _ = self.ad.port_base.call_param_callbacks(addr);
        match &result {
            Err(refusal) => adlog::print(
                &self.ad.port_base,
                TraceMask::ERROR,
                &adlog::err(
                    adlog::WRITE_FLOAT64,
                    &format!(
                        "ERROR status = {}, function ={reason}, value = {value:.6}",
                        refusal.c_status_code(),
                        value = CFloat(value)
                    ),
                ),
            ),
            Ok(()) => adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::WRITE_FLOAT64,
                    &format!("function={reason} value={value:.6}", value = CFloat(value)),
                ),
            ),
        }
        result.map_err(Refusal::into_record)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let reason = user.reason;
        // ADTimePix.cpp:424-425.
        let addr = self.ad.port_base.parse_user(user)?;
        let p = self.p;
        // C stores a base-class string without chaining to
        // asynNDArrayDriver::writeOctet (ADTimePix.cpp:443-444), so
        // NDAttributesFile never loads and NDFilePath is never checked; that
        // C defect is not kept.
        if reason < p.fw_time_stamp {
            return self.ad.write_octet(user, data);
        }
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
            adlog::print(
                &self.ad.port_base,
                TraceMask::IO_DRIVER,
                &adlog::log(
                    adlog::WRITE_OCTET,
                    &format!("Ignoring empty threshold list write for paramName={name}"),
                ),
            );
            return Ok(data.len());
        }
        // C keeps the store's status, lets a path check overwrite it, takes
        // the callbacks' only over success, and returns it raw
        // (ADTimePix.cpp:443-475).
        let code = |r: AsynResult<()>| r.err().map_or(0, |e| e.c_status_code());
        let stored = code(
            self.ad
                .port_base
                .set_string_param(reason, addr, value.clone()),
        );
        let checked = if reason == p.bp_c_file_path {
            Some(self.check_calibration_dir(
                p.bp_c_file_path,
                p.bp_c_file_path_exists,
                adlog::CHECK_BPC_PATH,
            ))
        } else if reason == p.da_cs_file_path {
            Some(self.check_calibration_dir(
                p.da_cs_file_path,
                p.da_cs_file_path_exists,
                adlog::CHECK_DACS_PATH,
            ))
        } else if let Some(channel) = CHANNEL_PATHS.iter().find(|c| (c.base)(&p) == reason) {
            Some(self.check_channel_path(channel))
        } else if reason == p.tof_tdc_reference {
            Some(self.send_measurement_config())
        } else {
            None
        };
        let mut status = checked.map_or(stored, code);
        let callbacks = code(self.ad.port_base.call_param_callbacks(addr));
        if status == 0 {
            status = callbacks;
        }
        if let Some(e) = AsynError::from_c_status_code(
            status,
            format!(
                "ADTimePix:{}: status={status}, function={reason}, paramName={name}, value={value}",
                adlog::WRITE_OCTET,
            ),
        ) {
            return Err(e);
        }
        adlog::print(
            &self.ad.port_base,
            TraceMask::IO_DRIVER,
            &adlog::log(
                adlog::WRITE_OCTET,
                &format!("function={reason}, paramName={name}, value={value}"),
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

    /// C `readInt32Array` (mask_io.cpp:32-249); `buf` holds the record's VAL.
    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let reason = user.reason;
        if reason == self.p.mask_bp_c {
            // C draws into the record's value (mask_io.cpp:99-129).
            return self.mask_bp_c(buf).map(|()| buf.len());
        }
        if reason == self.p.bp_c {
            // C mask_io.cpp:209-237: the packed pixel, with bit 8 (TPX3) or
            // bit 16 (MPX3) marking a masked one; an empty file leaves the
            // waveform as it was.
            let bpc = self.read_bpc().ok_or_else(Self::read_array_refused)?;
            if !bpc.is_empty() {
                let family = self.shared.family();
                let width = mask::bytes_per_pixel(family);
                let pixels = bpc.len().checked_div(width).unwrap_or(0);
                let copied = buf.len().min(pixels);
                for (pixel, slot) in buf[..copied].iter_mut().enumerate() {
                    let offset = pixel * width;
                    *slot = i32::from(mask::pixel_value(family, &bpc, offset).unwrap_or(0));
                    if mask::is_masked(family, &bpc, offset) {
                        *slot |= if width == 1 { 1 << 8 } else { 1 << 16 };
                    }
                }
                buf[copied..].fill(0);
            }
            self.ad.port_base.call_param_callbacks(0)?;
            return Ok(buf.len());
        }
        if reason == self.p.pixel_config_diff || reason == self.p.img_image_frame {
            // C answers these two with nElements, zero-filled past the data,
            // including before the first value, and returns before its
            // callParamCallbacks (mask_io.cpp:38-91).
            let data = self
                .ad
                .port_base
                .params
                .get_int32_array(reason, user.addr)
                .unwrap_or_default();
            let n = buf.len().min(data.len());
            buf[..n].copy_from_slice(&data[..n]);
            buf[n..].fill(0);
            return Ok(buf.len());
        }
        // asynPortDriver's readArray writes nothing, so C returns nElements
        // of the record's own VAL (asynPortDriver.cpp:1813-1817,
        // mask_io.cpp:92, :239-247).
        self.ad.port_base.call_param_callbacks(0)?;
        Ok(buf.len())
    }
}

/// C `checkFile` (mask_io.cpp:468-495): `stat`'s file type as 1 directory,
/// 2 regular file, 3 block, 4 FIFO, 6 character, 7 socket, 8 other; 0 when
/// `stat` fails.
fn check_file(path: &Path) -> i32 {
    let Ok(meta) = std::fs::metadata(path) else {
        return 0;
    };
    let kind = meta.file_type();
    if kind.is_dir() {
        return 1;
    }
    if kind.is_file() {
        return 2;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::FileTypeExt;
        if kind.is_block_device() {
            return 3;
        }
        if kind.is_fifo() {
            return 4;
        }
        if kind.is_char_device() {
            return 6;
        }
        if kind.is_socket() {
            return 7;
        }
    }
    8
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
        // "/" has no drive on Windows, so C's is_absolute() refuses it there.
        let (root, policy) = if cfg!(windows) {
            ("", "Blocked: calibration root is not absolute")
        } else {
            ("/", "Permissive: /")
        };
        assert_eq!(det.text(p.calibration_root, 0), root);
        assert_eq!(det.text(p.calibration_policy, 0), policy);
        let tmp = crate::calibration::canonical(&std::env::temp_dir()).unwrap();
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

    /// C `rowsCols` refuses an eight-chip detector that is not two TPX3
    /// quads with its WriteMsg (mask_io.cpp:258-279).
    #[test]
    fn rows_cols_refuses_an_eight_chip_layout_as_c() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Tpx3);
        let mut det = TimePix3Driver::new(
            "TPX_ROWS_COLS",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for (param, v) in [
            (p.pix_count, 8 * 65536),
            (p.row_len, 4),
            (p.number_of_chips, 8),
            (p.number_of_rows, 512),
        ] {
            det.ad.port_base.set_int32_param(param, 0, v).unwrap();
        }
        det.set_text(p.boards_id, "4100").unwrap();
        det.set_text(p.boards2_id, "4100").unwrap();
        assert_eq!(det.rows_cols(), None);
        assert_eq!(
            det.text(p.write_msg, 0),
            "Unsupported eight-chip mask layout: require two TPX3 quad boards"
        );
        det.set_text(p.boards2_id, "4101").unwrap();
        let g = det.rows_cols().unwrap();
        assert_eq!((g.rows, g.cols, g.x_chips, g.y_chips), (512, 1024, 4, 2));
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
        // C posts the refusal to WriteMsg only; the record gets the status
        // (ADTimePix.cpp:172-183).
        assert_eq!(e.message(), "");
        assert_eq!(
            det.text(p.write_msg, 0),
            "Destination blocked by startup allowlist: file:/data"
        );
        // C formats it into a char[384] (ADTimePix.cpp:176-178).
        let long = format!("file:/{}", "d".repeat(500));
        det.ad
            .port_base
            .set_string_param(p.raw_base, 0, long.as_str())
            .unwrap();
        assert!(
            det.write_int32(&mut AsynUser::new(p.write_data), 1)
                .is_err()
        );
        let msg = det.text(p.write_msg, 0);
        assert_eq!(msg.len(), 383);
        assert!(msg.starts_with("Destination blocked by startup allowlist: file:/ddd"));
        det.configure_destinations("tcp://*x").unwrap();
        assert_eq!(
            det.text(p.destination_policy, 0),
            "Blocked: destination allowlist contains an invalid entry"
        );
    }

    /// C's writeFloat64 returns asynError with no errorMessage when Serval
    /// refuses the configuration (ADTimePix.cpp:1107-1110).
    #[test]
    fn a_failed_float_write_hands_the_record_no_message() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_F64_REFUSED",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let ad = det.ad.params;
        let e = det
            .write_float64(&mut AsynUser::new(ad.acquire_time), 0.5)
            .unwrap_err();
        assert_eq!(e.message(), "");
    }

    /// Every failed write reaches the record as asynError, and the ERR line
    /// prints the status C held: the param library's 11 for an address past
    /// maxAddr (ADTimePix.cpp:866-873, :1106-1113).
    #[test]
    fn a_refused_write_is_asyn_error_and_the_line_keeps_the_code() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_SET_CODE";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let (p, ad) = (det.p, det.ad.params);
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
        h.trace().set_trace_mask(Some(name), TraceMask::ERROR);

        let refused = |e: AsynError| (e.c_status_code(), e.message());
        let e = h
            .write_int32_blocking(p.processed_img_output_type, 20, 1)
            .unwrap_err();
        assert_eq!(refused(e), (3, String::new()));
        let e = h
            .write_float64_blocking(p.img_processing_time, 20, 1.0)
            .unwrap_err();
        assert_eq!(refused(e), (3, String::new()));
        // ADDriver's own write at a bad address: getAddress fails with
        // asynError, which its line prints.
        let e = h.write_int32_blocking(ad.num_exposures, 20, 1).unwrap_err();
        assert_eq!(e.c_status_code(), 3);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for line in [
            adlog::err(
                adlog::WRITE_INT32,
                &format!(
                    "ERROR status=11 function={}, value=1",
                    p.processed_img_output_type
                ),
            ),
            adlog::err(
                adlog::WRITE_FLOAT64,
                &format!(
                    "ERROR status = 11, function ={}, value = 1.000000",
                    p.img_processing_time
                ),
            ),
            adlog::err(
                adlog::WRITE_INT32,
                &format!("ERROR status=3 function={}, value=1", ad.num_exposures),
            ),
        ] {
            assert!(text.contains(&line), "{line} not in {text}");
        }
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
        let tmp = crate::calibration::canonical(&std::env::temp_dir()).unwrap();
        let tmp = tmp.to_str().unwrap();
        // C takes `file:/…` only; on Windows that is the path on the current
        // drive, without its `C:` and with '/' separators.
        let tmp = if cfg!(windows) {
            tmp[2..].replace('\\', "/")
        } else {
            tmp.to_string()
        };
        let mut write = |reason, value: String| {
            det.write_octet(&mut AsynUser::new(reason), value.as_bytes())
                .is_ok()
        };
        assert!(write(p.raw_base, format!("file:{tmp}")));
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
        assert_eq!(det.text(p.raw_base, 0), format!("file:{tmp}"));
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
        let tmp = crate::calibration::canonical(&std::env::temp_dir()).unwrap();
        det.configure_calibration(tmp.to_str().unwrap()).unwrap();
        let tmp = tmp.join(format!("tpx-calib-dir-{}", std::process::id()));
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

    /// A refused store does not end writeOctet: the checks, the callbacks
    /// and the errorMessage still follow (ADTimePix.cpp:443-470).
    #[test]
    fn a_refused_octet_store_still_reports_the_write_octet_message() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_OCTET_STORE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let e = det
            .write_octet(&mut AsynUser::new(p.http_code), b"x")
            .unwrap_err();
        assert_eq!(e.c_status_code(), 8);
        let message = e.to_string();
        assert!(
            message.contains(&format!(
                "ADTimePix:{}: status=8,",
                "virtual asynStatus ADTimePix::writeOctet(asynUser*, const char*, size_t, size_t*)"
            )) && message.ends_with(&format!(
                ", function={}, paramName=TPX3_HTTP_CODE, value=x",
                p.http_code
            )),
            "{message}"
        );
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

    /// C `resetImgAccumulation` zeroes FramesSummed, TotalCounts and the
    /// processing time and re-posts the memory estimate
    /// (serval_stream.cpp:1421-1440).
    #[test]
    fn an_img_reset_posts_the_cleared_metrics() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        let mut det = TimePix3Driver::new(
            "TPX_IMG_RESET",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        shared.img.lock().add_shaped(&[1, 2], &[2, 1], 2);
        let base = &mut det.ad.port_base;
        base.set_int32_param(p.img_frames_summed, 0, 1).unwrap();
        base.set_int64_param(p.img_total_counts, 0, 3).unwrap();
        base.set_float64_param(p.img_processing_time, 0, 1.5)
            .unwrap();
        det.write_int32(&mut AsynUser::new(p.img_image_data_reset), 1)
            .unwrap();
        let base = &det.ad.port_base;
        assert_eq!(det.int(p.img_frames_summed, 0), 0);
        assert_eq!(base.get_int64_param(p.img_total_counts, 0).unwrap(), 0);
        assert_eq!(
            base.get_float64_param(p.img_processing_time, 0).unwrap(),
            0.0
        );
        // The kept 2-pixel UINT16 current frame, the window's zeroed 2-u64
        // sum and 2 pixels of waveforms.
        let expected = f64::from(4 + 16 + 40) / (1024.0 * 1024.0) + 0.1;
        assert_eq!(
            base.get_float64_param(p.img_memory_usage, 0).unwrap(),
            expected
        );
    }

    /// Int64Array callbacks, as (reason, values).
    type Recorded = Arc<parking_lot::Mutex<Vec<(usize, Vec<i64>)>>>;

    /// A driver whose Int64Array callbacks are recorded.
    fn array_recording_driver(name: &str) -> (TimePix3Driver, Arc<Shared>, Recorded) {
        use epics_rs::asyn::interrupt::InterruptFilter;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        det.ad.port_base.own_interrupts(InterruptGate::opened());
        let arrays = Arc::new(parking_lot::Mutex::new(Vec::new()));
        let seen = arrays.clone();
        std::mem::forget(
            det.ad
                .port_base
                .interrupts
                .register_sync_callback(
                    InterruptFilter {
                        addr: Some(0),
                        ..Default::default()
                    },
                    move |iv| {
                        if let ParamValue::Int64Array(a) = &iv.value {
                            seen.lock().push((iv.reason, a.to_vec()));
                        }
                    },
                )
                .unwrap(),
        );
        (det, shared, arrays)
    }

    /// ADTimePix.cpp:755-845: FramesToSum and RetentionLimitMB reconfigure a
    /// window that has a geometry at once, and say so when it has none.
    #[test]
    fn a_window_write_reconfigures_and_posts_the_window() {
        let (mut det, shared, arrays) = array_recording_driver("TPX_WINDOW_WRITE");
        let p = det.p;
        {
            let mut img = shared.img.lock();
            img.set_update_interval(5);
            img.add_shaped(&[1, 2], &[2, 1], 2);
            img.add_shaped(&[3, 4], &[2, 1], 2);
        }
        det.write_int32(&mut AsynUser::new(p.img_frames_to_sum), 1)
            .unwrap();
        assert_eq!(
            (
                det.int(p.img_effective_frames, 0),
                det.int(p.img_frames_summed, 0),
                det.text(p.img_retention_status, 0),
            ),
            (1, 1, "OK: 1 frames, 512 MiB".to_string())
        );
        assert_eq!(*arrays.lock(), vec![(p.img_image_sum_n_frames, vec![3, 4])]);
        // C sets the update counter due, so the next frame publishes the sum.
        assert!(shared.img.lock().add(&[5, 6]).sum_n.is_some());

        det.write_int32(&mut AsynUser::new(p.img_retention_limit_mb), 1)
            .unwrap();
        assert_eq!(shared.img.lock().retention_limit_mb(), 1);
        assert_eq!(det.text(p.img_retention_status, 0), "OK: 1 frames, 1 MiB");

        det.write_int32(&mut AsynUser::new(p.prv_hst_frames_to_sum), 3)
            .unwrap();
        assert_eq!(
            (
                det.int(p.prv_hst_effective_frames, 0),
                det.text(p.prv_hst_retention_status, 0),
            ),
            (0, "Waiting for histogram geometry".to_string())
        );
        assert_eq!(
            arrays.lock().len(),
            1,
            "only an Img FramesToSum posts the sum"
        );
    }

    /// C resetPrvHstAccumulation (ADTimePix.cpp:1365-1416).
    #[test]
    fn a_prv_hst_reset_posts_the_cleared_metrics_and_zero_arrays() {
        let (mut det, shared, arrays) = array_recording_driver("TPX_HST_RESET");
        let p = det.p;
        shared.hst.lock().add(&[1, 2, 3]);
        let base = &mut det.ad.port_base;
        base.set_int32_param(p.prv_hst_frame_bin_size, 0, 3)
            .unwrap();
        base.set_int32_param(p.prv_hst_frames_summed, 0, 1).unwrap();
        base.set_int32_param(p.prv_hst_frame_count, 0, 5).unwrap();
        base.set_int64_param(p.prv_hst_total_counts, 0, 6).unwrap();
        base.set_float64_param(p.prv_hst_processing_time, 0, 1.5)
            .unwrap();
        det.write_int32(&mut AsynUser::new(p.prv_hst_data_reset), 1)
            .unwrap();
        let base = &det.ad.port_base;
        assert_eq!(
            (
                det.int(p.prv_hst_frames_summed, 0),
                det.int(p.prv_hst_frame_count, 0),
                base.get_int64_param(p.prv_hst_total_counts, 0).unwrap(),
                base.get_float64_param(p.prv_hst_processing_time, 0)
                    .unwrap(),
            ),
            (0, 0, 0, 0.0)
        );
        // The window keeps its zeroed 3-u64 sum.
        assert_eq!(
            base.get_float64_param(p.prv_hst_memory_usage, 0).unwrap(),
            24.0 / (1024.0 * 1024.0) + tasks::PRV_HST_LINE_BUFFER as f64 / (1024.0 * 1024.0) + 0.1
        );
        assert_eq!(
            *arrays.lock(),
            vec![
                (p.prv_hst_histogram_data, vec![0; 3]),
                (p.prv_hst_histogram_sum_n_frames, vec![0; 3]),
            ]
        );
    }

    /// A stream path on TEST-NET-1, never a local address: acquireStart's
    /// bind probe cannot find it busy, as it can a shared localhost port
    /// that a concurrent test is probing.
    const UNBOUND_TCP: &str = "tcp://192.0.2.1:8089";

    /// A Serval that answers `{}` to everything but a request line holding
    /// `fail`, which gets a 500.
    fn fake_serval(fail: &'static str) -> String {
        fake_serval_with(fail, &[])
    }

    /// [`fake_serval`] answering a request line that starts with a route's
    /// prefix with that route's body.
    fn fake_serval_with(
        fail: &'static str,
        routes: &'static [(&'static str, &'static str)],
    ) -> String {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let (mut request, mut length) = (String::new(), 0);
                let _ = reader.read_line(&mut request);
                request = crate::http::without_serval_params(&request);
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
                let reply = routes
                    .iter()
                    .find(|(prefix, _)| request.starts_with(prefix))
                    .map_or("{}", |(_, body)| body);
                let _ = reader.get_mut().write_all(
                    format!(
                        "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                        reply.len()
                    )
                    .as_bytes(),
                );
            }
        });
        url
    }

    /// A threshold write Serval refuses leaves the record at Serval's value
    /// and publishes the reply (serval_http.cpp:589-660).
    #[test]
    fn a_refused_dac_write_restores_the_threshold_as_c() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        use std::io::{BufRead, BufReader, Read, Write};
        type Script = &'static [(&'static str, &'static str)];
        // Serval answers the requests in turn with (status, body).
        fn run(name: &'static str, script: Script) -> (bool, i32, i32, String, String, String) {
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let url = format!("http://{}", listener.local_addr().unwrap());
            std::thread::spawn(move || {
                for (stream, (status, reply)) in listener.incoming().flatten().zip(script) {
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
                    let _ = reader.get_mut().write_all(
                        format!(
                            "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                            reply.len()
                        )
                        .as_bytes(),
                    );
                }
            });
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&url)),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            det.ad
                .port_base
                .set_int32_param(p.vthreshold_fine, 0, 3)
                .unwrap();
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
                .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::WARNING);
            let (ok, value, code, msg) = h
                .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                    let ok = d
                        .write_int32(&mut AsynUser::new(p.vthreshold_fine).with_addr(0), 300)
                        .is_ok();
                    (
                        ok,
                        d.int(p.vthreshold_fine, 0),
                        d.int(p.http_code, 0),
                        d.text(p.write_msg, 0),
                    )
                })
                .unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            std::mem::forget(runtime);
            (
                ok,
                value,
                code,
                msg,
                text,
                format!("{url}/detector/chips/0/dacs/"),
            )
        }
        const OK: &str = "200 OK";
        const BAD: &str = "500 Internal Server Error";
        const HELD: &str = r#"{"Vthreshold_fine": 7, "Vfbk": 5}"#;
        let line = |context: &str, method: &str, url: &str, code: i32, body: &str| {
            adlog::http_failure_reply(context, method, url, code, body)
        };

        let (ok, value, code, msg, text, url) = run("TPX_DAC_GET", &[(BAD, "no chip")]);
        assert_eq!((ok, value, code, msg.as_str()), (false, 3, 500, "no chip"));
        assert!(text.contains(&line("writeDac GET chip 0", "GET", &url, 500, "no chip")));

        let (ok, value, code, msg, _, _) = run("TPX_DAC_GET_EMPTY", &[(BAD, "")]);
        assert_eq!(
            (ok, value, code, msg.as_str()),
            (false, 3, 500, "Failed to read DAC configuration")
        );

        let (ok, value, code, msg, text, _) = run("TPX_DAC_MISSING", &[(OK, r#"{"Vfbk": 5}"#)]);
        let missing = "DAC response is missing the requested field";
        assert_eq!((ok, value, code, msg.as_str()), (false, 3, 200, missing));
        // Each line ends in the C macros' `\n` (ADTimePixLog.h:41-71).
        let err = adlog::err(adlog::WRITE_DAC, &format!("writeDac GET chip 0: {missing}"));
        assert!(text.contains(&format!("{err}\n")), "{text:?}");

        let (ok, value, code, msg, _, _) = run("TPX_DAC_OK", &[(OK, HELD), (OK, "")]);
        assert_eq!(
            (ok, value, code, msg.as_str()),
            (true, 300, 200, "Uploaded DAC configuration.")
        );

        // Serval moved on before the rejection; the record follows it.
        let (ok, value, code, msg, text, url) = run(
            "TPX_DAC_PUT",
            &[
                (OK, HELD),
                (BAD, "rejected"),
                (OK, r#"{"Vthreshold_fine": 9}"#),
            ],
        );
        assert_eq!((ok, value, code, msg.as_str()), (false, 9, 500, "rejected"));
        assert!(text.contains(&line("writeDac PUT", "PUT", &url, 500, "rejected")));

        let (ok, value, code, msg, text, _) =
            run("TPX_DAC_PUT_STALE", &[(OK, HELD), (BAD, ""), (BAD, "")]);
        assert_eq!(
            (ok, value, code, msg.as_str()),
            (false, 7, 500, "Failed to upload DAC configuration")
        );
        assert!(text.contains(&adlog::warn(
            adlog::WRITE_DAC,
            "writeDac: failed to refresh Vthreshold_fine after rejected PUT; using pre-PUT value"
        )));
    }

    /// A Serval that answers every request `200 {}` and reports each
    /// request line and body.
    fn capturing_serval() -> (String, std::sync::mpsc::Receiver<(String, String)>) {
        capturing_serval_with(&[])
    }

    /// [`capturing_serval`] answering a request line that starts with a
    /// route's prefix with that route's body.
    fn capturing_serval_with(
        routes: &'static [(&'static str, &'static str)],
    ) -> (String, std::sync::mpsc::Receiver<(String, String)>) {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let (mut request, mut length) = (String::new(), 0);
                let _ = reader.read_line(&mut request);
                request = crate::http::without_serval_params(&request);
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
                let reply = routes
                    .iter()
                    .find(|(prefix, _)| request.starts_with(prefix))
                    .map_or("{}", |(_, body)| body);
                let _ = tx.send((request, String::from_utf8_lossy(&body).into_owned()));
                let _ = reader.get_mut().write_all(
                    format!(
                        "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                        reply.len()
                    )
                    .as_bytes(),
                );
            }
        });
        (url, rx)
    }

    /// Each PUT body is the text of C's dump call: writeDac's
    /// `dump(3, ' ', true)` and fileWriter's compact `dump()`, keys sorted as
    /// nlohmann's std::map holds them (serval_http.cpp:619, 2180).
    #[test]
    fn put_bodies_are_nlohmann_dumps() {
        let (url, rx) = capturing_serval_with(&[(
            "GET /detector/chips/0/dacs/ ",
            r#"{"Vthreshold_fine": 7, "Ibias": 1.5e-5, "Name": "d\u00e9"}"#,
        )]);
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_PUT_DUMP",
            Arc::new(ServalHttp::new(&url)),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let b = &mut det.ad.port_base;
        b.set_int32_param(p.vthreshold_fine, 0, 7).unwrap();
        b.set_string_param(p.raw1_base, 0, "file:/data").unwrap();
        b.set_int32_param(p.write_raw1, 0, 1).unwrap();
        det.write_int32(&mut AsynUser::new(p.vthreshold_fine).with_addr(0), 300)
            .unwrap();
        det.file_writer().unwrap();
        let puts: Vec<_> = rx
            .try_iter()
            .filter(|(line, _)| line.starts_with("PUT "))
            .collect();
        assert_eq!(
            puts[0],
            (
                "PUT /detector/chips/0/dacs/ HTTP/1.1\r\n".to_owned(),
                "{\n   \"Ibias\": 1.5e-05,\n   \"Name\": \"d\\u00e9\",\n   \"Vthreshold_fine\": 300\n}"
                    .to_owned()
            )
        );
        assert_eq!(puts[1].0, "PUT /server/destination HTTP/1.1\r\n");
        let body = &puts[1].1;
        let parsed: serde_json::Value = serde_json::from_str(body).unwrap();
        assert_eq!(*body, serval::dump(&parsed, None, false));
        assert!(body.contains(r#"[null,{"Base":"file:/data","#), "{body}");
    }

    /// fileWriter fills each channel at its own index, sends nothing when no
    /// channel is enabled, and names the channel a refusal came from
    /// (serval_http.cpp:2333-2425).
    #[test]
    fn file_writer_places_each_channel_as_c() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        type Setup = fn(&mut TimePix3Driver);
        fn run(name: &'static str, setup: Setup) -> (bool, Option<serde_json::Value>, String, i32) {
            let (url, rx) = capturing_serval();
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&url)),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            setup(&mut det);
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
            h.trace().set_trace_mask(
                Some(name),
                TraceMask::ERROR | TraceMask::IO_DRIVER | TraceMask::FLOW,
            );
            let (ok, format) = h
                .with_driver_blocking::<TimePix3Driver, _>(|d| {
                    (d.file_writer().is_ok(), d.int(d.p.img_format, 0))
                })
                .unwrap();
            let put = rx
                .try_iter()
                .find(|(line, _)| line.starts_with("PUT /server/destination "))
                .map(|(_, body)| serde_json::from_str(&body).unwrap());
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            std::mem::forget(runtime);
            (ok, put, text, format)
        }
        fn enable(d: &mut TimePix3Driver, write: usize, base: usize) {
            let b = &mut d.ad.port_base;
            b.set_string_param(base, 0, "file:/data").unwrap();
            b.set_int32_param(write, 0, 1).unwrap();
        }

        let (ok, put, text, _) = run("TPX_FW_NONE", |_| {});
        assert!(ok);
        assert_eq!(put, None);
        assert!(text.contains(&adlog::log(
            adlog::FILE_WRITER,
            "Configuring file writer channels"
        )));
        assert!(text.contains(&adlog::log(
            adlog::FILE_WRITER,
            "No channels are enabled. Skipping file writer configuration."
        )));

        let (ok, put, _, _) = run("TPX_FW_SLOT", |d| {
            let p = d.p;
            enable(d, p.write_raw1, p.raw1_base);
            enable(d, p.write_img1, p.img1_base);
            enable(d, p.write_prv_img1, p.prv_img1_base);
        });
        assert!(ok);
        let put = put.unwrap();
        assert_eq!(put["Raw"][0], serde_json::Value::Null);
        assert_eq!(put["Raw"][1]["Base"], json!("file:/data"));
        assert_eq!(put["Image"][0], serde_json::Value::Null);
        assert_eq!(put["Image"][1]["Base"], json!("file:/data"));
        assert_eq!(put["Preview"]["ImageChannels"][0], serde_json::Value::Null);
        assert_eq!(
            put["Preview"]["ImageChannels"][1]["Base"],
            json!("file:/data")
        );
        assert!(put["Preview"].get("SamplingMode").is_some());

        let (ok, put, text, _) = run("TPX_FW_REFUSED", |d| {
            let p = d.p;
            enable(d, p.write_img1, p.img1_base);
            d.ad.port_base.set_int32_param(p.img1_format, 0, 9).unwrap();
        });
        assert!(!ok);
        assert_eq!(put, None);
        assert!(text.contains(&adlog::err(
            adlog::FILE_WRITER,
            "Failed to configure image channel 1"
        )));

        let (ok, _, text, _) = run("TPX_FW_RAW_SPLIT", |d| {
            let p = d.p;
            enable(d, p.write_raw, p.raw_base);
            d.ad.port_base
                .set_int32_param(p.raw_split_strategy, 0, 9)
                .unwrap();
        });
        assert!(!ok);
        assert!(text.contains(&adlog::err(
            adlog::CONFIGURE_RAW_CHANNEL,
            "Invalid split strategy index: 9"
        )));
        assert!(text.contains(&adlog::err(
            adlog::FILE_WRITER,
            "Failed to configure raw channel 0"
        )));

        let (ok, put, text, _) = run("TPX_FW_HST", |d| {
            let p = d.p;
            enable(d, p.write_prv_hst, p.prv_hst_base);
            d.ad.port_base
                .set_int32_param(p.prv_hst_format, 0, 9)
                .unwrap();
        });
        assert!(!ok);
        assert_eq!(put, None);
        assert!(text.contains(&adlog::err(
            adlog::CONFIGURE_HISTOGRAM_CHANNEL,
            "Invalid histogram format index: 9"
        )));
        assert!(text.contains(&adlog::err(
            adlog::FILE_WRITER,
            "Failed to configure histogram channel"
        )));

        // A Medipix3 tcp image channel: tiff becomes jsonimage, the file
        // keys stay, and the empty list sends the default thresholds.
        let (ok, put, text, format) = run("TPX_FW_IMG_TCP", |d| {
            let p = d.p;
            d.shared.note_family(DetectorFamily::Mpx3);
            enable(d, p.write_img, p.img_base);
            let b = &mut d.ad.port_base;
            b.set_string_param(p.img_base, 0, "tcp://localhost:8451")
                .unwrap();
            b.set_string_param(p.img_ths, 0, "").unwrap();
            b.set_int32_param(p.img_format, 0, 0).unwrap();
        });
        assert!(ok);
        assert_eq!(format, 3);
        let put = put.unwrap();
        assert_eq!(put["Image"][0]["Format"], json!("jsonimage"));
        assert_eq!(put["Image"][0]["FilePattern"], json!(""));
        assert_eq!(
            put["Image"][0]["StopMeasurementOnDiskLimit"],
            json!("false")
        );
        assert_eq!(
            put["Image"][0]["Thresholds"],
            json!([0, 1, 2, 3, 4, 5, 6, 7])
        );
        assert!(text.contains(&adlog::log(
            adlog::CONFIGURE_IMAGE_CHANNEL,
            "TCP channel Image: format index 0 invalid for tcp; using jsonimage (3)"
        )));

        let (ok, _, text, _) = run("TPX_FW_IMG_THS", |d| {
            let p = d.p;
            d.shared.note_family(DetectorFamily::Mpx3);
            enable(d, p.write_prv_img1, p.prv_img1_base);
            d.ad.port_base
                .set_string_param(p.prv_img1_ths, 0, "1,x")
                .unwrap();
        });
        assert!(!ok);
        assert!(text.contains("Invalid threshold list for channel param"));
        assert!(text.contains(&adlog::err(
            adlog::FILE_WRITER,
            "Failed to configure preview image channel 1"
        )));

        let (ok, _, text, _) = run("TPX_FW_SAMPLING", |d| {
            let p = d.p;
            enable(d, p.write_prv_img, p.prv_img_base);
            d.ad.port_base
                .set_int32_param(p.prv_sampling_mode, 0, 2)
                .unwrap();
        });
        assert!(!ok);
        assert!(text.contains(&adlog::err(
            adlog::CONFIGURE_PREVIEW_SETTINGS,
            "Invalid sampling mode index: 2"
        )));
        assert!(text.contains(&adlog::err(
            adlog::FILE_WRITER,
            "Failed to configure preview settings"
        )));
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

    /// C keeps a refused set as the status and runs the action, whose status
    /// replaces it (ADTimePix.cpp:508, :533-536, :1093-1097).
    #[test]
    fn a_refused_set_still_runs_the_action() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_SET_ACTION",
            Arc::new(ServalHttp::new(&fake_serval("nothing fails"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let ad = det.ad.params;
        det.write_float64(&mut AsynUser::new(ad.acquire_time).with_addr(20), 0.5)
            .unwrap();
        det.ad
            .port_base
            .set_int32_param(ad.num_images, 0, 7)
            .unwrap();
        det.write_int32(&mut AsynUser::new(ad.image_mode).with_addr(20), 0)
            .unwrap();
        assert_eq!(det.int(ad.num_images, 0), 1);
    }

    /// C's port has 14 addresses (ADTimePix.h:923), so a record on
    /// address 13 has a parameter list.
    #[test]
    fn the_port_has_fourteen_addresses() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MAX_ADDR",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.ad.port_base.set_int32_param(p.adjust, 13, 1).unwrap();
        assert!(det.ad.port_base.set_int32_param(p.adjust, 14, 1).is_err());
        assert_eq!(tasks::new_outputs().len(), 14);
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
            &fake_serval_with(
                "\0",
                &[("GET /measurement ", r#"{"Info": {"Status": "DA_IDLE"}}"#)],
            ),
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

    /// The stop transaction posts every `GET /measurement` metric with its
    /// param's type, and the status with its pipeline index
    /// (acquisition_runtime.cpp:216-226, acquire.cpp:40-98).
    #[test]
    fn a_measurement_reply_posts_each_metric_with_its_param_type() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_MSMT_TYPES",
            Arc::new(ServalHttp::new(&fake_serval_with(
                "\0",
                &[(
                    "GET /measurement ",
                    r#"{"Info": {"PixelEventRate": 1200, "TdcEventRate": 3, "Tdc2EventRate": 4,
                        "StartDateTime": 1700000000123, "ElapsedTime": 2.5, "TimeLeft": 0.5,
                        "FrameCount": 9, "DroppedFrames": 1, "Status": "DA_STOPPED"}}"#,
                )],
            ))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let (p, acquire) = (det.p, det.ad.params.acquire);
        det.write_int32(&mut AsynUser::new(acquire), 1).unwrap();
        det.write_int32(&mut AsynUser::new(acquire), 0).unwrap();
        assert_eq!(det.int(p.pel_rate, 0), 1200);
        assert_eq!(det.int(p.tdc1_rate, 0), 3);
        assert_eq!(det.int(p.tdc2_rate, 0), 4);
        assert_eq!(
            det.ad.port_base.get_int64_param(p.start_time, 0).unwrap(),
            1_700_000_000_123
        );
        assert_eq!(det.dbl(p.elapsed_time, 0), 2.5);
        assert_eq!(det.dbl(p.time_left, 0), 0.5);
        assert_eq!(det.int(p.frame_count, 0), 9);
        assert_eq!(det.int(p.dropped_frames, 0), 1);
        assert_eq!(det.text(p.status, 0), "DA_STOPPED");
        assert_eq!(det.int(p.pipeline_state, 0), 3);
    }

    /// A `/measurement/start` refused with "Address already in use" rotates
    /// the preview ports and retries once; a second refusal fails with C's
    /// port message (acquire.cpp:392-409).
    #[test]
    fn a_start_port_conflict_rotates_and_retries_once() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        use std::io::{BufRead, BufReader, Read, Write};
        use std::sync::atomic::{AtomicUsize, Ordering};
        // Serval refusing the first `refusals` starts; returns the trace,
        // StatusMessage and the number of starts.
        fn run(name: &'static str, refusals: usize) -> (String, String, usize) {
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let url = format!("http://{}", listener.local_addr().unwrap());
            let starts = Arc::new(AtomicUsize::new(0));
            let seen = starts.clone();
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
                    let (status, reply) = if request.starts_with("GET /measurement/start") {
                        if seen.fetch_add(1, Ordering::SeqCst) < refusals {
                            ("500 Internal Server Error", "bind: Address already in use")
                        } else {
                            ("200 OK", "{}")
                        }
                    } else {
                        ("200 OK", "{}")
                    };
                    let _ = reader.get_mut().write_all(
                        format!(
                            "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                            reply.len()
                        )
                        .as_bytes(),
                    );
                }
            });
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&url)),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let (acquire, message) = (det.ad.params.acquire, det.ad.params.status_message);
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
                .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::WARNING);
            let _ = h.write_int32_blocking(acquire, 0, 1);
            let message = h
                .with_driver_blocking::<TimePix3Driver, _>(move |d| d.text(message, 0))
                .unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            std::mem::forget(runtime);
            (text, message, starts.load(Ordering::SeqCst))
        }
        let warn = adlog::warn(
            adlog::ACQUIRE_START,
            "measurement/start failed with port conflict; rotating preview ports and retrying once",
        );

        let (text, _, starts) = run("TPX_START_RETRY", 1);
        assert_eq!(starts, 2);
        assert!(text.contains(&warn), "{text}");
        assert!(
            !text.contains("acquireStart GET /measurement/start"),
            "{text}"
        );

        let (text, message, starts) = run("TPX_START_CONFLICT", 2);
        assert_eq!(starts, 2);
        assert!(text.contains(&warn), "{text}");
        assert_eq!(
            message,
            "Preview TCP port in use (Serval did not release a prior listener)"
        );
    }

    /// A refused tcp:// port prints C `parseTcpPath`'s ERR line before the
    /// path check's own (serval_stream.cpp:672-681, ADTimePix.cpp:346-354).
    #[test]
    fn a_bad_tcp_port_logs_parse_tcp_path_errors() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_PORT_PARSE";
        let (url, _rx) = capturing_serval();
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&url)),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
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
        let results = h
            .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                let channel = CHANNEL_PATHS
                    .iter()
                    .find(|c| (c.base)(&p) == p.prv_img_base)
                    .unwrap();
                ["tcp://h:x1", "tcp://h:0", "tcp://listen@h: 8089junk"].map(|base| {
                    d.ad.port_base
                        .set_string_param(p.prv_img_base, 0, base)
                        .unwrap();
                    d.check_channel_path(channel).is_ok()
                })
            })
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        std::mem::forget(runtime);
        assert_eq!(results, [false, false, true]);
        for line in [
            adlog::err(adlog::PARSE_TCP_PATH, "Failed to parse port: x1"),
            adlog::err(
                adlog::CHECK_PRV_IMG_PATH,
                "Failed to parse PrvImg TCP path: tcp://h:x1",
            ),
            adlog::err(
                adlog::PARSE_TCP_PATH,
                "Invalid port number: 0 (must be 1-65535)",
            ),
            adlog::log(
                adlog::CHECK_PRV_IMG_PATH,
                "Parsed PrvImg TCP path: host=h, port=8089",
            ),
        ] {
            assert!(text.contains(&line), "{line}\n{text}");
        }
    }

    /// C syncs and connects the image channels before it parses the
    /// histogram path (acquire.cpp:430-501), so the start's Parsed line
    /// precedes the bad PrvHst path's ERR line.
    #[test]
    fn a_start_parses_prv_hst_after_the_image_workers() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_HST_ORDER";
        let shared = Arc::new(Shared::new());
        serve_workers(&shared, true);
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&fake_serval("/server/destination"))),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for (reason, v) in [
            (p.write_prv_img, 1),
            (p.write_prv_hst, 1),
            (p.prv_hst_format, 4),
            (p.prv_hst_accumulation_enable, 1),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, v).unwrap();
        }
        det.set_text(p.prv_img_base, UNBOUND_TCP).unwrap();
        det.set_text(p.prv_hst_base, "tcp://h:0").unwrap();
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
        let (started, status) = h
            .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                let acquire = d.ad.params.acquire;
                let started = d.write_int32(&mut AsynUser::new(acquire), 1).is_ok();
                (started, d.text(d.ad.params.status_message, 0))
            })
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        std::mem::forget(runtime);
        assert!(!started);
        assert_eq!(status, "Histogram TCP path is invalid");
        let parsed = adlog::log(
            adlog::CHECK_PRV_IMG_PATH,
            "Parsed PrvImg TCP path: host=192.0.2.1, port=8089",
        );
        let err = adlog::err(
            adlog::PARSE_TCP_PATH,
            "Invalid port number: 0 (must be 1-65535)",
        );
        // ensurePreviewTcpPortsFree parses and syncs, then the start syncs
        // and only then parses PrvHst.
        let order: Vec<_> = text
            .lines()
            .filter_map(|l| {
                (l.ends_with(&parsed).then_some("parsed")).or(l.ends_with(&err).then_some("err"))
            })
            .collect();
        assert_eq!(order, ["err", "parsed", "parsed", "err"], "{text}");
    }

    /// An enabled tcp:// channel whose port is bound locally moves to the
    /// next free port, logged, and the destination is re-sent
    /// (acquire.cpp:229-293).
    #[test]
    fn a_busy_preview_port_moves_to_the_next_free_one() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_PORT_BUSY";
        let held = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let busy = held.local_addr().unwrap().port();
        let (url, rx) = capturing_serval();
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&url)),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        let b = &mut det.ad.port_base;
        b.set_int32_param(p.write_prv_img, 0, 1).unwrap();
        b.set_string_param(p.prv_img_base, 0, format!("tcp://127.0.0.1:{busy}"))
            .unwrap();
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
        h.trace().set_trace_mask(Some(name), TraceMask::IO_DRIVER);
        let (ok, base) = h
            .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                (
                    d.ensure_preview_tcp_ports_free(false),
                    d.text(p.prv_img_base, 0),
                )
            })
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        std::mem::forget(runtime);
        assert!(ok);
        let (host, moved) = crate::stream::parse_tcp_path(&base).unwrap();
        assert_eq!(base, format!("tcp://listen@{host}:{moved}"));
        assert!(moved > busy);
        assert!(text.contains(&adlog::log(
            adlog::ENSURE_PREVIEW_TCP_PORTS_FREE,
            &format!("Preview TCP port {busy} -> {moved} (port in use)")
        )));
        assert!(
            rx.try_iter()
                .any(|(line, _)| line.starts_with("PUT /server/destination "))
        );
        drop(held);
    }

    /// A `GET /measurement` reply that fails C `parseStatusResponse` is
    /// warned about by the start and refused by the stop
    /// (acquire.cpp:363-369, acquisition_runtime.cpp:226-235).
    #[test]
    fn an_invalid_measurement_reply_is_reported_by_start_and_stop() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_MSMT_INVALID";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&fake_serval_with(
                "\0",
                &[("GET /measurement ", "[]")],
            ))),
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
        h.trace()
            .set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::WARNING);
        h.write_int32_blocking(acquire, 0, 1).unwrap();
        // An unconfirmed stop faults (acquisition_runtime.cpp:245-252).
        assert!(h.write_int32_blocking(acquire, 0, 0).is_err());
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let why = "measurement JSON root is not an object";
        for line in [
            adlog::warn(
                adlog::ACQUIRE_START,
                &format!(
                    "Failed to validate measurement status: {why}, continuing after prior stop"
                ),
            ),
            adlog::err(
                adlog::STOP_ACQUISITION,
                &format!("transactional stop: invalid measurement response: {why}"),
            ),
        ] {
            assert!(text.contains(&line), "{line}: {text:?}");
        }
        std::mem::forget(runtime);
    }

    /// C `getServer` (serval_http.cpp:1611-1653): the wrapped Serval 4.x
    /// reply is read, an unset destination clears the readbacks, and a
    /// refused reply keeps them and reports HttpCode, WriteMsg and its line.
    #[test]
    fn the_server_destination_reply_is_validated_as_c() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        type Routes = &'static [(&'static str, &'static str)];
        // (port, failing route, routes, ok, readbacks, HttpCode, WriteMsg)
        type Case = (
            &'static str,
            &'static str,
            Routes,
            bool,
            [i32; 7],
            i32,
            &'static str,
        );
        let cases: [Case; 4] = [
            (
                "TPX_DEST_WRAPPED",
                "\0",
                &[(
                    "GET /server/destination",
                    r#"{"Destination":{"Raw":[{}],"Image":[{},{}],
                       "Preview":{"ImageChannels":[{}],"HistogramChannels":[{}]}}}"#,
                )],
                true,
                [1, 0, 1, 1, 1, 0, 1],
                -1,
                "",
            ),
            (
                "TPX_DEST_UNSET",
                "/server/destination",
                &[("GET /server/destination", "Destination is not set")],
                true,
                [0; 7],
                -1,
                "",
            ),
            (
                "TPX_DEST_BAD",
                "\0",
                &[("GET /server/destination", r#"{"Raw":[{}],"Image":{}}"#)],
                false,
                [1; 7],
                200,
                "server destination Image field is not an array",
            ),
            (
                "TPX_DEST_500",
                "/server/destination",
                &[("GET /server/destination", "boom")],
                false,
                [1; 7],
                500,
                "boom",
            ),
        ];
        for (name, fail, routes, ok, reads, code, msg) in cases {
            let url = fake_serval_with(fail, routes);
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&url)),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            let params = [
                p.write_raw_read,
                p.write_raw1_read,
                p.write_img_read,
                p.write_img1_read,
                p.write_prv_img_read,
                p.write_prv_img1_read,
                p.write_prv_hst_read,
            ];
            for param in params {
                det.ad.port_base.set_int32_param(param, 0, 1).unwrap();
            }
            det.ad
                .port_base
                .set_int32_param(p.http_code, 0, -1)
                .unwrap();
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
            h.trace().set_trace_mask(Some(name), TraceMask::ERROR);
            let (result, got, got_code, got_msg) = h
                .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                    let result = d.get_server();
                    let got: Vec<i32> = params.iter().map(|&r| d.int(r, 0)).collect();
                    let base = &d.ad.port_base;
                    (
                        result.is_ok(),
                        got,
                        base.get_int32_param(p.http_code, 0).unwrap(),
                        d.text(p.write_msg, 0),
                    )
                })
                .unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            assert_eq!(
                (result, got.as_slice(), got_code, got_msg.as_str()),
                (ok, reads.as_slice(), code, msg),
                "{name}"
            );
            let line = match code {
                200 => adlog::err(adlog::GET_SERVER, &format!("getServer: {msg}")),
                500 => adlog::http_failure_reply(
                    "getServer GET /server/destination",
                    "GET",
                    &format!("{url}/server/destination"),
                    500,
                    "boom",
                ),
                _ => String::new(),
            };
            if line.is_empty() {
                assert!(!text.contains("ERROR"), "{name}: {text:?}");
            } else {
                assert!(text.contains(&line), "{name}: {line}: {text:?}");
            }
            std::mem::forget(runtime);
        }
    }

    /// C `initAcquisition`'s failure paths (serval_http.cpp:2449-2493,
    /// :2526-2580, :2624-2648): what each refusal publishes and logs.
    #[test]
    fn init_acquisition_failures_are_published_as_c() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        type Routes = &'static [(&'static str, &'static str)];
        type Setup = fn(&mut TimePix3Driver);
        // (DetConnected, HttpCode, WriteMsg, StatusMessage, TriggerMode, PixelDepth, GainMode)
        type Seen = (i32, i32, String, String, i32, i32, i32);
        fn run(
            name: &'static str,
            fail: &'static str,
            routes: Routes,
            mpx3: bool,
            setup: Setup,
        ) -> (bool, Seen, String, String) {
            let url = fake_serval_with(fail, routes);
            let shared = Arc::new(Shared::new());
            if mpx3 {
                shared.note_family(DetectorFamily::Mpx3);
            }
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det =
                TimePix3Driver::new(name, Arc::new(ServalHttp::new(&url)), shared, 0, cmd_tx)
                    .unwrap();
            let p = det.p;
            det.ad
                .port_base
                .set_int32_param(p.det_connected, 0, 7)
                .unwrap();
            det.ad
                .port_base
                .set_int32_param(p.http_code, 0, -1)
                .unwrap();
            det.ad
                .port_base
                .set_int32_param(p.pixel_depth, 0, 24)
                .unwrap();
            setup(&mut det);
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
            h.trace().set_trace_mask(Some(name), TraceMask::ERROR);
            let (ok, seen) = h
                .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                    let ok = d.init_acquisition().is_ok();
                    let ad = d.ad.params;
                    (
                        ok,
                        (
                            d.int(p.det_connected, 0),
                            d.int(p.http_code, 0),
                            d.text(p.write_msg, 0),
                            d.text(ad.status_message, 0),
                            d.int(ad.trigger_mode, 0),
                            d.int(p.pixel_depth, 0),
                            d.int(p.gain_mode, 0),
                        ),
                    )
                })
                .unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            std::mem::forget(runtime);
            (ok, seen, text, format!("{url}/detector/config"))
        }
        let s = String::from;
        const GET: &str = "GET /detector/config ";
        const PUT: &str = "PUT /detector/config ";

        let (ok, seen, text, url) =
            run("TPX_INIT_GET", GET, &[(GET, "no detector")], false, |_| {});
        assert!(!ok);
        assert_eq!(
            seen,
            (
                0,
                500,
                s("no detector"),
                s("Failed to read detector configuration"),
                0,
                24,
                0
            )
        );
        let line = adlog::http_failure_reply(
            "initAcquisition GET /detector/config",
            "GET",
            &url,
            500,
            "no detector",
        );
        assert!(text.contains(&line), "{line}: {text:?}");

        let (ok, seen, text, _) = run("TPX_INIT_ROOT", "\0", &[(GET, "[]")], false, |_| {});
        let msg = "Invalid detector configuration response: \
                   detector configuration JSON root is not an object";
        assert!(!ok);
        assert_eq!(seen, (1, 200, s(msg), s(msg), 0, 24, 0));
        let line = adlog::err(adlog::INIT_ACQUISITION, &format!("initAcquisition: {msg}"));
        assert!(text.contains(&line), "{line}: {text:?}");

        const DET: (&str, &str) = (
            "GET /detector ",
            r#"{"Info":{"MpxType":6,"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256},"Config":{}}"#,
        );
        // A known family skips getDetector, whose readback would overwrite
        // the setpoint (serval_http.cpp:2477-2479).
        let (ok, seen, _, _) = run("TPX_INIT_BIAS", "\0", &[DET], false, |d| {
            d.shared.note_family(DetectorFamily::Tpx3);
            d.ad.port_base
                .set_int32_param(d.p.bias_enable, 0, 2)
                .unwrap();
        });
        assert!(!ok);
        assert_eq!(
            seen,
            (1, 400, s("BiasEnabled must be 0 or 1"), s(""), 0, 24, 0)
        );

        let (ok, seen, _, _) = run(
            "TPX_INIT_TRIG",
            "\0",
            &[(PUT, "accepted"), DET],
            false,
            |d| {
                let mode = d.ad.params.trigger_mode;
                d.ad.port_base.set_int32_param(mode, 0, 9).unwrap();
            },
        );
        assert!(ok);
        assert_eq!(seen, (1, 200, s("accepted"), s(""), 0, 24, 0));

        let (ok, seen, text, _) = run("TPX_INIT_MPX3_TRIG", "\0", &[], true, |d| {
            let mode = d.ad.params.trigger_mode;
            d.ad.port_base.set_int32_param(mode, 0, 5).unwrap();
            d.ad.port_base
                .set_int32_param(d.p.both_counters, 0, 1)
                .unwrap();
        });
        assert!(!ok);
        assert_eq!(seen, (1, 400, s(MPX3_TRIGGER_MSG), s(""), 5, 24, 0));
        let line = adlog::err(adlog::INIT_ACQUISITION, MPX3_TRIGGER_MSG);
        assert!(text.contains(&line), "{line}: {text:?}");

        // serval_http.cpp:2477-2479: an Unknown family is read from
        // /detector before the MPX3 checks.
        let (ok, seen, _, _) = run(
            "TPX_INIT_UNKNOWN",
            "\0",
            &[(
                "GET /detector ",
                r#"{"Info":{"MpxType":5,"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256},"Config":{}}"#,
            )],
            false,
            |d| {
                let mode = d.ad.params.trigger_mode;
                d.ad.port_base.set_int32_param(mode, 0, 5).unwrap();
                d.ad.port_base
                    .set_int32_param(d.p.both_counters, 0, 1)
                    .unwrap();
            },
        );
        assert!(!ok);
        assert_eq!(seen, (1, 400, s(MPX3_TRIGGER_MSG), s(""), 5, 12, 1));

        let (ok, seen, text, url) = run(
            "TPX_INIT_PUT",
            PUT,
            &[
                (PUT, "bad config"),
                (GET, r#"{"PixelDepth":"6","GainMode":"LGM"}"#),
            ],
            true,
            |_| {},
        );
        assert!(!ok);
        assert_eq!(
            seen,
            (
                1,
                500,
                s("bad config"),
                s("Failed to upload detector configuration"),
                0,
                6,
                2
            )
        );
        let line = adlog::http_failure_reply(
            "initAcquisition PUT /detector/config",
            "PUT",
            &url,
            500,
            "bad config",
        );
        assert!(text.contains(&line), "{line}: {text:?}");
    }

    /// A Medipix3 detector gets its own settings, with the depth normalised
    /// and published back (serval_http.cpp:2583-2617).
    #[test]
    fn a_medipix3_init_acquisition_sends_the_medipix3_settings() {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let (tx, rx) = std::sync::mpsc::channel::<String>();
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let (mut request, mut length) = (String::new(), 0);
                let _ = reader.read_line(&mut request);
                request = crate::http::without_serval_params(&request);
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
                if request.starts_with("PUT ") {
                    let _ = tx.send(String::from_utf8_lossy(&body).into_owned());
                }
                let reply = r#"{"Tdc":["P0","P0"],"PeriphClk80":true}"#;
                let _ = reader.get_mut().write_all(
                    format!(
                        "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                        reply.len()
                    )
                    .as_bytes(),
                );
            }
        });
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Mpx3);
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MPX3_BODY",
            Arc::new(ServalHttp::new(&url)),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for (reason, v) in [
            (p.both_counters, 1),
            (p.pixel_depth, 24),
            (p.gain_mode, 2),
            (p.colour, 1),
            (p.counter_select_out, 1),
            (p.i_delay2, 9),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, v).unwrap();
        }
        det.init_acquisition().unwrap();
        let body: serde_json::Value = serde_json::from_str(&rx.recv().unwrap()).unwrap();
        assert!(body.get("Tdc").is_none() && body.get("PeriphClk80").is_none());
        assert_eq!(
            [
                &body["BothCounters"],
                &body["PixelDepth"],
                &body["GainMode"],
                &body["Colour"],
                &body["ChargeSumming"],
                &body["CounterSelectOut"],
                &body["IDelayConfig"],
            ],
            [
                &json!(true),
                &json!(12),
                &json!("LGM"),
                &json!(true),
                &json!(false),
                &json!(1),
                &json!([0, 0, 9, 0]),
            ]
        );
        assert_eq!(det.int(p.pixel_depth, 0), 12);
        assert_eq!(
            det.text(det.ad.params.status_message, 0),
            MPX3_PIXEL_DEPTH_MSG
        );
    }

    /// Every calibration file goes through C `resolveCalibrationFile`: a
    /// path outside the root or a non-basename is refused before Serval or
    /// the filesystem sees it, and an accepted one travels resolved
    /// (ADTimePix.cpp:153-169, mask_io.cpp:546, :594, :667,
    /// serval_http.cpp:1675).
    #[test]
    fn calibration_files_resolve_inside_the_root() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_CALIB_RESOLVE";
        let tmp = crate::calibration::canonical(&std::env::temp_dir())
            .unwrap()
            .join(format!("{name}-{}", std::process::id()));
        let (root, outside) = (tmp.join("root"), tmp.join("outside"));
        std::fs::create_dir_all(&root).unwrap();
        std::fs::create_dir_all(&outside).unwrap();
        std::fs::write(root.join("a.dacs"), b"x").unwrap();
        std::fs::write(outside.join("a.dacs"), b"x").unwrap();
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&fake_serval_with(
                "\0",
                &[("GET /config/load", "loaded")],
            ))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        det.configure_calibration(root.to_str().unwrap()).unwrap();
        let p = det.p;
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port(name).unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let h = runtime.port_handle().clone();
        let log = tmp.join("trace");
        let file = std::fs::File::create(&log).unwrap();
        h.trace().set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        h.trace().set_trace_mask(Some(name), TraceMask::ERROR);
        let upload = |dir: String, file: &'static str| {
            h.with_driver_blocking::<TimePix3Driver, _>(move |d| {
                d.set_text(p.da_cs_file_path, &dir).unwrap();
                d.set_text(p.da_cs_file_name, file).unwrap();
                d.set_text(p.write_msg, "").unwrap();
                let ok = d
                    .write_int32(&mut AsynUser::new(p.write_da_cs_file), 1)
                    .is_ok();
                (ok, d.text(p.write_msg, 0))
            })
            .unwrap()
        };
        let dir = |d: &std::path::Path| format!("{}/", d.display());
        assert_eq!(
            upload(dir(&outside), "a.dacs"),
            (
                false,
                "Calibration path blocked: calibration directory is outside the approved root"
                    .to_string()
            )
        );
        assert_eq!(
            upload(dir(&root), "../outside/a.dacs"),
            (
                false,
                "Calibration path blocked: calibration filename must be a basename".to_string()
            )
        );
        assert_eq!(upload(dir(&root), "a.dacs"), (true, "loaded".to_string()));
        let text = std::fs::read_to_string(&log).unwrap();
        for line in [
            adlog::err(
                adlog::RESOLVE_CALIBRATION_FILE,
                &format!(
                    "Calibration path blocked: calibration directory is outside the approved \
                     root (directory=\"{}\", name=\"a.dacs\")",
                    dir(&outside)
                ),
            ),
            adlog::err(
                adlog::UPLOAD_DACS,
                "uploadDACS: calibration path validation failed; request not sent",
            ),
        ] {
            assert!(text.contains(&line), "{line}: {text:?}");
        }
        let _ = std::fs::remove_dir_all(&tmp);
        std::mem::forget(runtime);
    }

    /// A mask flag write only stores the flag; the operation runs on the
    /// MaskBPC read over its nElements, with C's FLOW line, WriteMsg and
    /// MaskBPC status (mask_io.cpp:95-130, :240-242, mask_geometry.cpp).
    #[test]
    fn a_mask_operation_runs_on_the_mask_waveform_read_as_c() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        let name = "TPX_MASK_READ";
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        for (reason, v) in [
            (p.pix_count, 65536),
            (p.row_len, 1),
            (p.number_of_chips, 1),
            (p.number_of_rows, 256),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, v).unwrap();
        }
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

        h.write_int32_blocking(p.mask_on_off_pel, 0, 3).unwrap();
        h.write_int32_blocking(p.mask_reset, 0, 1).unwrap();
        let read = |n: usize| {
            h.with_driver_blocking::<TimePix3Driver, _>(move |d| {
                let mut buf = vec![-1i32; n];
                let r = d
                    .read_int32_array(&AsynUser::new(d.p.mask_bp_c), &mut buf)
                    .map_err(|e| e.message());
                let status = d.ad.port_base.params.get_param_status(d.p.mask_bp_c, 0);
                (r, buf, d.text(d.p.write_msg, 0), status.unwrap().0)
            })
            .unwrap()
        };
        let (r, buf, _, status) = read(65536 + 2);
        assert_eq!(r, Ok(65538));
        assert!(buf[..65536].iter().all(|&v| v == 3), "reset assigns OnOff");
        assert_eq!(&buf[65536..], &[-1, -1], "VAL past the image is kept");
        assert_eq!(status, AsynStatus::Success);
        assert_eq!(
            h.with_driver_blocking::<TimePix3Driver, _>(|d| d.int(d.p.mask_reset, 0))
                .unwrap(),
            1,
            "C leaves the flag to the db"
        );

        let (r, _, msg, status) = read(100);
        assert_eq!(
            r,
            Err("asynPortDriver:readArray not implemented".to_string())
        );
        assert_eq!(msg, "mask waveform is smaller than detector geometry");
        assert_eq!(status, AsynStatus::Error);

        h.write_int32_blocking(p.mask_reset, 0, 0).unwrap();
        h.write_int32_blocking(p.mask_rectangle, 0, 1).unwrap();
        h.write_int32_blocking(p.mask_size_x, 0, -1).unwrap();
        let (r, _, msg, _) = read(65536);
        assert!(r.is_err());
        assert_eq!(msg, "invalid mask geometry or operation");

        // The draw edits the record's VAL, not a driver copy.
        h.write_int32_blocking(p.mask_size_x, 0, 1).unwrap();
        h.write_int32_blocking(p.mask_size_y, 0, 1).unwrap();
        h.write_int32_blocking(p.mask_on_off_pel, 0, 0).unwrap();
        let (r, buf, _, _) = read(65536);
        assert_eq!(r, Ok(65536));
        assert_eq!(&buf[..2], &[-2, -1], "unmasking clears bit 0 of VAL");
        assert!(buf[1..].iter().all(|&v| v == -1));

        h.write_int32_blocking(p.mask_rectangle, 0, 0).unwrap();
        let (r, _, _, status) = read(65536);
        assert_eq!(r, Ok(65536));
        assert_eq!(status, AsynStatus::Success);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for line in [
            "MaskBPC: reset (waveform nElements=65538)",
            "MaskBPC: reset (waveform nElements=100)",
            "MaskBPC: rectangle (waveform nElements=65536)",
            "MaskBPC: no draw op (nElements=65536)",
        ] {
            let line = adlog::log(adlog::READ_INT32_ARRAY, line);
            assert!(text.contains(&line), "{line}: {text:?}");
        }
        std::mem::forget(runtime);
    }

    /// MaskPel reads the family's mask bit through bpcImageByteOffsets: an
    /// MPX3 through its per-chip Layout, an undocumented family refused, and
    /// every mapping failure with C's WriteMsg (mask_io.cpp:131-151, :318-422).
    #[test]
    fn mask_pel_maps_the_bpc_through_bpc_image_byte_offsets_as_c() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        let name = "TPX_MASK_PEL";
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        let p = det.p;
        det.ad.port_base.set_int32_param(p.mask_pel, 0, 1).unwrap();
        let read = |det: &mut TimePix3Driver| {
            let mut buf = vec![0i32; 65536];
            let r = det
                .read_int32_array(&AsynUser::new(p.mask_bp_c), &mut buf)
                .map(|_| buf)
                .map_err(|e| e.message());
            (r, det.text(p.write_msg, 0))
        };

        let (r, msg) = read(&mut det);
        assert!(r.is_err());
        assert_eq!(
            msg,
            "UNKNOWN mask read unavailable: pixel mask encoding is not documented"
        );
        assert_eq!(
            det.ad
                .port_base
                .params
                .get_param_status(p.mask_bp_c, 0)
                .unwrap()
                .0,
            AsynStatus::Error
        );

        shared.note_family(DetectorFamily::Mpx3);
        let mut bpc = vec![0u8; 65536 * 2];
        bpc[(4 * 256 + 3) * 2 + 1] = 1;
        std::fs::write(dir.join("cal.bpc"), &bpc).unwrap();
        // readBPCfile runs first, and its size needs PixCount.
        det.ad.port_base.set_int32_param(p.pix_count, 0, 0).unwrap();
        let (r, msg) = read(&mut det);
        assert_eq!(r, Err("asynPortDriver:readArray not implemented".into()));
        assert_eq!(msg, "Invalid BPC geometry: pixels=0 bytes/pixel=2 slices=1");
        det.ad
            .port_base
            .set_int32_param(p.pix_count, 0, 65536)
            .unwrap();
        let (r, msg) = read(&mut det);
        assert_eq!(r, Err("asynPortDriver:readArray not implemented".into()));
        assert_eq!(msg, "Invalid BPC image geometry");

        for (reason, v) in [
            (p.row_len, 1),
            (p.number_of_chips, 1),
            (p.number_of_rows, 256),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, v).unwrap();
        }
        let (r, msg) = read(&mut det);
        assert!(r.is_err());
        assert_eq!(
            msg,
            format!("MPX3 chip 0 layout unavailable: {NLOHMANN_EMPTY_INPUT}")
        );
        for (layout, want) in [
            (
                r#"{"Chip":1,"X":0,"Y":0,"Orientation":"LtRTtB"}"#,
                "incomplete chip layout",
            ),
            (
                r#"{"Chip":0,"X":0,"Y":0,"Orientation":"Up"}"#,
                "unsupported or out-of-range chip layout",
            ),
        ] {
            det.ad
                .port_base
                .set_string_param(p.layout, 0, layout)
                .unwrap();
            let (r, msg) = read(&mut det);
            assert!(r.is_err());
            assert_eq!(msg, format!("MPX3 chip 0 layout unavailable: {want}"));
        }

        det.ad
            .port_base
            .set_string_param(
                p.layout,
                0,
                r#"{"Chip":0,"X":0,"Y":0,"Orientation":"LtRTtB"}"#,
            )
            .unwrap();
        let (r, _) = read(&mut det);
        let buf = r.unwrap();
        assert_eq!(buf[4 * 256 + 3], 1 << 1);
        assert_eq!(buf.iter().filter(|&&v| v != 0).count(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The BPC waveform is the family's packed pixel, a masked one marked by
    /// bit 8 (TPX3) or bit 16 (MPX3) (mask_io.cpp:209-237).
    #[test]
    fn the_bpc_waveform_is_the_packed_pixel_per_family() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        let name = "TPX_BPC_READ";
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        std::fs::write(dir.join("cal.bpc"), [0x81, 0x01, 0x00, 0x02]).unwrap();
        let p = det.p;
        let mut read = |family, pixels| {
            shared.note_family(family);
            det.ad
                .port_base
                .set_int32_param(p.pix_count, 0, pixels)
                .unwrap();
            let mut buf = vec![-1i32; 5];
            det.read_int32_array(&AsynUser::new(p.bp_c), &mut buf)
                .unwrap();
            (buf, det.int(p.bp_cn, 0))
        };
        assert_eq!(read(DetectorFamily::Unknown, 4), (vec![0; 5], -1));
        assert_eq!(
            read(DetectorFamily::Tpx3, 4),
            (vec![0x81 | 1 << 8, 0x01 | 1 << 8, 0, 2, 0], 2)
        );
        assert_eq!(
            read(DetectorFamily::Mpx3, 2),
            (vec![0x8101 | 1 << 16, 0x0002, 0, 0, 0], 1)
        );

        // mask_io.cpp:601-611: a file off the geometry's size is refused.
        det.ad.port_base.set_int32_param(p.pix_count, 0, 3).unwrap();
        let r = det.read_int32_array(&AsynUser::new(p.bp_c), &mut [0i32; 5]);
        assert!(r.is_err());
        assert_eq!(
            det.text(p.write_msg, 0),
            format!(
                "ReadBPC: BPC file size does not match detector geometry: {}",
                dir.join("cal.bpc").display()
            )
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The mask write uploads through uploadBPC, so its reply is HttpCode
    /// and WriteMsg and BPCFileName comes back afterwards
    /// (mask_io.cpp:559-560, :707-718).
    #[test]
    fn the_mask_write_uploads_through_upload_bpc() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MASK_UPLOAD",
            Arc::new(ServalHttp::new(&fake_serval_with(
                "GET /config/load",
                &[("GET /config/load", "load refused")],
            ))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        calibration_files(&mut det, "TPX_MASK_UPLOAD");
        let p = det.p;
        det.shared.note_family(DetectorFamily::Tpx3);
        det.ad.port_base.set_int32_param(p.pix_count, 0, 2).unwrap();
        det.set_text(p.mask_file_name, "mask.bpc").unwrap();
        assert!(det.write_bpc_file(&[1, 0]).is_none());
        assert_eq!(det.int(p.http_code, 0), 500);
        assert_eq!(det.text(p.write_msg, 0), "load refused");
        assert_eq!(det.text(p.bp_c_file_name, 0), "cal.bpc");
    }

    /// writeBPCfile refuses an undocumented family, a BPCFilePath that is
    /// not a directory and data off the geometry's size before any upload
    /// (mask_io.cpp:639-705).
    #[test]
    fn write_bpc_file_refuses_as_c_before_the_upload() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let name = "TPX_MASK_WRITE_REFUSED";
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        let p = det.p;

        assert!(det.write_bpc_file(&[1, 0]).is_none());
        assert_eq!(
            det.text(p.write_msg, 0),
            "UNKNOWN mask write blocked: pixel mask encoding is not documented"
        );

        det.shared.note_family(DetectorFamily::Tpx3);
        det.ad.port_base.set_int32_param(p.pix_count, 0, 3).unwrap();
        assert!(det.write_bpc_file(&[1, 0]).is_none());
        assert_eq!(det.text(p.mask_file_name, 0), "mask.bpc");
        assert_eq!(
            det.text(p.write_msg, 0),
            format!(
                "WriteBPC: invalid expected BPC size: {}",
                dir.join("mask.bpc").display()
            )
        );
        assert!(!dir.join("mask.bpc").exists());

        let file_path = dir.join("cal.bpc").display().to_string();
        det.set_text(p.bp_c_file_path, &file_path).unwrap();
        assert!(det.write_bpc_file(&[1, 0, 0]).is_none());
        assert!(
            det.text(p.write_msg, 0)
                .starts_with("Calibration path blocked")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    const MEASUREMENT_CONFIG_READBACK: &str = r#"{"Stem": {"Scan": {"Width": 64,
        "Height": 32, "DwellTime": 0.25}, "VirtualDetector": {"RadiusOuter": 9,
        "RadiusInner": 3}}, "TimeOfFlight": {"TdcReference": ["PN0123", "NP0123"],
        "Min": 1.5, "Max": 2.5}}"#;

    fn measurement_config_of(det: &TimePix3Driver) -> (i32, i32, f64, i32, i32, String, f64, f64) {
        let p = det.p;
        (
            det.int(p.stem_scan_width, 0),
            det.int(p.stem_scan_height, 0),
            det.dbl(p.stem_dwell_time, 0),
            det.int(p.stem_radius_outer, 0),
            det.int(p.stem_radius_inner, 0),
            det.text(p.tof_tdc_reference, 0),
            det.dbl(p.tof_min, 0),
            det.dbl(p.tof_max, 0),
        )
    }

    const READBACK: (i32, i32, f64, i32, i32, &str, f64, f64) =
        (64, 32, 0.25, 9, 3, "PN0123,NP0123", 1.5, 2.5);

    fn is_readback(seen: &(i32, i32, f64, i32, i32, String, f64, f64)) -> bool {
        let r = READBACK;
        *seen == (r.0, r.1, r.2, r.3, r.4, r.5.to_string(), r.6, r.7)
    }

    /// The Health scan re-reads Measurement.Config into the Stem and
    /// TimeOfFlight PVs in the write, even after a failed `/detector` read
    /// (ADTimePix.cpp:560, serval_http.cpp:2198-2232).
    #[test]
    fn the_health_scan_reads_back_the_measurement_config() {
        let runtime = crate::create_timepix3_detector(
            &epics_rs::asyn::registry::claim_port("TPX_MSMT_CFG_HEALTH").unwrap(),
            &fake_serval_with(
                "\0",
                &[("GET /measurement/config ", MEASUREMENT_CONFIG_READBACK)],
            ),
            0,
        )
        .unwrap();
        let handle = runtime.port_handle().clone();
        let health = handle
            .with_driver_blocking::<TimePix3Driver, _>(|d| d.p.health)
            .unwrap();
        assert!(handle.write_int32_blocking(health, 0, 1).is_err());
        let seen = handle
            .with_driver_blocking::<TimePix3Driver, _>(|d| measurement_config_of(d))
            .ok();
        assert!(seen.as_ref().is_some_and(is_readback), "{seen:?}");
    }

    /// A readback that fails C `parseConfigReadback` refuses the write
    /// before any PUT (serval_http.cpp:2256-2265).
    #[test]
    fn an_invalid_measurement_config_readback_refuses_the_put() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MSMT_CFG_INVALID",
            Arc::new(ServalHttp::new(&fake_serval_with(
                "PUT /measurement/config",
                &[("GET /measurement/config ", r#"{"Stem": 1}"#)],
            ))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.ad.port_base.set_int32_param(p.http_code, 0, 7).unwrap();
        assert!(
            det.write_int32(&mut AsynUser::new(p.stem_scan_width), 5)
                .is_err()
        );
        assert_eq!(
            det.text(p.write_msg, 0),
            "measurement Stem field is not an object"
        );
        // The PUT, which this Serval refuses with a 500, never went out.
        assert_eq!(det.int(p.http_code, 0), 200);
    }

    /// A rejected PUT restores the PVs from Serval's configuration and keeps
    /// the PUT's HttpCode (serval_http.cpp:2307-2320).
    #[test]
    fn a_rejected_measurement_config_put_restores_the_readback() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_MSMT_CFG_REJECT",
            Arc::new(ServalHttp::new(&fake_serval_with(
                "PUT /measurement/config",
                &[("GET /measurement/config ", MEASUREMENT_CONFIG_READBACK)],
            ))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        assert!(
            det.write_int32(&mut AsynUser::new(p.stem_scan_width), 5)
                .is_err()
        );
        assert!(is_readback(&measurement_config_of(&det)));
        assert_eq!(det.int(p.http_code, 0), 500);
    }

    /// WriteMsg carries the measurement-config PUT's reply, accepted or
    /// rejected (serval_http.cpp:2308-2309, :2318-2319).
    #[test]
    fn the_measurement_config_put_reply_is_write_msg() {
        for (fail, ok) in [("\0", true), ("PUT /measurement/config", false)] {
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
            let mut det = TimePix3Driver::new(
                "TPX_MSMT_CFG_MSG",
                Arc::new(ServalHttp::new(&fake_serval_with(
                    fail,
                    &[
                        ("GET /measurement/config ", MEASUREMENT_CONFIG_READBACK),
                        ("PUT /measurement/config ", "Serval says hi"),
                    ],
                ))),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            let result = det.write_int32(&mut AsynUser::new(p.stem_scan_width), 5);
            assert_eq!(result.is_ok(), ok);
            assert_eq!(det.text(p.write_msg, 0), "Serval says hi");
        }
    }

    const DETECTOR_READBACK: &str = r#"{"Info":{"MpxType":0,"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256},"Config":{}}"#;

    /// A successful rotation re-reads `/detector` in the write, and the
    /// write's status is getDetector's (ADTimePix.cpp:630-641).
    #[test]
    fn a_rotation_rereads_the_detector() {
        for (fail, ok) in [("\0", true), ("GET /detector ", false)] {
            let url = fake_serval_with(fail, &[("GET /detector ", DETECTOR_READBACK)]);
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
            let http = Arc::new(ServalHttp::new(&url));
            let mut det = TimePix3Driver::new(
                "TPX_ROTATE",
                http.clone(),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            let result = det.write_int32(&mut AsynUser::new(p.detector_orientation), 3);
            assert_eq!(result.is_ok(), ok, "{fail}");
            let sent: Vec<_> = http.sent.lock().iter().map(|(r, _)| r.clone()).collect();
            assert_eq!(
                sent.last().map(String::as_str),
                Some("GET /detector"),
                "{fail}"
            );
        }
    }

    /// The Health scan's status is getDetector's: a failed dashboard read
    /// does not fail it, a failed detector read does (ADTimePix.cpp:553-561).
    #[test]
    fn the_health_scan_returns_the_detector_status() {
        for (fail, ok) in [("GET /dashboard ", true), ("GET /detector ", false)] {
            let url = fake_serval_with(fail, &[("GET /detector ", DETECTOR_READBACK)]);
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
            let http = Arc::new(ServalHttp::new(&url));
            let mut det = TimePix3Driver::new(
                "TPX_HEALTH",
                http.clone(),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            let result = det.write_int32(&mut AsynUser::new(p.health), 1);
            assert_eq!(result.is_ok(), ok, "{fail}");
            let sent: Vec<_> = http.sent.lock().iter().map(|(r, _)| r.clone()).collect();
            assert_eq!(
                sent,
                ["GET /dashboard", "GET /detector", "GET /measurement/config"],
                "{fail}"
            );
        }
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

    /// Stand in for the four stream workers: every run a start admits
    /// connects, or fails at once when `connect` is false.
    fn serve_workers(shared: &Arc<Shared>, connect: bool) {
        for which in 0..4 {
            let shared = shared.clone();
            std::thread::spawn(move || {
                let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
                let socket = std::net::TcpStream::connect(listener.local_addr().unwrap()).unwrap();
                let worker = shared.stream_workers()[which];
                let mut served = None;
                loop {
                    let run = worker.wait_for_start(served);
                    served = Some(run);
                    if connect {
                        worker.mark_connected(run, &socket);
                    } else {
                        worker.fail(run);
                    }
                }
            });
        }
    }

    /// A start commits only once every enabled TCP channel's worker has
    /// connected (acquire.cpp:428-503, acquisition_runtime.cpp:66-82); a
    /// worker that does not fails the start, and a stop ends the runs.
    #[test]
    fn a_start_waits_for_its_stream_workers() {
        let start = |name: &str, connect: bool, integrated: bool| {
            let shared = Arc::new(Shared::new());
            serve_workers(&shared, connect);
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&fake_serval("/server/destination"))),
                shared.clone(),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            let (write, base) = if integrated {
                (p.write_prv_img1, p.prv_img1_base)
            } else {
                (p.write_prv_img, p.prv_img_base)
            };
            det.ad.port_base.set_int32_param(write, 0, 1).unwrap();
            det.set_text(base, UNBOUND_TCP).unwrap();
            let started = det
                .write_int32(&mut AsynUser::new(det.ad.params.acquire), 1)
                .is_ok();
            (det, shared, started)
        };

        let (mut det, shared, started) = start("TPX_READY_OK", true, false);
        assert!(started);
        assert!(shared.prv_img_worker.connected());
        det.write_int32(&mut AsynUser::new(det.ad.params.acquire), 0)
            .unwrap();
        assert!(!shared.prv_img_worker.connected());

        let (det, _, started) = start("TPX_READY_FAIL", false, false);
        assert!(!started);
        assert_eq!(
            det.text(det.ad.params.status_message, 0),
            "Preview stream did not become ready"
        );

        // acquire.cpp:448-459: an enabled PrvImg1 needs its own worker.
        let (mut det, shared, started) = start("TPX_READY_INT_OK", true, true);
        assert!(started);
        assert!(shared.prv_img1_worker.connected());
        det.write_int32(&mut AsynUser::new(det.ad.params.acquire), 0)
            .unwrap();
        let (det, _, started) = start("TPX_READY_INT_FAIL", false, true);
        assert!(!started);
        assert_eq!(
            det.text(det.ad.params.status_message, 0),
            "Integrated preview stream did not become ready"
        );
    }

    /// A base-class string reaches asynNDArrayDriver::writeOctet, which C
    /// skips (ADTimePix.cpp:443-444): NDFilePath is checked.
    #[test]
    fn a_base_class_octet_write_chains_to_the_base() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let mut det = TimePix3Driver::new(
            "TPX_OCTET_BASE",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let base = det.ad.params.base;
        let dir = std::env::temp_dir();
        let path = dir.to_str().unwrap().trim_end_matches('/').to_string();
        det.write_octet(&mut AsynUser::new(base.file_path), path.as_bytes())
            .unwrap();
        assert_eq!(det.int(base.file_path_exists, 0), 1);
        assert_eq!(det.text(base.file_path, 0), format!("{path}/"));
    }

    /// A start sends no configuration; the workers take the enabled
    /// channels' Base PVs (acquire.cpp:313-535, :201-223).
    #[test]
    fn a_start_sends_no_configuration_and_syncs_the_streams() {
        let path = format!("tcp://listen@{}", &UNBOUND_TCP["tcp://".len()..]);
        let shared = Arc::new(Shared::new());
        serve_workers(&shared, true);
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
        det.set_text(p.prv_img_base, &path).unwrap();
        det.write_int32(&mut AsynUser::new(det.ad.params.acquire), 1)
            .unwrap();
        let paths = shared.stream_paths();
        assert_eq!(paths.prv_img.as_deref(), Some(path.as_str()));
        assert_eq!(paths.img, None);
    }

    /// C starts the Img worker only with ImgAccumulationEnable set and the
    /// PrvHst worker only for a tcp:// jsonhisto (Format 4) channel with
    /// PrvHstAccumulationEnable set, and fails a start whose histogram path
    /// does not parse (acquire.cpp:461-500).
    #[test]
    fn a_start_gates_the_img_and_prv_hst_workers() {
        let start = |name: &str, img_acc: i32, format: i32, hst_acc: i32, hst: &str| {
            let shared = Arc::new(Shared::new());
            serve_workers(&shared, true);
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&fake_serval("/server/destination"))),
                shared.clone(),
                0,
                cmd_tx,
            )
            .unwrap();
            let p = det.p;
            for (reason, v) in [
                (p.write_img, 1),
                (p.img_accumulation_enable, img_acc),
                (p.write_prv_hst, 1),
                (p.prv_hst_format, format),
                (p.prv_hst_accumulation_enable, hst_acc),
            ] {
                det.ad.port_base.set_int32_param(reason, 0, v).unwrap();
            }
            det.set_text(p.img_base, UNBOUND_TCP).unwrap();
            det.set_text(p.prv_hst_base, hst).unwrap();
            let started = det
                .write_int32(&mut AsynUser::new(det.ad.params.acquire), 1)
                .is_ok();
            let paths = shared.stream_paths();
            (started, paths.img.is_some(), paths.prv_hst.is_some())
        };
        let tcp = "tcp://192.0.2.1:8451";
        assert_eq!(start("TPX_GATE_ALL", 1, 4, 1, tcp), (true, true, true));
        assert_eq!(start("TPX_GATE_IMG", 0, 4, 1, tcp), (true, false, true));
        assert_eq!(start("TPX_GATE_FMT", 1, 3, 1, tcp), (true, true, false));
        assert_eq!(start("TPX_GATE_HST", 1, 4, 0, tcp), (true, true, false));
        assert_eq!(
            start("TPX_GATE_FILE", 1, 4, 1, "file:///tmp"),
            (true, true, false)
        );
        let (started, ..) = start("TPX_GATE_BAD", 1, 4, 1, "tcp://localhost");
        assert!(!started);
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

    /// C's readInt32Array tail posts the pending parameters for every
    /// reason but the two it answers first (mask_io.cpp:38-91, :239-247).
    #[test]
    fn a_waveform_read_posts_pending_params_as_c() {
        use epics_rs::asyn::interrupt::InterruptFilter;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_READ_TAIL",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let p = det.p;
        det.ad.port_base.own_interrupts(InterruptGate::opened());
        let posted = Arc::new(parking_lot::Mutex::new(Vec::new()));
        let seen = posted.clone();
        std::mem::forget(
            det.ad
                .port_base
                .interrupts
                .register_sync_callback(
                    InterruptFilter {
                        reason: Some(p.http_code),
                        ..Default::default()
                    },
                    move |iv| {
                        if let ParamValue::Int32(v) = iv.value {
                            seen.lock().push(v);
                        }
                    },
                )
                .unwrap(),
        );
        for (reason, value, posts) in [
            (p.pixel_config_diff, 1, false),
            (p.img_image_frame, 2, false),
            (p.prv_hst_histogram_frame, 3, true),
        ] {
            posted.lock().clear();
            det.ad
                .port_base
                .set_int32_param(p.http_code, 0, value)
                .unwrap();
            let mut buf = [0; 2];
            det.read_int32_array(&AsynUser::new(reason), &mut buf)
                .unwrap();
            assert_eq!(posted.lock().contains(&value), posts, "reason {reason}");
        }
    }

    /// PROC on PrvHstHistogramFrame keeps the waveform's VAL: C's readArray
    /// writes nothing and returns nElements (mask_io.cpp:92, :239-247).
    #[test]
    fn a_processed_histogram_waveform_keeps_its_value() {
        use epics_rs::asyn::adapter::{AsynDeviceSupport, AsynLink};
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::base::server::device_support::DeviceSupport;
        use epics_rs::base::server::record::{Record, ScanType};
        use epics_rs::base::server::records::waveform::WaveformRecord;
        use epics_rs::base::types::{DbFieldType, EpicsValue};
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_PRVHST_PROC",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        det.ad
            .port_base
            .params
            .set_int32_array(det.p.prv_hst_histogram_frame, 0, vec![1, 2])
            .unwrap();
        let (runtime, _) = create_port_runtime(
            &epics_rs::asyn::registry::claim_port("TPX_PRVHST_PROC").unwrap(),
            det,
            RuntimeConfig::default(),
        )
        .unwrap();
        let link = AsynLink {
            port_name: "TPX_PRVHST_PROC".into(),
            addr: 0,
            timeout: Some(std::time::Duration::from_secs(1)),
            drv_info: "TPX3_PRV_HST_HISTOGRAM_FRAME".into(),
        };
        let mut ads =
            AsynDeviceSupport::from_handle(runtime.port_handle().clone(), link, "asynInt32Array");
        ads.set_record_info("TPX:PrvHstHistogramFrame", ScanType::Passive);
        let mut rec = WaveformRecord::new(4, DbFieldType::Long);
        ads.init(&mut rec).unwrap();
        rec.put_field("VAL", EpicsValue::LongArray(vec![5, 6, 7].into()))
            .unwrap();
        ads.read(&mut rec).unwrap();
        assert_eq!(
            rec.val(),
            Some(EpicsValue::LongArray(vec![5, 6, 7, 0].into()))
        );
        std::mem::forget(runtime);
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
            Arc::new(ServalHttp::new(&fake_serval("\0"))),
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

    /// A processed output type written at any address lands on list 0 as
    /// well, and an unknown type is 0 there (ADTimePix.cpp:593-597, :605-609).
    #[test]
    fn a_processed_output_type_always_lands_on_list_zero() {
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
            let _ = det.write_int32(&mut AsynUser::new(reason).with_addr(3), 1);
            assert_eq!(det.int(reason, 0), 1, "reason {reason}");
            let _ = det.write_int32(&mut AsynUser::new(reason).with_addr(3), 7);
            assert_eq!(det.int(reason, 0), 0, "reason {reason}");
        }
    }

    /// An octet write past maxAddr fails in parseAsynUser with asynError,
    /// before the store (ADTimePix.cpp:424-425).
    #[test]
    fn an_octet_write_past_max_addr_is_asyn_error() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_OCTET_ADDR",
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let e = det
            .write_octet(&mut AsynUser::new(det.p.write_msg).with_addr(20), b"x")
            .unwrap_err();
        assert_eq!(e.c_status_code(), 3);
    }

    /// WriteMsg is the reply body of the destination PUT, the detector
    /// config GET and PUT, and the calibration loads, accepted or not
    /// (serval_http.cpp:1691, :2183, :2452, :2627, mask_io.cpp:560).
    #[test]
    fn write_msg_is_the_serval_reply() {
        let new = |fail: &'static str, routes: &'static [(&'static str, &'static str)]| {
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
            TimePix3Driver::new(
                "TPX_WRITE_MSG",
                Arc::new(ServalHttp::new(&fake_serval_with(fail, routes))),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap()
        };
        let mut det = new("\0", &[("PUT /server/destination ", "destination set")]);
        det.ad
            .port_base
            .set_string_param(det.p.raw_base, 0, "file:/data")
            .unwrap();
        det.ad
            .port_base
            .set_int32_param(det.p.write_raw, 0, 1)
            .unwrap();
        let _ = det.file_writer();
        assert_eq!(det.text(det.p.write_msg, 0), "destination set");

        let mut det = new(
            "GET /detector/config ",
            &[("GET /detector/config ", "no detector")],
        );
        assert!(det.init_acquisition().is_err());
        assert_eq!(det.text(det.p.write_msg, 0), "no detector");

        let mut det = new(
            "PUT /detector/config ",
            &[("PUT /detector/config ", "bad config")],
        );
        assert!(det.init_acquisition().is_err());
        assert_eq!(det.text(det.p.write_msg, 0), "bad config");

        for dacs in [false, true] {
            let mut det = new("GET /config/load", &[("GET /config/load", "load refused")]);
            calibration_files(&mut det, "TPX_WRITE_MSG");
            let reason = if dacs {
                det.p.write_da_cs_file
            } else {
                det.p.write_bp_c_file
            };
            assert!(det.write_int32(&mut AsynUser::new(reason), 1).is_err());
            assert_eq!(det.text(det.p.write_msg, 0), "load refused");
        }
    }

    /// uploadBPC sends nothing for a BPC off the geometry's size
    /// (mask_io.cpp:535-539).
    #[test]
    fn upload_bpc_refuses_a_bpc_off_the_geometry() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let name = "TPX_BPC_UPLOAD_SIZE";
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        let p = det.p;
        det.ad.port_base.set_int32_param(p.pix_count, 0, 2).unwrap();
        let r = det.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name);
        assert_eq!(
            r.map_err(|e| e.message()),
            Err("uploadBPC: local BPC validation failed; request not sent".into())
        );
        assert_eq!(det.int(p.http_code, 0), 0);
        assert_eq!(
            det.text(p.write_msg, 0),
            format!(
                "ReadBPC: BPC file size does not match detector geometry: {}",
                dir.join("cal.bpc").display()
            )
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// PixelConfig is compared chip by chip with the BPC at the geometry's
    /// size; a missing BPC skips the comparison instead of failing, and a
    /// reply of the wrong length is refused (serval_http.cpp:1074-1290).
    #[test]
    fn refresh_pixel_config_compares_each_chip_as_c() {
        // 65536 zero bytes as Serval's JSON string.
        fn zeros() -> &'static str {
            Box::leak(format!("\"{}AA==\"", "AAAA".repeat(21845)).into_boxed_str())
        }
        fn run(name: &str, reply: &'static str, bpc: Option<&[u8]>) -> (TimePix3Driver, bool) {
            let routes: &'static [(&str, &str)] =
                Box::leak(Box::new([("GET /detector/chips/0/PixelConfig", reply)]));
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
            let shared = Arc::new(Shared::new());
            shared.note_family(DetectorFamily::Tpx3);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&fake_serval_with("\0", routes))),
                shared,
                0,
                cmd_tx,
            )
            .unwrap();
            let dir = calibration_files(&mut det, name);
            let p = det.p;
            for (reason, value) in [
                (p.pix_count, 65536),
                (p.row_len, 1),
                (p.number_of_chips, 1),
                (p.number_of_rows, 256),
            ] {
                det.ad.port_base.set_int32_param(reason, 0, value).unwrap();
            }
            match bpc {
                Some(bpc) => std::fs::write(dir.join("cal.bpc"), bpc).unwrap(),
                None => std::fs::remove_file(dir.join("cal.bpc")).unwrap(),
            }
            let ok = det.refresh_pixel_config().is_ok();
            let _ = std::fs::remove_dir_all(&dir);
            (det, ok)
        }
        let readback = |det: &mut TimePix3Driver| {
            let p = det.p;
            (
                det.int(p.pixel_config_match_bp_c, 0),
                det.ad
                    .port_base
                    .get_int64_param(p.pixel_config_mismatch_bytes, 0)
                    .unwrap(),
                det.int(p.pixel_config_len, 0),
                det.text(p.pixel_config_status, 0),
                det.text(p.write_msg, 0),
            )
        };

        let (mut det, ok) = run("TPX_PIXCFG_NOBPC", zeros(), None);
        assert!(ok);
        assert_eq!(
            readback(&mut det),
            (
                2,
                0,
                65536,
                "OK, no BPC file".into(),
                "PixelConfig refresh complete; BPC comparison skipped.".into()
            )
        );

        let mut bpc = vec![0u8; 65536];
        bpc[5] = 3;
        let (mut det, ok) = run("TPX_PIXCFG_DIFF", zeros(), Some(&bpc));
        assert!(ok);
        assert_eq!(
            readback(&mut det),
            (
                0,
                1,
                65536,
                "Mismatch 1 bytes".into(),
                "PixelConfig refresh complete.".into()
            )
        );
        let diff = det
            .ad
            .port_base
            .params
            .get_int32_array(det.p.pixel_config_diff, 0)
            .unwrap();
        assert_eq!(diff.len(), 262144);
        assert_eq!(diff.iter().filter(|&&d| d != 0).collect::<Vec<_>>(), [&3]);

        let (mut det, ok) = run("TPX_PIXCFG_SHORT", "\"AAAA\"", Some(&bpc));
        assert!(!ok);
        assert_eq!(
            readback(&mut det),
            (
                3,
                0,
                0,
                "Length mismatch (expected 65536 bytes)".into(),
                "PixelConfig refresh completed with one or more chip errors.".into()
            )
        );
    }

    /// refreshPixelConfig posts PixelConfigDiff with doCallbacksInt32Array,
    /// typed and before the closing callParamCallbacks that carries
    /// WriteMsg (serval_http.cpp:1274-1290).
    #[test]
    fn refresh_pixel_config_posts_the_diff_as_an_array_callback() {
        use epics_rs::asyn::interfaces::InterfaceType;
        use epics_rs::base::runtime::interrupt_accept::InterruptGate;
        let reply: &'static str =
            Box::leak(format!("\"{}AA==\"", "AAAA".repeat(21845)).into_boxed_str());
        let routes: &'static [(&str, &str)] =
            Box::leak(Box::new([("GET /detector/chips/0/PixelConfig", reply)]));
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Tpx3);
        let name = "TPX_PIXCFG_POST";
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new(&fake_serval_with("\0", routes))),
            shared,
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        std::fs::remove_file(dir.join("cal.bpc")).unwrap();
        let p = det.p;
        for (reason, value) in [
            (p.pix_count, 65536),
            (p.row_len, 1),
            (p.number_of_chips, 1),
            (p.number_of_rows, 256),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, value).unwrap();
        }
        let base = &mut det.ad.port_base;
        base.own_interrupts(InterruptGate::opened());
        for addr in 0..MAX_ADDR as i32 {
            base.call_param_callbacks(addr).unwrap();
        }
        let mut rx = base.interrupts.subscribe_async(1024, 1 << 21);
        assert!(det.refresh_pixel_config().is_ok());
        let _ = std::fs::remove_dir_all(&dir);
        let done = b"PixelConfig refresh complete; BPC comparison skipped.";
        let posted: Vec<_> = std::iter::from_fn(|| rx.try_recv().ok())
            .filter(|v| {
                (v.reason == p.pixel_config_diff && v.iface.is_some())
                    || matches!(&v.value, ParamValue::Octet(text) if text[..] == done[..])
            })
            .map(|v| (v.reason, v.iface))
            .collect();
        assert_eq!(
            posted,
            [
                (p.pixel_config_diff, Some(InterfaceType::Int32Array)),
                (p.write_msg, None),
            ]
        );
        // readInt32Array still serves the diff (mask_io.cpp:38-52).
        let mut buf = vec![-1; 4];
        let user = AsynUser::new(p.pixel_config_diff);
        assert_eq!(det.read_int32_array(&user, &mut buf).unwrap(), 4);
        assert_eq!(buf, [0; 4]);
    }

    /// The masked-pixel export names its file after the BPC, lists every
    /// masked pixel with its BPC and image coordinates in nlohmann's key
    /// order, and publishes the path it wrote (serval_http.cpp:911-1072).
    #[test]
    fn export_masked_pels_json_writes_beside_the_bpc_as_c() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let name = "TPX_MASKED_PELS";
        let shared = Arc::new(Shared::new());
        shared.note_family(DetectorFamily::Tpx3);
        let mut det = TimePix3Driver::new(
            name,
            Arc::new(ServalHttp::new("http://127.0.0.1:1")),
            shared.clone(),
            0,
            cmd_tx,
        )
        .unwrap();
        let dir = calibration_files(&mut det, name);
        let p = det.p;
        for (reason, value) in [
            (p.pix_count, 65536),
            (p.row_len, 1),
            (p.number_of_chips, 1),
            (p.number_of_rows, 256),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, value).unwrap();
        }
        det.set_text(p.bp_c_file_name, "cal.BPC").unwrap();
        det.set_text(p.det_type, "TPX3").unwrap();
        let mut bpc = vec![0u8; 65536];
        bpc[5] = 3;
        bpc[300] = 1;
        bpc[301] = 2;
        std::fs::write(dir.join("cal.BPC"), &bpc).unwrap();

        det.export_masked_pels_json(&bpc).unwrap();
        let out = dir.join("cal_masked_pels.json");
        assert_eq!(
            (
                det.text(p.masked_pels_json_path, 0),
                det.int(p.masked_pels_count, 0),
                det.text(p.masked_pels_export_status, 0),
            ),
            (
                out.display().to_string(),
                2,
                "OK: wrote 2 masked pels".into()
            )
        );
        let text = std::fs::read_to_string(&out).unwrap();
        assert!(text.starts_with("{\n  \"Bad pixels\": [\n    {\n      \"Median\": ["));
        let root: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(root["counts"]["masked_pels"], 2);
        assert_eq!(root["detector"]["type"], "TPX3");
        assert_eq!(
            root["source"]["bpc_path"],
            dir.join("cal.BPC").display().to_string()
        );
        let stamp = root["source"]["exported_at_utc"].as_str().unwrap();
        assert_eq!((stamp.len(), &stamp[10..11], &stamp[19..]), (20, "T", "Z"));
        let g = det.rows_cols().unwrap();
        let mut pixels: Vec<_> = root["masked_pels"]
            .as_array()
            .unwrap()
            .iter()
            .map(|row| {
                let (i, j) = (row["i"].as_i64().unwrap(), row["j"].as_i64().unwrap());
                let (k, _) = mask::pel_index(&g, 1, 0, i as i32, j as i32);
                assert_eq!(row["bpc_index"], i64::from(k));
                assert_eq!(row["bpc_pixel_index"], i64::from(k));
                assert_eq!(
                    (row["lx"].as_i64().unwrap(), row["ly"].as_i64().unwrap()),
                    (i64::from(k) % 256, i64::from(k) / 256)
                );
                (k, row["value"].as_u64().unwrap())
            })
            .collect();
        pixels.sort();
        assert_eq!(pixels, [(5, 3), (300, 1)]);
        assert_eq!(root["Bad pixels"].as_array().unwrap().len(), 2);

        det.export_masked_pels_json(&[]).unwrap();
        assert_eq!(
            (
                det.int(p.masked_pels_count, 0),
                det.text(p.masked_pels_export_status, 0),
                det.text(p.masked_pels_json_path, 0)
            ),
            (0, "Skipped: no BPC buffer".into(), String::new())
        );

        det.set_text(p.bp_c_file_name, "").unwrap();
        det.export_masked_pels_json(&bpc).unwrap();
        assert_eq!(
            (
                det.int(p.masked_pels_count, 0),
                det.text(p.masked_pels_export_status, 0)
            ),
            (0, "Skipped: BPCFileName empty".into())
        );
        shared.note_family(DetectorFamily::Unknown);
        det.export_masked_pels_json(&bpc).unwrap();
        assert_eq!(
            (
                det.int(p.masked_pels_count, 0),
                det.text(p.masked_pels_export_status, 0)
            ),
            (
                -1,
                "Unavailable: UNKNOWN pixel mask encoding is not documented".into()
            )
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_export_time_is_utc_iso8601() {
        let at = |secs| utc_iso8601(std::time::UNIX_EPOCH + std::time::Duration::from_secs(secs));
        assert_eq!(at(0), "1970-01-01T00:00:00Z");
        assert_eq!(at(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(at(1_792_195_199), "2026-10-16T23:59:59Z");
        assert_eq!(strip_bpc_ext("a.Bpc"), "a");
        assert_eq!(strip_bpc_ext(".bpc"), "");
        assert_eq!(strip_bpc_ext("bpc"), "bpc");
        assert_eq!(strip_bpc_ext("a.bpcx"), "a.bpcx");
    }

    /// Point the BPC and DACS file PVs at real files, which C
    /// `resolveCalibrationFile` requires before an upload.
    /// uploadBPC logs the status and the file Serval was told to load,
    /// accepted or not (mask_io.cpp:557).
    #[test]
    fn upload_bpc_logs_its_status_and_file() {
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::{TraceFile, TraceMask};
        for (name, fail, code) in [
            ("TPX_BPC_LOG_OK", "\0", 200),
            ("TPX_BPC_LOG_BAD", "GET /config/load", 500),
        ] {
            let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
            let mut det = TimePix3Driver::new(
                name,
                Arc::new(ServalHttp::new(&fake_serval(fail))),
                Arc::new(Shared::new()),
                0,
                cmd_tx,
            )
            .unwrap();
            calibration_files(&mut det, name);
            let p = det.p;
            let (runtime, _) = create_port_runtime(
                &epics_rs::asyn::registry::claim_port(name).unwrap(),
                det,
                RuntimeConfig::default(),
            )
            .unwrap();
            let h = runtime.port_handle().clone();
            let path = std::env::temp_dir().join(format!("{name}-{}.trace", std::process::id()));
            let file = std::fs::File::create(&path).unwrap();
            h.trace().set_trace_file(
                Some(name),
                TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
            );
            h.trace().set_trace_mask(Some(name), TraceMask::IO_DRIVER);
            let resolved = h
                .with_driver_blocking::<TimePix3Driver, _>(move |d| {
                    let _ = d.config_load("pixelconfig", p.bp_c_file_path, p.bp_c_file_name);
                    let (dir, file) = (d.text(p.bp_c_file_path, 0), d.text(p.bp_c_file_name, 0));
                    d.resolve_calibration_file(&dir, &file, PathAccess::Read)
                        .unwrap()
                })
                .unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            std::mem::forget(runtime);
            assert!(
                text.contains(&adlog::log(
                    adlog::UPLOAD_BPC,
                    &format!("uploadBPC: http_code={code} file={}", resolved.display())
                )),
                "{text}"
            );
        }
    }

    /// Each Serval request the port thread makes carries C's timeout
    /// (serval_http.cpp:557-638, :1106, :1296, :1612, :1682, :2180-2307,
    /// :2447-2635; acquire.cpp:352-392; acquisition_runtime.cpp:200-220).
    #[test]
    fn each_port_request_carries_c_timeout() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(4);
        let name = "TPX_PORT_TIMEOUTS";
        // A refused start rolls back.
        let http = Arc::new(ServalHttp::new(&fake_serval("/measurement/start")));
        let mut det =
            TimePix3Driver::new(name, http.clone(), Arc::new(Shared::new()), 0, cmd_tx).unwrap();
        calibration_files(&mut det, name);
        let p = det.p;
        for (reason, value) in [
            (p.pix_count, 65536),
            (p.row_len, 1),
            (p.number_of_chips, 1),
            (p.number_of_rows, 256),
            (p.write_raw, 1),
        ] {
            det.ad.port_base.set_int32_param(reason, 0, value).unwrap();
        }
        det.set_text(p.raw_base, "file:/data").unwrap();
        let sent = || std::mem::take(&mut *http.sent.lock());
        let (default, short) = (DEFAULT_TIMEOUT, TIMEOUT_5000_MS);
        let get = |path: &str, t| (format!("GET {path}"), t);
        let put = |path: &str, t| (format!("PUT {path}"), t);

        let _ = det.init_acquisition();
        assert_eq!(
            sent(),
            [
                get("/detector/config", default),
                get("/detector", short),
                put("/detector/config", default),
            ]
        );
        let _ = det.resync_mpx3_setpoints();
        assert_eq!(sent(), [get("/detector/config", default)]);
        let _ = det.send_measurement_config();
        assert_eq!(
            sent(),
            [
                get("/measurement/config", short),
                put("/measurement/config", default),
            ]
        );
        let _ = det.get_measurement_config();
        assert_eq!(sent(), [get("/measurement/config", short)]);
        let _ = det.file_writer();
        assert_eq!(sent(), [put("/server/destination", default)]);
        let _ = det.get_server();
        assert_eq!(sent(), [get("/server/destination", short)]);
        // A refused DAC write restores.
        let dacs = "/detector/chips/0/dacs/";
        let refusing = Arc::new(ServalHttp::new(&fake_serval_with(
            "PUT /detector/chips/0/dacs/",
            &[("GET /detector/chips/0/dacs/", r#"{"Vthreshold_fine": 4}"#)],
        )));
        let port_http = std::mem::replace(&mut det.http, refusing.clone());
        let _ = det.write_dac(0, p.vthreshold_fine, "Vthreshold_fine", 5, 4);
        det.http = port_http;
        assert_eq!(
            std::mem::take(&mut *refusing.sent.lock()),
            [get(dacs, default), put(dacs, default), get(dacs, default)]
        );
        let _ = det.rotate_layout(0);
        assert_eq!(sent(), [get("/detector/layout/rotate?reset=true", default)]);
        let _ = det.config_load("dacs", p.da_cs_file_path, p.da_cs_file_name);
        let sent_load = sent();
        assert_eq!(sent_load.len(), 1);
        assert!(
            sent_load[0]
                .0
                .starts_with("GET /config/load?format=dacs&file=")
        );
        assert_eq!(sent_load[0].1, default);
        let _ = det.refresh_pixel_config();
        assert_eq!(sent(), [get("/detector/chips/0/PixelConfig", short)]);
        assert!(det.shared.acquisition.begin_start());
        let _ = det.start_measurement();
        assert_eq!(
            sent(),
            [
                get("/measurement/stop", default),
                get("/measurement", default),
                get("/measurement/start", default),
            ]
        );
        let _ = det.stop_acquisition(false, "");
        assert_eq!(
            sent(),
            [
                get("/measurement/stop", default),
                get("/measurement", default),
            ]
        );
    }

    /// The `server=` line is the destination dumped with a 3-space indent
    /// and non-ASCII escaped, as C prints it (serval_http.cpp:2176).
    #[test]
    fn send_configuration_prints_the_destination_as_c() {
        let server = json!({"Raw": [{"Base": "file:/d\u{e4}ta", "QueueSize": 16}]});
        assert_eq!(
            send_configuration_line(&server),
            "server={\n   \"Raw\": [\n      {\n         \"Base\": \"file:/d\\u00e4ta\",\n         \"QueueSize\": 16\n      }\n   ]\n}"
        );
    }

    /// `name`'s calibration directory under a root of the temp directory, in
    /// the canonical form the policy resolves to. The default "/" root is not
    /// absolute on Windows, as in C, so the tests configure their own.
    fn calibration_files(det: &mut TimePix3Driver, name: &str) -> std::path::PathBuf {
        let tmp = crate::calibration::canonical(&std::env::temp_dir()).unwrap();
        det.configure_calibration(tmp.to_str().unwrap()).unwrap();
        let dir = tmp.join(format!("{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for file in ["cal.bpc", "cal.dacs"] {
            std::fs::write(dir.join(file), b"x").unwrap();
        }
        let p = det.p;
        let text = format!("{}/", dir.display());
        for (reason, text) in [
            (p.bp_c_file_path, text.as_str()),
            (p.bp_c_file_name, "cal.bpc"),
            (p.da_cs_file_path, text.as_str()),
            (p.da_cs_file_name, "cal.dacs"),
        ] {
            det.set_text(reason, text).unwrap();
        }
        // The one-byte cal.bpc is the geometry's size, as uploadBPC checks.
        det.ad.port_base.set_int32_param(p.pix_count, 0, 1).unwrap();
        dir
    }

    /// The upload runs, and its status is the write's, even where the
    /// one-shot's own store is refused (ADTimePix.cpp:614-621).
    #[test]
    fn a_refused_one_shot_store_still_uploads() {
        let (cmd_tx, _cmd_rx) = rt::command_channel::<Command>(1);
        let mut det = TimePix3Driver::new(
            "TPX_UPLOAD_ADDR",
            Arc::new(ServalHttp::new(&fake_serval("\0"))),
            Arc::new(Shared::new()),
            0,
            cmd_tx,
        )
        .unwrap();
        calibration_files(&mut det, "TPX_UPLOAD_ADDR");
        let p = det.p;
        for reason in [p.write_bp_c_file, p.write_da_cs_file] {
            det.ad.port_base.set_int32_param(p.http_code, 0, 0).unwrap();
            assert!(
                det.write_int32(&mut AsynUser::new(reason).with_addr(20), 1)
                    .is_ok(),
                "reason {reason}"
            );
            assert_eq!(det.int(p.http_code, 0), 200, "reason {reason}");
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
