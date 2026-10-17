//! Serval's endpoints and the JSON bodies the driver sends
//! (port of `serval_http.cpp`).
//!
//! The body builders are pure functions so the wire format can be tested
//! without a Serval.

use serde_json::{Value, json};

// ---------------------------------------------------------------------------
// Endpoints (C `serverURL + "/..."`)
// ---------------------------------------------------------------------------

pub const DASHBOARD: &str = "/dashboard";
pub const DETECTOR: &str = "/detector";
pub const DETECTOR_HEALTH: &str = "/detector/health";
pub const DETECTOR_CONFIG: &str = "/detector/config";
pub const SERVER_DESTINATION: &str = "/server/destination";
pub const MEASUREMENT: &str = "/measurement";
pub const MEASUREMENT_START: &str = "/measurement/start";
pub const MEASUREMENT_STOP: &str = "/measurement/stop";
pub const MEASUREMENT_CONFIG: &str = "/measurement/config";

pub fn chip_dacs(chip: i32) -> String {
    format!("/detector/chips/{chip}/dacs/")
}

pub fn chip_pixel_config(chip: i32) -> String {
    format!("/detector/chips/{chip}/PixelConfig")
}

/// `GET /config/load?format=…&file=…` — Serval loads the file from its *own*
/// filesystem; nothing is uploaded (C `uploadBPC`/`uploadDACS`).
pub fn config_load(format: &str, file: &str) -> String {
    format!("/config/load?format={format}&file={file}")
}

/// `GET /detector/layout/rotate?…` (C `rotateLayout`, serval_http.cpp:476).
///
/// The index is the `TPX3_DET_ORIENTATION` menu (UP, RIGHT, DOWN, LEFT,
/// UP_MIRRORED, RIGHT_MIRRORED, DOWN_MIRRORED, LEFT_MIRRORED).
pub fn layout_rotate(orientation: i32) -> Option<String> {
    let suffix = match orientation {
        0 => "",
        1 => "&direction=right",
        2 => "&direction=180",
        3 => "&direction=left",
        4 => "&flip=horizontal",
        5 => "&direction=right&flip=horizontal",
        6 => "&flip=vertical",
        7 => "&direction=right&flip=vertical",
        _ => return None,
    };
    Some(format!("/detector/layout/rotate?reset=true{suffix}"))
}

// ---------------------------------------------------------------------------
// Enumerations (the mbbo menus in the db templates)
// ---------------------------------------------------------------------------

/// `TPX3_DET_ORIENTATION` — the menu index is the position in this table
/// (C `mDetOrientationMap`, ADTimePix.cpp:1084-1091).
pub const ORIENTATIONS: [&str; 8] = [
    "UP",
    "RIGHT",
    "DOWN",
    "LEFT",
    "UP_MIRRORED",
    "RIGHT_MIRRORED",
    "DOWN_MIRRORED",
    "LEFT_MIRRORED",
];

/// The `TPX3_DET_ORIENTATION` index of `Layout.DetectorOrientation`.
///
/// UPSTREAM DEFECT (serval_http.cpp:1296): C looks the name up in a
/// `std::map` with `operator[]`, which *inserts* an unknown name with value 0
/// and so silently reports orientation UP — and the name it looks up has been
/// through `strip_quotes`, which mangles anything that is not a quoted string.
/// An unknown name leaves the parameter alone here.
pub fn orientation_index(name: &str) -> Option<i32> {
    ORIENTATIONS
        .iter()
        .position(|o| *o == name)
        .and_then(|i| i32::try_from(i).ok())
}

/// `TPX3_IMG_IMGFORMAT` and friends.
pub const FORMATS: [&str; 5] = ["tiff", "pgm", "png", "jsonimage", "jsonhisto"];
/// `TPX3_IMG_IMGMODE` and friends.
pub const MODES: [&str; 5] = ["count", "tot", "toa", "tof", "count_fb"];
/// `TPX3_IMG_INTMODE` and friends.
pub const INTEGRATION_MODES: [&str; 3] = ["sum", "average", "last"];
/// `TPX3_RAW_SPLITSTG`.
pub const SPLIT_STRATEGIES: [&str; 2] = ["single_file", "frame"];
/// `TPX3_PRV_SAMPLMODE`.
pub const SAMPLING_MODES: [&str; 2] = ["skipOnFrame", "skipOnPeriod"];
/// `ADTriggerMode`'s Serval names (C `kTriggerModes[].servalName`,
/// ADTimePix.cpp:900-910).
pub const TRIGGER_MODES: [&str; 9] = [
    "PEXSTART_NEXSTOP",
    "NEXSTART_PEXSTOP",
    "PEXSTART_TIMERSTOP",
    "NEXSTART_TIMERSTOP",
    "AUTOTRIGSTART_TIMERSTOP",
    "CONTINUOUS",
    "SOFTWARESTART_TIMERSTOP",
    "SOFTWARESTART_SOFTWARESTOP",
    "FOLLOWING",
];
/// `ADTriggerMode`'s menu, which the driver's `readEnum` supplies: index 7 is
/// shortened to fit an EPICS state string (C `kTriggerModes[].menuLabel`).
pub const TRIGGER_MODE_LABELS: [&str; 9] = [
    "PEXSTART_NEXSTOP",
    "NEXSTART_PEXSTOP",
    "PEXSTART_TIMERSTOP",
    "NEXSTART_TIMERSTOP",
    "AUTOTRIGSTART_TIMERSTOP",
    "CONTINUOUS",
    "SOFTWARESTART_TIMERSTOP",
    "SwReSt_SwReSp",
    "FOLLOWING",
];
/// `TPX3_MSMT_PIPELINE_STATE`'s menu (C `kPipelineStates`,
/// ADTimePix.cpp:920-926).
pub const PIPELINE_STATES: [(i32, &str); 5] = [
    (0, "DA_STARTING"),
    (1, "DA_RECORDING"),
    (2, "DA_STOPPING"),
    (3, "DA_IDLE"),
    (-1, "Unknown"),
];
/// `TPX3_CHAIN_MODE`.
pub const CHAIN_MODES: [&str; 3] = ["NONE", "LEADER", "FOLLOWER"];
/// `TPX3_POLARITY`.
pub const POLARITIES: [&str; 2] = ["Positive", "Negative"];
/// `TPX3_TDC0` / `TPX3_TDC1`.
pub const TDC_MODES: [&str; 6] = ["P0123", "N0123", "PN0123", "P0", "N0", "PN0"];

/// The name behind an enum index.
///
/// UPSTREAM DEFECT (serval_http.cpp:2286, 2322, 2329, 2335, 2355, 2363): C
/// indexes a `json` array with the raw PV value — `json::operator[]` on a
/// non-const array *grows the array with nulls* for an out-of-range index, so
/// an out-of-range enum silently PUTs `"TriggerMode": null` (and a negative
/// value is cast to `size_type`). Here an out-of-range index is an error the
/// caller has to handle.
pub fn enum_name(table: &[&'static str], index: i32) -> Option<&'static str> {
    usize::try_from(index)
        .ok()
        .and_then(|i| table.get(i))
        .copied()
}

// ---------------------------------------------------------------------------
// /server/destination — the channel configuration (C `fileWriter`)
// ---------------------------------------------------------------------------

/// One Raw channel (C `configureRawChannel`, serval_http.cpp:1574).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawChannel {
    pub base: String,
    pub file_pattern: String,
    pub split_strategy: i32,
    pub queue_size: i32,
}

/// One Image / Preview-image channel (C `configureImageChannel`,
/// serval_http.cpp:1638).
#[derive(Debug, Clone, PartialEq)]
pub struct ImageChannel {
    pub base: String,
    pub file_pattern: String,
    pub format: i32,
    pub mode: i32,
    pub integration_size: i32,
    pub integration_mode: i32,
    pub stop_on_disk_limit: bool,
    pub queue_size: i32,
}

/// The one histogram channel (C `configureHistogramChannel`,
/// serval_http.cpp:1867).
#[derive(Debug, Clone, PartialEq)]
pub struct HistogramChannel {
    pub image: ImageChannel,
    pub number_of_bins: i32,
    pub bin_width: f64,
    pub offset: f64,
}

/// The preview sampling settings (C `configurePreviewSettings`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreviewSettings {
    pub period: f64,
    pub sampling_mode: i32,
}

/// Everything `PUT /server/destination` carries.
#[derive(Debug, Clone, Default)]
pub struct Destination {
    pub raw: Vec<RawChannel>,
    pub image: Vec<ImageChannel>,
    pub preview_image: Vec<ImageChannel>,
    pub preview_histogram: Option<HistogramChannel>,
    pub preview: Option<PreviewSettings>,
}

/// A channel whose `Base` is a stream URI has no file on disk, so the
/// file-related keys must not be sent.
///
/// UPSTREAM DEFECT (serval_http.cpp:1603 vs 1881): C's `configureRawChannel`
/// treats only `tcp://` as a stream, `configureHistogramChannel` treats
/// `http://` *and* `tcp://`, and `configureImageChannel` checks neither — so an
/// image channel streaming over `tcp://` is still sent a `FilePattern` and a
/// `StopMeasurementOnDiskLimit`. One rule, applied to every channel.
pub fn is_stream(base: &str) -> bool {
    base.starts_with("tcp://") || base.starts_with("http://")
}

fn image_body(c: &ImageChannel) -> Result<Value, String> {
    let format = enum_name(&FORMATS, c.format).ok_or_else(|| format!("format {}", c.format))?;
    let mode = enum_name(&MODES, c.mode).ok_or_else(|| format!("mode {}", c.mode))?;

    let mut body = json!({
        "Base": c.base,
        "Format": format,
        "Mode": mode,
        "IntegrationSize": c.integration_size,
        "QueueSize": c.queue_size,
    });
    // C only sends IntegrationMode when the size is neither 0 nor 1
    // (serval_http.cpp:1800).
    if c.integration_size != 0 && c.integration_size != 1 {
        let m = enum_name(&INTEGRATION_MODES, c.integration_mode)
            .ok_or_else(|| format!("integration mode {}", c.integration_mode))?;
        body["IntegrationMode"] = json!(m);
    }
    if !is_stream(&c.base) {
        body["FilePattern"] = json!(c.file_pattern);
        // UPSTREAM DEFECT (serval_http.cpp:1518): C sends this as the *string*
        // "true"/"false" while Serval reports it back as a JSON boolean
        // (serval_http.cpp:1361). Sent as a boolean.
        body["StopMeasurementOnDiskLimit"] = json!(c.stop_on_disk_limit);
    }
    Ok(body)
}

fn raw_body(c: &RawChannel) -> Result<Value, String> {
    let mut body = json!({ "Base": c.base, "QueueSize": c.queue_size });
    if !is_stream(&c.base) {
        let split = enum_name(&SPLIT_STRATEGIES, c.split_strategy)
            .ok_or_else(|| format!("split strategy {}", c.split_strategy))?;
        body["FilePattern"] = json!(c.file_pattern);
        body["SplitStrategy"] = json!(split);
    }
    Ok(body)
}

/// The full `PUT /server/destination` body (C `fileWriter`,
/// serval_http.cpp:2095).
pub fn destination_body(d: &Destination) -> Result<Value, String> {
    let mut body = json!({});
    if !d.raw.is_empty() {
        body["Raw"] = Value::Array(d.raw.iter().map(raw_body).collect::<Result<_, _>>()?);
    }
    if !d.image.is_empty() {
        body["Image"] = Value::Array(d.image.iter().map(image_body).collect::<Result<_, _>>()?);
    }

    let mut preview = json!({});
    let mut has_preview = false;
    if !d.preview_image.is_empty() {
        preview["ImageChannels"] = Value::Array(
            d.preview_image
                .iter()
                .map(image_body)
                .collect::<Result<_, _>>()?,
        );
        has_preview = true;
    }
    if let Some(h) = &d.preview_histogram {
        let mut hb = image_body(&h.image)?;
        hb["NumberOfBins"] = json!(h.number_of_bins);
        hb["BinWidth"] = json!(h.bin_width);
        hb["Offset"] = json!(h.offset);
        preview["HistogramChannels"] = Value::Array(vec![hb]);
        has_preview = true;
    }
    if has_preview {
        if let Some(p) = d.preview {
            let sampling = enum_name(&SAMPLING_MODES, p.sampling_mode)
                .ok_or_else(|| format!("sampling mode {}", p.sampling_mode))?;
            preview["Period"] = json!(p.period);
            preview["SamplingMode"] = json!(sampling);
        }
        body["Preview"] = preview;
    }
    Ok(body)
}

// ---------------------------------------------------------------------------
// /detector/config (C `initAcquisition`, serval_http.cpp:2254)
// ---------------------------------------------------------------------------

/// The detector settings the driver owns.
#[derive(Debug, Clone, PartialEq)]
pub struct DetectorConfig {
    pub trigger_mode: i32,
    pub exposure_time: f64,
    pub trigger_period: f64,
    pub trigger_delay: f64,
    pub global_timestamp_interval: f64,
    pub n_triggers: i32,
    pub bias_voltage: i32,
    pub bias_enabled: bool,
    pub chain_mode: i32,
    pub polarity: i32,
    pub trigger_in: i32,
    pub trigger_out: i32,
    pub log_level: i32,
    pub external_reference_clock: bool,
    pub periph_clk80: bool,
    pub tdc0: i32,
    pub tdc1: i32,
}

/// Merge the driver's settings into the config Serval reported, which is what C
/// PUTs back (it GETs `/detector/config` first so unknown keys survive).
///
/// UPSTREAM DEFECT (serval_http.cpp:2320-2375): C writes `BiasEnabled`,
/// `ExternalReferenceClock` and `PeriphClk80` as the JSON *strings* `"true"` /
/// `"false"` while `getDetector` (serval_http.cpp:1269) reads the same fields
/// back as booleans. They are booleans here.
pub fn detector_config_body(current: &Value, c: &DetectorConfig) -> Result<Value, String> {
    let mut body = current.clone();
    if !body.is_object() {
        body = json!({});
    }
    let trigger = enum_name(&TRIGGER_MODES, c.trigger_mode)
        .ok_or_else(|| format!("trigger mode {}", c.trigger_mode))?;
    let chain = enum_name(&CHAIN_MODES, c.chain_mode)
        .ok_or_else(|| format!("chain mode {}", c.chain_mode))?;
    let polarity =
        enum_name(&POLARITIES, c.polarity).ok_or_else(|| format!("polarity {}", c.polarity))?;
    let tdc0 = enum_name(&TDC_MODES, c.tdc0).ok_or_else(|| format!("tdc0 {}", c.tdc0))?;
    let tdc1 = enum_name(&TDC_MODES, c.tdc1).ok_or_else(|| format!("tdc1 {}", c.tdc1))?;

    body["TriggerMode"] = json!(trigger);
    body["ExposureTime"] = json!(c.exposure_time);
    body["TriggerPeriod"] = json!(c.trigger_period);
    body["TriggerDelay"] = json!(c.trigger_delay);
    body["GlobalTimestampInterval"] = json!(c.global_timestamp_interval);
    body["nTriggers"] = json!(c.n_triggers);
    body["BiasVoltage"] = json!(c.bias_voltage);
    body["BiasEnabled"] = json!(c.bias_enabled);
    body["ChainMode"] = json!(chain);
    body["Polarity"] = json!(polarity);
    body["TriggerIn"] = json!(c.trigger_in);
    body["TriggerOut"] = json!(c.trigger_out);
    body["LogLevel"] = json!(c.log_level);
    body["ExternalReferenceClock"] = json!(c.external_reference_clock);
    body["PeriphClk80"] = json!(c.periph_clk80);
    body["Tdc"] = json!([tdc0, tdc1]);
    Ok(body)
}

// ---------------------------------------------------------------------------
// /measurement/config (C `sendMeasurementConfig`, serval_http.cpp:2033)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
pub struct MeasurementConfig {
    pub scan_width: i32,
    pub scan_height: i32,
    pub dwell_time: f64,
    pub radius_outer: i32,
    pub radius_inner: i32,
    pub tdc_reference: String,
    pub tof_min: f64,
    pub tof_max: f64,
}

/// The `TdcReference` array: C comma-splits the PV and falls back to
/// `["PN0123"]` when it is empty (serval_http.cpp:2062).
pub fn tdc_references(value: &str) -> Vec<String> {
    let refs: Vec<String> = value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .collect();
    if refs.is_empty() {
        vec!["PN0123".to_string()]
    } else {
        refs
    }
}

pub fn measurement_config_body(current: &Value, c: &MeasurementConfig) -> Value {
    let mut body = current.clone();
    if !body.is_object() {
        body = json!({});
    }
    body["Stem"]["Scan"]["Width"] = json!(c.scan_width);
    body["Stem"]["Scan"]["Height"] = json!(c.scan_height);
    body["Stem"]["Scan"]["DwellTime"] = json!(c.dwell_time);
    body["Stem"]["VirtualDetector"]["RadiusOuter"] = json!(c.radius_outer);
    body["Stem"]["VirtualDetector"]["RadiusInner"] = json!(c.radius_inner);
    body["TimeOfFlight"]["TdcReference"] = json!(tdc_references(&c.tdc_reference));
    body["TimeOfFlight"]["Min"] = json!(c.tof_min);
    body["TimeOfFlight"]["Max"] = json!(c.tof_max);
    body
}

// ---------------------------------------------------------------------------
// Reading Serval's replies
// ---------------------------------------------------------------------------

/// A validated `GET /measurement` reply (C `StatusSnapshot`,
/// serval_measurement.h); `None` is a field Serval did not send.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct StatusSnapshot {
    pub pixel_event_rate: Option<i32>,
    pub tdc1_event_rate: Option<i32>,
    pub tdc2_event_rate: Option<i32>,
    pub start_date_time: Option<i64>,
    pub elapsed_time: Option<f64>,
    pub time_left: Option<f64>,
    pub frame_count: Option<i32>,
    pub dropped_frames: Option<i32>,
    pub status: Option<String>,
}

impl StatusSnapshot {
    fn status_is_idle(&self) -> Option<bool> {
        self.status
            .as_deref()
            .map(|s| s == "DA_IDLE" || s == "DA_STOPPED")
    }

    /// A measurement `acquireStart` has to stop first (acquire.cpp:371-372).
    pub fn is_running(&self) -> bool {
        self.status_is_idle() == Some(false)
    }

    /// What ends `timePixCallback`'s poll (acquire.cpp:630-632).
    pub fn is_idle(&self) -> bool {
        self.status_is_idle() == Some(true)
    }

    /// C `isRemoteIdle` (acquisition_runtime.cpp:26): no status counts as
    /// stopped.
    pub fn is_remote_idle(&self) -> bool {
        self.status_is_idle() != Some(false)
    }
}

/// C `StatusResponseError` (serval_measurement.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusResponseError {
    EmptyBody,
    MalformedJson,
    InvalidRoot,
    InvalidInfo,
    InvalidStatus,
    InvalidMetric,
}

impl StatusResponseError {
    /// C `statusResponseErrorMessage` (serval_measurement.cpp:198).
    pub fn message(self) -> &'static str {
        match self {
            Self::EmptyBody => "empty measurement response",
            Self::MalformedJson => "malformed measurement JSON",
            Self::InvalidRoot => "measurement JSON root is not an object",
            Self::InvalidInfo => "measurement Info field is not an object",
            Self::InvalidStatus => "measurement Status field is not a string",
            Self::InvalidMetric => "measurement metric has an invalid value",
        }
    }
}

/// C `boundedInt`: any finite number in i32 range, truncated.
fn bounded_int(v: &Value) -> Option<i32> {
    let f = v.as_f64()?;
    (f.is_finite() && f >= f64::from(i32::MIN) && f <= f64::from(i32::MAX)).then_some(f as i32)
}

/// C `boundedInt64`: an integer in i64 range.
fn bounded_int64(v: &Value) -> Option<i64> {
    match v.as_u64() {
        Some(u) => i64::try_from(u).ok(),
        None => v.as_i64(),
    }
}

/// C `finiteDouble`.
fn finite_double(v: &Value) -> Option<f64> {
    v.as_f64().filter(|f| f.is_finite())
}

/// C `parseStatusResponse` (serval_measurement.cpp:131): a present field of
/// the wrong kind fails the whole reply, an absent one is left `None`.
pub fn parse_status_response(body: &str) -> Result<StatusSnapshot, StatusResponseError> {
    use StatusResponseError as E;
    if body.is_empty() {
        return Err(E::EmptyBody);
    }
    let m: Value = serde_json::from_str(body).map_err(|_| E::MalformedJson)?;
    let Value::Object(m) = m else {
        return Err(E::InvalidRoot);
    };

    fn field<T>(
        info: &serde_json::Map<String, Value>,
        name: &str,
        parse: fn(&Value) -> Option<T>,
    ) -> Result<Option<T>, StatusResponseError> {
        info.get(name)
            .map(|v| parse(v).ok_or(StatusResponseError::InvalidMetric))
            .transpose()
    }
    fn status(v: &Value) -> Result<String, StatusResponseError> {
        v.as_str()
            .map(str::to_string)
            .ok_or(StatusResponseError::InvalidStatus)
    }

    let mut snapshot = StatusSnapshot::default();
    let info = match m.get("Info") {
        Some(Value::Null) => None,
        Some(Value::Object(info)) => Some(info),
        Some(_) => return Err(E::InvalidInfo),
        None => None,
    };
    if let Some(info) = info {
        snapshot.pixel_event_rate = field(info, "PixelEventRate", bounded_int)?;
        snapshot.tdc1_event_rate = field(info, "Tdc1EventRate", bounded_int)?;
        snapshot.tdc2_event_rate = field(info, "Tdc2EventRate", bounded_int)?;
        snapshot.start_date_time = field(info, "StartDateTime", bounded_int64)?;
        snapshot.elapsed_time = field(info, "ElapsedTime", finite_double)?;
        snapshot.time_left = field(info, "TimeLeft", finite_double)?;
        snapshot.frame_count = field(info, "FrameCount", bounded_int)?;
        snapshot.dropped_frames = field(info, "DroppedFrames", bounded_int)?;
        if snapshot.tdc1_event_rate.is_none() {
            snapshot.tdc1_event_rate = field(info, "TdcEventRate", bounded_int)?;
        }
        snapshot.status = info.get("Status").map(status).transpose()?;
    }
    if snapshot.status.is_none() {
        snapshot.status = m.get("Status").map(status).transpose()?;
    }
    if snapshot.status.is_none() && m.get("Info") == Some(&Value::Null) {
        snapshot.status = Some("DA_IDLE".into());
    }
    Ok(snapshot)
}

/// C `ConfigResponseError` (serval_measurement.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigResponseError {
    HttpFailure,
    EmptyBody,
    MalformedJson,
    InvalidRoot,
}

impl ConfigResponseError {
    /// C `configResponseErrorMessage` (serval_measurement.cpp:235-250).
    pub fn message(self) -> &'static str {
        match self {
            Self::HttpFailure => "measurement configuration GET failed",
            Self::EmptyBody => "empty measurement configuration response",
            Self::MalformedJson => "malformed measurement configuration JSON",
            Self::InvalidRoot => "measurement configuration JSON root is not an object",
        }
    }
}

/// C `parseConfigResponse` (serval_measurement.cpp:212-233) for a 200 reply.
pub fn parse_config_response(body: &str) -> Result<Value, ConfigResponseError> {
    if body.is_empty() {
        return Err(ConfigResponseError::EmptyBody);
    }
    let config: Value =
        serde_json::from_str(body).map_err(|_| ConfigResponseError::MalformedJson)?;
    if config.is_object() {
        Ok(config)
    } else {
        Err(ConfigResponseError::InvalidRoot)
    }
}

/// C `ADTimePix3ServalDashboard::ResponseError` (serval_dashboard.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DashboardError {
    HttpFailure,
    EmptyBody,
    MalformedJson,
    InvalidRoot,
    MissingServer,
    InvalidServer,
    MissingDetector,
    InvalidDetector,
    InvalidDetectorType,
    InvalidMeasurement,
    InvalidSoftwareVersion,
    InvalidSoftwareTimestamp,
    InvalidDiskSpace,
    InvalidDiskEntry,
    InvalidDiskValue,
}

impl DashboardError {
    /// C `responseErrorMessage` (serval_dashboard.cpp:100-121).
    pub fn message(self) -> &'static str {
        match self {
            Self::HttpFailure => "dashboard GET failed",
            Self::EmptyBody => "empty dashboard response",
            Self::MalformedJson => "malformed dashboard JSON",
            Self::InvalidRoot => "dashboard root is not an object",
            Self::MissingServer => "dashboard Server field is missing",
            Self::InvalidServer => "dashboard Server field is not an object",
            Self::MissingDetector => "dashboard Detector field is missing",
            Self::InvalidDetector => "dashboard Detector field is neither null nor an object",
            Self::InvalidDetectorType => "dashboard detector type is missing or invalid",
            Self::InvalidMeasurement => "dashboard Measurement field is invalid",
            Self::InvalidSoftwareVersion => "dashboard software version is invalid",
            Self::InvalidSoftwareTimestamp => "dashboard software timestamp is invalid",
            Self::InvalidDiskSpace => "dashboard DiskSpace field is not an array",
            Self::InvalidDiskEntry => "dashboard DiskSpace entry is not an object",
            Self::InvalidDiskValue => "dashboard DiskSpace value has an invalid type",
        }
    }
}

/// A `GET /dashboard` reply that passed C `parseResponse`.
#[derive(Debug, Clone, PartialEq)]
pub struct Dashboard {
    pub response: Value,
    /// `Detector.DetectorType`; `None` when `Detector` is null.
    pub detector_type: Option<String>,
}

/// C `ADTimePix3ServalDashboard::parseResponse` (serval_dashboard.cpp:49-98):
/// the whole reply is checked before any of it is published.
pub fn parse_dashboard_response(code: i32, body: &str) -> Result<Dashboard, DashboardError> {
    use DashboardError as E;
    if code != 200 {
        return Err(E::HttpFailure);
    }
    if body.is_empty() {
        return Err(E::EmptyBody);
    }
    let response: Value = serde_json::from_str(body).map_err(|_| E::MalformedJson)?;
    let root = response.as_object().ok_or(E::InvalidRoot)?;
    let server = root.get("Server").ok_or(E::MissingServer)?;
    let server = server.as_object().ok_or(E::InvalidServer)?;
    let detector_type = match root.get("Detector").ok_or(E::MissingDetector)? {
        Value::Null => None,
        Value::Object(detector) => Some(
            detector
                .get("DetectorType")
                .and_then(Value::as_str)
                .filter(|t| !t.is_empty())
                .ok_or(E::InvalidDetectorType)?
                .to_string(),
        ),
        _ => return Err(E::InvalidDetector),
    };
    if root
        .get("Measurement")
        .is_some_and(|m| !m.is_null() && !m.is_object())
    {
        return Err(E::InvalidMeasurement);
    }
    let not_string = |key: &str| server.get(key).is_some_and(|v| !v.is_string());
    if not_string("SoftwareVersion") {
        return Err(E::InvalidSoftwareVersion);
    }
    if not_string("SoftwareTimestamp") {
        return Err(E::InvalidSoftwareTimestamp);
    }
    if let Some(disks) = server.get("DiskSpace") {
        for disk in disks.as_array().ok_or(E::InvalidDiskSpace)? {
            let disk = disk.as_object().ok_or(E::InvalidDiskEntry)?;
            // An absent or null value is accepted (`validOptional*`).
            let valid = |key: &str, ok: fn(&Value) -> bool| {
                disk.get(key).is_none_or(|v| v.is_null() || ok(v))
            };
            if !(valid("FreeSpace", |v| v.as_i64().is_some())
                && valid("WriteSpeed", Value::is_number)
                && valid("LowerLimit", |v| v.as_i64().is_some())
                && valid("DiskLimitReached", |v| {
                    v.is_boolean() || v.as_i64().and_then(|n| i32::try_from(n).ok()).is_some()
                }))
            {
                return Err(E::InvalidDiskValue);
            }
        }
    }
    Ok(Dashboard {
        response,
        detector_type,
    })
}

/// C `ADTimePix3ServalDetector::ParseError` (serval_detector.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorParseError {
    EmptyBody,
    MalformedJson,
    InvalidRoot,
    MissingInfo,
    MissingConfig,
    InvalidGeometry,
}

impl DetectorParseError {
    /// C `parseErrorMessage` (serval_detector.cpp:90-109).
    pub fn message(self) -> &'static str {
        match self {
            Self::EmptyBody => "empty detector response",
            Self::MalformedJson => "malformed detector JSON",
            Self::InvalidRoot => "detector JSON root is not an object",
            Self::MissingInfo => "detector response is missing the Info object",
            Self::MissingConfig => "detector response is missing the Config object",
            Self::InvalidGeometry => "detector response contains invalid geometry",
        }
    }
}

/// C `ADTimePix3ServalDetector::parseResponse` (serval_detector.cpp:60-88):
/// Info and Config are objects and the geometry is whole.
pub fn parse_detector_response(body: &str) -> Result<Value, DetectorParseError> {
    use DetectorParseError as E;
    if body.is_empty() {
        return Err(E::EmptyBody);
    }
    let detector: Value = serde_json::from_str(body).map_err(|_| E::MalformedJson)?;
    if !detector.is_object() {
        return Err(E::InvalidRoot);
    }
    let info = detector
        .get("Info")
        .filter(|v| v.is_object())
        .ok_or(E::MissingInfo)?;
    detector
        .get("Config")
        .filter(|v| v.is_object())
        .ok_or(E::MissingConfig)?;
    // C `positiveInt`/`nonnegativeInt`: an integer in int range.
    let int = |name: &str, min: i64| {
        info.get(name)
            .and_then(Value::as_i64)
            .filter(|&n| n >= min && n <= i64::from(i32::MAX))
    };
    let geometry = (|| {
        let pix_count = int("PixCount", 1)?;
        int("RowLen", 1)?;
        int("NumberOfChips", 1)?;
        let rows = int("NumberOfRows", 1)?;
        int("MpxType", 0)?;
        (pix_count % rows == 0).then_some(())
    })();
    geometry.ok_or(E::InvalidGeometry)?;
    Ok(detector)
}

/// A validated `GET /measurement/config` reply (C `ConfigSnapshot`,
/// serval_measurement.h); `None` is a field Serval did not send.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ConfigSnapshot {
    pub stem_scan_width: Option<i32>,
    pub stem_scan_height: Option<i32>,
    pub stem_dwell_time: Option<f64>,
    pub stem_radius_outer: Option<i32>,
    pub stem_radius_inner: Option<i32>,
    pub tof_tdc_reference: Option<String>,
    pub tof_min: Option<f64>,
    pub tof_max: Option<f64>,
}

/// C `ConfigReadbackError` (serval_measurement.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigReadbackError {
    HttpFailure,
    EmptyBody,
    MalformedJson,
    InvalidRoot,
    InvalidStem,
    InvalidScan,
    InvalidVirtualDetector,
    InvalidTimeOfFlight,
    InvalidMetric,
    InvalidTdcReference,
}

impl ConfigReadbackError {
    /// C `configReadbackErrorMessage` (serval_measurement.cpp:322-338).
    pub fn message(self) -> &'static str {
        match self {
            Self::HttpFailure => "measurement configuration GET failed",
            Self::EmptyBody => "empty measurement configuration response",
            Self::MalformedJson => "malformed measurement configuration JSON",
            Self::InvalidRoot => "measurement configuration root is not an object",
            Self::InvalidStem => "measurement Stem field is not an object",
            Self::InvalidScan => "measurement Stem.Scan field is not an object",
            Self::InvalidVirtualDetector => {
                "measurement Stem.VirtualDetector field is not an object"
            }
            Self::InvalidTimeOfFlight => "measurement TimeOfFlight field is not an object",
            Self::InvalidMetric => "measurement configuration metric has an invalid value",
            Self::InvalidTdcReference => "measurement TdcReference field is invalid",
        }
    }
}

impl From<ConfigResponseError> for ConfigReadbackError {
    fn from(e: ConfigResponseError) -> Self {
        match e {
            ConfigResponseError::HttpFailure => Self::HttpFailure,
            ConfigResponseError::EmptyBody => Self::EmptyBody,
            ConfigResponseError::MalformedJson => Self::MalformedJson,
            ConfigResponseError::InvalidRoot => Self::InvalidRoot,
        }
    }
}

/// C `parseConfigReadback` (serval_measurement.cpp:252-320) for a 200 reply:
/// a null field is absent, a present one of the wrong kind fails the reply.
pub fn parse_config_readback(body: &str) -> Result<ConfigSnapshot, ConfigReadbackError> {
    use ConfigReadbackError as E;
    let config = parse_config_response(body)?;

    /// The field unless it is absent or null.
    fn present<'a>(object: &'a Value, name: &str) -> Option<&'a Value> {
        object.get(name).filter(|v| !v.is_null())
    }
    /// C `optionalConfigInt`: an integer (not a float) in i32 range.
    fn int(object: &Value, name: &str) -> Result<Option<i32>, ConfigReadbackError> {
        present(object, name)
            .map(|v| {
                v.as_i64()
                    .and_then(|n| i32::try_from(n).ok())
                    .ok_or(E::InvalidMetric)
            })
            .transpose()
    }
    /// C `optionalConfigDouble`.
    fn double(object: &Value, name: &str) -> Result<Option<f64>, ConfigReadbackError> {
        present(object, name)
            .map(|v| finite_double(v).ok_or(E::InvalidMetric))
            .transpose()
    }

    let mut snapshot = ConfigSnapshot::default();
    if let Some(stem) = present(&config, "Stem") {
        if !stem.is_object() {
            return Err(E::InvalidStem);
        }
        if let Some(scan) = present(stem, "Scan") {
            if !scan.is_object() {
                return Err(E::InvalidScan);
            }
            snapshot.stem_scan_width = int(scan, "Width")?;
            snapshot.stem_scan_height = int(scan, "Height")?;
            snapshot.stem_dwell_time = double(scan, "DwellTime")?;
        }
        if let Some(detector) = present(stem, "VirtualDetector") {
            if !detector.is_object() {
                return Err(E::InvalidVirtualDetector);
            }
            snapshot.stem_radius_outer = int(detector, "RadiusOuter")?;
            snapshot.stem_radius_inner = int(detector, "RadiusInner")?;
        }
    }
    if let Some(tof) = present(&config, "TimeOfFlight") {
        if !tof.is_object() {
            return Err(E::InvalidTimeOfFlight);
        }
        if let Some(references) = present(tof, "TdcReference") {
            let references = references
                .as_array()
                .ok_or(E::InvalidTdcReference)?
                .iter()
                .map(|r| r.as_str().ok_or(E::InvalidTdcReference))
                .collect::<Result<Vec<_>, _>>()?;
            snapshot.tof_tdc_reference = Some(references.join(","));
        }
        snapshot.tof_min = double(tof, "Min")?;
        snapshot.tof_max = double(tof, "Max")?;
    }
    Ok(snapshot)
}

/// C `strip_quotes` (serval_http.cpp:91).
///
/// UPSTREAM DEFECT (serval_http.cpp:91-95): C removes the first and last
/// character unconditionally, so a `dump()` of a non-string (`null`, a number,
/// an array) is mangled — `null` becomes `ul`, and that mangled key is then
/// used to look up the detector orientation in a `std::map`, whose
/// `operator[]` *inserts* it with value 0 and silently reports orientation UP
/// (serval_http.cpp:1296). Here a JSON string yields its contents and anything
/// else its compact encoding.
pub fn json_to_string(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        Value::Null => String::new(),
        other => other.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C `parseResponse` (serval_dashboard.cpp:49-98).
    #[test]
    fn the_dashboard_reply_is_validated_as_c() {
        use DashboardError as E;
        let parse = |body: &str| parse_dashboard_response(200, body).map(|d| d.detector_type);
        assert_eq!(
            parse(r#"{"Server":{},"Detector":{"DetectorType":"TPX3"}}"#),
            Ok(Some("TPX3".into()))
        );
        assert_eq!(
            parse(r#"{"Server":{},"Detector":null,"Measurement":null}"#),
            Ok(None)
        );
        assert_eq!(
            parse(
                r#"{"Server":{"SoftwareVersion":"3","SoftwareTimestamp":"t","DiskSpace":[
                {"FreeSpace":1,"WriteSpeed":1.5,"LowerLimit":null,"DiskLimitReached":true},
                {"DiskLimitReached":1}]},"Detector":null}"#
            ),
            Ok(None)
        );
        assert_eq!(
            parse_dashboard_response(404, r#"{"Server":{},"Detector":null}"#),
            Err(E::HttpFailure)
        );
        for (body, error) in [
            ("", E::EmptyBody),
            ("{", E::MalformedJson),
            ("[]", E::InvalidRoot),
            (r#"{"Detector":null}"#, E::MissingServer),
            (r#"{"Server":1,"Detector":null}"#, E::InvalidServer),
            (r#"{"Server":{}}"#, E::MissingDetector),
            (r#"{"Server":{},"Detector":"TPX3"}"#, E::InvalidDetector),
            (r#"{"Server":{},"Detector":{}}"#, E::InvalidDetectorType),
            (
                r#"{"Server":{},"Detector":{"DetectorType":""}}"#,
                E::InvalidDetectorType,
            ),
            (
                r#"{"Server":{},"Detector":null,"Measurement":1}"#,
                E::InvalidMeasurement,
            ),
            (
                r#"{"Server":{"SoftwareVersion":3},"Detector":null}"#,
                E::InvalidSoftwareVersion,
            ),
            (
                r#"{"Server":{"SoftwareTimestamp":3},"Detector":null}"#,
                E::InvalidSoftwareTimestamp,
            ),
            (
                r#"{"Server":{"DiskSpace":{}},"Detector":null}"#,
                E::InvalidDiskSpace,
            ),
            (
                r#"{"Server":{"DiskSpace":[1]},"Detector":null}"#,
                E::InvalidDiskEntry,
            ),
            (
                r#"{"Server":{"DiskSpace":[{"FreeSpace":1.5}]},"Detector":null}"#,
                E::InvalidDiskValue,
            ),
            (
                r#"{"Server":{"DiskSpace":[{"WriteSpeed":"x"}]},"Detector":null}"#,
                E::InvalidDiskValue,
            ),
            (
                r#"{"Server":{"DiskSpace":[{"LowerLimit":9223372036854775808}]},"Detector":null}"#,
                E::InvalidDiskValue,
            ),
            (
                r#"{"Server":{"DiskSpace":[{"DiskLimitReached":2147483648}]},"Detector":null}"#,
                E::InvalidDiskValue,
            ),
        ] {
            assert_eq!(parse(body), Err(error), "{body}");
        }
    }

    /// C `parseResponse` (serval_detector.cpp:60-88).
    #[test]
    fn the_detector_reply_is_validated_as_c() {
        use DetectorParseError as E;
        let info = |extra: &str| {
            format!(
                r#"{{"Info":{{"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256,"MpxType":0{extra}}},"Config":{{}}}}"#
            )
        };
        assert!(parse_detector_response(&info("")).is_ok());
        assert_eq!(parse_detector_response(""), Err(E::EmptyBody));
        assert_eq!(parse_detector_response("{"), Err(E::MalformedJson));
        assert_eq!(parse_detector_response("[]"), Err(E::InvalidRoot));
        assert_eq!(
            parse_detector_response(r#"{"Info":1,"Config":{}}"#),
            Err(E::MissingInfo)
        );
        assert_eq!(
            parse_detector_response(r#"{"Info":{}}"#),
            Err(E::MissingConfig)
        );
        for bad in [
            r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":0,"MpxType":0},"Config":{}}"#,
            r#"{"Info":{"PixCount":65537,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256,"MpxType":0},"Config":{}}"#,
            r#"{"Info":{"PixCount":65536,"RowLen":1.5,"NumberOfChips":1,"NumberOfRows":256,"MpxType":0},"Config":{}}"#,
            r#"{"Info":{"PixCount":65536,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256,"MpxType":-1},"Config":{}}"#,
            r#"{"Info":{"PixCount":4294967296,"RowLen":1,"NumberOfChips":1,"NumberOfRows":256,"MpxType":0},"Config":{}}"#,
            r#"{"Info":{"RowLen":1,"NumberOfChips":1,"NumberOfRows":256,"MpxType":0},"Config":{}}"#,
        ] {
            assert_eq!(
                parse_detector_response(bad),
                Err(E::InvalidGeometry),
                "{bad}"
            );
        }
    }

    fn image(base: &str) -> ImageChannel {
        ImageChannel {
            base: base.to_string(),
            file_pattern: "f%d".into(),
            format: 3, // jsonimage
            mode: 0,   // count
            integration_size: 0,
            integration_mode: 0,
            stop_on_disk_limit: true,
            queue_size: 1024,
        }
    }

    #[test]
    fn an_out_of_range_enum_is_an_error_not_a_null() {
        assert_eq!(enum_name(&FORMATS, 3), Some("jsonimage"));
        assert_eq!(enum_name(&FORMATS, 5), None);
        assert_eq!(enum_name(&FORMATS, -1), None);
        let mut c = image("file:///data");
        c.format = 9;
        assert!(image_body(&c).is_err());
    }

    #[test]
    fn a_file_channel_carries_the_file_keys() {
        let body = image_body(&image("file:///data")).unwrap();
        assert_eq!(body["FilePattern"], json!("f%d"));
        // A boolean, not the string "true" (C, serval_http.cpp:1518).
        assert_eq!(body["StopMeasurementOnDiskLimit"], json!(true));
        assert_eq!(body["Format"], json!("jsonimage"));
        assert_eq!(body["Mode"], json!("count"));
    }

    #[test]
    fn a_stream_channel_carries_no_file_keys() {
        for base in ["tcp://localhost:8451", "http://localhost:8451"] {
            let body = image_body(&image(base)).unwrap();
            assert!(body.get("FilePattern").is_none(), "{base}");
            assert!(body.get("StopMeasurementOnDiskLimit").is_none(), "{base}");
            assert_eq!(body["Base"], json!(base));
        }
    }

    #[test]
    fn integration_mode_is_only_sent_for_a_real_integration() {
        let mut c = image("file:///data");
        for size in [0, 1] {
            c.integration_size = size;
            assert!(image_body(&c).unwrap().get("IntegrationMode").is_none());
        }
        c.integration_size = 10;
        c.integration_mode = 1;
        assert_eq!(image_body(&c).unwrap()["IntegrationMode"], json!("average"));
    }

    #[test]
    fn the_destination_body_nests_the_preview_channels() {
        let d = Destination {
            raw: vec![RawChannel {
                base: "file:///raw".into(),
                file_pattern: "r%d".into(),
                split_strategy: 1,
                queue_size: 16384,
            }],
            image: vec![image("file:///img")],
            preview_image: vec![image("tcp://localhost:8451")],
            preview_histogram: Some(HistogramChannel {
                image: image("tcp://localhost:8452"),
                number_of_bins: 16,
                bin_width: 1.0,
                offset: 0.0,
            }),
            preview: Some(PreviewSettings {
                period: 0.2,
                sampling_mode: 1,
            }),
        };
        let body = destination_body(&d).unwrap();
        assert_eq!(body["Raw"][0]["SplitStrategy"], json!("frame"));
        assert_eq!(body["Image"][0]["Base"], json!("file:///img"));
        assert_eq!(
            body["Preview"]["ImageChannels"][0]["Base"],
            json!("tcp://localhost:8451")
        );
        assert_eq!(body["Preview"]["HistogramChannels"][0]["NumberOfBins"], 16);
        assert_eq!(body["Preview"]["SamplingMode"], json!("skipOnPeriod"));
        assert_eq!(body["Preview"]["Period"], json!(0.2));
    }

    #[test]
    fn a_raw_stream_channel_keeps_no_split_strategy() {
        let d = Destination {
            raw: vec![RawChannel {
                base: "tcp://localhost:8450".into(),
                file_pattern: "r%d".into(),
                split_strategy: 0,
                queue_size: 16,
            }],
            ..Default::default()
        };
        let body = destination_body(&d).unwrap();
        assert!(body["Raw"][0].get("SplitStrategy").is_none());
        assert!(body.get("Preview").is_none());
    }

    fn config() -> DetectorConfig {
        DetectorConfig {
            trigger_mode: 5,
            exposure_time: 0.5,
            trigger_period: 1.0,
            trigger_delay: 0.0,
            global_timestamp_interval: 0.0,
            n_triggers: 1,
            bias_voltage: 103,
            bias_enabled: true,
            chain_mode: 0,
            polarity: 0,
            trigger_in: 0,
            trigger_out: 0,
            log_level: 1,
            external_reference_clock: false,
            periph_clk80: false,
            tdc0: 2,
            tdc1: 2,
        }
    }

    #[test]
    fn the_detector_config_sends_booleans_as_booleans() {
        let body = detector_config_body(&json!({"Unknown": 7}), &config()).unwrap();
        // C sends the strings "true"/"false" here (serval_http.cpp:2320).
        assert_eq!(body["BiasEnabled"], json!(true));
        assert_eq!(body["ExternalReferenceClock"], json!(false));
        assert_eq!(body["PeriphClk80"], json!(false));
        assert_eq!(body["TriggerMode"], json!("CONTINUOUS"));
        assert_eq!(body["Tdc"], json!(["PN0123", "PN0123"]));
        assert_eq!(body["BiasVoltage"], json!(103));
        // Keys Serval reported that the driver does not own survive the merge.
        assert_eq!(body["Unknown"], json!(7));
    }

    #[test]
    fn an_out_of_range_trigger_mode_fails_instead_of_sending_null() {
        let mut c = config();
        c.trigger_mode = 9;
        assert!(detector_config_body(&json!({}), &c).is_err());
        c.trigger_mode = -1;
        assert!(detector_config_body(&json!({}), &c).is_err());
    }

    #[test]
    fn tdc_references_split_on_commas_and_default() {
        assert_eq!(tdc_references(""), vec!["PN0123".to_string()]);
        assert_eq!(tdc_references("  "), vec!["PN0123".to_string()]);
        assert_eq!(
            tdc_references("P0123, N0123"),
            vec!["P0123".to_string(), "N0123".to_string()]
        );
    }

    #[test]
    fn the_measurement_config_body_nests_stem_and_tof() {
        let c = MeasurementConfig {
            scan_width: 64,
            scan_height: 32,
            dwell_time: 1e-3,
            radius_outer: 10,
            radius_inner: 2,
            tdc_reference: "P0123".into(),
            tof_min: 0.0,
            tof_max: 1.0,
        };
        let body = measurement_config_body(&json!({"Keep": 1}), &c);
        assert_eq!(body["Stem"]["Scan"]["Width"], json!(64));
        assert_eq!(body["Stem"]["VirtualDetector"]["RadiusInner"], json!(2));
        assert_eq!(body["TimeOfFlight"]["TdcReference"], json!(["P0123"]));
        assert_eq!(body["TimeOfFlight"]["Max"], json!(1.0));
        assert_eq!(body["Keep"], json!(1));
    }

    #[test]
    fn the_status_predicates_follow_their_c_callers() {
        let snap = |status: Option<&str>| StatusSnapshot {
            status: status.map(str::to_string),
            ..StatusSnapshot::default()
        };
        for idle in ["DA_IDLE", "DA_STOPPED"] {
            assert!(!snap(Some(idle)).is_running());
            assert!(snap(Some(idle)).is_idle());
            assert!(snap(Some(idle)).is_remote_idle());
        }
        assert!(snap(Some("DA_RECORDING")).is_running());
        assert!(!snap(Some("DA_RECORDING")).is_idle());
        assert!(!snap(Some("DA_RECORDING")).is_remote_idle());
        // No status: not running, not idle for the poll, idle for the stop.
        assert!(!snap(None).is_running());
        assert!(!snap(None).is_idle());
        assert!(snap(None).is_remote_idle());
    }

    /// C `parseConfigReadback` (serval_measurement.cpp:252-320).
    #[test]
    fn a_measurement_config_reply_is_validated_as_c_does() {
        use ConfigReadbackError as E;
        let parse = parse_config_readback;
        assert_eq!(parse(""), Err(E::EmptyBody));
        assert_eq!(parse("{"), Err(E::MalformedJson));
        assert_eq!(parse("[]"), Err(E::InvalidRoot));
        assert_eq!(parse(r#"{"Stem": 1}"#), Err(E::InvalidStem));
        assert_eq!(parse(r#"{"Stem": {"Scan": []}}"#), Err(E::InvalidScan));
        assert_eq!(
            parse(r#"{"Stem": {"VirtualDetector": "x"}}"#),
            Err(E::InvalidVirtualDetector)
        );
        assert_eq!(parse(r#"{"TimeOfFlight": 2}"#), Err(E::InvalidTimeOfFlight));
        assert_eq!(
            parse(r#"{"TimeOfFlight": {"TdcReference": "PN0123"}}"#),
            Err(E::InvalidTdcReference)
        );
        assert_eq!(
            parse(r#"{"TimeOfFlight": {"TdcReference": ["a", 1]}}"#),
            Err(E::InvalidTdcReference)
        );
        for bad in [
            r#"{"Stem": {"Scan": {"Width": 1.5}}}"#,
            r#"{"Stem": {"Scan": {"Height": 3000000000}}}"#,
            r#"{"Stem": {"Scan": {"DwellTime": "1"}}}"#,
            r#"{"Stem": {"VirtualDetector": {"RadiusInner": true}}}"#,
            r#"{"TimeOfFlight": {"Min": "0"}}"#,
        ] {
            assert_eq!(parse(bad), Err(E::InvalidMetric), "{bad}");
        }

        // Null is absent at every level.
        assert_eq!(
            parse(r#"{"Stem": null, "TimeOfFlight": {"TdcReference": null, "Min": null}}"#),
            Ok(ConfigSnapshot::default())
        );
        assert_eq!(
            parse(
                r#"{"Stem": {"Scan": {"Width": 64, "Height": -2, "DwellTime": 0.25},
                    "VirtualDetector": {"RadiusOuter": 9, "RadiusInner": 3}},
                    "TimeOfFlight": {"TdcReference": ["PN0123", "NP0123"], "Min": 1, "Max": 2.5},
                    "Corrections": {}}"#
            ),
            Ok(ConfigSnapshot {
                stem_scan_width: Some(64),
                stem_scan_height: Some(-2),
                stem_dwell_time: Some(0.25),
                stem_radius_outer: Some(9),
                stem_radius_inner: Some(3),
                tof_tdc_reference: Some("PN0123,NP0123".into()),
                tof_min: Some(1.0),
                tof_max: Some(2.5),
            })
        );
        assert_eq!(
            parse_config_response("[]"),
            Err(ConfigResponseError::InvalidRoot)
        );
    }

    /// C `parseStatusResponse` (serval_measurement.cpp:131-196).
    #[test]
    fn a_measurement_reply_is_validated_as_c_does() {
        use StatusResponseError as E;
        let parse = parse_status_response;
        assert_eq!(parse(""), Err(E::EmptyBody));
        assert_eq!(parse("{"), Err(E::MalformedJson));
        assert_eq!(parse("[]"), Err(E::InvalidRoot));
        assert_eq!(parse(r#"{"Info": 5}"#), Err(E::InvalidInfo));
        assert_eq!(parse(r#"{"Info": {"Status": 3}}"#), Err(E::InvalidStatus));
        assert_eq!(parse(r#"{"Status": null}"#), Err(E::InvalidStatus));
        for bad in [
            r#"{"Info": {"FrameCount": "7"}}"#,
            r#"{"Info": {"FrameCount": 1e10}}"#,
            r#"{"Info": {"DroppedFrames": null}}"#,
            r#"{"Info": {"StartDateTime": 1.5}}"#,
            r#"{"Info": {"StartDateTime": 18446744073709551615}}"#,
            r#"{"Info": {"TdcEventRate": "x"}}"#,
        ] {
            assert_eq!(parse(bad), Err(E::InvalidMetric), "{bad}");
        }

        let s = parse(
            r#"{"Info": {"PixelEventRate": 12.9, "TdcEventRate": 4, "Tdc2EventRate": 5,
                "StartDateTime": 1700000000000, "ElapsedTime": 1.5, "TimeLeft": 0.5,
                "FrameCount": 3, "DroppedFrames": 0, "Status": "DA_RECORDING"}}"#,
        )
        .unwrap();
        assert_eq!(s.pixel_event_rate, Some(12));
        // TdcEventRate stands in for a missing Tdc1EventRate.
        assert_eq!(s.tdc1_event_rate, Some(4));
        assert_eq!(s.tdc2_event_rate, Some(5));
        assert_eq!(s.start_date_time, Some(1_700_000_000_000));
        assert_eq!(s.elapsed_time, Some(1.5));
        assert_eq!(s.frame_count, Some(3));
        assert_eq!(s.status.as_deref(), Some("DA_RECORDING"));
        let s = parse(r#"{"Info": {"Tdc1EventRate": 1, "TdcEventRate": 9}}"#).unwrap();
        assert_eq!(s.tdc1_event_rate, Some(1));

        assert_eq!(
            parse(r#"{"Info": {}, "Status": "DA_STARTING"}"#)
                .unwrap()
                .status
                .as_deref(),
            Some("DA_STARTING")
        );
        assert_eq!(
            parse(r#"{"Info": null}"#).unwrap().status.as_deref(),
            Some("DA_IDLE")
        );
        assert_eq!(parse("{}").unwrap(), StatusSnapshot::default());
    }

    #[test]
    fn orientation_index_maps_the_menu() {
        assert_eq!(orientation_index("UP"), Some(0));
        assert_eq!(orientation_index("LEFT"), Some(3));
        assert_eq!(orientation_index("LEFT_MIRRORED"), Some(7));
        // C's std::map::operator[] would insert this and report UP (0).
        assert_eq!(orientation_index("ul"), None);
    }

    #[test]
    fn layout_rotate_rejects_an_unknown_orientation() {
        assert_eq!(
            layout_rotate(0).unwrap(),
            "/detector/layout/rotate?reset=true"
        );
        assert_eq!(
            layout_rotate(3).unwrap(),
            "/detector/layout/rotate?reset=true&direction=left"
        );
        assert_eq!(layout_rotate(8), None);
        assert_eq!(layout_rotate(-1), None);
    }

    #[test]
    fn json_to_string_does_not_mangle_a_non_string() {
        assert_eq!(json_to_string(&json!("Cu")), "Cu");
        // C's strip_quotes turns `null` into "ul" (serval_http.cpp:91).
        assert_eq!(json_to_string(&Value::Null), "");
        assert_eq!(json_to_string(&json!(["a", "b"])), r#"["a","b"]"#);
        assert_eq!(json_to_string(&json!(42)), "42");
    }

    #[test]
    fn the_endpoint_paths_match_c() {
        assert_eq!(chip_dacs(3), "/detector/chips/3/dacs/");
        assert_eq!(chip_pixel_config(0), "/detector/chips/0/PixelConfig");
        assert_eq!(
            config_load("dacs", "/x/y.dacs"),
            "/config/load?format=dacs&file=/x/y.dacs"
        );
    }
}
