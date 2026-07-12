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
/// `ADTriggerMode` (TimePix3Base.template).
pub const TRIGGER_MODES: [&str; 8] = [
    "PEXSTART_NEXSTOP",
    "NEXSTART_PEXSTOP",
    "PEXSTART_TIMERSTOP",
    "NEXSTART_TIMERSTOP",
    "AUTOTRIGSTART_TIMERSTOP",
    "CONTINUOUS",
    "SOFTWARESTART_TIMERSTOP",
    "SOFTWARESTART_SOFTWARESTOP",
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

/// `Info.Status` of `GET /measurement` — a measurement that is neither idle nor
/// stopped has to be stopped before a new one starts (C `acquireStart`,
/// acquire.cpp:113).
pub fn measurement_is_running(measurement: &Value) -> bool {
    match measurement.pointer("/Info/Status").and_then(Value::as_str) {
        Some(status) => status != "DA_IDLE" && status != "DA_STOPPED",
        // A missing or non-string status means "not running" in C.
        None => false,
    }
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
        c.trigger_mode = 8;
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
    fn a_running_measurement_is_anything_but_idle_or_stopped() {
        assert!(!measurement_is_running(
            &json!({"Info": {"Status": "DA_IDLE"}})
        ));
        assert!(!measurement_is_running(
            &json!({"Info": {"Status": "DA_STOPPED"}})
        ));
        assert!(measurement_is_running(
            &json!({"Info": {"Status": "DA_RECORDING"}})
        ));
        // A missing status is "not running" (C, acquire.cpp:126).
        assert!(!measurement_is_running(&json!({})));
        assert!(!measurement_is_running(&json!({"Info": {}})));
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
