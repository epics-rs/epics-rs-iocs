//! SIMPLON REST API client (port of `restApi.cpp`).
//!
//! The C driver hand-rolls HTTP/1.1 over a pool of raw sockets. This port keeps
//! the same URI layout, the same JSON request/response encoding and the same
//! command set, but drives them through `ureq`, which owns connection pooling,
//! keep-alive and header parsing. The codec (URI construction, PUT body, file
//! name patterns, sequence-id extraction) is factored into pure functions so it
//! can be tested without a detector.

use std::time::Duration;

use serde_json::Value;

/// Default request timeout (C `DEFAULT_TIMEOUT`, restApi.h:8).
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(20);
/// Timeout for `initialize` / `restart` (C `DEFAULT_TIMEOUT_INIT`).
pub const TIMEOUT_INIT: Duration = Duration::from_secs(240);
/// Timeout for `arm` (C `DEFAULT_TIMEOUT_ARM`).
pub const TIMEOUT_ARM: Duration = Duration::from_secs(120);

const DATA_NATIVE: &str = "application/json; charset=utf-8";
const DATA_TIFF: &str = "application/tiff";
const DATA_HDF5: &str = "application/hdf5";

/// The `$id` placeholder in a FileWriter name pattern (C `ID_STR`).
const ID_STR: &str = "$id";

/// SIMPLON API version the detector reports (C `eigerAPIVersion_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApiVersion {
    V1_6_0,
    V1_8_0,
}

impl ApiVersion {
    /// Parse the string returned by `/detector/api/version`. The C constructor
    /// throws for anything else (restApi.cpp:270-275).
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "1.6.0" => Some(Self::V1_6_0),
            "1.8.0" => Some(Self::V1_8_0),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::V1_6_0 => "1.6.0",
            Self::V1_8_0 => "1.8.0",
        }
    }
}

/// REST subsystems (C `sys_t`, restApi.h:19-37).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Sys {
    ApiVersion,
    DetConfig,
    DetStatus,
    FwConfig,
    FwStatus,
    FwCommand,
    Command,
    Data,
    MonConfig,
    MonStatus,
    MonImages,
    StreamConfig,
    StreamStatus,
    SysCommand,
}

impl Sys {
    /// Subsystems whose parameters are commands: write-only, no value to read
    /// (C `EigerParam::EigerParam`, eigerParam.cpp:342).
    pub fn is_command(self) -> bool {
        matches!(self, Sys::Command | Sys::FwCommand | Sys::SysCommand)
    }

    /// Subsystems whose parameters are detector status: read-only
    /// (C `EigerParam::baseFetch`, eigerParam.cpp:464-469).
    pub fn is_status(self) -> bool {
        matches!(
            self,
            Sys::DetStatus | Sys::FwStatus | Sys::MonStatus | Sys::StreamStatus
        )
    }

    /// The two-letter `drvInfo` subsystem code accepted by dynamic parameter
    /// creation (C `mSubSystemMap`, eigerDetector.cpp:217-224).
    pub fn from_drv_info_code(code: &str) -> Option<Self> {
        match code {
            "DS" => Some(Sys::DetStatus),
            "DC" => Some(Sys::DetConfig),
            "FS" => Some(Sys::FwStatus),
            "FC" => Some(Sys::FwConfig),
            "MS" => Some(Sys::MonStatus),
            "MC" => Some(Sys::MonConfig),
            "SS" => Some(Sys::StreamStatus),
            "SC" => Some(Sys::StreamConfig),
            _ => None,
        }
    }
}

/// URI prefix for a subsystem (C `RestAPI::RestAPI`, restApi.cpp:258-289).
pub fn subsystem_path(sys: Sys, api: ApiVersion) -> String {
    let v = api.as_str();
    match sys {
        Sys::ApiVersion => "/detector/api/version".to_string(),
        Sys::DetConfig => format!("/detector/api/{v}/config/"),
        Sys::DetStatus => format!("/detector/api/{v}/status/"),
        Sys::FwConfig => format!("/filewriter/api/{v}/config/"),
        Sys::FwStatus => format!("/filewriter/api/{v}/status/"),
        Sys::FwCommand => format!("/filewriter/api/{v}/command/"),
        Sys::Command => format!("/detector/api/{v}/command/"),
        Sys::Data => "/data/".to_string(),
        Sys::MonConfig => format!("/monitor/api/{v}/config/"),
        Sys::MonStatus => format!("/monitor/api/{v}/status/"),
        Sys::MonImages => format!("/monitor/api/{v}/images/"),
        Sys::StreamConfig => format!("/stream/api/{v}/config/"),
        Sys::StreamStatus => format!("/stream/api/{v}/status/"),
        Sys::SysCommand => format!("/system/api/{v}/command/"),
    }
}

/// Body of a PUT with a value (C `RestAPI::put`, restApi.cpp:699).
///
/// `raw_value` is already JSON-encoded (`"true"`, `"42"`, `"\"enabled\""`).
pub fn put_body(raw_value: &str) -> String {
    format!("{{\"value\": {raw_value}}}")
}

/// FileWriter master file name (C `RestAPI::buildMasterName`, restApi.cpp:203).
pub fn build_master_name(pattern: &str, seq_id: i32) -> String {
    match pattern.find(ID_STR) {
        Some(i) => format!(
            "{}{}{}_master.h5",
            &pattern[..i],
            seq_id,
            &pattern[i + ID_STR.len()..]
        ),
        None => format!("{pattern}_master.h5"),
    }
}

/// FileWriter data file name (C `RestAPI::buildDataName`, restApi.cpp:219).
pub fn build_data_name(n: usize, pattern: &str, seq_id: i32) -> String {
    match pattern.find(ID_STR) {
        Some(i) => format!(
            "{}{}{}_data_{:06}.h5",
            &pattern[..i],
            seq_id,
            &pattern[i + ID_STR.len()..],
            n
        ),
        None => format!("{pattern}_data_{n:06}.h5"),
    }
}

/// Extract the series/sequence id from an `arm` reply
/// (C `parseSequenceId`, restApi.cpp:157).
///
/// API 1.6.0 names the key `sequence id`; 1.8.0 renamed it to `series id`.
pub fn parse_sequence_id(body: &str) -> Result<i32, RestError> {
    let v: Value = serde_json::from_str(body)
        .map_err(|e| RestError::Parse(format!("arm reply is not JSON: {e}")))?;
    let id = v
        .get("sequence id")
        .or_else(|| v.get("series id"))
        .ok_or_else(|| RestError::Parse("no 'sequence id' or 'series id' in arm reply".into()))?;
    id.as_i64()
        .map(|n| n as i32)
        .ok_or_else(|| RestError::Parse(format!("sequence id is not a number: {id}")))
}

/// Extract the API version string from the `/detector/api/version` reply
/// (C `RestAPI::RestAPI`, restApi.cpp:261-269).
pub fn parse_api_version(body: &str) -> Result<ApiVersion, RestError> {
    let v: Value = serde_json::from_str(body)
        .map_err(|e| RestError::Parse(format!("version reply is not JSON: {e}")))?;
    let s = v
        .get("value")
        .and_then(Value::as_str)
        .ok_or_else(|| RestError::Parse("no 'value' in version reply".into()))?;
    ApiVersion::parse(s)
        .ok_or_else(|| RestError::Parse(format!("unknown API '{s}', must be 1.6.0 or 1.8.0")))
}

#[derive(Debug)]
pub enum RestError {
    Transport(String),
    /// The server answered, but not with the status this call requires.
    Status {
        path: String,
        code: u16,
    },
    Parse(String),
}

impl std::fmt::Display for RestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Transport(m) => write!(f, "transport: {m}"),
            Self::Status { path, code } => write!(f, "[{path}] server returned error code {code}"),
            Self::Parse(m) => write!(f, "parse: {m}"),
        }
    }
}

impl std::error::Error for RestError {}

pub type RestResult<T> = Result<T, RestError>;

/// Blocking SIMPLON REST client.
///
/// Cloneable and shared across the driver's tasks; `ureq::Agent` is internally
/// synchronized and pools connections, replacing the C driver's fixed array of
/// five mutex-guarded sockets.
#[derive(Clone)]
pub struct RestApi {
    agent: ureq::Agent,
    origin: String,
    api: ApiVersion,
}

impl RestApi {
    /// Connect and negotiate the API version, as the C constructor does before
    /// any other request (restApi.cpp:256-289).
    pub fn new(hostname: &str, port: u16) -> RestResult<Self> {
        let agent = ureq::Agent::config_builder()
            // We inspect status codes ourselves: `wait_file` treats 404 as
            // "not there yet", not as a transport failure.
            .http_status_as_error(false)
            .timeout_global(Some(DEFAULT_TIMEOUT))
            .build()
            .new_agent();
        let origin = format!("http://{hostname}:{port}");

        // Bootstrap with 1.6.0 paths: only /detector/api/version is version-free.
        let probe = Self {
            agent,
            origin,
            api: ApiVersion::V1_6_0,
        };
        let body = probe.get(Sys::ApiVersion, "", Duration::from_secs(10))?;
        let api = parse_api_version(&body)?;
        Ok(Self { api, ..probe })
    }

    pub fn api_version(&self) -> ApiVersion {
        self.api
    }

    /// GET a parameter and return its `value` field as a string.
    ///
    /// Used for the handful of values the driver needs *before* the parameter
    /// list exists (the model description and the sensor size).
    pub fn get_value(&self, sys: Sys, param: &str) -> RestResult<String> {
        let body = self.get(sys, param, DEFAULT_TIMEOUT)?;
        let json: serde_json::Value = serde_json::from_str(&body)
            .map_err(|e| RestError::Parse(format!("[{param}] unable to parse json: {e}")))?;
        match json.get("value") {
            Some(serde_json::Value::String(s)) => Ok(s.clone()),
            Some(v) => Ok(v.to_string()),
            None => Err(RestError::Parse(format!("[{param}] no value in [{body}]"))),
        }
    }

    fn url(&self, sys: Sys, param: &str) -> String {
        format!("{}{}{}", self.origin, subsystem_path(sys, self.api), param)
    }

    /// GET a parameter, returning the raw JSON body (C `RestAPI::get`).
    pub fn get(&self, sys: Sys, param: &str, timeout: Duration) -> RestResult<String> {
        let url = self.url(sys, param);
        let mut resp = self
            .agent
            .get(&url)
            .header("Accept", DATA_NATIVE)
            .config()
            .timeout_global(Some(timeout))
            .build()
            .call()
            .map_err(|e| RestError::Transport(format!("GET {url}: {e}")))?;
        let code = resp.status().as_u16();
        if code != 200 {
            return Err(RestError::Status { path: url, code });
        }
        resp.body_mut()
            .read_to_string()
            .map_err(|e| RestError::Transport(format!("GET {url}: reading body: {e}")))
    }

    /// PUT a parameter, returning the reply body — the detector lists the other
    /// parameters its write invalidated (C `RestAPI::put`).
    ///
    /// An empty `raw_value` sends a zero-length body, which is how commands are
    /// issued.
    pub fn put(
        &self,
        sys: Sys,
        param: &str,
        raw_value: &str,
        timeout: Duration,
    ) -> RestResult<String> {
        let url = self.url(sys, param);
        let body = if raw_value.is_empty() {
            String::new()
        } else {
            put_body(raw_value)
        };
        let mut resp = self
            .agent
            .put(&url)
            .header("Content-Type", DATA_NATIVE)
            .header("Accept-Encoding", "identity")
            .config()
            .timeout_global(Some(timeout))
            .build()
            .send(&body)
            .map_err(|e| RestError::Transport(format!("PUT {url}: {e}")))?;
        let code = resp.status().as_u16();
        if code != 200 {
            return Err(RestError::Status { path: url, code });
        }
        resp.body_mut()
            .read_to_string()
            .map_err(|e| RestError::Transport(format!("PUT {url}: reading body: {e}")))
    }

    // ---- Commands (C RestAPI::restart .. statusUpdate) ----

    pub fn restart(&self) -> RestResult<()> {
        self.put(Sys::SysCommand, "restart", "", TIMEOUT_INIT)
            .map(drop)
    }

    pub fn initialize(&self) -> RestResult<()> {
        self.put(Sys::Command, "initialize", "", TIMEOUT_INIT)
            .map(drop)
    }

    /// Arm and return the sequence id of the new series.
    pub fn arm(&self) -> RestResult<i32> {
        let reply = self.put(Sys::Command, "arm", "", TIMEOUT_ARM)?;
        parse_sequence_id(&reply)
    }

    /// Issue a trigger. `exposure` is `None` for INTS mode; for INTE mode it is
    /// the exposure to hold the trigger for.
    ///
    /// The detector returns from the INTE trigger PUT before the exposure has
    /// elapsed, so the C driver sleeps out the remainder (restApi.cpp:341-354).
    pub fn trigger(&self, timeout: Duration, exposure: Option<f64>) -> RestResult<()> {
        let Some(exposure) = exposure else {
            return self.put(Sys::Command, "trigger", "", timeout).map(drop);
        };

        let start = std::time::Instant::now();
        self.put(Sys::Command, "trigger", &format!("{exposure:.6}"), timeout)?;
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed < exposure {
            std::thread::sleep(Duration::from_secs_f64(exposure - elapsed));
        }
        Ok(())
    }

    pub fn disarm(&self) -> RestResult<()> {
        self.put(Sys::Command, "disarm", "", DEFAULT_TIMEOUT)
            .map(drop)
    }

    pub fn cancel(&self) -> RestResult<()> {
        self.put(Sys::Command, "cancel", "", DEFAULT_TIMEOUT)
            .map(drop)
    }

    pub fn abort(&self) -> RestResult<()> {
        self.put(Sys::Command, "abort", "", DEFAULT_TIMEOUT)
            .map(drop)
    }

    pub fn hv_reset(&self, reset_time: i32) -> RestResult<()> {
        let timeout = Duration::from_secs(reset_time.max(0) as u64 + 1);
        self.put(Sys::Command, "hv_reset", &reset_time.to_string(), timeout)
            .map(drop)
    }

    pub fn status_update(&self) -> RestResult<()> {
        self.put(Sys::Command, "status_update", "", DEFAULT_TIMEOUT)
            .map(drop)
    }

    // ---- File access (C RestAPI::getFileSize .. deleteFile) ----

    /// Poll for a file until it appears or `timeout` elapses
    /// (C `RestAPI::waitFile`, restApi.cpp:429).
    ///
    /// `Ok(true)` = present, `Ok(false)` = still absent after `timeout`. A
    /// status other than 200/404 is an error, as in C.
    pub fn wait_file(&self, filename: &str, timeout: Duration) -> RestResult<bool> {
        let url = self.url(Sys::Data, filename);
        let start = std::time::Instant::now();
        loop {
            let resp = self
                .agent
                .head(&url)
                .call()
                .map_err(|e| RestError::Transport(format!("HEAD {url}: {e}")))?;
            match resp.status().as_u16() {
                200 => return Ok(true),
                404 => {}
                code => return Err(RestError::Status { path: url, code }),
            }
            if start.elapsed() >= timeout {
                return Ok(false);
            }
        }
    }

    /// Download a data file (C `RestAPI::getFile`).
    pub fn get_file(&self, filename: &str) -> RestResult<Vec<u8>> {
        self.get_blob(Sys::Data, filename, DATA_HDF5)
    }

    /// Delete a data file (C `RestAPI::deleteFile`). The detector answers 204.
    pub fn delete_file(&self, filename: &str) -> RestResult<()> {
        let url = self.url(Sys::Data, filename);
        let resp = self
            .agent
            .delete(&url)
            .call()
            .map_err(|e| RestError::Transport(format!("DELETE {url}: {e}")))?;
        let code = resp.status().as_u16();
        if code != 204 {
            return Err(RestError::Status { path: url, code });
        }
        Ok(())
    }

    /// Fetch the newest monitor image as a TIFF blob
    /// (C `RestAPI::getMonitorImage`, restApi.cpp:510).
    ///
    /// `timeout_ms` is the detector-side long-poll timeout, passed in the query
    /// string; it is not the HTTP timeout.
    pub fn get_monitor_image(&self, timeout_ms: u32) -> RestResult<Vec<u8>> {
        self.get_blob(
            Sys::MonImages,
            &format!("monitor?timeout={timeout_ms}"),
            DATA_TIFF,
        )
    }

    /// GET a binary body (C `RestAPI::getBlob`).
    fn get_blob(&self, sys: Sys, name: &str, accept: &str) -> RestResult<Vec<u8>> {
        let url = self.url(sys, name);
        let mut resp = self
            .agent
            .get(&url)
            .header("Accept", accept)
            .call()
            .map_err(|e| RestError::Transport(format!("GET {url}: {e}")))?;
        let code = resp.status().as_u16();
        if code != 200 {
            return Err(RestError::Status { path: url, code });
        }
        resp.body_mut()
            // The detector sends Content-Length; ureq's default read limit is
            // smaller than a data file, so raise it to the pool's ceiling.
            .with_config()
            .limit(u64::MAX)
            .read_to_vec()
            .map_err(|e| RestError::Transport(format!("GET {url}: reading body: {e}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subsystem_paths_1_6_0() {
        let v = ApiVersion::V1_6_0;
        assert_eq!(subsystem_path(Sys::ApiVersion, v), "/detector/api/version");
        assert_eq!(
            subsystem_path(Sys::DetConfig, v),
            "/detector/api/1.6.0/config/"
        );
        assert_eq!(
            subsystem_path(Sys::DetStatus, v),
            "/detector/api/1.6.0/status/"
        );
        assert_eq!(
            subsystem_path(Sys::FwConfig, v),
            "/filewriter/api/1.6.0/config/"
        );
        assert_eq!(
            subsystem_path(Sys::FwStatus, v),
            "/filewriter/api/1.6.0/status/"
        );
        assert_eq!(
            subsystem_path(Sys::FwCommand, v),
            "/filewriter/api/1.6.0/command/"
        );
        assert_eq!(
            subsystem_path(Sys::Command, v),
            "/detector/api/1.6.0/command/"
        );
        assert_eq!(subsystem_path(Sys::Data, v), "/data/");
        assert_eq!(
            subsystem_path(Sys::MonConfig, v),
            "/monitor/api/1.6.0/config/"
        );
        assert_eq!(
            subsystem_path(Sys::MonStatus, v),
            "/monitor/api/1.6.0/status/"
        );
        assert_eq!(
            subsystem_path(Sys::MonImages, v),
            "/monitor/api/1.6.0/images/"
        );
        assert_eq!(
            subsystem_path(Sys::StreamConfig, v),
            "/stream/api/1.6.0/config/"
        );
        assert_eq!(
            subsystem_path(Sys::StreamStatus, v),
            "/stream/api/1.6.0/status/"
        );
        assert_eq!(
            subsystem_path(Sys::SysCommand, v),
            "/system/api/1.6.0/command/"
        );
    }

    #[test]
    fn subsystem_paths_1_8_0() {
        assert_eq!(
            subsystem_path(Sys::DetConfig, ApiVersion::V1_8_0),
            "/detector/api/1.8.0/config/"
        );
        // /data/ and the version URI are not version-qualified.
        assert_eq!(subsystem_path(Sys::Data, ApiVersion::V1_8_0), "/data/");
        assert_eq!(
            subsystem_path(Sys::ApiVersion, ApiVersion::V1_8_0),
            "/detector/api/version"
        );
    }

    #[test]
    fn put_body_wraps_raw_json_value() {
        assert_eq!(put_body("true"), r#"{"value": true}"#);
        assert_eq!(put_body("42"), r#"{"value": 42}"#);
        assert_eq!(put_body("\"enabled\""), r#"{"value": "enabled"}"#);
        assert_eq!(put_body("0.015"), r#"{"value": 0.015}"#);
    }

    #[test]
    fn master_name_substitutes_id_placeholder() {
        assert_eq!(build_master_name("series_$id", 7), "series_7_master.h5");
        assert_eq!(
            build_master_name("pre_$id_post", 12),
            "pre_12_post_master.h5"
        );
    }

    #[test]
    fn master_name_without_placeholder_appends_suffix() {
        assert_eq!(build_master_name("series", 7), "series_master.h5");
    }

    #[test]
    fn data_name_substitutes_id_and_zero_pads_index() {
        assert_eq!(
            build_data_name(1, "series_$id", 7),
            "series_7_data_000001.h5"
        );
        assert_eq!(
            build_data_name(123456, "pre_$id_post", 12),
            "pre_12_post_data_123456.h5"
        );
    }

    #[test]
    fn data_name_without_placeholder_appends_suffix() {
        assert_eq!(build_data_name(2, "series", 7), "series_data_000002.h5");
    }

    #[test]
    fn sequence_id_from_1_6_0_key() {
        assert_eq!(parse_sequence_id(r#"{"sequence id": 42}"#).unwrap(), 42);
    }

    #[test]
    fn sequence_id_from_1_8_0_key() {
        assert_eq!(parse_sequence_id(r#"{"series id": 43}"#).unwrap(), 43);
    }

    #[test]
    fn sequence_id_missing_is_an_error() {
        assert!(parse_sequence_id(r#"{"other": 1}"#).is_err());
        assert!(parse_sequence_id("not json").is_err());
    }

    #[test]
    fn api_version_parses_supported_versions_only() {
        assert_eq!(
            parse_api_version(r#"{"value": "1.6.0"}"#).unwrap(),
            ApiVersion::V1_6_0
        );
        assert_eq!(
            parse_api_version(r#"{"value": "1.8.0"}"#).unwrap(),
            ApiVersion::V1_8_0
        );
        assert!(parse_api_version(r#"{"value": "1.7.0"}"#).is_err());
        assert!(parse_api_version(r#"{"nope": "1.6.0"}"#).is_err());
    }

    #[test]
    fn drv_info_subsystem_codes() {
        assert_eq!(Sys::from_drv_info_code("DS"), Some(Sys::DetStatus));
        assert_eq!(Sys::from_drv_info_code("DC"), Some(Sys::DetConfig));
        assert_eq!(Sys::from_drv_info_code("FS"), Some(Sys::FwStatus));
        assert_eq!(Sys::from_drv_info_code("FC"), Some(Sys::FwConfig));
        assert_eq!(Sys::from_drv_info_code("MS"), Some(Sys::MonStatus));
        assert_eq!(Sys::from_drv_info_code("MC"), Some(Sys::MonConfig));
        assert_eq!(Sys::from_drv_info_code("SS"), Some(Sys::StreamStatus));
        assert_eq!(Sys::from_drv_info_code("SC"), Some(Sys::StreamConfig));
        assert_eq!(Sys::from_drv_info_code("XX"), None);
    }

    #[test]
    fn command_and_status_subsystem_classification() {
        assert!(Sys::Command.is_command());
        assert!(Sys::FwCommand.is_command());
        assert!(Sys::SysCommand.is_command());
        assert!(!Sys::DetConfig.is_command());

        assert!(Sys::DetStatus.is_status());
        assert!(Sys::FwStatus.is_status());
        assert!(Sys::MonStatus.is_status());
        assert!(Sys::StreamStatus.is_status());
        assert!(!Sys::DetConfig.is_status());
    }
}
