//! Metadata-driven detector parameters (port of `eigerParam.cpp`).
//!
//! Every remote parameter is described by the detector itself: a GET returns
//! `{"value": …, "value_type": …, "access_mode": …, "min": …, "max": …,
//! "allowed_values": […]}`. The C `EigerParam` caches that metadata on first
//! fetch and uses it to encode and clamp every subsequent write.
//!
//! This port keeps the metadata cache but splits the I/O from the param
//! library: `fetch`/`put` return a list of [`ParamUpdate`]s and the caller
//! applies them through whatever sink it owns (the driver writes them straight
//! into `PortDriverBase`, the background tasks push them through a
//! `PortHandle`). One codec, two sinks — no duplicated encode/decode logic.

use std::collections::HashMap;
use std::time::Duration;

use serde_json::Value;

use crate::rest::{DEFAULT_TIMEOUT, RestApi, RestError, RestResult, Sys};

/// Value flavour of a detector parameter (C `eiger_param_type_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Not yet fetched: the detector has not told us its type.
    Uninit,
    Bool,
    Int,
    UInt,
    Double,
    String,
    Enum,
    Command,
}

/// Access mode (C `eiger_access_mode_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    Ro,
    Rw,
    Wo,
}

/// asyn parameter type this parameter is bound to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsynType {
    Int32,
    Float64,
    Octet,
}

/// Cached detector-supplied metadata for one parameter.
#[derive(Debug, Clone)]
pub struct Meta {
    pub kind: Kind,
    pub access: Access,
    /// Clamp limits for `Int`/`UInt`/`Enum` (C `mMin.valInt` / `mMax.valInt`).
    pub min_int: Option<i32>,
    pub max_int: Option<i32>,
    /// Clamp limits for `Double` (C `mMin.valDouble` / `mMax.valDouble`).
    pub min_f64: Option<f64>,
    pub max_f64: Option<f64>,
    pub enum_values: Vec<String>,
    pub critical_values: Vec<String>,
}

impl Default for Meta {
    fn default() -> Self {
        Self {
            kind: Kind::Uninit,
            access: Access::Ro,
            min_int: None,
            max_int: None,
            min_f64: None,
            max_f64: None,
            enum_values: Vec::new(),
            critical_values: Vec::new(),
        }
    }
}

/// A value to write into the asyn parameter library.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamUpdate {
    Int32(usize, i32),
    Float64(usize, f64),
    Octet(usize, String),
}

/// The `value_type` char → [`Kind`] mapping (C `EigerParam::parseType`,
/// eigerParam.cpp:88-102).
///
/// `allowed_values` present promotes any type to `Enum`; a write-only
/// `access_mode` promotes it to `Command`. Both overrides are applied by the
/// caller before this mapping, exactly as in C.
fn kind_from_type_str(type_str: &str) -> Option<Kind> {
    match type_str.chars().next()? {
        's' => Some(Kind::String),
        // "string" became "list" in API 1.8.0 (EIGER2 v2020.1).
        'l' => Some(Kind::String),
        'f' => Some(Kind::Double),
        'b' => Some(Kind::Bool),
        'u' => Some(Kind::UInt),
        'i' => Some(Kind::Int),
        'e' => Some(Kind::Enum),
        'c' => Some(Kind::Command),
        _ => None,
    }
}

/// Read a JSON field as an array of strings, or as a single string
/// (C `EigerParam::parseArray`, eigerParam.cpp:40).
fn parse_string_array(v: &Value) -> Vec<String> {
    match v {
        Value::Array(items) => items
            .iter()
            .map(|i| match i {
                Value::String(s) => s.clone(),
                other => other.to_string(),
            })
            .collect(),
        Value::String(s) => vec![s.clone()],
        _ => Vec::new(),
    }
}

/// Render a JSON value the way the C driver's `frozen` tokens do: the raw text
/// of the token, with string quotes stripped
/// (C `EigerParam::parseValue`, eigerParam.cpp:169).
pub fn raw_value_text(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        Value::Bool(b) => if *b { "true" } else { "false" }.to_string(),
        other => other.to_string(),
    }
}

/// Decode the metadata block of a GET reply
/// (C `EigerParam::baseFetch`, eigerParam.cpp:448-504).
///
/// `sys` decides the access mode for command and status subsystems without
/// consulting the reply. `custom_enum` is set when the driver hard-coded the
/// enum ordering (`setEnumValues`), in which case the detector's
/// `allowed_values` are ignored.
pub fn parse_meta(
    body: &Value,
    sys: Sys,
    custom_enum: bool,
    existing: &Meta,
) -> Result<Meta, String> {
    let type_str = body
        .get("value_type")
        .and_then(Value::as_str)
        .ok_or("unable to find 'value_type' json field")?;

    // allowed_values wins over value_type; a write-only access_mode wins over both.
    let mut kind = if body.get("allowed_values").is_some() {
        Kind::Enum
    } else {
        kind_from_type_str(type_str)
            .ok_or_else(|| format!("unrecognized value type '{type_str}'"))?
    };
    let access_str = body.get("access_mode").and_then(Value::as_str);
    if access_str.is_some_and(|a| a.starts_with('w')) {
        kind = Kind::Command;
    }

    let access = if sys.is_command() {
        Access::Wo
    } else if sys.is_status() {
        Access::Ro
    } else {
        match access_str {
            Some("r") => Access::Ro,
            Some("w") => Access::Wo,
            Some("rw") => Access::Rw,
            // C falls back to read-only when access_mode is absent or
            // unparseable (eigerParam.cpp:471-472).
            _ => Access::Ro,
        }
    };

    let mut meta = Meta {
        kind,
        access,
        ..Meta::default()
    };

    if custom_enum {
        meta.kind = Kind::Enum;
        meta.enum_values = existing.enum_values.clone();
    } else if let Some(av) = body.get("allowed_values") {
        meta.enum_values = parse_string_array(av);
    }
    if let Some(cv) = body.get("critical_values") {
        meta.critical_values = parse_string_array(cv);
    }

    // Limits are typed by value_type, not by the promoted kind: an enum with an
    // integer value_type still reports numeric min/max we must not misread.
    let numeric_char = type_str.chars().next().unwrap_or('\0');
    match meta.kind {
        Kind::Int | Kind::UInt | Kind::Double => {
            let (min, max) = (body.get("min"), body.get("max"));
            if numeric_char == 'i' || numeric_char == 'u' {
                meta.min_int = min.and_then(json_as_i32);
                meta.max_int = max.and_then(json_as_i32);
                if min.is_some() && meta.min_int.is_none() {
                    return Err("failed to parse 'min' as integer".into());
                }
                if max.is_some() && meta.max_int.is_none() {
                    return Err("failed to parse 'max' as integer".into());
                }
            } else if numeric_char == 'f' {
                meta.min_f64 = min.and_then(json_as_f64);
                meta.max_f64 = max.and_then(json_as_f64);
                if min.is_some() && meta.min_f64.is_none() {
                    return Err("failed to parse 'min' as double".into());
                }
                if max.is_some() && meta.max_f64.is_none() {
                    return Err("failed to parse 'max' as double".into());
                }
            }
        }
        Kind::Enum => {
            meta.min_int = Some(0);
            meta.max_int = Some(meta.enum_values.len().saturating_sub(1) as i32);
        }
        _ => {}
    }

    Ok(meta)
}

/// Accept a JSON number, or a JSON string holding a number, as C's `sscanf`
/// against the raw token text does.
fn json_as_i32(v: &Value) -> Option<i32> {
    match v {
        Value::Number(n) => n.as_i64().map(|i| i as i32),
        Value::String(s) => s.trim().parse::<i32>().ok(),
        _ => None,
    }
}

fn json_as_f64(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse::<f64>().ok(),
        _ => None,
    }
}

/// Index of an enum value (C `EigerParam::getEnumIndex`).
pub fn enum_index(enum_values: &[String], value: &str) -> Option<usize> {
    enum_values.iter().position(|v| v == value)
}

// ---- Value encoding (C `EigerParam::toString`) ----

/// Encode a bool for the wire.
///
/// UPSTREAM DEFECT (eigerParam.cpp:226): C indexes the enum with `!value`, so
/// `put(false)` selects `enum_values[1]` ("enabled") and `put(true)` selects
/// `enum_values[0]` ("disabled") — inverted with respect to `fetch`, which maps
/// index 1 to `true` (eigerParam.cpp:548). The only caller that hits it,
/// `mMonitorEnable->put(false)` in `initParams`, therefore *enables* the
/// monitor interface at startup instead of disabling it. Encoded here as the
/// inverse of `fetch`: index == value.
pub fn encode_bool(meta: &Meta, value: bool) -> Result<String, String> {
    if meta.kind == Kind::Enum {
        let idx = usize::from(value);
        let name = meta
            .enum_values
            .get(idx)
            .ok_or_else(|| format!("enum has no value at index {idx}"))?;
        return Ok(encode_string(name));
    }
    Ok(if value { "true" } else { "false" }.to_string())
}

/// Encode an int for the wire; enum-typed parameters send the name at `value`.
pub fn encode_int(meta: &Meta, value: i32) -> Result<String, String> {
    if meta.kind == Kind::Enum {
        let name = usize::try_from(value)
            .ok()
            .and_then(|i| meta.enum_values.get(i))
            .ok_or_else(|| format!("enum has no value at index {value}"))?;
        return Ok(encode_string(name));
    }
    Ok(value.to_string())
}

/// Encode a double for the wire.
///
/// C prints with `setprecision(long double digits10 + 2)`; Rust's default
/// `Display` for f64 is the shortest representation that round-trips exactly,
/// which is the same number with less noise.
pub fn encode_f64(value: f64) -> String {
    if value == value.trunc() && value.is_finite() && value.abs() < 1e15 {
        // Keep integral doubles JSON-numeric ("1" not "1.0" is fine either way,
        // but a trailing .0 keeps them distinguishable from ints on the wire).
        format!("{value:.1}")
    } else {
        format!("{value}")
    }
}

/// Encode a string for the wire: JSON-quoted (C `toString(std::string)`).
pub fn encode_string(value: &str) -> String {
    Value::String(value.to_string()).to_string()
}

/// Clamp an int against the cached limits (C `EigerParam::put(int)`,
/// eigerParam.cpp:819-828).
pub fn clamp_int(meta: &Meta, mut value: i32) -> i32 {
    if let Some(min) = meta.min_int {
        value = value.max(min);
    }
    if let Some(max) = meta.max_int {
        value = value.min(max);
    }
    // Never write a negative value to an unsigned parameter.
    if meta.kind == Kind::UInt && value < 0 {
        value = 0;
    }
    value
}

/// Clamp a double against the cached limits (C `EigerParam::put(double)`).
pub fn clamp_f64(meta: &Meta, mut value: f64) -> f64 {
    if let Some(min) = meta.min_f64 {
        value = value.max(min);
    }
    if let Some(max) = meta.max_f64 {
        value = value.min(max);
    }
    value
}

/// One parameter's static description plus its cached metadata.
#[derive(Debug, Clone)]
pub struct ParamDef {
    pub index: usize,
    pub asyn_name: String,
    pub asyn_type: AsynType,
    pub sys: Sys,
    /// Detector-side name; empty for driver-only parameters (C `mRemote`).
    pub remote_name: String,
    pub meta: Meta,
    /// Minimum change that is worth a write (C `mEpsilon`).
    pub epsilon: f64,
    pub custom_enum: bool,
}

impl ParamDef {
    pub fn is_remote(&self) -> bool {
        !self.remote_name.is_empty()
    }
}

/// All detector parameters, keyed by asyn index.
pub struct ParamRegistry {
    defs: HashMap<usize, ParamDef>,
    /// Detector-config parameters by remote name — the PUT reply names the
    /// parameters a write invalidated, and only DetConfig ones are re-fetched
    /// (C `EigerParamSet::mDetConfigMap`).
    det_config: HashMap<String, usize>,
    /// Creation order, so `fetch_all` walks the parameters in the order the
    /// driver declared them (C iterates its index-ordered map).
    order: Vec<usize>,
}

impl Default for ParamRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamRegistry {
    pub fn new() -> Self {
        Self {
            defs: HashMap::new(),
            det_config: HashMap::new(),
            order: Vec::new(),
        }
    }

    /// Register a parameter (C `EigerParamSet::create`).
    pub fn add(
        &mut self,
        index: usize,
        asyn_name: &str,
        asyn_type: AsynType,
        sys: Sys,
        remote_name: &str,
    ) {
        let mut meta = Meta::default();
        // A command parameter has no value to fetch and is write-only from the
        // moment it is created (C `EigerParam::EigerParam`, eigerParam.cpp:342).
        if !remote_name.is_empty() && sys.is_command() {
            meta.kind = Kind::Command;
            meta.access = Access::Wo;
        } else if remote_name.is_empty() {
            // Driver-only parameter: its type comes from the asyn type.
            meta.kind = match asyn_type {
                AsynType::Int32 => Kind::Int,
                AsynType::Float64 => Kind::Double,
                AsynType::Octet => Kind::String,
            };
            meta.access = Access::Rw;
        }

        let def = ParamDef {
            index,
            asyn_name: asyn_name.to_string(),
            asyn_type,
            sys,
            remote_name: remote_name.to_string(),
            meta,
            epsilon: 0.0,
            custom_enum: false,
        };
        if !remote_name.is_empty() && sys == Sys::DetConfig {
            self.det_config.insert(remote_name.to_string(), index);
        }
        if !self.defs.contains_key(&index) {
            self.order.push(index);
        }
        self.defs.insert(index, def);
    }

    pub fn contains(&self, index: usize) -> bool {
        self.defs.contains_key(&index)
    }

    pub fn get(&self, index: usize) -> Option<&ParamDef> {
        self.defs.get(&index)
    }

    pub fn indices(&self) -> Vec<usize> {
        self.order.clone()
    }

    /// Hard-code the enum ordering, ignoring the detector's `allowed_values`
    /// (C `EigerParam::setEnumValues`).
    pub fn set_enum_values(&mut self, index: usize, values: &[&str]) {
        if let Some(d) = self.defs.get_mut(&index) {
            d.meta.enum_values = values.iter().map(|s| s.to_string()).collect();
            d.meta.kind = Kind::Enum;
            d.custom_enum = true;
        }
    }

    pub fn set_epsilon(&mut self, index: usize, epsilon: f64) {
        if let Some(d) = self.defs.get_mut(&index) {
            d.epsilon = epsilon;
        }
    }

    pub fn index_of_det_config(&self, remote_name: &str) -> Option<usize> {
        self.det_config.get(remote_name).copied()
    }
}

/// The registry plus the REST connection: performs a fetch or a put and hands
/// back the parameter-library writes the caller must apply.
pub struct ParamOps {
    pub rest: RestApi,
    pub reg: parking_lot::Mutex<ParamRegistry>,
}

impl ParamOps {
    pub fn new(rest: RestApi, reg: ParamRegistry) -> Self {
        Self {
            rest,
            reg: parking_lot::Mutex::new(reg),
        }
    }

    fn def(&self, index: usize) -> RestResult<ParamDef> {
        self.reg
            .lock()
            .get(index)
            .cloned()
            .ok_or_else(|| RestError::Parse(format!("no eiger parameter at index {index}")))
    }

    /// GET the parameter, refresh the cached metadata, and decode the value
    /// (C `EigerParam::baseFetch` + the typed `fetch` overloads).
    ///
    /// A write-only parameter has no value: C returns success without touching
    /// the param library, and so does this.
    pub fn fetch(&self, index: usize) -> RestResult<Vec<ParamUpdate>> {
        let def = self.def(index)?;
        if !def.is_remote() || def.meta.kind == Kind::Command || def.meta.access == Access::Wo {
            return Ok(Vec::new());
        }

        let body = self.rest.get(def.sys, &def.remote_name, DEFAULT_TIMEOUT)?;
        let json: Value = serde_json::from_str(&body).map_err(|e| {
            RestError::Parse(format!(
                "[{}] unable to parse json response: {e}",
                def.asyn_name
            ))
        })?;

        // The metadata is re-read on every fetch (as in C, which re-parses
        // min/max each time but keeps the type from the first reply).
        let meta = {
            let mut reg = self.reg.lock();
            let d = reg
                .defs
                .get_mut(&index)
                .ok_or_else(|| RestError::Parse(format!("no eiger parameter at index {index}")))?;
            let fresh = parse_meta(&json, d.sys, d.custom_enum, &d.meta)
                .map_err(|e| RestError::Parse(format!("[{}] {e}\n[{body}]", d.asyn_name)))?;
            // A custom enum keeps its hard-coded ordering forever.
            d.meta = fresh;
            d.meta.clone()
        };

        let raw = json
            .get("value")
            .map(raw_value_text)
            .ok_or_else(|| RestError::Parse(format!("[{}] no 'value' in reply", def.asyn_name)))?;

        self.decode_into_updates(&def, &meta, &raw)
    }

    /// Map a decoded raw value onto the asyn parameter it feeds.
    fn decode_into_updates(
        &self,
        def: &ParamDef,
        meta: &Meta,
        raw: &str,
    ) -> RestResult<Vec<ParamUpdate>> {
        let bad = |what: &str| {
            RestError::Parse(format!(
                "[{}] couldn't parse value '{raw}' as {what}",
                def.asyn_name
            ))
        };

        let update = match def.asyn_type {
            AsynType::Int32 => {
                let v = match meta.kind {
                    Kind::Enum => enum_index(&meta.enum_values, raw).ok_or_else(|| {
                        RestError::Parse(format!(
                            "[{}] '{raw}' is not one of {:?}",
                            def.asyn_name, meta.enum_values
                        ))
                    })? as i32,
                    Kind::Bool => match raw {
                        "true" => 1,
                        "false" => 0,
                        _ => return Err(bad("boolean")),
                    },
                    Kind::Int | Kind::UInt => {
                        raw.trim().parse::<i32>().map_err(|_| bad("integer"))?
                    }
                    other => {
                        return Err(RestError::Parse(format!(
                            "[{}] unexpected type {other:?} for an Int32 parameter",
                            def.asyn_name
                        )));
                    }
                };
                ParamUpdate::Int32(def.index, v)
            }
            AsynType::Float64 => {
                // The detector can return integers larger than 2^31, so int and
                // uint parameters are allowed to feed a Float64 (C fetch<double>,
                // eigerParam.cpp:626-632).
                let mut v: f64 = raw.trim().parse().map_err(|_| bad("double"))?;
                // FW_FREE units depend on the API version: 1.6.0 reports KB,
                // 1.8.0 reports bytes; both are published as GB
                // (C fetch<double>, eigerParam.cpp:645-653).
                if def.asyn_name == "FW_FREE" {
                    v = match self.rest.api_version() {
                        crate::rest::ApiVersion::V1_6_0 => v * 1024.0 / 1e9,
                        crate::rest::ApiVersion::V1_8_0 => v / 1e9,
                    };
                }
                ParamUpdate::Float64(def.index, v)
            }
            AsynType::Octet => ParamUpdate::Octet(def.index, raw.to_string()),
        };
        Ok(vec![update])
    }

    /// PUT a raw (already JSON-encoded) value and re-fetch whatever the
    /// detector says the write invalidated (C `EigerParam::basePut`).
    fn base_put(
        &self,
        def: &ParamDef,
        raw: &str,
        timeout: Duration,
    ) -> RestResult<Vec<ParamUpdate>> {
        if def.meta.access == Access::Ro {
            return Err(RestError::Parse(format!(
                "[{}] can't write to read-only parameter",
                def.asyn_name
            )));
        }

        let reply = self.rest.put(def.sys, &def.remote_name, raw, timeout)?;

        // The reply is a JSON array of the detector-config parameters this write
        // changed. An empty body (or `""`) means nothing else moved.
        let mut updates = Vec::new();
        let trimmed = reply.trim();
        if trimmed.is_empty() || trimmed == "\"\"" {
            return Ok(updates);
        }
        let json: Value = match serde_json::from_str(trimmed) {
            Ok(v) => v,
            Err(e) => {
                return Err(RestError::Parse(format!(
                    "[{}] unable to parse json response: {e}\n[{trimmed}]",
                    def.asyn_name
                )));
            }
        };
        for name in parse_string_array(&json) {
            let Some(idx) = self.reg.lock().index_of_det_config(&name) else {
                continue;
            };
            // A failure to refresh one changed parameter must not abandon the
            // rest, and must not fail the write that already landed.
            match self.fetch(idx) {
                Ok(mut u) => updates.append(&mut u),
                Err(e) => log::warn!("eiger: refetch of '{name}' after write failed: {e}"),
            }
        }
        Ok(updates)
    }

    /// Fetch the metadata if this is the first touch of the parameter
    /// (C: `if(mType == EIGER_P_UNINIT && fetch())`).
    fn ensure_meta(&self, index: usize) -> RestResult<ParamDef> {
        let def = self.def(index)?;
        if def.is_remote() && def.meta.kind == Kind::Uninit {
            self.fetch(index)?;
            return self.def(index);
        }
        Ok(def)
    }

    /// Write an i32 (C `EigerParam::put(int)`).
    pub fn put_int(&self, index: usize, value: i32) -> RestResult<Vec<ParamUpdate>> {
        let def = self.ensure_meta(index)?;
        let mut updates = Vec::new();
        let mut value = value;

        if def.is_remote() {
            if !matches!(
                def.meta.kind,
                Kind::Bool | Kind::Int | Kind::UInt | Kind::Enum | Kind::Command
            ) {
                return Err(RestError::Parse(format!(
                    "[{}] expected bool, int, uint or enum",
                    def.asyn_name
                )));
            }
            value = clamp_int(&def.meta, value);
            let raw = if def.meta.kind == Kind::Bool {
                encode_bool(&def.meta, value != 0)
            } else {
                encode_int(&def.meta, value)
            }
            .map_err(|e| RestError::Parse(format!("[{}] {e}", def.asyn_name)))?;
            updates = self.base_put(&def, &raw, DEFAULT_TIMEOUT)?;
        }

        let local = match def.asyn_type {
            AsynType::Int32 => ParamUpdate::Int32(index, value),
            AsynType::Octet => {
                let name = usize::try_from(value)
                    .ok()
                    .and_then(|i| def.meta.enum_values.get(i))
                    .ok_or_else(|| {
                        RestError::Parse(format!(
                            "[{}] enum has no value at index {value}",
                            def.asyn_name
                        ))
                    })?;
                ParamUpdate::Octet(index, name.clone())
            }
            AsynType::Float64 => ParamUpdate::Float64(index, value as f64),
        };
        updates.insert(0, local);
        Ok(updates)
    }

    /// Write a bool (C `EigerParam::put(bool)`).
    pub fn put_bool(&self, index: usize, value: bool) -> RestResult<Vec<ParamUpdate>> {
        let def = self.ensure_meta(index)?;
        if !def.is_remote() {
            return Ok(vec![ParamUpdate::Int32(index, i32::from(value))]);
        }
        if !matches!(def.meta.kind, Kind::Bool | Kind::Enum) {
            return Err(RestError::Parse(format!(
                "[{}] expected bool or enum",
                def.asyn_name
            )));
        }
        let raw = encode_bool(&def.meta, value)
            .map_err(|e| RestError::Parse(format!("[{}] {e}", def.asyn_name)))?;
        let mut updates = self.base_put(&def, &raw, DEFAULT_TIMEOUT)?;
        updates.insert(0, ParamUpdate::Int32(index, i32::from(value)));
        Ok(updates)
    }

    /// Write a double (C `EigerParam::put(double)`).
    ///
    /// `current` is the parameter's present value, used for the epsilon check
    /// that suppresses no-op writes to the detector.
    pub fn put_f64(&self, index: usize, value: f64, current: f64) -> RestResult<Vec<ParamUpdate>> {
        let def = self.ensure_meta(index)?;
        if def.epsilon != 0.0 && (current - value).abs() < def.epsilon {
            return Ok(Vec::new());
        }

        let mut value = value;
        let mut updates = Vec::new();
        if def.is_remote() {
            if def.meta.kind != Kind::Double {
                return Err(RestError::Parse(format!(
                    "[{}] expected double",
                    def.asyn_name
                )));
            }
            value = clamp_f64(&def.meta, value);
            updates = self.base_put(&def, &encode_f64(value), DEFAULT_TIMEOUT)?;
        }
        updates.insert(0, ParamUpdate::Float64(index, value));
        Ok(updates)
    }

    /// Write a string (C `EigerParam::put(std::string)`).
    pub fn put_str(&self, index: usize, value: &str) -> RestResult<Vec<ParamUpdate>> {
        let def = self.ensure_meta(index)?;
        if !def.is_remote() {
            return Ok(vec![ParamUpdate::Octet(index, value.to_string())]);
        }
        if !matches!(def.meta.kind, Kind::String | Kind::Enum) {
            return Err(RestError::Parse(format!(
                "[{}] expected string or enum",
                def.asyn_name
            )));
        }
        let idx = if def.meta.kind == Kind::Enum {
            Some(enum_index(&def.meta.enum_values, value).ok_or_else(|| {
                RestError::Parse(format!(
                    "[{}] '{value}' is not one of {:?}",
                    def.asyn_name, def.meta.enum_values
                ))
            })?)
        } else {
            None
        };

        let mut updates = self.base_put(&def, &encode_string(value), DEFAULT_TIMEOUT)?;
        let local = match (def.asyn_type, idx) {
            (AsynType::Int32, Some(i)) => ParamUpdate::Int32(index, i as i32),
            (AsynType::Int32, None) => {
                return Err(RestError::Parse(format!(
                    "[{}] can't write a non-enum string to an Int32 parameter",
                    def.asyn_name
                )));
            }
            _ => ParamUpdate::Octet(index, value.to_string()),
        };
        updates.insert(0, local);
        Ok(updates)
    }

    /// Issue a command parameter (write-only, no value).
    pub fn put_command(&self, index: usize) -> RestResult<Vec<ParamUpdate>> {
        let def = self.def(index)?;
        self.base_put(&def, "", DEFAULT_TIMEOUT)
    }

    /// Fetch every registered parameter (C `EigerParamSet::fetchAll`).
    ///
    /// A single failing parameter does not abandon the rest, matching C's
    /// `status |=` accumulation.
    pub fn fetch_all(&self) -> (Vec<ParamUpdate>, usize) {
        let mut updates = Vec::new();
        let mut failures = 0;
        // The guard has to be dropped before the loop body runs: a `for` holds
        // the temporaries of its iterator expression for the whole loop, and
        // `fetch` locks the same (non-reentrant) mutex on its first call.
        let indices = self.reg.lock().indices();
        for idx in indices {
            match self.fetch(idx) {
                Ok(mut u) => updates.append(&mut u),
                Err(e) => {
                    failures += 1;
                    log::warn!("eiger: initial fetch failed: {e}");
                }
            }
        }
        (updates, failures)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meta_of(body: &str, sys: Sys) -> Meta {
        let v: Value = serde_json::from_str(body).unwrap();
        parse_meta(&v, sys, false, &Meta::default()).unwrap()
    }

    /// `fetch_all` used to hold the registry lock across the loop body, and
    /// `fetch` locks the same non-reentrant mutex — the first iteration hung.
    /// The port it belongs to never came up, whatever the detector answered.
    #[test]
    fn fetch_all_does_not_deadlock_on_the_registry() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("listener");
        let port = listener.local_addr().unwrap().port();
        drop(listener);

        let mut reg = ParamRegistry::new();
        reg.add(0, "STATE", AsynType::Octet, Sys::DetStatus, "state");
        let ops = ParamOps::new(RestApi::new("127.0.0.1", port), reg);

        // Every request fails against a closed port, which is the point: the
        // call has to *return*.
        let (updates, failures) = ops.fetch_all();
        assert!(updates.is_empty());
        assert_eq!(failures, 1);
    }

    #[test]
    fn meta_float_with_limits() {
        let m = meta_of(
            r#"{"value": 0.5, "value_type": "float", "access_mode": "rw",
                "min": 0.0001, "max": 3600.0}"#,
            Sys::DetConfig,
        );
        assert_eq!(m.kind, Kind::Double);
        assert_eq!(m.access, Access::Rw);
        assert_eq!(m.min_f64, Some(0.0001));
        assert_eq!(m.max_f64, Some(3600.0));
        assert_eq!(m.min_int, None);
    }

    #[test]
    fn meta_uint_with_limits() {
        let m = meta_of(
            r#"{"value": 1, "value_type": "uint", "access_mode": "rw", "min": 1, "max": 1000000}"#,
            Sys::DetConfig,
        );
        assert_eq!(m.kind, Kind::UInt);
        assert_eq!(m.min_int, Some(1));
        assert_eq!(m.max_int, Some(1_000_000));
        assert_eq!(m.min_f64, None);
    }

    #[test]
    fn meta_allowed_values_promote_to_enum() {
        let m = meta_of(
            r#"{"value": "exts", "value_type": "string", "access_mode": "rw",
                "allowed_values": ["ints","inte","exts","exte"]}"#,
            Sys::DetConfig,
        );
        assert_eq!(m.kind, Kind::Enum);
        assert_eq!(m.enum_values, ["ints", "inte", "exts", "exte"]);
        // C sets the enum's clamp limits from the value count.
        assert_eq!(m.min_int, Some(0));
        assert_eq!(m.max_int, Some(3));
    }

    #[test]
    fn meta_list_type_is_a_string() {
        // API 1.8.0 renamed value_type "string" to "list".
        let m = meta_of(
            r#"{"value": "idle", "value_type": "list", "access_mode": "r"}"#,
            Sys::DetStatus,
        );
        assert_eq!(m.kind, Kind::String);
    }

    #[test]
    fn meta_write_only_access_makes_a_command() {
        let m = meta_of(
            r#"{"value_type": "string", "access_mode": "w"}"#,
            Sys::Command,
        );
        assert_eq!(m.kind, Kind::Command);
        assert_eq!(m.access, Access::Wo);
    }

    #[test]
    fn meta_status_subsystem_forces_read_only() {
        // The reply claims rw; the subsystem overrides it.
        let m = meta_of(
            r#"{"value": "idle", "value_type": "string", "access_mode": "rw"}"#,
            Sys::DetStatus,
        );
        assert_eq!(m.access, Access::Ro);
    }

    #[test]
    fn meta_missing_access_mode_defaults_to_read_only() {
        let m = meta_of(r#"{"value": 1, "value_type": "int"}"#, Sys::DetConfig);
        assert_eq!(m.access, Access::Ro);
    }

    #[test]
    fn meta_custom_enum_ignores_detector_allowed_values() {
        let v: Value = serde_json::from_str(
            r#"{"value": "enabled", "value_type": "string", "access_mode": "rw",
                "allowed_values": ["enabled","disabled"]}"#,
        )
        .unwrap();
        let existing = Meta {
            enum_values: vec!["disabled".into(), "enabled".into()],
            ..Meta::default()
        };
        let m = parse_meta(&v, Sys::MonConfig, true, &existing).unwrap();
        assert_eq!(m.kind, Kind::Enum);
        // The driver's hard-coded ordering survives, not the detector's.
        assert_eq!(m.enum_values, ["disabled", "enabled"]);
    }

    #[test]
    fn meta_unknown_value_type_is_an_error() {
        let v: Value = serde_json::from_str(r#"{"value_type": "quaternion"}"#).unwrap();
        assert!(parse_meta(&v, Sys::DetConfig, false, &Meta::default()).is_err());
    }

    #[test]
    fn raw_value_text_strips_string_quotes() {
        assert_eq!(raw_value_text(&Value::String("idle".into())), "idle");
        assert_eq!(raw_value_text(&serde_json::json!(true)), "true");
        assert_eq!(raw_value_text(&serde_json::json!(false)), "false");
        assert_eq!(raw_value_text(&serde_json::json!(42)), "42");
        assert_eq!(raw_value_text(&serde_json::json!(0.5)), "0.5");
    }

    #[test]
    fn encode_enum_bool_matches_fetch_direction() {
        // The upstream defect: C would encode `false` as enum_values[1].
        let m = Meta {
            kind: Kind::Enum,
            enum_values: vec!["disabled".into(), "enabled".into()],
            ..Meta::default()
        };
        assert_eq!(encode_bool(&m, false).unwrap(), "\"disabled\"");
        assert_eq!(encode_bool(&m, true).unwrap(), "\"enabled\"");
        // ...and fetch maps index 1 back to true, so the pair round-trips.
        assert_eq!(enum_index(&m.enum_values, "enabled"), Some(1));
        assert_eq!(enum_index(&m.enum_values, "disabled"), Some(0));
    }

    #[test]
    fn encode_plain_bool() {
        let m = Meta {
            kind: Kind::Bool,
            ..Meta::default()
        };
        assert_eq!(encode_bool(&m, true).unwrap(), "true");
        assert_eq!(encode_bool(&m, false).unwrap(), "false");
    }

    #[test]
    fn encode_enum_int_sends_the_name() {
        let m = Meta {
            kind: Kind::Enum,
            enum_values: vec!["ints".into(), "inte".into(), "exts".into()],
            ..Meta::default()
        };
        assert_eq!(encode_int(&m, 2).unwrap(), "\"exts\"");
        assert!(encode_int(&m, 5).is_err());
        assert!(encode_int(&m, -1).is_err());
    }

    #[test]
    fn encode_plain_int() {
        let m = Meta {
            kind: Kind::Int,
            ..Meta::default()
        };
        assert_eq!(encode_int(&m, -3).unwrap(), "-3");
    }

    #[test]
    fn encode_doubles_stay_json_numbers() {
        assert_eq!(encode_f64(0.015), "0.015");
        assert_eq!(encode_f64(1.0), "1.0");
        assert_eq!(encode_f64(-2.5e-7), "-0.00000025");
    }

    #[test]
    fn encode_strings_are_json_quoted() {
        assert_eq!(encode_string("series_$id"), "\"series_$id\"");
        // Embedded quotes must not break the body.
        assert_eq!(encode_string("a\"b"), "\"a\\\"b\"");
    }

    #[test]
    fn clamping_respects_typed_limits() {
        let m = Meta {
            kind: Kind::Int,
            min_int: Some(1),
            max_int: Some(10),
            ..Meta::default()
        };
        assert_eq!(clamp_int(&m, 0), 1);
        assert_eq!(clamp_int(&m, 11), 10);
        assert_eq!(clamp_int(&m, 5), 5);

        let u = Meta {
            kind: Kind::UInt,
            ..Meta::default()
        };
        assert_eq!(clamp_int(&u, -7), 0);

        let f = Meta {
            kind: Kind::Double,
            min_f64: Some(0.5),
            max_f64: Some(2.0),
            ..Meta::default()
        };
        assert_eq!(clamp_f64(&f, 0.1), 0.5);
        assert_eq!(clamp_f64(&f, 9.0), 2.0);
        assert_eq!(clamp_f64(&f, 1.0), 1.0);
    }

    #[test]
    fn registry_classifies_command_and_local_params() {
        let mut reg = ParamRegistry::new();
        reg.add(1, "INITIALIZE", AsynType::Int32, Sys::Command, "initialize");
        reg.add(2, "DATA_SOURCE", AsynType::Int32, Sys::DetConfig, "");
        reg.add(3, "NIMAGES", AsynType::Int32, Sys::DetConfig, "nimages");

        let cmd = reg.get(1).unwrap();
        assert_eq!(cmd.meta.kind, Kind::Command);
        assert_eq!(cmd.meta.access, Access::Wo);
        assert!(cmd.is_remote());

        let local = reg.get(2).unwrap();
        assert!(!local.is_remote());
        assert_eq!(local.meta.kind, Kind::Int);

        // Only DetConfig parameters are re-fetchable by remote name.
        assert_eq!(reg.index_of_det_config("nimages"), Some(3));
        assert_eq!(reg.index_of_det_config("initialize"), None);
    }
}
