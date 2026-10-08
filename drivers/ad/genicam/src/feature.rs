//! Port of ADGenICam `GenICamFeature.cpp`: one GenICam feature bound to one
//! asyn parameter, and the set of them a driver owns.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use asyn_rs::param::{EnumEntry, ParamType};
use asyn_rs::port::PortDriverBase;
use asyn_rs::trace::TraceMask;
use parking_lot::RwLock;

use ad_core_rs::driver::ImageMode;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GCFeatureType {
    Integer,
    Boolean,
    Enum,
    Double,
    DoubleMin,
    DoubleMax,
    String,
    Cmd,
    Unknown,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Convert {
    ToEpics,
    FromEpics,
}

pub type NodeResult<T> = Result<T, String>;

/// SDK access to one GenICam node: the pure virtuals of C `GenICamFeature`.
pub trait GenICamNode: Send + Sync {
    /// The feature type the node resolved to; the declared one unless that was
    /// `Unknown`.
    fn feature_type(&self) -> GCFeatureType;
    fn is_implemented(&self) -> bool;
    fn is_available(&self) -> bool;
    fn is_readable(&self) -> bool;
    fn is_writable(&self) -> bool;
    fn read_integer(&self) -> NodeResult<i64>;
    fn read_integer_min(&self) -> NodeResult<i64>;
    fn read_integer_max(&self) -> NodeResult<i64>;
    fn read_increment(&self) -> NodeResult<i64>;
    fn write_integer(&self, value: i64) -> NodeResult<()>;
    fn read_boolean(&self) -> NodeResult<bool>;
    fn write_boolean(&self, value: bool) -> NodeResult<()>;
    fn read_double(&self) -> NodeResult<f64>;
    fn read_double_min(&self) -> NodeResult<f64>;
    fn read_double_max(&self) -> NodeResult<f64>;
    fn write_double(&self, value: f64) -> NodeResult<()>;
    fn read_enum_index(&self) -> NodeResult<i32>;
    fn write_enum_index(&self, value: i32) -> NodeResult<()>;
    fn read_enum_string(&self) -> NodeResult<String>;
    fn read_enum_choices(&self) -> NodeResult<Vec<(String, i32)>>;
    fn read_string(&self) -> NodeResult<String>;
    fn write_string(&self, value: &str) -> NodeResult<()>;
    fn write_command(&self) -> NodeResult<()>;
}

/// A value read from a node, before it is fitted to an asyn parameter type.
#[derive(Clone, Debug, PartialEq)]
pub enum FeatureValue {
    Integer(i64),
    Double(f64),
    String(String),
}

impl FeatureValue {
    /// C `GenICamFeature::setParam`: the value cast to the parameter's type.
    pub fn store(&self, base: &mut PortDriverBase, index: usize, asyn_type: ParamType) {
        let _ = match (self, asyn_type) {
            (Self::Integer(v), ParamType::Int32) => base.set_int32_param(index, 0, *v as i32),
            (Self::Integer(v), ParamType::Int64) => base.set_int64_param(index, 0, *v),
            (Self::Integer(v), ParamType::Float64) => base.set_float64_param(index, 0, *v as f64),
            (Self::Double(v), ParamType::Int32) => base.set_int32_param(index, 0, *v as i32),
            (Self::Double(v), ParamType::Int64) => base.set_int64_param(index, 0, *v as i64),
            (Self::Double(v), ParamType::Float64) => base.set_float64_param(index, 0, *v),
            (Self::String(v), ParamType::Octet) => base.set_string_param(index, 0, v.clone()),
            _ => Ok(()),
        };
    }

    /// The same cast, for a value that reaches the parameter library through a
    /// `PortHandle` instead of the driver.
    pub fn to_param_value(&self, asyn_type: ParamType) -> Option<asyn_rs::param::ParamValue> {
        use asyn_rs::param::ParamValue;
        match (self, asyn_type) {
            (Self::Integer(v), ParamType::Int32) => Some(ParamValue::Int32(*v as i32)),
            (Self::Integer(v), ParamType::Int64) => Some(ParamValue::Int64(*v)),
            (Self::Integer(v), ParamType::Float64) => Some(ParamValue::Float64(*v as f64)),
            (Self::Double(v), ParamType::Int32) => Some(ParamValue::Int32(*v as i32)),
            (Self::Double(v), ParamType::Int64) => Some(ParamValue::Int64(*v as i64)),
            (Self::Double(v), ParamType::Float64) => Some(ParamValue::Float64(*v)),
            (Self::String(v), ParamType::Octet) => Some(ParamValue::Octet(v.clone().into_bytes())),
            _ => None,
        }
    }
}

/// GenICam `AcquisitionMode` enum values, which differ per camera; `None`
/// where the camera has no such mode.
#[derive(Clone, Copy, Debug, Default)]
pub struct AcquisitionModes {
    pub single_frame: Option<i32>,
    pub multi_frame: Option<i32>,
    pub continuous: Option<i32>,
}

pub struct GenICamFeature {
    asyn_name: String,
    asyn_type: ParamType,
    asyn_index: usize,
    feature_name: String,
    enum_choices: Vec<(String, i32)>,
    node: Box<dyn GenICamNode>,
}

impl GenICamFeature {
    pub fn new(
        asyn_name: &str,
        asyn_type: ParamType,
        asyn_index: usize,
        feature_name: &str,
        node: Box<dyn GenICamNode>,
    ) -> Self {
        Self {
            asyn_name: asyn_name.to_string(),
            asyn_type,
            asyn_index,
            feature_name: feature_name.to_string(),
            enum_choices: Vec::new(),
            node,
        }
    }

    pub fn asyn_index(&self) -> usize {
        self.asyn_index
    }

    pub fn asyn_name(&self) -> &str {
        &self.asyn_name
    }

    pub fn asyn_type(&self) -> ParamType {
        self.asyn_type
    }

    pub fn feature_name(&self) -> &str {
        &self.feature_name
    }

    pub fn feature_type(&self) -> GCFeatureType {
        self.node.feature_type()
    }

    pub fn node(&self) -> &dyn GenICamNode {
        self.node.as_ref()
    }

    fn param_as_i64(&self, base: &PortDriverBase) -> i64 {
        match self.asyn_type {
            ParamType::Int32 => base.get_int32_param(self.asyn_index, 0).unwrap_or(0) as i64,
            ParamType::Int64 => base.get_int64_param(self.asyn_index, 0).unwrap_or(0),
            ParamType::Float64 => base.get_float64_param(self.asyn_index, 0).unwrap_or(0.0) as i64,
            _ => 0,
        }
    }

    fn param_as_f64(&self, base: &PortDriverBase) -> f64 {
        match self.asyn_type {
            ParamType::Int32 => base.get_int32_param(self.asyn_index, 0).unwrap_or(0) as f64,
            ParamType::Int64 => base.get_int64_param(self.asyn_index, 0).unwrap_or(0) as f64,
            ParamType::Float64 => base.get_float64_param(self.asyn_index, 0).unwrap_or(0.0),
            _ => 0.0,
        }
    }

    fn warn(&self, base: &PortDriverBase, function: &str, msg: &str) {
        base.trace_print(
            TraceMask::WARNING,
            &format!("Param[{}] {function}: {msg}\n", self.asyn_name),
        );
    }

    fn err(&self, base: &PortDriverBase, function: &str, msg: &str) {
        base.trace_print(
            TraceMask::ERROR,
            &format!("Param[{}] {function}: {msg}\n", self.asyn_name),
        );
    }

    /// Write to the camera and read back. `value` is the value in EPICS units,
    /// or `None` to take it from the asyn parameter. Returns false when the
    /// feature could not be written.
    pub fn write(
        &mut self,
        base: &mut PortDriverBase,
        modes: &AcquisitionModes,
        value: Option<f64>,
        set_param: bool,
    ) -> bool {
        const F: &str = "GenICamFeature::write";
        let name = self.feature_name.clone();
        if !self.node.is_implemented() {
            self.warn(base, F, &format!("node {name} is not implemented"));
            return false;
        }
        if !self.node.is_available() {
            self.warn(base, F, &format!("node {name} is not available"));
            return false;
        }
        if !self.node.is_writable() {
            self.err(base, F, &format!("node {name} is not writable"));
            return false;
        }
        match self.write_node(base, modes, value, set_param) {
            Ok(()) => true,
            Err(e) => {
                self.err(base, F, &format!("feature {name} exception {e}"));
                false
            }
        }
    }

    fn write_node(
        &mut self,
        base: &mut PortDriverBase,
        modes: &AcquisitionModes,
        value: Option<f64>,
        set_param: bool,
    ) -> NodeResult<()> {
        const F: &str = "GenICamFeature::write";
        let name = self.feature_name.clone();
        let readback = match self.node.feature_type() {
            GCFeatureType::Integer => {
                let mut v = match value {
                    Some(v) => v as i64,
                    None => self.param_as_i64(base),
                };
                let max = self.node.read_integer_max()?;
                let min = self.node.read_integer_min()?;
                let inc = self.node.read_increment()?;
                if inc != 1 && inc != 0 {
                    v = (v / inc) * inc;
                }
                if v < min {
                    self.warn(
                        base,
                        F,
                        &format!(
                            "node {name} value {v} is less than minimum {min}, setting to minimum"
                        ),
                    );
                    v = min;
                }
                if v > max {
                    self.warn(
                        base,
                        F,
                        &format!(
                            "node {name} value {v} is greater than maximum {max}, setting to maximum"
                        ),
                    );
                    v = max;
                }
                self.node.write_integer(v)?;
                if !self.node.is_readable() {
                    return Ok(());
                }
                FeatureValue::Integer(self.node.read_integer()?)
            }
            GCFeatureType::Boolean => {
                let v = match value {
                    Some(v) => v != 0.0,
                    None => self.param_as_i64(base) != 0,
                };
                self.node.write_boolean(v)?;
                if !self.node.is_readable() {
                    return Ok(());
                }
                FeatureValue::Integer(self.node.read_boolean()? as i64)
            }
            GCFeatureType::Double => {
                let v = value.unwrap_or_else(|| self.param_as_f64(base));
                let mut v = self.convert_double_units(v, Convert::FromEpics);
                let max = self.node.read_double_max()?;
                let min = self.node.read_double_min()?;
                if v < min {
                    self.warn(
                        base,
                        F,
                        &format!(
                            "node {name} value {v} is less than minimum {min}, setting to minimum"
                        ),
                    );
                    v = min;
                }
                if v > max {
                    self.warn(
                        base,
                        F,
                        &format!(
                            "node {name} value {v} is greater than maximum {max}, setting to maximum"
                        ),
                    );
                    v = max;
                }
                self.node.write_double(v)?;
                if !self.node.is_readable() {
                    return Ok(());
                }
                let rb = self.node.read_double()?;
                FeatureValue::Double(self.convert_double_units(rb, Convert::ToEpics))
            }
            GCFeatureType::Enum => {
                let v = match value {
                    Some(v) => v as i32,
                    None => self.param_as_i64(base) as i32,
                };
                let v = self.convert_enum(base, modes, v, Convert::FromEpics);
                self.node.write_enum_index(v)?;
                if !self.node.is_readable() {
                    return Ok(());
                }
                let rb = self.node.read_enum_index()?;
                FeatureValue::Integer(self.convert_enum(base, modes, rb, Convert::ToEpics) as i64)
            }
            GCFeatureType::String => {
                let v = base
                    .get_string_param(self.asyn_index, 0)
                    .map(|b| String::from_utf8_lossy(b).into_owned())
                    .unwrap_or_default();
                self.node.write_string(&v)?;
                if !self.node.is_readable() {
                    return Ok(());
                }
                FeatureValue::String(self.node.read_string()?)
            }
            GCFeatureType::Cmd => {
                self.node.write_command()?;
                return Ok(());
            }
            _ => return Ok(()),
        };
        if set_param {
            readback.store(base, self.asyn_index, self.asyn_type);
        }
        Ok(())
    }

    /// C `doCallbacksEnum`, fired only when the table differs from the last
    /// one published.
    fn publish_enum_choices(&mut self, base: &mut PortDriverBase, choices: Vec<(String, i32)>) {
        if self.enum_choices == choices {
            return;
        }
        let entries: Vec<EnumEntry> = choices
            .iter()
            .map(|(string, value)| EnumEntry {
                string: string.clone(),
                value: *value,
                severity: 0,
            })
            .collect();
        self.enum_choices = choices;
        if let Err(e) = base.do_callbacks_enum(self.asyn_index, 0, entries.into()) {
            self.err(
                base,
                "GenICamFeature::read",
                &format!("doCallbacksEnum {e}"),
            );
        }
    }

    pub fn read(
        &mut self,
        base: &mut PortDriverBase,
        modes: &AcquisitionModes,
        set_param: bool,
    ) -> bool {
        const F: &str = "GenICamFeature::read";
        if !self.node.is_implemented() {
            return false;
        }
        let name = self.feature_name.clone();
        if self.node.feature_type() == GCFeatureType::Enum
            && self.asyn_name != "IMAGE_MODE"
            && (!self.node.is_available() || !self.node.is_readable())
        {
            self.publish_enum_choices(base, vec![("N.A.".to_string(), 0)]);
            return true;
        }
        if !self.node.is_available() {
            self.warn(base, F, &format!("node {name} is not available"));
            return false;
        }
        if !self.node.is_readable() {
            self.warn(base, F, &format!("node {name} is not readable"));
            return false;
        }
        match self.read_node(base, modes, set_param) {
            Ok(()) => true,
            Err(e) => {
                self.err(base, F, &format!("feature {name} exception {e}"));
                false
            }
        }
    }

    fn read_node(
        &mut self,
        base: &mut PortDriverBase,
        modes: &AcquisitionModes,
        set_param: bool,
    ) -> NodeResult<()> {
        let (value, always) = match self.node.feature_type() {
            GCFeatureType::Integer => (FeatureValue::Integer(self.node.read_integer()?), false),
            GCFeatureType::Boolean => (
                FeatureValue::Integer(self.node.read_boolean()? as i64),
                false,
            ),
            GCFeatureType::Double => {
                let v = self.node.read_double()?;
                (
                    FeatureValue::Double(self.convert_double_units(v, Convert::ToEpics)),
                    false,
                )
            }
            GCFeatureType::DoubleMin => {
                let v = self.node.read_double_min()?;
                (
                    FeatureValue::Double(self.convert_double_units(v, Convert::ToEpics)),
                    true,
                )
            }
            GCFeatureType::DoubleMax => {
                let v = self.node.read_double_max()?;
                (
                    FeatureValue::Double(self.convert_double_units(v, Convert::ToEpics)),
                    true,
                )
            }
            GCFeatureType::Enum => {
                let v = self.node.read_enum_index()?;
                let v = self.convert_enum(base, modes, v, Convert::ToEpics);
                if self.asyn_name != "IMAGE_MODE" {
                    let choices = self.node.read_enum_choices()?;
                    self.publish_enum_choices(base, choices);
                }
                (FeatureValue::Integer(v as i64), false)
            }
            GCFeatureType::String => (FeatureValue::String(self.node.read_string()?), false),
            _ => return Ok(()),
        };
        if set_param || always {
            value.store(base, self.asyn_index, self.asyn_type);
        }
        Ok(())
    }

    pub fn value_as_string(&self) -> String {
        if !(self.node.is_implemented() && self.node.is_readable()) {
            return "Not available".to_string();
        }
        let v = match self.node.feature_type() {
            GCFeatureType::String => self.node.read_string(),
            GCFeatureType::Integer => self.node.read_integer().map(|v| v.to_string()),
            GCFeatureType::Double => self.node.read_double().map(|v| format!("{v:.6}")),
            GCFeatureType::Boolean => self.node.read_boolean().map(|v| v.to_string()),
            GCFeatureType::Cmd => Ok(String::new()),
            GCFeatureType::Enum => self.node.read_enum_string(),
            _ => Err(String::new()),
        };
        v.unwrap_or_else(|_| "Not available".to_string())
    }

    fn convert_double_units(&self, input: f64, direction: Convert) -> f64 {
        if matches!(
            self.feature_name.as_str(),
            "ExposureTime" | "ExposureTimeAbs" | "TriggerDelay"
        ) {
            // EPICS uses seconds, GenICam uses microseconds
            match direction {
                Convert::ToEpics => input / 1.0e6,
                Convert::FromEpics => input * 1.0e6,
            }
        } else if self.asyn_name == "ACQ_PERIOD" {
            // EPICS uses period in seconds, GenICam uses rate in Hz
            1.0 / input
        } else {
            input
        }
    }

    fn convert_enum(
        &self,
        base: &PortDriverBase,
        modes: &AcquisitionModes,
        input: i32,
        direction: Convert,
    ) -> i32 {
        if self.asyn_name != "IMAGE_MODE" {
            return input;
        }
        match direction {
            Convert::ToEpics => {
                // If any mode is not supported then the readback is ambiguous;
                // keep the current EPICS value.
                if modes.single_frame.is_none()
                    || modes.multi_frame.is_none()
                    || modes.continuous.is_none()
                {
                    return self.param_as_i64(base) as i32;
                }
                if Some(input) == modes.continuous {
                    ImageMode::Continuous as i32
                } else if Some(input) == modes.multi_frame {
                    ImageMode::Multiple as i32
                } else if Some(input) == modes.single_frame {
                    ImageMode::Single as i32
                } else {
                    input
                }
            }
            Convert::FromEpics => {
                let continuous = modes.continuous.unwrap_or(-1);
                if input == ImageMode::Single as i32 {
                    // Some cameras, e.g. Mikrotron, don't support SingleFrame
                    modes.single_frame.unwrap_or(continuous)
                } else if input == ImageMode::Multiple as i32 {
                    // Some cameras, e.g. JAI, don't support MultiFrame
                    modes.multi_frame.unwrap_or(continuous)
                } else if input == ImageMode::Continuous as i32 {
                    continuous
                } else {
                    input
                }
            }
        }
    }

    /// The enum table for asynEnum `readEnum`, read live from the camera.
    pub fn enum_table(&self) -> NodeResult<Vec<(String, i32)>> {
        if !self.node.is_implemented() || !self.node.is_available() || !self.node.is_readable() {
            return Ok(vec![("N.A.".to_string(), 0)]);
        }
        self.node.read_enum_choices()
    }

    pub fn report(&self, out: &mut dyn std::fmt::Write, details: i32) {
        let _ = writeln!(out, "      Node name: {}", self.feature_name);
        let _ = writeln!(out, "          value: {}", self.value_as_string());
        if details <= 1 {
            return;
        }
        let n = &self.node;
        let _ = writeln!(out, "      asynIndex: {}", self.asyn_index);
        let _ = writeln!(out, "       asynName: {}", self.asyn_name);
        let _ = writeln!(out, "       asynType: {:?}", self.asyn_type);
        let _ = writeln!(out, "  isImplemented: {}", n.is_implemented());
        let _ = writeln!(out, "    isAvailable: {}", n.is_available());
        let _ = writeln!(out, "     isReadable: {}", n.is_readable());
        let _ = writeln!(out, "     isWritable: {}", n.is_writable());
        if n.feature_type() == GCFeatureType::Integer && n.is_readable() {
            let _ = writeln!(
                out,
                "        minimum: {}",
                n.read_integer_min().unwrap_or(0)
            );
            let _ = writeln!(
                out,
                "        maximum: {}",
                n.read_integer_max().unwrap_or(0)
            );
            let _ = writeln!(out, "      increment: {}", n.read_increment().unwrap_or(0));
        }
        if n.feature_type() == GCFeatureType::Double && n.is_readable() {
            let _ = writeln!(
                out,
                "        minimum: {}",
                n.read_double_min().unwrap_or(0.0)
            );
            let _ = writeln!(
                out,
                "        maximum: {}",
                n.read_double_max().unwrap_or(0.0)
            );
        }
        if n.feature_type() == GCFeatureType::Enum {
            for (i, (s, v)) in self.enum_choices.iter().enumerate() {
                let label = if i == 0 { "enums:" } else { "      " };
                let _ = writeln!(out, "          {label} {v}: {s}");
            }
        }
    }
}

/// Where a feature's value lands in the parameter library.
#[derive(Clone, Copy, Debug)]
pub struct FeatureBinding {
    pub asyn_index: usize,
    pub asyn_type: ParamType,
}

/// Feature name -> binding, readable from outside the port actor so that a
/// camera callback thread can turn a node value into a parameter update.
pub type FeatureIndex = Arc<RwLock<HashMap<String, FeatureBinding>>>;

#[derive(Default)]
pub struct GenICamFeatureSet {
    features: Vec<GenICamFeature>,
    by_name: HashMap<String, usize>,
    by_asyn: BTreeMap<usize, usize>,
    index: FeatureIndex,
    pub modes: AcquisitionModes,
}

impl GenICamFeatureSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn index(&self) -> FeatureIndex {
        self.index.clone()
    }

    /// The first feature bound to an asyn index keeps it, and the first
    /// feature inserted under a name is the one `get_by_name` returns.
    pub fn insert(&mut self, feature: GenICamFeature) {
        if self.by_asyn.contains_key(&feature.asyn_index) {
            return;
        }
        let slot = self.features.len();
        self.by_asyn.insert(feature.asyn_index, slot);
        if !feature.feature_name.is_empty() && !self.by_name.contains_key(&feature.feature_name) {
            self.by_name.insert(feature.feature_name.clone(), slot);
            self.index.write().insert(
                feature.feature_name.clone(),
                FeatureBinding {
                    asyn_index: feature.asyn_index,
                    asyn_type: feature.asyn_type,
                },
            );
        }
        self.features.push(feature);
    }

    pub fn get_by_name(&mut self, name: &str) -> Option<&mut GenICamFeature> {
        let slot = *self.by_name.get(name)?;
        self.features.get_mut(slot)
    }

    pub fn get_by_index(&mut self, index: usize) -> Option<&mut GenICamFeature> {
        let slot = *self.by_asyn.get(&index)?;
        self.features.get_mut(slot)
    }

    pub fn has_index(&self, index: usize) -> bool {
        self.by_asyn.contains_key(&index)
    }

    pub fn read_all(&mut self, base: &mut PortDriverBase) {
        let modes = self.modes;
        let slots: Vec<usize> = self.by_asyn.values().copied().collect();
        for slot in slots {
            self.features[slot].read(base, &modes, true);
        }
    }

    pub fn report(&self, out: &mut dyn std::fmt::Write, details: i32) {
        let _ = writeln!(out, "Feature list");
        let mut names: Vec<(&String, &usize)> = self.by_name.iter().collect();
        names.sort();
        for (_, slot) in names {
            let _ = writeln!(out);
            self.features[*slot].report(out, details);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use asyn_rs::param::ParamValue;
    use asyn_rs::port::PortFlags;
    use parking_lot::Mutex;

    /// An enum node whose table and availability the test moves.
    #[derive(Default)]
    struct EnumState {
        available: bool,
        choices: Vec<(String, i32)>,
    }

    #[derive(Clone, Default)]
    struct EnumNode(Arc<Mutex<EnumState>>);

    impl GenICamNode for EnumNode {
        fn feature_type(&self) -> GCFeatureType {
            GCFeatureType::Enum
        }
        fn is_implemented(&self) -> bool {
            true
        }
        fn is_available(&self) -> bool {
            self.0.lock().available
        }
        fn is_readable(&self) -> bool {
            true
        }
        fn is_writable(&self) -> bool {
            true
        }
        fn read_integer(&self) -> NodeResult<i64> {
            Err("enum".into())
        }
        fn read_integer_min(&self) -> NodeResult<i64> {
            Err("enum".into())
        }
        fn read_integer_max(&self) -> NodeResult<i64> {
            Err("enum".into())
        }
        fn read_increment(&self) -> NodeResult<i64> {
            Err("enum".into())
        }
        fn write_integer(&self, _: i64) -> NodeResult<()> {
            Err("enum".into())
        }
        fn read_boolean(&self) -> NodeResult<bool> {
            Err("enum".into())
        }
        fn write_boolean(&self, _: bool) -> NodeResult<()> {
            Err("enum".into())
        }
        fn read_double(&self) -> NodeResult<f64> {
            Err("enum".into())
        }
        fn read_double_min(&self) -> NodeResult<f64> {
            Err("enum".into())
        }
        fn read_double_max(&self) -> NodeResult<f64> {
            Err("enum".into())
        }
        fn write_double(&self, _: f64) -> NodeResult<()> {
            Err("enum".into())
        }
        fn read_enum_index(&self) -> NodeResult<i32> {
            Ok(self.0.lock().choices[0].1)
        }
        fn write_enum_index(&self, _: i32) -> NodeResult<()> {
            Ok(())
        }
        fn read_enum_string(&self) -> NodeResult<String> {
            Ok(self.0.lock().choices[0].0.clone())
        }
        fn read_enum_choices(&self) -> NodeResult<Vec<(String, i32)>> {
            Ok(self.0.lock().choices.clone())
        }
        fn read_string(&self) -> NodeResult<String> {
            Err("enum".into())
        }
        fn write_string(&self, _: &str) -> NodeResult<()> {
            Err("enum".into())
        }
        fn write_command(&self) -> NodeResult<()> {
            Err("enum".into())
        }
    }

    /// C fires `doCallbacksEnum` when, and only when, the table a read finds
    /// differs from the last one it published.
    #[test]
    fn a_read_publishes_the_enum_table_only_when_it_changed() {
        let mut base = PortDriverBase::new("gc_enum_table", 1, PortFlags::default());
        let index = base.create_param("GC_Mode", ParamType::Int32).unwrap();
        let mut tables = base.interrupts.subscribe_async();
        let mut published = move || {
            let mut out = Vec::new();
            while let Ok(iv) = tables.try_recv() {
                if let ParamValue::Enum { choices, .. } = iv.value {
                    out.push(choices.iter().map(|c| c.string.clone()).collect::<Vec<_>>());
                }
            }
            out
        };

        let node = EnumNode::default();
        *node.0.lock() = EnumState {
            available: true,
            choices: vec![("Off".into(), 0), ("On".into(), 1)],
        };
        let mut f = GenICamFeature::new(
            "GC_Mode",
            ParamType::Int32,
            index,
            "Mode",
            Box::new(node.clone()),
        );
        let modes = AcquisitionModes::default();

        assert!(f.read(&mut base, &modes, true));
        assert_eq!(published(), [["Off", "On"]], "first read");
        assert!(f.read(&mut base, &modes, true));
        assert!(published().is_empty(), "unchanged table");

        node.0.lock().choices.push(("Auto".into(), 2));
        assert!(f.read(&mut base, &modes, true));
        assert_eq!(published(), [["Off", "On", "Auto"]], "grown table");

        node.0.lock().available = false;
        assert!(f.read(&mut base, &modes, true));
        assert!(f.read(&mut base, &modes, true));
        assert_eq!(published(), [["N.A."]], "unavailable, published once");

        node.0.lock().available = true;
        assert!(f.read(&mut base, &modes, true));
        assert_eq!(published(), [["Off", "On", "Auto"]], "available again");
    }
}
