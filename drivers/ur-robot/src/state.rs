//! RTDE output variables: the type table and the data-package decoder.
//!
//! Ported from `ur_rtde/src/robot_state.cpp` (the `state_types_` map) and the
//! `RTDE_DATA_PACKAGE` branch of `RTDE::receiveData` (rtde.cpp:581).
//!
//! A data package carries no field names — it is a bare concatenation of values
//! in exactly the order the output recipe was registered. Decoding therefore
//! depends entirely on the recipe, which is why the recipe and the decoder live
//! together in [`OutputRecipe`].

use std::collections::HashMap;

/// Wire type of an RTDE output variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VarType {
    Double,
    Int32,
    Uint32,
    Uint64,
    /// Three packed `double`s.
    Vector3d,
    /// Six packed `double`s.
    Vector6d,
    /// Six packed `int32`s.
    Vector6Int32,
}

impl VarType {
    /// Encoded width in bytes.
    pub fn size(self) -> usize {
        match self {
            VarType::Int32 | VarType::Uint32 => 4,
            VarType::Double | VarType::Uint64 => 8,
            VarType::Vector3d => 24,
            VarType::Vector6d => 48,
            VarType::Vector6Int32 => 24,
        }
    }
}

/// A decoded value of one output variable.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Double(f64),
    Int32(i32),
    Uint32(u32),
    Uint64(u64),
    Doubles(Vec<f64>),
    Int32s(Vec<i32>),
}

impl Value {
    /// Scalar as `f64`, if this value is a scalar.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Double(v) => Some(*v),
            Value::Int32(v) => Some(*v as f64),
            Value::Uint32(v) => Some(*v as f64),
            Value::Uint64(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Scalar as `i32`, if this value is a scalar.
    pub fn as_i32(&self) -> Option<i32> {
        match self {
            Value::Double(v) => Some(*v as i32),
            Value::Int32(v) => Some(*v),
            // The digital-I/O bit words and `runtime_state` are unsigned on the
            // wire but land in asyn `Int32` params; keep the bit pattern rather
            // than saturating, so bit 31 survives the trip.
            Value::Uint32(v) => Some(*v as i32),
            Value::Uint64(v) => Some(*v as i32),
            _ => None,
        }
    }

    /// Scalar as `u32`, if this value is a scalar.
    pub fn as_u32(&self) -> Option<u32> {
        match self {
            Value::Uint32(v) => Some(*v),
            Value::Int32(v) => Some(*v as u32),
            Value::Uint64(v) => Some(*v as u32),
            Value::Double(v) => Some(*v as u32),
            _ => None,
        }
    }

    /// Vector of `f64`, if this value is a double vector.
    pub fn as_f64s(&self) -> Option<&[f64]> {
        match self {
            Value::Doubles(v) => Some(v),
            _ => None,
        }
    }

    /// Vector of `i32`, if this value is an int vector.
    pub fn as_i32s(&self) -> Option<&[i32]> {
        match self {
            Value::Int32s(v) => Some(v),
            _ => None,
        }
    }
}

/// Look up the wire type of an RTDE output variable name.
///
/// This is `RobotState::state_types_` (robot_state.cpp:6). The three-element
/// vectors are not distinguished by the map in the C++ — `receiveData` special-
/// cases four names by string compare (rtde.cpp:597) — so that rule is folded
/// into this single table instead, which removes the dual meaning of
/// "`vector<double>` = 6 elements, except when it is 3".
pub fn var_type(name: &str) -> Option<VarType> {
    use VarType::*;

    // Output registers: output_int_register_N (0..=47), output_double_register_N.
    if let Some(n) = name.strip_prefix("output_int_register_") {
        return n.parse::<u32>().ok().filter(|n| *n < 48).map(|_| Int32);
    }
    if let Some(n) = name.strip_prefix("output_double_register_") {
        return n.parse::<u32>().ok().filter(|n| *n < 48).map(|_| Double);
    }

    Some(match name {
        "timestamp" => Double,
        "actual_execution_time" => Double,
        "speed_scaling" => Double,
        "target_speed_fraction" => Double,
        "actual_momentum" => Double,
        "actual_main_voltage" => Double,
        "actual_robot_voltage" => Double,
        "actual_robot_current" => Double,
        "standard_analog_input0" => Double,
        "standard_analog_input1" => Double,
        "standard_analog_output0" => Double,
        "standard_analog_output1" => Double,
        "payload" => Double,

        "robot_mode" => Int32,
        "safety_mode" => Int32,

        "runtime_state" => Uint32,
        "robot_status_bits" => Uint32,
        "safety_status_bits" => Uint32,
        "output_bit_registers0_to_31" => Uint32,
        "output_bit_registers32_to_63" => Uint32,

        "actual_digital_input_bits" => Uint64,
        "actual_digital_output_bits" => Uint64,

        // rtde.cpp:597 — the only vector<double> entries decoded as 3 elements.
        "actual_tool_accelerometer" => Vector3d,
        "payload_cog" => Vector3d,
        "elbow_position" => Vector3d,
        "elbow_velocity" => Vector3d,

        "target_q" => Vector6d,
        "target_qd" => Vector6d,
        "target_qdd" => Vector6d,
        "target_current" => Vector6d,
        "target_moment" => Vector6d,
        "actual_q" => Vector6d,
        "actual_qd" => Vector6d,
        "actual_qdd" => Vector6d,
        "actual_current" => Vector6d,
        "actual_moment" => Vector6d,
        "joint_control_output" => Vector6d,
        "actual_TCP_pose" => Vector6d,
        "actual_TCP_speed" => Vector6d,
        "actual_TCP_force" => Vector6d,
        "target_TCP_pose" => Vector6d,
        "target_TCP_speed" => Vector6d,
        "joint_temperatures" => Vector6d,
        "actual_joint_voltage" => Vector6d,
        "ft_raw_wrench" => Vector6d,
        "payload_inertia" => Vector6d,
        "actual_current_as_torque" => Vector6d,

        "joint_mode" => Vector6Int32,

        _ => return None,
    })
}

/// The registered output recipe: the ordered variable list a data package
/// decodes against.
#[derive(Debug, Clone, Default)]
pub struct OutputRecipe {
    names: Vec<String>,
    types: Vec<VarType>,
}

/// Failure while decoding an `RTDE_DATA_PACKAGE`.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum DecodeError {
    #[error("data package truncated: variable '{name}' needs {need} bytes, {have} left")]
    Truncated {
        name: String,
        need: usize,
        have: usize,
    },
    #[error("data package is empty (no recipe-id byte)")]
    Empty,
}

impl OutputRecipe {
    /// Build a recipe from variable names, dropping any the type table does not
    /// know. The controller answers `NOT_FOUND` for those, so keeping them would
    /// desynchronise the decode offsets against a package that never carries them.
    pub fn new(names: &[String]) -> Self {
        let mut kept = Vec::new();
        let mut types = Vec::new();
        for name in names {
            if let Some(t) = var_type(name) {
                kept.push(name.clone());
                types.push(t);
            } else {
                log::warn!("ur-robot: unknown RTDE output variable '{name}', ignoring");
            }
        }
        Self { names: kept, types }
    }

    /// The variable names, in recipe order.
    pub fn names(&self) -> &[String] {
        &self.names
    }

    /// Total encoded size of one data package body, excluding the recipe-id byte.
    pub fn encoded_len(&self) -> usize {
        self.types.iter().map(|t| t.size()).sum()
    }

    /// Decode a data-package payload (recipe-id byte first, then the values).
    ///
    /// The C++ reads each field with no bounds check (`RTDEUtility::getDouble`
    /// indexes the vector directly, rtde_utility.h:221); a short package walks
    /// off the end. Here a short package is an error and the partial state is
    /// discarded.
    pub fn decode(&self, payload: &[u8]) -> Result<HashMap<String, Value>, DecodeError> {
        // rtde.cpp:584 — the recipe id leads the body and is skipped.
        let body = payload.split_first().ok_or(DecodeError::Empty)?.1;

        let mut out = HashMap::with_capacity(self.names.len());
        let mut off = 0usize;

        for (name, &ty) in self.names.iter().zip(&self.types) {
            let need = ty.size();
            let rest = &body[off.min(body.len())..];
            if rest.len() < need {
                return Err(DecodeError::Truncated {
                    name: name.clone(),
                    need,
                    have: rest.len(),
                });
            }
            let raw = &rest[..need];
            let value = match ty {
                VarType::Double => Value::Double(f64_at(raw, 0)),
                VarType::Int32 => Value::Int32(i32_at(raw, 0)),
                VarType::Uint32 => Value::Uint32(u32_at(raw, 0)),
                VarType::Uint64 => Value::Uint64(u64_at(raw, 0)),
                VarType::Vector3d => Value::Doubles((0..3).map(|i| f64_at(raw, i * 8)).collect()),
                VarType::Vector6d => Value::Doubles((0..6).map(|i| f64_at(raw, i * 8)).collect()),
                VarType::Vector6Int32 => {
                    Value::Int32s((0..6).map(|i| i32_at(raw, i * 4)).collect())
                }
            };
            out.insert(name.clone(), value);
            off += need;
        }

        Ok(out)
    }
}

fn f64_at(b: &[u8], i: usize) -> f64 {
    f64::from_be_bytes(b[i..i + 8].try_into().expect("checked width"))
}
fn i32_at(b: &[u8], i: usize) -> i32 {
    i32::from_be_bytes(b[i..i + 4].try_into().expect("checked width"))
}
fn u32_at(b: &[u8], i: usize) -> u32 {
    u32::from_be_bytes(b[i..i + 4].try_into().expect("checked width"))
}
fn u64_at(b: &[u8], i: usize) -> u64 {
    u64::from_be_bytes(b[i..i + 8].try_into().expect("checked width"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_table_matches_robot_state_cpp() {
        assert_eq!(var_type("timestamp"), Some(VarType::Double));
        assert_eq!(var_type("robot_mode"), Some(VarType::Int32));
        assert_eq!(var_type("safety_mode"), Some(VarType::Int32));
        assert_eq!(var_type("runtime_state"), Some(VarType::Uint32));
        assert_eq!(var_type("safety_status_bits"), Some(VarType::Uint32));
        assert_eq!(var_type("actual_digital_input_bits"), Some(VarType::Uint64));
        assert_eq!(
            var_type("actual_digital_output_bits"),
            Some(VarType::Uint64)
        );
        assert_eq!(var_type("actual_q"), Some(VarType::Vector6d));
        assert_eq!(var_type("joint_mode"), Some(VarType::Vector6Int32));
        assert_eq!(
            var_type("actual_tool_accelerometer"),
            Some(VarType::Vector3d)
        );
        assert_eq!(var_type("payload_cog"), Some(VarType::Vector3d));
        assert_eq!(var_type("output_int_register_12"), Some(VarType::Int32));
        assert_eq!(var_type("output_double_register_47"), Some(VarType::Double));
        assert_eq!(var_type("output_int_register_48"), None);
        assert_eq!(var_type("no_such_variable"), None);
    }

    #[test]
    fn vector3d_variables_are_24_bytes_not_48() {
        // The C++ picks vector-3 vs vector-6 by name compare at rtde.cpp:597;
        // getting this wrong desynchronises every field after it.
        assert_eq!(VarType::Vector3d.size(), 24);
        assert_eq!(VarType::Vector6d.size(), 48);
        assert_eq!(VarType::Vector6Int32.size(), 24);
    }

    #[test]
    fn recipe_drops_unknown_names() {
        let r = OutputRecipe::new(&[
            "timestamp".into(),
            "bogus_variable".into(),
            "robot_mode".into(),
        ]);
        assert_eq!(r.names(), &["timestamp".to_string(), "robot_mode".into()]);
        assert_eq!(r.encoded_len(), 8 + 4);
    }

    #[test]
    fn decode_walks_fields_in_recipe_order() {
        let r = OutputRecipe::new(&[
            "timestamp".into(),
            "actual_q".into(),
            "robot_mode".into(),
            "runtime_state".into(),
            "actual_digital_input_bits".into(),
            "joint_mode".into(),
            "actual_tool_accelerometer".into(),
        ]);

        let mut p = vec![1u8]; // recipe id
        p.extend_from_slice(&12.5f64.to_be_bytes());
        for v in [
            0.0f64,
            -std::f64::consts::FRAC_PI_2,
            0.0,
            -std::f64::consts::FRAC_PI_2,
            0.0,
            0.5,
        ] {
            p.extend_from_slice(&v.to_be_bytes());
        }
        p.extend_from_slice(&7i32.to_be_bytes());
        p.extend_from_slice(&2u32.to_be_bytes());
        p.extend_from_slice(&0x0000_0000_dead_beefu64.to_be_bytes());
        for v in [253i32, 253, 253, 253, 253, 253] {
            p.extend_from_slice(&v.to_be_bytes());
        }
        for v in [0.1f64, 0.2, 9.8] {
            p.extend_from_slice(&v.to_be_bytes());
        }
        assert_eq!(p.len(), 1 + r.encoded_len());

        let s = r.decode(&p).unwrap();
        assert_eq!(s["timestamp"], Value::Double(12.5));
        assert_eq!(
            s["actual_q"].as_f64s().unwrap(),
            &[
                0.0,
                -std::f64::consts::FRAC_PI_2,
                0.0,
                -std::f64::consts::FRAC_PI_2,
                0.0,
                0.5
            ]
        );
        assert_eq!(s["robot_mode"], Value::Int32(7));
        assert_eq!(s["runtime_state"], Value::Uint32(2));
        assert_eq!(s["actual_digital_input_bits"], Value::Uint64(0xdead_beef));
        assert_eq!(s["joint_mode"].as_i32s().unwrap(), &[253; 6]);
        assert_eq!(s["actual_tool_accelerometer"].as_f64s().unwrap().len(), 3);
    }

    #[test]
    fn decode_rejects_truncated_package() {
        let r = OutputRecipe::new(&["timestamp".into(), "actual_q".into()]);
        let mut p = vec![1u8];
        p.extend_from_slice(&1.0f64.to_be_bytes());
        p.extend_from_slice(&[0u8; 40]); // actual_q needs 48
        match r.decode(&p) {
            Err(DecodeError::Truncated { name, need, have }) => {
                assert_eq!(name, "actual_q");
                assert_eq!(need, 48);
                assert_eq!(have, 40);
            }
            other => panic!("expected Truncated, got {other:?}"),
        }
        assert_eq!(r.decode(&[]), Err(DecodeError::Empty));
    }

    #[test]
    fn safety_status_bits_keep_high_bit() {
        let r = OutputRecipe::new(&["safety_status_bits".into()]);
        let mut p = vec![1u8];
        p.extend_from_slice(&0x8000_0001u32.to_be_bytes());
        let s = r.decode(&p).unwrap();
        assert_eq!(s["safety_status_bits"].as_u32(), Some(0x8000_0001));
        assert_eq!(s["safety_status_bits"].as_i32(), Some(-2147483647));
    }
}
