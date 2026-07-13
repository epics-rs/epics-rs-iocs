//! PLC type × asyn type conversion matrix.
//!
//! Mirrors C `adsUpdateParameter` (adsAsynPortDriver.cpp:4261) for PLC → EPICS
//! and `writeInt32`/`writeInt64`/`writeFloat64`/`write*Array` for EPICS → PLC.
//! The pairing is deliberately narrow: the C driver rejects a combination it
//! has no arm for rather than guessing a coercion, and so do we.
//!
//! Two deliberate divergences from C, both fixed at source:
//!
//! * **Float → integer overflow.** C casts `(int)(*ADST_REAL64Var)` — undefined
//!   behavior when the value exceeds the integer range, in practice whatever the
//!   target's cvttsd2si emits (`INT_MIN` on x86). Rust's `as` saturates, which
//!   is defined and monotone.
//! * **`ADST_BIT` with `asynInt64`.** C's BIT arm handles `asynParamInt32` and
//!   `asynParamFloat64` but omits `asynParamInt64` (adsAsynPortDriver.cpp:4600),
//!   even though every other integer PLC type accepts all three. A `longin` with
//!   `DTYP=asynInt64` bound to a `BOOL` therefore fails to convert with no
//!   reason it should. BIT is an `int8_t` on the wire, so the widening is the
//!   same one the INT8 arm already performs; the omission is a gap in an
//!   otherwise uniform matrix, not a semantic choice.

use std::fmt;
use std::sync::Arc;

use asyn_rs::param::{ParamType, ParamValue};

use crate::ads::defs::AdsType;

/// Why a value could not be converted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvertError {
    /// C: "Type combination not supported. PLC type = %s, ASYN type = %s".
    Unsupported { plc: AdsType, asyn: ParamType },
    /// The PLC returned fewer bytes than the type needs.
    ShortData {
        plc: AdsType,
        need: usize,
        got: usize,
    },
}

impl fmt::Display for ConvertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unsupported { plc, asyn } => write!(
                f,
                "type combination not supported: PLC type = {}, asyn type = {:?}",
                plc.as_str(),
                asyn
            ),
            Self::ShortData { plc, need, got } => write!(
                f,
                "{} needs {} bytes, PLC returned {}",
                plc.as_str(),
                need,
                got
            ),
        }
    }
}

impl std::error::Error for ConvertError {}

/// The asyn parameter type that matches a PLC type with no coercion.
///
/// Used when the bound record's DTYP does not pin the type. Arrays take the
/// element type's natural array flavour; the C driver has no unsigned array
/// arms, so a `UINT`/`UDINT` array widens to the next signed array that can
/// hold it without loss.
pub fn natural_param_type(plc: AdsType, is_array: bool) -> Option<ParamType> {
    Some(match (plc, is_array) {
        (AdsType::String, _) => ParamType::Octet,
        (AdsType::Bit | AdsType::Int8, false) => ParamType::Int32,
        (AdsType::Bit | AdsType::Int8, true) => ParamType::Int8Array,
        (AdsType::Int16, false) => ParamType::Int32,
        (AdsType::Int16, true) => ParamType::Int16Array,
        (AdsType::Int32, false) => ParamType::Int32,
        (AdsType::Int32, true) => ParamType::Int32Array,
        (AdsType::Int64, false) => ParamType::Int64,
        (AdsType::UInt8, false) => ParamType::Int32,
        (AdsType::UInt8, true) => ParamType::Int16Array,
        (AdsType::UInt16, false) => ParamType::Int32,
        (AdsType::UInt16, true) => ParamType::Int32Array,
        (AdsType::UInt32, false) => ParamType::Int64,
        (AdsType::UInt32, true) => ParamType::Int64Array,
        (AdsType::UInt64, false) => ParamType::Int64,
        (AdsType::Real32, false) => ParamType::Float64,
        (AdsType::Real32, true) => ParamType::Float32Array,
        (AdsType::Real64, false) => ParamType::Float64,
        (AdsType::Real64, true) => ParamType::Float64Array,
        // INT64/UINT64 arrays, REAL80, WSTRING, BIGTYPE, VOID and unknown ids
        // have no asyn arm in the C driver either.
        _ => return None,
    })
}

/// Read one scalar out of the PLC's little-endian bytes as `f64`.
///
/// Every scalar arm of the C matrix widens through the PLC's own type before
/// casting to the asyn type; going via `f64` would lose the low bits of a
/// 64-bit integer, so the integer path keeps `i128`.
enum Scalar {
    Int(i128),
    Float(f64),
}

/// The byte width of a PLC type that can appear as a scalar, or `None` for one
/// that cannot (`STRING`, `BIGTYPE`, `WSTRING`, `REAL80`, `VOID`, unknown ids) —
/// the C matrix has no scalar arm for any of those.
fn scalar_width(plc: AdsType) -> Option<usize> {
    match plc {
        AdsType::String | AdsType::WString | AdsType::BigType | AdsType::Void | AdsType::Real80 => {
            None
        }
        other => other.element_size().filter(|&n| n > 0),
    }
}

fn read_scalar(plc: AdsType, data: &[u8], need: usize) -> Result<Scalar, ConvertError> {
    if data.len() < need {
        return Err(ConvertError::ShortData {
            plc,
            need,
            got: data.len(),
        });
    }
    let d = &data[..need];
    Ok(match plc {
        // ADST_BIT is an int8_t in the C matrix, not a 0/1 bool.
        AdsType::Int8 | AdsType::Bit => Scalar::Int(d[0] as i8 as i128),
        AdsType::UInt8 => Scalar::Int(d[0] as i128),
        AdsType::Int16 => Scalar::Int(i16::from_le_bytes([d[0], d[1]]) as i128),
        AdsType::UInt16 => Scalar::Int(u16::from_le_bytes([d[0], d[1]]) as i128),
        AdsType::Int32 => Scalar::Int(i32::from_le_bytes([d[0], d[1], d[2], d[3]]) as i128),
        AdsType::UInt32 => Scalar::Int(u32::from_le_bytes([d[0], d[1], d[2], d[3]]) as i128),
        AdsType::Int64 => Scalar::Int(i64::from_le_bytes(d.try_into().unwrap()) as i128),
        AdsType::UInt64 => Scalar::Int(u64::from_le_bytes(d.try_into().unwrap()) as i128),
        AdsType::Real32 => Scalar::Float(f32::from_le_bytes([d[0], d[1], d[2], d[3]]) as f64),
        AdsType::Real64 => Scalar::Float(f64::from_le_bytes(d.try_into().unwrap())),
        _ => unreachable!("scalar_width admitted only the types handled above"),
    })
}

/// PLC bytes → a scalar `ParamValue` of the bound record's type.
pub fn decode_scalar(
    plc: AdsType,
    data: &[u8],
    target: ParamType,
) -> Result<ParamValue, ConvertError> {
    let unsupported = || ConvertError::Unsupported { plc, asyn: target };
    if !matches!(
        target,
        ParamType::Int32 | ParamType::Int64 | ParamType::Float64
    ) {
        return Err(unsupported());
    }
    let width = scalar_width(plc).ok_or_else(unsupported)?;
    let scalar = read_scalar(plc, data, width)?;
    Ok(match (scalar, target) {
        (Scalar::Int(v), ParamType::Int32) => ParamValue::Int32(v as i32),
        (Scalar::Int(v), ParamType::Int64) => ParamValue::Int64(v as i64),
        (Scalar::Int(v), ParamType::Float64) => ParamValue::Float64(v as f64),
        (Scalar::Float(v), ParamType::Int32) => ParamValue::Int32(v as i32),
        (Scalar::Float(v), ParamType::Int64) => ParamValue::Int64(v as i64),
        (Scalar::Float(v), ParamType::Float64) => ParamValue::Float64(v),
        _ => unreachable!("target was checked above"),
    })
}

/// PLC bytes → an array `ParamValue`.
///
/// `ADST_STRING` is the special case the C driver flags as an array: it is a
/// byte blob that a record binds either as `asynInt8Array` (waveform of CHAR)
/// or as `asynOctet` (stringin/lsi).
pub fn decode_array(
    plc: AdsType,
    data: &[u8],
    target: ParamType,
) -> Result<ParamValue, ConvertError> {
    let unsupported = || ConvertError::Unsupported { plc, asyn: target };

    if plc == AdsType::String {
        return match target {
            ParamType::Int8Array => Ok(ParamValue::Int8Array(
                data.iter().map(|&b| b as i8).collect(),
            )),
            ParamType::Octet => Ok(ParamValue::Octet(decode_plc_string(data))),
            _ => Err(unsupported()),
        };
    }

    // The C matrix pairs exactly one array flavour with each PLC type.
    let elem = plc
        .element_size()
        .filter(|&n| n > 0)
        .ok_or_else(unsupported)?;
    let chunks = data.chunks_exact(elem);
    Ok(match (plc, target) {
        (AdsType::Int8 | AdsType::Bit, ParamType::Int8Array) => {
            ParamValue::Int8Array(chunks.map(|c| c[0] as i8).collect())
        }
        (AdsType::Int16, ParamType::Int16Array) => ParamValue::Int16Array(
            chunks
                .map(|c| i16::from_le_bytes([c[0], c[1]]))
                .collect::<Arc<[i16]>>(),
        ),
        (AdsType::Int32, ParamType::Int32Array) => ParamValue::Int32Array(
            chunks
                .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect::<Arc<[i32]>>(),
        ),
        (AdsType::Real32, ParamType::Float32Array) => ParamValue::Float32Array(
            chunks
                .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect::<Arc<[f32]>>(),
        ),
        (AdsType::Real64, ParamType::Float64Array) => ParamValue::Float64Array(
            chunks
                .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
                .collect::<Arc<[f64]>>(),
        ),
        _ => return Err(unsupported()),
    })
}

/// A PLC `STRING` is a NUL-terminated byte array in a fixed-size slot; the
/// bytes past the NUL are stale and must not reach the record.
fn decode_plc_string(data: &[u8]) -> String {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    String::from_utf8_lossy(&data[..end]).into_owned()
}

/// A scalar written from EPICS, before it is narrowed to the PLC's type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WriteValue {
    Int(i64),
    Float(f64),
}

/// EPICS scalar → PLC bytes, narrowed to the PLC variable's own type.
pub fn encode_scalar(plc: AdsType, value: WriteValue) -> Result<Vec<u8>, ConvertError> {
    let as_i128 = match value {
        WriteValue::Int(v) => v as i128,
        WriteValue::Float(v) => v as i128,
    };
    let as_f64 = match value {
        WriteValue::Int(v) => v as f64,
        WriteValue::Float(v) => v,
    };
    Ok(match plc {
        // Writing a BOOL: the PLC expects 0 or 1, and TwinCAT treats any
        // non-zero byte as TRUE, so a plain narrowing (which maps 256 → 0)
        // would turn a true value false. Normalize instead.
        AdsType::Bit => vec![u8::from(as_i128 != 0)],
        AdsType::Int8 => (as_i128 as i8).to_le_bytes().to_vec(),
        AdsType::UInt8 => (as_i128 as u8).to_le_bytes().to_vec(),
        AdsType::Int16 => (as_i128 as i16).to_le_bytes().to_vec(),
        AdsType::UInt16 => (as_i128 as u16).to_le_bytes().to_vec(),
        AdsType::Int32 => (as_i128 as i32).to_le_bytes().to_vec(),
        AdsType::UInt32 => (as_i128 as u32).to_le_bytes().to_vec(),
        AdsType::Int64 => (as_i128 as i64).to_le_bytes().to_vec(),
        AdsType::UInt64 => (as_i128 as u64).to_le_bytes().to_vec(),
        AdsType::Real32 => (as_f64 as f32).to_le_bytes().to_vec(),
        AdsType::Real64 => as_f64.to_le_bytes().to_vec(),
        _ => {
            return Err(ConvertError::Unsupported {
                plc,
                asyn: match value {
                    WriteValue::Int(_) => ParamType::Int32,
                    WriteValue::Float(_) => ParamType::Float64,
                },
            });
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn i32_of(v: &ParamValue) -> i32 {
        match v {
            ParamValue::Int32(x) => *x,
            other => panic!("expected Int32, got {other:?}"),
        }
    }
    fn i64_of(v: &ParamValue) -> i64 {
        match v {
            ParamValue::Int64(x) => *x,
            other => panic!("expected Int64, got {other:?}"),
        }
    }
    fn f64_of(v: &ParamValue) -> f64 {
        match v {
            ParamValue::Float64(x) => *x,
            other => panic!("expected Float64, got {other:?}"),
        }
    }

    /// The full C scalar matrix: every integer PLC type feeds Int32/Int64/Float64.
    #[test]
    fn integer_plc_types_feed_all_three_scalar_asyn_types() {
        let cases: &[(AdsType, Vec<u8>, i64)] = &[
            (AdsType::Int8, vec![0xFF], -1),
            (AdsType::UInt8, vec![0xFF], 255),
            (AdsType::Int16, (-2i16).to_le_bytes().to_vec(), -2),
            (AdsType::UInt16, 65535u16.to_le_bytes().to_vec(), 65535),
            (AdsType::Int32, (-3i32).to_le_bytes().to_vec(), -3),
            (
                AdsType::UInt32,
                4_000_000_000u32.to_le_bytes().to_vec(),
                4_000_000_000,
            ),
            (AdsType::Int64, (-4i64).to_le_bytes().to_vec(), -4),
            (AdsType::Bit, vec![1], 1),
        ];
        for (plc, bytes, expect) in cases {
            assert_eq!(
                i64_of(&decode_scalar(*plc, bytes, ParamType::Int64).unwrap()),
                *expect,
                "{plc:?} → Int64"
            );
            assert_eq!(
                f64_of(&decode_scalar(*plc, bytes, ParamType::Float64).unwrap()),
                *expect as f64,
                "{plc:?} → Float64"
            );
            assert_eq!(
                i32_of(&decode_scalar(*plc, bytes, ParamType::Int32).unwrap()),
                *expect as i32,
                "{plc:?} → Int32"
            );
        }
    }

    #[test]
    fn bit_is_signed_int8_not_a_bool() {
        // C reads ADST_BIT through `int8_t*`; a PLC byte of 0xFF reaches the
        // record as -1, not 1.
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Bit, &[0xFF], ParamType::Int32).unwrap()),
            -1
        );
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Bit, &[0], ParamType::Int32).unwrap()),
            0
        );
    }

    /// The BIT/Int64 gap fixed at source (C has no `asynParamInt64` arm here).
    #[test]
    fn bit_converts_to_int64_like_every_other_integer_type() {
        assert_eq!(
            i64_of(&decode_scalar(AdsType::Bit, &[1], ParamType::Int64).unwrap()),
            1
        );
    }

    #[test]
    fn uint64_keeps_its_low_bits_through_int64() {
        // Going via f64 would round this to 2^64; the i128 path must not.
        let bytes = (u64::MAX - 1).to_le_bytes();
        assert_eq!(
            i64_of(&decode_scalar(AdsType::UInt64, &bytes, ParamType::Int64).unwrap()),
            -2,
            "u64::MAX-1 reinterpreted as i64, matching C's (epicsInt64) cast"
        );
    }

    #[test]
    fn real_types_convert() {
        let f32b = 1.5f32.to_le_bytes();
        assert_eq!(
            f64_of(&decode_scalar(AdsType::Real32, &f32b, ParamType::Float64).unwrap()),
            1.5
        );
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Real32, &f32b, ParamType::Int32).unwrap()),
            1,
            "truncation toward zero, as C's (int) cast"
        );
        let f64b = (-2.9f64).to_le_bytes();
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Real64, &f64b, ParamType::Int32).unwrap()),
            -2
        );
    }

    /// C's `(int)(double)` is UB past INT_MAX; Rust saturates.
    #[test]
    fn float_to_int_saturates_instead_of_invoking_ub() {
        let huge = 1e30f64.to_le_bytes();
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Real64, &huge, ParamType::Int32).unwrap()),
            i32::MAX
        );
        let tiny = (-1e30f64).to_le_bytes();
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Real64, &tiny, ParamType::Int32).unwrap()),
            i32::MIN
        );
        let nan = f64::NAN.to_le_bytes();
        assert_eq!(
            i32_of(&decode_scalar(AdsType::Real64, &nan, ParamType::Int32).unwrap()),
            0
        );
    }

    #[test]
    fn unsupported_scalar_pairings_are_rejected() {
        // Arrays are not scalars.
        assert!(matches!(
            decode_scalar(AdsType::Int32, &[0; 4], ParamType::Int32Array),
            Err(ConvertError::Unsupported { .. })
        ));
        // C has no arm mapping a STRING to a scalar.
        assert!(matches!(
            decode_scalar(AdsType::String, b"abc", ParamType::Int32),
            Err(ConvertError::Unsupported { .. })
        ));
        // Nor for a struct.
        assert!(matches!(
            decode_scalar(AdsType::BigType, &[0; 8], ParamType::Float64),
            Err(ConvertError::Unsupported { .. })
        ));
    }

    #[test]
    fn short_data_is_rejected() {
        assert!(matches!(
            decode_scalar(AdsType::Int32, &[1, 2], ParamType::Int32),
            Err(ConvertError::ShortData {
                need: 4,
                got: 2,
                ..
            })
        ));
    }

    #[test]
    fn arrays_decode_to_their_paired_flavour() {
        let v = decode_array(
            AdsType::Real64,
            &[1.0f64.to_le_bytes(), 2.5f64.to_le_bytes()].concat(),
            ParamType::Float64Array,
        )
        .unwrap();
        match v {
            ParamValue::Float64Array(a) => assert_eq!(&*a, &[1.0, 2.5]),
            other => panic!("{other:?}"),
        }

        let v = decode_array(
            AdsType::Int16,
            &[(-1i16).to_le_bytes(), 300i16.to_le_bytes()].concat(),
            ParamType::Int16Array,
        )
        .unwrap();
        match v {
            ParamValue::Int16Array(a) => assert_eq!(&*a, &[-1, 300]),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn mismatched_array_flavour_is_rejected() {
        // A REAL64 array cannot feed an asynInt32Array record.
        assert!(matches!(
            decode_array(AdsType::Real64, &[0; 16], ParamType::Int32Array),
            Err(ConvertError::Unsupported { .. })
        ));
        // C supports no unsigned array arms.
        assert!(matches!(
            decode_array(AdsType::UInt32, &[0; 8], ParamType::Int32Array),
            Err(ConvertError::Unsupported { .. })
        ));
    }

    #[test]
    fn trailing_partial_element_is_dropped_not_misread() {
        // 5 bytes of an INT array is two elements and a stray byte.
        let v = decode_array(AdsType::Int16, &[1, 0, 2, 0, 9], ParamType::Int16Array).unwrap();
        match v {
            ParamValue::Int16Array(a) => assert_eq!(&*a, &[1, 2]),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plc_string_stops_at_the_nul() {
        // The PLC sends the whole fixed-size slot; the tail is stale data.
        let raw = b"Hello\0\xff\xfe garbage";
        match decode_array(AdsType::String, raw, ParamType::Octet).unwrap() {
            ParamValue::Octet(s) => assert_eq!(s, "Hello"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plc_string_as_int8_array_keeps_every_byte() {
        // A waveform of CHAR gets the raw slot, NUL and all — that is what the
        // C `asynParamInt8Array` arm hands to doCallbacksInt8Array.
        match decode_array(AdsType::String, b"Hi\0", ParamType::Int8Array).unwrap() {
            ParamValue::Int8Array(a) => assert_eq!(&*a, &[b'H' as i8, b'i' as i8, 0]),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn writes_narrow_to_the_plc_type() {
        assert_eq!(
            encode_scalar(AdsType::Int16, WriteValue::Int(-2)).unwrap(),
            (-2i16).to_le_bytes()
        );
        assert_eq!(
            encode_scalar(AdsType::Real32, WriteValue::Float(1.5)).unwrap(),
            1.5f32.to_le_bytes()
        );
        assert_eq!(
            encode_scalar(AdsType::Real64, WriteValue::Int(3)).unwrap(),
            3.0f64.to_le_bytes()
        );
        assert_eq!(
            encode_scalar(AdsType::UInt32, WriteValue::Float(7.9)).unwrap(),
            7u32.to_le_bytes()
        );
    }

    #[test]
    fn writing_a_bool_normalizes_to_zero_or_one() {
        assert_eq!(
            encode_scalar(AdsType::Bit, WriteValue::Int(0)).unwrap(),
            [0]
        );
        assert_eq!(
            encode_scalar(AdsType::Bit, WriteValue::Int(1)).unwrap(),
            [1]
        );
        // A plain `as u8` narrowing would send 0 here and silently clear the
        // BOOL the record just set true.
        assert_eq!(
            encode_scalar(AdsType::Bit, WriteValue::Int(256)).unwrap(),
            [1]
        );
        assert_eq!(
            encode_scalar(AdsType::Bit, WriteValue::Int(-1)).unwrap(),
            [1]
        );
    }

    #[test]
    fn writing_an_unsupported_plc_type_is_rejected() {
        assert!(matches!(
            encode_scalar(AdsType::BigType, WriteValue::Int(1)),
            Err(ConvertError::Unsupported { .. })
        ));
        assert!(matches!(
            encode_scalar(AdsType::String, WriteValue::Float(1.0)),
            Err(ConvertError::Unsupported { .. })
        ));
    }

    #[test]
    fn natural_types_follow_the_c_matrix() {
        assert_eq!(
            natural_param_type(AdsType::Real64, false),
            Some(ParamType::Float64)
        );
        assert_eq!(
            natural_param_type(AdsType::Int32, false),
            Some(ParamType::Int32)
        );
        assert_eq!(
            natural_param_type(AdsType::Real64, true),
            Some(ParamType::Float64Array)
        );
        assert_eq!(
            natural_param_type(AdsType::String, true),
            Some(ParamType::Octet)
        );
        // No asyn arm exists for these in C either.
        assert_eq!(natural_param_type(AdsType::BigType, false), None);
        assert_eq!(natural_param_type(AdsType::Real80, false), None);
        assert_eq!(natural_param_type(AdsType::Int64, true), None);
    }
}
