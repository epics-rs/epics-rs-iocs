//! Conversion between OPC UA `Variant`s and EPICS record values
//! (`DataElementOpen62541Leaf.h` `readScalar`/`readArray`/`writeScalar`/
//! `writeArray`, `DataElementOpen62541Leaf.cpp:300-1352`).
//!
//! Reading converts the *incoming* variant to what the record asks for, with a
//! range check that rejects a value the EPICS type cannot hold. Writing goes the
//! other way: the outgoing variant takes the type of the last value received
//! from the server, so the record never has to know the node's data type.
//!
//! The C expresses the range checks as an `isWithinRange<TO, FROM>` template
//! with 40 explicit specializations (`DataElementOpen62541Leaf.h:48-147`), one
//! per pair of C++ integer types, because C++'s usual arithmetic conversions
//! make the naive comparison wrong. Here every incoming numeric is first widened
//! into [`Number`] — signed, unsigned or float — and one checked conversion per
//! target closes the whole matrix.

use std::collections::BTreeMap;

use async_opcua::types::{
    Array, ByteString, DateTime, LocalizedText, QualifiedName, UAString, Variant,
    VariantScalarTypeId, VariantTypeId, XmlElement,
};

/// `EnumChoices` (`DataElement.h:25`) — an enumeration's value → name map, as
/// read from the server's type dictionary.
pub type EnumChoices = BTreeMap<u32, String>;

/// Why a value could not be moved between the record and the wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConvError {
    /// The value does not fit the target type (`READ_ALARM`/`WRITE_ALARM`,
    /// `INVALID`; C logs "out-of-bounds"/"value out of range").
    OutOfRange { value: String, target: &'static str },
    /// The OPC UA type has no conversion to/from this EPICS type at all
    /// (C logs "unsupported type kind"/"unsupported conversion").
    Unsupported { from: String, to: String },
    /// Incoming data is a scalar where an array was expected, or the reverse.
    NotAnArray,
    /// Array element type differs from the record's FTVL. The C requires an
    /// exact match for numeric arrays (`DataElementOpen62541Leaf.h:915`).
    ArrayTypeMismatch {
        incoming: String,
        target: &'static str,
    },
    /// The value is outside the enumeration's choices.
    NotAnEnumChoice { value: String },
}

impl std::fmt::Display for ConvError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConvError::OutOfRange { value, target } => {
                write!(f, "incoming data ({value}) out-of-bounds for {target}")
            }
            ConvError::Unsupported { from, to } => {
                write!(f, "unsupported conversion from {from} to {to}")
            }
            ConvError::NotAnArray => f.write_str("incoming data is not an array"),
            ConvError::ArrayTypeMismatch { incoming, target } => write!(
                f,
                "incoming data type ({incoming}) does not match EPICS array type ({target})"
            ),
            ConvError::NotAnEnumChoice { value } => {
                write!(f, "value {value} is not a choice of this enumeration")
            }
        }
    }
}

impl std::error::Error for ConvError {}

pub type Result<T> = std::result::Result<T, ConvError>;

/// Every OPC UA numeric widened into the three shapes EPICS can convert from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Number {
    Int(i64),
    Uint(u64),
    Float(f64),
}

impl std::fmt::Display for Number {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Number::Int(v) => write!(f, "{v}"),
            Number::Uint(v) => write!(f, "{v}"),
            Number::Float(v) => write!(f, "{v}"),
        }
    }
}

/// An EPICS scalar the device support can ask a variant to be read into
/// (`readScalar(epicsInt32|epicsInt64|epicsUInt32|epicsFloat64*)`).
pub trait ScalarTarget: Copy + Sized + std::fmt::Display {
    const NAME: &'static str;
    fn from_number(n: Number) -> Option<Self>;
    /// Parse the text of a STRING/LOCALIZEDTEXT node (`string_to`,
    /// `DataElementOpen62541Leaf.h:150-185`). Integers take a base prefix, as
    /// `strtol(s, 0, 0)` does.
    fn from_text(s: &str) -> Option<Self>;
    /// The record's own value, on the way out to the server.
    fn to_number(self) -> Number;
}

/// Range-checked float → integer. The C checks `!(v < lowest || v > max)` and
/// then `static_cast`s (`DataElementOpen62541Leaf.h:48-51`), which admits two
/// values it cannot convert:
///
/// * NaN and ±inf — both comparisons are false for NaN, so the check passes and
///   the cast is undefined behaviour;
/// * exactly 2^63 for an `epicsInt64` target — `INT64_MAX` converted to `double`
///   rounds *up* to 2^63, so the upper bound admits a value one past the range.
///
/// Both are rejected here, as out-of-range.
fn float_to_int(v: f64, lo: f64, hi_exclusive: f64) -> Option<i128> {
    if !v.is_finite() || v < lo || v >= hi_exclusive {
        return None;
    }
    Some(v.trunc() as i128)
}

macro_rules! impl_int_target {
    ($t:ty, $name:literal, $lo:expr, $hi_exclusive:expr) => {
        impl ScalarTarget for $t {
            const NAME: &'static str = $name;

            fn from_number(n: Number) -> Option<Self> {
                match n {
                    Number::Int(v) => <$t>::try_from(v).ok(),
                    Number::Uint(v) => <$t>::try_from(v).ok(),
                    Number::Float(v) => {
                        float_to_int(v, $lo, $hi_exclusive).and_then(|v| <$t>::try_from(v).ok())
                    }
                }
            }

            fn from_text(s: &str) -> Option<Self> {
                parse_int_base0(s).and_then(|v| <$t>::try_from(v).ok())
            }

            fn to_number(self) -> Number {
                #[allow(irrefutable_let_patterns)]
                if let Ok(v) = i64::try_from(self) {
                    Number::Int(v)
                } else {
                    Number::Uint(self as u64)
                }
            }
        }
    };
}

impl_int_target!(i32, "epicsInt32", -2147483648.0, 2147483648.0);
impl_int_target!(u32, "epicsUInt32", 0.0, 4294967296.0);
impl_int_target!(
    i64,
    "epicsInt64",
    -9223372036854775808.0,
    9223372036854775808.0
);

impl ScalarTarget for f64 {
    const NAME: &'static str = "epicsFloat64";

    fn from_number(n: Number) -> Option<Self> {
        Some(match n {
            Number::Int(v) => v as f64,
            Number::Uint(v) => v as f64,
            Number::Float(v) => v,
        })
    }

    fn from_text(s: &str) -> Option<Self> {
        s.trim().parse().ok()
    }

    fn to_number(self) -> Number {
        Number::Float(self)
    }
}

/// `strtol(s, nullptr, 0)` — decimal, `0x` hex, leading-`0` octal, signed.
fn parse_int_base0(s: &str) -> Option<i128> {
    let s = s.trim();
    let (negative, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s.strip_prefix('+').unwrap_or(s)),
    };
    let magnitude = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        i128::from_str_radix(hex, 16).ok()?
    } else if digits.len() > 1 && digits.starts_with('0') {
        i128::from_str_radix(&digits[1..], 8).ok()?
    } else {
        digits.parse().ok()?
    };
    Some(if negative { -magnitude } else { magnitude })
}

/// The scalar payload of a variant, as a number — the common part of the C's
/// per-type-kind switch in `readScalar`.
fn scalar_number(v: &Variant) -> Option<Number> {
    Some(match v {
        Variant::Boolean(b) => Number::Uint(u64::from(*b)),
        Variant::SByte(x) => Number::Int(i64::from(*x)),
        Variant::Byte(x) => Number::Uint(u64::from(*x)),
        Variant::Int16(x) => Number::Int(i64::from(*x)),
        Variant::UInt16(x) => Number::Uint(u64::from(*x)),
        Variant::Int32(x) => Number::Int(i64::from(*x)),
        Variant::UInt32(x) => Number::Uint(u64::from(*x)),
        Variant::Int64(x) => Number::Int(*x),
        Variant::UInt64(x) => Number::Uint(*x),
        Variant::Float(x) => Number::Float(f64::from(*x)),
        Variant::Double(x) => Number::Float(*x),
        _ => return None,
    })
}

/// The text payload of a variant, for the STRING and LOCALIZEDTEXT arms of the
/// C's numeric `readScalar` and for the whole of its string `readScalar`.
fn scalar_text(v: &Variant) -> Option<String> {
    Some(match v {
        Variant::String(s) => s.as_ref().to_string(),
        Variant::XmlElement(x) => x.to_string(),
        Variant::LocalizedText(lt) => lt.text.as_ref().to_string(),
        Variant::QualifiedName(qn) => qn.name.as_ref().to_string(),
        _ => return None,
    })
}

/// Name of the variant's type, for the error messages
/// (`variantTypeString`, `DataElementOpen62541.h:36`).
pub fn type_name(v: &Variant) -> String {
    match v {
        Variant::Empty => "Null".to_string(),
        Variant::Array(a) => format!("{:?}[{}]", a.value_type, a.values.len()),
        other => match other.type_id() {
            VariantTypeId::Scalar(id) => format!("{id:?}"),
            other => format!("{other:?}"),
        },
    }
}

fn describe(v: &Variant) -> String {
    match scalar_number(v) {
        Some(n) => format!("{} {n}", type_name(v)),
        None => type_name(v),
    }
}

/// Read a scalar variant into an EPICS numeric type (`readScalar<ET>`,
/// `DataElementOpen62541Leaf.h:697-861`).
///
/// `choices` is set when the node's data type is an enumeration; then the value
/// must be one of its choices, exactly as the C checks against `enumChoices`
/// (`DataElementOpen62541Leaf.h:806`).
pub fn read_scalar<T: ScalarTarget>(v: &Variant, choices: Option<&EnumChoices>) -> Result<T> {
    if let Some(n) = scalar_number(v) {
        let value = T::from_number(n).ok_or_else(|| ConvError::OutOfRange {
            value: describe(v),
            target: T::NAME,
        })?;
        if let Some(choices) = choices
            && !is_choice(choices, n)
        {
            return Err(ConvError::NotAnEnumChoice {
                value: n.to_string(),
            });
        }
        return Ok(value);
    }

    match scalar_text(v) {
        // `readScalar<ET>` accepts STRING and LOCALIZEDTEXT only; a
        // QUALIFIEDNAME reaches its `default` arm, so keep it out of here.
        Some(text) if !matches!(v, Variant::QualifiedName(_)) => {
            T::from_text(&text).ok_or_else(|| ConvError::OutOfRange {
                value: format!("String '{text}'"),
                target: T::NAME,
            })
        }
        _ => Err(ConvError::Unsupported {
            from: type_name(v),
            to: T::NAME.to_string(),
        }),
    }
}

fn is_choice(choices: &EnumChoices, n: Number) -> bool {
    let key = match n {
        Number::Int(v) => u32::try_from(v).ok(),
        Number::Uint(v) => u32::try_from(v).ok(),
        Number::Float(v) => (v >= 0.0 && v <= f64::from(u32::MAX)).then_some(v as u32),
    };
    key.is_some_and(|k| choices.contains_key(&k))
}

/// Read any variant as text (`readScalar(char*, len)`,
/// `DataElementOpen62541Leaf.cpp:300-455`) — the stringin/stringout/lsi/lso path.
///
/// Deviation from the C, documented rather than reproduced: the C falls back to
/// open62541's `UA_print` for every type kind it does not special-case, so the
/// exact text of an exotic type (Guid, NodeId, StatusCode, ...) is that
/// library's rendering. This port formats those with the `async-opcua` type's
/// own `Display`, which is not byte-identical for every type.
pub fn read_string(v: &Variant, choices: Option<&EnumChoices>) -> Result<String> {
    // An enumeration prints as its choice name; a value with no matching choice
    // falls through to the numeric text (`DataElementOpen62541Leaf.cpp:411-421`).
    if let (Some(choices), Some(n)) = (choices, scalar_number(v))
        && let Some(name) = enum_choice_name(choices, n)
    {
        return Ok(name.to_string());
    }

    Ok(match v {
        Variant::String(s) => s.as_ref().to_string(),
        Variant::XmlElement(x) => x.to_string(),
        Variant::LocalizedText(lt) => lt.text.as_ref().to_string(),
        Variant::QualifiedName(qn) => qn.name.as_ref().to_string(),
        Variant::ByteString(bs) => hex_encode(bs.as_ref()),
        Variant::DateTime(dt) => format_local_time(dt),
        // The C hands the raw byte(s) to the record's char buffer
        // (`DataElementOpen62541Leaf.cpp:402-409`).
        Variant::Byte(b) => String::from_utf8_lossy(&[*b]).into_owned(),
        Variant::SByte(b) => String::from_utf8_lossy(&[*b as u8]).into_owned(),
        Variant::Boolean(b) => b.to_string(),
        Variant::Float(x) => x.to_string(),
        Variant::Double(x) => x.to_string(),
        Variant::Guid(g) => g.to_string(),
        Variant::NodeId(id) => id.to_string(),
        Variant::StatusCode(s) => format!("{s}"),
        Variant::Array(a) if matches!(a.value_type, VariantScalarTypeId::Byte) => {
            let bytes: Vec<u8> = a
                .values
                .iter()
                .filter_map(|e| match e {
                    Variant::Byte(b) => Some(*b),
                    _ => None,
                })
                .collect();
            String::from_utf8_lossy(&bytes).into_owned()
        }
        other => match scalar_number(other) {
            Some(n) => n.to_string(),
            None => {
                return Err(ConvError::Unsupported {
                    from: type_name(other),
                    to: "CString".to_string(),
                });
            }
        },
    })
}

fn enum_choice_name(choices: &EnumChoices, n: Number) -> Option<&str> {
    let key = match n {
        Number::Int(v) => u32::try_from(v).ok()?,
        Number::Uint(v) => u32::try_from(v).ok()?,
        Number::Float(_) => return None,
    };
    choices.get(&key).map(String::as_str)
}

/// The C prints a DateTime in *local* time — it adds
/// `UA_DateTime_localTimeUtcOffset()` before formatting
/// (`DataElementOpen62541Leaf.cpp:393-400`).
fn format_local_time(dt: &DateTime) -> String {
    dt.as_chrono()
        .with_timezone(&chrono::Local)
        .format("%Y-%m-%dT%H:%M:%S%.9f")
        .to_string()
}

/// `printByteString` (`DataElementOpen62541Leaf.cpp:244-255`) — upper-case hex,
/// no separators.
pub fn hex_encode(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push(
            char::from_digit(u32::from(b >> 4), 16)
                .expect("nibble")
                .to_ascii_uppercase(),
        );
        s.push(
            char::from_digit(u32::from(b & 0xf), 16)
                .expect("nibble")
                .to_ascii_uppercase(),
        );
    }
    s
}

/// `parseByteString` (`DataElementOpen62541Leaf.cpp:257-296`) — hex digits in
/// pairs, blanks separate groups, and a group may carry a single odd digit
/// (`"1 23"` is `01 23`, while `"123"` is rejected: 12|3 or 1|23 is ambiguous).
pub fn hex_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(s.len().div_ceil(2));
    for group in s.split_whitespace() {
        let digits: Vec<u32> = group
            .chars()
            .map(|c| c.to_digit(16))
            .collect::<Option<_>>()?;
        // Digits pair up left to right, so a group of odd length leaves a digit
        // over at the end with no byte boundary to belong to. Only a group that
        // is a single digit — the whole group — is unambiguous.
        match digits.as_slice() {
            [single] => out.push(*single as u8),
            digits if digits.len() % 2 == 1 => return None,
            digits => {
                for pair in digits.chunks_exact(2) {
                    out.push(((pair[0] << 4) | pair[1]) as u8);
                }
            }
        }
    }
    Some(out)
}

/// Read an array variant into a `Vec` of EPICS array elements.
///
/// The C requires the OPC UA element type to match the record's FTVL exactly
/// (`DataElementOpen62541Leaf.h:915-919`); only the `epicsUInt8` target is wider,
/// accepting Byte, Boolean and a scalar ByteString
/// (`DataElementOpen62541Leaf.cpp:609-694`).
pub trait ArrayTarget: Sized {
    const NAME: &'static str;
    const ELEMENT: VariantScalarTypeId;
    fn from_element(v: &Variant) -> Option<Self>;
    fn to_element(&self) -> Variant;
}

macro_rules! impl_array_target {
    ($t:ty, $name:literal, $id:ident, $variant:ident) => {
        impl ArrayTarget for $t {
            const NAME: &'static str = $name;
            const ELEMENT: VariantScalarTypeId = VariantScalarTypeId::$id;

            fn from_element(v: &Variant) -> Option<Self> {
                match v {
                    Variant::$variant(x) => Some(*x),
                    _ => None,
                }
            }

            fn to_element(&self) -> Variant {
                Variant::$variant(*self)
            }
        }
    };
}

impl_array_target!(i8, "epicsInt8", SByte, SByte);
impl_array_target!(i16, "epicsInt16", Int16, Int16);
impl_array_target!(u16, "epicsUInt16", UInt16, UInt16);
impl_array_target!(i32, "epicsInt32", Int32, Int32);
impl_array_target!(u32, "epicsUInt32", UInt32, UInt32);
impl_array_target!(i64, "epicsInt64", Int64, Int64);
impl_array_target!(u64, "epicsUInt64", UInt64, UInt64);
impl_array_target!(f32, "epicsFloat32", Float, Float);
impl_array_target!(f64, "epicsFloat64", Double, Double);

pub fn read_array<T: ArrayTarget>(v: &Variant) -> Result<Vec<T>> {
    let array = as_array(v)?;
    if array.value_type != T::ELEMENT {
        return Err(ConvError::ArrayTypeMismatch {
            incoming: type_name(v),
            target: T::NAME,
        });
    }
    Ok(array.values.iter().filter_map(T::from_element).collect())
}

/// The `epicsUInt8` array: a Byte or Boolean array, or a scalar ByteString.
pub fn read_array_u8(v: &Variant) -> Result<Vec<u8>> {
    if let Variant::ByteString(bs) = v {
        return Ok(bs.as_ref().to_vec());
    }
    let array = as_array(v)?;
    match array.value_type {
        VariantScalarTypeId::Byte => Ok(array
            .values
            .iter()
            .filter_map(|e| match e {
                Variant::Byte(b) => Some(*b),
                _ => None,
            })
            .collect()),
        VariantScalarTypeId::Boolean => Ok(array
            .values
            .iter()
            .filter_map(|e| match e {
                Variant::Boolean(b) => Some(u8::from(*b)),
                _ => None,
            })
            .collect()),
        _ => Err(ConvError::ArrayTypeMismatch {
            incoming: type_name(v),
            target: "epicsUInt8",
        }),
    }
}

/// String arrays (`readArray(char*, len, num, ...)`,
/// `DataElementOpen62541Leaf.cpp:487-600`): String, XmlElement, LocalizedText,
/// QualifiedName and ByteString (hex-encoded) all read as text.
pub fn read_string_array(v: &Variant) -> Result<Vec<String>> {
    let array = as_array(v)?;
    array
        .values
        .iter()
        .map(|e| match e {
            Variant::String(s) => Ok(s.as_ref().to_string()),
            Variant::XmlElement(x) => Ok(x.to_string()),
            Variant::LocalizedText(lt) => Ok(lt.text.as_ref().to_string()),
            Variant::QualifiedName(qn) => Ok(qn.name.as_ref().to_string()),
            Variant::ByteString(bs) => Ok(hex_encode(bs.as_ref())),
            other => Err(ConvError::ArrayTypeMismatch {
                incoming: type_name(other),
                target: "epicsString",
            }),
        })
        .collect()
}

fn as_array(v: &Variant) -> Result<&Array> {
    match v {
        Variant::Array(a) => Ok(a),
        _ => Err(ConvError::NotAnArray),
    }
}

/// Build the outgoing variant for a numeric record value (`writeScalar<ET>`,
/// `DataElementOpen62541Leaf.h:961-1120`).
///
/// `incoming` is the last value received from the server: its type is the type
/// the value is written back as, so a record never needs to name the node's OPC
/// UA type.
pub fn write_scalar<T: ScalarTarget>(
    value: T,
    incoming: &Variant,
    choices: Option<&EnumChoices>,
) -> Result<Variant> {
    let n = value.to_number();
    let out_of_range = || ConvError::OutOfRange {
        value: value.to_string(),
        target: "the node's type",
    };

    // An enumeration only accepts one of its choices, whatever its width.
    if let Some(choices) = choices {
        if !is_choice(choices, n) {
            return Err(ConvError::NotAnEnumChoice {
                value: n.to_string(),
            });
        }
        return Ok(Variant::Int32(
            i32::from_number(n).ok_or_else(out_of_range)?,
        ));
    }

    Ok(match incoming {
        Variant::Boolean(_) => Variant::Boolean(!matches!(
            n,
            Number::Int(0) | Number::Uint(0) | Number::Float(0.0)
        )),
        Variant::SByte(_) => Variant::SByte(i8::try_from_number(n).ok_or_else(out_of_range)?),
        Variant::Byte(_) => Variant::Byte(u8::try_from_number(n).ok_or_else(out_of_range)?),
        Variant::Int16(_) => Variant::Int16(i16::try_from_number(n).ok_or_else(out_of_range)?),
        Variant::UInt16(_) => Variant::UInt16(u16::try_from_number(n).ok_or_else(out_of_range)?),
        Variant::Int32(_) => Variant::Int32(i32::from_number(n).ok_or_else(out_of_range)?),
        Variant::UInt32(_) => Variant::UInt32(u32::from_number(n).ok_or_else(out_of_range)?),
        Variant::Int64(_) => Variant::Int64(i64::from_number(n).ok_or_else(out_of_range)?),
        Variant::UInt64(_) => Variant::UInt64(u64::try_from_number(n).ok_or_else(out_of_range)?),
        Variant::Float(_) => Variant::Float(f32_from_number(n).ok_or_else(out_of_range)?),
        Variant::Double(_) => Variant::Double(f64::from_number(n).ok_or_else(out_of_range)?),
        Variant::String(_) => Variant::String(UAString::from(value.to_string())),
        Variant::LocalizedText(lt) => Variant::LocalizedText(Box::new(LocalizedText {
            // The locale is not the record's to change.
            locale: lt.locale.clone(),
            text: UAString::from(value.to_string()),
        })),
        other => {
            return Err(ConvError::Unsupported {
                from: T::NAME.to_string(),
                to: type_name(other),
            });
        }
    })
}

/// Build the outgoing variant for a string record value
/// (`writeScalar(const char*, len)`, `DataElementOpen62541Leaf.cpp:857-1110`).
///
/// Upstream C defect fixed here: the DOUBLE arm sets `outgoingData` but never
/// calls `markAsDirty()` nor clears the error return
/// (`DataElementOpen62541Leaf.cpp:1066-1073`, compare the FLOAT arm at
/// `:1056-1064`), so writing a string record to a Double node always raised
/// `WRITE_ALARM`/`INVALID` with "value out of range" and never sent anything.
pub fn write_string(
    value: &str,
    incoming: &Variant,
    choices: Option<&EnumChoices>,
) -> Result<Variant> {
    let out_of_range = || ConvError::OutOfRange {
        value: format!("\"{value}\""),
        target: "the node's type",
    };

    if let Some(choices) = choices {
        // Choice names are tried before numbers, so a choice whose name looks
        // like a number still means the choice (`DataElementOpen62541Leaf.cpp:1013-1016`).
        if let Some((v, _)) = choices.iter().find(|(_, name)| name.as_str() == value) {
            return Ok(Variant::Int32(
                i32::try_from(*v).map_err(|_| out_of_range())?,
            ));
        }
        let n = parse_int_base0(value).ok_or_else(out_of_range)?;
        let v = u32::try_from(n).ok().filter(|v| choices.contains_key(v));
        return match v {
            Some(v) => Ok(Variant::Int32(
                i32::try_from(v).map_err(|_| out_of_range())?,
            )),
            None => Err(ConvError::NotAnEnumChoice {
                value: value.to_string(),
            }),
        };
    }

    let int = || parse_int_base0(value).ok_or_else(out_of_range);
    let float = || value.trim().parse::<f64>().map_err(|_| out_of_range());

    Ok(match incoming {
        Variant::String(_) => Variant::String(UAString::from(value)),
        Variant::XmlElement(_) => Variant::XmlElement(XmlElement::from(value)),
        // "locale|text" sets the locale too; without the separator the node's
        // locale is kept (`DataElementOpen62541Leaf.cpp:891-905`).
        Variant::LocalizedText(lt) => {
            let (locale, text) = match value.split_once('|') {
                Some((locale, text)) => (UAString::from(locale), text),
                None => (lt.locale.clone(), value),
            };
            Variant::LocalizedText(Box::new(LocalizedText {
                locale,
                text: UAString::from(text),
            }))
        }
        // "namespace|name", likewise (`:907-923`).
        Variant::QualifiedName(qn) => {
            let (namespace_index, name) = match value.split_once('|') {
                Some((ns, name)) => (ns.trim().parse().unwrap_or(0), name),
                None => (qn.namespace_index, value),
            };
            Variant::QualifiedName(Box::new(QualifiedName {
                namespace_index,
                name: UAString::from(name),
            }))
        }
        Variant::ByteString(_) => Variant::ByteString(ByteString::from(
            hex_decode(value).ok_or_else(out_of_range)?,
        )),
        Variant::Boolean(_) => Variant::Boolean(value.starts_with(['Y', 'y', 'T', 't', '1'])),
        Variant::SByte(_) => Variant::SByte(i8::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::Byte(_) => Variant::Byte(u8::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::Int16(_) => Variant::Int16(i16::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::UInt16(_) => Variant::UInt16(u16::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::Int32(_) => Variant::Int32(i32::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::UInt32(_) => Variant::UInt32(u32::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::Int64(_) => Variant::Int64(i64::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::UInt64(_) => Variant::UInt64(u64::try_from(int()?).map_err(|_| out_of_range())?),
        Variant::Float(_) => {
            Variant::Float(f32_from_number(Number::Float(float()?)).ok_or_else(out_of_range)?)
        }
        Variant::Double(_) => Variant::Double(float()?),
        other => {
            return Err(ConvError::Unsupported {
                from: "CString".to_string(),
                to: type_name(other),
            });
        }
    })
}

/// Build the outgoing variant for an array record value (`writeArray<ET>`,
/// `DataElementOpen62541Leaf.h:1131-1163`): the node must already be an array of
/// exactly this element type.
pub fn write_array<T: ArrayTarget>(values: &[T], incoming: &Variant) -> Result<Variant> {
    let array = match incoming {
        Variant::Array(a) => a,
        _ => return Err(ConvError::NotAnArray),
    };
    if array.value_type != T::ELEMENT {
        return Err(ConvError::ArrayTypeMismatch {
            incoming: type_name(incoming),
            target: T::NAME,
        });
    }
    let elements: Vec<Variant> = values.iter().map(T::to_element).collect();
    Ok(Variant::Array(Box::new(
        Array::new(T::ELEMENT, elements).expect("elements are of the array's type"),
    )))
}

/// The `epicsUInt8` array on the way out: a Byte array, a Boolean array, or a
/// scalar ByteString node.
pub fn write_array_u8(values: &[u8], incoming: &Variant) -> Result<Variant> {
    match incoming {
        Variant::ByteString(_) => Ok(Variant::ByteString(ByteString::from(values.to_vec()))),
        Variant::Array(a) => {
            let elements: Vec<Variant> = match a.value_type {
                VariantScalarTypeId::Byte => values.iter().map(|b| Variant::Byte(*b)).collect(),
                VariantScalarTypeId::Boolean => {
                    values.iter().map(|b| Variant::Boolean(*b != 0)).collect()
                }
                _ => {
                    return Err(ConvError::ArrayTypeMismatch {
                        incoming: type_name(incoming),
                        target: "epicsUInt8",
                    });
                }
            };
            Ok(Variant::Array(Box::new(
                Array::new(a.value_type, elements).expect("elements are of the array's type"),
            )))
        }
        _ => Err(ConvError::NotAnArray),
    }
}

/// String arrays on the way out (`writeArray(const char*, len, num, ...)`).
pub fn write_string_array(values: &[String], incoming: &Variant) -> Result<Variant> {
    let array = match incoming {
        Variant::Array(a) => a,
        _ => return Err(ConvError::NotAnArray),
    };
    let elements: Vec<Variant> = match array.value_type {
        VariantScalarTypeId::String => values
            .iter()
            .map(|s| Variant::String(UAString::from(s.as_str())))
            .collect(),
        VariantScalarTypeId::XmlElement => values
            .iter()
            .map(|s| Variant::XmlElement(XmlElement::from(s.as_str())))
            .collect(),
        VariantScalarTypeId::LocalizedText => values
            .iter()
            .map(|s| {
                Variant::LocalizedText(Box::new(LocalizedText {
                    locale: UAString::null(),
                    text: UAString::from(s.as_str()),
                }))
            })
            .collect(),
        VariantScalarTypeId::ByteString => values
            .iter()
            .map(|s| {
                hex_decode(s)
                    .map(|b| Variant::ByteString(ByteString::from(b)))
                    .ok_or_else(|| ConvError::OutOfRange {
                        value: format!("\"{s}\""),
                        target: "ByteString",
                    })
            })
            .collect::<Result<_>>()?,
        _ => {
            return Err(ConvError::ArrayTypeMismatch {
                incoming: type_name(incoming),
                target: "epicsString",
            });
        }
    };
    Ok(Variant::Array(Box::new(
        Array::new(array.value_type, elements).expect("elements are of the array's type"),
    )))
}

/// The narrower integer targets, needed only on the write side (the node may be
/// a Byte or Int16 that no EPICS record type maps to directly).
trait NarrowTarget: Sized {
    fn try_from_number(n: Number) -> Option<Self>;
}

macro_rules! impl_narrow {
    ($t:ty, $lo:expr, $hi_exclusive:expr) => {
        impl NarrowTarget for $t {
            fn try_from_number(n: Number) -> Option<Self> {
                match n {
                    Number::Int(v) => <$t>::try_from(v).ok(),
                    Number::Uint(v) => <$t>::try_from(v).ok(),
                    Number::Float(v) => {
                        float_to_int(v, $lo, $hi_exclusive).and_then(|v| <$t>::try_from(v).ok())
                    }
                }
            }
        }
    };
}

impl_narrow!(i8, -128.0, 128.0);
impl_narrow!(u8, 0.0, 256.0);
impl_narrow!(i16, -32768.0, 32768.0);
impl_narrow!(u16, 0.0, 65536.0);
impl_narrow!(u64, 0.0, 18446744073709551616.0);

fn f32_from_number(n: Number) -> Option<f32> {
    let v = match n {
        Number::Int(v) => v as f64,
        Number::Uint(v) => v as f64,
        Number::Float(v) => v,
    };
    // `isWithinRange<UA_Float>` rejects a double outside the float range.
    if v.is_finite() && (v < f64::from(f32::MIN) || v > f64::from(f32::MAX)) {
        return None;
    }
    Some(v as f32)
}
