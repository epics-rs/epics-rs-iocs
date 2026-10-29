//! Decode a wire `epics:nt/NTNDArray:1.0` value into an `NDArray`.
//!
//! Mirrors ADCore's `NTNDArrayConverter` (`ntndArrayConverter.cpp`), the
//! reverse direction of what `epics_pva_rs::nt::nd_array` builds. Field
//! order/shape is `nt_nd_array_desc()`'s (pvxs `nt.cpp:196-251`).
//!
//! Parity notes (see `ntndArrayConverter.cpp`):
//! - `toValue`: uncompressed arrays take their `NDDataType` from the
//!   selected `value` union variant (`getValueType`); compressed arrays
//!   (`codec.name` non-empty) instead read the *original* (pre-compression)
//!   type from `codec.parameters` (an `int`) and store the selected
//!   variant's raw bytes verbatim as `UInt8` bytes — `codec.parameters`
//!   never advertises `level`/`shuffle`/`compressor`, so `Codec` fields
//!   beyond `original_data_type` are always `0` on decode (they play no
//!   role in decompression and this converter never reads them).
//! - `toTimeStamp`/`toDataTimeStamp` is a crossed mapping: wire `timeStamp`
//!   becomes `NDArray::timestamp` (`EpicsTimestamp`), and wire
//!   `dataTimeStamp` becomes `NDArray::time_stamp` (`f64`) — not the other
//!   way around.
//! - `toAttributes`: a `Boolean`-valued attribute is silently dropped
//!   (matches the C++ `toAttribute` dispatch's `default` case); a
//!   null/non-scalar `value` still produces an attribute, typed
//!   `NDAttrValue::Undefined`. Wire `tags`/`alarm`/per-attribute
//!   `timeStamp` are read on the wire schema but never used here, matching
//!   upstream.
//! - `getInfo`'s x/y/color dimension derivation (keyed off a "ColorMode"
//!   NDAttribute) is not reimplemented here — this decoder just carries any
//!   wire "ColorMode" attribute through generically, and callers use
//!   `NDArray::info()`, which already implements that exact table.

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::codec::{Codec, CodecName};
use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::timestamp::{EPICS_EPOCH_OFFSET, EpicsTimestamp};
use epics_rs::pva::pvdata::TypedScalarArray;
use epics_rs::pva::{PvField, PvStructure, ScalarValue};

#[derive(Debug, Clone, PartialEq)]
pub enum ConvertError {
    MissingField(&'static str),
    WrongFieldType(&'static str),
    NoUnionFieldSelected,
    UnsupportedValueType(String),
    UnrecognizedCodec(String),
    InvalidDataType(i64),
    /// The driver's pool refused the frame (pvaDriver.cpp:270-281).
    PoolExhausted,
}

impl std::fmt::Display for ConvertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingField(name) => write!(f, "missing NTNDArray field `{name}`"),
            Self::WrongFieldType(name) => write!(f, "unexpected wire type for field `{name}`"),
            Self::NoUnionFieldSelected => write!(f, "no value union field selected"),
            Self::UnsupportedValueType(name) => {
                write!(f, "unsupported value union variant `{name}`")
            }
            Self::UnrecognizedCodec(name) => write!(f, "unrecognized codec name `{name}`"),
            Self::InvalidDataType(v) => {
                write!(f, "invalid codec.parameters data type ordinal {v}")
            }
            Self::PoolExhausted => write!(f, "the NDArray pool refused the frame"),
        }
    }
}

impl std::error::Error for ConvertError {}

fn field<'a>(s: &'a PvStructure, name: &'static str) -> Result<&'a PvField, ConvertError> {
    s.get_field(name).ok_or(ConvertError::MissingField(name))
}

fn substructure<'a>(
    s: &'a PvStructure,
    name: &'static str,
) -> Result<&'a PvStructure, ConvertError> {
    match field(s, name)? {
        PvField::Structure(inner) => Ok(inner),
        _ => Err(ConvertError::WrongFieldType(name)),
    }
}

fn scalar_int(s: &PvStructure, name: &'static str) -> Result<i64, ConvertError> {
    match field(s, name)? {
        PvField::Scalar(ScalarValue::Int(v)) => Ok(*v as i64),
        PvField::Scalar(ScalarValue::Long(v)) => Ok(*v),
        _ => Err(ConvertError::WrongFieldType(name)),
    }
}

fn scalar_bool(s: &PvStructure, name: &'static str) -> Result<bool, ConvertError> {
    match field(s, name)? {
        PvField::Scalar(ScalarValue::Boolean(v)) => Ok(*v),
        _ => Err(ConvertError::WrongFieldType(name)),
    }
}

fn scalar_string(s: &PvStructure, name: &'static str) -> Result<String, ConvertError> {
    match field(s, name)? {
        PvField::Scalar(ScalarValue::String(v)) => Ok(v.to_string()),
        _ => Err(ConvertError::WrongFieldType(name)),
    }
}

fn read_time_t(s: &PvStructure, name: &'static str) -> Result<(i64, i32), ConvertError> {
    let t = substructure(s, name)?;
    let seconds_past_epoch = scalar_int(t, "secondsPastEpoch")?;
    let nanoseconds = scalar_int(t, "nanoseconds")? as i32;
    Ok((seconds_past_epoch, nanoseconds))
}

/// The `value` union's array, read in place on the wire structure until
/// [`WireValue::copy_into`] writes it into the frame: C views it and memcpys it
/// into `pData` once (`NTNDArrayConverter::toValue`,
/// ntndArrayConverter.cpp:242-243).
struct WireValue<'a> {
    data_type: NDDataType,
    len: usize,
    copy: fn(&PvField, &mut [u8]),
    array: &'a PvField,
}

impl WireValue<'_> {
    /// The array's native-endian bytes into `out`, which holds exactly them.
    fn copy_into(&self, out: &mut [u8]) {
        (self.copy)(self.array, out)
    }
}

/// Matches `NTNDArrayConverter::getValueType`'s variant-name-to-`NDDataType`
/// table. `booleanValue` (the one variant with no `NDDataType` counterpart)
/// errors, mirroring `getValueType` throwing on `pvBoolean`.
fn wire_value<'a>(variant_name: &str, array: &'a PvField) -> Result<WireValue<'a>, ConvertError> {
    macro_rules! typed {
        ($ndtype:expr, $ty:ty, $typed_variant:ident, $scalar_variant:ident) => {{
            let len = match array {
                PvField::ScalarArrayTyped(TypedScalarArray::$typed_variant(v)) => v.len(),
                PvField::ScalarArray(items) => {
                    if !items
                        .iter()
                        .all(|item| matches!(item, ScalarValue::$scalar_variant(_)))
                    {
                        return Err(ConvertError::WrongFieldType("value"));
                    }
                    items.len()
                }
                _ => return Err(ConvertError::WrongFieldType("value")),
            };
            let copy: fn(&PvField, &mut [u8]) = |array, out| {
                let out = out.as_chunks_mut::<{ size_of::<$ty>() }>().0;
                match array {
                    PvField::ScalarArrayTyped(TypedScalarArray::$typed_variant(v)) => {
                        for (dst, x) in out.iter_mut().zip(v.iter()) {
                            *dst = x.to_ne_bytes();
                        }
                    }
                    PvField::ScalarArray(items) => {
                        for (dst, item) in out.iter_mut().zip(items) {
                            if let ScalarValue::$scalar_variant(x) = item {
                                *dst = x.to_ne_bytes();
                            }
                        }
                    }
                    _ => unreachable!("wire_value checked the array"),
                }
            };
            Ok(WireValue {
                data_type: $ndtype,
                len,
                copy,
                array,
            })
        }};
    }
    match variant_name {
        "byteValue" => typed!(NDDataType::Int8, i8, Byte, Byte),
        "ubyteValue" => typed!(NDDataType::UInt8, u8, UByte, UByte),
        "shortValue" => typed!(NDDataType::Int16, i16, Short, Short),
        "ushortValue" => typed!(NDDataType::UInt16, u16, UShort, UShort),
        "intValue" => typed!(NDDataType::Int32, i32, Int, Int),
        "uintValue" => typed!(NDDataType::UInt32, u32, UInt, UInt),
        "longValue" => typed!(NDDataType::Int64, i64, Long, Long),
        "ulongValue" => typed!(NDDataType::UInt64, u64, ULong, ULong),
        "floatValue" => typed!(NDDataType::Float32, f32, Float, Float),
        "doubleValue" => typed!(NDDataType::Float64, f64, Double, Double),
        other => Err(ConvertError::UnsupportedValueType(other.to_string())),
    }
}

fn codec_name_from_str(name: &str) -> Result<CodecName, ConvertError> {
    match name {
        "" => Ok(CodecName::None),
        "jpeg" => Ok(CodecName::JPEG),
        "zlib" => Ok(CodecName::Zlib),
        "lz4" => Ok(CodecName::LZ4),
        "blosc" => Ok(CodecName::Blosc),
        "bslz4" => Ok(CodecName::BSLZ4),
        "lz4hdf5" => Ok(CodecName::LZ4HDF5),
        other => Err(ConvertError::UnrecognizedCodec(other.to_string())),
    }
}

/// Read the pre-compression `NDDataType` out of `codec.parameters` (an
/// `int`-carrying `any`), matching `getInfo`'s "else read int from
/// codec.parameters" branch.
fn codec_parameters_dtype(codec: &PvStructure) -> Result<NDDataType, ConvertError> {
    let variant = match field(codec, "parameters")? {
        PvField::Variant(v) => v,
        _ => return Err(ConvertError::WrongFieldType("codec.parameters")),
    };
    let ordinal = match &variant.value {
        PvField::Scalar(ScalarValue::Int(v)) => *v as i64,
        _ => return Err(ConvertError::WrongFieldType("codec.parameters")),
    };
    u8::try_from(ordinal)
        .ok()
        .and_then(NDDataType::from_ordinal)
        .ok_or(ConvertError::InvalidDataType(ordinal))
}

fn convert_dimension(wire: &PvStructure) -> Result<NDDimension, ConvertError> {
    Ok(NDDimension {
        size: scalar_int(wire, "size")? as usize,
        offset: scalar_int(wire, "offset")? as usize,
        binning: scalar_int(wire, "binning")? as usize,
        reverse: scalar_bool(wire, "reverse")?,
    })
}

fn scalar_value_to_ndattr(v: &ScalarValue) -> Option<NDAttrValue> {
    match v {
        // Matches `toAttributes`' `default:` case (pvBoolean falls through
        // unhandled) — the attribute is dropped, not added as `Undefined`.
        ScalarValue::Boolean(_) => None,
        ScalarValue::Byte(x) => Some(NDAttrValue::Int8(*x)),
        ScalarValue::UByte(x) => Some(NDAttrValue::UInt8(*x)),
        ScalarValue::Short(x) => Some(NDAttrValue::Int16(*x)),
        ScalarValue::UShort(x) => Some(NDAttrValue::UInt16(*x)),
        ScalarValue::Int(x) => Some(NDAttrValue::Int32(*x)),
        ScalarValue::UInt(x) => Some(NDAttrValue::UInt32(*x)),
        ScalarValue::Long(x) => Some(NDAttrValue::Int64(*x)),
        ScalarValue::ULong(x) => Some(NDAttrValue::UInt64(*x)),
        ScalarValue::Float(x) => Some(NDAttrValue::Float32(*x)),
        ScalarValue::Double(x) => Some(NDAttrValue::Float64(*x)),
        ScalarValue::String(s) => Some(NDAttrValue::String(s.to_string())),
    }
}

/// Convert one wire `epics:nt/NTAttribute:1.0` element. Returns `Ok(None)`
/// for a `Boolean`-valued attribute, which C++ silently drops rather than
/// adding.
fn convert_attribute(wire: &PvStructure) -> Result<Option<NDAttribute>, ConvertError> {
    let name = scalar_string(wire, "name")?;
    let descriptor = scalar_string(wire, "descriptor")?;
    let source_type = scalar_int(wire, "sourceType")?;
    let source_str = scalar_string(wire, "source")?;
    let source = match source_type {
        0 => NDAttrSource::Driver,
        1 => NDAttrSource::Param {
            port_name: String::new(),
            param_name: source_str,
        },
        2 => NDAttrSource::EpicsPV(source_str),
        3 => NDAttrSource::Function(source_str),
        4 => NDAttrSource::Constant(source_str),
        _ => NDAttrSource::Undefined,
    };

    let variant = match field(wire, "value")? {
        PvField::Variant(v) => v,
        _ => return Err(ConvertError::WrongFieldType("attribute.value")),
    };
    let value = match &variant.value {
        PvField::Scalar(sv) => match scalar_value_to_ndattr(sv) {
            Some(v) => v,
            None => return Ok(None),
        },
        // Null / structure / array / union `any` value: still added, typed
        // `Undefined` (`toUndefinedAttribute`).
        _ => NDAttrValue::Undefined,
    };
    Ok(Some(NDAttribute::new_static(
        name, descriptor, source, value,
    )))
}

/// Decode a wire `epics:nt/NTNDArray:1.0` structure (as delivered by
/// `PvaClient::pvmonitor_handle`'s callback) into a pool-owned `NDArray`.
pub fn decode_nt_nd_array(pool: &DriverPool, value: &PvField) -> Result<DriverArray, ConvertError> {
    let s = match value {
        PvField::Structure(s) => s,
        _ => return Err(ConvertError::WrongFieldType("<top>")),
    };

    let (selector, variant_name, array_value) = match field(s, "value")? {
        PvField::Union {
            selector,
            variant_name,
            value,
        } => (*selector, variant_name.as_str(), value.as_ref()),
        _ => return Err(ConvertError::WrongFieldType("value")),
    };
    if selector < 0 {
        return Err(ConvertError::NoUnionFieldSelected);
    }
    let value = wire_value(variant_name, array_value)?;

    let codec_s = substructure(s, "codec")?;
    let codec_name_str = scalar_string(codec_s, "name")?;
    let (data_type, codec) = if codec_name_str.is_empty() {
        (value.data_type, None)
    } else {
        // A compressed frame holds the array's bytes.
        let original_data_type = codec_parameters_dtype(codec_s)?;
        let name = codec_name_from_str(&codec_name_str)?;
        let compressed_size = scalar_int(s, "compressedSize")? as usize;
        (
            NDDataType::UInt8,
            Some(Codec {
                name,
                compressed_size,
                level: 0,
                shuffle: 0,
                compressor: 0,
                original_data_type,
            }),
        )
    };

    let dims = match field(s, "dimension")? {
        PvField::StructureArray(items) => items
            .iter()
            .map(|opt| {
                opt.as_ref()
                    .ok_or(ConvertError::MissingField("dimension[]"))
                    .and_then(convert_dimension)
            })
            .collect::<Result<Vec<_>, _>>()?,
        _ => return Err(ConvertError::WrongFieldType("dimension")),
    };

    let mut attributes = NDAttributeList::new();
    match field(s, "attribute")? {
        PvField::StructureArray(items) => {
            for opt in items.iter().flatten() {
                if let Some(attr) = convert_attribute(opt)? {
                    attributes.add(attr);
                }
            }
        }
        _ => return Err(ConvertError::WrongFieldType("attribute")),
    }

    let unique_id = scalar_int(s, "uniqueId")? as i32;

    // Crossed mapping (matches `toTimeStamp`/`toDataTimeStamp`): wire
    // `timeStamp` -> `NDArray::timestamp`, wire `dataTimeStamp` ->
    // `NDArray::time_stamp`.
    let (ts_sec, ts_nsec) = read_time_t(s, "timeStamp")?;
    let timestamp = EpicsTimestamp {
        sec: (ts_sec - EPICS_EPOCH_OFFSET as i64) as u32,
        nsec: ts_nsec as u32,
    };
    let (dts_sec, dts_nsec) = read_time_t(s, "dataTimeStamp")?;
    let time_stamp = (dts_sec as f64 + dts_nsec as f64 * 1e-9) - EPICS_EPOCH_OFFSET as f64;

    // C's alloc of `totalBytes` under the dims (pvaDriver.cpp:270), which a
    // compressed frame's dims do not size.
    let total_bytes = value.len * value.data_type.element_size();
    let mut array = pool
        .alloc_sized(dims, data_type, total_bytes)
        .map_err(|_| ConvertError::PoolExhausted)?;
    let mut frame = array.fill();
    value.copy_into(frame.data_mut().as_u8_slice_mut());
    frame.set_unique_id(unique_id);
    frame.set_timestamp(timestamp);
    frame.set_time_stamp(time_stamp);
    *frame.attributes_mut() = attributes;
    frame.set_codec(codec);
    Ok(array)
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::color::NDColorMode;
    use epics_rs::ad_core::ndarray::NDDataView;
    use epics_rs::pva::nt::nd_array::{
        NdArrayBuffer, NdAttribute as WireAttribute, NdCodec, NdDimension as WireDimension,
        NdTimeStamp, NtNdArray, nt_nd_array_value,
    };

    fn base_nt(value: NdArrayBuffer, dims: Vec<WireDimension>) -> NtNdArray {
        NtNdArray {
            value,
            codec: NdCodec::default(),
            compressed_size: 0,
            uncompressed_size: 0,
            unique_id: 42,
            data_time_stamp: NdTimeStamp {
                seconds_past_epoch: EPICS_EPOCH_OFFSET as i64 + 1_000,
                nanoseconds: 250_000_000,
                user_tag: 0,
            },
            alarm: Default::default(),
            time_stamp: NdTimeStamp {
                seconds_past_epoch: EPICS_EPOCH_OFFSET as i64 + 2_000,
                nanoseconds: 500_000_000,
                user_tag: 0,
            },
            dimension: dims,
            attribute: Vec::new(),
        }
    }

    fn decode(nt: &NtNdArray) -> DriverArray {
        let value = nt_nd_array_value(nt);
        decode_nt_nd_array(&DriverPool::standalone(0), &value)
            .unwrap_or_else(|e| panic!("decode failed: {e}"))
    }

    #[test]
    fn decodes_2d_mono_uint8_shape_and_dtype() {
        let nt = base_nt(
            NdArrayBuffer::UByte((0..12).collect()),
            vec![
                WireDimension {
                    size: 4,
                    ..Default::default()
                },
                WireDimension {
                    size: 3,
                    ..Default::default()
                },
            ],
        );
        let arr = decode(&nt);

        assert_eq!(arr.dims.len(), 2);
        assert_eq!(arr.dims[0].size, 4);
        assert_eq!(arr.dims[1].size, 3);
        match arr.data().view() {
            NDDataView::U8(v) => assert_eq!(v.len(), 12),
            other => panic!("expected U8 buffer, got {other:?}"),
        }
        assert!(arr.codec.is_none());
        assert_eq!(arr.unique_id, 42);
    }

    #[test]
    fn timestamp_and_data_timestamp_are_not_swapped() {
        // The single highest-risk-of-inversion detail: wire `timeStamp` ->
        // `NDArray::timestamp` (EpicsTimestamp), wire `dataTimeStamp` ->
        // `NDArray::time_stamp` (f64) — not the other way around.
        let nt = base_nt(NdArrayBuffer::UByte(vec![0].into()), vec![]);
        let arr = decode(&nt);

        assert_eq!(arr.timestamp.sec, 2_000);
        assert_eq!(arr.timestamp.nsec, 500_000_000);
        assert!((arr.time_stamp - 1_000.25).abs() < 1e-9);
    }

    /// Each variant's elements land in a frame of its type, in native byte
    /// order (ntndArrayConverter.cpp:243).
    #[test]
    fn decodes_all_ten_numeric_value_variants() {
        macro_rules! case {
            ($variant:ident, $nd:expr, $values:expr) => {{
                let values = $values;
                let bytes: Vec<u8> = values.iter().flat_map(|x| x.to_ne_bytes()).collect();
                (NdArrayBuffer::$variant(values.into()), $nd, bytes)
            }};
        }
        let cases: Vec<(NdArrayBuffer, NDDataType, Vec<u8>)> = vec![
            case!(Byte, NDDataType::Int8, vec![-1i8, 2]),
            case!(UByte, NDDataType::UInt8, vec![1u8, 2]),
            case!(Short, NDDataType::Int16, vec![-1i16, 2]),
            case!(UShort, NDDataType::UInt16, vec![1u16, 2]),
            case!(Int, NDDataType::Int32, vec![-1i32, 2]),
            case!(UInt, NDDataType::UInt32, vec![1u32, 2]),
            case!(Long, NDDataType::Int64, vec![-1i64, 2]),
            case!(ULong, NDDataType::UInt64, vec![1u64, 2]),
            case!(Float, NDDataType::Float32, vec![1.5f32, 2.5]),
            case!(Double, NDDataType::Float64, vec![1.5f64, 2.5]),
        ];
        for (buf, expected, bytes) in cases {
            let nt = base_nt(
                buf,
                vec![WireDimension {
                    size: 2,
                    ..Default::default()
                }],
            );
            let arr = decode(&nt);
            assert_eq!(arr.data().data_type(), expected);
            assert_eq!(arr.data().as_u8_slice(), &bytes[..], "{expected:?}");
        }
    }

    /// The untyped wire array (one `ScalarValue` per element) decodes like
    /// the typed one, and a stray element type is refused.
    #[test]
    fn an_untyped_value_array_decodes_its_elements() {
        let nt = base_nt(
            NdArrayBuffer::UShort(vec![0, 0].into()),
            vec![WireDimension {
                size: 2,
                ..Default::default()
            }],
        );
        let with_items = |items: Vec<ScalarValue>| {
            let mut wire = nt_nd_array_value(&nt);
            let PvField::Structure(s) = &mut wire else {
                panic!("NTNDArray is a structure");
            };
            let Some(PvField::Union { value, .. }) = s.get_field_mut("value") else {
                panic!("NTNDArray value is a union");
            };
            **value = PvField::ScalarArray(items);
            decode_nt_nd_array(&DriverPool::standalone(0), &wire)
        };
        let arr = with_items(vec![ScalarValue::UShort(7), ScalarValue::UShort(900)]).unwrap();
        assert!(matches!(arr.data().view(), NDDataView::U16(v) if v == [7, 900]));
        assert!(matches!(
            with_items(vec![ScalarValue::UShort(7), ScalarValue::Int(9)]),
            Err(ConvertError::WrongFieldType("value"))
        ));
    }

    /// pvaDriver.cpp:270-280 allocates with a NULL pData, so a frame over
    /// the pool's maxMemory is refused.
    #[test]
    fn a_frame_over_the_pool_limit_is_refused() {
        let nt = base_nt(
            NdArrayBuffer::UByte((0..12).collect()),
            vec![WireDimension {
                size: 12,
                ..Default::default()
            }],
        );
        let value = nt_nd_array_value(&nt);
        let err = decode_nt_nd_array(&DriverPool::standalone(8), &value)
            .err()
            .expect("decode error");
        assert!(matches!(err, ConvertError::PoolExhausted));
    }

    #[test]
    fn rejects_boolean_value_variant() {
        let nt = base_nt(NdArrayBuffer::Boolean(vec![true, false].into()), vec![]);
        let value = nt_nd_array_value(&nt);
        let err = decode_nt_nd_array(&DriverPool::standalone(0), &value)
            .err()
            .expect("decode error");
        assert!(matches!(err, ConvertError::UnsupportedValueType(_)));
    }

    #[test]
    fn compressed_array_stores_raw_bytes_and_original_dtype_from_codec_parameters() {
        use epics_rs::pva::pvdata::VariantValue;

        let mut nt = base_nt(
            NdArrayBuffer::UByte(vec![0xAA, 0xBB, 0xCC, 0xDD].into()),
            vec![WireDimension {
                size: 2,
                ..Default::default()
            }],
        );
        nt.codec = NdCodec {
            name: "lz4".into(),
            // Original (pre-compression) dtype: NDDataType::UInt16 == ordinal 3.
            parameters: Some(VariantValue::scalar(ScalarValue::Int(3))),
        };
        nt.compressed_size = 4;
        let arr = decode(&nt);

        match arr.data().view() {
            NDDataView::U8(v) => assert_eq!(v, &[0xAA, 0xBB, 0xCC, 0xDD]),
            other => panic!("expected raw U8 buffer, got {other:?}"),
        }
        let codec = arr
            .codec
            .as_ref()
            .expect("codec must be Some for compressed array");
        assert_eq!(codec.name, CodecName::LZ4);
        assert_eq!(codec.compressed_size, 4);
        assert_eq!(codec.original_data_type, NDDataType::UInt16);
        assert_eq!(codec.level, 0);
        assert_eq!(codec.shuffle, 0);
        assert_eq!(codec.compressor, 0);
    }

    #[test]
    fn rejects_unrecognized_codec_name() {
        let mut nt = base_nt(NdArrayBuffer::UByte(vec![1].into()), vec![]);
        nt.codec = NdCodec {
            name: "made-up-codec".into(),
            parameters: Some(epics_rs::pva::pvdata::VariantValue::scalar(
                ScalarValue::Int(1),
            )),
        };
        let value = nt_nd_array_value(&nt);
        let err = decode_nt_nd_array(&DriverPool::standalone(0), &value)
            .err()
            .expect("decode error");
        assert!(matches!(err, ConvertError::UnrecognizedCodec(_)));
    }

    #[test]
    fn color_mode_attribute_passes_through_to_ndarray_info() {
        // NDArray::info()'s x/y/color-dim table (reused rather than
        // reimplemented here) is keyed off a "ColorMode" NDAttribute. RGB1 =
        // color-interleaved: dim[0]=color, dim[1]=x, dim[2]=y.
        let mut nt = base_nt(
            NdArrayBuffer::UByte(vec![0; 3 * 4 * 5].into()),
            vec![
                WireDimension {
                    size: 3,
                    ..Default::default()
                },
                WireDimension {
                    size: 4,
                    ..Default::default()
                },
                WireDimension {
                    size: 5,
                    ..Default::default()
                },
            ],
        );
        nt.attribute = vec![WireAttribute::scalar(
            "ColorMode",
            ScalarValue::Int(NDColorMode::RGB1 as i32),
        )];
        let arr = decode(&nt);

        assert_eq!(
            arr.attributes.get("ColorMode").unwrap().value.as_i64(),
            Some(2)
        );
        let info = arr.info();
        assert_eq!(info.color_dim, 0);
        assert_eq!(info.x_dim, 1);
        assert_eq!(info.y_dim, 2);
        assert_eq!(info.color_size, 3);
        assert_eq!(info.x_size, 4);
        assert_eq!(info.y_size, 5);
    }

    #[test]
    fn two_d_mono_array_size_z_quirk_preserved_via_info() {
        // Upstream quirk (preserved, not fixed): a 2-D Mono array leaves
        // color_dim at its 0-default (same as x_dim), so `NDArraySizeZ`
        // (`dims[info.color.dim].size`) reads the X size, not 0.
        let nt = base_nt(
            NdArrayBuffer::UByte(vec![0; 6].into()),
            vec![
                WireDimension {
                    size: 3,
                    ..Default::default()
                },
                WireDimension {
                    size: 2,
                    ..Default::default()
                },
            ],
        );
        let arr = decode(&nt);
        let info = arr.info();

        assert_eq!(info.color_size, 0);
        assert_eq!(info.color_dim, info.x_dim);
        assert_eq!(arr.dims[info.color_dim].size, 3);
    }

    #[test]
    fn boolean_valued_attribute_is_silently_dropped() {
        let mut nt = base_nt(NdArrayBuffer::UByte(vec![0].into()), vec![]);
        nt.attribute = vec![
            WireAttribute::scalar("Kept", ScalarValue::Int(7)),
            WireAttribute::scalar("DroppedBool", ScalarValue::Boolean(true)),
        ];
        let arr = decode(&nt);

        assert!(arr.attributes.get("Kept").is_some());
        assert!(arr.attributes.get("DroppedBool").is_none());
    }

    #[test]
    fn null_valued_attribute_becomes_undefined_typed() {
        let mut nt = base_nt(NdArrayBuffer::UByte(vec![0].into()), vec![]);
        nt.attribute = vec![WireAttribute {
            name: "NullAttr".into(),
            ..Default::default()
        }];
        let arr = decode(&nt);

        let attr = arr
            .attributes
            .get("NullAttr")
            .expect("attribute must exist");
        assert_eq!(attr.value, NDAttrValue::Undefined);
    }

    #[test]
    fn attribute_source_type_maps_to_ndattr_source() {
        let mut nt = base_nt(NdArrayBuffer::UByte(vec![0].into()), vec![]);
        nt.attribute = vec![WireAttribute {
            name: "ParamAttr".into(),
            source_type: 1,
            source: "MY_PARAM".into(),
            value: epics_rs::pva::pvdata::VariantValue::scalar(ScalarValue::Int(1)),
            ..Default::default()
        }];
        let arr = decode(&nt);

        let attr = arr.attributes.get("ParamAttr").unwrap();
        assert_eq!(attr.source.source_string(), "MY_PARAM");
        assert!(matches!(attr.source, NDAttrSource::Param { .. }));
    }

    #[test]
    fn rejects_unselected_value_union() {
        let nt = base_nt(NdArrayBuffer::UByte(vec![0].into()), vec![]);
        let mut value = nt_nd_array_value(&nt);
        if let PvField::Structure(s) = &mut value
            && let Some(v) = s.get_field_mut("value")
        {
            *v = PvField::Union {
                selector: -1,
                variant_name: String::new(),
                value: Box::new(PvField::Null),
            };
        }
        let err = decode_nt_nd_array(&DriverPool::standalone(0), &value)
            .err()
            .expect("decode error");
        assert_eq!(err, ConvertError::NoUnionFieldSelected);
    }
}
