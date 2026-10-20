//! Boundary tests for the Variant ↔ EPICS conversions.

use std::str::FromStr;

use async_opcua::types::{
    Array, ByteString, ByteStringBody, DataValue, DateTime, DiagnosticInfo, ExpandedNodeId,
    ExtensionObject, Guid, LocalizedText, NodeId, QualifiedName, StatusCode, UAString, Variant,
    VariantScalarTypeId, XmlBody, XmlElement,
};
use opcua::value::{
    EnumChoices, ReadError, WriteError, hex_decode, hex_encode, read_array, read_array_u8,
    read_scalar, read_string, read_string_array, write_array, write_array_u8, write_scalar,
    write_string, write_string_array,
};

fn choices() -> EnumChoices {
    EnumChoices::from([
        (0, "Off".to_string()),
        (1, "On".to_string()),
        (7, "Fault".to_string()),
    ])
}

fn array(value_type: VariantScalarTypeId, values: Vec<Variant>) -> Variant {
    Variant::Array(Box::new(Array::new(value_type, values).expect("array")))
}

// ------------------------------------------------------------- scalar reads

#[test]
fn every_numeric_kind_reads_into_int32() {
    assert_eq!(read_scalar::<i32>(&Variant::Boolean(true), None), Ok(1));
    assert_eq!(read_scalar::<i32>(&Variant::Boolean(false), None), Ok(0));
    assert_eq!(read_scalar::<i32>(&Variant::SByte(-5), None), Ok(-5));
    assert_eq!(read_scalar::<i32>(&Variant::Byte(200), None), Ok(200));
    assert_eq!(read_scalar::<i32>(&Variant::Int16(-300), None), Ok(-300));
    assert_eq!(read_scalar::<i32>(&Variant::UInt16(60000), None), Ok(60000));
    assert_eq!(read_scalar::<i32>(&Variant::Int32(-7), None), Ok(-7));
    assert_eq!(read_scalar::<i32>(&Variant::UInt32(7), None), Ok(7));
    assert_eq!(read_scalar::<i32>(&Variant::Int64(-8), None), Ok(-8));
    assert_eq!(read_scalar::<i32>(&Variant::UInt64(8), None), Ok(8));
    assert_eq!(read_scalar::<i32>(&Variant::Float(1.9), None), Ok(1));
    assert_eq!(read_scalar::<i32>(&Variant::Double(-1.9), None), Ok(-1));
}

#[test]
fn an_integer_that_does_not_fit_the_record_is_out_of_range() {
    assert!(matches!(
        read_scalar::<i32>(&Variant::Int64(i64::from(i32::MAX) + 1), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    assert!(matches!(
        read_scalar::<i32>(&Variant::UInt32(u32::MAX), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    assert!(matches!(
        read_scalar::<u32>(&Variant::Int32(-1), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    assert!(matches!(
        read_scalar::<i64>(&Variant::UInt64(u64::MAX), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    // The widest value each target *does* hold.
    assert_eq!(
        read_scalar::<i32>(&Variant::Int64(i64::from(i32::MAX)), None),
        Ok(i32::MAX)
    );
    assert_eq!(
        read_scalar::<u32>(&Variant::UInt64(u64::from(u32::MAX)), None),
        Ok(u32::MAX)
    );
    assert_eq!(
        read_scalar::<i64>(&Variant::UInt64(i64::MAX as u64), None),
        Ok(i64::MAX)
    );
}

#[test]
fn a_float_at_the_integer_boundary() {
    assert_eq!(
        read_scalar::<i32>(&Variant::Double(2147483647.4), None),
        Ok(i32::MAX)
    );
    assert!(matches!(
        read_scalar::<i32>(&Variant::Double(2147483648.0), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    assert_eq!(
        read_scalar::<i32>(&Variant::Double(-2147483648.0), None),
        Ok(i32::MIN)
    );
    assert!(matches!(
        read_scalar::<i32>(&Variant::Double(-2147483649.0), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    // 2^63 is what `INT64_MAX` rounds to as a double, so the C's `value > max`
    // check lets it through and then casts it — undefined behaviour.
    assert!(matches!(
        read_scalar::<i64>(&Variant::Double(9223372036854775808.0), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    // The largest double below 2^63 does convert.
    assert_eq!(
        read_scalar::<i64>(&Variant::Double(9223372036854774784.0), None),
        Ok(9223372036854774784)
    );
}

#[test]
fn a_non_finite_float_never_becomes_an_integer() {
    // The C's `!(v < lowest || v > max)` is true for NaN, so the cast that
    // follows is undefined behaviour.
    for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            matches!(
                read_scalar::<i32>(&Variant::Double(v), None),
                Err(ReadError::OutOfBounds { .. })
            ),
            "{v}"
        );
        assert!(matches!(
            read_scalar::<i64>(&Variant::Double(v), None),
            Err(ReadError::OutOfBounds { .. })
        ));
        assert!(matches!(
            read_scalar::<u32>(&Variant::Double(v), None),
            Err(ReadError::OutOfBounds { .. })
        ));
    }
    // ... but a float record takes them unchanged.
    assert!(
        read_scalar::<f64>(&Variant::Double(f64::NAN), None)
            .expect("nan reads")
            .is_nan()
    );
}

#[test]
fn a_string_node_parses_into_a_numeric_record() {
    let s = |t: &str| Variant::String(UAString::from(t));
    assert_eq!(read_scalar::<i32>(&s("42"), None), Ok(42));
    assert_eq!(read_scalar::<i32>(&s("-42"), None), Ok(-42));
    assert_eq!(read_scalar::<i32>(&s("0x10"), None), Ok(16));
    assert_eq!(read_scalar::<i32>(&s("010"), None), Ok(8));
    assert_eq!(read_scalar::<f64>(&s("1.5e3"), None), Ok(1500.0));
    assert!(matches!(
        read_scalar::<i32>(&s("nonsense"), None),
        Err(ReadError::OutOfBounds { .. })
    ));
    // The C's `string_to(epicsInt32)` assigns a `long` to an `epicsInt32`
    // without a range check (`DataElementOpen62541Leaf.h:150-157`), silently
    // truncating; only its `epicsUInt32` overload checks.
    assert!(matches!(
        read_scalar::<i32>(&s("99999999999"), None),
        Err(ReadError::OutOfBounds { .. })
    ));

    let lt = Variant::LocalizedText(Box::new(LocalizedText::new("en", "17")));
    assert_eq!(read_scalar::<i32>(&lt, None), Ok(17));
}

/// `string_to` is `std::stoll(s, 0, 0)` (`DataElementOpen62541Leaf.h:150-177`):
/// glibc's prefix read, C23 `0b` under g++, C `isspace` only, the tail
/// ignored; no conversion and `ERANGE` throw.
#[test]
fn a_string_node_is_read_as_stoll() {
    let s = |t: &str| Variant::String(UAString::from(t));
    assert_eq!(read_scalar::<i32>(&s("12abc"), None), Ok(12));
    assert_eq!(read_scalar::<u32>(&s("0b101"), None), Ok(5));
    assert!(read_scalar::<i32>(&s("\u{a0}7"), None).is_err());
    assert!(read_scalar::<i32>(&s("+-5"), None).is_err());
    assert!(read_scalar::<u32>(&s("-1"), None).is_err());
    assert!(read_scalar::<i64>(&s("99999999999999999999"), None).is_err());
}

#[test]
fn a_type_with_no_numeric_meaning_is_unsupported() {
    let qn = Variant::QualifiedName(Box::new(QualifiedName::new(2, "Name")));
    assert!(matches!(
        read_scalar::<i32>(&qn, None),
        Err(ReadError::UnsupportedKind { .. })
    ));
    assert!(matches!(
        read_scalar::<i32>(&Variant::Empty, None),
        Err(ReadError::UnsupportedKind { .. })
    ));
}

#[test]
fn an_enum_value_must_be_one_of_its_choices() {
    let c = choices();
    assert_eq!(read_scalar::<i32>(&Variant::Int32(7), Some(&c)), Ok(7));
    assert!(matches!(
        read_scalar::<i32>(&Variant::Int32(3), Some(&c)),
        Err(ReadError::OutOfBounds { .. })
    ));
    // Without the type's choices, any in-range value passes.
    assert_eq!(read_scalar::<i32>(&Variant::Int32(3), None), Ok(3));
}

// ------------------------------------------------------------- string reads

#[test]
fn strings_localized_text_and_qualified_names_read_as_their_text() {
    assert_eq!(
        read_string(&Variant::String(UAString::from("text")), None),
        Ok("text".to_string())
    );
    assert_eq!(
        read_string(
            &Variant::LocalizedText(Box::new(LocalizedText::new("de", "Wert"))),
            None
        ),
        Ok("Wert".to_string())
    );
    assert_eq!(
        read_string(
            &Variant::QualifiedName(Box::new(QualifiedName::new(2, "Name"))),
            None
        ),
        Ok("Name".to_string())
    );
}

#[test]
fn a_byte_string_reads_as_upper_case_hex() {
    assert_eq!(
        read_string(
            &Variant::ByteString(ByteString::from(vec![0x0a, 0xff])),
            None
        ),
        Ok("0AFF".to_string())
    );
}

#[test]
fn an_enum_reads_as_its_choice_name_and_falls_back_to_the_number() {
    let c = choices();
    assert_eq!(
        read_string(&Variant::Int32(7), Some(&c)),
        Ok("Fault".to_string())
    );
    assert_eq!(
        read_string(&Variant::Int32(3), Some(&c)),
        Ok("3".to_string())
    );
}

#[test]
fn numbers_and_booleans_read_as_text() {
    assert_eq!(read_string(&Variant::Int32(-5), None), Ok("-5".to_string()));
    assert_eq!(
        read_string(&Variant::Double(1.5), None),
        Ok("1.500000".to_string())
    );
    assert_eq!(
        read_string(&Variant::Boolean(true), None),
        Ok("true".to_string())
    );
}

// ------------------------------------------------------ UA_print's text
//
// Every expected string below is what open62541 1.2.0-29's `UA_print` printed
// for the same value, from the amalgamation the C module builds with.

fn print(v: Variant) -> String {
    read_string(&v, None).expect("printable")
}

/// A variant inside a variant, which `UA_print` shows with its type.
fn nested(v: Variant) -> Variant {
    Variant::Variant(Box::new(v))
}

fn guid() -> Guid {
    Guid::from_str("12345678-9abc-def0-0123-456789abcdef").unwrap()
}

/// `readScalar(char*)` hands a kind it has no text of its own for to
/// `UA_print` (`DataElementOpen62541Leaf.cpp:422-426`).
#[test]
fn a_number_reads_as_ua_print_renders_it() {
    assert_eq!(print(Variant::Float(1.5)), "1.500000");
    assert_eq!(print(Variant::Float(0.1)), "0.100000");
    assert_eq!(print(Variant::Float(-0.0)), "-0.000000");
    assert_eq!(print(Variant::Float(1e-7)), "0.000000");
    assert_eq!(print(Variant::Float(123456.79)), "123456.789062");
    assert_eq!(print(Variant::Float(f32::NAN)), "nan");
    assert_eq!(print(Variant::Float(-f32::NAN)), "-nan");
    assert_eq!(print(Variant::Float(f32::INFINITY)), "inf");
    assert_eq!(print(Variant::Float(f32::NEG_INFINITY)), "-inf");
    // `printFloat` gives snprintf 32 bytes of its 64 and `printDouble` 64, so
    // C cuts these to "1000000015047466219876688855040" and 63 digits.
    assert_eq!(
        print(Variant::Float(1e30)),
        "1000000015047466219876688855040.000000"
    );
    assert_eq!(print(Variant::Double(1e300)).len(), 308);
    assert_eq!(print(Variant::Double(0.1)), "0.100000");
    assert_eq!(print(Variant::Double(-0.0)), "-0.000000");
    assert_eq!(print(Variant::Double(2.5e-10)), "0.000000");
    assert_eq!(print(Variant::Double(1e20)), "100000000000000000000.000000");
    assert_eq!(print(Variant::Double(std::f64::consts::PI)), "3.141593");
    assert_eq!(print(Variant::Double(0.0000015)), "0.000002");
    // Exact ties round to even.
    assert_eq!(print(Variant::Double(0.0078125)), "0.007812");
    assert_eq!(print(Variant::Double(0.0234375)), "0.023438");
    assert_eq!(print(Variant::Double(2.0000005)), "2.000001");
    assert_eq!(print(Variant::Double(1e-320)), "0.000000");
    assert_eq!(print(Variant::Double(-f64::NAN)), "-nan");
    assert_eq!(print(Variant::Double(f64::NEG_INFINITY)), "-inf");
    assert_eq!(print(Variant::Boolean(false)), "false");
    assert_eq!(print(Variant::Int16(-5)), "-5");
    assert_eq!(print(Variant::UInt16(65535)), "65535");
    assert_eq!(print(Variant::UInt32(4294967295)), "4294967295");
    assert_eq!(print(Variant::Int64(i64::MIN)), "-9223372036854775808");
    assert_eq!(print(Variant::UInt64(u64::MAX)), "18446744073709551615");
}

#[test]
fn an_identifier_reads_as_ua_print_renders_it() {
    let status = |code: u32| print(Variant::StatusCode(StatusCode::from(code)));
    assert_eq!(status(0), "Good");
    assert_eq!(status(0x800A_0000), "BadTimeout");
    assert_eq!(status(0x80AB_0000), "BadInvalidArgument");
    assert_eq!(status(0x00A6_0000), "GoodMoreData");

    assert_eq!(
        print(Variant::Guid(Box::new(guid()))),
        "12345678-9abc-def0-0123-456789abcdef"
    );

    let node = |id: NodeId| print(Variant::NodeId(Box::new(id)));
    assert_eq!(node(NodeId::new(0, 85u32)), "i=85");
    assert_eq!(node(NodeId::new(1, 4294967295u32)), "ns=1;i=4294967295");
    assert_eq!(node(NodeId::new(2, "foo bar;x%y")), "ns=2;s=foo bar;x%y");
    assert_eq!(
        node(NodeId::new(65535, guid())),
        "ns=65535;g=12345678-9abc-def0-0123-456789abcdef"
    );
    assert_eq!(
        node(NodeId::new(4, ByteString::from(vec![0, 1, 2, 250, 255]))),
        "ns=4;b=AAEC+v8="
    );
    assert_eq!(node(NodeId::new(0, "")), "s=");

    let expanded = |id: NodeId, uri: &str, server_index: u32| {
        print(Variant::ExpandedNodeId(Box::new(ExpandedNodeId {
            node_id: id,
            namespace_uri: if uri.is_empty() {
                UAString::null()
            } else {
                UAString::from(uri)
            },
            server_index,
        })))
    };
    let s = || NodeId::new(2, "foo bar;x%y");
    assert_eq!(expanded(s(), "", 0), "ns=2;s=foo bar;x%y");
    assert_eq!(expanded(s(), "", 3), "svr=3;ns=2;s=foo bar;x%y");
    assert_eq!(
        expanded(s(), "urn:a;b%c", 3),
        "svr=3;nsu=urn:a;b%c;s=foo bar;x%y"
    );
    assert_eq!(expanded(s(), "urn:a;b%c", 0), "nsu=urn:a;b%c;s=foo bar;x%y");
    assert_eq!(
        expanded(NodeId::new(0, 85u32), "urn:a;b%c", 0),
        "nsu=urn:a;b%c;i=85"
    );
}

/// The C adds the zone offset before `UA_print` (`:393-400`), so the time is
/// local and labelled with its offset.
#[test]
fn a_date_time_reads_as_local_time_with_its_offset() {
    let dt = DateTime::ymd_hms_nano(2026, 10, 20, 1, 2, 3, 456_789_100);
    let offset = chrono::Local::now().offset().local_minus_utc();
    let local = dt.as_chrono() + chrono::Duration::seconds(offset.into());
    let label = format!(
        "{}{:02}{:02}",
        if offset < 0 { '-' } else { '+' },
        offset.abs() / 3600,
        offset.abs() / 60 % 60
    );
    assert_eq!(
        print(Variant::DateTime(Box::new(dt))),
        format!("{} (UTC{label})", local.format("%Y-%m-%d %H:%M:%S%.3f"))
    );
}

/// The kinds `UA_print` shows as a structure, inside a variant as a node of
/// data type BaseDataType carries them.
#[test]
fn a_nested_value_reads_as_ua_print_renders_it() {
    assert_eq!(
        print(nested(Variant::Double(1.5))),
        "{\n\tDataType: Double,\n\tValue: 1.500000\n}"
    );
    assert_eq!(
        print(nested(Variant::Float(1.5))),
        "{\n\tDataType: Float,\n\tValue: 1.500000\n}"
    );
    assert_eq!(print(nested(Variant::Empty)), "NullVariant");
    let ints = || {
        Array::new(
            VariantScalarTypeId::Int32,
            vec![Variant::Int32(1), Variant::Int32(-2), Variant::Int32(3)],
        )
        .unwrap()
    };
    assert_eq!(
        print(nested(Variant::Array(Box::new(ints())))),
        "{\n\tDataType: Int32,\n\tValue: Array(3, Int32) {\n\t\t0: 1,\n\t\t1: -2,\n\t\t2: 3\n\t}\n}"
    );
    let mut dims = ints();
    dims.dimensions = Some(vec![3]);
    assert_eq!(
        print(nested(Variant::Array(Box::new(dims)))),
        "{\n\tDataType: Int32,\n\tValue: Array(3, Int32) {\n\t\t0: 1,\n\t\t1: -2,\n\t\t2: 3\n\t},\n\tArrayDimensions: Array(1, UInt32) {\n\t\t0: 3\n\t}\n}"
    );
    assert_eq!(
        print(nested(array(VariantScalarTypeId::String, vec![]))),
        "{\n\tDataType: String,\n\tValue: Array(0, String) {\n\t}\n}"
    );
    assert_eq!(
        print(nested(array(
            VariantScalarTypeId::String,
            vec![Variant::from("a"), Variant::from("b")]
        ))),
        "{\n\tDataType: String,\n\tValue: Array(2, String) {\n\t\t0: \"a\",\n\t\t1: \"b\"\n\t}\n}"
    );
    assert_eq!(
        print(nested(array(
            VariantScalarTypeId::LocalizedText,
            vec![
                Variant::LocalizedText(Box::new(LocalizedText::new("en", "a"))),
                Variant::LocalizedText(Box::new(LocalizedText::new("de", "b"))),
            ]
        ))),
        "{\n\tDataType: LocalizedText,\n\tValue: Array(2, LocalizedText) {\n\t\t0: {\n\t\t\tLocale: \"en\",\n\t\t\tText: \"a\"\n\t\t},\n\t\t1: {\n\t\t\tLocale: \"de\",\n\t\t\tText: \"b\"\n\t\t}\n\t}\n}"
    );
    assert_eq!(
        print(nested(array(
            VariantScalarTypeId::Variant,
            vec![nested(Variant::Int32(-7)), nested(Variant::Empty)]
        ))),
        "{\n\tDataType: Variant,\n\tValue: Array(2, Variant) {\n\t\t0: {\n\t\t\tDataType: Int32,\n\t\t\tValue: -7\n\t\t},\n\t\t1: NullVariant\n\t}\n}"
    );
    assert_eq!(
        print(nested(nested(Variant::from("he\"llo")))),
        "{\n\tDataType: Variant,\n\tValue: {\n\t\tDataType: String,\n\t\tValue: \"he\"llo\"\n\t}\n}"
    );
    assert_eq!(
        print(nested(Variant::QualifiedName(Box::new(
            QualifiedName::new(3, "nm")
        )))),
        "{\n\tDataType: QualifiedName,\n\tValue: {\n\t\tNamespaceIndex: 3,\n\t\tName: \"nm\"\n\t}\n}"
    );
    assert_eq!(
        print(nested(Variant::LocalizedText(Box::new(LocalizedText {
            locale: UAString::null(),
            text: UAString::null(),
        })))),
        "{\n\tDataType: LocalizedText,\n\tValue: {\n\t\tLocale: NullString,\n\t\tText: NullString\n\t}\n}"
    );
    // Inside a structure a ByteString is base64, not the hex of a record read.
    assert_eq!(
        print(nested(Variant::ByteString(ByteString::from(vec![
            0, 1, 2, 250, 255
        ])))),
        "{\n\tDataType: ByteString,\n\tValue: \"AAEC+v8=\"\n}"
    );
    assert_eq!(
        print(nested(Variant::ByteString(ByteString::null()))),
        "{\n\tDataType: ByteString,\n\tValue: NullByteString\n}"
    );
    assert_eq!(
        print(nested(Variant::String(UAString::null()))),
        "{\n\tDataType: String,\n\tValue: NullString\n}"
    );
    assert_eq!(
        print(nested(Variant::from(""))),
        "{\n\tDataType: String,\n\tValue: \"\"\n}"
    );
    assert_eq!(
        print(nested(Variant::XmlElement(XmlElement::from("he\"llo")))),
        "{\n\tDataType: XmlElement,\n\tValue: \"he\"llo\"\n}"
    );
    assert_eq!(
        print(nested(Variant::Guid(Box::new(guid())))),
        "{\n\tDataType: Guid,\n\tValue: 12345678-9abc-def0-0123-456789abcdef\n}"
    );
    assert_eq!(
        print(nested(Variant::StatusCode(StatusCode::BadTimeout))),
        "{\n\tDataType: StatusCode,\n\tValue: BadTimeout\n}"
    );
}

#[test]
fn a_data_value_and_a_diagnostic_info_read_as_ua_print_renders_them() {
    assert_eq!(print(Variant::DataValue(Box::default())), "{ }");
    assert_eq!(
        print(Variant::DataValue(Box::new(DataValue {
            status: Some(StatusCode::BadTimeout),
            ..Default::default()
        }))),
        "{\n\tStatus: BadTimeout\n}"
    );
    assert_eq!(
        print(Variant::DataValue(Box::new(DataValue {
            value: Some(Variant::from("he\"llo")),
            status: Some(StatusCode::BadTimeout),
            source_picoseconds: Some(7),
            ..Default::default()
        }))),
        "{\n\tValue: {\n\t\tDataType: String,\n\t\tValue: \"he\"llo\"\n\t},\n\tStatus: BadTimeout,\n\tSourcePicoseconds: 7\n}"
    );
    assert_eq!(print(Variant::DiagnosticInfo(Box::default())), "{ }");
    assert_eq!(
        print(Variant::DiagnosticInfo(Box::new(DiagnosticInfo {
            symbolic_id: Some(1),
            namespace_uri: Some(2),
            localized_text: Some(3),
            locale: Some(-1),
            additional_info: Some(UAString::from("more")),
            inner_status_code: Some(StatusCode::BadTimeout),
            inner_diagnostic_info: Some(Box::new(DiagnosticInfo {
                symbolic_id: Some(4),
                ..Default::default()
            })),
        }))),
        "{\n\tSymbolicId: 1,\n\tNamespaceUri: 2,\n\tLocalizedText: 3,\n\tLocale: -1,\n\tAdditionalInfo: \"more\",\n\tInnerStatusCode: BadTimeout,\n\tInnerDiagnosticInfo: {\n\t\tSymbolicId: 4\n\t}\n}"
    );
    assert_eq!(
        print(nested(Variant::DiagnosticInfo(Box::new(DiagnosticInfo {
            locale: Some(2),
            inner_status_code: Some(StatusCode::Good),
            ..Default::default()
        })))),
        "{\n\tDataType: DiagnosticInfo,\n\tValue: {\n\t\tLocale: 2,\n\t\tInnerStatusCode: Good\n\t}\n}"
    );
}

/// A structure of a type the session has no decoder for keeps its encoded
/// body. `printExtensionObject` closes that block with `addName(ctx, "}")`,
/// so C prints "}: "; the stray ": " is not copied.
#[test]
fn an_undecoded_structure_reads_as_ua_print_renders_it() {
    assert_eq!(
        print(Variant::ExtensionObject(ExtensionObject::null())),
        "ExtensionObject(No Body)"
    );
    let binary = ExtensionObject::new(ByteStringBody::new(
        ByteString::from(vec![0, 1, 2, 250, 255]),
        NodeId::new(2, 5004u32),
    ));
    assert_eq!(
        print(Variant::ExtensionObject(binary.clone())),
        "ExtensionObject(Binary Encoded) {\n\tDataType: ns=2;i=5004,\n\tBody: \"AAEC+v8=\"\n}"
    );
    assert_eq!(
        print(nested(Variant::ExtensionObject(binary))),
        "{\n\tDataType: ExtensionObject,\n\tValue: ExtensionObject(Binary Encoded) {\n\t\tDataType: ns=2;i=5004,\n\t\tBody: \"AAEC+v8=\"\n\t}\n}"
    );
    let xml = ExtensionObject::new(XmlBody::new(
        b"he\"llo".to_vec(),
        NodeId::new(2, 5005u32),
        "X".to_string(),
    ));
    assert_eq!(
        print(nested(Variant::ExtensionObject(xml))),
        "{\n\tDataType: ExtensionObject,\n\tValue: ExtensionObject(XML Encoded) {\n\t\tDataType: ns=2;i=5005,\n\t\tBody: \"he\"llo\"\n\t}\n}"
    );
    assert_eq!(
        print(nested(Variant::ExtensionObject(ExtensionObject::null()))),
        "{\n\tDataType: ExtensionObject,\n\tValue: ExtensionObject(No Body)\n}"
    );
}

/// The out-of-bounds line prints the value with `UA_print` too
/// (`DataElementOpen62541Leaf.h:830-837`).
#[test]
fn the_out_of_bounds_line_prints_the_value_as_ua_print_renders_it() {
    fn lines<T: std::fmt::Debug>(r: Result<T, ReadError>) -> Vec<String> {
        r.unwrap_err().lines()
    }
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::Double(1e10), None)),
        ["incoming data (Double 10000000000.000000) out-of-bounds for epicsInt32"]
    );
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::from("abc"), None)),
        ["incoming data (String \"abc\") out-of-bounds for epicsInt32"]
    );
    assert_eq!(
        lines(read_scalar::<i32>(
            &Variant::LocalizedText(Box::new(LocalizedText::new("de", "x"))),
            None
        )),
        [
            "incoming data (LocalizedText {\n\tLocale: \"de\",\n\tText: \"x\"\n}) out-of-bounds for epicsInt32"
        ]
    );
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::Guid(Box::new(guid())), None)),
        [
            "unsupported type kind Guid for incoming data",
            "incoming data (Guid 12345678-9abc-def0-0123-456789abcdef) out-of-bounds for epicsInt32"
        ]
    );
}

#[test]
fn hex_round_trip() {
    assert_eq!(hex_encode(&[0x00, 0x12, 0xab]), "0012AB");
    assert_eq!(hex_decode("0012AB"), Some(vec![0x00, 0x12, 0xab]));
    assert_eq!(hex_decode("ab"), Some(vec![0xab]));
    // A single odd digit is allowed only when it opens a group, because
    // otherwise the byte boundary is ambiguous (12|3 or 1|23?).
    assert_eq!(hex_decode("1 23"), Some(vec![0x01, 0x23]));
    assert_eq!(hex_decode("123"), None);
    assert_eq!(hex_decode(""), Some(vec![]));
    assert_eq!(hex_decode("xy"), None);
}

// -------------------------------------------------------------------- arrays

#[test]
fn a_numeric_array_must_match_the_records_ftvl_exactly() {
    let i32s = array(
        VariantScalarTypeId::Int32,
        vec![Variant::Int32(1), Variant::Int32(2)],
    );
    assert_eq!(read_array::<i32>(&i32s), Ok(vec![1, 2]));
    assert!(matches!(
        read_array::<i16>(&i32s),
        Err(ReadError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        read_array::<f64>(&i32s),
        Err(ReadError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        read_array::<i32>(&Variant::Int32(1)),
        Err(ReadError::NotAnArray)
    ));
}

#[test]
fn a_uint8_array_also_takes_booleans_and_a_byte_string() {
    let bytes = array(
        VariantScalarTypeId::Byte,
        vec![Variant::Byte(1), Variant::Byte(255)],
    );
    assert_eq!(read_array_u8(&bytes), Ok(vec![1, 255]));

    let bools = array(
        VariantScalarTypeId::Boolean,
        vec![Variant::Boolean(true), Variant::Boolean(false)],
    );
    assert_eq!(read_array_u8(&bools), Ok(vec![1, 0]));

    let bs = Variant::ByteString(ByteString::from(vec![9, 8, 7]));
    assert_eq!(read_array_u8(&bs), Ok(vec![9, 8, 7]));

    let i16s = array(VariantScalarTypeId::Int16, vec![Variant::Int16(1)]);
    assert!(matches!(
        read_array_u8(&i16s),
        Err(ReadError::ArrayTypeMismatch { .. })
    ));
}

#[test]
fn a_string_array_reads_every_text_like_element_type() {
    let strings = array(
        VariantScalarTypeId::String,
        vec![
            Variant::String(UAString::from("a")),
            Variant::String(UAString::from("b")),
        ],
    );
    assert_eq!(
        read_string_array(&strings),
        Ok(vec!["a".to_string(), "b".to_string()])
    );

    let texts = array(
        VariantScalarTypeId::LocalizedText,
        vec![Variant::LocalizedText(Box::new(LocalizedText::new(
            "en", "hello",
        )))],
    );
    assert_eq!(read_string_array(&texts), Ok(vec!["hello".to_string()]));
}

// -------------------------------------------------------------------- writes

#[test]
fn the_outgoing_value_takes_the_type_of_the_node() {
    assert_eq!(
        write_scalar(300i32, &Variant::Int16(0), None),
        Ok(Variant::Int16(300))
    );
    assert_eq!(
        write_scalar(1i32, &Variant::Boolean(false), None),
        Ok(Variant::Boolean(true))
    );
    assert_eq!(
        write_scalar(0i32, &Variant::Boolean(true), None),
        Ok(Variant::Boolean(false))
    );
    assert_eq!(
        write_scalar(1.5f64, &Variant::Float(0.0), None),
        Ok(Variant::Float(1.5))
    );
    assert_eq!(
        write_scalar(7i32, &Variant::String(UAString::null()), None),
        Ok(Variant::String(UAString::from("7")))
    );
    // The node's locale is not the record's to change.
    assert_eq!(
        write_scalar(
            7i32,
            &Variant::LocalizedText(Box::new(LocalizedText::new("de", "alt"))),
            None
        ),
        Ok(Variant::LocalizedText(Box::new(LocalizedText::new(
            "de", "7"
        ))))
    );
}

#[test]
fn an_outgoing_value_that_does_not_fit_the_node_is_out_of_range() {
    assert!(matches!(
        write_scalar(300i32, &Variant::Byte(0), None),
        Err(WriteError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(-1i32, &Variant::UInt32(0), None),
        Err(WriteError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(1e300f64, &Variant::Float(0.0), None),
        Err(WriteError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(1i32, &Variant::Empty, None),
        Err(WriteError::Unsupported { .. })
    ));
}

#[test]
fn an_outgoing_enum_value_must_be_one_of_the_choices() {
    let c = choices();
    assert_eq!(
        write_scalar(7i32, &Variant::Int32(0), Some(&c)),
        Ok(Variant::Int32(7))
    );
    assert!(matches!(
        write_scalar(3i32, &Variant::Int32(0), Some(&c)),
        Err(WriteError::OutOfRange { .. })
    ));
}

#[test]
fn a_string_record_writes_to_a_double_node() {
    // The C's DOUBLE arm sets `outgoingData` but never marks the element dirty
    // nor clears its error return (`DataElementOpen62541Leaf.cpp:1066-1073`), so
    // this write always failed with "value out of range" and never reached the
    // server.
    assert_eq!(
        write_string("1.5", &Variant::Double(0.0), None),
        Ok(Variant::Double(1.5))
    );
    assert!(matches!(
        write_string("nonsense", &Variant::Double(0.0), None),
        Err(WriteError::OutOfRange { .. })
    ));
}

#[test]
fn a_string_record_writes_every_scalar_node_type() {
    assert_eq!(
        write_string("0x10", &Variant::Int32(0), None),
        Ok(Variant::Int32(16))
    );
    assert_eq!(
        write_string("Yes", &Variant::Boolean(false), None),
        Ok(Variant::Boolean(true))
    );
    assert_eq!(
        write_string("no", &Variant::Boolean(true), None),
        Ok(Variant::Boolean(false))
    );
    assert_eq!(
        write_string("0AFF", &Variant::ByteString(ByteString::null()), None),
        Ok(Variant::ByteString(ByteString::from(vec![0x0a, 0xff])))
    );
    assert!(matches!(
        write_string("300", &Variant::Byte(0), None),
        Err(WriteError::OutOfRange { .. })
    ));
}

#[test]
fn a_string_record_can_set_the_locale_and_the_namespace() {
    let node = Variant::LocalizedText(Box::new(LocalizedText::new("de", "alt")));
    assert_eq!(
        write_string("en|new", &node, None),
        Ok(Variant::LocalizedText(Box::new(LocalizedText::new(
            "en", "new"
        ))))
    );
    // Without the separator the node keeps its locale.
    assert_eq!(
        write_string("new", &node, None),
        Ok(Variant::LocalizedText(Box::new(LocalizedText::new(
            "de", "new"
        ))))
    );

    let node = Variant::QualifiedName(Box::new(QualifiedName::new(2, "old")));
    assert_eq!(
        write_string("5|new", &node, None),
        Ok(Variant::QualifiedName(Box::new(QualifiedName::new(
            5, "new"
        ))))
    );
    assert_eq!(
        write_string("new", &node, None),
        Ok(Variant::QualifiedName(Box::new(QualifiedName::new(
            2, "new"
        ))))
    );
}

/// The namespace is C's `atoi` of the whole value, kept to 16 bits
/// (`DataElementOpen62541Leaf.cpp:930-931`).
#[test]
fn a_namespace_is_read_as_atoi_reads_it() {
    let node = Variant::QualifiedName(Box::new(QualifiedName::new(2, "old")));
    let namespace = |value: &str| match write_string(value, &node, None) {
        Ok(Variant::QualifiedName(qn)) => qn.namespace_index,
        other => panic!("{other:?}"),
    };
    assert_eq!(namespace(" 3abc|n"), 3);
    assert_eq!(namespace("+4|n"), 4);
    assert_eq!(namespace("x|n"), 0);
    assert_eq!(namespace("-1|n"), 65535);
    assert_eq!(namespace("70000|n"), 4464);
    // glibc's `(int) strtol`: `\v` is skipped, and the `long` saturates
    // before it is truncated.
    assert_eq!(namespace("\x0b5|n"), 5);
    use epics_rs::base::types::c_parse::long_to_int;
    use std::ffi::c_long;
    let wrapped = if c_long::BITS == 64 { 1 } else { u16::MAX };
    assert_eq!(namespace("4294967297|n"), wrapped);
    assert_eq!(
        namespace("99999999999999999999|n"),
        long_to_int(c_long::MAX) as u16
    );
    assert_eq!(
        namespace("-99999999999999999999|n"),
        long_to_int(c_long::MIN) as u16
    );
}

/// Each element may name its locale or namespace; one that does not takes
/// the node's element at its index, else the element before it
/// (`DataElementOpen62541Leaf.cpp:1167-1203`).
#[test]
fn a_string_array_sets_each_elements_locale_and_namespace() {
    let lt = |locale: &str, text: &str| {
        Variant::LocalizedText(Box::new(LocalizedText::new(locale, text)))
    };
    let node = array(
        VariantScalarTypeId::LocalizedText,
        vec![lt("de", "a"), lt("fr", "b")],
    );
    let values = ["en|x", "y", "z"].map(String::from);
    assert_eq!(
        write_string_array(&values, &node),
        Ok(array(
            VariantScalarTypeId::LocalizedText,
            vec![lt("en", "x"), lt("fr", "y"), lt("fr", "z")]
        ))
    );
    let empty = array(VariantScalarTypeId::LocalizedText, vec![]);
    assert_eq!(
        write_string_array(&["x".to_string()], &empty),
        Ok(array(
            VariantScalarTypeId::LocalizedText,
            vec![Variant::LocalizedText(Box::new(LocalizedText {
                locale: UAString::null(),
                text: "x".into(),
            }))]
        ))
    );

    let qn = |ns: u16, name: &str| Variant::QualifiedName(Box::new(QualifiedName::new(ns, name)));
    let node = array(VariantScalarTypeId::QualifiedName, vec![qn(2, "a")]);
    assert_eq!(
        write_string_array(&["y".to_string()], &node),
        Ok(array(VariantScalarTypeId::QualifiedName, vec![qn(2, "y")]))
    );
    let values = ["5|x", "y", "-1|z"].map(String::from);
    assert_eq!(
        write_string_array(&values, &node),
        Ok(array(
            VariantScalarTypeId::QualifiedName,
            vec![qn(5, "x"), qn(5, "y"), qn(65535, "z")]
        ))
    );
}

/// `strtol`/`strtoul(value, &end, 0)`, then `end != value &&
/// isWithinRange<T>` (`DataElementOpen62541Leaf.cpp:965-1056`).
#[test]
fn a_string_record_writes_an_integer_as_strtol_reads_it() {
    use std::ffi::c_ulong;
    assert_eq!(
        write_string("12abc", &Variant::Int32(0), None),
        Ok(Variant::Int32(12))
    );
    assert_eq!(
        write_string("0b11", &Variant::Byte(0), None),
        Ok(Variant::Byte(3))
    );
    assert!(write_string("\u{a0}7", &Variant::Int16(0), None).is_err());
    assert_eq!(
        write_string("1x", &Variant::Int32(0), Some(&choices())),
        Ok(Variant::Int32(1))
    );
    // strtoul wraps a negative to its own width.
    let long64 = c_ulong::BITS == 64;
    let max = if long64 {
        u64::MAX
    } else {
        u64::from(u32::MAX)
    };
    assert_eq!(
        write_string("-1", &Variant::UInt64(0), None),
        Ok(Variant::UInt64(max))
    );
    assert_eq!(
        write_string("-1", &Variant::UInt32(0), None).is_ok(),
        !long64
    );
    // C ignores errno and writes LONG_MAX; an overflow is refused here.
    assert!(write_string("99999999999999999999", &Variant::Int64(0), None).is_err());
}

#[test]
fn an_outgoing_enum_takes_a_choice_name_before_a_number() {
    let c = choices();
    assert_eq!(
        write_string("Fault", &Variant::Int32(0), Some(&c)),
        Ok(Variant::Int32(7))
    );
    assert_eq!(
        write_string("1", &Variant::Int32(0), Some(&c)),
        Ok(Variant::Int32(1))
    );
    assert!(matches!(
        write_string("3", &Variant::Int32(0), Some(&c)),
        Err(WriteError::OutOfRange { .. })
    ));
}

#[test]
fn an_outgoing_array_must_match_the_nodes_element_type() {
    let node = array(VariantScalarTypeId::Int32, vec![Variant::Int32(0)]);
    assert_eq!(
        write_array(&[1i32, 2], &node),
        Ok(array(
            VariantScalarTypeId::Int32,
            vec![Variant::Int32(1), Variant::Int32(2)]
        ))
    );
    assert!(matches!(
        write_array(&[1i16], &node),
        Err(WriteError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        write_array(&[1i32], &Variant::Int32(0)),
        Err(WriteError::NotAnArray)
    ));

    let node = array(VariantScalarTypeId::Boolean, vec![Variant::Boolean(false)]);
    assert_eq!(
        write_array_u8(&[1, 0], &node),
        Ok(array(
            VariantScalarTypeId::Boolean,
            vec![Variant::Boolean(true), Variant::Boolean(false)]
        ))
    );
    assert_eq!(
        write_array_u8(&[1, 2], &Variant::ByteString(ByteString::null())),
        Ok(Variant::ByteString(ByteString::from(vec![1, 2])))
    );

    let node = array(
        VariantScalarTypeId::String,
        vec![Variant::String(UAString::null())],
    );
    assert_eq!(
        write_string_array(&["a".to_string()], &node),
        Ok(array(
            VariantScalarTypeId::String,
            vec![Variant::String(UAString::from("a"))]
        ))
    );
}

// ---------------------------------------------------------- the C's lines

/// What the C logs for a value it cannot read, line for line
/// (`DataElementOpen62541Leaf.h:824-838`, `:911-919`).
#[test]
fn a_failed_read_logs_the_c_lines() {
    fn lines<T: std::fmt::Debug>(r: Result<T, ReadError>) -> Vec<String> {
        r.unwrap_err().lines()
    }
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::Int64(1 << 40), None)),
        ["incoming data (Int64 1099511627776) out-of-bounds for epicsInt32"]
    );
    // A value that is no choice of the enumeration is out of bounds too.
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::Int32(3), Some(&choices()))),
        ["incoming data (Int32 3) out-of-bounds for epicsInt32"]
    );
    // An unsupported kind names itself, then falls into the out-of-bounds line.
    assert_eq!(
        lines(read_scalar::<i32>(&Variant::Empty, None)),
        [
            "unsupported type kind None for incoming data",
            "incoming data (Null ) out-of-bounds for epicsInt32"
        ]
    );
    let i32s = array(VariantScalarTypeId::Int32, vec![Variant::Int32(1)]);
    assert_eq!(
        lines(read_array::<i16>(&i32s)),
        ["incoming data type (Int32) does not match EPICS array type (epicsInt16)"]
    );
    assert_eq!(
        lines(read_array::<i32>(&Variant::Int32(1))),
        ["incoming data is not an array"]
    );
}

/// What the C logs for a value it cannot send, line for line
/// (`DataElementOpen62541Leaf.h:1087-1096`, `:1128-1137`, `.cpp:1075-1100`).
#[test]
fn a_failed_write_logs_the_c_lines() {
    fn lines(r: Result<Variant, WriteError>) -> Vec<String> {
        r.unwrap_err().lines()
    }
    assert_eq!(
        lines(write_scalar(300i32, &Variant::Byte(0), None)),
        ["value out of range"]
    );
    assert_eq!(
        lines(write_scalar(3i32, &Variant::Int32(0), Some(&choices()))),
        ["value out of range"]
    );
    assert_eq!(
        lines(write_scalar(1i32, &Variant::Empty, None)),
        [
            "unsupported conversion from epicsInt32 to Null for outgoing data",
            "value out of range"
        ]
    );
    assert_eq!(
        lines(write_string("300", &Variant::Byte(0), None)),
        ["value \"300\" out of range"]
    );
    assert_eq!(
        lines(write_string("3", &Variant::Int32(0), Some(&choices()))),
        ["value \"3\" out of range"]
    );
    assert_eq!(
        lines(write_string(
            "x",
            &Variant::StatusCode(StatusCode::Good),
            None
        )),
        [
            "unsupported conversion from string to StatusCode for outgoing data",
            "value \"x\" out of range"
        ]
    );
    let i32s = array(VariantScalarTypeId::Int32, vec![Variant::Int32(0)]);
    assert_eq!(
        lines(write_array(&[1i16], &i32s)),
        [
            "OPC UA data type (Int32) does not match expected type (Int16) for EPICS array (epicsInt16)"
        ]
    );
    assert_eq!(
        lines(write_array(&[1i32], &Variant::Int32(0))),
        ["OPC UA data type is not an array"]
    );
}
