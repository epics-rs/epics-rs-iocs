//! Boundary tests for the Variant ↔ EPICS conversions.

use async_opcua::types::{
    Array, ByteString, LocalizedText, QualifiedName, UAString, Variant, VariantScalarTypeId,
};
use opcua::value::{
    ConvError, EnumChoices, hex_decode, hex_encode, read_array, read_array_u8, read_scalar,
    read_string, read_string_array, write_array, write_array_u8, write_scalar, write_string,
    write_string_array,
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
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        read_scalar::<i32>(&Variant::UInt32(u32::MAX), None),
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        read_scalar::<u32>(&Variant::Int32(-1), None),
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        read_scalar::<i64>(&Variant::UInt64(u64::MAX), None),
        Err(ConvError::OutOfRange { .. })
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
        Err(ConvError::OutOfRange { .. })
    ));
    assert_eq!(
        read_scalar::<i32>(&Variant::Double(-2147483648.0), None),
        Ok(i32::MIN)
    );
    assert!(matches!(
        read_scalar::<i32>(&Variant::Double(-2147483649.0), None),
        Err(ConvError::OutOfRange { .. })
    ));
    // 2^63 is what `INT64_MAX` rounds to as a double, so the C's `value > max`
    // check lets it through and then casts it — undefined behaviour.
    assert!(matches!(
        read_scalar::<i64>(&Variant::Double(9223372036854775808.0), None),
        Err(ConvError::OutOfRange { .. })
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
                Err(ConvError::OutOfRange { .. })
            ),
            "{v}"
        );
        assert!(matches!(
            read_scalar::<i64>(&Variant::Double(v), None),
            Err(ConvError::OutOfRange { .. })
        ));
        assert!(matches!(
            read_scalar::<u32>(&Variant::Double(v), None),
            Err(ConvError::OutOfRange { .. })
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
        Err(ConvError::OutOfRange { .. })
    ));
    // The C's `string_to(epicsInt32)` assigns a `long` to an `epicsInt32`
    // without a range check (`DataElementOpen62541Leaf.h:150-157`), silently
    // truncating; only its `epicsUInt32` overload checks.
    assert!(matches!(
        read_scalar::<i32>(&s("99999999999"), None),
        Err(ConvError::OutOfRange { .. })
    ));

    let lt = Variant::LocalizedText(Box::new(LocalizedText::new("en", "17")));
    assert_eq!(read_scalar::<i32>(&lt, None), Ok(17));
}

#[test]
fn a_type_with_no_numeric_meaning_is_unsupported() {
    let qn = Variant::QualifiedName(Box::new(QualifiedName::new(2, "Name")));
    assert!(matches!(
        read_scalar::<i32>(&qn, None),
        Err(ConvError::Unsupported { .. })
    ));
    assert!(matches!(
        read_scalar::<i32>(&Variant::Empty, None),
        Err(ConvError::Unsupported { .. })
    ));
}

#[test]
fn an_enum_value_must_be_one_of_its_choices() {
    let c = choices();
    assert_eq!(read_scalar::<i32>(&Variant::Int32(7), Some(&c)), Ok(7));
    assert!(matches!(
        read_scalar::<i32>(&Variant::Int32(3), Some(&c)),
        Err(ConvError::NotAnEnumChoice { .. })
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
        Ok("1.5".to_string())
    );
    assert_eq!(
        read_string(&Variant::Boolean(true), None),
        Ok("true".to_string())
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
        Err(ConvError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        read_array::<f64>(&i32s),
        Err(ConvError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        read_array::<i32>(&Variant::Int32(1)),
        Err(ConvError::NotAnArray)
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
        Err(ConvError::ArrayTypeMismatch { .. })
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
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(-1i32, &Variant::UInt32(0), None),
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(1e300f64, &Variant::Float(0.0), None),
        Err(ConvError::OutOfRange { .. })
    ));
    assert!(matches!(
        write_scalar(1i32, &Variant::Empty, None),
        Err(ConvError::Unsupported { .. })
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
        Err(ConvError::NotAnEnumChoice { .. })
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
        Err(ConvError::OutOfRange { .. })
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
        Err(ConvError::OutOfRange { .. })
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
        Err(ConvError::NotAnEnumChoice { .. })
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
        Err(ConvError::ArrayTypeMismatch { .. })
    ));
    assert!(matches!(
        write_array(&[1i32], &Variant::Int32(0)),
        Err(ConvError::NotAnArray)
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
