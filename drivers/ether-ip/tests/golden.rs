//! Byte-for-byte fixtures produced by the ORIGINAL C encoders.
//!
//! Every expected hex string in this file was emitted by a harness that
//! `#include`s the real `ether_ip.c` from `epics-modules/ether_ip` (with five
//! EPICS symbols stubbed: `epicsMutexOsiCreate`, `epicsMutexLock`,
//! `epicsMutexUnlock`, `hostToIPAddr`, `epicsTimeGetCurrent`,
//! `epicsTimeToStrftime`) and dumps the buffers its encoders build. They are
//! not hand-derived, so a mismatch here means the Rust port disagrees with the
//! C on the wire.
//!
//! The three decode fixtures marked BUG are the C's *wrong* answers; the port
//! deliberately differs and the tests below assert the corrected value while
//! naming what the C returned.

use ether_ip::cip::*;
use ether_ip::encap::{self, TransactionId};

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect()
}

fn tag(s: &str) -> ParsedTag {
    ParsedTag::parse(s).expect("tag parses")
}

fn path_of(s: &str) -> String {
    let t = tag(s);
    let mut out = Vec::new();
    t.encode_path(&mut out);
    hex(&out)
}

// ---------------------------------------------------------------------------
// Tag path encoding (C `make_CIP_ReadData`'s path half / `EIP_parse_tag`)
// ---------------------------------------------------------------------------

#[test]
fn tag_path_plain() {
    assert_eq!(tag("FRED").path_words(), 3);
    assert_eq!(path_of("FRED"), "910446524544");
}

#[test]
fn tag_path_odd_length_is_padded() {
    assert_eq!(tag("BOOLS").path_words(), 4);
    assert_eq!(path_of("BOOLS"), "9105424F4F4C5300");
}

#[test]
fn tag_path_with_small_element() {
    assert_eq!(tag("REALS[3]").path_words(), 5);
    assert_eq!(path_of("REALS[3]"), "91055245414C53002803");
}

#[test]
fn tag_path_element_needs_16_bits() {
    assert_eq!(tag("my_tag[300]").path_words(), 6);
    assert_eq!(path_of("my_tag[300]"), "91066D795F74616729002C01");
}

#[test]
fn tag_path_element_needs_32_bits() {
    assert_eq!(tag("big[70000]").path_words(), 6);
    assert_eq!(path_of("big[70000]"), "9103626967002A0070110100");
}

#[test]
fn tag_path_structure_member() {
    assert_eq!(tag("Struct.Member").path_words(), 8);
    assert_eq!(path_of("Struct.Member"), "910653747275637491064D656D626572");
}

#[test]
fn tag_path_two_indexed_levels() {
    assert_eq!(tag("arr[2].sub[3]").path_words(), 8);
    assert_eq!(path_of("arr[2].sub[3]"), "91036172720028029103737562002803");
}

#[test]
fn tag_path_round_trips_through_display() {
    for t in ["FRED", "REALS[3]", "Struct.Member", "arr[2].sub[3]"] {
        assert_eq!(tag(t).to_string(), t);
    }
}

// ---------------------------------------------------------------------------
// CIA paths (C `make_CIA_path`)
// ---------------------------------------------------------------------------

#[test]
fn cia_paths() {
    let cases: [(CipClass, u32, u8, usize, &str); 3] = [
        (CipClass::MessageRouter, 1, 0, 2, "20022401"),
        (CipClass::Identity, 1, 7, 3, "200124013007"),
        (CipClass::Symbol, 0x1234, 0, 3, "206B25003412"),
    ];
    for (class, instance, attr, words, expect) in cases {
        assert_eq!(cia_path_words(instance, attr), words, "{class:?}");
        let mut out = Vec::new();
        encode_cia_path(&mut out, class, instance, attr);
        assert_eq!(hex(&out), expect, "{class:?}");
    }
}

// ---------------------------------------------------------------------------
// ReadData / WriteData (C `make_CIP_ReadData` / `make_CIP_WriteData`)
// ---------------------------------------------------------------------------

#[test]
fn read_data_request() {
    let t = tag("REALS[3]");
    assert_eq!(read_data_size(&t), 14);
    let mut out = Vec::new();
    encode_read_data(&mut out, &t, 2);
    assert_eq!(hex(&out), "4C0591055245414C530028030200");
    assert_eq!(out.len(), read_data_size(&t));
}

#[test]
fn write_data_request_real() {
    let t = tag("FRED");
    let mut out = Vec::new();
    encode_write_data(&mut out, &t, CipType::Real, 1, &3.5f32.to_le_bytes());
    assert_eq!(hex(&out), "4D03910446524544CA00010000006040");
    assert_eq!(out.len(), write_data_size(&t, 4));
}

#[test]
fn write_data_request_two_dints() {
    let t = tag("Cnt");
    let raw = [0x44, 0x33, 0x22, 0x11, 0xDD, 0xCC, 0xBB, 0xAA];
    let mut out = Vec::new();
    encode_write_data(&mut out, &t, CipType::Dint, 2, &raw);
    assert_eq!(hex(&out), "4D039103436E7400C400020044332211DDCCBBAA");
    assert_eq!(out.len(), write_data_size(&t, 8));
}

// ---------------------------------------------------------------------------
// CM_Unconnected_Send (C `make_CM_Unconnected_Send`)
// ---------------------------------------------------------------------------

#[test]
fn tick_time_split() {
    assert_eq!(calc_tick_time(245_760), Some((10, 240)));
    assert_eq!(calc_tick_time(1_000), Some((2, 250)));
    assert_eq!(calc_tick_time(8_355_841), None);
}

#[test]
fn unconnected_send_pads_odd_messages() {
    // The message is padded to a 16-bit boundary, so 9 and 10 bytes cost the
    // same 24 bytes of envelope.
    assert_eq!(unconnected_send_size(10), 24);
    assert_eq!(unconnected_send_size(9), 24);

    let mut out = Vec::new();
    encode_unconnected_send(&mut out, &[0xEE; 10], 3);
    assert_eq!(
        hex(&out),
        "5202200624010AF00A00EEEEEEEEEEEEEEEEEEEE01000103"
    );
    assert_eq!(out.len(), unconnected_send_size(10));

    let mut out = Vec::new();
    encode_unconnected_send(&mut out, &[0xEE; 9], 0);
    assert_eq!(
        hex(&out),
        "5202200624010AF00900EEEEEEEEEEEEEEEEEE0001000100"
    );
    assert_eq!(out.len(), unconnected_send_size(9));
}

// ---------------------------------------------------------------------------
// MultiRequest (C `prepare_CIP_MultiRequest` / `CIP_MultiRequest_item`)
// ---------------------------------------------------------------------------

#[test]
fn multi_request_sizes() {
    assert_eq!(multi_request_size(2, 16), 28);
    assert_eq!(multi_response_size(2, 40), 50);
}

#[test]
fn multi_request_two_reads() {
    let mut a = Vec::new();
    encode_read_data(&mut a, &tag("A"), 1);
    let mut b = Vec::new();
    encode_read_data(&mut b, &tag("BB"), 1);

    let mut out = Vec::new();
    encode_multi_request(&mut out, &[a, b]);
    assert_eq!(
        hex(&out),
        "0A0220022401020006000E004C029101410001004C02910242420100"
    );
}

#[test]
fn multi_response_is_split_by_the_offset_table() {
    // Two sub-replies, at offsets 10 and 20 from the count field, sizes 10 and
    // 4 -- the C's `get_CIP_MultiRequest_Response` returns exactly these.
    let mut r = Vec::new();
    r.extend_from_slice(&[0x8A, 0x00, 0x00, 0x00]); // MR_Response header
    r.extend_from_slice(&2u16.to_le_bytes()); // count
    r.extend_from_slice(&6u16.to_le_bytes()); // offset[0]
    r.extend_from_slice(&16u16.to_le_bytes()); // offset[1]
    r.extend_from_slice(&[0xCC, 0x00, 0x00, 0x00, 0xC4, 0x00, 1, 0, 0, 0]); // sub 0
    r.extend_from_slice(&[0xCD, 0x00, 0x00, 0x00]); // sub 1

    assert!(check_multi_request_response(&r));
    let n = r.len();
    let s0 = get_multi_request_response(&r, n, 0).expect("sub 0");
    let s1 = get_multi_request_response(&r, n, 1).expect("sub 1");
    assert_eq!(s0.len(), 10);
    assert_eq!(s0[0], 0xCC);
    assert_eq!(s1.len(), 4);
    assert_eq!(s1[0], 0xCD);
    assert!(get_multi_request_response(&r, n, 2).is_none());
}

// ---------------------------------------------------------------------------
// MR_Response data offset (C `EIP_raw_MR_Response_data`)
// ---------------------------------------------------------------------------

#[test]
fn mr_response_data_skips_extended_status() {
    let no_ext = [0xCC, 0x00, 0x00, 0x00, 1, 2, 3, 4];
    assert_eq!(MrResponse::parse(&no_ext).unwrap().data_offset(), 4);

    let two_ext = [0xCC, 0x00, 0x00, 0x02, 9, 9, 9, 9, 1, 2, 3, 4];
    assert_eq!(MrResponse::parse(&two_ext).unwrap().data_offset(), 8);
    assert_eq!(
        MrResponse::parse(&two_ext).unwrap().data(two_ext.len()),
        &[1, 2, 3, 4]
    );
}

// ---------------------------------------------------------------------------
// Encapsulation (C `make_EncapsulationHeader` / `EIP_make_SendRRData`)
// ---------------------------------------------------------------------------

#[test]
fn transaction_id_is_eight_ascii_digits() {
    assert_eq!(TransactionId::from_counter(1).to_string(), "00000001");
    assert_eq!(TransactionId::from_counter(1).0, *b"00000001");
}

#[test]
fn register_session_request() {
    let out = encap::encode_register_session(TransactionId::from_counter(2));
    assert_eq!(
        hex(&out),
        "65000400000000000000000030303030303030320000000001000000"
    );
    assert_eq!(out.len(), encap::HEADER_SIZE + 4);
}

#[test]
fn send_rr_data_header() {
    // A 20-byte payload, session 0x12345678, context "00000001".
    let out = encap::encode_send_rr_data(0x1234_5678, TransactionId::from_counter(1), &[0u8; 20]);
    assert_eq!(
        hex(&out[..encap::RRDATA_SIZE]),
        "6F0024007856341200000000303030303030303100000000000000000000020000000000B2001400"
    );
    assert_eq!(out.len(), encap::RRDATA_SIZE + 20);

    // ... and it round-trips: the reply layout is the same shape.
    let rr = encap::decode_rr_data(&out).expect("decodes");
    assert_eq!(rr.header.command, encap::EC_SEND_RR_DATA);
    assert_eq!(rr.header.session, 0x1234_5678);
    assert_eq!(rr.data_length, 20);
    assert_eq!(rr.response.len(), 20);
}

#[test]
fn get_attribute_single_full_frame() {
    // The complete `EIP_Get_Attribute_Single(C_Identity, 1, 7)` frame.
    let words = cia_path_words(1, 7);
    let mut req = Vec::new();
    req.push(service::GET_ATTRIBUTE_SINGLE);
    req.push(words as u8);
    encode_cia_path(&mut req, CipClass::Identity, 1, 7);

    let out = encap::encode_send_rr_data(0x1234_5678, TransactionId::from_counter(3), &req);
    assert_eq!(
        hex(&out),
        "6F0018007856341200000000303030303030303300000000000000000000020000000000B200\
         08000E03200124013007"
            .replace(' ', "")
    );
}

// ---------------------------------------------------------------------------
// End-to-end frames: exactly what the scan task puts on the wire
// ---------------------------------------------------------------------------

#[test]
fn end_to_end_scan_frame() {
    // Read REALS[3] x4 and Cnt, slot 0, session 0x12345678, context "00000001".
    let mut r1 = Vec::new();
    encode_read_data(&mut r1, &tag("REALS[3]"), 4);
    let mut r2 = Vec::new();
    encode_read_data(&mut r2, &tag("Cnt"), 1);

    let mut multi = Vec::new();
    encode_multi_request(&mut multi, &[r1, r2]);

    let mut routed = Vec::new();
    encode_unconnected_send(&mut routed, &multi, 0);

    let frame = encap::encode_send_rr_data(0x1234_5678, TransactionId::from_counter(1), &routed);
    assert_eq!(frame.len(), 90);
    assert_eq!(
        hex(&frame),
        "6F0042007856341200000000303030303030303100000000000000000000020000000000B2003200\
         5202200624010AF024000A02200224010200060014004C0591055245414C5300280304004C039103\
         436E7400010001000100"
            .replace(' ', "")
    );
}

#[test]
fn end_to_end_write_frame() {
    // Write Cnt := 12345 as a DINT, slot 0, session 0x12345678, context "00000002".
    let t = tag("Cnt");
    let mut msg = Vec::new();
    encode_write_data(&mut msg, &t, CipType::Dint, 1, &12345i32.to_le_bytes());

    let mut routed = Vec::new();
    encode_unconnected_send(&mut routed, &msg, 0);

    let frame = encap::encode_send_rr_data(0x1234_5678, TransactionId::from_counter(2), &routed);
    assert_eq!(frame.len(), 70);
    assert_eq!(
        hex(&frame),
        "6F002E007856341200000000303030303030303200000000000000000000020000000000B2001E00\
         5202200624010AF010004D039103436E7400C40001003930000001000100"
            .replace(' ', "")
    );
}

// ---------------------------------------------------------------------------
// Type sizes (C `CIP_Type_size`)
// ---------------------------------------------------------------------------

#[test]
fn type_sizes() {
    let cases = [
        (CipType::Bool, 1),
        (CipType::Sint, 1),
        (CipType::Int, 2),
        (CipType::Dint, 4),
        (CipType::Lint, 8),
        (CipType::Real, 4),
        (CipType::Lreal, 8),
        (CipType::Bits, 4),
        (CipType::String, 0),
        (CipType::Struct, 0),
    ];
    for (t, size) in cases {
        assert_eq!(t.size(), size, "{t:?}");
        assert_eq!(CipType::from_code(t.code()), t, "{t:?}");
    }
}

// ---------------------------------------------------------------------------
// Value decode
// ---------------------------------------------------------------------------

/// Build a raw `[type][data]` buffer.
fn raw(t: CipType, data: &[u8]) -> Vec<u8> {
    let mut v = t.code().to_le_bytes().to_vec();
    v.extend_from_slice(data);
    v
}

#[test]
fn decode_real_and_lreal() {
    let r = raw(CipType::Real, &[0, 0, 0, 0, 0, 0, 0x80, 0xBE]); // [_, -0.25]
    assert_eq!(get_double(&r, 1), Some(-0.25));

    let l = raw(
        CipType::Lreal,
        &[0; 8]
            .iter()
            .copied()
            .chain((-3.75f64).to_le_bytes())
            .collect::<Vec<u8>>(),
    );
    assert_eq!(get_double(&l, 1), Some(-3.75));
}

#[test]
fn decode_bits_keeps_the_raw_pattern() {
    let b = raw(CipType::Bits, &0xDEAD_BEEFu32.to_le_bytes());
    assert_eq!(get_udint(&b, 0), Some(0xDEAD_BEEF));
}

#[test]
fn decode_lint() {
    let l = raw(CipType::Lint, &(-5i64).to_le_bytes());
    assert_eq!(get_lint(&l, 0), Some(-5));
}

/// UPSTREAM FIX (`ether_ip.c:1286-1290`, `get_CIP_double`): a DINT is unpacked
/// through `unpack_UDINT`, so the C hands an `ai` record 4.29497e+09 for a tag
/// holding -2. `get_CIP_DINT` gets this one right (4294967294 as an i32 is -2),
/// which is why the shipped ai path mostly escapes it -- but `get_CIP_double`
/// is public API and is what the REAL/LREAL branch shares.
#[test]
fn decode_negative_dint() {
    let d = raw(CipType::Dint, &[0, 0, 0, 0, 0xFE, 0xFF, 0xFF, 0xFF]);
    assert_eq!(get_dint(&d, 1), Some(-2));
    assert_eq!(get_double(&d, 1), Some(-2.0)); // C: 4.29497e+09
    assert_eq!(get_udint(&d, 1), Some(4_294_967_294)); // bit pattern, unchanged
}

/// UPSTREAM FIX (same site): an INT goes through `unpack_UINT`.
#[test]
fn decode_negative_int() {
    let i = raw(CipType::Int, &(-3i16).to_le_bytes());
    assert_eq!(get_dint(&i, 0), Some(-3));
    assert_eq!(get_double(&i, 0), Some(-3.0)); // C: 65533
}

/// UPSTREAM FIX (`ether_ip.c:1277-1281` and `:1361-1365`): SINT is read through
/// an unsigned `CN_USINT` in BOTH `get_CIP_double` and `get_CIP_DINT`, even
/// though `ether_ip.h:197` declares `typedef signed char CN_SINT`. A SINT of -2
/// reaches an `ai` record as 254 in the C.
#[test]
fn decode_negative_sint() {
    let s = raw(CipType::Sint, &[0xFE]);
    assert_eq!(get_dint(&s, 0), Some(-2)); // C: 254
    assert_eq!(get_double(&s, 0), Some(-2.0)); // C: 254
    assert_eq!(get_usint(&s, 0), Some(0xFE)); // the raw byte, for waveforms
}

#[test]
fn decode_bool_stays_unsigned() {
    let b = raw(CipType::Bool, &[0xFF]);
    assert_eq!(get_dint(&b, 0), Some(255));
    assert_eq!(get_double(&b, 0), Some(255.0));
}

// ---------------------------------------------------------------------------
// Logix STRING structs
// ---------------------------------------------------------------------------

/// A Logix STRING element: len (u32) then 84 characters.
fn string_struct(elements: &[&str]) -> Vec<u8> {
    let mut v = CipType::Struct.code().to_le_bytes().to_vec();
    v.extend_from_slice(&STRUCT_STRING_HANDLE.to_le_bytes());
    for s in elements {
        v.extend_from_slice(&(s.len() as u32).to_le_bytes());
        let mut buf = [0u8; STRUCT_STRING_BUF];
        buf[..s.len()].copy_from_slice(s.as_bytes());
        v.extend_from_slice(&buf);
    }
    v
}

#[test]
fn decode_string_elements() {
    let s = string_struct(&["Hello", "Bye"]);
    assert_eq!(get_string(&s, 0, 40).as_deref(), Some("Hello"));
    assert_eq!(get_string(&s, 1, 40).as_deref(), Some("Bye"));
}

/// UPSTREAM FIX (`ether_ip.c:1440-1447`): the `T_CIP_STRING` branch does
/// `memcpy(buffer, buf, size); *(buffer+size) = '\0';` -- `size` bytes copied
/// and a terminator written at `buffer[size]`, one byte past a caller who (like
/// `si_read`) passed a `size`-byte array. We bound the copy by `max - 1`.
#[test]
fn decode_string_respects_the_callers_buffer() {
    let s = string_struct(&["0123456789ABCDEF"]);
    // max = 8 means 7 characters plus the terminator.
    assert_eq!(get_string(&s, 0, 8).as_deref(), Some("0123456"));
    assert_eq!(get_string(&s, 0, 1).as_deref(), Some(""));
    assert_eq!(get_string(&s, 0, 0), None);
}

#[test]
fn put_string_matches_the_c_layout() {
    // C `put_CIP_STRING(data, "Wow", ...)` produces this 16-byte buffer.
    let mut buf = vec![0u8; 16];
    buf[..2].copy_from_slice(&CipType::Struct.code().to_le_bytes());
    buf[2..4].copy_from_slice(&STRUCT_STRING_HANDLE.to_le_bytes());
    assert!(put_string(&mut buf, "Wow"));
    assert_eq!(hex(&buf), "A002CE0F03000000576F770000000000");
}

#[test]
fn write_string_request() {
    // C `make_CIP_WriteData(Msg, T_CIP_STRUCT, "Hi")`.
    let mut data = vec![0u8; 16];
    data[..2].copy_from_slice(&CipType::Struct.code().to_le_bytes());
    data[2..4].copy_from_slice(&STRUCT_STRING_HANDLE.to_le_bytes());
    assert!(put_string(&mut data, "Hi"));

    let mut out = Vec::new();
    // The driver passes the buffer WITHOUT the leading type code.
    encode_write_string(&mut out, &tag("Msg"), 1, &data[TYPECODE_SIZE..], 480);
    assert_eq!(hex(&out), "4D0391034D736700A002CE0F010002000000486900");
}

// ---------------------------------------------------------------------------
// Bounds: every accessor is total. (The C reads past the end.)
// ---------------------------------------------------------------------------

#[test]
fn accessors_are_bounded() {
    let d = raw(CipType::Dint, &[1, 2, 3, 4]);
    assert_eq!(get_dint(&d, 0), Some(0x0403_0201));
    assert_eq!(get_dint(&d, 1), None);
    assert_eq!(get_double(&d, 9), None);
    assert_eq!(get_udint(&d, 1), None);
    assert_eq!(typecode(&[0x00]), None);
    assert!(MrResponse::parse(&[0xCC, 0x00, 0x00]).is_none());

    let mut d = d;
    assert!(put_dint(&mut d, 0, 7));
    assert!(!put_dint(&mut d, 1, 7));
}
