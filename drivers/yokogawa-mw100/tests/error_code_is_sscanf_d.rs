//! `parse_error_code` is `drvMW100.c:540`'s `sscanf("E1 %d %*s\r\n")`, which
//! glibc reads as `(int) strtol(.., 10)`: decimal only, and an out-of-`int`
//! code wraps rather than clamps where `long` is 64 bits; a 32-bit `long`
//! saturates first. Expected values are glibc 2.39.

use yokogawa_mw100::codec::parse_error_code;

#[test]
fn error_code_is_decimal_int_truncated_strtol() {
    let lp64 = std::ffi::c_long::BITS == 64;
    assert_eq!(parse_error_code(b"E1 010\r\n"), Some(10));
    assert_eq!(parse_error_code(b"E1 0x1F\r\n"), Some(0));
    assert_eq!(
        parse_error_code(b"E1 4294967297\r\n"),
        Some(if lp64 { 1 } else { i32::MAX })
    );
    assert_eq!(
        parse_error_code(b"E1 -2147483649\r\n"),
        Some(if lp64 { i32::MAX } else { i32::MIN })
    );
}
