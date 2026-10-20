//! `parse_error_code` is `drvMW100.c:540`'s `sscanf("E1 %d %*s\r\n")`, which
//! glibc reads as `(int) strtol(.., 10)`: decimal only, and an out-of-`int`
//! code wraps rather than clamps. Expected values are glibc 2.39.

use yokogawa_mw100::codec::parse_error_code;

#[test]
fn error_code_is_decimal_int_truncated_strtol() {
    assert_eq!(parse_error_code(b"E1 010\r\n"), Some(10));
    assert_eq!(parse_error_code(b"E1 0x1F\r\n"), Some(0));
    assert_eq!(parse_error_code(b"E1 4294967297\r\n"), Some(1));
    assert_eq!(parse_error_code(b"E1 -2147483649\r\n"), Some(i32::MAX));
}
