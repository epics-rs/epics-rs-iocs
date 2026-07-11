//! The `%0<nsum>` StreamDevice converter used throughout `teled_d.proto` /
//! `teled_h.proto`: a negated 8-bit sum ("NSUM" -- negative-sum) of every
//! byte in the message, formatted as 2 uppercase ASCII hex digits.
//!
//! Derived from the Teledyne ISCO D-Series manual (`documentation/
//! Teledyne_ISCO_D_Series_Manual.pdf`, Section 8, "Hexadecimal/Decimal
//! Format Using MODULO"), not guessed: the manual's own worked example sums
//! the bytes of `"R304STOP"` to `0x22F` (559 decimal), reduces mod `0x100` to
//! `0x2F` (47), and negates via `0x100 - 0x2F = 0xD1`, matching the `D1`
//! checksum shown appended to that frame in the example. [`nsum_checksum`]
//! reproduces exactly this: `(0x100 - (sum_of_bytes mod 0x100)) mod 0x100`,
//! rendered `{:02X}`.
//!
//! StreamDevice's `%0<nsum>` converter checksums the entire message rendered
//! so far when it appears in an `out` format (i.e. every byte the format
//! string has emitted up to that point, including any literal characters
//! before the first substituted parameter), and validates the same range
//! against the trailing 2 hex digits when it appears in an `in` format. Every
//! `format_*`/`parse_*` pair in [`crate::wire_d`]/[`crate::wire_h`] follows
//! that convention: the checksum spans from the start of the rendered body
//! up to (not including) the checksum's own 2 bytes.

/// Compute the 2-byte uppercase ASCII hex checksum StreamDevice's
/// `%0<nsum>` converter would append to (or validate against) `body`.
pub fn nsum_checksum(body: &[u8]) -> [u8; 2] {
    let sum: u8 = body.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    let cs = 0u8.wrapping_sub(sum);
    let hex = format!("{cs:02X}");
    let bytes = hex.as_bytes();
    [bytes[0], bytes[1]]
}

/// Append the checksum of `body` to `body` itself, returning the full
/// pre-terminator frame text (the terminating CR is applied separately by
/// the octet port's configured output EOS, matching every other driver in
/// this workspace -- see `connect.rs`'s module doc).
pub fn append_checksum(body: &str) -> String {
    let cs = nsum_checksum(body.as_bytes());
    let mut out = String::with_capacity(body.len() + 2);
    out.push_str(body);
    out.push(cs[0] as char);
    out.push(cs[1] as char);
    out
}

/// Validate that `trailing` (the last 2 bytes of a received reply) is the
/// correct `%0<nsum>` checksum of `body` (everything before those 2 bytes).
pub fn checksum_valid(body: &[u8], trailing: &[u8]) -> bool {
    trailing.len() == 2 && nsum_checksum(body) == [trailing[0], trailing[1]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_vendor_manual_worked_example() {
        // Section 8 manual derivation: sum('R'+'3'+'0'+'4'+'S'+'T'+'O'+'P')
        // = 0x22F -> mod 0x100 = 0x2F -> 0x100-0x2F = 0xD1 = "D1".
        assert_eq!(nsum_checksum(b"R304STOP"), *b"D1");
    }

    #[test]
    fn sum_matches_manual_intermediate_value() {
        let sum: u32 = b"R304STOP".iter().map(|&b| b as u32).sum();
        assert_eq!(sum, 0x22F);
        assert_eq!(sum % 0x100, 0x2F);
    }

    #[test]
    fn append_checksum_appends_two_hex_digits() {
        let frame = append_checksum("R304STOP");
        assert_eq!(frame, "R304STOPD1");
    }

    #[test]
    fn checksum_valid_accepts_correct_trailer_and_rejects_wrong_one() {
        assert!(checksum_valid(b"R304STOP", b"D1"));
        assert!(!checksum_valid(b"R304STOP", b"D2"));
        assert!(!checksum_valid(b"R304STOP", b"D"));
    }

    #[test]
    fn empty_body_checksum_is_zero() {
        // sum=0 -> 0x100-0=0x100 mod 0x100 = 0.
        assert_eq!(nsum_checksum(b""), *b"00");
    }

    #[test]
    fn wraps_correctly_when_sum_is_a_multiple_of_0x100() {
        // Two bytes 0x80+0x80=0x100 -> mod 0x100 = 0 -> checksum "00".
        assert_eq!(nsum_checksum(&[0x80, 0x80]), *b"00");
    }
}
