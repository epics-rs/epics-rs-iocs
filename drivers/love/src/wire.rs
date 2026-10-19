//! Frame checksum/parsing helpers ported from `drvLove.c`'s `sendCommand`,
//! `evalMessage`, and `calcChecksum`. The Love RS-485 wire protocol frames
//! every message as `STX(1) ADDR(2 hex) BODY... CHECKSUM(2 hex)`, where
//! `CHECKSUM` is a sum-of-bytes-mod-256 over everything between STX and the
//! checksum itself (host->device frames also insert a literal `'L'` after
//! STX; device->device replies do not).
//!
//! Every numeric field comes from a `sscanf("%Nx"/"%Nd", ...)` in C at the
//! field's offset — [`scan_hex`]/[`scan_dec`].

/// C `calcChecksum`: sum of bytes mod 256 (C accumulates into `unsigned
/// long` then truncates with `& 0xFF`; wrapping `u8` addition is the same
/// value since addition mod 256 is associative/commutative).
pub fn calc_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// C locale `isspace`, which `sscanf` and `atol` skip.
fn is_c_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// C `sscanf(&reply[start],"%<width>x"|"%<width>d",&v)` on a reply that ends,
/// as C's buffer does, at its NUL. Leading whitespace does not count toward
/// `width`, so the conversion can run on into the next field; a sign, and
/// for hex a `0x` prefix, does. C leaves `v` unset when nothing converts;
/// that reads as 0 here.
fn scan(reply: &[u8], start: usize, width: usize, radix: u32) -> i64 {
    let s = reply.get(start..).unwrap_or(&[]);
    let s = &s[s.iter().take_while(|&&b| is_c_space(b)).count()..];
    let s = &s[..s.len().min(width)];
    let (neg, s) = match s {
        [b'-', rest @ ..] => (true, rest),
        [b'+', rest @ ..] => (false, rest),
        _ => (false, s),
    };
    let s = match s {
        [b'0', b'x' | b'X', rest @ ..] if radix == 16 => rest,
        _ => s,
    };
    let magnitude = s
        .iter()
        .map_while(|&b| (b as char).to_digit(radix))
        .fold(0i64, |acc, d| acc * i64::from(radix) + i64::from(d));
    if neg { -magnitude } else { magnitude }
}

/// C `sscanf(&reply[start],"%<width>x",&v)` into an `unsigned int`.
pub fn scan_hex(reply: &[u8], start: usize, width: usize) -> u32 {
    scan(reply, start, width, 16) as u32
}

/// C `sscanf(&reply[start],"%<width>d",&v)` into an `int`.
pub fn scan_dec(reply: &[u8], start: usize, width: usize) -> i32 {
    scan(reply, start, width, 10) as i32
}

/// C `static char* errCodes[]` (`drvLove.c:231-244`) — indexed by the
/// 2-digit error code in a 7-byte error reply. Trace-log text only; never
/// reaches an EPICS record (see [`eval_message`]'s error-reply branch).
pub const ERR_CODES: &[&str] = &[
    "00 - Not used.",
    "01 - Undefined command. Command not within acceptable range.",
    "02 - Checksum error on received data from Host.",
    "03 - Command not performed by instrument.",
    "04 - Illegal ASCII characters received.",
    "05 - Data field error. Not enough, too many, or improper positioning.",
    "06 - Undefined command. Command not within acceptable range.",
    "07 - Not used.",
    "08 - Hardware fault. Return to Factory for service.",
    "09 - Hardware fault. Return to Factory for service.",
    "10 - Undefined command. Command not within acceptable range.",
];

/// Outcome of [`eval_message`]'s success path: the reply payload with the
/// `STX(1) FILTER(1) ADDR(2) ... CHECKSUM(2)` envelope stripped, matching
/// what C `evalMessage` leaves in `pout` (the driver's `inpMsg` reused
/// in-place: `memcpy(pout,&pinp[4],len)`).
#[derive(Debug, PartialEq, Eq)]
pub enum EvalError {
    /// C: `*pinp != '\002'`.
    MissingStx,
    /// C: `*pcount < 7`.
    TooShort,
    /// C: `*pcount == 7` — a device error reply. Carries the decoded error
    /// text for logging (or a fallback if the code is out of
    /// [`ERR_CODES`]'s 0-10 range; C indexes `errCodes[errNum]` with no
    /// bounds check, undefined behavior for `errNum > 10`, substituted here
    /// with a defined fallback message).
    DeviceError(&'static str),
    /// C: computed checksum != the 2 hex digits at the message tail.
    ChecksumMismatch,
}

/// C `evalMessage`: validate STX/length/checksum and return the payload
/// (the bytes between the 4-byte header and the 2-byte trailing checksum).
/// `raw` is the reply already read up to (and, by the underlying octet
/// port's own EOS framing, not including) the input EOS byte — see the
/// module doc on frame layout.
pub fn eval_message(raw: &[u8]) -> Result<&[u8], EvalError> {
    if raw.first() != Some(&0x02) {
        return Err(EvalError::MissingStx);
    }
    if raw.len() < 7 {
        return Err(EvalError::TooShort);
    }
    if raw.len() == 7 {
        // C `atol(pinp + 5)`: the frame ends two bytes on.
        let err_num = scan_dec(raw, 5, 2) as usize;
        let text = ERR_CODES
            .get(err_num)
            .copied()
            .unwrap_or("unknown error code");
        return Err(EvalError::DeviceError(text));
    }

    let len = raw.len() - 3; // minus STX and the 2-byte checksum
    let cs_pos = raw.len() - 2;
    let cs_val = calc_checksum(&raw[1..1 + len]);
    let cs_msg = scan_hex(raw, cs_pos, 2);
    if cs_msg != cs_val as u32 {
        return Err(EvalError::ChecksumMismatch);
    }

    let payload_len = len - 3; // further minus FILTER and 2-byte ADDR
    Ok(&raw[4..4 + payload_len])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calc_checksum_sums_bytes_mod_256() {
        assert_eq!(calc_checksum(b"00"), b'0' + b'0');
        assert_eq!(calc_checksum(&[0xFFu8; 3]), (0xFFu32 * 3) as u8);
    }

    #[test]
    fn checksum_format_space_pads_like_c_percent_2x() {
        // C `sprintf(outMsg,"\002L%s%2X",tmpMsg,cs)` -- `%2X` has no `0`
        // flag, so C pads with a space, not a zero, for cs < 0x10.
        assert_eq!(format!("{:2X}", 5u8), " 5");
        assert_eq!(format!("{:2X}", 0xABu8), "AB");
    }

    #[test]
    fn scan_stops_at_first_non_digit_and_defaults_to_zero() {
        assert_eq!(scan_hex(b"1A", 0, 2), 0x1A);
        assert_eq!(scan_hex(b"ZZ", 0, 2), 0);
        assert_eq!(scan_hex(b"", 0, 2), 0);
        assert_eq!(scan_dec(b"1234", 0, 4), 1234);
        assert_eq!(scan_dec(b"-007", 0, 4), -7);
        assert_eq!(scan_dec(b"+42", 0, 4), 42);
        assert_eq!(scan_dec(b"xx", 0, 4), 0);
        assert_eq!(scan_dec(b"0102", 10, 4), 0);
    }

    #[test]
    fn scan_takes_width_after_whitespace_and_runs_past_the_field() {
        assert_eq!(scan_dec(b"01020304", 2, 4), 203);
        assert_eq!(scan_hex(b"  0101", 0, 4), 0x0101);
        assert_eq!(scan_dec(b"\x0b\x0c7", 0, 1), 7);
        assert_eq!(scan_dec(b"-123", 0, 2), -1);
    }

    #[test]
    fn scan_hex_takes_a_sign_and_a_0x_prefix_like_percent_x() {
        assert_eq!(scan_hex(b"0x1F", 0, 4), 0x1F);
        assert_eq!(scan_hex(b"0x1F", 0, 3), 0x1);
        assert_eq!(scan_hex(b"-1", 0, 2), u32::MAX);
    }

    #[test]
    fn eval_message_rejects_missing_stx_and_short_messages() {
        assert_eq!(eval_message(b"XL01001100"), Err(EvalError::MissingStx));
        assert_eq!(eval_message(b"\x02L0100"), Err(EvalError::TooShort));
    }

    #[test]
    fn eval_message_decodes_a_7_byte_device_error_reply() {
        // STX + 4 header bytes + 2-digit error code (no checksum in this
        // branch) -- e.g. error code "02" (checksum error).
        let raw = b"\x02XXXX02";
        assert_eq!(
            eval_message(raw),
            Err(EvalError::DeviceError(
                "02 - Checksum error on received data from Host."
            ))
        );
    }

    #[test]
    fn eval_message_out_of_range_error_code_gets_a_defined_fallback() {
        let raw = b"\x02XXXX99";
        assert_eq!(
            eval_message(raw),
            Err(EvalError::DeviceError("unknown error code"))
        );
    }

    #[test]
    fn eval_message_validates_checksum_and_strips_the_envelope() {
        // header(STX + FILTER + ADDR) = "\x02X01", payload = "0064",
        // checksum over pinp[1..pcount-2) = "X010064".
        let body = b"X010064";
        let cs = calc_checksum(body);
        let mut raw = vec![0x02u8];
        raw.extend_from_slice(body);
        raw.extend_from_slice(format!("{cs:02X}").as_bytes());

        assert_eq!(eval_message(&raw), Ok(&b"0064"[..]));
    }

    #[test]
    fn eval_message_accepts_the_space_padded_checksum_sscanf_reads() {
        // C `sscanf(&pinp[csPos],"%2x")` skips the pad `sendCommand`'s own
        // `%2X` writes for a checksum below 0x10.
        let body = b"X01999980";
        let cs = calc_checksum(body);
        assert!(cs < 0x10);
        let mut raw = vec![0x02u8];
        raw.extend_from_slice(body);
        raw.extend_from_slice(format!("{cs:2X}").as_bytes());

        assert_eq!(eval_message(&raw), Ok(&b"999980"[..]));
    }

    #[test]
    fn eval_message_error_code_skips_leading_whitespace_like_atol() {
        assert_eq!(
            eval_message(b"\x02XXXX 2"),
            Err(EvalError::DeviceError(
                "02 - Checksum error on received data from Host."
            ))
        );
    }

    #[test]
    fn eval_message_rejects_a_bad_checksum() {
        let raw = b"\x02X010064FF";
        assert_eq!(eval_message(raw), Err(EvalError::ChecksumMismatch));
    }
}
