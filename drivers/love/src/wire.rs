//! Frame checksum/parsing helpers ported from `drvLove.c`'s `sendCommand`,
//! `evalMessage`, and `calcChecksum`. The Love RS-485 wire protocol frames
//! every message as `STX(1) ADDR(2 hex) BODY... CHECKSUM(2 hex)`, where
//! `CHECKSUM` is a sum-of-bytes-mod-256 over everything between STX and the
//! checksum itself (host->device frames also insert a literal `'L'` after
//! STX; device->device replies do not).
//!
//! Unlike the delaygen drivers' `wire::atoi`/`atof` (plain, unbounded C
//! `atoi`/`atof`), every numeric field here comes from a *fixed-width*
//! `sscanf("%Nx"/"%Nd", ...)` in C — [`parse_hex`]/[`parse_dec`] mirror that
//! by taking an already-width-sliced `&[u8]` (see [`field`]) rather than an
//! unbounded string.

/// C `calcChecksum`: sum of bytes mod 256 (C accumulates into `unsigned
/// long` then truncates with `& 0xFF`; wrapping `u8` addition is the same
/// value since addition mod 256 is associative/commutative).
pub fn calc_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// C `sscanf(s,"%Nx",&v)` restricted to an already width-sliced buffer:
/// parse the leading run of hex digits, `0` if none.
pub fn parse_hex(s: &[u8]) -> u32 {
    let mut i = 0;
    while i < s.len() && s[i].is_ascii_hexdigit() {
        i += 1;
    }
    if i == 0 {
        return 0;
    }
    u32::from_str_radix(std::str::from_utf8(&s[..i]).unwrap(), 16).unwrap_or(0)
}

/// C `sscanf(s,"%Nd",&v)` restricted to an already width-sliced buffer:
/// parse an optionally-signed leading run of decimal digits, `0` if none.
pub fn parse_dec(s: &[u8]) -> i32 {
    let mut i = 0;
    let neg = match s.first() {
        Some(b'-') => {
            i = 1;
            true
        }
        Some(b'+') => {
            i = 1;
            false
        }
        _ => false,
    };
    let start = i;
    while i < s.len() && s[i].is_ascii_digit() {
        i += 1;
    }
    if i == start {
        return 0;
    }
    let magnitude: i32 = std::str::from_utf8(&s[start..i])
        .unwrap()
        .parse()
        .unwrap_or(0);
    if neg { -magnitude } else { magnitude }
}

/// Slice `payload[start..start+width]`, clamped to what's actually
/// available (never panics on a short/malformed device reply) — the Rust
/// equivalent of `sscanf`'s width cap combined with C's implicit
/// null-terminator stop.
pub fn field(payload: &[u8], start: usize, width: usize) -> &[u8] {
    payload
        .get(start..)
        .map(|s| &s[..s.len().min(width)])
        .unwrap_or(&[])
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
        let err_num = parse_dec(&raw[5..7]) as usize;
        let text = ERR_CODES
            .get(err_num)
            .copied()
            .unwrap_or("unknown error code");
        return Err(EvalError::DeviceError(text));
    }

    let len = raw.len() - 3; // minus STX and the 2-byte checksum
    let cs_pos = raw.len() - 2;
    let cs_val = calc_checksum(&raw[1..1 + len]);
    let cs_msg = parse_hex(&raw[cs_pos..cs_pos + 2]);
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
    fn parse_hex_stops_at_first_non_hex_and_defaults_to_zero() {
        assert_eq!(parse_hex(b"1A"), 0x1A);
        assert_eq!(parse_hex(b"ZZ"), 0);
        assert_eq!(parse_hex(b""), 0);
    }

    #[test]
    fn parse_dec_handles_sign_and_junk() {
        assert_eq!(parse_dec(b"1234"), 1234);
        assert_eq!(parse_dec(b"-007"), -7);
        assert_eq!(parse_dec(b"+42"), 42);
        assert_eq!(parse_dec(b"xx"), 0);
    }

    #[test]
    fn field_clamps_to_available_length_without_panicking() {
        assert_eq!(field(b"0102", 0, 4), b"0102");
        assert_eq!(field(b"01", 0, 4), b"01");
        assert_eq!(field(b"0102", 2, 4), b"02");
        assert_eq!(field(b"0102", 10, 4), b"");
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
    fn eval_message_rejects_a_bad_checksum() {
        let raw = b"\x02X010064FF";
        assert_eq!(eval_message(raw), Err(EvalError::ChecksumMismatch));
    }
}
