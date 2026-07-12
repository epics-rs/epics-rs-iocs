//! Heidenhain ND261 display unit protocol (`devAiHeidND261.c`).
//!
//! One transaction: write `^B\n` (`STX` + LF), read the reply the ND261 pushes
//! back, terminated by `\n\n` (the C set that terminator as the port's input EOS
//! itself, `devAiHeidND261.c:167`; here `st.cmd` sets it with
//! `asynOctetSetInputEos`).
//!
//! The reply is a sign character followed by the number:
//! `pai->val = sscanf(&inbuf[1], "%lf") * (inbuf[0] == '-' ? -1 : 1)`
//! (`devAiHeidND261.c:233-234`).

use crate::fmt::scan_f64;

/// `^B` then LF — the readout request (`devAiHeidND261.c:104-105`).
pub const READ_COMMAND: [u8; 2] = [0x02, b'\n'];

/// The C rejected any reply shorter than this (`devAiHeidND261.c:222`).
pub const MIN_REPLY_LEN: usize = 14;

/// The value the C forced into the record when the read failed
/// (`devAiHeidND261.c:223`).
pub const INVALID_VALUE: f64 = 99999.0;

/// Decode one reply into a position.
///
/// `devAiHeidND261.c:229-231` chops two more characters off the reply
/// (`inbuf[sz - termlen] = '\0'`) *after* asyn has already stripped the `\n\n`
/// terminator, so it always dropped the last two characters of the data. Whether
/// those two characters are digits of the number or trailing unit/status
/// characters is not derivable from the C, and the ND261 manual is not
/// available, so this port does not chop them: `sscanf("%lf")` stops at the
/// first non-numeric character either way, and the chop can only ever have
/// truncated the number. See the module docs.
pub fn parse_position(reply: &str) -> Result<f64, String> {
    if reply.len() < MIN_REPLY_LEN {
        return Err(format!(
            "reply is {} bytes, the ND261 sends at least {MIN_REPLY_LEN}: {reply:?}",
            reply.len()
        ));
    }
    let (sign, rest) = reply.split_at(1);
    let magnitude = scan_f64(rest).ok_or_else(|| format!("reply carries no number: {reply:?}"))?;
    let sign = if sign == "-" { -1.0 } else { 1.0 };
    Ok(sign * magnitude)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_read_command_is_stx_lf() {
        assert_eq!(READ_COMMAND, [0x02, 0x0a]);
    }

    #[test]
    fn the_sign_comes_from_the_first_character() {
        assert_eq!(parse_position("+ 0012.3456 mm").unwrap(), 12.3456);
        assert_eq!(parse_position("- 0012.3456 mm").unwrap(), -12.3456);
        // The C treats every character that is not '-' as a plus sign.
        assert_eq!(parse_position("  0012.3456 mm").unwrap(), 12.3456);
    }

    #[test]
    fn short_and_numberless_replies_are_rejected() {
        assert!(parse_position("+ 12.3").is_err());
        assert!(parse_position("+ no number  ").is_err());
    }
}
