//! Heidenhain ND261 display unit protocol (`devAiHeidND261.c`).
//!
//! One transaction: write `^B\n` (`STX` + LF), set the input EOS to `\n\n`,
//! and read the reply the ND261 pushes back (`devAiHeidND261.c:164-168`).
//!
//! The reply is a sign character followed by the number:
//! `pai->val = sscanf(&inbuf[1], "%lf") * (inbuf[0] == '-' ? -1 : 1)`
//! (`devAiHeidND261.c:233-234`).

use epics_rs::base::calc::engine::scanf::sscanf;
use epics_rs::base::calc::engine::value::StackValue;

use crate::user_param::c_str;

/// `^B` then LF — the readout request (`devAiHeidND261.c:104-105`).
pub const READ_COMMAND: [u8; 2] = [0x02, b'\n'];

/// The terminator the ND261 appends (`devAiHeidND261.c:100`).
pub const TERM: &[u8] = b"\n\n";

/// `pPvt->nchar`, the read size (`devAiHeidND261.c:103`).
pub const NCHAR: usize = 18;

/// The C rejected any reply shorter than this (`devAiHeidND261.c:222`).
pub const MIN_REPLY_LEN: usize = 14;

/// The value the C forces into the record when the read fails
/// (`devAiHeidND261.c:223`).
pub const INVALID_VALUE: f64 = 99999.0;

/// Decode one reply (`nread` bytes, EOS stripped) into a position.
///
/// `devAiHeidND261.c:229-231` chops two more characters off the reply
/// (`inbuf[sz - termlen] = '\0'`) *after* asyn has already stripped the `\n\n`
/// terminator, so it always dropped the last two characters of the data. Whether
/// those are digits of the number or trailing unit/status characters is not
/// derivable from the C, and the ND261 manual is not available, so this port
/// does not chop them: `sscanf("%lf")` stops at the first non-numeric character
/// either way, and the chop can only ever have truncated the number.
///
/// C leaves `x` uninitialised when `sscanf` matches nothing and stores that
/// garbage in VAL with no alarm; here a reply with no number is a failed read.
pub fn parse_position(reply: &[u8]) -> Option<f64> {
    if reply.len() < MIN_REPLY_LEN {
        return None;
    }
    let Ok(StackValue::Double(magnitude)) = sscanf(c_str(&reply[1..]), b"%lf") else {
        return None;
    };
    let sign = if reply[0] == b'-' { -1.0 } else { 1.0 };
    Some(sign * magnitude)
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
        assert_eq!(parse_position(b"+ 0012.3456 mm"), Some(12.3456));
        assert_eq!(parse_position(b"- 0012.3456 mm"), Some(-12.3456));
        // The C treats every character that is not '-' as a plus sign.
        assert_eq!(parse_position(b"  0012.3456 mm"), Some(12.3456));
    }

    #[test]
    fn short_and_numberless_replies_are_rejected() {
        assert_eq!(parse_position(b"+ 12.3"), None);
        assert_eq!(parse_position(b"+ no number here"), None);
        assert_eq!(parse_position(b"+ \xff\xfe number here"), None);
    }
}
