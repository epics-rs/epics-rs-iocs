//! Eurotherm bisync frames (`devXxEurotherm.c`).
//!
//! Both frames the C builds start with the same address preamble
//! (`devXxEurotherm.c:230-236`):
//!
//! ```text
//! EOT  G G  L L
//! ```
//!
//! where `G` is the group address and `L` the local address, each sent twice as
//! an ASCII digit (`'0' + address`).
//!
//! * A value write (`writeAo`) continues with `STX`, the payload — `sprintf` of
//!   the record's format, so the parameter mnemonic is the literal part of that
//!   format (`FMT=SL%4.0lf` → `SL 123`) — then `ETX` and the block check
//!   character, the XOR of everything after the `STX` including the `ETX`
//!   (`devXxEurotherm.c:238-249`).
//! * A read request (`writeSo`) continues with the mnemonic and `ENQ`, with no
//!   `STX` and no checksum (`devXxEurotherm.c:265-277`).
//!
//! The C let the record override the trailing byte with `TERM=`; every database
//! in the module sets it to the default it already has (`TERM=03` = `ETX` on the
//! ao records, `TERM=05` = `ENQ` on the stringout records, `ipApp/Db/Eurotherm.db`),
//! so the port fixes the two terminators instead of carrying the mini-language.

use crate::fmt::format_c_double;

pub const EOT: u8 = 0x04;
pub const STX: u8 = 0x02;
pub const ETX: u8 = 0x03;
pub const ENQ: u8 = 0x05;

/// The largest address the C can send: it writes `'0' + address` as one ASCII
/// character (`devXxEurotherm.c:231-234`).
pub const MAX_ADDRESS: u8 = 9;

fn preamble(group: u8, local: u8) -> Result<Vec<u8>, String> {
    if group > MAX_ADDRESS || local > MAX_ADDRESS {
        return Err(format!(
            "the Eurotherm addresses are single digits: group {group}, local {local}"
        ));
    }
    Ok(vec![
        EOT,
        b'0' + group,
        b'0' + group,
        b'0' + local,
        b'0' + local,
    ])
}

/// Block check character: the XOR of the frame after `STX`, `ETX` included.
pub fn checksum(payload: &[u8]) -> u8 {
    payload.iter().fold(ETX, |bcc, byte| bcc ^ byte)
}

/// `EOT G G L L STX <payload> ETX <BCC>` — the C's `writeAo`. `format` is the
/// record's payload format, e.g. `SL%4.0lf`.
pub fn write_value(group: u8, local: u8, format: &str, value: f64) -> Result<Vec<u8>, String> {
    let payload = format_c_double(format, value)?;
    let mut frame = preamble(group, local)?;
    frame.push(STX);
    frame.extend_from_slice(payload.as_bytes());
    frame.push(ETX);
    frame.push(checksum(payload.as_bytes()));
    Ok(frame)
}

/// `EOT G G L L <mnemonic> ENQ` — the C's `writeSo`.
pub fn read_request(group: u8, local: u8, mnemonic: &str) -> Result<Vec<u8>, String> {
    let mut frame = preamble(group, local)?;
    frame.extend_from_slice(mnemonic.as_bytes());
    frame.push(ENQ);
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_value_write_is_framed_and_checksummed() {
        // Eurotherm.db: field(OUT, "@asyn($(PORT))FMT=SL%4.0lf,TERM=03")
        let frame = write_value(0, 1, "SL%4.0lf", 123.4).unwrap();
        let payload = b"SL 123";
        let mut expected = vec![EOT, b'0', b'0', b'1', b'1', STX];
        expected.extend_from_slice(payload);
        expected.push(ETX);
        expected.push(checksum(payload));
        assert_eq!(frame, expected);

        // The BCC is the XOR of the payload and the ETX, as the C's loop is:
        // for (i=7, checksum=buffer[6]; buffer[i]; i++) checksum ^= buffer[i];
        let bcc = b'S' ^ b'L' ^ b' ' ^ b'1' ^ b'2' ^ b'3' ^ ETX;
        assert_eq!(*frame.last().unwrap(), bcc);
    }

    #[test]
    fn a_read_request_carries_the_mnemonic_and_no_checksum() {
        // Eurotherm.db: stringout VAL "SP", field(OUT, "@asyn($(PORT))TERM=05")
        assert_eq!(
            read_request(0, 0, "SP").unwrap(),
            vec![EOT, b'0', b'0', b'0', b'0', b'S', b'P', ENQ]
        );
    }

    #[test]
    fn the_addresses_are_ascii_digits() {
        let frame = write_value(2, 9, "SL%.0f", 1.0).unwrap();
        assert_eq!(&frame[..5], &[EOT, b'2', b'2', b'9', b'9']);
        assert!(write_value(10, 0, "SL%.0f", 1.0).is_err());
        assert!(read_request(0, 10, "SP").is_err());
    }

    #[test]
    fn a_format_that_does_not_take_a_double_is_rejected() {
        assert!(write_value(0, 0, "SL", 1.0).is_err());
    }
}
