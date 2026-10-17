//! Eurotherm bisync frames (`devXxEurotherm.c`).
//!
//! Both frames start with the address preamble `EOT G G L L`, the group and
//! local addresses each sent twice as `'0' + address` (`devXxEurotherm.c:
//! 230-236`).
//!
//! * `writeAo` continues with `STX`, the `sprintf` of the record's format, the
//!   terminator (`ETX` by default) and the block check character, the XOR of
//!   everything after the `STX` (`:238-249`).
//! * `writeSo` continues with the record's VAL from `IX=` and the terminator
//!   (`ENQ` by default), with no `STX` and no checksum (`:265-277`).

use crate::user_param::c_str;

pub const EOT: u8 = 0x04;
pub const STX: u8 = 0x02;
pub const ETX: u8 = 0x03;
pub const ENQ: u8 = 0x05;

/// `'0' + address` into a C `char`.
fn digit(address: i32) -> u8 {
    (i32::from(b'0') + address) as u8
}

fn preamble(group: i32, local: i32) -> Vec<u8> {
    vec![EOT, digit(group), digit(group), digit(local), digit(local)]
}

/// `writeAo`'s frame. The checksum loop stops at `i < 98`, which bounds what
/// after the `STX` is sent to 92 bytes.
pub fn value_frame(group: i32, local: i32, payload: &[u8], term: &[u8]) -> Vec<u8> {
    let mut body = [c_str(payload), c_str(term)].concat();
    body.truncate(92);
    let mut frame = preamble(group, local);
    frame.push(STX);
    // `checksum = buffer[6]`, the payload's first byte, NUL when it is empty.
    let checksum = body.iter().fold(0, |bcc, byte| bcc ^ byte);
    frame.extend_from_slice(&body);
    frame.push(checksum);
    frame
}

/// `writeSo`'s frame: `strncat` of at most the 95 bytes left in `buffer`.
pub fn request_frame(group: i32, local: i32, payload: &[u8], term: &[u8]) -> Vec<u8> {
    let mut frame = preamble(group, local);
    let payload = c_str(payload);
    frame.extend_from_slice(&payload[..payload.len().min(95)]);
    frame.extend_from_slice(c_str(term));
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_value_frame_is_checksummed_over_payload_and_terminator() {
        let frame = value_frame(0, 1, b"SL 123", &[ETX]);
        let mut want = vec![EOT, b'0', b'0', b'1', b'1', STX];
        want.extend_from_slice(b"SL 123");
        want.push(ETX);
        want.push(b'S' ^ b'L' ^ b' ' ^ b'1' ^ b'2' ^ b'3' ^ ETX);
        assert_eq!(frame, want);
    }

    #[test]
    fn a_request_frame_has_no_stx_and_no_checksum() {
        assert_eq!(
            request_frame(0, 0, b"SP", &[ENQ]),
            [EOT, b'0', b'0', b'0', b'0', b'S', b'P', ENQ]
        );
    }

    /// C sends `'0' + address` whatever the address.
    #[test]
    fn addresses_are_sent_as_c_chars() {
        assert_eq!(
            &request_frame(2, 12, b"", b"")[..],
            &[EOT, b'2', b'2', b'<', b'<']
        );
    }

    #[test]
    fn a_long_value_is_cut_where_the_checksum_loop_stops() {
        let frame = value_frame(0, 0, &[b'1'; 120], &[ETX]);
        assert_eq!(frame.len(), 6 + 92 + 1);
    }
}
