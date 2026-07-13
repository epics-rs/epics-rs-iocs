//! Raw TCP framing (`drvGM10_comm.c`): every response starts with a 2-byte
//! `E`+type header (`gm10_response_reader`). `'0'`=OK, `'1'`=ERROR,
//! `'2'`=CHAIN_ERRORS (GM10-only), `'A'`=ASCII (terminated by `"EN\r\n"`),
//! `'B'`=BINARY (8-byte header, 4-byte big-endian length at offset 4, then
//! that many more bytes). Every write is preceded by a 20ms sleep
//! (`gm10_simple_writer`/`gm10_writer`).

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

/// Pure terminator check for the OK/ERROR/CHAIN_ERRORS frame shape:
/// read until the buffer ends in `\r\n` (`gm10_ok_error_reader`).
pub fn is_crlf_terminated(buf: &[u8]) -> bool {
    buf.len() > 1 && buf[buf.len() - 2] == b'\r' && buf[buf.len() - 1] == b'\n'
}

/// Pure terminator check for the ASCII frame shape: read until the buffer
/// ends in `"EN\r\n"` (`gm10_ascii_reader`).
pub fn is_ascii_terminated(buf: &[u8]) -> bool {
    buf.len() > 3 && buf[buf.len() - 4..] == *b"EN\r\n"
}

/// Total byte count of a binary frame (8-byte header + payload), given the
/// first 8 bytes already read. `header[0..2]` must be `"EB"`.
/// (`gm10_binary_reader`: `datalen = ntohl(*(uint32_t*)(inbuffer+4))`,
/// total = `8 + datalen`.)
pub fn binary_frame_total_len(header: &[u8; 8]) -> Option<usize> {
    if &header[0..2] != b"EB" {
        return None;
    }
    let datalen = u32::from_be_bytes([header[4], header[5], header[6], header[7]]);
    Some(8 + datalen as usize)
}

#[derive(Debug, Clone)]
pub enum RawResponse {
    /// `"E0\r\n"` — no further payload.
    Ok,
    /// `"E1,<code>:1:<param>\r\n"` (raw bytes, header included).
    Error(Vec<u8>),
    /// `"E2,...\r\n"` (GM10-only chained-error variant, not decoded further).
    ChainErrors(Vec<u8>),
    /// `"EA..."` up to and including the trailing `"EN\r\n"`.
    Ascii(Vec<u8>),
    /// Full binary frame, header included.
    Binary(Vec<u8>),
}

const MAX_FRAME: usize = 1 << 20;

fn read_until<R: Read>(
    stream: &mut R,
    buf: &mut Vec<u8>,
    done: impl Fn(&[u8]) -> bool,
) -> io::Result<()> {
    let mut chunk = [0u8; 4096];
    while !done(buf) {
        if buf.len() > MAX_FRAME {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "frame too large",
            ));
        }
        let n = stream.read(&mut chunk)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "connection closed",
            ));
        }
        buf.extend_from_slice(&chunk[..n]);
    }
    Ok(())
}

/// Read one full response frame (`gm10_response_reader`).
pub fn read_response<R: Read>(stream: &mut R) -> io::Result<RawResponse> {
    let mut header = [0u8; 2];
    stream.read_exact(&mut header)?;
    if header[0] != b'E' {
        // C: throws away whatever else is in the socket buffer.
        let mut sink = [0u8; 4096];
        let _ = stream.read(&mut sink);
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "response did not start with 'E'",
        ));
    }
    let mut buf = header.to_vec();
    match header[1] {
        b'0' => {
            read_until(stream, &mut buf, is_crlf_terminated)?;
            Ok(RawResponse::Ok)
        }
        b'1' => {
            read_until(stream, &mut buf, is_crlf_terminated)?;
            Ok(RawResponse::Error(buf))
        }
        b'2' => {
            read_until(stream, &mut buf, is_crlf_terminated)?;
            Ok(RawResponse::ChainErrors(buf))
        }
        b'A' => {
            read_until(stream, &mut buf, is_ascii_terminated)?;
            Ok(RawResponse::Ascii(buf))
        }
        b'B' => {
            let mut rest = [0u8; 6];
            stream.read_exact(&mut rest)?;
            buf.extend_from_slice(&rest);
            let mut header8 = [0u8; 8];
            header8.copy_from_slice(&buf[..8]);
            let total = binary_frame_total_len(&header8)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "bad binary header"))?;
            read_until(stream, &mut buf, |b| b.len() >= total)?;
            Ok(RawResponse::Binary(buf))
        }
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unknown response type",
        )),
    }
}

/// Write a command, honoring the mandatory 20ms pre-write delay
/// (`gm10_simple_writer`/`gm10_writer`).
pub fn write_command(stream: &mut TcpStream, command: &str) -> io::Result<()> {
    std::thread::sleep(Duration::from_millis(20));
    stream.write_all(command.as_bytes())
}

/// Strip the ASCII frame's leading 4-byte header (`"EAxx"`) that every
/// `drvGM10.c` consumer skips via `ptr += 4` without inspecting it, and the
/// trailing `"EN\r\n"` terminator. Returns the interior payload as `&str`
/// (lossily, matching C's byte-oriented parsing).
pub fn ascii_payload(raw: &[u8]) -> &[u8] {
    let body = if raw.len() >= 4 { &raw[4..] } else { &[] };
    if body.len() >= 4 && &body[body.len() - 4..] == b"EN\r\n" {
        &body[..body.len() - 4]
    } else {
        body
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crlf_terminator() {
        assert!(!is_crlf_terminated(b"E0"));
        assert!(!is_crlf_terminated(b"E0\r"));
        assert!(is_crlf_terminated(b"E0\r\n"));
    }

    #[test]
    fn ascii_terminator() {
        assert!(!is_ascii_terminated(b"EAxxORec,1\r\n"));
        assert!(is_ascii_terminated(b"EAxxORec,1\r\nEN\r\n"));
    }

    #[test]
    fn binary_header_total_len() {
        let mut header = [0u8; 8];
        header[0] = b'E';
        header[1] = b'B';
        header[4..8].copy_from_slice(&40u32.to_be_bytes());
        assert_eq!(binary_frame_total_len(&header), Some(48));
    }

    #[test]
    fn binary_header_rejects_wrong_type() {
        let mut header = [0u8; 8];
        header[0] = b'E';
        header[1] = b'A';
        assert_eq!(binary_frame_total_len(&header), None);
    }

    #[test]
    fn read_response_dispatches_ok() {
        let mut cursor = io::Cursor::new(b"E0\r\n".to_vec());
        match read_response(&mut cursor).unwrap() {
            RawResponse::Ok => {}
            other => panic!("expected Ok, got {other:?}"),
        }
    }

    #[test]
    fn read_response_dispatches_ascii_and_strips_payload() {
        let mut cursor = io::Cursor::new(b"EAxxORec,1\r\nEN\r\n".to_vec());
        match read_response(&mut cursor).unwrap() {
            RawResponse::Ascii(raw) => {
                assert_eq!(ascii_payload(&raw), b"ORec,1\r\n");
            }
            other => panic!("expected Ascii, got {other:?}"),
        }
    }

    #[test]
    fn read_response_dispatches_binary() {
        let mut payload = vec![b'E', b'B', 0, 0];
        payload.extend_from_slice(&4u32.to_be_bytes());
        payload.extend_from_slice(&[1, 2, 3, 4]);
        let mut cursor = io::Cursor::new(payload.clone());
        match read_response(&mut cursor).unwrap() {
            RawResponse::Binary(raw) => assert_eq!(raw, payload),
            other => panic!("expected Binary, got {other:?}"),
        }
    }

    #[test]
    fn read_response_rejects_non_e_header() {
        let mut cursor = io::Cursor::new(b"XX".to_vec());
        assert!(read_response(&mut cursor).is_err());
    }
}
