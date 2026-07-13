//! PMAC ethernet packet framing, ported from
//! `pmacApp/pmacAsynIPPortSrc/pmacAsynIPPort.c`.
//!
//! A Turbo PMAC reached over ethernet (TCP port 1025) does not speak raw ASCII:
//! every command is wrapped in an 8-byte Delta Tau "ethernet command" header,
//! and responses are pulled with `VR_PMAC_READREADY` / `VR_PMAC_GETBUFFER`
//! probes. The C module implements this as an **asyn interpose layer** slotted
//! between the EOS layer and `drvAsynIPPort`, so the motor driver above it sees
//! a plain ASCII octet port. asyn-rs has the same interpose concept
//! ([`OctetInterpose`]), so this port keeps the same shape — [`PmacIpInterpose`]
//! is the layer, and [`pmac_asyn_ip_configure_command`] is the C
//! `pmacAsynIPConfigure` that builds the port with the layer installed. The
//! driver in [`crate::controller`] therefore stays transport-agnostic: it works
//! over serial, over plain TCP, and over this framing, exactly as the C
//! `pmacController` does.
//!
//! ## Protocol
//!
//! ```text
//! header:  RequestType:u8  Request:u8  wValue:u16  wIndex:u16  wLength:u16(BE)
//! ```
//!
//! An ASCII command goes out as `VR_DOWNLOAD`/`VR_PMAC_GETRESPONSE` with the
//! command bytes appended; a single control character (ctrl-B/C/F/G/P/V) goes
//! out as `VR_UPLOAD`/`VR_CTRL_RESPONSE` with the character in `wValue` and no
//! payload. Replies arrive in one of three shapes, and the read state machine
//! normalizes all of them to an ACK-terminated message for the EOS layer above:
//!
//! ```text
//! data<CR>data<CR>…data<CR><ACK>     normal (already ACK-terminated)
//! <BELL>ERRxxx<CR>                   error   (ACK appended by this layer)
//! <STX>data<CR>                      ctrl    (ACK appended by this layer)
//! ```
//!
//! ## Requires `I3=2` and `I6=1` on the controller
//!
//! As the C header note says: the PMAC must be configured to terminate
//! responses with ACK and to report errors, or nothing below terminates.
//!
//! ## Byte order
//!
//! C writes the packed struct straight to the socket, so `wValue`/`wIndex` go
//! out in **host** order while `wLength` is explicitly `htons()`-ed to network
//! order. On the little-endian hosts this driver runs on, host order for
//! `wValue` *is* the Delta Tau wire order (the field is little-endian in the
//! protocol, USB-style); this port writes it little-endian explicitly, which is
//! byte-identical on LE and additionally correct on a big-endian host, where
//! the C would put the control character in the wrong byte.

use std::sync::Arc;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::eos::EosInterpose;
use epics_rs::asyn::interpose::{EomReason, OctetInterpose, OctetNext, OctetReadResult};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceManager;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_str_req, req_string};

/// Maximum payload of one ethernet command (C `ETHERNET_DATA_SIZE`).
const ETHERNET_DATA_SIZE: usize = 1492;
/// `+1` so an ACK can always be appended (C `INPUT_SIZE`).
const INPUT_SIZE: usize = ETHERNET_DATA_SIZE + 1;
/// The 8-byte packed header (C `ETHERNET_CMD_HEADER`).
const HEADER_LEN: usize = 8;

const STX: u8 = 0x02;
const ACK: u8 = 0x06;
const BELL: u8 = 0x07;

/// Control characters the PMAC takes as `VR_CTRL_RESPONSE` commands
/// (C `ctrlCommands[]`: ctrl-B, C, F, G, P, V).
const CTRL_COMMANDS: [u8; 6] = [0x02, 0x03, 0x06, 0x07, 0x0E, 0x12];

/// `RequestType` field.
const VR_UPLOAD: u8 = 0xC0;
const VR_DOWNLOAD: u8 = 0x40;

/// `Request` field (only the ones this layer issues; the C module likewise
/// leaves WRITEBUFFER / FWDOWNLOAD / SETMEM / IPADDRESS unimplemented).
const VR_PMAC_FLUSH: u8 = 0xB3;
const VR_PMAC_GETRESPONSE: u8 = 0xBF;
const VR_PMAC_READREADY: u8 = 0xC2;
const VR_CTRL_RESPONSE: u8 = 0xC4;
const VR_PMAC_GETBUFFER: u8 = 0xC5;

/// Build one 8-byte ethernet command header.
fn header(request_type: u8, request: u8, w_value: u16, w_index: u16, w_length: u16) -> [u8; 8] {
    let mut h = [0u8; 8];
    h[0] = request_type;
    h[1] = request;
    h[2..4].copy_from_slice(&w_value.to_le_bytes());
    h[4..6].copy_from_slice(&w_index.to_le_bytes());
    // C: `wLength = htons(n)` — this one field is network (big-endian) order.
    h[6..8].copy_from_slice(&w_length.to_be_bytes());
    h
}

/// Frame an outgoing payload: the ASCII-command form (C `writeIt`, else branch).
/// Returns the bytes to put on the socket. A payload longer than
/// [`ETHERNET_DATA_SIZE`] is truncated, as in C (large transfers would need
/// `VR_PMAC_WRITEBUFFER`, which neither the C module nor this port implements).
fn frame_command(data: &[u8]) -> (Vec<u8>, usize) {
    let n = data.len().min(ETHERNET_DATA_SIZE);
    let mut out = Vec::with_capacity(HEADER_LEN + n);
    out.extend_from_slice(&header(
        VR_DOWNLOAD,
        VR_PMAC_GETRESPONSE,
        0,
        0,
        n as u16, // wLength = htons(numchars)
    ));
    out.extend_from_slice(&data[..n]);
    (out, n)
}

/// Frame a single control character (C `writeIt`, ctrl branch): header only,
/// with the character in `wValue`.
fn frame_ctrl(c: u8) -> Vec<u8> {
    header(VR_UPLOAD, VR_CTRL_RESPONSE, c as u16, 0, 0).to_vec()
}

fn is_timeout(e: &AsynError) -> bool {
    matches!(
        e,
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        }
    )
}

/// A fresh [`AsynUser`] for this layer's own probe I/O, carrying the caller's
/// timeout. `AsynUser` is not `Clone` (it owns a `dyn Any` payload) and
/// [`OctetInterpose::read`] only borrows it immutably, while `next.write`
/// needs `&mut` — so the probes get their own user, which is all the base
/// driver reads from it (the timeout).
fn probe_user(user: &AsynUser) -> AsynUser {
    AsynUser::default().with_timeout(user.timeout)
}

/// The PMAC ethernet framing layer (C `pmacPvt` + its `asynOctet` methods).
pub struct PmacIpInterpose {
    /// Bytes read from the socket but not yet handed to the layer above
    /// (C `inBuf`/`inBufHead`/`inBufTail`).
    in_buf: Vec<u8>,
    head: usize,
    tail: usize,
}

impl Default for PmacIpInterpose {
    fn default() -> Self {
        Self::new()
    }
}

impl PmacIpInterpose {
    pub fn new() -> Self {
        Self {
            in_buf: vec![0u8; INPUT_SIZE],
            head: 0,
            tail: 0,
        }
    }

    /// Ask the PMAC whether it has response data waiting (C `pmacReadReady`).
    /// Returns false on any I/O failure, as C does.
    fn read_ready(&mut self, user: &AsynUser, next: &mut dyn OctetNext) -> bool {
        let mut u = probe_user(user);
        // C sends wLength = htons(2) — the size of the reply it then reads.
        if next
            .write(&mut u, &header(VR_UPLOAD, VR_PMAC_READREADY, 0, 0, 2))
            .is_err()
        {
            return false;
        }
        let mut reply = [0u8; 2];
        match next.read(user, &mut reply) {
            Ok(r) => r.nbytes_transferred == 2 && reply[0] != 0,
            Err(_) => false,
        }
    }

    /// Ask the PMAC to send up to `maxchars` buffered response bytes
    /// (C `sendPmacGetBuffer`).
    fn get_buffer(
        &mut self,
        user: &AsynUser,
        next: &mut dyn OctetNext,
        maxchars: usize,
    ) -> AsynResult<()> {
        let mut u = probe_user(user);
        next.write(
            &mut u,
            &header(
                VR_UPLOAD,
                VR_PMAC_GETBUFFER,
                0,
                0,
                maxchars.min(u16::MAX as usize) as u16,
            ),
        )?;
        Ok(())
    }

    /// Refill [`Self::in_buf`] from the socket (C `readResponse`). If the first
    /// read times out with nothing, poll `READREADY` and, if the PMAC says it
    /// has data, issue a `GETBUFFER` and read once more.
    fn read_response(
        &mut self,
        user: &AsynUser,
        next: &mut dyn OctetNext,
        maxchars: usize,
    ) -> AsynResult<usize> {
        let maxchars = maxchars.min(INPUT_SIZE);
        if maxchars == 0 {
            return Ok(0);
        }

        let mut result = next.read(user, &mut self.in_buf[..maxchars]);
        let mut n = match &result {
            Ok(r) => r.nbytes_transferred,
            Err(_) => 0,
        };

        // C: `status == asynTimeout && thisRead == 0 && pasynUser->timeout > 0`.
        if n == 0
            && result.as_ref().err().is_some_and(is_timeout)
            && !user.timeout.is_zero()
            && self.read_ready(user, next)
        {
            self.get_buffer(user, next, maxchars)?;
            result = next.read(user, &mut self.in_buf[..maxchars]);
            n = match &result {
                Ok(r) => r.nbytes_transferred,
                Err(_) => 0,
            };
        }

        if n > 0 {
            // C: a timeout that still delivered bytes is a success.
            self.tail = 0;
            self.head = n;
            return Ok(n);
        }
        match result {
            Ok(_) => Ok(0),
            Err(e) => Err(e),
        }
    }
}

impl OctetInterpose for PmacIpInterpose {
    /// C `readIt`: drain [`Self::in_buf`] a byte at a time, terminating the
    /// message on ACK/LF, or on the CR that closes a `<BELL>`/`<STX>` reply —
    /// appending an ACK in the latter case so the EOS layer above sees the one
    /// terminator it can be configured with.
    fn read(
        &mut self,
        user: &AsynUser,
        buf: &mut [u8],
        next: &mut dyn OctetNext,
    ) -> AsynResult<OctetReadResult> {
        let maxchars = buf.len();
        if maxchars == 0 {
            return Ok(OctetReadResult {
                nbytes_transferred: 0,
                eom_reason: EomReason::CNT,
            });
        }

        let mut n_read = 0usize;
        let mut bell = false;
        let mut initial_read = true;
        let mut status: AsynResult<()> = Ok(());

        loop {
            if self.tail != self.head {
                let c = self.in_buf[self.tail];
                self.tail += 1;
                buf[n_read] = c;

                if c == BELL || c == STX {
                    bell = true;
                }
                if c == b'\r' && bell {
                    // `<BELL>ERRxxx<CR>` / `<STX>data<CR>`: no ACK is coming, so
                    // synthesize one. If there is no room, the CR is overwritten
                    // by the ACK rather than dropping the terminator (C).
                    n_read += 1;
                    if n_read + 1 > maxchars {
                        buf[n_read - 1] = ACK;
                    } else {
                        buf[n_read] = ACK;
                        n_read += 1;
                    }
                    break;
                }
                if c == ACK || c == b'\n' {
                    // LF terminates too; it is rewritten to ACK so the EOS layer
                    // above only ever needs the one terminator.
                    if c == b'\n' {
                        buf[n_read] = ACK;
                    }
                    n_read += 1;
                    break;
                }
                n_read += 1;
                if n_read >= maxchars {
                    break;
                }
                continue;
            }

            // Buffer empty. On every refill after the first, prompt the PMAC for
            // more of the response before reading (C `readIt`, `!initialRead`).
            if !initial_read && self.read_ready(user, next) {
                self.get_buffer(user, next, maxchars - n_read)?;
            }
            let this_read = match self.read_response(user, next, maxchars - n_read) {
                Ok(n) => n,
                Err(e) => {
                    status = Err(e);
                    0
                }
            };
            initial_read = false;
            if status.is_err() || this_read == 0 {
                break;
            }
        }

        // C returns the partial data alongside the error status; the Rust
        // signature is either/or, so bytes win — a message that terminated
        // properly is a success even if a later probe read timed out.
        if n_read > 0 {
            return Ok(OctetReadResult {
                nbytes_transferred: n_read,
                eom_reason: EomReason::CNT,
            });
        }
        status?;
        Ok(OctetReadResult {
            nbytes_transferred: 0,
            eom_reason: EomReason::empty(),
        })
    }

    /// C `writeIt`.
    fn write(
        &mut self,
        user: &mut AsynUser,
        data: &[u8],
        next: &mut dyn OctetNext,
    ) -> AsynResult<usize> {
        if data.len() == 1 && CTRL_COMMANDS.contains(&data[0]) {
            let packet = frame_ctrl(data[0]);
            let n = next.write(user, &packet)?;
            return Ok(if n == HEADER_LEN { data.len() } else { 0 });
        }
        let (packet, payload) = frame_command(data);
        let n = next.write(user, &packet)?;
        // C reports the payload bytes accepted, not the framed byte count.
        Ok(if n > HEADER_LEN {
            (n - HEADER_LEN).min(payload)
        } else {
            0
        })
    }

    /// C `flushIt`: send `VR_PMAC_FLUSH`, read the one-byte acknowledgement
    /// (whose value C deliberately does not check), drop the input buffer, then
    /// flush the layer below.
    fn flush(&mut self, user: &mut AsynUser, next: &mut dyn OctetNext) -> AsynResult<()> {
        let _ = next.write(user, &header(VR_DOWNLOAD, VR_PMAC_FLUSH, 0, 0, 0));
        let mut ack = [0u8; 1];
        let probe = probe_user(user);
        let _ = next.read(&probe, &mut ack);
        self.tail = 0;
        self.head = 0;
        next.flush(user)
    }
}

/// `pmacAsynIPConfigure(portName, hostInfo)` — C
/// `pmacAsynIPPort.c::pmacAsynIPConfigure`: create the TCP port, install the
/// PMAC framing layer, and put an EOS layer above it.
///
/// The C function is `drvAsynIPPortConfigure(..., noProcessEos=1)` followed by
/// `pmacAsynIPPortConfigureEos`, which interposes the framing layer directly on
/// the driver and then `asynInterposeEosConfig`s an EOS layer above it. This
/// builds the same three-layer stack in one step. The input EOS is left for the
/// driver to set (C `pmacController::lowLevelPortConnect` sets IEOS `\x06`
/// (ACK) and OEOS `\r` on its own asyn user), so a startup script that uses
/// this command must still `asynOctetSetInputEos(port, 0, "\006")` and
/// `asynOctetSetOutputEos(port, 0, "\r")` — the crate's `st.cmd` does.
pub fn pmac_asyn_ip_configure_command(trace: Arc<TraceManager>) -> CommandDef {
    CommandDef::new(
        "pmacAsynIPConfigure",
        vec![arg_str_req("portName"), arg_str_req("hostInfo")],
        "pmacAsynIPConfigure(portName, hostInfo) - Create an octet port to a PMAC over \
         ethernet (host:port, typically <ip>:1025), with the PMAC ethernet packet framing \
         and an EOS layer installed. Requires I3=2 and I6=1 on the controller.",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port = req_string(args, 0, "portName")?;
            let host = req_string(args, 1, "hostInfo")?;

            let mut driver = DrvAsynIPPort::new(&port, &host)
                .map_err(|e| format!("pmacAsynIPConfigure: {e}"))?;
            // Layer 0 is dispatched first (outermost). The EOS layer must see
            // the ACK-terminated stream the framing layer produces, so it goes
            // on first and the framing layer sits between it and the socket —
            // the same order as C (interposeInterface, then
            // asynInterposeEosConfig above it).
            driver.push_interpose(Box::new(EosInterpose::default()));
            driver.push_interpose(Box::new(PmacIpInterpose::new()));

            let (handle, _jh) = create_port_runtime(driver, RuntimeConfig::default());
            epics_rs::asyn::asyn_record::register_port(
                &port,
                handle.port_handle().clone(),
                trace.clone(),
            )
            .map_err(|e| format!("pmacAsynIPConfigure: {e}"))?;
            drop(handle);

            ctx.println(&format!(
                "pmacAsynIPConfigure: PMAC ethernet port '{port}' -> {host}"
            ));
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A base layer that replays a canned byte stream and records writes.
    struct MockBase {
        to_read: Vec<Vec<u8>>,
        written: Vec<Vec<u8>>,
    }

    impl MockBase {
        fn new(reads: &[&[u8]]) -> Self {
            Self {
                to_read: reads.iter().map(|r| r.to_vec()).collect(),
                written: Vec::new(),
            }
        }
    }

    impl OctetNext for MockBase {
        fn read(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<OctetReadResult> {
            if self.to_read.is_empty() {
                return Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "read timeout".into(),
                });
            }
            let chunk = self.to_read.remove(0);
            let n = chunk.len().min(buf.len());
            buf[..n].copy_from_slice(&chunk[..n]);
            Ok(OctetReadResult {
                nbytes_transferred: n,
                eom_reason: EomReason::empty(),
            })
        }

        fn write(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.written.push(data.to_vec());
            Ok(data.len())
        }

        fn flush(&mut self, _user: &mut AsynUser) -> AsynResult<()> {
            Ok(())
        }
    }

    #[test]
    fn command_is_framed_as_getresponse_with_big_endian_length() {
        let (packet, n) = frame_command(b"#1 ? F P");
        assert_eq!(n, 8);
        assert_eq!(
            &packet[..HEADER_LEN],
            &[0x40, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        );
        assert_eq!(&packet[HEADER_LEN..], b"#1 ? F P");
    }

    #[test]
    fn a_long_command_is_truncated_to_the_payload_limit() {
        let long = vec![b'x'; ETHERNET_DATA_SIZE + 10];
        let (packet, n) = frame_command(&long);
        assert_eq!(n, ETHERNET_DATA_SIZE);
        assert_eq!(packet.len(), HEADER_LEN + ETHERNET_DATA_SIZE);
        // wLength still describes the truncated payload.
        assert_eq!(
            u16::from_be_bytes([packet[6], packet[7]]),
            ETHERNET_DATA_SIZE as u16
        );
    }

    #[test]
    fn a_control_character_is_framed_as_ctrl_response_with_no_payload() {
        // ctrl-P (0x0E) — the character rides in wValue, little-endian.
        let packet = frame_ctrl(0x0E);
        assert_eq!(packet, vec![0xC0, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn write_dispatches_ctrl_chars_and_ascii_differently() {
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[]);
        let mut user = AsynUser::default();

        let n = layer.write(&mut user, b"\x02", &mut base).unwrap();
        assert_eq!(n, 1);
        assert_eq!(base.written[0][1], 0xC4); // VR_CTRL_RESPONSE
        assert_eq!(base.written[0].len(), HEADER_LEN);

        let n = layer.write(&mut user, b"#1J+", &mut base).unwrap();
        assert_eq!(n, 4);
        assert_eq!(base.written[1][1], 0xBF); // VR_PMAC_GETRESPONSE
        assert_eq!(&base.written[1][HEADER_LEN..], b"#1J+");
    }

    #[test]
    fn read_returns_an_ack_terminated_reply_unchanged() {
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"100\r-100\r\x06"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 64];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(&buf[..r.nbytes_transferred], b"100\r-100\r\x06");
    }

    #[test]
    fn read_appends_an_ack_to_a_bell_error_reply() {
        // `<BELL>ERR003<CR>` carries no ACK; the layer synthesizes one so the
        // EOS layer above terminates the message.
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"\x07ERR003\r"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 64];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(&buf[..r.nbytes_transferred], b"\x07ERR003\r\x06");
    }

    #[test]
    fn read_appends_an_ack_to_an_stx_reply() {
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"\x02data\r"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 64];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(&buf[..r.nbytes_transferred], b"\x02data\r\x06");
    }

    #[test]
    fn read_rewrites_a_terminating_lf_to_ack() {
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"100\n"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 64];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(&buf[..r.nbytes_transferred], b"100\x06");
    }

    #[test]
    fn read_reassembles_a_reply_split_across_packets() {
        // Second and later refills prompt the PMAC first: READREADY (2-byte
        // reply, first byte non-zero) then GETBUFFER, then the data read.
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[
            b"100\r",      // first read
            b"\x01\x00",   // READREADY says "data waiting"
            b"-100\r\x06", // the rest, after GETBUFFER
        ]);
        let user = AsynUser::default();
        let mut buf = [0u8; 64];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(&buf[..r.nbytes_transferred], b"100\r-100\r\x06");
        // The probes went out as headers: READREADY then GETBUFFER.
        assert_eq!(base.written[0][1], 0xC2);
        assert_eq!(base.written[1][1], 0xC5);
    }

    #[test]
    fn read_stops_at_the_caller_buffer_size() {
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"123456789\r\x06"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 4];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(r.nbytes_transferred, 4);
        assert_eq!(&buf[..4], b"1234");
    }

    #[test]
    fn a_bell_reply_that_fills_the_buffer_overwrites_the_cr_with_ack() {
        // C: "If maxchars is reached overwrite <CR> with ACK, so that no more
        // reads will be done from the EOS layer."
        let mut layer = PmacIpInterpose::new();
        let mut base = MockBase::new(&[b"\x07E\r"]);
        let user = AsynUser::default();
        let mut buf = [0u8; 3];

        let r = layer.read(&user, &mut buf, &mut base).unwrap();
        assert_eq!(r.nbytes_transferred, 3);
        assert_eq!(&buf[..3], b"\x07E\x06");
    }

    #[test]
    fn flush_sends_the_flush_packet_and_drops_buffered_input() {
        let mut layer = PmacIpInterpose::new();
        layer.in_buf[0] = b'x';
        layer.head = 1;
        let mut base = MockBase::new(&[b"\x40"]);
        let mut user = AsynUser::default();

        layer.flush(&mut user, &mut base).unwrap();
        assert_eq!(base.written[0][1], 0xB3); // VR_PMAC_FLUSH
        assert_eq!(layer.head, 0);
        assert_eq!(layer.tail, 0);
    }
}
