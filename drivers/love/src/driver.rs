//! Love PID controller port driver, ported from `drvLove.c`.
//!
//! `drvLove` implements `asynInt32`/`asynUInt32Digital` *directly* — records
//! bind to it via standard asyn generic device support (DTYP `asynInt32`
//! for `longin`/`longout`, DTYP `asynUInt32Digital` with `@asynMask(...)`
//! for `mbbi`/`bi`) — and does its own RS-485 ASCII/checksum framing
//! against a *separate*, underlying octet port (found the same way the
//! delaygen drivers find theirs: `crate::connect::init_serial_port` looks up
//! an already-configured serial/IP port, driven by blocking requests).
//!
//! # EOS ownership
//! `setDefaultEos` (`drvLove.c:640-660`) is called unconditionally at the
//! end of `drvLoveInit`, hardcoding input EOS `\006` (ACK) and output EOS
//! `\003` (ETX) on the underlying serial port — driver-owned, not
//! iocsh-script-owned (unlike DG645/Colby/CoherentSDG in `delaygen`, where
//! the startup `.cmd` fragment sets EOS; neither `st.cmd.linux` nor
//! `love.iocsh` ever call `asynOctetSetInputEos`/`asynOctetSetOutputEos`
//! for Love). [`INPUT_EOS`]/[`OUTPUT_EOS`] carry the fixed bytes;
//! `crate::connect::set_default_eos` applies them after the port is
//! registered, as C does.
//!
//! # Command table
//! `COMMANDS` transcribes `CmdTable[]` (`drvLove.c:255-270`) verbatim: 12
//! commands, each with a read conversion, an optional model-dependent write
//! command, and per-model (1600 / 16A) read/write strings.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **Outgoing checksum is space-padded, not zero-padded**: C `sprintf`
//!   uses `"%2X"` (no `0` flag) for the trailing checksum byte, so a
//!   checksum `< 0x10` is sent as e.g. `" 3"` (space, digit) rather than
//!   `"03"`. Rust's default numeric fill is also space for `{:2X}`, so
//!   `build_frame` reproduces this without special-casing it.
//! - **A read/write timeout resends the *same* frame and re-reads**, up to
//!   3 total attempts, sleeping `K_TUNE` (0.1s) before each — including
//!   re-sending after a *read* timeout, not just a write timeout (C
//!   `executeCommand`, `drvLove.c:722-769`, calls `sendCommand` again every
//!   loop iteration; only `retry==0` rebuilds the frame). A non-timeout
//!   error (checksum failure, malformed reply) aborts immediately with no
//!   retry.
//! - **A 7-byte reply is always a device error reply** (`evalMessage`,
//!   `drvLove.c:565-623): no checksum is validated in this branch at all,
//!   and the 2-digit error code is decoded purely for a trace-log message
//!   (see [`wire::ERR_CODES`]) -- it is never wire-observable at an EPICS
//!   record, since the caller always treats a 7-byte reply as an error
//!   regardless of the code's value.
//! - **The write-response check parses only the first 2 payload bytes**
//!   (`processWriteResponse`, `drvLove.c:772-787`) as a decimal code;
//!   nonzero is a failure. The protocol guarantees at least 2 payload bytes
//!   whenever `eval_message` succeeds (`payload_len = pcount-6 >= 2` since
//!   the success branch requires `pcount >= 8`).
//! - **`getSignedValue`'s sign field is parsed differently per model**
//!   (`drvLove.c:906-931`): decimal (`%2d`, nonzero -> negate) for model
//!   1600, hex (`%2x`, bit 0 -> negate) for model 16A. Both this AND the
//!   read/write *command strings* are model-dependent -- see `COMMANDS`.
//! - **A negative write value is negated via `wrapping_neg`**, not plain
//!   `-value` (`putData`, `drvLove.c:949-968`: `*value *= -1`) -- C's
//!   negation of `INT32_MIN` is signed-integer-overflow UB; on every real
//!   twos-complement target it silently wraps back to `INT32_MIN`, which
//!   `wrapping_neg` reproduces as defined behavior instead of a Rust debug
//!   panic. Not reachable by any real PID setpoint.
//! - **`drv_user_create` bounds-checks `addr` against `1..=K_INSTRMAX`**
//!   before use. C's `create()` (`drvLove.c:1106-1138`) computes
//!   `&pport->instr[addr-1]` with *no* range check at all -- an
//!   out-of-range `addr` (0, negative, or `> K_INSTRMAX`) is an
//!   out-of-bounds C array access (undefined behavior). This port makes it
//!   a defined `AsynError::AddressOutOfRange`, the same choice already made
//!   for equivalent latent-UB addr paths in `delaygen::coherent_sdg`.
//! - **The model used to pick a command's read/write string is resolved
//!   fresh on every read/write call** (via `user.addr` and the shared
//!   [`registry`](crate::registry) table), not cached once at `drvUserCreate`/bind time
//!   the way C caches `pinst->pcmd` in the created `Inst`. asyn-rs
//!   0.22.1's `DrvUserInfo` has no field to carry that kind of per-record
//!   extra state (only `reason`/`max_octet_len`) -- see the `registry`
//!   module doc. This is a non-observable difference under the module's
//!   own documented required usage ("every controller must be configured
//!   prior to IOC initialization"), which every shipped startup script
//!   (`st.cmd.linux`, `love.iocsh`) already follows.

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::registry::{K_INSTRMAX, Model, ModelTable};
use crate::wire::{EvalError, calc_checksum, eval_message, field, parse_dec, parse_hex};

/// C `#define K_TUNE (0.1)` -- sleep before every send attempt, including
/// the first.
const K_TUNE: Duration = Duration::from_millis(100);

/// C `executeCommand`'s `for(i=0;i<3;++i)` retry loop.
const MAX_ATTEMPTS: u32 = 3;

/// asyn reason used for octet transactions against the underlying serial
/// port (mirrors `delaygen::wire::OCTET_REASON`).
const OCTET_REASON: usize = 0;

/// C `pport->inpMsg[20]` -- the fixed reply buffer size passed as
/// `maxchars` to `pasynOctet->read`.
const REPLY_BUF_SIZE: usize = 20;

/// C `setDefaultEos`'s hardcoded input EOS: `'\006'` (ACK).
pub const INPUT_EOS: &[u8] = &[0x06];

/// C `setDefaultEos`'s hardcoded output EOS: `'\003'` (ETX).
pub const OUTPUT_EOS: &[u8] = &[0x03];

/// C `CmdTable[].read`/`.write` conversion, keyed by command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReadConv {
    /// `getValue`: 4 hex sign-word + 4 decimal data digits.
    Value,
    /// `getStatus`: 4 hex status word, no sign/data split.
    Status,
    /// `getSignedValue`: 2-byte sign field (decimal for 1600, hex for 16A)
    /// + 4 decimal data digits.
    SignedValue,
    /// `getData`: 2 hex digits, no sign.
    Data,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WriteConv {
    /// `doNull`: always fails, no I/O.
    Null,
    /// `putData`: sign-magnitude encoded write.
    Data,
}

/// C `CmdTbl` row (`CmdTable[]`, `drvLove.c:255-270`). `read`/`write` are
/// indexed by [`Model`] (`Model1600 as usize` / `Model16A as usize`).
struct CommandSpec {
    name: &'static str,
    read_conv: ReadConv,
    write_conv: WriteConv,
    read: [&'static str; 2],
    write: [Option<&'static str>; 2],
}

const COMMANDS: &[CommandSpec] = &[
    CommandSpec {
        name: "Value",
        read_conv: ReadConv::Value,
        write_conv: WriteConv::Null,
        read: ["00", "00"],
        write: [None, None],
    },
    CommandSpec {
        name: "SP1",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Data,
        read: ["0100", "0101"],
        write: [Some("0200"), Some("0200")],
    },
    CommandSpec {
        name: "SP2",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Data,
        read: ["0102", "0105"],
        write: [Some("0202"), Some("0204")],
    },
    CommandSpec {
        name: "AlLo",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Data,
        read: ["0104", "0106"],
        write: [Some("0204"), Some("0207")],
    },
    CommandSpec {
        name: "AlHi",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Data,
        read: ["0105", "0107"],
        write: [Some("0205"), Some("0208")],
    },
    CommandSpec {
        name: "Peak",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Null,
        read: ["011A", "011D"],
        write: [None, None],
    },
    CommandSpec {
        name: "Valley",
        read_conv: ReadConv::SignedValue,
        write_conv: WriteConv::Null,
        read: ["011B", "011E"],
        write: [None, None],
    },
    CommandSpec {
        name: "AlSts",
        read_conv: ReadConv::Status,
        write_conv: WriteConv::Null,
        read: ["00", "00"],
        write: [None, None],
    },
    CommandSpec {
        name: "AlMode",
        read_conv: ReadConv::Data,
        write_conv: WriteConv::Null,
        read: ["0337", "031D"],
        write: [None, None],
    },
    CommandSpec {
        name: "InpTyp",
        read_conv: ReadConv::Data,
        write_conv: WriteConv::Null,
        read: ["0323", "0317"],
        write: [None, None],
    },
    CommandSpec {
        name: "ComSts",
        read_conv: ReadConv::Data,
        write_conv: WriteConv::Null,
        read: ["032A", "0324"],
        write: [None, None],
    },
    CommandSpec {
        name: "Decpts",
        read_conv: ReadConv::Data,
        write_conv: WriteConv::Null,
        read: ["0324", "031A"],
        write: [None, None],
    },
];

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// True for a timeout, including one `AsynError::PartialRead` wraps to carry
/// the bytes transferred before it. A bare `AsynError::Status` variant match
/// would miss that wrapper and misclassify every partial-transfer timeout —
/// routine on this port, since it installs the EOS interpose — as an
/// unexpected error.
fn is_timeout(e: &AsynError) -> bool {
    e.status() == AsynStatus::Timeout
}

/// C `sendCommand`'s `retry==0` framing: `STX 'L' ADDR(2 hex) BODY
/// CHECKSUM(2 hex, space-padded)`.
fn build_frame(addr: i32, body: &str) -> String {
    let tmp = format!("{addr:02X}{body}");
    let cs = calc_checksum(tmp.as_bytes());
    format!("\u{2}L{tmp}{cs:2X}")
}

/// C `getValue`.
fn parse_value(payload: &[u8]) -> i32 {
    let sign = parse_hex(field(payload, 0, 4));
    let mut value = parse_dec(field(payload, 4, 4));
    if sign & 0x0001 != 0 {
        value = -value;
    }
    value
}

/// C `getStatus`.
fn parse_status(payload: &[u8]) -> i32 {
    parse_hex(field(payload, 0, 4)) as i32
}

/// C `getSignedValue`.
fn parse_signed_value(payload: &[u8], model: Model) -> i32 {
    let info = field(payload, 0, 2);
    let mut value = parse_dec(field(payload, 2, 4));
    let negate = match model {
        Model::Model1600 => parse_dec(info) != 0,
        Model::Model16A => parse_hex(info) & 0x0001 != 0,
    };
    if negate {
        value = -value;
    }
    value
}

/// C `getData`.
fn parse_data(payload: &[u8]) -> i32 {
    parse_hex(field(payload, 0, 2)) as i32
}

/// C `putData`: sign-magnitude, `%4.4d` decimal + `%2.2X` hex sign byte.
fn format_put_data(write_cmd: &str, value: i32) -> String {
    let (sign, data): (u8, i32) = if value < 0 {
        (0xFF, value.wrapping_neg())
    } else {
        (0x00, value)
    };
    format!("{write_cmd}{data:04}{sign:02X}")
}

/// C `processWriteResponse`.
fn process_write_response(payload: &[u8]) -> AsynResult<()> {
    if parse_dec(field(payload, 0, 2)) != 0 {
        return Err(protocol_error("write command failed"));
    }
    Ok(())
}

/// The tail C `sendCommand`/`recvReply` print after a failed transfer.
fn status_text(status: AsynStatus) -> &'static str {
    match status {
        AsynStatus::Timeout => "asynTimeout",
        AsynStatus::Overflow => "asynOverflow",
        AsynStatus::Error => "asynError",
        _ => "failed - unknown Asyn error",
    }
}

/// C `evalMessage`'s `ASYN_TRACE_ERROR` line for each refusal.
fn eval_error_line(e: &EvalError, count: usize) -> String {
    match e {
        EvalError::MissingStx => "drvLove::evalMessage start char missing\n".into(),
        EvalError::TooShort => format!("drvLove::evalMessage message length ({count}) error\n"),
        EvalError::DeviceError(text) => {
            format!("drvLove::evalMessage error message received \"{text}\"\n")
        }
        EvalError::ChecksumMismatch => "drvLove::evalMessage checksum failed\n".into(),
    }
}

fn eval_error_to_asyn(e: EvalError) -> AsynError {
    match e {
        EvalError::MissingStx => protocol_error("evalMessage start char missing"),
        EvalError::TooShort => protocol_error("evalMessage message length error"),
        EvalError::DeviceError(text) => {
            protocol_error(format!("error message received \"{text}\""))
        }
        EvalError::ChecksumMismatch => protocol_error("evalMessage checksum failed"),
    }
}

/// Love port driver state (C `struct Port`, minus the `Serport`/asyn
/// registration bookkeeping the Rust runtime already owns).
pub struct LoveDriver {
    base: PortDriverBase,
    /// C `Serport::name`, for `reportIt`.
    ser_name: String,
    /// C `Serport::pasynUser`: the serial port, its address and timeout.
    serial: PortHandle,
    ser_addr: i32,
    timeout: Duration,
    models: ModelTable,
    /// C `Instr::isConn`: set by `connectIt`, cleared by `disconnectIt`.
    is_conn: [bool; K_INSTRMAX],
}

impl LoveDriver {
    /// C `drvLoveInit(lovPort,serPort,serAddr)`. Requests go to `serial` at
    /// `ser_addr` with `timeout`; the serial port gets [`INPUT_EOS`]/[`OUTPUT_EOS`] after this port is
    /// registered (`crate::connect::set_default_eos`). `models` is the shared per-address model table (see the
    /// `registry` module doc) -- the caller constructs it, passes a clone
    /// here, and registers another clone under `port_name` for a later
    /// `drvLoveConfig` call to find.
    pub fn new(
        port_name: &str,
        ser_name: &str,
        serial: PortHandle,
        ser_addr: i32,
        timeout: Duration,
        models: ModelTable,
    ) -> AsynResult<Self> {
        // C registers the love port with the serial port's canBlock and
        // autoConnect (drvLove.c:363-364).
        let mut base = PortDriverBase::new(
            port_name,
            K_INSTRMAX + 1,
            PortFlags {
                multi_device: true,
                can_block: serial.can_block(),
                destructible: true,
            },
        );
        base.set_auto_connect(serial.connect_device(ser_addr).auto_connect());

        Ok(Self {
            base,
            ser_name: ser_name.to_owned(),
            serial,
            ser_addr,
            timeout,
            models,
            is_conn: [false; K_INSTRMAX],
        })
    }

    /// C `sendCommand` + `recvReply`, looped by C `executeCommand`, with
    /// their `ASYN_TRACE_ERROR` lines (drvLove.c:573-608, :736-767, :817-866).
    fn execute_command(&self, addr: i32, body: &str) -> AsynResult<Vec<u8>> {
        let frame = build_frame(addr, body);

        for retry in 0..MAX_ATTEMPTS {
            std::thread::sleep(K_TUNE);

            let write = RequestOp::OctetWrite {
                data: frame.as_bytes().to_vec(),
            };
            if let Err(e) = self.serial.submit_blocking(write, self.serial_user()) {
                self.trace_error(&format!(
                    "drvLove::sendCommand - retries({retry}) {}\n",
                    status_text(e.status())
                ));
                if is_timeout(&e) {
                    self.trace_error("drvLove::executeCommand write timeout, retrying\n");
                    continue;
                }
                self.trace_error(&format!(
                    "drvLove::executeCommand write failure - Sent \"{frame}\" \n"
                ));
                return Err(e);
            }

            let read = RequestOp::OctetRead {
                buf_size: REPLY_BUF_SIZE,
            };
            let raw = match self.serial.submit_blocking(read, self.serial_user()) {
                Ok(result) => {
                    let raw = result.data.unwrap_or_default();
                    if EomReason::from_bits_truncate(result.eom_reason) != EomReason::EOS {
                        self.trace_error("drvLove::recvReply eos failure\n");
                        self.trace_read_failure(&frame, &raw);
                        return Err(protocol_error("recvReply eos failure"));
                    }
                    raw
                }
                Err(e) => {
                    self.trace_error(&format!("drvLove::recvReply {}\n", status_text(e.status())));
                    if is_timeout(&e) {
                        self.trace_error("drvLove::executeCommand read timeout, retrying\n");
                        continue;
                    }
                    // C prints whatever the failed read left in inpMsg.
                    let rcvd = e.partial_read().map(|p| p.data.as_slice()).unwrap_or(&[]);
                    self.trace_read_failure(&frame, rcvd);
                    return Err(e);
                }
            };

            return match eval_message(&raw) {
                Ok(payload) => Ok(payload.to_vec()),
                Err(e) => {
                    self.trace_error(&eval_error_line(&e, raw.len()));
                    self.trace_read_failure(&frame, &raw);
                    Err(eval_error_to_asyn(e))
                }
            };
        }

        self.trace_error("drvLove::executeCommand retries exceeded\n");
        // C returns asynError, not the last attempt's asynTimeout, so the
        // record alarms READ/WRITE rather than TIMEOUT.
        Err(protocol_error("executeCommand retries exceeded"))
    }

    /// What C's `pasynOctetSyncIO` would queue for `Serport::pasynUser`.
    fn serial_user(&self) -> AsynUser {
        AsynUser::new(OCTET_REASON)
            .with_addr(self.ser_addr)
            .with_timeout(self.timeout)
            .locking_port(&self.serial)
    }

    fn trace_error(&self, msg: &str) {
        self.base
            .trace_print(TraceMask::ERROR, format_args!("{msg}"));
    }

    fn trace_read_failure(&self, frame: &str, rcvd: &[u8]) {
        self.trace_error(&format!(
            "drvLove::executeCommand read failure - Sent \"{frame}\" Rcvd \"{}\" \n",
            String::from_utf8_lossy(rcvd)
        ));
    }

    /// C `connectIt` (drvLove.c:993-1044).
    fn connect_it(&mut self, user: &AsynUser) -> AsynResult<()> {
        let addr = user.addr;
        if !self.serial.is_connected_blocking(self.ser_addr)? {
            return Err(protocol_error(format!(
                "port {} not connected to {}",
                self.base.port_name, self.ser_name
            )));
        }
        self.check_addr("connectIt", addr)?;
        if addr > 0 {
            if self.is_conn[(addr - 1) as usize] {
                self.trace_error(&format!(
                    "drvLove::connectIt {} device {addr} already connected\n",
                    self.base.port_name
                ));
                return Err(protocol_error("already connected"));
            }
            self.is_conn[(addr - 1) as usize] = true;
        }
        self.base.set_addr_connected(addr, true);
        Ok(())
    }

    /// C `disconnectIt` (drvLove.c:1047-1095).
    fn disconnect_it(&mut self, user: &AsynUser) -> AsynResult<()> {
        let addr = user.addr;
        self.check_addr("disconnectIt", addr)?;
        let conn = if addr < 0 {
            self.base.dp_connected(addr)
        } else {
            std::mem::take(&mut self.is_conn[(addr - 1) as usize])
        };
        if !conn {
            return Err(protocol_error("not connected"));
        }
        self.base.set_addr_connected(addr, false);
        Ok(())
    }

    fn check_addr(&self, func: &str, addr: i32) -> AsynResult<()> {
        if addr == 0 || addr > K_INSTRMAX as i32 {
            self.trace_error(&format!(
                "drvLove::{func} {} illegal addr {addr}\n",
                self.base.port_name
            ));
            return Err(protocol_error(format!("illegal addr {addr}")));
        }
        Ok(())
    }

    fn model_for(&self, addr: i32) -> Model {
        self.models.lock().unwrap()[(addr - 1) as usize]
    }

    /// C's `"%s error %s"` with `pport->pasynUser->errorMessage` (drvLove.c:1192-1325).
    /// That user is the love port's own, which nothing writes, so the serial
    /// request's text never reaches the record.
    fn record_error(&self, e: AsynError) -> AsynError {
        AsynError::Status {
            status: e.status(),
            message: format!("{} error ", self.base.port_name),
        }
    }

    fn do_read(&mut self, addr: i32, reason: usize) -> AsynResult<i32> {
        if !(1..=K_INSTRMAX as i32).contains(&addr) {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        let spec = &COMMANDS[reason];
        let model = self.model_for(addr);
        let read_cmd = spec.read[model as usize];

        let payload = self.execute_command(addr, read_cmd)?;
        Ok(match spec.read_conv {
            ReadConv::Value => parse_value(&payload),
            ReadConv::Status => parse_status(&payload),
            ReadConv::SignedValue => parse_signed_value(&payload, model),
            ReadConv::Data => parse_data(&payload),
        })
    }

    fn do_write(&mut self, addr: i32, reason: usize, value: i32) -> AsynResult<()> {
        if !(1..=K_INSTRMAX as i32).contains(&addr) {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        let spec = &COMMANDS[reason];
        let write_cmd = match spec.write_conv {
            WriteConv::Null => {
                return Err(protocol_error(format!(
                    "{} does not support write",
                    spec.name
                )));
            }
            WriteConv::Data => {
                let model = self.model_for(addr);
                spec.write[model as usize]
                    .expect("every WriteConv::Data row has a write string for both models")
            }
        };

        let body = format_put_data(write_cmd, value);
        let payload = self.execute_command(addr, &body)?;
        process_write_response(&payload).inspect_err(|_| {
            self.trace_error("drvLove::processWriteResponse write command failed\n")
        })
    }
}

impl PortDriver for LoveDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn connect(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.connect_it(user)
    }

    fn connect_addr(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.connect_it(user)
    }

    fn disconnect(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.disconnect_it(user)
    }

    fn disconnect_addr(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.disconnect_it(user)
    }

    /// C `reportIt` (drvLove.c:978-990).
    fn report(&mut self, out: &mut dyn std::fmt::Write, _level: i32) {
        let _ = writeln!(
            out,
            "    {} is connected to {}",
            self.base.port_name, self.ser_name
        );
        for (i, &conn) in self.is_conn.iter().enumerate() {
            if conn {
                let _ = writeln!(out, "        Addr {} is connected", i + 1);
            }
        }
    }

    /// C `create()` (asynDrvUser, `drvLove.c:1106-1138`).
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        let (drv_info, addr) = (req.drv_info.as_str(), req.addr);
        if !(1..=K_INSTRMAX as i32).contains(&addr) {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        for (i, spec) in COMMANDS.iter().enumerate() {
            if spec.name.eq_ignore_ascii_case(drv_info) {
                return Ok(DrvUserInfo::from_reason(i));
            }
        }
        Err(protocol_error(format!(
            "failure to find command {drv_info}"
        )))
    }

    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        self.do_read(user.addr, user.reason)
            .map_err(|e| self.record_error(e))
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.do_write(user.addr, user.reason, value)
            .map_err(|e| self.record_error(e))
    }

    /// C `readUInt32` (`drvLove.c:1296-1333`) -- the `mask` argument is
    /// accepted but never applied by the C driver's own body (only used in
    /// a trace-log format string); the mask is applied here to match the
    /// asyn-rs convention already used by `delaygen::coherent_sdg` (every
    /// `read_uint32_digital` override masks its own return value), which is
    /// harmless here since the record layer re-applies (and shifts by) the
    /// same mask regardless.
    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        let value = self
            .do_read(user.addr, user.reason)
            .map_err(|e| self.record_error(e))?;
        Ok((value as u32) & mask)
    }

    /// C `writeUInt32` (`drvLove.c:1260-1293`): `mask` is accepted but
    /// never applied -- `value` passes straight through to the same write
    /// path as `writeInt32`.
    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        _mask: u32,
    ) -> AsynResult<()> {
        self.do_write(user.addr, user.reason, value as i32)
            .map_err(|e| self.record_error(e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::sync_io::SyncIOHandle;

    #[test]
    fn command_table_names_and_strings_match_c_source() {
        assert_eq!(COMMANDS.len(), 12);
        let expected = [
            ("Value", ["00", "00"], [None, None]),
            ("SP1", ["0100", "0101"], [Some("0200"), Some("0200")]),
            ("SP2", ["0102", "0105"], [Some("0202"), Some("0204")]),
            ("AlLo", ["0104", "0106"], [Some("0204"), Some("0207")]),
            ("AlHi", ["0105", "0107"], [Some("0205"), Some("0208")]),
            ("Peak", ["011A", "011D"], [None, None]),
            ("Valley", ["011B", "011E"], [None, None]),
            ("AlSts", ["00", "00"], [None, None]),
            ("AlMode", ["0337", "031D"], [None, None]),
            ("InpTyp", ["0323", "0317"], [None, None]),
            ("ComSts", ["032A", "0324"], [None, None]),
            ("Decpts", ["0324", "031A"], [None, None]),
        ];
        for (spec, (name, read, write)) in COMMANDS.iter().zip(expected.iter()) {
            assert_eq!(spec.name, *name);
            assert_eq!(spec.read, *read);
            assert_eq!(spec.write, *write);
        }
    }

    #[test]
    fn build_frame_matches_c_sendcommand_layout() {
        // addr=1, body="0100" -> tmp="010100", checksum = sum('0'+'1'+'0'+'1'+'0'+'0').
        let frame = build_frame(1, "0100");
        let tmp = "010100";
        let cs = calc_checksum(tmp.as_bytes());
        assert_eq!(frame, format!("\u{2}L{tmp}{cs:2X}"));
        assert!(frame.starts_with("\u{2}L01"));
    }

    #[test]
    fn parse_value_applies_sign_from_stat_field() {
        // stat="0001" (bit0 set -> negate), data="0042".
        assert_eq!(parse_value(b"00010042"), -42);
        assert_eq!(parse_value(b"00000042"), 42);
    }

    #[test]
    fn parse_status_returns_raw_stat_word() {
        assert_eq!(parse_status(b"0F0A0000"), 0x0F0A);
    }

    #[test]
    fn parse_signed_value_sign_field_is_model_dependent() {
        // model 1600: info parsed as DECIMAL, nonzero -> negate.
        assert_eq!(parse_signed_value(b"010042", Model::Model1600), -42);
        assert_eq!(parse_signed_value(b"000042", Model::Model1600), 42);
        // model 16A: info parsed as HEX, bit0 -> negate.
        assert_eq!(parse_signed_value(b"010042", Model::Model16A), -42);
        assert_eq!(parse_signed_value(b"020042", Model::Model16A), 42);
    }

    #[test]
    fn parse_data_returns_raw_hex_word() {
        assert_eq!(parse_data(b"3701"), 0x37);
    }

    #[test]
    fn format_put_data_is_sign_magnitude_zero_padded() {
        assert_eq!(format_put_data("0200", 42), "0200004200");
        assert_eq!(format_put_data("0200", -42), "02000042FF");
    }

    #[test]
    fn format_put_data_does_not_panic_on_i32_min() {
        // C `*value *= -1` on INT32_MIN is signed-overflow UB; wrapping_neg
        // keeps this a defined (if nonsensical) no-op rather than a Rust
        // debug-build panic. Not reachable by any real PID setpoint.
        let out = format_put_data("0200", i32::MIN);
        assert!(out.starts_with("0200-2147483648"));
    }

    #[test]
    fn process_write_response_fails_on_nonzero_code() {
        assert!(process_write_response(b"00").is_ok());
        assert!(process_write_response(b"01xyz").is_err());
    }

    /// `AsynError::PartialRead` is how `asyn-rs` 0.24+ carries a timed-out
    /// read's transferred bytes (see the fix note on `is_timeout`); a bare
    /// `AsynError::Status` match would miss it entirely.
    #[test]
    fn is_timeout_recognizes_a_partial_read_wrapped_timeout() {
        use epics_rs::asyn::interpose::{EomReason, PartialOctetRead};

        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(PartialOctetRead {
            data: b"\x02L01".to_vec(),
            eom_reason: EomReason::empty(),
        });
        assert!(is_timeout(&e));
        assert_eq!(
            e.partial_read().map(|p| p.data.as_slice()),
            Some(&b"\x02L01"[..])
        );
    }

    #[test]
    fn is_timeout_rejects_a_real_error() {
        assert!(!is_timeout(&protocol_error("boom")));
    }

    /// A serial port whose every write times out.
    struct TimeoutSerial {
        base: PortDriverBase,
    }

    impl PortDriver for TimeoutSerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_octet(&mut self, _user: &mut AsynUser, _data: &[u8]) -> AsynResult<usize> {
            Err(AsynError::Status {
                status: AsynStatus::Timeout,
                message: "write timeout".into(),
            })
        }
    }

    fn love_on(serial: impl PortDriver, name: &str) -> LoveDriver {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let claim = claim_port(&serial.base().port_name.clone()).expect("serial port name");
        let (runtime, _thread) = create_port_runtime(&claim, serial, RuntimeConfig::default())
            .expect("serial port runtime");
        let models = std::sync::Arc::new(std::sync::Mutex::new([Model::default(); K_INSTRMAX]));
        let serial = runtime.port_handle().clone();
        LoveDriver::new(name, "SER", serial, 0, Duration::from_millis(100), models)
            .expect("love driver")
    }

    /// Every timed-out attempt prints C's sendCommand and executeCommand
    /// lines, and the give-up prints "retries exceeded" (drvLove.c:739-767).
    #[test]
    fn timed_out_attempts_print_c_trace_lines() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::TraceFile;

        let serial = TimeoutSerial {
            base: PortDriverBase::new(
                "LOVETESTTRC",
                1,
                PortFlags {
                    multi_device: false,
                    can_block: true,
                    destructible: true,
                },
            ),
        };
        let love = love_on(serial, "LOVETESTTRCL");
        let claim = claim_port("LOVETESTTRCL").expect("love port name");
        let (runtime, _thread) =
            create_port_runtime(&claim, love, RuntimeConfig::default()).expect("love port runtime");
        let path = std::env::temp_dir().join(format!("love-trace-{}.txt", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        runtime.port_handle().trace().set_trace_file(
            Some("LOVETESTTRCL"),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        let user =
            SyncIOHandle::from_handle(runtime.port_handle().clone(), 1, Duration::from_secs(5));
        assert!(user.read_int32(0).is_err());

        let out = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for line in [
            "drvLove::sendCommand - retries(0) asynTimeout",
            "drvLove::sendCommand - retries(2) asynTimeout",
            "drvLove::executeCommand write timeout, retrying",
            "drvLove::executeCommand retries exceeded",
        ] {
            assert!(out.contains(line), "missing {line:?} in {out:?}");
        }
    }

    /// C registers the love port with the serial port's canBlock and
    /// autoConnect (drvLove.c:363-364).
    #[test]
    fn the_love_port_takes_can_block_and_auto_connect_from_the_serial_port() {
        for (can_block, auto_connect) in [(true, true), (false, false)] {
            let name = format!("LOVETESTFLG{}", u8::from(can_block));
            let mut base = PortDriverBase::new(
                &name,
                1,
                PortFlags {
                    multi_device: false,
                    can_block,
                    destructible: true,
                },
            );
            base.set_auto_connect(auto_connect);
            let love = love_on(TimeoutSerial { base }, &format!("{name}L"));
            assert_eq!(love.base().flags.can_block, can_block);
            assert_eq!(love.base().is_auto_connect(), auto_connect);
        }
    }

    /// C `connectIt` refuses while the serial port is down, so the love port
    /// never comes up and a request is refused at the queue without a
    /// command reaching the serial line (drvLove.c:1003-1014).
    #[test]
    fn a_down_serial_port_keeps_the_love_port_down() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::TraceFile;

        let mut base = PortDriverBase::new(
            "LOVETESTDWN",
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        base.set_auto_connect(false);
        base.init_connected(false);
        let love = love_on(TimeoutSerial { base }, "LOVETESTDWNL");
        let claim = claim_port("LOVETESTDWNL").expect("love port name");
        let (runtime, _thread) =
            create_port_runtime(&claim, love, RuntimeConfig::default()).expect("love port runtime");
        let path = std::env::temp_dir().join(format!("love-down-{}.txt", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        runtime.port_handle().trace().set_trace_file(
            Some("LOVETESTDWNL"),
            TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(file))),
        );
        let user =
            SyncIOHandle::from_handle(runtime.port_handle().clone(), 1, Duration::from_secs(5));
        let err = user.read_int32(0).expect_err("love port is down");
        assert_eq!(err.status(), AsynStatus::Disconnected, "{err:?}");
        assert!(!runtime.port_handle().is_connected_blocking(-1).unwrap());

        let out = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(!out.contains("drvLove::sendCommand"), "{out:?}");
    }

    /// A failed request reports C's `"<port> error "`, not the serial
    /// port's own message (drvLove.c:1202).
    #[test]
    fn a_failed_write_reports_the_love_port_not_the_serial_text() {
        let serial = TimeoutSerial {
            base: PortDriverBase::new(
                "LOVETESTERR",
                1,
                PortFlags {
                    multi_device: false,
                    can_block: true,
                    destructible: true,
                },
            ),
        };
        let mut love = love_on(serial, "LOVETESTERRL");
        let reason = COMMANDS
            .iter()
            .position(|c| matches!(c.write_conv, WriteConv::Data))
            .unwrap();
        let mut user = AsynUser::new(reason).with_addr(1);
        let e = love.write_int32(&mut user, 5).unwrap_err();
        assert_eq!(e.message(), "LOVETESTERRL error ");
        // C `executeCommand` gives up with asynError (drvLove.c:768).
        assert_eq!(e.status(), AsynStatus::Error);
        let e = love
            .read_int32(&AsynUser::new(reason).with_addr(1))
            .unwrap_err();
        assert_eq!(e.message(), "LOVETESTERRL error ");
    }

    /// C `reportIt` (drvLove.c:978-990): the port pair, then every address
    /// a request has reached.
    #[test]
    fn report_prints_reportit_lines() {
        let serial = TimeoutSerial {
            base: PortDriverBase::new(
                "LOVETESTRPT",
                1,
                PortFlags {
                    multi_device: false,
                    can_block: true,
                    destructible: true,
                },
            ),
        };
        let mut love = love_on(serial, "LOVETESTRPTL");
        love.is_conn[3] = true;
        let mut out = String::new();
        love.report(&mut out, 0);
        assert_eq!(
            out,
            "    LOVETESTRPTL is connected to SER\n        Addr 4 is connected\n"
        );
    }

    /// C `executeCommand` (drvLove.c:767-768) ends three timed-out attempts
    /// with asynError.
    #[test]
    fn exhausted_retries_return_error_not_timeout() {
        let serial = TimeoutSerial {
            base: PortDriverBase::new(
                "LOVETESTTMO",
                1,
                PortFlags {
                    multi_device: false,
                    can_block: true,
                    destructible: true,
                },
            ),
        };
        let love = love_on(serial, "LOVETESTTMOL");
        let err = love.execute_command(1, "00").unwrap_err();
        assert_eq!(err.status(), AsynStatus::Error);
    }

    /// Answers every read with a well-formed reply ended by `eom`.
    struct ReplySerial {
        base: PortDriverBase,
        eom: EomReason,
    }

    impl PortDriver for ReplySerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }

        fn io_read_octet_eom(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> AsynResult<(usize, EomReason)> {
            let body = b"L010000";
            let reply = format!("\u{2}L010000{:02X}", calc_checksum(body));
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            Ok((reply.len(), self.eom))
        }
    }

    fn reply_love(name: &str, eom: EomReason) -> LoveDriver {
        let base = PortDriverBase::new(&format!("{name}SER"), 1, PortFlags::default());
        love_on(ReplySerial { base, eom }, name)
    }

    fn report_of(love: &mut LoveDriver) -> String {
        let mut out = String::new();
        love.report(&mut out, 0);
        out
    }

    /// Only C `connectIt` sets `isConn` and only `disconnectIt` clears it
    /// (drvLove.c:1031, :1086-1093), so a request does not list the address in
    /// `reportIt`.
    #[test]
    fn disconnect_drops_the_address_from_the_report() {
        let mut love = reply_love("LOVETESTDIS", EomReason::EOS);
        let addr2 = AsynUser::new(0).with_addr(2);
        let listed = "    LOVETESTDIS is connected to SER\n        Addr 2 is connected\n";
        let unlisted = "    LOVETESTDIS is connected to SER\n";

        love.do_read(2, 0).unwrap();
        assert_eq!(report_of(&mut love), unlisted);
        assert!(love.disconnect_addr(&addr2).is_err());
        love.connect_addr(&addr2).unwrap();
        assert_eq!(report_of(&mut love), listed);
        love.disconnect_addr(&addr2).unwrap();
        assert_eq!(report_of(&mut love), unlisted);
        assert!(!love.base().is_device_connected(2));
        assert!(love.disconnect_addr(&addr2).is_err());

        love.connect_addr(&addr2).unwrap();
        assert_eq!(report_of(&mut love), listed);
        assert!(love.base().is_device_connected(2));
        assert!(love.connect_addr(&addr2).is_err());
        assert!(love.connect_addr(&AsynUser::new(0).with_addr(0)).is_err());
    }

    /// C `recvReply` (drvLove.c:844-848) refuses a reply the EOS did not end,
    /// before `evalMessage` looks at it.
    #[test]
    fn a_reply_not_ended_by_the_eos_is_an_error() {
        assert!(
            reply_love("LOVETESTEOS", EomReason::EOS)
                .execute_command(1, "00")
                .is_ok()
        );
        for eom in [
            EomReason::CNT,
            EomReason::empty(),
            EomReason::EOS | EomReason::CNT,
        ] {
            let err = reply_love("LOVETESTCNT", eom)
                .execute_command(1, "00")
                .unwrap_err();
            assert_eq!(err.status(), AsynStatus::Error, "{eom:?}");
        }
    }
}
