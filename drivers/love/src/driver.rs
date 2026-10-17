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

use std::fmt::Write as _;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::{
    DpCells, DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags,
};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{PortCalls, RequestOp};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::{AsynUser, UserTrace};

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

/// C's `pasynUser->reason` for every love user: `create` sets only
/// `drvUser` (drvLove.c:1129), so devAsyn* leaves it 0.
const C_REASON: i32 = 0;

/// The address of C's `pport->pasynUser`, which `drvLoveInit` connects to
/// the port itself (drvLove.c:434).
const PORT_USER: i32 = -1;

/// C `asynPrint(pasynUser, …)` for a love user at `$addr`.
macro_rules! love_print {
    ($self:expr, $addr:expr, $mask:ident, $($arg:tt)*) => {
        $self.base.trace_print_at(
            $addr,
            C_REASON,
            TraceMask::$mask,
            file!(),
            line!(),
            format_args!($($arg)*),
        )
    };
}

/// What C's `%s` prints of a reply buffer: up to its first NUL.
fn c_str(bytes: &[u8]) -> &[u8] {
    &bytes[..bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len())]
}

/// [`love_print!`] for a [`CallerTrace`], which prints on the serial port's
/// actor while a command holds it.
macro_rules! caller_print {
    ($trace:expr, $mask:ident, $($arg:tt)*) => {
        $trace.print(TraceMask::$mask, file!(), line!(), format_args!($($arg)*))
    };
}

/// [`caller_print!`] for a line with a device buffer in it, which C
/// prints with `%s`.
macro_rules! caller_write {
    ($trace:expr, $mask:ident, $body:expr) => {
        $trace.write(TraceMask::$mask, file!(), line!(), $body)
    };
}

/// The caller's `pasynUser` as C's `executeCommand` prints with it: the
/// love port's trace linkage the actor stamped on the request, and the
/// caller's address. A user that never reached the actor prints nothing.
#[derive(Clone)]
struct CallerTrace {
    trace: Option<UserTrace>,
    addr: i32,
}

impl CallerTrace {
    fn print(&self, mask: TraceMask, file: &str, line: u32, args: std::fmt::Arguments<'_>) {
        if let Some(t) = &self.trace {
            t.print(self.addr, C_REASON, mask, file, line, args);
        }
    }

    fn write(
        &self,
        mask: TraceMask,
        file: &str,
        line: u32,
        body: impl FnOnce(&mut epics_rs::asyn::trace::TraceOut<'_>) -> std::fmt::Result,
    ) {
        if let Some(t) = &self.trace {
            t.manager.print_with(
                Some(&t.port),
                Some(self.addr),
                C_REASON,
                mask,
                (file, line),
                body,
            );
        }
    }
}

/// One love command, run on the serial port's actor under its lock.
struct Command {
    trace: CallerTrace,
    frame: String,
    ser_addr: i32,
    timeout: Duration,
}

impl Command {
    /// C `executeCommand` with its `sendCommand`, `recvReply` and
    /// `evalMessage` (drvLove.c:564-623, :723-768, :790-867). Their
    /// `pasynOctet->write`/`read` go straight to the serial port, as here.
    fn execute(&self, calls: &mut dyn PortCalls) -> AsynResult<Vec<u8>> {
        let t = &self.trace;
        let frame = &self.frame;
        caller_print!(t, FLOW, "drvLove::executeCommand\n");

        for retry in 0..MAX_ATTEMPTS {
            std::thread::sleep(K_TUNE);

            caller_print!(t, FLOW, "drvLove::sendCommand - retries({retry})\n");
            let write = RequestOp::OctetWrite {
                data: frame.as_bytes().to_vec(),
            };
            if let Err(e) = calls.call(&write, self.serial_user()) {
                caller_print!(
                    t,
                    ERROR,
                    "drvLove::sendCommand - retries({retry}) {}\n",
                    status_text(e.status())
                );
                if is_timeout(&e) {
                    caller_print!(
                        t,
                        ERROR,
                        "drvLove::executeCommand write timeout, retrying\n"
                    );
                    continue;
                }
                caller_print!(
                    t,
                    ERROR,
                    "drvLove::executeCommand write failure - Sent \"{frame}\" \n"
                );
                return Err(e);
            }
            // C's format has a second `"%s"` with no argument (drvLove.c:815).
            caller_print!(
                t,
                IO_FILTER,
                "drvLove::sendCommand - retries({retry}),data \"{frame}\"\n"
            );
            caller_print!(t, IO_FILTER, "drvLove::executeCommand write \"{frame}\"\n");

            caller_print!(t, FLOW, "drvLove::recvReplay\n");
            let read = RequestOp::OctetRead {
                buf_size: REPLY_BUF_SIZE,
            };
            let raw = match calls.call(&read, self.serial_user()) {
                Ok(result) => {
                    let raw = result.data.unwrap_or_default();
                    if EomReason::from_bits_truncate(result.eom_reason) != EomReason::EOS {
                        caller_print!(t, ERROR, "drvLove::recvReply eos failure\n");
                        self.trace_read_failure(&raw);
                        return Err(protocol_error("recvReply eos failure"));
                    }
                    raw
                }
                Err(e) => {
                    caller_print!(t, ERROR, "drvLove::recvReply {}\n", status_text(e.status()));
                    if is_timeout(&e) {
                        caller_print!(t, ERROR, "drvLove::executeCommand read timeout, retrying\n");
                        continue;
                    }
                    // C prints whatever the failed read left in inpMsg.
                    let rcvd = e.partial_read().map(|p| p.data.as_slice()).unwrap_or(&[]);
                    self.trace_read_failure(rcvd);
                    return Err(e);
                }
            };

            caller_print!(t, FLOW, "drvLove::evalMessage\n");
            let evaluated = eval_message(&raw);
            // evalMessage rewrites inpMsg in place: the payload, or for a device
            // error reply its 3 bytes after the header (drvLove.c:617-619).
            let inp: &[u8] = match &evaluated {
                Ok(payload) => {
                    caller_print!(t, FLOW, "drvLove::evalMessage message received\n");
                    payload
                }
                Err(e) => {
                    caller_print!(t, ERROR, "{}", eval_error_line(e, raw.len()));
                    match e {
                        EvalError::DeviceError(_) => &raw[4..],
                        _ => &raw,
                    }
                }
            };
            caller_write!(t, IO_FILTER, |out| {
                write!(out, "drvLove::recvReply {} \"", inp.len())?;
                out.write_bytes(c_str(inp))?;
                out.write_str("\"\n")
            });
            return match evaluated {
                Ok(payload) => {
                    caller_write!(t, IO_FILTER, |out| {
                        out.write_str("drvLove::executeCommand read \"")?;
                        out.write_bytes(c_str(payload))?;
                        out.write_str("\"\n")
                    });
                    Ok(payload.to_vec())
                }
                Err(e) => {
                    self.trace_read_failure(inp);
                    Err(eval_error_to_asyn(e))
                }
            };
        }

        caller_print!(t, ERROR, "drvLove::executeCommand retries exceeded\n");
        // C returns asynError, not the last attempt's asynTimeout, so the
        // record alarms READ/WRITE rather than TIMEOUT.
        Err(protocol_error("executeCommand retries exceeded"))
    }

    /// C's `Serport::pasynUser` on the serial port.
    fn serial_user(&self) -> AsynUser {
        AsynUser::new(OCTET_REASON)
            .with_addr(self.ser_addr)
            .with_timeout(self.timeout)
    }

    fn trace_read_failure(&self, rcvd: &[u8]) {
        let frame = &self.frame;
        caller_write!(self.trace, ERROR, |out| {
            write!(
                out,
                "drvLove::executeCommand read failure - Sent \"{frame}\" Rcvd \""
            )?;
            out.write_bytes(c_str(rcvd))?;
            out.write_str("\" \n")
        });
    }
}

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
    /// The serial user's `dpCommon`, which C's `isConnected` reads without
    /// queueing on the serial port (asynManager.c:2331-2342).
    serial_dp: DpCells,
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
            serial_dp: serial.connect_device(ser_addr),
            serial,
            ser_addr,
            timeout,
            models,
            is_conn: [false; K_INSTRMAX],
        })
    }

    /// C `serialPort->pasynUser`: the serial address and timeout a command's
    /// write and read run with, the frame for `addr`, and the caller's
    /// trace linkage.
    fn command(&self, trace: Option<UserTrace>, addr: i32, body: &str) -> Command {
        Command {
            trace: CallerTrace { trace, addr },
            frame: build_frame(addr, body),
            ser_addr: self.ser_addr,
            timeout: self.timeout,
        }
    }

    /// C `connectIt` (drvLove.c:993-1044).
    fn connect_it(&mut self, user: &AsynUser) -> AsynResult<()> {
        let addr = user.addr;
        love_print!(self, addr, FLOW, "drvLove::connectIt\n");
        if !self.serial_dp.connected() {
            return Err(protocol_error(format!(
                "port {} not connected to {}",
                self.base.port_name, self.ser_name
            )));
        }
        self.check_addr("connectIt", addr)?;
        let already = if addr > 0 {
            std::mem::replace(&mut self.is_conn[(addr - 1) as usize], true)
        } else {
            self.base.dp_connected(addr)
        };
        if already {
            love_print!(
                self,
                addr,
                ERROR,
                "drvLove::connectIt {} device {addr} already connected\n",
                self.base.port_name
            );
            return Err(protocol_error("already connected"));
        }
        self.base.set_addr_connected(addr, true);
        Ok(())
    }

    /// C `disconnectIt` (drvLove.c:1047-1095).
    fn disconnect_it(&mut self, user: &AsynUser) -> AsynResult<()> {
        let addr = user.addr;
        love_print!(self, addr, FLOW, "drvLove::disconnectIt\n");
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
            love_print!(
                self,
                addr,
                ERROR,
                "drvLove::{func} {} illegal addr {addr}\n",
                self.base.port_name
            );
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

    fn do_read(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let (addr, reason) = (user.addr, user.reason);
        if !(1..=K_INSTRMAX as i32).contains(&addr) {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        let spec = &COMMANDS[reason];
        let model = self.model_for(addr);
        let read_cmd = spec.read[model as usize];

        let payload = self.locked_command(user.trace.clone(), addr, read_cmd)?;
        Ok(match spec.read_conv {
            ReadConv::Value => {
                love_print!(self, PORT_USER, FLOW, "drvLove::getValue\n");
                parse_value(&payload)
            }
            ReadConv::Status => {
                love_print!(self, PORT_USER, FLOW, "drvLove::getStatus\n");
                parse_status(&payload)
            }
            ReadConv::SignedValue => {
                love_print!(self, PORT_USER, FLOW, "drvLove::getSignedValue\n");
                parse_signed_value(&payload, model)
            }
            ReadConv::Data => {
                love_print!(self, PORT_USER, FLOW, "drvLove::getData\n");
                parse_data(&payload)
            }
        })
    }

    fn do_write(&mut self, user: &AsynUser, value: i32) -> AsynResult<()> {
        let (addr, reason) = (user.addr, user.reason);
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

        love_print!(self, PORT_USER, FLOW, "drvLove::putData\n");
        let body = format_put_data(write_cmd, value);
        let payload = self.locked_command(user.trace.clone(), addr, &body)?;
        love_print!(self, PORT_USER, FLOW, "drvLove::processWriteResponse\n");
        process_write_response(&payload).inspect_err(|_| {
            love_print!(
                self,
                PORT_USER,
                ERROR,
                "drvLove::processWriteResponse write command failed\n"
            )
        })
    }

    /// C's `lockPort` / `executeCommand` / `unlockPort` in every read and
    /// write (drvLove.c:1198-1200): the serial port stays locked from before
    /// the write until after the read, so no other serial user's request
    /// lands between a command and its reply.
    fn locked_command(
        &self,
        trace: Option<UserTrace>,
        addr: i32,
        body: &str,
    ) -> AsynResult<Vec<u8>> {
        let command = self.command(trace, addr, body);
        let caller = command.trace.clone();
        caller_print!(caller, FLOW, "drvLove::lockPort\n");
        // A shut-down serial port refuses the lock (asynManager.c:1790-1795),
        // and C's lockPort (drvLove.c:698-700) copies that into the error
        // text. C then runs executeCommand anyway, which writes through the
        // pasynOctetPvt cached at init (:544, :813) into the driver its
        // shutdown deleted (asynPortDriver.cpp:3980): a use-after-free. The
        // write and read are refused here instead.
        let result = self
            .serial
            .lock_port_blocking(move |calls| command.execute(calls))
            .map_err(|e| AsynError::Status {
                status: e.status(),
                message: format!("{} error {}", self.ser_name, e.message()),
            });
        caller_print!(caller, FLOW, "drvLove::unlockPort\n");
        result.and_then(|r| r)
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
        love_print!(self, addr, FLOW, "drvLove::create\n");
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
        love_print!(self, user.addr, FLOW, "drvLove::readInt32\n");
        let value = self.do_read(user).map_err(|e| self.record_error(e))?;
        love_print!(
            self,
            user.addr,
            IO_FILTER,
            "drvLove::readInt32 readback from {} is {value}\n",
            self.base.port_name
        );
        Ok(value)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        love_print!(self, user.addr, FLOW, "drvLove::writeInt32\n");
        self.do_write(user, value).map_err(|e| self.record_error(e))
    }

    /// drvLove.c has no getBounds, so C's is asynInt32Base's
    /// `*low = *high = 0` (asynInt32Base.c:99).
    fn get_bounds_int32(&self, _user: &AsynUser) -> AsynResult<(i32, i32)> {
        Ok((0, 0))
    }

    /// C `readUInt32` (`drvLove.c:1296-1333`) -- the `mask` argument is
    /// accepted but never applied by the C driver's own body (only used in
    /// a trace-log format string); the mask is applied here to match the
    /// asyn-rs convention already used by `delaygen::coherent_sdg` (every
    /// `read_uint32_digital` override masks its own return value), which is
    /// harmless here since the record layer re-applies (and shifts by) the
    /// same mask regardless.
    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        love_print!(self, user.addr, FLOW, "drvLove::readUInt32\n");
        let value = self.do_read(user).map_err(|e| self.record_error(e))? as u32;
        love_print!(
            self,
            user.addr,
            IO_FILTER,
            "drvLove::readUInt32 readback from {} is 0x{value:X},mask=0x{mask:X}\n",
            self.base.port_name
        );
        Ok(value & mask)
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
        love_print!(self, user.addr, FLOW, "drvLove::writeUInt32\n");
        self.do_write(user, value as i32)
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

    /// A serial port whose write holds its actor for a second.
    struct SlowSerial {
        base: PortDriverBase,
    }

    impl PortDriver for SlowSerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            std::thread::sleep(Duration::from_secs(1));
            Ok(data.len())
        }
    }

    /// C `connectIt` asks `isConnected`, which reads the serial user's
    /// `dpCommon` without queueing (drvLove.c:1003, asynManager.c:2331-2342),
    /// so it answers while the serial port is busy.
    #[test]
    fn connect_it_does_not_wait_on_a_busy_serial_port() {
        let base = PortDriverBase::new("LOVETESTBSY", 1, PortFlags::default());
        let mut love = love_on(SlowSerial { base }, "LOVETESTBSYL");
        let serial = love.serial.clone();
        assert!(serial.is_connected_blocking(0).unwrap());
        let write = std::thread::spawn(move || {
            serial.submit_blocking(
                RequestOp::OctetWrite {
                    data: b"x".to_vec(),
                },
                AsynUser::new(OCTET_REASON),
            )
        });
        std::thread::sleep(Duration::from_millis(200));
        let start = std::time::Instant::now();
        love.connect_it(&AsynUser::default().with_addr(1)).unwrap();
        assert!(
            start.elapsed() < Duration::from_millis(500),
            "{:?}",
            start.elapsed()
        );
        write.join().unwrap().unwrap();
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
        let err = love.locked_command(None, 1, "00").unwrap_err();
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

    /// A love port on `serial` in its own runtime, with the port's trace
    /// lines in one file and device 1's in another; returns the device
    /// reader and the two files.
    fn traced_love(
        serial: impl PortDriver,
        name: &str,
    ) -> (SyncIOHandle, std::path::PathBuf, std::path::PathBuf) {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::TraceFile;

        let love = love_on(serial, name);
        let claim = claim_port(name).expect("love port name");
        let (runtime, _thread) =
            create_port_runtime(&claim, love, RuntimeConfig::default()).expect("love port runtime");
        let handle = runtime.port_handle().clone();
        claim
            .publish(handle.clone(), &epics_rs::asyn::registry::open_owner())
            .unwrap();
        std::mem::forget(runtime);
        let file = |what: &str| {
            let path =
                std::env::temp_dir().join(format!("love-{name}-{what}-{}.txt", std::process::id()));
            let f = std::fs::File::create(&path).unwrap();
            (
                path,
                TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(f))),
            )
        };
        let (port_path, port_file) = file("port");
        let (dev_path, dev_file) = file("dev");
        let trace = handle.trace();
        trace.set_trace_mask(Some(name), TraceMask::ERROR | TraceMask::FLOW);
        trace.set_trace_file(Some(name), port_file);
        trace.set_device_trace_mask(
            name,
            1,
            TraceMask::ERROR | TraceMask::FLOW | TraceMask::IO_FILTER,
        );
        trace.set_device_trace_file(name, 1, dev_file);
        (
            SyncIOHandle::from_handle(handle, 1, Duration::from_secs(5)),
            port_path,
            dev_path,
        )
    }

    fn read_trace(path: &std::path::Path) -> Vec<u8> {
        let out = std::fs::read(path).unwrap();
        let _ = std::fs::remove_file(path);
        out
    }

    /// C prints a request's lines with the caller's `pasynUser`, so through
    /// its device, and the conversions with `pport->pasynUser`, so through
    /// the port (drvLove.c:1223-1251, :879).
    #[test]
    fn a_read_prints_through_the_caller_device_and_the_port_user() {
        let base = PortDriverBase::new("LOVETESTDEVSER", 1, PortFlags::default());
        let serial = ReplySerial {
            base,
            eom: EomReason::EOS,
        };
        let (user, port_path, dev_path) = traced_love(serial, "LOVETESTDEV");
        assert_eq!(user.read_int32(0).unwrap(), 0);

        let port = String::from_utf8(read_trace(&port_path)).unwrap();
        let dev = String::from_utf8(read_trace(&dev_path)).unwrap();
        let mut at = 0;
        for line in [
            "drvLove::readInt32\n",
            "drvLove::lockPort\n",
            "drvLove::executeCommand\n",
            "drvLove::sendCommand - retries(0)\n",
            "drvLove::sendCommand - retries(0),data \"\u{2}L0100C1\"\n",
            "drvLove::executeCommand write \"\u{2}L0100C1\"\n",
            "drvLove::recvReplay\n",
            "drvLove::evalMessage\n",
            "drvLove::evalMessage message received\n",
            "drvLove::recvReply 4 \"0000\"\n",
            "drvLove::executeCommand read \"0000\"\n",
            "drvLove::unlockPort\n",
            "drvLove::readInt32 readback from LOVETESTDEV is 0\n",
        ] {
            let Some(i) = dev[at..].find(line) else {
                panic!("missing {line:?} after {at} in {dev:?}");
            };
            at += i + line.len();
        }
        assert!(!dev.contains("getValue"), "{dev:?}");
        assert!(port.contains("drvLove::getValue\n"), "{port:?}");
        assert!(!port.contains("readInt32"), "{port:?}");
    }

    /// A reply whose checksum fails.
    struct BadReplySerial {
        base: PortDriverBase,
    }

    impl PortDriver for BadReplySerial {
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
            let reply = b"\x02L01\xff000000";
            buf[..reply.len()].copy_from_slice(reply);
            Ok((reply.len(), EomReason::EOS))
        }
    }

    /// C prints the failed reply with `%s`, its bytes as they are
    /// (drvLove.c:760).
    #[test]
    fn a_failed_reply_is_printed_raw() {
        let base = PortDriverBase::new("LOVETESTRAWSER", 1, PortFlags::default());
        let (user, port_path, dev_path) = traced_love(BadReplySerial { base }, "LOVETESTRAW");
        assert!(user.read_int32(0).is_err());
        let _ = read_trace(&port_path);
        let dev = read_trace(&dev_path);
        let want: &[u8] = b"drvLove::executeCommand read failure - Sent \"\x02L0100C1\" Rcvd \"\x02L01\xff000000\" \n";
        assert!(
            dev.windows(want.len()).any(|w| w == want),
            "{}",
            String::from_utf8_lossy(&dev)
        );
        let want: &[u8] = b"drvLove::evalMessage checksum failed\n";
        assert!(dev.windows(want.len()).any(|w| w == want));
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

        love.do_read(&addr2).unwrap();
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
                .locked_command(None, 1, "00")
                .is_ok()
        );
        for eom in [
            EomReason::CNT,
            EomReason::empty(),
            EomReason::EOS | EomReason::CNT,
        ] {
            let err = reply_love("LOVETESTCNT", eom)
                .locked_command(None, 1, "00")
                .unwrap_err();
            assert_eq!(err.status(), AsynStatus::Error, "{eom:?}");
        }
    }

    /// Logs every write and read, and signals when the love frame's write
    /// begins, which then holds the line long enough for another user to
    /// queue on the port.
    struct LoggingSerial {
        base: PortDriverBase,
        log: std::sync::Arc<std::sync::Mutex<Vec<String>>>,
        writing: std::sync::mpsc::SyncSender<()>,
    }

    impl PortDriver for LoggingSerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            if data.first() == Some(&0x02) {
                self.writing.send(()).unwrap();
                std::thread::sleep(Duration::from_millis(300));
            }
            let text = String::from_utf8_lossy(data).into_owned();
            self.log.lock().unwrap().push(format!("write {text:?}"));
            Ok(data.len())
        }

        fn io_read_octet_eom(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> AsynResult<(usize, EomReason)> {
            self.log.lock().unwrap().push("read".into());
            let body = b"L010000";
            let reply = format!("\u{2}L010000{:02X}", calc_checksum(body));
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            Ok((reply.len(), EomReason::EOS))
        }
    }

    /// C holds `lockPort` on the serial user from before a command's write
    /// until after its read (drvLove.c:1198-1200), so another user of the
    /// serial port cannot get between them.
    #[test]
    fn another_serial_user_cannot_interleave_a_command() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        let log = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let (writing, love_writes) = std::sync::mpsc::sync_channel(1);
        let serial = LoggingSerial {
            base: PortDriverBase::new("LOVETESTLCKSER", 1, PortFlags::default()),
            log: log.clone(),
            writing,
        };
        let love = love_on(serial, "LOVETESTLCK");
        let serial = love.serial.clone();
        let claim = claim_port("LOVETESTLCK").expect("love port name");
        let (runtime, _thread) =
            create_port_runtime(&claim, love, RuntimeConfig::default()).expect("love port runtime");
        let user =
            SyncIOHandle::from_handle(runtime.port_handle().clone(), 1, Duration::from_secs(5));
        let read = std::thread::spawn(move || user.read_int32(0));

        love_writes.recv().unwrap();
        serial
            .submit_blocking(
                RequestOp::OctetWrite {
                    data: b"other".to_vec(),
                },
                AsynUser::new(OCTET_REASON),
            )
            .unwrap();
        assert_eq!(read.join().unwrap().unwrap(), 0);

        let frame = build_frame(1, COMMANDS[0].read[Model::default() as usize]);
        assert_eq!(
            *log.lock().unwrap(),
            [
                format!("write {frame:?}"),
                "read".to_owned(),
                "write \"other\"".to_owned(),
            ]
        );
    }

    /// A serial port that records whether a write reached it.
    struct ShutSerial {
        base: PortDriverBase,
        wrote: std::sync::Arc<std::sync::atomic::AtomicBool>,
    }

    impl PortDriver for ShutSerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.wrote.store(true, std::sync::atomic::Ordering::SeqCst);
            Ok(data.len())
        }
    }

    /// C on a shut-down serial port: both lockPort lines print, lockPort's
    /// refusal is copied into the error text (drvLove.c:690-700), and
    /// readInt32 then reports `"<port> error "` (:1237-1240). No write
    /// reaches the shut-down driver.
    #[test]
    fn a_shut_down_serial_port_refuses_the_command() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::TraceFile;

        let wrote = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let serial = ShutSerial {
            base: PortDriverBase::new(
                "LOVETESTSHDSER",
                1,
                PortFlags {
                    multi_device: false,
                    can_block: true,
                    destructible: true,
                },
            ),
            wrote: wrote.clone(),
        };
        let mut love = love_on(serial, "LOVETESTSHD");
        let serial = love.serial.clone();
        let file = |what: &str| {
            let path =
                std::env::temp_dir().join(format!("love-shd-{what}-{}.txt", std::process::id()));
            let f = std::fs::File::create(&path).unwrap();
            (
                path,
                TraceFile::File(std::sync::Arc::new(std::sync::Mutex::new(f))),
            )
        };
        let (ser_path, ser_file) = file("ser");
        serial
            .trace()
            .set_trace_mask(Some("LOVETESTSHDSER"), TraceMask::ERROR | TraceMask::FLOW);
        serial
            .trace()
            .set_trace_file(Some("LOVETESTSHDSER"), ser_file);
        serial
            .submit_blocking(RequestOp::ShutdownPort, AsynUser::default())
            .unwrap();

        let e = love.locked_command(None, 1, "00").unwrap_err();
        assert_eq!(e.status(), AsynStatus::Disabled);
        assert_eq!(
            e.message(),
            "SER error asynManager:lockPort: port has been shut down"
        );
        let e = love.read_int32(&AsynUser::new(0).with_addr(1)).unwrap_err();
        assert_eq!(e.status(), AsynStatus::Disabled);
        assert_eq!(e.message(), "LOVETESTSHD error ");
        assert!(!wrote.load(std::sync::atomic::Ordering::SeqCst));

        let ser = String::from_utf8(read_trace(&ser_path)).unwrap();
        assert_eq!(
            ser.matches("LOVETESTSHDSER lockPort\n").count(),
            2,
            "{ser:?}"
        );
        assert!(!ser.contains("unlockPort"), "{ser:?}");

        // The love port's own lines, through the caller's device.
        let claim = claim_port("LOVETESTSHD").expect("love port name");
        let (runtime, _thread) =
            create_port_runtime(&claim, love, RuntimeConfig::default()).expect("love port runtime");
        let handle = runtime.port_handle().clone();
        claim
            .publish(handle.clone(), &epics_rs::asyn::registry::open_owner())
            .unwrap();
        let (dev_path, dev_file) = file("dev");
        handle
            .trace()
            .set_device_trace_mask("LOVETESTSHD", 1, TraceMask::ERROR | TraceMask::FLOW);
        handle
            .trace()
            .set_device_trace_file("LOVETESTSHD", 1, dev_file);
        let user = SyncIOHandle::from_handle(handle, 1, Duration::from_secs(5));
        assert!(user.read_int32(0).is_err());
        let dev = String::from_utf8(read_trace(&dev_path)).unwrap();
        let mut at = 0;
        for line in [
            "drvLove::readInt32\n",
            "drvLove::lockPort\n",
            "drvLove::unlockPort\n",
        ] {
            let found = dev[at..]
                .find(line)
                .unwrap_or_else(|| panic!("{line:?} in {dev:?}"));
            at += found + line.len();
        }
        assert!(!dev.contains("drvLove::executeCommand"), "{dev:?}");
    }

    #[test]
    fn get_bounds_is_asyn_int32_base_s() {
        let mut base = PortDriverBase::new(
            "LOVETESTBND",
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        base.set_auto_connect(false);
        let love = love_on(TimeoutSerial { base }, "LOVETESTBNDL");
        assert_eq!(love.get_bounds_int32(&AsynUser::default()).unwrap(), (0, 0));
    }
}
