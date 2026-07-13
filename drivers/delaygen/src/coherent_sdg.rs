//! Coherent Synchronization/Delay Generator (SDG) asyn port driver.
//!
//! Ported from `drvAsynCoherentSDG.cpp`. Like Colby (see
//! [`crate::colby`]), every "parameter" is an asyn multi-device *address*
//! (0-12, [`COMMANDS`] below, transcribed verbatim from the C source's own
//! `commandTable[]`) — but unlike Colby, `create()` (asynDrvUser) here DOES
//! validate the address eagerly at record-bind time
//! (`findCommand(addr)==NULL => asynError`), which
//! [`CoherentSdgDriver::drv_user_create`] reproduces.
//!
//! # EOS ownership
//! `coherentSDG.cmd` configures the underlying octet port with input EOS
//! `\r` and output EOS `\r` (`asynOctetSetInputEos`/`asynOctetSetOutputEos`),
//! which the IOC's `st.cmd` must reproduce. The driver itself never appends
//! a terminator.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **`cvtIdent` always returns the literal `"Coherent SDG"`** — addr 0's
//!   `status?` query is still sent and its reply still awaited (an
//!   init-time-style connectivity check that also occurs on every
//!   subsequent read of addr 0), but the reply content itself is discarded;
//!   see [`CoherentSdgDriver::read_octet`].
//! - **`cvtStrBin`'s MSB-first bit parsing** (`read:sta:bwd?`, addr 3):
//!   character `i` (from the left) of the reply sets bit `len-1-i` when it
//!   is exactly `'1'` — any other character (including `'0'`) leaves that
//!   bit clear. A reply longer than 32 characters would shift-overflow
//!   (`1<<j` for `j>=32`) — undefined behavior in C; [`parse_str_bin`]
//!   substitutes the defined behavior of ignoring bits beyond position 31
//!   (real `read:sta:bwd?` replies are a handful of characters).
//! - **Write success is `epicsStrCaseCmp(reply,"OK")==0`**, not the
//!   transport status — `writeIntParam`/`writeFloatParam`/`writeCmdOnly`
//!   all send their command via the *same* writeRead-with-reply path as a
//!   read, then classify success by the reply text. See [`is_ok_reply`].
//! - **Unconditional 100 ms post-transaction delay** (`WRITEREADDELAY`) —
//!   the shared C `writeRead()` helper sleeps 100 ms after *every*
//!   transaction (read or write), regardless of success or failure, before
//!   returning. [`device_write_read`] reproduces this exactly (including on
//!   the error path).
//! - **Interface/address cross-checking is stricter here than in C.** C's
//!   `readFloat64`/`writeFloat64`/`readUInt32`/`writeUInt32`/`readOctet`/
//!   `writeOctet` call into `findCommand(addr)`'s conv/write function
//!   *unconditionally*, with no check that the row's conv type actually
//!   matches the calling interface — e.g. a hypothetical `ai` record wired
//!   to addr 1 (an int-only row) would hand `cvtStrInt` a `epicsFloat64*`
//!   output pointer and corrupt 4 of its 8 bytes. The shipped
//!   `drvAsynCoherentSDG.db` never wires a mismatched interface/address
//!   pair, so this is dead code in practice; this port makes the mismatch a
//!   defined `AsynError::AddressOutOfRange` in every read/write override
//!   below rather than reproducing the type confusion.
//! - **This driver does not replicate `pport->init==0` guards** (checked at
//!   the top of every C interface method) — `init` is set exactly once, at
//!   the very end of `drvAsynCoherentSDG()`, and the port is registered
//!   with `pasynManager` (making it reachable by any I/O request) only
//!   *after* that assignment, so the guard can never observably fire.
//! - **This driver does not replicate the C driver's process-wide singleton
//!   restriction** (`if(pports) return -1`) — see the identical note in
//!   [`crate::colby`]'s module doc for the rationale.
//! - **This driver does not replicate `report()`'s diagnostic text** (dbior
//!   output) — the C version prints internal reference/connection counters
//!   (`conns`/`refs`/`pvs`/`discos`/`writeReads`) that this port does not
//!   track, since none of it is wire-observable protocol. Matches the
//!   DG645 port's precedent (no `report()` override there either).

use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::wire::{atof, atoi, write_read};

/// C `WRITEREADDELAY` (`0.100` seconds).
const WRITEREADDELAY: Duration = Duration::from_millis(100);

/// C `commandTable[].readConv` — how a read reply becomes the value
/// returned to device support.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ReadConv {
    /// `cvtIdent`: reply discarded, always resolves to the literal
    /// `"Coherent SDG"`.
    Ident,
    /// `cvtStrInt`: `atoi(reply)`.
    StrInt,
    /// `cvtStrBin`: MSB-first `'1'` bit parsing, see [`parse_str_bin`].
    StrBin,
    /// `cvtStrFloat`: `atof(reply)`.
    StrFloat,
    /// `cvtSink`/`readSink`: no I/O, value is always the zero-equivalent
    /// for the calling interface.
    Sink,
}

/// C `commandTable[].writeCommand`/`writeFunc`.
#[derive(Clone, Copy, Debug)]
enum WriteKind {
    /// `writeSink`: always succeeds, no I/O.
    Sink,
    /// `writeIntParam`: `sprintf(outBuf,writeCommand,value)` where
    /// `writeCommand` is a `%W.Wd`-style integer format. Every row in this
    /// table has `width == precision`, so precision-driven zero-padding
    /// alone reproduces the format; `width=0` reproduces the unpadded
    /// `"...%d"` rows (addr 9, 11).
    IntFmt { prefix: &'static str, width: usize },
    /// `writeFloatParam`: `sprintf(outBuf,writeCommand,value)` where
    /// `writeCommand` is `"...%06.1f"` in every row that uses it.
    FloatFmt { prefix: &'static str },
    /// `writeCmdOnly`: sends the literal command, ignores the record value
    /// entirely.
    CmdOnly(&'static str),
}

/// C `Command` row (`commandTable[]`, `drvAsynCoherentSDG.cpp:225-251`).
struct CommandSpec {
    /// C `Command.ident`, matched against the record's asyn address
    /// (`findCommand(addr)`) — not a separate tag string.
    addr: i32,
    /// C `Command.readCommand` — `""` means `readSink` (no I/O).
    read_cmd: &'static str,
    read_conv: ReadConv,
    write: WriteKind,
}

const COMMANDS: &[CommandSpec] = &[
    CommandSpec {
        addr: 0,
        read_cmd: "status?",
        read_conv: ReadConv::Ident,
        write: WriteKind::Sink,
    },
    CommandSpec {
        addr: 1,
        read_cmd: "read:rate?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:rate",
            width: 4,
        },
    },
    CommandSpec {
        addr: 2,
        read_cmd: "read:bwd?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::Sink,
    },
    CommandSpec {
        addr: 3,
        read_cmd: "read:sta:bwd?",
        read_conv: ReadConv::StrBin,
        write: WriteKind::CmdOnly("reset:bwd"),
    },
    CommandSpec {
        addr: 4,
        read_cmd: "read:rf?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:rf",
            width: 1,
        },
    },
    CommandSpec {
        addr: 5,
        read_cmd: "read:mode?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:mode",
            width: 1,
        },
    },
    CommandSpec {
        addr: 6,
        read_cmd: "",
        read_conv: ReadConv::Sink,
        write: WriteKind::CmdOnly("man:trig"),
    },
    CommandSpec {
        addr: 7,
        read_cmd: "read:c1?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:c1",
            width: 1,
        },
    },
    CommandSpec {
        addr: 8,
        read_cmd: "read:del:c1?",
        read_conv: ReadConv::StrFloat,
        write: WriteKind::FloatFmt {
            prefix: "set:del:c1",
        },
    },
    CommandSpec {
        addr: 9,
        read_cmd: "read:c2?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:c2",
            width: 0,
        },
    },
    CommandSpec {
        addr: 10,
        read_cmd: "read:del:c2?",
        read_conv: ReadConv::StrFloat,
        write: WriteKind::FloatFmt {
            prefix: "set:del:c2",
        },
    },
    CommandSpec {
        addr: 11,
        read_cmd: "read:c3?",
        read_conv: ReadConv::StrInt,
        write: WriteKind::IntFmt {
            prefix: "set:c3",
            width: 0,
        },
    },
    CommandSpec {
        addr: 12,
        read_cmd: "read:del:c3?",
        read_conv: ReadConv::StrFloat,
        write: WriteKind::FloatFmt {
            prefix: "set:del:c3",
        },
    },
];

fn find_spec(addr: i32) -> Option<&'static CommandSpec> {
    COMMANDS.iter().find(|c| c.addr == addr)
}

/// C `sprintf(outBuf,"...%W.Wd",value)` — see [`WriteKind::IntFmt`].
fn format_int_write(prefix: &str, width: usize, value: u32) -> String {
    format!("{prefix} {value:0width$}")
}

/// C `sprintf(outBuf,"...%06.1f",value)` — see [`WriteKind::FloatFmt`].
fn format_float_write(prefix: &str, value: f64) -> String {
    format!("{prefix} {value:06.1}")
}

/// C `epicsStrCaseCmp(inpBuf,"OK")==0` — the write-success check shared by
/// `writeIntParam`/`writeFloatParam`/`writeCmdOnly`.
fn is_ok_reply(reply: &str) -> bool {
    reply.eq_ignore_ascii_case("OK")
}

/// C `cvtStrBin`: MSB-first `'1'`-only bit parsing. See the module doc's
/// preserved-quirks note for the 32-bit truncation substitution.
fn parse_str_bin(reply: &str) -> u32 {
    let bytes = reply.as_bytes();
    let len = bytes.len();
    let mut k: u32 = 0;
    for (i, &b) in bytes.iter().enumerate() {
        if b == b'1' {
            let j = len - 1 - i;
            if j < 32 {
                k += 1 << j;
            }
        }
    }
    k
}

fn ok_or_device_error(prefix: &str, reply: &str) -> AsynResult<()> {
    if is_ok_reply(reply) {
        Ok(())
    } else {
        Err(AsynError::Status {
            status: AsynStatus::Error,
            message: format!("{prefix}: device replied {reply:?} (expected \"OK\")"),
        })
    }
}

/// C `writeRead(pport,pasynUser,outBuf,inpBuf,inputSize,eomReason)` — used
/// for both reads (`readParam`) and writes (`writeIntParam`/
/// `writeFloatParam`/`writeCmdOnly`) alike. The 100 ms delay runs
/// unconditionally, even when the I/O itself failed.
fn device_write_read(handle: &SyncIOHandle, cmd: &str) -> AsynResult<String> {
    let result = write_read(handle, cmd);
    std::thread::sleep(WRITEREADDELAY);
    result
}

/// Coherent SDG port driver state (C `struct Port`).
pub struct CoherentSdgDriver {
    base: PortDriverBase,
    handle: SyncIOHandle,
}

impl CoherentSdgDriver {
    /// C `drvAsynCoherentSDG(myport,ioport,ioaddr)`: queries `status?` once
    /// as an init-time connectivity check (the C driver aborts driver
    /// creation if this fails, storing `"*COMM FAILED*"` — moot here since a
    /// failed `new()` never produces a driver instance to report through).
    pub fn new(port_name: &str, handle: SyncIOHandle) -> AsynResult<Self> {
        let base = PortDriverBase::new(
            port_name,
            13,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );

        device_write_read(&handle, "status?")?;

        Ok(Self { base, handle })
    }
}

impl PortDriver for CoherentSdgDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `create()` (asynDrvUser): `findCommand(addr)==NULL => asynError`
    /// — unlike Colby, CoherentSDG validates the address eagerly at
    /// record-bind time.
    fn drv_user_create(&mut self, _drv_info: &str, addr: i32) -> AsynResult<DrvUserInfo> {
        if find_spec(addr).is_some() {
            Ok(DrvUserInfo::from_reason(0))
        } else {
            Err(AsynError::AddressOutOfRange(addr))
        }
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        match spec.read_conv {
            ReadConv::StrFloat => {
                let reply = device_write_read(&self.handle, spec.read_cmd)?;
                Ok(atof(&reply))
            }
            _ => Err(AsynError::AddressOutOfRange(user.addr)),
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        match spec.write {
            WriteKind::FloatFmt { prefix } => {
                let cmd = format_float_write(prefix, value);
                let reply = device_write_read(&self.handle, &cmd)?;
                ok_or_device_error(prefix, &reply)
            }
            _ => Err(AsynError::AddressOutOfRange(user.addr)),
        }
    }

    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        let raw = match spec.read_conv {
            ReadConv::StrInt => {
                let reply = device_write_read(&self.handle, spec.read_cmd)?;
                atoi(&reply) as u32
            }
            ReadConv::StrBin => {
                let reply = device_write_read(&self.handle, spec.read_cmd)?;
                parse_str_bin(&reply)
            }
            ReadConv::Sink => 0,
            _ => return Err(AsynError::AddressOutOfRange(user.addr)),
        };
        Ok(raw & mask)
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        _mask: u32,
    ) -> AsynResult<()> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        match spec.write {
            WriteKind::IntFmt { prefix, width } => {
                let cmd = format_int_write(prefix, width, value);
                let reply = device_write_read(&self.handle, &cmd)?;
                ok_or_device_error(prefix, &reply)
            }
            WriteKind::CmdOnly(cmd) => {
                let reply = device_write_read(&self.handle, cmd)?;
                ok_or_device_error(cmd, &reply)
            }
            WriteKind::Sink => Ok(()),
            WriteKind::FloatFmt { .. } => Err(AsynError::AddressOutOfRange(user.addr)),
        }
    }

    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        match spec.read_conv {
            ReadConv::Ident => {
                // The reply is still awaited (a connectivity check) even
                // though its content is discarded -- see the module doc's
                // cvtIdent quirk note.
                let _ = device_write_read(&self.handle, spec.read_cmd)?;
                let text = "Coherent SDG";
                let bytes = text.as_bytes();
                let n = bytes.len().min(buf.len());
                buf[..n].copy_from_slice(&bytes[..n]);
                Ok(n)
            }
            _ => Err(AsynError::AddressOutOfRange(user.addr)),
        }
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let spec = find_spec(user.addr).ok_or(AsynError::AddressOutOfRange(user.addr))?;
        match spec.write {
            // C: writeSink always succeeds, and writeOctet's caller reports
            // `strlen(data)` bytes "written" whenever status is success --
            // even though writeSink performs no I/O at all.
            WriteKind::Sink => Ok(data.len()),
            _ => Err(AsynError::AddressOutOfRange(user.addr)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_table_addrs_and_read_cmds_match_c_source() {
        assert_eq!(COMMANDS.len(), 13);
        let expected = [
            (0, "status?"),
            (1, "read:rate?"),
            (2, "read:bwd?"),
            (3, "read:sta:bwd?"),
            (4, "read:rf?"),
            (5, "read:mode?"),
            (6, ""),
            (7, "read:c1?"),
            (8, "read:del:c1?"),
            (9, "read:c2?"),
            (10, "read:del:c2?"),
            (11, "read:c3?"),
            (12, "read:del:c3?"),
        ];
        for (spec, (addr, read_cmd)) in COMMANDS.iter().zip(expected.iter()) {
            assert_eq!(spec.addr, *addr);
            assert_eq!(spec.read_cmd, *read_cmd);
        }
    }

    #[test]
    fn format_int_write_zero_pads_to_width() {
        assert_eq!(format_int_write("set:rate", 4, 5), "set:rate 0005");
        assert_eq!(format_int_write("set:rf", 1, 1), "set:rf 1");
        assert_eq!(format_int_write("set:c2", 0, 7), "set:c2 7");
    }

    #[test]
    fn format_float_write_matches_c_06_1f() {
        assert_eq!(format_float_write("set:del:c1", 5.0), "set:del:c1 0005.0");
        assert_eq!(format_float_write("set:del:c1", -1.5), "set:del:c1 -001.5");
    }

    #[test]
    fn is_ok_reply_is_case_insensitive() {
        assert!(is_ok_reply("OK"));
        assert!(is_ok_reply("ok"));
        assert!(is_ok_reply("Ok"));
        assert!(!is_ok_reply("ERROR"));
        assert!(!is_ok_reply(""));
    }

    #[test]
    fn parse_str_bin_is_msb_first_and_only_recognizes_1() {
        assert_eq!(parse_str_bin("101"), 0b101);
        assert_eq!(parse_str_bin("001"), 0b001);
        assert_eq!(parse_str_bin(""), 0);
        // Non-'1' characters (including '0' and anything else) clear the bit.
        assert_eq!(parse_str_bin("1x1"), 0b101);
    }
}
