//! Colby Instruments PDL-100A programmable delay line asyn port driver.
//!
//! Ported from `drvAsynColby.cpp`. Unlike DG645's tag-based dispatch, every
//! "parameter" here is an asyn multi-device *address* (0-15, per the C
//! source's own `Developer notes` table reproduced below) — `create()`
//! (asynDrvUser) never reads `drvInfo` at all, so [`ColbyDriver`] overrides
//! [`PortDriver::drv_user_create`] to skip the tag lookup entirely and PVs
//! select behavior purely via their `@asyn(PORT,ADDR)` link's address.
//!
//! ```text
//!    addr    Function
//! --------------------------------------
//!    0       Write delay
//!    1       Read delay
//!    2       Increment step
//!    3       Decrement step
//!    4       Write step
//!    5       Read step
//!    6       Write units (ns,ps)
//!    7       Read identification
//!    8       Read IP address
//!    9       Read Gateway address
//!    10      Read network mask
//!    11      Read TCP/IP port number
//!    12      Read DHCP status
//!    13      Read MAC address
//!    14      Reset
//!    15      Calibrate
//! --------------------------------------
//! ```
//!
//! # EOS ownership
//! `colby.cmd` configures the underlying octet port with input EOS `:` and
//! output EOS `\r\n` (`asynOctetSetInputEos`/`asynOctetSetOutputEos`), which
//! the IOC's `st.cmd` must reproduce. That framing is separate from the
//! serial-echo stripping described below, which the driver itself performs
//! on top of whatever the port's EOS already delivered.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **addr 7** ("Read identification" per the developer-notes table above)
//!   has **no handler** in any interface — `readFloat64`/`writeFloat64`/
//!   `readUInt32`/`writeUInt32`/`readItRaw` all fall through to their
//!   `default: return(asynError)` branch for addr 7. `*IDN?` is queried once
//!   at init and stored only for `report()` (dbior text), never exposed as a
//!   readable PV. Reproduced verbatim — see [`ColbyDriver::report`].
//! - **addr 6 write** (units) ignores the asynUInt32Digital `mask` argument
//!   entirely, comparing the raw `value` to `0` (`if(value==0) ... else ...`
//!   — `mask` is only ever used in the trace log line). With the upstream
//!   `bo` record's `@asynMask($(PORT),6,0x01)` link this is unobservable
//!   (`value` is already 0/1), but it is not a masked write in the general
//!   asynUInt32Digital sense. Reproduced verbatim.
//! - **Serial-interface echo stripping**: the Colby device echoes the sent
//!   command line (terminated `\r\n`) before its actual reply when talking
//!   over serial (iface=1). `writeRead`'s serial branch skips
//!   `strlen(outBuf)+2` bytes into the framed reply, then takes everything
//!   up to the first `\r`/`\n` — see [`strip_serial_echo`]. `writeOnly`'s
//!   serial branch issues one extra blocking read afterward purely to drain
//!   that echo line; its result (success *or* failure) is completely
//!   discarded in the C source (not even assigned to a variable), which
//!   [`device_write_only`] reproduces via `let _ = ...`.
//! - **`NET?` reply parsing**: comma-split via `epicsStrtok_r` — consecutive
//!   delimiters never yield an empty token, reproduced by
//!   [`split_net_reply`] filtering empty splits.
//! - **`DEL?`/`STEP?` reply parsing**: `sscanf(inpBuf,"%e",&readback)` into a
//!   32-bit C `float` (not `double`) — see [`crate::wire::atof_f32`] and
//!   [`scale_seconds_to_units`]. A reply with no leading numeric token leaves
//!   C's `readback` uninitialized (undefined behavior); this port substitutes
//!   the defined value `0.0`, which is not itself a wire-protocol fabrication
//!   since real device replies always carry a numeric token.
//! - **This driver does not replicate the C driver's process-wide singleton
//!   restriction** (`if(pports) return -1` in `drvAsynColby` — the C driver
//!   can create only *one* Colby port total, system-wide, regardless of
//!   name, because its state was a single static `Port*`). That restriction
//!   is an artifact of the original static-storage implementation, not
//!   observable wire protocol; asyn-rs's `PortManager`/`asyn_record`
//!   registries already reject a duplicate port *name* on their own.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;

use crate::connect::OctetLink;
use epics_rs::asyn::user::AsynUser;

use crate::wire::{self, atof_f32, write_only, write_read};

/// C `IFACE_ETHER`/`IFACE_SERIAL` — selects whether [`device_write_only`]/
/// [`device_write_read`] perform the extra echo-draining/echo-stripping
/// steps.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Iface {
    Ether,
    Serial,
}

impl Iface {
    /// C's validation `switch(iface){case IFACE_ETHER: case IFACE_SERIAL:
    /// break; default: ... return(-1);}` in `drvAsynColby()` — only `0`
    /// (Ethernet) or `1` (Serial) is accepted.
    pub fn from_arg(iface: i32) -> Option<Self> {
        match iface {
            0 => Some(Iface::Ether),
            1 => Some(Iface::Serial),
            _ => None,
        }
    }
}

/// C `epicsStrCaseCmp(pport->units,"ns")==0` — case-insensitive; any string
/// that isn't (case-insensitively) exactly `"ns"` is treated as `"ps"`.
fn is_ns(units: &str) -> bool {
    units.eq_ignore_ascii_case("ns")
}

/// C `sprintf(outBuf,"%s %-.3f %s",pcmd,value,pport->units)` — `%-.3f` with
/// no field width prints identically to `%.3f`.
fn format_float_cmd(pcmd: &str, value: f64, units: &str) -> String {
    format!("{pcmd} {value:.3} {units}")
}

/// C `(epicsFloat64)readback/1.0E-9` (ns) or `/1.0E-12` (ps) — `readback` is
/// the device's raw seconds value; dividing by 1ns/1ps converts it to the
/// configured engineering unit.
fn scale_seconds_to_units(readback: f32, units: &str) -> f64 {
    let readback = readback as f64;
    if is_ns(units) {
        readback / 1.0E-9
    } else {
        readback / 1.0E-12
    }
}

/// C `writeRead()`'s serial-only post-processing:
/// `epicsStrtok_r(&inpBuf[strlen(outBuf)+2], IFACE_TERM, &next)`. The Colby
/// device echoes the sent command line (terminated `\r\n`) before its actual
/// reply when talking over serial; this isolates the reply.
fn strip_serial_echo(raw: &str, sent_cmd: &str) -> String {
    let offset = sent_cmd.len() + 2;
    let rest = raw.get(offset..).unwrap_or("");
    let end = rest.find(['\r', '\n']).unwrap_or(rest.len());
    rest[..end].to_string()
}

/// C `readItRaw`'s `NET?` comma-split via `epicsStrtok_r(inpBuf,",",&next)`
/// — consecutive delimiters never yield an empty token.
fn split_net_reply(reply: &str) -> Vec<String> {
    reply
        .split(',')
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .collect()
}

/// C `writeRead(pasynUser,outBuf,inpBuf,inputSize,iface)`.
fn device_write_read(handle: &OctetLink, iface: Iface, cmd: &str) -> AsynResult<String> {
    let raw = write_read(handle, cmd).inspect_err(|f| {
        handle.reset();
        handle.print(
            TraceMask::ERROR,
            format_args!(
                "writeRead: error sending {cmd},sent={},recv={},err={}",
                f.sent,
                f.received,
                f.status()
            ),
        )
    })?;
    Ok(if iface == Iface::Serial {
        strip_serial_echo(&raw, cmd)
    } else {
        raw
    })
}

/// C `writeOnly(pasynUser,outBuf,iface)`. The serial-only drain read runs
/// whatever the write returned, and C discards its result (not even
/// assigned to a variable) — `let _ = ...` reproduces that verbatim.
fn device_write_only(handle: &OctetLink, iface: Iface, cmd: &str) -> AsynResult<()> {
    let result = write_only(handle, cmd).inspect_err(|f| {
        handle.reset();
        handle.print(
            TraceMask::ERROR,
            format_args!(
                "writeOnly: error sending {cmd},sent={},err={}",
                f.sent,
                f.status()
            ),
        )
    });
    if iface == Iface::Serial {
        let _ = wire::read_only(handle);
    }
    Ok(result?)
}

/// Colby Instruments PDL-100A port driver state (C `struct Port`).
pub struct ColbyDriver {
    base: PortDriverBase,
    handle: OctetLink,
    iface: Iface,
    /// C `pport->units` — a free-form string from the iocsh `units` arg,
    /// normalized to literal `"ns"`/`"ps"` only by an addr-6 write. Scaling
    /// and the addr-6 read both classify via [`is_ns`] (anything not
    /// case-insensitively `"ns"` behaves as `"ps"`), matching C exactly.
    units: String,
    /// C `pport->ident` — captured once at init from `*IDN?`, empty when it
    /// went unanswered; see the addr-7 quirk note above for why this is
    /// otherwise unused outside `report()`.
    ident: String,
}

impl ColbyDriver {
    /// C `drvAsynColby(myport,ioport,addr,units,iface)` up to the
    /// interface registrations: no traffic yet.
    pub fn new(port_name: &str, handle: OctetLink, units: &str, iface: Iface) -> Self {
        let base = PortDriverBase::new(
            port_name,
            16,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        Self {
            base,
            handle,
            iface,
            units: units.to_string(),
            ident: String::new(),
        }
    }

    /// The rest of C `drvAsynColby`: the `*IDN?` query. C returns -1 on its
    /// failure with no line of its own and no init flag, so the port keeps
    /// serving every transfer.
    pub fn initialize(&mut self) -> AsynResult<()> {
        self.ident = device_write_read(&self.handle, self.iface, "*IDN?")?;
        Ok(())
    }
}

impl PortDriver for ColbyDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// C `create()` (asynDrvUser): never reads `drvInfo`, dispatch is purely
    /// by `pasynManager->getAddr`. The shared reason value is meaningless
    /// here since every read/write override below branches on `user.addr`.
    fn drv_user_create(&mut self, _req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        Ok(DrvUserInfo::from_reason(0))
    }

    /// C `report()`: `fprintf(fp,"    %s units %s\n",pport->ident,pport->units);`.
    fn report(&mut self, out: &mut dyn std::fmt::Write, _level: i32) {
        let _ = writeln!(out, "    {} units {}", self.ident, self.units);
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        self.do_read_float64(user)
            .map_err(|_| wire::record_error(AsynStatus::Error))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.do_write_float64(user, value)
            .map_err(|_| wire::record_error(AsynStatus::Error))
    }

    fn read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        self.do_read_uint32_digital(user, mask)
            .map_err(|_| wire::record_error(AsynStatus::Error))
    }

    fn write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        mask: u32,
    ) -> AsynResult<()> {
        self.do_write_uint32_digital(user, value, mask)
            .map_err(|_| wire::record_error(AsynStatus::Error))
    }

    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        self.do_read_octet(user, buf)
            .map_err(|_| wire::record_error(AsynStatus::Error))
    }

    /// C `writeItRaw`: always a no-op, `*nbytes=0` unconditionally — no
    /// `getAddr`/switch at all, unlike every other interface method here.
    fn write_octet(&mut self, _user: &mut AsynUser, _data: &[u8]) -> AsynResult<usize> {
        Ok(0)
    }
}

/// The C transfers; [`PortDriver`] hands their errors to the record as
/// [`wire::record_error`].
impl ColbyDriver {
    fn do_read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let cmd = match user.addr {
            0 | 1 => "DEL?",
            4 | 5 => "STEP?",
            other => return Err(AsynError::AddressOutOfRange(other)),
        };
        let reply = device_write_read(&self.handle, self.iface, cmd)?;
        let readback = atof_f32(&reply);
        Ok(scale_seconds_to_units(readback, &self.units))
    }

    fn do_write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let pcmd = match user.addr {
            0 => "DEL",
            4 => "STEP",
            other => return Err(AsynError::AddressOutOfRange(other)),
        };
        let cmd = format_float_cmd(pcmd, value, &self.units);
        device_write_only(&self.handle, self.iface, &cmd)
    }

    fn do_read_uint32_digital(&mut self, user: &AsynUser, mask: u32) -> AsynResult<u32> {
        match user.addr {
            // C: `*value = 0;` unconditionally — the device's actual
            // increment/decrement state is never queried back.
            2 | 3 => Ok(0),
            6 => Ok((if is_ns(&self.units) { 0u32 } else { 1u32 }) & mask),
            other => Err(AsynError::AddressOutOfRange(other)),
        }
    }

    fn do_write_uint32_digital(
        &mut self,
        user: &mut AsynUser,
        value: u32,
        _mask: u32,
    ) -> AsynResult<()> {
        let cmd = match user.addr {
            2 => "INC",
            3 => "DEC",
            6 => {
                self.units = if value == 0 { "ns" } else { "ps" }.to_string();
                return Ok(());
            }
            14 => "*RST",
            15 => "*CAL",
            other => return Err(AsynError::AddressOutOfRange(other)),
        };
        device_write_only(&self.handle, self.iface, cmd)
    }

    fn do_read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let text = match user.addr {
            8..=12 => {
                let reply = device_write_read(&self.handle, self.iface, "NET?")?;
                let tokens = split_net_reply(&reply);
                let idx = (user.addr - 8) as usize;
                tokens
                    .get(idx)
                    .cloned()
                    .ok_or_else(|| wire::record_error(AsynStatus::Error))?
            }
            13 => device_write_read(&self.handle, self.iface, "NETM?")?,
            other => return Err(AsynError::AddressOutOfRange(other)),
        };
        let bytes = text.as_bytes();
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iface_from_arg_accepts_only_0_or_1() {
        assert_eq!(Iface::from_arg(0), Some(Iface::Ether));
        assert_eq!(Iface::from_arg(1), Some(Iface::Serial));
        assert_eq!(Iface::from_arg(2), None);
        assert_eq!(Iface::from_arg(-1), None);
    }

    #[test]
    fn is_ns_matches_c_epicsstrcasecmp() {
        assert!(is_ns("ns"));
        assert!(is_ns("NS"));
        assert!(is_ns("Ns"));
        assert!(!is_ns("ps"));
        assert!(!is_ns("PS"));
        assert!(!is_ns("garbage"));
    }

    #[test]
    fn format_float_cmd_matches_c_sprintf_precision() {
        assert_eq!(format_float_cmd("DEL", 5.0, "ns"), "DEL 5.000 ns");
        assert_eq!(format_float_cmd("STEP", -1.5, "ps"), "STEP -1.500 ps");
        assert_eq!(format_float_cmd("DEL", 0.12345, "ns"), "DEL 0.123 ns");
    }

    #[test]
    fn scale_seconds_to_units_converts_ns_and_ps() {
        assert!((scale_seconds_to_units(5.0e-9, "ns") - 5.0).abs() < 1e-6);
        assert!((scale_seconds_to_units(5.0e-12, "ps") - 5.0).abs() < 1e-6);
        // Anything not case-insensitively "ns" scales as ps.
        assert!((scale_seconds_to_units(5.0e-12, "garbage") - 5.0).abs() < 1e-6);
    }

    #[test]
    fn strip_serial_echo_isolates_reply_after_command_echo() {
        // Raw frame: "<echoed DEL?>\r\n<reply>" (port's own colon EOS
        // already stripped by the framework before this function sees it).
        let raw = "DEL?\r\n5.000000E-09";
        assert_eq!(strip_serial_echo(raw, "DEL?"), "5.000000E-09");
    }

    #[test]
    fn strip_serial_echo_stops_at_embedded_newline() {
        let raw = "STEP?\r\n1.000000E-12\r\nextra";
        assert_eq!(strip_serial_echo(raw, "STEP?"), "1.000000E-12");
    }

    #[test]
    fn split_net_reply_skips_empty_tokens_like_strtok() {
        assert_eq!(
            split_net_reply("10.0.0.1,10.0.0.254,255.255.255.0,7000,0"),
            vec!["10.0.0.1", "10.0.0.254", "255.255.255.0", "7000", "0"]
        );
        assert_eq!(split_net_reply("a,,b"), vec!["a", "b"]);
    }
}

#[cfg(test)]
mod init_tests {
    use super::*;
    use crate::connect::mute_link;

    /// C has no init flag: an unanswered `*IDN?` leaves the ident empty and
    /// the port serving.
    #[test]
    fn a_failed_identification_keeps_the_port_serving() {
        let mut colby =
            ColbyDriver::new("COLBY_MUTE", mute_link("COLBY_MUTE_IO"), "ns", Iface::Ether);
        assert!(colby.initialize().is_err());
        assert_eq!(colby.ident, "");
        let units = colby.read_uint32_digital(&AsynUser::default().with_addr(6), 1);
        assert_eq!(units.unwrap(), 0);
    }

    /// C returns a bare asynError and never writes the record's errorMessage.
    #[test]
    fn a_failed_transfer_reaches_the_record_as_bare_asyn_error() {
        let mut colby = ColbyDriver::new(
            "COLBY_MUTE2",
            mute_link("COLBY_MUTE2_IO"),
            "ns",
            Iface::Ether,
        );
        for addr in [0, 7] {
            let e = colby
                .read_float64(&AsynUser::default().with_addr(addr))
                .unwrap_err();
            assert_eq!(e.message(), "");
            assert_eq!(e.status(), AsynStatus::Error);
        }
    }
}
