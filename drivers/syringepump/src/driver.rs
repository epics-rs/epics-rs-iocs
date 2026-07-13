//! Teledyne ISCO D/H-series syringe pump port driver: a single `PortDriver`
//! parameterized by [`Family`], translating `teled_d.proto`/`teled_h.proto`
//! ([`crate::wire_d`]/[`crate::wire_h`]) into asyn `Int32`/`Float64`/`Octet`
//! reads and writes. Named-parameter dispatch (`drv_user_create` matching a
//! command name string, mirroring real `asynPortDriver::createParam`/
//! `drvUserCreate`) follows `drivers/love`'s established pattern in this
//! workspace.
//!
//! # Addressing: pump letter, not per-record configuration
//! [`PUMP_LETTERS`] maps asyn `addr` (0-5) to the wire pump letter
//! positionally -- `A`,`B`,`C`,`D`,`AB`,`CD`, matching every `PUMP=` value
//! ISCO/Vindum's sibling `.substitutions` files instantiate (D-series;
//! H only ever ships `A`/`B` template instances upstream, but the `.proto`
//! itself places no restriction on the letter). Unlike `drivers/love`'s
//! per-address `Model` (a genuinely runtime-configurable fact about which
//! physical unit is wired to which address), a pump's letter is fixed by
//! which template instantiation created the record -- so no `*Config`
//! iocsh call is needed here; [`pump_letter`] is a pure positional lookup.
//!
//! # Preserved upstream bug: only D's `setRun` is pump-parameterized
//! `teledynePumpD.template`/`teledynePumpH.template` both hardcode
//! `$(p=A)` in every OUT/INP link **except** D's `setRun`, which alone uses
//! `$(p=$(PUMP))` (confirmed by grepping every `$(p=` occurrence in both
//! files). That means, upstream, instantiating either template a second
//! time for pump B talks to physical pump A on the wire for every command
//! but D's Run -- H's own `setRun` hardcodes `$(p=A)` too, on top of
//! `teled_h.proto`'s `setRun` being fully wire-literal to begin with (see
//! [`wire_h::format_set_run`]).
//!
//! This driver itself is NOT where that bug lives: every `do_*` method
//! below resolves the pump letter uniformly from `user.addr` via
//! [`pump_letter`], for every command, D or H, Run or otherwise -- a
//! correctly-parameterized implementation. The bug is reproduced instead in
//! `iocs/syringepump-ioc/db/teledynePumpD.template`/`teledynePumpH.template`,
//! which hardcode `addr=0` (pump "A") in the ported OUT/INP link text for
//! every command except D's `SetRun` (see those templates' module doc) --
//! matching upstream's own call-site bug rather than a protocol-level one.
//! Listed under this crate's UNFIXED note (see the crate root doc).
//!
//! # SendCmd: write-then-cached-read
//! `sendCmd`'s upstream wiring (`CmdReply`, a `waveform` record with only
//! `INP`, "written to" via a separate `stringout`+`aSub` `formatCmd`
//! subroutine that computes a 2-hex-digit length prefix client-side before
//! writing into `CmdReply.VAL`) has no direct asyn analogue -- asyn has no
//! "write triggers reformatting of this record's own VAL" convention.
//! Ported instead as a standard asyn write-then-read pair: [`write_octet`]
//! takes the raw, un-prefixed command text, computes the length prefix
//! itself (removing the client-side `formatCmd` step -- a documented,
//! wire-*equivalent* deviation: the exact same bytes reach the wire either
//! way), executes the round trip, and caches the reply string;
//! [`read_octet`] on the same reason returns that cached reply. Neither
//! `sendCmd` nor `ping` is wired into either ported db template (matching
//! upstream, which doesn't wire them either) -- `sendCmd` is still exposed
//! here for parity with `teled_d.proto`/`teled_h.proto`'s full command set,
//! `ping` is implemented in [`crate::wire_d`]/[`crate::wire_h`] with unit
//! tests but not exposed through `drv_user_create` at all (no db field
//! needs it, upstream or ported).

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::{wire_d, wire_h};

/// asyn reason used for octet transactions against the underlying serial/IP
/// port (mirrors `love`/`scaler974`'s `OCTET_REASON` convention).
const OCTET_REASON: usize = 0;

/// Generous fixed reply buffer -- the longest expected reply is a free-text
/// `getModel`/`getStatus`/`sendCmd` capture; `CmdReply`'s upstream waveform
/// is sized `NELM 80`.
const REPLY_BUF_SIZE: usize = 128;

/// Positional pump-letter table indexed by asyn `addr`. See the module doc.
pub const PUMP_LETTERS: [&str; 6] = ["A", "B", "C", "D", "AB", "CD"];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    D,
    H,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    SetRun,
    SetStop,
    SetRem,
    SetLoc,
    SetPress,
    SetMFlow,
    GetVol,
    GetPress,
    GetSetPress,
    GetFlow,
    GetMFlow,
    GetStatus,
    SendCmd,
    /// H only.
    GetUnit,
    /// H only.
    GetMode,
    /// H only.
    GetId,
    /// H only.
    SetRefill,
    /// H only.
    GetRefillLimit,
    /// H only.
    SetRefillRate,
    /// H only.
    SetDigital,
}

impl Command {
    fn available_for(self, family: Family) -> bool {
        use Command::*;
        match self {
            GetUnit | GetMode | GetId | SetRefill | GetRefillLimit | SetRefillRate | SetDigital => {
                family == Family::H
            }
            _ => true,
        }
    }
}

/// Command name (the `drvInfo` string a db link's `@asyn(port,addr)
/// CommandName` supplies), matched case-insensitively -- mirrors
/// `drivers/love::driver::COMMANDS`'s convention. Position in this table
/// *is* the asyn `reason`.
const COMMAND_NAMES: &[(&str, Command)] = &[
    ("SetRun", Command::SetRun),
    ("SetStop", Command::SetStop),
    ("SetRem", Command::SetRem),
    ("SetLoc", Command::SetLoc),
    ("SetPress", Command::SetPress),
    ("SetMFlow", Command::SetMFlow),
    ("GetVol", Command::GetVol),
    ("GetPress", Command::GetPress),
    ("GetSetPress", Command::GetSetPress),
    ("GetFlow", Command::GetFlow),
    ("GetMFlow", Command::GetMFlow),
    ("GetStatus", Command::GetStatus),
    ("SendCmd", Command::SendCmd),
    ("GetUnit", Command::GetUnit),
    ("GetMode", Command::GetMode),
    ("GetID", Command::GetId),
    ("SetRefill", Command::SetRefill),
    ("GetRefillLimit", Command::GetRefillLimit),
    ("SetRefillRate", Command::SetRefillRate),
    ("SetDigital", Command::SetDigital),
];

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

fn pump_letter(addr: i32) -> AsynResult<&'static str> {
    PUMP_LETTERS
        .get(addr as usize)
        .copied()
        .ok_or(AsynError::AddressOutOfRange(addr))
}

/// Teledyne D/H-series port driver state.
pub struct TeledyneDriver {
    base: PortDriverBase,
    handle: SyncIOHandle,
    family: Family,
    /// `\$1` in both `.proto` files -- the destination unit digit. Every
    /// shipped db template macro defaults to `$(u=6)` and nothing upstream
    /// ever overrides it; modeled here as a per-driver-instance value (set
    /// once at `TeledyneDInit`/`TeledyneHInit`, matching a real RS-485
    /// controller's fixed unit-number DIP switch) rather than a per-record
    /// concept.
    unit: u8,
    /// Cache for `sendCmd`'s write-then-read pairing; see the module doc's
    /// "SendCmd" section.
    last_send_cmd_reply: Option<String>,
}

impl TeledyneDriver {
    pub fn new(port_name: &str, handle: SyncIOHandle, family: Family, unit: u8) -> Self {
        let base = PortDriverBase::new(
            port_name,
            PUMP_LETTERS.len(),
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        Self {
            base,
            handle,
            family,
            unit,
            last_send_cmd_reply: None,
        }
    }

    fn command(&self, reason: usize) -> AsynResult<Command> {
        COMMAND_NAMES
            .get(reason)
            .map(|(_, c)| *c)
            .ok_or_else(|| protocol_error("invalid reason"))
    }

    fn write_frame(&self, out: &str) -> AsynResult<()> {
        self.handle.write_octet(OCTET_REASON, out.as_bytes())?;
        Ok(())
    }

    fn execute(&self, out: &str) -> AsynResult<Vec<u8>> {
        self.write_frame(out)?;
        self.handle.read_octet(OCTET_REASON, REPLY_BUF_SIZE)
    }

    fn parse_ack(&self, reply: &[u8]) -> AsynResult<()> {
        match self.family {
            Family::D => wire_d::parse_ack(reply).map_err(|e| protocol_error(format!("{e:?}"))),
            Family::H => wire_h::parse_ack(reply).map_err(|e| protocol_error(format!("{e:?}"))),
        }
    }
}

impl PortDriver for TeledyneDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn drv_user_create(&mut self, drv_info: &str, addr: i32) -> AsynResult<DrvUserInfo> {
        if !(0..PUMP_LETTERS.len() as i32).contains(&addr) {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        for (i, (name, cmd)) in COMMAND_NAMES.iter().enumerate() {
            if name.eq_ignore_ascii_case(drv_info) {
                if !cmd.available_for(self.family) {
                    return Err(protocol_error(format!(
                        "{drv_info} not supported by {:?}-series",
                        self.family
                    )));
                }
                return Ok(DrvUserInfo::from_reason(i));
            }
        }
        Err(protocol_error(format!(
            "failure to find command {drv_info}"
        )))
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let cmd = self.command(user.reason)?;
        let pump = pump_letter(user.addr)?;
        match (self.family, cmd) {
            (Family::D, Command::GetVol) => {
                let reply = self.execute(&wire_d::format_get_vol(self.unit, pump))?;
                wire_d::parse_get_vol(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            (Family::D, Command::GetPress) => {
                let reply = self.execute(&wire_d::format_get_press(self.unit, pump))?;
                wire_d::parse_get_press(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            (Family::D, Command::GetSetPress) => {
                let reply = self.execute(&wire_d::format_get_set_press(self.unit, pump))?;
                wire_d::parse_get_set_press(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            (Family::D, Command::GetFlow) => {
                let reply = self.execute(&wire_d::format_get_flow(self.unit, pump))?;
                wire_d::parse_get_flow(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            (Family::D, Command::GetMFlow) => {
                let reply = self.execute(&wire_d::format_get_mflow(self.unit, pump))?;
                wire_d::parse_get_mflow(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            (Family::H, Command::GetRefillLimit) => {
                let reply = self.execute(&wire_h::format_get_refill_limit(self.unit, pump))?;
                wire_h::parse_get_refill_limit(&reply).map_err(|e| protocol_error(format!("{e:?}")))
            }
            _ => Err(protocol_error(format!(
                "{cmd:?} does not support Float64 read on {:?}-series",
                self.family
            ))),
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let cmd = self.command(user.reason)?;
        let pump = pump_letter(user.addr)?;
        let out = match (self.family, cmd) {
            (Family::D, Command::SetPress) => wire_d::format_set_press(self.unit, pump, value),
            (Family::H, Command::SetPress) => wire_h::format_set_press(self.unit, pump, value),
            (Family::D, Command::SetMFlow) => wire_d::format_set_mflow(self.unit, pump, value),
            (Family::H, Command::SetMFlow) => wire_h::format_set_mflow(self.unit, pump, value),
            (Family::H, Command::SetRefillRate) => wire_h::format_set_refill_rate(self.unit, value),
            _ => {
                return Err(protocol_error(format!(
                    "{cmd:?} does not support Float64 write on {:?}-series",
                    self.family
                )));
            }
        };
        let reply = self.execute(&out)?;
        self.parse_ack(&reply)
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let cmd = self.command(user.reason)?;
        let pump = pump_letter(user.addr)?;

        // setDigital has no `in` clause upstream (fire-and-forget) -- must
        // not attempt a read after writing it.
        if self.family == Family::H && cmd == Command::SetDigital {
            let choice = u8::try_from(value).map_err(|_| {
                protocol_error(format!("SetDigital: selector {value} out of range"))
            })?;
            let out = wire_h::format_set_digital(self.unit, choice)
                .map_err(|e| protocol_error(format!("{e:?}")))?;
            return self.write_frame(&out);
        }

        let out = match (self.family, cmd) {
            (Family::D, Command::SetRun) => wire_d::format_set_run(self.unit, pump),
            (Family::H, Command::SetRun) => wire_h::format_set_run(self.unit, pump),
            (Family::D, Command::SetStop) => wire_d::format_set_stop(self.unit, pump),
            (Family::H, Command::SetStop) => wire_h::format_set_stop(self.unit, pump),
            (Family::D, Command::SetRem) => wire_d::format_set_rem(self.unit),
            (Family::H, Command::SetRem) => wire_h::format_set_rem(self.unit),
            (Family::D, Command::SetLoc) => wire_d::format_set_loc(self.unit),
            (Family::H, Command::SetLoc) => wire_h::format_set_loc(self.unit),
            (Family::H, Command::SetRefill) => wire_h::format_set_refill(self.unit),
            _ => {
                return Err(protocol_error(format!(
                    "{cmd:?} does not support Int32 write on {:?}-series",
                    self.family
                )));
            }
        };
        let reply = self.execute(&out)?;
        self.parse_ack(&reply)
    }

    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let cmd = self.command(user.reason)?;
        let pump = pump_letter(user.addr)?;

        if cmd == Command::SendCmd {
            let text = self.last_send_cmd_reply.take().ok_or_else(|| {
                protocol_error("SendCmd: no reply cached -- write a command first")
            })?;
            let bytes = text.as_bytes();
            let n = bytes.len().min(buf.len());
            buf[..n].copy_from_slice(&bytes[..n]);
            return Ok(n);
        }

        let text = match (self.family, cmd) {
            (Family::D, Command::GetStatus) => {
                let reply = self.execute(&wire_d::format_get_status(self.unit, pump))?;
                wire_d::parse_get_status(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetStatus) => {
                let reply = self.execute(&wire_h::format_get_status(self.unit, pump))?;
                wire_h::parse_get_status(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetUnit) => {
                let reply = self.execute(&wire_h::format_get_unit(self.unit))?;
                wire_h::parse_captured_string(&reply)
                    .map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetMode) => {
                let reply = self.execute(&wire_h::format_get_mode(self.unit))?;
                wire_h::parse_captured_string(&reply)
                    .map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetId) => {
                let reply = self.execute(&wire_h::format_get_id(self.unit))?;
                wire_h::parse_captured_string(&reply)
                    .map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetVol) => {
                let reply = self.execute(&wire_h::format_get_vol(self.unit, pump))?;
                wire_h::parse_get_vol(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetPress) => {
                let reply = self.execute(&wire_h::format_get_press(self.unit, pump))?;
                wire_h::parse_get_press(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetSetPress) => {
                let reply = self.execute(&wire_h::format_get_set_press(self.unit, pump))?;
                wire_h::parse_get_set_press(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetFlow) => {
                let reply = self.execute(&wire_h::format_get_flow(self.unit, pump))?;
                wire_h::parse_get_flow(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            (Family::H, Command::GetMFlow) => {
                let reply = self.execute(&wire_h::format_get_mflow(self.unit, pump))?;
                wire_h::parse_get_mflow(&reply).map_err(|e| protocol_error(format!("{e:?}")))?
            }
            _ => {
                return Err(protocol_error(format!(
                    "{cmd:?} does not support Octet read on {:?}-series",
                    self.family
                )));
            }
        };
        let bytes = text.as_bytes();
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }

    /// `sendCmd`'s write half -- see the module doc's "SendCmd" section.
    /// `data` is the raw, un-prefixed command text (e.g. `"IDENTIFY"`); the
    /// 2-hex-digit length prefix `teled_{d,h}.proto`'s `sendCmd` requires as
    /// the first 2 characters of its `%s` argument is computed here.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let cmd = self.command(user.reason)?;
        if cmd != Command::SendCmd {
            return Err(protocol_error(format!(
                "{cmd:?} does not support Octet write"
            )));
        }
        let text = std::str::from_utf8(data)
            .map_err(|_| protocol_error("SendCmd: command text must be valid UTF-8"))?;
        let len_prefix = format!("{:02X}", text.len().min(0xFF));
        let caller_body = format!("{len_prefix}{text}");
        let out = match self.family {
            Family::D => wire_d::format_send_cmd(self.unit, &caller_body),
            Family::H => wire_h::format_send_cmd(self.unit, &caller_body),
        };
        let reply = self.execute(&out)?;
        let parsed =
            match self.family {
                Family::D => wire_d::parse_captured_string(&reply)
                    .map_err(|e| protocol_error(format!("{e:?}"))),
                Family::H => wire_h::parse_send_cmd_reply(&reply)
                    .map_err(|e| protocol_error(format!("{e:?}"))),
            }?;
        self.last_send_cmd_reply = Some(parsed);
        Ok(data.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_available_for_gates_h_only_commands() {
        assert!(!Command::GetUnit.available_for(Family::D));
        assert!(Command::GetUnit.available_for(Family::H));
        assert!(Command::SetRun.available_for(Family::D));
        assert!(Command::SetRun.available_for(Family::H));
    }

    #[test]
    fn pump_letter_resolves_positionally() {
        assert_eq!(pump_letter(0).unwrap(), "A");
        assert_eq!(pump_letter(1).unwrap(), "B");
        assert_eq!(pump_letter(5).unwrap(), "CD");
        assert!(pump_letter(6).is_err());
        assert!(pump_letter(-1).is_err());
    }

    #[test]
    fn command_names_are_unique_and_match_case_insensitively() {
        let mut seen = std::collections::HashSet::new();
        for (name, _) in COMMAND_NAMES {
            assert!(seen.insert(name.to_ascii_lowercase()), "duplicate: {name}");
        }
        assert!(
            COMMAND_NAMES
                .iter()
                .any(|(n, _)| n.eq_ignore_ascii_case("setrun"))
        );
    }
}
