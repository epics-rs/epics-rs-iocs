//! Dynamic `DeviceSupport` for all 8 MW100 record types
//! (`devMW100_{ai,ao,bi,bo,mbbi,mbbo,longin,stringin}.c`).
//!
//! Structurally this mirrors `yokogawa-gm10`'s `device_support.rs` (one
//! shared DTYP string, `resolve_operation` as the single owner of every
//! `devMW100_*.c::init_record`'s command/address grammar, lazy resolution in
//! `DeviceSupport::init` since only there is the concrete record type
//! known) — but the grammar itself is NOT a copy of GM10's: every rule below
//! was re-derived from the actual MW100 C sources, and two things are
//! genuinely different:
//!
//! - `ao`'s address switch (`devMW100_ao.c::init_record`) has no `A` (Math)
//!   arm at all, falling through to `default: return 1` — unlike GM10's
//!   `ao`, which accepts Math at `init_record` and only rejects it later, in
//!   `write()`. `parse_address_no_math` rejects Math structurally at
//!   resolve time to match.
//! - IOSCANPVT routing goes through ONE family-generic dispatcher,
//!   `mw100_channel_io_handler(dq, family, channel)` (`drvMW100.c:1643-1660`),
//!   shared by `ai`'s VAL (all 4 families) and `mbbi`'s VAL_STATUS/ALARM/
//!   ALARMS/CH_STATUS's *read* path (Signal-or-Math only) — Signal routes
//!   dynamically via [`Instrument::signal_interrupt_category`], Math is
//!   always Input, Comm/Const are always Output. `family_category` is that
//!   one dispatcher, reused everywhere the C source reuses it.

use crate::codec;
use crate::instrument::{Command, Instrument, InterruptCategory, Registry};
use crate::link::{self, ChannelAddress, ChannelFamily};
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::runtime::sync::mpsc::{self, Receiver, Sender};
use epics_rs::base::server::database::DeviceDoneSource;
use epics_rs::base::server::device_support::{
    DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, WriteStart,
};
use epics_rs::base::server::record::{Record, ScanType};
use epics_rs::base::types::EpicsValue;
use epics_rs::ca::server::ioc_app::DeviceSupportContext;
use std::sync::Arc;

/// Upstream DTYP shared by every MW100 record type (`mw100Support.dbd`).
pub const DTYP: &str = "Yokogawa MW100";

/// One variant per legal `(record type, link command)` pair validated by a
/// `devMW100_*.c::init_record`.
#[derive(Debug, Clone, Copy)]
enum Operation {
    // -- ai (devMW100_ai.c): VAL, any of the 4 address families. --
    AnalogVal(ChannelAddress),
    // -- ao (devMW100_ao.c): VAL, Signal/Comm/Const only (no Math arm). --
    AnalogSet(ChannelAddress),
    // -- bi (devMW100_bi.c) --
    BinaryVal(u32),
    ModulePresence(usize),
    SettingsMode,
    MeasurementModeFlag,
    ComputeModeFlag,
    ErrorFlag,
    AlarmFlag,
    // -- bo (devMW100_bo.c): get_ioint_info is NULL for every command. --
    BinarySet(u32),
    InputTrig,
    OutputTrig,
    InfoTrig,
    StatTrig,
    OpModeSet,
    ErrorClearSet,
    AlarmAckSet,
    // -- mbbi (devMW100_mbbi.c) --
    ModuleSpeed(usize),
    ChStatus(ChannelAddress),
    ChMode(u32),
    ValStatus(ChannelAddress),
    Alarm(ChannelAddress, u8),
    Alarms(ChannelAddress),
    // -- mbbo (devMW100_mbbo.c) --
    ComputeCmdSet,
    // -- longin (devMW100_longin.c): VAL, Signal only. --
    IntegerVal(u32),
    ModuleModel(usize),
    ModuleNumber(usize),
    // -- stringin (devMW100_stringin.c) --
    IpAddr,
    ModuleCode(usize),
    ModuleString(usize),
    Unit(ChannelAddress),
    ErrorText(u32),
    Expr(u32),
}

/// `mw100_channel_io_handler`'s family dispatch (`drvMW100.c:1643-1660`):
/// Signal depends on the channel's live `ChannelType`, Math is always Input,
/// Comm/Const are always Output. Shared by every operation whose C
/// `get_ioint_info` routes through this one handler, regardless of which
/// record type asks (`ai`'s VAL and `mbbi`'s VAL_STATUS/ALARM/ALARMS/
/// CH_STATUS all reach it the same way).
fn family_category(
    family: ChannelFamily,
    index: u32,
    instrument: &Instrument,
) -> InterruptCategory {
    match family {
        ChannelFamily::Signal => instrument.signal_interrupt_category(index),
        ChannelFamily::Math => InterruptCategory::Input,
        ChannelFamily::Comm | ChannelFamily::Const => InterruptCategory::Output,
    }
}

/// `None` for the write-only (bo/mbbo) operations, matching their C dset
/// registering `get_ioint_info: NULL`.
fn operation_interrupt_category(
    op: Operation,
    instrument: &Instrument,
) -> Option<InterruptCategory> {
    Some(match op {
        Operation::AnalogVal(addr) => family_category(addr.family, addr.index, instrument),
        Operation::IntegerVal(channel) | Operation::BinaryVal(channel) => {
            family_category(ChannelFamily::Signal, channel, instrument)
        }
        Operation::ModulePresence(_) => InterruptCategory::Info,
        Operation::SettingsMode | Operation::MeasurementModeFlag | Operation::ComputeModeFlag => {
            InterruptCategory::Status
        }
        Operation::ErrorFlag => InterruptCategory::Error,
        Operation::AlarmFlag => InterruptCategory::Input,
        Operation::ModuleSpeed(_) => InterruptCategory::Info,
        Operation::ChStatus(_) | Operation::ChMode(_) => InterruptCategory::Info,
        Operation::ValStatus(addr) | Operation::Alarm(addr, _) | Operation::Alarms(addr) => {
            family_category(addr.family, addr.index, instrument)
        }
        Operation::IpAddr
        | Operation::ModuleModel(_)
        | Operation::ModuleNumber(_)
        | Operation::ModuleCode(_)
        | Operation::ModuleString(_)
        | Operation::Unit(_)
        | Operation::Expr(_) => InterruptCategory::Info,
        Operation::ErrorText(_) => InterruptCategory::Error,
        Operation::AnalogSet(_)
        | Operation::BinarySet(_)
        | Operation::InputTrig
        | Operation::OutputTrig
        | Operation::InfoTrig
        | Operation::StatTrig
        | Operation::OpModeSet
        | Operation::ErrorClearSet
        | Operation::AlarmAckSet
        | Operation::ComputeCmdSet => return None,
    })
}

fn bad(record_type: &str, msg: impl std::fmt::Display) -> CaError {
    CaError::LinkError(format!("MW100 {record_type} link: {msg}"))
}

fn require_no_arg(arg: Option<&str>, record_type: &str, command: &str) -> CaResult<()> {
    if arg.is_some() {
        return Err(bad(record_type, format!("{command} takes no argument")));
    }
    Ok(())
}

/// `MODULE_PRESENCE` (bi) / `MODULE_STRING` (stringin): a bare 0-5 module
/// index (`MAX_MODULES`), with the same first-byte 0-5 check
/// every `devMW100_*.c::init_record` applies before `atoi` (`atoi` returning
/// 0 on a non-numeric argument is ambiguous with the genuinely valid address
/// 0).
fn parse_module_index(arg: Option<&str>, record_type: &str, command: &str) -> CaResult<usize> {
    let arg = arg.ok_or_else(|| bad(record_type, format!("{command} requires an argument")))?;
    if !matches!(arg.as_bytes().first(), Some(b'0'..=b'5')) {
        return Err(bad(
            record_type,
            format!("{command} argument must start with 0-5"),
        ));
    }
    let i = codec::c_atoi(arg.as_bytes());
    if !(0..=5).contains(&i) {
        return Err(bad(
            record_type,
            format!("{command} module index out of range 0-5"),
        ));
    }
    Ok(i as usize)
}

/// `MODULE_SPEED` (mbbi) / `MODULE_CODE` (stringin) / `MODULE_MODEL` /
/// `MODULE_NUMBER` (longin): a module index gated on `mw100_test_module`,
/// unlike `MODULE_PRESENCE`/`MODULE_STRING` which report presence itself and
/// so bypass the gate.
fn parse_gated_module_index(
    arg: Option<&str>,
    record_type: &str,
    command: &str,
    instrument: &Instrument,
) -> CaResult<usize> {
    let module = parse_module_index(arg, record_type, command)?;
    if instrument.test_module(module) {
        return Err(bad(record_type, "module does not exist"));
    }
    Ok(module)
}

/// `VAL` on bi/bo/longin: a bare Signal (digit) address only — every one of
/// those three files' `switch(arg[0])` has only digit cases, no `A`/`C`/`K`
/// arm at all.
fn parse_signal_only(arg: Option<&str>, record_type: &str, command: &str) -> CaResult<u32> {
    let arg = arg.ok_or_else(|| {
        bad(
            record_type,
            format!("{command} requires a channel argument"),
        )
    })?;
    let addr = link::parse_channel_address(arg)
        .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
    if addr.family != ChannelFamily::Signal {
        return Err(bad(
            record_type,
            format!("{command} requires a Signal address"),
        ));
    }
    Ok(addr.index)
}

/// `VAL` on `ao` (`devMW100_ao.c::init_record`): Signal/Comm/Const only. The
/// address switch has no `A` (Math) arm at all, falling through to
/// `default: return 1` — unlike GM10's `ao`, which accepts Math at resolve
/// time and rejects it only in `write()`. Rejecting here, structurally, is
/// the correct port of what the C source actually does.
fn parse_address_no_math(
    arg: Option<&str>,
    record_type: &str,
    command: &str,
) -> CaResult<ChannelAddress> {
    let arg = arg.ok_or_else(|| {
        bad(
            record_type,
            format!("{command} requires an address argument"),
        )
    })?;
    let addr = link::parse_channel_address(arg)
        .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
    if addr.family == ChannelFamily::Math {
        return Err(bad(
            record_type,
            format!("{command} does not support Math addresses"),
        ));
    }
    Ok(addr)
}

/// Shared Signal-or-Math address validation for mbbi's CH_STATUS/CH_MODE/
/// VAL_STATUS/ALARM/ALARMS and stringin's UNIT (every one of those address
/// switches has only digit/`A` arms, no `C`/`K` at all), plus the
/// Signal-existence gate (`mw100_test_signal`) applied uniformly regardless
/// of which of those commands is asking.
fn validate_signal_or_math_address(
    addr: ChannelAddress,
    record_type: &str,
    instrument: &Instrument,
) -> CaResult<ChannelAddress> {
    if matches!(addr.family, ChannelFamily::Comm | ChannelFamily::Const) {
        return Err(bad(
            record_type,
            "command does not support Comm/Const addresses",
        ));
    }
    if addr.family == ChannelFamily::Signal && instrument.test_signal(addr.index) {
        return Err(bad(record_type, "channel does not exist"));
    }
    Ok(addr)
}

fn parse_signal_or_math_address(
    arg: Option<&str>,
    record_type: &str,
    command: &str,
    instrument: &Instrument,
) -> CaResult<ChannelAddress> {
    let arg = arg.ok_or_else(|| {
        bad(
            record_type,
            format!("{command} requires an address argument"),
        )
    })?;
    let addr = link::parse_channel_address(arg)
        .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
    validate_signal_or_math_address(addr, record_type, instrument)
}

/// The single owner of every `devMW100_*.c::init_record` command/address
/// validation, keyed by the concrete record's `record_type()` since
/// `DeviceSupportContext` cannot supply it.
fn resolve_operation(
    record_type: &str,
    command: &str,
    arg: Option<&str>,
    instrument: &Instrument,
) -> CaResult<Operation> {
    match record_type {
        "ai" => {
            if command != "VAL" {
                return Err(bad(record_type, "only VAL is supported"));
            }
            let arg = arg.ok_or_else(|| bad(record_type, "VAL requires an address argument"))?;
            let addr = link::parse_channel_address(arg)
                .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
            if addr.family == ChannelFamily::Signal && instrument.test_analog_signal(addr.index) {
                return Err(bad(record_type, "channel is not an analog signal"));
            }
            Ok(Operation::AnalogVal(addr))
        }
        "ao" => {
            if command != "VAL" {
                return Err(bad(record_type, "only VAL is supported"));
            }
            let addr = parse_address_no_math(arg, record_type, command)?;
            if addr.family == ChannelFamily::Signal
                && instrument.test_output_analog_signal(addr.index)
            {
                return Err(bad(record_type, "channel is not an analog output"));
            }
            Ok(Operation::AnalogSet(addr))
        }
        "bi" => match command {
            "VAL" => {
                let channel = parse_signal_only(arg, record_type, command)?;
                if instrument.test_binary_signal(channel) {
                    return Err(bad(record_type, "channel is not a binary signal"));
                }
                Ok(Operation::BinaryVal(channel))
            }
            "MODULE_PRESENCE" => Ok(Operation::ModulePresence(parse_module_index(
                arg,
                record_type,
                command,
            )?)),
            "SETTINGS_MODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::SettingsMode)
            }
            "MEASURE_MODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::MeasurementModeFlag)
            }
            "COMPUTE_MODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::ComputeModeFlag)
            }
            "ERROR_FLAG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::ErrorFlag)
            }
            "ALARM_FLAG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::AlarmFlag)
            }
            other => Err(bad(record_type, format!("unrecognized command '{other}'"))),
        },
        "bo" => match command {
            "VAL" => {
                let channel = parse_signal_only(arg, record_type, command)?;
                if instrument.test_output_binary_signal(channel) {
                    return Err(bad(record_type, "channel is not a binary output"));
                }
                Ok(Operation::BinarySet(channel))
            }
            "INP_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::InputTrig)
            }
            "OUT_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::OutputTrig)
            }
            "INFO_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::InfoTrig)
            }
            "STAT_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::StatTrig)
            }
            "OPMODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::OpModeSet)
            }
            "ERROR_CLEAR" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::ErrorClearSet)
            }
            "ALARM_ACK" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::AlarmAckSet)
            }
            other => Err(bad(record_type, format!("unrecognized command '{other}'"))),
        },
        "mbbi" => match command {
            "MODULE_SPEED" => Ok(Operation::ModuleSpeed(parse_gated_module_index(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "CH_STATUS" => Ok(Operation::ChStatus(parse_signal_or_math_address(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "CH_MODE" => {
                let addr = parse_signal_or_math_address(arg, record_type, command, instrument)?;
                if addr.family != ChannelFamily::Signal {
                    return Err(bad(record_type, "CH_MODE requires a Signal address"));
                }
                Ok(Operation::ChMode(addr.index))
            }
            "VAL_STATUS" => Ok(Operation::ValStatus(parse_signal_or_math_address(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "ALARM" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "ALARM requires an address argument"))?;
                let (addr, sub) = link::parse_alarm_address(arg)
                    .ok_or_else(|| bad(record_type, format!("bad alarm address '{arg}'")))?;
                let addr = validate_signal_or_math_address(addr, record_type, instrument)?;
                Ok(Operation::Alarm(addr, sub))
            }
            "ALARMS" => Ok(Operation::Alarms(parse_signal_or_math_address(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            other => Err(bad(record_type, format!("unrecognized command '{other}'"))),
        },
        "mbbo" => {
            if command != "COMPUTE_CMD" {
                return Err(bad(record_type, "only COMPUTE_CMD is supported"));
            }
            require_no_arg(arg, record_type, command)?;
            Ok(Operation::ComputeCmdSet)
        }
        "longin" => match command {
            "VAL" => {
                let channel = parse_signal_only(arg, record_type, command)?;
                if instrument.test_integer_signal(channel) {
                    return Err(bad(record_type, "channel is not an integer signal"));
                }
                Ok(Operation::IntegerVal(channel))
            }
            "MODULE_MODEL" => Ok(Operation::ModuleModel(parse_gated_module_index(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "MODULE_NUMBER" => Ok(Operation::ModuleNumber(parse_gated_module_index(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            other => Err(bad(record_type, format!("unrecognized command '{other}'"))),
        },
        "stringin" => match command {
            // devMW100_stringin.c's IP_ADDR branch never checks `arg ==
            // NULL` at all (unlike every other command here) — an arg is
            // silently accepted and ignored, not rejected.
            "IP_ADDR" => Ok(Operation::IpAddr),
            "MODULE_CODE" => Ok(Operation::ModuleCode(parse_gated_module_index(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "MODULE_STRING" => Ok(Operation::ModuleString(parse_module_index(
                arg,
                record_type,
                command,
            )?)),
            "ERROR" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "ERROR requires a channel argument"))?;
                let channel = codec::c_atoi(arg.as_bytes());
                if !(1..=3).contains(&channel) {
                    return Err(bad(record_type, "ERROR channel must be 1-3"));
                }
                Ok(Operation::ErrorText(channel as u32))
            }
            "UNIT" => Ok(Operation::Unit(parse_signal_or_math_address(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "EXPR" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "EXPR requires a math address argument"))?;
                let addr = link::parse_channel_address(arg)
                    .ok_or_else(|| bad(record_type, format!("bad math address '{arg}'")))?;
                if addr.family != ChannelFamily::Math {
                    return Err(bad(record_type, "EXPR requires an A<n> math address"));
                }
                Ok(Operation::Expr(addr.index))
            }
            other => Err(bad(record_type, format!("unrecognized command '{other}'"))),
        },
        other => Err(bad(
            other,
            "MW100 device support does not implement this record type",
        )),
    }
}

/// Records whose C `init_record` seeds an initial value directly
/// (`devMW100_ao.c:146-150`: VAL via `mw100_analog_get`;
/// `devMW100_bo.c:178-185`: RVAL via `mw100_binary_get`, then `return 0`,
/// so `boRecord.c:156-161` sets `val = rval ? 1 : 0`).
fn seed_initial_value(
    instrument: &Instrument,
    op: Operation,
    record: &mut dyn Record,
) -> CaResult<()> {
    match op {
        Operation::AnalogSet(addr) => {
            let v = instrument.analog_get(addr.family, addr.index);
            record.put_field("VAL", EpicsValue::Double(v))?;
        }
        Operation::BinarySet(channel) => {
            let v = instrument.binary_get(channel);
            record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
        }
        _ => {}
    }
    Ok(())
}

fn resolve_link(registry: &Registry, link_text: &str, record: &mut dyn Record) -> Option<Resolved> {
    let parsed = link::parse_link(link_text)?;
    let instrument = registry.get(parsed.device)?;
    let op = resolve_operation(
        record.record_type(),
        parsed.command,
        parsed.arg,
        &instrument,
    )
    .ok()?;
    seed_initial_value(&instrument, op, record).ok()?;
    Some(Resolved { instrument, op })
}

struct Resolved {
    instrument: Arc<Instrument>,
    op: Operation,
}

/// Shared `DeviceSupport` for every MW100 record type. The link is parsed
/// (and the device looked up / operation resolved) lazily in `init()`, since
/// only there is the concrete record type known.
pub struct MwDevice {
    registry: Arc<Registry>,
    link_text: String,
    is_io_intr: bool,
    resolved: Option<Resolved>,
    record_type: &'static str,
    /// Senders of the dummy IOSCANPVT an invalid input record scans on.
    dummy_scan: Vec<Sender<()>>,
}

impl MwDevice {
    fn new(registry: Arc<Registry>, link_text: String) -> Self {
        Self {
            registry,
            link_text,
            is_io_intr: false,
            resolved: None,
            record_type: "",
            dummy_scan: Vec::new(),
        }
    }
}

impl DeviceSupport for MwDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    // C init_record failures are a silent `return 1` that leave
    // `invalid_flag` set (devMW100_ai.c:94-160): read/write then return 0
    // and get_ioint_info hands out a dummy IOSCANPVT. `resolved == None` is
    // that flag; `Err` would print and alarm.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        self.record_type = record.record_type();
        self.resolved = resolve_link(&self.registry, &self.link_text, record);
        let Some(Resolved { op, .. }) = self.resolved else {
            // C's status 1 converts nothing (boRecord.c:159, mbboRecord.c:143).
            return Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched));
        };
        // devMW100_ao.c:146-152 returns 2 and defines the VAL it seeded.
        if record.record_type() == "ao" {
            let udf = match op {
                Operation::AnalogSet(_) => DeviceUdf::Defined,
                _ => DeviceUdf::Untouched,
            };
            return Ok(DeviceInitOutcome::DoNotConvert(udf));
        }
        Ok(DeviceInitOutcome::ok())
    }

    fn set_record_info(&mut self, _name: &str, scan: ScanType) {
        self.is_io_intr = matches!(scan, ScanType::IoIntr);
    }

    fn io_intr_receiver(&mut self) -> Option<Receiver<()>> {
        let Some(resolved) = self.resolved.as_ref() else {
            // The input dsets hand an invalid record `dummy_ioscanpvt`, which
            // nothing requests (devMW100_ai.c:171-172); ao/bo/mbbo have no
            // get_ioint_info at all.
            if !matches!(
                self.record_type,
                "ai" | "bi" | "longin" | "mbbi" | "stringin"
            ) {
                return None;
            }
            let (tx, rx) = mpsc::channel(1);
            self.dummy_scan.push(tx);
            return Some(rx);
        };
        let category = operation_interrupt_category(resolved.op, &resolved.instrument)?;
        Some(resolved.instrument.register_interrupt(category))
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Some(Resolved { instrument, op }) = self.resolved.as_ref() else {
            return Ok(DeviceReadOutcome::ok());
        };
        let is_io_intr = self.is_io_intr;
        match *op {
            Operation::AnalogVal(addr) => {
                if !is_io_intr {
                    instrument.channel_start(addr.family, addr.index);
                }
                let v = instrument.analog_get(addr.family, addr.index);
                record.put_field("VAL", EpicsValue::Double(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::IntegerVal(channel) => {
                if !is_io_intr {
                    instrument.channel_start(ChannelFamily::Signal, channel);
                }
                let v = instrument.integer_get(channel);
                record.put_field("VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::BinaryVal(channel) => {
                if !is_io_intr {
                    instrument.channel_start(ChannelFamily::Signal, channel);
                }
                let v = instrument.binary_get(channel);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ModulePresence(module) => {
                let v = instrument.module_presence(module);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::SettingsMode => {
                let v = instrument.get_settings_mode();
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::MeasurementModeFlag => {
                let v = instrument.get_measurement_mode();
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ComputeModeFlag => {
                let v = instrument.get_compute_mode();
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ErrorFlag => {
                let v = instrument.get_error_flag();
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::AlarmFlag => {
                let v = instrument.get_alarm_flag();
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ModuleSpeed(module) => {
                let v = instrument.module_speed(module);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ChStatus(addr) => {
                let v = instrument.get_channel_status(addr.family, addr.index);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ChMode(channel) => {
                let v = instrument.get_channel_mode(channel);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::ValStatus(addr) => {
                let v = instrument.get_data_status(addr.family, addr.index);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::Alarm(addr, sub) => {
                let v = instrument.get_alarm(addr.family, addr.index, sub);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::Alarms(addr) => {
                let v = instrument.get_alarm(addr.family, addr.index, 0);
                record.put_field("RVAL", EpicsValue::ULong(v as u32))?;
                Ok(DeviceReadOutcome::ok())
            }
            Operation::IpAddr => {
                record.put_field(
                    "VAL",
                    EpicsValue::String(instrument.peer_address.clone().into()),
                )?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::ModuleModel(module) => {
                let v = instrument.module_model(module);
                record.put_field("VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::ModuleNumber(module) => {
                let v = instrument.module_number(module);
                record.put_field("VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::ModuleCode(module) => {
                let v = instrument.module_code(module);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::ModuleString(module) => {
                let v = instrument.module_string(module);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::Unit(addr) => {
                let v = instrument.channel_get_egu(addr.family, addr.index);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::ErrorText(channel) => {
                let v = instrument.get_error(channel);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::Expr(channel) => {
                let v = instrument.channel_get_expr(channel);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::AnalogSet(_)
            | Operation::BinarySet(_)
            | Operation::InputTrig
            | Operation::OutputTrig
            | Operation::InfoTrig
            | Operation::StatTrig
            | Operation::OpModeSet
            | Operation::ErrorClearSet
            | Operation::AlarmAckSet
            | Operation::ComputeCmdSet => Err(CaError::LinkError(
                "MW100 device support: this operation is write-only".into(),
            )),
        }
    }

    // write_begin always answers, so the framework never calls this.
    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    // C write_* queues the command with the record and returns PACT; the
    // queue thread re-processes the record once it ran (drvMW100.c:1418-1424).
    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        let Some(Resolved { instrument, op }) = self.resolved.as_ref() else {
            return Ok(WriteStart::Completed);
        };
        let command = output_command(*op, record)?;
        // devMW100_bo.c:194-197: REC_TRIG and REC_ERROR drop VAL back to 0
        // on the completing pass. Completion re-enters no device code, so
        // it is cleared here, still before the cycle's monitor.
        if matches!(
            op,
            Operation::InputTrig
                | Operation::OutputTrig
                | Operation::InfoTrig
                | Operation::StatTrig
                | Operation::ErrorClearSet
        ) {
            record.put_field("VAL", EpicsValue::Enum(0))?;
        }
        instrument.submit_write(command, done.device_done());
        Ok(WriteStart::Pending)
    }
}

fn output_command(op: Operation, record: &dyn Record) -> CaResult<Command> {
    let command = match op {
        Operation::AnalogSet(addr) => {
            let val = match record.get_field("VAL") {
                Some(EpicsValue::Double(v)) => v,
                _ => return Err(CaError::FieldNotFound("VAL".into())),
            };
            match addr.family {
                ChannelFamily::Signal => Command::SetSignalOutput(addr.index, val),
                ChannelFamily::Comm => Command::SetComm(addr.index, val),
                ChannelFamily::Const => Command::SetConst(addr.index, val),
                ChannelFamily::Math => {
                    unreachable!("ao resolve_operation rejects Math via parse_address_no_math")
                }
            }
        }
        Operation::BinarySet(channel) => {
            let on = match record.get_field("VAL") {
                Some(EpicsValue::Enum(v)) => v != 0,
                _ => return Err(CaError::FieldNotFound("VAL".into())),
            };
            Command::SetBinaryOutput(channel, on)
        }
        Operation::InputTrig => Command::ReadAllInputs,
        Operation::OutputTrig => Command::ReadAllOutputs,
        Operation::InfoTrig => Command::ReadAllInfos,
        Operation::StatTrig => Command::ReadStatus,
        Operation::OpModeSet => {
            let v = match record.get_field("VAL") {
                Some(EpicsValue::Enum(v)) => i32::from(v),
                _ => return Err(CaError::FieldNotFound("VAL".into())),
            };
            Command::SetOpMode(v)
        }
        Operation::ErrorClearSet => Command::ClearError,
        Operation::AlarmAckSet => Command::AcknowledgeAlarms,
        Operation::ComputeCmdSet => {
            // An mbbo without states serves VAL as DBF_USHORT.
            let v = match record.get_field("VAL") {
                Some(EpicsValue::Enum(v) | EpicsValue::UShort(v)) => i32::from(v),
                _ => return Err(CaError::FieldNotFound("VAL".into())),
            };
            Command::SetCompute(v)
        }
        Operation::AnalogVal(_)
        | Operation::IntegerVal(_)
        | Operation::BinaryVal(_)
        | Operation::ModulePresence(_)
        | Operation::SettingsMode
        | Operation::MeasurementModeFlag
        | Operation::ComputeModeFlag
        | Operation::ErrorFlag
        | Operation::AlarmFlag
        | Operation::ModuleSpeed(_)
        | Operation::ChStatus(_)
        | Operation::ChMode(_)
        | Operation::ValStatus(_)
        | Operation::Alarm(_, _)
        | Operation::Alarms(_)
        | Operation::IpAddr
        | Operation::ModuleModel(_)
        | Operation::ModuleNumber(_)
        | Operation::ModuleCode(_)
        | Operation::ModuleString(_)
        | Operation::Unit(_)
        | Operation::ErrorText(_)
        | Operation::Expr(_) => {
            return Err(CaError::LinkError(
                "MW100 device support: this operation is read-only".into(),
            ));
        }
    };
    Ok(command)
}

/// Dynamic device-support factory for `DTYP == "Yokogawa MW100"`. Registered
/// once via `IocApplication::register_dynamic_device_support`; `registry`
/// must already hold every `mw100Configure`d instrument by the time records
/// wire up (i.e. the startup script configures ports before `iocInit`).
pub fn factory(
    registry: Arc<Registry>,
) -> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    move |ctx: &DeviceSupportContext| {
        if ctx.dtyp != DTYP {
            return None;
        }
        let raw = if !ctx.inp.is_empty() {
            ctx.inp
        } else {
            ctx.out
        };
        // `ctx.inp`/`ctx.out` carry the db field's raw text verbatim
        // (record_instance.rs stores it unstripped), so an INST_IO link still
        // has its leading `@` marker. Same strip-and-trim as the framework's
        // own builtin INST_IO devices (e.g. builtin_devices::stdio).
        let link_text = raw.strip_prefix('@').unwrap_or(raw).trim();
        Some(
            Box::new(MwDevice::new(registry.clone(), link_text.to_string()))
                as Box<dyn DeviceSupport>,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instrument::test_support::{
        ascii_frame, connect_default_fixture, connect_fixture_then, empty_fo1_binary,
        fe1_one_signal_channel, is0_all_zero, one_record_fd1_binary, spawn_fake_device,
    };
    use epics_rs::base::runtime::task::test_block_on;
    use epics_rs::base::server::database::{DeviceDone, PvDatabase};
    use epics_rs::base::server::device_support::wire_device_to_record;
    use epics_rs::base::server::recgbl::alarm_status;
    use epics_rs::base::server::record::{AlarmSeverity, FieldDesc};
    use epics_rs::base::server::records::bo::BoRecord;
    use epics_rs::base::server::records::mbbo::MbboRecord;
    use std::collections::HashMap;
    use std::net::TcpListener;
    use std::time::Duration;

    struct FakeRecord {
        record_type: &'static str,
        fields: HashMap<&'static str, EpicsValue>,
    }

    impl FakeRecord {
        fn new(record_type: &'static str) -> Self {
            Self {
                record_type,
                fields: HashMap::new(),
            }
        }
    }

    impl Record for FakeRecord {
        fn record_type(&self) -> &'static str {
            self.record_type
        }

        fn get_field(&self, name: &str) -> Option<EpicsValue> {
            self.fields.get(name).cloned()
        }

        fn put_field(&mut self, name: &str, value: EpicsValue) -> CaResult<()> {
            self.fields.insert(
                match name {
                    "VAL" => "VAL",
                    "RVAL" => "RVAL",
                    other => panic!("FakeRecord: unexpected field '{other}'"),
                },
                value,
            );
            Ok(())
        }

        fn declared_fields(&self) -> &'static [FieldDesc] {
            &[]
        }
    }

    fn registry_with(name: &str, instrument: Arc<Instrument>) -> Arc<Registry> {
        let registry = Arc::new(Registry::new());
        registry.insert_connected(name, instrument);
        registry
    }

    // -- resolve_operation: ai --

    #[test]
    fn ai_val_const_channel_resolves_ungated() {
        let instrument = connect_default_fixture();
        let op = resolve_operation("ai", "VAL", Some("K1"), &instrument).unwrap();
        assert!(matches!(
            op,
            Operation::AnalogVal(ChannelAddress {
                family: ChannelFamily::Const,
                index: 1
            })
        ));
    }

    #[test]
    fn ai_val_rejects_nonexistent_signal_channel() {
        // Fixture module 0 is a `MX110-UNV-M06`: 6 InputAnalog channels, so
        // addresses 1-6 all exist. 7 is the first genuinely unconfigured one.
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ai", "VAL", Some("7"), &instrument).is_err());
    }

    #[test]
    fn ai_rejects_unrecognized_command() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ai", "FOO", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: ao --

    #[test]
    fn ao_val_const_channel_resolves_ungated() {
        let instrument = connect_default_fixture();
        let op = resolve_operation("ao", "VAL", Some("K1"), &instrument).unwrap();
        assert!(matches!(
            op,
            Operation::AnalogSet(ChannelAddress {
                family: ChannelFamily::Const,
                index: 1
            })
        ));
    }

    #[test]
    fn ao_val_rejects_nonexistent_signal_channel() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ao", "VAL", Some("2"), &instrument).is_err());
    }

    #[test]
    fn ao_val_rejects_math_address_structurally() {
        // devMW100_ao.c's arg switch has no 'A' (Math) arm at all — unlike
        // GM10's `ao`, which only rejects Math later, in `write()`.
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ao", "VAL", Some("A1"), &instrument).is_err());
    }

    // -- resolve_operation: bi --

    #[test]
    fn bi_val_rejects_non_binary_signal_channel() {
        let instrument = connect_default_fixture();
        // Channel 1 exists but is InputAnalog, not binary.
        assert!(resolve_operation("bi", "VAL", Some("1"), &instrument).is_err());
        // Channel 7 does not exist at all.
        assert!(resolve_operation("bi", "VAL", Some("7"), &instrument).is_err());
    }

    #[test]
    fn bi_module_presence_parses_and_validates_digit() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("bi", "MODULE_PRESENCE", Some("0"), &instrument).unwrap(),
            Operation::ModulePresence(0)
        ));
        assert!(resolve_operation("bi", "MODULE_PRESENCE", Some("x"), &instrument).is_err());
    }

    #[test]
    fn bi_settings_mode_is_bare() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("bi", "SETTINGS_MODE", None, &instrument).unwrap(),
            Operation::SettingsMode
        ));
        assert!(resolve_operation("bi", "SETTINGS_MODE", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: bo --

    #[test]
    fn bo_val_rejects_nonexistent_or_wrong_type_channel() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("bo", "VAL", Some("1"), &instrument).is_err());
        assert!(resolve_operation("bo", "VAL", Some("7"), &instrument).is_err());
    }

    #[test]
    fn bo_inp_trig_is_bare() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("bo", "INP_TRIG", None, &instrument).unwrap(),
            Operation::InputTrig
        ));
        assert!(resolve_operation("bo", "INP_TRIG", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: mbbi --

    #[test]
    fn mbbi_ch_status_rejects_comm_const() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("mbbi", "CH_STATUS", Some("1"), &instrument).unwrap(),
            Operation::ChStatus(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 1
            })
        ));
        assert!(resolve_operation("mbbi", "CH_STATUS", Some("K1"), &instrument).is_err());
        assert!(resolve_operation("mbbi", "CH_STATUS", Some("C1"), &instrument).is_err());
    }

    #[test]
    fn mbbi_ch_mode_requires_signal_family() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("mbbi", "CH_MODE", Some("A1"), &instrument).is_err());
        assert!(matches!(
            resolve_operation("mbbi", "CH_MODE", Some("1"), &instrument).unwrap(),
            Operation::ChMode(1)
        ));
    }

    #[test]
    fn mbbi_alarm_requires_sub_index() {
        let instrument = connect_default_fixture();
        let op = resolve_operation("mbbi", "ALARM", Some("1.2"), &instrument).unwrap();
        assert!(matches!(
            op,
            Operation::Alarm(
                ChannelAddress {
                    family: ChannelFamily::Signal,
                    index: 1
                },
                2
            )
        ));
        assert!(resolve_operation("mbbi", "ALARM", Some("1"), &instrument).is_err());
    }

    #[test]
    fn mbbi_alarms_rejects_nonexistent_signal_channel() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("mbbi", "ALARMS", Some("7"), &instrument).is_err());
    }

    #[test]
    fn mbbi_module_speed_gates_existence() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("mbbi", "MODULE_SPEED", Some("0"), &instrument).unwrap(),
            Operation::ModuleSpeed(0)
        ));
        // Module 1 was never reported present by the CF0 fixture.
        assert!(resolve_operation("mbbi", "MODULE_SPEED", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: mbbo --

    #[test]
    fn mbbo_compute_cmd_is_bare_and_exclusive() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("mbbo", "COMPUTE_CMD", None, &instrument).unwrap(),
            Operation::ComputeCmdSet
        ));
        assert!(resolve_operation("mbbo", "COMPUTE_CMD", Some("1"), &instrument).is_err());
        assert!(resolve_operation("mbbo", "OTHER", None, &instrument).is_err());
    }

    // -- resolve_operation: longin --

    #[test]
    fn longin_val_rejects_non_integer_channel() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("longin", "VAL", Some("1"), &instrument).is_err());
        // No Math/Comm/Const arm at all in devMW100_longin.c's VAL switch.
        assert!(resolve_operation("longin", "VAL", Some("A1"), &instrument).is_err());
    }

    #[test]
    fn longin_module_model_gates_existence() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("longin", "MODULE_MODEL", Some("0"), &instrument).unwrap(),
            Operation::ModuleModel(0)
        ));
        assert!(resolve_operation("longin", "MODULE_MODEL", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: stringin --

    #[test]
    fn stringin_ip_addr_ignores_arg() {
        // devMW100_stringin.c's IP_ADDR branch never checks `arg == NULL`
        // at all, unlike GM10's IP_ADDR (and unlike every other MW100
        // command) — an arg is silently accepted and ignored.
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "IP_ADDR", None, &instrument).unwrap(),
            Operation::IpAddr
        ));
        assert!(matches!(
            resolve_operation("stringin", "IP_ADDR", Some("x"), &instrument).unwrap(),
            Operation::IpAddr
        ));
    }

    #[test]
    fn stringin_module_code_gates_existence_module_string_does_not() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "MODULE_CODE", Some("0"), &instrument).unwrap(),
            Operation::ModuleCode(0)
        ));
        assert!(resolve_operation("stringin", "MODULE_CODE", Some("1"), &instrument).is_err());
        assert!(matches!(
            resolve_operation("stringin", "MODULE_STRING", Some("1"), &instrument).unwrap(),
            Operation::ModuleString(1)
        ));
    }

    #[test]
    fn stringin_unit_rejects_comm_const_and_missing_channel() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "UNIT", Some("1"), &instrument).unwrap(),
            Operation::Unit(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 1
            })
        ));
        assert!(resolve_operation("stringin", "UNIT", Some("K1"), &instrument).is_err());
        assert!(resolve_operation("stringin", "UNIT", Some("7"), &instrument).is_err());
    }

    #[test]
    fn stringin_error_channel_bounds() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "ERROR", Some("2"), &instrument).unwrap(),
            Operation::ErrorText(2)
        ));
        assert!(resolve_operation("stringin", "ERROR", Some("0"), &instrument).is_err());
        assert!(resolve_operation("stringin", "ERROR", Some("4"), &instrument).is_err());
        // atoi: trailing text is ignored.
        assert!(matches!(
            resolve_operation("stringin", "ERROR", Some("3x"), &instrument).unwrap(),
            Operation::ErrorText(3)
        ));
    }

    #[test]
    fn module_index_is_first_byte_gated_atoi() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "MODULE_STRING", Some("2x"), &instrument).unwrap(),
            Operation::ModuleString(2)
        ));
        assert!(matches!(
            resolve_operation("stringin", "MODULE_STRING", Some("004"), &instrument).unwrap(),
            Operation::ModuleString(4)
        ));
        // First byte must be '0'..'5' (devMW100_stringin.c:122), even when
        // atoi would land in range.
        assert!(resolve_operation("stringin", "MODULE_STRING", Some("6"), &instrument).is_err());
        assert!(resolve_operation("stringin", "MODULE_STRING", Some("+1"), &instrument).is_err());
        assert!(resolve_operation("stringin", "MODULE_STRING", Some("06"), &instrument).is_err());
    }

    #[test]
    fn stringin_expr_requires_math_family() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "EXPR", Some("A1"), &instrument).unwrap(),
            Operation::Expr(1)
        ));
        assert!(resolve_operation("stringin", "EXPR", Some("1"), &instrument).is_err());
    }

    #[test]
    fn unknown_record_type_is_rejected() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("waveform", "VAL", Some("1"), &instrument).is_err());
    }

    // -- factory --

    #[test]
    fn factory_declines_non_matching_dtyp() {
        let registry = registry_with("mw100dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            record_type: "ai",
            dtyp: "Some Other Device",
            inp: "mw100dev VAL:1",
            out: "",
        };
        assert!(f(&ctx).is_none());
    }

    #[test]
    fn factory_claims_matching_dtyp() {
        let registry = registry_with("mw100dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            record_type: "ai",
            dtyp: DTYP,
            inp: "mw100dev VAL:1",
            out: "",
        };
        assert!(f(&ctx).is_some());
    }

    #[test]
    fn factory_strips_leading_at_from_inst_io_link_text() {
        // Regression: `DeviceSupportContext.inp`/`.out` carry the db field's
        // raw text verbatim, including the `@` INST_IO marker a real
        // `.template`-loaded link has (e.g. "@mw100dev VAL:1"). Before this
        // fix, `init()` looked the device up under the literal name
        // "@mw100dev" and failed with "unknown MW100 device '@mw100dev'"
        // even though `mw100Init` registers it as "mw100dev".
        let registry = registry_with("mw100dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            record_type: "ai",
            dtyp: DTYP,
            inp: "@mw100dev VAL:1",
            out: "",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("ai");
        device.init(&mut record).unwrap();
    }

    #[test]
    fn failed_init_is_silent_and_record_is_a_no_op() {
        let registry = registry_with("mw100dev", connect_default_fixture());
        let f = factory(registry);
        for (record_type, inp, out) in [
            ("ai", "nosuchdev VAL:1", ""),
            ("ai", "mw100dev VAL:7", ""),
            ("bo", "", "mw100dev INP_TRIG:1"),
            ("ai", "", ""),
        ] {
            let ctx = DeviceSupportContext {
                record_type,
                dtyp: DTYP,
                inp,
                out,
            };
            let mut device = f(&ctx).unwrap();
            let mut record = FakeRecord::new(record_type);
            assert!(matches!(
                device.init(&mut record),
                Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched))
            ));
            assert_eq!(
                device.io_intr_receiver().is_some(),
                record_type == "ai",
                "{record_type}: dummy IOSCANPVT only for an input dset"
            );
            let outcome = device.read(&mut record).unwrap();
            assert!(!outcome.did_compute());
            device.write(&mut record).unwrap();
            assert!(record.fields.is_empty());
        }
    }

    // -- end-to-end: read (cache-only, I/O Intr scan) --

    #[test]
    fn ai_read_end_to_end_returns_computed_val() {
        let registry = registry_with("mw100dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            record_type: "ai",
            dtyp: DTYP,
            inp: "mw100dev VAL:1",
            out: "",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("ai");
        device.init(&mut record).unwrap();
        device.set_record_info("TEST:AI", ScanType::IoIntr);

        let outcome = device.read(&mut record).unwrap();
        assert!(outcome.did_compute());
        assert_eq!(record.fields.get("VAL"), Some(&EpicsValue::Double(1.234)));
    }

    // -- end-to-end: write (queued, completed by the actor) --

    /// A record named `TEST` carrying `link` as its OUT, wired to a device
    /// whose fixture then answers `extra`.
    async fn wired_output(
        record: Box<dyn Record>,
        link: &str,
        extra: Vec<Vec<u8>>,
    ) -> (PvDatabase, std::thread::JoinHandle<Vec<String>>) {
        let (instrument, device_thread) = connect_fixture_then(extra);
        (wire_record(record, link, instrument).await, device_thread)
    }

    async fn wire_record(
        record: Box<dyn Record>,
        link: &str,
        instrument: Arc<Instrument>,
    ) -> PvDatabase {
        let f = factory(registry_with("mw100dev", instrument));
        let ctx = DeviceSupportContext {
            record_type: record.record_type(),
            dtyp: DTYP,
            inp: "",
            out: link,
        };
        let device = f(&ctx).unwrap();
        let db = PvDatabase::new();
        db.add_record("TEST", record).await.unwrap();
        let rec = db.get_record("TEST").unwrap();
        let mut inst = rec.write();
        inst.common.dtyp = DTYP.into();
        // The framework's init, so the record's own status-0 arm runs.
        wire_device_to_record(&mut inst, device);
        drop(inst);
        db
    }

    #[test]
    fn relay_bo_seeds_val_from_the_relay_state() {
        test_block_on(async {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let addr = listener.local_addr().unwrap();
            let cf0 = b"0xxxMX125-MKC-M10xxxMX125-MKC-M10x\r\nE";
            // FO1: channel 1 relay on.
            let mut fo1 = vec![b'E', b'B', 0, 0];
            fo1.extend_from_slice(&30u32.to_le_bytes());
            fo1.extend_from_slice(&[0u8; 20]);
            fo1.extend_from_slice(&1u16.to_le_bytes());
            fo1.extend_from_slice(&[0u8; 2]);
            fo1.extend_from_slice(&1u32.to_le_bytes());
            fo1.extend_from_slice(&[0u8; 2]);
            let responses = vec![
                b"E0\r\n".to_vec(),
                ascii_frame(cf0),
                is0_all_zero(),
                fe1_one_signal_channel(),
                ascii_frame(b"E"),
                ascii_frame(b"E"),
                ascii_frame(b"E"),
                ascii_frame(b"E"),
                empty_fo1_binary(), // FD1: no input records
                fo1,
                ascii_frame(b"E"),
                ascii_frame(b"E"),
            ];
            let device_thread = spawn_fake_device(listener, responses);
            let instrument = Instrument::connect_to(addr).unwrap();
            device_thread.join().unwrap();
            let db = wire_record(Box::new(BoRecord::new(0)), "mw100dev VAL:1", instrument).await;
            let rec = db.get_record("TEST").unwrap();
            let inst = rec.read();
            assert_eq!(inst.record.get_field("VAL"), Some(EpicsValue::Enum(1)));
            assert_eq!(inst.record.get_field("RVAL"), Some(EpicsValue::ULong(1)));
        });
    }

    /// Put `val` and wait for the queued write to end the cycle.
    async fn put_and_complete(db: &PvDatabase, val: EpicsValue) -> (u16, AlarmSeverity) {
        db.put_record_field_from_ca_no_notify("TEST", "VAL", val)
            .await
            .unwrap();
        for _ in 0..500 {
            {
                let rec = db.get_record("TEST").unwrap();
                let inst = rec.read();
                if !inst.is_processing() {
                    return (inst.common.stat, inst.common.sevr);
                }
            }
            epics_rs::base::runtime::task::sleep(Duration::from_millis(5)).await;
        }
        panic!("the queued write never completed");
    }

    #[test]
    fn bo_inp_trig_write_is_queued_and_completes() {
        test_block_on(async {
            let (db, device_thread) = wired_output(
                Box::new(BoRecord::new(0)),
                "mw100dev INP_TRIG",
                vec![one_record_fd1_binary()],
            )
            .await;
            let (_, sevr) = put_and_complete(&db, EpicsValue::Enum(1)).await;
            assert_eq!(sevr, AlarmSeverity::NoAlarm);
            assert_eq!(
                device_thread.join().unwrap().last().unwrap(),
                "FD1,001,A300\r\n"
            );
        });
    }

    #[test]
    fn write_begin_queues_the_write() {
        test_block_on(async {
            let (instrument, device_thread) = connect_fixture_then(vec![one_record_fd1_binary()]);
            let f = factory(registry_with("mw100dev", instrument));
            let ctx = DeviceSupportContext {
                record_type: "bo",
                dtyp: DTYP,
                inp: "",
                out: "mw100dev INP_TRIG",
            };
            let mut device = f(&ctx).unwrap();
            let mut record = FakeRecord::new("bo");
            device.init(&mut record).unwrap();
            let db = PvDatabase::new();
            let cell = DeviceDone::new(
                &db,
                &epics_rs::base::server::record::RecordInstance::new(
                    "TEST",
                    epics_rs::base::server::records::bo::BoRecord::default(),
                ),
            );
            let start = device
                .write_begin(&mut record, &DeviceDoneSource::new(&cell))
                .unwrap();
            assert!(matches!(start, WriteStart::Pending));
            assert_eq!(
                device_thread.join().unwrap().last().unwrap(),
                "FD1,001,A300\r\n"
            );
        });
    }

    #[test]
    fn trigger_and_error_clear_reset_val_alarm_ack_does_not() {
        test_block_on(async {
            for (link, reply, expect) in [
                ("mw100dev INP_TRIG", one_record_fd1_binary(), 0),
                ("mw100dev OUT_TRIG", empty_fo1_binary(), 0),
                ("mw100dev ERROR_CLEAR", b"E0\r\n".to_vec(), 0),
                ("mw100dev OPMODE", b"E0\r\n".to_vec(), 1),
            ] {
                let (db, device_thread) =
                    wired_output(Box::new(BoRecord::new(0)), link, vec![reply]).await;
                put_and_complete(&db, EpicsValue::Enum(1)).await;
                device_thread.join().unwrap();
                let rec = db.get_record("TEST").unwrap();
                let val = rec.read().record.get_field("VAL");
                assert_eq!(val, Some(EpicsValue::Enum(expect)), "{link}");
            }
        });
    }

    #[test]
    fn compute_cmd_out_of_range_sends_nothing_and_completes() {
        test_block_on(async {
            // mw100_system.template's ComputationSet defines its states.
            let mut record = MbboRecord::new(0);
            record
                .put_field("ZRST", EpicsValue::String("Start".into()))
                .unwrap();
            let (db, device_thread) = wired_output(
                Box::new(record),
                "mw100dev COMPUTE_CMD",
                vec![b"E0\r\n".to_vec()],
            )
            .await;
            let (_, sevr) = put_and_complete(&db, EpicsValue::Enum(4)).await;
            assert_eq!(sevr, AlarmSeverity::NoAlarm);
            put_and_complete(&db, EpicsValue::Enum(2)).await;
            let received = device_thread.join().unwrap();
            assert_eq!(received.last().unwrap(), "EX2\r\n");
            assert!(!received.iter().any(|l| l.starts_with("EX4")));
        });
    }

    #[test]
    fn compute_cmd_without_states_writes() {
        test_block_on(async {
            let (db, device_thread) = wired_output(
                Box::new(MbboRecord::new(0)),
                "mw100dev COMPUTE_CMD",
                vec![b"E0\r\n".to_vec()],
            )
            .await;
            let (_, sevr) = put_and_complete(&db, EpicsValue::UShort(1)).await;
            assert_eq!(sevr, AlarmSeverity::NoAlarm);
            assert_eq!(device_thread.join().unwrap().last().unwrap(), "EX1\r\n");
        });
    }

    #[test]
    fn device_error_reply_does_not_alarm_the_write() {
        test_block_on(async {
            let (db, device_thread) = wired_output(
                Box::new(BoRecord::new(0)),
                "mw100dev ERROR_CLEAR",
                vec![b"E1 205 x\r\n".to_vec()],
            )
            .await;
            let (stat, sevr) = put_and_complete(&db, EpicsValue::Enum(1)).await;
            assert_eq!(
                (stat, sevr),
                (alarm_status::NO_ALARM, AlarmSeverity::NoAlarm)
            );
            assert_eq!(device_thread.join().unwrap().last().unwrap(), "CE0\r\n");
        });
    }

    /// C ao init_record returns 2 with the seeded VAL defined; bo returns 0
    /// so the record converts the RVAL the dset left.
    #[test]
    fn init_status_is_c_ao_two_and_bo_zero() {
        use epics_rs::base::server::device_support::DeviceUdf;
        let f = factory(registry_with("mw100dev", connect_default_fixture()));
        let init = |record_type: &'static str, out: &'static str| {
            let ctx = DeviceSupportContext {
                record_type,
                dtyp: DTYP,
                inp: "",
                out,
            };
            let mut device = f(&ctx).unwrap();
            device.init(&mut FakeRecord::new(record_type)).unwrap()
        };
        assert_eq!(
            init("ao", "mw100dev VAL:K1"),
            DeviceInitOutcome::DoNotConvert(DeviceUdf::Defined)
        );
        assert_eq!(init("bo", "mw100dev INP_TRIG"), DeviceInitOutcome::ok());
    }
}
