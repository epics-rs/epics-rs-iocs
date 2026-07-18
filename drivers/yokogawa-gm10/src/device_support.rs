//! Dynamic `DeviceSupport` for all 8 GM10 record types
//! (`devGM10_{ai,ao,bi,bo,mbbi,mbbo,longin,stringin}.c`).
//!
//! `gm10Support.dbd` binds every record type under the SAME DTYP string:
//! `device(<type>,INST_IO,devGM10_<type>,"Yokogawa GM10")` — the C dset
//! (hence the field layout/read-write shape) is selected by which record
//! type is loading it, not by DTYP. `DeviceSupportContext` (the dynamic
//! factory's only input) carries `dtyp`/`inp`/`out` but no record-type
//! field, so [`GmDevice`] defers command resolution to
//! [`DeviceSupport::init`], which does receive the concrete `&mut dyn
//! Record` and can call `record.record_type()`. [`resolve_operation`] is
//! the single place that mirrors every `devGM10_*.c::init_record`'s
//! command/address validation.

use crate::instrument::{Command, Instrument, InterruptCategory, Registry};
use crate::link::{self, ChannelAddress, ChannelFamily};
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::device_support::{DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::record::{Record, ScanType};
use epics_rs::base::types::EpicsValue;
use epics_rs::ca::server::ioc_app::DeviceSupportContext;
use std::sync::Arc;

/// Upstream DTYP shared by every GM10 record type (`gm10Support.dbd`).
pub const DTYP: &str = "Yokogawa GM10";

/// One variant per legal `(record type, link command)` pair validated by
/// a `devGM10_*.c::init_record`.
#[derive(Debug, Clone, Copy)]
enum Operation {
    // -- ai (devGM10_ai.c): VAL, any of the 5 address families. --
    AnalogVal(ChannelAddress),
    // -- ao (devGM10_ao.c): VAL, any of the 5 address families. --
    AnalogSet(ChannelAddress),
    // -- bi (devGM10_bi.c) --
    BinaryVal(u32),
    ModulePresence(usize),
    SettingsMode,
    RecordingModeFlag,
    ComputeModeFlag,
    ErrorFlag,
    AlarmFlag,
    // -- bo (devGM10_bo.c) --
    BinarySet(u32),
    ChanTrig,
    MiscTrig,
    InfoTrig,
    StatTrig,
    RecordingModeSet,
    ErrorClearSet,
    AlarmAckSet,
    // -- mbbi (devGM10_mbbi.c) --
    ChStatus(ChannelAddress),
    ChMode(u32),
    ValStatus(ChannelAddress),
    Alarm(ChannelAddress, u8),
    Alarms(ChannelAddress),
    // -- mbbo (devGM10_mbbo.c) --
    ComputeCmdSet,
    // -- longin (devGM10_longin.c): VAL, Signal only. --
    IntegerVal(u32),
    // -- stringin (devGM10_stringin.c) --
    IpAddr,
    ModuleString(usize),
    Unit(ChannelAddress),
    ErrorText(u32),
    Expr(u32),
}

fn family_category(family: ChannelFamily) -> InterruptCategory {
    match family {
        ChannelFamily::Signal | ChannelFamily::Math | ChannelFamily::Comm => {
            InterruptCategory::Channel
        }
        ChannelFamily::Const | ChannelFamily::VarConst => InterruptCategory::Misc,
    }
}

/// `None` for the write-only (bo/mbbo) operations, matching their C dset
/// registering `get_ioint_info: NULL`.
fn operation_interrupt_category(op: Operation) -> Option<InterruptCategory> {
    Some(match op {
        Operation::AnalogVal(addr) => family_category(addr.family),
        Operation::IntegerVal(_) | Operation::BinaryVal(_) => InterruptCategory::Channel,
        Operation::ModulePresence(_) => InterruptCategory::Info,
        Operation::SettingsMode | Operation::RecordingModeFlag | Operation::ComputeModeFlag => {
            InterruptCategory::Status
        }
        Operation::ErrorFlag => InterruptCategory::Error,
        Operation::AlarmFlag => InterruptCategory::Channel,
        Operation::ChStatus(_) | Operation::ChMode(_) => InterruptCategory::Info,
        Operation::ValStatus(addr) => family_category(addr.family),
        Operation::Alarm(addr, _) | Operation::Alarms(addr) => family_category(addr.family),
        Operation::IpAddr
        | Operation::ModuleString(_)
        | Operation::Unit(_)
        | Operation::Expr(_) => InterruptCategory::Info,
        Operation::ErrorText(_) => InterruptCategory::Error,
        Operation::AnalogSet(_)
        | Operation::BinarySet(_)
        | Operation::ChanTrig
        | Operation::MiscTrig
        | Operation::InfoTrig
        | Operation::StatTrig
        | Operation::RecordingModeSet
        | Operation::ErrorClearSet
        | Operation::AlarmAckSet
        | Operation::ComputeCmdSet => return None,
    })
}

fn bad(record_type: &str, msg: impl std::fmt::Display) -> CaError {
    CaError::LinkError(format!("GM10 {record_type} link: {msg}"))
}

fn require_no_arg(arg: Option<&str>, record_type: &str, command: &str) -> CaResult<()> {
    if arg.is_some() {
        return Err(bad(record_type, format!("{command} takes no argument")));
    }
    Ok(())
}

/// `MODULE_PRESENCE` (bi) / `MODULE_STRING` (stringin): a bare 0-9 module
/// index, with the same defensive first-byte digit check the C source
/// applies before `atoi` (`atoi` returning 0 on a non-numeric argument is
/// ambiguous with the genuinely valid address 0).
fn parse_module_index(arg: Option<&str>, record_type: &str, command: &str) -> CaResult<usize> {
    let arg = arg.ok_or_else(|| bad(record_type, format!("{command} requires an argument")))?;
    if !arg.as_bytes().first().is_some_and(u8::is_ascii_digit) {
        return Err(bad(
            record_type,
            format!("{command} argument must be numeric"),
        ));
    }
    let i: i32 = arg
        .parse()
        .map_err(|_| bad(record_type, format!("{command} argument must be numeric")))?;
    if !(0..=9).contains(&i) {
        return Err(bad(
            record_type,
            format!("{command} module index out of range 0-9"),
        ));
    }
    Ok(i as usize)
}

/// `VAL` on bi/bo/longin: a bare Signal (digit) address only — no
/// `A`/`C`/`K`/`W` prefix accepted.
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

/// Shared mbbi address validation (`devGM10_mbbi.c:166-171`): Const/
/// VarConst are never accepted (the address switch has no `K`/`W` arm),
/// and a Signal address must exist on the device.
fn validate_mbbi_address(
    addr: ChannelAddress,
    record_type: &str,
    instrument: &Instrument,
) -> CaResult<ChannelAddress> {
    if matches!(addr.family, ChannelFamily::Const | ChannelFamily::VarConst) {
        return Err(bad(
            record_type,
            "command does not support Const/VarConst addresses",
        ));
    }
    if addr.family == ChannelFamily::Signal && instrument.test_signal(addr.index) {
        return Err(bad(record_type, "channel does not exist"));
    }
    Ok(addr)
}

fn parse_mbbi_address(
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
    validate_mbbi_address(addr, record_type, instrument)
}

/// The single owner of every `devGM10_*.c::init_record` command/address
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
            let arg = arg.ok_or_else(|| bad(record_type, "VAL requires an address argument"))?;
            let addr = link::parse_channel_address(arg)
                .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
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
            "RECORDING_MODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::RecordingModeFlag)
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
            "CHAN_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::ChanTrig)
            }
            "MISC_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::MiscTrig)
            }
            "INFO_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::InfoTrig)
            }
            "STAT_TRIG" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::StatTrig)
            }
            "RECORDING_MODE" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::RecordingModeSet)
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
            "CH_STATUS" => Ok(Operation::ChStatus(parse_mbbi_address(
                arg,
                record_type,
                command,
                instrument,
            )?)),
            "CH_MODE" => {
                let addr = parse_mbbi_address(arg, record_type, command, instrument)?;
                if addr.family != ChannelFamily::Signal {
                    return Err(bad(record_type, "CH_MODE requires a Signal address"));
                }
                Ok(Operation::ChMode(addr.index))
            }
            "VAL_STATUS" => Ok(Operation::ValStatus(parse_mbbi_address(
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
                let addr = validate_mbbi_address(addr, record_type, instrument)?;
                Ok(Operation::Alarm(addr, sub))
            }
            "ALARMS" => Ok(Operation::Alarms(parse_mbbi_address(
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
        "longin" => {
            if command != "VAL" {
                return Err(bad(record_type, "only VAL is supported"));
            }
            let channel = parse_signal_only(arg, record_type, command)?;
            if instrument.test_integer_signal(channel) {
                return Err(bad(record_type, "channel is not an integer signal"));
            }
            Ok(Operation::IntegerVal(channel))
        }
        "stringin" => match command {
            "IP_ADDR" => {
                require_no_arg(arg, record_type, command)?;
                Ok(Operation::IpAddr)
            }
            "MODULE_STRING" => Ok(Operation::ModuleString(parse_module_index(
                arg,
                record_type,
                command,
            )?)),
            "ERROR" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "ERROR requires a channel argument"))?;
                let channel: u32 = arg
                    .parse()
                    .map_err(|_| bad(record_type, "ERROR channel must be numeric"))?;
                if channel == 0 || channel > 3 {
                    return Err(bad(record_type, "ERROR channel must be 1-3"));
                }
                Ok(Operation::ErrorText(channel))
            }
            "UNIT" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "UNIT requires an address argument"))?;
                let addr = link::parse_channel_address(arg)
                    .ok_or_else(|| bad(record_type, format!("bad channel address '{arg}'")))?;
                // devGM10_stringin.c's UNIT arg switch has no K/W arm.
                if matches!(addr.family, ChannelFamily::Const | ChannelFamily::VarConst) {
                    return Err(bad(
                        record_type,
                        "UNIT does not support Const/VarConst addresses",
                    ));
                }
                if addr.family == ChannelFamily::Signal && instrument.test_signal(addr.index) {
                    return Err(bad(record_type, "channel does not exist"));
                }
                Ok(Operation::Unit(addr))
            }
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
            "GM10 device support does not implement this record type",
        )),
    }
}

/// Records whose C `init_record` seeds an initial value directly
/// (`devGM10_ao.c`: VAL via `gm10_analog_get`; `devGM10_bo.c`: RVAL via
/// `gm10_binary_get`, asymmetrically — VAL is left at the record's
/// default since C's `init_record` never calls `process()`).
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

struct Resolved {
    instrument: Arc<Instrument>,
    op: Operation,
}

/// Shared `DeviceSupport` for every GM10 record type. The link is parsed
/// (and the device looked up / operation resolved) lazily in `init()`,
/// since only there is the concrete record type known.
pub struct GmDevice {
    registry: Arc<Registry>,
    link_text: String,
    is_io_intr: bool,
    resolved: Option<Resolved>,
}

impl GmDevice {
    fn new(registry: Arc<Registry>, link_text: String) -> Self {
        Self {
            registry,
            link_text,
            is_io_intr: false,
            resolved: None,
        }
    }

    fn resolved(&self) -> CaResult<&Resolved> {
        self.resolved
            .as_ref()
            .ok_or_else(|| CaError::LinkError("GM10 device support not initialized".into()))
    }
}

impl DeviceSupport for GmDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let parsed = link::parse_link(&self.link_text).ok_or_else(|| {
            CaError::LinkError(format!("malformed GM10 link: '{}'", self.link_text))
        })?;
        let instrument = self.registry.get(parsed.device).ok_or_else(|| {
            CaError::LinkError(format!("unknown GM10 device '{}'", parsed.device))
        })?;
        let op = resolve_operation(
            record.record_type(),
            parsed.command,
            parsed.arg,
            &instrument,
        )?;
        seed_initial_value(&instrument, op, record)?;
        self.resolved = Some(Resolved { instrument, op });
        Ok(())
    }

    fn set_record_info(&mut self, _name: &str, scan: ScanType) {
        self.is_io_intr = matches!(scan, ScanType::IoIntr);
    }

    fn io_intr_receiver(&mut self) -> Option<epics_rs::base::runtime::sync::mpsc::Receiver<()>> {
        let resolved = self.resolved.as_ref()?;
        let category = operation_interrupt_category(resolved.op)?;
        Some(resolved.instrument.register_interrupt(category))
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Resolved { instrument, op } = self.resolved()?;
        let is_io_intr = self.is_io_intr;
        match *op {
            Operation::AnalogVal(addr) => {
                if !is_io_intr {
                    instrument.channel_start(addr.family, addr.index)?;
                }
                let v = instrument.analog_get(addr.family, addr.index);
                record.put_field("VAL", EpicsValue::Double(v))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::IntegerVal(channel) => {
                if !is_io_intr {
                    instrument.channel_start(ChannelFamily::Signal, channel)?;
                }
                let v = instrument.integer_get(channel);
                record.put_field("VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::BinaryVal(channel) => {
                if !is_io_intr {
                    instrument.channel_start(ChannelFamily::Signal, channel)?;
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
            Operation::RecordingModeFlag => {
                let v = instrument.get_recording_mode();
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
                Ok(DeviceReadOutcome::computed())
            }
            Operation::ModuleString(module) => {
                let v = instrument.module_string(module);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::Unit(addr) => {
                let v = instrument.channel_get_egu(addr.family, addr.index);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::ErrorText(channel) => {
                let v = instrument.get_error(channel);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::Expr(channel) => {
                let v = instrument.channel_get_expr(channel);
                record.put_field("VAL", EpicsValue::String(v.into()))?;
                Ok(DeviceReadOutcome::computed())
            }
            Operation::AnalogSet(_)
            | Operation::BinarySet(_)
            | Operation::ChanTrig
            | Operation::MiscTrig
            | Operation::InfoTrig
            | Operation::StatTrig
            | Operation::RecordingModeSet
            | Operation::ErrorClearSet
            | Operation::AlarmAckSet
            | Operation::ComputeCmdSet => Err(CaError::LinkError(
                "GM10 device support: this operation is write-only".into(),
            )),
        }
    }

    fn write(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let Resolved { instrument, op } = self.resolved()?;
        match *op {
            Operation::AnalogSet(addr) => {
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::Double(v)) => v,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                let command = match addr.family {
                    ChannelFamily::Signal => Command::SetSignalOutput(addr.index, val),
                    ChannelFamily::Comm => Command::SetComm(addr.index, val),
                    ChannelFamily::Const => Command::SetConst(addr.index, val),
                    ChannelFamily::VarConst => Command::SetVarConst(addr.index, val),
                    ChannelFamily::Math => {
                        return Err(CaError::LinkError(
                            "ao does not support Math addresses".into(),
                        ));
                    }
                };
                instrument.submit(command)?;
            }
            Operation::BinarySet(channel) => {
                let on = match record.get_field("VAL") {
                    Some(EpicsValue::Enum(v)) => v != 0,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                instrument.submit(Command::SetBinaryOutput(channel, on))?;
            }
            Operation::ChanTrig => instrument.submit(Command::ReadAllData)?,
            Operation::MiscTrig => instrument.submit(Command::ReadAllMisc)?,
            Operation::InfoTrig => instrument.submit(Command::ReadAllInfos)?,
            Operation::StatTrig => instrument.submit(Command::ReadStatus)?,
            Operation::RecordingModeSet => {
                let on = match record.get_field("VAL") {
                    Some(EpicsValue::Enum(v)) => v != 0,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                instrument.submit(Command::SetOpMode(on))?;
            }
            Operation::ErrorClearSet => instrument.submit(Command::ClearError)?,
            Operation::AlarmAckSet => instrument.submit(Command::AcknowledgeAlarms)?,
            Operation::ComputeCmdSet => {
                let v = match record.get_field("VAL") {
                    Some(EpicsValue::Enum(v)) => v as u8,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                instrument.submit(Command::SetCompute(v))?;
            }
            Operation::AnalogVal(_)
            | Operation::IntegerVal(_)
            | Operation::BinaryVal(_)
            | Operation::ModulePresence(_)
            | Operation::SettingsMode
            | Operation::RecordingModeFlag
            | Operation::ComputeModeFlag
            | Operation::ErrorFlag
            | Operation::AlarmFlag
            | Operation::ChStatus(_)
            | Operation::ChMode(_)
            | Operation::ValStatus(_)
            | Operation::Alarm(_, _)
            | Operation::Alarms(_)
            | Operation::IpAddr
            | Operation::ModuleString(_)
            | Operation::Unit(_)
            | Operation::ErrorText(_)
            | Operation::Expr(_) => {
                return Err(CaError::LinkError(
                    "GM10 device support: this operation is read-only".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Dynamic device-support factory for `DTYP == "Yokogawa GM10"`. Registered
/// once via `IocApplication::register_dynamic_device_support`; `registry`
/// must already hold every `gm10Configure`d instrument by the time records
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
            Box::new(GmDevice::new(registry.clone(), link_text.to_string()))
                as Box<dyn DeviceSupport>,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instrument::test_support::{
        ascii_frame, connect_default_fixture, one_record_fdata_binary, spawn_fake_device,
    };
    use epics_rs::base::server::record::FieldDesc;
    use std::collections::HashMap;
    use std::net::TcpListener;

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

        fn with(mut self, name: &'static str, value: EpicsValue) -> Self {
            self.fields.insert(name, value);
            self
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
        registry.insert(name.to_string(), instrument).unwrap();
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
        // Fixture module 0 is a `GX90XA-06`: 6 InputAnalog channels, so
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
        let op = resolve_operation("ao", "VAL", Some("W1"), &instrument).unwrap();
        assert!(matches!(
            op,
            Operation::AnalogSet(ChannelAddress {
                family: ChannelFamily::VarConst,
                index: 1
            })
        ));
    }

    #[test]
    fn ao_val_rejects_nonexistent_signal_channel() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ao", "VAL", Some("2"), &instrument).is_err());
    }

    // -- resolve_operation: bi --

    #[test]
    fn bi_val_rejects_non_binary_signal_channel() {
        let instrument = connect_default_fixture();
        // Channel 1 exists but is InputAnalog, not binary.
        assert!(resolve_operation("bi", "VAL", Some("1"), &instrument).is_err());
        // Channel 2 does not exist at all.
        assert!(resolve_operation("bi", "VAL", Some("2"), &instrument).is_err());
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
        assert!(resolve_operation("bo", "VAL", Some("2"), &instrument).is_err());
    }

    #[test]
    fn bo_chan_trig_is_bare() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("bo", "CHAN_TRIG", None, &instrument).unwrap(),
            Operation::ChanTrig
        ));
        assert!(resolve_operation("bo", "CHAN_TRIG", Some("1"), &instrument).is_err());
    }

    // -- resolve_operation: mbbi --

    #[test]
    fn mbbi_ch_status_rejects_const_varconst() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("mbbi", "CH_STATUS", Some("1"), &instrument).unwrap(),
            Operation::ChStatus(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 1
            })
        ));
        assert!(resolve_operation("mbbi", "CH_STATUS", Some("K1"), &instrument).is_err());
    }

    #[test]
    fn mbbi_ch_mode_requires_signal_family() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("mbbi", "CH_MODE", Some("A1"), &instrument).is_err());
        // Channel 1 exists (Signal), so CH_MODE resolves even though it
        // is not an output channel — `get_channel_mode` itself returns 0
        // for non-output types (devGM10_mbbi.c has no output-type gate).
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
        // Fixture module 0 is a `GX90XA-06`: 6 InputAnalog channels, so
        // addresses 1-6 all exist. 7 is the first genuinely unconfigured one.
        let instrument = connect_default_fixture();
        assert!(resolve_operation("mbbi", "ALARMS", Some("7"), &instrument).is_err());
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
        assert!(resolve_operation("longin", "VAL", Some("A1"), &instrument).is_err());
    }

    // -- resolve_operation: stringin --

    #[test]
    fn stringin_ip_addr_is_bare() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "IP_ADDR", None, &instrument).unwrap(),
            Operation::IpAddr
        ));
        assert!(resolve_operation("stringin", "IP_ADDR", Some("x"), &instrument).is_err());
    }

    #[test]
    fn stringin_unit_rejects_const_varconst_and_missing_channel() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "UNIT", Some("1"), &instrument).unwrap(),
            Operation::Unit(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 1
            })
        ));
        assert!(resolve_operation("stringin", "UNIT", Some("K1"), &instrument).is_err());
        // Fixture module 0 is a `GX90XA-06`: 6 InputAnalog channels, so
        // addresses 1-6 all exist. 7 is the first genuinely unconfigured one.
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
        let registry = registry_with("gm10dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            dtyp: "Some Other Device",
            inp: "gm10dev VAL:1",
            out: "",
        };
        assert!(f(&ctx).is_none());
    }

    #[test]
    fn factory_claims_matching_dtyp() {
        let registry = registry_with("gm10dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            dtyp: DTYP,
            inp: "gm10dev VAL:1",
            out: "",
        };
        assert!(f(&ctx).is_some());
    }

    #[test]
    fn factory_strips_leading_at_from_inst_io_link_text() {
        // Regression: `DeviceSupportContext.inp`/`.out` carry the db field's
        // raw text verbatim, including the `@` INST_IO marker a real
        // `.template`-loaded link has (e.g. "@gm10dev VAL:1"). Before this
        // fix, `init()` looked the device up under the literal name
        // "@gm10dev" and failed with "unknown GM10 device '@gm10dev'" even
        // though `gm10Init` registers it as "gm10dev".
        let registry = registry_with("gm10dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            dtyp: DTYP,
            inp: "@gm10dev VAL:1",
            out: "",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("ai");
        device.init(&mut record).unwrap();
    }

    // -- end-to-end: read (cache-only, I/O Intr scan) --

    #[test]
    fn ai_read_end_to_end_returns_computed_val() {
        let registry = registry_with("gm10dev", connect_default_fixture());
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            dtyp: DTYP,
            inp: "gm10dev VAL:1",
            out: "",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("ai");
        device.init(&mut record).unwrap();
        device.set_record_info("TEST:AI", ScanType::IoIntr);

        let outcome = device.read(&mut record).unwrap();
        assert!(outcome.did_compute);
        assert_eq!(record.fields.get("VAL"), Some(&EpicsValue::Double(1.234)));
    }

    // -- end-to-end: write (one live wire round trip) --

    #[test]
    fn bo_chan_trig_write_end_to_end_submits_read_all_data() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = vec![
            crate::instrument::test_support::fsysconf_one_input_analog_module(),
            ascii_frame(b"ORec,0\r\n"),
            ascii_frame(b"OMath,0\r\n"),
            ascii_frame(b"N 0001 DEGC ,3\nE"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            one_record_fdata_binary(),
            ascii_frame(b"aaaaaaaa001,12.5\nE"),
            ascii_frame(b"aaaaaaaa001,-7.5\nE"),
        ];
        // The write itself triggers exactly one more FData round trip
        // (`Command::ReadAllData` -> `cmd_fdata_all` -> `"FData,1\r\n"`).
        responses.push(one_record_fdata_binary());
        let device_thread = spawn_fake_device(listener, responses);
        let instrument = crate::instrument::Instrument::connect_to(addr).unwrap();

        let registry = registry_with("gm10dev", instrument);
        let f = factory(registry);
        let ctx = DeviceSupportContext {
            dtyp: DTYP,
            inp: "",
            out: "gm10dev CHAN_TRIG",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("bo").with("VAL", EpicsValue::Enum(1));
        device.init(&mut record).unwrap();
        device.write(&mut record).unwrap();

        let received = device_thread.join().unwrap();
        assert_eq!(received.last().unwrap(), "FData,1\r\n");
    }
}
