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
//! Record` and can call `record.record_type()`. `resolve_operation` is
//! the single place that mirrors every `devGM10_*.c::init_record`'s
//! command/address validation.

use crate::instrument::{Command, Instrument, InterruptCategory, ReadFailure, Registry};
use crate::link::{self, ChannelAddress, ChannelFamily};
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::database::DeviceDoneSource;
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, ReadStart,
    WriteStart,
};
use epics_rs::base::server::recgbl::alarm_status::{COMM_ALARM, READ_ALARM};
use epics_rs::base::server::record::{AlarmSeverity, Record, ScanType};
use epics_rs::base::types::EpicsValue;
use epics_rs::ca::server::ioc_app::DeviceSupportContext;
use std::sync::Arc;

/// Upstream DTYP shared by every GM10 record type (`gm10Support.dbd`).
pub const DTYP: &str = "Yokogawa GM10";

/// Every devGM10_* dset's `init(pass)` (devGM10_ai.c:73-79 and siblings):
/// pass 0 `scanIoInit`s the dset's `dummy_ioscanpvt`. The port has no
/// per-source I/O Intr list to create; the dummy is the per-record channel
/// `io_intr_receiver` hands an invalid input record.
pub fn dev_init(_after: bool) {}

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
    let (i, _) = link::strtol(arg);
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
            // devGM10_ao.c:99-139 has no `A` arm.
            if addr.family == ChannelFamily::Math {
                return Err(bad(record_type, "Math addresses are read-only"));
            }
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
            // devGM10_stringin.c:102-107 ignores any argument.
            "IP_ADDR" => Ok(Operation::IpAddr),
            "MODULE_STRING" => Ok(Operation::ModuleString(parse_module_index(
                arg,
                record_type,
                command,
            )?)),
            "ERROR" => {
                let arg =
                    arg.ok_or_else(|| bad(record_type, "ERROR requires a channel argument"))?;
                let (channel, _) = link::strtol(arg);
                if channel <= 0 || channel > 3 {
                    return Err(bad(record_type, "ERROR channel must be 1-3"));
                }
                Ok(Operation::ErrorText(channel as u32))
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
/// `gm10_binary_get`; the record's status-0 tail derives VAL from it).
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
    /// The C dset of this record type has `get_ioint_info` (input records).
    has_ioint_info: bool,
    is_io_intr: bool,
    resolved: Option<Resolved>,
    /// Keeps the never-fired `dummy_ioscanpvt` receiver open.
    dummy_ioscan: Option<tokio::sync::mpsc::Sender<()>>,
    /// Why the latest pass has no current value or its write never landed.
    fault: Option<Fault>,
    /// The outcome of this record's queued reads.
    read_failure: Arc<ReadFailure>,
}

#[derive(Clone, Copy)]
enum Fault {
    /// The unit was unreachable: a read had only the stale cache, or a
    /// write never reached it.
    Offline,
    /// The unit answered the pass's queued read with an error.
    ReadFailed,
}

impl GmDevice {
    fn new(registry: Arc<Registry>, link_text: String, record_type: &str) -> Self {
        Self {
            registry,
            link_text,
            has_ioint_info: matches!(record_type, "ai" | "bi" | "longin" | "mbbi" | "stringin"),
            is_io_intr: false,
            resolved: None,
            dummy_ioscan: None,
            fault: None,
            read_failure: Arc::default(),
        }
    }

    fn resolve(&self, record: &mut dyn Record) -> CaResult<Resolved> {
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
        Ok(Resolved { instrument, op })
    }
}

impl DeviceSupport for GmDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    /// A failed C `init_record` returns 1 with `invalid_flag` set and prints
    /// nothing; read and write then return 0 untouched and the record has
    /// no I/O Intr source (`devGM10_ai.c:104-187`). `resolved == None` is
    /// that state.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        self.resolved = self.resolve(record).ok();
        let Some(Resolved { op, .. }) = self.resolved else {
            // C's status 1 converts nothing (boRecord.c:159, mbboRecord.c:143).
            return Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched));
        };
        // devGM10_ao.c:92-99 returns 2 and defines the VAL it seeded.
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

    fn io_intr_receiver(&mut self) -> Option<epics_rs::base::runtime::sync::mpsc::Receiver<()>> {
        let Some(resolved) = self.resolved.as_ref() else {
            // An invalid input record gets `dummy_ioscanpvt` (`devGM10_ai.c:179-180`).
            if !self.has_ioint_info {
                return None;
            }
            let (tx, rx) = tokio::sync::mpsc::channel(1);
            self.dummy_ioscan = Some(tx);
            return Some(rx);
        };
        let category = operation_interrupt_category(resolved.op)?;
        Some(resolved.instrument.register_interrupt(category))
    }

    // A VAL read that is not I/O Intr queues the channel's refresh and
    // stays PACT; its re-entered pass reads the cache (devGM10_ai.c:199-214,
    // devGM10_bi.c:261-280, devGM10_longin.c:181-197). The scan thread
    // never waits on the instrument.
    fn read_begin(
        &mut self,
        _record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        let Some(Resolved { instrument, op }) = self.resolved.as_ref() else {
            return Ok(ReadStart::Synchronous);
        };
        if self.is_io_intr || done.pact() {
            return Ok(ReadStart::Synchronous);
        }
        let queued = match *op {
            Operation::AnalogVal(addr) => instrument.channel_start(
                addr.family,
                addr.index,
                done.device_done(),
                &self.read_failure,
            ),
            Operation::IntegerVal(channel) | Operation::BinaryVal(channel) => instrument
                .channel_start(
                    ChannelFamily::Signal,
                    channel,
                    done.device_done(),
                    &self.read_failure,
                ),
            _ => false,
        };
        Ok(if queued {
            ReadStart::Pending
        } else {
            ReadStart::Synchronous
        })
    }

    // C serves the cache as if current on an offline unit, and after the
    // unit answered the queued read with an error: `load_data_values` and
    // `load_misc_values` return 1 and `command_process` drops it
    // (drvGM10.c:965-966, 1102-1103, 1130-1131, 1263-1309). This pass keeps
    // the record's value and alarms instead. The configured address is the
    // one value not read from the unit.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Some(Resolved { instrument, op }) = &self.resolved else {
            return Ok(DeviceReadOutcome::ok());
        };
        let read_failed = self.read_failure.take();
        self.fault = if !matches!(op, Operation::IpAddr) && !instrument.online() {
            Some(Fault::Offline)
        } else if read_failed {
            Some(Fault::ReadFailed)
        } else {
            None
        };
        if self.fault.is_some() {
            return Ok(DeviceReadOutcome::failed());
        }
        match *op {
            Operation::AnalogVal(addr) => {
                let v = instrument.analog_get(addr.family, addr.index);
                record.put_field("VAL", EpicsValue::Double(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::IntegerVal(channel) => {
                let v = instrument.integer_get(channel);
                record.put_field("VAL", EpicsValue::Long(v))?;
                Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
            }
            Operation::BinaryVal(channel) => {
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

    // write_begin queues every write; the framework calls this only after
    // WriteStart::Synchronous, which write_begin never answers.
    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    /// C `write_*` sets PACT and `qmesg`s the command; `queue_func` ends the
    /// cycle once it ran (`drvGM10.c:1311-1338`). A command nothing queued
    /// fails the pass now, where C's `qmesg` leaves the record PACT for good
    /// on a unit `init_gm10` gave up on.
    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        let (Some(Resolved { instrument, .. }), Some(command)) =
            (&self.resolved, self.output_command(record)?)
        else {
            return Ok(WriteStart::Completed);
        };
        let queued = instrument.submit_write(command, done.device_done());
        self.fault = (!queued).then_some(Fault::Offline);
        if !queued {
            return Err(CaError::LinkError(
                "GM10 device support: unit offline".into(),
            ));
        }
        Ok(WriteStart::Pending)
    }

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        let stat = match self.fault? {
            Fault::Offline => COMM_ALARM,
            Fault::ReadFailed => READ_ALARM,
        };
        Some(DeviceAlarm {
            stat,
            sevr: AlarmSeverity::Invalid as u16,
            msg: "",
        })
    }

    // A write whose exchange the link lost ends its pass with the alarm.
    // devGM10_bo.c:192-197: the completing pass drops a REC_TRIG or REC_ERROR
    // bo's VAL back to 0.
    fn async_write_completed(&mut self, record: &mut dyn Record) {
        let Some(Resolved { instrument, op }) = self.resolved.as_ref() else {
            return;
        };
        self.fault = (!instrument.online()).then_some(Fault::Offline);
        if matches!(
            op,
            Operation::ChanTrig
                | Operation::MiscTrig
                | Operation::InfoTrig
                | Operation::StatTrig
                | Operation::ErrorClearSet
        ) {
            let _ = record.put_field("VAL", EpicsValue::Enum(0));
        }
    }
}

impl GmDevice {
    /// The command an output record's write queues; `None` while invalid.
    fn output_command(&self, record: &mut dyn Record) -> CaResult<Option<Command>> {
        let Some(Resolved { op, .. }) = &self.resolved else {
            return Ok(None);
        };
        Ok(Some(match *op {
            Operation::AnalogSet(addr) => {
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::Double(v)) => v,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                match addr.family {
                    ChannelFamily::Signal => Command::SetSignalOutput(addr.index, val),
                    ChannelFamily::Comm => Command::SetComm(addr.index, val),
                    ChannelFamily::Const => Command::SetConst(addr.index, val),
                    ChannelFamily::VarConst => Command::SetVarConst(addr.index, val),
                    ChannelFamily::Math => {
                        return Err(CaError::LinkError(
                            "ao does not support Math addresses".into(),
                        ));
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
            Operation::ChanTrig => Command::ReadAllData,
            Operation::MiscTrig => Command::ReadAllMisc,
            Operation::InfoTrig => Command::ReadAllInfos,
            Operation::StatTrig => Command::ReadStatus,
            Operation::RecordingModeSet => {
                let v = match record.get_field("VAL") {
                    Some(EpicsValue::Enum(v)) => v,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                Command::SetOpMode(v)
            }
            Operation::ErrorClearSet => Command::ClearError,
            Operation::AlarmAckSet => Command::AcknowledgeAlarms,
            Operation::ComputeCmdSet => {
                let v = match record.get_field("VAL") {
                    Some(EpicsValue::Enum(v)) => v,
                    _ => return Err(CaError::FieldNotFound("VAL".into())),
                };
                Command::SetCompute(v)
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
        }))
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
        Some(Box::new(GmDevice::new(
            registry.clone(),
            link_text.to_string(),
            ctx.record_type,
        )) as Box<dyn DeviceSupport>)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instrument::test_support::{
        ascii_frame, connect_default_fixture, default_init_responses, fdata_binary,
        one_record_fdata_binary, registry_with_offline, spawn_fake_device,
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

    #[test]
    fn ao_val_rejects_math_address() {
        let instrument = connect_default_fixture();
        assert!(resolve_operation("ao", "VAL", Some("A1"), &instrument).is_err());
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
    fn stringin_ip_addr_ignores_argument() {
        let instrument = connect_default_fixture();
        for arg in [None, Some("x")] {
            assert!(matches!(
                resolve_operation("stringin", "IP_ADDR", arg, &instrument).unwrap(),
                Operation::IpAddr
            ));
        }
    }

    #[test]
    fn numeric_arguments_read_like_c_atoi() {
        let instrument = connect_default_fixture();
        assert!(matches!(
            resolve_operation("stringin", "ERROR", Some("2x"), &instrument).unwrap(),
            Operation::ErrorText(2)
        ));
        assert!(matches!(
            resolve_operation("bi", "MODULE_PRESENCE", Some("3z"), &instrument).unwrap(),
            Operation::ModulePresence(3)
        ));
        assert!(matches!(
            resolve_operation("mbbi", "ALARM", Some("1.2"), &instrument).unwrap(),
            Operation::Alarm(_, 2)
        ));
        assert!(resolve_operation("mbbi", "ALARM", Some("1.2x"), &instrument).is_err());
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
            record_type: "ai",
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
            record_type: "ai",
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
            record_type: "ai",
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
            record_type: "ai",
            dtyp: DTYP,
            inp: "gm10dev VAL:1",
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

    // -- end-to-end: write (one live wire round trip) --

    /// C ao init_record returns 2 with the seeded VAL defined; bo returns 0
    /// so the record converts the RVAL the dset left.
    #[test]
    fn init_status_is_c_ao_two_and_bo_zero() {
        use epics_rs::base::server::device_support::DeviceUdf;
        let f = factory(registry_with("gm10dev", connect_default_fixture()));
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
            init("ao", "gm10dev VAL:W1"),
            DeviceInitOutcome::DoNotConvert(DeviceUdf::Defined)
        );
        assert_eq!(init("bo", "gm10dev CHAN_TRIG"), DeviceInitOutcome::ok());
    }

    #[test]
    fn bo_val_init_derives_val_from_seeded_rval() {
        use crate::cache::ChannelType;
        use epics_rs::base::runtime::task::test_block_on;
        use epics_rs::base::server::database::PvDatabase;
        use epics_rs::base::server::device_support::wire_device_to_record;
        use epics_rs::base::server::records::bo::BoRecord;
        test_block_on(async {
            let instrument = connect_default_fixture();
            crate::instrument::test_support::set_signal(
                &instrument,
                2,
                ChannelType::OutputBinary,
                1,
            );
            let f = factory(registry_with("gm10dev", instrument));
            let ctx = DeviceSupportContext {
                record_type: "bo",
                dtyp: DTYP,
                inp: "",
                out: "gm10dev VAL:2",
            };
            let device = f(&ctx).unwrap();
            let db = PvDatabase::new();
            db.add_record("TEST", Box::new(BoRecord::new(0)))
                .await
                .unwrap();
            let rec = db.get_record("TEST").unwrap();
            rec.write().common.dtyp = DTYP.into();
            // The framework's init, so boRecord's status-0 RVAL->VAL arm runs.
            wire_device_to_record(&db, &rec, device);
            let inst = rec.read();
            assert_eq!(inst.record.get_field("VAL"), Some(EpicsValue::Enum(1)));
            assert_eq!(inst.record.get_field("RVAL"), Some(EpicsValue::ULong(1)));
        });
    }

    #[test]
    fn bo_error_clear_device_error_reply_is_not_a_write_error() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = crate::instrument::test_support::default_init_responses();
        responses.push(b"E1,205:1:12\r\n".to_vec());
        responses.push(ascii_frame(b"junk 'Something broke' junk"));
        let device_thread = spawn_fake_device(listener, responses);
        let instrument = crate::instrument::Instrument::connect_to(addr).unwrap();
        let f = factory(registry_with("gm10dev", instrument));
        let ctx = DeviceSupportContext {
            record_type: "bo",
            dtyp: DTYP,
            inp: "",
            out: "gm10dev ERROR_CLEAR",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("bo").with("VAL", EpicsValue::Enum(1));
        device.init(&mut record).unwrap();
        let db = epics_rs::base::server::database::PvDatabase::new();
        let done = epics_rs::base::server::database::DeviceDone::new(
            &db,
            &epics_rs::base::server::record::RecordInstance::new(
                "GM10:ERR",
                epics_rs::base::server::records::bo::BoRecord::default(),
            ),
        );
        let start = device
            .write_begin(&mut record, &DeviceDoneSource::new(&done))
            .unwrap();
        assert!(matches!(start, WriteStart::Pending));
        let received = device_thread.join().unwrap();
        assert_eq!(received.last().unwrap(), "_ERR,205\r\n");
    }

    #[test]
    fn bo_write_begin_queues_the_command_and_stays_pending() {
        use epics_rs::base::server::database::{DeviceDone, PvDatabase};
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = crate::instrument::test_support::default_init_responses();
        responses.push(one_record_fdata_binary());
        let device_thread = spawn_fake_device(listener, responses);
        let instrument = crate::instrument::Instrument::connect_to(addr).unwrap();
        let f = factory(registry_with("gm10dev", instrument));
        let ctx = DeviceSupportContext {
            record_type: "bo",
            dtyp: DTYP,
            inp: "",
            out: "gm10dev CHAN_TRIG",
        };
        let mut device = f(&ctx).unwrap();
        let mut record = FakeRecord::new("bo");
        device.init(&mut record).unwrap();
        record.fields.insert("VAL", EpicsValue::Enum(1));
        let db = PvDatabase::new();
        let done = DeviceDone::new(
            &db,
            &epics_rs::base::server::record::RecordInstance::new(
                "GM10:TRIG",
                epics_rs::base::server::records::bo::BoRecord::default(),
            ),
        );
        let start = device
            .write_begin(&mut record, &DeviceDoneSource::new(&done))
            .unwrap();
        assert!(matches!(start, WriteStart::Pending));
        // VAL drops to 0 on the completing pass, not while PACT holds.
        assert_eq!(record.fields.get("VAL"), Some(&EpicsValue::Enum(1)));
        device.async_write_completed(&mut record);
        assert_eq!(record.fields.get("VAL"), Some(&EpicsValue::Enum(0)));
        let received = device_thread.join().unwrap();
        assert_eq!(received.last().unwrap(), "FData,1\r\n");
    }

    #[test]
    fn failed_init_is_silent_and_inert() {
        use tokio::sync::mpsc::error::TryRecvError;
        let db = epics_rs::base::server::database::PvDatabase::new();
        let done = epics_rs::base::server::database::DeviceDone::new(
            &db,
            &epics_rs::base::server::record::RecordInstance::new(
                "GM10:BOGUS",
                epics_rs::base::server::records::bo::BoRecord::default(),
            ),
        );
        let registry = registry_with("gm10dev", connect_default_fixture());
        let f = factory(registry);
        for (record_type, inp, out) in [
            ("ai", "gm10dev VAL:7", ""),
            ("ai", "nosuchdev VAL:1", ""),
            ("bo", "", "gm10dev BOGUS"),
        ] {
            let ctx = DeviceSupportContext {
                record_type,
                dtyp: DTYP,
                inp,
                out,
            };
            let mut device = f(&ctx).unwrap();
            let mut record = FakeRecord::new(record_type);
            assert_eq!(
                device.init(&mut record).unwrap(),
                DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched)
            );
            let mut io_intr = device.io_intr_receiver();
            assert_eq!(io_intr.is_some(), record_type == "ai");
            if let Some(rx) = io_intr.as_mut() {
                assert!(rx.try_recv().is_err_and(|e| e == TryRecvError::Empty));
            }
            if record_type == "ai" {
                assert!(!device.read(&mut record).unwrap().did_compute());
            } else {
                let start = device
                    .write_begin(&mut record, &DeviceDoneSource::new(&done))
                    .unwrap();
                assert!(matches!(start, WriteStart::Completed));
            }
            assert!(record.fields.is_empty(), "{inp}{out}");
        }
    }

    // -- queue_func ends the passes it ran (drvGM10.c:1327-1332) --

    const FULL_BAND_CHILD: &str = "GM10_FULL_BAND_CHILD";

    /// Holds the low band's one worker until its `FullBand` drops.
    #[derive(Clone, Default)]
    struct Hold(Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>);

    impl epics_rs::base::runtime::background::CallbackBody for Hold {
        fn run(&self) {
            let (open, wake) = &*self.0;
            let mut open = open.lock().unwrap();
            while !*open {
                open = wake.wait(open).unwrap();
            }
        }
    }

    struct FullBand(Hold);

    impl Drop for FullBand {
        fn drop(&mut self) {
            let (open, wake) = &*(self.0).0;
            *open.lock().unwrap() = true;
            wake.notify_all();
        }
    }

    /// Runs `body` in a child that re-runs `test`, with the low callback
    /// band refusing work: its size and threads are set once per process,
    /// before it arms. The records' PRIO is LOW.
    fn with_full_low_band(test: &str, body: impl FnOnce()) {
        use epics_rs::base::runtime::background::callback_executor::{
            set_parallel_threads, set_queue_size,
        };
        use epics_rs::base::runtime::background::{CallbackCell, CallbackError};
        use epics_rs::base::runtime::task::{CallbackPriority, background_arm, request_cell};
        if std::env::var_os(FULL_BAND_CHILD).is_none() {
            let out = std::process::Command::new(std::env::current_exe().unwrap())
                .args([test, "--exact", "--nocapture"])
                .env(FULL_BAND_CHILD, "1")
                .output()
                .unwrap();
            let stdout = String::from_utf8_lossy(&out.stdout);
            let stderr = String::from_utf8_lossy(&out.stderr);
            assert!(out.status.success(), "{stdout}{stderr}");
            assert!(stdout.contains("1 passed"), "{stdout}");
            return;
        }
        set_queue_size(1);
        set_parallel_threads(1, Some(CallbackPriority::Low));
        background_arm();
        let hold = Hold::default();
        let _full = (0..64)
            .find_map(|_| {
                match request_cell(CallbackPriority::Low, &CallbackCell::new(hold.clone())) {
                    Ok(()) => None,
                    Err(CallbackError::QueueFull) => Some(FullBand(hold.clone())),
                    Err(e) => panic!("{e:?}"),
                }
            })
            .expect("the low band never filled");
        body();
    }

    /// `AI` reads channel 1 and `AO` writes K1 of one instrument, whose
    /// fixture then answers `extra`; `AO`'s FLNK is `flnk`.
    async fn ai_and_ao(
        flnk: &str,
        extra: Vec<Vec<u8>>,
    ) -> (
        Arc<epics_rs::base::server::database::PvDatabase>,
        std::thread::JoinHandle<Vec<String>>,
    ) {
        let (instrument, device) = fixture_then(extra);
        (ai_and_ao_on(flnk, instrument).await, device)
    }

    /// An instrument loaded by the init fixture, which then answers `extra`.
    fn fixture_then(
        extra: Vec<Vec<u8>>,
    ) -> (Arc<Instrument>, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.extend(extra);
        let device = spawn_fake_device(listener, responses);
        (Instrument::connect_to(addr).unwrap(), device)
    }

    /// [`ai_and_ao`] on `instrument`.
    async fn ai_and_ao_on(
        flnk: &str,
        instrument: Arc<Instrument>,
    ) -> Arc<epics_rs::base::server::database::PvDatabase> {
        ai_and_ao_with(flnk, "", instrument).await
    }

    /// [`ai_and_ao_on`] beside the records `more`.
    async fn ai_and_ao_with(
        flnk: &str,
        more: &str,
        instrument: Arc<Instrument>,
    ) -> Arc<epics_rs::base::server::database::PvDatabase> {
        use epics_rs::base::server::ioc_builder::IocBuilder;
        use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
        // gm10Support.dbd's ai and ao rows, as yokogawa-gm10-ioc lists them.
        static AI: &[DevSup] = &[DevSup {
            name: "devGM10_ai",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 3, 4], false),
        }];
        static AO: &[DevSup] = &[DevSup {
            name: "devGM10_ao",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }];
        let records = format!(
            r#"
record(ai, "AI") {{
    field(DTYP, "{DTYP}")
    field(INP, "@gm10dev VAL:1")
}}
record(ao, "AO") {{
    field(DTYP, "{DTYP}")
    field(OUT, "@gm10dev VAL:K1")
    field(FLNK, "{flnk}")
}}
{more}
"#
        );
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &[("ai", AI), ("ao", AO)],
                factory(registry_with("gm10dev", instrument)),
            )
            .db_string(&records, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        db
    }

    /// Whether `AI` and `AO` are still PACT after up to two seconds.
    async fn pact_after_settling(db: &epics_rs::base::server::database::PvDatabase) -> [bool; 2] {
        let pact = |name: &str| db.get_record(name).unwrap().read().is_processing();
        for _ in 0..400 {
            if !pact("AI") && !pact("AO") {
                break;
            }
            epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(5)).await;
        }
        [pact("AI"), pact("AO")]
    }

    /// Channel 1 reading 5.678 (FChInfo gives it 3 decimals).
    fn channel_1_reads_5678() -> Vec<u8> {
        fdata_binary(&[(1, 1, 0, [0; 4], 5678)])
    }

    /// `records` over a unit that never connected.
    async fn offline_db(records: &str) -> Arc<epics_rs::base::server::database::PvDatabase> {
        use epics_rs::base::server::ioc_builder::IocBuilder;
        use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
        static AI: &[DevSup] = &[DevSup {
            name: "devGM10_ai",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 3, 4], false),
        }];
        static AO: &[DevSup] = &[DevSup {
            name: "devGM10_ao",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }];
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &[("ai", AI), ("ao", AO)],
                factory(registry_with_offline("gm10dev")),
            )
            .db_string(records, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        db
    }

    /// (PACT, STAT, SEVR) of `name`.
    fn pact_and_alarm(
        db: &epics_rs::base::server::database::PvDatabase,
        name: &str,
    ) -> (bool, EpicsValue, EpicsValue) {
        (
            db.get_record(name).unwrap().read().is_processing(),
            db.get_pv(format!("{name}.STAT")).unwrap(),
            db.get_pv(format!("{name}.SEVR")).unwrap(),
        )
    }

    /// A read on a unit that never connected has no queue thread to end
    /// it and nothing current to serve: the pass that starts it ends it,
    /// keeps VAL and fails with COMM_ALARM, INVALID.
    #[test]
    fn a_read_on_an_offline_unit_ends_its_pass_with_comm_alarm() {
        let records = format!(
            r#"record(ai, "AI") {{ field(DTYP, "{DTYP}") field(INP, "@gm10dev VAL:K1") field(VAL, "7") }}"#
        );
        epics_rs::base::runtime::task::test_block_on(async {
            let db = offline_db(&records).await;
            let put = db
                .put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
                .await;
            assert_eq!(
                pact_and_alarm(&db, "AI"),
                (false, EpicsValue::Short(9), EpicsValue::Short(3)),
                "PACT, STAT, SEVR"
            );
            assert!(put.is_err(), "the put fails with the pass");
            assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(7.0));
        });
    }

    /// A write to a unit that never connected never reaches it: the pass
    /// ends now with COMM_ALARM, INVALID and the put fails.
    #[test]
    fn a_write_to_an_offline_unit_ends_its_pass_with_comm_alarm() {
        let records = format!(
            r#"record(ao, "AO") {{ field(DTYP, "{DTYP}") field(OUT, "@gm10dev VAL:K1") }}"#
        );
        epics_rs::base::runtime::task::test_block_on(async {
            let db = offline_db(&records).await;
            let put = db
                .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                .await;
            assert_eq!(
                pact_and_alarm(&db, "AO"),
                (false, EpicsValue::Short(9), EpicsValue::Short(3)),
                "PACT, STAT, SEVR"
            );
            assert!(put.is_err(), "the put fails with the pass");
        });
    }

    /// `AI` read and `AO` written once, then let settle. A put fails when
    /// its pass ends at once, which is not what these tests look at.
    async fn read_and_write(db: &epics_rs::base::server::database::PvDatabase) {
        let _ = db
            .put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
            .await;
        let _ = db
            .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
            .await;
        pact_after_settling(db).await;
    }

    /// (PACT, STAT, SEVR) and VAL of `AI` once a read of it settled.
    async fn proc_ai(
        db: &epics_rs::base::server::database::PvDatabase,
    ) -> ((bool, EpicsValue, EpicsValue), EpicsValue) {
        db.put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
            .await
            .unwrap();
        pact_after_settling(db).await;
        (pact_and_alarm(db, "AI"), db.get_pv("AI.VAL").unwrap())
    }

    const COMM_INVALID: (bool, EpicsValue, EpicsValue) =
        (false, EpicsValue::Short(9), EpicsValue::Short(3));
    const NO_ALARM: (bool, EpicsValue, EpicsValue) =
        (false, EpicsValue::Short(0), EpicsValue::Short(0));

    /// A reachable unit that answers the read with a device error has no
    /// current value for it: the pass keeps VAL and ends with READ_ALARM,
    /// INVALID. The next read the unit answers with data clears it.
    #[test]
    fn a_device_error_reply_to_a_read_ends_its_pass_with_read_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (db, _device) = ai_and_ao(
                "",
                vec![
                    b"E1,205:1:12\r\n".to_vec(),
                    ascii_frame(b"junk 'Something broke' junk"),
                    channel_1_reads_5678(),
                ],
            )
            .await;
            let after_device_error = proc_ai(&db).await;
            let after_data = proc_ai(&db).await;
            assert_eq!(
                [after_device_error, after_data],
                [
                    (
                        (false, EpicsValue::Short(1), EpicsValue::Short(3)),
                        EpicsValue::Double(0.0)
                    ),
                    (NO_ALARM, EpicsValue::Double(5.678))
                ],
                "(PACT, STAT, SEVR) and VAL after each read"
            );
        });
    }

    /// An I/O Intr record scans once on each change of the link: losing the
    /// unit ends its pass with COMM_ALARM, INVALID, and the unit's return
    /// clears that at once. A further exchange that changes nothing scans
    /// nothing. `CNT` counts the record's passes; the unit's frames carry
    /// the cached 1.234, so the value IN reads does not depend on timing.
    #[test]
    fn io_intr_records_scan_once_on_each_change_of_the_link() {
        use epics_rs::base::runtime::task::sleep;
        use std::time::Duration;
        epics_rs::base::runtime::task::test_block_on(async {
            let cached = || fdata_binary(&[(1, 1, 0, [0; 4], 1234)]);
            let (instrument, _device) = fixture_then(vec![
                b"XX\r\n".to_vec(),
                b"XX\r\n".to_vec(),
                cached(),
                cached(),
            ]);
            let more = format!(
                r#"
record(ai, "IN") {{
    field(DTYP, "{DTYP}")
    field(INP, "@gm10dev VAL:1")
    field(SCAN, "I/O Intr")
    field(FLNK, "CNT")
}}
record(calc, "CNT") {{ field(CALC, "A+1") field(INPA, "CNT") }}
"#
            );
            let db = ai_and_ao_with("", &more, instrument).await;
            // `iocRun`: I/O Intr pulses may now process.
            db.scan_control().run();
            let mut trace = Vec::new();
            for passes in [1.0, 1.0, 2.0, 2.0] {
                proc_ai(&db).await;
                for _ in 0..400 {
                    if db.get_pv("CNT.VAL").unwrap() == EpicsValue::Double(passes) {
                        break;
                    }
                    sleep(Duration::from_millis(5)).await;
                }
                sleep(Duration::from_millis(50)).await;
                trace.push((
                    pact_and_alarm(&db, "IN"),
                    db.get_pv("IN.VAL").unwrap(),
                    db.get_pv("CNT.VAL").unwrap(),
                ));
            }
            let at =
                |alarm, val, passes| (alarm, EpicsValue::Double(val), EpicsValue::Double(passes));
            assert_eq!(
                trace,
                [
                    at(COMM_INVALID, 0.0, 1.0),
                    at(COMM_INVALID, 0.0, 1.0),
                    at(NO_ALARM, 1.234, 2.0),
                    at(NO_ALARM, 1.234, 2.0),
                ],
                "IN's (PACT, STAT, SEVR), VAL and passes after the read that loses \
                 the unit, a second failed read, the read the unit answers, and one more"
            );
        });
    }

    /// The unit hangs up once it loaded: the read finds no reply and keeps
    /// VAL, the write never lands, and both passes end with COMM_ALARM.
    #[test]
    fn a_link_lost_after_boot_ends_reads_and_writes_with_comm_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (db, _device) = ai_and_ao("", Vec::new()).await;
            read_and_write(&db).await;
            assert_eq!(
                [pact_and_alarm(&db, "AI"), pact_and_alarm(&db, "AO")],
                [COMM_INVALID, COMM_INVALID],
                "PACT, STAT, SEVR of AI and AO"
            );
            assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(0.0));
        });
    }

    /// A reply that is no frame alarms the read; the unit's next whole
    /// frame clears it and serves the value it carried.
    #[test]
    fn a_good_exchange_after_a_wire_error_clears_the_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (db, _device) =
                ai_and_ao("", vec![b"XX\r\n".to_vec(), channel_1_reads_5678()]).await;
            let after_wire_error = proc_ai(&db).await;
            let after_good_frame = proc_ai(&db).await;
            assert_eq!(
                [after_wire_error, after_good_frame],
                [
                    (COMM_INVALID, EpicsValue::Double(0.0)),
                    (NO_ALARM, EpicsValue::Double(5.678))
                ],
                "(PACT, STAT, SEVR) and VAL after each read"
            );
        });
    }

    /// Nothing serves a unit whose queue thread panicked: its cache is
    /// never refreshed again, so reads and writes end with COMM_ALARM.
    #[test]
    fn a_panicked_queue_thread_ends_reads_and_writes_with_comm_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (instrument, _device) = fixture_then(Vec::new());
            let db = ai_and_ao_on("", instrument.clone()).await;
            instrument.panic_queue_thread();
            read_and_write(&db).await;
            assert_eq!(
                [pact_and_alarm(&db, "AI"), pact_and_alarm(&db, "AO")],
                [COMM_INVALID, COMM_INVALID],
                "PACT, STAT, SEVR of AI and AO"
            );
            assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(0.0));
        });
    }

    /// A read and a write queued behind the job that panics the queue
    /// thread never run, so their passes end with COMM_ALARM as it unwinds.
    #[test]
    fn records_queued_when_the_queue_thread_panics_end_their_passes_with_comm_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (instrument, _device) = fixture_then(Vec::new());
            let db = ai_and_ao_on("", instrument.clone()).await;
            let fire = instrument.queue_panic();
            db.put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
                .await
                .unwrap();
            db.put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                .await
                .unwrap();
            fire.send(()).unwrap();
            pact_after_settling(&db).await;
            assert_eq!(
                [pact_and_alarm(&db, "AI"), pact_and_alarm(&db, "AO")],
                [COMM_INVALID, COMM_INVALID],
                "PACT, STAT, SEVR of AI and AO"
            );
            assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(0.0));
        });
    }

    /// A read and a write whose command is running when the queue thread
    /// panics end their passes with COMM_ALARM as it unwinds.
    #[test]
    fn records_running_when_the_queue_thread_panics_end_their_passes_with_comm_alarm() {
        epics_rs::base::runtime::task::test_block_on(async {
            let mut ends = Vec::new();
            for (record, field, value) in [
                ("AI", "PROC", EpicsValue::Short(1)),
                ("AO", "VAL", EpicsValue::Double(2.5)),
            ] {
                let (instrument, _device) = fixture_then(Vec::new());
                let db = ai_and_ao_on("", instrument.clone()).await;
                instrument.panic_in_next_command();
                db.put_record_field_from_ca_no_notify(record, field, value)
                    .await
                    .unwrap();
                pact_after_settling(&db).await;
                ends.push((record, pact_and_alarm(&db, record)));
            }
            assert_eq!(
                ends,
                [("AI", COMM_INVALID), ("AO", COMM_INVALID)],
                "each record's PACT, STAT, SEVR"
            );
        });
    }

    #[test]
    fn a_read_and_a_write_end_on_the_queue_thread_while_their_band_is_full() {
        with_full_low_band(
            "device_support::tests::a_read_and_a_write_end_on_the_queue_thread_while_their_band_is_full",
            || {
                epics_rs::base::runtime::task::test_block_on(async {
                    let (db, device) =
                        ai_and_ao("", vec![channel_1_reads_5678(), b"E0\r\n".to_vec()]).await;
                    db.put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
                        .await
                        .unwrap();
                    db.put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                        .await
                        .unwrap();
                    assert_eq!(pact_after_settling(&db).await, [false, false], "PACT");
                    assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(5.678));
                    assert_eq!(
                        device.join().unwrap()[10..],
                        ["FData,1,0001,0001\r\n", "SKConst,1,2.5\r\n"]
                    );
                });
            },
        );
    }

    /// The write's completing pass runs on the queue thread, and its FLNK
    /// reads the same instrument: that read must queue, not wait on the
    /// thread running it.
    #[test]
    fn a_write_whose_flnk_reads_the_same_instrument_ends_both_passes() {
        with_full_low_band(
            "device_support::tests::a_write_whose_flnk_reads_the_same_instrument_ends_both_passes",
            || {
                epics_rs::base::runtime::task::test_block_on(async {
                    let (db, device) =
                        ai_and_ao("AI", vec![b"E0\r\n".to_vec(), channel_1_reads_5678()]).await;
                    db.put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                        .await
                        .unwrap();
                    assert_eq!(pact_after_settling(&db).await, [false, false], "PACT");
                    assert_eq!(db.get_pv("AI.VAL").unwrap(), EpicsValue::Double(5.678));
                    assert_eq!(
                        device.join().unwrap()[10..],
                        ["SKConst,1,2.5\r\n", "FData,1,0001,0001\r\n"]
                    );
                });
            },
        );
    }
}
