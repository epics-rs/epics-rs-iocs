//! Dynamic `DeviceSupport` for all 8 GM10 record types
//! (`devGM10_{ai,ao,bi,bo,mbbi,mbbo,longin,stringin}.c`).
//!
//! `gm10Support.dbd` binds every record type under the SAME DTYP string:
//! `device(<type>,INST_IO,devGM10_<type>,"Yokogawa GM10")` — the C dset
//! (hence the field layout/read-write shape) is selected by which record
//! type is loading it, not by DTYP. `DeviceSupportContext` (the dynamic
//! factory's only input) carries `dtyp`/`inp`/`out` but no record-type
//! field, so [`GmDevice`] defers command resolution to
//! [`DeviceSupport::init_common`], which does receive the concrete `&mut dyn
//! Record` and can call `record.record_type()`. `resolve_operation` is
//! the single place that mirrors every `devGM10_*.c::init_record`'s
//! command/address validation.

use crate::instrument::{Command, Instrument, InterruptCategory, ReadFailure, Registry};
use crate::link::{self, ChannelAddress, ChannelFamily};
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::database::{AsyncDbHandle, DeviceDoneSource};
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DevicePost, DeviceReadOutcome, DeviceSupport, DeviceUdf,
    DsetInit, IoIntrAction, ReadStart, WriteStart,
};
use epics_rs::base::server::recgbl::alarm_status::{COMM_ALARM, READ_ALARM};
use epics_rs::base::server::recgbl::{AlarmResetResult, EventMask};
use epics_rs::base::server::record::{AlarmSeverity, CommonFields, Record, ScanType};
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

impl Operation {
    /// False for the configured address, the one value that is not read
    /// from the unit (`gm10_system_info`, drvGM10.c:1620-1630).
    fn reads_the_unit(self) -> bool {
        !matches!(self, Operation::IpAddr)
    }
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

/// The status a `devGM10_*.c::init_record` returns for a link it resolved
/// and seeded: ao's 2 with the VAL it seeded defined (devGM10_ao.c:150-158),
/// every other 0.
fn init_outcome(op: Operation) -> DeviceInitOutcome {
    match op {
        Operation::AnalogSet(_) => DeviceInitOutcome::DoNotConvert(DeviceUdf::Defined),
        _ => DeviceInitOutcome::ok(),
    }
}

/// The record's side of iocInit after a dset `init_record` returned
/// `outcome`, as the framework runs it: what a record whose unit was present
/// at boot was left with.
fn replay_init_tail(
    record: &mut dyn Record,
    common: &mut CommonFields,
    outcome: DeviceInitOutcome,
) {
    record.after_dset_init();
    let dset_init = match outcome {
        DeviceInitOutcome::Ok(udf) => {
            apply_udf(common, udf);
            if let Some(udf) = record.dset_init_convert() {
                common.udf = u8::from(udf);
            }
            DsetInit::Ok
        }
        DeviceInitOutcome::DoNotConvert(udf) => {
            apply_udf(common, udf);
            DsetInit::DoNotConvert
        }
        DeviceInitOutcome::Dead { .. } => DsetInit::Failed,
    };
    record.init_record_tail(common);
    record.seed_deadband_tracking(dset_init);
}

fn apply_udf(common: &mut CommonFields, udf: DeviceUdf) {
    match udf {
        DeviceUdf::Untouched => {}
        DeviceUdf::Defined => common.udf = 0,
        DeviceUdf::Undefined => common.udf = 1,
    }
}

/// The checks of the record's `init_record` against the loaded unit.
fn check_link(link_text: &str, instrument: &Instrument, record_type: &str) -> Option<Operation> {
    let parsed = link::parse_link(link_text)?;
    resolve_operation(record_type, parsed.command, parsed.arg, instrument).ok()
}

struct Resolved {
    instrument: Arc<Instrument>,
    op: Operation,
}

/// Where the record's link stands.
enum Binding {
    /// C's `invalid_flag` (devGM10_ai.c:104-187): read and write do nothing,
    /// and an input scans on a dummy IOSCANPVT.
    Inert,
    /// The unit has not loaded since boot, so the link is unchecked.
    Waiting(Waiting),
    Bound(Resolved),
}

/// A record whose setup waits for its unit's first load.
struct Waiting {
    instrument: Arc<Instrument>,
    /// The alarm iocInit left (iocInit.c:520-522), until a pass runs.
    boot_alarm: Option<(u16, AlarmSeverity)>,
    /// The I/O Intr sender handed out before the category could be known.
    interrupt: Option<tokio::sync::mpsc::Sender<()>>,
}

/// Shared `DeviceSupport` for every GM10 record type. The link is parsed
/// (and the device looked up / operation resolved) lazily in
/// `init_common()`, since only there is the concrete record type known.
pub struct GmDevice {
    registry: Arc<Registry>,
    link_text: String,
    name: String,
    db: Option<AsyncDbHandle>,
    record_type: &'static str,
    /// The C dset of this record type has `get_ioint_info` (input records).
    has_ioint_info: bool,
    is_io_intr: bool,
    binding: Binding,
    /// Keeps the never-fired `dummy_ioscanpvt` receiver open.
    dummy_ioscan: Option<tokio::sync::mpsc::Sender<()>>,
    /// Why the latest pass has no current value or its write never landed.
    fault: Option<Fault>,
    /// The outcome of this record's queued reads.
    read_failure: Arc<ReadFailure>,
    /// The posts of the setup that just ran.
    posts: Vec<DevicePost>,
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
            name: String::new(),
            db: None,
            record_type: "",
            has_ioint_info: matches!(record_type, "ai" | "bi" | "longin" | "mbbi" | "stringin"),
            is_io_intr: false,
            binding: Binding::Inert,
            dummy_ioscan: None,
            fault: None,
            read_failure: Arc::default(),
            posts: Vec::new(),
        }
    }

    /// The deferred `init_record`, on the queue thread under the record's
    /// lock, once the unit's first load completed: the record is checked and
    /// seeded as a present-at-boot init does it, so an output takes the
    /// unit's starting value over a put that failed before. Its alarm stays
    /// the one its last pass set, or the one iocInit left if none ran.
    fn settle(&mut self, record: &mut dyn Record, common: &mut CommonFields) {
        let Binding::Waiting(waiting) = std::mem::replace(&mut self.binding, Binding::Inert) else {
            return;
        };
        let Waiting {
            instrument,
            boot_alarm,
            interrupt,
        } = waiting;
        let op = check_link(&self.link_text, &instrument, self.record_type);
        if let Some(tx) = interrupt {
            match op.and_then(operation_interrupt_category) {
                Some(category) => instrument.attach_interrupt(category, tx),
                None => self.dummy_ioscan = Some(tx),
            }
        }
        let val = record.get_field("VAL");
        let op = op.filter(|&op| seed_initial_value(&instrument, op, record).is_ok());
        if let Some(op) = op {
            replay_init_tail(record, common, init_outcome(op));
        }
        self.post_setup(record, common, boot_alarm, val);
        self.binding = match op {
            Some(op) => Binding::Bound(Resolved { instrument, op }),
            None => Binding::Inert,
        };
    }

    /// The posts C's `monitor` makes for a VAL the setup moved from `val`
    /// and for `boot_alarm`, the alarm init replaced, put back.
    fn post_setup(
        &mut self,
        record: &dyn Record,
        common: &mut CommonFields,
        boot_alarm: Option<(u16, AlarmSeverity)>,
        val: Option<EpicsValue>,
    ) {
        let (prev_stat, prev_sevr) = (common.stat, common.sevr);
        let (stat, sevr) = boot_alarm.unwrap_or((prev_stat, prev_sevr));
        (common.stat, common.sevr) = (stat, sevr);
        let reset = AlarmResetResult {
            alarm_changed: (prev_stat, prev_sevr) != (stat, sevr),
            prev_sevr,
            prev_stat,
            amsg_changed: false,
            acks_posted: false,
        };
        let now = record.get_field("VAL");
        let mut mask = EventMask::NONE;
        if now != val {
            mask |= EventMask::VALUE | EventMask::LOG;
        }
        if reset.alarm_changed {
            mask |= EventMask::ALARM;
        }
        if let Some(value) = now.filter(|_| !mask.is_empty()) {
            self.posts.push(DevicePost {
                field: "VAL",
                mask,
                value,
            });
        }
        self.posts.extend(DevicePost::alarm_reset(common, &reset));
    }
}

/// The queue thread's run of a record's [`GmDevice::settle`].
fn settle_on_load(db: Option<AsyncDbHandle>, name: &str) {
    let Some(db) = db else {
        return;
    };
    let posts = db.with_held(name, |instance| {
        instance.with_device(|record, common, dev| {
            dev.io_intr_callback(record, common);
            dev.take_device_posts()
        })
    });
    db.post_events(name, posts.flatten().unwrap_or_default());
}

impl DeviceSupport for GmDevice {
    fn dtyp(&self) -> &str {
        DTYP
    }

    /// A failed C `init_record` returns 1 with `invalid_flag` set and prints
    /// nothing; read and write then return 0 untouched and the record has
    /// no I/O Intr source (`devGM10_ai.c:104-187`). `Binding::Inert` is that
    /// state. C's status 1 converts nothing (boRecord.c:159,
    /// mbboRecord.c:143), and neither does a record whose unit has not
    /// loaded: its `settle` converts.
    fn init_common(
        &mut self,
        record: &mut dyn Record,
        common: &mut CommonFields,
    ) -> CaResult<DeviceInitOutcome> {
        self.record_type = record.record_type();
        let unconverted = Ok(DeviceInitOutcome::DoNotConvert(DeviceUdf::Untouched));
        let Some(instrument) =
            link::parse_link(&self.link_text).and_then(|parsed| self.registry.get(parsed.device))
        else {
            self.binding = Binding::Inert;
            return unconverted;
        };
        // devGM10_stringin.c:101-104 checks nothing on IP_ADDR, so the
        // configured address is served from boot, loaded or not.
        if let Some(op) = check_link(&self.link_text, &instrument, self.record_type)
            .filter(|op| !op.reads_the_unit())
        {
            self.binding = Binding::Bound(Resolved { instrument, op });
            return Ok(init_outcome(op));
        }
        let settle = {
            let (db, name) = (self.db.clone(), self.name.clone());
            move || settle_on_load(db, &name)
        };
        if instrument.after_first_load(settle) {
            self.binding = Binding::Waiting(Waiting {
                instrument,
                boot_alarm: Some((common.stat, common.sevr)),
                interrupt: None,
            });
            (common.stat, common.sevr) = (COMM_ALARM, AlarmSeverity::Invalid);
            return unconverted;
        }
        let op = check_link(&self.link_text, &instrument, self.record_type)
            .filter(|&op| seed_initial_value(&instrument, op, record).is_ok());
        let Some(op) = op else {
            self.binding = Binding::Inert;
            return unconverted;
        };
        self.binding = Binding::Bound(Resolved { instrument, op });
        Ok(init_outcome(op))
    }

    fn set_record_info(&mut self, name: &str, scan: ScanType) {
        self.name = name.to_string();
        self.is_io_intr = matches!(scan, ScanType::IoIntr);
    }

    fn bind_database(&mut self, db: AsyncDbHandle) {
        self.db = Some(db);
    }

    // The settle is this device's only callback under its record's lock.
    fn io_intr_callback(
        &mut self,
        record: &mut dyn Record,
        common: &mut CommonFields,
    ) -> IoIntrAction {
        self.settle(record, common);
        IoIntrAction::Skip
    }

    fn take_device_posts(&mut self) -> Vec<DevicePost> {
        std::mem::take(&mut self.posts)
    }

    fn io_intr_receiver(&mut self) -> Option<epics_rs::base::runtime::sync::mpsc::Receiver<()>> {
        let waiting = match &mut self.binding {
            Binding::Bound(resolved) => {
                let category = operation_interrupt_category(resolved.op)?;
                return Some(resolved.instrument.register_interrupt(category));
            }
            Binding::Waiting(waiting) => Some(waiting),
            Binding::Inert => None,
        };
        // An invalid input record gets `dummy_ioscanpvt` (`devGM10_ai.c:179-180`).
        // A waiting record's sender is placed when the link is checked.
        if !self.has_ioint_info {
            return None;
        }
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        match waiting {
            Some(waiting) => waiting.interrupt = Some(tx),
            None => self.dummy_ioscan = Some(tx),
        }
        Some(rx)
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
        let Binding::Bound(Resolved { instrument, op }) = &self.binding else {
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
        let (instrument, op) = match &mut self.binding {
            Binding::Bound(Resolved { instrument, op }) => (instrument, op),
            Binding::Waiting(waiting) => {
                waiting.boot_alarm = None;
                self.fault = Some(Fault::Offline);
                return Ok(DeviceReadOutcome::failed());
            }
            Binding::Inert => {
                self.fault = None;
                return Ok(DeviceReadOutcome::ok());
            }
        };
        let read_failed = self.read_failure.take();
        self.fault = if op.reads_the_unit() && !instrument.online() {
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
    /// on a unit `init_gm10` gave up on. A unit that has not loaded since
    /// boot is offline the same way, and writes nothing.
    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        let queued = match &mut self.binding {
            Binding::Bound(Resolved { instrument, op }) => {
                let command = output_command(*op, record)?;
                instrument.submit_write(command, done.device_done())
            }
            Binding::Waiting(waiting) => {
                waiting.boot_alarm = None;
                false
            }
            Binding::Inert => {
                self.fault = None;
                return Ok(WriteStart::Completed);
            }
        };
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

    // A write whose exchange the link lost ends its pass with the alarm; one
    // that landed defines the record, which C does on every completing pass
    // (devGM10_bo.c:199). devGM10_bo.c:192-197: the completing pass drops a
    // REC_TRIG or REC_ERROR bo's VAL back to 0.
    fn async_write_completed_common(&mut self, record: &mut dyn Record, common: &mut CommonFields) {
        let Binding::Bound(Resolved { instrument, op }) = &self.binding else {
            return;
        };
        self.fault = (!instrument.online()).then_some(Fault::Offline);
        if self.fault.is_none() {
            common.udf = 0;
        }
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

/// The command an output record's write of `op` queues.
fn output_command(op: Operation, record: &dyn Record) -> CaResult<Command> {
    Ok(match op {
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
    })
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
        absent_at_boot, ascii_frame, connect_default_fixture, default_init_responses, fdata_binary,
        one_record_fdata_binary, registry_with_offline, spawn_fake_device,
        spawn_fake_device_sessions, spawn_gated_fake_device,
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
        device
            .init_common(&mut record, &mut CommonFields::default())
            .unwrap();
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
        device
            .init_common(&mut record, &mut CommonFields::default())
            .unwrap();
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
            device
                .init_common(
                    &mut FakeRecord::new(record_type),
                    &mut CommonFields::default(),
                )
                .unwrap()
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
        device
            .init_common(&mut record, &mut CommonFields::default())
            .unwrap();
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
        device
            .init_common(&mut record, &mut CommonFields::default())
            .unwrap();
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
        device.async_write_completed_common(&mut record, &mut CommonFields::default());
        assert_eq!(record.fields.get("VAL"), Some(&EpicsValue::Enum(0)));
        let received = device_thread.join().unwrap();
        assert_eq!(received.last().unwrap(), "FData,1\r\n");
    }

    /// A bo `BO` writing `CHAN_TRIG` to a unit the init fixture loaded, which
    /// then answers `extra`.
    async fn chan_trig_bo(
        extra: Vec<Vec<u8>>,
    ) -> (
        Arc<epics_rs::base::server::database::PvDatabase>,
        std::thread::JoinHandle<Vec<String>>,
    ) {
        use epics_rs::base::server::ioc_builder::IocBuilder;
        use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
        static BO: &[DevSup] = &[DevSup {
            name: "devGM10_bo",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }];
        let (instrument, device) = fixture_then(extra);
        let records = format!(
            r#"
record(bo, "BO") {{
    field(DTYP, "{DTYP}")
    field(OUT, "@gm10dev CHAN_TRIG")
}}
"#
        );
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &[("bo", BO)],
                factory(registry_with("gm10dev", instrument)),
            )
            .db_string(&records, &HashMap::new())
            .unwrap()
            .build()
            .await
            .unwrap();
        (db, device)
    }

    /// Put `val` to `BO.field` and let the pass it starts settle.
    async fn put_bo_and_settle(
        db: &epics_rs::base::server::database::PvDatabase,
        field: &str,
        val: EpicsValue,
    ) {
        let _ = db
            .put_record_field_from_ca_no_notify("BO", field, val)
            .await;
        for _ in 0..500 {
            if !db.get_record("BO").unwrap().read().is_processing() {
                return;
            }
            epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(5)).await;
        }
        panic!("the queued write never completed");
    }

    fn bo_udf_and_alarm(db: &epics_rs::base::server::database::PvDatabase) -> [EpicsValue; 3] {
        ["UDF", "STAT", "SEVR"].map(|f| db.get_pv(format!("BO.{f}")).unwrap())
    }

    /// devGM10_bo.c:199: a write's completing pass defines the record, so a
    /// bo a client marked undefined is clean from its next pass.
    #[test]
    fn a_landed_write_clears_udf() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (db, _device) =
                chan_trig_bo(vec![one_record_fdata_binary(), one_record_fdata_binary()]).await;
            put_bo_and_settle(&db, "UDF", EpicsValue::UChar(1)).await;
            put_bo_and_settle(&db, "PROC", EpicsValue::Short(1)).await;
            assert_eq!(
                bo_udf_and_alarm(&db),
                [
                    EpicsValue::UChar(0),
                    EpicsValue::Short(0),
                    EpicsValue::Short(0)
                ]
            );
        });
    }

    /// A write queued on a unit that hung up once it loaded never lands, and
    /// the record stays undefined.
    #[test]
    fn a_write_the_unit_never_got_leaves_udf_set() {
        epics_rs::base::runtime::task::test_block_on(async {
            let (db, device) = chan_trig_bo(Vec::new()).await;
            device.join().unwrap();
            put_bo_and_settle(&db, "UDF", EpicsValue::UChar(1)).await;
            assert_eq!(
                bo_udf_and_alarm(&db),
                [
                    EpicsValue::UChar(1),
                    EpicsValue::Short(17),
                    EpicsValue::Short(3)
                ],
                "UDF, then UDF_ALARM, INVALID"
            );
        });
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
                device
                    .init_common(&mut record, &mut CommonFields::default())
                    .unwrap(),
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

    impl Hold {
        fn wait(&self) {
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
        use epics_rs::base::runtime::task::{
            CallbackPriority, background_arm, background_callback_stats, spawn_blocking_background,
        };
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
        // A cell is never refused, so the worker is held and its one ring
        // slot taken by ad-hoc requests until one is refused.
        let low_overflows = || {
            background_callback_stats(false).unwrap()[CallbackPriority::Low.index()].num_overflow
        };
        let hold = Hold::default();
        let (running_tx, running_rx) = std::sync::mpsc::channel();
        let holder = hold.clone();
        drop(spawn_blocking_background(
            CallbackPriority::Low,
            move |_| {
                running_tx.send(()).unwrap();
                holder.wait();
            },
        ));
        running_rx
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap();
        let refused = low_overflows() + 1;
        let _full = (0..64)
            .find_map(|_| {
                let filler = hold.clone();
                drop(spawn_blocking_background(
                    CallbackPriority::Low,
                    move |_| filler.wait(),
                ));
                (low_overflows() >= refused).then(|| FullBand(hold.clone()))
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
        gated_fixture_then(extra, None)
    }

    /// [`fixture_then`], the reply to the first command after the load held
    /// back until `gate` fires.
    fn fixture_holding_then(
        extra: Vec<Vec<u8>>,
        gate: std::sync::mpsc::Receiver<()>,
    ) -> (Arc<Instrument>, std::thread::JoinHandle<Vec<String>>) {
        gated_fixture_then(extra, Some(gate))
    }

    fn gated_fixture_then(
        extra: Vec<Vec<u8>>,
        gate: Option<std::sync::mpsc::Receiver<()>>,
    ) -> (Arc<Instrument>, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        let gate = gate.map(|gate| (responses.len(), gate));
        responses.extend(extra);
        let device = spawn_gated_fake_device(listener, responses, gate);
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
        static STRINGIN: &[DevSup] = &[DevSup {
            name: "devGM10_stringin",
            choice: DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 3, 4], false),
        }];
        let (db, _) = IocBuilder::new()
            .register_dynamic_device_support(
                &[("ai", AI), ("ao", AO), ("stringin", STRINGIN)],
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

    /// A read on a unit that has not loaded since boot has nothing current
    /// to serve: the pass that starts it ends it, keeps VAL and fails with
    /// COMM_ALARM, INVALID.
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

    /// A unit that has not loaded since boot is offline: a put to an output
    /// on it writes nothing, and the pass that starts it ends it with
    /// COMM_ALARM, INVALID.
    #[test]
    fn a_put_to_a_unit_that_never_loaded_ends_its_pass_with_comm_alarm() {
        let records = format!(
            r#"record(ao, "AO") {{ field(DTYP, "{DTYP}") field(OUT, "@gm10dev VAL:K1") }}"#
        );
        epics_rs::base::runtime::task::test_block_on(async {
            let db = offline_db(&records).await;
            let put = db
                .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                .await;
            assert_eq!(pact_and_alarm(&db, "AO"), COMM_INVALID, "PACT, STAT, SEVR");
            assert!(put.is_err(), "the put fails with the pass");
        });
    }

    /// The configured address is not read from the unit, so a record
    /// serving it works from boot, whether its unit ever loads or not.
    #[test]
    fn the_configured_address_is_served_before_the_unit_loads() {
        let records = format!(
            r#"record(stringin, "IP") {{ field(DTYP, "{DTYP}") field(INP, "@gm10dev IP_ADDR") }}"#
        );
        epics_rs::base::runtime::task::test_block_on(async {
            let db = offline_db(&records).await;
            let at_boot = pact_and_alarm(&db, "IP");
            let put = db
                .put_record_field_from_ca_no_notify("IP", "PROC", EpicsValue::Short(1))
                .await;
            assert_eq!(
                (
                    at_boot,
                    pact_and_alarm(&db, "IP"),
                    db.get_pv("IP.VAL").unwrap()
                ),
                (
                    (false, EpicsValue::Short(17), EpicsValue::Short(3)),
                    NO_ALARM,
                    EpicsValue::String("127.0.0.1".into())
                ),
                "(PACT, STAT, SEVR) at boot and after a read, then VAL"
            );
            assert!(put.is_ok(), "{put:?}");
        });
    }

    /// Waits up to five seconds for `AO` to leave the COMM_ALARM its unit's
    /// absence at boot set.
    async fn until_ao_settles(db: &epics_rs::base::server::database::PvDatabase) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while pact_and_alarm(db, "AO") == COMM_INVALID && std::time::Instant::now() < deadline {
            epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(5)).await;
        }
    }

    /// What `device` received, once it ended within five seconds.
    async fn received_within(device: std::thread::JoinHandle<Vec<String>>) -> Option<Vec<String>> {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while !device.is_finished() && std::time::Instant::now() < deadline {
            epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(5)).await;
        }
        device.is_finished().then(|| device.join().unwrap())
    }

    /// The commands of a present unit's boot load.
    fn boot_load_commands() -> Vec<String> {
        fixture_then(Vec::new()).1.join().unwrap()
    }

    /// Records on a unit absent at boot start in COMM_ALARM, INVALID. Once
    /// the unit loads, `AO` holds what a present unit's boot leaves it,
    /// its starting value included, and both records are served.
    #[test]
    fn records_on_a_unit_absent_at_boot_are_served_once_it_loads() {
        epics_rs::base::runtime::task::test_block_on(async {
            let present = {
                let db = ai_and_ao_on("", connect_default_fixture()).await;
                (pact_and_alarm(&db, "AO"), db.get_pv("AO.VAL").unwrap())
            };
            let mut responses = default_init_responses();
            responses.extend([channel_1_reads_5678(), b"E0\r\n".to_vec()]);
            let (instrument, appear, device) = absent_at_boot(responses);
            let db = ai_and_ao_on("", instrument).await;
            let before = [pact_and_alarm(&db, "AI"), pact_and_alarm(&db, "AO")];
            appear.send(()).unwrap();
            until_ao_settles(&db).await;
            let loaded = (pact_and_alarm(&db, "AO"), db.get_pv("AO.VAL").unwrap());
            let read = proc_ai(&db).await;
            let _ = db
                .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                .await;
            pact_after_settling(&db).await;
            let written = pact_and_alarm(&db, "AO");
            assert_eq!(
                (before, loaded, read, written),
                (
                    [COMM_INVALID, COMM_INVALID],
                    present,
                    (NO_ALARM, EpicsValue::Double(5.678)),
                    NO_ALARM
                ),
                "AI and AO before the load, AO after it, then a read of AI and a write of AO"
            );
            let mut wire = boot_load_commands();
            wire.extend([
                "FData,1,0001,0001\r\n".to_string(),
                "SKConst,1,2.5\r\n".to_string(),
            ]);
            assert_eq!(received_within(device).await, Some(wire));
        });
    }

    /// A channel the unit turns out not to have when it loads late leaves
    /// its record as the boot check on a present unit leaves it: inert.
    #[test]
    fn a_link_the_late_load_rejects_leaves_its_record_inert() {
        use epics_rs::base::server::database::PvDatabase;
        let bad = format!(
            r#"record(ai, "BAD") {{ field(DTYP, "{DTYP}") field(INP, "@gm10dev VAL:7") }}"#
        );
        epics_rs::base::runtime::task::test_block_on(async {
            let proc_bad = |db: Arc<PvDatabase>| async move {
                let _ = db
                    .put_record_field_from_ca_no_notify("BAD", "PROC", EpicsValue::Short(1))
                    .await;
                epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(50)).await;
                (pact_and_alarm(&db, "BAD"), db.get_pv("BAD.VAL").unwrap())
            };
            let present = proc_bad(ai_and_ao_with("", &bad, connect_default_fixture()).await).await;
            let (instrument, appear, _device) = absent_at_boot(default_init_responses());
            let db = ai_and_ao_with("", &bad, instrument).await;
            let before = proc_bad(db.clone()).await;
            appear.send(()).unwrap();
            until_ao_settles(&db).await;
            let after = proc_bad(db.clone()).await;
            assert_eq!(
                [before, after],
                [(COMM_INVALID, EpicsValue::Double(0.0)), present],
                "(PACT, STAT, SEVR) and VAL after a read before the load and after it"
            );
        });
    }

    /// A put made before the unit's first load fails as on any offline unit
    /// and never reaches it; the load then gives `AO` the unit's starting
    /// value, as a present unit's boot does, and the next put is written.
    #[test]
    fn a_put_before_the_first_load_writes_nothing_and_the_load_seeds_val() {
        epics_rs::base::runtime::task::test_block_on(async {
            let starting_val = ai_and_ao_on("", connect_default_fixture())
                .await
                .get_pv("AO.VAL")
                .unwrap();
            let mut responses = default_init_responses();
            responses.push(b"E0\r\n".to_vec());
            let (instrument, appear, device) = absent_at_boot(responses);
            let db = ai_and_ao_on("", instrument.clone()).await;
            let early = db
                .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(2.5))
                .await;
            let failed = pact_and_alarm(&db, "AO");
            appear.send(()).unwrap();
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            while !instrument.online() && std::time::Instant::now() < deadline {
                epics_rs::base::runtime::task::sleep(std::time::Duration::from_millis(5)).await;
            }
            let loaded = (pact_and_alarm(&db, "AO"), db.get_pv("AO.VAL").unwrap());
            let late = db
                .put_record_field_from_ca_no_notify("AO", "VAL", EpicsValue::Double(3.5))
                .await;
            pact_after_settling(&db).await;
            assert_eq!(
                (failed, loaded, pact_and_alarm(&db, "AO")),
                (COMM_INVALID, (COMM_INVALID, starting_val), NO_ALARM),
                "AO after the early put, after the load, after the late put"
            );
            assert!(early.is_err(), "the early put fails with its pass");
            assert!(late.is_ok(), "{late:?}");
            let mut wire = boot_load_commands();
            wire.push("SKConst,1,3.5\r\n".to_string());
            assert_eq!(received_within(device).await, Some(wire));
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

    /// A unit that stops answering ends the read with COMM_ALARM once
    /// READ_TIMEOUT passes, and the reply it sends after that answers no
    /// later read.
    #[test]
    fn a_read_the_unit_never_answers_ends_with_comm_alarm() {
        use crate::instrument::READ_TIMEOUT;
        use std::time::{Duration, Instant};
        epics_rs::base::runtime::task::test_block_on(async {
            let (late, gate) = std::sync::mpsc::channel();
            let (instrument, _device) =
                fixture_holding_then(vec![channel_1_reads_5678(), channel_1_reads_5678()], gate);
            let db = ai_and_ao_on("", instrument).await;
            let mut ends = Vec::new();
            for _ in 0..2 {
                let _ = db
                    .put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
                    .await;
                let deadline = Instant::now() + READ_TIMEOUT + Duration::from_secs(3);
                while pact_and_alarm(&db, "AI").0 && Instant::now() < deadline {
                    epics_rs::base::runtime::task::sleep(Duration::from_millis(5)).await;
                }
                ends.push((pact_and_alarm(&db, "AI"), db.get_pv("AI.VAL").unwrap()));
                let _ = late.send(());
            }
            assert_eq!(
                ends,
                [
                    (COMM_INVALID, EpicsValue::Double(0.0)),
                    (COMM_INVALID, EpicsValue::Double(0.0))
                ],
                "(PACT, STAT, SEVR) and VAL after the read with no reply and the next"
            );
        });
    }

    /// `AI` over a unit that plays `sessions`, one per connection: (PACT,
    /// STAT, SEVR) and VAL after the read right after the boot load, and
    /// after the first later read that does not end with COMM_ALARM, if one
    /// does within five seconds.
    async fn reads_across_a_reconnect(
        sessions: Vec<Vec<Vec<u8>>>,
    ) -> (
        [((bool, EpicsValue, EpicsValue), EpicsValue); 2],
        std::thread::JoinHandle<Vec<Vec<String>>>,
    ) {
        use std::time::{Duration, Instant};
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let device = spawn_fake_device_sessions(listener, sessions);
        let db = ai_and_ao_on("", Instrument::connect_to(addr).unwrap()).await;
        let read = || async {
            let _ = db
                .put_record_field_from_ca_no_notify("AI", "PROC", EpicsValue::Short(1))
                .await;
            pact_after_settling(&db).await;
            (pact_and_alarm(&db, "AI"), db.get_pv("AI.VAL").unwrap())
        };
        let first = read().await;
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut later = read().await;
        while later.0 == COMM_INVALID && Instant::now() < deadline {
            epics_rs::base::runtime::task::sleep(Duration::from_millis(50)).await;
            later = read().await;
        }
        ([first, later], device)
    }

    /// A unit that hangs up after it loaded fails the next read with
    /// COMM_ALARM; once it accepts a connection again it is loaded as at
    /// boot and reads are answered again.
    #[test]
    fn a_unit_that_hangs_up_is_loaded_again_and_read() {
        epics_rs::base::runtime::task::test_block_on(async {
            let mut back = default_init_responses();
            back.push(channel_1_reads_5678());
            let (ends, device) =
                reads_across_a_reconnect(vec![default_init_responses(), back]).await;
            assert_eq!(
                ends,
                [
                    (COMM_INVALID, EpicsValue::Double(0.0)),
                    (NO_ALARM, EpicsValue::Double(5.678))
                ],
                "(PACT, STAT, SEVR) and VAL after the read the hang-up fails and the next answered"
            );
            let received = device.join().unwrap();
            assert_eq!(
                received[1][..received[0].len()],
                received[0],
                "the commands of the load at boot and of the load on the new connection"
            );
        });
    }

    /// A unit that hangs up again in the middle of the load on a new
    /// connection is connected to and loaded once more.
    #[test]
    fn a_load_the_unit_cuts_short_is_tried_again() {
        epics_rs::base::runtime::task::test_block_on(async {
            let mut back = default_init_responses();
            back.push(channel_1_reads_5678());
            let (ends, _device) =
                reads_across_a_reconnect(vec![default_init_responses(), Vec::new(), back]).await;
            assert_eq!(
                ends,
                [
                    (COMM_INVALID, EpicsValue::Double(0.0)),
                    (NO_ALARM, EpicsValue::Double(5.678))
                ],
                "(PACT, STAT, SEVR) and VAL after the read the hang-up fails and the next answered"
            );
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
