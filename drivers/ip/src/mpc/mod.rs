//! `devMPC.c` — the `asyn MPC` device support for the MPC / Digitel ion-pump
//! controller and its titanium sublimation pump.
//!
//! Per-record device support on ai, ao, bi, bo, mbbo, stringin and stringout,
//! as in C: the link is `@asyn(port address)COMMAND supply` on the octet port,
//! and every process queues one flush/write/read exchange whose callback
//! re-enters the record.

pub mod protocol;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{CellBody, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, ReadStart,
    WriteStart,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::recgbl::alarm_status::{READ_ALARM, WRITE_ALARM};
use epics_rs::base::server::record::{AlarmSeverity, Record, ScanType};
use epics_rs::base::types::EpicsValue;

use protocol::{Command, DISPLAY_STRINGS, c_atoi, c_strtod};

/// `device(<type>,INST_IO,dev<Type>MPC,"asyn MPC")` (`ipSupport.dbd`).
pub const DTYP: &str = "asyn MPC";

/// The record types `ipSupport.dbd` declares `asyn MPC` for.
pub const RECORD_TYPES: [&str; 7] = ["ai", "ao", "bi", "bo", "mbbo", "stringin", "stringout"];

/// `MPC_TIMEOUT` (`devMPC.c:133`).
const TIMEOUT: Duration = Duration::from_secs(3);

/// `MPC_BUFFER_SIZE` (`devMPC.c:132`).
const BUFFER_SIZE: usize = 50;

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// `ASYN_EOM_EOS`, the only end C accepts (`devMPC.c:861`).
const EOM_EOS: u32 = 0x02;

/// Dynamic device-support factory for `asyn MPC`. The link is parsed in
/// `init`, where C's `initCommon` parses it.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        let kind = Kind::of(ctx.record_type).filter(|_| ctx.dtyp == DTYP)?;
        let link = if kind.is_output() { ctx.out } else { ctx.inp };
        Some(Box::new(DevMpc::new(kind, link)) as Box<dyn DeviceSupport>)
    }
}

/// C's `recType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Ai,
    Ao,
    Bi,
    Bo,
    Mbbo,
    Si,
    So,
}

impl Kind {
    pub fn of(record_type: &str) -> Option<Self> {
        Some(match record_type {
            "ai" => Self::Ai,
            "ao" => Self::Ao,
            "bi" => Self::Bi,
            "bo" => Self::Bo,
            "mbbo" => Self::Mbbo,
            "stringin" => Self::Si,
            "stringout" => Self::So,
            _ => return None,
        })
    }

    fn is_output(self) -> bool {
        matches!(self, Self::Ao | Self::Bo | Self::Mbbo | Self::So)
    }

    /// The routine name C's error lines carry.
    fn routine(self) -> &'static str {
        match self {
            Self::Ai => "readAi",
            Self::Ao => "writeAo",
            Self::Bi => "readBi",
            Self::Bo => "writeBo",
            Self::Mbbo => "writeMbbo",
            Self::Si => "readSi",
            Self::So => "writeSo",
        }
    }
}

/// `asynPrint` on the record's asynUser, connected to `port` at address 0.
fn print(handle: &PortHandle, mask: TraceMask, args: std::fmt::Arguments<'_>) {
    handle
        .trace()
        .print(Some(handle.port_name()), Some(ADDR), 0, mask, args);
}

/// `recGblSetSevr`: a strictly higher severity replaces the pending one.
fn raise(alarm: &mut Option<(u16, AlarmSeverity)>, stat: u16, sevr: AlarmSeverity) {
    if alarm.is_none_or(|(_, old)| sevr > old) {
        *alarm = Some((stat, sevr));
    }
}

/// What `devMPCCallback` leaves in `devMPCPvt`.
struct Exchange {
    /// `sendBuf`, which a pass that builds no command sends again.
    send: String,
    /// `recBuf`, as bytes: C's offsets and `strlen` count bytes.
    rec_buf: Vec<u8>,
    /// `pPvt->status == asynSuccess`.
    ok: bool,
    /// `prevReadStatus`, as C's asynStatus number.
    prev_read_status: i32,
    /// The callback's `recGblSetSevr`.
    alarm: Option<(u16, AlarmSeverity)>,
}

/// The queued request's body — `devMPCCallback`'s I/O half.
struct Job {
    name: String,
    command: Command,
    handle: PortHandle,
    exchange: Mutex<Exchange>,
    /// The record's completion and band, and whether it ends a write.
    reentry: Mutex<Option<(DeviceDone, CallbackPriority, bool)>>,
}

impl Job {
    fn arm(&self, done: &DeviceDoneSource<'_>, write: bool) {
        let mut slot = self.reentry.lock().unwrap();
        match &mut *slot {
            Some((_, prio, w)) => (*prio, *w) = (done.priority(), write),
            None => *slot = Some((done.device_done(), done.priority(), write)),
        }
    }

    fn trace(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        print(&self.handle, mask, args);
    }

    /// The exchange, from `memset(recBuf)` to `goto done`.
    fn exchange(&self, port: &mut dyn PortCalls, ex: &mut Exchange) {
        let name = &self.name;
        let command = self.command.number();
        ex.rec_buf.clear();
        let user = || AsynUser::new(0).with_addr(ADDR).with_timeout(TIMEOUT);
        let _ = port.call(&RequestOp::Flush, user());
        let write = port.call(
            &RequestOp::OctetWrite {
                data: ex.send.as_bytes().to_vec(),
            },
            user(),
        );
        if let Err(e) = write {
            ex.ok = false;
            self.trace(
                TraceMask::ERROR,
                format_args!(
                    "devMPC::devMPCCallback write error, status={} error= {}\n",
                    e.c_status_code(),
                    e.message()
                ),
            );
            raise(&mut ex.alarm, WRITE_ALARM, AlarmSeverity::Invalid);
            return;
        }
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devMPC::devMPCCallback {name} nwrite={}, output={}\n",
                ex.send.len(),
                ex.send
            ),
        );
        let read = port.call(
            &RequestOp::OctetRead {
                buf_size: BUFFER_SIZE,
            },
            user(),
        );
        let (data, eom) = match read {
            Err(e) => {
                ex.ok = false;
                let status = e.c_status_code();
                if status != ex.prev_read_status {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!(
                            "devMPC::devMPCCallback {name} read error, status={status} error= {}\n",
                            e.message()
                        ),
                    );
                    ex.prev_read_status = status;
                }
                raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
                return;
            }
            Ok(r) => {
                ex.ok = true;
                if ex.prev_read_status != 0 {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!("devMPC::devMPCCallback {name} read status back to normal\n"),
                    );
                    ex.prev_read_status = 0;
                }
                (r.data.unwrap_or_default(), r.eom_reason)
            }
        };
        if eom != EOM_EOS {
            self.trace(
                TraceMask::ERROR,
                format_args!("devMPC::devMPCCallback {name} read error, eomReason={eom}\n"),
            );
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            return;
        }
        let nread = data.len();
        if nread == BUFFER_SIZE {
            self.trace(
                TraceMask::ERROR,
                format_args!("devMPC::devMPCCallback {name} message too large={nread}\n"),
            );
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            return;
        }
        let input = String::from_utf8_lossy(&data);
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!("devMPC::devMPCCallback {name} nread={nread}, input={input}\n"),
        );
        if nread < 4 {
            self.trace(
                TraceMask::ERROR,
                format_args!("devMPC::devMPCCallback {name} message too small={nread}\n"),
            );
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            ex.ok = false;
            return;
        }
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devMPC: {name} command ({command}) received (before processing) len={nread}, |{input}|\n"
            ),
        );
        ex.rec_buf = protocol::strip_reply(&data);
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devMPC: {name} command ({command}) received (after processing) |{}|\n",
                String::from_utf8_lossy(&ex.rec_buf)
            ),
        );
    }
}

impl CellBody for Job {
    fn process(&self, port: &mut dyn PortCalls) {
        let mut ex = self.exchange.lock().unwrap();
        self.exchange(port, &mut ex);
    }

    /// No `timeoutUser`, so only a port shutting down with the request queued
    /// lands here; the pass reads it as a failed exchange.
    fn unrun(&self, _why: Unrun) {
        let mut ex = self.exchange.lock().unwrap();
        ex.ok = false;
        ex.rec_buf.clear();
    }

    /// `dbScanLock(pr); prset->process(pr)` at the end of `devMPCCallback`.
    fn done(self: Arc<Self>) {
        let armed = self.reentry.lock().unwrap().clone();
        if let Some((done, prio, write)) = armed {
            let _ = if write {
                done.complete(prio, Ok(()))
            } else {
                done.reprocess(prio)
            };
        }
    }

    /// `createAsynUser(devMPCCallback, 0)` (`devMPC.c:214`).
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    /// `sprintf(pPvt->address, "%02X", address)`.
    address: String,
    /// `pPvt->parameter`, the supply.
    parameter: i32,
}

pub struct DevMpc {
    kind: Kind,
    link: String,
    /// C `pr->name`.
    name: String,
    live: Option<Live>,
    /// The alarm the last completed pass raised.
    alarm: Option<(u16, AlarmSeverity)>,
}

fn double(record: &dyn Record, field: &str) -> f64 {
    record
        .get_field(field)
        .and_then(|v| v.to_f64())
        .unwrap_or(0.0)
}

fn string(record: &dyn Record, field: &str) -> String {
    match record.get_field(field) {
        Some(EpicsValue::String(s)) => String::from_utf8_lossy(s.as_bytes()).into_owned(),
        _ => String::new(),
    }
}

impl DevMpc {
    pub fn new(kind: Kind, link: &str) -> Self {
        Self {
            kind,
            link: link.to_owned(),
            name: String::new(),
            live: None,
            alarm: None,
        }
    }

    fn wrong_type(&self, live: &Live) {
        print(
            &live.job.handle,
            TraceMask::ERROR,
            format_args!(
                "devMPC::{} {} Wrong record type \n",
                self.kind.routine(),
                self.name
            ),
        );
    }

    /// The `pact == 0` half of `readX` / `writeX`: the `(hexCmd, data)` it
    /// builds, or `None` where C builds nothing and resends `sendBuf`.
    fn command(&self, live: &Live, record: &dyn Record) -> Option<(u8, String)> {
        use Command::*;
        let param = live.parameter;
        let cmd = live.job.command;
        let val = || double(record, "VAL");
        let built = match (self.kind, cmd) {
            (Kind::Ai, GetSpVal12 | GetSpVal34 | GetSpVal56 | GetSpVal78) => {
                let stpt = (cmd.number() - GetSpVal12.number()) / 2 * 2 + param;
                Some((0x3c, stpt.to_string()))
            }
            (Kind::Ai, GetPres) => Some((0x0b, param.to_string())),
            (Kind::Ai, GetCur) => Some((0x0a, param.to_string())),
            (Kind::Ai, GetVolt) => Some((0x0c, param.to_string())),
            (Kind::Ai, GetSize) => Some((0x11, param.to_string())),
            (Kind::Bi, GetSpS12 | GetSpS34 | GetSpS56 | GetSpS78) => {
                let stpt = cmd.number() - GetSpS12.number() + param;
                Some((0x3c, stpt.to_string()))
            }
            (Kind::Bi, GetAutoRestart) => Some((0x34, String::new())),
            (Kind::Si, GetStatus) => Some((0x0d, param.to_string())),
            (Kind::Si, GetTspStat) => Some((0x2a, String::new())),
            // `(int) pr->val`.
            (Kind::Ao, SetSize) => Some((0x12, format!("{param},{}", val() as i32))),
            (Kind::Ao, SetSp12 | SetSp34 | SetSp56 | SetSp78) => {
                let stpt = (cmd.number() - SetSp12.number()) * 2 + param;
                let p = format!("{:>7}", crate::fmt::exp_c(val(), 1));
                Some((0x3d, format!("{stpt},{param},{p},{p}")))
            }
            (Kind::Bo, SetStart | SetStop) => {
                Some((if val() == 0.0 { 0x37 } else { 0x38 }, param.to_string()))
            }
            (Kind::Bo, SetLock | SetUnlock) => {
                Some((if val() == 0.0 { 0x44 } else { 0x45 }, param.to_string()))
            }
            (Kind::Bo, SetAutoRestart) => Some((0x33, yes_no(val()))),
            (Kind::Bo, SetTspAutoAdv) => Some((0x2c, yes_no(val()))),
            (Kind::Bo, SetTspOff) => Some((0x28, String::new())),
            (Kind::Bo, SetTspClear) => Some((0x2b, String::new())),
            (Kind::Bo, SetTspContinuous) => Some((0x2d, String::new())),
            (Kind::Bo, SetTspDegas) => Some((0x2f, String::new())),
            (Kind::Mbbo, SetUnit | SetDis) => {
                // C indexes DisplayStr[] with RVAL unchecked.
                let rval = double(record, "RVAL") as usize;
                let display = DISPLAY_STRINGS.get(rval).copied().unwrap_or("");
                let hex = if cmd == SetUnit { 0x0e } else { 0x25 };
                Some((hex, format!("{param}{display}")))
            }
            (Kind::Mbbo, SetTspFilament) => {
                Some((0x29, (double(record, "RVAL") as u32 as i32).to_string()))
            }
            (Kind::So, SetTspTimed) => Some((0x27, string(record, "VAL"))),
            (Kind::So, SetTspSublimation) => Some((0x2e, string(record, "VAL"))),
            _ => None,
        };
        if built.is_some() {
            return built;
        }
        self.wrong_type(live);
        // readSi and writeBo build nothing on their default arm; the others
        // build hexCmd 0 with the empty data, writeSo with VAL.
        match self.kind {
            Kind::Si | Kind::Bo => None,
            Kind::So => Some((0, string(record, "VAL"))),
            _ => Some((0, String::new())),
        }
    }

    /// `buildCommand` then `startIOCommon`. A refused `queueRequest` returns
    /// -1 with PACT already set, so the record stays active, as in C.
    fn start(&mut self, record: &dyn Record, done: &DeviceDoneSource<'_>) {
        self.alarm = None;
        let Some(live) = self.live.as_ref() else {
            return;
        };
        if let Some((hex, data)) = self.command(live, record) {
            let send = protocol::build_command(&live.address, hex, &data);
            print(
                &live.job.handle,
                TraceMask::IO_DEVICE,
                format_args!(
                    "devMPC::buildCommand {} command 0x{hex:X} len={} string=|{send}|\n",
                    self.name,
                    send.len()
                ),
            );
            live.job.exchange.lock().unwrap().send = send;
        }
        live.job.arm(done, self.kind.is_output());
        let user = AsynUser::new(0)
            .with_addr(ADDR)
            .with_priority(QueuePriority::Low);
        let _ = live.job.handle.queue_request(&live.cell, &user);
    }

    /// The exchange's outcome and `recBuf`, with its alarm taken.
    fn outcome(&mut self) -> (bool, Vec<u8>) {
        let Some(live) = self.live.as_ref() else {
            return (false, Vec::new());
        };
        let mut ex = live.job.exchange.lock().unwrap();
        self.alarm = ex.alarm.take();
        (ex.ok, ex.rec_buf.clone())
    }

    fn error(&self, args: std::fmt::Arguments<'_>) {
        if let Some(live) = &self.live {
            print(&live.job.handle, TraceMask::ERROR, args);
        }
    }

    /// `checkRtnSize`.
    fn short(&self, rec_buf: &[u8], min: usize) -> bool {
        let short = rec_buf.len() < min;
        if short {
            self.error(format_args!(
                "devMPC, {} rtnSize={}, should be >{min}, response={}\n",
                self.name,
                rec_buf.len(),
                String::from_utf8_lossy(rec_buf)
            ));
        }
        short
    }

    /// `readAi` with `pact == 1`: VAL and EGU, both cleared first.
    fn read_ai(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        use Command::*;
        let (ok, buf) = self.outcome();
        record.put_field("VAL", EpicsValue::Double(0.0))?;
        record.put_field("EGU", EpicsValue::String("".into()))?;
        let failed = Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
        if !ok {
            return failed;
        }
        let cmd = self.live.as_ref().map(|l| l.job.command);
        let bytes = buf.as_slice();
        let head = |n: usize| &bytes[..n.min(bytes.len())];
        let (value, egu) = match cmd {
            Some(GetPres) => {
                if self.short(&buf, 9) {
                    return failed;
                }
                (c_strtod(head(7)), text(&bytes[8..]))
            }
            Some(GetCur) => {
                if self.short(&buf, 7) {
                    return failed;
                }
                (c_strtod(head(7)), "AMPS".to_owned())
            }
            Some(GetVolt) => {
                if self.short(&buf, 1) {
                    return failed;
                }
                (c_strtod(bytes), "VOLTS".to_owned())
            }
            Some(GetSize) => {
                let Some(l) = bytes.iter().position(|&b| b == b'L') else {
                    self.error(format_args!(
                        "devMPC::readAi, {} cannot find L, response={}\n",
                        self.name,
                        text(bytes)
                    ));
                    return failed;
                };
                (c_strtod(&bytes[..l]), "L/S".to_owned())
            }
            Some(GetSpVal12 | GetSpVal34 | GetSpVal56 | GetSpVal78) => {
                if self.short(&buf, 11) {
                    return failed;
                }
                (c_strtod(&bytes[4..11]), "TORR".to_owned())
            }
            _ => {
                if let Some(live) = &self.live {
                    self.wrong_type(live);
                }
                (0.0, String::new())
            }
        };
        record.put_field("VAL", EpicsValue::Double(value))?;
        record.put_field("EGU", EpicsValue::String(egu.as_str().into()))?;
        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }

    /// `readBi` with `pact == 1`: RVAL, cleared first.
    fn read_bi(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        use Command::*;
        let (ok, buf) = self.outcome();
        record.put_field("RVAL", EpicsValue::ULong(0))?;
        if !ok || self.short(&buf, 2) {
            return Ok(DeviceReadOutcome::ok());
        }
        let value = match self.live.as_ref().map(|l| l.job.command) {
            // `sscanf(&recBuf[rtnSize-1], "%d", &value)`.
            Some(GetSpS12 | GetSpS34 | GetSpS56 | GetSpS78) => c_atoi(&buf[buf.len() - 1..]),
            Some(GetAutoRestart) => i32::from(buf == b"YES"),
            _ => {
                if let Some(live) = &self.live {
                    self.wrong_type(live);
                }
                0
            }
        };
        record.put_field("RVAL", EpicsValue::ULong(value as u32))?;
        Ok(DeviceReadOutcome::converted(DeviceUdf::Defined))
    }

    /// `readSi` with `pact == 1`: VAL, cleared first.
    fn read_si(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let (ok, buf) = self.outcome();
        record.put_field("VAL", EpicsValue::String("".into()))?;
        if !ok {
            return Ok(DeviceReadOutcome::ok());
        }
        if buf.len() > 39 {
            raise(&mut self.alarm, READ_ALARM, AlarmSeverity::Invalid);
            self.error(format_args!(
                "devMPC::readSi message too big in {} \n",
                self.name
            ));
            return Ok(DeviceReadOutcome::ok());
        }
        record.put_field("VAL", EpicsValue::String(text(&buf).as_str().into()))?;
        Ok(DeviceReadOutcome::converted(DeviceUdf::Defined))
    }
}

/// `recBuf` text for a record field or a trace line.
fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

fn yes_no(val: f64) -> String {
    if val != 0.0 { "YES" } else { "NO" }.to_owned()
}

impl DeviceSupport for DevMpc {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn holds_dpvt(&self) -> bool {
        self.live.is_some()
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    /// C `initCommon`. Every refusal is its `bad:` arm, `pr->pact = 1`.
    fn init(&mut self, _record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        let name = self.name.clone();
        let link = match parse_asyn_link(&self.link) {
            Ok(link) => link,
            Err(e) => {
                let message = match e {
                    AsynError::InvalidLinkSyntax(m) => m,
                    other => other.to_string(),
                };
                errlog_vprintf(format_args!(
                    "devXxMPC::initCommon {name} bad link {message}\n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };
        let Ok(handle) = crate::connect_octet(&link.port_name, 0) else {
            return Ok(DeviceInitOutcome::dead());
        };
        let user_param = link.drv_info.trim();
        if user_param.is_empty() {
            errlog_vprintf(format_args!(
                "devMPC::initCommon {name} invalid userParam (null)\n"
            ));
            return Ok(DeviceInitOutcome::dead());
        }
        let (word, parameter) = protocol::split_user_param(user_param);
        let address = format!("{:02X}", link.addr);
        let Some(command) = Command::from_name(word) else {
            print(
                &handle,
                TraceMask::ERROR,
                format_args!("devMPC::init_common {name}, unknown command={word}\n"),
            );
            return Ok(DeviceInitOutcome::dead());
        };
        print(
            &handle,
            TraceMask::FLOW,
            format_args!(
                "devMPC::initCommon name={name}; command string={word} command={}, \
                 address={address}; parameter={parameter};\n",
                command.number()
            ),
        );
        let job = Arc::new(Job {
            name,
            command,
            handle: handle.clone(),
            exchange: Mutex::new(Exchange {
                send: String::new(),
                rec_buf: Vec::new(),
                ok: true,
                prev_read_status: 0,
                alarm: None,
            }),
            reentry: Mutex::new(None),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live {
            job,
            cell,
            address,
            parameter,
        });
        Ok(DeviceInitOutcome::ok())
    }

    fn read_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        if done.pact() || self.live.is_none() {
            return Ok(ReadStart::Synchronous);
        }
        self.start(record, done);
        Ok(ReadStart::Pending)
    }

    /// C `MPCConvert`, the slot of both devAiMPC and devAoMPC.
    fn has_special_linconv(&self) -> bool {
        matches!(self.kind, Kind::Ai | Kind::Ao)
    }

    /// `MPCConvert` writes through an `aiRecord *` cast (`devMPC.c:914`), which on
    /// an ao lands on LOPR and the low half of ASLO; that C defect is not
    /// copied, so an ao keeps only the record's EOFF rebase.
    fn special_linconv(&mut self, record: &mut dyn Record, _after: bool) -> CaResult<()> {
        if self.kind == Kind::Ai {
            crate::unit_slope(record)?;
        }
        Ok(())
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        match self.kind {
            Kind::Ai => self.read_ai(record),
            Kind::Bi => self.read_bi(record),
            Kind::Si => self.read_si(record),
            _ => Ok(DeviceReadOutcome::ok()),
        }
    }

    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        if self.live.is_none() {
            return Ok(WriteStart::Completed);
        }
        self.start(record, done);
        Ok(WriteStart::Pending)
    }

    /// `writeX` with `pact == 1`: a reply longer than `OK` alarms.
    fn async_write_completed(&mut self) {
        let (ok, buf) = self.outcome();
        if ok && buf.len() > 2 {
            raise(&mut self.alarm, READ_ALARM, AlarmSeverity::Invalid);
            self.error(format_args!(
                "devMPC::{} message too big in {}\n",
                self.kind.routine(),
                self.name
            ));
        }
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        self.alarm.map(|(stat, sevr)| DeviceAlarm {
            stat,
            sevr: sevr as u16,
            msg: "",
        })
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::base::server::records::ai::AiRecord;
    use epics_rs::base::server::records::bi::BiRecord;
    use epics_rs::base::server::records::stringin::StringinRecord;

    use super::*;

    fn live_dev(kind: Kind, command: Command, port: &str) -> (DevMpc, PortHandle) {
        use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
        use epics_rs::asyn::manager::{PortManager, claim_port_driver};
        use epics_rs::asyn::port::PortDriver;
        use epics_rs::asyn::registry::open_owner;

        let mut driver = DrvAsynIPPort::new(port, "127.0.0.1:1").unwrap();
        driver.base_mut().init_auto_connect(false);
        let runtime = PortManager::new()
            .register_port(claim_port_driver(port).unwrap(), driver, &open_owner())
            .unwrap();
        let handle = runtime.port_handle().clone();
        std::mem::forget(runtime);
        let job = Arc::new(Job {
            name: "R:x".into(),
            command,
            handle: handle.clone(),
            exchange: Mutex::new(Exchange {
                send: String::new(),
                rec_buf: Vec::new(),
                ok: true,
                prev_read_status: 0,
                alarm: None,
            }),
            reentry: Mutex::new(None),
        });
        let mut dev = DevMpc::new(kind, "");
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
            address: "05".into(),
            parameter: 1,
        });
        (dev, handle)
    }

    /// A port without `asynOctet` fails `findInterface`: `bad:`, `pact = 1`
    /// (devMPC.c:228-229).
    #[test]
    fn a_port_without_octet_kills_the_record() {
        crate::no_octet_port("MPC_INT32_ONLY");
        let mut dev = DevMpc::new(Kind::Ai, "@asyn(MPC_INT32_ONLY 5)GET_PRESSURE 1");
        dev.set_record_info("R:x", ScanType::Passive);
        let outcome = dev.init(&mut AiRecord::default()).unwrap();
        assert_eq!(outcome, DeviceInitOutcome::dead());
        assert!(!dev.holds_dpvt());
    }

    fn landed(dev: &DevMpc, ok: bool, rec_buf: impl AsRef<[u8]>) {
        let mut ex = dev.live.as_ref().unwrap().job.exchange.lock().unwrap();
        (ex.ok, ex.rec_buf) = (ok, rec_buf.as_ref().to_vec());
    }

    fn ai(dev: &mut DevMpc, ok: bool, rec_buf: impl AsRef<[u8]>) -> (f64, String) {
        landed(dev, ok, rec_buf);
        let mut rec = AiRecord::default();
        rec.val = 5.0;
        dev.read(&mut rec).unwrap();
        (rec.val, string(&rec, "EGU"))
    }

    /// `readAi`'s decode, one case per C arm; a failure leaves VAL 0, EGU "".
    #[test]
    fn ai_replies_decode_as_read_ai_does() {
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetPres, "MPC_AI_PRES");
        assert_eq!(ai(&mut dev, true, "1.2E-09 MBAR"), (1.2e-9, "MBAR".into()));
        assert_eq!(ai(&mut dev, true, "1.2E-09"), (0.0, "".into()));
        assert_eq!(ai(&mut dev, false, "1.2E-09 MBAR"), (0.0, "".into()));
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetSize, "MPC_AI_SIZE");
        assert_eq!(ai(&mut dev, true, "30L/S"), (30.0, "L/S".into()));
        assert_eq!(ai(&mut dev, true, "30"), (0.0, "".into()));
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetSpVal34, "MPC_AI_SP");
        assert_eq!(
            ai(&mut dev, true, "3,1,1.0E-06,1.0E-06"),
            (1e-6, "TORR".into())
        );
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetCur, "MPC_AI_CUR");
        assert_eq!(ai(&mut dev, true, "2.0E-06"), (2e-6, "AMPS".into()));
    }

    /// `recBuf` is bytes: C's offsets and `strlen` count bytes, so a reply
    /// that is not ASCII neither panics at `&recBuf[8]` nor changes length.
    #[test]
    fn a_non_ascii_reply_slices_by_byte() {
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetPres, "MPC_AI_BYTES");
        assert_eq!(
            ai(&mut dev, true, "1.2E-09é TORR"),
            (1.2e-9, "\u{fffd} TORR".into())
        );
        let (mut dev, _h) = live_dev(Kind::Si, Command::GetStatus, "MPC_SI_BYTES");
        let mut rec = StringinRecord::default();
        landed(&dev, true, [0xff; 20]);
        dev.read(&mut rec).unwrap();
        assert!(dev.last_alarm().is_none(), "20 bytes is not > 39");
    }

    #[test]
    fn bi_and_stringin_replies_decode_as_c_does() {
        let (mut dev, _h) = live_dev(Kind::Bi, Command::GetSpS12, "MPC_BI");
        let mut rec = BiRecord::default();
        landed(&dev, true, "1,1,1.0E-06,1.0E-06,1");
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.get_field("RVAL"), Some(EpicsValue::ULong(1)));
        landed(&dev, true, "1");
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.get_field("RVAL"), Some(EpicsValue::ULong(0)));

        let (mut dev, _h) = live_dev(Kind::Si, Command::GetStatus, "MPC_SI");
        let mut rec = StringinRecord::default();
        landed(&dev, true, "RUNNING");
        dev.read(&mut rec).unwrap();
        assert_eq!(string(&rec, "VAL"), "RUNNING");
        assert!(dev.last_alarm().is_none());
        landed(&dev, true, "x".repeat(40));
        dev.read(&mut rec).unwrap();
        assert_eq!(string(&rec, "VAL"), "");
        assert_eq!(dev.last_alarm().map(|a| a.stat), Some(READ_ALARM));
    }

    /// The `pact == 0` commands, built as `buildCommand` does.
    #[test]
    fn commands_build_as_dev_mpc_does() {
        use epics_rs::base::server::records::ao::AoRecord;
        let (dev, _h) = live_dev(Kind::Ao, Command::SetSp34, "MPC_AO");
        let mut rec = AoRecord::default();
        rec.val = 1e-6;
        let live = dev.live.as_ref().unwrap();
        assert_eq!(
            dev.command(live, &rec),
            Some((0x3d, "3,1,1.0E-06,1.0E-06".into()))
        );
        let (dev, _h) = live_dev(Kind::Ai, Command::GetSpVal56, "MPC_AI_CMD");
        let live = dev.live.as_ref().unwrap();
        assert_eq!(
            dev.command(live, &AiRecord::default()),
            Some((0x3c, "5".into()))
        );
        let (dev, _h) = live_dev(Kind::Bi, Command::GetSpS78, "MPC_BI_CMD");
        let live = dev.live.as_ref().unwrap();
        assert_eq!(
            dev.command(live, &BiRecord::default()),
            Some((0x3c, "7".into()))
        );
    }

    #[test]
    fn the_factory_serves_asyn_mpc_on_its_seven_record_types() {
        let factory = device_support_factory();
        for rtyp in RECORD_TYPES {
            let ctx = DeviceSupportContext {
                record_type: rtyp,
                dtyp: DTYP,
                inp: "@asyn(P 5)GET_STATUS 1",
                out: "@asyn(P 5)SET_STOP 1",
            };
            assert!(factory(&ctx).is_some(), "{rtyp}");
        }
        let ctx = DeviceSupportContext {
            record_type: "mbbi",
            dtyp: DTYP,
            inp: "",
            out: "",
        };
        assert!(factory(&ctx).is_none());
    }
}
