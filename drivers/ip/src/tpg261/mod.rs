//! `devTPG261.c` — the `asyn TPG261` device support for the Pfeiffer TPG261 /
//! TPG262 gauge controller.
//!
//! Per-record device support on ai, bi, mbbi, stringin, ao, bo and mbbo, as in
//! C: the link is `@asyn(port gauge)COMMAND` on the octet port, and every
//! process queues one command/`<ENQ>` exchange whose callback re-enters the
//! record.

pub mod protocol;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::asyn_record::get_port;
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
use epics_rs::base::server::recgbl::alarm_status::READ_ALARM;
use epics_rs::base::server::record::{AlarmSeverity, Record, ScanType};
use epics_rs::base::types::EpicsValue;

use protocol::{Command, ENQ, NAK, Scan};

/// `device(<type>,INST_IO,dev<Type>TPG261,"asyn TPG261")` (`ipSupport.dbd`).
pub const DTYP: &str = "asyn TPG261";

/// The record types `ipSupport.dbd` declares `asyn TPG261` for.
pub const RECORD_TYPES: [&str; 7] = ["ai", "bi", "mbbi", "stringin", "ao", "bo", "mbbo"];

/// `TPG261_TIMEOUT` (`devTPG261.c:87`).
const TIMEOUT: Duration = Duration::from_secs(2);

/// `TPG261_BUFFER_SIZE` (`devTPG261.c:86`).
const BUFFER_SIZE: usize = 64;

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// Dynamic device-support factory for `asyn TPG261`. The link is parsed in
/// `init`, where C's `initCommon` parses it.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        let kind = Kind::of(ctx.record_type).filter(|_| ctx.dtyp == DTYP)?;
        let link = if kind.is_output() { ctx.out } else { ctx.inp };
        Some(Box::new(DevTpg261::new(kind, link)) as Box<dyn DeviceSupport>)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Ai,
    Bi,
    Mbbi,
    Si,
    Ao,
    Bo,
    Mbbo,
}

impl Kind {
    pub fn of(record_type: &str) -> Option<Self> {
        Some(match record_type {
            "ai" => Self::Ai,
            "bi" => Self::Bi,
            "mbbi" => Self::Mbbi,
            "stringin" => Self::Si,
            "ao" => Self::Ao,
            "bo" => Self::Bo,
            "mbbo" => Self::Mbbo,
            _ => return None,
        })
    }

    fn is_output(self) -> bool {
        matches!(self, Self::Ao | Self::Bo | Self::Mbbo)
    }

    /// The routine name C's error lines carry.
    fn routine(self) -> &'static str {
        match self {
            Self::Ai => "readAi",
            Self::Bi => "readBi",
            Self::Mbbi => "readMbbi",
            Self::Si => "readSi",
            Self::Ao => "writeAo",
            Self::Bo => "writeBo",
            Self::Mbbo => "writeMbbo",
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

/// What `devTPG261Callback` leaves in `devTPG261Pvt`.
struct Exchange {
    /// `sendBuf`.
    send: String,
    /// `recBuf`: only a complete exchange replaces it, so a failed one leaves
    /// the last reply for the pass to decode again.
    rec_buf: String,
    /// The callback's `recGblSetSevr`.
    alarm: Option<(u16, AlarmSeverity)>,
}

/// The queued request's body — `devTPG261Callback`'s I/O half.
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

    /// One write then read, as C's two halves do: the write's status is not
    /// looked at, and an empty or failed read is `nread < 1`.
    fn round_trip(&self, port: &mut dyn PortCalls, data: &[u8], tag: &str) -> Option<Vec<u8>> {
        let name = &self.name;
        let user = || AsynUser::new(0).with_addr(ADDR).with_timeout(TIMEOUT);
        let nwrite = match port.call(
            &RequestOp::OctetWrite {
                data: data.to_vec(),
            },
            user(),
        ) {
            Ok(_) => data.len(),
            Err(_) => 0,
        };
        let output = String::from_utf8_lossy(data);
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTPG261::devTPG261Callback {tag} {name} nwrite={nwrite}, output=[{output}]\n"
            ),
        );
        let read = port.call(
            &RequestOp::OctetRead {
                buf_size: BUFFER_SIZE,
            },
            user(),
        );
        let input = read.ok().and_then(|r| r.data).unwrap_or_default();
        let nread = input.len();
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTPG261::devTPG261Callback {tag} {name} nread={nread}, input=[{}]\n",
                String::from_utf8_lossy(&input)
            ),
        );
        if nread < 1 {
            self.trace(
                TraceMask::ERROR,
                format_args!(
                    "devTPG261::devTPG261Callback {tag} {name} message too small={nread}\n"
                ),
            );
            return None;
        }
        Some(input)
    }

    /// The exchange, up to `finish:`.
    fn exchange(&self, port: &mut dyn PortCalls, ex: &mut Exchange) {
        // C overwrites this status with the write's, unread.
        let _ = port.call(
            &RequestOp::SetInputEos {
                eos: b"\r\n".to_vec(),
            },
            AsynUser::new(0).with_addr(ADDR),
        );
        let Some(reply) = self.round_trip(port, ex.send.as_bytes(), "Cmd") else {
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            return;
        };
        if reply == [NAK] {
            self.trace(
                TraceMask::ERROR,
                format_args!(
                    "devTPG261::devTPG261Callback Cmd {} message Error \n",
                    self.name
                ),
            );
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            return;
        }
        let Some(data) = self.round_trip(port, &[ENQ], "Val") else {
            raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
            return;
        };
        ex.rec_buf = String::from_utf8_lossy(&data).into_owned();
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTPG261: {} command ({}) received (after processing) |{}|\n",
                self.name,
                self.command.number(),
                ex.rec_buf
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
    /// lands here; the pass decodes the last reply, as after a failed read.
    fn unrun(&self, _why: Unrun) {
        let mut ex = self.exchange.lock().unwrap();
        raise(&mut ex.alarm, READ_ALARM, AlarmSeverity::Invalid);
    }

    /// `dbScanLock(pr); prset->process(pr)` at `finish:`.
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

    /// `createAsynUser(devTPG261Callback, 0)` (`devTPG261.c:175`).
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    /// `pPvt->address`, the gauge.
    address: i32,
}

pub struct DevTpg261 {
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

/// `vals[index]` of a `sscanf` into `int vals[]`. C reads outside the array
/// for a gauge address other than 1 or 2, and leaves unscanned slots
/// unset; both read 0 here.
fn slot(vals: &[Option<i32>], index: i32) -> i32 {
    usize::try_from(index)
        .ok()
        .and_then(|i| vals.get(i).copied().flatten())
        .unwrap_or(0)
}

impl DevTpg261 {
    pub fn new(kind: Kind, link: &str) -> Self {
        Self {
            kind,
            link: link.to_owned(),
            name: String::new(),
            live: None,
            alarm: None,
        }
    }

    fn error(&self, args: std::fmt::Arguments<'_>) {
        if let Some(live) = &self.live {
            print(&live.job.handle, TraceMask::ERROR, args);
        }
    }

    fn wrong_type(&self) {
        self.error(format_args!(
            "devTPG261::{} {} Wrong record type \n",
            self.kind.routine(),
            self.name
        ));
    }

    /// The `pact == 0` half of `readX` / `writeX`: the command it builds.
    /// C's default arm sends an unset buffer; it sends nothing but `<CR>` here.
    fn command(&self, live: &Live, record: &dyn Record) -> String {
        use Command::*;
        let a = live.address;
        let setpoint = |stpt: i32| {
            let p = format!("{:>8}", crate::fmt::exp_c(double(record, "VAL"), 4));
            format!("SP{stpt},{},{p},{p}", a - 1)
        };
        let built = match (self.kind, live.job.command) {
            (Kind::Ai, GetPressure) => format!("PR{a}"),
            (Kind::Ai, GetSpVal1) => format!("SP{}", 2 * a - 1),
            (Kind::Ai, GetSpVal2) => format!("SP{}", 2 * a),
            (Kind::Bi, GetSpS1 | GetSpS2) => "SPS".to_owned(),
            (Kind::Bi, GetSensor) => "SEN".to_owned(),
            (Kind::Mbbi, GetUnit) => "UNI".to_owned(),
            (Kind::Si, GetId) => "TID".to_owned(),
            (Kind::Ao, SetSp1) => setpoint(2 * a - 1),
            (Kind::Ao, SetSp2) => setpoint(2 * a),
            (Kind::Bo, SetSensor) => {
                let on = double(record, "VAL") as i32 + 1;
                if a == 1 {
                    format!("SEN,{on},0")
                } else {
                    format!("SEN,0,{on}")
                }
            }
            (Kind::Mbbo, SetUnit) => format!("UNI,{}", double(record, "RVAL") as u32 as i32),
            _ => {
                self.wrong_type();
                String::new()
            }
        };
        built + "\r"
    }

    /// `buildCommand` then `startIOCommon`. A refused `queueRequest` returns
    /// -1 with PACT already set, so the record stays active, as in C.
    fn start(&mut self, record: &dyn Record, done: &DeviceDoneSource<'_>) {
        self.alarm = None;
        let Some(live) = self.live.as_ref() else {
            return;
        };
        let send = self.command(live, record);
        print(
            &live.job.handle,
            TraceMask::IO_DEVICE,
            format_args!(
                "devTPG261::buildCommand {} len={} string=|{send}|\n",
                self.name,
                send.len()
            ),
        );
        live.job.exchange.lock().unwrap().send = send;
        live.job.arm(done, self.kind.is_output());
        let user = AsynUser::new(0)
            .with_addr(ADDR)
            .with_priority(QueuePriority::Low);
        let _ = live.job.handle.queue_request(&live.cell, &user);
    }

    /// `recBuf`, with the callback's alarm taken.
    fn rec_buf(&mut self) -> String {
        let Some(live) = self.live.as_ref() else {
            return String::new();
        };
        let mut ex = live.job.exchange.lock().unwrap();
        self.alarm = ex.alarm.take();
        ex.rec_buf.clone()
    }

    fn command_of(&self) -> Option<Command> {
        self.live.as_ref().map(|l| l.job.command)
    }

    fn address(&self) -> i32 {
        self.live.as_ref().map_or(0, |l| l.address)
    }

    /// `readAi` with `pact == 1`; returns 2, so VAL is the float it scanned.
    fn read_ai(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let buf = self.rec_buf();
        let mut scan = Scan::new(&buf);
        let value = match self.command_of() {
            Some(Command::GetPressure) => {
                let status = scan.int().unwrap_or(0);
                let value = scan.comma().float().unwrap_or(0.0);
                if status == 4 {
                    raise(&mut self.alarm, READ_ALARM, AlarmSeverity::Minor);
                }
                if status != 0 && status != 4 {
                    raise(&mut self.alarm, READ_ALARM, AlarmSeverity::Invalid);
                    self.error(format_args!(
                        "devTPG261::readAi Gauge error {} status =[{status}] \n",
                        self.name
                    ));
                }
                value
            }
            Some(Command::GetSpVal1 | Command::GetSpVal2) => {
                scan.int();
                scan.comma().float().unwrap_or(0.0)
            }
            _ => {
                self.wrong_type();
                0.0
            }
        };
        record.put_field("VAL", EpicsValue::Double(f64::from(value)))?;
        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }

    /// `readBi` / `readMbbi` with `pact == 1`: RVAL.
    fn read_rval(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        use Command::*;
        let buf = self.rec_buf();
        let a = self.address();
        let ints = |n: usize| {
            let mut scan = Scan::new(&buf);
            let mut vals = vec![scan.int()];
            for _ in 1..n {
                vals.push(scan.comma().int());
            }
            vals
        };
        let value = match (self.kind, self.command_of()) {
            (Kind::Bi, Some(GetSpS1)) => slot(&ints(4), 2 * a - 2),
            (Kind::Bi, Some(GetSpS2)) => slot(&ints(4), 2 * a - 1),
            (Kind::Bi, Some(GetSensor)) => slot(&ints(2), a - 1),
            (Kind::Mbbi, Some(GetUnit)) => slot(&ints(1), 0),
            _ => {
                self.wrong_type();
                0
            }
        };
        record.put_field("RVAL", EpicsValue::ULong(value as u32))?;
        Ok(DeviceReadOutcome::converted(DeviceUdf::Defined))
    }

    /// `readSi` with `pact == 1`: gauge 1's half of `recBuf` before the comma,
    /// any other gauge's after it. C dereferences a NULL `strchr` on a reply
    /// without a comma; the whole reply, or nothing, is taken here.
    fn read_si(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let buf = self.rec_buf();
        let value = match buf.split_once(',') {
            Some((first, _)) if self.address() == 1 => first,
            Some((_, second)) => second,
            None if self.address() == 1 => buf.as_str(),
            None => "",
        };
        record.put_field("VAL", EpicsValue::String(value.into()))?;
        Ok(DeviceReadOutcome::converted(DeviceUdf::Defined))
    }
}

impl DeviceSupport for DevTpg261 {
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
                    "devTPG261::initCommon {name} bad link {message}\n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };
        let Some(port) = get_port(&link.port_name) else {
            return Ok(DeviceInitOutcome::dead());
        };
        let handle = port.handle.clone();
        let user_param = link.drv_info.trim();
        if user_param.is_empty() {
            errlog_vprintf(format_args!(
                "devTPG261::initCommon {name} invalid userParam (null)\n"
            ));
            return Ok(DeviceInitOutcome::dead());
        }
        let word = user_param.split_ascii_whitespace().next().unwrap_or("");
        let Some(command) = Command::from_name(word) else {
            print(
                &handle,
                TraceMask::ERROR,
                format_args!("devTPG261::init_common {name}, unknown command={word}\n"),
            );
            return Ok(DeviceInitOutcome::dead());
        };
        let address = link.addr;
        print(
            &handle,
            TraceMask::FLOW,
            format_args!(
                "devTPG261::initCommon name={name}; command string={word} command={} \
                 address={address}\n",
                command.number()
            ),
        );
        let job = Arc::new(Job {
            name,
            command,
            handle: handle.clone(),
            exchange: Mutex::new(Exchange {
                send: String::new(),
                rec_buf: String::new(),
                alarm: None,
            }),
            reentry: Mutex::new(None),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live { job, cell, address });
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

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        match self.kind {
            Kind::Ai => self.read_ai(record),
            Kind::Bi | Kind::Mbbi => self.read_rval(record),
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

    /// `writeX` with `pact == 1`: an empty `recBuf` alarms.
    fn async_write_completed(&mut self) {
        if self.rec_buf().is_empty() {
            raise(&mut self.alarm, READ_ALARM, AlarmSeverity::Invalid);
            self.error(format_args!(
                "devTPG261::{} reply message error {}\n",
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
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::request::RequestResult;
    use epics_rs::base::server::records::ai::AiRecord;
    use epics_rs::base::server::records::ao::AoRecord;
    use epics_rs::base::server::records::bi::BiRecord;
    use epics_rs::base::server::records::bo::BoRecord;
    use epics_rs::base::server::records::stringin::StringinRecord;

    use super::*;

    fn live_dev(kind: Kind, command: Command, address: i32, port: &str) -> (DevTpg261, PortHandle) {
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
                rec_buf: String::new(),
                alarm: None,
            }),
            reentry: Mutex::new(None),
        });
        let mut dev = DevTpg261::new(kind, "");
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
            address,
        });
        (dev, handle)
    }

    fn landed(dev: &DevTpg261, rec_buf: &str) {
        dev.live
            .as_ref()
            .unwrap()
            .job
            .exchange
            .lock()
            .unwrap()
            .rec_buf = rec_buf.to_owned();
    }

    fn command(dev: &DevTpg261, record: &dyn Record) -> String {
        dev.command(dev.live.as_ref().unwrap(), record)
    }

    /// `readAi`: the pressure's float and its status alarm; a setpoint's
    /// lower bound.
    #[test]
    fn ai_replies_decode_as_read_ai_does() {
        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetPressure, 1, "TPG_AI_PRES");
        let mut rec = AiRecord::default();
        landed(&dev, "0,1.2300E-06");
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.val, f64::from(1.23e-6f32));
        assert!(dev.last_alarm().is_none());
        landed(&dev, "4,0.0000E+00");
        dev.read(&mut rec).unwrap();
        assert_eq!(
            dev.last_alarm().map(|a| a.sevr),
            Some(AlarmSeverity::Minor as u16)
        );
        landed(&dev, "5,2.0000E-03");
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.val, f64::from(2e-3f32));
        assert_eq!(
            dev.last_alarm().map(|a| a.sevr),
            Some(AlarmSeverity::Invalid as u16)
        );

        let (mut dev, _h) = live_dev(Kind::Ai, Command::GetSpVal2, 2, "TPG_AI_SP");
        landed(&dev, "1,5.0000E-06,7.0000E-06");
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.val, f64::from(5e-6f32));
    }

    /// `readBi`: SPS and SEN pick the gauge's slot; an address outside 1..2
    /// reads 0.
    #[test]
    fn bi_replies_pick_the_gauge_slot() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static PORTS: AtomicUsize = AtomicUsize::new(0);
        let rval = |kind, command, address, reply: &str| {
            let (mut dev, _h) = live_dev(
                kind,
                command,
                address,
                &format!("TPG_BI_{}", PORTS.fetch_add(1, Ordering::SeqCst)),
            );
            let mut rec = BiRecord::default();
            landed(&dev, reply);
            dev.read(&mut rec).unwrap();
            rec.get_field("RVAL")
        };
        assert_eq!(
            rval(Kind::Bi, Command::GetSpS2, 2, "0,0,0,1"),
            Some(EpicsValue::ULong(1))
        );
        assert_eq!(
            rval(Kind::Bi, Command::GetSpS1, 2, "0,0,1,0"),
            Some(EpicsValue::ULong(1))
        );
        assert_eq!(
            rval(Kind::Bi, Command::GetSensor, 1, "2,0"),
            Some(EpicsValue::ULong(2))
        );
        assert_eq!(
            rval(Kind::Bi, Command::GetSensor, 0, "2,1"),
            Some(EpicsValue::ULong(0))
        );
    }

    /// `readSi`: gauge 1 reads the part before the comma, gauge 2 after.
    #[test]
    fn stringin_reads_its_gauge_s_half_of_tid() {
        for (address, want) in [(1, "PKR"), (2, "IKR")] {
            let (mut dev, _h) = live_dev(
                Kind::Si,
                Command::GetId,
                address,
                &format!("TPG_SI_{address}"),
            );
            let mut rec = StringinRecord::default();
            landed(&dev, "PKR,IKR");
            dev.read(&mut rec).unwrap();
            assert_eq!(rec.get_field("VAL"), Some(EpicsValue::String(want.into())));
        }
    }

    /// The `pact == 0` commands, as `readX` / `writeX` build them.
    #[test]
    fn commands_build_as_dev_tpg261_does() {
        let (dev, _h) = live_dev(Kind::Ai, Command::GetSpVal1, 2, "TPG_CMD_SP");
        assert_eq!(command(&dev, &AiRecord::default()), "SP3\r");
        let (dev, _h) = live_dev(Kind::Ai, Command::GetPressure, 0, "TPG_CMD_PR");
        assert_eq!(command(&dev, &AiRecord::default()), "PR0\r");
        let (dev, _h) = live_dev(Kind::Ao, Command::SetSp2, 1, "TPG_CMD_SET");
        let mut ao = AoRecord::default();
        ao.val = 1e-6;
        assert_eq!(command(&dev, &ao), "SP2,0,1.0000E-06,1.0000E-06\r");
        let (dev, _h) = live_dev(Kind::Bo, Command::SetSensor, 2, "TPG_CMD_SEN");
        let mut bo = BoRecord::default();
        bo.put_field("VAL", EpicsValue::Enum(1)).unwrap();
        assert_eq!(command(&dev, &bo), "SEN,0,2\r");
    }

    /// A port that logs each op and answers reads `<ACK>` then the data.
    struct Wire(Vec<String>);

    impl PortCalls for Wire {
        fn call(&mut self, op: &RequestOp, _user: AsynUser) -> AsynResult<RequestResult> {
            let reads = self.0.iter().filter(|s| *s == "read").count();
            Ok(match op {
                RequestOp::SetInputEos { eos } => {
                    self.0.push(format!("ieos {eos:?}"));
                    RequestResult::write_ok()
                }
                RequestOp::OctetWrite { data } => {
                    self.0.push(format!("write {data:?}"));
                    RequestResult::write_ok()
                }
                RequestOp::OctetRead { .. } => {
                    self.0.push("read".into());
                    let reply = if reads == 0 { "\x06" } else { "0,1.0E-06" };
                    RequestResult::octet_read(reply.into(), reply.len())
                }
                _ => {
                    self.0.push(format!("{op:?}"));
                    RequestResult::write_ok()
                }
            })
        }
    }

    /// `devTPG261Callback` sets the input EOS to `<CR><LF>` before its first
    /// write, so the device works whatever the port was configured with.
    #[test]
    fn the_callback_sets_its_input_eos_before_writing() {
        let (dev, _h) = live_dev(Kind::Ai, Command::GetPressure, 1, "TPG_IEOS");
        let job = &dev.live.as_ref().unwrap().job;
        job.exchange.lock().unwrap().send = "PR1\r".into();
        let mut wire = Wire(Vec::new());
        job.process(&mut wire);
        let ieos = format!("ieos {:?}", b"\r\n".to_vec());
        let write = |d: &[u8]| format!("write {:?}", d.to_vec());
        assert_eq!(
            wire.0,
            [
                ieos,
                write(b"PR1\r"),
                "read".into(),
                write(&[ENQ]),
                "read".into()
            ]
        );
        assert_eq!(job.exchange.lock().unwrap().rec_buf, "0,1.0E-06");
    }

    #[test]
    fn the_factory_serves_asyn_tpg261_on_its_seven_record_types() {
        let factory = device_support_factory();
        for rtyp in RECORD_TYPES {
            let ctx = DeviceSupportContext {
                record_type: rtyp,
                dtyp: DTYP,
                inp: "@asyn(P 1)GET_PRESSURE",
                out: "@asyn(P 1)SET_SP1",
            };
            assert!(factory(&ctx).is_some(), "{rtyp}");
        }
        let ctx = DeviceSupportContext {
            record_type: "stringout",
            dtyp: DTYP,
            inp: "",
            out: "",
        };
        assert!(factory(&ctx).is_none());
    }
}
