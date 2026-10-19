//! `devTelevac.c` — the `asyn Televac` device support for the Televac vacuum
//! gauge controller.
//!
//! Per-record ai and bi device support, as in C: the link is
//! `@asyn(port addr)COMMAND n` on the octet port, `n` the station or relay,
//! and every process queues one write/read exchange whose callback re-enters
//! the record. ipApp ships no database for it.

pub mod protocol;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{CellBody, OctetReadOp, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};
use epics_rs::base::server::device_support::{
    DeviceAlarm, DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, ReadStart,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::recgbl::alarm_status::READ_ALARM;
use epics_rs::base::server::record::{AlarmSeverity, Record, ScanType};
use epics_rs::base::types::EpicsValue;

use crate::scan::Scan;
use crate::user_param::c_str;
use protocol::Command;

/// `device(ai|bi,INST_IO,dev<Type>Televac,"asyn Televac")` (`ipSupport.dbd`).
pub const DTYP: &str = "asyn Televac";

/// `Televac_TIMEOUT` (`devTelevac.c:59`).
const TIMEOUT: Duration = Duration::from_secs(1);

/// `Televac_BUFFER_SIZE` (`devTelevac.c:58`).
const BUFFER_SIZE: usize = 25;

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// Dynamic device-support factory for `asyn Televac`. The link is parsed in
/// `init`, where C's `initCommon` parses it.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        let ai = match ctx.record_type {
            "ai" => true,
            "bi" => false,
            _ => return None,
        };
        (ctx.dtyp == DTYP).then(|| Box::new(DevTelevac::new(ai, ctx.inp)) as Box<dyn DeviceSupport>)
    }
}

/// `asynPrint` on the record's asynUser, connected to `port` at address 0.
fn print(handle: &PortHandle, mask: TraceMask, args: std::fmt::Arguments<'_>) {
    handle
        .trace()
        .print(Some(handle.port_name()), Some(ADDR), 0, mask, args);
}

/// What `devTelevacCallback` leaves in `devTelevacPvt`.
#[derive(Default)]
struct Exchange {
    /// `sendBuf`.
    send: String,
    /// `recBuf`.
    rec_buf: Vec<u8>,
    /// The callback's `recGblSetSevr`.
    alarm: bool,
}

/// The queued request's body — `devTelevacCallback`.
struct Job {
    name: String,
    command: Command,
    handle: PortHandle,
    exchange: Mutex<Exchange>,
    /// The record's completion and band.
    reentry: Mutex<Option<(DeviceDone, CallbackPriority)>>,
}

impl Job {
    fn trace(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        print(&self.handle, mask, args);
    }
}

impl CellBody for Job {
    /// Both statuses are discarded, as in C: a failed read is `nread < 1`.
    fn process(&self, port: &mut dyn PortCalls) {
        let name = &self.name;
        let mut ex = self.exchange.lock().unwrap();
        let user = || AsynUser::new(0).with_addr(ADDR).with_timeout(TIMEOUT);
        let data = ex.send.as_bytes().to_vec();
        let nwrite = match port.call(&RequestOp::OctetWrite { data }, user()) {
            Ok(_) => ex.send.len(),
            Err(_) => 0,
        };
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTelevac::devTelevacCallback {name} nwrite={nwrite}, output={}\n",
                ex.send
            ),
        );
        // C reads `input` whatever the status.
        let (input, _) = port
            .octet_read(
                OctetReadOp::Read {
                    buf_size: BUFFER_SIZE,
                },
                user(),
            )
            .into_parts();
        let nread = input.len();
        // C prints nwrite here.
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTelevac::devTelevacCallback {name} nread={nread}, input={}\n",
                String::from_utf8_lossy(&input)
            ),
        );
        if nread < 1 {
            self.trace(
                TraceMask::ERROR,
                format_args!("devTelevac::devTelevacCallback {name} message too small={nread}\n"),
            );
            ex.alarm = true;
        }
        ex.rec_buf = c_str(&input).to_vec();
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devTelevac: {name} command ({}) received (after processing) |{}|\n",
                self.command.number(),
                String::from_utf8_lossy(&ex.rec_buf)
            ),
        );
    }

    /// No `timeoutUser`, so only a port shutting down with the request queued
    /// lands here; the pass completes as a read of nothing.
    fn unrun(&self, _why: Unrun) {
        let mut ex = self.exchange.lock().unwrap();
        ex.rec_buf.clear();
        ex.alarm = true;
    }

    /// `dbScanLock(pr); prset->process(pr)` at the end of the callback.
    fn done(self: Arc<Self>) {
        let armed = self.reentry.lock().unwrap().clone();
        if let Some((done, prio)) = armed {
            let _ = done.reprocess(prio);
        }
    }

    /// `createAsynUser(devTelevacCallback, 0)` (`devTelevac.c:117`).
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    /// `pPvt->parameter`, the station or relay.
    parameter: i32,
}

pub struct DevTelevac {
    /// ai, or bi.
    ai: bool,
    link: String,
    /// C `pr->name`.
    name: String,
    live: Option<Live>,
    /// The callback's alarm, taken by the pass that completed it.
    alarm: bool,
}

impl DevTelevac {
    pub fn new(ai: bool, link: &str) -> Self {
        Self {
            ai,
            link: link.to_owned(),
            name: String::new(),
            live: None,
            alarm: false,
        }
    }

    fn routine(&self) -> &'static str {
        if self.ai { "readAi" } else { "readBi" }
    }

    fn wrong_type(&self, live: &Live) {
        print(
            &live.job.handle,
            TraceMask::ERROR,
            format_args!(
                "devTelevac::{} {} Wrong record type \n",
                self.routine(),
                self.name
            ),
        );
    }

    /// The `pact == 0` half of `readAi` / `readBi`, then `startIOCommon`. The
    /// wrong-type arm still queues its empty `sendBuf`, as C does; a refused
    /// `queueRequest` returns -1 with PACT already set, so the record stays
    /// active.
    fn start(&mut self, done: &DeviceDoneSource<'_>) {
        self.alarm = false;
        let Some(live) = self.live.as_ref() else {
            return;
        };
        let send = live
            .job
            .command
            .request(self.ai, live.parameter)
            .unwrap_or_else(|| {
                self.wrong_type(live);
                String::new()
            });
        print(
            &live.job.handle,
            TraceMask::IO_DEVICE,
            format_args!(
                "devTelevac::{} {} command {} len={} string=|{send}|\n",
                self.routine(),
                self.name,
                live.job.command.number(),
                send.len()
            ),
        );
        live.job.exchange.lock().unwrap().send = send;
        {
            let mut slot = live.job.reentry.lock().unwrap();
            match &mut *slot {
                Some((_, prio)) => *prio = done.priority(),
                None => *slot = Some((done.device_done(), done.priority())),
            }
        }
        let user = AsynUser::new(0)
            .with_addr(ADDR)
            .with_priority(QueuePriority::Low);
        let _ = live.job.handle.queue_request(&live.cell, &user);
    }
}

impl DeviceSupport for DevTelevac {
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
                    "devTelevac::initCommon {name} bad link {message}\n"
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
                "devTelevac::initCommon {name} invalid userParam (null)\n"
            ));
            return Ok(DeviceInitOutcome::dead());
        }
        // `sscanf(userParam, "%s %d", command, &i)`. C sends an uninitialised
        // `i` when the number is missing; that link is refused here.
        let (word, rest) = user_param
            .split_once(|c: char| c.is_ascii_whitespace())
            .unwrap_or((user_param, ""));
        let Some(parameter) = Scan::new(rest.as_bytes()).int() else {
            errlog_vprintf(format_args!(
                "devTelevac::initCommon {name} invalid userParam {user_param}\n"
            ));
            return Ok(DeviceInitOutcome::dead());
        };
        let Some(command) = Command::from_name(word) else {
            print(
                &handle,
                TraceMask::ERROR,
                format_args!("devTelevac::init_common {name}, unknown command={word}\n"),
            );
            return Ok(DeviceInitOutcome::dead());
        };
        print(
            &handle,
            TraceMask::FLOW,
            format_args!(
                "devTelevac::initCommon name={name}; command string={word} command={}, \
                 address={:02X}; parameter={parameter};\n",
                command.number(),
                link.addr as u32
            ),
        );
        let job = Arc::new(Job {
            name,
            command,
            handle: handle.clone(),
            exchange: Mutex::new(Exchange::default()),
            reentry: Mutex::new(None),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live {
            job,
            cell,
            parameter,
        });
        Ok(DeviceInitOutcome::ok())
    }

    fn read_begin(
        &mut self,
        _record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        if done.pact() || self.live.is_none() {
            return Ok(ReadStart::Synchronous);
        }
        self.start(done);
        Ok(ReadStart::Pending)
    }

    /// C `TelevacConvert`, devAiTelevac's slot; devBiTelevac has none.
    fn has_special_linconv(&self) -> bool {
        self.ai
    }

    fn special_linconv(&mut self, record: &mut dyn Record, _after: bool) -> CaResult<()> {
        crate::unit_slope(record)
    }

    /// `readAi` / `readBi` with `pact == 1`. Both clear UDF whatever the
    /// reply; `readAi` returns 2, `readBi` 0 so RVAL converts.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Some(live) = self.live.as_ref() else {
            return Ok(DeviceReadOutcome::ok());
        };
        let rec_buf = {
            let mut ex = live.job.exchange.lock().unwrap();
            self.alarm = std::mem::take(&mut ex.alarm);
            ex.rec_buf.clone()
        };
        let command = live.job.command;
        if command.request(self.ai, live.parameter).is_none() {
            self.wrong_type(live);
        } else if self.ai {
            record.put_field("VAL", EpicsValue::Double(protocol::pressure(&rec_buf)))?;
        } else {
            let rval = protocol::relay_rval(&rec_buf, live.parameter);
            record.put_field("RVAL", EpicsValue::ULong(rval))?;
        }
        Ok(if self.ai {
            DeviceReadOutcome::computed(DeviceUdf::Defined)
        } else {
            DeviceReadOutcome::converted(DeviceUdf::Defined)
        })
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        self.alarm.then_some(DeviceAlarm {
            stat: READ_ALARM,
            sevr: AlarmSeverity::Invalid as u16,
            msg: "",
        })
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::error::{AsynResult, AsynStatus};
    use epics_rs::asyn::interpose::EomReason;
    use epics_rs::asyn::request::{OctetRead, RequestResult};
    use epics_rs::base::server::records::ai::AiRecord;
    use epics_rs::base::server::records::bi::BiRecord;

    use super::*;

    fn live_dev(ai: bool, command: Command, parameter: i32, port: &str) -> DevTelevac {
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
            exchange: Mutex::new(Exchange::default()),
            reentry: Mutex::new(None),
        });
        let mut dev = DevTelevac::new(ai, "");
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
            parameter,
        });
        dev
    }

    /// A port that logs each op and answers a read with `reply`, or times out.
    struct Wire {
        log: Vec<String>,
        reply: Option<&'static [u8]>,
    }

    impl PortCalls for Wire {
        fn call(&mut self, op: &RequestOp, _user: AsynUser) -> AsynResult<RequestResult> {
            self.log.push(match op {
                RequestOp::OctetWrite { data } => {
                    format!("write {}", String::from_utf8_lossy(data))
                }
                RequestOp::OctetRead { buf_size, .. } => format!("read {buf_size}"),
                other => format!("{other:?}"),
            });
            match (op, self.reply) {
                (RequestOp::OctetRead { .. }, Some(reply)) => Ok(RequestResult::octet_read(
                    OctetRead::new(reply.to_vec(), EomReason::empty(), Ok(())),
                    reply.len(),
                )),
                (RequestOp::OctetRead { .. }, None) => Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "timeout".into(),
                }),
                _ => Ok(RequestResult::write_ok()),
            }
        }
    }

    /// One pass: the `pact == 0` send buffer, the callback's exchange, then
    /// the `pact == 1` decode.
    fn pass(dev: &mut DevTelevac, record: &mut dyn Record, reply: Option<&'static [u8]>) -> Wire {
        let live = dev.live.as_ref().unwrap();
        let send = live
            .job
            .command
            .request(dev.ai, live.parameter)
            .unwrap_or_default();
        live.job.exchange.lock().unwrap().send = send;
        let mut wire = Wire {
            log: Vec::new(),
            reply,
        };
        live.job.process(&mut wire);
        dev.read(record).unwrap();
        wire
    }

    #[test]
    fn an_ai_pass_writes_its_request_and_decodes_the_pressure() {
        let mut dev = live_dev(true, Command::GetPressure, 2, "TVAC_AI");
        let mut rec = AiRecord::default();
        let wire = pass(&mut dev, &mut rec, Some(b"2=7.60-2T"));
        assert_eq!(wire.log, ["write R2", "read 25"]);
        assert_eq!(rec.val, f64::from(7.6f32) * 10f64.powf(-2.0));
        assert!(dev.last_alarm().is_none());
    }

    /// `nread < 1` alarms READ/INVALID, and the empty reply reads 9.9e+9.
    #[test]
    fn a_read_of_nothing_alarms_and_reads_over_range() {
        let mut dev = live_dev(true, Command::GetRelayOn, 1, "TVAC_SILENT");
        let mut rec = AiRecord::default();
        let wire = pass(&mut dev, &mut rec, None);
        assert_eq!(wire.log, ["write SP1N", "read 25"]);
        assert_eq!(rec.val, f64::from(9.9f32) * 1e9);
        let alarm = dev.last_alarm().unwrap();
        assert_eq!(
            (alarm.stat, alarm.sevr),
            (READ_ALARM, AlarmSeverity::Invalid as u16)
        );
    }

    #[test]
    fn a_bi_pass_reads_its_relay_bit_into_rval() {
        let mut dev = live_dev(false, Command::GetRelayStatus, 3, "TVAC_BI");
        let mut rec = BiRecord::default();
        let wire = pass(&mut dev, &mut rec, Some(b"05"));
        assert_eq!(wire.log, ["write RY", "read 25"]);
        assert_eq!(rec.get_field("RVAL"), Some(EpicsValue::ULong(4)));
    }

    /// The wrong-type arm sends an empty buffer and leaves VAL alone.
    #[test]
    fn a_bi_command_on_an_ai_leaves_val() {
        let mut dev = live_dev(true, Command::GetRelayStatus, 1, "TVAC_WRONG");
        let mut rec = AiRecord::default();
        rec.val = 5.0;
        let wire = pass(&mut dev, &mut rec, Some(b"0F"));
        assert_eq!(wire.log, ["write ", "read 25"]);
        assert_eq!(rec.val, 5.0);
    }

    #[test]
    fn the_factory_serves_asyn_televac_on_ai_and_bi() {
        let factory = device_support_factory();
        let ctx = |record_type, dtyp| DeviceSupportContext {
            record_type,
            dtyp,
            inp: "@asyn(P 0)GET_PRESSURE 1",
            out: "",
        };
        assert!(factory(&ctx("ai", DTYP)).is_some());
        assert!(factory(&ctx("bi", DTYP)).is_some());
        assert!(factory(&ctx("ao", DTYP)).is_none());
        assert!(factory(&ctx("ai", "asyn TPG261")).is_none());
    }

    /// A port without `asynOctet` fails `findInterface`: `bad:`, `pact = 1`
    /// (devTelevac.c:136-137).
    #[test]
    fn a_port_without_octet_kills_the_record() {
        crate::no_octet_port("TV_INT32_ONLY");
        let mut dev = DevTelevac::new(true, "@asyn(TV_INT32_ONLY 0)GET_PRESSURE 1");
        dev.set_record_info("R:i32", ScanType::Passive);
        let outcome = dev.init(&mut AiRecord::default()).unwrap();
        assert_eq!(outcome, DeviceInitOutcome::dead());
    }
}
