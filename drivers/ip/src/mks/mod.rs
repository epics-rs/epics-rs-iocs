//! `devAiMKS.c` — the `HPS SensaVac 937` ai device support for the MKS 937
//! vacuum gauge controller.
//!
//! One record per gauge, as in C: `init` reads the controller's units (`SU`)
//! and gauge types (`SG`) for that record and sets its EGU / PREC / LOPR /
//! HOPR, and every process queues one `Rn` exchange on the octet port named in
//! `INP` (`@asyn(port gauge)`, gauge 1..5) and writes VAL directly.
//!
//! One deliberate deviation: C sets `pai->val = 0.` before decoding every
//! reply and then suppresses the alarm on the 937's spurious `SYNTAX!` /
//! `NotCMD!` (`devAiMKS.c:317-347`), publishing 0 Torr as good data. Here
//! those replies leave VAL alone.

pub mod protocol;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{CellBody, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::error::{CaError, CaResult};
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

use crate::Unconnected;
use protocol::{GaugeType, Reading};

/// `device(ai,INST_IO,devAiMKS,"HPS SensaVac 937")` (`ipSupport.dbd`).
pub const DTYP: &str = "HPS SensaVac 937";

/// `TIMEOUT` (`devAiMKS.c:65`).
const TIMEOUT: Duration = Duration::from_secs(2);

/// `MAX_RESPONSE_LEN` (`devAiMKS.c:66`).
const MAX_RESPONSE_LEN: usize = 10;

/// The reply length C accepts from every 937 query (`nread != 7`).
const REPLY_LEN: usize = 7;

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// Dynamic device-support factory for `HPS SensaVac 937`. The link is parsed
/// in `init`, where C's `initAi` parses it and reports a bad one.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        (ctx.dtyp == DTYP).then(|| Box::new(DevAiMks::new(ctx.inp)) as Box<dyn DeviceSupport>)
    }
}

/// `asynPrint` on the record's asynUser, connected to `port` at address 0.
fn print(handle: &PortHandle, mask: TraceMask, args: std::fmt::Arguments<'_>) {
    handle
        .trace()
        .print(Some(handle.port_name()), Some(ADDR), 0, mask, args);
}

/// `pasynOctetSyncIO->writeReadOnce(port, 0, data, ..., TIMEOUT, ...)`,
/// with its own failure line.
fn write_read_once(port: &str, data: &[u8]) -> Result<Vec<u8>, ()> {
    let (io, connected) = SyncIOHandle::connect_port(port, ADDR, TIMEOUT, InterfaceType::Octet);
    if let Err(e) = connected {
        io.print(
            0,
            TraceMask::ERROR,
            format_args!("asynOctetSyncIO connect failed {}\n", e.message()),
        );
        return Err(());
    }
    io.write_read(0, data, MAX_RESPONSE_LEN).map_err(|e| {
        io.print(
            0,
            TraceMask::ERROR,
            format_args!("asynOctetSyncIO writeReadOnce failed {}\n", e.message()),
        );
    })
}

/// The pass a queued request's callback re-enters — C's `dbScanLock(pai);
/// prset->process(pai)` at the end of `callbackAi`.
#[derive(Default)]
struct Reentry(Mutex<Option<(DeviceDone, CallbackPriority)>>);

impl Reentry {
    fn arm(&self, done: &DeviceDoneSource<'_>) {
        let mut slot = self.0.lock().unwrap();
        match &mut *slot {
            Some((_, prio)) => *prio = done.priority(),
            None => *slot = Some((done.device_done(), done.priority())),
        }
    }

    fn reprocess(&self) {
        let armed = self.0.lock().unwrap().clone();
        if let Some((done, prio)) = armed {
            let _ = done.reprocess(prio);
        }
    }
}

/// The queued request's body — C `callbackAi`'s I/O half.
struct Job {
    name: String,
    handle: PortHandle,
    /// `pPvt->readCommand`, `R<gauge>`.
    command: Vec<u8>,
    /// The read's bytes, `None` when its status was not success.
    reply: Mutex<Option<Vec<u8>>>,
    reentry: Reentry,
}

impl CellBody for Job {
    fn process(&self, port: &mut dyn PortCalls) {
        let user = || AsynUser::new(0).with_addr(ADDR).with_timeout(TIMEOUT);
        // C discards the write's status; the read's decides.
        let _ = port.call(
            &RequestOp::OctetWrite {
                data: self.command.clone(),
            },
            user(),
        );
        let read = port.call(
            &RequestOp::OctetRead {
                buf_size: MAX_RESPONSE_LEN,
            },
            user(),
        );
        let (data, ok) = match read {
            Ok(r) => (r.data.unwrap_or_default(), true),
            Err(e) => (
                e.partial_read().map(|p| p.data.clone()).unwrap_or_default(),
                false,
            ),
        };
        print(
            &self.handle,
            TraceMask::IO_DEVICE,
            format_args!(
                "devAiMKS record={}, len={}, response=\n{}\n",
                self.name,
                data.len(),
                String::from_utf8_lossy(&data)
            ),
        );
        *self.reply.lock().unwrap() = ok.then_some(data);
    }

    /// No `timeoutUser`, so only a port shutting down with the request queued
    /// lands here; the pass reads it as a failed read.
    fn unrun(&self, _why: Unrun) {
        *self.reply.lock().unwrap() = None;
    }

    fn done(self: Arc<Self>) {
        self.reentry.reprocess();
    }

    /// `createAsynUser(callbackAi, 0)` (`devAiMKS.c:118`).
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    /// Whether this cycle's request was queued, so `read` has a reply.
    queued: bool,
}

pub struct DevAiMks {
    inp: String,
    /// C `pai->name`.
    name: String,
    /// `gaugeLow` / `gaugeHigh`, in the controller's units.
    low: f64,
    high: f64,
    live: Option<Live>,
    /// `initAi`'s unknown-gauge-type `recGblSetSevr`, which C leaves pending
    /// until the first completed process resets the alarms.
    init_alarm: bool,
    /// The alarm the last completed read raised.
    alarm: Option<AlarmSeverity>,
}

impl DevAiMks {
    pub fn new(inp: &str) -> Self {
        Self {
            inp: inp.to_owned(),
            name: String::new(),
            low: 0.0,
            high: 0.0,
            live: None,
            init_alarm: false,
            alarm: None,
        }
    }

    /// `recGblSetSevr(pai, READ_ALARM, sevr)`.
    fn raise(&mut self, sevr: AlarmSeverity) {
        if self.alarm.is_none_or(|old| sevr > old) {
            self.alarm = Some(sevr);
        }
    }

    /// One of `initAi`'s `writeReadOnce` queries, refused unless it returns
    /// the 7 characters C requires.
    fn query(&self, handle: &PortHandle, port: &str, command: &str) -> Option<String> {
        let reply = write_read_once(port, command.as_bytes()).unwrap_or_default();
        let text = String::from_utf8_lossy(&reply).into_owned();
        if reply.len() != REPLY_LEN {
            print(
                handle,
                TraceMask::ERROR,
                format_args!(
                    "devAiMKS ERROR, record={}, nread={}, response=\n{text}\n",
                    self.name,
                    reply.len()
                ),
            );
            return None;
        }
        print(
            handle,
            TraceMask::IO_DEVICE,
            format_args!(
                "devAiMKS::initAi record={}, nread={}, response=\n{text}\n",
                self.name,
                reply.len()
            ),
        );
        Some(text)
    }
}

fn double(record: &dyn Record, field: &str) -> f64 {
    match record.get_field(field) {
        Some(EpicsValue::Double(v)) => v,
        _ => 0.0,
    }
}

impl DeviceSupport for DevAiMks {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    /// C `initAi`. Every refusal is its `bad:` arm, `pai->pact = 1`.
    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        let name = self.name.clone();
        let link = match parse_asyn_link(&self.inp) {
            Ok(link) => link,
            Err(e) => {
                let message = match e {
                    AsynError::InvalidLinkSyntax(m) => m,
                    other => other.to_string(),
                };
                errlog_vprintf(format_args!(
                    "devAiMKS::initAi {name} bad link {message} \n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };
        let gauge = match u8::try_from(link.addr) {
            Ok(g @ 1..=5) => g,
            _ => {
                errlog_vprintf(format_args!(
                    "devAiMKS::init_record invalid gauge {}\n",
                    link.addr
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };
        let port_name = link.port_name;
        let handle = match crate::connect_octet(&port_name, 0) {
            Ok(handle) => handle,
            Err(Unconnected::Device(_)) => {
                errlog_vprintf(format_args!(
                    "devAiMKS::initAi {name} cannot connect to device {port_name}\n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
            Err(Unconnected::NoOctet) => {
                errlog_vprintf(format_args!(
                    "devAiMKS::initAi {name} cannot find asynOctet interface\n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };

        if record.get_field("PREC") == Some(EpicsValue::Short(0)) {
            record.put_field("PREC", EpicsValue::Short(1))?;
        }

        let dead = || {
            Ok(DeviceInitOutcome::dead_with_alarm(
                READ_ALARM,
                AlarmSeverity::Major,
            ))
        };
        let Some(units) = self.query(&handle, &port_name, protocol::READ_UNITS) else {
            return dead();
        };
        record.put_field("EGU", EpicsValue::String(units.as_str().into()))?;
        let Some(types) = self.query(&handle, &port_name, protocol::READ_GAUGE_TYPES) else {
            return dead();
        };
        let code = protocol::gauge_type_field(&types, gauge).unwrap_or("");
        match GaugeType::from_code(code) {
            Some(gauge_type) => (self.low, self.high) = gauge_type.limits(),
            None => {
                print(
                    &handle,
                    TraceMask::ERROR,
                    format_args!("devAiMKS, unknown gauge type {code}\n"),
                );
                self.init_alarm = true;
            }
        }
        let mult = protocol::unit_multiplier(&units);
        self.low *= mult;
        self.high *= mult;
        if double(record, "LOPR") == 0.0 && double(record, "HOPR") == 0.0 {
            record.put_field("LOPR", EpicsValue::Double(self.low))?;
            record.put_field("HOPR", EpicsValue::Double(self.high))?;
        }
        print(
            &handle,
            TraceMask::IO_DEVICE,
            format_args!(
                "devAiMKS record={name}, gauge_str={code}, lopr={:.6}, hopr={:.6}\n",
                double(record, "LOPR"),
                double(record, "HOPR")
            ),
        );

        let job = Arc::new(Job {
            name,
            handle: handle.clone(),
            command: protocol::read_pressure(gauge).into_bytes(),
            reply: Mutex::new(None),
            reentry: Reentry::default(),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live {
            job,
            cell,
            queued: false,
        });
        Ok(DeviceInitOutcome::ok())
    }

    /// C `readAi` with `pact == 0`: queue the exchange.
    fn read_begin(
        &mut self,
        _record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        let Some(live) = self.live.as_mut().filter(|_| !done.pact()) else {
            return Ok(ReadStart::Synchronous);
        };
        live.job.reentry.arm(done);
        let user = AsynUser::new(0)
            .with_addr(ADDR)
            .with_timeout(TIMEOUT)
            .with_priority(QueuePriority::Low);
        live.queued = live.job.handle.queue_request(&live.cell, &user).is_ok();
        Ok(if live.queued {
            ReadStart::Pending
        } else {
            ReadStart::Synchronous
        })
    }

    /// `callbackAi`'s decode and `readAi` with `pact == 1`. C ignores a
    /// refused `queueRequest` and leaves the record active forever; here the
    /// refusal is read as the failed read it is.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let live = self
            .live
            .as_mut()
            .ok_or_else(|| CaError::LinkError("devAiMKS not initialised".into()))?;
        let reply = if std::mem::take(&mut live.queued) {
            live.job.reply.lock().unwrap().take()
        } else {
            None
        };
        let (name, handle) = (live.job.name.clone(), live.job.handle.clone());
        self.alarm = None;
        if std::mem::take(&mut self.init_alarm) {
            self.raise(AlarmSeverity::Major);
        }
        let raw = match reply {
            Some(raw) if raw.len() == REPLY_LEN => raw,
            _ => {
                self.raise(AlarmSeverity::Major);
                return Ok(DeviceReadOutcome::computed(DeviceUdf::Defined));
            }
        };
        let text = String::from_utf8_lossy(&raw);
        let val = match protocol::parse_reading(&text) {
            Reading::Pressure(value) => Some(value),
            Reading::AboveRange => {
                self.raise(AlarmSeverity::Minor);
                Some(self.high)
            }
            Reading::BelowRange => {
                self.raise(AlarmSeverity::Minor);
                Some(self.low)
            }
            Reading::NoGauge => {
                print(
                    &handle,
                    TraceMask::FLOW,
                    format_args!("devAiMKS record={name}, no gauge, etc.\n"),
                );
                self.raise(AlarmSeverity::Major);
                Some(0.0)
            }
            Reading::Spurious => {
                print(
                    &handle,
                    TraceMask::FLOW,
                    format_args!("devAiMKS record={name}, SYNTAX or NotCMD response.\n"),
                );
                None
            }
            Reading::Unknown => {
                print(
                    &handle,
                    TraceMask::ERROR,
                    format_args!(
                        "devAiMKS record={name}, unknown response len={} response={text}\n",
                        text.len()
                    ),
                );
                self.raise(AlarmSeverity::Major);
                Some(0.0)
            }
        };
        if let Some(val) = val {
            record.put_field("VAL", EpicsValue::Double(val))?;
        }
        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }

    fn last_alarm(&self) -> Option<DeviceAlarm<'_>> {
        self.alarm.map(|sevr| DeviceAlarm {
            stat: READ_ALARM,
            sevr: sevr as u16,
            msg: "",
        })
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }

    /// C `convertAi` fills the slot and does nothing; the record still
    /// rebases EOFF to EGUL around the call.
    fn has_special_linconv(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::manager::{PortManager, claim_port_driver};
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::runtime::port::PortRuntimeHandle;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::base::server::records::ai::AiRecord;

    use super::*;

    /// A registered octet port that never connects, its trace in a file.
    fn refused_port(port: &str) -> (PortRuntimeHandle, PathBuf) {
        let mut driver = DrvAsynIPPort::new(port, "127.0.0.1:1").unwrap();
        driver.base_mut().init_auto_connect(false);
        let runtime = PortManager::new()
            .register_port(claim_port_driver(port).unwrap(), driver, &open_owner())
            .unwrap();
        let path = std::env::temp_dir().join(format!("{port}-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        runtime
            .port_handle()
            .trace()
            .set_trace_file(Some(port), TraceFile::File(Arc::new(Mutex::new(file))));
        (runtime, path)
    }

    fn init(port: &str, record: &str, gauge: u8) -> (DevAiMks, DeviceInitOutcome) {
        let mut dev = DevAiMks::new(&format!("@asyn({port} {gauge})"));
        dev.set_record_info(record, ScanType::Passive);
        let outcome = dev.init(&mut AiRecord::default()).unwrap();
        (dev, outcome)
    }

    /// Each record runs C's `initAi` once, against a dead controller: the
    /// SyncIO failure line, then the error line naming the record, and the
    /// record is dead with READ/MAJOR.
    #[test]
    fn a_failed_units_read_names_the_record_and_kills_it() {
        let port = "MKS_REFUSED";
        let (_runtime, path) = refused_port(port);
        let records = ["ipExample:cc1", "ipExample:pr2"];
        for (record, gauge) in records.iter().zip([1, 5]) {
            let (_, outcome) = init(port, record, gauge);
            assert_eq!(
                outcome,
                DeviceInitOutcome::dead_with_alarm(READ_ALARM, AlarmSeverity::Major)
            );
        }
        let log = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        let lines: Vec<_> = log.lines().filter(|l| !l.is_empty()).collect();
        assert_eq!(lines.len(), 4, "{log}");
        for (pair, record) in lines.chunks(2).zip(records) {
            assert!(
                pair[0].ends_with(&format!(
                    "asynOctetSyncIO writeReadOnce failed asynManager::queueLockPort \
                     queueRequest failed: port {port} not connected"
                )),
                "{log}"
            );
            assert!(
                pair[1].ends_with(&format!(
                    "devAiMKS ERROR, record={record}, nread=0, response="
                )),
                "{log}"
            );
        }
    }

    /// `initAi`'s refusals before any I/O: no port, no gauge, no link.
    #[test]
    fn init_refuses_what_c_refuses_before_any_io() {
        let (_, outcome) = init("MKS_NO_SUCH_PORT", "R:nop", 1);
        assert_eq!(outcome, DeviceInitOutcome::dead());
        let (_, outcome) = init("MKS_NO_SUCH_PORT", "R:g6", 6);
        assert_eq!(outcome, DeviceInitOutcome::dead());
        let mut dev = DevAiMks::new("not a link");
        assert_eq!(
            dev.init(&mut AiRecord::default()).unwrap(),
            DeviceInitOutcome::dead()
        );
    }

    /// A port without asynOctet fails `findInterface`: C prints its line and
    /// takes `bad:` before any I/O (devAiMKS.c:152-156).
    #[test]
    fn a_port_without_octet_kills_the_record_before_any_io() {
        use epics_rs::base::runtime::log::{
            errlog_add_listener, errlog_flush, errlog_remove_listener,
        };

        crate::no_octet_port("MKS_INT32_ONLY");
        let lines = Arc::new(Mutex::new(Vec::<String>::new()));
        let id = errlog_add_listener({
            let lines = lines.clone();
            move |line: &str| lines.lock().unwrap().push(line.to_string())
        });
        let (_, outcome) = init("MKS_INT32_ONLY", "R:i32", 1);
        errlog_flush();
        errlog_remove_listener(id);
        assert_eq!(outcome, DeviceInitOutcome::dead());
        let lines = lines.lock().unwrap();
        let want = "devAiMKS::initAi R:i32 cannot find asynOctet interface\n";
        assert!(lines.iter().any(|l| l == want), "{lines:#?}");
    }

    fn decode(dev: &mut DevAiMks, reply: Option<&[u8]>, val: f64) -> (f64, Option<u16>) {
        let live = dev.live.as_mut().unwrap();
        live.queued = true;
        *live.job.reply.lock().unwrap() = reply.map(<[u8]>::to_vec);
        let mut rec = AiRecord::default();
        rec.val = val;
        let outcome = dev.read(&mut rec).unwrap();
        assert!(outcome.did_compute());
        (rec.val, dev.last_alarm().map(|a| a.sevr))
    }

    /// `callbackAi`'s decode, one case per C arm.
    #[test]
    fn replies_decode_as_callback_ai_does() {
        let port = "MKS_DECODE";
        let (runtime, _path) = refused_port(port);
        let handle = runtime.port_handle().clone();
        let job = Arc::new(Job {
            name: "R:cc1".into(),
            handle: handle.clone(),
            command: b"R1".to_vec(),
            reply: Mutex::new(None),
            reentry: Reentry::default(),
        });
        let mut dev = DevAiMks::new("");
        (dev.low, dev.high) = (1e-11, 1e-2);
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
            queued: false,
        });
        let (major, minor) = (
            Some(AlarmSeverity::Major as u16),
            Some(AlarmSeverity::Minor as u16),
        );

        assert_eq!(decode(&mut dev, Some(b"1.2E-06"), 5.0), (1.2e-6, None));
        assert_eq!(decode(&mut dev, Some(b"H 1.0E0"), 5.0), (1e-2, minor));
        assert_eq!(decode(&mut dev, Some(b"L 1.0E0"), 5.0), (1e-11, minor));
        assert_eq!(decode(&mut dev, Some(b"NO GAUG"), 5.0), (0.0, major));
        assert_eq!(decode(&mut dev, Some(b"garbage"), 5.0), (0.0, major));
        // A failed or short read alarms and leaves VAL alone.
        assert_eq!(decode(&mut dev, None, 5.0), (5.0, major));
        assert_eq!(decode(&mut dev, Some(b"1.2"), 5.0), (5.0, major));
        // C writes 0 here with no alarm; VAL is left alone instead.
        assert_eq!(decode(&mut dev, Some(b"SYNTAX!"), 5.0), (5.0, None));

        // The unknown-gauge-type alarm from init lands on the first read.
        dev.init_alarm = true;
        assert_eq!(decode(&mut dev, Some(b"1.2E-06"), 5.0), (1.2e-6, major));
        assert_eq!(decode(&mut dev, Some(b"1.2E-06"), 5.0), (1.2e-6, None));
    }
}
