//! `devVacSen` — the `asyn VacSen` device support for the `vs` record.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::adapter::AsynLink;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{CellBody, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::database::DeviceDoneSource;
use epics_rs::base::server::device_support::{
    DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, ReadStart,
};
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::{Record, ScanType};

use super::{Reentry, asyn_print, c_str, octet_read, octet_write, user};
use crate::protocol::vac_sen::{
    self, Config, DevType, Readings, ResponseBuf, check_control_reply, configure, control_command,
    decode, place_offset, read_command, skips_read, strip_read_reply,
};
use crate::records::vs::{DGS_FIELD, IG1_FIELD, IG2_FIELD, VsRecord};

/// `asyn VacSen`.
pub const DTYP: &str = "asyn VacSen";

/// C `vacSen_TIMEOUT` (`devVacSen.c:65`) — the C callback overwrites
/// `pasynUser->timeout` with this constant before every I/O, so the
/// link-parsed timeout is never used.
const IO_TIMEOUT: Duration = Duration::from_secs(3);

const CALLBACK: &str = "devVacSen::callback";
const WRITE_READ: &str = "devVacSen::writeRead";

/// What `readWrite_vs`'s first pass did, for the pass that follows.
enum Pass {
    Idle,
    /// `queueRequest` refused: COMM_ALARM, no I/O, no decode.
    Refused,
    /// The callback ran; its outcome is in [`Job::exchange`].
    Queued,
}

/// C's `devVacSenPvt` as the port-thread callback shares it with the record.
struct Exchange {
    /// C `pPvt->command == cmdControl`'s `sendBuf`.
    control: Option<Vec<u8>>,
    /// C `pPvt->errCount`, persisted across process cycles.
    err_count: i32,
    /// The callback's `recGblSetSevr(pr, READ_ALARM, INVALID_ALARM)`.
    read_alarm: bool,
    /// C `pPvt->recBuf`.
    recbuf: ResponseBuf,
}

/// The queued request's body — C `devVacSenCallback`.
struct Job {
    name: String,
    cfg: Config,
    handle: PortHandle,
    addr: i32,
    exchange: Mutex<Exchange>,
    reentry: Reentry,
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    pass: Pass,
}

pub struct VacSen {
    link: AsynLink,
    /// C `pr->name`, for the init refusal lines.
    name: String,
    live: Option<Live>,
}

impl VacSen {
    /// Build from the parsed `INP` link. Configuration (which needs the
    /// record's `TYPE`) and the port connection are deferred to `init`.
    pub fn new(link: AsynLink) -> Self {
        Self {
            link,
            name: String::new(),
            live: None,
        }
    }
}

impl Job {
    fn trace(&self, mask: TraceMask, args: std::fmt::Arguments<'_>) {
        asyn_print(&self.handle, mask, args);
    }

    /// One `devVacSenWriteRead`: write, read up to `READ_SIZE`, and flush only
    /// when the read filled the buffer — the C heuristic for "no terminator
    /// seen, drop the junk".
    fn write_read(&self, port: &mut dyn PortCalls, send: &[u8]) -> Vec<u8> {
        let name = &self.name;
        let n = match octet_write(port, user(self.addr, IO_TIMEOUT), send) {
            Ok(n) => n,
            Err(e) => {
                self.trace(
                    TraceMask::ERROR,
                    format_args!(
                        "{WRITE_READ} {name} pasynOctet->write, status={}, error={}",
                        e.status() as i32,
                        e.message()
                    ),
                );
                0
            }
        };
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "{WRITE_READ} {name} nwrite={n} output=[{}]",
                String::from_utf8_lossy(send)
            ),
        );
        let read = octet_read(port, user(self.addr, IO_TIMEOUT), vac_sen::READ_SIZE);
        if let Some(e) = &read.err {
            self.trace(
                TraceMask::ERROR,
                format_args!(
                    "{WRITE_READ} {name} pasynOctet->read, status={}, error={}",
                    e.status() as i32,
                    e.message()
                ),
            );
        }
        if read.data.len() == vac_sen::READ_SIZE {
            // C prints this at ERROR whatever the flush returned.
            let (status, error) = match port.call(&RequestOp::Flush, user(self.addr, IO_TIMEOUT)) {
                Ok(_) => (0, String::new()),
                Err(e) => (e.status() as i32, e.message()),
            };
            self.trace(
                TraceMask::ERROR,
                format_args!(
                    "{WRITE_READ} {name} pasynOctet->flush, status={status}, error={error}"
                ),
            );
        }
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "{WRITE_READ} {name} nread={} input=[{}]",
                read.data.len(),
                c_str(&read.data)
            ),
        );
        read.data
    }
}

impl CellBody for Job {
    fn process(&self, port: &mut dyn PortCalls) {
        let mut ex = self.exchange.lock().unwrap();
        let ex = &mut *ex;
        let (name, dev) = (&self.name, self.cfg.dev);
        ex.read_alarm = false;

        // A control command that fails or is rejected jumps to `finish` before
        // the read loop, leaving `errCount` untouched.
        let sweep = match &ex.control {
            Some(sendbuf) => {
                let reply = self.write_read(port, sendbuf);
                if reply.is_empty() {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!("{CALLBACK} {name} Cmd reply too small=0"),
                    );
                    false
                } else if check_control_reply(dev, &reply).is_err() {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!("{CALLBACK} {name} Cmd reply has error=[{}]", c_str(&reply)),
                    );
                    ex.read_alarm = true;
                    false
                } else {
                    true
                }
            }
            None => true,
        };

        // The eight-slot read loop. A short/long reply or a device-flagged
        // error counts an error and jumps to `finish`.
        let mut response = ResponseBuf::default();
        if sweep {
            let swept = (0..8).filter(|&i| !skips_read(&self.cfg, i)).all(|i| {
                let mut reply = self.write_read(port, &read_command(&self.cfg, i));
                if reply.is_empty() || reply.len() > 50 {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!(
                            "{CALLBACK} {name} Read reply too small/too large ={}",
                            reply.len()
                        ),
                    );
                    return false;
                }
                match strip_read_reply(dev, i, &reply) {
                    Ok(data) => {
                        response.strcpy_at(place_offset(dev, i), &data);
                        true
                    }
                    Err(_) => {
                        // C overwrites a CC10 reply's STX before printing it.
                        if dev == DevType::Cc10 {
                            reply[0] = b'Z';
                        }
                        self.trace(
                            TraceMask::ERROR,
                            format_args!(
                                "{CALLBACK} {name} Read reply has error=[{}]",
                                c_str(&reply)
                            ),
                        );
                        false
                    }
                }
            });
            if swept {
                ex.err_count = 0;
            } else {
                ex.err_count += 1;
            }
        }

        // C `finish`: `recBuf` holds the response only on a clean read.
        ex.recbuf = if ex.err_count == 0 {
            response
        } else {
            ResponseBuf::default()
        };
    }

    /// Nothing cancels the request and it has no timeout callback, so only a
    /// port shutting down with it queued leaves it unrun — a path C, whose
    /// ports are never deleted, has no counterpart for. The pass then decodes
    /// nothing read.
    fn unrun(&self, why: Unrun) {
        assert!(matches!(why, Unrun::Refused(_)), "vac request {why:?}");
        let mut ex = self.exchange.lock().unwrap();
        ex.read_alarm = false;
        ex.recbuf = ResponseBuf::default();
    }

    fn done(self: Arc<Self>) {
        self.reentry.reprocess();
    }

    /// `createAsynUser(callback, 0)` (devVacSen.c:125): a request on a disconnected
    /// device runs.
    fn timeout_user(&self) -> bool {
        false
    }
}

fn vs(record: &mut dyn Record) -> CaResult<&mut VsRecord> {
    record
        .as_any_mut()
        .and_then(|a| a.downcast_mut::<VsRecord>())
        .ok_or_else(|| CaError::TypeMismatch("VacSen requires a vs record".into()))
}

impl DeviceSupport for VacSen {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        let rec = vs(record)?;

        let dev = DevType::from_index(rec.tipe)
            .ok_or_else(|| CaError::FieldNotFound(format!("vs TYPE index {}", rec.tipe)))?;

        // Every C failure below is a `goto bad` — errlogPrintf "record
        // disabled", `pr->pact = 1`, no alarm (`devVacSen.c:306-314`). The
        // TYPE/downcast checks above stay `Err`; they have no C arm.
        // C's `bad:` prints its own line after whichever refusal led there; a
        // failed `connectDevice` prints nothing else.
        let bad = |name: &str| {
            errlog_vprintf(format_args!(
                "devVacSen::init {name} Problem initializing - record disabled.\n"
            ));
            Ok(DeviceInitOutcome::dead())
        };
        let cfg = match configure(dev, self.link.addr, &self.link.drv_info) {
            Ok(cfg) => cfg,
            Err(e) => {
                errlog_vprintf(format_args!("devVacSen::init {} {e}\n", self.name));
                return bad(&self.name);
            }
        };

        let Some(port) = get_port(&self.link.port_name) else {
            return bad(&self.name);
        };
        let job = Arc::new(Job {
            name: self.name.clone(),
            cfg,
            handle: port.handle.clone(),
            addr: self.link.addr,
            exchange: Mutex::new(Exchange {
                control: None,
                err_count: 0,
                read_alarm: false,
                recbuf: ResponseBuf::default(),
            }),
            reentry: Reentry::default(),
        });
        let cell = port.handle.new_cell(job.clone());
        self.live = Some(Live {
            job,
            cell,
            pass: Pass::Idle,
        });
        Ok(DeviceInitOutcome::ok())
    }

    /// C `readWrite_vs` with `pact == 0`.
    fn read_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        // The pass a completion re-entered is `pact == 1`: `read` decodes.
        let Some(live) = self.live.as_mut().filter(|_| !done.pact()) else {
            return Ok(ReadStart::Synchronous);
        };
        let job = &live.job;
        let rec = vs(record)?;
        rec.dev_alarm = None;

        // One control command per cycle when a `SPC_MOD` field changed, else
        // a plain read. MM200/CC10/MX200 have no control commands; C still
        // leaves the prefix in `sendBuf` for them.
        let (code, sendbuf) = if rec.chgc != 0 {
            let cmd = if rec.chgc & IG1_FIELD != 0 {
                rec.ig1s as usize
            } else if rec.chgc & IG2_FIELD != 0 {
                2 + rec.ig2s as usize
            } else if rec.chgc & DGS_FIELD != 0 {
                4 + rec.dgss as usize
            } else {
                0
            };
            rec.chgc = 0;
            match control_command(&job.cfg, cmd) {
                Some(sendbuf) => (1, sendbuf),
                None => (0, job.cfg.prefix.clone()),
            }
        } else {
            (0, Vec::new())
        };
        job.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devVacSen::readWrite {} command {code} len={} string=|{}|",
                self.name,
                sendbuf.len(),
                String::from_utf8_lossy(&sendbuf)
            ),
        );
        job.exchange.lock().unwrap().control = (code == 1).then_some(sendbuf);

        job.reentry.arm(done);
        match job
            .handle
            .queue_request(&live.cell, &user(job.addr, IO_TIMEOUT))
        {
            Ok(_) => {
                live.pass = Pass::Queued;
                Ok(ReadStart::Pending)
            }
            Err(e) => {
                job.trace(
                    TraceMask::ERROR,
                    format_args!(
                        "devVacSen::readWrite {} ERROR calling queueRequest status={}, error={}",
                        self.name,
                        e.status() as i32,
                        e.message()
                    ),
                );
                live.pass = Pass::Refused;
                Ok(ReadStart::Synchronous)
            }
        }
    }

    /// C `readWrite_vs` with `pact == 1`, or the end of a refused first pass.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let live = self
            .live
            .as_mut()
            .ok_or_else(|| CaError::LinkError("VacSen not initialised".into()))?;
        let rec = vs(record)?;
        match std::mem::replace(&mut live.pass, Pass::Idle) {
            Pass::Queued => {}
            Pass::Refused => {
                rec.dev_alarm = Some(alarm_status::COMM_ALARM);
                return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
            }
            Pass::Idle => {
                return Err(CaError::LinkError("VacSen read without a request".into()));
            }
        }
        let cfg = &live.job.cfg;
        let ex = live.job.exchange.lock().unwrap();
        if ex.read_alarm {
            rec.dev_alarm = Some(alarm_status::READ_ALARM);
        }

        // The error-count gate.
        let err_limit = rec.err as i32;
        if ex.err_count >= err_limit {
            // recGblSetSevr(READ_ALARM, INVALID); udf = 0.
            rec.dev_alarm = Some(alarm_status::READ_ALARM);
            rec.dev_ran = true;
            return Ok(DeviceReadOutcome::computed(DeviceUdf::Defined));
        }
        if ex.err_count > 0 {
            // Transient error: keep the last good readings, clear UDF.
            rec.dev_ran = true;
            return Ok(DeviceReadOutcome::computed(DeviceUdf::Defined));
        }

        // Full decode. Fields the device type does not rewrite keep their
        // previous record values (C never resets CGAP/CGBP/SP*/SP*R).
        let prev = Readings {
            cgap: rec.cgap,
            cgbp: rec.cgbp,
            sp: rec.sp,
            spr: rec.spr,
            ..Readings::default()
        };
        let out = decode(cfg.dev, cfg.spt, &ex.recbuf, &prev);

        rec.val = out.val;
        rec.cgap = out.cgap;
        rec.cgbp = out.cgbp;
        rec.ig1r = out.ig1r;
        rec.ig2r = out.ig2r;
        rec.dgsr = out.dgsr;
        rec.sp = out.sp;
        rec.spr = out.spr;

        // C: `lprs = log10(val); lcap = log10(cgap); lcbp = log10(cgbp);
        // pres = val; udf = 0;`
        rec.lprs = rec.val.log10();
        rec.lcap = rec.cgap.log10();
        rec.lcbp = rec.cgbp.log10();
        rec.pres = rec.val;
        rec.dev_ran = true;

        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        // The `vs` record is read-only from the wire's perspective; control
        // commands are issued inside `read` on the same processing pass as the
        // status read, exactly as the C `readWrite_vs` does.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::manager::{PortManager, claim_port_driver};
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::base::server::database::{DeviceDone, PvDatabase};
    use epics_rs::base::server::record::RecordInstance;

    use super::*;

    /// C `readWrite_vs`'s refused `queueRequest`: its one-line print through
    /// the port's trace, COMM_ALARM, and no decode.
    #[test]
    fn a_refused_queue_request_raises_comm_and_prints_its_line() {
        let port = "VSREFUSED";
        let claim = claim_port_driver(port).unwrap();
        let mut driver = DrvAsynIPPort::new(port, "127.0.0.1:1").unwrap();
        driver.base_mut().init_auto_connect(false);
        let runtime = PortManager::new()
            .register_port(claim, driver, &open_owner())
            .unwrap();
        let path = std::env::temp_dir().join(format!("{port}-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        runtime
            .port_handle()
            .trace()
            .set_trace_file(Some(port), TraceFile::File(Arc::new(Mutex::new(file))));

        let mut dev = VacSen::new(AsynLink {
            port_name: port.into(),
            addr: 0,
            timeout: None,
            drv_info: String::new(),
        });
        dev.set_record_info("V:G1", ScanType::Passive);
        let mut rec = VsRecord {
            val: 1e-7,
            ..Default::default()
        };
        assert!(matches!(dev.init(&mut rec), Ok(o) if o == DeviceInitOutcome::ok()));
        let db = PvDatabase::new();
        let done = DeviceDone::new(&db, &RecordInstance::new("V:G1", VsRecord::default()));

        let start = dev
            .read_begin(&mut rec, &DeviceDoneSource::new(&done))
            .unwrap();
        assert!(matches!(start, ReadStart::Synchronous));
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.dev_alarm, Some(alarm_status::COMM_ALARM));
        assert_eq!(rec.val, 1e-7);

        let log = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            log.contains(
                "devVacSen::readWrite V:G1 ERROR calling queueRequest status=4, \
                 error=port VSREFUSED not connected\n"
            ),
            "{log}"
        );
    }

    /// The pass a completion re-enters is C `readWrite_vs` with `pact == 1`:
    /// it decodes what landed and queues nothing.
    #[test]
    fn the_continuation_pass_does_not_queue_again() {
        let port = "VSREENTRY";
        let claim = claim_port_driver(port).unwrap();
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let driver = DrvAsynIPPort::new(port, &addr).unwrap();
        let _runtime = PortManager::new()
            .register_port(claim, driver, &open_owner())
            .unwrap();

        let mut dev = VacSen::new(AsynLink {
            port_name: port.into(),
            addr: 0,
            timeout: None,
            drv_info: String::new(),
        });
        dev.set_record_info("V:G2", ScanType::Passive);
        let mut rec = VsRecord::default();
        assert!(matches!(dev.init(&mut rec), Ok(o) if o == DeviceInitOutcome::ok()));
        let db = PvDatabase::new();
        let done = DeviceDone::new(&db, &RecordInstance::new("V:G2", VsRecord::default()));

        let start = dev
            .read_begin(&mut rec, &DeviceDoneSource::new(&done))
            .unwrap();
        assert!(matches!(start, ReadStart::Pending));
        // The listener never answers: the exchange ends on its timeout, and
        // the cell is free to queue again, as it is when C re-enters.
        let deadline = std::time::Instant::now() + 4 * IO_TIMEOUT;
        while dev
            .live
            .as_ref()
            .unwrap()
            .job
            .exchange
            .lock()
            .unwrap()
            .err_count
            == 0
        {
            assert!(
                std::time::Instant::now() < deadline,
                "the exchange never ended"
            );
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        let again = dev
            .read_begin(&mut rec, &DeviceDoneSource::continuing(&done))
            .unwrap();
        assert!(matches!(again, ReadStart::Synchronous));
    }
}
