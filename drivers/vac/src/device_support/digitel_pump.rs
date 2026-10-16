//! `devDigitelPump` — the `asyn DigitelPump` device support for the `digitel`
//! record.

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
use epics_rs::base::types::PvString;

use super::{Reentry, asyn_print, c_str, octet_read, octet_write, user};
use crate::protocol::digitel::{
    self, Config, ControlFields, DevType, MAX_CONSEC_ERRORS, RESET_COMMANDS, ReadSlot, Readings,
    ResponseBuf, build_command, check_control_reply, configure, control_command, decode,
    initial_err_count, read_slot, strip_read_reply,
};
use crate::records::digitel::DigitelRecord;

/// `asyn DigitelPump`.
pub const DTYP: &str = "asyn DigitelPump";

/// C `DigitelPump_TIMEOUT` (`devDigitelPump.c:63`) — both C callback paths
/// overwrite `pasynUser->timeout` with this constant before every I/O
/// (`devDigitelPump.c:889,1162`), so the link-parsed timeout is never used.
const IO_TIMEOUT: Duration = Duration::from_secs(1);

/// `SIMM`'s `YES` menu index.
const YES: u16 = 1;

const CALLBACK: &str = "devDigitelPumpCallback";
const PROCESS: &str = "devDigitelPumpProcess";

/// C `pPvt->command` (`devDigitelPump.h:10`), with the control command's
/// `sendBuf`.
enum Command {
    Read,
    Control(Vec<u8>),
    Reset,
}

impl Command {
    fn code(&self) -> i32 {
        match self {
            Command::Read => 0,
            Command::Control(_) => 1,
            Command::Reset => 2,
        }
    }
}

/// What `readWrite_dg`'s first pass did, for the pass that follows.
enum Pass {
    Idle,
    /// SIMM: the record computes the readings, no dset call in C.
    Simulated,
    /// `queueRequest` refused: COMM_ALARM, no I/O, no decode.
    Refused,
    /// The callback ran; its outcome is in [`Job::exchange`].
    Queued,
}

/// C's `devDigitelPumpPvt` as the port-thread callback shares it with the
/// record: what the first pass asks for and what the callback leaves.
struct Exchange {
    command: Command,
    /// C `pPvt->errCount`, persisted across process cycles.
    err_count: i32,
    /// The callback's `recGblSetSevr(pr, READ_ALARM, INVALID_ALARM)`.
    read_alarm: bool,
    /// C `pPvt->recBuf`.
    recbuf: ResponseBuf,
}

/// The queued request's body — C `devDigitelPumpCallback`.
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

pub struct DigitelPump {
    link: AsynLink,
    /// C `pr->name`, for the init refusal lines.
    name: String,
    live: Option<Live>,
}

impl DigitelPump {
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

    /// One `devDigitelPumpProcess`: flush, write, read; the bytes read.
    fn process_dg(&self, port: &mut dyn PortCalls, send: &[u8]) -> Vec<u8> {
        let (name, sent) = (&self.name, String::from_utf8_lossy(send));
        let mut reply = Vec::new();
        // All QPC commands are at least ten bytes; C leaves `*nread` stale on a
        // shorter one, which this reports as nothing read.
        if self.cfg.dev == DevType::Qpc && send.len() < 10 {
            self.trace(
                TraceMask::ERROR,
                format_args!("{PROCESS} sendBuf too small  command=[{sent}]"),
            );
        } else {
            let _ = port.call(&RequestOp::Flush, user(self.addr, IO_TIMEOUT));
            match octet_write(port, user(self.addr, IO_TIMEOUT), send) {
                Err(e) => self.trace(
                    TraceMask::ERROR,
                    format_args!(
                        "{PROCESS} write failed {} Bytes read: 0, Reason: 0",
                        e.message()
                    ),
                ),
                Ok(n) => self.trace(
                    TraceMask::IO_DEVICE,
                    format_args!("{PROCESS} {name} nwrite={n} output=[{sent}]"),
                ),
            }
            let read = octet_read(port, user(self.addr, IO_TIMEOUT), digitel::READ_SIZE);
            match &read.err {
                Some(e) => self.trace(
                    TraceMask::ERROR,
                    format_args!(
                        "{PROCESS} read failed {} Bytes read: {}, Reason: {}",
                        e.message(),
                        read.data.len(),
                        read.eom
                    ),
                ),
                None => self.trace(
                    TraceMask::IO_DEVICE,
                    format_args!(
                        "{PROCESS} {name} nread={} input=[{}]",
                        read.data.len(),
                        c_str(&read.data)
                    ),
                ),
            }
            reply = read.data;
        }
        self.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "{PROCESS} {name} nread={} input=[{}]",
                reply.len(),
                c_str(&reply)
            ),
        );
        reply
    }
}

impl CellBody for Job {
    fn process(&self, port: &mut dyn PortCalls) {
        let mut ex = self.exchange.lock().unwrap();
        let ex = &mut *ex;
        let (name, dev) = (&self.name, self.cfg.dev);
        let mut response = ResponseBuf::default();
        ex.read_alarm = false;

        // A control command that fails or is rejected jumps to `finish` before
        // the read loop, leaving `errCount` untouched.
        let sweep = match &ex.command {
            Command::Control(sendbuf) => {
                let reply = self.process_dg(port, sendbuf);
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
            Command::Reset => {
                // `SL3`/`SL4` are sent verbatim and their replies ignored.
                self.process_dg(port, RESET_COMMANDS[0]);
                self.process_dg(port, RESET_COMMANDS[1]);
                true
            }
            Command::Read => true,
        };

        // The eleven-slot read loop: each stored reply lands at `30*i`; a short
        // or device-flagged reply counts an error and jumps to `finish`.
        if sweep {
            let swept = (0..digitel::READ_SLOTS).all(|i| {
                let ReadSlot::Send(payload) = read_slot(&self.cfg, i) else {
                    return true;
                };
                let reply = self.process_dg(port, &build_command(&self.cfg, &payload));
                if reply.is_empty() {
                    self.trace(
                        TraceMask::ERROR,
                        format_args!("{CALLBACK} {name} Read reply too small=0"),
                    );
                    return false;
                }
                match strip_read_reply(dev, i, &reply) {
                    Ok(data) => {
                        self.trace(
                            TraceMask::IO_DEVICE,
                            format_args!("{CALLBACK} {name} Read reply = [{}]", c_str(&data)),
                        );
                        response.strcpy_at(30 * i, &data);
                        true
                    }
                    Err(_) => {
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

        // C `finish`: unlike `devVacSen`, the copy is unconditional — `recBuf`
        // holds whatever the (possibly partial) read produced.
        ex.recbuf = response;
    }

    /// C's request has no `timeoutUser` and is never canceled, so it always
    /// runs; one that did not is the callback's `finish` with nothing read.
    fn unrun(&self, _why: Unrun) {
        let mut ex = self.exchange.lock().unwrap();
        ex.read_alarm = false;
        ex.recbuf = ResponseBuf::default();
    }

    fn done(self: Arc<Self>) {
        self.reentry.reprocess();
    }
}

fn digitel(record: &mut dyn Record) -> CaResult<&mut DigitelRecord> {
    record
        .as_any_mut()
        .and_then(|a| a.downcast_mut::<DigitelRecord>())
        .ok_or_else(|| CaError::TypeMismatch("DigitelPump requires a digitel record".into()))
}

impl DeviceSupport for DigitelPump {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<DeviceInitOutcome> {
        let rec = digitel(record)?;

        let dev = DevType::from_index(rec.tipe)
            .ok_or_else(|| CaError::FieldNotFound(format!("digitel TYPE index {}", rec.tipe)))?;

        // Every C failure below is a `goto bad` — errlogPrintf, `pr->pact = 1`,
        // no alarm (`devDigitelPump.c:263-269`): the record is dead, not
        // flagged-and-scanning. The TYPE/downcast checks above stay `Err`;
        // they have no C arm (menu/dset binding makes them unreachable).
        let cfg = match configure(dev, self.link.addr, &self.link.drv_info) {
            Ok(cfg) => cfg,
            Err(e) => {
                errlog_vprintf(format_args!("devDigitelPump::init {} {e}\n", self.name));
                return Ok(DeviceInitOutcome::dead());
            }
        };

        let Some(port) = get_port(&self.link.port_name) else {
            // A plain `printf`, carrying `connectDevice`'s errorMessage.
            let port = &self.link.port_name;
            println!(
                "can't connect to serial port {port} asynManager:connectDevice port {port} not found"
            );
            return Ok(DeviceInitOutcome::dead());
        };
        let job = Arc::new(Job {
            name: self.name.clone(),
            cfg,
            handle: port.handle.clone(),
            addr: self.link.addr,
            exchange: Mutex::new(Exchange {
                command: Command::Read,
                err_count: initial_err_count(dev),
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
        Ok(DeviceInitOutcome::Live)
    }

    /// C `readWrite_dg` with `pact == 0`, and the record's `!pact` snapshot.
    fn read_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        let Some(live) = self.live.as_mut() else {
            return Ok(ReadStart::Synchronous);
        };
        let job = &live.job;
        let rec = digitel(record)?;
        rec.dev_alarm = None;

        // `digitelRecord.c` `process` snapshots the readbacks into the `I*`
        // shadow fields before any I/O. Only setpoints 1-3 (`isp1..isp3`) are
        // captured; `ISP4` is deliberately left untouched, as in C.
        rec.ival = rec.val;
        rec.ilva = rec.lval;
        rec.imod = rec.modr;
        rec.ibak = rec.bakr;
        rec.icol = rec.cool;
        rec.isp[0] = rec.set[0];
        rec.isp[1] = rec.set[1];
        rec.isp[2] = rec.set[2];
        rec.iacw = rec.accw;
        rec.iaci = rec.acci;
        rec.ipty = rec.ptyp;
        rec.ibkn = rec.bkin;
        rec.is_ = rec.spr;
        rec.ih = rec.shr;
        rec.im = rec.smr;
        rec.ii = rec.svr;
        rec.ib3 = rec.s3br;
        rec.it3 = rec.s3tr;
        rec.iton = rec.tonl;
        rec.icrn = rec.crnt;
        rec.ivol = rec.volt;
        rec.ierr = rec.err;

        // C never reaches the dset in simulation (`digitelRecord.c:309`); the
        // record's `process()` computes VAL/MODR/SET/CRNT from SVMO/SVS1/SVS2/
        // SVCR.
        if rec.simm == YES {
            live.pass = Pass::Simulated;
            return Ok(ReadStart::Synchronous);
        }

        // A changed control field wins, else a Digitel with a pending error
        // count is reset, else a plain read.
        let mut ex = job.exchange.lock().unwrap();
        ex.command = if rec.flgs != 0 {
            let fields = ControlFields {
                dspl: rec.dspl,
                mods: rec.mods,
                klck: rec.klck,
                baks: rec.baks,
                bkin: rec.bkin,
                spfg: rec.spfg,
                sps: rec.sps,
                spr: rec.spr,
                shs: rec.shs,
                shr: rec.shr,
                sms: rec.sms,
                smr: rec.smr,
                svs: rec.svs,
                svr: rec.svr,
            };
            let cc = control_command(&job.cfg, rec.flgs, &fields);
            if cc.clear_spfg {
                rec.spfg = 0;
            }
            rec.flgs = 0;
            Command::Control(build_command(&job.cfg, &cc.payload))
        } else if ex.err_count != 0 && job.cfg.dev.is_digitel() {
            Command::Reset
        } else {
            Command::Read
        };
        let sendbuf: &[u8] = match &ex.command {
            Command::Control(sendbuf) => sendbuf,
            Command::Read | Command::Reset => b"",
        };
        job.trace(
            TraceMask::IO_DEVICE,
            format_args!(
                "devDigitelPump::readWrite_dg {} command {} len={} string=|{}|",
                self.name,
                ex.command.code(),
                sendbuf.len(),
                String::from_utf8_lossy(sendbuf)
            ),
        );
        drop(ex);

        job.reentry.arm(done);
        match job
            .handle
            .queue_request(&live.cell, &user(job.addr, IO_TIMEOUT))
        {
            Ok(()) => {
                live.pass = Pass::Queued;
                Ok(ReadStart::Pending)
            }
            Err(e) => {
                job.trace(
                    TraceMask::ERROR,
                    format_args!(
                        "devDigitelPump::readWrite {} ERROR calling queueRequest\n    status={}, error={}",
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

    /// C `readWrite_dg` with `pact == 1`, or the end of a refused first pass.
    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let live = self
            .live
            .as_mut()
            .ok_or_else(|| CaError::LinkError("DigitelPump not initialised".into()))?;
        let rec = digitel(record)?;
        match std::mem::replace(&mut live.pass, Pass::Idle) {
            Pass::Queued => {}
            // The record's own simulation branch clears UDF.
            Pass::Simulated => return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched)),
            Pass::Refused => {
                rec.dev_alarm = Some(alarm_status::COMM_ALARM);
                return Ok(DeviceReadOutcome::computed(DeviceUdf::Untouched));
            }
            Pass::Idle => {
                return Err(CaError::LinkError(
                    "DigitelPump read without a request".into(),
                ));
            }
        }
        let cfg = &live.job.cfg;
        let ex = live.job.exchange.lock().unwrap();
        if ex.read_alarm {
            rec.dev_alarm = Some(alarm_status::READ_ALARM);
        }

        rec.err = ex.err_count as i16;
        if ex.err_count > MAX_CONSEC_ERRORS {
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

        // Full decode. Fields the reply does not rewrite keep their previous
        // record values, matching C's carry-over of VOLT/CRNT/TONL/ACCW/ACCI/
        // COOL/BKIN/SP*R/S*HR/S3TR.
        let prev = Readings {
            volt: rec.volt,
            crnt: rec.crnt,
            tonl: rec.tonl as i64,
            accw: rec.accw,
            acci: rec.acci,
            cool: rec.cool as i64,
            bkin: rec.bkin,
            spr: rec.spr,
            shr: rec.shr,
            s3tr: rec.s3tr,
            ..Readings::default()
        };
        let out = decode(cfg, &ex.recbuf, &prev);

        rec.val = out.val;
        rec.lval = out.lval;
        rec.volt = out.volt;
        rec.crnt = out.crnt;
        rec.tonl = out.tonl as u32;
        rec.modr = out.modr;
        rec.cmor = out.cmor;
        rec.bakr = out.bakr;
        rec.set = out.set;
        rec.accw = out.accw;
        rec.acci = out.acci;
        rec.cool = out.cool as f64;
        rec.ptyp = out.ptyp;
        rec.bkin = out.bkin;
        rec.spr = out.spr;
        rec.shr = out.shr;
        rec.smr = out.smr;
        rec.svr = out.svr;
        rec.s3br = out.s3br;
        rec.s3tr = out.s3tr;
        if let Some(modl) = out.modl {
            rec.modl = PvString::from_bytes(modl);
        }
        if let Some(vers) = out.vers {
            rec.vers = PvString::from_bytes(vers);
        }
        rec.dev_ran = true;

        Ok(DeviceReadOutcome::computed(DeviceUdf::Defined))
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        // Control commands are issued inside `read` on the same processing pass
        // as the status read, exactly as C's `readWrite_dg` does.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::asyn::request::RequestResult;
    use epics_rs::asyn::runtime::port::PortRuntimeHandle;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::AsynUser;
    use epics_rs::base::server::database::{DeviceDone, PvDatabase};
    use epics_rs::base::server::record::RecordInstance;

    use super::*;

    /// A D500 pump on an IP port that never connects, its trace in a file.
    fn pump(port: &str) -> (DigitelPump, PathBuf, PortRuntimeHandle) {
        let mut driver = DrvAsynIPPort::new(port, "127.0.0.1:1").unwrap();
        driver.base_mut().init_auto_connect(false);
        let runtime = PortManager::new()
            .register_port(driver, &open_owner())
            .unwrap();
        let path = std::env::temp_dir().join(format!("{port}-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        runtime
            .port_handle()
            .trace()
            .set_trace_file(Some(port), TraceFile::File(Arc::new(Mutex::new(file))));
        let mut dev = DigitelPump::new(AsynLink {
            port_name: port.into(),
            addr: 0,
            timeout: None,
            drv_info: "0".into(),
        });
        dev.set_record_info("V:IP1", ScanType::Passive);
        let mut rec = d500();
        assert!(matches!(dev.init(&mut rec), Ok(DeviceInitOutcome::Live)));
        (dev, path, runtime)
    }

    fn d500() -> DigitelRecord {
        DigitelRecord {
            tipe: DevType::D500 as u16,
            ..Default::default()
        }
    }

    fn traced(path: &PathBuf) -> String {
        let text = std::fs::read_to_string(path).unwrap();
        let _ = std::fs::remove_file(path);
        text
    }

    /// C `readWrite_dg`'s refused `queueRequest`: both lines through the
    /// port's trace, COMM_ALARM, and ERR left as it was.
    #[test]
    fn a_refused_queue_request_raises_comm_and_prints_both_lines() {
        let (mut dev, path, _runtime) = pump("DGREFUSED");
        let mut rec = DigitelRecord { err: 1, ..d500() };
        let db = PvDatabase::new();
        let done = DeviceDone::new(&db, &RecordInstance::new("V:IP1", d500()));

        let start = dev
            .read_begin(&mut rec, &DeviceDoneSource::new(&done))
            .unwrap();
        assert!(matches!(start, ReadStart::Synchronous));
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.dev_alarm, Some(alarm_status::COMM_ALARM));
        assert_eq!(rec.err, 1);

        let log = traced(&path);
        assert!(
            log.contains(
                "devDigitelPump::readWrite V:IP1 ERROR calling queueRequest\n    \
                 status=4, error=port DGREFUSED not connected\n"
            ),
            "{log}"
        );
    }

    /// A port that answers every read with nothing.
    struct Silent;

    impl PortCalls for Silent {
        fn call(&mut self, op: &RequestOp, _user: AsynUser) -> AsynResult<RequestResult> {
            Ok(match op {
                RequestOp::OctetRead { .. } => RequestResult::octet_read(Vec::new(), 0),
                _ => RequestResult::write_ok(),
            })
        }
    }

    /// C `devDigitelPumpCallback` on an empty reply: one error counted, the
    /// line printed, and the pass it re-enters holds the last readings.
    #[test]
    fn an_empty_read_reply_counts_one_error() {
        let (mut dev, path, _runtime) = pump("DGSILENT");
        let live = dev.live.as_mut().unwrap();
        live.job.exchange.lock().unwrap().err_count = 0;
        live.job.process(&mut Silent);
        live.pass = Pass::Queued;

        let mut rec = DigitelRecord {
            val: 6e-9,
            ..d500()
        };
        dev.read(&mut rec).unwrap();
        assert_eq!(rec.err, 1);
        assert_eq!(rec.dev_alarm, None);
        assert_eq!(rec.val, 6e-9);
        let log = traced(&path);
        assert!(
            log.contains("devDigitelPumpCallback V:IP1 Read reply too small=0\n"),
            "{log}"
        );
    }
}
