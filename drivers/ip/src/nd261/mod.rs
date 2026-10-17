//! `devAiHeidND261.c` — the `asyn ai HeidND261` device support for the
//! Heidenhain ND261 display unit.
//!
//! Per-record, as in C: each process queues one request whose callback writes
//! `^B\n`, sets the input EOS to `\n\n`, reads the reply and re-enters the
//! record. A failed or short read leaves VAL at 99999 with UDF set.

pub mod protocol;

use std::sync::{Arc, Mutex};

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynStatus};
use epics_rs::asyn::port::QueuePriority;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{CellBody, PortCalls, RequestCell, RequestOp, Unrun};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::{AsynUser, timeout_from_secs};
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::{DeviceDone, DeviceDoneSource};
use epics_rs::base::server::device_support::{
    DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, DeviceUdf, ReadStart,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::record::{Record, ScanType};
use epics_rs::base::types::EpicsValue;

use protocol::{INVALID_VALUE, NCHAR, READ_COMMAND, TERM, parse_position};

/// The `device(ai,INST_IO,devAiHeidND261,…)` line of `ipSupport.dbd`.
pub const DTYP: &str = "asyn ai HeidND261";

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// `pPvt->timeout`, seconds (`devAiHeidND261.c:102`).
const TIMEOUT: f64 = 1.0;

/// Dynamic device-support factory for `devAiHeidND261`. The link is parsed in
/// `init`, where C's `init_record` parses it.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        (ctx.record_type == "ai" && ctx.dtyp == DTYP)
            .then(|| Box::new(DevHeidNd261::new(ctx.inp)) as Box<dyn DeviceSupport>)
    }
}

/// What `devAiHeidND261Callback` leaves in `devAiHeidND261Pvt`.
#[derive(Default)]
struct Exchange {
    /// `inbuf[0..nread]`.
    read: Vec<u8>,
    /// `pPvt->status`, the read's.
    status: i32,
}

/// The queued request's body — `devAiHeidND261Callback`.
struct Job {
    handle: PortHandle,
    exchange: Mutex<Exchange>,
    /// The record's completion and band.
    reentry: Mutex<Option<(DeviceDone, CallbackPriority)>>,
}

fn user() -> AsynUser {
    AsynUser::new(0)
        .with_addr(ADDR)
        .with_timeout_opt(timeout_from_secs(TIMEOUT))
}

impl CellBody for Job {
    /// The write's and `setInputEos`'s statuses are discarded: C overwrites
    /// the first with the read's and never reads the second.
    fn process(&self, port: &mut dyn PortCalls) {
        let data = READ_COMMAND.to_vec();
        let _ = port.call(&RequestOp::OctetWrite { data }, user());
        let eos = TERM.to_vec();
        let _ = port.call(&RequestOp::SetInputEos { eos }, user());
        let read = port.call(&RequestOp::OctetRead { buf_size: NCHAR }, user());
        let mut ex = self.exchange.lock().unwrap();
        (ex.status, ex.read) = match read {
            Ok(r) => (0, r.data.unwrap_or_default()),
            Err(e) => (
                e.c_status_code(),
                e.partial_read().map(|p| p.data.clone()).unwrap_or_default(),
            ),
        };
    }

    /// No `timeoutUser`, so only a port shutting down with the request queued
    /// lands here; the pass completes as a failed read.
    fn unrun(&self, why: Unrun) {
        let mut ex = self.exchange.lock().unwrap();
        ex.read.clear();
        ex.status = match why {
            Unrun::Refused(e) => e.c_status_code(),
            _ => AsynStatus::Error as i32,
        };
    }

    /// `dbScanLock(pr); prset->process(pr)` at the end of the callback.
    fn done(self: Arc<Self>) {
        let armed = self.reentry.lock().unwrap().clone();
        if let Some((done, prio)) = armed {
            let _ = done.reprocess(prio);
        }
    }

    /// `createAsynUser(devAiHeidND261Callback, 0)`.
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
}

pub struct DevHeidNd261 {
    link: String,
    /// C `pr->name`.
    name: String,
    live: Option<Live>,
}

impl DevHeidNd261 {
    pub fn new(link: &str) -> Self {
        Self {
            link: link.to_owned(),
            name: String::new(),
            live: None,
        }
    }

    /// `completeIO`.
    fn complete(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let Some(live) = self.live.as_ref() else {
            return Ok(DeviceReadOutcome::ok());
        };
        let position = {
            let ex = live.job.exchange.lock().unwrap();
            if ex.status == 0 {
                parse_position(&ex.read)
            } else {
                None
            }
        };
        Ok(match position {
            Some(v) => {
                record.put_field("VAL", EpicsValue::Double(v))?;
                DeviceReadOutcome::computed(DeviceUdf::Defined)
            }
            None => {
                record.put_field("VAL", EpicsValue::Double(INVALID_VALUE))?;
                DeviceReadOutcome::no_value(DeviceUdf::Undefined)
            }
        })
    }
}

impl DeviceSupport for DevHeidNd261 {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn holds_dpvt(&self) -> bool {
        self.live.is_some()
    }

    fn set_record_info(&mut self, name: &str, _scan: ScanType) {
        self.name = name.to_owned();
    }

    /// C `init_record`. Every refusal is its `bad:` arm, `pr->pact = 1`.
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
                    "devAiHeidND261::init {name} bad link {message}\n"
                ));
                return Ok(DeviceInitOutcome::dead());
            }
        };
        let Some(port) = get_port(&link.port_name) else {
            return Ok(DeviceInitOutcome::dead());
        };
        let handle = port.handle.clone();
        let trace = |args: std::fmt::Arguments<'_>| {
            handle.trace().print(
                Some(handle.port_name()),
                Some(ADDR),
                0,
                TraceMask::IO_DEVICE,
                args,
            );
        };
        let term = String::from_utf8_lossy(TERM);
        trace(format_args!("devAiHeidND261 {name} term = '{term}'\n"));
        // C prints the double timeout with `%d` and the int nchar with `%s`;
        // the values are what it meant.
        trace(format_args!("   timeout = {}\n", TIMEOUT as i32));
        trace(format_args!("   format = %lf\n"));
        trace(format_args!("   nchar = {NCHAR}\n"));
        let job = Arc::new(Job {
            handle: handle.clone(),
            exchange: Mutex::new(Exchange::default()),
            reentry: Mutex::new(None),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live { job, cell });
        Ok(DeviceInitOutcome::ok())
    }

    /// `io` with `pact == 0`. A refused `queueRequest` returns -1 with PACT
    /// already set, so the record stays active, as in C.
    fn read_begin(
        &mut self,
        _record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<ReadStart> {
        let Some(live) = self.live.as_ref().filter(|_| !done.pact()) else {
            return Ok(ReadStart::Synchronous);
        };
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
        Ok(ReadStart::Pending)
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        self.complete(record)
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::request::RequestResult;
    use epics_rs::base::server::records::ai::AiRecord;

    use super::*;

    fn live_dev(port: &str) -> DevHeidNd261 {
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
            handle: handle.clone(),
            exchange: Mutex::new(Exchange::default()),
            reentry: Mutex::new(None),
        });
        let mut dev = DevHeidNd261::new("");
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
        });
        dev
    }

    /// A port that logs each op and answers a read with `reply`, or fails it.
    struct Wire {
        log: Vec<String>,
        reply: Option<Vec<u8>>,
    }

    impl PortCalls for Wire {
        fn call(&mut self, op: &RequestOp, _user: AsynUser) -> AsynResult<RequestResult> {
            self.log.push(match op {
                RequestOp::SetInputEos { eos } => format!("ieos {eos:?}"),
                RequestOp::OctetWrite { data } => format!("write {data:?}"),
                RequestOp::OctetRead { buf_size } => format!("read {buf_size}"),
                other => format!("{other:?}"),
            });
            match (op, &self.reply) {
                (RequestOp::OctetRead { .. }, Some(reply)) => {
                    Ok(RequestResult::octet_read(reply.clone(), reply.len()))
                }
                (RequestOp::OctetRead { .. }, None) => Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "timeout".into(),
                }),
                _ => Ok(RequestResult::write_ok()),
            }
        }
    }

    fn run(reply: Option<&[u8]>, port: &str) -> (Wire, AiRecord, DeviceReadOutcome) {
        let mut dev = live_dev(port);
        let mut wire = Wire {
            log: Vec::new(),
            reply: reply.map(<[u8]>::to_vec),
        };
        dev.live.as_ref().unwrap().job.process(&mut wire);
        let mut rec = AiRecord::default();
        let out = dev.read(&mut rec).unwrap();
        (wire, rec, out)
    }

    /// The callback: `^B\n`, then `setInputEos("\n\n")`, then an 18-byte read.
    #[test]
    fn the_callback_writes_then_sets_its_input_eos_then_reads() {
        let (wire, rec, out) = run(Some(b"- 0012.3456 mm"), "ND261_OK");
        assert_eq!(wire.log, ["write [2, 10]", "ieos [10, 10]", "read 18"]);
        assert_eq!(rec.val, -12.3456);
        assert!(out.did_compute());
        assert_eq!(out.udf(), DeviceUdf::Defined);
    }

    /// `completeIO`'s failure arm: VAL 99999, UDF 1, return -1.
    #[test]
    fn a_failed_or_short_read_is_99999_and_undefined() {
        for (reply, port) in [(None, "ND261_TMO"), (Some(&b"+ 12.3"[..]), "ND261_SHORT")] {
            let (_, rec, out) = run(reply, port);
            assert_eq!(rec.val, INVALID_VALUE, "{port}");
            assert!(out.status.read_failed(), "{port}");
            assert_eq!(out.udf(), DeviceUdf::Undefined, "{port}");
        }
    }

    #[test]
    fn the_factory_serves_only_its_ai() {
        let factory = device_support_factory();
        let ctx = |record_type, dtyp| DeviceSupportContext {
            record_type,
            dtyp,
            inp: "@asyn(P)",
            out: "",
        };
        assert!(factory(&ctx("ai", DTYP)).is_some());
        assert!(factory(&ctx("ao", DTYP)).is_none());
        assert!(factory(&ctx("ai", "asyn MPC")).is_none());
    }
}
