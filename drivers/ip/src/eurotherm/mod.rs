//! `devXxEurotherm.c` — the `asyn ao Eurotherm` and `asyn so Eurotherm`
//! device support for the Eurotherm 800/2000 temperature controllers.
//!
//! Per-record and write-only, as in C: the link is
//! `@asyn(port)TERM=..,IX=..,FMT=..,TO=..,GAD=..,LAD=..`, and every process
//! queues one frame write whose callback re-enters the record. The reply is
//! left for a `devXxStrParm` record to read (`Eurotherm.db`).

pub mod protocol;

use std::sync::{Arc, Mutex};

use epics_rs::asyn::adapter::parse_asyn_link;
use epics_rs::asyn::error::AsynError;
use epics_rs::asyn::manager::PortManager;
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
    DeviceInitOutcome, DeviceReadOutcome, DeviceSupport, WriteStart,
};
use epics_rs::base::server::ioc_app::DeviceSupportContext;
use epics_rs::base::server::record::{Record, ScanType};
use epics_rs::base::types::EpicsValue;

use crate::Unconnected;
use crate::fmt::{c_literal, format_c_double};
use crate::mpc::protocol::{c_atoi, c_strtod};
use crate::user_param::{after, c_str, term, up_to_comma};
use protocol::{ENQ, ETX};

/// The `device(ao,…)` and `device(stringout,…)` lines of `ipSupport.dbd`.
pub const DEVICES: [(&str, &str); 2] = [
    ("ao", "asyn ao Eurotherm"),
    ("stringout", "asyn so Eurotherm"),
];

/// The address C's `connectDevice(pasynUser, port, 0)` binds.
const ADDR: i32 = 0;

/// Dynamic device-support factory for the two Eurotherm dsets. The link is
/// parsed in `init`, where C's `initCommon` parses it.
pub fn device_support_factory()
-> impl Fn(&DeviceSupportContext) -> Option<Box<dyn DeviceSupport>> + Send + Sync + 'static {
    |ctx: &DeviceSupportContext| {
        let kind = match (ctx.record_type, ctx.dtyp) {
            ("ao", dtyp) if dtyp == DEVICES[0].1 => Kind::Ao,
            ("stringout", dtyp) if dtyp == DEVICES[1].1 => Kind::So,
            _ => return None,
        };
        Some(Box::new(DevEurotherm::new(kind, ctx.out)) as Box<dyn DeviceSupport>)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Ao,
    So,
}

/// `devEurothermPvt`'s configuration, as `initAo`/`initSo` and `initCommon`
/// fill it.
#[derive(Debug, Clone, PartialEq)]
struct Parm {
    term: Vec<u8>,
    /// `bufferStartIndex`.
    index: i32,
    format: String,
    /// Seconds.
    timeout: f64,
    group: i32,
    local: i32,
}

impl Parm {
    fn parse(kind: Kind, user_param: &str) -> Self {
        let (term_byte, format) = match kind {
            Kind::Ao => (ETX, "%f"),
            Kind::So => (ENQ, "%s"),
        };
        let mut parm = Self {
            term: vec![term_byte],
            index: 0,
            format: format.to_owned(),
            timeout: 3.0,
            group: 0,
            // 0 is the 800 series' default; on a 2000 it broadcasts.
            local: 0,
        };
        if let Some(hex) = after(user_param, "TERM=") {
            parm.term = term(hex);
        }
        if let Some(v) = after(user_param, "IX=") {
            parm.index = c_atoi(v.as_bytes());
        }
        if let Some(v) = after(user_param, "FMT=") {
            parm.format = up_to_comma(v);
        }
        if let Some(v) = after(user_param, "TO=") {
            parm.timeout = c_strtod(v.as_bytes());
        }
        if let Some(v) = after(user_param, "GAD=") {
            parm.group = c_atoi(v.as_bytes());
        }
        if let Some(v) = after(user_param, "LAD=") {
            parm.local = c_atoi(v.as_bytes());
        }
        parm
    }
}

/// The queued request's body — `devEurothermCallback`.
struct Job {
    timeout: f64,
    handle: PortHandle,
    /// `buffer[0..nchar]`.
    frame: Mutex<Vec<u8>>,
    /// The record's completion and band.
    reentry: Mutex<Option<(DeviceDone, CallbackPriority)>>,
}

impl CellBody for Job {
    /// The write's status only reaches `completeIOCommon`, whose return the
    /// output records do not look at.
    fn process(&self, port: &mut dyn PortCalls) {
        let data = self.frame.lock().unwrap().clone();
        let user = AsynUser::new(0)
            .with_addr(ADDR)
            .with_timeout_opt(timeout_from_secs(self.timeout));
        let _ = port.call(&RequestOp::OctetWrite { data }, user);
    }

    fn unrun(&self, _why: Unrun) {}

    /// `dbScanLock(pr); prset->process(pr)` at the end of the callback.
    fn done(self: Arc<Self>) {
        let armed = self.reentry.lock().unwrap().clone();
        if let Some((done, prio)) = armed {
            let _ = done.complete(prio);
        }
    }

    /// `createAsynUser(devEurothermCallback, 0)`.
    fn timeout_user(&self) -> bool {
        false
    }
}

struct Live {
    job: Arc<Job>,
    cell: Arc<RequestCell>,
    parm: Parm,
}

pub struct DevEurotherm {
    kind: Kind,
    link: String,
    /// C `pr->name`.
    name: String,
    live: Option<Live>,
}

impl DevEurotherm {
    pub fn new(kind: Kind, link: &str) -> Self {
        Self {
            kind,
            link: link.to_owned(),
            name: String::new(),
            live: None,
        }
    }

    /// `writeAo` / `writeSo` with `pact == 0`: the frame. A format that does
    /// not take a double is undefined in C's `sprintf` and sends no payload.
    fn frame(&self, parm: &Parm, record: &dyn Record) -> Vec<u8> {
        match self.kind {
            Kind::Ao => {
                let value = match record.get_field("VAL") {
                    Some(EpicsValue::Double(v)) => v,
                    _ => 0.0,
                };
                let payload = match format_c_double(&parm.format, value) {
                    Ok(text) => text.into_bytes(),
                    Err(e) => match c_literal(&parm.format) {
                        Some(text) => text.into_bytes(),
                        None => {
                            if let Some(live) = &self.live {
                                print(
                                    &live.job.handle,
                                    TraceMask::ERROR,
                                    format_args!("devXxEurotherm {} {e}\n", self.name),
                                );
                            }
                            Vec::new()
                        }
                    },
                };
                protocol::value_frame(parm.group, parm.local, &payload, &parm.term)
            }
            Kind::So => {
                let val = match record.get_field("VAL") {
                    Some(EpicsValue::String(s)) => s.to_string().into_bytes(),
                    _ => Vec::new(),
                };
                let at = usize::try_from(parm.index).unwrap_or(usize::MAX);
                let payload = val.get(at..).unwrap_or_default();
                protocol::request_frame(parm.group, parm.local, payload, &parm.term)
            }
        }
    }
}

/// `asynPrint` on the record's asynUser, connected to `port` at address 0.
fn print(handle: &PortHandle, mask: TraceMask, args: std::fmt::Arguments<'_>) {
    handle
        .trace()
        .print(Some(handle.port_name()), Some(ADDR), 0, mask, args);
}

/// `initCommon`'s `bad:` arm after a failed asynManager call: the line its
/// `asynPrint` writes on the still unconnected asynUser, then `pact = 1`.
fn bad(name: &str, message: &str) -> DeviceInitOutcome {
    PortManager::global().trace_manager().print(
        None,
        None,
        0,
        TraceMask::ERROR,
        format_args!("{name} asynManager error {message}\n"),
    );
    DeviceInitOutcome::dead()
}

impl DeviceSupport for DevEurotherm {
    fn dtyp(&self) -> &str {
        match self.kind {
            Kind::Ao => DEVICES[0].1,
            Kind::So => DEVICES[1].1,
        }
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
                    "devXxEurotherm::initCommon {name} bad link {message}\n"
                ));
                return Ok(bad(&name, &message));
            }
        };
        let handle = match crate::connect_octet(&link.port_name, 0) {
            Ok(handle) => handle,
            Err(Unconnected::Device(message)) => return Ok(bad(&name, &message)),
            Err(Unconnected::NoOctet) => return Ok(DeviceInitOutcome::dead()),
        };
        let user_param = link.drv_info.as_str();
        let parm = Parm::parse(self.kind, user_param);
        let trace = |args: std::fmt::Arguments<'_>| print(&handle, TraceMask::IO_DEVICE, args);
        if !user_param.is_empty() {
            trace(format_args!(
                "XxEurotherm::initCommon, userParam = '{user_param}'\n"
            ));
            if after(user_param, "TERM=").is_some() {
                for byte in &parm.term {
                    trace(format_args!(
                        "devXxEurotherm:initCommon term[i] = 0x{byte:x}\n"
                    ));
                }
            }
        }
        let term = String::from_utf8_lossy(c_str(&parm.term)).into_owned();
        trace(format_args!("devXxEurotherm {name} term = '{term}'\n"));
        trace(format_args!("   bufferStartIndex = {}\n", parm.index));
        // C prints the double with `%d`; the seconds are what it meant.
        trace(format_args!("   timeout = {}\n", parm.timeout as i32));
        trace(format_args!("   format = {}\n", parm.format));
        trace(format_args!("   group address = {}\n", parm.group));
        trace(format_args!("   local address = {}\n", parm.local));
        let job = Arc::new(Job {
            timeout: parm.timeout,
            handle: handle.clone(),
            frame: Mutex::new(Vec::new()),
            reentry: Mutex::new(None),
        });
        let cell = handle.new_cell(job.clone());
        self.live = Some(Live { job, cell, parm });
        Ok(DeviceInitOutcome::ok())
    }

    fn read(&mut self, _record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        Ok(DeviceReadOutcome::ok())
    }

    /// `startIOCommon`. A refused `queueRequest` returns -1 with PACT already
    /// set, so the record stays active, as in C.
    fn write_begin(
        &mut self,
        record: &mut dyn Record,
        done: &DeviceDoneSource<'_>,
    ) -> CaResult<WriteStart> {
        let Some(live) = self.live.as_ref() else {
            return Ok(WriteStart::Completed);
        };
        *live.job.frame.lock().unwrap() = self.frame(&live.parm, record);
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
        Ok(WriteStart::Pending)
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::request::RequestResult;
    use epics_rs::base::server::records::ao::AoRecord;
    use epics_rs::base::server::records::stringout::StringoutRecord;

    use super::protocol::{EOT, STX};
    use super::*;

    fn live_dev(kind: Kind, user_param: &str, port: &str) -> DevEurotherm {
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
        let parm = Parm::parse(kind, user_param);
        let job = Arc::new(Job {
            timeout: parm.timeout,
            handle: handle.clone(),
            frame: Mutex::new(Vec::new()),
            reentry: Mutex::new(None),
        });
        let mut dev = DevEurotherm::new(kind, "");
        dev.live = Some(Live {
            cell: handle.new_cell(job.clone()),
            job,
            parm,
        });
        dev
    }

    fn init_traced(link: &str, name: &str) -> (DeviceInitOutcome, String) {
        use epics_rs::asyn::trace::TraceFile;

        let path = std::env::temp_dir().join(format!("eurotherm-{name}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        let trace = PortManager::global().trace_manager().clone();
        trace.set_trace_file(None, TraceFile::File(Arc::new(Mutex::new(file))));
        let mut dev = DevEurotherm::new(Kind::Ao, link);
        dev.set_record_info(name, ScanType::Passive);
        let outcome = dev.init(&mut AoRecord::default()).unwrap();
        trace.set_trace_file(None, TraceFile::Stderr);
        let log = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        (outcome, log)
    }

    /// `initCommon`'s `bad:` arm prints the failed `connectDevice`'s or
    /// `parseLink`'s errorMessage (devXxEurotherm.c:180-184) before
    /// `pact = 1`.
    #[test]
    fn a_failed_connect_or_link_prints_the_asyn_manager_error() {
        let (outcome, log) = init_traced("@asyn(EU_NO_SUCH_PORT)", "R:np");
        assert_eq!(outcome, DeviceInitOutcome::dead());
        assert!(
            log.trim_end().ends_with(
                "R:np asynManager error asynManager:connectDevice port EU_NO_SUCH_PORT not found"
            ),
            "{log:?}"
        );
        let message = match parse_asyn_link("not a link") {
            Err(AsynError::InvalidLinkSyntax(m)) => m,
            other => panic!("{other:?}"),
        };
        let (outcome, log) = init_traced("not a link", "R:bl");
        assert_eq!(outcome, DeviceInitOutcome::dead());
        assert!(
            log.trim_end()
                .ends_with(&format!("R:bl asynManager error {message}")),
            "{log:?}"
        );
    }

    /// A port without `asynOctet` fails `findInterface` with asynSuccess:
    /// `bad:` prints nothing and sets `pact = 1` (devXxEurotherm.c:110).
    #[test]
    fn a_port_without_octet_kills_the_record_silently() {
        crate::no_octet_port("EU_INT32_ONLY");
        let (outcome, log) = init_traced("@asyn(EU_INT32_ONLY)", "R:i32");
        assert_eq!(outcome, DeviceInitOutcome::dead());
        assert_eq!(log, "");
    }

    fn frame(dev: &DevEurotherm, record: &dyn Record) -> Vec<u8> {
        dev.frame(&dev.live.as_ref().unwrap().parm, record)
    }

    struct Wire(Vec<Vec<u8>>);

    impl PortCalls for Wire {
        fn call(&mut self, op: &RequestOp, _user: AsynUser) -> AsynResult<RequestResult> {
            if let RequestOp::OctetWrite { data } = op {
                self.0.push(data.clone());
            }
            Ok(RequestResult::write_ok())
        }
    }

    #[test]
    fn user_param_parses_as_init_common_does() {
        let p = Parm::parse(Kind::Ao, "FMT=SL%4.0lf,TERM=03");
        assert_eq!(
            (p.format.as_str(), p.term.as_slice()),
            ("SL%4.0lf", &[ETX][..])
        );
        assert_eq!((p.timeout, p.group, p.local), (3.0, 0, 0));
        let p = Parm::parse(Kind::So, "GAD=1,LAD=2,TO=0.5,IX=1");
        assert_eq!((p.format.as_str(), p.term.as_slice()), ("%s", &[ENQ][..]));
        assert_eq!((p.timeout, p.group, p.local, p.index), (0.5, 1, 2, 1));
    }

    /// `writeAo`: `EOT G G L L STX <sprintf> <TERM> <BCC>`, written by the
    /// queued callback.
    #[test]
    fn an_ao_writes_its_checksummed_frame() {
        let dev = live_dev(Kind::Ao, "FMT=SL%4.0lf,TERM=03,LAD=1", "EURO_AO");
        let mut ao = AoRecord::default();
        ao.val = 123.4;
        let want = protocol::value_frame(0, 1, b"SL 123", &[ETX]);
        assert_eq!(frame(&dev, &ao), want);
        assert_eq!(&want[..6], &[EOT, b'0', b'0', b'1', b'1', STX]);
        let live = dev.live.as_ref().unwrap();
        *live.job.frame.lock().unwrap() = want.clone();
        let mut wire = Wire(Vec::new());
        live.job.process(&mut wire);
        assert_eq!(wire.0, [want]);
    }

    /// `writeSo`: VAL from `IX=` after the preamble, then `ENQ`.
    #[test]
    fn a_stringout_writes_its_request_from_ix() {
        let dev = live_dev(Kind::So, "IX=1,GAD=2", "EURO_SO");
        let mut so = StringoutRecord::default();
        so.put_field("VAL", EpicsValue::String("XPV".into()))
            .unwrap();
        assert_eq!(
            frame(&dev, &so),
            [EOT, b'2', b'2', b'0', b'0', b'P', b'V', ENQ]
        );
    }

    #[test]
    fn the_factory_serves_ao_and_stringout() {
        let factory = device_support_factory();
        for (rtyp, dtyp) in DEVICES {
            let ctx = DeviceSupportContext {
                record_type: rtyp,
                dtyp,
                inp: "",
                out: "@asyn(P)",
            };
            assert!(factory(&ctx).is_some(), "{rtyp}");
        }
        let ctx = DeviceSupportContext {
            record_type: "ao",
            dtyp: "asyn so Eurotherm",
            inp: "",
            out: "",
        };
        assert!(factory(&ctx).is_none());
    }
}
