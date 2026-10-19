//! Driver test rigs: a recording serial line, where every write is kept and
//! a read answers the last command written through the test's `reply` (or
//! takes the next line it queued), and a count of
//! the process's `motorPoller` threads.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

type Reply = Box<dyn Fn(&str) -> String + Send + Sync>;
type Lines = Box<dyn Fn(&str) -> Vec<String> + Send + Sync>;
type Fails = Box<dyn Fn(&str) -> bool + Send + Sync>;

fn no() -> Fails {
    Box::new(|_| false)
}

struct Line {
    base: PortDriverBase,
    sent: Arc<Mutex<Vec<String>>>,
    /// The asyn address of each write in `sent`.
    addrs: Arc<Mutex<Vec<i32>>>,
    /// The last write that was more than a terminator: GCS2 writes a command
    /// and its `\n` separately, and the reply belongs to the command.
    command: String,
    answer: Answer,
    fails: Fails,
    /// Commands whose reply read times out after delivering the reply.
    cuts: Fails,
}

enum Answer {
    /// Every read answers the last command.
    Last(Reply),
    /// A command queues its lines; a read takes the next one and times out
    /// once none is left, as a real line does.
    Queued(Lines, VecDeque<String>),
}

impl PortDriver for Line {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let line = String::from_utf8_lossy(data).into_owned();
        self.addrs.lock().unwrap().push(user.addr);
        if !data.iter().all(|b| matches!(b, b'\r' | b'\n')) {
            self.command.clone_from(&line);
            if let Answer::Queued(lines, queue) = &mut self.answer {
                *queue = lines(&self.command).into();
            }
        }
        let failed = (self.fails)(&line);
        self.sent.lock().unwrap().push(line);
        if failed {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "LineRig: write failed".into(),
            });
        }
        Ok(data.len())
    }
    // A line takes any terminator, as a port with `asynInterposeEos` does, so
    // a driver that sets EOS on connect can be tested on it.
    fn set_input_eos(&mut self, user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
        self.base.set_input_eos(user.addr, eos);
        Ok(())
    }
    fn set_output_eos(&mut self, user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
        self.base.set_output_eos(user.addr, eos);
        Ok(())
    }
    fn get_input_eos(&self, user: &AsynUser) -> AsynResult<Vec<u8>> {
        Ok(self.base.input_eos(user.addr).to_vec())
    }
    fn get_output_eos(&self, user: &AsynUser) -> AsynResult<Vec<u8>> {
        Ok(self.base.output_eos(user.addr).to_vec())
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
        let reply = match &mut self.answer {
            Answer::Last(reply) => reply(&self.command),
            Answer::Queued(_, queue) => match queue.pop_front() {
                Some(reply) => reply,
                None => {
                    return OctetReadResult::failed(AsynError::Status {
                        status: AsynStatus::Timeout,
                        message: "LineRig: nothing to read".into(),
                    });
                }
            },
        };
        let n = reply.len().min(buf.len());
        buf[..n].copy_from_slice(&reply.as_bytes()[..n]);
        let read = OctetReadResult::ok(n, EomReason::empty());
        if (self.cuts)(&self.command) {
            return OctetReadResult {
                status: Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "LineRig: reply cut by a timeout".into(),
                }),
                ..read
            };
        }
        read
    }
}

/// The test's side of one port.
pub struct LineRig {
    sent: Arc<Mutex<Vec<String>>>,
    addrs: Arc<Mutex<Vec<i32>>>,
}

impl LineRig {
    /// A port named `name`, and the handle a driver talks to it through.
    /// `reply` maps the last command written, less a terminator written on
    /// its own, to what a read returns.
    pub fn new(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        Self::failing(name, reply, |_| false)
    }

    /// [`Self::new`], with each write `fails` picks recorded and then
    /// refused with an asyn error.
    pub fn failing(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
        fails: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        Self::open(
            name,
            1,
            Answer::Last(Box::new(reply)),
            Box::new(fails),
            no(),
        )
    }

    /// [`Self::new`], with the reply to each command `cuts` picks delivered
    /// by a read that then times out.
    pub fn cut(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
        cuts: impl Fn(&str) -> bool + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        Self::open(name, 1, Answer::Last(Box::new(reply)), no(), Box::new(cuts))
    }

    /// [`Self::new`] for a device that answers a command with `lines`, one
    /// per read, then sends nothing.
    pub fn queued(
        name: &str,
        lines: impl Fn(&str) -> Vec<String> + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        Self::open(
            name,
            1,
            Answer::Queued(Box::new(lines), VecDeque::new()),
            no(),
            no(),
        )
    }

    /// [`Self::new`] on a multi-device port of `max_addr` addresses, as a
    /// GPIB port is; [`Self::addresses`] gives where each write went.
    pub fn multi_device(
        name: &str,
        max_addr: usize,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        Self::open(name, max_addr, Answer::Last(Box::new(reply)), no(), no())
    }

    fn open(
        name: &str,
        max_addr: usize,
        answer: Answer,
        fails: Fails,
        cuts: Fails,
    ) -> (Self, SyncIOHandle) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let addrs = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(
                name,
                max_addr,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            sent: sent.clone(),
            addrs: addrs.clone(),
            command: String::new(),
            answer,
            fails,
            cuts,
        };
        let claim = claim_port(name).unwrap();
        let (rt, _jh) = create_port_runtime(&claim, line, RuntimeConfig::default()).unwrap();
        // Published, so a driver that looks the port up by name finds it.
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        (Self { sent, addrs }, io)
    }

    /// The messages written since the last call.
    pub fn sent(&self) -> Vec<String> {
        std::mem::take(&mut *self.sent.lock().unwrap())
    }

    /// The address of each write since the last call.
    pub fn addresses(&self) -> Vec<i32> {
        std::mem::take(&mut *self.addrs.lock().unwrap())
    }
}

/// The `motorPoller` threads of this process; nextest runs each test in its
/// own.
#[cfg(target_os = "linux")]
pub fn pollers() -> usize {
    std::fs::read_dir("/proc/self/task")
        .unwrap()
        .filter_map(|t| std::fs::read_to_string(t.ok()?.path().join("comm")).ok())
        .filter(|name| name.trim_end() == "motorPoller")
        .count()
}

/// Set in the child [`child_stdout`] runs.
const CHILD: &str = "MOTOR_TESTING_CHILD";

/// Whether this is the child [`child_stdout`] started.
pub fn in_child() -> bool {
    std::env::var_os(CHILD).is_some()
}

/// Run test `test` of `module` (its `module_path!()`) again as a child of
/// this binary and return its stdout, where a C `printf` at iocInit lands:
/// no iocsh redirect reaches it.
pub fn child_stdout(module: &str, test: &str) -> String {
    let module = module.split_once("::").map_or("", |(_, m)| m);
    let name = format!("{module}::{test}");
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", &name, "--nocapture", "--test-threads=1"])
        .env(CHILD, "1")
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).unwrap()
}

/// Run `cmd` with `args` as the shell does, on a thread of its own.
pub async fn run_command(
    cmd: epics_rs::base::server::iocsh::registry::CommandDef,
    args: Vec<epics_rs::base::server::iocsh::registry::ArgValue>,
) {
    call_command(cmd, args).await.unwrap();
}

/// [`run_command`], returning what the command returned: an `Err` is what
/// the shell would print for it.
pub async fn call_command(
    cmd: epics_rs::base::server::iocsh::registry::CommandDef,
    args: Vec<epics_rs::base::server::iocsh::registry::ArgValue>,
) -> epics_rs::base::server::iocsh::registry::CommandResult {
    use epics_rs::base::runtime::task::BlockingBridge;
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::iocsh::registry::CommandContext;

    let bridge = BlockingBridge::capture();
    off_runtime(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx)
    })
    .await
}

/// iocInit's drvSup init, where each model-1 driver's `motor_init` runs
/// over the cards its `Config` filled.
pub async fn drvsup_init() {
    use epics_rs::base::server::ioc_app::{InitHookState, init_hook_announce};
    off_runtime(|| init_hook_announce(InitHookState::AfterInitDrvSup)).await;
}

/// The errlog lines of iocInit from drvSup init through record init, with
/// the motor records of `db` loaded: what C's console shows for the cards
/// and their records.
pub async fn ioc_init_errlog(db: &str) -> Vec<String> {
    use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
    use epics_rs::base::server::ioc_builder::IocBuilder;
    use epics_rs::motor::devices::{device_support_factory, devices};

    let lines = Arc::new(Mutex::new(Vec::new()));
    let sink = lines.clone();
    let id = errlog_add_listener(move |m| sink.lock().unwrap().push(m.trim_end().to_owned()));
    drvsup_init().await;
    let (name, factory) = epics_rs::motor::motor_record_factory();
    let _db = IocBuilder::new()
        .register_record_type(name, factory)
        .register_dynamic_device_support(&devices(), device_support_factory())
        .db_string(db, &std::collections::HashMap::new())
        .unwrap()
        .build()
        .await
        .unwrap();
    errlog_flush();
    errlog_remove_listener(id);
    lines.lock().unwrap().clone()
}

/// Block until controller port `port`'s poller has ended its startup pass,
/// and every axis added to it running has had its first poll. Both run on
/// the poller's own thread as C's do (asynMotorController.cpp:630), so what
/// they set or trace is there to read once this returns. Call it off the
/// test's runtime.
pub fn await_first_polls(port: &str) {
    use epics_rs::motor::devices::first_polls_pending;
    await_zero(|| first_polls_pending(port).unwrap_or_else(|| panic!("no controller port {port}")));
}

/// [`await_first_polls`] for every controller in the process: what a shell
/// command started or added to, which the command does not name.
fn await_every_first_poll() {
    await_zero(epics_rs::motor::poll_loop::first_polls_pending_in_process);
}

fn await_zero(pending: impl Fn() -> usize) {
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    while pending() > 0 {
        assert!(
            std::time::Instant::now() < deadline,
            "a first poll never ran"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Run `f` on a thread of its own, awaited so this runtime keeps running
/// while it does.
async fn off_runtime<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    use epics_rs::base::runtime::task::sleep;
    let shell = std::thread::spawn(move || {
        let out = f();
        await_every_first_poll();
        out
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap()
}

/// Run `cmd` with `args` as [`run_command`], and return the `motorPoller`
/// count once the first has started.
#[cfg(target_os = "linux")]
pub async fn pollers_after(
    cmd: epics_rs::base::server::iocsh::registry::CommandDef,
    args: Vec<epics_rs::base::server::iocsh::registry::ArgValue>,
) -> usize {
    use epics_rs::base::runtime::task::sleep;

    run_command(cmd, args).await;
    for _ in 0..100 {
        if pollers() > 0 {
            break;
        }
        sleep(Duration::from_millis(10)).await;
    }
    sleep(Duration::from_millis(200)).await;
    pollers()
}

/// C `asynReport(level, port)`'s text for asyn port `port`.
pub fn port_report(port: &str, level: i32) -> String {
    epics_rs::asyn::manager::PortManager::global()
        .find_port_handle(port)
        .unwrap()
        .report_blocking(level)
        .unwrap()
}

/// C `dbior(driver, level)`'s text for the drvet `driver`, less its
/// `Driver: ` header.
pub fn driver_report(driver: &str, level: i32) -> String {
    let report = epics_rs::base::server::driver_support::find_driver_support(driver)
        .unwrap()
        .report(level)
        .unwrap();
    String::from_utf8(report).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A create command that starts the poller and then adds an axis to it,
    /// as a driver whose constructor runs its own task does.
    #[cfg(target_os = "linux")]
    #[epics_rs::base::epics_test]
    async fn an_axis_joining_a_started_poller_does_not_stall_the_helper() {
        use epics_rs::asyn::interfaces::motor::AsynMotor;
        use epics_rs::base::server::iocsh::registry::{
            ArgValue, CommandContext, CommandDef, CommandOutcome,
        };
        use epics_rs::motor::sim_motor::SimMotor;

        let senders = Arc::new(Mutex::new(Vec::new()));
        let kept = senders.clone();
        let create = CommandDef::new(
            "joinCreate",
            vec![],
            "joinCreate()",
            move |_: &[ArgValue], ctx: &CommandContext| {
                crate::register_port(ctx, "joinStarted", 1)?;
                let controller = crate::Controller::new();
                controller.start("joinStarted", 10, 100, 0);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(SimMotor::default()));
                let tx = controller.install(crate::asyn_user("joinStarted", 0), motor)?;
                kept.lock().unwrap().push(tx);
                Ok(CommandOutcome::Continue)
            },
        );
        assert_eq!(pollers_after(create, vec![]).await, 1);
        assert_eq!(senders.lock().unwrap().len(), 1);
    }

    #[test]
    fn the_rig_s_port_is_found_by_name() {
        let (rig, _io) = LineRig::new("lineRigByName", |_| "ok".into());
        let port = epics_rs::asyn::asyn_record::get_port("lineRigByName").expect("published");
        let io = SyncIOHandle::from_handle(port.handle.clone(), 0, Duration::from_secs(1));
        io.write_octet(0, b"hello").unwrap();
        assert_eq!(rig.sent(), ["hello"]);
    }

    #[test]
    fn a_terminator_written_on_its_own_keeps_the_command_s_reply() {
        let (rig, io) = LineRig::new("lineRigSplit", |cmd| match cmd {
            "POS? 1" => "1=0.5".into(),
            _ => String::new(),
        });
        io.write_octet(0, b"POS? 1").unwrap();
        io.write_octet(0, b"\n").unwrap();
        let (reply, status) = io.read_octet(0, 64).into_parts();
        status.unwrap();
        assert_eq!(reply, b"1=0.5");
        assert_eq!(rig.sent(), ["POS? 1", "\n"]);
    }

    #[test]
    fn the_rig_s_port_takes_the_eos_a_driver_sets() {
        use crate::connect::{EosRefusal, connect_with_eos};
        let (_rig, _io) = LineRig::new("lineRigEos", |_| String::new());
        let io = connect_with_eos(
            "lineRigEos",
            0,
            Duration::from_secs(1),
            b"\n",
            b"",
            EosRefusal::BothFatal,
        );
        assert!(io.is_ok());
        let port = epics_rs::asyn::asyn_record::get_port("lineRigEos").unwrap();
        let user = AsynUser::new(0).with_addr(0);
        assert_eq!(port.handle.get_input_eos_blocking(user).unwrap(), b"\n");
    }
}
