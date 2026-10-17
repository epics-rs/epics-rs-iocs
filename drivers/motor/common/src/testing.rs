//! Driver test rigs: a recording serial line, where every write is kept and
//! a read answers the last command written through the test's `reply`, and a
//! count of
//! the process's `motorPoller` threads.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

type Reply = Box<dyn Fn(&str) -> String + Send + Sync>;

struct Line {
    base: PortDriverBase,
    sent: Arc<Mutex<Vec<String>>>,
    /// The last write that was more than a terminator: GCS2 writes a command
    /// and its `\n` separately, and the reply belongs to the command.
    command: String,
    reply: Reply,
}

impl PortDriver for Line {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let line = String::from_utf8_lossy(data).into_owned();
        if !data.iter().all(|b| matches!(b, b'\r' | b'\n')) {
            self.command.clone_from(&line);
        }
        self.sent.lock().unwrap().push(line);
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
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let reply = (self.reply)(&self.command);
        let n = reply.len().min(buf.len());
        buf[..n].copy_from_slice(&reply.as_bytes()[..n]);
        Ok(n)
    }
}

/// The test's side of one port.
pub struct LineRig {
    sent: Arc<Mutex<Vec<String>>>,
}

impl LineRig {
    /// A port named `name`, and the handle a driver talks to it through.
    /// `reply` maps the last command written, less a terminator written on
    /// its own, to what a read returns.
    pub fn new(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (Self, SyncIOHandle) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let line = Line {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            sent: sent.clone(),
            command: String::new(),
            reply: Box::new(reply),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _jh) = create_port_runtime(&claim, line, RuntimeConfig::default()).unwrap();
        // Published, so a driver that looks the port up by name finds it.
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        (Self { sent }, io)
    }

    /// The messages written since the last call.
    pub fn sent(&self) -> Vec<String> {
        std::mem::take(&mut *self.sent.lock().unwrap())
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

/// Run `cmd` with `args` as the shell does, on a thread of its own.
pub async fn run_command(
    cmd: epics_rs::base::server::iocsh::registry::CommandDef,
    args: Vec<epics_rs::base::server::iocsh::registry::ArgValue>,
) {
    use epics_rs::base::runtime::task::{BlockingBridge, sleep};
    use epics_rs::base::server::database::PvDatabase;
    use epics_rs::base::server::iocsh::registry::CommandContext;

    let bridge = BlockingBridge::capture();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
    });
    // Awaited, not joined: an axis joining a started poller waits on it, and
    // the poller's thread is spawned by a task this runtime has to run.
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap();
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
    epics_rs::base::server::driver_support::find_driver_support(driver)
        .unwrap()
        .report(level)
        .unwrap()
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
                controller.start(ctx, 10, 100, 0);
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
        assert_eq!(io.read_octet(0, 64).unwrap(), b"1=0.5");
        assert_eq!(rig.sent(), ["POS? 1", "\n"]);
    }

    #[test]
    fn the_rig_s_port_takes_the_eos_a_driver_sets() {
        use crate::connect::{EosRefusal, connect_with_eos};
        let (_rig, _io) = LineRig::new("lineRigEos", |_| String::new());
        let io = connect_with_eos(
            "lineRigEos",
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
