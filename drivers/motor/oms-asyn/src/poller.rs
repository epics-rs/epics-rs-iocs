//! C `omsBaseController::omsPoller` (omsBaseController.cpp:584-844): one
//! thread per controller reads every axis at once — one command per quantity —
//! writes the axes' status parameters and posts each axis, as its
//! `callParamCallbacks` does, through the axis's motor poll loop.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::MotorStatus;
use epics_rs::base::runtime::exit::at_exit;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::sync::mpsc;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};
use epics_rs::motor::poll_loop::{FirstPollPending, PollCommand};

use motor_common::util::{leading_hex, long_to_int, strtol};

use crate::oms::{AXIS_CHARS, OmsController};

/// C `driverName`.
const DRIVER: &str = "omsBaseDriver";

/// C `OMS_MAX_AXES`.
const OMS_MAX_AXES: usize = 10;

/// C `STATUSSTRINGLEN`: an axis's `"MDNN,"` in the `RI` reply.
const STATUS_LEN: usize = 5;

/// The `forcedFastPolls` every Config command passes to `startPoller`.
const FORCED_FAST_POLLS: u32 = 10;

/// An axis's status parameters and the omsBaseAxis members the poller keeps.
struct AxisState {
    status: MotorStatus,
    /// C `moveDelay`.
    move_delay: i32,
    /// C `invertLimit`, from `LT?`.
    invert_limit: bool,
    /// C `homing`: a home was sent and the axis has not read done since.
    homing: bool,
}

/// What the poller shares with the axes: their status parameters, guarded as
/// C's port lock guards the parameter library, and the poll event
/// (`pollEventId_`).
#[derive(Default)]
pub struct PollShared {
    axes: Mutex<Vec<AxisState>>,
    event: Mutex<bool>,
    signal: Condvar,
    /// C `shuttingDown_`.
    shutting_down: AtomicBool,
}

impl PollShared {
    /// Init's parameters for the next axis (omsBaseController.cpp:519-545);
    /// axes are added in order, as Init creates them.
    pub(crate) fn add_axis(&self, index: usize, has_encoder: bool, invert_limit: bool) {
        let mut axes = self.axes();
        debug_assert_eq!(axes.len(), index);
        axes.push(AxisState {
            status: MotorStatus {
                done: true,
                has_encoder,
                ..MotorStatus::default()
            },
            move_delay: 0,
            invert_limit,
            homing: false,
        });
    }

    pub(crate) fn status(&self, axis: usize) -> MotorStatus {
        self.axes()[axis].status.clone()
    }

    /// A move's `setIntegerParam(motorStatusDone_, 0)`
    /// (asynMotorController.cpp:300-357), whatever the move returned. Its
    /// `wakeupPoller` is the motor poll loop's pass after the move, whose
    /// `OmsAxis::poll` signals; a second signal here
    /// ran a second controller read per move.
    pub(crate) fn moved(&self, axis: usize) {
        self.axes()[axis].status.done = false;
    }

    /// C home()'s `homing = 1`.
    pub(crate) fn homed(&self, axis: usize) {
        self.axes()[axis].homing = true;
    }

    /// C `homing`, for the report.
    pub fn homing(&self, axis: usize) -> bool {
        self.axes()[axis].homing
    }

    /// C `numAxes` once Init has created the axes.
    pub fn axis_count(&self) -> usize {
        self.axes().len()
    }

    /// `writeFloat64`'s `setDoubleParam(motorPosition_, value)` ahead of
    /// `setPosition`.
    pub(crate) fn set_position(&self, axis: usize, position: f64) {
        self.axes()[axis].status.position = position;
    }

    /// `writeFloat64`'s `setDoubleParam(motorEncoderPosition_, value)`,
    /// ahead of its encoder check (omsBaseController.cpp:329-333).
    pub(crate) fn set_encoder_position(&self, axis: usize, position: f64) {
        self.axes()[axis].status.encoder_position = position;
    }

    fn axes(&self) -> MutexGuard<'_, Vec<AxisState>> {
        self.axes.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `epicsEventSignal(pollEventId_)`.
    pub(crate) fn wakeup(&self) {
        *self.event.lock().unwrap_or_else(|e| e.into_inner()) = true;
        self.signal.notify_one();
    }

    /// C `waitInterruptible`: whether the event ended the wait.
    pub(crate) fn wait(&self, timeout: Duration) -> bool {
        let set = self.event.lock().unwrap_or_else(|e| e.into_inner());
        let (mut set, _) = self
            .signal
            .wait_timeout_while(set, timeout, |set| !*set)
            .unwrap_or_else(|e| e.into_inner());
        std::mem::take(&mut *set)
    }

    fn shutting_down(&self) -> bool {
        self.shutting_down.load(Ordering::Acquire)
    }
}

/// C `startPoller` and the constructor's `epicsAtExit(callShutdown)`.
/// `posters` are the axes' poll loops, in axis order; `first_pass` counts
/// the poller's first pass on the axes' controller.
pub fn start(
    controller: &Arc<Mutex<OmsController>>,
    posters: Vec<mpsc::Sender<PollCommand>>,
    first_pass: FirstPollPending,
    moving_poll: Duration,
    idle_poll: Duration,
    fw_minor: i32,
) {
    let mut poller = OmsPoller::new(controller.clone(), moving_poll, idle_poll, fw_minor);
    poller.posters = posters;
    poller.first_pass = Some(first_pass);
    // C creates the event with the controller; the axes' startup polls
    // signalled this one before the poller existed.
    poller.shared.wait(Duration::ZERO);
    let shared = poller.shared.clone();
    at_exit(format!("omsPoller {}", poller.port_name), move || {
        shared.shutting_down.store(true, Ordering::Release)
    });
    let name = format!("OMSPoller-{}", poller.lock().controller_number());
    if let Err(e) = spawn_dedicated_thread(
        name,
        ThreadPriority::Low,
        StackSizeClass::Medium,
        move || poller.run(),
    ) {
        epics_rs::base::stderr_println!("epicsThreadCreate: {e}");
    }
}

/// C `omsPoller`'s loop state.
struct OmsPoller {
    controller: Arc<Mutex<OmsController>>,
    shared: Arc<PollShared>,
    port_name: String,
    num_axes: usize,
    /// Any axis has an encoder, as the poller reads once at its start.
    use_encoder: bool,
    /// C `fwMinor`, for `watchdogOK`.
    fw_minor: i32,
    moving_poll: Duration,
    idle_poll: Duration,
    loop_break_count: u32,
    any_moving: bool,
    fast_polls: u32,
    /// C `enabled`, cleared by a failed watchdog check.
    enabled: bool,
    /// C `closedLoopStatus`, an array that keeps its values across loops.
    closed_loop: [bool; OMS_MAX_AXES],
    /// Each axis's poll loop, which posts its status on change.
    posters: Vec<mpsc::Sender<PollCommand>>,
    /// Held until the first pass ends, for readers after the create command.
    first_pass: Option<FirstPollPending>,
}

impl OmsPoller {
    fn new(
        controller: Arc<Mutex<OmsController>>,
        moving_poll: Duration,
        idle_poll: Duration,
        fw_minor: i32,
    ) -> Self {
        let (shared, port_name) = {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            (ctrl.poll_shared().clone(), ctrl.port_name().to_owned())
        };
        let (num_axes, use_encoder) = {
            let axes = shared.axes();
            let use_encoder = axes.iter().any(|a| a.status.has_encoder);
            (axes.len(), use_encoder)
        };
        Self {
            controller,
            shared,
            port_name,
            num_axes,
            use_encoder,
            fw_minor,
            moving_poll,
            idle_poll,
            loop_break_count: 0,
            any_moving: false,
            fast_polls: 0,
            enabled: true,
            closed_loop: [false; OMS_MAX_AXES],
            posters: Vec::new(),
            first_pass: None,
        }
    }

    fn lock(&self) -> MutexGuard<'_, OmsController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn send_receive(&self, cmd: &str) -> AsynResult<String> {
        self.lock().send_receive(cmd)
    }

    fn run(mut self) {
        loop {
            if self.shared.shutting_down() {
                break;
            }
            if self.loop_break_count > 10 {
                errlog_vprintf(format_args!(
                    "{DRIVER}:BasePoller:{}: Error: {} consecutive unsuccessful attempts to read from motor card\n",
                    self.port_name, self.loop_break_count
                ));
                break;
            }
            let loop_start = Instant::now();
            let pass = self.pass();
            self.first_pass = None;
            let Some(timeout) = pass else {
                continue;
            };
            if self.shared.shutting_down() {
                break;
            }
            let remaining = timeout.saturating_sub(loop_start.elapsed());
            if self.wait_interruptible(remaining, timeout == self.idle_poll) {
                self.fast_polls = FORCED_FAST_POLLS;
            }
        }
    }

    /// C `waitInterruptible` (omsMAXnet.cpp:168-206, omsMXA.cpp:163-201):
    /// the wait in slices, each after a read that lets a pending
    /// notification signal the event. C takes the idle slice on
    /// `timeout == idlePollPeriod_` with the time left, which no loop that
    /// took any time meets; here the slice follows the period chosen.
    fn wait_interruptible(&self, timeout: Duration, idle: bool) -> bool {
        let slice = if idle {
            self.idle_poll / 5
        } else {
            self.moving_poll / 20
        };
        self.lock().flush_line();
        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.enabled {
                self.lock().poll_line();
            }
            if self.shared.wait(slice) {
                return true;
            }
        }
        false
    }

    /// One loop up to its wait (omsBaseController.cpp:636-815): `None` where
    /// C `continue`s on a failed read, else the period C waits out.
    fn pass(&mut self) -> Option<Duration> {
        // C's retry loop evaluates getAxesStatus before `retry_count < 5`, so
        // a sixth read goes out and the loop fails whatever it answers.
        let mut retry_count = 0;
        let mut status = self.axes_status();
        while status.is_none() && retry_count < 5 {
            std::thread::sleep(Duration::from_millis(100));
            retry_count += 1;
            status = self.axes_status();
        }
        let Some(status) = status.filter(|_| retry_count <= 4) else {
            errlog_vprintf(format_args!(
                "{DRIVER}:BasePoller:{}: error reading axis status ({retry_count} attempts)\n",
                self.port_name
            ));
            self.loop_break_count += 1;
            self.lock().reset_connection();
            return None;
        };
        let Some(positions) = self.axes_array("AM PP;") else {
            self.loop_break_count += 1;
            return None;
        };
        let encoders = if self.use_encoder {
            let Some(encoders) = self.axes_array("AM PE;") else {
                self.loop_break_count += 1;
                return None;
            };
            encoders
        } else {
            [0; OMS_MAX_AXES]
        };
        self.loop_break_count = 0;

        let have_closed_loop = !self.any_moving && self.closed_loop_status();
        let velocities = self.axes_array("AM;RV;");
        let encoder_status = self.send_receive("AM;EA;").ok();
        let limits = self
            .send_receive("AM;QL;")
            .ok()
            .and_then(|s| leading_hex(s.trim_start()));
        if self.enabled {
            self.watchdog_ok();
        }

        self.any_moving = false;
        let status = status.as_bytes();
        let mut posts = Vec::with_capacity(self.num_axes);
        let mut axes = self.shared.axes();
        for (i, axis) in axes.iter_mut().enumerate() {
            if !self.enabled || self.shared.shutting_down() {
                break;
            }
            let flag = |k: usize| status.get(i * STATUS_LEN + k).copied();
            let st = &mut axis.status;

            if self.use_encoder && st.has_encoder {
                if let Some(buffer) = &encoder_status {
                    match substring(buffer, i) {
                        Some(field) => st.slip_stall = field.as_bytes().get(2) == Some(&b'S'),
                        None => errlog_vprintf(format_args!(
                            "{DRIVER}:BasePoller:{}: error parsing encoder status string {buffer}\n",
                            self.port_name
                        )),
                    }
                }
                st.encoder_position = f64::from(encoders[i]);
            }

            if flag(1) == Some(b'D') {
                st.problem = false;
                axis.move_delay = 0;
                st.done = true;
                st.moving = false;
                axis.homing = false;
            } else if velocities.is_some_and(|v| v[i] == 0) {
                if st.moving {
                    if flag(2) == Some(b'L') {
                        st.problem = false;
                        axis.move_delay = 0;
                        st.done = true;
                        st.moving = false;
                        axis.homing = false;
                        if flag(0) == Some(b'P') {
                            st.high_limit = true;
                        } else {
                            st.low_limit = true;
                        }
                    } else {
                        axis.move_delay += 1;
                        if axis.move_delay >= 5 {
                            st.done = true;
                            st.moving = false;
                            st.problem = true;
                            axis.move_delay = 0;
                            axis.homing = false;
                        }
                    }
                }
            } else {
                axis.move_delay = 0;
                st.problem = false;
                st.done = false;
                st.moving = true;
            }
            // One C param, motorStatusMoving_, is both the pace and the bit.
            st.msta_moving = st.moving;
            self.any_moving |= st.moving;

            if let Some(flags) = limits {
                let offset = if i > 7 { 8 } else { 0 };
                st.low_limit = (flags & (1 << (i + offset)) != 0) ^ axis.invert_limit;
                st.high_limit = (flags & (1 << (i + offset + 8)) != 0) ^ axis.invert_limit;
            }
            st.home = flag(3) == Some(b'H');
            st.direction = flag(0) == Some(b'P');
            st.position = f64::from(positions[i]);
            if have_closed_loop {
                st.gain_support = self.closed_loop[i];
            }
            posts.push(st.clone());
        }
        drop(axes);
        // `callParamCallbacks`, sent outside the parameter lock: a poll loop
        // reading the parameters must not hold up a full channel.
        for (poster, status) in self.posters.iter().zip(posts) {
            let _ = poster.blocking_send(PollCommand::Post(status));
        }

        Some(if self.fast_polls > 0 {
            self.fast_polls -= 1;
            self.moving_poll
        } else if self.any_moving {
            self.moving_poll
        } else {
            self.idle_poll
        })
    }

    /// C `getAxesStatus`: `RI` for all axes, failed unless it starts with a
    /// direction and covers every axis. C's `numAxes * 5 - 1` is compared
    /// unsigned, so no axes fails every read.
    fn axes_status(&self) -> Option<String> {
        let status = self.send_receive("AM;RI;").ok()?;
        let needed = (self.num_axes * STATUS_LEN).wrapping_sub(1);
        ((status.starts_with('P') || status.starts_with('M')) && status.len() >= needed)
            .then_some(status)
    }

    fn axes_array(&self, cmd: &str) -> Option<[i32; OMS_MAX_AXES]> {
        axes_array(&self.lock(), self.num_axes, cmd)
    }

    /// C `getClosedLoopStatus` (omsBaseController.cpp:852-891): one `CL?`
    /// for all axes, or before firmware 1.30 a `?PM` per axis. C sets 1
    /// for a reply that is *not* `on`; its status is the last axis's.
    fn closed_loop_status(&mut self) -> bool {
        if !self.lock().modern() {
            let mut found = true;
            for (i, ch) in AXIS_CHARS.iter().take(self.num_axes).enumerate() {
                found = match self.send_receive(&format!("A{ch} ?PM")) {
                    Ok(reply) => {
                        self.closed_loop[i] = !reply.starts_with("=on");
                        true
                    }
                    Err(_) => false,
                };
            }
            return found;
        }
        let Ok(reply) = self.send_receive("AM;CL?;") else {
            return false;
        };
        let mut found = true;
        for i in 0..self.num_axes {
            found = match substring(&reply, i) {
                Some(field) => {
                    self.closed_loop[i] = !field.starts_with("on");
                    true
                }
                None => false,
            };
        }
        found
    }

    /// C `watchdogOK`; `useWatchdog` is set for both MAXnet and MXA. C reads
    /// the reply buffer whatever the read returned.
    fn watchdog_ok(&mut self) {
        if self.fw_minor < 33 {
            return;
        }
        let (reply, _) = self.lock().send_receive_buffer("#WS");
        let reply = reply.as_bytes();
        if reply.first() == Some(&b'=') && reply.get(1) != Some(&b'0') {
            errlog_vprintf(format_args!(
                "{DRIVER}:watchdogOK:{}: *** CAUTION watchdog not running, disabling card ***\n",
                self.port_name
            ));
            self.enabled = false;
        }
    }
}

/// C `getAxesArray`: one value per axis from a comma-separated reply.
/// A failed read fails silently, a bad reply with a line.
fn axes_array(ctrl: &OmsController, num_axes: usize, cmd: &str) -> Option<[i32; OMS_MAX_AXES]> {
    let reply = ctrl.send_receive(cmd).ok()?;
    if reply.len() < num_axes.wrapping_sub(1) {
        errlog_vprintf(format_args!(
            "{DRIVER}:getAxesArray:{}: read string too short {}\n",
            ctrl.port_name(),
            reply.len()
        ));
        return None;
    }
    let (values, count) = parse_axes_array(&reply);
    if count != num_axes {
        errlog_vprintf(format_args!(
            "{DRIVER}:getAxesArray:{}: array string conversion error, count: {count}, axes: {num_axes}, input: >{reply}<\n",
            ctrl.port_name()
        ));
        return None;
    }
    Some(values)
}

/// The end of C `Init` (omsBaseController.cpp:565-568): every axis's
/// position from one `AM PP;`, so a record initialised before the poller's
/// first good loop starts from the controller, not from 0.
pub(crate) fn init_positions(controller: &Mutex<OmsController>) {
    let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
    let shared = ctrl.poll_shared();
    let num_axes = shared.axes().len();
    if let Some(positions) = axes_array(&ctrl, num_axes, "AM PP;") {
        for (axis, position) in positions.iter().take(num_axes).enumerate() {
            shared.set_position(axis, f64::from(*position));
        }
    }
}

/// C `getSubstring`: field `number` of a comma-separated string, empty
/// fields included.
fn substring(buffer: &str, number: usize) -> Option<&str> {
    if buffer.len() < number {
        return None;
    }
    buffer.split(',').nth(number)
}

/// `getAxesArray`'s parse: the values and C's `count`, which counts a
/// trailing comma as one more empty field.
fn parse_axes_array(reply: &str) -> ([i32; OMS_MAX_AXES], usize) {
    let b = reply.as_bytes();
    let mut values = [0; OMS_MAX_AXES];
    let (mut start, mut count) = (0, 0);
    for value in values.iter_mut() {
        if b.get(start) == Some(&b',') {
            count += 1;
            start += 1;
            if start >= b.len() {
                count += 1;
                break;
            }
            continue;
        }
        let (parsed, end) = strtol(&b[start..], 10);
        if end == 0 {
            break;
        }
        *value = long_to_int(parsed);
        count += 1;
        let end = start + end;
        if end >= b.len() {
            break;
        }
        start = end + 1;
    }
    (values, count)
}

#[cfg(test)]
pub(crate) mod tests {
    use std::collections::HashMap;

    use epics_rs::asyn::exception::AsynException;
    use epics_rs::asyn::interfaces::motor::AsynMotor;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::port_handle::PortHandle;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::sync_io::SyncIOHandle;
    use epics_rs::asyn::user::AsynUser;

    use std::sync::atomic::AtomicUsize;

    use super::*;
    use crate::oms::OmsAxis;
    use motor_common::testing::Polled;

    type Replies = Arc<Mutex<HashMap<String, String>>>;

    /// A controller answering each command from a table, then the
    /// `unsolicited` lines in order; records the commands. A `|` in a reply
    /// separates the lines the controller sends after it.
    struct Scripted {
        base: PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        replies: Replies,
        reply: Vec<u8>,
        unsolicited: Unsolicited,
        reads: Arc<AtomicUsize>,
    }

    type Unsolicited = Arc<Mutex<std::collections::VecDeque<String>>>;

    impl PortDriver for Scripted {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let cmd = String::from_utf8_lossy(data).into_owned();
            let reply = self.replies.lock().unwrap().get(&cmd).cloned();
            let reply = reply.unwrap_or_default();
            let mut lines = reply.split('|');
            self.reply = lines.next().unwrap_or_default().as_bytes().to_vec();
            let mut unsolicited = self.unsolicited.lock().unwrap();
            unsolicited.extend(lines.map(str::to_owned));
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            self.reads.fetch_add(1, Ordering::Relaxed);
            let mut reply = std::mem::take(&mut self.reply);
            if reply.is_empty() {
                let next = self.unsolicited.lock().unwrap().pop_front();
                reply = next.unwrap_or_default().into_bytes();
            }
            buf[..reply.len()].copy_from_slice(&reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
        fn io_flush(&mut self, _user: &mut AsynUser) -> AsynResult<()> {
            self.reply.clear();
            self.unsolicited.lock().unwrap().clear();
            Ok(())
        }
    }

    pub(crate) struct Rig {
        pub(crate) controller: Arc<Mutex<OmsController>>,
        pub(crate) axes: Vec<OmsAxis>,
        port: PortHandle,
        pub(crate) written: Arc<Mutex<Vec<String>>>,
        replies: Replies,
        unsolicited: Unsolicited,
        reads: Arc<AtomicUsize>,
    }

    /// A MAXnet on port `name` with an `=E`/`=h` X axis and an `=O`/`=l` Y
    /// axis, answering `replies` past their construction.
    pub(crate) fn rig(name: &str, axes: usize, replies: &[(&str, &str)]) -> Rig {
        let written = Arc::new(Mutex::new(Vec::new()));
        let table: HashMap<String, String> = [
            ("AX PS?", "=E"),
            ("AX LT?", "=h"),
            ("AY PS?", "=O"),
            ("AY LT?", "=l"),
        ]
        .iter()
        .chain(replies)
        .map(|(c, r)| (c.to_string(), r.to_string()))
        .collect();
        let replies = Arc::new(Mutex::new(table));
        let unsolicited = Unsolicited::default();
        let reads = Arc::new(AtomicUsize::new(0));
        let driver = Scripted {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            written: written.clone(),
            replies: replies.clone(),
            reply: Vec::new(),
            unsolicited: unsolicited.clone(),
            reads: reads.clone(),
        };
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let port = rt.port_handle().clone();
        let io = SyncIOHandle::from_handle(port.clone(), 0, Duration::from_millis(100));
        let mut controller = OmsController::new(io, name, name, "MAXnet");
        controller.watch_connection().unwrap();
        let controller = Arc::new(Mutex::new(controller));
        let axes = (0..axes)
            .map(|i| OmsAxis::new(controller.clone(), i).unwrap())
            .collect();
        written.lock().unwrap().clear();
        Rig {
            controller,
            axes,
            port,
            written,
            replies,
            unsolicited,
            reads,
        }
    }

    fn poller(rig: &Rig) -> OmsPoller {
        OmsPoller::new(
            rig.controller.clone(),
            Duration::from_millis(100),
            Duration::from_secs(1),
            33,
        )
    }

    const HEALTHY: [(&str, &str); 8] = [
        ("AM;RI;", "PNNN,MDLH"),
        ("AM PP;", "100,-5"),
        ("AM PE;", "7,0"),
        ("AM;CL?;", "on,of"),
        ("AM;RV;", "20,0"),
        ("AM;EA;", "ENS,EN"),
        ("AM;QL;", "0"),
        ("#WS", "=0"),
    ];

    /// One read of each quantity serves both axes; their status is what
    /// the poller loop makes of it.
    #[test]
    fn one_read_per_quantity_serves_every_axis() {
        let mut rig = rig("omsPollerHealthy", 2, &HEALTHY);
        let mut poller = poller(&rig);
        assert_eq!(poller.pass(), Some(Duration::from_millis(100)));
        assert_eq!(
            *rig.written.lock().unwrap(),
            HEALTHY.map(|(cmd, _)| cmd.to_string())
        );

        let user = AsynUser::default();
        let x = rig.axes[0].polled(&user).unwrap();
        assert_eq!(
            (x.done, x.moving, x.msta_moving, x.problem),
            (false, true, true, false)
        );
        assert_eq!((x.position, x.encoder_position), (100.0, 7.0));
        assert!(x.slip_stall && x.direction && !x.home);
        assert!(!x.gain_support, "C sets 0 for an `on` field");
        assert!(!x.low_limit && !x.high_limit);

        let y = rig.axes[1].polled(&user).unwrap();
        assert_eq!((y.done, y.moving, y.msta_moving), (true, false, false));
        assert_eq!((y.position, y.encoder_position), (-5.0, 0.0));
        assert!(!y.direction && y.home && y.gain_support);
        assert!(y.low_limit && y.high_limit, "LT? =l inverts a clear flag");

        rig.axes[1]
            .move_absolute(&user, 10.0, 1.0, 100.0, 1000.0)
            .unwrap();
        assert!(!rig.axes[1].polled(&user).unwrap().done);
        assert!(
            rig.axes[1].polled(&user).unwrap().home,
            "untouched until the next loop"
        );
    }

    /// Each axis is posted at the end of the loop; a status poll wakes the
    /// poller as C `omsBaseAxis::poll` does.
    #[test]
    fn the_loop_posts_every_axis_and_a_poll_wakes_it() {
        let mut rig = rig("omsPollerPosts", 2, &HEALTHY);
        let mut poller = poller(&rig);
        let (tx, mut rx): (Vec<_>, Vec<_>) = (0..2).map(|_| mpsc::channel(4)).unzip();
        poller.posters = tx;
        poller.pass();
        for (i, rx) in rx.iter_mut().enumerate() {
            let Ok(PollCommand::Post(status)) = rx.try_recv() else {
                panic!("axis {i} not posted");
            };
            assert_eq!(status, rig.axes[i].polled(&AsynUser::default()).unwrap());
            assert!(rx.try_recv().is_err());
        }
        assert!(poller.shared.wait(Duration::ZERO));
        assert!(!poller.shared.wait(Duration::ZERO));
    }

    /// A notification reaching the line while the poller waits ends the
    /// wait, as C's zero-length reads feeding `asynCallback` do
    /// (omsMAXnet.cpp:55-73, 187-200).
    #[test]
    fn a_notification_ends_the_poller_wait() {
        let rig = rig("omsWaitNotified", 1, &HEALTHY);
        let poller = poller(&rig);
        let line = rig.unsolicited.clone();
        let notify = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            line.lock().unwrap().push_back("%000 00000001".into());
        });
        let start = Instant::now();
        assert!(poller.wait_interruptible(Duration::from_secs(2), false));
        assert!(start.elapsed() < Duration::from_secs(1));
        notify.join().unwrap();
    }

    /// An idle loop waits in idlePollPeriod/5 slices. C picks that slice
    /// only when the time left equals the idle period, which a loop that
    /// took any time never meets, so it waits in movingPollPeriod/20 slices
    /// (omsMAXnet.cpp:178-181); here the slice follows the period chosen.
    #[test]
    fn an_idle_loop_waits_in_idle_slices() {
        let mut done = HEALTHY.to_vec();
        done.push(("AM;RI;", "PDNN,MDNN"));
        let rig = rig("omsWaitIdleSlice", 2, &done);
        let poller = poller(&rig);
        let shared = poller.shared.clone();
        let run = std::thread::spawn(move || poller.run());
        std::thread::sleep(Duration::from_millis(1500));
        let before = rig.reads.load(Ordering::Relaxed);
        std::thread::sleep(Duration::from_secs(1));
        let reads = rig.reads.load(Ordering::Relaxed) - before;
        shared.shutting_down.store(true, Ordering::Release);
        shared.wakeup();
        run.join().unwrap();
        // Five slice reads and one loop's reads; C's slices make ~200.
        assert!(reads < 40, "{reads} reads in an idle second");
    }

    /// The wait starts with a flush, so a line already waiting is dropped
    /// unseen (omsMAXnet.cpp:184-186).
    #[test]
    fn a_line_waiting_before_the_wait_is_flushed() {
        let rig = rig("omsWaitFlushed", 1, &HEALTHY);
        rig.unsolicited
            .lock()
            .unwrap()
            .push_back("%000 00000001".into());
        assert!(!poller(&rig).wait_interruptible(Duration::from_millis(100), false));
    }

    /// A notification ahead of a command's reply is skipped and signals the
    /// poll event, C sendReceive's `isNotification` loop
    /// (omsMAXnet.cpp:279-295, 310-327).
    #[test]
    fn a_notification_ahead_of_a_reply_is_skipped_and_wakes_the_poller() {
        let rig = rig("omsReplyNotified", 1, &[("AM;QL;", "%000 00000001|5")]);
        let ctrl = rig.controller.lock().unwrap();
        assert!(!ctrl.poll_shared().wait(Duration::ZERO));
        assert_eq!(ctrl.send_receive("AM;QL;").unwrap(), "5");
        assert!(ctrl.poll_shared().wait(Duration::ZERO));
    }

    /// `"000 01"` is the controller's command error, which C prints and
    /// skips without signalling (omsMAXnet.cpp:316-318).
    #[test]
    fn a_command_error_notification_is_skipped_without_a_wake() {
        let rig = rig("omsReplyCmdErr", 1, &[("AM;QL;", "%000 01000000|5")]);
        let ctrl = rig.controller.lock().unwrap();
        assert_eq!(ctrl.send_receive("AM;QL;").unwrap(), "5");
        assert!(!ctrl.poll_shared().wait(Duration::ZERO));
    }

    /// Init's `AM PP;` positions stand until the poller's first good loop.
    #[test]
    fn init_reads_every_axis_position() {
        let mut rig = rig("omsInitPositions", 2, &[("AM PP;", "3000,-5")]);
        init_positions(&rig.controller);
        assert_eq!(*rig.written.lock().unwrap(), ["AM PP;"]);
        let user = AsynUser::default();
        assert_eq!(rig.axes[0].polled(&user).unwrap().position, 3000.0);
        assert_eq!(rig.axes[1].polled(&user).unwrap().position, -5.0);
    }

    /// Init's DONE=1 stands through a loop that finds an axis at rest with
    /// no done flag, as one reads after power-up.
    #[test]
    fn an_axis_at_rest_without_a_done_flag_keeps_init_s_done() {
        let mut idle = HEALTHY.to_vec();
        idle.extend([("AM;RI;", "PNNN,MNNN"), ("AM;RV;", "0,0")]);
        let mut rig = rig("omsInitDone", 2, &idle);
        poller(&rig).pass();
        let user = AsynUser::default();
        for axis in &mut rig.axes {
            assert!(axis.polled(&user).unwrap().done);
        }
    }

    /// writeFloat64 sets REP ahead of its encoder check: it stands on an
    /// axis with no encoder, and on one with until the loop reads it.
    #[test]
    fn an_encoder_position_write_sets_the_encoder_position() {
        let mut rig = rig("omsSetEncoder", 2, &HEALTHY);
        let user = AsynUser::default();
        for axis in &mut rig.axes {
            axis.set_encoder_position(&user, 42.0).unwrap();
            assert_eq!(axis.polled(&user).unwrap().encoder_position, 42.0);
        }
        assert_eq!(*rig.written.lock().unwrap(), ["AX LPE42;"]);
        poller(&rig).pass();
        assert_eq!(rig.axes[0].polled(&user).unwrap().encoder_position, 7.0);
        assert_eq!(rig.axes[1].polled(&user).unwrap().encoder_position, 42.0);
    }

    /// C `homing`: set by home(), cleared by the loop that finds the axis
    /// done (omsBaseAxis.cpp:150, omsBaseController.cpp:737).
    #[test]
    fn a_home_holds_homing_until_the_loop_finds_the_axis_done() {
        let mut rig = rig("omsPollerHoming", 2, &HEALTHY);
        let shared = rig.controller.lock().unwrap().poll_shared().clone();
        let user = AsynUser::default();
        for axis in &mut rig.axes {
            axis.home(&user, 1.0, 100.0, 1000.0, true).unwrap();
        }
        assert!(shared.homing(0) && shared.homing(1));
        poller(&rig).pass();
        assert!(shared.homing(0), "X is still moving");
        assert!(!shared.homing(1), "Y reads done");
    }

    /// A move signals the poller once, through the poll that follows it.
    #[test]
    fn a_move_wakes_the_poller_only_through_its_poll() {
        let mut rig = rig("omsPollerMoveWake", 1, &HEALTHY);
        let shared = rig.controller.lock().unwrap().poll_shared().clone();
        let user = AsynUser::default();
        rig.axes[0]
            .move_absolute(&user, 10.0, 1.0, 100.0, 1000.0)
            .unwrap();
        assert!(!shared.wait(Duration::ZERO));
        rig.axes[0].polled(&user).unwrap();
        assert!(shared.wait(Duration::ZERO));
    }

    /// A status that never comes: six `RI` reads for the whole controller,
    /// then one resetConnection, which with autoConnect off disconnects and
    /// connects the serial port.
    #[test]
    fn a_dead_status_read_resets_the_connection_once_per_loop() {
        let rig = rig("omsPollerDeadStatus", 2, &[("AM;RI;", "garbage")]);
        rig.port.set_auto_connect(-1, false).unwrap();
        let seen = Arc::new(Mutex::new(0));
        let heard = seen.clone();
        let _g = rig.port.exception_callback_add(0, move |ev| {
            if ev.exception == AsynException::Connect {
                *heard.lock().unwrap() += 1;
            }
        });
        let mut poller = poller(&rig);
        assert_eq!(poller.pass(), None);
        assert_eq!(*rig.written.lock().unwrap(), ["AM;RI;"; 6]);
        assert_eq!(*seen.lock().unwrap(), 2);
        assert_eq!(poller.loop_break_count, 1);
        assert!(rig.controller.lock().unwrap().port_connected());
    }

    /// The poller ends after eleven loops in a row fail, here on the
    /// position read.
    #[test]
    fn the_poller_stops_after_eleven_failed_loops() {
        let rig = rig("omsPollerBreak", 1, &[("AM;RI;", "PDNN,")]);
        poller(&rig).run();
        let written = rig.written.lock().unwrap();
        assert_eq!(written.iter().filter(|c| *c == "AM PP;").count(), 11);
        assert_eq!(written.len(), 22);
    }

    /// A watchdog that is not running disables the card: the reads go on,
    /// the axes keep their status.
    #[test]
    fn a_stopped_watchdog_disables_the_card() {
        let mut rig = rig("omsPollerWatchdog", 2, &HEALTHY);
        rig.replies
            .lock()
            .unwrap()
            .insert("#WS".into(), "=1".into());
        let mut poller = poller(&rig);
        assert_eq!(poller.pass(), Some(Duration::from_secs(1)));
        assert!(!poller.enabled);
        assert_eq!(
            rig.axes[0].polled(&AsynUser::default()).unwrap().position,
            0.0
        );
        rig.written.lock().unwrap().clear();
        poller.pass();
        assert!(!rig.written.lock().unwrap().iter().any(|c| c == "#WS"));
    }

    #[test]
    fn axes_arrays_count_as_get_axes_array_does() {
        assert_eq!(parse_axes_array("0,5000,0").1, 3);
        assert_eq!(parse_axes_array("0,5000,0").0[1], 5000);
        assert_eq!(parse_axes_array(",,,").1, 4);
        assert_eq!(parse_axes_array(",,,,").1, 5);
        assert_eq!(parse_axes_array("0").1, 1);
        assert_eq!(parse_axes_array("5,").1, 1);
        assert_eq!(parse_axes_array("-7, 3").0[..2], [-7, 3]);
        assert_eq!(parse_axes_array("").1, 0);
    }

    /// omsBaseController.cpp:1051: `int intVal = strtol(..)` of an
    /// overflow is `(int)LONG_MIN` and `(int)LONG_MAX`, 0 and -1 on LP64.
    #[test]
    fn an_overflowing_axis_value_is_strtol_cast_to_int() {
        use std::ffi::c_long;
        let (min, max) = (long_to_int(c_long::MIN), long_to_int(c_long::MAX));
        let (values, count) = parse_axes_array("-99999999999999999999,99999999999999999999");
        assert_eq!((values[..2].to_vec(), count), (vec![min, max], 2));
        let mut at_min = [0; OMS_MAX_AXES];
        at_min[0] = min;
        assert_eq!(parse_axes_array(&c_long::MIN.to_string()), (at_min, 1));
        // strtol skips a `\v` as isspace.
        assert_eq!(parse_axes_array("\x0b5").0[0], 5);
    }
}
