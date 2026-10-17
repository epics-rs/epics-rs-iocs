//! C `omsBaseController::omsPoller` (omsBaseController.cpp:584-844): one
//! thread per controller reads every axis at once — one command per quantity —
//! and writes the axes' status parameters. [`crate::oms::OmsAxis::poll`] only
//! reads those parameters back for the motor poll loops, which deliver them
//! to the records as `callParamCallbacks` does.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::MotorStatus;
use epics_rs::base::runtime::exit::at_exit;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::{StackSizeClass, ThreadPriority, spawn_dedicated_thread};

use motor_common::util::leading_hex;

use crate::oms::OmsController;

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
                has_encoder,
                ..MotorStatus::default()
            },
            move_delay: 0,
            invert_limit,
        });
    }

    pub(crate) fn status(&self, axis: usize) -> MotorStatus {
        self.axes()[axis].status.clone()
    }

    /// A move's `setIntegerParam(motorStatusDone_, 0)` and `wakeupPoller`
    /// (asynMotorController.cpp:300-357), whatever the move returned.
    pub(crate) fn moved(&self, axis: usize) {
        self.axes()[axis].status.done = false;
        self.wakeup();
    }

    /// `writeFloat64`'s `setDoubleParam(motorPosition_, value)` ahead of
    /// `setPosition`.
    pub(crate) fn set_position(&self, axis: usize, position: f64) {
        self.axes()[axis].status.position = position;
    }

    fn axes(&self) -> MutexGuard<'_, Vec<AxisState>> {
        self.axes.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// C `epicsEventSignal(pollEventId_)`.
    fn wakeup(&self) {
        *self.event.lock().unwrap_or_else(|e| e.into_inner()) = true;
        self.signal.notify_one();
    }

    /// C `waitInterruptible`: whether the event ended the wait.
    fn wait(&self, timeout: Duration) -> bool {
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
pub fn start(
    controller: &Arc<Mutex<OmsController>>,
    moving_poll: Duration,
    idle_poll: Duration,
    fw_minor: i32,
) {
    let poller = OmsPoller::new(controller.clone(), moving_poll, idle_poll, fw_minor);
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
        eprintln!("epicsThreadCreate: {e}");
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
            let Some(timeout) = self.pass() else {
                continue;
            };
            if self.shared.shutting_down() {
                break;
            }
            if self
                .shared
                .wait(timeout.saturating_sub(loop_start.elapsed()))
            {
                self.fast_polls = FORCED_FAST_POLLS;
            }
        }
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
            } else if velocities.is_some_and(|v| v[i] == 0) {
                if st.moving {
                    if flag(2) == Some(b'L') {
                        st.problem = false;
                        axis.move_delay = 0;
                        st.done = true;
                        st.moving = false;
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
                        }
                    }
                }
            } else {
                axis.move_delay = 0;
                st.problem = false;
                st.done = false;
                st.moving = true;
            }
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
        }
        drop(axes);

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

    /// C `getAxesArray`: one value per axis from a comma-separated reply.
    /// A failed read fails silently, a bad reply with a line.
    fn axes_array(&self, cmd: &str) -> Option<[i32; OMS_MAX_AXES]> {
        let reply = self.send_receive(cmd).ok()?;
        if reply.len() < self.num_axes.wrapping_sub(1) {
            errlog_vprintf(format_args!(
                "{DRIVER}:getAxesArray:{}: read string too short {}\n",
                self.port_name,
                reply.len()
            ));
            return None;
        }
        let (values, count) = parse_axes_array(&reply);
        if count != self.num_axes {
            errlog_vprintf(format_args!(
                "{DRIVER}:getAxesArray:{}: array string conversion error, count: {count}, axes: {}, input: >{reply}<\n",
                self.port_name, self.num_axes
            ));
            return None;
        }
        Some(values)
    }

    /// C `getClosedLoopStatus` in the `CL?` form this port assumes. C sets 1
    /// for a field that is *not* `on`; its status is the last field's.
    fn closed_loop_status(&mut self) -> bool {
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
        let Some((parsed, end)) = strtol(&b[start..]) else {
            break;
        };
        *value = parsed;
        count += 1;
        let end = start + end;
        if end >= b.len() {
            break;
        }
        start = end + 1;
    }
    (values, count)
}

/// C `strtol(s, &end, 10)` cast to `int`: the value and where it ended,
/// `None` when no digits were converted.
fn strtol(s: &[u8]) -> Option<(i32, usize)> {
    let mut at = s.iter().take_while(|c| c.is_ascii_whitespace()).count();
    let negative = s.get(at) == Some(&b'-');
    if matches!(s.get(at), Some(b'+' | b'-')) {
        at += 1;
    }
    let digits = s[at..].iter().take_while(|c| c.is_ascii_digit()).count();
    if digits == 0 {
        return None;
    }
    let magnitude = s[at..at + digits].iter().fold(0i64, |v, d| {
        v.saturating_mul(10).saturating_add(i64::from(d - b'0'))
    });
    let value = if negative { -magnitude } else { magnitude };
    Some((value as i32, at + digits))
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use epics_rs::asyn::exception::AsynException;
    use epics_rs::asyn::interfaces::motor::AsynMotor;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::port_handle::PortHandle;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::sync_io::SyncIOHandle;
    use epics_rs::asyn::user::AsynUser;

    use super::*;
    use crate::oms::OmsAxis;

    type Replies = Arc<Mutex<HashMap<String, String>>>;

    /// A controller answering each command from a table; records them all.
    struct Scripted {
        base: PortDriverBase,
        written: Arc<Mutex<Vec<String>>>,
        replies: Replies,
        reply: Vec<u8>,
    }

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
            self.reply = reply.unwrap_or_default().into_bytes();
            self.written.lock().unwrap().push(cmd);
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = std::mem::take(&mut self.reply);
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }

    struct Rig {
        controller: Arc<Mutex<OmsController>>,
        axes: Vec<OmsAxis>,
        port: PortHandle,
        written: Arc<Mutex<Vec<String>>>,
        replies: Replies,
    }

    /// A MAXnet on port `name` with an `=E`/`=h` X axis and an `=O`/`=l` Y
    /// axis, answering `replies` past their construction.
    fn rig(name: &str, axes: usize, replies: &[(&str, &str)]) -> Rig {
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
        let driver = Scripted {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            written: written.clone(),
            replies: replies.clone(),
            reply: Vec::new(),
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
        let x = rig.axes[0].poll(&user).unwrap();
        assert_eq!((x.done, x.moving, x.problem), (false, true, false));
        assert_eq!((x.position, x.encoder_position), (100.0, 7.0));
        assert!(x.slip_stall && x.direction && !x.home);
        assert!(!x.gain_support, "C sets 0 for an `on` field");
        assert!(!x.low_limit && !x.high_limit);

        let y = rig.axes[1].poll(&user).unwrap();
        assert_eq!((y.done, y.moving), (true, false));
        assert_eq!((y.position, y.encoder_position), (-5.0, 0.0));
        assert!(!y.direction && y.home && y.gain_support);
        assert!(y.low_limit && y.high_limit, "LT? =l inverts a clear flag");

        rig.axes[1]
            .move_absolute(&user, 10.0, 1.0, 100.0, 1000.0)
            .unwrap();
        assert!(!rig.axes[1].poll(&user).unwrap().done);
        assert!(
            rig.axes[1].poll(&user).unwrap().home,
            "untouched until the next loop"
        );
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
            rig.axes[0].poll(&AsynUser::default()).unwrap().position,
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
}
