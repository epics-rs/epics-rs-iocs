//! Animatics SmartMotor integrated servo controller driver (serial ASCII).
//!
//! Ported from `motorSmartMotor/smartMotorApp/src/drvSmartMotor.cc` +
//! `devSmartMotor.cc`. A SmartMotor is an integrated servo + drive; commands
//! use the Animatics language (`P=`, `D=`, `V=`, `A=`, `G`, `S`, `RP`, `RBt`,
//! …) and are terminated by the port output EOS `\n`; replies by input EOS
//! `\r` (`SmartMotorConfig` sets both, as C `motor_init`). Set commands produce no reply;
//! status is read back with `R*` query commands.
//!
//! ## Wire modes
//!
//! [`SmartMotorController::new`] auto-detects the mode by probing `RBe`, as C
//! `motor_init`:
//!
//! - **Single motor (no echo):** commands are sent verbatim and queries reply
//!   terminated by `\r`; one axis.
//! - **Daisy chain (echo on):** each command is prefixed with the binary
//!   address byte `129 + axis` and the chain echoes it terminated by `\n`
//!   before the `\r`-terminated response. Every transaction switches the port
//!   input EOS to `\n` to discard the echo and back to `\r` (C `send_mess`),
//!   and axes are counted up to the first that does not answer `RBe`.
//!
//! ## Units
//!
//! The SmartMotor works natively in encoder counts (`RP` returns counts, `P=`
//! takes counts) with no resolution scaling, so the asyn-rs motor boundary is
//! counts: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is counts. Velocity and acceleration go out times C's
//! `VCONFAC` / `ACONFAC`, per PID sample.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `R*` reads run inside
//!   [`poll`](AsynMotor::poll).
//! - `HOME_FOR`/`HOME_REV` and the PID-gain commands are unsupported in C
//!   (`build_trans` sends nothing and returns `ERROR`); [`home`](AsynMotor::home)
//!   returns an error and [`set_pid_gain`](AsynMotor::set_pid_gain) is a no-op.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, Model1Motor, MotorStatus, PidGainKind, PollFailure,
};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;

use motor_common::model1::{init_poll, init_read};
use motor_common::util::{atof, atoi, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 20;

/// Read timeout (C `recv_mess` 0.25 s); writes use the caller's timeout.
const RECV_TIMEOUT: Duration = Duration::from_millis(250);

/// Response terminator (C `input_terminator`).
const INPUT_EOS: &[u8] = b"\r";

/// Echo terminator in daisy-chain mode (C `echo_terminator`).
const ECHO_EOS: &[u8] = b"\n";

/// Command terminator (C `output_terminator`).
const OUTPUT_EOS: &[u8] = b"\n";

/// Most axes on one chain (C `MAX_AXIS`).
const MAX_AXIS: usize = 32;

/// Address byte of axis 0 in daisy-chain mode (C `SmartMotor_addr[0]`).
const FIRST_ADDR: u8 = 129;

/// Counts/s to the `V=` scaled counts per PID sample (C `VCONFAC`).
const VCONFAC: f64 = 16.1063;

/// Counts/s² to the `A=` scaled counts per PID sample² (C `ACONFAC`).
const ACONFAC: f64 = 3.958322E-3;

/// Minimum acceleration the controller accepts (C forces `A >= 2`).
const MIN_ACCEL: i32 = 2;

/// Scale and round an acceleration to the controller value, clamped to the minimum the
/// SmartMotor accepts (C overrides `A <= 1` to `2`, saying so once per process).
fn clamped_accel(acceleration: f64) -> i32 {
    static INVALID_ACCMSG_LATCH: AtomicBool = AtomicBool::new(false);
    let cacc = nint(acceleration * ACONFAC);
    if cacc > 1 {
        return cacc;
    }
    if !INVALID_ACCMSG_LATCH.swap(true, Ordering::Relaxed) {
        // errPrintf(-1, ...): no errSym name, then a space for the nonzero
        // status. The file and line are ours.
        errlog_vprintf(format_args!(
            " filename=\"{}\" line number={} Overriding invalid acceleration; A < 2.\n",
            file!(),
            line!()
        ));
    }
    MIN_ACCEL
}

fn smartmotor_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial port for one SmartMotor or a
/// daisy chain of them.
pub struct SmartMotorController {
    writer: SyncIOHandle,
    reader: SyncIOHandle,
    /// Daisy-chain echo mode (C `num_motors != 1` in `send_mess`).
    echo: bool,
    num_axes: usize,
    comm_state: CommState,
}

/// Controller communication state (C `cntrl->status`): one failed `RBt` is
/// retried silently, a second in a row is a comm error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

impl SmartMotorController {
    /// Connect and count the motors (C `motor_init`): set the port EOS, probe
    /// `RBe` on axis 0 bare, switch to echo mode if the command comes back
    /// echoed, then probe each axis address up to the first that does not
    /// answer `0`/`1` in three tries. Performs blocking serial I/O.
    pub fn new(port: PortHandle, timeout: Duration) -> AsynResult<Self> {
        // C ignores every setInputEos/setOutputEos status
        // (drvSmartMotor.cc:449-454, 598-601).
        let writer = SyncIOHandle::from_handle(port.clone(), 0, timeout);
        let _ = writer.set_output_eos(0, OUTPUT_EOS);
        let _ = writer.set_input_eos(0, INPUT_EOS);
        let mut ctrl = Self {
            writer,
            reader: SyncIOHandle::from_handle(port, 0, RECV_TIMEOUT),
            echo: false,
            num_axes: 0,
            comm_state: CommState::Normal,
        };
        ctrl.flush();

        let mut total = 0;
        while total < MAX_AXIS {
            let mut answered = false;
            let mut retry = 0;
            while !answered && retry < 3 {
                let reply = ctrl.query(total, "RBe").unwrap_or_default();
                if total == 0 && !ctrl.echo && reply.starts_with("RBe") {
                    // Echo ON: start over with addressed commands.
                    ctrl.echo = true;
                    retry = 0;
                    ctrl.flush();
                }
                answered = reply.starts_with(['0', '1']);
                retry += 1;
            }
            if !answered {
                break;
            }
            total += 1;
            if !ctrl.echo {
                break;
            }
        }
        if total == 0 {
            return Err(smartmotor_err("SmartMotor: no valid response to RBe probe"));
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// Motors found by [`new`](Self::new): 1 without echo, else the chain length.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn flush(&self) {
        let _ = self.writer.flush(0);
    }

    /// Write one command to `axis` (C `send_mess`). In echo mode it carries the
    /// axis address byte and the echo is read to `\n` and discarded. Like C
    /// (drvSmartMotor.cc:441-454) no status is kept: a lost write shows up
    /// in the reply read that follows, and the input EOS is restored to `\r`
    /// either way.
    fn send(&self, axis: usize, cmd: &str) {
        self.send_bytes(axis, cmd.as_bytes());
    }

    fn send_bytes(&self, axis: usize, cmd: &[u8]) {
        if self.echo {
            let mut out = Vec::with_capacity(cmd.len() + 1);
            out.push(FIRST_ADDR + axis as u8);
            out.extend_from_slice(cmd);
            let _ = self.writer.write_octet(0, &out);
            let _ = self.writer.set_input_eos(0, ECHO_EOS);
            let _ = self.reader.read_octet(0, READ_BUF);
        } else {
            let _ = self.writer.write_octet(0, cmd);
        }
        let _ = self.writer.set_input_eos(0, INPUT_EOS);
    }

    /// Send a query to `axis` and return its `\r`-terminated reply (trimmed).
    fn query(&self, axis: usize, cmd: &str) -> AsynResult<String> {
        self.send(axis, cmd);
        let (raw, status) = self.reader.read_octet(0, READ_BUF).into_parts();
        status?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text.trim_matches(['\r', '\n', '\0', ' ']).to_string())
    }

    /// [`query`](Self::query) whose failed read is an empty reply, as C
    /// `recv_mess` leaves the buffer for the `atoi`/`atof` after it.
    fn reply(&self, axis: usize, cmd: &str) -> String {
        self.query(axis, cmd).unwrap_or_default()
    }
}

/// One SmartMotor on the controller's port. Implements [`AsynMotor`].
pub struct SmartMotorAxis {
    controller: Arc<Mutex<SmartMotorController>>,
    axis: usize,
    /// C `motor_info->position`, updated only when `RP` moves.
    prev_position: i32,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed `RBt` (RETRY, drvSmartMotor.cc:252), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next (COMM_ERR, :258). Per axis, as the
    /// controller's state can move on under another axis's poll.
    failure: PollFailure,
    last_status: MotorStatus,
    /// What `motor_init`'s `set_status` read, until the motor task takes it.
    init_status: Option<AsynResult<MotorStatus>>,
}

impl SmartMotorAxis {
    /// Construct axis `axis` (0-based chain position) of a controller and
    /// read its status, as `motor_init`'s `set_status` does
    /// (drvSmartMotor.cc:660).
    pub fn new(controller: Arc<Mutex<SmartMotorController>>, axis: usize) -> Self {
        let mut me = Self {
            controller,
            axis,
            prev_position: 0,
            failure: PollFailure::NONE,
            last_status: MotorStatus {
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
            init_status: None,
        };
        me.init_status = Some(init_poll(&mut me, &AsynUser::new(0)));
        me
    }

    fn lock(&self) -> MutexGuard<'_, SmartMotorController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program speed/acceleration before a move (C sends `V=`/`A=` as separate
    /// transactions ahead of the move command). The record skips SET_ACCEL
    /// for a zero acceleration (motorRecord.cc:940); one sent is clamped.
    fn program_speeds(ctrl: &SmartMotorController, axis: usize, velocity: f64, acceleration: f64) {
        ctrl.send(axis, &format!("V={}", nint(velocity * VCONFAC)));
        if acceleration > 0.0 {
            ctrl.send(axis, &format!("A={}", clamped_accel(acceleration)));
        }
    }
}

impl Model1Motor for SmartMotorAxis {
    fn take_init_status(&mut self) -> MotorStatus {
        let read = self.init_status.take().expect("taken once");
        init_read(read, self.last_status.clone(), self.poll_failure())
    }
}

impl AsynMotor for SmartMotorAxis {
    fn poll_failure(&self) -> PollFailure {
        self.failure
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, self.axis, velocity, acceleration);
        ctrl.send(self.axis, &format!("P={}", nint(position)));
        ctrl.send(self.axis, "G");
        Ok(())
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        SmartMotorAxis::program_speeds(&ctrl, self.axis, velocity, acceleration);
        ctrl.send(self.axis, &format!("D={}", nint(distance)));
        ctrl.send(self.axis, "G");
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        // The record's unguarded SET_ACCEL (motorRecord.cc:2141), then C JOG:
        // "MV\rV=<v>\rG" — velocity mode, set signed velocity, go.
        let ctrl = self.lock();
        ctrl.send(self.axis, &format!("A={}", clamped_accel(acceleration)));
        ctrl.send(self.axis, &format!("MV\rV={}\rG", nint(velocity * VCONFAC)));
        Ok(())
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        // C: Animatics SmartMotors do not use home positions (build_trans ERROR).
        // C still sends the record's SET_VELOCITY V= ahead of that ERROR; not copied.
        Err(smartmotor_err("SmartMotor: homing is not supported"))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send(self.axis, "S");
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.send(self.axis, &format!("O={}", nint(position)));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        if enable {
            // C ENABLE_TORQUE: position mode, hold at the current actual position.
            ctrl.send(self.axis, "MP\ra=@P\rP=a\rG");
        } else {
            // C DISABL_TORQUE: de-energize.
            ctrl.send(self.axis, "OFF");
        }
        Ok(())
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // C: SET_PGAIN/IGAIN/DGAIN send nothing and return ERROR — no-op here.
        Ok(())
    }

    /// `PRIMITIVE`: " " + INIT as its own message (devSmartMotor.cc:211-215).
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        let mut msg = Vec::with_capacity(init.len() + 1);
        msg.push(b' ');
        msg.extend_from_slice(init);
        self.lock().send_bytes(self.axis, &msg);
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser, status: &mut MotorStatus) -> AsynResult<()> {
        *status = self.poll_status(None)?;
        Ok(())
    }

    /// drvSmartMotor.cc:287-291: a moving motor reports the record's CDIR.
    /// The :384 write back to `mr->cdir` is dead, as the direction is already
    /// CDIR here.
    fn poll_in_motion(
        &mut self,
        _user: &AsynUser,
        cdir: &mut bool,
        status: &mut MotorStatus,
    ) -> AsynResult<()> {
        *status = self.poll_status(Some(*cdir))?;
        Ok(())
    }
}

impl SmartMotorAxis {
    /// C `set_status`; `cdir` is the record's while its move is held.
    fn poll_status(&mut self, cdir: Option<bool>) -> AsynResult<MotorStatus> {
        let controller = self.controller.clone();
        let mut ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());

        // Trajectory-busy flag: RBt == 0 means the move is done. C
        // drvSmartMotor.cc:239-263: an empty or failed RBt fails the poll
        // raising nothing once (RETRY); a second in a row raises COMM_ERR.
        let rbt = ctrl.query(self.axis, "RBt").ok().filter(|r| !r.is_empty());
        let Some(rbt) = rbt else {
            (ctrl.comm_state, self.failure) = match ctrl.comm_state {
                CommState::Normal => (CommState::Retry, PollFailure::NONE),
                _ => (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR),
            };
            return Err(smartmotor_err(format!(
                "SmartMotor axis {}: no RBt reply",
                self.axis
            )));
        };
        ctrl.comm_state = CommState::Normal;
        let done = atoi(&rbt) == 0;

        let rp = atof(&ctrl.reply(self.axis, "RP"));

        let plus_ls = atoi(&ctrl.reply(self.axis, "RBp")) != 0;
        let minus_ls = atoi(&ctrl.reply(self.axis, "RBm")) != 0;

        // Latched right/left travel-limit flags: clear them if set (C Zr/Zl).
        if atoi(&ctrl.reply(self.axis, "RBr")) != 0 {
            ctrl.send(self.axis, "Zr");
        }
        if atoi(&ctrl.reply(self.axis, "RBl")) != 0 {
            ctrl.send(self.axis, "Zl");
        }

        // RBo != 0 means "not on commanded position" (C clears EA_POSITION).
        let on_position = atoi(&ctrl.reply(self.axis, "RBo")) == 0;

        // C motor_init sets encoder_present, so the encoder is a second RP,
        // truncated by its (epicsInt32) cast.
        let encoder_position = atof(&ctrl.reply(self.axis, "RP")) as i32;

        let velocity_raw = atoi(&ctrl.reply(self.axis, "RV"));
        drop(ctrl);

        // drvSmartMotor.cc:271-291: an unchanged RP keeps the direction, a
        // moving motor reports the record's.
        let mut direction = self.last_status.direction;
        if rp != f64::from(self.prev_position) {
            let position = nint(rp);
            direction = position >= self.prev_position;
            self.prev_position = position;
        }
        if let Some(cdir) = cdir {
            direction = cdir;
        }
        let position = self.prev_position;

        let velocity = if direction {
            velocity_raw as f64
        } else {
            -(velocity_raw as f64)
        };

        let status = MotorStatus {
            position: position as f64,
            encoder_position: f64::from(encoder_position),
            velocity,
            done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            direction,
            powered: on_position,
            has_encoder: true,
            gain_support: true,
            ..MotorStatus::default()
        };
        self.last_status = status.clone();
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_common::testing::Polled;

    #[test]
    fn accel_clamps_to_minimum() {
        assert_eq!(clamped_accel(1.0 / ACONFAC), 2); // C overrides A <= 1 to 2
        assert_eq!(clamped_accel(0.0), 2);
        assert_eq!(clamped_accel(-5.0), 2);
        assert_eq!(clamped_accel(2.4 / ACONFAC), 2); // NINT rounds down
        assert_eq!(clamped_accel(2.6 / ACONFAC), 3); // NINT rounds up
        assert_eq!(clamped_accel(10_000.0), 40);
    }

    /// A daisy chain of one echoing SmartMotor on a port with no EOS support
    /// of its own unless `takes_eos`, so every EOS set is refused.
    struct Chain {
        base: epics_rs::asyn::port::PortDriverBase,
        takes_eos: bool,
        replies: std::collections::VecDeque<Vec<u8>>,
        refuse_writes: Arc<std::sync::atomic::AtomicBool>,
        silent: Arc<std::sync::atomic::AtomicBool>,
        reads: Arc<std::sync::atomic::AtomicUsize>,
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    fn chain(name: &str) -> Chain {
        Chain {
            base: epics_rs::asyn::port::PortDriverBase::new(
                name,
                1,
                epics_rs::asyn::port::PortFlags::default(),
                epics_rs::asyn::port::InterfaceMask::UNMIGRATED_ALL,
                epics_rs::asyn::port::InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            takes_eos: false,
            replies: Default::default(),
            refuse_writes: Default::default(),
            silent: Default::default(),
            reads: Default::default(),
            sent: Default::default(),
        }
    }

    impl epics_rs::asyn::port::PortDriver for Chain {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn set_input_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            match self.takes_eos {
                true => Ok(()),
                false => Err(smartmotor_err("no EOS support")),
            }
        }
        fn set_output_eos(&mut self, user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
            self.set_input_eos(user, eos)
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            if self
                .refuse_writes
                .load(std::sync::atomic::Ordering::Relaxed)
            {
                return Err(smartmotor_err("write refused"));
            }
            self.sent.lock().unwrap().push(data.to_vec());
            if self.silent.load(std::sync::atomic::Ordering::Relaxed) {
                return Ok(data.len());
            }
            self.replies.push_back(data.to_vec());
            if data.first() == Some(&FIRST_ADDR) {
                self.replies.push_back(b"1".to_vec());
            }
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            self.reads
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let reply = self.replies.pop_front().unwrap_or_default();
            buf[..reply.len()].copy_from_slice(&reply);
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    #[test]
    fn a_port_refusing_every_eos_set_still_finds_the_chain() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let (rt, _actor) = create_port_runtime(
            &claim_port("smartEosRefused").unwrap(),
            chain("smartEosRefused"),
            RuntimeConfig::default(),
        )
        .unwrap();
        let port = rt.port_handle().clone();
        assert!(
            port.set_input_eos_blocking(AsynUser::new(0), INPUT_EOS)
                .is_err()
        );
        let ctrl = SmartMotorController::new(port, Duration::from_secs(1)).unwrap();
        assert!(ctrl.echo);
        assert_eq!(ctrl.num_axes(), 1);
    }

    /// drvSmartMotor.cc:598-604, :622, :449-453: every flush and EOS set is
    /// `pasynOctetSyncIO`'s, which prints its TRACEIO_DEVICE line.
    #[test]
    fn init_flushes_and_sets_the_eos_as_asyn_octet_sync_io() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use epics_rs::asyn::trace::{TraceFile, TraceMask};

        let name = "smartSyncIo";
        let mut chain = chain(name);
        chain.takes_eos = true;
        let (rt, _actor) =
            create_port_runtime(&claim_port(name).unwrap(), chain, RuntimeConfig::default())
                .unwrap();
        let trace = rt.port_handle().trace();
        let path = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
        trace.set_trace_file(
            Some(name),
            TraceFile::File(Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()))),
        );
        trace.set_trace_mask(Some(name), TraceMask::IO_DEVICE);
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(ctrl.echo);
        // The init flush and the echo restart's.
        assert_eq!(text.matches("asynOctetSyncIO flush\n").count(), 2, "{text}");
        assert!(
            text.contains("asynOctetSyncIO setOutputEos eoslen 1\n"),
            "{text}"
        );
        assert!(
            text.contains("asynOctetSyncIO setInputEos eoslen 1\n"),
            "{text}"
        );
    }

    #[test]
    fn a_refused_write_still_reads_the_echo_and_the_command_succeeds() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use std::sync::atomic::Ordering;

        let chain = chain("smartWriteRefused");
        let refuse = chain.refuse_writes.clone();
        let reads = chain.reads.clone();
        let (rt, _actor) = create_port_runtime(
            &claim_port("smartWriteRefused").unwrap(),
            chain,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let mut axis = SmartMotorAxis::new(Arc::new(Mutex::new(ctrl)), 0);

        refuse.store(true, Ordering::Relaxed);
        let before = reads.load(Ordering::Relaxed);
        axis.stop(&AsynUser::new(0), 0.0).unwrap();
        assert_eq!(reads.load(Ordering::Relaxed), before + 1, "the echo read");
    }

    #[test]
    fn init_goes_out_addressed_after_a_space() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let chain = chain("smartInit");
        let sent = chain.sent.clone();
        let (rt, _actor) = create_port_runtime(
            &claim_port("smartInit").unwrap(),
            chain,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let mut axis = SmartMotorAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        sent.lock().unwrap().clear();

        axis.primitive(&AsynUser::new(0), b"EIGN(2)").unwrap();
        assert_eq!(*sent.lock().unwrap(), vec![b"\x81 EIGN(2)".to_vec()]);
    }

    /// drvSmartMotor.cc:239-263: a first missed `RBt` is RETRY (nothing
    /// raised, nothing posted), the next COMM_ERR, and a reply clears it.
    #[test]
    fn a_missed_rbt_is_retry_then_comm_error() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use std::sync::atomic::Ordering;

        let chain = chain("smartRetry");
        let silent = chain.silent.clone();
        let (rt, _actor) = create_port_runtime(
            &claim_port("smartRetry").unwrap(),
            chain,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let mut axis = SmartMotorAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        let user = AsynUser::new(0);

        silent.store(true, Ordering::Relaxed);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        silent.store(false, Ordering::Relaxed);
        let s = axis.polled(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        silent.store(true, Ordering::Relaxed);
        assert!(axis.polled(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }

    /// drvSmartMotor.cc:248-262 exit stores what motor_init prepared
    /// (:648-658, GAIN_SUPPORT and EA_PRESENT), with both COMM_ERR bits once
    /// past RETRY.
    #[test]
    fn a_failed_init_read_keeps_the_encoder_bits() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
        use std::sync::atomic::Ordering;

        let chain = chain("smartInitFail");
        let silent = chain.silent.clone();
        let (rt, _actor) = create_port_runtime(
            &claim_port("smartInitFail").unwrap(),
            chain,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let ctrl = Arc::new(Mutex::new(ctrl));
        silent.store(true, Ordering::Relaxed);
        let mut first = SmartMotorAxis::new(ctrl.clone(), 0);
        let mut second = SmartMotorAxis::new(ctrl, 1);
        let prepared = MotorStatus {
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        assert_eq!(first.take_init_status(), prepared);
        assert_eq!(
            second.take_init_status(),
            MotorStatus {
                problem: true,
                comms_error: true,
                ..prepared
            }
        );
    }

    /// devSmartMotor.cc:190-191: V= and A= are counts/s times VCONFAC and
    /// counts/s² times ACONFAC; a jog's V= too.
    #[test]
    fn speeds_scale_per_pid_sample() {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let chain = chain("smartScale");
        let sent = chain.sent.clone();
        let (rt, _actor) = create_port_runtime(
            &claim_port("smartScale").unwrap(),
            chain,
            RuntimeConfig::default(),
        )
        .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        let mut axis = SmartMotorAxis::new(Arc::new(Mutex::new(ctrl)), 0);
        let user = AsynUser::new(0);
        sent.lock().unwrap().clear();

        axis.move_absolute(&user, 100.0, 0.0, 1000.0, 10_000.0)
            .unwrap();
        axis.move_velocity(&user, 0.0, -1000.0, 0.0).unwrap();
        // devSmartMotor.cc:249-251: a zero velocity still goes out; the
        // record sends no zero acceleration.
        axis.move_relative(&user, 5.0, 0.0, 0.0, 0.0).unwrap();
        let sent: Vec<_> = sent.lock().unwrap().drain(..).collect();
        assert_eq!(
            sent,
            [
                &b"\x81V=16106"[..],
                b"\x81A=40",
                b"\x81P=100",
                b"\x81G",
                b"\x81A=2",
                b"\x81MV\rV=-16106\rG",
                b"\x81V=0",
                b"\x81D=5",
                b"\x81G",
            ]
        );
    }

    /// One SmartMotor without echo answering each query from a table, or
    /// nothing at all while `quiet`.
    struct Single {
        base: epics_rs::asyn::port::PortDriverBase,
        last: String,
        replies: Arc<Mutex<std::collections::HashMap<&'static str, &'static str>>>,
        quiet: Arc<std::sync::atomic::AtomicBool>,
    }

    impl epics_rs::asyn::port::PortDriver for Single {
        fn base(&self) -> &epics_rs::asyn::port::PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut epics_rs::asyn::port::PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.last = String::from_utf8_lossy(data).into_owned();
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let reply = if self.quiet.load(std::sync::atomic::Ordering::SeqCst) {
                ""
            } else {
                let replies = self.replies.lock().unwrap();
                replies.get(self.last.as_str()).copied().unwrap_or("0")
            };
            if reply == "!" {
                return epics_rs::asyn::interpose::OctetReadResult::failed(smartmotor_err(
                    "read failed",
                ));
            }
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            epics_rs::asyn::interpose::OctetReadResult::ok(
                reply.len(),
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    type Replies = Arc<Mutex<std::collections::HashMap<&'static str, &'static str>>>;

    /// The axis, its reply table and quiet flag, and the port runtime to keep.
    fn single(
        name: &str,
    ) -> (
        SmartMotorAxis,
        Replies,
        Arc<std::sync::atomic::AtomicBool>,
        Box<dyn std::any::Any>,
    ) {
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

        let replies: Replies = Default::default();
        let quiet: Arc<std::sync::atomic::AtomicBool> = Default::default();
        let driver = Single {
            base: epics_rs::asyn::port::PortDriverBase::new(
                name,
                1,
                epics_rs::asyn::port::PortFlags::default(),
                epics_rs::asyn::port::InterfaceMask::UNMIGRATED_ALL,
                epics_rs::asyn::port::InterfaceMask::UNMIGRATED_INTERRUPTS,
            ),
            last: String::new(),
            replies: replies.clone(),
            quiet: quiet.clone(),
        };
        let (rt, actor) =
            create_port_runtime(&claim_port(name).unwrap(), driver, RuntimeConfig::default())
                .unwrap();
        let ctrl =
            SmartMotorController::new(rt.port_handle().clone(), Duration::from_secs(1)).unwrap();
        (
            SmartMotorAxis::new(Arc::new(Mutex::new(ctrl)), 0),
            replies,
            quiet,
            Box::new((rt, actor)),
        )
    }

    /// drvSmartMotor.cc:268-357: once RBt answers, a failed read is C
    /// `recv_mess`'s empty buffer, read as 0, and the status still posts.
    #[test]
    fn a_missed_reply_after_rbt_reads_as_zero() {
        let (mut axis, replies, _, _rt) = single("smartZero");
        let user = AsynUser::new(0);
        replies
            .lock()
            .unwrap()
            .extend([("RP", "!"), ("RBp", "!"), ("RBo", "!"), ("RV", "!")]);
        let s = axis.polled(&user).unwrap();
        assert_eq!(s.position, 0.0);
        assert!(!s.high_limit && s.powered && !s.comms_error && !s.problem);
    }

    /// drvSmartMotor.cc:271-291: an unchanged position keeps the direction,
    /// and a held move reports its record's CDIR.
    #[test]
    fn direction_is_kept_at_rest_and_commanded_while_moving() {
        let (mut axis, replies, _, _rt) = single("smartDir");
        let user = AsynUser::new(0);
        let set = |rbt, rp| {
            let mut r = replies.lock().unwrap();
            r.insert("RBt", rbt);
            r.insert("RP", rp);
        };
        set("0", "-50");
        assert!(!axis.polled(&user).unwrap().direction);
        set("0", "-50");
        assert!(!axis.polled(&user).unwrap().direction, "kept at rest");
        set("1", "-60");
        let mut cdir = true;
        let s = axis.polled_in_motion(&user, &mut cdir).unwrap();
        assert!(s.direction && cdir, "the record's CDIR, left unchanged");
        set("0", "100");
        let mut cdir = false;
        assert!(!axis.polled_in_motion(&user, &mut cdir).unwrap().direction);
        set("0", "90");
        assert!(
            !axis.polled(&user).unwrap().direction,
            "position delta at rest"
        );
    }

    /// drvSmartMotor.cc:332-348: the encoder is its own RP after RBo,
    /// truncated, where the position is rounded.
    #[test]
    fn encoder_is_a_second_rp_truncated() {
        let (mut axis, replies, _, _rt) = single("smartEnc");
        replies.lock().unwrap().insert("RP", "-7.6");
        let s = axis.polled(&AsynUser::new(0)).unwrap();
        assert_eq!((s.position, s.encoder_position), (-8.0, -7.0));
    }
}
