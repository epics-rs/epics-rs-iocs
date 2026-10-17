//! Parker ACR series controller driver (ASCII, over an asyn octet port).
//!
//! Ported from `motorParker/parkerApp/src/ACRMotorDriver.cpp` (a model-3
//! `asynMotorController`/`asynMotorAxis` pair) — covers the ACR family including
//! the Aries. Commands are ASCII, addressed by axis name (`AXIS<n>`), and
//! CR-terminated by the port's input and output EOS, which the startup script
//! sets as C's does. Connects over a `drvAsynIPPort`.
//!
//! ## Echo and reply model
//!
//! The controller constructor disables command echo (`ECHO 4`); the echo of that
//! command itself is drained once at startup (the C driver uses its initial
//! binary-I/O read as the flush). Afterwards, set commands (`JOG ...`, `RES`,
//! `DRIVE`) are write-only; only register reads (`?P<reg>`), the `PPU` query and
//! the `DRIVE <axis>` power query read a reply.
//!
//! ## Units
//!
//! `PPU` (pulses per engineering unit) is read from the controller: the
//! position registers hold the record's steps (pulses) while `JOG` and `RES`
//! take engineering units, so commanded steps are divided by `PPU` and sent
//! `%f`, as C.
//!
//! ## Kill clear
//!
//! Jog motion commands are prefixed with `Ctrl-Y` (0x19), which clears the kill
//! latch on all axes (e.g. after a limit hit), matching C.
//!
//! ## Not modeled (documented)
//!
//! The controller-wide binary I/O registers (`ACR_BINARY_IN`/`OUT`, the
//! `ACR_READ_BINARY_IO` trigger) and the per-axis jerk (`ACR_JERK`) are auxiliary
//! asyn parameters outside the motor record and are not exposed. `set_pid_gain`
//! is a no-op (the jerk write is the only tuning the C driver offers, and it is
//! not a PID gain).

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind, PollFailure};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi};

/// Response buffer size.
const READ_BUF: usize = 64;

/// Ctrl-Y — prefixes jog commands to clear the all-axis kill latch (C `CtlY`).
const CTL_Y: char = '\u{19}';

/// Settle time after `ECHO 4` before draining its echo (C `epicsThreadSleep`).
const ECHO_SETTLE: Duration = Duration::from_millis(500);

/// Flags-register bit indicating the axis is moving (C `0x1000000`).
const FLAG_MOVING: i32 = 0x0100_0000;

/// Shared ACR controller endpoint owning the asyn octet handle.
pub struct AcrController {
    handle: SyncIOHandle,
}

impl AcrController {
    /// Wrap a connected octet handle and disable command echo (`ECHO 4`),
    /// draining the echo of that command so subsequent replies parse cleanly.
    pub fn new(handle: SyncIOHandle) -> Self {
        let ctrl = Self { handle };
        let _ = ctrl.write_only("ECHO 4");
        std::thread::sleep(ECHO_SETTLE);
        // Drain the echoed "ECHO 4" line (best effort; the port may already be
        // quiet if echo was off).
        let _ = ctrl.handle.read_octet(0, READ_BUF);
        ctrl
    }

    /// Write a command with no reply expected (echo is disabled).
    fn write_only(&self, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, cmd.as_bytes())?;
        Ok(())
    }

    /// Write a query and read its reply.
    fn query(&self, cmd: &str) -> AsynResult<String> {
        let raw = self.handle.write_read(0, cmd.as_bytes(), READ_BUF)?;
        Ok(String::from_utf8_lossy(&raw)
            .trim_matches(['\r', '\n', '\0', ' '])
            .to_string())
    }

    /// Read a controller register (`?P<reg>`).
    fn read_reg(&self, reg: i32) -> AsynResult<String> {
        self.query(&format!("?P{reg}"))
    }
}

fn acr_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// One ACR axis sharing a controller. Implements [`AsynMotor`].
pub struct AcrAxis {
    controller: Arc<Mutex<AcrController>>,
    /// Axis name used in commands (`AXIS<n>`).
    name: String,
    /// Pulses per engineering unit (read from the controller); `None` when
    /// the constructor's `PPU` query failed and C left `pulsesPerUnit_` unset.
    pulses_per_unit: Option<f64>,
    encoder_pos_reg: i32,
    theory_pos_reg: i32,
    limits_reg: i32,
    flags_reg: i32,
}

impl AcrAxis {
    /// Construct axis `axis_no` and read its `PPU`, matching the C `ACRAxis`
    /// constructor register layout. Blocking I/O. As C, whatever `PPU`
    /// answers is kept and a failed query keeps the axis
    /// (ACRMotorDriver.cpp:297-306).
    pub fn new(controller: Arc<Mutex<AcrController>>, axis_no: usize) -> Self {
        let axis_no = axis_no as i32;
        let name = format!("AXIS{axis_no}");
        let reply = controller
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .query(&format!("{name} PPU"));
        let pulses_per_unit = reply.ok().map(|reply| atof(&reply));
        Self {
            controller,
            name,
            pulses_per_unit,
            encoder_pos_reg: 12290 + 256 * axis_no,
            theory_pos_reg: 12294 + 256 * axis_no,
            limits_reg: 4600 + axis_no,
            flags_reg: 4120 + axis_no,
        }
    }

    fn lock(&self) -> MutexGuard<'_, AcrController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Convert an engineering value to controller units (÷ PPU). C divides by
    /// an uninitialised `pulsesPerUnit_` when its PPU query failed, and sends
    /// `inf` for a PPU of 0; both C defects refuse the command instead.
    fn scaled(&self, value: f64) -> AsynResult<f64> {
        match self.pulses_per_unit {
            None => Err(acr_err(format!("{}: PPU was never read", self.name))),
            Some(0.0) => Err(acr_err(format!("{}: PPU is 0", self.name))),
            Some(ppu) => Ok(value / ppu),
        }
    }

    /// Write the common jog acceleration and velocity for a motion command.
    fn set_jog_speed(
        &self,
        ctrl: &AcrController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        ctrl.write_only(&format!(
            "{} JOG ACC {:.6}",
            self.name,
            self.scaled(acceleration)?
        ))?;
        ctrl.write_only(&format!(
            "{} JOG VEL {:.6}",
            self.name,
            self.scaled(velocity)?
        ))
    }
}

impl AsynMotor for AcrAxis {
    /// C `ACRAxis::poll`'s `skip` label sets only `motorStatusProblem_`
    /// (ACRMotorDriver.cpp).
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
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
        self.set_jog_speed(&ctrl, velocity, acceleration)?;
        ctrl.write_only(&format!(
            "{CTL_Y}:{} JOG ABS {:.6}",
            self.name,
            self.scaled(position)?
        ))
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
        self.set_jog_speed(&ctrl, velocity, acceleration)?;
        ctrl.write_only(&format!(
            "{CTL_Y}:{} JOG INC {:.6}",
            self.name,
            self.scaled(distance)?
        ))
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let (speed, forward) = if velocity < 0.0 {
            (-velocity, false)
        } else {
            (velocity, true)
        };
        let ctrl = self.lock();
        self.set_jog_speed(&ctrl, speed, acceleration)?;
        ctrl.write_only(&format!(
            "{CTL_Y}:{} JOG {}",
            self.name,
            if forward { "FWD" } else { "REV" }
        ))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        self.set_jog_speed(&ctrl, velocity, acceleration)?;
        ctrl.write_only(&format!(
            "{CTL_Y}:{} JOG HOME {}",
            self.name,
            if forward { 1 } else { -1 }
        ))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!("{} JOG OFF", self.name))
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!("{} RES {:.6}", self.name, self.scaled(position)?))?;
        ctrl.write_only(&format!("{} JOG REN", self.name))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_only(&format!(
            "DRIVE {} {}",
            if enable { "ON" } else { "OFF" },
            self.name
        ))
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        // No PID-gain support in the C ACR driver.
        Ok(())
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();

        let encoder_position = atof(&ctrl.read_reg(self.encoder_pos_reg)?);
        let theory_position = atof(&ctrl.read_reg(self.theory_pos_reg)?);

        let flags = atoi(&ctrl.read_reg(self.flags_reg)?);
        let moving = (flags & FLAG_MOVING) != 0;

        let limits = atoi(&ctrl.read_reg(self.limits_reg)?);
        let high_limit = (limits & 0x1) != 0;
        let low_limit = (limits & 0x2) != 0;
        let at_home = (limits & 0x4) != 0;

        let drive = ctrl.query(&format!("DRIVE {}", self.name))?;
        let powered = drive.contains("ON");
        drop(ctrl);

        Ok(MotorStatus {
            position: theory_position,
            encoder_position,
            velocity: 0.0,
            done: !moving,
            moving,
            // C sets these only when the constructor read PPU.
            has_encoder: self.pulses_per_unit.is_some(),
            gain_support: self.pulses_per_unit.is_some(),
            high_limit,
            low_limit,
            home: at_home,
            powered,
            ..MotorStatus::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    /// A failed PPU query keeps the axis with no encoder or gain support and
    /// refuses motion rather than dividing by an unset PPU.
    #[test]
    fn an_axis_of_a_missing_port_is_kept_and_never_scales() {
        let (handle, _) = SyncIOHandle::connect_port(
            "ACR_NONE",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let controller = Arc::new(Mutex::new(AcrController::new(handle)));
        let mut axis = AcrAxis::new(controller, 0);
        assert_eq!(axis.pulses_per_unit, None);
        let e = axis
            .move_absolute(&AsynUser::new(0), 1.0, 0.0, 1.0, 1.0)
            .unwrap_err();
        assert_eq!(e.message(), "AXIS0: PPU was never read");
        assert!(axis.poll(&AsynUser::new(0)).is_err());
    }

    /// A port that records what it is sent and answers nothing.
    struct Recorder {
        base: PortDriverBase,
        written: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl PortDriver for Recorder {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.written.lock().unwrap().push(data.to_vec());
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _: &AsynUser, _: &mut [u8]) -> AsynResult<usize> {
            Err(acr_err("no reply"))
        }
    }

    /// C writes its commands bare and leaves the CR to the port's output EOS
    /// (ACRMotorDriver.cpp:68, st.cmd.acr).
    #[test]
    fn commands_go_out_without_a_terminator_of_their_own() {
        let name = "ACR_BARE";
        let written = Arc::new(Mutex::new(Vec::new()));
        let driver = Recorder {
            base: PortDriverBase::new(name, 1, PortFlags::default()),
            written: written.clone(),
        };
        let claim = claim_port(name).unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let handle = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::ZERO);
        let controller = Arc::new(Mutex::new(AcrController::new(handle)));
        let _ = AcrAxis::new(controller, 0);
        assert_eq!(
            written.lock().unwrap()[..2],
            [b"ECHO 4".to_vec(), b"AXIS0 PPU".to_vec()]
        );
    }

    #[test]
    fn register_layout_matches_c() {
        // encoder=12290+256n, theory=12294+256n, limits=4600+n, flags=4120+n.
        let ax = |n: i32| (12290 + 256 * n, 12294 + 256 * n, 4600 + n, 4120 + n);
        assert_eq!(ax(0), (12290, 12294, 4600, 4120));
        assert_eq!(ax(2), (12802, 12806, 4602, 4122));
    }

    /// C keeps any PPU it reads; a 0 refuses scaling instead of sending `inf`,
    /// and a negative PPU scales as C does.
    #[test]
    fn a_zero_ppu_never_scales() {
        let (handle, _) = SyncIOHandle::connect_port(
            "ACR_PPU0",
            0,
            std::time::Duration::ZERO,
            InterfaceType::Octet,
        );
        let mut axis = AcrAxis::new(Arc::new(Mutex::new(AcrController::new(handle))), 0);
        axis.pulses_per_unit = Some(0.0);
        assert_eq!(axis.scaled(1.0).unwrap_err().message(), "AXIS0: PPU is 0");
        axis.pulses_per_unit = Some(-4.0);
        assert_eq!(axis.scaled(2.0).unwrap(), -0.5);
    }

    #[test]
    fn moving_flag_bit() {
        // Bit 24 = moving; other bits do not signal motion.
        let moving = |flags: i32| (flags & FLAG_MOVING) != 0;
        assert!(moving(0x0100_0000));
        assert!(moving(0x0100_0001));
        assert!(!moving(0x0000_0001));
        assert!(!moving(0x00FF_FFFF));
    }
}
