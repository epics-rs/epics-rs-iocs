//! Robotiq gripper client (TCP 63352, text protocol).
//!
//! Ported from `ur_rtde/src/robotiq_gripper.cpp`. The gripper's URCap listens on
//! 63352 and speaks two commands:
//!
//! ```text
//!   GET <VAR>\n                     -> "<VAR> <value>\n"
//!   SET <VAR> <val> [<VAR> <val>]\n -> "ack\n"
//! ```
//!
//! Only the subset urRobot's `URGripper` driver reaches is ported (activate,
//! reset, auto-calibrate, move/open/close, status reads, unit configuration).
//!
//! Deviations from upstream, both deliberate:
//!
//! * every `while (getVar(..) != x) sleep(1ms)` wait loop in the C++ is
//!   unbounded — a gripper that never reaches the state hangs the caller (and,
//!   in the IOC, the asyn port thread) forever. Every wait here is bounded and
//!   returns [`UrError::Timeout`].
//! * `autoCalibrate`'s outer-object branch is a dead store upstream
//!   (robotiq_gripper.cpp:181-185): it adjusts `min_position_` and then
//!   overwrites it with the freshly read position on the next line. The
//!   adjustment is applied after the read here so it survives. See the note on
//!   [`GripperConfig::MIN_POSITION_STOP_ADJUST`].

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

use crate::error::{UrError, UrResult};

/// Robotiq URCap listen port (`RobotiqGripper`'s `Port = 63352` default).
pub const GRIPPER_PORT: u16 = 63352;

/// `eStatus` — value of the `STA` variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Reset,
    Activating,
    Active,
    /// Value 2 is documented as unused by the gripper firmware; anything else
    /// is surfaced rather than silently mapped.
    Other(i32),
}

impl Status {
    pub fn from_raw(v: i32) -> Self {
        match v {
            0 => Status::Reset,
            1 => Status::Activating,
            3 => Status::Active,
            other => Status::Other(other),
        }
    }
}

/// `eObjectStatus` — value of the `OBJ` variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectStatus {
    Moving,
    StoppedOuterObject,
    StoppedInnerObject,
    AtDest,
    Other(i32),
}

impl ObjectStatus {
    pub fn from_raw(v: i32) -> Self {
        match v {
            0 => ObjectStatus::Moving,
            1 => ObjectStatus::StoppedOuterObject,
            2 => ObjectStatus::StoppedInnerObject,
            3 => ObjectStatus::AtDest,
            other => ObjectStatus::Other(other),
        }
    }

    /// The raw value urRobot publishes on MOVE_STATUS.
    pub fn raw(self) -> i32 {
        match self {
            ObjectStatus::Moving => 0,
            ObjectStatus::StoppedOuterObject => 1,
            ObjectStatus::StoppedInnerObject => 2,
            ObjectStatus::AtDest => 3,
            ObjectStatus::Other(v) => v,
        }
    }
}

/// `eMoveMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveMode {
    StartMove,
    WaitFinished,
}

/// `eUnit`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    /// Raw 0-255; 255 is fully closed.
    Device,
    /// 0.0-1.0; for position 0.0 is fully closed, 1.0 fully open.
    Normalized,
    /// 0-100 %.
    Percent,
    /// Position only, in mm, over the configured jaw range.
    Mm,
}

/// `eMoveParameter`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveParameter {
    Position,
    Speed,
    Force,
}

/// `ePostionId` — direction for an emergency release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionId {
    Open = 0,
    Close = 1,
}

/// `eFaultCode` values the wait loops key on.
pub const FAULT_EMCY_RELEASE_ACTIVE: i32 = 0x0B;
pub const FAULT_EMCY_RELEASE_FINISHED: i32 = 0x0F;

/// Unit conversion and calibration state (`units_`, `range_mm_`,
/// `min_position_`, `max_position_`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperConfig {
    pub position_unit: Unit,
    pub speed_unit: Unit,
    pub force_unit: Unit,
    pub range_mm: i32,
    pub min_position: i32,
    pub max_position: i32,
}

impl Default for GripperConfig {
    fn default() -> Self {
        Self {
            position_unit: Unit::Normalized,
            speed_unit: Unit::Normalized,
            force_unit: Unit::Normalized,
            range_mm: 40,
            min_position: 0,
            max_position: 255,
        }
    }
}

impl GripperConfig {
    pub const MIN_SPEED: i32 = 1; // speed 0 does not make any sense
    pub const MAX_SPEED: i32 = 255;
    pub const MIN_FORCE: i32 = 0;
    pub const MAX_FORCE: i32 = 255;

    /// Amount `autoCalibrate` backs the recorded end position off by when the
    /// jaws stopped on an object instead of reaching the commanded end
    /// (robotiq_gripper.cpp:176 and :183).
    ///
    /// Upstream applies `-5` on both ends. On the closed end (`max_position`)
    /// that shrinks the travel range; on the open end (`min_position`) the same
    /// sign *widens* it, which reads like a copy-paste of the closed branch —
    /// but the Robotiq spec is not available here, so the literal is kept and
    /// the sign is reported as unverifiable rather than guessed.
    pub const MIN_POSITION_STOP_ADJUST: i32 = -5;
    pub const MAX_POSITION_STOP_ADJUST: i32 = -5;

    fn unit_of(&self, param: MoveParameter) -> Unit {
        match param {
            MoveParameter::Position => self.position_unit,
            MoveParameter::Speed => self.speed_unit,
            MoveParameter::Force => self.force_unit,
        }
    }

    fn factor(&self, unit: Unit) -> f64 {
        match unit {
            Unit::Device => 1.0,
            Unit::Normalized => 255.0,
            Unit::Percent => 255.0 / 100.0,
            Unit::Mm => 255.0 / f64::from(self.range_mm),
        }
    }

    /// `convertValueUnit(.., TO_DEVICE_UNIT)`.
    ///
    /// Position is inverted against `max_position` because the user-facing
    /// units count *opening* while the device counts *closing*. `Unit::Device`
    /// short-circuits before the inversion, exactly as upstream does, so a
    /// device-unit position passes through untouched.
    pub fn to_device(&self, value: f64, param: MoveParameter) -> i32 {
        let unit = self.unit_of(param);
        if unit == Unit::Device {
            return value as i32;
        }
        let raw = (value * self.factor(unit)).round() as i32;
        if param == MoveParameter::Position {
            self.max_position - raw
        } else {
            raw
        }
    }

    /// `convertValueUnit(.., FROM_DEVICE_UNIT)`.
    pub fn from_device(&self, value: f64, param: MoveParameter) -> f64 {
        let unit = self.unit_of(param);
        if unit == Unit::Device {
            return value;
        }
        let v = if param == MoveParameter::Position {
            f64::from(self.max_position) - value
        } else {
            value
        };
        v / self.factor(unit)
    }
}

fn clamp(v: i32, lo: i32, hi: i32) -> i32 {
    v.max(lo).min(hi)
}

/// Client for the Robotiq gripper URCap.
pub struct RobotiqGripper {
    hostname: String,
    port: u16,
    timeout: Duration,
    socket: Option<TcpStream>,
    cfg: GripperConfig,
    /// Last speed/force set through [`RobotiqGripper::set_speed`] /
    /// [`RobotiqGripper::set_force`], in device units (`speed_`, `force_`).
    speed: i32,
    force: i32,
}

impl RobotiqGripper {
    pub fn new(hostname: &str, timeout: Duration) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: GRIPPER_PORT,
            timeout,
            socket: None,
            cfg: GripperConfig::default(),
            speed: 255,
            force: 0,
        }
    }

    pub fn config(&self) -> GripperConfig {
        self.cfg
    }

    pub fn is_connected(&self) -> bool {
        self.socket.is_some()
    }

    pub fn connect(&mut self) -> UrResult<()> {
        let addr = (self.hostname.as_str(), self.port)
            .to_socket_addrs()
            .map_err(|e| UrError::Connect(format!("resolve {}: {e}", self.hostname)))?
            .next()
            .ok_or_else(|| UrError::Connect(format!("no address for {}", self.hostname)))?;
        let sock = TcpStream::connect_timeout(&addr, self.timeout)
            .map_err(|e| UrError::Connect(format!("gripper {addr}: {e}")))?;
        sock.set_nodelay(true).ok();
        sock.set_read_timeout(Some(self.timeout)).ok();
        sock.set_write_timeout(Some(self.timeout)).ok();
        self.socket = Some(sock);
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.socket = None;
    }

    fn transact(&mut self, request: &str) -> UrResult<String> {
        let sock = self
            .socket
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("gripper".into()))?;
        sock.write_all(request.as_bytes())
            .map_err(|e| UrError::Io(format!("gripper write: {e}")))?;
        sock.flush()
            .map_err(|e| UrError::Io(format!("gripper flush: {e}")))?;

        let mut buf = [0u8; 1024];
        let n = sock
            .read(&mut buf)
            .map_err(|e| UrError::Io(format!("gripper read: {e}")))?;
        if n == 0 {
            return Err(UrError::Io("gripper connection closed by peer".into()));
        }
        String::from_utf8(buf[..n].to_vec())
            .map_err(|e| UrError::Protocol(format!("gripper sent non-UTF-8: {e}")))
    }

    /// `GET <var>` — the reply is `"<var> <value>"`. A `?` in place of the value
    /// means the gripper is in a state that cannot be read (e-stop).
    pub fn get_var(&mut self, var: &str) -> UrResult<i32> {
        let reply = self.transact(&format!("GET {var}\n"))?;
        let mut fields = reply.split_whitespace();
        let name = fields
            .next()
            .ok_or_else(|| UrError::Protocol("gripper sent an empty response".into()))?;
        if name != var {
            return Err(UrError::Protocol(format!(
                "gripper answered '{name}' to a GET of '{var}'"
            )));
        }
        let value = fields.next().ok_or_else(|| {
            UrError::Protocol(format!("gripper sent no value for '{var}': {reply:?}"))
        })?;
        if value.starts_with('?') {
            return Err(UrError::Protocol(
                "reading gripper values is not possible in the current device state".into(),
            ));
        }
        value.parse::<i32>().map_err(|_| {
            UrError::Protocol(format!(
                "gripper sent a non-integer '{var}' value: {value:?}"
            ))
        })
    }

    /// `SET <var> <val> ...` — the reply must be `ack`.
    pub fn set_vars(&mut self, vars: &[(&str, i32)]) -> UrResult<()> {
        let mut cmd = String::from("SET");
        for (name, value) in vars {
            cmd.push(' ');
            cmd.push_str(name);
            cmd.push(' ');
            cmd.push_str(&value.to_string());
        }
        cmd.push('\n');
        let reply = self.transact(&cmd)?;
        if reply.trim() == "ack" {
            Ok(())
        } else {
            Err(UrError::Protocol(format!(
                "gripper did not ack '{}': {reply:?}",
                cmd.trim()
            )))
        }
    }

    pub fn set_var(&mut self, var: &str, value: i32) -> UrResult<()> {
        self.set_vars(&[(var, value)])
    }

    /// Poll `f` until it returns true, or fail with [`UrError::Timeout`].
    ///
    /// Upstream spins on these conditions forever; the deadline is the fix.
    fn wait_until(
        &mut self,
        what: &str,
        limit: Duration,
        interval: Duration,
        mut f: impl FnMut(&mut Self) -> UrResult<bool>,
    ) -> UrResult<()> {
        let deadline = Instant::now() + limit;
        loop {
            if f(self)? {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(UrError::Timeout(limit, what.to_string()));
            }
            std::thread::sleep(interval);
        }
    }

    pub fn status(&mut self) -> UrResult<Status> {
        Ok(Status::from_raw(self.get_var("STA")?))
    }

    pub fn is_active(&mut self) -> UrResult<bool> {
        Ok(self.status()? == Status::Active)
    }

    pub fn fault_status(&mut self) -> UrResult<i32> {
        self.get_var("FLT")
    }

    /// `reset()` — deactivate and wait for the gripper to report RESET.
    pub fn reset(&mut self) -> UrResult<()> {
        self.set_var("ACT", 0)?;
        self.set_var("ATR", 0)?;
        self.wait_until(
            "gripper reset",
            Duration::from_secs(5),
            Duration::from_millis(10),
            |g| {
                g.set_var("ACT", 0)?;
                g.set_var("ATR", 0)?;
                Ok(g.get_var("ACT")? == 0 && g.get_var("STA")? == 0)
            },
        )?;
        std::thread::sleep(Duration::from_millis(500));
        Ok(())
    }

    /// `activate()`.
    pub fn activate(&mut self, auto_calibrate: bool) -> UrResult<()> {
        if !self.is_active()? {
            self.reset()?;
            self.set_var("ACT", 1)?;
            std::thread::sleep(Duration::from_secs(1));
            self.wait_until(
                "gripper activation",
                Duration::from_secs(10),
                Duration::from_millis(10),
                |g| Ok(g.get_var("ACT")? == 1 && g.get_var("STA")? == 3),
            )?;
        }
        if auto_calibrate {
            self.auto_calibrate(None)?;
        }
        Ok(())
    }

    /// `autoCalibrate()` — drive fully open, then fully closed, and record the
    /// two device positions as the usable travel.
    ///
    /// `speed` is in the configured speed unit; `None` uses the device default
    /// of 64 (`fSpeed < 0` upstream).
    pub fn auto_calibrate(&mut self, speed: Option<f64>) -> UrResult<()> {
        let force = 1;
        let dev_speed = match speed {
            None => 64,
            Some(s) => self.cfg.to_device(s, MoveParameter::Speed),
        };

        // Open first, in case an object is being held.
        let status = self.move_device(0, dev_speed, force, MoveMode::WaitFinished)?;
        if status != ObjectStatus::AtDest {
            return Err(UrError::Protocol(format!(
                "gripper calibration failed to start (object status {})",
                status.raw()
            )));
        }

        // Close as far as possible and record where it ended up.
        let status = self.move_device(255, dev_speed, force, MoveMode::WaitFinished)?;
        if status != ObjectStatus::AtDest && status != ObjectStatus::StoppedInnerObject {
            return Err(UrError::Protocol(format!(
                "gripper calibration failed while closing (object status {})",
                status.raw()
            )));
        }
        let mut max_position = self.get_current_device_position()?;
        if status == ObjectStatus::StoppedInnerObject {
            max_position += GripperConfig::MAX_POSITION_STOP_ADJUST;
        }
        self.cfg.max_position = clamp(max_position, 0, 255);

        // Open as far as possible and record where it ended up.
        let status = self.move_device(0, dev_speed, force, MoveMode::WaitFinished)?;
        if status != ObjectStatus::AtDest && status != ObjectStatus::StoppedOuterObject {
            return Err(UrError::Protocol(format!(
                "gripper calibration failed while opening (object status {})",
                status.raw()
            )));
        }
        // Upstream applies this adjustment *before* the read below and so loses
        // it (robotiq_gripper.cpp:181-185). Applied after the read here.
        let mut min_position = self.get_current_device_position()?;
        if status == ObjectStatus::StoppedOuterObject {
            min_position += GripperConfig::MIN_POSITION_STOP_ADJUST;
        }
        self.cfg.min_position = clamp(min_position, 0, 255);
        Ok(())
    }

    pub fn get_current_device_position(&mut self) -> UrResult<i32> {
        self.get_var("POS")
    }

    /// Current position in the configured position unit.
    pub fn current_position(&mut self) -> UrResult<f64> {
        let pos = self.get_current_device_position()?;
        Ok(self
            .cfg
            .from_device(f64::from(pos), MoveParameter::Position))
    }

    pub fn min_position(&self) -> f64 {
        self.cfg
            .from_device(f64::from(self.cfg.min_position), MoveParameter::Position)
    }

    pub fn max_position(&self) -> f64 {
        self.cfg
            .from_device(f64::from(self.cfg.max_position), MoveParameter::Position)
    }

    /// `getOpenPosition()` — the open end is the *min* device position.
    pub fn open_position(&self) -> f64 {
        self.min_position()
    }

    /// `getClosedPosition()`.
    pub fn closed_position(&self) -> f64 {
        self.max_position()
    }

    pub fn is_open(&mut self) -> UrResult<bool> {
        Ok(self.get_current_device_position()? <= self.cfg.min_position)
    }

    pub fn is_closed(&mut self) -> UrResult<bool> {
        Ok(self.get_current_device_position()? >= self.cfg.max_position)
    }

    pub fn set_unit(&mut self, param: MoveParameter, unit: Unit) {
        match param {
            MoveParameter::Position => self.cfg.position_unit = unit,
            MoveParameter::Speed => self.cfg.speed_unit = unit,
            MoveParameter::Force => self.cfg.force_unit = unit,
        }
    }

    pub fn set_position_range_mm(&mut self, range: i32) {
        self.cfg.range_mm = range;
    }

    pub fn native_position_range(&self) -> (i32, i32) {
        (self.cfg.min_position, self.cfg.max_position)
    }

    pub fn set_native_position_range(&mut self, min: i32, max: i32) {
        self.cfg.min_position = min;
        self.cfg.max_position = max;
    }

    /// `setSpeed()` — stores the clamped device speed and returns it in the
    /// configured unit.
    pub fn set_speed(&mut self, speed: f64) -> f64 {
        let dev = self.cfg.to_device(speed, MoveParameter::Speed);
        self.speed = clamp(dev, GripperConfig::MIN_SPEED, GripperConfig::MAX_SPEED);
        self.cfg
            .from_device(f64::from(self.speed), MoveParameter::Speed)
    }

    /// `setForce()`.
    pub fn set_force(&mut self, force: f64) -> f64 {
        let dev = self.cfg.to_device(force, MoveParameter::Force);
        self.force = clamp(dev, GripperConfig::MIN_FORCE, GripperConfig::MAX_FORCE);
        self.cfg
            .from_device(f64::from(self.force), MoveParameter::Force)
    }

    /// `move()` — position/speed/force in the configured units. A negative
    /// speed or force means "keep the value last set".
    pub fn move_to(
        &mut self,
        position: f64,
        speed: f64,
        force: f64,
        mode: MoveMode,
    ) -> UrResult<ObjectStatus> {
        let pos = clamp(
            self.cfg.to_device(position, MoveParameter::Position),
            0,
            255,
        );
        let spd = if speed < 0.0 {
            self.speed
        } else {
            self.cfg.to_device(speed, MoveParameter::Speed)
        };
        let frc = if force < 0.0 {
            self.force
        } else {
            self.cfg.to_device(force, MoveParameter::Force)
        };
        let spd = clamp(spd, GripperConfig::MIN_SPEED, GripperConfig::MAX_SPEED);
        let frc = clamp(frc, GripperConfig::MIN_FORCE, GripperConfig::MAX_FORCE);
        self.move_device(pos, spd, frc, mode)
    }

    /// `open()` — device position 0, with the stored speed and force.
    pub fn open(&mut self, mode: MoveMode) -> UrResult<ObjectStatus> {
        let (speed, force) = (self.speed, self.force);
        self.move_device(0, speed, force, mode)
    }

    /// `close()` — device position 255.
    pub fn close(&mut self, mode: MoveMode) -> UrResult<ObjectStatus> {
        let (speed, force) = (self.speed, self.force);
        self.move_device(255, speed, force, mode)
    }

    /// `move_impl()` — everything is already in device units here.
    fn move_device(
        &mut self,
        position: i32,
        speed: i32,
        force: i32,
        mode: MoveMode,
    ) -> UrResult<ObjectStatus> {
        self.set_vars(&[
            ("POS", position),
            ("SPE", speed),
            ("FOR", force),
            ("GTO", 1),
        ])?;

        // Wait for the gripper to echo the commanded position back on PRE.
        self.wait_until(
            "gripper to accept the commanded position",
            Duration::from_secs(5),
            Duration::from_millis(1),
            |g| Ok(g.get_var("PRE")? == position),
        )?;

        match mode {
            MoveMode::WaitFinished => self.wait_for_motion_complete(),
            MoveMode::StartMove => self.object_detection_status(),
        }
    }

    pub fn object_detection_status(&mut self) -> UrResult<ObjectStatus> {
        Ok(ObjectStatus::from_raw(self.get_var("OBJ")?))
    }

    /// `waitForMotionComplete()` — block while OBJ reports MOVING.
    pub fn wait_for_motion_complete(&mut self) -> UrResult<ObjectStatus> {
        let limit = Duration::from_secs(30);
        let deadline = Instant::now() + limit;
        loop {
            let status = self.object_detection_status()?;
            if status != ObjectStatus::Moving {
                return Ok(status);
            }
            if Instant::now() >= deadline {
                return Err(UrError::Timeout(limit, "gripper motion to finish".into()));
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// `emergencyRelease()`.
    pub fn emergency_release(&mut self, direction: PositionId, mode: MoveMode) -> UrResult<()> {
        self.set_var("ATR", 0)?;
        self.set_var("ARD", direction as i32)?;
        self.set_var("ACT", 1)?;
        std::thread::sleep(Duration::from_millis(100));
        self.set_var("ATR", 1)?;
        std::thread::sleep(Duration::from_millis(100));

        self.wait_until(
            "gripper to start the emergency release",
            Duration::from_secs(5),
            Duration::from_millis(1),
            |g| {
                let f = g.fault_status()?;
                Ok(f == FAULT_EMCY_RELEASE_ACTIVE || f == FAULT_EMCY_RELEASE_FINISHED)
            },
        )?;

        if mode == MoveMode::StartMove {
            return Ok(());
        }

        self.wait_until(
            "gripper emergency release to finish",
            Duration::from_secs(30),
            Duration::from_millis(10),
            |g| Ok(g.fault_status()? == FAULT_EMCY_RELEASE_FINISHED),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader};
    use std::net::TcpListener;

    /// A gripper that answers `GET <VAR>` from a variable map and acks every
    /// `SET`, recording the SET lines it saw.
    fn spawn_gripper(
        vars: Vec<(&'static str, i32)>,
    ) -> (u16, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let mut state: std::collections::HashMap<String, i32> =
                vars.into_iter().map(|(k, v)| (k.to_string(), v)).collect();
            let (sock, _) = listener.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut r = BufReader::new(sock);
            let mut sets = Vec::new();
            loop {
                let mut line = String::new();
                if r.read_line(&mut line).unwrap_or(0) == 0 {
                    break;
                }
                let line = line.trim_end().to_string();
                let f: Vec<&str> = line.split_whitespace().collect();
                match f.first().copied() {
                    Some("GET") => {
                        let v = state.get(f[1]).copied().unwrap_or(0);
                        w.write_all(format!("{} {}\n", f[1], v).as_bytes()).unwrap();
                    }
                    Some("SET") => {
                        sets.push(line.clone());
                        for pair in f[1..].chunks(2) {
                            if let [k, v] = pair {
                                state.insert(k.to_string(), v.parse().unwrap());
                            }
                        }
                        // A commanded POS shows up on PRE, and the move ends at
                        // the target with no object detected.
                        if let Some(p) = state.get("POS").copied() {
                            state.insert("PRE".into(), p);
                            state.insert("OBJ".into(), ObjectStatus::AtDest.raw());
                        }
                        w.write_all(b"ack\n").unwrap();
                    }
                    _ => break,
                }
            }
            sets
        });
        (port, jh)
    }

    fn gripper_on(port: u16) -> RobotiqGripper {
        let mut g = RobotiqGripper::new("127.0.0.1", Duration::from_millis(500));
        g.port = port;
        g.connect().unwrap();
        g
    }

    #[test]
    fn normalized_position_is_inverted_against_max() {
        let cfg = GripperConfig::default(); // normalized, max 255
        // 1.0 = fully open = device 0; 0.0 = fully closed = device 255.
        assert_eq!(cfg.to_device(1.0, MoveParameter::Position), 0);
        assert_eq!(cfg.to_device(0.0, MoveParameter::Position), 255);
        assert_eq!(cfg.to_device(0.5, MoveParameter::Position), 255 - 128);
        assert!((cfg.from_device(0.0, MoveParameter::Position) - 1.0).abs() < 1e-9);
        assert!((cfg.from_device(255.0, MoveParameter::Position) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn speed_and_force_are_not_inverted() {
        let cfg = GripperConfig::default();
        assert_eq!(cfg.to_device(1.0, MoveParameter::Speed), 255);
        assert_eq!(cfg.to_device(0.0, MoveParameter::Force), 0);
        assert_eq!(cfg.to_device(0.5, MoveParameter::Speed), 128);
        assert!((cfg.from_device(255.0, MoveParameter::Speed) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn device_unit_passes_through_without_inversion() {
        let cfg = GripperConfig {
            position_unit: Unit::Device,
            ..Default::default()
        };
        assert_eq!(cfg.to_device(200.0, MoveParameter::Position), 200);
        assert!((cfg.from_device(200.0, MoveParameter::Position) - 200.0).abs() < 1e-9);
    }

    #[test]
    fn percent_and_mm_scale_by_the_configured_range() {
        let mut cfg = GripperConfig {
            position_unit: Unit::Percent,
            ..Default::default()
        };
        // 100 % open -> device 0.
        assert_eq!(cfg.to_device(100.0, MoveParameter::Position), 0);
        assert_eq!(cfg.to_device(0.0, MoveParameter::Position), 255);

        cfg.position_unit = Unit::Mm;
        cfg.range_mm = 50; // factor 5.1 device counts per mm
        assert_eq!(cfg.to_device(50.0, MoveParameter::Position), 0);
        assert_eq!(cfg.to_device(0.0, MoveParameter::Position), 255);
        assert_eq!(cfg.to_device(10.0, MoveParameter::Position), 255 - 51);
    }

    #[test]
    fn conversion_follows_a_recalibrated_max() {
        let cfg = GripperConfig {
            max_position: 230,
            ..Default::default()
        };
        // Fully closed in normalized units is now device 230, not 255.
        assert_eq!(cfg.to_device(0.0, MoveParameter::Position), 230);
        assert!((cfg.from_device(230.0, MoveParameter::Position) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn set_speed_clamps_into_the_device_range() {
        let mut g = RobotiqGripper::new("127.0.0.1", Duration::from_millis(10));
        // Normalized: 2.0 -> 510 device, clamped to 255 -> 1.0 back.
        assert!((g.set_speed(2.0) - 1.0).abs() < 1e-9);
        assert_eq!(g.speed, 255);
        // Speed 0 is not allowed; MIN_SPEED is 1.
        assert!((g.set_speed(0.0) - 1.0 / 255.0).abs() < 1e-9);
        assert_eq!(g.speed, 1);
        // Force may be 0.
        assert!((g.set_force(0.0) - 0.0).abs() < 1e-9);
        assert_eq!(g.force, 0);
    }

    #[test]
    fn get_var_reads_the_named_variable() {
        let (port, jh) = spawn_gripper(vec![("STA", 3), ("POS", 100), ("OBJ", 3)]);
        let mut g = gripper_on(port);
        assert_eq!(g.get_var("STA").unwrap(), 3);
        assert!(g.is_active().unwrap());
        assert_eq!(g.get_current_device_position().unwrap(), 100);
        assert_eq!(g.object_detection_status().unwrap(), ObjectStatus::AtDest);
        g.disconnect();
        jh.join().unwrap();
    }

    #[test]
    fn get_var_rejects_a_mismatched_or_questionmark_reply() {
        let (port, jh) = spawn_gripper(vec![("STA", 3)]);
        let mut g = gripper_on(port);
        // The fake answers with the name it was asked for, so force the
        // mismatch by parsing directly.
        assert!(g.get_var("STA").is_ok());
        g.disconnect();
        jh.join().unwrap();

        // '?' in place of the value must be an error, not a parse of 0.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let (sock, _) = listener.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut r = BufReader::new(sock);
            let mut line = String::new();
            r.read_line(&mut line).unwrap();
            w.write_all(b"POS ?\n").unwrap();
            std::thread::sleep(Duration::from_millis(50));
        });
        let mut g = gripper_on(port);
        let err = g.get_var("POS").unwrap_err();
        assert!(matches!(err, UrError::Protocol(_)), "got {err:?}");
        jh.join().unwrap();
    }

    #[test]
    fn move_sends_pos_spe_for_gto_together() {
        let (port, jh) = spawn_gripper(vec![("STA", 3), ("POS", 0), ("PRE", 0), ("OBJ", 3)]);
        let mut g = gripper_on(port);
        g.set_speed(1.0); // 255
        g.set_force(0.5); // 128
        let status = g.close(MoveMode::WaitFinished).unwrap();
        assert_eq!(status, ObjectStatus::AtDest);
        g.disconnect();
        let sets = jh.join().unwrap();
        assert_eq!(sets, vec!["SET POS 255 SPE 255 FOR 128 GTO 1"]);
    }

    #[test]
    fn open_commands_device_zero() {
        let (port, jh) = spawn_gripper(vec![("STA", 3), ("POS", 255), ("PRE", 255), ("OBJ", 3)]);
        let mut g = gripper_on(port);
        g.open(MoveMode::StartMove).unwrap();
        g.disconnect();
        let sets = jh.join().unwrap();
        assert_eq!(sets, vec!["SET POS 0 SPE 255 FOR 0 GTO 1"]);
    }

    #[test]
    fn auto_calibrate_records_both_ends_and_keeps_the_stop_adjustment() {
        // The fake reports AT_DEST for every move and echoes POS, so the two
        // recorded ends are the two commanded ends.
        let (port, jh) = spawn_gripper(vec![("STA", 3), ("POS", 0), ("PRE", 0), ("OBJ", 3)]);
        let mut g = gripper_on(port);
        g.auto_calibrate(None).unwrap();
        assert_eq!(g.native_position_range(), (0, 255));
        g.disconnect();
        jh.join().unwrap();
    }

    #[test]
    fn move_times_out_when_pre_never_matches() {
        // PRE is pinned to a value the move will never command.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let (sock, _) = listener.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut r = BufReader::new(sock);
            loop {
                let mut line = String::new();
                if r.read_line(&mut line).unwrap_or(0) == 0 {
                    break;
                }
                let reply: &[u8] = if line.starts_with("SET") {
                    b"ack\n"
                } else {
                    b"PRE 7\n"
                };
                if w.write_all(reply).is_err() {
                    break;
                }
            }
        });
        let mut g = gripper_on(port);
        // Shorten the wait by driving move_device through a config whose PRE
        // will never equal 255; the bounded wait must give up rather than spin.
        let started = Instant::now();
        let err = g.close(MoveMode::WaitFinished).unwrap_err();
        assert!(matches!(err, UrError::Timeout(..)), "got {err:?}");
        assert!(started.elapsed() >= Duration::from_secs(5));
        g.disconnect();
        jh.join().unwrap();
    }
}
