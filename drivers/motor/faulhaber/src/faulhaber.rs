//! Faulhaber MCDC2805 DC servo motor controller driver (serial ASCII).
//!
//! Ported from `motorFaulhaber/faulhaberApp/src/drvMCDC2805.cc` +
//! `devMCDC2805.cc`. The MCDC2805 is a DC servo positioner working in encoder
//! counts. Several modules can share one serial line, each addressed by a node
//! number prefixed to every command (e.g. node 0 status query is `0GST`).
//! Commands are terminated by the serial port's output EOS (CR, set in the
//! startup script). Set commands (`LA`/`LR`/`M`/`SP`/`V`/…) get no reply; only
//! the `VER`/`GST`/`POS`/`GAST`/`GV` queries return one.
//!
//! ## Units
//!
//! The controller works natively in encoder counts with no resolution scaling
//! (`POS` returns counts, `LA` takes counts), so the asyn-rs motor boundary is
//! counts: positions pass through with `NINT` rounding, the record's `MRES` is
//! 1, and its `EGU` is counts. Velocities cross the boundary in counts/s and
//! accelerations in counts/s²; the controller wants rev/min and rev/s², so they
//! are converted with the axis's counts-per-rev (the `MCDC2805Config`
//! `countsPerRev` argument, which also programs the controller via `ENCRES`).
//! This replaces the C driver's use of the record's `SREV` field, which is not
//! visible at the asyn-rs boundary.
//!
//! ## Done detection
//!
//! There is no reliable "reached target" bit for jog moves or velocity=0 stops,
//! so — exactly as C `set_status` — done is derived from the actual velocity
//! (`GV`) being zero.
//!
//! ## Deviations from C (documented)
//!
//! - The C poller is a background thread; here the `GST`/`POS`/`GAST`/`GV`
//!   reads and the limit-switch hold run inside [`poll`](AsynMotor::poll).
//! - `countsPerRev` is a config argument (sent once via `ENCRES` at init)
//!   rather than the record's `SREV` re-sent before every velocity command.

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PidGainKind};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, nint};

/// Response buffer size (C `BUFF_SIZE`).
const READ_BUF: usize = 128;

/// Command terminator: CR, matching the startup-script output EOS.
const TERMINATOR: &[u8] = b"\r";

/// Maximum modules per serial line (C `MCDC2805_NUM_CARDS` axis table width).
const MAX_MOTORS: usize = 8;

fn faulhaber_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Shared controller endpoint: owns the serial handle and node-prefixed framing.
pub struct FaulhaberController {
    handle: SyncIOHandle,
    ident: String,
    num_axes: usize,
}

impl FaulhaberController {
    /// Connect and probe up to `num_motors` nodes with `VER` (C `motor_init`):
    /// each node is retried three times, and probing stops at the first node
    /// that never answers. The first node's reply becomes the identification.
    /// Performs blocking serial I/O.
    pub fn new(handle: SyncIOHandle, num_motors: usize) -> AsynResult<Self> {
        let mut ctrl = Self {
            handle,
            ident: String::new(),
            num_axes: 0,
        };
        let n = num_motors.min(MAX_MOTORS);
        let mut total = 0;
        for node in 0..n {
            let mut reply = None;
            for _ in 0..3 {
                if let Ok(r) = ctrl.query(node, "VER")
                    && !r.is_empty()
                {
                    reply = Some(r);
                    break;
                }
            }
            match reply {
                Some(r) => {
                    if node == 0 {
                        ctrl.ident = r;
                    }
                    total += 1;
                }
                None => break,
            }
        }
        if total == 0 {
            return Err(faulhaber_err("MCDC2805: no nodes responded to VER"));
        }
        ctrl.num_axes = total;
        Ok(ctrl)
    }

    /// The identification string (node 0's `VER` reply).
    pub fn ident(&self) -> &str {
        &self.ident
    }

    /// Number of responding nodes detected at init.
    pub fn num_axes(&self) -> usize {
        self.num_axes
    }

    fn framed(node: usize, cmd: &str) -> Vec<u8> {
        let prefix = node.to_string();
        let mut out = Vec::with_capacity(prefix.len() + cmd.len() + TERMINATOR.len());
        out.extend_from_slice(prefix.as_bytes());
        out.extend_from_slice(cmd.as_bytes());
        out.extend_from_slice(TERMINATOR);
        out
    }

    /// Send a write-only command to `node` (no reply expected).
    fn write_cmd(&self, node: usize, cmd: &str) -> AsynResult<()> {
        self.handle.write_octet(0, &Self::framed(node, cmd))?;
        Ok(())
    }

    /// Send a query to `node` and return its reply, trimmed of framing.
    fn query(&self, node: usize, cmd: &str) -> AsynResult<String> {
        self.handle.write_octet(0, &Self::framed(node, cmd))?;
        let raw = self.handle.read_octet(0, READ_BUF)?;
        let text = String::from_utf8_lossy(&raw);
        Ok(text
            .trim_matches(|c: char| c.is_control() || c.is_whitespace())
            .to_string())
    }
}

/// Convert a boundary velocity (counts/s) to the controller's rev/min.
fn vel_to_rpm(velocity: f64, counts_per_rev: f64) -> i32 {
    if counts_per_rev <= 0.0 {
        0
    } else {
        nint(velocity / counts_per_rev * 60.0)
    }
}

/// Convert a boundary acceleration (counts/s²) to the controller's rev/s².
fn accel_to_rev(acceleration: f64, counts_per_rev: f64) -> i32 {
    if counts_per_rev <= 0.0 {
        0
    } else {
        nint(acceleration / counts_per_rev)
    }
}

/// One MCDC2805 axis sharing a controller. Implements [`AsynMotor`].
pub struct FaulhaberAxis {
    controller: Arc<Mutex<FaulhaberController>>,
    /// 0-based node number prefixed to this axis's commands.
    node: usize,
    counts_per_rev: f64,
    /// Last integer position (counts); C `motor_info->position`.
    prev_position: i32,
    last_status: MotorStatus,
}

impl FaulhaberAxis {
    /// Construct axis `node` (0-based) and run the C `motor_init` per-axis setup:
    /// RS-232 velocity source (`SOR 0`), limit-switch and homing configuration,
    /// and program the encoder resolution (`ENCRES`). Performs blocking serial
    /// I/O.
    pub fn new(
        controller: Arc<Mutex<FaulhaberController>>,
        node: usize,
        counts_per_rev: f64,
    ) -> AsynResult<Self> {
        {
            let ctrl = controller.lock().unwrap_or_else(|e| e.into_inner());
            // Velocity control source = RS-232.
            ctrl.write_cmd(node, "SOR 0")?;
            // Limit-switch / homing configuration (C motor_init sequence).
            ctrl.write_cmd(node, "REFIN")?;
            ctrl.write_cmd(node, "HP7")?;
            ctrl.write_cmd(node, "HB6")?;
            ctrl.write_cmd(node, "HD2")?;
            ctrl.write_cmd(node, "HL1")?;
            ctrl.write_cmd(node, "HA1")?;
            ctrl.write_cmd(node, "CAHOSEQ")?;
            // Program the encoder resolution once (C sends this before every
            // velocity command using mr->srev).
            ctrl.write_cmd(node, &format!("ENCRES{}", nint(counts_per_rev)))?;
        }
        Ok(Self {
            controller,
            node,
            counts_per_rev,
            prev_position: 0,
            last_status: MotorStatus {
                done: true,
                gain_support: true,
                has_encoder: true,
                ..MotorStatus::default()
            },
        })
    }

    fn lock(&self) -> MutexGuard<'_, FaulhaberController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Program acceleration then speed for an upcoming move, if supplied.
    fn program_motion(
        &self,
        ctrl: &FaulhaberController,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        if acceleration > 0.0 {
            ctrl.write_cmd(
                self.node,
                &format!("AC {}", accel_to_rev(acceleration, self.counts_per_rev)),
            )?;
        }
        if velocity > 0.0 {
            ctrl.write_cmd(
                self.node,
                &format!("SP {}", vel_to_rpm(velocity, self.counts_per_rev)),
            )?;
        }
        Ok(())
    }
}

impl AsynMotor for FaulhaberAxis {
    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let ctrl = self.lock();
        self.program_motion(&ctrl, velocity, acceleration)?;
        ctrl.write_cmd(self.node, &format!("LA {}", nint(position)))?;
        ctrl.write_cmd(self.node, "M")
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
        self.program_motion(&ctrl, velocity, acceleration)?;
        ctrl.write_cmd(self.node, &format!("LR {}", nint(distance)))?;
        ctrl.write_cmd(self.node, "M")
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
    ) -> AsynResult<()> {
        // C JOG: set speed, then enter velocity mode with the signed rate.
        let rpm = vel_to_rpm(velocity, self.counts_per_rev);
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("SP {}", rpm.abs()))?;
        ctrl.write_cmd(self.node, &format!("V {rpm}"))
    }

    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        // C HOME_*: set the homing speed (signed by direction) and start the
        // controller's homing sequence.
        let mut rpm = vel_to_rpm(velocity, self.counts_per_rev);
        if !forward {
            rpm = -rpm;
        }
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("HOSP{rpm}"))?;
        ctrl.write_cmd(self.node, "GOHOSEQ")
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        // C STOP_AXIS: velocity 0.
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, "V 0")
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        // C LOAD_POS: redefine the current position (HO = home offset).
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, &format!("HO {}", nint(position)))
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let ctrl = self.lock();
        ctrl.write_cmd(self.node, if enable { "EN" } else { "DI" })
    }

    fn set_pid_gain(&mut self, _user: &AsynUser, kind: PidGainKind, gain: f64) -> AsynResult<()> {
        // C SET_PGAIN/SET_IGAIN scale the 0..1 gain by 255; derivative is
        // unsupported.
        let ctrl = self.lock();
        match kind {
            PidGainKind::Proportional => {
                ctrl.write_cmd(self.node, &format!("POR {}", nint(gain * 255.0)))
            }
            PidGainKind::Integral => {
                ctrl.write_cmd(self.node, &format!("I {}", nint(gain * 255.0)))
            }
            PidGainKind::Derivative => Ok(()),
        }
    }

    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let ctrl = self.lock();
        let gst = ctrl.query(self.node, "GST");
        let pos = ctrl.query(self.node, "POS");
        let gast = ctrl.query(self.node, "GAST");
        let gv = ctrl.query(self.node, "GV");

        // C: an empty GST reply is a comms failure (RETRY once, then hard error).
        let Ok(status_buff) = gst else {
            drop(ctrl);
            self.last_status = MotorStatus {
                comms_error: true,
                problem: true,
                ..self.last_status.clone()
            };
            return Ok(self.last_status.clone());
        };
        if status_buff.is_empty() {
            drop(ctrl);
            self.last_status = MotorStatus {
                comms_error: true,
                problem: true,
                ..self.last_status.clone()
            };
            return Ok(self.last_status.clone());
        }

        let status_bytes = status_buff.as_bytes();
        let home = status_bytes.get(6) == Some(&b'1');

        let motor_data = pos.as_deref().map(atof).unwrap_or(0.0);
        let new_position = nint(motor_data);
        let direction = if new_position != self.prev_position {
            new_position >= self.prev_position
        } else {
            self.last_status.direction
        };
        let plusdir = direction;

        // Limit switches (GAST: byte 0 = plus, byte 1 = minus).
        let gast_bytes = gast.unwrap_or_default();
        let gast_bytes = gast_bytes.as_bytes();
        let plus_ls = gast_bytes.first() == Some(&b'1');
        let minus_ls = gast_bytes.get(1) == Some(&b'1');
        let ls_active = (plus_ls && plusdir) || (minus_ls && !plusdir);

        // If an active limit switch stopped motion, hold position with an
        // absolute move to the current position (C set_status).
        if ls_active {
            ctrl.write_cmd(self.node, &format!("LA {}", nint(motor_data)))?;
            ctrl.write_cmd(self.node, "M")?;
        }

        // Actual velocity → done (reliable across jog and velocity=0 stops).
        let velocity = gv.as_deref().map(|s| nint(atof(s))).unwrap_or(0);
        let done = velocity == 0;
        drop(ctrl);

        self.prev_position = new_position;
        let signed_velocity = if plusdir { velocity } else { -velocity };

        self.last_status = MotorStatus {
            position: motor_data,
            encoder_position: motor_data,
            velocity: signed_velocity as f64,
            direction,
            done,
            moving: !done,
            high_limit: plus_ls,
            low_limit: minus_ls,
            home,
            comms_error: false,
            problem: false,
            gain_support: true,
            has_encoder: true,
            ..MotorStatus::default()
        };
        Ok(self.last_status.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framed_prefixes_node_and_terminator() {
        assert_eq!(FaulhaberController::framed(0, "GST"), b"0GST\r");
        assert_eq!(FaulhaberController::framed(3, "LA 1000"), b"3LA 1000\r");
    }

    #[test]
    fn vel_to_rpm_converts_counts_per_sec() {
        // 1000 counts/s at 1000 counts/rev = 1 rev/s = 60 rpm.
        assert_eq!(vel_to_rpm(1000.0, 1000.0), 60);
        // Half a rev/s.
        assert_eq!(vel_to_rpm(500.0, 1000.0), 30);
        // Guard against zero counts/rev.
        assert_eq!(vel_to_rpm(1000.0, 0.0), 0);
    }

    #[test]
    fn accel_to_rev_converts_counts_per_sec2() {
        // 3000 counts/s² at 1000 counts/rev = 3 rev/s².
        assert_eq!(accel_to_rev(3000.0, 1000.0), 3);
        assert_eq!(accel_to_rev(3000.0, 0.0), 0);
    }
}
