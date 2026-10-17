//! The model-1 Aerotech Soloist (`drvSoloist.cc` + `devSoloist.cc`):
//! `SoloistSetup` / `SoloistConfig`, the one axis of card `card` reached
//! through `DTYP "Soloist"`.
//!
//! The Soloist answers every `\n`-separated command with its own line: `%`
//! and the value, `!` refused, `#` faulted. Distances go out in user units,
//! steps times the `PosScaleFactor` resolution, with the decimals it implies.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{
    AsynMotor, MotorStatus, PidGainKind, PollFailure, PostMessage,
};
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use motor_common::util::{atof, atoi, nint};

/// `Soloist_NUM_CARDS`.
pub const SOLOIST_NUM_CARDS: i32 = 10;
/// `TIMEOUT`: a `HOME` is answered only once the axis is home.
pub const TIMEOUT: Duration = Duration::from_secs(20);
/// `BUFF_SIZE`.
const BUFF_SIZE: usize = 100;
/// `ASCII_ACK_CHAR`.
const ACK: u8 = b'%';

/// `AXISSTATUS()` bits.
const ENABLED_BIT: i32 = 0x0000_0001;
const IN_POSITION_BIT: i32 = 0x0000_0004;
const IN_MOTION_BIT: i32 = 0x0000_0008;
const DIRECTION_BIT: i32 = 0x0000_0200;
const HOME_LIMIT_BIT: i32 = 0x0100_0000;
const HOME_MARKER_BIT: i32 = 0x0200_0000;
/// `AXISFAULT()` bits.
const CW_FAULT_BIT: i32 = 0x004;
const CCW_FAULT_BIT: i32 = 0x008;

/// C `CommStatus`.
#[derive(Clone, Copy, PartialEq)]
enum CommState {
    Normal,
    Retry,
    CommErr,
}

/// One Soloist: its port and communication state.
pub struct SoloistCard {
    handle: SyncIOHandle,
    addr: i32,
    comm: CommState,
}

impl SoloistCard {
    /// `handle` connected at asyn address `addr`.
    pub fn new(handle: SyncIOHandle, addr: i32) -> Self {
        Self {
            handle,
            addr,
            comm: CommState::Normal,
        }
    }

    /// `send_mess` then `recv_mess`: one reply read per command in `msg`,
    /// concatenated; empty when any of them failed.
    fn ask(&self, msg: &[u8]) -> Vec<u8> {
        let count = msg.split(|&b| b == b'\n').filter(|c| !c.is_empty()).count();
        if count == 0 || self.handle.write_octet(0, msg).is_err() {
            return Vec::new();
        }
        let mut reply = Vec::new();
        for _ in 0..count {
            match self.handle.read_octet(0, BUFF_SIZE) {
                Ok(part) if !part.is_empty() => reply.extend_from_slice(&part),
                _ => return Vec::new(),
            }
        }
        reply
    }

    /// The value of `query`'s `%` reply.
    fn query(&self, query: &str) -> Option<String> {
        let reply = self.ask(query.as_bytes());
        (reply.first() == Some(&ACK)).then(|| String::from_utf8_lossy(&reply[1..]).into_owned())
    }

    /// `pasynOctetSyncIO->flush`.
    fn flush(&self) {
        if let Ok(port) = self.handle.port() {
            let user = AsynUser::new(0).with_addr(self.addr);
            let _ = port.submit_blocking(RequestOp::Flush, user);
        }
    }
}

/// What `motor_init` read of the card's axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisSetup {
    encoder: bool,
    gain: bool,
    /// `drive_resolution`: user units per step.
    resolution: f64,
    /// `res_decpts`.
    digits: usize,
    /// `home_dparam`, the `HomeDirection` parameter.
    home_dparam: u32,
}

/// `motor_init`'s probe and set-up of one card: any reply to `NONE` finds
/// it, an acknowledged `AxisName` its one axis, then the axis is stopped and
/// its parameters read. `None` leaves the card without an axis, as C does.
pub fn motor_init(card: &SoloistCard) -> Option<AxisSetup> {
    card.flush();
    if !(0..3).any(|_| !card.ask(b"NONE").is_empty()) {
        return None;
    }
    // UserString1, C's report ident.
    card.query("GETPARM(265)");
    card.query("GETPARM(257)")?;
    let encoder = card.query("GETPARM(58)").is_some_and(|v| atoi(&v) > 0);
    let gain = card.query("GETPARM(33)").is_some_and(|v| atoi(&v) != 3);
    card.ask(b"ABORT");
    let resolution = card
        .query("GETPARM(3)")
        .map_or(1.0, |v| 1.0 / atof(&v).abs());
    let digits = decimals(resolution);
    // HomeOffset, ThresholdSoftCCW and ThresholdSoftCW, read for nothing.
    for parm in [108, 47, 48] {
        card.query(&format!("GETPARM({parm})"));
    }
    let home_dparam = card.query("GETPARM(106)").map_or(0, |v| atoi(&v) as u32);
    Some(AxisSetup {
        encoder,
        gain,
        resolution,
        digits,
        home_dparam,
    })
}

/// `res_decpts`: two decimals past the resolution's, at least one.
fn decimals(resolution: f64) -> usize {
    ((-resolution.log10()) as i32 + 2).max(1) as usize
}

/// The one axis of a model-1 Soloist.
pub struct DrvSoloistAxis {
    card: Arc<Mutex<SoloistCard>>,
    setup: AxisSetup,
    /// What the last failed poll raises: nothing on C `set_status`'s first
    /// missed reply (RETRY, drvSoloist.cc:232), `CNTRL_COMM_ERR` and
    /// `RA_PROBLEM` on the next or on a reply other than `%` (COMM_ERR,
    /// :237, :246).
    failure: PollFailure,
    status: MotorStatus,
    /// The record's PREM and POST for the move it sends next.
    strings: Option<(Vec<u8>, Option<PostMessage>)>,
    /// The move in flight's POST, `nodeptr->postmsgptr`. C sends it without
    /// reading its reply, which every later status read then takes for its
    /// own; its reply is read away here.
    post: Option<PostMessage>,
}

impl DrvSoloistAxis {
    /// The axis [`motor_init`] set up, its status read as `motor_init` ends.
    pub fn new(card: Arc<Mutex<SoloistCard>>, setup: AxisSetup) -> Self {
        let mut axis = Self {
            card,
            setup,
            failure: PollFailure::NONE,
            status: MotorStatus {
                has_encoder: setup.encoder,
                gain_support: setup.gain,
                ..MotorStatus::default()
            },
            strings: None,
            post: None,
        };
        let _ = axis.poll(&AsynUser::new(0));
        axis
    }

    fn lock(&self) -> MutexGuard<'_, SoloistCard> {
        self.card.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A record command, its replies read away as the motor task does for
    /// a card with `cmnd_response`.
    fn send(&self, command: impl AsRef<[u8]>) {
        self.lock().ask(command.as_ref());
    }

    /// `"%.*f"` of `steps` in user units.
    fn units(&self, steps: f64) -> String {
        format!("{:.*}", self.setup.digits, steps * self.setup.resolution)
    }

    /// `SET_VELOCITY`: the default speed and the home feed rate.
    fn velocity(&self, velocity: f64) {
        let v = self.units(velocity);
        self.send(format!("SETPARM 102, {v}"));
        self.send(format!("SETPARM 107, {v}"));
    }

    /// `SET_ACCEL`, unless zero.
    fn accel(&self, acceleration: f64) {
        if acceleration > 0.0 {
            self.send(format!("SETPARM 103, {}", self.units(acceleration)));
        }
    }

    /// `motion` after the record's PREM line, its POST kept for `poll`.
    fn motion(&mut self, motion: &str) {
        let (prem, post) = self.strings.take().unwrap_or_default();
        self.post = post;
        let mut msg = prem;
        if !msg.is_empty() {
            msg.push(b'\n');
        }
        msg.extend_from_slice(motion.as_bytes());
        self.send(msg);
    }

    fn move_to(&mut self, command: &str, target: f64, velocity: f64, acceleration: f64) {
        self.velocity(velocity);
        self.accel(acceleration);
        let motion = format!("{command} D{}", self.units(target));
        self.motion(&motion);
    }
}

impl AsynMotor for DrvSoloistAxis {
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
        self.move_to("MOVEABS", position, velocity, acceleration);
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
        self.move_to("MOVEINC", distance, velocity, acceleration);
        Ok(())
    }

    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.accel(acceleration);
        let motion = format!("FREERUN F {}", self.units(velocity));
        self.motion(&motion);
        Ok(())
    }

    /// `HomeDirection` set for `forward`, then `HOME`. C sends the PREM
    /// with neither: restarting the transaction for `HOME` drops the PREM
    /// line and the POST pointer with it.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        self.velocity(velocity);
        self.accel(acceleration);
        let hparam = if forward {
            self.setup.home_dparam | 1
        } else {
            self.setup.home_dparam & !1
        };
        self.setup.home_dparam = hparam;
        self.send(format!("SETPARM 106, {}", hparam as i32));
        self.motion("HOME");
        Ok(())
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        self.send("ABORT");
        Ok(())
    }

    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(format!("SETPOSCMD {}", self.units(position)));
        Ok(())
    }

    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        self.send(if enable { "ENABLE" } else { "DISABLE" });
        Ok(())
    }

    /// The Soloist takes no gains over this protocol.
    fn set_pid_gain(&mut self, _user: &AsynUser, _kind: PidGainKind, _gain: f64) -> AsynResult<()> {
        Ok(())
    }

    /// `ThresholdSoftCW`.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(format!("SETPARM 48, {}", self.units(position)));
        Ok(())
    }

    /// `ThresholdSoftCCW`.
    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        self.send(format!("SETPARM 47, {}", self.units(position)));
        Ok(())
    }

    /// `PRIMITIVE`: INIT as it is.
    fn primitive(&mut self, _user: &AsynUser, init: &[u8]) -> AsynResult<()> {
        self.send(init);
        Ok(())
    }

    fn set_move_strings(
        &mut self,
        _user: &AsynUser,
        prem: &[u8],
        post: Option<PostMessage>,
    ) -> AsynResult<()> {
        self.strings = Some((prem.to_vec(), post));
        Ok(())
    }

    /// `set_status`: `AXISSTATUS()`, `AXISFAULT()` and `PFBKPROG()`. A first
    /// missed status fails the poll raising nothing; a second, or a reply
    /// other than `%`, raises the comms error. C zeroes the position when `PFBKPROG()`
    /// is not acknowledged; it is kept here.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let card = self.card.clone();
        let mut ctrl = card.lock().unwrap_or_else(|e| e.into_inner());
        let reply = ctrl.ask(b"AXISSTATUS()");
        if reply.first() != Some(&ACK) {
            (ctrl.comm, self.failure) = if reply.is_empty() && ctrl.comm == CommState::Normal {
                (CommState::Retry, PollFailure::NONE)
            } else {
                (CommState::CommErr, PollFailure::PROBLEM_AND_COMMS_ERROR)
            };
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "Soloist: AXISSTATUS() not acknowledged".into(),
            });
        }
        ctrl.comm = CommState::Normal;
        let axis_status = atoi(&String::from_utf8_lossy(&reply[1..]));
        let fault = ctrl.ask(b"AXISFAULT()");
        let fault = atoi(&String::from_utf8_lossy(fault.get(1..).unwrap_or_default()));
        let pfbk = ctrl.query("PFBKPROG()").map(|v| atof(&v));
        drop(ctrl);

        let status = &mut self.status;
        status.comms_error = false;
        status.slip_stall = false;
        status.direction = axis_status & DIRECTION_BIT != 0;
        status.done = axis_status & IN_POSITION_BIT != 0;
        status.home = axis_status & HOME_LIMIT_BIT != 0;
        status.powered = axis_status & ENABLED_BIT != 0;
        status.encoder_home = axis_status & HOME_MARKER_BIT != 0;
        status.moving = axis_status & IN_MOTION_BIT != 0;
        status.high_limit = fault & CW_FAULT_BIT != 0;
        status.low_limit = fault & CCW_FAULT_BIT != 0;
        let ls_active =
            (status.high_limit && status.direction) || (status.low_limit && !status.direction);
        if let Some(pfbk) = pfbk {
            let data = pfbk / self.setup.resolution;
            if data != status.position {
                status.position = f64::from(nint(data));
                status.encoder_position = if self.setup.encoder {
                    status.position
                } else {
                    0.0
                };
            }
        }
        status.velocity = 0.0;
        status.problem = false;
        let status = status.clone();
        if (status.done || ls_active)
            && let Some(post) = self.post.take().map(|p| p.get())
            && !post.is_empty()
        {
            self.send(post);
        }
        Ok(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digits_follow_the_resolution() {
        assert_eq!(decimals(1.0 / 10000.0), 6);
        assert_eq!(decimals(1.0), 2);
        assert_eq!(decimals(1.0 / 3.0), 2);
        assert_eq!(decimals(1000.0), 1);
    }

    /// drvSoloist.cc:222-251: a first missed `AXISSTATUS()` is RETRY
    /// (nothing raised, nothing posted), the next COMM_ERR, and a reply
    /// clears it.
    #[test]
    fn a_missed_axis_status_is_retry_then_comm_error() {
        use motor_common::testing::LineRig;
        use std::sync::atomic::{AtomicBool, Ordering};
        let quiet = Arc::new(AtomicBool::new(false));
        let q = quiet.clone();
        let (_line, io) = LineRig::new("soloistRetry", move |_| {
            if q.load(Ordering::SeqCst) {
                String::new()
            } else {
                "%0".into()
            }
        });
        let card = Arc::new(Mutex::new(SoloistCard::new(io, 0)));
        let setup = AxisSetup {
            encoder: false,
            gain: false,
            resolution: 1.0,
            digits: 0,
            home_dparam: 0,
        };
        let mut axis = DrvSoloistAxis::new(card, setup);
        let user = AsynUser::new(0);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::PROBLEM_AND_COMMS_ERROR);
        quiet.store(false, Ordering::SeqCst);
        let s = axis.poll(&user).unwrap();
        assert!(!s.comms_error && !s.problem);
        quiet.store(true, Ordering::SeqCst);
        assert!(axis.poll(&user).is_err());
        assert_eq!(axis.poll_failure(), PollFailure::NONE);
    }
}
