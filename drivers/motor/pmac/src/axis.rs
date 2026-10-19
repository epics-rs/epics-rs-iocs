//! A single real (motor) axis of a PMAC, ported from
//! `pmacApp/pmacAsynMotorPortSrc/pmacAxis.cpp` plus the axis-addressed half of
//! `pmacController::writeFloat64` (set-position and soft limits, which C routes
//! through the controller because they arrive as asyn parameters rather than
//! `asynMotorAxis` methods).
//!
//! ## Initial status (framework gap, half ported)
//!
//! `pmacAxis::getAxisInitialStatus` reads `I{n}13 I{n}14 I{n}30 I{n}31 I{n}33`
//! and pushes the controller's soft limits and PID gains *up* into the motor
//! record (`motorHighLimit_`, `motorLowLimit_`, `motorPGain_` …). The asyn-rs
//! [`MotorStatus`] carries no limit or gain readback fields, so there is no
//! boundary to push them through. The exchange is still sent, because only its
//! success sets C's `motorStatusHasEncoder_`. The record-to-controller
//! direction (`set_high_limit` / `set_low_limit`) is ported and does work.

use std::sync::{Arc, Mutex};

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PortSets};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::controller::{ErrorThrottle, PmacController, pmac_err};
use crate::cs_groups::{AxisPoll, DeferredMove};
use crate::protocol::{
    AXIS_GENERAL_PROB2, CID_GEOBRICK, CID_PMAC, IX24_LIMITS_DISABLED, STATUS1_AMP_ENABLED,
    STATUS1_DESIRED_VELOCITY_ZERO, STATUS1_MOTOR_ON, STATUS1_NEG_LIMIT_SET, STATUS1_POS_LIMIT_SET,
    STATUS2_ERR_FOLLOW_ERR, STATUS2_HOME_COMPLETE, STATUS2_IN_POSITION, may_disable_limits,
    parse_axis_status, parse_cid, parse_home_flags_geobrick, parse_home_flags_pmac,
    parse_initial_status, parse_ix24,
};

pub struct PmacAxis {
    controller: Arc<Mutex<PmacController>>,
    axis: i32,
    /// C `limitsDisabled_`: this driver turned the hardware limits off for a
    /// home move and owes the controller a re-enable.
    limits_disabled: bool,
    /// C `previous_position_` / `previous_direction_`, used to derive DIR.
    previous_position: f64,
    previous_direction: bool,
    /// C `amp_enabled_` / `fatal_following_`, read by `stop`.
    amp_enabled: bool,
    fatal_following: bool,
    /// C `motorStatusHasEncoder_`: the initial status read succeeded.
    pub(crate) has_encoder: bool,
    /// The status the last poll posted, which a failed status read keeps.
    last: MotorStatus,
    /// C's per-axis `printErrors` / `printNextError_`.
    throttle: ErrorThrottle,
    /// The controller-port steps not yet handed to the owner.
    sets: PortSets,
}

/// Lock the shared controller. Taken as a free function on the field rather than
/// a `&self` method so that a locked controller and a mutated axis field can
/// coexist (disjoint borrows).
fn lock(controller: &Arc<Mutex<PmacController>>) -> std::sync::MutexGuard<'_, PmacController> {
    controller
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Re-enable the hardware limits this driver disabled for a home move
/// (C ` i{n}24=i{n}24&$FDFFFF`).
fn enable_limits_command(axis: i32) -> String {
    format!("i{axis}24=i{axis}24&$FDFFFF")
}

/// The velocity/acceleration preamble every move shares (C `vel_buff` +
/// `acc_buff`): `I{n}22` is the jog speed in counts/msec, `I{n}20` the jog
/// acceleration *time* in msec.
fn speed_preamble(axis: i32, scale: f64, velocity: f64, acceleration: f64) -> String {
    let mut preamble = String::new();
    if velocity != 0.0 {
        preamble.push_str(&format!(
            "I{axis}22={:.6} ",
            CFloat(velocity.abs() / (scale * 1000.0))
        ));
    }
    if velocity != 0.0 && acceleration != 0.0 {
        preamble.push_str(&format!(
            "I{axis}20={:.6} ",
            CFloat((velocity / acceleration).abs() * 1000.0)
        ));
    }
    preamble
}

impl PmacAxis {
    pub fn new(controller: Arc<Mutex<PmacController>>, axis: i32) -> Self {
        // C getAxisInitialStatus; axis 0 skips it.
        let mut sets = PortSets::default();
        let has_encoder = axis != 0 && {
            let c = lock(&controller);
            let status = c
                .write_read(&format!(
                    "I{axis}13 I{axis}14 I{axis}30 I{axis}31 I{axis}33"
                ))
                .ok()
                .and_then(|resp| parse_initial_status(&resp));
            if let Some(status) = &status {
                sets = c.initial_status_sets(axis, status);
            } else {
                c.trace().print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "pmacAxis::getAxisInitialStatus: Error: initial status poll failed on \
                         axis {axis}.\n"
                    ),
                );
                c.trace().print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "pmacAxis::pmacAxis: getAxisInitialStatus failed to return asynSuccess. \
                         Controller: {}, Axis: {axis}.\n",
                        c.port_name()
                    ),
                );
            }
            status.is_some()
        };
        Self {
            controller,
            axis,
            limits_disabled: false,
            previous_position: 0.0,
            previous_direction: false,
            amp_enabled: false,
            fatal_following: false,
            has_encoder,
            last: MotorStatus::default(),
            throttle: ErrorThrottle::default(),
            sets,
        }
    }

    pub fn axis_number(&self) -> i32 {
        self.axis
    }

    /// C `pmacAxis::move`, both the immediate and the deferred branch.
    fn do_move(
        &mut self,
        position: f64,
        relative: bool,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let axis = self.axis;
        let mut controller = lock(&self.controller);
        let scale = f64::from(controller.axis_scale(axis));
        let mut command = speed_preamble(axis, scale, velocity, acceleration);

        if controller.moves_deferred() {
            let distance = if relative {
                position.abs()
            } else {
                (self.previous_position - position).abs()
            };
            controller.defer_move(
                axis,
                DeferredMove {
                    position: position / scale,
                    relative,
                    time_ms: if velocity != 0.0 {
                        (distance / velocity).abs() * 1000.0
                    } else {
                        0.0
                    },
                },
            );
        } else {
            let jog = if relative { "J^" } else { "J=" };
            command.push_str(&format!(
                "#{axis} {jog}{position:.2}",
                position = CFloat(position / scale)
            ));
        }

        if self.limits_disabled {
            command.push(' ');
            command.push_str(&enable_limits_command(axis));
            self.limits_disabled = false;
        }

        // The deferred branch with neither a speed preamble nor a limits
        // re-enable has nothing to send. C sends the empty string to the
        // controller here; skipping the round trip is the same on the wire.
        if command.trim().is_empty() {
            return Ok(());
        }
        controller.command(&command)
    }

    /// The `REMOVE_LIMITS_ON_HOME` block of C `pmacAxis::home`: ask the
    /// controller what kind of home flag this axis uses and, if the axis homes
    /// *onto* a hardware limit, disable the limits for the duration of the home.
    /// Returns the command suffix to append.
    fn home_limits_suffix(
        &mut self,
        controller: &PmacController,
        velocity: f64,
        forward: bool,
    ) -> AsynResult<String> {
        let axis = self.axis;
        let error = |what: std::fmt::Arguments<'_>| {
            controller.trace().print_self(
                TraceMask::ERROR,
                format_args!(
                    "Controller {} Addr {axis}. pmacAxis::home: ERROR {what}.\n",
                    controller.port_name()
                ),
            );
            pmac_err(what.to_string())
        };
        let cid = controller
            .reply("cid")
            .map_err(|_| error(format_args!("Reading Controller Type")))?;
        // C's controller_type stays 0 when the reply does not scan.
        let cid = parse_cid(&cid).unwrap_or(0);

        let flags = match cid {
            CID_GEOBRICK => {
                // Geobrick LV: I-variables 70{n}2/70{n}3 for axes 1-4,
                // 71{n}2/71{n}3 for axes 5-8.
                let query = if axis < 5 {
                    format!("I70{axis}2 I70{axis}3 i{axis}24 i{axis}23 i{axis}26")
                } else {
                    let n = axis - 4;
                    format!("I71{n}2 I71{n}3 i{axis}24 i{axis}23 i{axis}26")
                };
                controller
                    .reply(&query)
                    .ok()
                    .as_deref()
                    .and_then(parse_home_flags_geobrick)
            }
            CID_PMAC => {
                // VME Turbo PMAC 2: the flags live on the MACRO station.
                let ms = ((axis - 1) / 2) * 4 + (axis - 1) % 2;
                let query = format!("ms{ms},i912 ms{ms},i913 i{axis}24 i{axis}23 i{axis}26");
                controller
                    .reply(&query)
                    .ok()
                    .as_deref()
                    .and_then(parse_home_flags_pmac)
            }
            other => {
                return Err(error(format_args!("Unknown controller type = {other}")));
            }
        };
        let flags = flags.ok_or_else(|| error(format_args!("Cannot Read Home Flags")))?;

        // The record's home velocity wins over the axis's configured i{n}23.
        let home_velocity = if velocity != 0.0 {
            (if forward { 1.0 } else { -1.0 }) * velocity.abs() / 1000.0
        } else {
            flags.home_velocity
        };

        if may_disable_limits(&flags, home_velocity) {
            self.limits_disabled = true;
            Ok(format!(" i{axis}24=i{axis}24|$20000"))
        } else {
            // C homes anyway: an axis that does not home onto a limit simply
            // keeps its limits. C prints `type` in hex and `flag` in decimal.
            controller.trace().print_self(
                TraceMask::ERROR,
                format_args!(
                    "pmacAxis::home: Error: Cannot disable limits to home PMAC controller {}, \
                     axis {axis}, type:{:x}, flag:${}, vel:{home_velocity:.6}, mode:0x{:x}, \
                     offset: {}\n",
                    controller.port_name(),
                    flags.home_type as i32,
                    flags.home_flag as i32,
                    flags.flag_mode,
                    flags.home_offset as i32,
                    home_velocity = CFloat(home_velocity)
                ),
            );
            Ok(String::new())
        }
    }
}

impl AsynMotor for PmacAxis {
    /// C `pmacAxis::getAxisStatus` sets no status bit on a failed read; the
    /// failure goes to the controller's `PMAC_C_COMMSERROR`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::NONE
    }

    /// pmacAxis.cpp:100.
    fn constructor_calls_param_callbacks(&self) -> bool {
        true
    }

    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(position, false, velocity, acceleration)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        self.do_move(distance, true, velocity, acceleration)
    }

    /// C `pmacAxis::moveVelocity`.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let axis = self.axis;
        let controller = lock(&self.controller);
        let mut command = speed_preamble(
            axis,
            f64::from(controller.axis_scale(axis)),
            velocity,
            acceleration,
        );
        let jog = if velocity < 0.0 { "J-" } else { "J+" };
        command.push_str(&format!("#{axis} {jog}"));
        if self.limits_disabled {
            command.push(' ');
            command.push_str(&enable_limits_command(axis));
            self.limits_disabled = false;
        }
        controller.command(&command)
    }

    /// C `pmacAxis::home`.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        _acceleration: f64,
        forward: bool,
    ) -> AsynResult<()> {
        let axis = self.axis;
        let shared = Arc::clone(&self.controller);
        let controller = lock(&shared);
        let suffix = self.home_limits_suffix(&controller, velocity, forward)?;
        controller.command(&format!("#{axis} HOME{suffix}"))
    }

    /// C `pmacAxis::stop`. Only jog-stop an axis whose amplifier is on: a `J/`
    /// would otherwise power up an axis that was deliberately left off. An axis
    /// in a fatal following error is jog-stopped too, to clear it.
    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let axis = self.axis;
        let mut controller = lock(&self.controller);
        controller.clear_deferred(axis);
        if self.amp_enabled || self.fatal_following {
            controller.abort_cs_motion(axis)?;
            controller.command(&format!("#{axis} J/ M{axis}40=1"))
        } else {
            // Just set the in-position bit, so the record stops waiting.
            controller.command(&format!("M{axis}40=1"))
        }
    }

    /// C `pmacController::writeFloat64`, `motorPosition_` branch: rewrite the
    /// axis's actual and target position registers (M{n}62 / M{n}61, both in
    /// 1/32-count units scaled by the axis's I{n}08) and re-close the loop.
    fn set_position(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let controller = lock(&self.controller);
        let scale = f64::from(controller.axis_scale(axis));
        let counts = (position * 32.0 / scale + 0.5).floor() as i64;

        controller.command(&format!(
            "#{axis}K M{axis}61={counts}*I{axis}08 M{axis}62={counts}*I{axis}08"
        ))?;
        controller.command(&format!("#{axis}J/"))?;

        let config = controller.axis_config(axis);
        if config.encoder_axis != 0 {
            let enc = config.encoder_axis;
            let enc_counts = ((counts as f64) * config.encoder_ratio + 0.5).floor() as i64;
            controller.command(&format!(
                "#{enc}K M{enc}61={enc_counts}*I{enc}08 M{enc}62={enc_counts}*I{enc}08"
            ))?;
            controller.command(&format!("#{enc}J/"))?;
        }
        Ok(())
    }

    /// C `pmacAxis::setClosedLoop`.
    fn set_closed_loop(&mut self, _user: &AsynUser, enable: bool) -> AsynResult<()> {
        let axis = self.axis;
        let command = if enable {
            format!("#{axis} J/")
        } else {
            format!("#{axis} K")
        };
        lock(&self.controller).command(&command)
    }

    /// C `pmacController::writeFloat64`, `motorHighLimit_` branch.
    fn set_high_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let controller = lock(&self.controller);
        let scale = f64::from(controller.axis_scale(axis));
        controller.command(&format!(
            "I{axis}13={position:.6}",
            position = CFloat(position / scale)
        ))
    }

    /// C `pmacController::writeFloat64`, `motorLowLimit_` branch.
    fn set_low_limit(&mut self, _user: &AsynUser, position: f64) -> AsynResult<()> {
        let axis = self.axis;
        let controller = lock(&self.controller);
        let scale = f64::from(controller.axis_scale(axis));
        controller.command(&format!(
            "I{axis}14={position:.6}",
            position = CFloat(position / scale)
        ))
    }

    /// C `pmacController::writeInt32`, `motorDeferMoves_` branch, which C takes
    /// at every address: the motor record defers with 1, releases with 0.
    fn set_deferred_moves(&mut self, _user: &AsynUser, defer: bool) -> AsynResult<()> {
        let mut controller = lock(&self.controller);
        let release = controller.set_moves_deferred(i32::from(defer));
        if let Some(value) = release.comms_error {
            self.sets.append(controller.comms_error_sets(value));
        }
        release.status
    }

    /// C `pmacAxis::getAxisStatus`.
    fn poll(&mut self, _user: &AsynUser) -> AsynResult<MotorStatus> {
        let axis = self.axis;
        let mut controller = lock(&self.controller);
        let config = controller.axis_config(axis);
        let controller_problem = controller.controller_problem();
        let print_errors = self.throttle.print_errors();

        // One round trip for the two status words, the following error and the
        // position. With an open-loop axis the encoder position comes back from
        // a different motor, so ask that one for its position instead of asking
        // this one for its following error.
        let query = if config.encoder_axis != 0 {
            format!("#{axis} ? P #{} P", config.encoder_axis)
        } else {
            format!("#{axis} ? F P")
        };
        let read = controller.reply(&query);
        let parsed = read.as_deref().ok().and_then(parse_axis_status);
        let Some((status1, status2, first, second)) = parsed else {
            // C's poll returns before the read with no low-level user.
            if controller.connected() {
                let (status, response) = match &read {
                    Ok(reply) => (0, reply.as_str()),
                    Err(e) => (e.c_status_code(), ""),
                };
                controller.trace().print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "drvPmacAxisGetStatus: not all status values returned. Status: \
                         {status}\nCommand :{query}\nResponse:{response}"
                    ),
                );
            }
            // C keeps every status parameter but the power-on bit, which it
            // reads from the zeroed status words, and returns success
            // (pmacAxis.cpp:494-498, 606-613).
            self.amp_enabled = false;
            self.last.powered = false;
            self.last.moving = false;
            return Ok(self.last.clone());
        };

        // Closed-loop axis: the third field is the following error and the
        // position is the sum of the two. Open-loop axis: they are the two
        // motors' positions, already absolute.
        let (position, encoder_position) = if config.encoder_axis != 0 {
            (first, second)
        } else {
            (first + second, second)
        };
        let scale = f64::from(controller.axis_scale(axis));
        let (position, encoder_position) = (position * scale, encoder_position * scale);

        let direction = if position > self.previous_position {
            true
        } else if position == self.previous_position {
            self.previous_direction
        } else {
            false
        };
        self.previous_position = position;
        self.previous_direction = direction;

        let flags = decode_axis_status(status1, status2, controller.is_deferred(axis));
        self.amp_enabled = flags.amp_enabled;
        self.fatal_following = flags.following_error;

        let mut problem = flags.general_problem || controller_problem;
        // If the hardware limits are off and *we* did not turn them off to home,
        // something else did: that is a problem the operator must see.
        if !config.limits_check_disabled
            && !self.limits_disabled
            // C skips the check when the read fails (pmacAxis.cpp:578-579).
            && let Some(ix24) = controller
                .write_read(&format!("i{axis}24"))
                .ok()
                .as_deref()
                .and_then(parse_ix24)
            && ix24 & IX24_LIMITS_DISABLED != 0
        {
            problem = true;
            if print_errors {
                controller.trace().print_self(
                    TraceMask::ERROR,
                    format_args!(
                        "*** WARNING *** Limits are disabled on controller {}, axis {axis}\n",
                        controller.port_name()
                    ),
                );
                self.throttle.print_next = false;
            }
        }
        if !problem {
            self.throttle.print_next = true;
        }

        // The home move that borrowed the limits has finished: give them back.
        if self.limits_disabled
            && flags.home_complete
            && flags.desired_velocity_zero
            && controller.command(&enable_limits_command(axis)).is_ok()
        {
            self.limits_disabled = false;
        }

        controller.record_poll(
            axis,
            AxisPoll {
                position,
                moving: !flags.done,
            },
        );

        self.last = MotorStatus {
            position,
            encoder_position,
            done: flags.done,
            // C paces on `!done` (pmacAxis.cpp:538-542) and sets
            // motorStatusMoving_ from the velocity/amp bits (548).
            moving: !flags.done,
            msta_moving: flags.moving,
            high_limit: flags.high_limit,
            low_limit: flags.low_limit,
            homed: flags.home_complete,
            powered: flags.amp_enabled,
            problem,
            direction,
            slip_stall: flags.following_error,
            gain_support: true,
            has_encoder: self.has_encoder,
            ..MotorStatus::default()
        };
        Ok(self.last.clone())
    }
}

/// What the two axis status words mean, before the controller-wide problem bits
/// are folded in (C `pmacAxis::getAxisStatus`, the `setIntegerParam` block).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AxisFlags {
    done: bool,
    moving: bool,
    high_limit: bool,
    low_limit: bool,
    home_complete: bool,
    amp_enabled: bool,
    desired_velocity_zero: bool,
    following_error: bool,
    general_problem: bool,
}

fn decode_axis_status(status1: u32, status2: u32, deferred: bool) -> AxisFlags {
    let amp_enabled = status1 & STATUS1_AMP_ENABLED != 0;
    let motor_on = status1 & STATUS1_MOTOR_ON != 0;
    let desired_velocity_zero = status1 & STATUS1_DESIRED_VELOCITY_ZERO != 0;

    // A deferred axis is never done: its move has not been sent yet.
    let done = if deferred {
        false
    } else {
        // An amplifier that dropped out mid-move will never reach position; call
        // it done so the record stops instead of piling up following errors.
        status2 & STATUS2_IN_POSITION != 0 || !motor_on || !amp_enabled
    };

    AxisFlags {
        done,
        moving: !desired_velocity_zero && motor_on && amp_enabled,
        high_limit: status1 & STATUS1_POS_LIMIT_SET != 0,
        low_limit: status1 & STATUS1_NEG_LIMIT_SET != 0,
        home_complete: status2 & STATUS2_HOME_COMPLETE != 0,
        amp_enabled,
        desired_velocity_zero,
        following_error: status2 & STATUS2_ERR_FOLLOW_ERR != 0,
        general_problem: status2 & AXIS_GENERAL_PROB2 != 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_constructor_calls_back() {
        let controller = PmacController::new("pmacCallback", "pmacCallbackLow", 0, None);
        let axis = PmacAxis::new(Arc::new(Mutex::new(controller)), 1);
        assert!(axis.constructor_calls_param_callbacks());
    }

    #[test]
    fn speed_preamble_matches_c_format() {
        // I{n}22 is counts/msec, I{n}20 the acceleration time in msec.
        assert_eq!(
            speed_preamble(3, 1.0, 2000.0, 1000.0),
            "I322=2.000000 I320=2000.000000 "
        );
    }

    #[test]
    fn speed_preamble_omits_unset_fields() {
        assert_eq!(speed_preamble(1, 1.0, 0.0, 500.0), "");
        // C only emits the acceleration when a velocity came with it: I{n}20 is
        // a *time*, and without the velocity there is nothing to divide by.
        assert_eq!(speed_preamble(1, 1.0, 1000.0, 0.0), "I122=1.000000 ");
    }

    #[test]
    fn speed_preamble_uses_the_magnitude_of_a_reverse_jog() {
        assert_eq!(
            speed_preamble(2, 1.0, -2000.0, 1000.0),
            "I222=2.000000 I220=2000.000000 "
        );
    }

    #[test]
    fn enable_limits_command_clears_bit_17() {
        assert_eq!(enable_limits_command(7), "i724=i724&$FDFFFF");
    }

    /// A powered, closed-loop, moving axis: motor on, amp on, desired velocity
    /// non-zero, not in position.
    const MOVING1: u32 = STATUS1_MOTOR_ON | STATUS1_AMP_ENABLED;

    #[test]
    fn a_moving_axis_is_not_done() {
        let flags = decode_axis_status(MOVING1, 0, false);
        assert!(!flags.done);
        assert!(flags.moving);
        assert!(flags.amp_enabled);
    }

    /// Desired velocity zero but not yet in position: C keeps polling fast
    /// (`!done`) with motorStatusMoving_ already 0.
    #[test]
    fn a_settling_axis_is_not_done_and_not_moving() {
        let flags = decode_axis_status(MOVING1 | STATUS1_DESIRED_VELOCITY_ZERO, 0, false);
        assert!(!flags.done);
        assert!(!flags.moving);
    }

    #[test]
    fn in_position_is_done_and_not_moving() {
        let flags = decode_axis_status(
            MOVING1 | STATUS1_DESIRED_VELOCITY_ZERO,
            STATUS2_IN_POSITION,
            false,
        );
        assert!(flags.done);
        assert!(!flags.moving);
    }

    #[test]
    fn an_amp_that_dropped_out_mid_move_reports_done() {
        // Motor on, still commanded to move, but the amplifier is off and the
        // axis is nowhere near position: without this the record would wait for
        // a move that can never finish.
        let flags = decode_axis_status(STATUS1_MOTOR_ON, 0, false);
        assert!(flags.done);
        assert!(!flags.moving);
        assert!(!flags.amp_enabled);
    }

    #[test]
    fn a_deactivated_motor_reports_done() {
        // ix00 = 0: the motor is not activated at all.
        let flags = decode_axis_status(STATUS1_AMP_ENABLED, 0, false);
        assert!(flags.done);
        assert!(!flags.moving);
    }

    #[test]
    fn a_deferred_axis_is_never_done_even_when_in_position() {
        // The demand is still sitting in the controller's deferred store: the
        // record must not see DONE before the move has even been sent.
        let flags = decode_axis_status(
            MOVING1 | STATUS1_DESIRED_VELOCITY_ZERO,
            STATUS2_IN_POSITION,
            true,
        );
        assert!(!flags.done);
    }

    #[test]
    fn limits_home_and_error_bits_map_straight_through() {
        let flags = decode_axis_status(
            MOVING1 | STATUS1_POS_LIMIT_SET | STATUS1_NEG_LIMIT_SET,
            STATUS2_HOME_COMPLETE | STATUS2_ERR_FOLLOW_ERR,
            false,
        );
        assert!(flags.high_limit);
        assert!(flags.low_limit);
        assert!(flags.home_complete);
        assert!(flags.following_error);
        // A following error is not, by itself, one of the general problem bits
        // (C PMAX_AXIS_GENERAL_PROB2 is desired-stop | amp-fault); it reaches the
        // record as MSTA's slip/stall bit instead.
        assert!(!flags.general_problem);
    }

    #[test]
    fn an_amp_fault_is_a_general_problem() {
        use crate::protocol::{STATUS2_AMP_FAULT, STATUS2_DESIRED_STOP};
        assert!(decode_axis_status(MOVING1, STATUS2_AMP_FAULT, false).general_problem);
        assert!(decode_axis_status(MOVING1, STATUS2_DESIRED_STOP, false).general_problem);
    }
}
