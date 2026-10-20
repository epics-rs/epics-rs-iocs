//! nPoint LC400 controller driver (`LC400MotorDriver.cpp`, a model-3
//! `asynMotorController`).
//!
//! The LC400 is a register machine spoken to in packed little-endian binary
//! frames, each ending in `0x55`: `A0 addr` reads one 32-bit word (answered
//! `A0 addr value 55`), `A2 addr value` writes one, `A3 value` writes the next
//! word of an array started by `A2`, and `A4 addr count` reads `count` words
//! (answered `A4 addr`, the words, then `55`).
//!
//! A move is a wavetable: a trapezoidal profile of up to `LC400_W_MAXPTS`
//! points is written to the channel's table, read back and compared, then
//! played once.

use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interfaces::motor::{AsynMotor, MotorStatus, PollFailure, PortSets};
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{
    AxisUser, ControllerParams, ControllerReport, FinishedWrite, ParamWrite, WriteHook,
    WriteOutcome, WriteValue, has_motor_axis,
};

/// `LC400_TIMEOUT`.
pub const LC400_TIMEOUT: Duration = Duration::from_secs(5);
/// `LC400_MIN_DELAY`: one wavetable cycle, in seconds.
const MIN_DELAY: f64 = 24e-6;
/// `LC400_MAX_ARRAY`: the most points C's waveform buffers hold.
const MAX_ARRAY: i32 = 83300;
/// The end-of-frame byte.
const EOS: u8 = 0x55;

/// The firmware string's address and length in words.
const FW_ADDR: u32 = 0x1183_0028;
const FW_WORDS: usize = 5;

// Per-channel register offsets (LC400MemoryAddr.h).
const ST_RANGE_CMD: u32 = 0x078;
const ST_RANGE_TYPE: u32 = 0x044;
const ST_DIGITAL_SP: u32 = 0x218;
const ST_DIGITAL_POS: u32 = 0x334;
const SERVO_STATE: u32 = 0x084;
const P_GAIN: u32 = 0x720;
const I_GAIN: u32 = 0x728;
const D_GAIN: u32 = 0x730;
const WAV_ENABLE: u32 = 0x1F4;
const WAV_INDEX: u32 = 0x1F8;
const WAV_DELAY: u32 = 0x200;
const WAV_END: u32 = 0x204;
const WAV_ACTIVE: u32 = 0x208;
const WAV_ITERATIONS: u32 = 0xD50;
const WAV_COUNT: u32 = 0xD54;

/// `getBaseAddress`: channel registers of axes 0-2, `None` beyond.
fn base_address(axis: usize) -> Option<u32> {
    [0x1183_1000, 0x1183_2000, 0x1183_3000].get(axis).copied()
}

/// `getWavetableAddress`.
fn wavetable_address(axis: usize) -> Option<u32> {
    [0xC000_0000, 0xC005_4000, 0xC00A_8000].get(axis).copied()
}

fn lc400_err(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// The reasons of the parameters `LC400Controller` creates.
#[derive(Clone, Copy)]
struct Lc400Params {
    range: usize,
    range_type: usize,
    digital_pos: usize,
    servo_state: usize,
    p_gain: usize,
    i_gain: usize,
    d_gain: usize,
    firmware: usize,
    w_enable: usize,
    w_index: usize,
    w_delay: usize,
    w_end_idx: usize,
    w_active: usize,
    w_iter: usize,
    w_iter_cts: usize,
    w_max_pts: usize,
}

/// What C keeps per axis, in `LC400Axis` or in its parameters.
#[derive(Default)]
struct AxisState {
    /// `range`: the stage range, read once at creation.
    range: i32,
    /// `posHardLimit`, `negHardLimit`, `tolHardLimit`, in counts.
    pos_hard: i32,
    neg_hard: i32,
    tol_hard: i32,
    /// `motorStatusHighLimit_` / `motorStatusLowLimit_`, which a poll
    /// latches and a move reads.
    high_limit: bool,
    low_limit: bool,
    /// `motorStatusDone_`, as the last poll set it.
    done: bool,
    /// `LC400_DIGITAL_POS`, as the last poll set it: where a move starts.
    digital_pos: f64,
    /// `LC400_W_MAXPTS`.
    max_pts: i32,
}

impl AxisState {
    /// `getCts`.
    fn cts(&self, position: f64) -> i32 {
        (position * 1_048_574.0 / f64::from(self.range)) as i32
    }
}

/// One LC400 and its axes.
pub struct Lc400Controller {
    /// `None` when C's connect failed, which C reports and carries on from.
    handle: Option<SyncIOHandle>,
    params: Option<Lc400Params>,
    firmware: Vec<u8>,
    axes: Vec<AxisState>,
}

impl Lc400Controller {
    /// `LC400Controller` and its `LC400Axis` constructors: read the firmware
    /// string and each axis's range.
    pub fn new(handle: Option<SyncIOHandle>, num_axes: usize) -> Self {
        let mut controller = Self {
            handle,
            params: None,
            firmware: Vec::new(),
            axes: (0..num_axes).map(|_| AxisState::default()).collect(),
        };
        match controller.read_array(FW_ADDR, FW_WORDS) {
            Ok(words) => {
                let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
                let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
                controller.firmware = bytes[..end].to_vec();
            }
            Err(_) => epics_rs::base::stderr_println!(
                "LC400MotorDriver:LC400Controller: cannot read LC400 controller firmware version"
            ),
        }
        println!(
            "Firmware Version: {}",
            String::from_utf8_lossy(&controller.firmware)
        );
        for axis in 0..num_axes {
            if let Some(address) = base_address(axis) {
                controller.axes[axis].range = controller
                    .read_single(address + ST_RANGE_CMD)
                    .unwrap_or_default();
            }
            controller.set_hard_limits(axis, 0.0, 0.0, 0.0);
        }
        controller
    }

    pub fn num_axes(&self) -> usize {
        self.axes.len()
    }

    fn handle(&self) -> AsynResult<&SyncIOHandle> {
        self.handle
            .as_ref()
            .ok_or_else(|| lc400_err("not connected to the LC400"))
    }

    /// Read exactly `len` bytes: C issues one read for a whole frame, which
    /// leaves the rest of its buffer unset when the frame arrives in pieces.
    fn read_exact(&self, len: usize) -> AsynResult<Vec<u8>> {
        let handle = self.handle()?;
        let mut buf = Vec::with_capacity(len);
        while buf.len() < len {
            let (chunk, status) = handle.read_octet(0, len - buf.len()).into_parts();
            status?;
            if chunk.is_empty() {
                return Err(lc400_err("short LC400 reply"));
            }
            buf.extend_from_slice(&chunk);
        }
        Ok(buf)
    }

    /// `readSingle`.
    fn read_single(&self, addr: u32) -> AsynResult<i32> {
        let mut out = vec![0xA0];
        out.extend_from_slice(&addr.to_le_bytes());
        out.push(EOS);
        self.handle()?.write_octet(0, &out)?;
        let reply = self.read_exact(10)?;
        Ok(i32::from_le_bytes([reply[5], reply[6], reply[7], reply[8]]))
    }

    /// `writeSingle`.
    fn write_single(&self, addr: u32, value: i32) -> AsynResult<()> {
        let mut out = vec![0xA2];
        out.extend_from_slice(&addr.to_le_bytes());
        out.extend_from_slice(&value.to_le_bytes());
        out.push(EOS);
        self.handle()?.write_octet(0, &out).map(drop)
    }

    /// `writeArray`: the first word with `A2`, the rest as one `A3` run.
    fn write_array(&self, addr: u32, values: &[i32]) -> AsynResult<()> {
        let Some((first, rest)) = values.split_first() else {
            return Ok(());
        };
        self.write_single(addr, *first)?;
        let out: Vec<u8> = rest
            .iter()
            .flat_map(|v| {
                let mut frame = vec![0xA3];
                frame.extend_from_slice(&v.to_le_bytes());
                frame.push(EOS);
                frame
            })
            .collect();
        self.handle()?.write_octet(0, &out).map(drop)
    }

    /// `readArray`.
    fn read_array(&self, addr: u32, count: usize) -> AsynResult<Vec<i32>> {
        let mut out = vec![0xA4];
        out.extend_from_slice(&addr.to_le_bytes());
        out.extend_from_slice(&(count as u32).to_le_bytes());
        out.push(EOS);
        let (header, status) = self.handle()?.write_read(0, &out, 5).into_parts();
        status?;
        if header.len() < 5 {
            self.read_exact(5 - header.len())?;
        }
        let words = self.read_exact(4 * count)?;
        self.read_exact(1)?;
        Ok(words
            .as_chunks::<4>()
            .0
            .iter()
            .map(|w| i32::from_le_bytes(*w))
            .collect())
    }

    /// A float register: two words, low first, as C's `doubleToInt` union.
    fn read_double(&self, addr: u32) -> AsynResult<f64> {
        let lo = self.read_single(addr)? as u32;
        let hi = self.read_single(addr + 4)? as u32;
        Ok(f64::from_bits(u64::from(hi) << 32 | u64::from(lo)))
    }

    fn write_double(&self, addr: u32, value: f64) -> AsynResult<()> {
        let bits = value.to_bits();
        self.write_single(addr, bits as u32 as i32)?;
        self.write_single(addr + 4, (bits >> 32) as u32 as i32)
    }

    /// `LC400Axis::setHardLimits`.
    fn set_hard_limits(&mut self, axis: usize, pos: f64, neg: f64, tol: f64) {
        let state = &mut self.axes[axis];
        state.pos_hard = state.cts(pos);
        state.neg_hard = state.cts(neg);
        state.tol_hard = state.cts(tol);
        println!(
            "hardLimits pos,neg,tol = {},{},{}",
            state.pos_hard, state.neg_hard, state.tol_hard
        );
    }

    /// `updateAuxParams`: the static parameters of `axis` into `set` as
    /// each is read, only while every axis is done; a failed read ends it
    /// with those before it kept (LC400MotorDriver.cpp:500-517).
    fn aux_params(&self, address: u32, set: &mut Vec<(usize, ParamValue)>) -> AsynResult<()> {
        let Some(p) = self.params else {
            return Ok(());
        };
        if self.axes.iter().any(|a| !a.done) {
            return Ok(());
        }
        for (reason, offset) in [
            (p.range, ST_RANGE_CMD),
            (p.range_type, ST_RANGE_TYPE),
            (p.servo_state, SERVO_STATE),
        ] {
            set.push((
                reason,
                ParamValue::Int32(self.read_single(address + offset)?),
            ));
        }
        for (reason, offset) in [(p.p_gain, P_GAIN), (p.i_gain, I_GAIN), (p.d_gain, D_GAIN)] {
            set.push((
                reason,
                ParamValue::Float64(self.read_double(address + offset)?),
            ));
        }
        Ok(())
    }
}

/// `LC400ConfigAxis`: hard limits of `axis`, in position units.
pub fn config_axis(controller: &Mutex<Lc400Controller>, axis: i32, high: f64, low: f64, tol: f64) {
    let mut ctrl = lock(controller);
    if axis < 0 || axis as usize >= ctrl.num_axes() {
        println!("invalid axis number: {axis}");
        return;
    }
    ctrl.set_hard_limits(axis as usize, high, low, tol);
}

fn lock(controller: &Mutex<Lc400Controller>) -> MutexGuard<'_, Lc400Controller> {
    controller.lock().unwrap_or_else(|e| e.into_inner())
}

/// `genTrapezoid`'s waveform: `points[0]` is overwritten by the caller.
struct Waveform {
    cycle_count: usize,
    points: Vec<i32>,
}

/// `genTrapezoid`: `ipos` to `fpos` in at most `max_pts` points of a
/// trapezoidal velocity profile, each held `cycle_count + 1` cycles.
fn gen_trapezoid(ipos: f64, fpos: f64, mut vel: f64, acc: f64, max_pts: usize) -> Waveform {
    let d_tot = (fpos - ipos).abs();
    let mut t_acc = vel / acc;
    let mut d_acc = vel * vel / (2.0 * acc);
    let mut d_cnt = d_tot - 2.0 * d_acc;
    let mut t_cnt = d_cnt / vel;
    if t_cnt < 0.0 {
        t_cnt = 0.0;
        d_cnt = 0.0;
        d_acc = d_tot / 2.0;
        t_acc = (2.0 * d_acc / acc).sqrt();
        vel = acc * t_acc;
    }
    let t_tot = 2.0 * t_acc + t_cnt;
    let mut cycle_count = (t_tot / MIN_DELAY / max_pts as f64).ceil() as usize;
    let t_per_cycle = cycle_count as f64 * MIN_DELAY;
    let npts = ((t_tot / t_per_cycle).floor() as usize).min(max_pts);

    // C fills data[i+1], and the last point into data[0]; one spare slot.
    let mut data = vec![0i32; npts + 1];
    let dir = if fpos > ipos { 1.0 } else { -1.0 };
    for i in 0..npts {
        let t = i as f64 * t_per_cycle;
        if i < npts / 2 && t < t_acc {
            data[i + 1] = (ipos + dir * acc * t * t / 2.0) as i32;
        } else if t < t_acc + t_cnt {
            let tt = t - t_acc;
            data[i + 1] = (ipos + dir * (d_acc + vel * tt)) as i32;
        } else {
            let tt = t - t_acc - t_cnt;
            let at = if i < npts - 1 { i + 1 } else { 0 };
            data[at] = (ipos + dir * (d_acc + d_cnt + vel * tt - acc * tt * tt / 2.0)) as i32;
        }
    }
    if cycle_count == 0 {
        epics_rs::base::stderr_println!(
            "Cycle Count was calculated to be 0, which shouldn't happen. Setting it to 1"
        );
        cycle_count = 1;
    }
    data.truncate(npts);
    Waveform {
        cycle_count: cycle_count - 1,
        points: data,
    }
}

/// Where a move goes: relative moves are from the last polled position.
#[derive(Clone, Copy)]
enum Target {
    Absolute(f64),
    Relative(f64),
}

/// One LC400 channel.
pub struct Lc400Axis {
    controller: Arc<Mutex<Lc400Controller>>,
    axis: usize,
    /// What the last poll or move set, for the poller's callParamCallbacks.
    sets: PortSets,
}

impl Lc400Axis {
    pub fn new(controller: Arc<Mutex<Lc400Controller>>, axis: usize) -> Self {
        Self {
            controller,
            axis,
            sets: PortSets::default(),
        }
    }

    /// The base's `motorStatusDone_` = 0 after a move start, refused or not
    /// (asynMotorController.cpp:301-353), on the DONE updateAuxParams reads.
    fn started(&self, moved: AsynResult<()>) -> AsynResult<()> {
        lock(&self.controller).axes[self.axis].done = false;
        moved
    }

    /// `LC400Axis::move`.
    fn move_to(&mut self, target: Target, velocity: f64, acceleration: f64) -> AsynResult<()> {
        let (Some(ch), Some(wav)) = (base_address(self.axis), wavetable_address(self.axis)) else {
            return Err(lc400_err(format!("LC400 has no axis {}", self.axis)));
        };
        // genTrapezoid asserts both; a zero rate would divide by it.
        if velocity <= 0.0 || acceleration <= 0.0 {
            return Err(lc400_err(
                "LC400 move needs a positive velocity and acceleration",
            ));
        }
        let mut updates = Vec::new();
        let ctrl = lock(&self.controller);
        let result = ctrl.play(
            self.axis,
            (ch, wav),
            target,
            velocity,
            acceleration,
            &mut updates,
        );
        drop(ctrl);
        // C sets each readback as it reads it; whatever was read before a
        // failure reaches the next callParamCallbacks.
        for (reason, value) in updates {
            self.sets.axis(reason, value);
        }
        result
    }
}

/// `LC400Axis::stop`'s writes, which `move` starts with: the status of the
/// last, C ignoring the others.
fn halt(ctrl: &Lc400Controller, ch: u32, wav: u32) -> AsynResult<()> {
    let _ = ctrl.write_single(ch + WAV_ACTIVE, 0);
    // C writes its uninitialised `currentPos` when the read fails
    // (LC400MotorDriver.cpp:679-680, :703-704); not copied, the write is
    // skipped.
    if let Ok(current) = ctrl.read_single(ch + ST_DIGITAL_POS) {
        let _ = ctrl.write_single(wav, current);
    }
    ctrl.write_single(ch + WAV_INDEX, 0)
}

impl Lc400Controller {
    /// The body of `LC400Axis::move` after its axis checks, on the channel and
    /// wavetable addresses `(ch, wav)`.
    fn play(
        &self,
        axis: usize,
        (ch, wav): (u32, u32),
        target: Target,
        velocity: f64,
        acceleration: f64,
        updates: &mut Vec<(usize, ParamValue)>,
    ) -> AsynResult<()> {
        let halted = halt(self, ch, wav);
        let state = &self.axes[axis];
        let initial = state.digital_pos;
        let mut position = match target {
            Target::Absolute(position) => position,
            Target::Relative(distance) => initial + distance,
        };
        if (position > initial && state.high_limit) || (position < initial && state.low_limit) {
            return halted;
        }
        if state.pos_hard != 0 && position > f64::from(state.pos_hard) {
            position = f64::from(state.pos_hard);
        } else if state.neg_hard != 0 && position < f64::from(state.neg_hard) {
            position = f64::from(state.neg_hard);
        }
        // C divides by LC400_W_MAXPTS when it is 0 and overruns its
        // LC400_MAX_ARRAY buffers above it.
        if !(1..=MAX_ARRAY).contains(&state.max_pts) {
            return Err(lc400_err(format!(
                "LC400_W_MAXPTS {} is outside 1..={MAX_ARRAY}",
                state.max_pts
            )));
        }
        let mut points = gen_trapezoid(
            initial,
            position,
            velocity,
            acceleration,
            state.max_pts as usize,
        );
        // A move too short for one point leaves C's writeArray counting down
        // from 0 - 1; there is nothing to play.
        let Some(first) = points.points.first_mut() else {
            return halted;
        };
        *first = initial as i32;
        self.write_array(wav, &points.points)?;
        // The controller needs time to take the array.
        sleep(Duration::from_millis(10));
        let readback = self.read_array(wav, points.points.len())?;
        if let Some(i) = (0..points.points.len()).find(|&i| points.points[i] != readback[i]) {
            println!(
                "something is wrong with array pos[{i}] wrote {} read {}",
                points.points[i], readback[i]
            );
            return Err(lc400_err("LC400 wavetable readback differs"));
        }

        self.write_single(ch + WAV_ACTIVE, 0)?;
        self.write_single(ch + WAV_ENABLE, 2)?;
        self.write_single(ch + WAV_DELAY, points.cycle_count as i32)?;
        self.write_single(ch + WAV_END, points.points.len() as i32 - 1)?;
        self.write_single(ch + WAV_ITERATIONS, 1)?;
        self.write_single(ch + WAV_COUNT, 1)?;

        // C reads WAV_ACTIVE a second time for LC400_W_ENABLE; the enable
        // register is read here.
        if let Some(p) = self.params {
            for (reason, register) in [
                (p.w_active, WAV_ACTIVE),
                (p.w_enable, WAV_ENABLE),
                (p.w_index, WAV_INDEX),
                (p.w_delay, WAV_DELAY),
                (p.w_end_idx, WAV_END),
                (p.w_iter, WAV_ITERATIONS),
                (p.w_iter_cts, WAV_COUNT),
            ] {
                updates.push((reason, ParamValue::Int32(self.read_single(ch + register)?)));
            }
        }

        self.write_single(ch + WAV_ACTIVE, 1)?;
        // C spins until the table starts, forever when the start write or a
        // read fails (LC400MotorDriver.cpp:797-805); either failure ends the
        // wait here.
        while self.read_single(ch + WAV_INDEX)? == 0 {
            sleep(Duration::from_millis(10));
        }
        self.write_single(wav, position as i32)
    }
}

impl AsynMotor for Lc400Axis {
    /// C `LC400Axis::poll`'s `skip` label sets only `motorStatusProblem_`.
    fn poll_failure(&self) -> PollFailure {
        PollFailure::PROBLEM
    }

    /// LC400MotorDriver.cpp:666.
    fn constructor_calls_param_callbacks(&self) -> bool {
        true
    }

    fn move_absolute(
        &mut self,
        _user: &AsynUser,
        position: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.move_to(Target::Absolute(position), velocity, acceleration);
        self.started(moved)
    }

    fn move_relative(
        &mut self,
        _user: &AsynUser,
        distance: f64,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let moved = self.move_to(Target::Relative(distance), velocity, acceleration);
        self.started(moved)
    }

    /// A move to the hard limit, or half the range when none is set. C
    /// reads axis 0's limit bits for every axis; each reads its own here.
    fn move_velocity(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        velocity: f64,
        acceleration: f64,
    ) -> AsynResult<()> {
        let target = {
            let ctrl = lock(&self.controller);
            let state = &ctrl.axes[self.axis];
            if (velocity > 0.0 && state.high_limit) || (velocity < 0.0 && state.low_limit) {
                drop(ctrl);
                return self.started(Ok(()));
            }
            let half = f64::from(state.cts(f64::from(state.range / 2)));
            if velocity < 0.0 {
                if state.neg_hard != 0 {
                    f64::from(state.neg_hard)
                } else {
                    -half
                }
            } else if state.pos_hard != 0 {
                f64::from(state.pos_hard)
            } else {
                half
            }
        };
        let moved = self.move_to(Target::Absolute(target), velocity.abs(), acceleration);
        self.started(moved)
    }

    /// The LC400 has no home; C keeps `asynMotorAxis::home`.
    fn home(
        &mut self,
        _user: &AsynUser,
        _min_velocity: f64,
        _velocity: f64,
        _acceleration: f64,
        _forward: bool,
    ) -> AsynResult<()> {
        self.started(Ok(()))
    }

    fn stop(&mut self, _user: &AsynUser, _acceleration: f64) -> AsynResult<()> {
        let (Some(ch), Some(wav)) = (base_address(self.axis), wavetable_address(self.axis)) else {
            return Err(lc400_err(format!("LC400 has no axis {}", self.axis)));
        };
        halt(&lock(&self.controller), ch, wav)
    }

    /// C keeps `asynMotorAxis::setPosition`.
    fn set_position(&mut self, _user: &AsynUser, _position: f64) -> AsynResult<()> {
        Ok(())
    }

    /// `LC400Axis::poll`: the position, then WAV_ACTIVE for done; the limit
    /// bits latch at a hard limit once done and clear while moving away.
    fn poll(&mut self, _user: &AsynUser, out: &mut MotorStatus) -> AsynResult<()> {
        let Some(address) = base_address(self.axis) else {
            return Err(lc400_err(format!("LC400 has no axis {}", self.axis)));
        };
        let mut ctrl = lock(&self.controller);
        let digital_pos = ctrl.params.map(|p| p.digital_pos);
        let mut updates = Vec::new();
        let result = (|| -> AsynResult<()> {
            let current = ctrl.read_single(address + ST_DIGITAL_POS)?;
            ctrl.axes[self.axis].digital_pos = f64::from(current);
            if let Some(reason) = digital_pos {
                updates.push((reason, ParamValue::Float64(f64::from(current))));
            }
            out.position = f64::from(current);
            let busy = ctrl.read_single(address + WAV_ACTIVE)?;
            let done = busy == 0;
            if done {
                ctrl.aux_params(address, &mut updates)?;
            }
            let state = &mut ctrl.axes[self.axis];
            if done {
                if state.pos_hard != 0 && current >= state.pos_hard {
                    state.high_limit = true;
                    out.high_limit = true;
                }
                if state.neg_hard != 0 && current <= state.neg_hard {
                    state.low_limit = true;
                    out.low_limit = true;
                }
            } else {
                if current < state.pos_hard - state.tol_hard {
                    state.high_limit = false;
                    out.high_limit = false;
                }
                if current > state.neg_hard + state.tol_hard {
                    state.low_limit = false;
                    out.low_limit = false;
                }
            }
            state.done = done;
            out.done = done;
            out.moving = !done;
            Ok(())
        })();
        drop(ctrl);
        out.problem = result.is_err();
        out.powered = true;
        for (reason, value) in updates {
            self.sets.axis(reason, value);
        }
        Ok(())
    }

    fn take_port_sets(&mut self) -> PortSets {
        std::mem::take(&mut self.sets)
    }
}

/// `LC400Controller`'s parameters, its `writeInt32` / `writeFloat64` and
/// its report.
pub struct Lc400ControllerParams {
    controller: Arc<Mutex<Lc400Controller>>,
    /// C `movingPollPeriod_` / `idlePollPeriod_`, in seconds.
    poll_periods: (f64, f64),
}

impl Lc400ControllerParams {
    pub fn new(controller: Arc<Mutex<Lc400Controller>>, moving_poll: f64, idle_poll: f64) -> Self {
        Self {
            controller,
            poll_periods: (moving_poll, idle_poll),
        }
    }
}

/// The channel registers of axis `addr`; C's `asynError` for an axis
/// with none.
fn axis_address(addr: i32) -> AsynResult<u32> {
    base_address(addr as usize).ok_or_else(|| lc400_err("no LC400 axis"))
}

/// `writeInt32` / `writeFloat64`'s code around the base, which the
/// record's writes reach as well as the port's.
struct Lc400WriteHook;

impl WriteHook for Lc400WriteHook {
    /// C refuses a write to an axis with no controller address before the
    /// base.
    fn check(&self, addr: i32, _function: usize) -> AsynResult<()> {
        axis_address(addr).map(drop)
    }

    /// C `writeFloat64` traces every write at its end, the base's included.
    fn after_write(&self, write: &FinishedWrite<'_>) -> WriteOutcome {
        let WriteValue::Float64(value) = write.value else {
            return WriteOutcome::default();
        };
        let function = write.function;
        match write.status {
            Err(e) => write.print(
                TraceMask::ERROR,
                format_args!(
                    "LC400MotorDriver:writeFloat64: error, status={} function={function}, value={value:.6}\n",
                    e.status() as i32, value = CFloat(value)
                ),
            ),
            Ok(()) => write.print(
                TraceMask::IO_DRIVER,
                format_args!("LC400MotorDriver:writeFloat64: function={function}, value={value:.6}\n", value = CFloat(value)),
            ),
        }
        WriteOutcome::default()
    }
}

impl ControllerParams for Lc400ControllerParams {
    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let params = Lc400Params {
            range: base.create_param("LC400_RANGE", ParamType::Int32)?,
            range_type: base.create_param("LC400_RANGE_TYPE", ParamType::Int32)?,
            digital_pos: base.create_param("LC400_DIGITAL_POS", ParamType::Float64)?,
            servo_state: base.create_param("LC400_SERVO_STATE", ParamType::Int32)?,
            p_gain: base.create_param("LC400_P_GAIN", ParamType::Float64)?,
            i_gain: base.create_param("LC400_I_GAIN", ParamType::Float64)?,
            d_gain: base.create_param("LC400_D_GAIN", ParamType::Float64)?,
            firmware: base.create_param("LC400_FW", ParamType::Octet)?,
            w_enable: base.create_param("LC400_W_ENABLE", ParamType::Int32)?,
            w_index: base.create_param("LC400_W_INDEX", ParamType::Int32)?,
            w_delay: base.create_param("LC400_W_DELAY", ParamType::Int32)?,
            w_end_idx: base.create_param("LC400_W_ENDIDX", ParamType::Int32)?,
            w_active: base.create_param("LC400_W_ACTIVE", ParamType::Int32)?,
            w_iter: base.create_param("LC400_W_ITER", ParamType::Int32)?,
            w_iter_cts: base.create_param("LC400_W_ITERCTS", ParamType::Int32)?,
            w_max_pts: base.create_param("LC400_W_MAXPTS", ParamType::Int32)?,
        };
        let mut ctrl = lock(&self.controller);
        base.set_string_param(params.firmware, 0, &ctrl.firmware)?;
        for (axis, state) in ctrl.axes.iter().enumerate() {
            base.set_int32_param(params.range, axis as i32, state.range)?;
        }
        ctrl.params = Some(params);
        Ok(())
    }

    fn write_hook(&self) -> Option<Arc<dyn WriteHook>> {
        Some(Arc::new(Lc400WriteHook))
    }

    /// C `LC400Controller::report` (LC400MotorDriver.cpp:181-199).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let (moving, idle) = self.poll_periods;
        let _ = writeln!(
            out,
            "LC400 motor driver {}, numAxes={}, moving poll period={moving:.6}, \
             idle poll period={idle:.6}",
            ctx.port_name(),
            ctx.num_axes(),
            moving = CFloat(moving),
            idle = CFloat(idle),
        );
        if level > 0 {
            for axis in 0..ctx.num_axes() {
                let _ = writeln!(out, " axis {axis}");
            }
        }
        ctx.report(out, level);
    }

    /// C `writeInt32`: the controller's own registers are written here and
    /// the base never runs for them, so nothing is stored or posted.
    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: i32,
    ) -> ParamWrite {
        // C dereferences the null axis there (LC400MotorDriver.cpp:551-554); refused as the base does.
        if !has_motor_axis(base, user) {
            return ParamWrite::PassToBase(Ok(()));
        }
        let address = match axis_address(user.addr) {
            Ok(address) => address,
            Err(e) => return ParamWrite::Handled(Err(e)),
        };
        let mut ctrl = lock(&self.controller);
        let Some(p) = ctrl.params else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let register = match user.reason {
            r if r == p.servo_state => SERVO_STATE,
            r if r == p.w_enable => WAV_ENABLE,
            r if r == p.w_index => WAV_INDEX,
            r if r == p.w_delay => WAV_DELAY,
            r if r == p.w_end_idx => WAV_END,
            r if r == p.w_active => WAV_ACTIVE,
            r if r == p.w_iter => WAV_ITERATIONS,
            r if r == p.w_max_pts => {
                if let Some(state) = ctrl.axes.get_mut(user.addr as usize) {
                    state.max_pts = value;
                }
                return ParamWrite::PassToBase(Ok(()));
            }
            _ => return ParamWrite::PassToBase(Ok(())),
        };
        ParamWrite::Handled(ctrl.write_single(address + register, value))
    }

    /// C `writeFloat64`, its own registers handled as in
    /// [`Self::write_int32`]; [`Lc400WriteHook`] traces them.
    fn write_float64(
        &mut self,
        base: &mut PortDriverBase,
        user: AxisUser<'_>,
        value: f64,
    ) -> ParamWrite {
        // C dereferences the null axis there (LC400MotorDriver.cpp:593-601); refused as the base does.
        if !has_motor_axis(base, user) {
            return ParamWrite::PassToBase(Ok(()));
        }
        let address = match axis_address(user.addr) {
            Ok(address) => address,
            Err(e) => return ParamWrite::Handled(Err(e)),
        };
        let ctrl = lock(&self.controller);
        let Some(p) = ctrl.params else {
            return ParamWrite::PassToBase(Ok(()));
        };
        ParamWrite::Handled(match user.reason {
            r if r == p.digital_pos => ctrl.write_single(address + ST_DIGITAL_SP, value as i32),
            r if r == p.p_gain => ctrl.write_double(address + P_GAIN, value),
            r if r == p.i_gain => ctrl.write_double(address + I_GAIN, value),
            r if r == p.d_gain => ctrl.write_double(address + D_GAIN, value),
            _ => return ParamWrite::PassToBase(Ok(())),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use epics_rs::asyn::interfaces::motor::PortSet;
    use epics_rs::asyn::manager::PortManager;
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortFlags};
    use epics_rs::asyn::port_handle::PortHandle;
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::motor::builder::MotorBuilder;
    use epics_rs::motor::controller_port::index;
    use epics_rs::motor::devices::{register_axis, register_controller_port};
    use epics_rs::motor::trace::TraceUser;

    use super::*;
    use motor_common::testing::Polled;

    #[test]
    fn the_constructor_calls_back() {
        let controller = Arc::new(Mutex::new(Lc400Controller::new(None, 1)));
        assert!(Lc400Axis::new(controller, 0).constructor_calls_param_callbacks());
    }

    /// A four-axis LC400 port with no line: axis 3 has no controller
    /// address, and the controller's own registers fail to write.
    fn port(name: &str) -> (PortHandle, Lc400Params) {
        let controller = Arc::new(Mutex::new(Lc400Controller::new(None, 4)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let port = register_controller_port(&open_owner(), name, 4, params).unwrap();
        for addr in 0..4 {
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(Lc400Axis::new(controller.clone(), addr)));
            let user = TraceUser::Asyn {
                port: name.to_owned(),
                addr: addr as i32,
            };
            register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        }
        let params = lock(&controller).params.unwrap();
        (port, params)
    }

    /// Run `body` with `port`'s trace file a fresh file, at C's error and
    /// IO_DRIVER masks; return its lines.
    fn traced(port: &str, body: impl FnOnce()) -> Vec<String> {
        let path = std::env::temp_dir().join(format!("lc400-trace-{port}-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        let manager = PortManager::global();
        let trace = manager.trace_manager();
        trace.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let file = Arc::new(Mutex::new(file));
        trace.set_trace_file(Some(port), TraceFile::File(file.clone()));
        // An axis's own slot keeps the file it had.
        for addr in 0..4 {
            trace.set_device_trace_file(port, addr, TraceFile::File(file.clone()));
        }
        body();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        text.lines().map(str::to_owned).collect()
    }

    /// LC400MotorDriver.cpp:480-486 reads every axis's motorStatusDone_,
    /// which the base clears after every move start, refused or not
    /// (asynMotorController.cpp:301-353), so another axis's move skips the
    /// aux params at once.
    #[test]
    fn another_axis_s_move_start_skips_the_aux_params() {
        let (_rig, io) =
            motor_common::testing::LineRig::new("lc400MoveStart", |_| "AAAAA\0\0\0\0Z".into());
        let controller = Arc::new(Mutex::new(Lc400Controller::new(Some(io), 2)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let _port =
            register_controller_port(&open_owner(), "lc400MoveStartPort", 2, params).unwrap();
        let p = lock(&controller).params.unwrap();
        let mut axes = [
            Lc400Axis::new(controller.clone(), 0),
            Lc400Axis::new(controller, 1),
        ];
        let user = AsynUser::new(0);
        for axis in &mut axes {
            axis.polled(&user).unwrap();
        }
        // Refused: LC400_W_MAXPTS is 0.
        assert!(axes[1].move_absolute(&user, 0.0, 0.0, 1.0, 1.0).is_err());
        axes[0].take_port_sets();
        axes[0].polled(&user).unwrap();
        let reasons: Vec<usize> = axes[0]
            .take_port_sets()
            .into_iter()
            .map(|set| match set {
                PortSet::Axis { reason, .. } => reason,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(reasons, [p.digital_pos]);
    }

    /// LC400MotorDriver.cpp:882, :913, :922-925: a failed position read
    /// skips the DONE set, so it keeps the DONE a move start cleared.
    #[test]
    fn a_failed_position_read_keeps_the_done_a_move_cleared() {
        use std::sync::atomic::{AtomicBool, Ordering};
        let fail = Arc::new(AtomicBool::new(false));
        let fails = fail.clone();
        let (_rig, io) = motor_common::testing::LineRig::failing(
            "lc400FailedPos",
            |_| "AAAAA\0\0\0\0Z".into(),
            move |_| fails.load(Ordering::SeqCst),
        );
        let controller = Arc::new(Mutex::new(Lc400Controller::new(Some(io), 1)));
        let mut axis = Lc400Axis::new(controller, 0);
        let user = AsynUser::new(0);
        let mut status = MotorStatus::default();
        axis.poll(&user, &mut status).unwrap();
        assert!(status.done && !status.problem);
        // What the poll loop does when a move starts.
        status.done = false;
        fail.store(true, Ordering::SeqCst);
        let _ = axis.poll(&user, &mut status);
        assert!(!status.done && status.problem && status.powered);
    }

    /// LC400MotorDriver.cpp poll / updateAuxParams: the digital position,
    /// then, once every axis is done, the aux params, all on the axis's own
    /// address in the order C sets them.
    #[test]
    fn a_poll_s_params_go_out_on_the_axis_in_c_s_order() {
        // Every register reads 0: position 0, WAV_ACTIVE 0 (done).
        let (_rig, io) =
            motor_common::testing::LineRig::new("lc400PollSets", |_| "AAAAA\0\0\0\0Z".into());
        let controller = Arc::new(Mutex::new(Lc400Controller::new(Some(io), 1)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let _port =
            register_controller_port(&open_owner(), "lc400PollSetsPort", 1, params).unwrap();
        let p = lock(&controller).params.unwrap();
        let mut axis = Lc400Axis::new(controller, 0);
        let user = AsynUser::new(0);
        let reasons = |axis: &mut Lc400Axis| -> Vec<usize> {
            axis.take_port_sets()
                .into_iter()
                .map(|set| match set {
                    PortSet::Axis { reason, .. } => reason,
                    other => panic!("{other:?}"),
                })
                .collect()
        };
        // The first poll finds the axis done; the aux params wait for it.
        axis.polled(&user).unwrap();
        assert_eq!(reasons(&mut axis), [p.digital_pos]);
        axis.polled(&user).unwrap();
        assert_eq!(
            reasons(&mut axis),
            [
                p.digital_pos,
                p.range,
                p.range_type,
                p.servo_state,
                p.p_gain,
                p.i_gain,
                p.d_gain
            ]
        );
    }

    /// LC400MotorDriver.cpp:500-517: a failed aux read ends updateAuxParams
    /// at `goto skip`, and the params set before it go out with the poll.
    #[test]
    fn a_failed_aux_read_keeps_the_params_read_before_it() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        // Writes until the one refused; unarmed at usize::MAX.
        let left = Arc::new(AtomicUsize::new(usize::MAX));
        let countdown = left.clone();
        let (_rig, io) = motor_common::testing::LineRig::failing(
            "lc400AuxFail",
            |_| "AAAAA\0\0\0\0Z".into(),
            move |_| countdown.fetch_sub(1, Ordering::SeqCst) == 1,
        );
        let controller = Arc::new(Mutex::new(Lc400Controller::new(Some(io), 1)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let _port = register_controller_port(&open_owner(), "lc400AuxFailPort", 1, params).unwrap();
        let p = lock(&controller).params.unwrap();
        let mut axis = Lc400Axis::new(controller, 0);
        let user = AsynUser::new(0);
        axis.polled(&user).unwrap();
        axis.take_port_sets();
        // Position, WAV_ACTIVE, range, range type, then SERVO_STATE refused.
        left.store(5, Ordering::SeqCst);
        assert!(axis.polled(&user).unwrap().problem);
        let reasons: Vec<usize> = axis
            .take_port_sets()
            .into_iter()
            .map(|set| match set {
                PortSet::Axis { reason, .. } => reason,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(reasons, [p.digital_pos, p.range, p.range_type]);
    }

    /// An LC400 register map: words written by `A2`/`A3` read back by
    /// `A0`/`A4`, and a table played (`WAV_ACTIVE` 1) starts at index 1.
    struct Lc400Sim {
        base: PortDriverBase,
        words: std::collections::HashMap<u32, i32>,
        next: u32,
        reply: std::collections::VecDeque<u8>,
    }

    impl PortDriver for Lc400Sim {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            let word = |at: usize| u32::from_le_bytes(data[at..at + 4].try_into().unwrap());
            let mut at = 0;
            while at < data.len() {
                match data[at] {
                    0xA0 => {
                        let addr = word(at + 1);
                        let value = self.words.get(&addr).copied().unwrap_or_default();
                        self.reply.extend(&data[at..at + 5]);
                        self.reply.extend(value.to_le_bytes());
                        self.reply.push_back(EOS);
                        at += 6;
                    }
                    0xA2 => {
                        let (addr, value) = (word(at + 1), word(at + 5) as i32);
                        self.words.insert(addr, value);
                        if addr == base_address(0).unwrap() + WAV_ACTIVE && value == 1 {
                            self.words.insert(addr - WAV_ACTIVE + WAV_INDEX, 1);
                        }
                        self.next = addr + 4;
                        at += 10;
                    }
                    0xA3 => {
                        self.words.insert(self.next, word(at + 1) as i32);
                        self.next += 4;
                        at += 6;
                    }
                    0xA4 => {
                        let (addr, count) = (word(at + 1), word(at + 5));
                        self.reply.extend(&data[at..at + 5]);
                        for i in 0..count {
                            let value = self.words.get(&(addr + 4 * i)).copied();
                            self.reply.extend(value.unwrap_or_default().to_le_bytes());
                        }
                        self.reply.push_back(EOS);
                        at += 10;
                    }
                    other => panic!("frame {other:#x}"),
                }
            }
            Ok(data.len())
        }
        fn io_read_octet(
            &mut self,
            _user: &AsynUser,
            buf: &mut [u8],
        ) -> epics_rs::asyn::interpose::OctetReadResult {
            let n = self.reply.len().min(buf.len());
            for (slot, byte) in buf.iter_mut().zip(self.reply.drain(..n)) {
                *slot = byte;
            }
            epics_rs::asyn::interpose::OctetReadResult::ok(
                n,
                epics_rs::asyn::interpose::EomReason::empty(),
            )
        }
    }

    /// LC400MotorDriver.cpp:779-795 move: the wavetable readbacks, on the
    /// axis's own address in the order C sets them.
    #[test]
    fn a_move_s_wavetable_readbacks_go_out_on_the_axis_in_c_s_order() {
        let name = "lc400MoveSim";
        let claim = claim_port(name).unwrap();
        let sim = Lc400Sim {
            base: PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_ALL,
            ),
            words: Default::default(),
            next: 0,
            reply: Default::default(),
        };
        let (rt, _jh) = create_port_runtime(&claim, sim, RuntimeConfig::default()).unwrap();
        let io = SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(1));
        let controller = Arc::new(Mutex::new(Lc400Controller::new(Some(io), 1)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let _port =
            register_controller_port(&open_owner(), "lc400MoveSetsPort", 1, params).unwrap();
        let p = lock(&controller).params.unwrap();
        lock(&controller).axes[0].max_pts = 8;
        let mut axis = Lc400Axis::new(controller, 0);
        axis.move_absolute(&AsynUser::new(0), 1000.0, 0.0, 1000.0, 4000.0)
            .unwrap();
        let sets: Vec<(usize, i32)> = axis
            .take_port_sets()
            .into_iter()
            .map(|set| match set {
                PortSet::Axis {
                    reason,
                    value: ParamValue::Int32(value),
                    ..
                } => (reason, value),
                other => panic!("{other:?}"),
            })
            .collect();
        let table = gen_trapezoid(0.0, 1000.0, 1000.0, 4000.0, 8);
        let expected = [
            (p.w_active, 0),
            (p.w_enable, 2),
            (p.w_index, 0),
            (p.w_delay, table.cycle_count as i32),
            (p.w_end_idx, table.points.len() as i32 - 1),
            (p.w_iter, 1),
            (p.w_iter_cts, 1),
        ];
        assert_eq!(sets, expected);
    }

    /// C dereferences the null axis of an address with no axis
    /// (LC400MotorDriver.cpp:551-554); rs refuses there as the base does,
    /// before the driver's own state changes.
    #[test]
    fn a_write_with_no_axis_is_refused_before_the_driver_s_code() {
        let controller = Arc::new(Mutex::new(Lc400Controller::new(None, 4)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let port = register_controller_port(&open_owner(), "lc400NoAxis", 4, params).unwrap();
        let p = lock(&controller).params.unwrap();
        let before = lock(&controller).axes[1].max_pts;
        let refused = port.write_int32_blocking(p.w_max_pts, 1, before + 9);
        assert_eq!(refused.unwrap_err().status(), AsynStatus::Error);
        assert_eq!(lock(&controller).axes[1].max_pts, before);
        assert!(port.write_float64_blocking(p.p_gain, 1, 0.5).is_err());
    }

    /// A user at address -1 sets axis 0's W_MAXPTS, getAddress's axis for
    /// it (asynPortDriver.cpp:1901-1914).
    #[test]
    fn a_write_at_address_minus_one_reaches_axis_0() {
        let controller = Arc::new(Mutex::new(Lc400Controller::new(None, 4)));
        let params = Lc400ControllerParams::new(controller.clone(), 0.0, 0.0);
        let name = "lc400AddrM1";
        let port = register_controller_port(&open_owner(), name, 4, params).unwrap();
        let motor: Arc<Mutex<dyn AsynMotor>> =
            Arc::new(Mutex::new(Lc400Axis::new(controller.clone(), 0)));
        let user = TraceUser::Asyn {
            port: name.to_owned(),
            addr: 0,
        };
        register_axis(MotorBuilder::new(motor).trace_user(user).build_axis().axis).unwrap();
        let p = lock(&controller).params.unwrap();
        port.write_int32_blocking(p.w_max_pts, -1, 77).unwrap();
        assert_eq!(lock(&controller).axes[0].max_pts, 77);
    }

    #[test]
    fn a_base_write_to_an_axis_with_no_address_is_refused_before_the_base() {
        let (port, _) = port("lc400NoAddr");
        port.write_float64_blocking(index::REC_RESOLUTION, 0, 0.5)
            .unwrap();
        assert_eq!(
            port.read_float64_blocking(index::REC_RESOLUTION, 0)
                .unwrap(),
            0.5
        );
        let refused = port.write_float64_blocking(index::REC_RESOLUTION, 3, 0.5);
        assert_eq!(refused.unwrap_err().status(), AsynStatus::Error);
        assert!(
            port.read_float64_blocking(index::REC_RESOLUTION, 3)
                .is_err()
        );
        assert!(
            port.write_int32_blocking(index::REC_DIRECTION, 3, 1)
                .is_err()
        );
    }

    #[test]
    fn every_float_write_is_traced_as_c_writes_it_the_base_included() {
        let (port, p) = port("lc400Trace");
        let lines = traced("lc400Trace", || {
            port.write_float64_blocking(index::REC_RESOLUTION, 0, 0.5)
                .unwrap();
            assert!(port.write_float64_blocking(p.p_gain, 0, 2.0).is_err());
            assert!(
                port.write_float64_blocking(index::REC_RESOLUTION, 3, 0.5)
                    .is_err()
            );
            // writeInt32 prints nothing.
            assert!(port.write_int32_blocking(p.servo_state, 0, 1).is_err());
        });
        let resolution = index::REC_RESOLUTION;
        let p_gain = p.p_gain;
        let expected = [
            format!("LC400MotorDriver:writeFloat64: function={resolution}, value=0.500000"),
            format!(
                "LC400MotorDriver:writeFloat64: error, status=3 function={p_gain}, value=2.000000"
            ),
            format!(
                "LC400MotorDriver:writeFloat64: error, status=3 function={resolution}, value=0.500000"
            ),
        ];
        let lines: Vec<_> = lines
            .iter()
            .filter(|l| l.contains("LC400MotorDriver"))
            .collect();
        assert_eq!(lines.len(), expected.len(), "{lines:?}");
        for (line, want) in lines.iter().zip(&expected) {
            assert!(
                line.ends_with(want.as_str()),
                "{line:?} does not end with {want:?}"
            );
        }
    }

    #[test]
    fn trapezoid_ends_at_the_target_and_ramps_in_between() {
        // 1000 counts at 1000 counts/s, 4000 counts/s^2: 1.25 s. Each point
        // holds ceil(1.25 / 24e-6 / 20000) = 3 cycles, written as 2, so
        // 1.25 / 72e-6 = 17361 points.
        let w = gen_trapezoid(0.0, 1000.0, 1000.0, 4000.0, 20000);
        assert_eq!(w.points.len(), 17361);
        assert_eq!(w.cycle_count, 2);
        // The last point lands in slot 0, at the target to within a count.
        assert!((w.points[0] - 1000).abs() <= 1, "{}", w.points[0]);
        assert!(w.points.windows(2).skip(1).all(|p| p[1] >= p[0]));
    }

    #[test]
    fn trapezoid_too_short_to_cruise_becomes_a_triangle() {
        let w = gen_trapezoid(100.0, 0.0, 1000.0, 1000.0, 1000);
        // C genTrapezoid's cycle_count and data_len for the same input.
        assert_eq!((w.cycle_count, w.points.len()), (26, 976));
        assert!(w.points[1..].iter().all(|&p| (0..=100).contains(&p)));
        assert!(w.points[1] >= w.points[w.points.len() - 1]);
    }

    #[test]
    fn double_register_is_low_word_first() {
        let bits = 1.5f64.to_bits();
        let (lo, hi) = (bits as u32, (bits >> 32) as u32);
        assert_eq!(f64::from_bits(u64::from(hi) << 32 | u64::from(lo)), 1.5);
    }
}
