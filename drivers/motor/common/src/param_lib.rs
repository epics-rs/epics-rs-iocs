//! paramLib.c: the per-axis parameter store a model-2 driver keeps its
//! `motorAxisParam_t` values in, which flags each change for the callback
//! `motorAxisSetCallback` installs (drvMotorAsyn's `intCallback`).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::interfaces::motor::{MotorStatus, StatusWrite};

use crate::drv_motor_asyn::function;

/// `MOTOR_AXIS_NUM_PARAMS` (motor_interface.h:333), the store's size.
pub const NUM_PARAMS: usize = 28;

/// The callback `paramCallCallback` runs with the changed indexes.
pub type Callback = Arc<dyn Fn(&[i32]) + Send + Sync>;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
enum Val {
    #[default]
    Undef,
    Double(f64),
    Int(i32),
}

#[derive(Default)]
struct Inner {
    vals: [Val; NUM_PARAMS],
    flags: [bool; NUM_PARAMS],
    force: bool,
    callback: Option<Callback>,
}

/// C `paramList` with `startVal` 0: one per axis, as every model-2 driver
/// creates it (`motorParam->create(0, MOTOR_AXIS_NUM_PARAMS)`).
#[derive(Default)]
pub struct Params(Mutex<Inner>);

/// `PARAM_ERROR`: an index outside the store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfRange;

fn slot(index: i32) -> Result<usize, OutOfRange> {
    usize::try_from(index)
        .ok()
        .filter(|&i| i < NUM_PARAMS)
        .ok_or(OutOfRange)
}

impl Params {
    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn set(&self, index: i32, val: Val) -> Result<(), OutOfRange> {
        let i = slot(index)?;
        let mut inner = self.lock();
        if inner.vals[i] != val {
            inner.flags[i] = true;
            inner.vals[i] = val;
        }
        Ok(())
    }

    /// C `paramSetInteger`: a change of value or of type flags it.
    pub fn set_integer(&self, index: i32, value: i32) -> Result<(), OutOfRange> {
        self.set(index, Val::Int(value))
    }

    /// C `paramSetDouble`.
    pub fn set_double(&self, index: i32, value: f64) -> Result<(), OutOfRange> {
        self.set(index, Val::Double(value))
    }

    /// C `paramGetInteger`: a double rounds half up; `None` for a parameter
    /// never set, which C reports as success leaving `*value` untouched.
    pub fn get_integer(&self, index: i32) -> Result<Option<i32>, OutOfRange> {
        Ok(match self.lock().vals[slot(index)?] {
            Val::Double(d) => Some((d + 0.5).floor() as i32),
            Val::Int(i) => Some(i),
            Val::Undef => None,
        })
    }

    /// C `paramGetDouble`.
    pub fn get_double(&self, index: i32) -> Result<Option<f64>, OutOfRange> {
        Ok(match self.lock().vals[slot(index)?] {
            Val::Double(d) => Some(d),
            Val::Int(i) => Some(f64::from(i)),
            Val::Undef => None,
        })
    }

    /// C `paramSetCallback`: every parameter set so far goes to the new
    /// callback on the next [`Self::call_callback`].
    pub fn set_callback(&self, callback: Callback) {
        let mut inner = self.lock();
        for i in 0..NUM_PARAMS {
            if inner.vals[i] != Val::Undef {
                inner.flags[i] = true;
            }
        }
        inner.callback = Some(callback);
    }

    /// C `paramCallCallback`: the changed indexes, in order, to the callback,
    /// which runs after the store is unlocked so that it can read it back.
    pub fn call_callback(&self) {
        let (callback, changed) = {
            let mut inner = self.lock();
            let changed: Vec<i32> = (0..NUM_PARAMS)
                .filter(|&i| std::mem::take(&mut inner.flags[i]))
                .map(|i| i as i32)
                .collect();
            if !(inner.force || !changed.is_empty()) || inner.callback.is_none() {
                return;
            }
            inner.force = false;
            (inner.callback.clone(), changed)
        };
        if let Some(callback) = callback {
            callback(&changed);
        }
    }

    /// C `paramForceCallback`: the next [`Self::call_callback`] runs even with
    /// nothing changed.
    pub fn force_callback(&self) {
        self.lock().force = true;
    }

    /// The `motorAxis` parameter `write` names, set to its value: a
    /// command's set that the poller also posts to the record.
    pub fn apply(&self, write: StatusWrite) {
        let bit = |f, v: bool| self.set_integer(f, i32::from(v));
        let _ = match write {
            StatusWrite::Position(v) => self.set_double(function::POSITION, v),
            StatusWrite::EncoderPosition(v) => self.set_double(function::ENCODER_POSN, v),
            StatusWrite::Done(v) => bit(function::DONE, v),
            StatusWrite::PowerOn(v) => bit(function::POWER_ON, v),
            StatusWrite::Problem(v) => bit(function::PROBLEM, v),
            StatusWrite::Direction(v) => bit(function::DIRECTION, v),
            StatusWrite::HighHardLimit(v) => bit(function::HIGH_HARD_LIMIT, v),
            StatusWrite::HomeSignal(v) => bit(function::HOME_SIGNAL, v),
            StatusWrite::Moving(v) => bit(function::MOVING, v),
            StatusWrite::LowHardLimit(v) => bit(function::LOW_HARD_LIMIT, v),
            StatusWrite::CommsError(v) => bit(function::COMM_ERROR, v),
        };
    }

    /// The record's status as the store holds it, an undefined parameter
    /// reading 0; the poller paces on not-done.
    pub fn motor_status(&self) -> MotorStatus {
        let bit = |f| self.get_integer(f).ok().flatten().unwrap_or(0) != 0;
        let double = |f| self.get_double(f).ok().flatten().unwrap_or(0.0);
        MotorStatus {
            position: double(function::POSITION),
            encoder_position: double(function::ENCODER_POSN),
            velocity: double(function::ACTUAL_VEL),
            done: bit(function::DONE),
            moving: !bit(function::DONE),
            msta_moving: bit(function::MOVING),
            high_limit: bit(function::HIGH_HARD_LIMIT),
            low_limit: bit(function::LOW_HARD_LIMIT),
            home: bit(function::HOME_SIGNAL),
            encoder_home: bit(function::HOME_ENCODER),
            powered: bit(function::POWER_ON),
            problem: bit(function::PROBLEM),
            direction: bit(function::DIRECTION),
            slip: bit(function::SLIP),
            slip_stall: bit(function::FOLLOWING_ERROR),
            comms_error: bit(function::COMM_ERROR),
            homed: bit(function::HOMED),
            gain_support: bit(function::HAS_CLOSED_LOOP),
            has_encoder: bit(function::HAS_ENCODER),
            ..MotorStatus::default()
        }
    }

    /// C `paramDump`, which a driver's report prints (paramLib.c:299-319).
    pub fn dump(&self, out: &mut dyn std::fmt::Write) {
        let vals = self.lock().vals;
        let _ = writeln!(out, "Number of parameters is: {NUM_PARAMS}");
        for (i, val) in vals.iter().enumerate() {
            let _ = match val {
                Val::Double(v) => writeln!(out, "Parameter {i} is a double, value {v:.6}"),
                Val::Int(v) => writeln!(out, "Parameter {i} is an integer, value {v}"),
                Val::Undef => writeln!(out, "Parameter {i} is undefined"),
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recorder(params: &Params) -> Arc<Mutex<Vec<Vec<i32>>>> {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let sink = calls.clone();
        params.set_callback(Arc::new(move |changed: &[i32]| {
            sink.lock().unwrap().push(changed.to_vec());
        }));
        calls
    }

    #[test]
    fn a_callback_sees_what_changed_since_the_last() {
        let params = Params::default();
        params.set_double(0, 1.5).unwrap();
        let calls = recorder(&params);
        params.set_integer(12, 1).unwrap();
        params.call_callback();
        params.set_integer(12, 1).unwrap();
        params.call_callback();
        params.set_double(12, 1.0).unwrap();
        params.call_callback();
        assert_eq!(*calls.lock().unwrap(), [vec![0, 12], vec![12]]);
        params.force_callback();
        params.call_callback();
        assert_eq!(calls.lock().unwrap().last().unwrap(), &Vec::<i32>::new());
    }

    #[test]
    fn reads_convert_and_report_undefined_and_out_of_range() {
        let params = Params::default();
        params.set_double(0, 2.5).unwrap();
        params.set_double(9, -2.5).unwrap();
        params.set_integer(12, 1).unwrap();
        assert_eq!(params.get_integer(0), Ok(Some(3)));
        assert_eq!(params.get_integer(9), Ok(Some(-2)));
        assert_eq!(params.get_double(12), Ok(Some(1.0)));
        assert_eq!(params.get_integer(13), Ok(None));
        assert_eq!(params.get_integer(28), Err(OutOfRange));
        assert_eq!(params.set_double(-1, 0.0), Err(OutOfRange));
    }

    #[test]
    fn the_callback_can_read_the_store_back() {
        let params = Arc::new(Params::default());
        let seen = Arc::new(Mutex::new(None));
        let (back, sink) = (Arc::downgrade(&params), seen.clone());
        params.set_callback(Arc::new(move |_: &[i32]| {
            *sink.lock().unwrap() = back.upgrade().unwrap().get_double(0).unwrap();
        }));
        params.set_double(0, 4.0).unwrap();
        params.call_callback();
        assert_eq!(*seen.lock().unwrap(), Some(4.0));
    }
}
