//! paramLib.c: the per-axis parameter store a model-2 driver keeps its
//! `motorAxisParam_t` values in, which flags each change for the callback
//! `motorAxisSetCallback` installs (drvMotorAsyn's `intCallback`).

use std::sync::{Arc, Mutex, MutexGuard};

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
