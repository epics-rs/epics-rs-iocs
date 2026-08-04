//! `scalerRecord` device support for the USB-CTR08.
//!
//! Upstream loads `scaler.db` with `DTYP="Asyn Scaler"` and lets
//! `devScalerAsyn.c` reach the board through the port's `SCALER_*` drvInfo
//! strings. `scaler-rs` binds a [`ScalerDriver`] implementation instead, so
//! that is the single path from the record to the hardware here -- the counter
//! scan is driven straight through [`crate::scaler`] rather than round-tripping
//! through parameters no record reads.

use std::sync::{Arc, Mutex};

use epics_rs::base::error::CaResult;
use epics_rs::scaler::device_support::scaler_asyn::ScalerDriver;
use epics_rs::scaler::records::scaler::MAX_SCALER_CHANNELS;
use meascomp::device::DaqDevice;

use crate::params::MAX_COUNTERS;
use crate::poller::PollerState;
use crate::scaler;

/// The USB-CTR08's 8 counters as a `scalerRecord`.
///
/// Shares `device` and `state` with the port driver and its poller: the poller
/// is what advances `ScalerState::counts` and raises `done` while a count is
/// armed, exactly as it did before this record existed.
pub struct CtrScalerDriver {
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
}

impl CtrScalerDriver {
    pub fn new(device: Arc<Mutex<DaqDevice>>, state: Arc<Mutex<PollerState>>) -> Self {
        Self { device, state }
    }
}

impl ScalerDriver for CtrScalerDriver {
    fn reset(&mut self) -> CaResult<()> {
        let dev = self.device.lock().unwrap();
        let mut st = self.state.lock().unwrap();
        scaler::reset_scaler(&dev, &mut st.scaler);
        st.scaler.presets = [0; MAX_COUNTERS];
        Ok(())
    }

    fn read(&mut self, counts: &mut [u32; MAX_SCALER_CHANNELS]) -> CaResult<()> {
        let st = self.state.lock().unwrap();
        for (i, c) in st.scaler.counts.iter().enumerate() {
            counts[i] = *c as u32;
        }
        Ok(())
    }

    fn write_preset(&mut self, channel: usize, preset: u32) -> CaResult<u32> {
        if channel < MAX_COUNTERS {
            self.state.lock().unwrap().scaler.presets[channel] = preset as u64;
        }
        Ok(preset)
    }

    fn arm(&mut self, start: bool) -> CaResult<()> {
        let dev = self.device.lock().unwrap();
        let mut st = self.state.lock().unwrap();
        if start {
            if let Err(e) = scaler::start_scaler(&dev, &mut st.scaler) {
                log::error!("start_scaler error: {e}");
            }
        } else {
            scaler::stop_scaler(&dev, &mut st.scaler);
        }
        Ok(())
    }

    /// Read-and-clear, as C `devScalerAsyn.c::scaler_done` is: the record polls
    /// this every process cycle and must see a completed count exactly once.
    fn done(&mut self) -> bool {
        let mut st = self.state.lock().unwrap();
        let done = st.scaler.done;
        st.scaler.done = false;
        done
    }

    fn num_channels(&self) -> usize {
        MAX_COUNTERS
    }
}
