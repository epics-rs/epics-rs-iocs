//! `scalerRecord` device support for the USB-CTR08.
//!
//! Upstream loads `scaler.db` with `DTYP="Asyn Scaler"` and lets
//! `devScalerAsyn.c` reach the board through the port's `SCALER_*` drvInfo
//! strings. `scaler-rs` binds a [`ScalerDriver`] implementation instead, so
//! that is the single path from the record to the hardware here -- the counter
//! scan is driven straight through [`crate::scaler`] rather than round-tripping
//! through parameters no record reads.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::error::CaResult;
use epics_rs::scaler::device_support::scaler_asyn::ScalerDriver;
use epics_rs::scaler::records::scaler::MAX_SCALER_CHANNELS;
use meascomp::device::DaqDevice;

use crate::params::MAX_COUNTERS;
use crate::poller::PollerState;
use crate::scaler;
use crate::trace::{self, DRIVER};

/// The USB-CTR08's 8 counters as a `scalerRecord`.
///
/// Shares `device` and `state` with the port driver and its poller: the poller
/// is what advances `ScalerState::counts` and raises `done` while a count is
/// armed, exactly as it did before this record existed.
pub struct CtrScalerDriver {
    device: Arc<Mutex<DaqDevice>>,
    state: Arc<Mutex<PollerState>>,
    /// The USB-CTR port, whose trace a read prints through as C's
    /// `readInt32Array(scalerRead_)` does on the scaler record's asynUser.
    port: PortHandle,
}

impl CtrScalerDriver {
    pub fn new(
        device: Arc<Mutex<DaqDevice>>,
        state: Arc<Mutex<PollerState>>,
        port: PortHandle,
    ) -> Self {
        Self {
            device,
            state,
            port,
        }
    }
}

impl ScalerDriver for CtrScalerDriver {
    /// C skips a scaler reset or arm while the MCS runs (drvUSBCTR.cpp:1160,
    /// 1171): the counters belong to the scan until it ends.
    fn reset(&mut self) -> CaResult<()> {
        let dev = self.device.lock().unwrap();
        let mut st = self.state.lock().unwrap();
        if st.mcs.running {
            return Ok(());
        }
        let num_counters = st.num_counters;
        scaler::reset_scaler(&dev, &mut st.scaler, num_counters);
        st.scaler.presets = [0; MAX_COUNTERS];
        Ok(())
    }

    fn read(&mut self, counts: &mut [u32; MAX_SCALER_CHANNELS]) -> CaResult<()> {
        let st = self.state.lock().unwrap();
        for (i, c) in st.scaler.counts.iter().enumerate() {
            counts[i] = *c as u32;
        }
        let d = counts.map(|c| c as i32);
        trace::print(
            &self.port,
            Some(0),
            TraceMask::FLOW,
            format_args!(
                "{DRIVER}:readInt32Array: scalerReadCommand: read {} chans, \
                 data={} {} {} {} {} {} {} {}",
                counts.len(),
                d[0],
                d[1],
                d[2],
                d[3],
                d[4],
                d[5],
                d[6],
                d[7]
            ),
        );
        Ok(())
    }

    fn write_preset(&mut self, channel: usize, preset: u32) -> CaResult<u32> {
        let mut st = self.state.lock().unwrap();
        if channel < st.num_counters {
            st.scaler.presets[channel] = preset as u64;
        }
        Ok(preset)
    }

    fn arm(&mut self, start: bool) -> CaResult<()> {
        let dev = self.device.lock().unwrap();
        let mut st = self.state.lock().unwrap();
        if st.mcs.running {
            return Ok(());
        }
        if start {
            let num_counters = st.num_counters;
            if let Err(e) = scaler::start_scaler(&dev, &mut st.scaler, num_counters) {
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

    /// C `scalerChannels_ = numCounters_` (drvUSBCTR.cpp:418).
    fn num_channels(&self) -> usize {
        self.state.lock().unwrap().num_counters
    }
}
