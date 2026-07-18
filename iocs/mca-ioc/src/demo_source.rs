//! `DemoSourceDriver` — a synthetic asyn signal source for exercising
//! [`mca::fastsweep::FastSweepDriver`] end-to-end. Not a port of any
//! upstream C file: `drvFastSweep.cpp`'s real upstream input is any
//! conforming asyn `asynInt32Array`/`asynFloat64` source (a scaler card, a
//! waveform digitizer, ...); this IOC needs a stand-in to sweep from so the
//! boot/CA smoke test has something to erase/acquire/read back.
//!
//! Publishes two params, matching what
//! [`FastSweepDriver::connect`](mca::fastsweep::FastSweepDriver::connect)'s
//! default `dataString`/`intervalString` ("DATA"/"SCAN_PERIOD") expect:
//! - `"DATA"` (`asynInt32Array`, `maxSignals` elements) — a triangular peak
//!   that drifts one channel per period, so an acquiring sweep shows a
//!   moving, recognizable shape rather than flat noise.
//! - `"SCAN_PERIOD"` (`asynFloat64`) — the fixed sample period, published
//!   once at connect and never changed; FastSweep reads it once as its
//!   `computeNumAverage` seed.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::{ParamType, ParamValue};
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

pub const DEMO_SOURCE_DATA: &str = "DATA";
pub const DEMO_SOURCE_SCAN_PERIOD: &str = "SCAN_PERIOD";

pub struct DemoSourceDriver {
    base: PortDriverBase,
}

impl PortDriver for DemoSourceDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
}

/// A triangular peak of height 50 centered on `phase % max_signals`,
/// falling off by 5 per channel of distance, floored at 1 so every channel
/// stays nonzero.
fn synthesize(max_signals: usize, phase: usize) -> Vec<i32> {
    let center = (phase % max_signals) as i32;
    (0..max_signals as i32)
        .map(|i| (50 - (i - center).abs() * 5).max(1))
        .collect()
}

fn spawn_producer(handle: PortHandle, data_reason: usize, max_signals: usize, period: Duration) {
    thread::Builder::new()
        .name("demo-source-produce".into())
        .spawn(move || {
            let mut phase: usize = 0;
            loop {
                thread::sleep(period);
                let data = synthesize(max_signals, phase);
                phase = phase.wrapping_add(1);
                let updates = vec![ParamSetValue::new(
                    data_reason,
                    0,
                    ParamValue::Int32Array(Arc::from(data)),
                )];
                if handle.set_params_and_notify_blocking(0, updates).is_err() {
                    break;
                }
            }
        })
        .expect("failed to spawn the demo source producer thread");
}

/// Registers a new asyn port `port_name` that publishes `"DATA"`
/// (`maxSignals` elements, once per `period`) and a fixed `"SCAN_PERIOD"`.
/// The caller (this IOC's `main.rs`) still owns registering the port by
/// name via `asyn_record::register_port` — this only builds and starts it.
pub fn connect(
    port_name: &str,
    max_signals: usize,
    period: Duration,
) -> AsynResult<PortRuntimeHandle> {
    let mut base = PortDriverBase::new(
        port_name,
        1,
        PortFlags {
            multi_device: false,
            can_block: false,
            destructible: true,
        },
    );
    let data_reason = base.create_param(DEMO_SOURCE_DATA, ParamType::Int32Array)?;
    let period_reason = base.create_param(DEMO_SOURCE_SCAN_PERIOD, ParamType::Float64)?;
    base.params
        .set_float64(period_reason, 0, period.as_secs_f64())?;

    let driver = DemoSourceDriver { base };
    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    spawn_producer(
        runtime_handle.port_handle().clone(),
        data_reason,
        max_signals,
        period,
    );
    Ok(runtime_handle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthesize_peaks_at_the_current_phase_and_floors_at_one() {
        let data = synthesize(10, 3);
        assert_eq!(data[3], 50);
        assert_eq!(data[0], 35);
        assert!(data.iter().all(|&v| v >= 1));
    }

    #[test]
    fn synthesize_wraps_the_peak_across_the_channel_range() {
        let data = synthesize(4, 9); // phase % max_signals == 1
        assert_eq!(data[1], 50);
    }
}
