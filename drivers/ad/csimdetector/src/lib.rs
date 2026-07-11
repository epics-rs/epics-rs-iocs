//! Port of the areaDetector `ADCSimDetector` driver (`ADCSimDetector.cpp`).
//!
//! Upstream: <https://github.com/areaDetector/ADCSimDetector>, driver version
//! 2.5.0. A simulated ADC: it produces one 2-D NDArray of shape
//! `[MAX_SIGNALS, numTimePoints]` per frame plus eight 1-D per-signal arrays,
//! and derives `asynNDArrayDriver` (not `ADDriver`).
//!
//! The eight waveform generators live in [`signals`] and are pure, so they are
//! unit-tested against the C expressions without an IOC.

pub mod driver;
pub mod params;
pub mod rng;
pub mod signals;
pub mod task;
pub mod types;

pub use driver::{CSimDetector, CSimDetectorRuntime, create_c_sim_detector};
pub use params::CSimParams;
pub use types::MAX_SIGNALS;
