//! Port of the areaDetector `ADSimDetector` driver (`simDetector.cpp`).
//!
//! Upstream: <https://github.com/areaDetector/ADSimDetector>, driver version
//! 2.11.0. The image generation, parameter set, defaults and acquisition state
//! machine follow `simDetectorApp/src/simDetector.cpp` line for line; see
//! `image.rs` for the four pattern generators.

pub mod driver;
pub mod image;
pub mod params;
pub mod rng;
pub mod shutter;
pub mod task;
pub mod types;

pub use driver::{SimDetector, SimDetectorRuntime, create_sim_detector};
pub use params::SimParams;
pub use types::{SimMode, SineOperation};
