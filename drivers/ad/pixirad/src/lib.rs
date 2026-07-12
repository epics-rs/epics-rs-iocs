//! Pixirad CdTe photon-counting detector (areaDetector `ADPixirad`).
//!
//! The box takes ASCII commands on a TCP socket and broadcasts two UDP
//! streams: the image data, in packets of raw chip bit planes, and the
//! environment (temperatures, humidity, high voltage). Nothing arrives as
//! pixels — see [`decode`] for what has to happen to a frame before it is an
//! image.

pub mod connection;
pub mod decode;
pub mod driver;
pub mod params;
pub mod protocol;
pub mod task;
pub mod thresholds;
pub mod types;
pub mod udp;

pub use driver::{PixiradDetector, PixiradRuntime, create_pixirad_detector};
