//! areaDetector base driver for GenICam cameras: a port of ADGenICam.
//!
//! A camera SDK plugs in through [`GenICamBackend`] (capture control, node
//! creation) and [`GenICamNode`] (access to one feature node).

pub mod driver;
pub mod feature;
pub mod unpack;

pub use driver::{ADGenICam, DriverCtx, GCParams, GenICamBackend};
pub use feature::{
    AcquisitionModes, FeatureBinding, FeatureIndex, FeatureValue, GCFeatureType, GenICamFeature,
    GenICamFeatureSet, GenICamNode, NodeResult,
};
