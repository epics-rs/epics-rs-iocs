//! Newport HXP hexapod controller driver (TCP, port of `epics-modules/motor`
//! `motorNewport` `HXPController`/`HXPAxis`).
//!
//! The HXP speaks the same ASCII RPC protocol as the XPS (`hxp_drivers.cpp` is
//! the hexapod build of `XPS_C8_drivers.cpp`), so this driver reuses the XPS
//! wire layer ([`crate::xps::rpc`]) and its group-level RPC wrappers, adding
//! only the `Hexapod*` functions in [`commands`]. The hexapod is one
//! six-axis group named `HEXAPOD` (axes X, Y, Z, U, V, W): every status /
//! position read is group-wide, so the [`HxpController`] polls the group once
//! and each [`HxpAxis`] serves its slice of the cached result.

pub mod axis;
pub mod commands;
pub mod controller;

pub use axis::HxpAxis;
pub use controller::{HXP_GROUP, HXP_MRES, HxpController, MoveCoordSys, NUM_HXP_AXES};
