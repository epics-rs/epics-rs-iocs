//! Rust port of the `epics-modules/quadEM` electrometer drivers.
//!
//! [`drv_quad_em`] is the shared base class (`drvQuadEM`): the `QE_*`
//! parameter library, the sample ring buffer, the averaging/trigger semantics,
//! and the per-address NDArray callbacks. Each device module sits on top and
//! contributes only its wire protocol.
//!
//! Ported devices:
//!
//! - [`tetramm`] — CaenEls TetrAMM (`caenSrc/drvTetrAMM.cpp`)
//!
//! Only devices reachable over a byte stream (TCP/UDP/serial) are in scope.
//! `nslsSrc/drvNSLS2_EM` and `nslsSrc/drvNSLS2_IC` drive memory-mapped FPGA
//! registers and I²C respectively and are out of scope.

pub mod drv_quad_em;
pub mod octet;
pub mod tetramm;
pub mod tetramm_proto;

pub use tetramm::{TetrAmmRuntime, create_tetramm};
