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
//! - [`ahxxx`] — Elettra/CaenEls AH401B and AH401D (`caenSrc/drvAHxxx.cpp`)
//! - [`nsls_em`] — NSLS Precision Integrator (`nslsSrc/drvNSLS_EM.cpp`)
//!
//! Only devices reachable over a byte stream (TCP/UDP/serial) are in scope.
//! `nslsSrc/drvNSLS2_EM` and `nslsSrc/drvNSLS2_IC` drive memory-mapped FPGA
//! registers and I²C respectively and are out of scope.

pub mod ahxxx;
pub mod ahxxx_proto;
pub mod drv_quad_em;
pub mod iocsh;
pub mod nsls_em;
pub mod nsls_em_proto;
pub mod octet;
pub mod tetramm;
pub mod tetramm_proto;

pub use ahxxx::{AhxxxRuntime, create_ahxxx};
pub use nsls_em::{NslsEmRuntime, create_nsls_em};
pub use tetramm::{TetrAmmRuntime, create_tetramm};
