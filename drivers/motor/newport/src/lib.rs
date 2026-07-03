//! Newport motor controller drivers, ported from `epics-modules/motor`
//! `motorNewport`.
//!
//! One crate per Newport controller family; each controller model is a module
//! implementing asyn-rs `AsynMotor`. The first supported model is the
//! single-axis [`Smc100Axis`] (serial ASCII). Additional Newport controllers
//! (ESP300, MM4000, XPS, …) are added as sibling modules here.

pub mod agap;
pub mod agilis;
pub mod conex;
pub mod ioc;
pub mod protocol;
pub mod smc100;
mod util;

pub use agap::{AgapAxis, AgapController};
pub use agilis::{AgUcAxis, AgUcController};
pub use conex::ConexAxis;
pub use ioc::NewportHolder;
pub use smc100::Smc100Axis;
