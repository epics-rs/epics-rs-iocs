//! Kohzu SC-200/400/800 stepper motor controller driver.
//!
//! Ported from `epics-modules/motor` motorKohzu (`drvSC800.cc` +
//! `devSC800.cc`, the model-1 dev/drv pair). One [`KohzuController`] is shared
//! (`Arc<Mutex<_>>`) by its per-axis [`KohzuAxis`]es; the [`ioc`] module
//! provides the `SC800Setup` / `SC800Config` iocsh commands.

pub mod ioc;
pub mod kohzu;

pub use kohzu::{KohzuAxis, KohzuController};
