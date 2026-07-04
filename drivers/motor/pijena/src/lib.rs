//! piezosystem jena GmbH E-516 (PIJEDS) closed-loop piezo controller driver.
//!
//! Ported from `epics-modules/motor` motorPiJena (`drvPIJEDS.cc` +
//! `devPIJEDS.cc`, the model-1 dev/drv pair). One [`PiJedsController`] is shared
//! (`Arc<Mutex<_>>`) by its per-axis [`PiJedsAxis`]es; the [`ioc`] module
//! provides the `PIJEDSSetup` / `PIJEDSConfig` iocsh commands.

pub mod ioc;
pub mod pijeds;

pub use pijeds::{PiJedsAxis, PiJedsController};
