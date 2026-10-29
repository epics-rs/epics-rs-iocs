//! C's `dset` for each `device()` line `dbd/` declares, transcribed from the
//! C initialisers; the generated `DEVICES_*` tables name one per row.

use epics_rs::base::server::record::Dset;

/// `devDigitelPump` (`devDigitelPump.c:94-103`).
pub const DEVDIGITELPUMP: Dset = Dset::new(5, &[2, 4], false);
/// `devVacSen` (`devVacSen.c:98-107`).
pub const DEVVACSEN: Dset = Dset::new(5, &[2, 4], false);
