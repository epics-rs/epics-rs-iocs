//! Shared core of `epics-modules/mca`'s `mcaApp/mcaSrc`: the asyn MCA
//! interface contract ([`interface`], ported from `mca.h`/`drvMca.h`), the
//! [`dev_mca_asyn::DevMcaAsyn`] device support binding `mca-rs`'s
//! `McaRecord` to any conforming asyn MCA driver (ported from
//! `devMcaAsyn.c`), and the first proving driver, [`fastsweep`] (ported from
//! `drvFastSweep.cpp`).
//!
//! `mcaSum.c` (ROI summing) is not ported here — it is already implemented
//! in `mca_rs::record::roi` (`sum_rois`), called by `McaRecord::process()`
//! itself.

pub mod dev_mca_asyn;
pub mod fastsweep;
pub mod interface;
