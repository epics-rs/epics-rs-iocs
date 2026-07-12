//! areaDetector driver for Dectris Pilatus detectors, ported from
//! `ADPilatus/pilatusApp/src/pilatusDetector.cpp` (driver version 2.9.0).
//!
//! The detector is controlled over a camserver TCP ASCII socket; images are
//! read back from the TIFF files camserver writes to a shared file system.
//!
//! # Framing
//!
//! `pilatusDetector` appends no terminator itself. The example `st.cmd` sets
//! `asynOctetSetOutputEos("camserver", 0, "\x0A")` and
//! `asynOctetSetInputEos("camserver", 0, "\x18")`, so the asyn port appends
//! `\n` to every command and splits replies on camserver's `0x18` (CAN) byte.
//! This port reproduces both in its own `st.cmd`.
//!
//! # Deviations from the C driver
//!
//! * **CBF is not supported.** C links CBFlib and calls `readCbf` for `.cbf`
//!   file names; this port logs an error and fails the read.
//! * **Camserver I/O runs on worker threads.** A Rust `PortDriver` method
//!   executes inside its port actor's current-thread runtime and cannot block
//!   on a second port, so the work C does inline in `writeInt32` /
//!   `writeFloat64` / `writeOctet` is queued to a `PilatusCmdTask` thread. The
//!   command order and the camserver byte stream are unchanged; only the moment
//!   the record's write callback returns differs.
//! * **`createFileName` / `checkPath` are reimplemented** in [`file_name`],
//!   because `ad-core-rs` 0.22.1 does not expose them.
//! * **Multi-strip TIFFs decode correctly.** C's `readTiff` passes strip index
//!   `0` on every loop iteration; the `tiff` crate reads every strip. Camserver
//!   writes single-strip files, where the two agree.
//! * **Bad-pixel indices are bounds-checked.** C indexes `pImage->pData`
//!   unchecked.
//! * **Float-to-int conversions saturate** (`as i32`) rather than invoking C's
//!   undefined behaviour on out-of-range values.
//! * **The flat-field `NDArray` is `NDInt32`.** C allocates it as `NDUInt32`
//!   but only ever accesses it through an `epicsInt32 *`.
//! * **Driver attributes (`NDAttributesFile`) are not attached** to published
//!   arrays; C calls `getAttributes(pImage->pAttributeList)`. The
//!   `TIFFImageDescription` attribute read from the file *is* attached.
//!
//! Retro-fixed upstream defects (were reproduced for wire parity, now fixed at
//! source per user policy; see `doc/upstream-c-defects.md`):
//! * #8 — `readTiff` returned `asynSuccess` with an unwritten buffer when its
//!   retry loop expired; now returns an error so no image is published.
//! * #9 — `readBadPixelFile`'s replacement index used the image height as the
//!   row stride (`ygood * ny + xgood`); now uses the width (`ygood * nx`).
//!
//! Upstream defects still reproduced deliberately and marked in the source:
//! the `thread` reply's channel 3 overwriting channel 0, `averageFlatField`
//! being NaN when no pixel reaches `MinFlatField`, and `pilatusStatus` reusing
//! one `temp` / `humid` pair across all channels.

pub mod camserver;
pub mod driver;
pub mod file_name;
pub mod image;
pub mod params;
pub mod protocol;
pub mod task;
pub mod types;

pub use driver::{PilatusDriver, PilatusRuntime, create_pilatus_detector};
pub use params::PilatusParams;
