//! areaDetector driver for MAR 345 online image-plate detectors, ported from
//! `ADmar345/mar345App/src/mar345.cpp`.
//!
//! The detector is controlled over a TCP ASCII socket to the `mar345dtb`
//! program; each command is an uppercase `COMMAND …` line and progress is
//! reported back as free-form status lines. The driver waits for a specific
//! `"… Ended o.k."` substring to know a slow operation (erase / scan / mode
//! change) finished. Acquired frames are read back from the `.mar<size>` files
//! `mar345dtb` writes: a text header line `CCP4 packed image, X: …, Y: …`
//! followed by the CCP4 "pck" run-length/​delta packed pixel stream, decoded by
//! [`pck::get_pck`].
//!
//! # Framing
//!
//! `mar345` appends no terminator itself. The example `st.cmd` sets
//! `asynOctetSetInputEos("marServer", 0, "\n")` and
//! `asynOctetSetOutputEos("marServer", 0, "\n")`, so the asyn port appends `\n`
//! to every command and splits replies on `\n`. This port reproduces both in
//! its own `st.cmd`.
//!
//! # Deviations from the C driver
//!
//! * **Server I/O runs on a worker thread.** A Rust `PortDriver` method executes
//!   inside its port actor's current-thread runtime and cannot block on a second
//!   asyn port, so the two C contexts (the port thread's inline `writeInt32`
//!   bookkeeping and `mar345Task`) are split: `writeInt32` only sets the shared
//!   `mode` and signals an event, and a single [`task`] worker thread owns the
//!   [`server::Server`] and performs every socket round-trip and file read —
//!   exactly the work `mar345Task` does under the C driver lock. Command order
//!   and the marServer byte stream are unchanged.
//! * **The exposure timer is emulated inline.** C arms an `epicsTimer` that
//!   signals the stop event when the exposure time expires; this port folds that
//!   deadline into the exposure wait loop, so no separate timer thread or
//!   `epicsTimerCancel` on abort is needed.
//! * **`createFileName` is reimplemented** in [`file_name`], because
//!   `ad-core-rs` 0.22.1 does not expose it.
//! * **The published NDArray uses the header (`X`,`Y`) dimensions.** C allocates
//!   the buffer from `NDArraySizeX`/`NDArraySizeY` (set by the last mode change)
//!   and lets `get_pck` fill it using the header dimensions; the two are equal in
//!   normal operation. This port sizes the pck output to those same params and
//!   decodes into it, clamping any pixel write that would exceed the buffer
//!   rather than reproducing C's out-of-bounds write on a header/param mismatch.
//! * **The unused `pData` scratch buffer is not allocated.** C's constructor
//!   allocs a 3450×3450 `NDInt16` array into `this->pData` and never reads it;
//!   only the observable `ADMaxSizeX`/`ADMaxSizeY = 3450` writes are reproduced.
//! * **`ADAcquireBusy` is not driven.** `mar345.cpp` only ever writes
//!   `ADAcquire` (never `ADAcquireBusy`); this port matches that exactly.
//! * **Base-class `writeOctet` side effects are not reproduced.** mar345 does not
//!   override `writeOctet`; the default stores the string but does not track
//!   `NDFilePathExists` or reload `NDAttributesFile`.

pub mod driver;
pub mod file_name;
pub mod params;
pub mod pck;
pub mod protocol;
pub mod server;
pub mod task;
pub mod types;

pub use driver::{Mar345Driver, Mar345Runtime, create_mar345_detector};
pub use params::Mar345Params;
