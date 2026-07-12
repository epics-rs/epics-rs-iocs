//! areaDetector driver for MAR marCCD detectors, ported from
//! `ADmarCCD/marCCDApp/src/marCCD.cpp` (driver version 2.3.0).
//!
//! The detector is controlled over a `marccd_server` TCP ASCII socket; images
//! are read back from the 16-bit TIFF files the server writes to a shared file
//! system. The server exposes a packed state word (`get_state`) whose nibbles
//! carry a per-task (acquire / readout / correct / write / dezinger / series)
//! queued/executing/error status; the acquisition state machine polls it to
//! sequence overlapped exposure, readout, correction and file writing.
//!
//! # Framing
//!
//! `marCCD` appends no terminator itself. The example `st.cmd` sets
//! `asynOctetSetInputEos("marServer", 0, "\n")` and
//! `asynOctetSetOutputEos("marServer", 0, "\n")`, so the asyn port appends `\n`
//! to every command and splits replies on `\n`. This port reproduces both in
//! its own `st.cmd`.
//!
//! # Deviations from the C driver
//!
//! * **marServer I/O runs on worker threads.** A Rust `PortDriver` method
//!   executes inside its port actor's current-thread runtime and cannot block
//!   on a second port, so the three C contexts (the port thread's
//!   `writeInt32` / `writeFloat64` inline work, `marCCDTask`, `getImageDataTask`)
//!   become three worker threads sharing one [`server::Server`] behind a
//!   `tokio::sync::Mutex` — the analog of C's driver lock. Command order and
//!   the marServer byte stream are unchanged; only the moment a record's write
//!   callback returns differs, and `getServerMode` / `getConfig` / `getState`
//!   run just after construction (via [`task::Cmd::Init`]) instead of
//!   synchronously inside it.
//! * **The exposure timer is emulated inline.** C arms an `epicsTimer` that
//!   signals the stop event when the internal-trigger exposure time expires;
//!   this port folds that deadline into the exposure wait loop, so no separate
//!   timer thread or `epicsTimerCancel` on abort is needed. External trigger
//!   modes have no timer in either version.
//! * **Published images are 16-bit only.** C's `readTiff` copies raw strip
//!   bytes into an `NDUInt16` buffer; [`image::decode_tiff`] accepts a 16-bit
//!   single-sample TIFF and rejects other bit depths.
//! * **Multi-strip TIFFs decode correctly.** C's `readTiff` passes strip index
//!   `0` on every loop iteration; the `tiff` crate reads every strip. marServer
//!   writes single-strip files, where the two agree.
//! * **`createFileName` is reimplemented** in [`file_name`], because
//!   `ad-core-rs` 0.22.1 does not expose it.
//! * **`getConfig` parse failures fall back to 0.** Where a `get_size` /
//!   `get_bin` reply cannot be parsed this port stores 0; C leaves the previous
//!   `sscanf` target unchanged.
//! * **Base-class `writeOctet` side effects are not reproduced.** marCCD does
//!   not override `writeOctet`; the base `asynNDArrayDriver::writeOctet` updates
//!   `NDFilePathExists` (via `checkPath`) and reloads `NDAttributesFile`. This
//!   port only stores the string, so `NDFilePathExists` is not tracked.
//! * **`ADAcquireBusy` is cleared directly by the worker** at acquisition
//!   completion (alongside `ADAcquire`), approximating
//!   `asynNDArrayDriver::setIntegerParam`'s side effect for the common
//!   `ADWaitForPlugins == 0` case; the plugin-wait interaction is not modelled.
//!
//! Retro-fixed upstream defects (were reproduced for wire parity, now fixed at
//! source per user policy; see `doc/upstream-c-defects.md`):
//! * #13 — `readTiff` returned `asynSuccess` with an unwritten buffer when its
//!   retry loop expired; now returns an error so no image is published.
//! * #14 — `getImageData` published the NDArray even when the read failed; it
//!   now propagates the error and does not publish.
//!
//! Upstream defects still reproduced deliberately and marked in the source:
//! the duplicated `MarState_RBV` record in `marCCD.template`. One latent C bug
//! is *not* reproduced: `collectSeries` returning early on a file-template
//! error (which spins the acquisition task); this port cleans up and stops
//! instead.

pub mod driver;
pub mod file_name;
pub mod image;
pub mod params;
pub mod protocol;
pub mod server;
pub mod task;
pub mod types;

pub use driver::{MarccdDriver, MarccdRuntime, create_marccd_detector};
pub use params::MarccdParams;
