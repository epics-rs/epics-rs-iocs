//! Port of the areaDetector `NDDriverStdArrays` driver (`NDDriverStdArrays.cpp`).
//!
//! Upstream: <https://github.com/areaDetector/NDDriverStdArrays>, driver
//! version 1.3.0. Unlike a detector, this driver has no acquisition thread: it
//! turns standard EPICS waveform-record writes into NDArrays, letting any
//! Channel Access client inject arrays into an areaDetector IOC.
//!
//! An `asynXXXArrayOut` write lands in one of the `write_*_array` handlers,
//! which copy the samples into the working buffer `pArrays[0]` and — depending
//! on the callback and append modes — publish the assembled array downstream.
//!
//! The pure buffer arithmetic (fill/copy with C cast semantics, the dimension
//! product, and the multi-dimensional current index) lives in [`convert`] and
//! is unit-tested directly against the C expressions.
//!
//! ## Framework-forced deviations
//!
//! * **Synchronous publish.** C's `doCallbacks` calls
//!   `doCallbacksGenericPointer` inline from the write handler. In `epics-rs`
//!   that handler runs inside the port actor's current-thread runtime where the
//!   `async` publish cannot be awaited, so the finished array is handed to a
//!   dedicated publisher task (see [`task`]). Under `blocking_callbacks=1` C
//!   back-pressures the Channel Access write thread; here the publisher task
//!   absorbs the back-pressure instead.
//! * **Per-publish snapshot.** C stamps and publishes the live `pArrays[0]`
//!   pointer, so append-mode re-publishes share one growing buffer; this port
//!   snapshots the buffer at each publish (identical observable data).
//! * **`copyBuffer` out-of-bounds fold.** For `stride > 1` combined with
//!   `nextElement > 0` the C `copyBuffer` can index past the buffer end
//!   (out-of-bounds / undefined). This port folds the absolute index back into
//!   the buffer with `% total` to stay memory-safe; the common `stride == 1`
//!   path is byte-for-byte identical to C.
//! * **`Int64`/`UInt64` output.** The C `fillBuffer`/`copyBuffer` switches have
//!   no `NDInt64`/`NDUInt64` case, so an array of those output types is left at
//!   its allocated zero; this port reproduces that no-op.
//! * **No `getAttributes`.** The `NDAttributesFile` machinery is not wired for
//!   this driver, so only the `ColorMode` attribute is attached to published
//!   arrays.

pub mod convert;
pub mod driver;
pub mod params;
pub mod task;

pub use driver::{DRIVER_VERSION, NdStdArraysDriver, NdStdArraysRuntime, create_nd_std_arrays};
pub use params::{CallbackMode, NdsaParams};
