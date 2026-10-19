//! MicroEpsilon capaNCDT6200 capacitive displacement sensor, ported from
//! `epics-modules/microEpsilon`. Upstream ships two independent asyn ports
//! per physical unit:
//!
//! - **L0 (config port)** -- `capaNCDT6200Sup/Db/capaNCDT6200.proto`, a
//!   StreamDevice ASCII command/reply protocol. microepsilon-ioc runs the
//!   upstream file on stream-rs; nothing of it lives in this crate.
//! - **L1 (data port)** -- `capaNCDT6200Sup/src/capaNCDT6200Sup.c`, a fully
//!   custom native C asyn driver with its own dedicated background reader
//!   thread ingesting a raw binary TCP measurement stream (fixed-format
//!   packets, up to 4 channels of displacement data). Ported byte-for-byte,
//!   including its packet framing/parsing, duplicate/missed-packet
//!   detection, averaging/throttle, connection-health statistics, and two
//!   preserved upstream oddities in the per-channel raw-value decode (see
//!   [`packet`]'s module doc). See [`packet`] / [`data_driver`].
//!
//! - [`packet`] -- L1 binary data-packet framing/parsing and the
//!   duplicate/missed-packet + averaging/throttle state machine (pure,
//!   unit-tested, no I/O).
//! - [`data_driver`] -- L1 `PortDriver` implementation, including the
//!   internal `_RBK` sub-port and the lazily-started background reader
//!   thread.

pub mod data_driver;
pub mod packet;
