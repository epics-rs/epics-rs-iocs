//! MicroEpsilon capaNCDT6200 capacitive displacement sensor, ported from
//! `epics-modules/microEpsilon`. Upstream ships two independent asyn ports
//! per physical unit:
//!
//! - **L0 (config port)** -- `capaNCDT6200Sup/Db/capaNCDT6200.proto`, a
//!   StreamDevice ASCII command/reply protocol (~30 commands: sample rate,
//!   averaging, trigger mode, per-channel status/info/linearization, data
//!   port number, analog filter). Translated here into a native asyn
//!   command table -- no StreamDevice engine in epics-rs 0.22.1. See
//!   [`wire_config`] / [`config_driver`].
//! - **L1 (data port)** -- `capaNCDT6200Sup/src/capaNCDT6200Sup.c`, a fully
//!   custom native C asyn driver with its own dedicated background reader
//!   thread ingesting a raw binary TCP measurement stream (fixed-format
//!   packets, up to 4 channels of displacement data). Ported byte-for-byte,
//!   including its packet framing/parsing, duplicate/missed-packet
//!   detection, averaging/throttle, connection-health statistics, and two
//!   preserved upstream oddities in the per-channel raw-value decode (see
//!   [`packet`]'s module doc). See [`packet`] / [`data_driver`].
//!
//! - [`wire_config`] -- L0 `.proto` command format/reply-parse functions.
//! - [`config_driver`] -- L0 `PortDriver` implementation.
//! - [`connect`] -- L0 octet-port lookup + EOS setup helper.
//! - [`packet`] -- L1 binary data-packet framing/parsing and the
//!   duplicate/missed-packet + averaging/throttle state machine (pure,
//!   unit-tested, no I/O).
//! - [`data_driver`] -- L1 `PortDriver` implementation, including the
//!   internal `_RBK` sub-port and the lazily-started background reader
//!   thread.

pub mod config_driver;
pub mod connect;
pub mod data_driver;
pub mod packet;
pub mod wire_config;
