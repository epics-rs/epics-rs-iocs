//! Teledyne ISCO D/H-series syringe pump asyn port driver, ported from
//! `epics-modules/SyringePump`'s `teled_d.proto`/`teled_h.proto`
//! (StreamDevice protocol files, translated -- per task decision -- into a
//! native asyn port driver rather than run through a StreamDevice engine).
//!
//! ISCO and Vindum, the module's other two pump families, are Modbus-only
//! upstream (`drvModbusAsynConfigure` + generic `.template`s, no
//! `.proto`/StreamDevice anywhere in their db) and need no driver code of
//! their own -- they're wired directly in `iocs/syringepump-ioc` against
//! `epics-modbus-rs`. See that crate's module doc for the full scope-split
//! rationale.
//!
//! - [`checksum`] -- the `%0<nsum>` frame checksum shared by both families.
//! - [`wire_d`] / [`wire_h`] -- per-family wire format/reply-parse
//!   functions, transcribed byte-for-byte from each `.proto` file.
//! - [`connect`] -- the `TeledyneDInit`/`TeledyneHInit` octet-port lookup +
//!   EOS setup helper.
//! - [`driver`] -- the `PortDriver` implementation.

pub mod checksum;
pub mod connect;
pub mod driver;
pub mod wire_d;
pub mod wire_h;
