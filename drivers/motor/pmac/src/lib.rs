//! Delta Tau Turbo PMAC / Geobrick motion controller support, ported from
//! [`epics-modules/tpmac`](https://github.com/epics-modules/tpmac)
//! (`pmacApp/`).
//!
//! The controller speaks a line-oriented ASCII command language over serial,
//! raw TCP, or a PMAC-specific ethernet packet protocol. Everything this crate
//! does goes through an asyn octet port, so it needs no vendor library and no
//! bus hardware.
//!
//! ```text
//! motor record ──DTYP PMAC_{ctrl}_{axis}──▶ PmacAxis  ─┐
//! motor record ──DTYP PMACCS_{cs}_{1..9}──▶ PmacCsAxis─┤
//!                                                      ├─▶ octet port ─▶ PMAC
//!                                    PmacIpInterpose ──┘   (framing, ethernet only)
//! ```
//!
//! # What was ported, and what was not
//!
//! `pmacApp/` holds five source trees. They are not alternative drivers for the
//! same thing: three of them are the *comms path* (which bus the ASCII protocol
//! travels over) and two are the *driver* on top.
//!
//! | Sub-tree | Lines | Verdict | Evidence |
//! |---|---|---|---|
//! | `pmacAsynMotorPortSrc/` | 2237 | **Ported** ([`axis`], [`controller`], [`cs_groups`]) | Model-3 asyn motor driver (`asynMotorController`/`asynMotorAxis`); pure ASCII over an asyn octet port. Maps 1:1 onto [`AsynMotor`][epics_rs::asyn::interfaces::motor::AsynMotor]. |
//! | `pmacAsynIPPortSrc/` | 774 | **Ported** ([`ethernet`]) | An asyn *interpose* over `drvAsynIPPort` adding the PMAC ethernet packet header. asyn-rs has the same concept ([`OctetInterpose`][epics_rs::asyn::interpose::OctetInterpose]), so it is ported as one — the driver above it stays transport-agnostic exactly as in C. |
//! | `pmacAsynCoordSrc/` | 1035 | **Ported** ([`coord`]) | Coordinate-system (kinematic) axes. It is a *model-2* driver (`motorAxisDrvSET_t`) in C, but it only ever talks ASCII to the same octet port — nothing model-2-specific reaches the wire, so it re-emerges as a second [`AsynMotor`][epics_rs::asyn::interfaces::motor::AsynMotor] implementation. |
//! | `pmacApp/src/` (`pmacVme.c`, `drvPmac.c`, `devPmacMbx.c`, `devPmacRam.c`, `statusRecord.c`, `tsubRecord.c`) | 7938 | **Not ported: hardware-infeasible** | The VME DPRAM path. `pmacVme.c`, `drvPmac.c` and `pmacDriver.c` include `devLib.h` and call `devRegisterAddress`/`devConnectInterrupt`/`devEnableInterruptLevel` against the `atVMEA24`/`atVMEA16` address spaces, then reach the mailbox and dual-ported RAM through raw pointers. There is no VME bus and no `devLib` in this workspace: this is not a protocol to re-implement, it is a memory-mapped card. |
//! | `pmacApp/src_eth/` (`pmacEthernet.c`, `drvPmacEth.c`, `pmacRam.c`, `devPmacMbx.c`, `devPmacRam.c`, `statusRecord.c`) | 6504 | **Not ported: feasible but superseded** | The same DPRAM/mailbox *record support* as `pmacApp/src/`, reached over the ethernet protocol (`VR_PMAC_GETMEM`/`SETMEM` in `vendcmds.h`) instead of VME. Nothing here serves hardware the model-3 driver cannot: it addresses the same controllers through a second, older path, and its `pmacEthernet.c` framing is the same packet header [`ethernet`] already implements. Porting it would mean porting two custom record types (`statusRecord`, `tsubRecord`) whose only consumer is that DPRAM support. |
//!
//! `statusRecord` / `tsubRecord` (`pmacApp/src/`) are therefore not ported
//! either: the brief's condition — "only worth porting if a feasible comm path
//! uses them" — is not met. No ported code path reads DPRAM.
//!
//! # Deviations from C
//!
//! Each is argued where it lives; in summary:
//!
//! - **Raw-step scaling is gone.** `pmacSetAxisScale` /
//!   `pmacSetCoordStepsPerUnit` configure a driver-side multiplier whose inverse
//!   the IOC must put in the record's `MRES`. The two cancel, and the motor
//!   record already does the division, so this port speaks controller counts
//!   (real axes) and PLC EGU (CS axes) with `MRES = 1`. See [`axis`].
//! - **Controller-level PVs become configuration.** The C driver's
//!   `PMAC_C_FEEDRATE_POLL` / `_LIMIT`, the deferred-move *mode* and the
//!   coordinate-system group selection are asyn parameters on address 0, which
//!   the asyn-rs motor boundary does not carry; they become arguments of
//!   `pmacCreateController` and the `pmacCsGroupSwitch` command. Their effect on
//!   the record (the PROBLEM bit) is unchanged. See [`controller`].
//! - **Deferred moves live on the controller,** not on each axis. See
//!   [`controller`].
//!
//! # Upstream C defects fixed in this port
//!
//! - A rejected command (`<BELL>ERRxxx<CR>`) was reported as success by
//!   `lowLevelWriteRead` / `motorAxisWriteRead`; every move, home, stop and
//!   set-position could fail silently. Fixed in
//!   [`octet_write_read`][controller::octet_write_read].
//! - `pmacCsGroups::switchToGroup` indexed an axis-keyed `std::map` with the
//!   loop counter, so any group whose axes are not exactly `0..n-1` was mapped
//!   wrong (and `map::operator[]` grew the map mid-iteration). Fixed in
//!   [`cs_groups`].
//! - `pmacAsynCoord.c`'s `motorAxisMove` ignored its `relative` argument: a
//!   relative move on a CS axis drove to the absolute position. Fixed in
//!   [`coord`].
//! - `drvPmacGetAxesStatus` set the record's PROBLEM bit from the CS amp-fault
//!   bit and then immediately overwrote it with the runtime-error bit, so an amp
//!   fault never reached the record. Fixed in [`coord`].
//! - `pmacAsynIPPort.c` wrote the 16-bit `wValue` header field in host byte
//!   order while byte-swapping `wLength` explicitly, so the packet header was
//!   wrong on a big-endian IOC. Fixed in [`ethernet`].
//! - `pmacController.cpp` built commands with `sprintf(buf, "%s…", buf, …)` —
//!   overlapping source and destination, undefined behaviour — in
//!   `processDeferredMoves` and `pmacCsGroups`. Structurally absent here.

pub mod axis;
pub mod controller;
pub mod coord;
pub mod cs_groups;
pub mod ethernet;
pub mod ioc;
pub mod protocol;

pub use axis::PmacAxis;
pub use controller::{DeferredMode, PmacController};
pub use coord::{PmacCoordSystem, PmacCsAxis};
pub use ioc::pmac_commands;
