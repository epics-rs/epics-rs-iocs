//! Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X MCA/MCS asyn driver, ported from
//! `mcaApp/AmptekSrc/drvAmptek.cpp` (upstream `epics-modules/mca`).
//!
//! # Feasibility
//! `drvAmptek.cpp` only ever drives the DP5 through a thin facade over
//! `CConsoleHelper` -- a handful of `SendCommand`/`ReceiveData` calls and
//! the ASCII configuration helpers -- not the ~10k-line `AmptekSrc` SDK
//! tree at large. Ported here:
//! * [`protocol`] -- binary wire framing/checksum ([`SendCommand.cpp`],
//!   `ParsePacket.cpp`, `DP5Protocol.h`).
//! * [`status`] -- the DP4-format status block decode (`DP5Status.cpp`).
//! * [`ascii_cmd`] -- ASCII `KEY=value;` configuration string helpers
//!   (`AsciiCmdUtilities.cpp`).
//! * [`net_finder`] -- Silicon Labs NetFinder discovery
//!   (`NetFinder.cpp`).
//! * [`transport`] -- the Ethernet/UDP transport (`DppSocket.cpp`'s
//!   `DppInterfaceEthernet` path).
//! * [`driver`] -- the asyn `PortDriver` implementation
//!   (`drvAmptek.cpp`/`drvAmptek.h`).
//!
//! **Not ported** (feasibility-gated, see the crate's port report rather
//! than this doc for the full determination):
//! * USB (`DppLibUsb.cpp`, `libusb`-based) -- needs a Rust USB crate
//!   (`rusb`/`nusb`) not currently in this workspace.
//! * Serial (`DppInterfaceSerial`) -- unimplemented even in the upstream
//!   C driver itself (`ConnectDpp`'s and `SendCommand`'s serial branches
//!   are both empty no-ops), so there is nothing to port.
//! * NetFinder broadcast discovery's per-network-interface enumeration
//!   (`osiSockDiscoverBroadcastAddresses`) -- scoped down to a single
//!   global broadcast; see [`transport::AmptekUdpTransport::discover_broadcast`].

pub mod ascii_cmd;
pub mod config;
pub mod driver;
pub mod net_finder;
pub mod protocol;
pub mod status;
pub mod transport;
