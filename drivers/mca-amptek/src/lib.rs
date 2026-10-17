//! Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X MCA/MCS asyn driver, ported from
//! `mcaApp/AmptekSrc/drvAmptek.cpp` (upstream `epics-modules/mca`).
//!
//! # Feasibility
//! `drvAmptek.cpp` only ever drives the DP5 through a thin facade over
//! `CConsoleHelper` -- a handful of `SendCommand`/`ReceiveData` calls and
//! the ASCII configuration helpers -- not the ~10k-line `AmptekSrc` SDK
//! tree at large. Ported here:
//! * [`protocol`] -- binary wire framing/checksum (`SendCommand.cpp`,
//!   `ParsePacket.cpp`, `DP5Protocol.h`).
//! * [`status`] -- the DP4-format status block decode (`DP5Status.cpp`).
//! * [`ascii_cmd`] -- ASCII `KEY=value;` configuration string helpers
//!   (`AsciiCmdUtilities.cpp`).
//! * [`net_finder`] -- Silicon Labs NetFinder discovery
//!   (`NetFinder.cpp`).
//! * [`transport`] -- the Ethernet/UDP transport (`DppSocket.cpp`'s
//!   `DppInterfaceEthernet` path).
//! * [`usb`] -- the USB transport (`DppLibUsb.cpp` and `ConsoleHelper.cpp`'s
//!   `LibUsb_*`) over a [`usb::UsbBus`]. [`driver::AmptekDriver::new`] has
//!   no bus to give it and rejects interface 1;
//!   [`driver::AmptekDriver::with_usb_bus`] takes one.
//! * [`driver`] -- the asyn `PortDriver` implementation
//!   (`drvAmptek.cpp`/`drvAmptek.h`).
//!
//! Serial (`DppInterfaceSerial`) is unimplemented in the C driver itself
//! (`ConnectDpp`'s and `SendCommand`'s serial branches are empty), so a
//! Serial port is built as C builds it and never connects.

pub mod ascii_cmd;
pub mod config;
pub mod driver;
mod link;
pub mod net_finder;
pub mod protocol;
pub mod status;
pub mod transport;
pub mod usb;
