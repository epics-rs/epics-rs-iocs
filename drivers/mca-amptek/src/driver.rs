//! `drvAmptek` -- the Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X asyn MCA/MCS
//! driver, ported from `drvAmptek.cpp`/`drvAmptek.h` (`mcaApp/AmptekSrc`).
//! Ethernet/UDP, USB, and C's never-connecting Serial port.
//!
//! # Architecture
//! Every dispatch (`write_int32`, `read_configuration_from_hardware`, ...)
//! runs its UDP or USB round trip inline, on whichever thread asyn-rs's
//! `can_block: true` worker calls it from -- there is no separate
//! command/response thread, matching upstream C's own single-threaded
//! `CConsoleHelper` (the vendor SDK has no internal concurrency either; the
//! 10 ms/50 ms/100 ms waits [`crate::transport`] reproduces are the same
//! blocking waits `drvAmptek.cpp` itself performs on asyn's own worker
//! thread).
//!
//! # Restructuring vs. C
//! - **USB in a build without it is rejected at construction**:
//!   [`AmptekDriver::new`] returns `Err` for `DppInterfaceUSB` when the
//!   `usb` feature is off, where C would not build at all. Serial follows C: the constructor forces
//!   `directMode=0` (`drvAmptek.cpp:140-147`) and `ConnectDpp`'s Serial
//!   branch is empty, so the port exists and every connect fails. C's
//!   unrecognized-type arm (`drvAmptek.cpp:148-152`) is followed too: the
//!   line is traced, and the port exists with no address, so every connect
//!   fails until a `hostInfo` option write gives it one.
//! - **A successful `connect` always marks the user's address connected.**
//!   C's `connect` (`drvAmptek.cpp:161-183`) calls `exceptionConnect` only
//!   when it opened the DPP link itself, so an address asked while the
//!   link is already up -- SCA devices 1-7 always, device 0 after a
//!   port-level reconnect -- never connects, and its output writes time
//!   out. A C defect, not copied.
//! - **No `pData_`/cached-spectrum-buffer state.** C keeps a `pData_`
//!   scratch buffer `mcaErase_` zeroes and `mcaNumChannels_` reallocates,
//!   but every real spectrum read (`readInt32Array`) goes straight to a
//!   fresh device round trip, never through `pData_` -- it is write-only,
//!   dead state. Not ported; see also [`crate::protocol::decode_spectrum`]'s
//!   doc for the one place that dead buffer's staleness would otherwise
//!   have been (falsely) observable.
//! - **`report()` is not overridden.** `drvAmptek::report` (`drvAmptek.cpp:
//!   1070-1090+`) is diagnostic-only (interface type, serial number, preset
//!   settings/timers for a human reading the IOC shell) and not part of the
//!   asyn MCA command interface `devMcaAsyn` drives; the asyn-rs
//!   `PortDriver::report` default (port name, timestamp, EOS, parameter
//!   dump) is used instead.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **`mcaReadStatus_`'s `HV` special-case for `dppDP5G`** echoes back the
//!   cached `amptekSetHighVoltage_` (addr 0) instead of the device's
//!   decoded `HV` reading (`drvAmptek.cpp:880-886`) -- DP5G apparently
//!   doesn't report HV in its status block, so C substitutes the setpoint.
//! - **`readInt32`'s `mcaAcquiring_` handling ignores the caller's `addr`**
//!   (`drvAmptek.cpp:926-953`, no `getAddress` call at all) -- acquisition
//!   state is a single device-wide concept, and a record bound to any
//!   nonzero addr reads/drives the same shared flag as addr 0.
//! - **`writeInt32`/`writeFloat64` cache the value before dispatch, and
//!   never combine that cache-write's status with the dispatch branch's own
//!   status** (`drvAmptek.cpp:857-858,911-913,988-991`) -- a failed
//!   `setIntegerParam`/`setDoubleParam` (which in practice only fails for an
//!   out-of-range `command`/`addr`) is silently overwritten by whatever the
//!   dispatch branch returns.
//! - **`sendConfigurationFile`'s split-send discards the first half's
//!   status** (`drvAmptek.cpp:400-435`): if the config string exceeds 512
//!   bytes it is sent in two `AmptekDriver::send_command_string` calls,
//!   and only the second's result is returned, even if the first failed.
//!
//! # Fixed (not reproduced) upstream defects
//! - **`ParsePacket.cpp:179-199` (`CParsePacket::ParseCmd`) switches on
//!   `PIN->PID2` alone, ignoring `PIN->PID1`.** `PID2` is not a globally
//!   unique namespace -- `RCVPT_CONFIG_READBACK` (`DP5Protocol.h:225`) and
//!   `PID2_ACK_UNRECOG` (`DP5Protocol.h:245`) are both `0x07`. C's
//!   `sendCommandString` (`drvAmptek.cpp:464`) calls `ParseCmd`
//!   unconditionally on whatever packet `ReceiveData()` just classified, so
//!   a well-formed configuration-readback response arriving in reply to a
//!   `XMTPT_SEND_CONFIG_PACKET_EX` send would be misread as an
//!   "Unrecognized Command" ack error, with the config text itself reported
//!   as the "offending command". `AmptekDriver::send_command_string`
//!   instead only calls [`protocol::parse_cmd`] when
//!   [`protocol::parse_packet`] classified the response as
//!   [`ReceivedPacket::Ack`] -- a classification that (unlike `ParseCmd`)
//!   keys off `(pid1, pid2)` jointly, so the collision is structurally
//!   unrepresentable rather than patched around.
//!
//! See also `ascii_cmd.rs`'s `AsciiCmdUtilities.h:17` note.

use std::net::Ipv4Addr;
use std::path::Path;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::AlarmSeverity;

use mca::interface::McaReason;

use crate::ascii_cmd;
use crate::config::{self, ConfigFields, MAX_SCAS, ScaFields};
use crate::link::DppLink;
use crate::protocol::{self, ReceivedPacket, TransmitPacketType};
use crate::status::{self, Dp5Status};
use crate::transport::AmptekUdpTransport;
use crate::usb::{self, DppLibUsb, UsbBus};

/// The address C's single-argument `getIntegerParam(index)` and friends
/// read: list 0.
const PARAM_ADDR: i32 = 0;

/// `DppInterface_t` (`DppConst.h`).
const DPP_INTERFACE_ETHERNET: i32 = 0;
const DPP_INTERFACE_USB: i32 = 1;
const DPP_INTERFACE_SERIAL: i32 = 2;

/// `drvAmptek.cpp:51`: `#define MAX_FAILED_SENDS 10`.
const MAX_FAILED_SENDS: u32 = 10;

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C's `return asynDisconnected` on `!CH_.isConnected` leaves
/// `errorMessage` empty.
fn disconnected_error() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Disconnected,
        message: String::new(),
    }
}

/// The 38 `amptekXxxString` drvInfo params (`drvAmptek.h`), in
/// `drvAmptek`'s constructor's `createParam` order (`drvAmptek.cpp:98-138`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum AmptekReason {
    InputPolarity = 0,
    Clock = 1,
    Gain = 2,
    Gate = 3,
    McaSource = 4,
    PurEnable = 5,
    RtdEnable = 6,
    FastThreshold = 7,
    SlowThreshold = 8,
    PeakingTime = 9,
    FastPeakingTime = 10,
    FlatTopTime = 11,
    ConfigFile = 12,
    SaveConfigFile = 13,
    LoadConfigFile = 14,
    SlowCounts = 15,
    FastCounts = 16,
    DetTemp = 17,
    SetDetTemp = 18,
    BoardTemp = 19,
    HighVoltage = 20,
    SetHighVoltage = 21,
    MCSLowChannel = 22,
    MCSHighChannel = 23,
    Model = 24,
    Firmware = 25,
    Build = 26,
    Fpga = 27,
    SerialNumber = 28,
    AuxOut1 = 29,
    AuxOut2 = 30,
    AuxOut34 = 31,
    Connect1 = 32,
    Connect2 = 33,
    SCAOutputWidth = 34,
    SCALowChannel = 35,
    SCAHighChannel = 36,
    SCAOutputLevel = 37,
}

impl AmptekReason {
    pub const COUNT: usize = 38;

    pub const ALL: [AmptekReason; Self::COUNT] = [
        Self::InputPolarity,
        Self::Clock,
        Self::Gain,
        Self::Gate,
        Self::McaSource,
        Self::PurEnable,
        Self::RtdEnable,
        Self::FastThreshold,
        Self::SlowThreshold,
        Self::PeakingTime,
        Self::FastPeakingTime,
        Self::FlatTopTime,
        Self::ConfigFile,
        Self::SaveConfigFile,
        Self::LoadConfigFile,
        Self::SlowCounts,
        Self::FastCounts,
        Self::DetTemp,
        Self::SetDetTemp,
        Self::BoardTemp,
        Self::HighVoltage,
        Self::SetHighVoltage,
        Self::MCSLowChannel,
        Self::MCSHighChannel,
        Self::Model,
        Self::Firmware,
        Self::Build,
        Self::Fpga,
        Self::SerialNumber,
        Self::AuxOut1,
        Self::AuxOut2,
        Self::AuxOut34,
        Self::Connect1,
        Self::Connect2,
        Self::SCAOutputWidth,
        Self::SCALowChannel,
        Self::SCAHighChannel,
        Self::SCAOutputLevel,
    ];

    /// The C `amptekXxxString` drvInfo constant for this reason (`drvAmptek.h`).
    pub const fn drv_info(self) -> &'static str {
        match self {
            Self::InputPolarity => "AMPTEK_INPUT_POLARITY",
            Self::Clock => "AMPTEK_CLOCK",
            Self::Gain => "AMPTEK_GAIN",
            Self::Gate => "AMPTEK_GATE",
            Self::McaSource => "AMPTEK_MCA_SOURCE",
            Self::PurEnable => "AMPTEK_PUR_ENABLE",
            Self::RtdEnable => "AMPTEK_RTD_ENABLE",
            Self::FastThreshold => "AMPTEK_FAST_THRESHOLD",
            Self::SlowThreshold => "AMPTEK_SLOW_THRESHOLD",
            Self::PeakingTime => "AMPTEK_PEAKING_TIME",
            Self::FastPeakingTime => "AMPTEK_FAST_PEAKING_TIME",
            Self::FlatTopTime => "AMPTEK_FLAT_TOP_TIME",
            Self::ConfigFile => "AMPTEK_CONFIG_FILE",
            Self::SaveConfigFile => "AMPTEK_SAVE_CONFIG_FILE",
            Self::LoadConfigFile => "AMPTEK_LOAD_CONFIG_FILE",
            Self::SlowCounts => "AMPTEK_SLOW_COUNTS",
            Self::FastCounts => "AMPTEK_FAST_COUNTS",
            Self::DetTemp => "AMPTEK_DET_TEMP",
            Self::SetDetTemp => "AMPTEK_SET_DET_TEMP",
            Self::BoardTemp => "AMPTEK_BOARD_TEMP",
            Self::HighVoltage => "AMPTEK_HIGH_VOLTAGE",
            Self::SetHighVoltage => "AMPTEK_SET_HIGH_VOLTAGE",
            Self::MCSLowChannel => "AMPTEK_MCS_LOW_CHANNEL",
            Self::MCSHighChannel => "AMPTEK_MCS_HIGH_CHANNEL",
            Self::Model => "AMPTEK_MODEL",
            Self::Firmware => "AMPTEK_FIRMWARE",
            Self::Build => "AMPTEK_BUILD",
            Self::Fpga => "AMPTEK_FPGA",
            Self::SerialNumber => "AMPTEK_SERIAL_NUMBER",
            Self::AuxOut1 => "AMPTEK_AUX_OUT1",
            Self::AuxOut2 => "AMPTEK_AUX_OUT2",
            Self::AuxOut34 => "AMPTEK_AUX_OUT34",
            Self::Connect1 => "AMPTEK_CONNECT1",
            Self::Connect2 => "AMPTEK_CONNECT2",
            Self::SCAOutputWidth => "AMPTEK_SCA_OUTPUT_WIDTH",
            Self::SCALowChannel => "AMPTEK_SCA_LOW_CHANNEL",
            Self::SCAHighChannel => "AMPTEK_SCA_HIGH_CHANNEL",
            Self::SCAOutputLevel => "AMPTEK_SCA_OUTPUT_LEVEL",
        }
    }

    /// The `asynParamXxx` type each `createParam` call in the constructor
    /// uses (`drvAmptek.cpp:98-138`).
    pub const fn param_type(self) -> ParamType {
        match self {
            Self::Gain
            | Self::FastThreshold
            | Self::SlowThreshold
            | Self::PeakingTime
            | Self::FlatTopTime
            | Self::SlowCounts
            | Self::FastCounts
            | Self::DetTemp
            | Self::SetDetTemp
            | Self::BoardTemp
            | Self::HighVoltage => ParamType::Float64,
            Self::ConfigFile | Self::Model | Self::Firmware | Self::Fpga => ParamType::Octet,
            _ => ParamType::Int32,
        }
    }

    pub fn create_params(base: &mut PortDriverBase) -> AsynResult<[usize; Self::COUNT]> {
        let mut reasons = [0usize; Self::COUNT];
        for reason in Self::ALL {
            reasons[reason as usize] = base.create_param(reason.drv_info(), reason.param_type())?;
        }
        Ok(reasons)
    }
}

/// `hostToIPAddr` + `ntohl`/dotted-quad formatting (`drvAmptek.cpp:290-301`,
/// Ethernet branch only -- the non-Ethernet branch, a bare passthrough of
/// `addressInfo_` with no resolution, is unreachable here since USB/Serial
/// are rejected at construction). [`std::net::ToSocketAddrs`] handles both
/// a dotted-quad string and a real hostname through the OS resolver, the
/// same two cases `hostToIPAddr` (EPICS's own `gethostbyname`/`inet_addr`
/// wrapper) handles.
fn resolve_ipv4(host: &str) -> Option<Ipv4Addr> {
    use std::net::{SocketAddr, ToSocketAddrs};
    (host, 0u16)
        .to_socket_addrs()
        .ok()?
        .find_map(|addr| match addr {
            SocketAddr::V4(v4) => Some(*v4.ip()),
            _ => None,
        })
}

/// `CConsoleHelper::ProcessCfgReadEx`'s raw-config assembly
/// (`ConsoleHelper.cpp:769-825`): each `;` in the readback data is followed
/// by `\r\n` when building the string [`AmptekDriver::save_configuration_file`]
/// later writes out (and [`config::parse_configuration`] parses -- the
/// insertion is harmless there since every `KEY=value;` field is found by
/// substring search regardless of what follows it).
fn insert_crlf_after_semicolons(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len());
    for &b in data {
        s.push(b as char);
        if b == b';' {
            s.push_str("\r\n");
        }
    }
    s
}

/// `CConsoleHelper::ReceiveData` (`ConsoleHelper.cpp:690-733`) on the
/// `DP5Proto`/`DP5Stat` state it updates; see [`AmptekDriver::receive_data`].
fn receive_packet(
    pin: &mut protocol::PacketHeader,
    last_status: &mut Dp5Status,
    spectrum: &mut Vec<i32>,
    raw: &[u8],
) -> Option<ReceivedPacket> {
    let packet = protocol::parse_packet(raw, pin);
    match &packet {
        ReceivedPacket::Status { data } => *last_status = status::process_status(data),

        ReceivedPacket::Spectrum { pid2, data } => {
            // `ProcessSpectrumEx` (`ConsoleHelper.cpp:736-754`): an even
            // PID2 carries the status block after the channels.
            let (channels, values) = protocol::decode_spectrum(*pid2, data);
            if pid2 & 1 == 0
                && let Some(block) = data.get(channels * 3..channels * 3 + 64)
            {
                *last_status = status::process_status(block);
            }
            *spectrum = values;
        }
        ReceivedPacket::ConfigReadback { .. } | ReceivedPacket::Ack { .. } => {}
        ReceivedPacket::Error { .. } => {
            // C spelling (`ConsoleHelper.cpp:725`).
            eprintln!("Error recevied");
            return None;
        }
        ReceivedPacket::Unprocessed => return None,
    }
    Some(packet)
}

/// C's `drvAmptek` (`drvAmptek.h`).
pub struct AmptekDriver {
    base: PortDriverBase,
    mca: [usize; McaReason::COUNT],
    amptek: [usize; AmptekReason::COUNT],
    /// `CH_`'s interface, selected by `drvAmptek::interfaceType_`; its
    /// connection state is `CConsoleHelper::isConnected`.
    link: DppLink,
    /// `drvAmptek::addressInfo_`: `None` is C's NULL, left by an unknown
    /// interface type until a `hostInfo` option write sets it.
    address_info: Option<String>,
    /// `drvAmptek::acquiring_`.
    acquiring: bool,
    /// `drvAmptek::failedSends_`.
    failed_sends: u32,
    /// `CConsoleHelper::NumDevices`: devices the last connect attempt found.
    num_devices: usize,
    /// `CH_.DP5Proto.PIN`'s header, persistent across packets.
    pin: protocol::PacketHeader,
    /// `CConsoleHelper::DP5Stat.m_DP5_Status`, the last successfully
    /// decoded status block. Initialized from an all-zero block (matching
    /// a zero-initialized C struct member) rather than `Option` -- every
    /// read site already tolerates the all-zero baseline the same way C's
    /// own pre-connect zeroed struct does.
    last_status: Dp5Status,
    /// `CConsoleHelper::DP5Proto.SPECTRUM`: the last spectrum
    /// [`Self::receive_data`] decoded. `readInt32Array` copies it whenever
    /// `ReceiveData` succeeds, whatever packet type arrived.
    spectrum: Vec<i32>,
    /// `CConsoleHelper::HwCfgDP5`.
    hw_cfg_dp5: String,
}

impl AmptekDriver {
    /// `drvAmptekConfigure(portName, interfaceType, addressInfo, directMode)`
    /// (`drvAmptek.cpp:63-159`). Returns `Err` for USB (`1`) only in a
    /// build without USB; an unknown type builds C's address-less port.
    pub fn new(
        port_name: &str,
        interface_type: i32,
        address_info: &str,
        direct_mode: bool,
    ) -> AsynResult<Self> {
        // `CH_` is a member, so every instance opens its socket first.
        Self::configure(
            port_name,
            interface_type,
            address_info,
            direct_mode,
            AmptekUdpTransport::bind(),
        )
    }

    fn configure(
        port_name: &str,
        interface_type: i32,
        address_info: &str,
        direct_mode: bool,
        transport: AmptekUdpTransport,
    ) -> AsynResult<Self> {
        // `drvAmptek.cpp:140-153`; C forces `directMode=0` off Ethernet.
        let link = match interface_type {
            DPP_INTERFACE_ETHERNET => DppLink::ethernet(transport, direct_mode),
            DPP_INTERFACE_USB => {
                let bus = usb::host_bus().ok_or_else(|| {
                    protocol_error("unsupported interface type=1: built without the usb feature")
                })?;
                DppLink::Usb(DppLibUsb::new(bus))
            }
            DPP_INTERFACE_SERIAL => DppLink::Serial,
            _ => {
                // The port exists, with no address.
                let link = DppLink::ethernet(transport, direct_mode);
                let mut driver = Self::with_link(port_name, address_info, link)?;
                driver.address_info = None;
                driver.trace_error(&format!(
                    "drvAmptek::drvAmptek unknown interface type={interface_type}"
                ));
                return Ok(driver);
            }
        };
        Self::with_link(port_name, address_info, link)
    }

    /// A USB port (`interfaceType` 1) whose libusb calls go to `bus`.
    /// `address_info` is empty for the first DP5, else its serial number.
    pub fn with_usb_bus(
        port_name: &str,
        address_info: &str,
        bus: Box<dyn UsbBus>,
    ) -> AsynResult<Self> {
        Self::with_link(port_name, address_info, DppLink::Usb(DppLibUsb::new(bus)))
    }

    fn with_link(port_name: &str, address_info: &str, link: DppLink) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            MAX_SCAS,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        // C's registration connect runs while asynPortDriver's constructor
        // waits in waitConnect, so the base asynPortDriver::connect answers it
        // and the port comes up connected; the DPP connect is device 0's.
        base.init_connected(true);

        let mca = McaReason::create_params(&mut base)?;
        let amptek = AmptekReason::create_params(&mut base)?;

        Ok(AmptekDriver {
            base,
            mca,
            amptek,
            link,
            address_info: Some(address_info.to_string()),
            acquiring: false,
            failed_sends: 0,
            num_devices: 0,
            pin: protocol::PacketHeader::default(),
            last_status: status::process_status(&[0u8; 64]),
            spectrum: Vec::new(),
            hw_cfg_dp5: String::new(),
        })
    }

    /// `drvAmptek::checkFailedComm` (`drvAmptek.cpp:250-258`);
    /// `function_name` is the caller's, as C passes it.
    fn check_failed_comm(&mut self, function_name: &str) {
        self.failed_sends += 1;
        if self.failed_sends > MAX_FAILED_SENDS {
            self.trace_error(&format!(
                "drvAmptek::{function_name} too many failed status queries, disconnecting"
            ));
            // `disconnect(pasynUserSelf)`: pasynUserSelf is device 0's.
            self.do_disconnect(0);
        }
    }

    /// `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "...\n")`: every one of
    /// C's error formats ends in the newline, which `msg`, also the
    /// caller's error message, does not carry.
    fn trace_error(&self, msg: &str) {
        self.base
            .trace_print(TraceMask::ERROR, format_args!("{msg}\n"));
    }

    /// `asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "...\n")`.
    fn trace_io(&self, msg: &str) {
        self.base
            .trace_print(TraceMask::IO_DRIVER, format_args!("{msg}"));
    }

    /// `drvAmptek::connect` (`drvAmptek.cpp:161-183`) for a user at `addr`,
    /// which `exceptionConnect` marks: -1 the port, otherwise that device.
    fn do_connect(&mut self, user: &AsynUser) -> AsynResult<()> {
        let addr = self.base.get_address(user.addr, user.reason)?;
        if !self.link.is_connected() {
            if addr > 0 {
                return Err(protocol_error(""));
            }
            if let Err(e) = self.connect_device(user) {
                // C defect not copied: C leaves the link open here, and a
                // USB device it keeps claimed fails its next connect.
                self.link.close();
                self.num_devices = 0;
                return Err(e);
            }
            self.failed_sends = 0;
        }
        // C defect not copied: drvAmptek::connect skips exceptionConnect when the link is already up.
        self.base.set_addr_connected(user.addr, true);
        Ok(())
    }

    /// `drvAmptek::disconnect` (`drvAmptek.cpp:185-198`) for a user at
    /// `addr`; reached also from [`Self::check_failed_comm`] and the
    /// `hostInfo` option write.
    fn do_disconnect(&mut self, addr: i32) {
        if !self.link.is_connected() {
            return;
        }
        self.link.close();
        self.num_devices = 0;
        self.base.set_addr_connected(addr, false);
        let _ = self.set_params_alarm(alarm_status::COMM_ALARM, AlarmSeverity::Invalid as u16);
    }

    /// `drvAmptek::setParamsAlarm` (`drvAmptek.cpp:237-248`): every MCA and
    /// Amptek param, at every SCA address, gets the same alarm status/
    /// severity. C calls only `setParamAlarmStatus`/`setParamAlarmSeverity`,
    /// so each param's asyn status is kept as it is.
    fn set_params_alarm(&mut self, alarm_status: u16, alarm_severity: u16) -> AsynResult<()> {
        for addr in 0..MAX_SCAS as i32 {
            for &idx in self.mca.iter().chain(self.amptek.iter()) {
                let (status, _, _) = self.base.get_param_status(idx, addr)?;
                self.base
                    .set_param_status(idx, addr, status, alarm_status, alarm_severity)?;
            }
            self.base.call_param_callbacks(addr)?;
        }
        Ok(())
    }

    /// `CConsoleHelper::SendCommand` (`ConsoleHelper.cpp:85-104`): send the
    /// fixed-shape request packet for `cmd` and wait for a response, with
    /// no bookkeeping and no parsing -- both are the caller's decision (see
    /// [`Self::send_command`] and [`Self::receive_data`]).
    fn raw_send(&mut self, cmd: TransmitPacketType) -> AsynResult<Vec<u8>> {
        let expected_size = if cmd == TransmitPacketType::SendStatus {
            72
        } else {
            24648
        };
        Ok(self
            .link
            .send(&protocol::build_command(cmd), expected_size)?)
    }

    /// `drvAmptek::sendCommand` (`drvAmptek.cpp:361-382`): success means
    /// only "got any response bytes". The bytes are C's `DP5Proto.PacketIn`,
    /// returned for the callers that go on to [`Self::receive_data`].
    fn send_command(&mut self, cmd: TransmitPacketType) -> AsynResult<Vec<u8>> {
        self.require_connected()?;
        let raw = self.raw_send(cmd)?;
        if raw.is_empty() {
            let msg = format!(
                "drvAmptek::sendCommand error calling CH_.SendCommand({})",
                cmd as i32
            );
            self.trace_error(&msg);
            self.check_failed_comm("sendCommand");
            return Err(protocol_error(msg));
        }
        self.failed_sends = 0;
        self.trace_io(&format!(
            "drvAmptek::sendCommand called CH_.SendCommand({}) OK\n",
            cmd as i32
        ));
        Ok(raw)
    }

    /// `CConsoleHelper::ReceiveData` (`ConsoleHelper.cpp:690-733`): parse a
    /// response and update the cached status/spectrum its packet carries.
    /// `None` is C's `false` (a framing error or an unrouted PID).
    fn receive_data(&mut self, raw: &[u8]) -> Option<ReceivedPacket> {
        receive_packet(
            &mut self.pin,
            &mut self.last_status,
            &mut self.spectrum,
            raw,
        )
    }

    fn require_connected(&self) -> AsynResult<()> {
        if self.link.is_connected() {
            Ok(())
        } else {
            Err(disconnected_error())
        }
    }

    /// `drvAmptek::connectDevice` (`drvAmptek.cpp:279-359`), tracing on the
    /// caller's `pasynUser`, not `pasynUserSelf`.
    fn connect_device(&mut self, user: &AsynUser) -> AsynResult<()> {
        // C: "drvAmptekConfigure() was called with invalid interface type".
        let Some(address_info) = self.address_info.clone() else {
            return Err(protocol_error(""));
        };
        // `ConnectDpp`'s result and `NumDevices`, and whether the direct
        // path took it.
        let (confirmed, num_devices, direct) = match &mut self.link {
            DppLink::Ethernet {
                udp,
                direct_mode,
                dpp_addr,
            } => {
                let ip = resolve_ipv4(&address_info).ok_or_else(|| {
                    protocol_error(format!("Cannot resolve address: {address_info}"))
                })?;
                let (confirmed, num_devices) = if *direct_mode {
                    udp.connect_direct(ip)?
                } else {
                    udp.connect_default(ip)?
                };
                if confirmed {
                    *dpp_addr = Some(ip);
                }
                (confirmed, num_devices, *direct_mode)
            }
            DppLink::Usb(usb) => {
                // C copies `addressInfo_` into a 16-byte buffer as a format
                // string first; neither the truncation nor the format is
                // copied.
                let (pin, last_status, spectrum) =
                    (&mut self.pin, &mut self.last_status, &mut self.spectrum);
                let (confirmed, num_devices) = usb.connect(&address_info, |reply, sn| {
                    receive_packet(pin, last_status, spectrum, reply).is_some()
                        && last_status.serial_number == Some(sn)
                });
                (confirmed, num_devices, false)
            }
            DppLink::Serial => (false, 0, false),
        };
        self.num_devices = num_devices;

        if direct {
            if !confirmed {
                return Err(protocol_error(format!(
                    "Network DPP device {} not found",
                    address_info
                )));
            }
            self.base.trace_print_at(
                user.addr,
                user.reason as i32,
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "drvAmptek::connectDevice DPP device {} connected\n",
                    address_info
                ),
            );
        } else {
            if !confirmed {
                return Err(protocol_error(format!(
                    "DPP device {} not found, total devices found={}",
                    address_info, self.num_devices
                )));
            }
            self.base.trace_print_at(
                user.addr,
                user.reason as i32,
                TraceMask::IO_DRIVER,
                file!(),
                line!(),
                format_args!(
                    "drvAmptek::connectDevice DPP device {} connected, total devices found={}\n",
                    address_info, self.num_devices
                ),
            );
        }
        self.after_connect()
    }

    /// `drvAmptek::connectDevice` past `ConnectDpp` (`drvAmptek.cpp:330-358`).
    fn after_connect(&mut self) -> AsynResult<()> {
        // `CH_.DP5Stat.m_DP5_Status.SerialNumber = 0;` before the status
        // round trip (`drvAmptek.cpp:330`) -- reset the whole cached status
        // to its zeroed baseline here rather than just the one field, so a
        // status round-trip failure right after a successful discovery
        // cannot leave a stale *previous* connection's status behind.
        self.last_status = status::process_status(&[0u8; 64]);
        let raw = self.raw_send(TransmitPacketType::SendStatus)?;
        if raw.is_empty() {
            return Err(protocol_error(
                "error calling SendCommand for XMTPT_SEND_STATUS",
            ));
        }
        if self.receive_data(&raw).is_none() {
            return Err(protocol_error(
                "calling ReceiveData() for XMTPT_SEND_STATUS",
            ));
        }

        // `readConfigurationFromHardware()`'s return value is not checked
        // here (`drvAmptek.cpp:341`) -- connection proceeds regardless.
        let _ = self.read_configuration_from_hardware();

        let dpp_type = self.last_status.dpp_type;
        self.base.set_string_param(
            self.amptek[AmptekReason::Model as usize],
            0,
            dpp_type.name(),
        )?;
        self.base.set_int32_param(
            self.amptek[AmptekReason::SerialNumber as usize],
            0,
            // C stores `(unsigned long)-1` for an invalid serial; the int
            // param reads it as -1 (`DP5Status.cpp:51-55`).
            self.last_status.serial_number.map_or(-1, |n| n as i32),
        )?;
        let build = if self.last_status.firmware > 0x65 {
            i32::from(self.last_status.build)
        } else {
            0
        };
        self.base
            .set_int32_param(self.amptek[AmptekReason::Build as usize], 0, build)?;
        self.base.set_string_param(
            self.amptek[AmptekReason::Firmware as usize],
            0,
            status::byte_version_to_string(self.last_status.firmware),
        )?;
        self.base.set_string_param(
            self.amptek[AmptekReason::Fpga as usize],
            0,
            status::byte_version_to_string(self.last_status.fpga),
        )?;

        self.set_params_alarm(alarm_status::NO_ALARM, AlarmSeverity::NoAlarm as u16)?;
        Ok(())
    }

    /// `drvAmptek::sendCommandString` (`drvAmptek.cpp:437-477`).
    fn send_command_string(&mut self, command_string: &str) -> AsynResult<()> {
        self.require_connected()?;
        self.trace_io(&format!(
            "drvAmptek::sendCommandString sending configuration string:\n{command_string}\n"
        ));
        let packet = protocol::build_config_packet_ex(command_string);
        let send_status: AsynResult<()> = match self.raw_config_round_trip(packet) {
            Ok(raw) if raw.is_empty() => {
                let msg = format!(
                    "drvAmptek::sendCommandString error calling \
                     CH_.SendCommand_Config(XMTPT_SEND_CONFIG_PACKET_EX, {command_string})"
                );
                self.trace_error(&msg);
                self.check_failed_comm("sendCommandString");
                // C returns before the readback (`drvAmptek.cpp:450-455`).
                return Err(protocol_error(msg));
            }
            Ok(raw) => {
                self.failed_sends = 0;
                match self.receive_data(&raw) {
                    None => {
                        let msg = format!(
                            "drvAmptek::sendCommandString error in response for \
                             CH_.SendCommand_Config(XMTPT_SEND_CONFIG_PACKET_EX, {command_string})"
                        );
                        self.trace_error(&msg);
                        Err(protocol_error(msg))
                    }
                    Some(ReceivedPacket::Ack { status, data }) => {
                        let source = protocol::pid2_text_to_string("Send config", status);
                        let error = protocol::parse_cmd(&source, status, &data);
                        if error.is_empty() {
                            Ok(())
                        } else {
                            let msg = format!("drvAmptek::sendCommandString ACK error {error}");
                            self.trace_error(&msg);
                            Err(protocol_error(msg))
                        }
                    }
                    Some(_) => Ok(()),
                }
            }
            Err(e) => Err(e),
        };

        // "Some commands could have made it to the hardware, read back
        // configuration" (`drvAmptek.cpp:472-475`) -- unconditional, and
        // only overrides `send_status` if the send+ack half was itself
        // clean.
        let readback_status = self.read_configuration_from_hardware();
        if send_status.is_ok() {
            readback_status
        } else {
            send_status
        }
    }

    fn raw_config_round_trip(&mut self, packet: Vec<u8>) -> AsynResult<Vec<u8>> {
        Ok(self.link.send(&packet, 8)?)
    }

    /// `drvAmptek::saveConfigurationFile` (`drvAmptek.cpp:384-398`).
    fn save_configuration_file(&self, file_name: &str) -> AsynResult<()> {
        let content = format!("[DP5 Configuration File]\r\n{}", self.hw_cfg_dp5);
        std::fs::write(file_name, content).map_err(|e| {
            self.trace_error(&format!(
                "drvAmptek::saveConfigurationFile Couldn't open {file_name} for writing."
            ));
            AsynError::from(e)
        })
    }

    /// `drvAmptek::sendConfigurationFile` (`drvAmptek.cpp:400-435`).
    fn send_configuration_file(&mut self, file_name: &str) -> AsynResult<()> {
        let pc5_present = self.last_status.pc5_present;
        let dpp_type = self.last_status.dpp_type;
        let is_dp5_rev_dx_gains = self.last_status.is_dp5_rev_dx_gains;
        let dpp_eco = self.last_status.dpp_eco;

        let raw_cfg = ascii_cmd::get_dp5_cfg_str(Path::new(file_name));
        let cfg = ascii_cmd::remove_cmd_by_device_type(
            &raw_cfg,
            pc5_present,
            dpp_type,
            is_dp5_rev_dx_gains,
            dpp_eco,
        );

        if cfg.len() > 512 {
            let split = ascii_cmd::get_cmd_chunk(&cfg);
            println!("\t\t\tConfiguration Split at: {split}");
            // C discards the first half's status unconditionally
            // (`drvAmptek.cpp:429-433`, a plain reassignment) -- see the
            // module doc's "Preserved upstream quirks" note.
            let _ = self.send_command_string(&cfg[..split]);
            self.send_command_string(&cfg[split..])
        } else {
            self.send_command_string(&cfg)
        }
    }

    /// C `getIntegerParam(index, &v)` as its callers see it: `paramList`
    /// zeroes `v` before the lookup (`asynPortDriver.cpp:304`), and every
    /// drvAmptek caller of these three ignores the status.
    fn int_param(&self, reason: usize) -> i32 {
        self.base.get_int32_param(reason, PARAM_ADDR).unwrap_or(0)
    }

    /// C `getDoubleParam`; see [`Self::int_param`] (`asynPortDriver.cpp:386`).
    fn double_param(&self, reason: usize) -> f64 {
        self.base
            .get_float64_param(reason, PARAM_ADDR)
            .unwrap_or(0.0)
    }

    /// C `getStringParam(index, std::string&)` into a fresh `string`, which
    /// stays empty when the lookup throws (`asynPortDriver.cpp:572-591`).
    fn string_param(&self, reason: usize) -> String {
        self.base
            .get_string_param(reason, PARAM_ADDR)
            .map(|b| String::from_utf8_lossy(b).into_owned())
            .unwrap_or_default()
    }

    /// `drvAmptek::sendSCAs` (`drvAmptek.cpp:479-508`): SCA `i` from addr
    /// `i`, where its `Amptek_SCAn.db` record writes it. C's
    /// `getIntegerParam(i, ...)` zeroes `itemp` for a param never set at
    /// addr `i` (`asynPortDriver.cpp:304`).
    fn send_scas(&mut self) -> AsynResult<()> {
        let reasons = [
            self.amptek[AmptekReason::SCALowChannel as usize],
            self.amptek[AmptekReason::SCAHighChannel as usize],
            self.amptek[AmptekReason::SCAOutputLevel as usize],
        ];
        let mut scas = [ScaFields {
            low_channel: 0,
            high_channel: 0,
            output_level: 0,
        }; MAX_SCAS];
        for (i, sca) in scas.iter_mut().enumerate() {
            let [low, high, level] =
                reasons.map(|reason| self.base.get_int32_param(reason, i as i32).unwrap_or(0));
            *sca = ScaFields {
                low_channel: low,
                high_channel: high,
                output_level: level,
            };
        }
        let command_string = config::format_scas(&scas);
        self.send_command_string(&command_string)
    }

    /// `drvAmptek::sendConfiguration` (`drvAmptek.cpp:510-658`): the 27
    /// fields gathered from both the Amptek and MCA reason tables, addr 0.
    fn send_configuration(&mut self) -> AsynResult<()> {
        let a = &self.amptek;
        let m = &self.mca;
        let fields = ConfigFields {
            clock: self.int_param(a[AmptekReason::Clock as usize]) as u8,
            input_polarity: self.int_param(a[AmptekReason::InputPolarity as usize]) as u8,
            peaking_time: self.double_param(a[AmptekReason::PeakingTime as usize]),
            fast_peaking_time: self.int_param(a[AmptekReason::FastPeakingTime as usize]) as u8,
            flat_top_time: self.double_param(a[AmptekReason::FlatTopTime as usize]),
            gain: self.double_param(a[AmptekReason::Gain as usize]),
            slow_threshold: self.double_param(a[AmptekReason::SlowThreshold as usize]),
            fast_threshold: self.double_param(a[AmptekReason::FastThreshold as usize]),
            num_channels: self.int_param(m[McaReason::NumChannels as usize]),
            gate: self.int_param(a[AmptekReason::Gate as usize]) as u8,
            preset_real_time: self.double_param(m[McaReason::PresetRealTime as usize]),
            preset_live_time: self.double_param(m[McaReason::PresetLiveTime as usize]),
            preset_counts: self.double_param(m[McaReason::PresetCounts as usize]),
            preset_low_channel: self.int_param(m[McaReason::PresetLowChannel as usize]),
            preset_high_channel: self.int_param(m[McaReason::PresetHighChannel as usize]),
            mca_source: self.int_param(a[AmptekReason::McaSource as usize]) as u8,
            pur_enable: self.int_param(a[AmptekReason::PurEnable as usize]) as u8,
            set_high_voltage: self.int_param(a[AmptekReason::SetHighVoltage as usize]),
            set_det_temp: self.double_param(a[AmptekReason::SetDetTemp as usize]),
            mcs_low_channel: self.int_param(a[AmptekReason::MCSLowChannel as usize]),
            mcs_high_channel: self.int_param(a[AmptekReason::MCSHighChannel as usize]),
            dwell_time: self.double_param(m[McaReason::DwellTime as usize]),
            aux_out1: self.int_param(a[AmptekReason::AuxOut1 as usize]) as u8,
            aux_out2: self.int_param(a[AmptekReason::AuxOut2 as usize]) as u8,
            aux_out34: self.int_param(a[AmptekReason::AuxOut34 as usize]),
            connect1: self.int_param(a[AmptekReason::Connect1 as usize]) as u8,
            connect2: self.int_param(a[AmptekReason::Connect2 as usize]) as u8,
            sca_output_width: self.int_param(a[AmptekReason::SCAOutputWidth as usize]) as u8,
        };
        let command_string = config::format_configuration(&fields);
        self.send_command_string(&command_string)
    }

    /// `drvAmptek::parseConfiguration`'s apply half (`drvAmptek.cpp:750-844`):
    /// each successfully-parsed field is written into its param; a `None`
    /// field (parse failure or device-type-gated) leaves the cached param
    /// unchanged, matching C only calling `set*Param` on success.
    fn apply_parsed_configuration(&mut self) -> AsynResult<()> {
        let dpp_type = self.last_status.dpp_type;
        self.trace_io(&format!(
            "drvAmptek::parseConfiguration configuration string from device:\n{}\n",
            self.hw_cfg_dp5
        ));
        let (parsed, errors) = config::parse_configuration(&self.hw_cfg_dp5, dpp_type);
        for msg in &errors {
            self.trace_error(msg);
        }
        let a = &self.amptek;
        let m = &self.mca;

        macro_rules! apply_int {
            ($field:expr, $idx:expr) => {
                if let Some(v) = $field {
                    self.base.set_int32_param($idx, 0, i32::from(v))?;
                }
            };
        }
        macro_rules! apply_float {
            ($field:expr, $idx:expr) => {
                if let Some(v) = $field {
                    self.base.set_float64_param($idx, 0, v)?;
                }
            };
        }

        apply_int!(parsed.clock, a[AmptekReason::Clock as usize]);
        apply_int!(
            parsed.input_polarity,
            a[AmptekReason::InputPolarity as usize]
        );
        apply_float!(parsed.peaking_time, a[AmptekReason::PeakingTime as usize]);
        apply_int!(
            parsed.fast_peaking_time,
            a[AmptekReason::FastPeakingTime as usize]
        );
        apply_float!(parsed.flat_top_time, a[AmptekReason::FlatTopTime as usize]);
        apply_float!(parsed.gain, a[AmptekReason::Gain as usize]);
        apply_float!(
            parsed.slow_threshold,
            a[AmptekReason::SlowThreshold as usize]
        );
        apply_float!(
            parsed.fast_threshold,
            a[AmptekReason::FastThreshold as usize]
        );
        if let Some(v) = parsed.num_channels {
            self.base
                .set_int32_param(m[McaReason::NumChannels as usize], 0, v)?;
        }
        apply_int!(parsed.gate, a[AmptekReason::Gate as usize]);
        if let Some(v) = parsed.preset_real_time {
            self.base
                .set_float64_param(m[McaReason::PresetRealTime as usize], 0, v)?;
        }
        if let Some(v) = parsed.preset_live_time {
            self.base
                .set_float64_param(m[McaReason::PresetLiveTime as usize], 0, v)?;
        }
        if let Some(v) = parsed.preset_counts {
            self.base
                .set_float64_param(m[McaReason::PresetCounts as usize], 0, v)?;
        }
        if let Some(v) = parsed.preset_low_channel {
            self.base
                .set_int32_param(m[McaReason::PresetLowChannel as usize], 0, v)?;
        }
        if let Some(v) = parsed.preset_high_channel {
            self.base
                .set_int32_param(m[McaReason::PresetHighChannel as usize], 0, v)?;
        }
        apply_int!(parsed.mca_source, a[AmptekReason::McaSource as usize]);
        apply_int!(parsed.pur_enable, a[AmptekReason::PurEnable as usize]);
        if let Some(v) = parsed.set_high_voltage {
            self.base
                .set_int32_param(a[AmptekReason::SetHighVoltage as usize], 0, v)?;
        }
        apply_float!(parsed.set_det_temp, a[AmptekReason::SetDetTemp as usize]);
        if let Some(v) = parsed.mcs_low_channel {
            self.base
                .set_int32_param(a[AmptekReason::MCSLowChannel as usize], 0, v)?;
        }
        if let Some(v) = parsed.mcs_high_channel {
            self.base
                .set_int32_param(a[AmptekReason::MCSHighChannel as usize], 0, v)?;
        }
        if let Some(v) = parsed.dwell_time {
            self.base
                .set_float64_param(m[McaReason::DwellTime as usize], 0, v)?;
        }
        apply_int!(parsed.aux_out1, a[AmptekReason::AuxOut1 as usize]);
        apply_int!(parsed.aux_out2, a[AmptekReason::AuxOut2 as usize]);
        apply_int!(parsed.connect1, a[AmptekReason::Connect1 as usize]);
        apply_int!(parsed.connect2, a[AmptekReason::Connect2 as usize]);

        Ok(())
    }

    /// `drvAmptek::readConfigurationFromHardware` (`drvAmptek.cpp:1028-1066`).
    fn read_configuration_from_hardware(&mut self) -> AsynResult<()> {
        self.require_connected()?;
        let dpp_type = self.last_status.dpp_type;
        let query = ascii_cmd::create_full_readback_cmd(
            self.last_status.pc5_present,
            dpp_type,
            self.last_status.is_dp5_rev_dx_gains,
            self.last_status.dpp_eco,
        );
        let packet = protocol::build_full_read_config_packet(&query);

        // "This function is normally called after sending the new
        // configuration, which can take time before the unit will respond
        // to the next command. Loop for up to 1 second waiting."
        // (`drvAmptek.cpp:1037-1045`.)
        let mut raw = Vec::new();
        let mut sent = false;
        for _ in 0..100 {
            raw = self.raw_config_round_trip(packet.clone())?;
            if !raw.is_empty() {
                sent = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        if !sent {
            let msg = "drvAmptek::readConfigurationFromHardware error calling \
                       SendCommand_Config() for XMTPT_FULL_READ_CONFIG_PACKET";
            self.trace_error(msg);
            return Err(protocol_error(msg));
        }
        self.failed_sends = 0;

        match self.receive_data(&raw) {
            Some(ReceivedPacket::ConfigReadback { data }) => {
                self.hw_cfg_dp5 = insert_crlf_after_semicolons(&data);
                self.apply_parsed_configuration()
            }
            None => {
                let msg = "drvAmptek::readConfigurationFromHardware error calling \
                           ReceiveData() for XMTPT_FULL_READ_CONFIG_PACKET";
                self.trace_error(msg);
                self.check_failed_comm("readConfigurationFromHardware");
                Err(protocol_error(msg))
            }
            // `ReceiveData()==true` but `HwCfgReady` was never set --
            // matches C falling through to `return asynError` when the
            // classified packet wasn't a config readback.
            Some(_) => Err(protocol_error(
                "XMTPT_FULL_READ_CONFIG_PACKET response was not a configuration readback",
            )),
        }
    }

    /// `drvAmptek::writeInt32`'s `mcaReadStatus_` branch (`drvAmptek.cpp:
    /// 873-894`).
    fn handle_read_status(&mut self) -> AsynResult<()> {
        let raw = self.send_command(TransmitPacketType::SendStatus)?;
        match self.receive_data(&raw) {
            Some(_) => {
                let s = self.last_status;
                self.base.set_float64_param(
                    self.amptek[AmptekReason::SlowCounts as usize],
                    0,
                    s.slow_count,
                )?;
                self.base.set_float64_param(
                    self.amptek[AmptekReason::FastCounts as usize],
                    0,
                    s.fast_count,
                )?;
                self.base.set_float64_param(
                    self.amptek[AmptekReason::DetTemp as usize],
                    0,
                    s.det_temp,
                )?;
                self.base.set_float64_param(
                    self.amptek[AmptekReason::BoardTemp as usize],
                    0,
                    s.dp5_temp,
                )?;
                if s.dpp_type != status::DppType::Dp5G {
                    self.base.set_float64_param(
                        self.amptek[AmptekReason::HighVoltage as usize],
                        0,
                        s.high_voltage,
                    )?;
                } else {
                    let setpoint =
                        self.int_param(self.amptek[AmptekReason::SetHighVoltage as usize]);
                    self.base.set_float64_param(
                        self.amptek[AmptekReason::HighVoltage as usize],
                        0,
                        f64::from(setpoint),
                    )?;
                }
                Ok(())
            }
            None => {
                self.check_failed_comm("writeInt32");
                Err(protocol_error(
                    "calling ReceiveData() for XMTPT_SEND_STATUS",
                ))
            }
        }
    }
}

impl PortDriver for AmptekDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn connect(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.do_connect(user)
    }

    fn connect_addr(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.do_connect(user)
    }

    fn disconnect(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.do_disconnect(user.addr);
        Ok(())
    }

    fn disconnect_addr(&mut self, user: &AsynUser) -> AsynResult<()> {
        self.do_disconnect(user.addr);
        Ok(())
    }

    /// `drvAmptek::readOption` (`drvAmptek.cpp:200-219`): only `"hostInfo"`
    /// is recognized.
    fn get_option(&self, key: &str) -> AsynResult<String> {
        if key.eq_ignore_ascii_case("hostInfo") {
            // glibc's `%s` of C's NULL `addressInfo_`.
            Ok(self.address_info.clone().unwrap_or_else(|| "(null)".into()))
        } else {
            Err(protocol_error(format!("Unsupported key \"{key}\"")))
        }
    }

    /// `drvAmptek::writeOption` (`drvAmptek.cpp:221-235`): only
    /// `"hostInfo"` is recognized (writing it disconnects); an empty key is
    /// a silent no-op, matching C's `epicsStrCaseCmp(key, "") != 0` guard.
    fn set_option(&mut self, user: &mut AsynUser, key: &str, value: &str) -> AsynResult<()> {
        if key.eq_ignore_ascii_case("hostInfo") {
            self.address_info = Some(value.to_string());
            self.do_disconnect(user.addr);
            Ok(())
        } else if !key.is_empty() {
            Err(protocol_error(format!("Unsupported key \"{key}\"")))
        } else {
            Ok(())
        }
    }

    /// `drvAmptek::writeInt32` (`drvAmptek.cpp:846-924`).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.require_connected()?;
        let addr = self.base.get_address(user.addr, user.reason)?;
        let reason = user.reason;
        let mut status = self.base.set_int32_param(reason, addr, value);

        if reason == self.mca[McaReason::StartAcquire as usize] {
            if !self.acquiring {
                status = self
                    .send_command(TransmitPacketType::EnableMcaMcs)
                    .map(drop);
                if status.is_ok() {
                    self.acquiring = true;
                    self.base
                        .set_int32_param(self.mca[McaReason::Acquiring as usize], 0, 1)?;
                }
            }
        } else if reason == self.mca[McaReason::StopAcquire as usize] {
            status = self
                .send_command(TransmitPacketType::DisableMcaMcs)
                .map(drop);
        } else if reason == self.mca[McaReason::Erase as usize] {
            status = self
                .send_command(TransmitPacketType::SendClearSpectrumStatus)
                .map(drop);
        } else if reason == self.mca[McaReason::ReadStatus as usize] {
            status = self.handle_read_status();
        } else if reason == self.amptek[AmptekReason::LoadConfigFile as usize] {
            let file_name = self.string_param(self.amptek[AmptekReason::ConfigFile as usize]);
            status = self.send_configuration_file(&file_name);
        } else if reason == self.amptek[AmptekReason::SaveConfigFile as usize] {
            let file_name = self.string_param(self.amptek[AmptekReason::ConfigFile as usize]);
            status = self.save_configuration_file(&file_name);
        } else if reason == self.amptek[AmptekReason::SCALowChannel as usize]
            || reason == self.amptek[AmptekReason::SCAHighChannel as usize]
            || reason == self.amptek[AmptekReason::SCAOutputLevel as usize]
        {
            status = self.send_scas();
        } else {
            // "All other commands are parameters so we send the
            // configuration" (`drvAmptek.cpp:910-913`).
            status = self.send_configuration();
        }

        self.base.call_param_callbacks(addr)?;
        status
    }

    /// `drvAmptek::readInt32` (`drvAmptek.cpp:926-953`): `mcaAcquiring_`
    /// ignores `user.addr` entirely -- see the module doc's "Preserved
    /// upstream quirks" note.
    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        if user.reason == self.mca[McaReason::Acquiring as usize] {
            let s = self.last_status;
            let preset_done = s.precnt_reached
                || s.preset_rt_done
                || s.preset_lt_done
                || s.mcs_done
                || !s.mca_enabled;
            if preset_done {
                if self.acquiring {
                    self.send_command(TransmitPacketType::DisableMcaMcs)?;
                    self.acquiring = false;
                }
            } else {
                self.acquiring = true;
            }
            Ok(i32::from(self.acquiring))
        } else {
            let addr = self.base.get_address(user.addr, user.reason)?;
            self.base.get_int32_param(user.reason, addr)
        }
    }

    /// `drvAmptek::readFloat64` (`drvAmptek.cpp:955-974`).
    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let s = self.last_status;
        if user.reason == self.mca[McaReason::ElapsedLiveTime as usize] {
            Ok(s.accumulation_time)
        } else if user.reason == self.mca[McaReason::ElapsedRealTime as usize] {
            Ok(s.real_time)
        } else if user.reason == self.mca[McaReason::ElapsedCounts as usize] {
            Ok(s.slow_count)
        } else {
            let addr = self.base.get_address(user.addr, user.reason)?;
            self.base.get_float64_param(user.reason, addr)
        }
    }

    /// `drvAmptek::writeFloat64` (`drvAmptek.cpp:977-995`): every write
    /// unconditionally sends the full configuration, regardless of which
    /// param changed.
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.require_connected()?;
        let addr = self.base.get_address(user.addr, user.reason)?;
        self.base.set_float64_param(user.reason, addr, value)?;
        let status = self.send_configuration();
        self.base.call_param_callbacks(addr)?;
        status
    }

    /// `asynPortDriver::readOctet`.
    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let addr = self.base.get_address(user.addr, user.reason)?;
        let bytes = self.base.get_string_param(user.reason, addr)?;
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }

    /// `asynPortDriver::writeOctet`.
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let addr = self.base.get_address(user.addr, user.reason)?;
        self.base.set_string_param(user.reason, addr, data)?;
        self.base.call_param_callbacks(addr)?;
        user.print(
            TraceMask::IO_DRIVER,
            file!(),
            line!(),
            format_args!(
                "asynPortDriver:writeOctet: function={}, name={}, value={}\n",
                user.reason,
                self.base.params.param_name(user.reason).unwrap_or(""),
                String::from_utf8_lossy(data)
            ),
        );
        Ok(data.len())
    }

    /// `drvAmptek::readInt32Array` (`drvAmptek.cpp:997-1026`).
    fn read_int32_array(&mut self, _user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let raw = self.send_command(TransmitPacketType::SendSpectrumStatus)?;
        if self.receive_data(&raw).is_none() {
            let msg = "drvAmptek::readInt32Array error calling ReceiveData() for XMTPT_SEND_SPECTRUM_STATUS";
            self.trace_error(msg);
            self.check_failed_comm("readInt32Array");
            return Err(protocol_error(msg));
        }
        let n = self.spectrum.len().min(buf.len());
        buf[..n].copy_from_slice(&self.spectrum[..n]);
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::net::UdpSocket;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};

    /// A loopback DP5: answers each request with `reply(request)` and logs
    /// every request it received.
    struct FakeDp5 {
        stop: Arc<AtomicBool>,
        requests: Arc<Mutex<Vec<Vec<u8>>>>,
        thread: Option<std::thread::JoinHandle<()>>,
    }

    impl FakeDp5 {
        fn spawn(mut reply: impl FnMut(&[u8]) -> Option<Vec<u8>> + Send + 'static) -> (Self, u16) {
            let socket = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
            socket
                .set_read_timeout(Some(Duration::from_millis(5)))
                .unwrap();
            let port = socket.local_addr().unwrap().port();
            let stop = Arc::new(AtomicBool::new(false));
            let requests = Arc::new(Mutex::new(Vec::new()));
            let (stop2, requests2) = (stop.clone(), requests.clone());
            let thread = std::thread::spawn(move || {
                let mut buf = [0u8; 2048];
                while !stop2.load(Ordering::Relaxed) {
                    let Ok((n, from)) = socket.recv_from(&mut buf) else {
                        continue;
                    };
                    requests2.lock().unwrap().push(buf[..n].to_vec());
                    if let Some(r) = reply(&buf[..n]) {
                        for chunk in r.chunks(1024) {
                            socket.send_to(chunk, from).unwrap();
                        }
                    }
                }
            });
            (
                FakeDp5 {
                    stop,
                    requests,
                    thread: Some(thread),
                },
                port,
            )
        }
    }

    impl FakeDp5 {
        /// `(PID1, PID2)` of every request received so far.
        fn pids(&self) -> Vec<(u8, u8)> {
            let requests = self.requests.lock().unwrap();
            requests.iter().map(|r| (r[2], r[3])).collect()
        }
    }

    impl Drop for FakeDp5 {
        fn drop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(t) = self.thread.take() {
                let _ = t.join();
            }
        }
    }

    fn netfinder_reply() -> Vec<u8> {
        let mut data = vec![0u8; 24];
        data[20..24].copy_from_slice(&[127, 0, 0, 1]);
        let mut raw = vec![protocol::SYNC1, protocol::SYNC2, 0x82, 0x08, 0, 24];
        raw.extend_from_slice(&data);
        raw.extend_from_slice(&[0, 0]);
        raw
    }

    /// The default reply for each request `drvAmptek` sends; `status` is
    /// the 64-byte status block.
    fn default_reply(req: &[u8], status: &[u8; 64]) -> Option<Vec<u8>> {
        match (req[2], req[3]) {
            (0x03, 0x07) => Some(netfinder_reply()),
            (0x01, 0x01) => Some(protocol::pack_out(0x80, 0x01, status)),
            (0x20, 0x03) => Some(protocol::pack_out(0x82, 0x07, b"CLCK=AUTO;MCAC=1024;")),
            (0x20, 0x02) | (0xF0, _) | (0x02, 0x04) => Some(protocol::pack_out(0xFF, 0, &[])),
            _ => None,
        }
    }

    /// A direct-mode driver wired to a [`FakeDp5`] on loopback.
    fn driver_with_fake(
        reply: impl FnMut(&[u8]) -> Option<Vec<u8>> + Send + 'static,
    ) -> (AmptekDriver, FakeDp5) {
        let (fake, port) = FakeDp5::spawn(reply);
        let transport = AmptekUdpTransport::bind_on_command_port(port).unwrap();
        let link = DppLink::ethernet(transport, true);
        let driver = AmptekDriver::with_link("fake", "127.0.0.1", link).unwrap();
        (driver, fake)
    }

    fn err_message(err: &AsynError) -> &str {
        match err {
            AsynError::Status { message, .. } => message,
            other => panic!("not a status error: {other:?}"),
        }
    }

    fn connect(driver: &mut AmptekDriver) -> AsynResult<()> {
        PortDriver::connect(driver, &AsynUser::new(0))
    }

    /// `SerialNumber = -1` when `RAW[29] >= 128` (`DP5Status.cpp:51-55`).
    #[test]
    fn invalid_serial_number_reads_minus_one() {
        let mut status = [0u8; 64];
        status[29] = 0x80;
        let (mut driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        connect(&mut driver).unwrap();
        let idx = driver.amptek[AmptekReason::SerialNumber as usize];
        assert_eq!(driver.base.get_int32_param(idx, 0).unwrap(), -1);
    }

    /// A 256-channel spectrum packet; an even `pid2` appends `status`.
    fn spectrum_reply(pid2: u8, status: &[u8; 64]) -> Vec<u8> {
        let mut data: Vec<u8> = (0..256u32).flat_map(|i| [i as u8, 0, 0]).collect();
        if pid2 & 1 == 0 {
            data.extend_from_slice(status);
        }
        protocol::pack_out(0x81, pid2, &data)
    }

    /// `readInt32Array` sends `XMTPT_SEND_SPECTRUM_STATUS` once
    /// (`drvAmptek.cpp:1003-1008`), and a spectrum+status packet refreshes
    /// the cached status (`ConsoleHelper.cpp:747-753`).
    #[test]
    fn read_int32_array_sends_once_and_refreshes_status() {
        let status = [0u8; 64];
        let mut fresh = [0u8; 64];
        fresh[4] = 77; // SlowCount
        let (mut driver, fake) = driver_with_fake(move |req| match (req[2], req[3]) {
            (0x02, 0x03) => Some(spectrum_reply(0x02, &fresh)),
            _ => default_reply(req, &status),
        });
        connect(&mut driver).unwrap();
        let before = fake.pids().len();

        let mut buf = vec![0i32; 300];
        let n = driver
            .read_int32_array(&AsynUser::new(0), &mut buf)
            .unwrap();
        assert_eq!(n, 256);
        assert_eq!(buf[5], 5);
        assert_eq!(fake.pids()[before..], [(0x02, 0x03)]);

        let counts = AsynUser::new(driver.mca[McaReason::ElapsedCounts as usize]);
        assert_eq!(driver.read_float64(&counts).unwrap(), 77.0);
    }

    /// An odd PID2 (spectrum only) leaves the cached status alone.
    #[test]
    fn spectrum_without_status_keeps_cached_status() {
        let mut status = [0u8; 64];
        status[4] = 9;
        let (mut driver, _fake) = driver_with_fake(move |req| match (req[2], req[3]) {
            (0x02, 0x03) => Some(spectrum_reply(0x01, &[0u8; 64])),
            _ => default_reply(req, &status),
        });
        connect(&mut driver).unwrap();
        let mut buf = vec![0i32; 256];
        driver
            .read_int32_array(&AsynUser::new(0), &mut buf)
            .unwrap();
        let counts = AsynUser::new(driver.mca[McaReason::ElapsedCounts as usize]);
        assert_eq!(driver.read_float64(&counts).unwrap(), 9.0);
    }

    /// An empty-payload error ack fails the write with C's text
    /// (`drvAmptek.cpp:464-469`, `ParsePacket.cpp:194-195`).
    #[test]
    fn config_ack_error_without_payload_is_i2c_packet() {
        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| match (req[2], req[3]) {
            (0x20, 0x02) => Some(protocol::pack_out(0xFF, 0x05, &[])),
            _ => default_reply(req, &status),
        });
        connect(&mut driver).unwrap();
        let err = driver.send_command_string("GAIN=1;").unwrap_err();
        assert_eq!(
            err_message(&err),
            "drvAmptek::sendCommandString ACK error Send config: Bad Parameter\tI2C packet"
        );
    }

    /// The port is multi-device (asynPortDriver.cpp:4014), so an SCA write
    /// lands at its record's ADDR and `sendSCAs` sends SCA `i` from addr `i`,
    /// zero for a param never set there (`drvAmptek.cpp:479-508,855-857`).
    #[test]
    fn sca_writes_land_at_their_addr_and_send_scas_reads_each() {
        let status = [0u8; 64];
        let (mut driver, fake) = driver_with_fake(move |req| default_reply(req, &status));
        connect(&mut driver).unwrap();
        let low = driver.amptek[AmptekReason::SCALowChannel as usize];
        let level = driver.amptek[AmptekReason::SCAOutputLevel as usize];
        let mut user = AsynUser::new(low).with_addr(3);
        driver.write_int32(&mut user, 5).unwrap();
        let mut user = AsynUser::new(level).with_addr(7);
        driver.write_int32(&mut user, 1).unwrap();

        assert_eq!(driver.base.get_int32_param(low, 3).unwrap(), 5);
        assert!(driver.base.get_int32_param(low, 0).is_err());
        assert_eq!(
            driver.read_int32(&AsynUser::new(low).with_addr(3)).unwrap(),
            5
        );

        let requests = fake.requests.lock().unwrap();
        let sent: Vec<String> = requests
            .iter()
            .filter(|r| (r[2], r[3]) == (0x20, 0x02))
            .map(|r| String::from_utf8_lossy(&r[6..r.len() - 2]).into_owned())
            .collect();
        assert!(
            sent[0].starts_with("SCAI=1;SCAL=0;SCAH=0;SCAO=OFF;"),
            "{}",
            sent[0]
        );
        assert!(
            sent[0].contains("SCAI=4;SCAL=5;SCAH=0;SCAO=OFF;"),
            "{}",
            sent[0]
        );
        assert!(
            sent[1].contains("SCAI=4;SCAL=5;SCAH=0;SCAO=OFF;"),
            "{}",
            sent[1]
        );
        assert!(
            sent[1].ends_with("SCAI=8;SCAL=0;SCAH=0;SCAO=HIGH;"),
            "{}",
            sent[1]
        );
    }

    /// A param the device never reported sends C's zeroed value
    /// (`asynPortDriver.cpp:304`, `:386`): the readback set only CLCK and
    /// MCAC, and a Gain write still sends the whole configuration.
    #[test]
    fn send_configuration_sends_zero_for_an_undefined_param() {
        let status = [0u8; 64];
        let (mut driver, fake) = driver_with_fake(move |req| default_reply(req, &status));
        connect(&mut driver).unwrap();
        let gain = driver.amptek[AmptekReason::Gain as usize];
        assert!(driver.base.get_float64_param(gain, 0).is_err());
        let polarity = driver.amptek[AmptekReason::InputPolarity as usize];
        assert!(driver.base.get_int32_param(polarity, 0).is_err());
        driver.write_float64(&mut AsynUser::new(gain), 2.5).unwrap();

        let requests = fake.requests.lock().unwrap();
        let sent: Vec<String> = requests
            .iter()
            .filter(|r| (r[2], r[3]) == (0x20, 0x02))
            .map(|r| String::from_utf8_lossy(&r[6..r.len() - 2]).into_owned())
            .collect();
        assert_eq!(sent.len(), 1, "{sent:?}");
        assert!(
            sent[0].starts_with("CLCK=AUTO;AINP=POS;TPEA=0.000000;"),
            "{}",
            sent[0]
        );
        assert!(sent[0].contains("GAIN=2.500000;"), "{}", sent[0]);
    }

    /// An unset ConfigFile is C's empty `string` (`drvAmptek.cpp:897-903`):
    /// the save fails opening "", not on the param.
    #[test]
    fn save_config_file_with_no_file_name_tries_the_empty_name() {
        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        connect(&mut driver).unwrap();
        let save = driver.amptek[AmptekReason::SaveConfigFile as usize];
        let err = driver.write_int32(&mut AsynUser::new(save), 1).unwrap_err();
        assert!(!matches!(err, AsynError::ParamUndefined(_)), "{err:?}");
    }

    /// A failed `XMTPT_DISABLE_MCA_MCS` fails the read and keeps
    /// `acquiring_` set (`drvAmptek.cpp:941-943`).
    #[test]
    fn acquiring_read_returns_disable_mca_mcs_error() {
        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| match (req[2], req[3]) {
            (0xF0, _) => None,
            _ => default_reply(req, &status),
        });
        connect(&mut driver).unwrap();
        driver.acquiring = true;
        let user = AsynUser::new(driver.mca[McaReason::Acquiring as usize]);
        assert!(driver.read_int32(&user).is_err());
        assert!(driver.acquiring);
    }

    /// C's two "not found" texts (`drvAmptek.cpp:315-326`).
    #[test]
    fn connect_not_found_texts_match_c() {
        let (mut driver, _fake) = driver_with_fake(|_| None);
        let err = connect(&mut driver).unwrap_err();
        assert_eq!(err_message(&err), "Network DPP device 127.0.0.1 not found");

        // No broadcast interface: the search finds nothing.
        let transport = AmptekUdpTransport::bind_on_command_port(1).unwrap();
        let mut driver =
            AmptekDriver::with_link("fake", "127.0.0.1", DppLink::ethernet(transport, false))
                .unwrap();
        let err = connect(&mut driver).unwrap_err();
        assert_eq!(
            err_message(&err),
            "DPP device 127.0.0.1 not found, total devices found=0"
        );
    }

    /// A failed `epicsSocketCreate` is printed and the port still builds
    /// (`DppSocket.cpp:18-21`); its connects then find nothing.
    #[test]
    fn a_failed_socket_builds_a_port_that_never_connects() {
        for (direct, text) in [
            (true, "Network DPP device 127.0.0.1 not found"),
            (
                false,
                "DPP device 127.0.0.1 not found, total devices found=0",
            ),
        ] {
            let transport = AmptekUdpTransport::open(Err(std::io::Error::other("no socket")));
            let mut driver = AmptekDriver::configure(
                "nosock",
                DPP_INTERFACE_ETHERNET,
                "127.0.0.1",
                direct,
                transport,
            )
            .unwrap();
            assert_eq!(err_message(&connect(&mut driver).unwrap_err()), text);
        }
    }

    /// A config send nobody answers fails without the readback query
    /// (`drvAmptek.cpp:450-455`).
    #[test]
    fn unanswered_config_send_skips_the_readback() {
        let status = [0u8; 64];
        let (mut driver, fake) = driver_with_fake(move |req| match (req[2], req[3]) {
            (0x20, 0x02) => None,
            _ => default_reply(req, &status),
        });
        connect(&mut driver).unwrap();
        let before = fake.pids().len();
        assert!(driver.send_command_string("GAIN=1;").is_err());
        assert_eq!(fake.pids()[before..], [(0x20, 0x02)]);
    }

    #[test]
    fn all_covers_every_reason_in_declaration_order() {
        assert_eq!(AmptekReason::ALL.len(), AmptekReason::COUNT);
        for (i, r) in AmptekReason::ALL.iter().enumerate() {
            assert_eq!(*r as usize, i);
        }
    }

    #[test]
    fn drv_info_strings_match_c_verbatim() {
        assert_eq!(
            AmptekReason::InputPolarity.drv_info(),
            "AMPTEK_INPUT_POLARITY"
        );
        assert_eq!(AmptekReason::Clock.drv_info(), "AMPTEK_CLOCK");
        assert_eq!(AmptekReason::Gain.drv_info(), "AMPTEK_GAIN");
        assert_eq!(AmptekReason::Gate.drv_info(), "AMPTEK_GATE");
        assert_eq!(AmptekReason::McaSource.drv_info(), "AMPTEK_MCA_SOURCE");
        assert_eq!(AmptekReason::PurEnable.drv_info(), "AMPTEK_PUR_ENABLE");
        assert_eq!(AmptekReason::RtdEnable.drv_info(), "AMPTEK_RTD_ENABLE");
        assert_eq!(
            AmptekReason::FastThreshold.drv_info(),
            "AMPTEK_FAST_THRESHOLD"
        );
        assert_eq!(
            AmptekReason::SlowThreshold.drv_info(),
            "AMPTEK_SLOW_THRESHOLD"
        );
        assert_eq!(AmptekReason::PeakingTime.drv_info(), "AMPTEK_PEAKING_TIME");
        assert_eq!(
            AmptekReason::FastPeakingTime.drv_info(),
            "AMPTEK_FAST_PEAKING_TIME"
        );
        assert_eq!(AmptekReason::FlatTopTime.drv_info(), "AMPTEK_FLAT_TOP_TIME");
        assert_eq!(AmptekReason::ConfigFile.drv_info(), "AMPTEK_CONFIG_FILE");
        assert_eq!(
            AmptekReason::SaveConfigFile.drv_info(),
            "AMPTEK_SAVE_CONFIG_FILE"
        );
        assert_eq!(
            AmptekReason::LoadConfigFile.drv_info(),
            "AMPTEK_LOAD_CONFIG_FILE"
        );
        assert_eq!(AmptekReason::SlowCounts.drv_info(), "AMPTEK_SLOW_COUNTS");
        assert_eq!(AmptekReason::FastCounts.drv_info(), "AMPTEK_FAST_COUNTS");
        assert_eq!(AmptekReason::DetTemp.drv_info(), "AMPTEK_DET_TEMP");
        assert_eq!(AmptekReason::SetDetTemp.drv_info(), "AMPTEK_SET_DET_TEMP");
        assert_eq!(AmptekReason::BoardTemp.drv_info(), "AMPTEK_BOARD_TEMP");
        assert_eq!(AmptekReason::HighVoltage.drv_info(), "AMPTEK_HIGH_VOLTAGE");
        assert_eq!(
            AmptekReason::SetHighVoltage.drv_info(),
            "AMPTEK_SET_HIGH_VOLTAGE"
        );
        assert_eq!(
            AmptekReason::MCSLowChannel.drv_info(),
            "AMPTEK_MCS_LOW_CHANNEL"
        );
        assert_eq!(
            AmptekReason::MCSHighChannel.drv_info(),
            "AMPTEK_MCS_HIGH_CHANNEL"
        );
        assert_eq!(AmptekReason::Model.drv_info(), "AMPTEK_MODEL");
        assert_eq!(AmptekReason::Firmware.drv_info(), "AMPTEK_FIRMWARE");
        assert_eq!(AmptekReason::Build.drv_info(), "AMPTEK_BUILD");
        assert_eq!(AmptekReason::Fpga.drv_info(), "AMPTEK_FPGA");
        assert_eq!(
            AmptekReason::SerialNumber.drv_info(),
            "AMPTEK_SERIAL_NUMBER"
        );
        assert_eq!(AmptekReason::AuxOut1.drv_info(), "AMPTEK_AUX_OUT1");
        assert_eq!(AmptekReason::AuxOut2.drv_info(), "AMPTEK_AUX_OUT2");
        assert_eq!(AmptekReason::AuxOut34.drv_info(), "AMPTEK_AUX_OUT34");
        assert_eq!(AmptekReason::Connect1.drv_info(), "AMPTEK_CONNECT1");
        assert_eq!(AmptekReason::Connect2.drv_info(), "AMPTEK_CONNECT2");
        assert_eq!(
            AmptekReason::SCAOutputWidth.drv_info(),
            "AMPTEK_SCA_OUTPUT_WIDTH"
        );
        assert_eq!(
            AmptekReason::SCALowChannel.drv_info(),
            "AMPTEK_SCA_LOW_CHANNEL"
        );
        assert_eq!(
            AmptekReason::SCAHighChannel.drv_info(),
            "AMPTEK_SCA_HIGH_CHANNEL"
        );
        assert_eq!(
            AmptekReason::SCAOutputLevel.drv_info(),
            "AMPTEK_SCA_OUTPUT_LEVEL"
        );
    }

    #[test]
    fn param_types_match_c_verbatim() {
        for r in [
            AmptekReason::Gain,
            AmptekReason::FastThreshold,
            AmptekReason::SlowThreshold,
            AmptekReason::PeakingTime,
            AmptekReason::FlatTopTime,
            AmptekReason::SlowCounts,
            AmptekReason::FastCounts,
            AmptekReason::DetTemp,
            AmptekReason::SetDetTemp,
            AmptekReason::BoardTemp,
            AmptekReason::HighVoltage,
        ] {
            assert_eq!(r.param_type(), ParamType::Float64, "{r:?}");
        }
        for r in [
            AmptekReason::ConfigFile,
            AmptekReason::Model,
            AmptekReason::Firmware,
            AmptekReason::Fpga,
        ] {
            assert_eq!(r.param_type(), ParamType::Octet, "{r:?}");
        }
        assert_eq!(AmptekReason::Clock.param_type(), ParamType::Int32);
        assert_eq!(AmptekReason::SCAOutputLevel.param_type(), ParamType::Int32);
    }

    #[test]
    fn new_builds_usb_only_with_a_host_bus() {
        let built = AmptekDriver::new("test1", 1, "", false);
        assert_eq!(built.is_ok(), usb::host_bus().is_some());
        if let Ok(driver) = built {
            assert!(matches!(driver.link, DppLink::Usb(_)));
        }
    }

    /// C builds a Serial port with `directMode=0` whose every connect finds
    /// nothing (`drvAmptek.cpp:144-147`, `ConsoleHelper.cpp:57-58`).
    #[test]
    fn a_serial_port_builds_and_never_connects() {
        let mut driver = AmptekDriver::new("test2", 2, "/dev/ttyS0", true).unwrap();
        assert!(matches!(driver.link, DppLink::Serial));
        assert_eq!(
            err_message(&connect(&mut driver).unwrap_err()),
            "DPP device /dev/ttyS0 not found, total devices found=0"
        );
        assert!(!driver.link.is_connected());
    }

    /// C traces an unknown interface type and leaves the port with a NULL
    /// `addressInfo_` (`drvAmptek.cpp:148-152`): `connectDevice` is a bare
    /// asynError (:285-288) until a `hostInfo` write sets one (:224-225).
    #[test]
    fn an_unknown_interface_type_traces_and_builds_an_address_less_port() {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::TraceFile;

        let path = std::env::temp_dir().join(format!("amptek-iface-{}.trace", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let services = PortServices::global();
        services
            .trace()
            .set_trace_file(Some("test3"), TraceFile::File(file));

        let mut driver = AmptekDriver::new("test3", 99, "192.168.0.1", false).unwrap();
        let out = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            out.ends_with(" drvAmptek::drvAmptek unknown interface type=99\n"),
            "{out:?}"
        );
        assert!(driver.base.is_connected());
        assert_eq!(err_message(&connect(&mut driver).unwrap_err()), "");
        assert_eq!(driver.get_option("hostInfo").unwrap(), "(null)");
        driver
            .set_option(&mut AsynUser::new(0), "hostInfo", "192.168.0.2")
            .unwrap();
        assert_eq!(driver.get_option("hostInfo").unwrap(), "192.168.0.2");
    }

    /// C's port is connected by the base asynPortDriver::connect at
    /// registration; device 0 carries the DPP link.
    #[test]
    fn new_starts_with_the_port_connected_and_device_0_not() {
        let driver = AmptekDriver::new("test4", 0, "192.168.0.1", false).unwrap();
        assert!(driver.base.is_connected());
        assert!(!driver.base.is_device_connected(0));
    }

    #[test]
    fn device_0_connect_failure_leaves_the_port_connected() {
        let (mut driver, _fake) = driver_with_fake(|_| None);
        let user = AsynUser::new(0).with_addr(0);
        assert!(driver.connect_addr(&user).is_err());
        assert!(driver.base.is_connected());
        assert!(!driver.base.is_device_connected(0));
    }

    #[test]
    fn device_connect_marks_that_device_only() {
        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        driver.connect_addr(&AsynUser::new(0).with_addr(0)).unwrap();
        assert!(driver.base.is_device_connected(0));
        assert!(!driver.base.is_device_connected(3));
        driver.connect_addr(&AsynUser::new(0).with_addr(3)).unwrap();
        assert!(driver.base.is_device_connected(3));
    }

    #[test]
    fn sca_device_connect_before_the_link_is_up_sends_nothing() {
        let status = [0u8; 64];
        let (mut driver, fake) = driver_with_fake(move |req| default_reply(req, &status));
        assert!(driver.connect_addr(&AsynUser::new(0).with_addr(3)).is_err());
        assert!(!driver.base.is_device_connected(3));
        assert!(fake.pids().is_empty());
    }

    /// `disconnect`'s `exceptionDisconnect(pasynUser)` (`drvAmptek.cpp:194`)
    /// downs and announces only the caller's dpCommon.
    #[test]
    fn disconnect_announces_on_the_callers_dp_common_only() {
        use epics_rs::asyn::exception::{AsynException, ExceptionEvent};
        use epics_rs::asyn::registry::PortRegistry;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;

        type Heard = Arc<Mutex<Vec<(AsynException, i32)>>>;
        fn recorder(heard: &Heard) -> impl Fn(&ExceptionEvent<'_>) + Send + Sync + 'static {
            let heard = Arc::clone(heard);
            move |ev| heard.lock().unwrap().push((ev.exception, ev.addr))
        }

        let status = [0u8; 64];
        let (driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        let claim = PortRegistry::new().claim("fake").unwrap();
        let (runtime, _actor) =
            create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        let handle = runtime.port_handle();
        let (device0, port): (Heard, Heard) = Default::default();
        let _g0 = handle.exception_callback_add(0, recorder(&device0));
        let _gp = handle.exception_callback_add(-1, recorder(&port));

        handle.connect_addr_blocking(0).unwrap();
        handle.disconnect_addr_blocking(0).unwrap();

        let connect = AsynException::Connect;
        assert_eq!(*device0.lock().unwrap(), [(connect, 0), (connect, 0)]);
        assert!(port.lock().unwrap().is_empty());
        assert!(!handle.connect_device(0).connected());
        assert!(handle.connect_device(-1).connected());
    }

    /// The trace port "fake" prints, with the PORT prefix, while a user with
    /// reason 5 connects device 0 to a [`FakeDp5`] whose configuration has
    /// two keys. A direct-built driver traces through the process-global
    /// services; nextest runs each test in its own process.
    fn trace_of_a_connect(mask: TraceMask) -> String {
        use epics_rs::asyn::services::PortServices;
        use epics_rs::asyn::trace::{TraceFile, TraceInfoMask};

        let path = std::env::temp_dir().join(format!("mca-amptek-{}.trace", std::process::id()));
        let file = Arc::new(Mutex::new(std::fs::File::create(&path).unwrap()));
        let services = PortServices::global();
        let trace = services.trace();
        trace.register_port("fake", true);
        trace.set_trace_mask(Some("fake"), mask);
        trace.set_trace_info_mask(Some("fake"), TraceInfoMask::PORT);
        trace.set_trace_file(Some("fake"), TraceFile::File(file));

        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        driver.connect_addr(&AsynUser::new(5).with_addr(0)).unwrap();
        let out = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        out
    }

    /// `connectDevice` traces its connected line on the caller's
    /// `pasynUser`, not `pasynUserSelf` (`drvAmptek.cpp:311`).
    #[test]
    fn connect_device_traces_on_the_callers_user() {
        let out = trace_of_a_connect(TraceMask::ERROR | TraceMask::IO_DRIVER);
        assert!(
            out.contains("[fake,0,5] drvAmptek::connectDevice DPP device 127.0.0.1 connected\n"),
            "{out}"
        );
    }

    /// Each of C's error formats ends in its newline (`drvAmptek.cpp:671`).
    #[test]
    fn error_lines_end_in_c_newline() {
        let out = trace_of_a_connect(TraceMask::ERROR);
        assert!(
            out.contains(
                "[fake,0,0] drvAmptek::parseConfigEnum string AINP= not found in configuration\n\
                 [fake,0,0] drvAmptek::parseConfigDouble string TPEA= not found in configuration\n"
            ),
            "{out}"
        );
    }

    /// A USB driver over DP5s reporting serial numbers `serials`.
    fn usb_driver(
        address: &str,
        serials: &[u8],
    ) -> (AmptekDriver, Arc<Mutex<crate::usb::fake::Log>>) {
        use crate::usb::fake::{FakeBus, FakeDevice};
        let devices = serials
            .iter()
            .map(|&sn| {
                let mut status = [0u8; 64];
                status[26] = sn;
                FakeDevice::dp5(move |req| {
                    default_reply(req, &status).ok_or(crate::usb::UsbError::Timeout)
                })
            })
            .collect();
        let bus = FakeBus::new(devices);
        let log = bus.log.clone();
        let driver = AmptekDriver::with_usb_bus("usb", address, Box::new(bus)).unwrap();
        (driver, log)
    }

    /// `LibUsb_Connect_DPP_by_SN` opens each DP5 until one reports the
    /// serial; later commands go to that one.
    #[test]
    fn a_usb_serial_number_connects_the_device_that_reports_it() {
        let (mut driver, log) = usb_driver("9", &[7, 9]);
        connect(&mut driver).unwrap();
        let idx = driver.amptek[AmptekReason::SerialNumber as usize];
        assert_eq!(driver.base.get_int32_param(idx, 0).unwrap(), 9);
        let log = log.lock().unwrap();
        assert_eq!(log.opened, vec![0, 1]);
        assert_eq!(log.released, vec![0]);
        assert!(log.sent.iter().skip(1).all(|sent| sent.0 == 1));
    }

    #[test]
    fn an_empty_usb_address_connects_the_first_dp5() {
        let (mut driver, log) = usb_driver("", &[7, 9]);
        connect(&mut driver).unwrap();
        let idx = driver.amptek[AmptekReason::SerialNumber as usize];
        assert_eq!(driver.base.get_int32_param(idx, 0).unwrap(), 7);
        assert_eq!(log.lock().unwrap().opened, vec![0]);
    }

    #[test]
    fn an_unmatched_usb_serial_number_is_not_found() {
        let (mut driver, _) = usb_driver("5", &[7, 9]);
        assert_eq!(
            err_message(&connect(&mut driver).unwrap_err()),
            "DPP device 5 not found, total devices found=0"
        );
    }

    /// C keeps the device claimed when the status query after `ConnectDpp`
    /// fails; it is released here, so the next connect can claim it.
    #[test]
    fn a_failed_usb_connect_releases_the_device() {
        use crate::usb::fake::{FakeBus, FakeDevice};
        let bus = FakeBus::new(vec![FakeDevice::dp5(|_| {
            Err(crate::usb::UsbError::Timeout)
        })]);
        let log = bus.log.clone();
        let mut driver = AmptekDriver::with_usb_bus("usb", "", Box::new(bus)).unwrap();
        assert_eq!(
            err_message(&connect(&mut driver).unwrap_err()),
            "error calling SendCommand for XMTPT_SEND_STATUS"
        );
        assert!(!driver.link.is_connected());
        assert_eq!(log.lock().unwrap().released, vec![0]);
    }

    /// `disconnect` closes the USB connection (`drvAmptek.cpp:190`).
    #[test]
    fn disconnect_releases_the_usb_device() {
        let (mut driver, log) = usb_driver("", &[7]);
        connect(&mut driver).unwrap();
        driver.do_disconnect(0);
        assert!(!driver.link.is_connected());
        assert_eq!(log.lock().unwrap().released, vec![0]);
    }

    /// `checkFailedComm` disconnects `pasynUserSelf`, device 0.
    #[test]
    fn failed_comm_disconnects_device_0_not_the_port() {
        let status = [0u8; 64];
        let (mut driver, _fake) = driver_with_fake(move |req| default_reply(req, &status));
        driver.connect_addr(&AsynUser::new(0).with_addr(0)).unwrap();
        driver.failed_sends = MAX_FAILED_SENDS;
        driver.check_failed_comm("test");
        assert!(!driver.link.is_connected());
        assert!(!driver.base.is_device_connected(0));
        assert!(driver.base.is_connected());
    }

    /// `setParamsAlarm` touches only the alarm fields (`drvAmptek.cpp:237-248`).
    #[test]
    fn disconnect_sets_comm_alarm_and_keeps_param_status() {
        let mut driver = AmptekDriver::new("alarm1", 0, "192.168.0.1", false).unwrap();
        let gain = driver.amptek[AmptekReason::Gain as usize];
        driver
            .base
            .set_param_status(gain, 3, AsynStatus::Timeout, 0, 0)
            .unwrap();
        if let DppLink::Ethernet { dpp_addr, .. } = &mut driver.link {
            *dpp_addr = Some(Ipv4Addr::LOCALHOST);
        }
        driver.do_disconnect(0);

        let invalid = AlarmSeverity::Invalid as u16;
        assert_eq!(
            driver.base.get_param_status(gain, 3).unwrap(),
            (AsynStatus::Timeout, alarm_status::COMM_ALARM, invalid)
        );
        assert_eq!(
            driver.base.get_param_status(gain, 0).unwrap(),
            (AsynStatus::Success, alarm_status::COMM_ALARM, invalid)
        );
    }

    #[test]
    fn insert_crlf_after_semicolons_matches_processcfgreadex() {
        assert_eq!(
            insert_crlf_after_semicolons(b"CLCK=AUTO;TPEA=12.5;"),
            "CLCK=AUTO;\r\nTPEA=12.5;\r\n"
        );
    }

    #[test]
    fn resolve_ipv4_parses_dotted_quad() {
        assert_eq!(
            resolve_ipv4("192.168.0.42"),
            Some(Ipv4Addr::new(192, 168, 0, 42))
        );
    }
}
