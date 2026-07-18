//! `drvAmptek` -- the Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X asyn MCA/MCS
//! driver, ported from `drvAmptek.cpp`/`drvAmptek.h` (`mcaApp/AmptekSrc`).
//! Ethernet/UDP only -- see [`crate`]'s module doc for the USB/Serial
//! feasibility gate.
//!
//! # Architecture
//! Every dispatch (`write_int32`, `read_configuration_from_hardware`, ...)
//! runs its UDP round trip inline, on whichever thread asyn-rs's
//! `can_block: true` worker calls it from -- there is no separate
//! command/response thread, matching upstream C's own single-threaded
//! `CConsoleHelper` (the vendor SDK has no internal concurrency either; the
//! 10 ms/50 ms/100 ms waits [`crate::transport`] reproduces are the same
//! blocking waits `drvAmptek.cpp` itself performs on asyn's own worker
//! thread).
//!
//! # Restructuring vs. C
//! - **`multi_device: true`, despite `drvAmptek`'s own `asynFlags` argument
//!   never setting `ASYN_MULTIDEVICE`** (`drvAmptek.cpp:68`: only
//!   `ASYN_CANBLOCK`, with the comment "not multi-device"). Real C asyn
//!   grants per-address parameter storage from `maxAddr > 1` alone,
//!   independent of that flag -- `drvAmptek` passes `MAX_SCAS` as `maxAddr`
//!   specifically so `sendSCAs`'s `getIntegerParam(i, ...)` calls
//!   (`drvAmptek.cpp:479-508`, `i=0..MAX_SCAS`) read genuinely distinct
//!   per-channel storage. asyn-rs instead normalizes every address to 0 for
//!   param storage unless `PortFlags::multi_device` is set, so setting it
//!   here is required to preserve that per-SCA storage, not a stylistic
//!   choice -- a literal flag translation would silently collapse all 8 SCA
//!   channels onto one shared set of fields.
//! - **USB/Serial rejected at construction**, not merely left to fail at
//!   connect time. C's constructor switch (`drvAmptek.cpp:140-153`) forces
//!   `directMode=0` for `DppInterfaceUSB`/`DppInterfaceSerial` and
//!   *proceeds* -- the driver instance exists but can never actually
//!   connect (`ConnectDpp`'s USB branch needs `libusb`, its Serial branch is
//!   an empty no-op; see [`crate`]'s module doc). [`AmptekDriver::new`]
//!   instead returns `Err` for anything but the Ethernet interface, making
//!   the feasibility gate visible at IOC-startup time (an iocsh
//!   config-command failure) rather than a confusing later connect failure.
//!   C's genuinely-unrecognized-type arm (partial construction, a bare
//!   `return;` leaving the instance half-built, `drvAmptek.cpp:148-152`) is
//!   likewise replaced by the same `Err` return.
//! - **`connect()`'s `addr > 0` early return is omitted.** C's `connect`
//!   (`drvAmptek.cpp:161-183`) special-cases a per-address connect request
//!   as a cheap "is the port already connected" check -- an artifact of
//!   `pasynManager` invoking `connect` once per configured address. asyn-rs
//!   only ever calls a `PortDriver`'s port-level `connect` for addr 0 (see
//!   `connect_addr`/`disconnect_addr` for the per-address hooks), so that
//!   branch has no equivalent call site here.
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
//!   bytes it is sent in two [`AmptekDriver::send_command_string`] calls,
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
//!   as the "offending command". [`AmptekDriver::send_command_string`]
//!   instead only calls [`protocol::offending_command_text`] when
//!   [`protocol::parse_packet`] classified the response as
//!   [`ReceivedPacket::Ack`] -- a classification that (unlike `ParseCmd`)
//!   keys off `(pid1, pid2)` jointly, so the collision is structurally
//!   unrepresentable rather than patched around.
//!
//! See also `protocol.rs`'s `AsciiCmdUtilities.h:17` and `net_finder.rs`'s
//! `DppSocket.cpp:428-450` notes for the two defects found while porting
//! the wire/discovery layers this module depends on.

use std::net::{Ipv4Addr, SocketAddrV4};
use std::path::Path;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::AlarmSeverity;

use mca::interface::McaReason;

use crate::ascii_cmd;
use crate::config::{self, ConfigFields, MAX_SCAS, ScaFields};
use crate::protocol::{self, ReceivedPacket, TransmitPacketType};
use crate::status::{self, Dp5Status};
use crate::transport::{AmptekUdpTransport, NETFINDER_BROADCAST_PORT};

/// `DppInterface_t` (`DppConst.h`): only `DppInterfaceEthernet` (`0`) is
/// accepted by [`AmptekDriver::new`] -- see the module doc's "USB/Serial
/// rejected at construction" note.
const DPP_INTERFACE_ETHERNET: i32 = 0;

/// `drvAmptek.cpp:51`: `#define MAX_FAILED_SENDS 10`.
const MAX_FAILED_SENDS: u32 = 10;

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

fn disconnected_error() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Disconnected,
        message: "not connected".to_string(),
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

/// C's `drvAmptek` (`drvAmptek.h`).
pub struct AmptekDriver {
    base: PortDriverBase,
    mca: [usize; McaReason::COUNT],
    amptek: [usize; AmptekReason::COUNT],
    transport: AmptekUdpTransport,
    address_info: String,
    direct_mode: bool,
    /// `CConsoleHelper::isConnected` -- the confirmed device address, once
    /// [`AmptekDriver::connect_device`] has probed it; `None` before the
    /// first connect and after every disconnect (`connectDevice` re-resolves
    /// `addressInfo_` fresh on each attempt, `drvAmptek.cpp:290-301`, so
    /// nothing here is cached across a disconnect/reconnect cycle).
    target: Option<Ipv4Addr>,
    /// `drvAmptek::acquiring_`.
    acquiring: bool,
    /// `drvAmptek::failedSends_`.
    failed_sends: u32,
    /// `CConsoleHelper::DP5Stat.m_DP5_Status`, the last successfully
    /// decoded status block. Initialized from an all-zero block (matching
    /// a zero-initialized C struct member) rather than `Option` -- every
    /// read site already tolerates the all-zero baseline the same way C's
    /// own pre-connect zeroed struct does.
    last_status: Dp5Status,
    /// `CConsoleHelper::HwCfgDP5`.
    hw_cfg_dp5: String,
}

impl AmptekDriver {
    /// `drvAmptekConfigure(portName, interfaceType, addressInfo, directMode)`
    /// (`drvAmptek.cpp:63-159`). Returns `Err` for any `interface_type`
    /// other than Ethernet (`0`) -- see the module doc's "USB/Serial
    /// rejected at construction" note.
    pub fn new(
        port_name: &str,
        interface_type: i32,
        address_info: &str,
        direct_mode: bool,
    ) -> AsynResult<Self> {
        if interface_type != DPP_INTERFACE_ETHERNET {
            return Err(protocol_error(format!(
                "unsupported interface type={interface_type}: only Ethernet (0) is supported \
                 by this port (USB/Serial are feasibility-gated -- see the mca-amptek crate doc)"
            )));
        }

        let mut base = PortDriverBase::new(
            port_name,
            MAX_SCAS,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        base.init_connected(false);

        let mca = McaReason::create_params(&mut base)?;
        let amptek = AmptekReason::create_params(&mut base)?;
        let transport = AmptekUdpTransport::bind()?;

        Ok(AmptekDriver {
            base,
            mca,
            amptek,
            transport,
            address_info: address_info.to_string(),
            direct_mode,
            target: None,
            acquiring: false,
            failed_sends: 0,
            last_status: status::process_status(&[0u8; 64]),
            hw_cfg_dp5: String::new(),
        })
    }

    /// `drvAmptek::checkFailedComm` (`drvAmptek.cpp:250-258`).
    fn check_failed_comm(&mut self) {
        self.failed_sends += 1;
        if self.failed_sends > MAX_FAILED_SENDS {
            self.do_disconnect();
        }
    }

    /// The disconnect transition itself, shared by [`PortDriver::disconnect`]
    /// and [`Self::check_failed_comm`]'s "too many failed sends" path
    /// (`drvAmptek.cpp:185-198,252-257`).
    fn do_disconnect(&mut self) {
        if !self.base.is_connected() {
            return;
        }
        self.target = None;
        self.base.set_connected(false);
        let _ = self.set_params_alarm(
            AsynStatus::Error,
            alarm_status::COMM_ALARM,
            AlarmSeverity::Invalid as u16,
        );
    }

    /// `drvAmptek::setParamsAlarm` (`drvAmptek.cpp:237-248`): every MCA and
    /// Amptek param, at every SCA address, gets the same alarm status/
    /// severity.
    fn set_params_alarm(
        &mut self,
        status: AsynStatus,
        alarm_status: u16,
        alarm_severity: u16,
    ) -> AsynResult<()> {
        for addr in 0..MAX_SCAS as i32 {
            for &idx in self.mca.iter().chain(self.amptek.iter()) {
                self.base
                    .set_param_status(idx, addr, status, alarm_status, alarm_severity)?;
            }
            self.base.call_param_callbacks(addr)?;
        }
        Ok(())
    }

    /// `CConsoleHelper::SendCommand` (`ConsoleHelper.cpp:85-104`, Ethernet
    /// branch): send the fixed-shape request packet for `cmd` to `target`
    /// and wait for a response, with no bookkeeping and no parsing -- both
    /// are the caller's decision (see [`Self::send_command`] and
    /// [`Self::send_and_receive`]).
    fn raw_send(&self, target: Ipv4Addr, cmd: TransmitPacketType) -> AsynResult<Vec<u8>> {
        let expected_size = if cmd == TransmitPacketType::SendStatus {
            72
        } else {
            24648
        };
        Ok(self
            .transport
            .send_packet_inet(target, &protocol::build_command(cmd), expected_size)?)
    }

    /// `drvAmptek::sendCommand` (`drvAmptek.cpp:361-382`): fire-and-forget
    /// -- success means only "got any response bytes", the content is never
    /// parsed. [`Self::send_and_receive`] is the variant the three callers
    /// that also need the response parsed use instead.
    fn send_command(&mut self, cmd: TransmitPacketType) -> AsynResult<()> {
        let target = self.target.ok_or_else(disconnected_error)?;
        let raw = self.raw_send(target, cmd)?;
        if raw.is_empty() {
            self.check_failed_comm();
            return Err(protocol_error(format!(
                "error calling CH_.SendCommand({cmd:?})"
            )));
        }
        self.failed_sends = 0;
        Ok(())
    }

    /// `CH_.SendCommand(cmd)` + the separate `CH_.ReceiveData()` parse step
    /// (`ConsoleHelper.cpp:690-733`), collapsed into one round trip since
    /// there is no real gap between them (`ReceiveData` always parses
    /// whatever `SendCommand` just stored). `Err` only for the send half
    /// failing (empty response); a *parse* failure (framing error or
    /// unmatched PID) surfaces as `Ok(ReceivedPacket::Error{..})` -- callers
    /// match on that themselves, since C's own callers disagree on whether
    /// a parse failure also counts toward `failedSends_` (some do, some
    /// don't -- see each call site below).
    fn send_and_receive(
        &mut self,
        target: Ipv4Addr,
        cmd: TransmitPacketType,
    ) -> AsynResult<ReceivedPacket> {
        let raw = self.raw_send(target, cmd)?;
        if raw.is_empty() {
            self.check_failed_comm();
            return Err(protocol_error(format!(
                "error calling CH_.SendCommand({cmd:?})"
            )));
        }
        self.failed_sends = 0;
        Ok(protocol::parse_packet(&raw))
    }

    /// `drvAmptek::connectDevice` (`drvAmptek.cpp:279-359`).
    fn connect_device(&mut self) -> AsynResult<()> {
        let ip = resolve_ipv4(&self.address_info).ok_or_else(|| {
            protocol_error(format!("Cannot resolve address: {}", self.address_info))
        })?;

        let confirmed = if self.direct_mode {
            self.transport.connect_direct(ip)?
        } else {
            self.transport.discover_default(
                SocketAddrV4::new(Ipv4Addr::BROADCAST, NETFINDER_BROADCAST_PORT),
                ip,
            )?
        };
        if !confirmed {
            return Err(protocol_error(format!(
                "DPP device {} not found",
                self.address_info
            )));
        }

        // `CH_.DP5Stat.m_DP5_Status.SerialNumber = 0;` before the status
        // round trip (`drvAmptek.cpp:330`) -- reset the whole cached status
        // to its zeroed baseline here rather than just the one field, so a
        // status round-trip failure right after a successful discovery
        // cannot leave a stale *previous* connection's status behind.
        self.last_status = status::process_status(&[0u8; 64]);
        let raw = self.raw_send(ip, TransmitPacketType::SendStatus)?;
        if raw.is_empty() {
            return Err(protocol_error(
                "error calling SendCommand for XMTPT_SEND_STATUS",
            ));
        }
        let ReceivedPacket::Status { data } = protocol::parse_packet(&raw) else {
            return Err(protocol_error(
                "calling ReceiveData() for XMTPT_SEND_STATUS",
            ));
        };
        self.last_status = status::process_status(&data);
        self.target = Some(ip);

        // `readConfigurationFromHardware()`'s return value is not checked
        // here (`drvAmptek.cpp:341`) -- connection proceeds regardless.
        let _ = self.read_configuration_from_hardware();

        let dpp_type = self.last_status.dpp_type;
        self.base.set_string_param(
            self.amptek[AmptekReason::Model as usize],
            0,
            dpp_type.name().to_string(),
        )?;
        self.base.set_int32_param(
            self.amptek[AmptekReason::SerialNumber as usize],
            0,
            self.last_status.serial_number.unwrap_or(0) as i32,
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

        self.set_params_alarm(
            AsynStatus::Success,
            alarm_status::NO_ALARM,
            AlarmSeverity::NoAlarm as u16,
        )?;
        Ok(())
    }

    /// `drvAmptek::sendCommandString` (`drvAmptek.cpp:437-477`).
    fn send_command_string(&mut self, command_string: &str) -> AsynResult<()> {
        let target = self.target.ok_or_else(disconnected_error)?;
        let packet = protocol::build_config_packet_ex(command_string);
        let send_status: AsynResult<()> = match self.raw_config_round_trip(target, packet) {
            Ok(raw) if raw.is_empty() => {
                self.check_failed_comm();
                Err(protocol_error(format!(
                    "error calling CH_.SendCommand_Config(XMTPT_SEND_CONFIG_PACKET_EX, {command_string})"
                )))
            }
            Ok(raw) => {
                self.failed_sends = 0;
                match protocol::parse_packet(&raw) {
                    ReceivedPacket::Error { status } => Err(protocol_error(format!(
                        "error in response for CH_.SendCommand_Config(XMTPT_SEND_CONFIG_PACKET_EX, \
                         {command_string}): {status:?}"
                    ))),
                    ReceivedPacket::Ack { status, data } => {
                        match protocol::offending_command_text(status, &data) {
                            Some(text) => Err(protocol_error(format!("ACK error {text}"))),
                            None => Ok(()),
                        }
                    }
                    _ => Ok(()),
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

    fn raw_config_round_trip(&self, target: Ipv4Addr, packet: Vec<u8>) -> AsynResult<Vec<u8>> {
        Ok(self.transport.send_packet_inet(target, &packet, 8)?)
    }

    /// `drvAmptek::saveConfigurationFile` (`drvAmptek.cpp:384-398`).
    fn save_configuration_file(&self, file_name: &str) -> AsynResult<()> {
        let content = format!("[DP5 Configuration File]\r\n{}", self.hw_cfg_dp5);
        std::fs::write(file_name, content)?;
        Ok(())
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
            // C discards the first half's status unconditionally
            // (`drvAmptek.cpp:429-433`, a plain reassignment) -- see the
            // module doc's "Preserved upstream quirks" note.
            let _ = self.send_command_string(&cfg[..split]);
            self.send_command_string(&cfg[split..])
        } else {
            self.send_command_string(&cfg)
        }
    }

    /// `drvAmptek::sendSCAs` (`drvAmptek.cpp:479-508`).
    fn send_scas(&mut self) -> AsynResult<()> {
        let mut scas = [ScaFields {
            low_channel: 0,
            high_channel: 0,
            output_level: 0,
        }; MAX_SCAS];
        for (i, sca) in scas.iter_mut().enumerate() {
            let addr = i as i32;
            sca.low_channel = self
                .base
                .get_int32_param(self.amptek[AmptekReason::SCALowChannel as usize], addr)?;
            sca.high_channel = self
                .base
                .get_int32_param(self.amptek[AmptekReason::SCAHighChannel as usize], addr)?;
            sca.output_level = self
                .base
                .get_int32_param(self.amptek[AmptekReason::SCAOutputLevel as usize], addr)?
                as u8;
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
            clock: self
                .base
                .get_int32_param(a[AmptekReason::Clock as usize], 0)? as u8,
            input_polarity: self
                .base
                .get_int32_param(a[AmptekReason::InputPolarity as usize], 0)?
                as u8,
            peaking_time: self
                .base
                .get_float64_param(a[AmptekReason::PeakingTime as usize], 0)?,
            fast_peaking_time: self
                .base
                .get_int32_param(a[AmptekReason::FastPeakingTime as usize], 0)?
                as u8,
            flat_top_time: self
                .base
                .get_float64_param(a[AmptekReason::FlatTopTime as usize], 0)?,
            gain: self
                .base
                .get_float64_param(a[AmptekReason::Gain as usize], 0)?,
            slow_threshold: self
                .base
                .get_float64_param(a[AmptekReason::SlowThreshold as usize], 0)?,
            fast_threshold: self
                .base
                .get_float64_param(a[AmptekReason::FastThreshold as usize], 0)?,
            num_channels: self
                .base
                .get_int32_param(m[McaReason::NumChannels as usize], 0)?,
            gate: self
                .base
                .get_int32_param(a[AmptekReason::Gate as usize], 0)? as u8,
            preset_real_time: self
                .base
                .get_float64_param(m[McaReason::PresetRealTime as usize], 0)?,
            preset_live_time: self
                .base
                .get_float64_param(m[McaReason::PresetLiveTime as usize], 0)?,
            preset_counts: self
                .base
                .get_float64_param(m[McaReason::PresetCounts as usize], 0)?,
            preset_low_channel: self
                .base
                .get_int32_param(m[McaReason::PresetLowChannel as usize], 0)?,
            preset_high_channel: self
                .base
                .get_int32_param(m[McaReason::PresetHighChannel as usize], 0)?,
            mca_source: self
                .base
                .get_int32_param(a[AmptekReason::McaSource as usize], 0)?
                as u8,
            pur_enable: self
                .base
                .get_int32_param(a[AmptekReason::PurEnable as usize], 0)?
                as u8,
            set_high_voltage: self
                .base
                .get_int32_param(a[AmptekReason::SetHighVoltage as usize], 0)?,
            set_det_temp: self
                .base
                .get_float64_param(a[AmptekReason::SetDetTemp as usize], 0)?,
            mcs_low_channel: self
                .base
                .get_int32_param(a[AmptekReason::MCSLowChannel as usize], 0)?,
            mcs_high_channel: self
                .base
                .get_int32_param(a[AmptekReason::MCSHighChannel as usize], 0)?,
            dwell_time: self
                .base
                .get_float64_param(m[McaReason::DwellTime as usize], 0)?,
            aux_out1: self
                .base
                .get_int32_param(a[AmptekReason::AuxOut1 as usize], 0)? as u8,
            aux_out2: self
                .base
                .get_int32_param(a[AmptekReason::AuxOut2 as usize], 0)? as u8,
            aux_out34: self
                .base
                .get_int32_param(a[AmptekReason::AuxOut34 as usize], 0)?,
            connect1: self
                .base
                .get_int32_param(a[AmptekReason::Connect1 as usize], 0)?
                as u8,
            connect2: self
                .base
                .get_int32_param(a[AmptekReason::Connect2 as usize], 0)?
                as u8,
            sca_output_width: self
                .base
                .get_int32_param(a[AmptekReason::SCAOutputWidth as usize], 0)?
                as u8,
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
        let parsed = config::parse_configuration(&self.hw_cfg_dp5, dpp_type);
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
        let target = self.target.ok_or_else(disconnected_error)?;
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
            raw = self.raw_config_round_trip(target, packet.clone())?;
            if !raw.is_empty() {
                sent = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        if !sent {
            return Err(protocol_error(
                "error calling SendCommand_Config() for XMTPT_FULL_READ_CONFIG_PACKET",
            ));
        }
        self.failed_sends = 0;

        match protocol::parse_packet(&raw) {
            ReceivedPacket::ConfigReadback { data } => {
                self.hw_cfg_dp5 = insert_crlf_after_semicolons(&data);
                self.apply_parsed_configuration()
            }
            ReceivedPacket::Error { status } => {
                self.check_failed_comm();
                Err(protocol_error(format!(
                    "error calling ReceiveData() for XMTPT_FULL_READ_CONFIG_PACKET: {status:?}"
                )))
            }
            // `ReceiveData()==true` but `HwCfgReady` was never set --
            // matches C falling through to `return asynError` when the
            // classified packet wasn't a config readback.
            _ => Err(protocol_error(
                "XMTPT_FULL_READ_CONFIG_PACKET response was not a configuration readback",
            )),
        }
    }

    /// `drvAmptek::writeInt32`'s `mcaReadStatus_` branch (`drvAmptek.cpp:
    /// 873-894`).
    fn handle_read_status(&mut self) -> AsynResult<()> {
        let target = self.target.ok_or_else(disconnected_error)?;
        match self.send_and_receive(target, TransmitPacketType::SendStatus)? {
            ReceivedPacket::Status { data } => {
                self.last_status = status::process_status(&data);
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
                    let setpoint = self
                        .base
                        .get_int32_param(self.amptek[AmptekReason::SetHighVoltage as usize], 0)?;
                    self.base.set_float64_param(
                        self.amptek[AmptekReason::HighVoltage as usize],
                        0,
                        f64::from(setpoint),
                    )?;
                }
                Ok(())
            }
            ReceivedPacket::Error { status } => {
                self.check_failed_comm();
                Err(protocol_error(format!(
                    "calling ReceiveData() for XMTPT_SEND_STATUS: {status:?}"
                )))
            }
            _ => Ok(()),
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

    /// `drvAmptek::connect` (`drvAmptek.cpp:161-183`), minus the `addr > 0`
    /// early return -- see the module doc.
    fn connect(&mut self, _user: &AsynUser) -> AsynResult<()> {
        if self.base.is_connected() {
            return Ok(());
        }
        match self.connect_device() {
            Ok(()) => {
                self.failed_sends = 0;
                self.base.set_connected(true);
                Ok(())
            }
            Err(e) => {
                self.target = None;
                Err(e)
            }
        }
    }

    /// `drvAmptek::disconnect` (`drvAmptek.cpp:185-198`).
    fn disconnect(&mut self, _user: &AsynUser) -> AsynResult<()> {
        self.do_disconnect();
        Ok(())
    }

    /// `drvAmptek::readOption` (`drvAmptek.cpp:200-219`): only `"hostInfo"`
    /// is recognized.
    fn get_option(&self, key: &str) -> AsynResult<String> {
        if key.eq_ignore_ascii_case("hostInfo") {
            Ok(self.address_info.clone())
        } else {
            Err(protocol_error(format!("Unsupported key \"{key}\"")))
        }
    }

    /// `drvAmptek::writeOption` (`drvAmptek.cpp:221-235`): only
    /// `"hostInfo"` is recognized (writing it disconnects); an empty key is
    /// a silent no-op, matching C's `epicsStrCaseCmp(key, "") != 0` guard.
    fn set_option(&mut self, user: &mut AsynUser, key: &str, value: &str) -> AsynResult<()> {
        if key.eq_ignore_ascii_case("hostInfo") {
            self.address_info = value.to_string();
            self.do_disconnect();
            Ok(())
        } else if !key.is_empty() {
            Err(protocol_error(format!("Unsupported key \"{key}\"")))
        } else {
            let _ = user;
            Ok(())
        }
    }

    /// `drvAmptek::writeInt32` (`drvAmptek.cpp:846-924`).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        if !self.base.is_connected() {
            return Err(disconnected_error());
        }
        let addr = user.addr;
        let reason = user.reason;
        let mut status = self.base.set_int32_param(reason, addr, value);

        if reason == self.mca[McaReason::StartAcquire as usize] {
            if !self.acquiring {
                status = self.send_command(TransmitPacketType::EnableMcaMcs);
                if status.is_ok() {
                    self.acquiring = true;
                    self.base
                        .set_int32_param(self.mca[McaReason::Acquiring as usize], 0, 1)?;
                }
            }
        } else if reason == self.mca[McaReason::StopAcquire as usize] {
            status = self.send_command(TransmitPacketType::DisableMcaMcs);
        } else if reason == self.mca[McaReason::Erase as usize] {
            status = self.send_command(TransmitPacketType::SendClearSpectrumStatus);
        } else if reason == self.mca[McaReason::ReadStatus as usize] {
            status = self.handle_read_status();
        } else if reason == self.amptek[AmptekReason::LoadConfigFile as usize] {
            let file_name = self
                .base
                .get_string_param(self.amptek[AmptekReason::ConfigFile as usize], 0)?
                .to_string();
            status = self.send_configuration_file(&file_name);
        } else if reason == self.amptek[AmptekReason::SaveConfigFile as usize] {
            let file_name = self
                .base
                .get_string_param(self.amptek[AmptekReason::ConfigFile as usize], 0)?
                .to_string();
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
                if self.acquiring && self.send_command(TransmitPacketType::DisableMcaMcs).is_ok() {
                    self.acquiring = false;
                }
            } else {
                self.acquiring = true;
            }
            Ok(i32::from(self.acquiring))
        } else {
            self.base.get_int32_param_strict(user.reason, user.addr)
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
            self.base.get_float64_param_strict(user.reason, user.addr)
        }
    }

    /// `drvAmptek::writeFloat64` (`drvAmptek.cpp:977-995`): every write
    /// unconditionally sends the full configuration, regardless of which
    /// param changed.
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        if !self.base.is_connected() {
            return Err(disconnected_error());
        }
        let addr = user.addr;
        self.base.set_float64_param(user.reason, addr, value)?;
        let status = self.send_configuration();
        self.base.call_param_callbacks(addr)?;
        status
    }

    /// `drvAmptek::readInt32Array` (`drvAmptek.cpp:997-1026`).
    fn read_int32_array(&mut self, _user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let target = self.target.ok_or_else(disconnected_error)?;
        self.send_command(TransmitPacketType::SendSpectrumStatus)?;

        match self.send_and_receive(target, TransmitPacketType::SendSpectrumStatus)? {
            ReceivedPacket::Spectrum { pid2, data } => {
                let (channels, values) = protocol::decode_spectrum(pid2, &data);
                let n = channels.min(buf.len()).min(values.len());
                buf[..n].copy_from_slice(&values[..n]);
                Ok(n)
            }
            ReceivedPacket::Error { status } => {
                self.check_failed_comm();
                Err(protocol_error(format!(
                    "error calling ReceiveData() for XMTPT_SEND_SPECTRUM_STATUS: {status:?}"
                )))
            }
            // `ReceiveData()==true` but not a spectrum packet: C would
            // reread its stale `pData_`-style buffer here (see the module
            // doc's "no cached-spectrum-buffer state" note); this port has
            // no such buffer to stale-read, so it reports zero channels
            // rather than fabricating spectrum content.
            _ => Ok(0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn new_rejects_non_ethernet_interface_types() {
        assert!(AmptekDriver::new("test1", 1, "192.168.0.1", false).is_err());
        assert!(AmptekDriver::new("test2", 2, "192.168.0.1", false).is_err());
        assert!(AmptekDriver::new("test3", 99, "192.168.0.1", false).is_err());
    }

    #[test]
    fn new_accepts_ethernet_interface_type_and_starts_disconnected() {
        let driver = AmptekDriver::new("test4", 0, "192.168.0.1", false).unwrap();
        assert!(!driver.base.is_connected());
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
