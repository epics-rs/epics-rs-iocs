//! L0 config-port `PortDriver`: translates `capaNCDT6200.proto`'s 31
//! StreamDevice command blocks ([`crate::wire_config`]) into a native asyn
//! command table (no StreamDevice engine in epics-rs 0.22.1), plus 33 "sink"
//! reasons for the db records upstream's StreamDevice cross-writes directly
//! from another record's reply-parse (`queryVer`'s `version2M`,
//! `queryChanStatus`/`queryStatus`'s `chan{1-4}StatM`, `queryLinMode`'s
//! `chan{1-4}LinModeM`, `queryChan{1-4}Info`'s
//! `chan{N}{ANO,NAM,SNO,OFS,RNG,UNT}`). Ported per the approved option (B)
//! fan-out flattening: every upstream cross-write becomes an ordinary asyn
//! param this driver populates via [`PortDriverBase::set_int32_param`]/
//! [`PortDriverBase::set_string_param`] + `call_param_callbacks` when its
//! trigger command's reply arrives, rather than the trigger's own db record
//! ever holding that data.
//!
//! # mbbi/mbbo bind directly, no split needed
//! epics-rs 0.22.1's `asynInt32` device support is record-type-generic
//! (`asyn-rs::adapter::universal_asyn_factory` dispatches on DTYP + link
//! syntax, not record type), and mbbi/mbbo's own record support already
//! round-trips the ZRVL/ONVL/../FFVL-table raw code through RVAL on both
//! read and write (mbbo's `convert()` populates RVAL from the table before
//! the device write runs; mbbi's `apply_raw_readback` stores the device's
//! raw value into RVAL and derives VAL via reverse table lookup) -- the same
//! mechanism ai/ao/longin/longout use, not a Soft-Channel pass-through. Every
//! upstream mbbi/mbbo therefore binds directly (`DTYP="asynInt32"`,
//! `INP`/`OUT="@asyn($(Link),0) <ReasonName>"`), same as upstream's own
//! record types, with no synthetic ground-truth/display split in the ported
//! db.
//!
//! # `clearMathFunc`: one wire command, channel number carried in the write value
//! Upstream wires 4 separate `longout` records (`ch{1-4}ClearMathFuncC`),
//! each hardcoding its own channel number as that record's own `VAL`
//! (`capaNCDT6200.proto`'s `clearMathFunc` block has a single `%d`, filled
//! from whatever value the record processes with). Ported as a single
//! `ClearMathFunc` reason whose `write_int32` value IS the channel number --
//! matching [`wire_config::format_clear_math_func`]'s signature -- rather
//! than 4 separate reasons.
//!
//! # `queryChanStatus`/`queryStatus`: genuine duplicate fan-out targets
//! Both wire commands (`$CHS` and `$STS`, two distinct protocol blocks) fan
//! out into the exact same four `Chan{1-4}Stat` sinks. Not a typo upstream;
//! reproduced as two distinct [`Command`] variants that both write the same
//! four sink reasons.

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::wire_config as wire;

/// asyn reason used for octet transactions against the underlying serial/IP
/// port (mirrors `love`/`syringepump`/`delaygen`'s `OCTET_REASON`/transport
/// convention in this workspace).
const TRANSPORT_REASON: usize = 0;

/// Reply buffer size for the underlying transport read -- generous enough
/// for the longest expected reply (`queryChan{1-4}Info`'s 6-field line, or
/// `measDataM`'s up-to-250-byte welcome-text capture).
const REPLY_BUF_SIZE: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    GrabWelcomeText,
    QueryVer,
    QuerySampleTime,
    SetSampleTime,
    QueryTrigMode,
    SetTrigMode,
    QueryAvgTypeMode,
    SetAvgTypeMode,
    QueryAvgNumMode,
    SetAvgNumMode,
    QueryChanStatus,
    QueryChan1Info,
    QueryChan2Info,
    QueryChan3Info,
    QueryChan4Info,
    QueryLinMode,
    SetCh1LinearMode,
    SetCh2LinearMode,
    SetCh3LinearMode,
    SetCh4LinearMode,
    ClearMathFunc,
    QueryStatus,
    QueryDataPort,
    SetDataPort,
    QueryMeasData,
    SetCh1LinearPoint,
    SetCh2LinearPoint,
    SetCh3LinearPoint,
    SetCh4LinearPoint,
    QueryAnalogFilter,
    SetAnalogFilter,
}

/// Command name (the `drvInfo` string a db link's `@asyn(port,0)
/// CommandName` supplies), matched case-insensitively. Position in this
/// table *is* the asyn reason, 0-based (mirrors `drivers/syringepump`'s
/// `COMMAND_NAMES` convention) -- one entry per `capaNCDT6200.proto` block.
const TRIGGER_NAMES: &[(&str, Command)] = &[
    ("GrabWelcomeText", Command::GrabWelcomeText),
    ("QueryVer", Command::QueryVer),
    ("QuerySampleTime", Command::QuerySampleTime),
    ("SetSampleTime", Command::SetSampleTime),
    ("QueryTrigMode", Command::QueryTrigMode),
    ("SetTrigMode", Command::SetTrigMode),
    ("QueryAvgTypeMode", Command::QueryAvgTypeMode),
    ("SetAvgTypeMode", Command::SetAvgTypeMode),
    ("QueryAvgNumMode", Command::QueryAvgNumMode),
    ("SetAvgNumMode", Command::SetAvgNumMode),
    ("QueryChanStatus", Command::QueryChanStatus),
    ("QueryChan1Info", Command::QueryChan1Info),
    ("QueryChan2Info", Command::QueryChan2Info),
    ("QueryChan3Info", Command::QueryChan3Info),
    ("QueryChan4Info", Command::QueryChan4Info),
    ("QueryLinMode", Command::QueryLinMode),
    ("SetCh1LinearMode", Command::SetCh1LinearMode),
    ("SetCh2LinearMode", Command::SetCh2LinearMode),
    ("SetCh3LinearMode", Command::SetCh3LinearMode),
    ("SetCh4LinearMode", Command::SetCh4LinearMode),
    ("ClearMathFunc", Command::ClearMathFunc),
    ("QueryStatus", Command::QueryStatus),
    ("QueryDataPort", Command::QueryDataPort),
    ("SetDataPort", Command::SetDataPort),
    ("QueryMeasData", Command::QueryMeasData),
    ("SetCh1LinearPoint", Command::SetCh1LinearPoint),
    ("SetCh2LinearPoint", Command::SetCh2LinearPoint),
    ("SetCh3LinearPoint", Command::SetCh3LinearPoint),
    ("SetCh4LinearPoint", Command::SetCh4LinearPoint),
    ("QueryAnalogFilter", Command::QueryAnalogFilter),
    ("SetAnalogFilter", Command::SetAnalogFilter),
];

const N_TRIGGERS: usize = TRIGGER_NAMES.len();

/// Sink reasons: populated only by a trigger command's fan-out dispatch, no
/// wire round trip of their own. Discriminants are sequential from 0 so
/// `sink_reason` can compute the asyn reason directly (`N_TRIGGERS + sink as
/// usize`) -- kept in lockstep with [`SINK_NAMES`] by the
/// `sink_names_match_enum_order` unit test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sink {
    Version2,
    Chan1Stat,
    Chan2Stat,
    Chan3Stat,
    Chan4Stat,
    Chan1LinMode,
    Chan2LinMode,
    Chan3LinMode,
    Chan4LinMode,
    Chan1Ano,
    Chan1Nam,
    Chan1Sno,
    Chan1Ofs,
    Chan1Rng,
    Chan1Unt,
    Chan2Ano,
    Chan2Nam,
    Chan2Sno,
    Chan2Ofs,
    Chan2Rng,
    Chan2Unt,
    Chan3Ano,
    Chan3Nam,
    Chan3Sno,
    Chan3Ofs,
    Chan3Rng,
    Chan3Unt,
    Chan4Ano,
    Chan4Nam,
    Chan4Sno,
    Chan4Ofs,
    Chan4Rng,
    Chan4Unt,
}

/// Order MUST match [`Sink`]'s declaration order (see that enum's doc).
const SINK_NAMES: &[(&str, Sink)] = &[
    ("Version2", Sink::Version2),
    ("Chan1Stat", Sink::Chan1Stat),
    ("Chan2Stat", Sink::Chan2Stat),
    ("Chan3Stat", Sink::Chan3Stat),
    ("Chan4Stat", Sink::Chan4Stat),
    ("Chan1LinMode", Sink::Chan1LinMode),
    ("Chan2LinMode", Sink::Chan2LinMode),
    ("Chan3LinMode", Sink::Chan3LinMode),
    ("Chan4LinMode", Sink::Chan4LinMode),
    ("Chan1Ano", Sink::Chan1Ano),
    ("Chan1Nam", Sink::Chan1Nam),
    ("Chan1Sno", Sink::Chan1Sno),
    ("Chan1Ofs", Sink::Chan1Ofs),
    ("Chan1Rng", Sink::Chan1Rng),
    ("Chan1Unt", Sink::Chan1Unt),
    ("Chan2Ano", Sink::Chan2Ano),
    ("Chan2Nam", Sink::Chan2Nam),
    ("Chan2Sno", Sink::Chan2Sno),
    ("Chan2Ofs", Sink::Chan2Ofs),
    ("Chan2Rng", Sink::Chan2Rng),
    ("Chan2Unt", Sink::Chan2Unt),
    ("Chan3Ano", Sink::Chan3Ano),
    ("Chan3Nam", Sink::Chan3Nam),
    ("Chan3Sno", Sink::Chan3Sno),
    ("Chan3Ofs", Sink::Chan3Ofs),
    ("Chan3Rng", Sink::Chan3Rng),
    ("Chan3Unt", Sink::Chan3Unt),
    ("Chan4Ano", Sink::Chan4Ano),
    ("Chan4Nam", Sink::Chan4Nam),
    ("Chan4Sno", Sink::Chan4Sno),
    ("Chan4Ofs", Sink::Chan4Ofs),
    ("Chan4Rng", Sink::Chan4Rng),
    ("Chan4Unt", Sink::Chan4Unt),
];

fn sink_reason(sink: Sink) -> usize {
    N_TRIGGERS + sink as usize
}

fn sink_param_type(sink: Sink) -> ParamType {
    match sink {
        Sink::Version2
        | Sink::Chan1Nam
        | Sink::Chan1Unt
        | Sink::Chan2Nam
        | Sink::Chan2Unt
        | Sink::Chan3Nam
        | Sink::Chan3Unt
        | Sink::Chan4Nam
        | Sink::Chan4Unt => ParamType::Octet,
        _ => ParamType::Int32,
    }
}

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// microEpsilon capaNCDT6200 L0 config-port driver state.
pub struct ConfigDriver {
    base: PortDriverBase,
    handle: SyncIOHandle,
}

impl ConfigDriver {
    pub fn new(port_name: &str, handle: SyncIOHandle) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        for (name, sink) in SINK_NAMES {
            base.create_param(name, sink_param_type(*sink))?;
        }
        Ok(Self { base, handle })
    }

    fn command(&self, reason: usize) -> AsynResult<Command> {
        TRIGGER_NAMES
            .get(reason)
            .map(|(_, c)| *c)
            .ok_or_else(|| protocol_error("invalid reason"))
    }

    fn write_frame(&self, out: &str) -> AsynResult<()> {
        self.handle.write_octet(TRANSPORT_REASON, out.as_bytes())?;
        Ok(())
    }

    fn execute(&self, out: &str) -> AsynResult<Vec<u8>> {
        self.write_frame(out)?;
        self.handle.read_octet(TRANSPORT_REASON, REPLY_BUF_SIZE)
    }

    fn wire_err(e: wire::ParseError) -> AsynError {
        protocol_error(format!("{e:?}"))
    }

    fn set_sink_int32(&mut self, sink: Sink, value: i32) -> AsynResult<()> {
        self.base.set_int32_param(sink_reason(sink), 0, value)
    }

    fn set_sink_string(&mut self, sink: Sink, value: String) -> AsynResult<()> {
        self.base.set_string_param(sink_reason(sink), 0, value)
    }

    /// `queryChanStatus`/`queryStatus` share this fan-out (see module doc).
    fn fan_out_chan_status(&mut self, values: [i32; 4]) -> AsynResult<()> {
        self.set_sink_int32(Sink::Chan1Stat, values[0])?;
        self.set_sink_int32(Sink::Chan2Stat, values[1])?;
        self.set_sink_int32(Sink::Chan3Stat, values[2])?;
        self.set_sink_int32(Sink::Chan4Stat, values[3])?;
        self.base.call_param_callbacks(0)
    }

    fn fan_out_lin_mode(&mut self, values: [i32; 4]) -> AsynResult<()> {
        self.set_sink_int32(Sink::Chan1LinMode, values[0])?;
        self.set_sink_int32(Sink::Chan2LinMode, values[1])?;
        self.set_sink_int32(Sink::Chan3LinMode, values[2])?;
        self.set_sink_int32(Sink::Chan4LinMode, values[3])?;
        self.base.call_param_callbacks(0)
    }

    fn fan_out_chan_info(&mut self, channel: u8, info: wire::ChanInfo) -> AsynResult<()> {
        let (ano, nam, sno, ofs, rng, unt) = match channel {
            1 => (
                Sink::Chan1Ano,
                Sink::Chan1Nam,
                Sink::Chan1Sno,
                Sink::Chan1Ofs,
                Sink::Chan1Rng,
                Sink::Chan1Unt,
            ),
            2 => (
                Sink::Chan2Ano,
                Sink::Chan2Nam,
                Sink::Chan2Sno,
                Sink::Chan2Ofs,
                Sink::Chan2Rng,
                Sink::Chan2Unt,
            ),
            3 => (
                Sink::Chan3Ano,
                Sink::Chan3Nam,
                Sink::Chan3Sno,
                Sink::Chan3Ofs,
                Sink::Chan3Rng,
                Sink::Chan3Unt,
            ),
            4 => (
                Sink::Chan4Ano,
                Sink::Chan4Nam,
                Sink::Chan4Sno,
                Sink::Chan4Ofs,
                Sink::Chan4Rng,
                Sink::Chan4Unt,
            ),
            _ => return Err(protocol_error(format!("invalid channel {channel}"))),
        };
        self.set_sink_int32(ano, info.article_number)?;
        self.set_sink_string(nam, info.name)?;
        self.set_sink_int32(sno, info.serial_number)?;
        self.set_sink_int32(ofs, info.offset)?;
        self.set_sink_int32(rng, info.range)?;
        self.set_sink_string(unt, info.unit)?;
        self.base.call_param_callbacks(0)
    }
}

impl PortDriver for ConfigDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        let (drv_info, addr) = (req.drv_info.as_str(), req.addr);
        if addr != 0 {
            return Err(AsynError::AddressOutOfRange(addr));
        }
        for (i, (name, _)) in TRIGGER_NAMES.iter().enumerate() {
            if name.eq_ignore_ascii_case(drv_info) {
                return Ok(DrvUserInfo::from_reason(i));
            }
        }
        for (name, sink) in SINK_NAMES {
            if name.eq_ignore_ascii_case(drv_info) {
                return Ok(DrvUserInfo::from_reason(sink_reason(*sink)));
            }
        }
        Err(protocol_error(format!(
            "failure to find command {drv_info}"
        )))
    }

    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let Ok(cmd) = self.command(user.reason) else {
            // Not a trigger reason -- fall through to the default
            // params-cache read for a sink reason.
            return self.base.get_int32_param(user.reason, user.addr);
        };
        match cmd {
            Command::QuerySampleTime => {
                let reply = self.execute(&wire::format_query_sample_time())?;
                wire::parse_query_sample_time(&reply).map_err(Self::wire_err)
            }
            Command::QueryTrigMode => {
                let reply = self.execute(&wire::format_query_trig_mode())?;
                wire::parse_query_trig_mode(&reply).map_err(Self::wire_err)
            }
            Command::QueryAvgTypeMode => {
                let reply = self.execute(&wire::format_query_avg_type_mode())?;
                wire::parse_query_avg_type_mode(&reply).map_err(Self::wire_err)
            }
            Command::QueryAvgNumMode => {
                let reply = self.execute(&wire::format_query_avg_num_mode())?;
                wire::parse_query_avg_num_mode(&reply).map_err(Self::wire_err)
            }
            Command::QueryDataPort => {
                let reply = self.execute(&wire::format_query_data_port())?;
                wire::parse_query_data_port(&reply).map_err(Self::wire_err)
            }
            Command::QueryMeasData => {
                let reply = self.execute(&wire::format_query_meas_data())?;
                wire::parse_query_meas_data(&reply).map_err(Self::wire_err)
            }
            Command::QueryAnalogFilter => {
                let reply = self.execute(&wire::format_query_analog_filter())?;
                wire::parse_query_analog_filter(&reply).map_err(Self::wire_err)
            }
            _ => Err(protocol_error(format!(
                "{cmd:?} does not support Int32 read"
            ))),
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let cmd = self.command(user.reason)?;
        match cmd {
            Command::SetSampleTime => {
                let reply = self.execute(&wire::format_set_sample_time(value))?;
                wire::parse_set_sample_time_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetTrigMode => {
                let reply = self.execute(&wire::format_set_trig_mode(value))?;
                wire::parse_set_trig_mode_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetAvgTypeMode => {
                let reply = self.execute(&wire::format_set_avg_type_mode(value))?;
                wire::parse_set_avg_type_mode_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetAvgNumMode => {
                let reply = self.execute(&wire::format_set_avg_num_mode(value))?;
                wire::parse_set_avg_num_mode_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetCh1LinearMode
            | Command::SetCh2LinearMode
            | Command::SetCh3LinearMode
            | Command::SetCh4LinearMode => {
                let channel = match cmd {
                    Command::SetCh1LinearMode => 1,
                    Command::SetCh2LinearMode => 2,
                    Command::SetCh3LinearMode => 3,
                    _ => 4,
                };
                let reply = self.execute(&wire::format_set_linear_mode(channel, value))?;
                wire::parse_set_linear_mode_ack(&reply, channel)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::ClearMathFunc => {
                let reply = self.execute(&wire::format_clear_math_func(value))?;
                wire::parse_clear_math_func_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetDataPort => {
                let reply = self.execute(&wire::format_set_data_port(value))?;
                wire::parse_set_data_port_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetCh1LinearPoint
            | Command::SetCh2LinearPoint
            | Command::SetCh3LinearPoint
            | Command::SetCh4LinearPoint => {
                let channel = match cmd {
                    Command::SetCh1LinearPoint => 1,
                    Command::SetCh2LinearPoint => 2,
                    Command::SetCh3LinearPoint => 3,
                    _ => 4,
                };
                let reply = self.execute(&wire::format_set_linear_point(channel, value))?;
                wire::parse_set_linear_point_ack(&reply, channel)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            Command::SetAnalogFilter => {
                let reply = self.execute(&wire::format_set_analog_filter(value))?;
                wire::parse_set_analog_filter_ack(&reply)
                    .map_err(Self::wire_err)
                    .map(|_| ())
            }
            _ => Err(protocol_error(format!(
                "{cmd:?} does not support Int32 write"
            ))),
        }
    }

    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let text = if let Ok(cmd) = self.command(user.reason) {
            match cmd {
                Command::GrabWelcomeText => {
                    // No `out` clause upstream -- a pure "read whatever line
                    // is waiting" primitive, no write first.
                    let reply = self.handle.read_octet(TRANSPORT_REASON, REPLY_BUF_SIZE)?;
                    wire::parse_welcome_text(&reply).map_err(Self::wire_err)?
                }
                Command::QueryVer => {
                    let reply = self.execute(&wire::format_query_ver())?;
                    let (v1, v2) = wire::parse_query_ver(&reply).map_err(Self::wire_err)?;
                    self.set_sink_string(Sink::Version2, v2)?;
                    self.base.call_param_callbacks(0)?;
                    v1
                }
                _ => {
                    return Err(protocol_error(format!(
                        "{cmd:?} does not support Octet read"
                    )));
                }
            }
        } else {
            self.base
                .get_string_param(user.reason, user.addr)?
                .to_string()
        };
        let bytes = text.as_bytes();
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let cmd = self.command(user.reason)?;
        // Every fan-out trigger below has no `%s`/`%d` substitution from the
        // record's own written value upstream (`out "$CHS";` etc, a fixed
        // literal) -- `data` is intentionally ignored; writing ANY value to
        // the trigger record is what fires the wire round trip.
        match cmd {
            Command::QueryChanStatus => {
                let reply = self.execute(&wire::format_query_chan_status())?;
                let values = wire::parse_chan_status(&reply).map_err(Self::wire_err)?;
                self.fan_out_chan_status(values)?;
            }
            Command::QueryStatus => {
                let reply = self.execute(&wire::format_query_status())?;
                let values = wire::parse_status(&reply).map_err(Self::wire_err)?;
                self.fan_out_chan_status(values)?;
            }
            Command::QueryLinMode => {
                let reply = self.execute(&wire::format_query_lin_mode())?;
                let values = wire::parse_lin_mode(&reply).map_err(Self::wire_err)?;
                self.fan_out_lin_mode(values)?;
            }
            Command::QueryChan1Info
            | Command::QueryChan2Info
            | Command::QueryChan3Info
            | Command::QueryChan4Info => {
                let channel = match cmd {
                    Command::QueryChan1Info => 1,
                    Command::QueryChan2Info => 2,
                    Command::QueryChan3Info => 3,
                    _ => 4,
                };
                let reply = self.execute(&wire::format_query_chan_info(channel))?;
                let info = wire::parse_chan_info(&reply, channel).map_err(Self::wire_err)?;
                self.fan_out_chan_info(channel, info)?;
            }
            _ => {
                return Err(protocol_error(format!(
                    "{cmd:?} does not support Octet write"
                )));
            }
        }
        Ok(data.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trigger_names_are_unique_and_match_case_insensitively() {
        let mut seen = std::collections::HashSet::new();
        for (name, _) in TRIGGER_NAMES {
            assert!(seen.insert(name.to_ascii_lowercase()), "duplicate: {name}");
        }
        assert!(
            TRIGGER_NAMES
                .iter()
                .any(|(n, _)| n.eq_ignore_ascii_case("querysampletime"))
        );
    }

    #[test]
    fn sink_names_are_unique_and_match_case_insensitively() {
        let mut seen = std::collections::HashSet::new();
        for (name, _) in SINK_NAMES {
            assert!(seen.insert(name.to_ascii_lowercase()), "duplicate: {name}");
        }
    }

    #[test]
    fn sink_names_match_enum_order() {
        for (i, (_, sink)) in SINK_NAMES.iter().enumerate() {
            assert_eq!(
                *sink as usize, i,
                "Sink enum declaration order must match SINK_NAMES order"
            );
        }
    }

    #[test]
    fn trigger_and_sink_reasons_do_not_overlap() {
        assert_eq!(sink_reason(Sink::Version2), N_TRIGGERS);
        assert_eq!(
            sink_reason(Sink::Chan4Unt),
            N_TRIGGERS + SINK_NAMES.len() - 1
        );
    }

    #[test]
    fn clear_math_func_is_a_single_reason_not_four() {
        let count = TRIGGER_NAMES
            .iter()
            .filter(|(_, c)| *c == Command::ClearMathFunc)
            .count();
        assert_eq!(count, 1);
    }
}
