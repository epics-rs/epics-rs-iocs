//! ASCII configuration field formatting/parsing, ported from
//! `drvAmptek::sendConfiguration`/`sendSCAs`/`parseConfiguration`
//! (`drvAmptek.cpp:33-44,479-844`). Kept independent of asyn param
//! storage -- [`crate::driver`] reads/writes the asyn params into/out of
//! [`ConfigFields`]/[`ScaFields`], these functions only build/parse the
//! wire strings.

use std::fmt::Write as _;

use crate::status::DppType;

pub const CLOCK_STRINGS: &[&str] = &["AUTO", "20", "80"];
pub const POLARITY_STRINGS: &[&str] = &["POS", "NEG"];
pub const GATE_STRINGS: &[&str] = &["OFF", "HI", "LOW"];
pub const PUR_ENABLE_STRINGS: &[&str] = &["ON", "OFF", "MAX"];
pub const MCA_SOURCE_STRINGS: &[&str] = &["NORM", "MCS", "FAST", "PUR", "RTD"];
pub const FAST_PEAKING_TIME_STRINGS: &[&str] = &["50", "100", "200", "400", "800", "1600", "3200"];
pub const AUX_OUTPUT_STRINGS: &[&str] = &[
    "OFF", "ICR", "PILEUP", "MCSTB", "ONESH", "DETRES", "MCAEN", "PEAKH", "SCA8", "RTDOS",
    "RTDREJ", "VETO", "LIVE", "STREAM",
];
pub const CONNECT1_STRINGS: &[&str] = &["DAC", "AUXOUT1", "AUXIN1"];
pub const CONNECT2_STRINGS: &[&str] = &["AUXOUT2", "AUXIN2", "GATEH", "GATEL"];
pub const SCA_OUTPUT_WIDTH_STRINGS: &[&str] = &["100", "1000"];
pub const SCA_OUTPUT_LEVEL_STRINGS: &[&str] = &["OFF", "HIGH", "LOW"];

/// `drvAmptek.cpp:47`: `#define MAX_SCAS 8` -- "We only support the 8
/// SCAs that have hardware outputs."
pub const MAX_SCAS: usize = 8;

/// The fields `sendConfiguration` sends, in send order
/// (`drvAmptek.cpp:510-658`). Enum fields hold the index into their
/// choice array (the same value an mbbo/mbbi record's `VAL` holds).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfigFields {
    pub clock: u8,
    pub input_polarity: u8,
    pub peaking_time: f64,
    pub fast_peaking_time: u8,
    pub flat_top_time: f64,
    pub gain: f64,
    pub slow_threshold: f64,
    pub fast_threshold: f64,
    pub num_channels: i32,
    pub gate: u8,
    pub preset_real_time: f64,
    pub preset_live_time: f64,
    pub preset_counts: f64,
    pub preset_low_channel: i32,
    pub preset_high_channel: i32,
    pub mca_source: u8,
    pub pur_enable: u8,
    pub set_high_voltage: i32,
    pub set_det_temp: f64,
    pub mcs_low_channel: i32,
    pub mcs_high_channel: i32,
    pub dwell_time: f64,
    pub aux_out1: u8,
    pub aux_out2: u8,
    pub aux_out34: i32,
    pub connect1: u8,
    pub connect2: u8,
    pub sca_output_width: u8,
}

/// `drvAmptek::sendConfiguration` (`drvAmptek.cpp:510-658`): the 27
/// `KEY=value;` fields in exact send order. `%f` fields format with 6
/// decimal places (C's `sprintf("%f", ...)` default precision).
pub fn format_configuration(f: &ConfigFields) -> String {
    let mut s = String::new();
    let _ = write!(s, "CLCK={};", CLOCK_STRINGS[f.clock as usize]);
    let _ = write!(s, "AINP={};", POLARITY_STRINGS[f.input_polarity as usize]);
    let _ = write!(s, "TPEA={:.6};", f.peaking_time);
    let _ = write!(
        s,
        "TPFA={};",
        FAST_PEAKING_TIME_STRINGS[f.fast_peaking_time as usize]
    );
    let _ = write!(s, "TFLA={:.6};", f.flat_top_time);
    let _ = write!(s, "GAIN={:.6};", f.gain);
    let _ = write!(s, "THSL={:.6};", f.slow_threshold);
    let _ = write!(s, "THFA={:.6};", f.fast_threshold);
    let _ = write!(s, "MCAC={};", f.num_channels);
    let _ = write!(s, "GATE={};", GATE_STRINGS[f.gate as usize]);
    let _ = write!(s, "PRER={:.6};", f.preset_real_time);
    let _ = write!(s, "PRET={:.6};", f.preset_live_time);
    let _ = write!(s, "PREC={};", f.preset_counts as i32); // `(int)dtemp` truncation, drvAmptek.cpp:580
    let _ = write!(s, "PRCL={};", f.preset_low_channel);
    let _ = write!(s, "PRCH={};", f.preset_high_channel);
    let _ = write!(s, "MCAS={};", MCA_SOURCE_STRINGS[f.mca_source as usize]);
    let _ = write!(s, "PURE={};", PUR_ENABLE_STRINGS[f.pur_enable as usize]);
    let _ = write!(s, "HVSE={};", f.set_high_voltage);
    let _ = write!(s, "TECS={:.6};", f.set_det_temp);
    let _ = write!(s, "MCSL={};", f.mcs_low_channel);
    let _ = write!(s, "MCSH={};", f.mcs_high_channel);
    let _ = write!(s, "MCST={:.6};", f.dwell_time);
    let _ = write!(s, "AUO1={};", AUX_OUTPUT_STRINGS[f.aux_out1 as usize]);
    let _ = write!(s, "AUO2={};", AUX_OUTPUT_STRINGS[f.aux_out2 as usize]);
    let _ = write!(s, "AU34={};", f.aux_out34);
    let _ = write!(s, "CON1={};", CONNECT1_STRINGS[f.connect1 as usize]);
    let _ = write!(s, "CON2={};", CONNECT2_STRINGS[f.connect2 as usize]);
    let _ = write!(
        s,
        "SCAW={};",
        SCA_OUTPUT_WIDTH_STRINGS[f.sca_output_width as usize]
    );
    s
}

/// One SCA channel's fields (`sendSCAs`, `drvAmptek.cpp:479-508`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaFields {
    pub low_channel: i32,
    pub high_channel: i32,
    pub output_level: u8,
}

/// `drvAmptek::sendSCAs` (`drvAmptek.cpp:479-508`): one `SCAI=n;SCAL=..;
/// SCAH=..;SCAO=..;` group per SCA, 1-indexed (`i+1`), for all
/// [`MAX_SCAS`] channels.
pub fn format_scas(scas: &[ScaFields; MAX_SCAS]) -> String {
    let mut s = String::new();
    for (i, sca) in scas.iter().enumerate() {
        let _ = write!(s, "SCAI={};", i + 1);
        let _ = write!(s, "SCAL={};", sca.low_channel);
        let _ = write!(s, "SCAH={};", sca.high_channel);
        let _ = write!(
            s,
            "SCAO={};",
            SCA_OUTPUT_LEVEL_STRINGS[sca.output_level as usize]
        );
    }
    s
}

/// `drvAmptek::parseConfigDouble`/`parseConfigInt`/`parseConfigEnum`'s
/// shared lookup: find the first occurrence of `KEY=` anywhere in `cfg`
/// (`strstr`), not just at a field boundary.
fn find_value_after<'a>(cfg: &'a str, key: &str) -> Option<&'a str> {
    let pos = cfg.find(key)?;
    Some(&cfg[pos + key.len()..])
}

/// The leading `[+-]?[0-9]*\.?[0-9]*` prefix of `s` -- what C's
/// `sscanf(pos, "%lf", ...)`/`sscanf(pos, "%d", ...)` would consume for
/// every value `sendConfiguration` ever actually emits (`sprintf("%f",
/// ...)` never produces scientific notation, hex, or `inf`/`nan`, so
/// those `%f`/`%d` grammar corners are not reproduced).
fn leading_decimal(s: &str) -> &str {
    let bytes = s.as_bytes();
    let mut i = 0;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }
    let mut end = i;
    let mut seen_dot = false;
    while i < bytes.len() {
        match bytes[i] {
            b'0'..=b'9' => {
                i += 1;
                end = i;
            }
            b'.' if !seen_dot => {
                seen_dot = true;
                i += 1;
            }
            _ => break,
        }
    }
    &s[..end]
}

/// `drvAmptek::parseConfigDouble` (`drvAmptek.cpp:661-690`): `"OFF"` ->
/// `0.0`; else the leading decimal prefix; `None` if `key` isn't present
/// or nothing numeric follows it (C's two distinct `asynError` returns,
/// collapsed here since both leave the target param unchanged).
fn parse_config_double(cfg: &str, key: &str) -> Option<f64> {
    let rest = find_value_after(cfg, key)?;
    if rest.starts_with("OFF") {
        return Some(0.0);
    }
    let prefix = leading_decimal(rest);
    if prefix.is_empty() || prefix == "+" || prefix == "-" {
        None
    } else {
        prefix.parse().ok()
    }
}

/// `drvAmptek::parseConfigInt` (`drvAmptek.cpp:692-721`): as
/// [`parse_config_double`] but `%d` (no decimal point).
fn parse_config_int(cfg: &str, key: &str) -> Option<i32> {
    let rest = find_value_after(cfg, key)?;
    if rest.starts_with("OFF") {
        return Some(0);
    }
    let bytes = rest.as_bytes();
    let mut i = 0;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if i == digits_start {
        None
    } else {
        rest[..i].parse().ok()
    }
}

/// `drvAmptek::parseConfigEnum` (`drvAmptek.cpp:723-748`): the index of
/// the first `choices` entry that `rest` (the text right after `KEY=`)
/// starts with.
fn parse_config_enum(cfg: &str, key: &str, choices: &[&str]) -> Option<u8> {
    let rest = find_value_after(cfg, key)?;
    choices
        .iter()
        .position(|c| rest.starts_with(c))
        .map(|i| i as u8)
}

/// Each field of [`ConfigFields`] parsed independently, `None` where
/// `parseConfigDouble`/`Int`/`Enum` would have failed (or the field is
/// gated out for `dpp_type` -- `GATE`/`TECS`) or is intentionally never
/// read back (`AU34`, `SCAW` -- both sent by `sendConfiguration` but
/// absent from `drvAmptek::parseConfiguration`, `drvAmptek.cpp:750-844`,
/// the `AU34` omission is explained in C's own comment: "For some reason
/// the PX5 does not send AU34 in the configuration"). Left `None` fields
/// mean the corresponding asyn param is left unchanged, matching
/// `setDoubleParam`/`setIntegerParam` only being called on success.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ParsedConfig {
    pub clock: Option<u8>,
    pub input_polarity: Option<u8>,
    pub peaking_time: Option<f64>,
    pub fast_peaking_time: Option<u8>,
    pub flat_top_time: Option<f64>,
    pub gain: Option<f64>,
    pub slow_threshold: Option<f64>,
    pub fast_threshold: Option<f64>,
    pub num_channels: Option<i32>,
    pub gate: Option<u8>,
    pub preset_real_time: Option<f64>,
    pub preset_live_time: Option<f64>,
    pub preset_counts: Option<f64>,
    pub preset_low_channel: Option<i32>,
    pub preset_high_channel: Option<i32>,
    pub mca_source: Option<u8>,
    pub pur_enable: Option<u8>,
    pub set_high_voltage: Option<i32>,
    pub set_det_temp: Option<f64>,
    pub mcs_low_channel: Option<i32>,
    pub mcs_high_channel: Option<i32>,
    pub dwell_time: Option<f64>,
    pub aux_out1: Option<u8>,
    pub aux_out2: Option<u8>,
    pub connect1: Option<u8>,
    pub connect2: Option<u8>,
}

/// `drvAmptek::parseConfiguration` (`drvAmptek.cpp:750-844`).
pub fn parse_configuration(cfg: &str, dpp_type: DppType) -> ParsedConfig {
    ParsedConfig {
        clock: parse_config_enum(cfg, "CLCK=", CLOCK_STRINGS),
        input_polarity: parse_config_enum(cfg, "AINP=", POLARITY_STRINGS),
        peaking_time: parse_config_double(cfg, "TPEA="),
        fast_peaking_time: parse_config_enum(cfg, "TPFA=", FAST_PEAKING_TIME_STRINGS),
        flat_top_time: parse_config_double(cfg, "TFLA="),
        gain: parse_config_double(cfg, "GAIN="),
        slow_threshold: parse_config_double(cfg, "THSL="),
        fast_threshold: parse_config_double(cfg, "THFA="),
        num_channels: parse_config_int(cfg, "MCAC="),
        gate: if dpp_type == DppType::Dp5 || dpp_type == DppType::Mca8000D {
            parse_config_enum(cfg, "GATE=", GATE_STRINGS)
        } else {
            None
        },
        preset_real_time: parse_config_double(cfg, "PRER="),
        preset_live_time: parse_config_double(cfg, "PRET="),
        preset_counts: parse_config_double(cfg, "PREC="),
        preset_low_channel: parse_config_int(cfg, "PRCL="),
        preset_high_channel: parse_config_int(cfg, "PRCH="),
        mca_source: parse_config_enum(cfg, "MCAS=", MCA_SOURCE_STRINGS),
        pur_enable: parse_config_enum(cfg, "PURE=", PUR_ENABLE_STRINGS),
        set_high_voltage: parse_config_int(cfg, "HVSE="),
        set_det_temp: if dpp_type == DppType::Dp5 || dpp_type == DppType::Px5 {
            parse_config_double(cfg, "TECS=")
        } else {
            None
        },
        mcs_low_channel: parse_config_int(cfg, "MCSL="),
        mcs_high_channel: parse_config_int(cfg, "MCSH="),
        dwell_time: parse_config_double(cfg, "MCST="),
        aux_out1: parse_config_enum(cfg, "AUO1=", AUX_OUTPUT_STRINGS),
        aux_out2: parse_config_enum(cfg, "AUO2=", AUX_OUTPUT_STRINGS),
        connect1: parse_config_enum(cfg, "CON1=", CONNECT1_STRINGS),
        connect2: parse_config_enum(cfg, "CON2=", CONNECT2_STRINGS),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_fields() -> ConfigFields {
        ConfigFields {
            clock: 0,
            input_polarity: 1,
            peaking_time: 12.5,
            fast_peaking_time: 2,
            flat_top_time: 0.8,
            gain: 100.0,
            slow_threshold: 1.5,
            fast_threshold: 30.0,
            num_channels: 1024,
            gate: 1,
            preset_real_time: 0.0,
            preset_live_time: 0.0,
            preset_counts: 5000.0,
            preset_low_channel: 0,
            preset_high_channel: 1023,
            mca_source: 0,
            pur_enable: 1,
            set_high_voltage: 200,
            set_det_temp: -20.0,
            mcs_low_channel: 0,
            mcs_high_channel: 1023,
            dwell_time: 0.1,
            aux_out1: 6,
            aux_out2: 0,
            aux_out34: 0,
            connect1: 0,
            connect2: 0,
            sca_output_width: 0,
        }
    }

    #[test]
    fn format_configuration_matches_field_order_and_precision() {
        let s = format_configuration(&sample_fields());
        assert_eq!(
            s,
            "CLCK=AUTO;AINP=NEG;TPEA=12.500000;TPFA=200;TFLA=0.800000;GAIN=100.000000;THSL=1.500000;\
             THFA=30.000000;MCAC=1024;GATE=HI;PRER=0.000000;PRET=0.000000;PREC=5000;PRCL=0;PRCH=1023;\
             MCAS=NORM;PURE=OFF;HVSE=200;TECS=-20.000000;MCSL=0;MCSH=1023;MCST=0.100000;AUO1=MCAEN;\
             AUO2=OFF;AU34=0;CON1=DAC;CON2=AUXOUT2;SCAW=100;"
        );
    }

    #[test]
    fn format_scas_produces_one_group_per_channel_1_indexed() {
        let scas = [ScaFields {
            low_channel: 10,
            high_channel: 20,
            output_level: 2,
        }; MAX_SCAS];
        let s = format_scas(&scas);
        assert!(s.starts_with("SCAI=1;SCAL=10;SCAH=20;SCAO=LOW;"));
        assert!(s.contains("SCAI=8;SCAL=10;SCAH=20;SCAO=LOW;"));
        assert_eq!(s.matches("SCAI=").count(), MAX_SCAS);
    }

    #[test]
    fn parse_configuration_round_trips_through_format_for_dp5() {
        let fields = sample_fields();
        let cfg = format_configuration(&fields);
        let parsed = parse_configuration(&cfg, DppType::Dp5);
        assert_eq!(parsed.clock, Some(fields.clock));
        assert_eq!(parsed.input_polarity, Some(fields.input_polarity));
        assert_eq!(parsed.peaking_time, Some(fields.peaking_time));
        assert_eq!(parsed.fast_peaking_time, Some(fields.fast_peaking_time));
        assert_eq!(parsed.gate, Some(fields.gate)); // DP5: GATE is parsed
        assert_eq!(parsed.set_det_temp, Some(fields.set_det_temp)); // DP5: TECS is parsed
        assert_eq!(parsed.mca_source, Some(fields.mca_source));
        assert_eq!(parsed.connect2, Some(fields.connect2));
    }

    /// DP5G: neither GATE nor TECS are read back (`drvAmptek.cpp:785,814`).
    #[test]
    fn parse_configuration_gates_gate_and_tecs_by_device_type() {
        let cfg = format_configuration(&sample_fields());
        let parsed = parse_configuration(&cfg, DppType::Dp5G);
        assert_eq!(parsed.gate, None);
        assert_eq!(parsed.set_det_temp, None);
    }

    #[test]
    fn parse_config_double_off_maps_to_zero() {
        assert_eq!(parse_config_double("TECS=OFF;", "TECS="), Some(0.0));
    }

    #[test]
    fn parse_config_double_missing_key_is_none() {
        assert_eq!(parse_config_double("CLCK=AUTO;", "TPEA="), None);
    }

    #[test]
    fn parse_config_enum_matches_first_prefix() {
        assert_eq!(
            parse_config_enum("MCAS=FAST;", "MCAS=", MCA_SOURCE_STRINGS),
            Some(2)
        );
        assert_eq!(
            parse_config_enum("MCAS=NOPE;", "MCAS=", MCA_SOURCE_STRINGS),
            None
        );
    }

    #[test]
    fn parse_config_int_stops_at_non_digit() {
        assert_eq!(parse_config_int("MCAC=1024;NEXT=1;", "MCAC="), Some(1024));
    }
}
