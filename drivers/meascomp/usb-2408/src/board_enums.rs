//! C `MultiFunction`'s USB-2408-2AO enum tables (drvMultiFunction.cpp:
//! 349-367), `mapRange` (:1328-1376) and temperature scales. The range and
//! scale parameters hold UL for Windows codes, which every uldaq call maps.

use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::trace::TraceMask;

use crate::trace::{DRIVER_NAME, SelfTrace};

// cbw.h gain codes, as C defines them with a CBW_ prefix (:39-81).
pub const CBW_BIP60VOLTS: i32 = 20;
pub const CBW_BIP30VOLTS: i32 = 23;
pub const CBW_BIP20VOLTS: i32 = 15;
pub const CBW_BIP15VOLTS: i32 = 21;
pub const CBW_BIP10VOLTS: i32 = 1;
pub const CBW_BIP5VOLTS: i32 = 0;
pub const CBW_BIP4VOLTS: i32 = 16;
pub const CBW_BIP2PT5VOLTS: i32 = 2;
pub const CBW_BIP2VOLTS: i32 = 14;
pub const CBW_BIP1PT25VOLTS: i32 = 3;
pub const CBW_BIP1VOLTS: i32 = 4;
pub const CBW_BIPPT625VOLTS: i32 = 5;
pub const CBW_BIPPT5VOLTS: i32 = 6;
pub const CBW_BIPPT25VOLTS: i32 = 12;
pub const CBW_BIPPT2VOLTS: i32 = 13;
pub const CBW_BIPPT1VOLTS: i32 = 7;
pub const CBW_BIPPT05VOLTS: i32 = 8;
pub const CBW_BIPPT01VOLTS: i32 = 9;
pub const CBW_BIPPT005VOLTS: i32 = 10;
pub const CBW_BIPPT312VOLTS: i32 = 17;
pub const CBW_BIPPT156VOLTS: i32 = 18;
pub const CBW_BIPPT125VOLTS: i32 = 22;
pub const CBW_BIPPT078VOLTS: i32 = 19;
pub const CBW_UNI10VOLTS: i32 = 100;
pub const CBW_UNI5VOLTS: i32 = 101;
pub const CBW_UNI4VOLTS: i32 = 114;
pub const CBW_UNI2PT5VOLTS: i32 = 102;
pub const CBW_UNI2VOLTS: i32 = 103;
pub const CBW_UNI1PT25VOLTS: i32 = 104;
pub const CBW_UNI1VOLTS: i32 = 105;
pub const CBW_UNIPT5VOLTS: i32 = 110;
pub const CBW_UNIPT25VOLTS: i32 = 111;
pub const CBW_UNIPT2VOLTS: i32 = 112;
pub const CBW_UNIPT1VOLTS: i32 = 106;
pub const CBW_UNIPT05VOLTS: i32 = 113;
pub const CBW_UNIPT01VOLTS: i32 = 107;
pub const CBW_MA0TO20: i32 = 204;

/// C `AI_CHAN_TYPE_VOLTAGE`/`AI_CHAN_TYPE_TC`: uldaq's `AiChanType` on
/// Linux (:222-223), the ANALOG_IN_TYPE values.
pub const AI_CHAN_TYPE_VOLTAGE: i32 = uldaq_sys::AI_VOLTAGE as i32;
pub const AI_CHAN_TYPE_TC: i32 = uldaq_sys::AI_TC as i32;

// C's temperature scales (:228-232), the TEMPERATURE_SCALE values.
pub const CELSIUS: i32 = 0;
pub const FAHRENHEIT: i32 = 1;
pub const KELVIN: i32 = 2;
pub const VOLTS: i32 = 4;
pub const NOSCALE: i32 = 5;

/// `inputRangeUSB_2408`.
pub const INPUT_RANGE: &[(&str, i32)] = &[
    ("+= 10V", CBW_BIP10VOLTS),
    ("+= 5V", CBW_BIP5VOLTS),
    ("+= 2.5V", CBW_BIP2PT5VOLTS),
    ("+= 1.25V", CBW_BIP1PT25VOLTS),
    ("+= 0.625V", CBW_BIPPT625VOLTS),
    ("+= 0.312V", CBW_BIPPT312VOLTS),
    ("+= 0.156V", CBW_BIPPT156VOLTS),
    ("+= 0.078V", CBW_BIPPT078VOLTS),
];

/// `outputRangeUSB_2408`.
pub const OUTPUT_RANGE: &[(&str, i32)] = &[("+= 10V", CBW_BIP10VOLTS)];

/// `inputTypeUSB_2408`.
pub const INPUT_TYPE: &[(&str, i32)] = &[
    ("Volts", AI_CHAN_TYPE_VOLTAGE),
    ("TC deg.", AI_CHAN_TYPE_TC),
];

/// C `readEnum`'s copy of one table, every severity 0.
pub fn entries(table: &[(&str, i32)]) -> Vec<EnumEntry> {
    table
        .iter()
        .map(|&(string, value)| EnumEntry {
            string: string.to_owned(),
            value,
            severity: 0,
        })
        .collect()
}

/// C `mapRange`'s table: a cbw gain's uldaq `Range`.
fn ul_range(gain: i32) -> Option<i32> {
    use uldaq_sys::*;
    Some(match gain {
        CBW_BIP60VOLTS => BIP60VOLTS,
        CBW_BIP30VOLTS => BIP30VOLTS,
        CBW_BIP20VOLTS => BIP20VOLTS,
        CBW_BIP15VOLTS => BIP15VOLTS,
        CBW_BIP10VOLTS => BIP10VOLTS,
        CBW_BIP5VOLTS => BIP5VOLTS,
        CBW_BIP4VOLTS => BIP4VOLTS,
        CBW_BIP2PT5VOLTS => BIP2PT5VOLTS,
        CBW_BIP2VOLTS => BIP2VOLTS,
        CBW_BIP1PT25VOLTS => BIP1PT25VOLTS,
        CBW_BIP1VOLTS => BIP1VOLTS,
        CBW_BIPPT625VOLTS => BIPPT625VOLTS,
        CBW_BIPPT5VOLTS => BIPPT5VOLTS,
        CBW_BIPPT25VOLTS => BIPPT25VOLTS,
        CBW_BIPPT2VOLTS => BIPPT2VOLTS,
        CBW_BIPPT1VOLTS => BIPPT1VOLTS,
        CBW_BIPPT05VOLTS => BIPPT05VOLTS,
        CBW_BIPPT01VOLTS => BIPPT01VOLTS,
        CBW_BIPPT005VOLTS => BIPPT005VOLTS,
        CBW_BIPPT312VOLTS => BIPPT312VOLTS,
        CBW_BIPPT156VOLTS => BIPPT156VOLTS,
        CBW_BIPPT125VOLTS => BIPPT125VOLTS,
        CBW_BIPPT078VOLTS => BIPPT078VOLTS,
        CBW_UNI10VOLTS => UNI10VOLTS,
        CBW_UNI5VOLTS => UNI5VOLTS,
        CBW_UNI4VOLTS => UNI4VOLTS,
        CBW_UNI2PT5VOLTS => UNI2PT5VOLTS,
        CBW_UNI2VOLTS => UNI2VOLTS,
        CBW_UNI1PT25VOLTS => UNI1PT25VOLTS,
        CBW_UNI1VOLTS => UNI1VOLTS,
        CBW_UNIPT5VOLTS => UNIPT5VOLTS,
        CBW_UNIPT25VOLTS => UNIPT25VOLTS,
        CBW_UNIPT2VOLTS => UNIPT2VOLTS,
        CBW_UNIPT1VOLTS => UNIPT1VOLTS,
        CBW_UNIPT05VOLTS => UNIPT05VOLTS,
        CBW_UNIPT01VOLTS => UNIPT01VOLTS,
        CBW_MA0TO20 => MA0TO20,
        _ => return None,
    })
}

/// C `mapRange(gain, &range)` at a uldaq call site. C passes an unmapped
/// gain's uninitialised `Range` on; here it is -1, which uldaq refuses
/// with ERR_BAD_RANGE.
pub fn map_range(trace: &SelfTrace, gain: i32) -> i32 {
    ul_range(gain).unwrap_or_else(|| {
        trace.print(
            TraceMask::ERROR,
            format_args!("{DRIVER_NAME}::mapRange unsupported gain={gain}\n"),
        );
        -1
    })
}

/// C `pollerThread`'s switch (:2809-2819) from a TEMPERATURE_SCALE to the
/// `TempScale` ulTIn takes.
pub fn temp_scale(trace: &SelfTrace, scale: i32) -> i32 {
    match scale {
        CELSIUS => uldaq_sys::TS_CELSIUS,
        FAHRENHEIT => uldaq_sys::TS_FAHRENHEIT,
        KELVIN => uldaq_sys::TS_KELVIN,
        VOLTS => uldaq_sys::TS_VOLTS,
        NOSCALE => uldaq_sys::TS_NOSCALE,
        _ => {
            trace.print(
                TraceMask::ERROR,
                format_args!("{DRIVER_NAME}::pollerThread unsupported Scale={scale}\n"),
            );
            uldaq_sys::TS_CELSIUS
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_2408_range_maps_to_its_uldaq_range() {
        use uldaq_sys::*;
        let mapped: Vec<_> = INPUT_RANGE.iter().map(|&(_, g)| ul_range(g)).collect();
        assert_eq!(
            mapped,
            [
                BIP10VOLTS,
                BIP5VOLTS,
                BIP2PT5VOLTS,
                BIP1PT25VOLTS,
                BIPPT625VOLTS,
                BIPPT312VOLTS,
                BIPPT156VOLTS,
                BIPPT078VOLTS
            ]
            .map(Some)
        );
        assert_eq!(ul_range(OUTPUT_RANGE[0].1), Some(BIP10VOLTS));
        // C's commented-out CBW_BIP1PT67VOLTS and an unknown code.
        assert_eq!(ul_range(11), None);
        assert_eq!(ul_range(99), None);
    }

    #[test]
    fn a_temperature_scale_is_cs_code_and_3_falls_back_to_celsius() {
        use uldaq_sys::*;
        let trace = SelfTrace::default();
        let mapped = [0, 1, 2, 3, 4, 5].map(|s| temp_scale(&trace, s));
        assert_eq!(
            mapped,
            [
                TS_CELSIUS,
                TS_FAHRENHEIT,
                TS_KELVIN,
                TS_CELSIUS,
                TS_VOLTS,
                TS_NOSCALE
            ]
        );
    }
}
