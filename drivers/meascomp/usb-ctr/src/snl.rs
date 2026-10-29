//! `USBCTR_SNL.st`, compiled by snc-rs from `build.rs` and run on
//! `epics-seq-rs`: reads the MCS spectra and the absolute time base when a
//! client asks (`{P}DoReadAll`) and when the hardware stops acquiring, then
//! releases the `{P}Acquiring` busy record `measCompMCS.template` sets.
//!
//! The escaped C is three `#include`s; what SNL calls from them is `atoi`.

use epics_libcom_rs::runtime::stdlib;
use epics_seq_rs::c::*;

macro_rules! seqg_escaped {
    (r#"#include <stdlib.h>"#) => {};
    (r#"#include <errlog.h>"#) => {};
    (r#"#include <string.h>"#) => {};
}

epics_seq_rs::include_program!("USBCTR_SNL");

/// `atoi`, for `atoi(macValueGet("NUM_COUNTERS"))`.
fn atoi(s: impl CSrc) -> i32 {
    s.with_c_bytes(|b| stdlib::atoi(b))
}

#[cfg(test)]
mod tests {
    use epics_seq_rs::{CharPtr, ProgOptions, VarModel};

    use super::*;

    /// USBCTR_SNL.st:1, with `option +r; option -c;` (:11, :15).
    #[test]
    fn the_program_is_the_st_one() {
        assert_eq!(USBCTR_SNL.validate(), Ok(()));
        assert_eq!(
            USBCTR_SNL.params,
            "P=USBCTR:MCS:, R=mca, NUM_COUNTERS=8,  FIELD=READ"
        );
        assert_eq!(USBCTR_SNL.options, ProgOptions::new(ProgOptions::NEWEF));
        assert!(matches!(USBCTR_SNL.vars, VarModel::Reentrant));
    }

    #[test]
    fn num_counters_is_glibc_atoi() {
        let value = CharPtr::from_bytes(b" +3x");
        assert_eq!(atoi(value), 3);
        assert_eq!(atoi("-2"), -2);
        use std::ffi::c_long;
        // strtol saturates at LONG_MIN/LONG_MAX, then truncates to int.
        assert_eq!(atoi("-99999999999999999999"), c_long::MIN as i32);
        assert_eq!(atoi("99999999999999999999"), c_long::MAX as i32);
        // 2^32 + 3 truncates to 3 where it fits a long.
        let wrapped = if c_long::BITS == 64 { 3 } else { i32::MAX };
        assert_eq!(atoi("4294967299"), wrapped);
    }
}
