//! PI (Physik Instrumente) legacy per-model motor controller drivers
//! (`motorPI`), distinct from the already-ported GCS2 generic stage
//! controller path (`motor-pi-gcs2`). Module-per-controller.

pub mod c630;
pub mod c662;
pub mod c663;
pub mod c844;
pub mod c848;
pub mod c862;
pub mod e516;
pub mod e517;
pub mod e710;
pub mod e816;
pub mod ioc;

pub use c630::{PIC630Axis, PIC630Controller};
pub use c662::{PIC662Axis, PIC662Controller};
pub use c663::{PIC663Axis, PIC663Controller};
pub use c844::{PIC844Axis, PIC844Controller};
pub use c848::{PIC848Axis, PIC848Controller};
pub use c862::{PIC862Axis, PIC862Controller};
pub use e516::{PIE516Axis, PIE516Controller};
pub use e517::{PIE517Axis, PIE517Controller};
pub use e710::{PIE710Axis, PIE710Controller};
pub use e816::{PIE816Axis, PIE816Controller};

use motor_common::util::{long_to_int, strtol};

/// C `sscanf(s, "%d", &out)` for the E-series piezo status reads: glibc's
/// `(int) strtol`, `None` when the field has no digits. The E-516/E-517/E-816
/// `set_status` chains gate every step on `recv_mess(...) && sscanf(buff,
/// "%d", &x)`, so a reply with no parseable integer must fail the read (drop
/// to the comm-debounce path) rather than silently reading `0` as `atoi`
/// would.
pub(crate) fn scan_int(s: &str) -> Option<i32> {
    match strtol(s.as_bytes(), 10) {
        (_, 0) => None,
        (value, _) => Some(long_to_int(value)),
    }
}
