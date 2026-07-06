//! PI (Physik Instrumente) legacy per-model motor controller drivers
//! (`motorPI`), distinct from the already-ported GCS2 generic stage
//! controller path (`motor-pi-gcs2`). Module-per-controller.

pub mod c663;
pub mod c862;
pub mod e516;
pub mod e517;
pub mod e710;
pub mod e816;
pub mod ioc;

pub use c663::{PIC663Axis, PIC663Controller};
pub use c862::{PIC862Axis, PIC862Controller};
pub use e516::{PIE516Axis, PIE516Controller};
pub use e517::{PIE517Axis, PIE517Controller};
pub use e710::{PIE710Axis, PIE710Controller};
pub use e816::{PIE816Axis, PIE816Controller};

/// C `sscanf(s, "%d", &out)` success semantics for the E-series piezo status
/// reads: `Some(int)` when a leading (optionally signed) decimal integer is
/// present, `None` when the field has no digits. The E-516/E-517/E-816
/// `set_status` chains gate every step on `recv_mess(...) && sscanf(buff,
/// "%d", &x)`, so a reply with no parseable integer must fail the read (drop
/// to the comm-debounce path) rather than silently reading `0` — which is why
/// this returns `Option` instead of reusing `util::atoi` (which yields `0` on
/// junk and cannot distinguish "parsed 0" from "no digits").
pub(crate) fn scan_int(s: &str) -> Option<i32> {
    let t = s.trim_start();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i == digits_start {
        return None; // no digit after the optional sign: C sscanf returns 0.
    }
    t.get(..i).and_then(|p| p.parse::<i32>().ok())
}

/// The `SA{accel},` wire fragment for a move, or an empty string when the
/// acceleration is not strictly positive.
///
/// C's record layer guards every ordinary move/home `SET_ACCEL` with
/// `if (accel > 0.0)` ("Don't send SET_ACCEL = 0.0", motorRecord.cc), so a
/// zero acceleration leaves the controller's configured acceleration
/// untouched rather than reprogramming it to 0. The value written is the
/// truncated integer, matching devPIC862/devPIC663 `sprintf("SA%d,", ...)`.
/// (Jog is the exception — it sends `SA` unconditionally — so it does not
/// use this helper.)
pub(crate) fn accel_field(acceleration: f64) -> String {
    if acceleration > 0.0 {
        format!("SA{},", acceleration as i32)
    } else {
        String::new()
    }
}

#[cfg(test)]
mod tests {
    use super::accel_field;

    #[test]
    fn accel_field_guards_zero_like_c() {
        // accel == 0.0 -> no SA fragment (C's `if (accel > 0.0)` guard).
        assert_eq!(accel_field(0.0), "");
        assert_eq!(accel_field(-1.0), "");
        // accel > 0.0 -> SA with the truncated integer value.
        assert_eq!(accel_field(500.0), "SA500,");
        // C guards on the f64 (> 0.0) then truncates: 0.4 passes the guard
        // and is sent as SA0, matching devPIC862's sprintf("SA%d,", (int)v).
        assert_eq!(accel_field(0.4), "SA0,");
    }
}
