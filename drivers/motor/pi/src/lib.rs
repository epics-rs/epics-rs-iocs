//! PI (Physik Instrumente) legacy per-model motor controller drivers
//! (`motorPI`), distinct from the already-ported GCS2 generic stage
//! controller path (`motor-pi-gcs2`). Module-per-controller.

pub mod c630;
pub mod c662;
pub mod c663;
pub mod c844;
pub mod c848;
pub mod c862;
pub mod ioc;

pub use c630::{PIC630Axis, PIC630Controller};
pub use c662::{PIC662Axis, PIC662Controller};
pub use c663::{PIC663Axis, PIC663Controller};
pub use c844::{PIC844Axis, PIC844Controller};
pub use c848::{PIC848Axis, PIC848Controller};
pub use c862::{PIC862Axis, PIC862Controller};

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
