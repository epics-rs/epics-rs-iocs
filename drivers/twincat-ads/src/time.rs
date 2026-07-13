//! PLC time (Windows FILETIME) → EPICS timestamp.
//!
//! TwinCAT stamps notifications and `MAIN.fbSystemTime` with a Windows
//! FILETIME: 100 ns ticks since 1601-01-01 UTC. asyn-rs carries parameter
//! timestamps as [`SystemTime`], so we land on the UNIX epoch and let the
//! framework do the EPICS-epoch shift.
//!
//! Upstream defect fixed at source: C `windowsToEpicsTimeStamp`
//! (adsAsynPortDriverUtils.cpp:560) computes the sub-second remainder as
//! `plcTime - (ts->secPastEpoch * WINDOWS_TICK_PER_SEC)`. `secPastEpoch` is
//! `uint32_t` and `WINDOWS_TICK_PER_SEC` is an `int`, so the product is
//! evaluated in 32-bit unsigned arithmetic and wraps: for any real timestamp
//! (`secPastEpoch` ≈ 1.1e9 in 2026) `1.1e9 * 1e7` overflows `uint32_t` by five
//! orders of magnitude, so the wrapped product is subtracted from the 64-bit
//! `plcTime` and `nsec` is garbage — routinely far outside [0, 1e9). Taking the
//! remainder with `%` keeps the whole computation in 64 bits and cannot
//! overflow.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// FILETIME ticks per second (100 ns each).
pub const WINDOWS_TICK_PER_SEC: u64 = 10_000_000;
/// Seconds between the Windows epoch (1601-01-01) and the UNIX epoch.
pub const SEC_TO_UNIX_EPOCH: u64 = 11_644_473_600;

/// Convert a Windows FILETIME to a [`SystemTime`].
///
/// Returns `None` for a FILETIME earlier than the UNIX epoch — including the
/// all-zero stamp a PLC sends before its clock is set, which must not be
/// mistaken for a valid 1601 timestamp.
pub fn windows_to_system_time(plc_time: u64) -> Option<SystemTime> {
    let unix_ticks = plc_time.checked_sub(SEC_TO_UNIX_EPOCH * WINDOWS_TICK_PER_SEC)?;
    let secs = unix_ticks / WINDOWS_TICK_PER_SEC;
    let nanos = (unix_ticks % WINDOWS_TICK_PER_SEC) * 100;
    Some(UNIX_EPOCH + Duration::new(secs, nanos as u32))
}

/// Rebuild a FILETIME from the PLC's two `MAIN.fbSystemTime` DWORDs.
///
/// The bulk read fetches `timeLoDW` and `timeHiDW` as separate sum-up entries
/// (adsAsynPortDriver.cpp:1366-1376); the FILETIME is the 64-bit value they
/// halve.
pub fn filetime_from_dwords(lo: u32, hi: u32) -> u64 {
    ((hi as u64) << 32) | (lo as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 2024-01-01T00:00:00Z — UNIX 1_704_067_200.
    const FILETIME_2024: u64 = (1_704_067_200 + SEC_TO_UNIX_EPOCH) * WINDOWS_TICK_PER_SEC;

    #[test]
    fn converts_whole_second() {
        let t = windows_to_system_time(FILETIME_2024).unwrap();
        let d = t.duration_since(UNIX_EPOCH).unwrap();
        assert_eq!(d.as_secs(), 1_704_067_200);
        assert_eq!(d.subsec_nanos(), 0);
    }

    #[test]
    fn sub_second_remainder_stays_in_range() {
        // The C 32-bit overflow bug lands here: for this magnitude of
        // secPastEpoch the wrapped product makes nsec meaningless. The
        // remainder must be a real sub-second value.
        for ticks in [1u64, 1234, 5_000_000, WINDOWS_TICK_PER_SEC - 1] {
            let t = windows_to_system_time(FILETIME_2024 + ticks).unwrap();
            let d = t.duration_since(UNIX_EPOCH).unwrap();
            assert_eq!(d.as_secs(), 1_704_067_200, "ticks={ticks}");
            assert_eq!(d.subsec_nanos(), (ticks * 100) as u32, "ticks={ticks}");
            assert!(d.subsec_nanos() < 1_000_000_000);
        }
    }

    #[test]
    fn hundred_ns_resolution_is_preserved() {
        let t = windows_to_system_time(FILETIME_2024 + 7).unwrap();
        assert_eq!(
            t.duration_since(UNIX_EPOCH).unwrap().subsec_nanos(),
            700,
            "one FILETIME tick is 100 ns"
        );
    }

    #[test]
    fn pre_unix_epoch_is_rejected() {
        assert!(windows_to_system_time(0).is_none(), "unset PLC clock");
        assert!(windows_to_system_time(SEC_TO_UNIX_EPOCH * WINDOWS_TICK_PER_SEC - 1).is_none());
        assert!(windows_to_system_time(SEC_TO_UNIX_EPOCH * WINDOWS_TICK_PER_SEC).is_some());
    }

    #[test]
    fn dwords_recombine_into_filetime() {
        assert_eq!(
            filetime_from_dwords(0x89AB_CDEF, 0x0123_4567),
            0x0123_4567_89AB_CDEF
        );
        assert_eq!(filetime_from_dwords(0, 0), 0);
        assert_eq!(filetime_from_dwords(u32::MAX, u32::MAX), u64::MAX);
    }

    #[test]
    fn far_future_filetime_does_not_overflow() {
        // Year 2100-ish; the 64-bit path must still produce a sane split.
        let ft = (4_102_444_800u64 + SEC_TO_UNIX_EPOCH) * WINDOWS_TICK_PER_SEC + 999_999;
        let d = windows_to_system_time(ft)
            .unwrap()
            .duration_since(UNIX_EPOCH)
            .unwrap();
        assert_eq!(d.as_secs(), 4_102_444_800);
        assert_eq!(d.subsec_nanos(), 99_999_900);
    }
}
