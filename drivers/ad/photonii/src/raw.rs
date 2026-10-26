//! Reading the `.raw` frame files p2util writes (C `PhotonII::readRaw`).
//!
//! p2util writes the frame to disk and tells the driver where; the driver
//! polls for that file to appear with the expected size and a modification
//! time at or after the start of the acquisition, then reads it.

use std::fs;
use std::io::Read;
use std::path::Path;
use std::time::{Duration, SystemTime};

use crate::types::{PII_PIXEL_BYTES, PII_SIZE_X, PII_SIZE_Y};

/// Why a `.raw` file could not be turned into a frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RawError {
    /// The file holds a different number of bytes than the detector geometry
    /// calls for.
    WrongSize { expected: usize, got: usize },
    /// Opening or reading the file failed.
    Io(String),
}

impl std::fmt::Display for RawError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WrongSize { expected, got } => {
                write!(f, "raw file holds {got} bytes, expected {expected}")
            }
            Self::Io(e) => f.write_str(e),
        }
    }
}

impl std::error::Error for RawError {}

/// Bytes one full frame occupies on disk.
pub fn frame_bytes() -> usize {
    PII_SIZE_X * PII_SIZE_Y * PII_PIXEL_BYTES
}

/// Read a `.raw` file straight into a frame's 32-bit signed pixels, as C
/// `fread`s it into `pImage->pData` (PhotonII.cpp:209).
///
/// C read the file straight into an `NDArray` allocated with `ADSizeX` ×
/// `ADSizeY` while sizing the read from the *detector* geometry, so shrinking
/// `SizeX` overran the heap buffer. The driver never sends a region of interest
/// to p2util — every frame it writes is a full sensor readout — so the size is
/// fixed here and a file of any other length is refused instead of copied.
pub fn read_raw(path: &Path, pixels: &mut [i32]) -> Result<(), RawError> {
    let io = |e: std::io::Error| RawError::Io(e.to_string());
    let mut file = fs::File::open(path).map_err(io)?;
    let expected = frame_bytes();
    let got = file.metadata().map_err(io)?.len() as usize;
    if got != expected {
        return Err(RawError::WrongSize { expected, got });
    }
    // One pass from the file into the pixels, through a stack chunk.
    let mut chunk = [0u8; 16 * 1024];
    for out in pixels.chunks_mut(chunk.len() / PII_PIXEL_BYTES) {
        let bytes = &mut chunk[..out.len() * PII_PIXEL_BYTES];
        file.read_exact(bytes).map_err(io)?;
        for (p, c) in out.iter_mut().zip(bytes.as_chunks::<PII_PIXEL_BYTES>().0) {
            *p = i32::from_le_bytes(*c);
        }
    }
    Ok(())
}

/// What a poll of the frame file found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileState {
    /// Not there yet (or not readable yet).
    Missing,
    /// There, but written before this acquisition started — a leftover.
    Stale,
    /// There and new, but not yet the full frame; p2util is still writing.
    Incomplete { got: u64 },
    /// There, new, and complete.
    Ready,
}

/// One poll of the frame file, with C's two acceptance tests: the size must be
/// exactly one frame and the modification time must not predate the start of
/// the acquisition.
///
/// `not_before` is compared at whole-second resolution, as C's
/// `difftime(st_mtime, acqStartTime)` was.
pub fn check_raw_file(path: &Path, not_before: SystemTime, expected: usize) -> FileState {
    let Ok(meta) = fs::metadata(path) else {
        return FileState::Missing;
    };
    let Ok(mtime) = meta.modified() else {
        return FileState::Missing;
    };
    if !at_or_after_second(mtime, not_before) {
        return FileState::Stale;
    }
    let size = meta.len();
    if size != expected as u64 {
        return FileState::Incomplete { got: size };
    }
    FileState::Ready
}

/// `mtime >= floor(not_before)`, in whole seconds — file modification times
/// have one-second resolution on many file systems, so a frame written in the
/// same second the acquisition started must count as new.
fn at_or_after_second(mtime: SystemTime, not_before: SystemTime) -> bool {
    let floor = |t: SystemTime| {
        t.duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or(Duration::ZERO)
            .as_secs()
    };
    floor(mtime) >= floor(not_before)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    fn scratch(name: &str) -> std::path::PathBuf {
        static N: AtomicU32 = AtomicU32::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let mut p = std::env::temp_dir();
        p.push(format!("photonii-test-{}-{n}-{name}", std::process::id()));
        p
    }

    /// `read_raw` of `bytes` written to a scratch file, into stale pixels.
    fn read(name: &str, bytes: &[u8]) -> Result<Vec<i32>, RawError> {
        let p = scratch(name);
        fs::write(&p, bytes).unwrap();
        let mut pixels = vec![0x5a5a_5a5a; PII_SIZE_X * PII_SIZE_Y];
        let read = read_raw(&p, &mut pixels).map(|()| pixels);
        fs::remove_file(&p).ok();
        read
    }

    #[test]
    fn read_raw_reads_little_endian_int32_pixels() {
        let mut bytes = vec![0u8; frame_bytes()];
        bytes[0..4].copy_from_slice(&1_000_000i32.to_le_bytes());
        bytes[4..8].copy_from_slice(&(-7i32).to_le_bytes());
        let last = frame_bytes() - 4;
        bytes[last..].copy_from_slice(&42i32.to_le_bytes());

        let pixels = read("le.raw", &bytes).unwrap();
        assert_eq!(pixels[0], 1_000_000);
        assert_eq!(pixels[1], -7);
        assert!(pixels[2..pixels.len() - 1].iter().all(|&p| p == 0));
        assert_eq!(pixels[pixels.len() - 1], 42);
    }

    #[test]
    fn read_raw_rejects_a_short_file() {
        assert!(matches!(
            read("short.raw", &vec![0u8; frame_bytes() - 4]),
            Err(RawError::WrongSize { expected, got })
                if expected == frame_bytes() && got == frame_bytes() - 4
        ));
    }

    #[test]
    fn read_raw_rejects_a_long_file() {
        assert!(matches!(
            read("long.raw", &vec![0u8; frame_bytes() + 1]),
            Err(RawError::WrongSize { expected, got })
                if expected == frame_bytes() && got == frame_bytes() + 1
        ));
    }

    #[test]
    fn read_raw_reports_a_missing_file() {
        let mut pixels = vec![0; PII_SIZE_X * PII_SIZE_Y];
        assert!(matches!(
            read_raw(&scratch("absent.raw"), &mut pixels),
            Err(RawError::Io(_))
        ));
    }

    #[test]
    fn check_raw_file_reports_a_missing_file() {
        let p = scratch("missing.raw");
        assert_eq!(
            check_raw_file(&p, SystemTime::UNIX_EPOCH, 16),
            FileState::Missing
        );
    }

    #[test]
    fn check_raw_file_reports_a_partial_write() {
        let p = scratch("partial.raw");
        fs::write(&p, [0u8; 8]).unwrap();
        assert_eq!(
            check_raw_file(&p, SystemTime::UNIX_EPOCH, 16),
            FileState::Incomplete { got: 8 }
        );
        fs::remove_file(&p).ok();
    }

    #[test]
    fn check_raw_file_accepts_a_complete_new_file() {
        let p = scratch("ready.raw");
        fs::write(&p, [0u8; 16]).unwrap();
        assert_eq!(
            check_raw_file(&p, SystemTime::UNIX_EPOCH, 16),
            FileState::Ready
        );
        fs::remove_file(&p).ok();
    }

    #[test]
    fn check_raw_file_rejects_a_file_from_a_previous_run() {
        let p = scratch("stale.raw");
        fs::write(&p, [0u8; 16]).unwrap();
        // An acquisition that starts an hour from now must not accept the file
        // written just above.
        let later = SystemTime::now() + Duration::from_secs(3600);
        assert_eq!(check_raw_file(&p, later, 16), FileState::Stale);
        fs::remove_file(&p).ok();
    }

    #[test]
    fn check_raw_file_accepts_a_file_written_in_the_start_second() {
        let p = scratch("same-second.raw");
        let now = SystemTime::now();
        fs::write(&p, [0u8; 16]).unwrap();
        assert_eq!(check_raw_file(&p, now, 16), FileState::Ready);
        fs::remove_file(&p).ok();
    }
}
