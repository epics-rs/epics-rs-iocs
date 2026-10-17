//! C `ADTimePix3BpcFile` (bpc_file_io.cpp): BPC files are read and written
//! only at the size the detector geometry gives.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Ok,
    InvalidExpectedSize,
    OpenFailed,
    SizeMismatch,
    ReadFailed,
    WriteFailed,
    SyncFailed,
    RenameFailed,
}

impl Status {
    /// C `statusMessage`.
    pub fn message(self) -> &'static str {
        match self {
            Self::Ok => "ok",
            Self::InvalidExpectedSize => "invalid expected BPC size",
            Self::OpenFailed => "unable to open BPC file",
            Self::SizeMismatch => "BPC file size does not match detector geometry",
            Self::ReadFailed => "incomplete BPC file read",
            Self::WriteFailed => "incomplete BPC file write",
            Self::SyncFailed => "unable to synchronize staged file",
            Self::RenameFailed => "unable to atomically replace target file",
        }
    }
}

/// C `expectedSize`: pixels × bytes per pixel × threshold slices, `None`
/// for a non-positive factor or an overflow.
pub fn expected_size(
    pixel_count: i32,
    bytes_per_pixel: i32,
    threshold_slices: i32,
) -> Option<usize> {
    if pixel_count <= 0 || bytes_per_pixel <= 0 || threshold_slices <= 0 {
        return None;
    }
    (pixel_count as usize)
        .checked_mul(bytes_per_pixel as usize)?
        .checked_mul(threshold_slices as usize)
}

/// C `readExact`. An `ifstream` opened `ate` fails to open when the seek to
/// the end fails, so that is `OpenFailed` here too.
pub fn read_exact(path: &Path, expected_size: usize) -> Result<Vec<u8>, Status> {
    if expected_size == 0 || expected_size > i64::MAX as usize {
        return Err(Status::InvalidExpectedSize);
    }
    let mut input = File::open(path).map_err(|_| Status::OpenFailed)?;
    let end = input
        .seek(SeekFrom::End(0))
        .map_err(|_| Status::OpenFailed)?;
    if end != expected_size as u64 {
        return Err(Status::SizeMismatch);
    }
    let mut data = vec![0u8; expected_size];
    input
        .seek(SeekFrom::Start(0))
        .and_then(|_| input.read_exact(&mut data))
        .map_err(|_| Status::ReadFailed)?;
    Ok(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_expected_size_refuses_a_non_positive_factor_and_an_overflow() {
        assert_eq!(expected_size(65536, 1, 1), Some(65536));
        assert_eq!(expected_size(262144, 2, 1), Some(524288));
        assert_eq!(expected_size(0, 1, 1), None);
        assert_eq!(expected_size(1, -1, 1), None);
        assert_eq!(expected_size(1, 1, 0), None);
        assert_eq!(expected_size(i32::MAX, i32::MAX, i32::MAX), None);
    }

    #[test]
    fn a_file_is_read_only_at_the_expected_size() {
        let dir = std::env::temp_dir().join(format!("tpx3-bpc-read-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("a.bpc");
        std::fs::write(&file, [1u8, 2, 3, 4]).unwrap();

        assert_eq!(read_exact(&file, 4), Ok(vec![1, 2, 3, 4]));
        assert_eq!(read_exact(&file, 5), Err(Status::SizeMismatch));
        assert_eq!(read_exact(&file, 0), Err(Status::InvalidExpectedSize));
        assert_eq!(
            read_exact(&dir.join("missing.bpc"), 4),
            Err(Status::OpenFailed)
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
