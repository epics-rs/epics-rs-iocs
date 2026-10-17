//! C `ADTimePix3BpcFile` (bpc_file_io.cpp): BPC files are read and written
//! only at the size the detector geometry gives.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

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

/// C `writeExact`.
pub fn write_exact(path: &Path, data: &[u8], expected_size: usize) -> Status {
    if expected_size == 0 || data.len() != expected_size || expected_size > i64::MAX as usize {
        return Status::InvalidExpectedSize;
    }
    write_atomic(path, data)
}

/// C `writeAtomic`: stage `.NAME.tmp.XXXXXX` beside `path` with the
/// target's mode (0640 for a new file), sync it and rename it over `path`.
pub fn write_atomic(path: &Path, data: &[u8]) -> Status {
    if path.as_os_str().is_empty() {
        return Status::InvalidExpectedSize;
    }
    let (Some(parent), Some(name)) = (
        path.parent().filter(|p| !p.as_os_str().is_empty()),
        path.file_name(),
    ) else {
        return Status::OpenFailed;
    };
    let mode = std::fs::metadata(path).map_or(0o640, |m| permission_bits(&m));
    let Some((staged, mut file)) = mkstemp(parent, name) else {
        return Status::OpenFailed;
    };
    if set_mode(&file, mode).is_err() {
        drop(file);
        let _ = std::fs::remove_file(&staged);
        return Status::OpenFailed;
    }
    let mut status = match file.write_all(data) {
        Ok(()) => Status::Ok,
        Err(_) => Status::WriteFailed,
    };
    if status == Status::Ok && file.sync_all().is_err() {
        status = Status::SyncFailed;
    }
    drop(file);
    if status == Status::Ok && std::fs::rename(&staged, path).is_err() {
        status = Status::RenameFailed;
    }
    if status != Status::Ok {
        let _ = std::fs::remove_file(&staged);
    }
    status
}

/// `mkstemp`: a new file named `.NAME.tmp.` plus six random characters,
/// created exclusively with mode 0600.
fn mkstemp(parent: &Path, name: &std::ffi::OsStr) -> Option<(PathBuf, File)> {
    use std::hash::{BuildHasher, RandomState};
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    let state = RandomState::new();
    for attempt in 0u32..100 {
        let mut bits = state.hash_one((attempt, std::time::SystemTime::now()));
        let suffix: String = (0..6)
            .map(|_| {
                let c = CHARS[(bits % CHARS.len() as u64) as usize];
                bits /= CHARS.len() as u64;
                c as char
            })
            .collect();
        let mut staged_name = std::ffi::OsString::from(".");
        staged_name.push(name);
        staged_name.push(format!(".tmp.{suffix}"));
        let staged = parent.join(staged_name);
        let mut options = std::fs::OpenOptions::new();
        options.read(true).write(true).create_new(true);
        #[cfg(unix)]
        std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
        match options.open(&staged) {
            Ok(file) => return Some((staged, file)),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(_) => return None,
        }
    }
    None
}

#[cfg(unix)]
fn permission_bits(meta: &std::fs::Metadata) -> u32 {
    std::os::unix::fs::PermissionsExt::mode(&meta.permissions()) & 0o777
}

#[cfg(not(unix))]
fn permission_bits(_: &std::fs::Metadata) -> u32 {
    0o640
}

/// `fchmod`.
#[cfg(unix)]
fn set_mode(file: &File, mode: u32) -> std::io::Result<()> {
    file.set_permissions(std::os::unix::fs::PermissionsExt::from_mode(mode))
}

#[cfg(not(unix))]
fn set_mode(_: &File, _: u32) -> std::io::Result<()> {
    Ok(())
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

    #[test]
    fn a_write_replaces_the_target_atomically_with_its_mode() {
        let dir = std::env::temp_dir().join(format!("tpx3-bpc-write-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("m.bpc");

        assert_eq!(write_exact(&file, &[1, 2], 3), Status::InvalidExpectedSize);
        assert!(!file.exists());
        assert_eq!(write_exact(&file, &[1, 2, 3], 3), Status::Ok);
        assert_eq!(std::fs::read(&file).unwrap(), [1, 2, 3]);
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = |f: &Path| std::fs::metadata(f).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode(&file), 0o640);
            std::fs::set_permissions(&file, std::fs::Permissions::from_mode(0o604)).unwrap();
            assert_eq!(write_exact(&file, &[4, 5, 6], 3), Status::Ok);
            assert_eq!(mode(&file), 0o604);
        }
        assert_eq!(std::fs::read(&file).unwrap(), [4, 5, 6]);

        // Renaming over a directory fails, and the staged file is removed.
        std::fs::create_dir(dir.join("d.bpc")).unwrap();
        assert_eq!(write_atomic(&dir.join("d.bpc"), &[1]), Status::RenameFailed);
        assert_eq!(
            write_atomic(Path::new("bare.bpc"), &[1]),
            Status::OpenFailed
        );
        assert_eq!(
            write_atomic(&dir.join("missing/x.bpc"), &[1]),
            Status::OpenFailed
        );
        let mut names: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        names.sort();
        assert_eq!(names, ["d.bpc", "m.bpc"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
