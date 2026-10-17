//! Calibration path containment (C `calibration_path_policy.cpp`).

use std::path::{Component, Path, PathBuf};

/// C `PathAccess`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathAccess {
    Read,
    Write,
}

/// C `PathStatus`; `Ok` is the `Ok` arm of the `Result`s below.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathError {
    RootNotConfigured,
    RootNotAbsolute,
    RootUnavailable,
    DirectoryUnavailable,
    DirectoryOutsideRoot,
    InvalidFileName,
    TargetUnavailable,
    TargetNotRegular,
    TargetOutsideRoot,
}

impl PathError {
    /// C `statusMessage`.
    pub fn message(self) -> &'static str {
        match self {
            Self::RootNotConfigured => "calibration root is not configured",
            Self::RootNotAbsolute => "calibration root is not absolute",
            Self::RootUnavailable => "calibration root is not an accessible directory",
            Self::DirectoryUnavailable => "calibration directory is not accessible",
            Self::DirectoryOutsideRoot => "calibration directory is outside the approved root",
            Self::InvalidFileName => "calibration filename must be a basename",
            Self::TargetUnavailable => "calibration target is not accessible",
            Self::TargetNotRegular => "calibration target is not a regular file",
            Self::TargetOutsideRoot => "calibration target resolves outside the approved root",
        }
    }
}

/// C `PathPolicy`: an unconfigured policy (empty root) refuses everything.
#[derive(Debug, Clone, Default)]
pub struct PathPolicy {
    root: Option<PathBuf>,
}

/// C `fs::canonical`. Rust's Windows form is a verbatim `\\?\` path, which
/// MSVC's `canonical` strips back to `C:\…` or `\\server\share\…`.
pub fn canonical(path: &Path) -> std::io::Result<PathBuf> {
    let canonical = std::fs::canonicalize(path)?;
    #[cfg(windows)]
    let canonical = without_verbatim_prefix(canonical);
    Ok(canonical)
}

#[cfg(windows)]
fn without_verbatim_prefix(path: PathBuf) -> PathBuf {
    use std::path::Prefix;
    let kind = match path.components().next() {
        Some(Component::Prefix(p)) => Some(p.kind()),
        _ => None,
    };
    let text = path.to_string_lossy();
    match kind {
        Some(Prefix::VerbatimDisk(_)) => PathBuf::from(&text[4..]),
        Some(Prefix::VerbatimUNC(..)) => PathBuf::from(format!("\\\\{}", &text[8..])),
        _ => path.clone(),
    }
}

fn canonical_dir(path: &Path) -> Option<PathBuf> {
    let canonical = canonical(path).ok()?;
    canonical.is_dir().then_some(canonical)
}

impl PathPolicy {
    /// C `configure`: the policy is left unconfigured on any error.
    pub fn configure(&mut self, root: &str) -> Result<(), PathError> {
        self.root = None;
        if root.is_empty() {
            return Err(PathError::RootNotConfigured);
        }
        if !Path::new(root).is_absolute() {
            return Err(PathError::RootNotAbsolute);
        }
        self.root = Some(canonical_dir(Path::new(root)).ok_or(PathError::RootUnavailable)?);
        Ok(())
    }

    /// C `validateDirectory`.
    pub fn validate_directory(&self, directory: &str) -> Result<PathBuf, PathError> {
        let root = self.root.as_ref().ok_or(PathError::RootNotConfigured)?;
        let dir = canonical_dir(Path::new(directory)).ok_or(PathError::DirectoryUnavailable)?;
        if !dir.starts_with(root) {
            return Err(PathError::DirectoryOutsideRoot);
        }
        Ok(dir)
    }

    /// C `resolve`: `file_name` must be a bare file name inside `directory`.
    pub fn resolve(
        &self,
        directory: &str,
        file_name: &str,
        access: PathAccess,
    ) -> Result<PathBuf, PathError> {
        let root = self.root.as_ref().ok_or(PathError::RootNotConfigured)?;
        let mut components = Path::new(file_name).components();
        // C `isBaseName`: `filename() == name`, which a trailing separator
        // ('\\' too on Windows) fails.
        if file_name.ends_with(std::path::is_separator)
            || !matches!(
                (components.next(), components.next()),
                (Some(Component::Normal(_)), None)
            )
        {
            return Err(PathError::InvalidFileName);
        }
        let requested = self.validate_directory(directory)?.join(file_name);
        let target = if access == PathAccess::Read || requested.exists() {
            let target = canonical(&requested).map_err(|_| PathError::TargetUnavailable)?;
            if access == PathAccess::Read && !target.is_file() {
                return Err(PathError::TargetNotRegular);
            }
            target
        } else {
            // weakly_canonical of a canonical directory plus a bare name.
            requested
        };
        if !target.starts_with(root) {
            return Err(PathError::TargetOutsideRoot);
        }
        Ok(target)
    }

    pub fn root(&self) -> String {
        self.root
            .as_ref()
            .map(|r| r.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    pub fn permissive(&self) -> bool {
        self.root.as_deref() == Some(Path::new("/"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(dir.join("root/sub")).unwrap();
        std::fs::create_dir_all(dir.join("outside")).unwrap();
        std::fs::write(dir.join("root/sub/a.bpc"), b"x").unwrap();
        std::fs::write(dir.join("outside/b.bpc"), b"x").unwrap();
        let (target, link) = (dir.join("outside/b.bpc"), dir.join("root/sub/link.bpc"));
        #[cfg(unix)]
        std::os::unix::fs::symlink(target, link).unwrap();
        // Windows needs a privilege to make a symlink; without it the
        // link cases are skipped.
        #[cfg(windows)]
        let _ = std::os::windows::fs::symlink_file(target, link);
        dir
    }

    #[cfg(windows)]
    #[test]
    fn a_canonical_path_has_no_verbatim_prefix() {
        for (raw, plain) in [
            (r"\\?\C:\cal\a.bpc", r"C:\cal\a.bpc"),
            (r"\\?\UNC\host\share\a.bpc", r"\\host\share\a.bpc"),
            (r"C:\cal\a.bpc", r"C:\cal\a.bpc"),
        ] {
            assert_eq!(without_verbatim_prefix(raw.into()), PathBuf::from(plain));
        }
        let tmp = canonical(&std::env::temp_dir()).unwrap();
        assert!(!tmp.to_string_lossy().starts_with(r"\\?\"), "{tmp:?}");
    }

    #[test]
    fn an_unconfigured_or_relative_root_refuses() {
        let mut p = PathPolicy::default();
        assert_eq!(p.validate_directory("/"), Err(PathError::RootNotConfigured));
        assert_eq!(p.configure(""), Err(PathError::RootNotConfigured));
        assert_eq!(p.configure("rel"), Err(PathError::RootNotAbsolute));
        let missing = std::env::temp_dir().join("no/such/dir/anywhere");
        assert_eq!(
            p.configure(missing.to_str().unwrap()),
            Err(PathError::RootUnavailable)
        );
        // "/" has no drive on Windows, so C's is_absolute() refuses it there.
        if cfg!(windows) {
            assert_eq!(p.configure("/"), Err(PathError::RootNotAbsolute));
        } else {
            p.configure("/").unwrap();
            assert!(p.permissive());
        }
    }

    #[test]
    fn a_directory_outside_the_root_refuses() {
        let t = tree("tpx-calib-dir");
        let mut p = PathPolicy::default();
        p.configure(t.join("root").to_str().unwrap()).unwrap();
        assert!(!p.permissive());
        let sub = t.join("root/sub/");
        assert_eq!(
            p.validate_directory(sub.to_str().unwrap()),
            Ok(canonical(&t.join("root/sub")).unwrap())
        );
        let escape = t.join("root/../outside");
        assert_eq!(
            p.validate_directory(escape.to_str().unwrap()),
            Err(PathError::DirectoryOutsideRoot)
        );
        assert_eq!(
            p.validate_directory(t.join("root/none").to_str().unwrap()),
            Err(PathError::DirectoryUnavailable)
        );
        let _ = std::fs::remove_dir_all(&t);
    }

    #[test]
    fn a_target_is_a_basename_inside_the_root() {
        let t = tree("tpx-calib-resolve");
        let mut p = PathPolicy::default();
        p.configure(t.join("root").to_str().unwrap()).unwrap();
        let sub = t.join("root/sub");
        let sub = sub.to_str().unwrap();
        assert!(p.resolve(sub, "a.bpc", PathAccess::Read).is_ok());
        let mut bad = vec!["", ".", "..", "x/a.bpc", "/a.bpc", "a.bpc/", "./a.bpc"];
        if cfg!(windows) {
            bad.extend([r"a.bpc\", r"x\a.bpc", r"C:a.bpc", r"\a.bpc"]);
        }
        for bad in bad {
            assert_eq!(
                p.resolve(sub, bad, PathAccess::Read),
                Err(PathError::InvalidFileName),
                "{bad:?}"
            );
        }
        assert_eq!(
            p.resolve(sub, "none.bpc", PathAccess::Read),
            Err(PathError::TargetUnavailable)
        );
        assert!(p.resolve(sub, "new.bpc", PathAccess::Write).is_ok());
        if std::fs::symlink_metadata(t.join("root/sub/link.bpc")).is_ok() {
            assert_eq!(
                p.resolve(sub, "link.bpc", PathAccess::Read),
                Err(PathError::TargetOutsideRoot)
            );
            assert_eq!(
                p.resolve(sub, "link.bpc", PathAccess::Write),
                Err(PathError::TargetOutsideRoot)
            );
        }
        let _ = std::fs::remove_dir_all(&t);
    }
}
