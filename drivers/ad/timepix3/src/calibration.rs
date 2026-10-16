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

fn canonical_dir(path: &Path) -> Option<PathBuf> {
    let canonical = std::fs::canonicalize(path).ok()?;
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
        // C `isBaseName`: `filename() == name`, which a trailing '/' fails.
        if file_name.ends_with('/')
            || !matches!(
                (components.next(), components.next()),
                (Some(Component::Normal(_)), None)
            )
        {
            return Err(PathError::InvalidFileName);
        }
        let requested = self.validate_directory(directory)?.join(file_name);
        let target = if access == PathAccess::Read || requested.exists() {
            let target =
                std::fs::canonicalize(&requested).map_err(|_| PathError::TargetUnavailable)?;
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
        std::os::unix::fs::symlink(dir.join("outside/b.bpc"), dir.join("root/sub/link.bpc"))
            .unwrap();
        dir
    }

    #[test]
    fn an_unconfigured_or_relative_root_refuses() {
        let mut p = PathPolicy::default();
        assert_eq!(p.validate_directory("/"), Err(PathError::RootNotConfigured));
        assert_eq!(p.configure(""), Err(PathError::RootNotConfigured));
        assert_eq!(p.configure("rel"), Err(PathError::RootNotAbsolute));
        assert_eq!(
            p.configure("/no/such/dir/anywhere"),
            Err(PathError::RootUnavailable)
        );
        p.configure("/").unwrap();
        assert!(p.permissive());
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
            Ok(t.join("root/sub").canonicalize().unwrap())
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
        for bad in ["", ".", "..", "x/a.bpc", "/a.bpc", "a.bpc/", "./a.bpc"] {
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
        assert_eq!(
            p.resolve(sub, "link.bpc", PathAccess::Read),
            Err(PathError::TargetOutsideRoot)
        );
        assert_eq!(
            p.resolve(sub, "link.bpc", PathAccess::Write),
            Err(PathError::TargetOutsideRoot)
        );
        let _ = std::fs::remove_dir_all(&t);
    }
}
