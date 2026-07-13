//! Fetch raw bytes from a URL string.
//!
//! Mirrors GraphicsMagick's `Image::read(URLString)`, which accepts an
//! `http://`/`https://` URL, a `file://` URL, or a bare filesystem path (its
//! generic image-format-detection reads treat an unrecognized scheme as a
//! filename). This driver replaces GraphicsMagick's networking with `ureq`
//! and its file I/O with `std::fs`.

use std::io::Read as _;

#[derive(Debug)]
pub enum FetchError {
    /// C++ `readImage()`: `if (strlen(URLString) == 0) return(asynError);`
    EmptyUrl,
    Io(std::io::Error),
    Http(String),
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FetchError::EmptyUrl => write!(f, "empty URL"),
            FetchError::Io(e) => write!(f, "I/O error: {e}"),
            FetchError::Http(e) => write!(f, "HTTP error: {e}"),
        }
    }
}

impl std::error::Error for FetchError {}

/// Fetch the raw bytes named by `url`.
///
/// Dispatch order: `file://` scheme, then `http(s)://` scheme, then a bare
/// filesystem path as a fallback (matching GraphicsMagick's behavior of
/// treating any string without a recognized URL scheme as a filename).
pub fn fetch_bytes(url: &str) -> Result<Vec<u8>, FetchError> {
    if url.is_empty() {
        return Err(FetchError::EmptyUrl);
    }
    if let Some(path) = url.strip_prefix("file://") {
        return std::fs::read(path).map_err(FetchError::Io);
    }
    if url.starts_with("http://") || url.starts_with("https://") {
        let resp = ureq::get(url)
            .call()
            .map_err(|e| FetchError::Http(e.to_string()))?;
        let mut buf = Vec::new();
        resp.into_reader()
            .read_to_end(&mut buf)
            .map_err(FetchError::Io)?;
        return Ok(buf);
    }
    std::fs::read(url).map_err(FetchError::Io)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_url_is_rejected() {
        assert!(matches!(fetch_bytes(""), Err(FetchError::EmptyUrl)));
    }

    #[test]
    fn bare_path_reads_local_file() {
        let dir = std::env::temp_dir();
        let path = dir.join("ad_url_fetch_test_bare.bin");
        std::fs::write(&path, b"bare-path-bytes").unwrap();

        let result = fetch_bytes(path.to_str().unwrap()).unwrap();

        std::fs::remove_file(&path).ok();
        assert_eq!(result, b"bare-path-bytes");
    }

    #[test]
    fn file_scheme_reads_local_file() {
        let dir = std::env::temp_dir();
        let path = dir.join("ad_url_fetch_test_scheme.bin");
        std::fs::write(&path, b"file-scheme-bytes").unwrap();

        let url = format!("file://{}", path.to_str().unwrap());
        let result = fetch_bytes(&url).unwrap();

        std::fs::remove_file(&path).ok();
        assert_eq!(result, b"file-scheme-bytes");
    }

    #[test]
    fn missing_file_is_io_error() {
        let dir = std::env::temp_dir();
        let path = dir.join("ad_url_fetch_test_does_not_exist.bin");
        std::fs::remove_file(&path).ok();

        assert!(matches!(
            fetch_bytes(path.to_str().unwrap()),
            Err(FetchError::Io(_))
        ));
    }
}
