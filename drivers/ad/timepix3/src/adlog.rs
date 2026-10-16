//! ADTimePix3's trace lines, as `ADTimePixLog.h`'s `ERR`/`WARN`/`LOG` and
//! `serval_http.cpp`'s `logHttpFailure`/`logHttpWarning` print them. The
//! function part is GCC's `__PRETTY_FUNCTION__`, which C builds by default.

use crate::http::HttpError;

const LOG_HTTP_FAILURE: &str = "void ADTimePix::logHttpFailure(const char*, const char*, \
                                const std::string&, long int, const std::string&)";
const LOG_HTTP_WARNING: &str = "void ADTimePix::logHttpWarning(const char*, const char*, \
                                const std::string&, long int, const std::string&)";

pub const REFRESH_PIXEL_CONFIG: &str = "asynStatus ADTimePix::refreshPixelConfigFromServal()";
pub const WRITE_BPC_FILE: &str = "asynStatus ADTimePix::writeBPCfile(char**, int*)";
pub const TIME_PIX_CALLBACK: &str = "void ADTimePix::timePixCallback()";

/// `ERR`/`ERR_ARGS`, at `ASYN_TRACE_ERROR`.
pub fn err(func: &str, msg: &str) -> String {
    format!("ERROR | ADTimePix::{func}: {msg}")
}

/// `WARN`/`WARN_ARGS`, at `ASYN_TRACE_WARNING`.
pub fn warn(func: &str, msg: &str) -> String {
    format!("WARN | ADTimePix::{func}: {msg}")
}

/// `LOG`/`LOG_ARGS`, at `ASYN_TRACEIO_DRIVER`.
pub fn log(func: &str, msg: &str) -> String {
    format!("ADTimePix::{func}: {msg}")
}

/// C `trimHttpBodyForLog`: control characters become spaces, the body is cut
/// at 512 bytes and marked with `...`.
fn trim_body(body: &str) -> String {
    const MAX: usize = 512;
    let bytes = body.as_bytes();
    let kept: Vec<u8> = bytes[..bytes.len().min(MAX)]
        .iter()
        .map(|&c| if c < 32 || c == 127 { b' ' } else { c })
        .collect();
    let mut out = String::from_utf8_lossy(&kept).into_owned();
    if bytes.len() > MAX {
        out.push_str("...");
    }
    out
}

/// `"{context}: {method} {url} HTTP {status}: {body}"`; `None` when the
/// request got a 200 and only its body failed to parse, where C prints no
/// HTTP line. A request that never reached Serval is cpr's status 0.
fn http_line(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    let (code, body) = match e {
        HttpError::Status { code, body, .. } => (i64::from(*code), trim_body(body)),
        HttpError::Transport(_) => (0, String::new()),
        HttpError::Parse(_) => return None,
    };
    Some(format!("{context}: {method} {url} HTTP {code}: {body}"))
}

pub fn http_failure(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    http_line(context, method, url, e).map(|m| err(LOG_HTTP_FAILURE, &m))
}

pub fn http_warning(context: &str, method: &str, url: &str, e: &HttpError) -> Option<String> {
    http_line(context, method, url, e).map(|m| warn(LOG_HTTP_WARNING, &m))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn http_failure_is_log_http_failure_text() {
        let e = HttpError::Status {
            url: String::new(),
            code: 404,
            body: "no\nsuch".into(),
        };
        assert_eq!(
            http_failure("rotateLayout", "GET", "http://s/x", &e).unwrap(),
            "ERROR | ADTimePix::void ADTimePix::logHttpFailure(const char*, const char*, \
             const std::string&, long int, const std::string&): rotateLayout: GET http://s/x \
             HTTP 404: no such"
        );
        assert_eq!(
            http_failure("c", "GET", "u", &HttpError::Transport("x".into())).unwrap(),
            format!("ERROR | ADTimePix::{LOG_HTTP_FAILURE}: c: GET u HTTP 0: ")
        );
        assert!(http_failure("c", "GET", "u", &HttpError::Parse("x".into())).is_none());
    }

    #[test]
    fn long_body_is_cut_at_512() {
        let body = "a".repeat(600);
        let t = trim_body(&body);
        assert_eq!(t.len(), 515);
        assert!(t.ends_with("a..."));
    }
}
