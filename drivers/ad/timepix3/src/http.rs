//! The blocking HTTP client for Serval (port of the `ADTimePix3ServalHttp`
//! helpers, serval_http_client.cpp).
//!
//! C uses cpr (libcurl); this uses `ureq`. Serval is plain HTTP, so no TLS
//! stack is linked.

use std::time::Duration;

/// C `kDefaultTimeoutMs` (serval_http.h:18), what a helper call without a
/// timeout argument gets.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(10000);
/// The 5000 ms C passes explicitly at its short polls.
pub const TIMEOUT_5000_MS: Duration = Duration::from_millis(5000);

#[derive(Debug)]
pub enum HttpError {
    Transport(String),
    Status {
        url: String,
        code: u16,
        body: String,
    },
    Parse(String),
}

impl std::fmt::Display for HttpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Transport(m) => write!(f, "{m}"),
            Self::Status { url, code, body } => write!(f, "{url}: HTTP {code}: {body}"),
            Self::Parse(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for HttpError {}

pub type HttpResult<T> = Result<T, HttpError>;

impl HttpError {
    /// The HTTP status the driver publishes in `TPX3_HTTP_CODE`; 0 when the
    /// request never reached the server.
    pub fn code(&self) -> i32 {
        match self {
            Self::Status { code, .. } => i32::from(*code),
            _ => 0,
        }
    }

    /// The reply body cpr would hold: empty unless Serval answered.
    pub fn body(&self) -> &str {
        match self {
            Self::Status { body, .. } => body,
            _ => "",
        }
    }
}

/// A Serval endpoint client.
#[derive(Clone)]
pub struct ServalHttp {
    agent: ureq::Agent,
    base: String,
    /// The `Authorization` header, sent only when credentials are configured.
    auth: Option<String>,
    /// Each request's method, path and timeout, for the per-site tests.
    #[cfg(test)]
    pub(crate) sent: std::sync::Arc<parking_lot::Mutex<Vec<(String, Duration)>>>,
}

impl ServalHttp {
    pub fn new(server_url: &str) -> Self {
        Self::with_credentials(server_url, env_credentials())
    }

    /// A client sending Basic auth with `credentials` (user, password).
    pub fn with_credentials(server_url: &str, credentials: Option<(String, String)>) -> Self {
        let agent = ureq::Agent::config_builder()
            // The driver inspects status codes itself.
            .http_status_as_error(false)
            .timeout_global(Some(DEFAULT_TIMEOUT))
            .build()
            .new_agent();
        Self {
            agent,
            // A trailing slash would double up in every path.
            base: server_url.trim_end_matches('/').to_string(),
            auth: credentials.map(|(user, pass)| basic_auth_header(&user, &pass)),
            #[cfg(test)]
            sent: Default::default(),
        }
    }

    pub fn base_url(&self) -> &str {
        &self.base
    }

    /// C `get`: GET with the query `?anon=true&key=value` (`kServalParams`).
    pub fn get(&self, path: &str, timeout: Duration) -> HttpResult<String> {
        self.get_with(path, timeout, true, false)
    }

    /// C `getAuthOnly`: GET with no query parameters.
    pub fn get_auth_only(&self, path: &str, timeout: Duration) -> HttpResult<String> {
        self.get_with(path, timeout, false, false)
    }

    /// C `getJson`: GET with no query parameters and a
    /// `Content-Type: application/json` header.
    pub fn get_json(&self, path: &str, timeout: Duration) -> HttpResult<String> {
        self.get_with(path, timeout, false, true)
    }

    fn get_with(
        &self,
        path: &str,
        timeout: Duration,
        serval_params: bool,
        json_header: bool,
    ) -> HttpResult<String> {
        #[cfg(test)]
        self.sent.lock().push((format!("GET {path}"), timeout));
        let url = format!("{}{path}", self.base);
        let mut req = self.agent.get(&url);
        if serval_params {
            req = req.query("anon", "true").query("key", "value");
        }
        if json_header {
            req = req.header("Content-Type", "application/json");
        }
        if let Some(auth) = &self.auth {
            req = req.header("Authorization", auth);
        }
        let mut resp = req
            .config()
            .timeout_global(Some(timeout))
            .build()
            .call()
            .map_err(|e| HttpError::Transport(format!("GET {url}: {e}")))?;
        let code = resp.status().as_u16();
        let body = resp
            .body_mut()
            .read_to_string()
            .map_err(|e| HttpError::Transport(format!("GET {url}: reading body: {e}")))?;
        if code != 200 {
            return Err(HttpError::Status { url, code, body });
        }
        Ok(body)
    }

    /// C `putJson`: PUT `body`, already dumped as the caller's C `dump` call
    /// lays it out, returning the reply body.
    pub fn put_json(&self, path: &str, body: &str, timeout: Duration) -> HttpResult<String> {
        #[cfg(test)]
        self.sent.lock().push((format!("PUT {path}"), timeout));
        let url = format!("{}{path}", self.base);
        let mut req = self
            .agent
            .put(&url)
            .header("Content-Type", "application/json");
        if let Some(auth) = &self.auth {
            req = req.header("Authorization", auth);
        }
        let mut resp = req
            .config()
            .timeout_global(Some(timeout))
            .build()
            .send(body)
            .map_err(|e| HttpError::Transport(format!("PUT {url}: {e}")))?;
        let code = resp.status().as_u16();
        let reply = resp
            .body_mut()
            .read_to_string()
            .map_err(|e| HttpError::Transport(format!("PUT {url}: reading body: {e}")))?;
        if code != 200 {
            return Err(HttpError::Status {
                url,
                code,
                body: reply,
            });
        }
        Ok(reply)
    }
}

/// A request line without the query `get` adds, which Serval ignores: what
/// the test Servals route on.
#[cfg(test)]
pub(crate) fn without_serval_params(request_line: &str) -> String {
    request_line.replacen("?anon=true&key=value", "", 1)
}

/// C `servalCredentials` (serval_http_client.cpp:21-35): read once from
/// `ADTIMEPIX_SERVAL_USERNAME`/`ADTIMEPIX_SERVAL_PASSWORD`, and configured
/// only when both are set and non-empty.
fn env_credentials() -> Option<(String, String)> {
    static CREDENTIALS: std::sync::OnceLock<Option<(String, String)>> = std::sync::OnceLock::new();
    CREDENTIALS
        .get_or_init(|| {
            credentials_from(
                std::env::var("ADTIMEPIX_SERVAL_USERNAME").ok(),
                std::env::var("ADTIMEPIX_SERVAL_PASSWORD").ok(),
            )
        })
        .clone()
}

fn credentials_from(user: Option<String>, pass: Option<String>) -> Option<(String, String)> {
    match (user, pass) {
        (Some(user), Some(pass)) if !user.is_empty() && !pass.is_empty() => Some((user, pass)),
        _ => None,
    }
}

/// cpr's `AuthMode::BASIC` header.
fn basic_auth_header(user: &str, pass: &str) -> String {
    format!(
        "Basic {}",
        encode_base64(format!("{user}:{pass}").as_bytes())
    )
}

/// The standard base64 alphabet.
const B64: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

fn encode_base64(data: &[u8]) -> String {
    let mut out = String::new();
    for chunk in data.chunks(3) {
        let b = [
            chunk[0],
            *chunk.get(1).unwrap_or(&0),
            *chunk.get(2).unwrap_or(&0),
        ];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        for i in 0..4 {
            if i <= chunk.len() {
                out.push(B64[((n >> (18 - 6 * i)) & 0x3f) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base64_round_trips() {
        assert_eq!(encode_base64(b""), "");
        for case in ["f", "fo", "foo", "foob", "fooba", "foobar"] {
            let encoded = encode_base64(case.as_bytes());
            assert_eq!(
                crate::mask::decode_base64_strict(&encoded).unwrap(),
                case.as_bytes(),
                "round trip of {case:?} via {encoded:?}"
            );
        }
        // The RFC 4648 vectors.
        assert_eq!(encode_base64(b"foobar"), "Zm9vYmFy");
        assert_eq!(encode_base64(b"fo"), "Zm8=");
    }

    #[test]
    fn the_base_url_loses_a_trailing_slash() {
        assert_eq!(
            ServalHttp::new("http://localhost:8081/").base_url(),
            "http://localhost:8081"
        );
    }

    #[test]
    fn credentials_need_both_names_set_and_non_empty() {
        let some = |v: &str| Some(v.to_owned());
        assert_eq!(
            credentials_from(some("u"), some("p")),
            Some(("u".into(), "p".into()))
        );
        assert_eq!(credentials_from(some("u"), None), None);
        assert_eq!(credentials_from(None, some("p")), None);
        assert_eq!(credentials_from(some(""), some("p")), None);
        assert_eq!(credentials_from(some("u"), some("")), None);
        assert_eq!(basic_auth_header("user", "pass"), "Basic dXNlcjpwYXNz");
    }

    /// Basic auth goes on every request when credentials are configured and
    /// on none otherwise (serval_http_client.cpp:44-95).
    #[test]
    fn auth_is_sent_only_when_configured() {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let (tx, rx) = std::sync::mpsc::channel::<Option<String>>();
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let mut auth = None;
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                    if line.to_ascii_lowercase().starts_with("authorization:") {
                        auth = Some(line["authorization:".len()..].trim().to_owned());
                    }
                }
                let _ = tx.send(auth);
                let _ = reader.get_mut().write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                );
            }
        });
        let timeout = Duration::from_secs(5);
        let with = ServalHttp::with_credentials(&url, Some(("u".into(), "p".into())));
        with.get("/a", timeout).unwrap();
        with.put_json("/b", "{}", timeout).unwrap();
        let without = ServalHttp::with_credentials(&url, None);
        without.get("/a", timeout).unwrap();
        without.put_json("/b", "{}", timeout).unwrap();
        let seen: Vec<_> = rx.iter().take(4).collect();
        let basic = Some(basic_auth_header("u", "p"));
        assert_eq!(seen, [basic.clone(), basic, None, None]);
    }

    /// The caller's timeout bounds the whole request, as `cpr::Timeout`
    /// does (serval_http_client.cpp:44-95).
    #[test]
    fn each_helper_gives_up_at_its_timeout() {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                std::thread::spawn(move || {
                    let mut reader = BufReader::new(stream);
                    let mut line = String::new();
                    while reader.read_line(&mut line).unwrap_or(0) > 0 && line != "\r\n" {
                        line.clear();
                    }
                    std::thread::sleep(Duration::from_millis(500));
                    let _ = reader.get_mut().write_all(
                        b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                    );
                });
            }
        });
        let http = ServalHttp::with_credentials(&url, None);
        let (short, long) = (Duration::from_millis(100), Duration::from_secs(5));
        assert!(http.get("/a", short).is_err());
        assert!(http.get_auth_only("/a", short).is_err());
        assert!(http.get_json("/a", short).is_err());
        assert!(http.put_json("/a", "{}", short).is_err());
        assert_eq!(http.get("/a", long).unwrap(), "{}");
        assert_eq!(http.put_json("/a", "{}", long).unwrap(), "{}");
    }

    /// Each helper's query and headers as serval_http_client.cpp sends them:
    /// only `get` carries `kServalParams`, only `getJson`/`putJson` carry
    /// `kJsonHeader`, and none overrides curl's `Accept: */*`.
    #[test]
    fn each_helper_sends_c_query_and_headers() {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let (tx, rx) = std::sync::mpsc::channel::<(String, Vec<String>)>();
        std::thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let mut reader = BufReader::new(stream);
                let mut request = String::new();
                reader.read_line(&mut request).unwrap();
                let mut headers = Vec::new();
                let mut length = 0;
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                    let line = line.trim().to_ascii_lowercase();
                    if let Some(n) = line.strip_prefix("content-length:") {
                        length = n.trim().parse().unwrap();
                    }
                    if line.starts_with("accept:") || line.starts_with("content-type:") {
                        headers.push(line);
                    }
                }
                let mut body = vec![0; length];
                reader.read_exact(&mut body).unwrap();
                let _ = tx.send((request.trim().to_owned(), headers));
                let _ = reader.get_mut().write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                );
            }
        });
        let http = ServalHttp::with_credentials(&url, None);
        http.get("/a", DEFAULT_TIMEOUT).unwrap();
        http.get("", DEFAULT_TIMEOUT).unwrap();
        http.get_auth_only("/b?x=1", DEFAULT_TIMEOUT).unwrap();
        http.get_json("/c", DEFAULT_TIMEOUT).unwrap();
        http.put_json("/d", "{}", DEFAULT_TIMEOUT).unwrap();
        let seen: Vec<_> = rx.iter().take(5).collect();
        let accept = "accept: */*".to_owned();
        let json = "content-type: application/json".to_owned();
        assert_eq!(
            seen,
            [
                (
                    "GET /a?anon=true&key=value HTTP/1.1".into(),
                    vec![accept.clone()]
                ),
                (
                    "GET /?anon=true&key=value HTTP/1.1".into(),
                    vec![accept.clone()]
                ),
                ("GET /b?x=1 HTTP/1.1".into(), vec![accept.clone()]),
                ("GET /c HTTP/1.1".into(), vec![accept.clone(), json.clone()]),
                ("PUT /d HTTP/1.1".into(), vec![accept, json]),
            ]
        );
    }
}
