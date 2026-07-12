//! The blocking HTTP client for Serval (port of `serval_http.cpp`'s
//! `ADTimePix3ServalHttp` helpers, serval_http.cpp:63-89).
//!
//! C uses cpr (libcurl); this uses `ureq`. Serval is plain HTTP, so no TLS
//! stack is linked.

use std::time::Duration;

use serde_json::Value;

/// The timeout C passes to its polling GETs (`SERVAL_TIMEOUT_MS`).
pub const TIMEOUT_POLL: Duration = Duration::from_secs(5);
/// The timeout C passes to the two configuration PUTs.
pub const TIMEOUT_CONFIG: Duration = Duration::from_secs(10);

/// Serval's credentials (C `kServalAuth`, serval_http.cpp:36).
const USER: &str = "user";
const PASS: &str = "pass";

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
}

/// The base64 alphabet Serval encodes `PixelConfig` with.
const B64: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Decode standard base64.
///
/// UPSTREAM DEFECT (serval_http.cpp:49): C looks the character up with
/// `strchr(kChars, c)`, which *matches the terminating NUL* for `c == 0` and
/// returns index 64 — a NUL byte in the payload is silently decoded as a valid
/// 6-bit group instead of failing. C also validates neither the padding nor the
/// length. This returns `None` for any byte outside the alphabet.
pub fn decode_base64(input: &str) -> Option<Vec<u8>> {
    let bytes: Vec<u8> = input
        .bytes()
        .filter(|b| !b.is_ascii_whitespace())
        .take_while(|&b| b != b'=')
        .collect();
    let mut out = Vec::with_capacity(bytes.len() * 3 / 4);
    let mut acc: u32 = 0;
    let mut nbits = 0u32;
    for b in bytes {
        let v = B64.iter().position(|&c| c == b)? as u32;
        acc = (acc << 6) | v;
        nbits += 6;
        if nbits >= 8 {
            nbits -= 8;
            out.push((acc >> nbits) as u8);
        }
    }
    Some(out)
}

/// A Serval endpoint client.
#[derive(Clone)]
pub struct ServalHttp {
    agent: ureq::Agent,
    base: String,
}

impl ServalHttp {
    pub fn new(server_url: &str) -> Self {
        let agent = ureq::Agent::config_builder()
            // The driver inspects status codes itself.
            .http_status_as_error(false)
            .timeout_global(Some(TIMEOUT_POLL))
            .build()
            .new_agent();
        Self {
            agent,
            // A trailing slash would double up in every path.
            base: server_url.trim_end_matches('/').to_string(),
        }
    }

    pub fn base_url(&self) -> &str {
        &self.base
    }

    /// GET, returning the body.
    ///
    /// UPSTREAM DEFECT (serval_http.cpp:37): every C `get()` appends the query
    /// string `?anon=true&key=value` — cpr sample-code parameters that mean
    /// nothing to Serval. They are not sent here.
    ///
    /// UPSTREAM DEFECT (serval_http.cpp:77-87): C attaches HTTP Basic auth to
    /// its GETs but not to `getJson`/`putJson`, so on a Serval that enforces
    /// auth every PUT (and the measurement-config GET) would 401. Auth is
    /// applied uniformly here.
    ///
    /// UPSTREAM DEFECT (serval_http.cpp:185, 569, 1490, 2199, 2254): several C
    /// calls pass no timeout at all and can park the calling thread forever.
    /// Every request here carries one.
    pub fn get(&self, path: &str, timeout: Duration) -> HttpResult<String> {
        let url = format!("{}{path}", self.base);
        let mut resp = self
            .agent
            .get(&url)
            .header("Accept", "application/json")
            .header("Authorization", basic_auth_header())
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

    /// GET and parse the body as JSON.
    pub fn get_json(&self, path: &str, timeout: Duration) -> HttpResult<Value> {
        let body = self.get(path, timeout)?;
        serde_json::from_str(&body)
            .map_err(|e| HttpError::Parse(format!("GET {path}: unparseable json: {e}")))
    }

    /// PUT a JSON body, returning the reply body.
    pub fn put_json(&self, path: &str, body: &Value, timeout: Duration) -> HttpResult<String> {
        let url = format!("{}{path}", self.base);
        let text = body.to_string();
        let mut resp = self
            .agent
            .put(&url)
            .header("Content-Type", "application/json")
            .header("Authorization", basic_auth_header())
            .config()
            .timeout_global(Some(timeout))
            .build()
            .send(&text)
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

/// C sends Basic auth as a header built by cpr; `ureq` needs it spelled out.
/// (Kept as a free function so the encoding is testable.)
pub fn basic_auth_header() -> String {
    format!(
        "Basic {}",
        encode_base64(format!("{USER}:{PASS}").as_bytes())
    )
}

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
        for case in ["", "f", "fo", "foo", "foob", "fooba", "foobar"] {
            let encoded = encode_base64(case.as_bytes());
            assert_eq!(
                decode_base64(&encoded).unwrap(),
                case.as_bytes(),
                "round trip of {case:?} via {encoded:?}"
            );
        }
        // The RFC 4648 vectors.
        assert_eq!(encode_base64(b"foobar"), "Zm9vYmFy");
        assert_eq!(encode_base64(b"fo"), "Zm8=");
    }

    #[test]
    fn base64_rejects_a_nul_instead_of_decoding_it() {
        // C's strchr(kChars, '\0') matches the terminating NUL and yields
        // index 64, silently corrupting the decode (serval_http.cpp:49).
        assert_eq!(decode_base64("AA\0A"), None);
        assert_eq!(decode_base64("!!!!"), None);
    }

    #[test]
    fn base64_decodes_a_padded_payload() {
        assert_eq!(decode_base64("AAAA").unwrap(), vec![0, 0, 0]);
        assert_eq!(decode_base64("/w==").unwrap(), vec![0xff]);
        // Whitespace between groups is skipped, as in a wrapped payload.
        assert_eq!(decode_base64("Zm9v\nYmFy").unwrap(), b"foobar");
    }

    #[test]
    fn the_base_url_loses_a_trailing_slash() {
        assert_eq!(
            ServalHttp::new("http://localhost:8081/").base_url(),
            "http://localhost:8081"
        );
    }

    #[test]
    fn basic_auth_is_the_c_credential_pair() {
        assert_eq!(basic_auth_header(), "Basic dXNlcjpwYXNz");
    }
}
