//! The client half of ixwebsocket 11.4.6's opening handshake
//! (`WebSocketTransport::connectToUrl`, `WebSocketHandshake::clientHandshake`),
//! as drvFX4 builds it: no TLS, no zlib. A failure is ixwebsocket's
//! `errorInfo.reason`, the text drvFX4's onError prints.

use epics_rs::base::types::c_parse::atoi;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio_tungstenite::WebSocketStream;
use tokio_tungstenite::tungstenite::handshake::client::generate_key;
use tokio_tungstenite::tungstenite::handshake::derive_accept_key;
use tokio_tungstenite::tungstenite::protocol::Role;

/// `connectToUrl`'s `maxRedirections`.
const MAX_REDIRECTIONS: usize = 10;

/// `userAgent()` without `IXWEBSOCKET_USE_TLS` / `IXWEBSOCKET_USE_ZLIB`.
const USER_AGENT: &str = "ixwebsocket/11.4.6 linux nossl";

/// `WebSocketTransport::connectToUrl`: follow up to ten redirections.
pub(crate) async fn connect_to_url(url: &str) -> Result<WebSocketStream<TcpStream>, String> {
    let mut remote = url.to_string();
    let mut last = Refused::new(0, String::new());
    for _ in 0..MAX_REDIRECTIONS {
        let Some(parsed) = parse_url(&remote) else {
            return Err(format!("Could not parse url: '{url}'"));
        };
        if parsed.protocol == "wss" {
            return Err("TLS support is not enabled on this platform.".into());
        }
        let refused = match client_handshake(&remote, &parsed).await {
            Ok(tcp) => return Ok(WebSocketStream::from_raw_socket(tcp, Role::Client, None).await),
            Err(refused) => refused,
        };
        if (300..400).contains(&refused.status) {
            match header(&refused.headers, "Location") {
                Some(location) => {
                    remote = location.to_string();
                    last = refused;
                    continue;
                }
                None => {
                    return Err(format!(
                        "Missing Location Header for HTTP Redirect response. \
                         Rejecting connection to {url}, status: {}",
                        refused.status
                    ));
                }
            }
        }
        return Err(refused.reason);
    }
    Err(last.reason)
}

/// A `WebSocketInitResult` with `success` false.
struct Refused {
    status: i32,
    reason: String,
    /// Only the not-101 result carries them.
    headers: Headers,
}

impl Refused {
    fn new(status: i32, reason: String) -> Self {
        Self {
            status,
            reason,
            headers: Vec::new(),
        }
    }
}

/// `WebSocketHttpHeaders`: a map keyed case-insensitively.
type Headers = Vec<(String, String)>;

fn header<'a>(headers: &'a Headers, name: &str) -> Option<&'a str> {
    headers
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case(name))
        .map(|(_, v)| v.as_str())
}

/// `CaseInsensitiveLess::cmp`.
fn ci_less(a: &str, b: &str) -> bool {
    a.bytes()
        .map(|c| c.to_ascii_lowercase())
        .lt(b.bytes().map(|c| c.to_ascii_lowercase()))
}

/// `WebSocketHandshake::clientHandshake`; `url` is the URL being tried.
async fn client_handshake(url: &str, u: &Url) -> Result<TcpStream, Refused> {
    let host = &u.host;
    let port = u.port;
    // getaddrinfo's numeric service is strtoul'd and truncated to 16 bits.
    let mut tcp = ix_tcp_connect(host, port as u16).await.map_err(|e| {
        Refused::new(
            0,
            format!("Unable to connect to {host} on port {port}, error: {e}"),
        )
    })?;

    let key = generate_key();
    let bracketed = if host.contains(':') {
        format!("[{host}]")
    } else {
        host.clone()
    };
    let request = format!(
        "GET {path} HTTP/1.1\r\n\
         Host: {bracketed}:{port}\r\n\
         Upgrade: websocket\r\n\
         Connection: Upgrade\r\n\
         Sec-WebSocket-Version: 13\r\n\
         Sec-WebSocket-Key: {key}\r\n\
         User-Agent: {USER_AGENT}\r\n\
         Origin: {protocol}://{bracketed}:{port}\r\n\
         \r\n",
        path = u.path,
        protocol = u.protocol,
    );
    if tcp.write_all(request.as_bytes()).await.is_err() {
        return Err(Refused::new(
            0,
            format!("Failed sending GET request to {url}"),
        ));
    }

    let Some(line) = read_line(&mut tcp).await else {
        return Err(Refused::new(
            0,
            format!("Failed reading HTTP status line from {url}"),
        ));
    };
    let (version, status) = parse_status_line(&line);
    if version != "HTTP/1.1" {
        return Err(Refused::new(
            status,
            format!(
                "Expecting HTTP/1.1, got {version}. Rejecting connection to {url}, \
                 status: {status}, HTTP Status line: {line}"
            ),
        ));
    }
    let Some(headers) = parse_http_headers(&mut tcp).await else {
        return Err(Refused::new(status, "Error parsing HTTP headers".into()));
    };
    if status != 101 {
        return Err(Refused {
            status,
            reason: format!(
                "Expecting status 101 (Switching Protocol), got {status} status \
                 connecting to {url}, HTTP Status line: {line}"
            ),
            headers,
        });
    }
    let Some(connection) = header(&headers, "connection") else {
        return Err(Refused::new(status, "Missing connection value".into()));
    };
    // ixwebsocket's insensitiveStringCompare is `!(a < b)`: any value not
    // ordered before "Upgrade" passes.
    if ci_less(connection, "Upgrade") {
        return Err(Refused::new(
            status,
            format!("Invalid connection value: {connection}"),
        ));
    }
    if header(&headers, "sec-websocket-accept").unwrap_or_default()
        != derive_accept_key(key.as_bytes())
    {
        return Err(Refused::new(
            status,
            "Invalid Sec-WebSocket-Accept value".into(),
        ));
    }
    Ok(tcp)
}

async fn read_byte(tcp: &mut TcpStream) -> Option<u8> {
    tcp.read_u8().await.ok()
}

/// `Socket::readLine`: bytes up to a `\n`, or one byte past a `\r`.
async fn read_line(tcp: &mut TcpStream) -> Option<String> {
    let mut line = Vec::with_capacity(64);
    while line.len() < 2 || (line[line.len() - 2] != b'\r' && line[line.len() - 1] != b'\n') {
        line.push(read_byte(tcp).await?);
    }
    Some(String::from_utf8_lossy(&line).into_owned())
}

/// `Http::parseStatusLine`.
fn parse_status_line(line: &str) -> (String, i32) {
    // std::getline yields no token for an empty tail.
    let mut tokens: Vec<&str> = line.split(' ').collect();
    if tokens.last() == Some(&"") {
        tokens.pop();
    }
    let version = tokens.first().map(|t| http_trim(t)).unwrap_or_default();
    let status = tokens.get(1).map_or(-1, |t| istream_int(&http_trim(t)));
    (version, status)
}

/// `Http::trim`: every space, CR and LF goes, wherever it is.
fn http_trim(s: &str) -> String {
    s.chars()
        .filter(|c| !matches!(c, ' ' | '\n' | '\r'))
        .collect()
}

/// `std::stringstream >> int`: 0 when no digits, saturated on overflow.
fn istream_int(s: &str) -> i32 {
    let s = s.trim_start_matches(|c: char| c.is_ascii_whitespace() || c == '\x0b');
    let sign = usize::from(s.starts_with(['+', '-']));
    let digits = s[sign..].bytes().take_while(u8::is_ascii_digit).count();
    if digits == 0 {
        return 0;
    }
    s[..sign + digits].parse().unwrap_or(if s.starts_with('-') {
        i32::MIN
    } else {
        i32::MAX
    })
}

/// `parseHttpHeaders`: lines of at most 1023 bytes up to a bare CRLF; a
/// line without a colon is skipped.
async fn parse_http_headers(tcp: &mut TcpStream) -> Option<Headers> {
    let mut headers = Headers::new();
    let mut line = [0u8; 1024];
    loop {
        let mut colon = 0;
        let mut i = 0;
        while i < 2 || (i < 1023 && line[i - 2] != b'\r' && line[i - 1] != b'\n') {
            line[i] = read_byte(tcp).await?;
            if line[i] == b':' && colon == 0 {
                colon = i;
            }
            i += 1;
        }
        if line[0] == b'\r' && line[1] == b'\n' {
            break;
        }
        if colon > 0 {
            // std::string(line) stops at a NUL.
            let len = line[..i].iter().position(|&c| c == 0).unwrap_or(i);
            let text = &line[..len];
            let mut start = colon + 1;
            while start < text.len() && text[start] == b' ' {
                start += 1;
            }
            let name = String::from_utf8_lossy(&text[..colon.min(text.len())]).into_owned();
            let value = if start < text.len() {
                let v = &text[start..];
                let end = v
                    .iter()
                    .rposition(|c| !(c.is_ascii_whitespace() || *c == 0x0b))
                    .map_or(0, |p| p + 1);
                String::from_utf8_lossy(&v[..end]).into_owned()
            } else {
                String::new()
            };
            match headers
                .iter_mut()
                .find(|(k, _)| k.eq_ignore_ascii_case(&name))
            {
                Some(entry) => entry.1 = value,
                None => headers.push((name, value)),
            }
        }
    }
    Some(headers)
}

/// `UrlParser::parse`'s results.
#[derive(Debug, PartialEq)]
struct Url {
    protocol: String,
    host: String,
    /// With the query appended.
    path: String,
    port: i32,
}

/// `UrlParser::parse` over `clParseURL::ParseURL`; `None` is a parse error.
fn parse_url(url: &str) -> Option<Url> {
    let b = url.as_bytes();
    let scheme_end = url.find(':')?;
    let scheme = &url[..scheme_end];
    if !scheme
        .bytes()
        .all(|c| c.is_ascii_alphabetic() || matches!(c, b'+' | b'-' | b'.'))
    {
        return None;
    }
    let protocol = scheme.to_ascii_lowercase();
    let mut cur = scheme_end + 1;
    if cur + 1 >= b.len() || b[cur] != b'/' || b[cur + 1] != b'/' {
        return None;
    }
    cur += 2;

    let authority_end = |from: usize| url[from..].find(['/', '?']).map_or(url.len(), |i| from + i);
    let find_from = |from: usize, c: char| url[from..].find(c).map(|i| from + i);
    let end = authority_end(cur);
    if let Some(at) = find_from(cur, '@').filter(|&at| at < end) {
        if find_from(at + 1, '@').is_some_and(|p| p < end) {
            return None;
        }
        cur = at + 1;
    }
    let end = authority_end(cur);
    let authority = &url[cur..end];
    let open = authority.find('[');
    if authority.contains(']') && open.is_none() {
        return None;
    }
    if open.is_some_and(|p| p != 0) {
        return None;
    }
    if open.is_none() && authority.matches(':').count() > 1 {
        return None;
    }

    let (host, port) = if cur < end && b[cur] == b'[' {
        let close = find_from(cur + 1, ']').filter(|&c| c < end)?;
        let host = &url[cur + 1..close];
        if host.is_empty() {
            return None;
        }
        let after = close + 1;
        let port = if after < end {
            if b[after] != b':' || after + 1 == end {
                return None;
            }
            &url[after + 1..end]
        } else {
            ""
        };
        (host, port)
    } else {
        match find_from(cur, ':').filter(|&c| c < end) {
            Some(c) if c + 1 == end => return None,
            Some(c) => (&url[cur..c], &url[c + 1..end]),
            None => (&url[cur..end], ""),
        }
    };
    if host.is_empty() || !port.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }

    // `end` is a '/', a '?' or the end of the URL.
    cur = end;
    let mut path = "";
    let mut query = "";
    if cur < url.len() {
        if b[cur] == b'/' {
            cur += 1;
        }
        match url[cur..].find(['?', '#']).map(|i| cur + i) {
            None => path = &url[cur..],
            Some(path_end) => {
                path = &url[cur..path_end];
                if b[path_end] == b'?' {
                    let rest = &url[path_end + 1..];
                    query = rest.find('#').map_or(rest, |i| &rest[..i]);
                }
            }
        }
    }

    // clParseURL::GetPort: atoi, then 1..=65535 or the scheme's port.
    let n = atoi(port);
    let port = if (1..=65535).contains(&n) {
        n
    } else {
        match protocol.as_str() {
            "ws" | "http" => 80,
            "wss" | "https" => 443,
            _ => -1,
        }
    };
    let mut path = if path.starts_with('/') {
        path.to_string()
    } else {
        format!("/{path}")
    };
    if !query.is_empty() {
        path.push('?');
        path.push_str(query);
    }
    Some(Url {
        protocol,
        host: host.to_string(),
        path,
        port,
    })
}

/// ixwebsocket's `SocketConnect::connect`: each resolved address in turn,
/// failing with the last address's `errMsg` (IXSocketConnect.cpp:32-124).
pub(crate) async fn ix_tcp_connect(host: &str, port: u16) -> Result<TcpStream, String> {
    let addrs = tokio::net::lookup_host((host, port)).await.map_err(|e| {
        // C's errMsg is gai_strerror alone.
        let text = e.to_string();
        text.strip_prefix("failed to lookup address information: ")
            .map_or(text.clone(), str::to_string)
    })?;
    let mut err = String::new();
    for addr in addrs {
        match ix_connect_address(addr).await {
            Ok(stream) => return Ok(stream),
            Err(e) => err = e,
        }
    }
    Err(err)
}

/// `connectToAddress`: a non-blocking connect whose immediate failure is the
/// bare strerror, and whose failure found by poll is `Connect error: ...`.
async fn ix_connect_address(addr: std::net::SocketAddr) -> Result<TcpStream, String> {
    use socket2::{Domain, Protocol, Socket, Type};

    let socket = Socket::new(Domain::for_address(addr), Type::STREAM, Some(Protocol::TCP))
        .map_err(|_| "Cannot create a socket".to_string())?;
    let _ = socket.set_tcp_nodelay(true);
    socket.set_nonblocking(true).map_err(|e| strerror(&e))?;
    match socket.connect(&addr.into()) {
        Ok(()) => {}
        Err(e) if connect_in_progress(&e) => {}
        Err(e) => return Err(strerror(&e)),
    }
    let polled = |e: &std::io::Error| format!("Connect error: {}", strerror(e));
    let stream = TcpStream::from_std(socket.into()).map_err(|e| polled(&e))?;
    stream.writable().await.map_err(|e| polled(&e))?;
    match stream.take_error() {
        Ok(None) => Ok(stream),
        Ok(Some(e)) | Err(e) => Err(polled(&e)),
    }
}

/// `Socket::isWaitNeeded`: the connect continues in the background.
fn connect_in_progress(e: &std::io::Error) -> bool {
    #[cfg(unix)]
    if e.raw_os_error() == Some(libc::EINPROGRESS) {
        return true;
    }
    e.kind() == std::io::ErrorKind::WouldBlock
}

/// C's `strerror(errno)`: the OS text without Rust's ` (os error N)`.
pub(crate) fn strerror(e: &std::io::Error) -> String {
    let text = e.to_string();
    match e.raw_os_error() {
        Some(code) => text
            .strip_suffix(&format!(" (os error {code})"))
            .map_or(text.clone(), str::to_string),
        None => text,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};

    fn rt() -> tokio::runtime::Runtime {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("runtime")
    }

    /// Serve each of `replies` to one connection, after reading its request.
    fn server(replies: Vec<String>) -> u16 {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        std::thread::spawn(move || {
            for reply in replies {
                let (mut sock, _) = listener.accept().expect("accept");
                let mut req = Vec::new();
                let mut byte = [0u8];
                while !req.ends_with(b"\r\n\r\n") && sock.read(&mut byte).unwrap_or(0) == 1 {
                    req.push(byte[0]);
                }
                let _ = sock.write_all(reply.as_bytes());
            }
        });
        port
    }

    fn refusal(replies: &[&str]) -> (u16, String) {
        let port = server(replies.iter().map(|r| r.to_string()).collect());
        let url = format!("ws://127.0.0.1:{port}");
        let Err(reason) = rt().block_on(connect_to_url(&url)) else {
            panic!("the handshake must fail");
        };
        (port, reason)
    }

    #[test]
    fn a_not_101_status_names_the_raw_status_line() {
        let (port, reason) = refusal(&["HTTP/1.1 404 Not Found\r\n\r\n"]);
        assert_eq!(
            reason,
            format!(
                "Expecting status 101 (Switching Protocol), got 404 status connecting to \
                 ws://127.0.0.1:{port}, HTTP Status line: HTTP/1.1 404 Not Found\r\n"
            )
        );
    }

    #[test]
    fn an_http_1_0_reply_is_rejected_before_its_headers() {
        let (port, reason) = refusal(&["HTTP/1.0 200 OK\r\n"]);
        assert_eq!(
            reason,
            format!(
                "Expecting HTTP/1.1, got HTTP/1.0. Rejecting connection to \
                 ws://127.0.0.1:{port}, status: 200, HTTP Status line: HTTP/1.0 200 OK\r\n"
            )
        );
    }

    #[test]
    fn a_closed_socket_fails_the_status_line_or_the_headers() {
        let (port, reason) = refusal(&[""]);
        assert_eq!(
            reason,
            format!("Failed reading HTTP status line from ws://127.0.0.1:{port}")
        );
        let (_, reason) = refusal(&["HTTP/1.1 101 Switching Protocols\r\nConnection: Up"]);
        assert_eq!(reason, "Error parsing HTTP headers");
    }

    #[test]
    fn a_101_is_checked_for_connection_and_accept() {
        let (_, reason) = refusal(&["HTTP/1.1 101 Switching Protocols\r\n\r\n"]);
        assert_eq!(reason, "Missing connection value");
        let (_, reason) = refusal(&["HTTP/1.1 101 OK\r\nconnection:  keep-alive \r\n\r\n"]);
        assert_eq!(reason, "Invalid connection value: keep-alive");
        let (_, reason) = refusal(&["HTTP/1.1 101 OK\r\nConnection: upgrade\r\n\r\n"]);
        assert_eq!(reason, "Invalid Sec-WebSocket-Accept value");
    }

    #[test]
    fn a_redirect_without_location_names_the_first_url() {
        let (port, reason) = refusal(&["HTTP/1.1 301 Moved\r\n\r\n"]);
        assert_eq!(
            reason,
            format!(
                "Missing Location Header for HTTP Redirect response. Rejecting \
                 connection to ws://127.0.0.1:{port}, status: 301"
            )
        );
    }

    #[test]
    fn a_redirect_is_followed_to_its_location() {
        let target = server(vec!["HTTP/1.1 403 Forbidden\r\n\r\n".into()]);
        let (_, reason) = refusal(&[&format!(
            "HTTP/1.1 302 Found\r\nlocation: ws://127.0.0.1:{target}/x?y=1\r\n\r\n"
        )]);
        assert_eq!(
            reason,
            format!(
                "Expecting status 101 (Switching Protocol), got 403 status connecting to \
                 ws://127.0.0.1:{target}/x?y=1, HTTP Status line: HTTP/1.1 403 Forbidden\r\n"
            )
        );
    }

    #[test]
    fn a_tungstenite_server_completes_the_handshake() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        std::thread::spawn(move || {
            let (sock, _) = listener.accept().expect("accept");
            let _ws = tokio_tungstenite::tungstenite::accept(sock).expect("handshake");
            std::thread::sleep(std::time::Duration::from_secs(2));
        });
        rt().block_on(connect_to_url(&format!("ws://127.0.0.1:{port}")))
            .expect("connected");
    }

    #[test]
    fn status_lines_parse_as_ixwebsocket_does() {
        assert_eq!(
            parse_status_line("HTTP/1.1 101 OK\r\n"),
            ("HTTP/1.1".into(), 101)
        );
        assert_eq!(parse_status_line("HTTP/1.1\r\n"), ("HTTP/1.1".into(), -1));
        assert_eq!(
            parse_status_line("HTTP/1.1  101\r\n"),
            ("HTTP/1.1".into(), 0)
        );
        assert_eq!(
            parse_status_line("HTTP/1.1 10x1\r\n"),
            ("HTTP/1.1".into(), 10)
        );
        assert_eq!(parse_status_line("HTTP/1.1\r "), ("HTTP/1.1".into(), -1));
    }

    #[test]
    fn urls_parse_as_ixwebsocket_does() {
        let url = |protocol: &str, host: &str, path: &str, port| Url {
            protocol: protocol.into(),
            host: host.into(),
            path: path.into(),
            port,
        };
        assert_eq!(
            parse_url("ws://10.0.0.5"),
            Some(url("ws", "10.0.0.5", "/", 80))
        );
        assert_eq!(
            parse_url("WS://u:p@h:8080/a/b?q=1#f"),
            Some(url("ws", "h", "/a/b?q=1", 8080))
        );
        assert_eq!(parse_url("ws://[::1]:99"), Some(url("ws", "::1", "/", 99)));
        assert_eq!(parse_url("ws://h:0"), Some(url("ws", "h", "/", 80)));
        assert_eq!(parse_url("wss://h?x"), Some(url("wss", "h", "/?x", 443)));
        for bad in [
            "ws:/h", "ws://", "ws://h:", "ws://h:x", "ws://::1", "ws://a]", "1s://h",
        ] {
            assert_eq!(parse_url(bad), None, "{bad}");
        }
    }
}
