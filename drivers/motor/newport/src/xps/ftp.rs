//! FTP upload of PVT trajectory files to the XPS.
//!
//! C uploads the generated trajectory file into the controller's trajectory
//! directory before verifying/executing it (`XPSController::buildProfile`,
//! XPSController.cpp:709-747). Two transports there: plain FTP for XPS-C/Q, and
//! SFTP (`xpsSFTPUpload`) for XPS-D.
//!
//! This is the plain-FTP path, with the blocking `suppaftp` client; the XPS-D
//! upload is [`super::sftp`]. [`ftp_connect`] and the calls after it are
//! xps_ftp.c itself, which the trajectory SNL programs call.

use std::io::{Cursor, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, TcpStream};

use suppaftp::FtpStream;

/// The XPS FTP control port.
const XPS_FTP_PORT: u16 = 21;

/// Upload `contents` as `file_name` into `directory` on the XPS at `host`,
/// logging in with `user`/`password` (plain FTP, binary mode).
///
/// `host` is the controller IP or hostname without a port. On any FTP step
/// failing, returns a message naming the step (mirrors C, which reports which
/// of connect/cd/store/disconnect failed).
pub fn upload_trajectory(
    host: &str,
    directory: &str,
    file_name: &str,
    contents: &str,
    user: &str,
    password: &str,
) -> Result<(), String> {
    let mut ftp = FtpStream::connect((host, XPS_FTP_PORT))
        .map_err(|e| format!("FTP connect to {host}:{XPS_FTP_PORT} failed: {e}"))?;
    ftp.login(user, password)
        .map_err(|e| format!("FTP login failed: {e}"))?;
    // XPS trajectory files are ASCII, but store binary so no CR/LF translation
    // mangles the line endings the controller parses.
    ftp.transfer_type(suppaftp::types::FileType::Binary)
        .map_err(|e| format!("FTP set binary mode failed: {e}"))?;
    ftp.cwd(directory)
        .map_err(|e| format!("FTP change dir to {directory} failed: {e}"))?;
    let mut reader = Cursor::new(contents.as_bytes());
    ftp.put_file(file_name, &mut reader)
        .map_err(|e| format!("FTP store {file_name} failed: {e}"))?;
    ftp.quit()
        .map_err(|e| format!("FTP disconnect failed: {e}"))?;
    Ok(())
}

/// xps_ftp.h `RETURN_SIZE`: one `recv`.
const RETURN_SIZE: usize = 1500;

/// C `ftpConnect`: the control connection to `ip`:21, logged in, after
/// `PASV` and `TYPE I`; else -1 (no connection), -2 (`USER` refused) or
/// -3 (`PASS` refused). `ip` is dotted-quad, as `inet_addr` reads it.
pub fn ftp_connect(ip: &str, login: &str, password: &str) -> Result<TcpStream, i32> {
    let ip: Ipv4Addr = ip.parse().map_err(|_| -1)?;
    ftp_connect_to(SocketAddr::from((ip, XPS_FTP_PORT)), login, password)
}

/// [`ftp_connect`] to `addr`.
fn ftp_connect_to(addr: SocketAddr, login: &str, password: &str) -> Result<TcpStream, i32> {
    let mut sock = TcpStream::connect(addr).map_err(|_| -1)?;
    // C loops on recv until the banner holds a newline, forever once the
    // server closes; here the close fails the connect.
    let mut banner = Vec::new();
    while !banner.contains(&b'\n') {
        match recv(&mut sock) {
            Some(chunk) if !chunk.is_empty() => banner.extend_from_slice(&chunk),
            _ => return Err(-1),
        }
    }
    if send_ftp_command_and_receive(&mut sock, &format!("USER {login}")).is_none() {
        return Err(-2);
    }
    if send_ftp_command_and_receive(&mut sock, &format!("PASS {password}")).is_none() {
        return Err(-3);
    }
    send_ftp_command_and_receive(&mut sock, "PASV");
    send_ftp_command_and_receive(&mut sock, "TYPE I");
    Ok(sock)
}

/// C `ftpDisconnect`: `close`, with no `QUIT`.
pub fn ftp_disconnect(sock: TcpStream) -> i32 {
    drop(sock);
    0
}

/// C `ftpChangeDir`: `CWD destination`, -1 only when the server answers
/// 530 or 421.
pub fn ftp_change_dir(sock: &mut TcpStream, destination: &str) -> i32 {
    match send_ftp_command_and_receive(sock, &format!("CWD {destination}")) {
        Some(_) => 0,
        None => -1,
    }
}

/// C `ftpStoreFile`: the local file `filename` stored under the same name
/// over a `PASV` data connection, then the transfer reply read; -1 when
/// the data connection, the `STOR` or the local file fails.
pub fn ftp_store_file(sock: &mut TcpStream, filename: &str) -> i32 {
    let Some(mut data) = get_port(sock).and_then(|addr| TcpStream::connect(addr).ok()) else {
        epics_rs::base::stderr_println!(
            "Cound not connect to FTP server to retrieve file {filename}"
        );
        return -1;
    };
    if send_ftp_command_and_receive(sock, &format!("STOR {filename}")).is_none() {
        return -1;
    }
    let Ok(contents) = std::fs::read(filename) else {
        epics_rs::base::stderr_println!(
            "Cound not open file \"{filename}\" for reading on local host"
        );
        return -1;
    };
    let _ = data.write_all(&contents);
    drop(data);
    // "226 Transfer complete."
    recv(sock);
    0
}

/// One C `recv` of up to `RETURN_SIZE` bytes; `None` is its -1.
fn recv(sock: &mut TcpStream) -> Option<Vec<u8>> {
    let mut buf = vec![0; RETURN_SIZE];
    let n = sock.read(&mut buf).ok()?;
    buf.truncate(n);
    Some(buf)
}

/// C `code`: the reply's three-digit code, `atoi` of its first three bytes.
fn code(reply: &[u8]) -> i32 {
    crate::util::atoi(&reply[..reply.len().min(3)])
}

/// C `sendFtpCommandAndReceive`: `command` and a newline, then one reply,
/// read on to its last line when it is multi-line. `None` is C's -1, a
/// 530 or 421 reply; any other code passes.
fn send_ftp_command_and_receive(sock: &mut TcpStream, command: &str) -> Option<Vec<u8>> {
    let _ = sock.write_all(format!("{command}\n").as_bytes());
    // C writes its -1 recv into str_rec[-1]; it reads as an empty reply.
    let mut reply = recv(sock).unwrap_or_default();
    match code(&reply) {
        530 => {
            epics_rs::base::stderr_println!("Connection error");
            return None;
        }
        421 => {
            epics_rs::base::stderr_println!("Connection time out");
            return None;
        }
        _ => {}
    }
    if reply.get(3) == Some(&b'-') {
        let last = [reply[0], reply[1], reply[2], b' '];
        // C searches each recv alone for the code and a space with a byte
        // after them, and spins forever on a close; a close ends it here.
        while !(0..reply.len().saturating_sub(4)).any(|j| reply[j..j + 4] == last) {
            match recv(sock) {
                Some(chunk) if !chunk.is_empty() => reply = chunk,
                _ => break,
            }
        }
    }
    Some(reply)
}

/// C `getPort`: `PASV`, and the data address its `227` reply names.
/// C reads that from a fixed offset 27, the XPS's byte after `(`, and runs
/// off the reply when it is another; that reply here names none.
fn get_port(sock: &mut TcpStream) -> Option<SocketAddr> {
    let reply = send_ftp_command_and_receive(sock, "PASV").unwrap_or_default();
    let reply = String::from_utf8_lossy(&reply);
    let (_, rest) = reply.split_once('(')?;
    let (fields, _) = rest.split_once(')')?;
    let f: Vec<&str> = fields.split(',').collect();
    let [h1, h2, h3, h4, p1, p2] = f[..] else {
        return None;
    };
    let ip: Ipv4Addr = format!("{h1}.{h2}.{h3}.{h4}").parse().ok()?;
    let port = crate::util::atoi(p1) * 256 + crate::util::atoi(p2);
    Some(SocketAddr::from((ip, u16::try_from(port).ok()?)))
}

#[cfg(test)]
mod tests {
    use std::io::{BufRead, BufReader};
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::thread;

    use super::*;

    /// A one-session FTP server on an ephemeral port: the commands it got,
    /// and the stored file's bytes, sent down `log`. `user` is the reply
    /// to `USER`.
    fn server(user: &'static str) -> (SocketAddr, mpsc::Receiver<String>) {
        let control = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = control.local_addr().unwrap();
        let (log, rx) = mpsc::channel();
        thread::spawn(move || {
            let (sock, _) = control.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut lines = BufReader::new(sock);
            w.write_all(b"220-XPS FTP\r\n220 ready\r\n").unwrap();
            let mut data = None;
            let mut line = String::new();
            while lines.read_line(&mut line).unwrap_or(0) > 0 {
                let cmd = line.trim_end().to_string();
                line.clear();
                log.send(cmd.clone()).unwrap();
                let reply = match cmd.split(' ').next().unwrap() {
                    "USER" => user.to_string(),
                    "PASS" => "230 logged in\r\n".into(),
                    "PASV" => {
                        let l = TcpListener::bind("127.0.0.1:0").unwrap();
                        let p = l.local_addr().unwrap().port();
                        data = Some(l);
                        format!(
                            "227 Entering Passive Mode (127,0,0,1,{},{})\r\n",
                            p / 256,
                            p % 256
                        )
                    }
                    "TYPE" => "200 binary\r\n".into(),
                    "CWD" => "550-no such\r\n550 directory\r\n".into(),
                    "STOR" => {
                        w.write_all(b"150 send\r\n").unwrap();
                        let (mut d, _) = data.take().unwrap().accept().unwrap();
                        let mut bytes = String::new();
                        d.read_to_string(&mut bytes).unwrap();
                        log.send(bytes).unwrap();
                        "226 Transfer complete.\r\n".into()
                    }
                    _ => "500 ?\r\n".into(),
                };
                w.write_all(reply.as_bytes()).unwrap();
            }
            log.send("CLOSE".into()).unwrap();
        });
        (addr, rx)
    }

    /// xps_ftp.c: login, `PASV` and `TYPE I`; a 550 `CWD` passes, as only
    /// 530 and 421 fail; `STOR` sends the local file; disconnect closes
    /// without `QUIT`.
    #[test]
    fn the_session_is_xps_ftp_c_one() {
        let dir = std::env::temp_dir().join(format!("xps-ftp-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("traj.trj");
        std::fs::write(&file, "0.5, 1, 2\n").unwrap();
        let file = file.to_str().unwrap();
        let (addr, log) = server("331 password\r\n");
        let mut sock = ftp_connect_to(addr, "Administrator", "pw").unwrap();
        assert_eq!(ftp_change_dir(&mut sock, "/Admin/public/Trajectories"), 0);
        assert_eq!(ftp_store_file(&mut sock, file), 0);
        assert_eq!(ftp_disconnect(sock), 0);
        let log: Vec<String> = log.iter().collect();
        assert_eq!(
            log,
            [
                "USER Administrator",
                "PASS pw",
                "PASV",
                "TYPE I",
                "CWD /Admin/public/Trajectories",
                "PASV",
                &format!("STOR {file}"),
                "0.5, 1, 2\n",
                "CLOSE",
            ]
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ftpConnect's -2 is a 530 or 421 `USER` reply, and -1 no server.
    #[test]
    fn connect_failures_are_c_codes() {
        let (addr, _log) = server("530 not logged in\r\n");
        assert_eq!(ftp_connect_to(addr, "u", "p").unwrap_err(), -2);
        let free = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap();
        assert_eq!(ftp_connect_to(free, "u", "p").unwrap_err(), -1);
        assert_eq!(ftp_connect("xps.local", "u", "p").unwrap_err(), -1);
    }
}
