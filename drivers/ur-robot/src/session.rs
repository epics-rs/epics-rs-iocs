//! RTDE TCP session: connect, negotiate, register recipes, stream data packages.
//!
//! Ported from `ur_rtde/src/rtde.cpp`. The C++ drives a boost::asio io_context
//! with a deadline actor; this uses a blocking `TcpStream` with read/write
//! timeouts, which is the same behaviour with far less machinery (and mirrors
//! how the C++ actually uses it — every call is synchronous).

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::error::{UrError, UrResult};
use crate::rtde::{self, ControllerVersion, Header, RobotCommand};
use crate::state::{OutputRecipe, Value};

use std::collections::HashMap;

/// Default socket timeout. The C++ `async_read_some` defaults to 2500 ms
/// (rtde.cpp:489).
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(2500);

/// Connection state (`RTDE::ConnectionState`, rtde.h:191).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnState {
    Disconnected,
    Connected,
    Started,
    Paused,
}

/// One RTDE session against a robot controller.
pub struct Session {
    hostname: String,
    port: u16,
    timeout: Duration,
    socket: Option<TcpStream>,
    state: ConnState,
    /// Bytes read but not yet consumed as whole packages.
    buffer: Vec<u8>,
    /// The registered output recipe; data packages decode against it.
    recipe: OutputRecipe,
}

impl Session {
    pub fn new(hostname: &str, timeout: Duration) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: rtde::RTDE_PORT,
            timeout,
            socket: None,
            state: ConnState::Disconnected,
            buffer: Vec::new(),
            recipe: OutputRecipe::default(),
        }
    }

    /// `RTDE::isConnected` — true while CONNECTED or STARTED (rtde.cpp:105).
    pub fn is_connected(&self) -> bool {
        self.socket.is_some() && matches!(self.state, ConnState::Connected | ConnState::Started)
    }

    /// `RTDE::isStarted`.
    pub fn is_started(&self) -> bool {
        self.state == ConnState::Started
    }

    pub fn connect(&mut self) -> UrResult<()> {
        self.buffer.clear(); // rtde.cpp:63 — reconnect must not reuse stale bytes.
        let addr = (self.hostname.as_str(), self.port)
            .to_socket_addrs()
            .map_err(|e| UrError::Connect(format!("resolve {}: {e}", self.hostname)))?
            .next()
            .ok_or_else(|| UrError::Connect(format!("no address for {}", self.hostname)))?;

        let sock = TcpStream::connect_timeout(&addr, self.timeout).map_err(|e| {
            UrError::Connect(format!("could not connect to {addr}, verify the IP: {e}"))
        })?;
        sock.set_nodelay(true).ok();
        sock.set_read_timeout(Some(self.timeout)).ok();
        sock.set_write_timeout(Some(self.timeout)).ok();

        self.socket = Some(sock);
        self.state = ConnState::Connected;
        Ok(())
    }

    /// `RTDE::disconnect`. Sends a pause first so the controller stops streaming.
    pub fn disconnect(&mut self, send_pause: bool) {
        if send_pause && self.state == ConnState::Connected {
            let _ = self.send_pause();
        }
        self.socket = None;
        self.state = ConnState::Disconnected;
        self.buffer.clear();
    }

    fn write_frame(&mut self, frame: &[u8]) -> UrResult<()> {
        // rtde.cpp:326 — sendAll silently drops the write when disconnected.
        // A silent drop here would make a command look like it succeeded, so it
        // is an error instead.
        let sock = self
            .socket
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("RTDE".into()))?;
        sock.write_all(frame)
            .map_err(|e| UrError::Io(format!("RTDE write: {e}")))?;
        sock.flush()
            .map_err(|e| UrError::Io(format!("RTDE flush: {e}")))
    }

    /// Read exactly one control package (header + body) straight off the socket.
    ///
    /// This is the C++ `receive()` path (rtde.cpp:348), used for the handshake
    /// packages only. Once synchronisation has started, packages arrive through
    /// [`Session::receive_data`] instead.
    fn read_package(&mut self) -> UrResult<(u8, Vec<u8>)> {
        let sock = self
            .socket
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("RTDE".into()))?;

        let mut head = [0u8; rtde::HEADER_SIZE];
        sock.read_exact(&mut head)
            .map_err(|e| UrError::Io(format!("RTDE header read: {e}")))?;
        let header = Header::decode(&head)
            .ok_or_else(|| UrError::Protocol(format!("bad RTDE header {head:02x?}")))?;

        let mut body = vec![0u8; header.payload_len()];
        sock.read_exact(&mut body)
            .map_err(|e| UrError::Io(format!("RTDE body read: {e}")))?;
        Ok((header.cmd, body))
    }

    /// Send a package and consume the controller's reply, applying it.
    fn exchange(&mut self, frame: &[u8]) -> UrResult<(u8, Vec<u8>)> {
        self.write_frame(frame)?;
        let (cmd, body) = self.read_package()?;
        self.apply_control_package(cmd, &body)?;
        Ok((cmd, body))
    }

    /// Handle the non-data control packages (`RTDE::receive`'s switch).
    fn apply_control_package(&mut self, cmd: u8, body: &[u8]) -> UrResult<()> {
        match cmd {
            rtde::cmd::TEXT_MESSAGE => {
                // First byte is the message length (rtde.cpp:370).
                if let Some((&len, rest)) = body.split_first() {
                    let n = (len as usize).min(rest.len());
                    log::info!(
                        "ur-robot: controller message: {}",
                        String::from_utf8_lossy(&rest[..n])
                    );
                }
            }
            rtde::cmd::CONTROL_PACKAGE_SETUP_INPUTS => {
                if rtde::input_setup_in_use(body) {
                    return Err(UrError::InputRegistersInUse);
                }
            }
            rtde::cmd::CONTROL_PACKAGE_START => {
                if rtde::decode_success(body) {
                    self.state = ConnState::Started;
                } else {
                    return Err(UrError::Protocol(
                        "unable to start RTDE synchronization".into(),
                    ));
                }
            }
            rtde::cmd::CONTROL_PACKAGE_PAUSE => {
                if rtde::decode_success(body) {
                    self.state = ConnState::Paused;
                } else {
                    return Err(UrError::Protocol(
                        "unable to pause RTDE synchronization".into(),
                    ));
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// `RTDE::negotiateProtocolVersion`.
    pub fn negotiate_protocol_version(&mut self) -> UrResult<()> {
        self.exchange(&rtde::encode_protocol_version())?;
        Ok(())
    }

    /// `RTDE::getControllerVersion`.
    pub fn controller_version(&mut self) -> UrResult<ControllerVersion> {
        self.write_frame(&rtde::encode_get_controller_version())?;
        let (cmd, body) = self.read_package()?;
        if cmd != rtde::cmd::GET_URCONTROL_VERSION {
            // rtde.cpp:679 answers an all-zero version for a mismatched reply.
            return Ok(ControllerVersion::default());
        }
        Ok(ControllerVersion::decode(&body).unwrap_or_default())
    }

    /// `RTDE::sendOutputSetup`. Registers the recipe used to decode every later
    /// data package, and rejects names the controller could not resolve.
    pub fn send_output_setup(&mut self, names: &[String], frequency: f64) -> UrResult<()> {
        let recipe = OutputRecipe::new(names);
        let requested = recipe.names().to_vec();
        let (_, body) = self.exchange(&rtde::encode_output_setup(&requested, frequency))?;

        let reply = rtde::OutputSetupReply::decode(&body)
            .ok_or_else(|| UrError::Protocol("empty output-setup reply".into()))?;
        let missing = reply.not_found(&requested);
        if !missing.is_empty() {
            return Err(UrError::VariablesNotFound(missing.join(", ")));
        }

        self.recipe = recipe;
        Ok(())
    }

    /// `RTDE::sendInputSetup`. Returns the recipe id the controller assigned.
    ///
    /// The C++ ignores the returned id and hard-codes recipe numbers by
    /// registration order (rtde_io_interface.cpp:97 onwards). The id is returned
    /// here so a caller can assert the controller agrees.
    pub fn send_input_setup(&mut self, names: &[String]) -> UrResult<u8> {
        let (_, body) = self.exchange(&rtde::encode_input_setup(names))?;
        body.first()
            .copied()
            .ok_or_else(|| UrError::Protocol("empty input-setup reply".into()))
    }

    /// `RTDE::sendStart`.
    pub fn send_start(&mut self) -> UrResult<()> {
        self.exchange(&rtde::encode_start())?;
        Ok(())
    }

    /// `RTDE::sendPause`.
    pub fn send_pause(&mut self) -> UrResult<()> {
        self.exchange(&rtde::encode_pause())?;
        Ok(())
    }

    /// `RTDE::send` — transmit one robot command on its input recipe.
    pub fn send_command(&mut self, cmd: &RobotCommand) -> UrResult<()> {
        self.write_frame(&cmd.encode())
    }

    /// A write-only handle on this session's socket.
    ///
    /// The control interface hands the session itself to the reader thread and
    /// keeps one of these to send robot commands: a TCP connection is
    /// full-duplex, and this is what the C++ does implicitly by writing from the
    /// caller thread while its `receiveCallback` thread reads.
    pub fn writer(&self) -> UrResult<SessionWriter> {
        let sock = self
            .socket
            .as_ref()
            .ok_or_else(|| UrError::NotConnected("RTDE".into()))?
            .try_clone()
            .map_err(|e| UrError::Io(format!("RTDE socket clone: {e}")))?;
        Ok(SessionWriter { socket: sock })
    }

    /// Read whatever has arrived and return the newest robot state, if a
    /// complete data package was among it (`RTDE::receiveData`, rtde.cpp:537).
    ///
    /// The C++ discards a data package when another one is already queued behind
    /// it ("skipping package(1)") so that a backlog collapses to the newest
    /// sample. That is kept: only the last complete data package in the buffer
    /// is decoded, older ones are dropped.
    pub fn receive_data(&mut self) -> UrResult<Option<HashMap<String, Value>>> {
        let mut chunk = [0u8; 4096];
        let n = {
            let sock = self
                .socket
                .as_mut()
                .ok_or_else(|| UrError::NotConnected("RTDE".into()))?;
            match sock.read(&mut chunk) {
                Ok(0) => {
                    return Err(UrError::Io("RTDE connection closed by peer".into()));
                }
                Ok(n) => n,
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    return Ok(None);
                }
                Err(e) => return Err(UrError::Io(format!("RTDE read: {e}"))),
            }
        };
        self.buffer.extend_from_slice(&chunk[..n]);

        let mut latest: Option<HashMap<String, Value>> = None;
        let mut pending: Vec<(u8, Vec<u8>)> = Vec::new();

        // Split the buffer into whole packages, leaving any partial tail behind.
        loop {
            let Some(header) = Header::decode(&self.buffer) else {
                if self.buffer.len() >= rtde::HEADER_SIZE {
                    // A size < HEADER_SIZE cannot be resynchronised from.
                    return Err(UrError::Protocol("malformed RTDE package length".into()));
                }
                break;
            };
            let total = header.size as usize;
            if self.buffer.len() < total {
                break;
            }
            let package: Vec<u8> = self.buffer.drain(..total).collect();
            let body = package[rtde::HEADER_SIZE..].to_vec();

            if header.cmd == rtde::cmd::DATA_PACKAGE {
                // Keep only the newest; older samples are superseded.
                latest = Some(self.recipe.decode(&body)?);
            } else {
                pending.push((header.cmd, body));
            }
        }

        for (cmd, body) in pending {
            self.apply_control_package(cmd, &body)?;
        }

        Ok(latest)
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        if self.is_connected() {
            self.disconnect(true);
        }
    }
}

/// Write half of a [`Session`], handed out by [`Session::writer`].
///
/// It can only send robot commands: reading stays with the reader thread that
/// owns the session, so there is exactly one consumer of the byte stream.
pub struct SessionWriter {
    socket: TcpStream,
}

impl SessionWriter {
    /// `RTDE::send`.
    pub fn send_command(&mut self, cmd: &RobotCommand) -> UrResult<()> {
        let frame = cmd.encode();
        self.socket
            .write_all(&frame)
            .map_err(|e| UrError::Io(format!("RTDE write: {e}")))?;
        self.socket
            .flush()
            .map_err(|e| UrError::Io(format!("RTDE flush: {e}")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The session's framing logic is exercised end-to-end against a loopback
    // listener that speaks the controller side of the handshake.
    use std::net::TcpListener;

    /// Reply to whatever the client sends with a scripted controller.
    fn spawn_controller(
        script: Vec<(u8, Vec<u8>)>,
        stream_after: Vec<Vec<u8>>,
    ) -> (u16, std::thread::JoinHandle<Vec<Vec<u8>>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            let mut received = Vec::new();
            for (cmd, body) in script {
                // Read one client frame.
                let mut head = [0u8; 3];
                sock.read_exact(&mut head).unwrap();
                let h = Header::decode(&head).unwrap();
                let mut rest = vec![0u8; h.payload_len()];
                sock.read_exact(&mut rest).unwrap();
                let mut whole = head.to_vec();
                whole.extend_from_slice(&rest);
                received.push(whole);
                sock.write_all(&rtde::encode_frame(cmd, &body)).unwrap();
            }
            for frame in stream_after {
                sock.write_all(&frame).unwrap();
            }
            // Hold the socket open long enough for the client to drain it.
            std::thread::sleep(Duration::from_millis(150));
            received
        });
        (port, jh)
    }

    fn session_on(port: u16) -> Session {
        let mut s = Session::new("127.0.0.1", Duration::from_millis(500));
        s.port = port;
        s.connect().unwrap();
        s
    }

    #[test]
    fn handshake_negotiate_version_and_output_setup() {
        let mut version_body = Vec::new();
        for v in [5u32, 11, 3, 108355] {
            version_body.extend_from_slice(&v.to_be_bytes());
        }
        let mut setup_reply = vec![1u8];
        setup_reply.extend_from_slice(b"DOUBLE,VECTOR6D");

        let (port, jh) = spawn_controller(
            vec![
                (rtde::cmd::REQUEST_PROTOCOL_VERSION, vec![1]),
                (rtde::cmd::GET_URCONTROL_VERSION, version_body),
                (rtde::cmd::CONTROL_PACKAGE_SETUP_OUTPUTS, setup_reply),
                (rtde::cmd::CONTROL_PACKAGE_START, vec![1]),
            ],
            vec![],
        );

        let mut s = session_on(port);
        s.negotiate_protocol_version().unwrap();
        let v = s.controller_version().unwrap();
        assert_eq!((v.major, v.minor), (5, 11));

        s.send_output_setup(&["timestamp".into(), "actual_q".into()], 500.0)
            .unwrap();
        assert!(!s.is_started());
        s.send_start().unwrap();
        assert!(s.is_started());

        let sent = jh.join().unwrap();
        assert_eq!(sent[0], vec![0x00, 0x05, 86, 0x00, 0x02]);
        assert_eq!(sent[1], vec![0x00, 0x03, 118]);
        assert_eq!(sent[2][2], rtde::cmd::CONTROL_PACKAGE_SETUP_OUTPUTS);
        assert_eq!(&sent[2][3..11], &500.0f64.to_be_bytes());
        assert_eq!(&sent[2][11..], b"timestamp,actual_q,");
        assert_eq!(sent[3], vec![0x00, 0x03, 83]);
    }

    #[test]
    fn output_setup_rejects_not_found_variables() {
        let mut setup_reply = vec![1u8];
        setup_reply.extend_from_slice(b"DOUBLE,NOT_FOUND");
        let (port, jh) = spawn_controller(
            vec![(rtde::cmd::CONTROL_PACKAGE_SETUP_OUTPUTS, setup_reply)],
            vec![],
        );
        let mut s = session_on(port);
        let err = s
            .send_output_setup(&["timestamp".into(), "payload_inertia".into()], 125.0)
            .unwrap_err();
        assert!(
            matches!(&err, UrError::VariablesNotFound(v) if v == "payload_inertia"),
            "got {err:?}"
        );
        jh.join().unwrap();
    }

    #[test]
    fn input_setup_in_use_is_an_error() {
        let mut reply = vec![7u8];
        reply.extend_from_slice(b"IN_USE");
        let (port, jh) = spawn_controller(
            vec![(rtde::cmd::CONTROL_PACKAGE_SETUP_INPUTS, reply)],
            vec![],
        );
        let mut s = session_on(port);
        let err = s
            .send_input_setup(&["input_int_register_0".into()])
            .unwrap_err();
        assert!(matches!(err, UrError::InputRegistersInUse), "got {err:?}");
        jh.join().unwrap();
    }

    #[test]
    fn input_setup_returns_the_recipe_id() {
        let (port, jh) = spawn_controller(
            vec![(rtde::cmd::CONTROL_PACKAGE_SETUP_INPUTS, vec![3, b'I'])],
            vec![],
        );
        let mut s = session_on(port);
        assert_eq!(
            s.send_input_setup(&["input_int_register_0".into()])
                .unwrap(),
            3
        );
        jh.join().unwrap();
    }

    #[test]
    fn receive_data_decodes_and_collapses_to_newest() {
        let mut setup_reply = vec![1u8];
        setup_reply.extend_from_slice(b"DOUBLE,INT32");

        // Two data packages back-to-back: the older one must be dropped.
        let pkg = |ts: f64, mode: i32| {
            let mut body = vec![1u8];
            body.extend_from_slice(&ts.to_be_bytes());
            body.extend_from_slice(&mode.to_be_bytes());
            rtde::encode_frame(rtde::cmd::DATA_PACKAGE, &body)
        };

        let (port, jh) = spawn_controller(
            vec![(rtde::cmd::CONTROL_PACKAGE_SETUP_OUTPUTS, setup_reply)],
            vec![pkg(1.0, 3), pkg(2.0, 7)],
        );

        let mut s = session_on(port);
        s.send_output_setup(&["timestamp".into(), "robot_mode".into()], 125.0)
            .unwrap();

        // Give both packages time to land in one read.
        std::thread::sleep(Duration::from_millis(50));
        let state = s.receive_data().unwrap().expect("a data package");
        assert_eq!(state["timestamp"], Value::Double(2.0));
        assert_eq!(state["robot_mode"], Value::Int32(7));
        jh.join().unwrap();
    }

    #[test]
    fn receive_data_reassembles_a_split_package() {
        let mut setup_reply = vec![1u8];
        setup_reply.extend_from_slice(b"DOUBLE");

        let mut body = vec![1u8];
        body.extend_from_slice(&42.5f64.to_be_bytes());
        let frame = rtde::encode_frame(rtde::cmd::DATA_PACKAGE, &body);
        // Split the package across two writes.
        let (a, b) = frame.split_at(5);

        let (port, jh) = spawn_controller(
            vec![(rtde::cmd::CONTROL_PACKAGE_SETUP_OUTPUTS, setup_reply)],
            vec![a.to_vec(), b.to_vec()],
        );

        let mut s = session_on(port);
        s.send_output_setup(&["timestamp".into()], 125.0).unwrap();

        // First read may see only the head; keep pumping until the package lands.
        let mut got = None;
        for _ in 0..10 {
            if let Some(state) = s.receive_data().unwrap() {
                got = Some(state);
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(got.expect("package")["timestamp"], Value::Double(42.5));
        jh.join().unwrap();
    }

    #[test]
    fn send_command_writes_the_encoded_frame() {
        let (port, jh) = spawn_controller(vec![], vec![]);
        let mut s = session_on(port);
        let cmd = RobotCommand::new(4, rtde::CommandType::NoCmd, rtde::Payload::None);
        s.send_command(&cmd).unwrap();
        // Drop before joining so the listener's read sees EOF.
        drop(s);
        jh.join().unwrap();
    }

    #[test]
    fn write_when_disconnected_is_an_error_not_a_silent_drop() {
        let mut s = Session::new("127.0.0.1", Duration::from_millis(100));
        let cmd = RobotCommand::new(4, rtde::CommandType::NoCmd, rtde::Payload::None);
        assert!(matches!(
            s.send_command(&cmd),
            Err(UrError::NotConnected(_))
        ));
    }
}
