//! UR dashboard server client (TCP 29999, line-oriented text).
//!
//! Ported from `ur_rtde/src/dashboard_client.cpp`. Every command is a single
//! `\n`-terminated line and the server answers with exactly one line. The server
//! also greets a new connection with a banner line, which `connect()` consumes.
//!
//! Only the commands urRobot's dashboard driver actually issues are ported.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Duration;

use crate::error::{UrError, UrResult};

/// Dashboard server port (`DashboardClient`'s `port = 29999` default).
pub const DASHBOARD_PORT: u16 = 29999;

/// PolyScope version as reported by the `PolyscopeVersion` command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PolyScopeVersion {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
    pub build: u32,
}

impl PolyScopeVersion {
    /// `isInRemoteControl` and `getSerialNumber` are only available from
    /// PolyScope 5.6 (dashboard_client.cpp:324).
    pub fn supports_remote_control_query(&self) -> bool {
        self.major == 5 && self.minor > 5
    }
}

/// Pull the first `d+.d+.d+.d+` out of the reply
/// (`PolyScopeVersion::parse`, dashboard_enums.cpp:191).
pub fn parse_polyscope_version(text: &str) -> Option<PolyScopeVersion> {
    let bytes = text.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if !bytes[i].is_ascii_digit() {
            i += 1;
            continue;
        }
        let mut nums = Vec::new();
        let mut j = i;
        for _ in 0..4 {
            let start = j;
            while j < bytes.len() && bytes[j].is_ascii_digit() {
                j += 1;
            }
            if j == start {
                break;
            }
            let Ok(n) = text[start..j].parse::<u32>() else {
                break;
            };
            nums.push(n);
            if nums.len() == 4 {
                break;
            }
            if j < bytes.len() && bytes[j] == b'.' {
                j += 1;
            } else {
                break;
            }
        }
        if nums.len() == 4 {
            return Some(PolyScopeVersion {
                major: nums[0],
                minor: nums[1],
                patch: nums[2],
                build: nums[3],
            });
        }
        // Not a 4-part version; resume scanning past this run of digits.
        i = j.max(i + 1);
    }
    None
}

/// Client for the dashboard server.
pub struct DashboardClient {
    hostname: String,
    port: u16,
    timeout: Duration,
    io: Option<BufReader<TcpStream>>,
}

impl DashboardClient {
    pub fn new(hostname: &str, timeout: Duration) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: DASHBOARD_PORT,
            timeout,
            io: None,
        }
    }

    pub fn is_connected(&self) -> bool {
        self.io.is_some()
    }

    pub fn connect(&mut self) -> UrResult<()> {
        let sock = crate::net::connect_tcp(
            &self.hostname,
            self.port,
            Some(crate::net::CONNECT_TIMEOUT),
        )
        .map_err(|_| UrError::Connect("Timeout connecting to UR dashboard server.".into()))?;
        sock.set_nodelay(true).ok();
        sock.set_read_timeout(Some(self.timeout)).ok();
        sock.set_write_timeout(Some(self.timeout)).ok();
        self.io = Some(BufReader::new(sock));

        // The server greets the connection; drop the banner (dashboard_client.cpp:66).
        self.receive()?;
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.io = None;
    }

    fn send(&mut self, line: &str) -> UrResult<()> {
        let io = self
            .io
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("dashboard".into()))?;
        io.get_mut()
            .write_all(line.as_bytes())
            .map_err(|e| UrError::Io(format!("dashboard write: {e}")))?;
        io.get_mut()
            .flush()
            .map_err(|e| UrError::Io(format!("dashboard flush: {e}")))
    }

    fn receive(&mut self) -> UrResult<String> {
        let io = self
            .io
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("dashboard".into()))?;
        let mut line = String::new();
        let n = io
            .read_line(&mut line)
            .map_err(|e| UrError::Io(format!("dashboard read: {e}")))?;
        if n == 0 {
            return Err(UrError::Io("dashboard connection closed by peer".into()));
        }
        // Strip the newline (and a CR if the server sent one).
        Ok(line.trim_end_matches(['\r', '\n']).to_string())
    }

    /// Send one command line and return the server's single-line reply.
    fn command(&mut self, command: &str) -> UrResult<String> {
        self.send(&format!("{command}\n"))?;
        self.receive()
    }

    /// Send a command whose reply must equal `expect`; otherwise the reply
    /// is the error, as upstream throws it.
    fn command_expect(&mut self, command: &str, expect: &str) -> UrResult<()> {
        let reply = self.command(command)?;
        if reply != expect {
            return Err(UrError::Dashboard(reply));
        }
        Ok(())
    }

    // --- commands used by urRobot's dashboard driver ---

    pub fn load_urp(&mut self, name: &str) -> UrResult<()> {
        let reply = self.command(&format!("load {name}"))?;
        if !reply.contains("Loading program:") {
            return Err(UrError::Dashboard(reply));
        }
        Ok(())
    }

    pub fn play(&mut self) -> UrResult<()> {
        self.command_expect("play", "Starting program")
    }

    pub fn stop(&mut self) -> UrResult<()> {
        self.command_expect("stop", "Stopped")
    }

    pub fn pause(&mut self) -> UrResult<()> {
        self.command_expect("pause", "Pausing program")
    }

    pub fn shutdown(&mut self) -> UrResult<()> {
        self.command("shutdown").map(|_| ())
    }

    pub fn power_on(&mut self) -> UrResult<()> {
        self.command("power on").map(|_| ())
    }

    pub fn power_off(&mut self) -> UrResult<()> {
        self.command("power off").map(|_| ())
    }

    pub fn brake_release(&mut self) -> UrResult<()> {
        self.command("brake release").map(|_| ())
    }

    pub fn unlock_protective_stop(&mut self) -> UrResult<()> {
        self.command_expect("unlock protective stop", "Protective stop releasing")
            .map_err(|e| match e {
                UrError::Dashboard(reply) => {
                    UrError::Dashboard(format!("Unlock protective stop failure: {reply}"))
                }
                e => e,
            })
    }

    pub fn close_popup(&mut self) -> UrResult<()> {
        self.command("close popup").map(|_| ())
    }

    pub fn close_safety_popup(&mut self) -> UrResult<()> {
        self.command("close safety popup").map(|_| ())
    }

    pub fn restart_safety(&mut self) -> UrResult<()> {
        self.command("restart safety").map(|_| ())
    }

    pub fn popup(&mut self, message: &str) -> UrResult<()> {
        self.command(&format!("popup {message}")).map(|_| ())
    }

    /// `running` — the reply is matched case-insensitively (dashboard_client.cpp:157).
    pub fn running(&mut self) -> UrResult<bool> {
        let reply = self.command("running")?;
        Ok(reply.to_ascii_lowercase().contains("true"))
    }

    pub fn program_state(&mut self) -> UrResult<String> {
        self.command("programState")
    }

    pub fn robot_mode(&mut self) -> UrResult<String> {
        self.command("robotmode")
    }

    pub fn safety_status(&mut self) -> UrResult<String> {
        self.command("safetystatus")
    }

    pub fn loaded_program(&mut self) -> UrResult<String> {
        self.command("get loaded program")
    }

    pub fn robot_model(&mut self) -> UrResult<String> {
        self.command("get robot model")
    }

    /// `isProgramSaved` — upstream matches `"True"` case-**sensitively** here even
    /// though `running` lowercases first; the dashboard replies `"True <name>"`,
    /// so the capital is what the server actually sends and it is kept.
    pub fn is_program_saved(&mut self) -> UrResult<bool> {
        let reply = self.command("isProgramSaved")?;
        Ok(reply.contains("True"))
    }

    /// `PolyscopeVersion`, reduced to the version numbers.
    pub fn polyscope_version(&mut self) -> UrResult<PolyScopeVersion> {
        let reply = self.command("PolyscopeVersion")?;
        parse_polyscope_version(&reply)
            .ok_or_else(|| UrError::Dashboard("PolyScopeVersion::parse failed".into()))
    }

    /// `is in remote control` — only supported from PolyScope 5.6. Like
    /// upstream it asks for the version on every call
    /// (dashboard_client.cpp:323), so a dashboard that first connects after
    /// boot still answers from the live version.
    pub fn is_in_remote_control(&mut self) -> UrResult<bool> {
        if !self.polyscope_version()?.supports_remote_control_query() {
            eprintln!(
                "Warning! isInRemoteControl() function is not supported on the dashboard \
                 server for PolyScope versions less than 5.6.0"
            );
            return Ok(false);
        }
        let reply = self.command("is in remote control")?;
        Ok(reply.contains("true"))
    }

    /// `get serial number` — only supported from PolyScope 5.6; asks for the
    /// version first, as upstream (dashboard_client.cpp:392).
    pub fn serial_number(&mut self) -> UrResult<String> {
        if !self.polyscope_version()?.supports_remote_control_query() {
            return Err(UrError::Dashboard(
                "getSerialNumber() function is not supported on the dashboard server for \
                 PolyScope versions less than 5.6.0"
                    .into(),
            ));
        }
        let reply = self.command("get serial number")?;
        if reply.chars().all(|c| c.is_ascii_digit()) && !reply.is_empty() {
            Ok(reply)
        } else {
            Err(UrError::Dashboard(format!(
                "getSerialNumber() function did not return a number. The following was \
                 returned: {reply}"
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::BufReader as StdBufReader;
    use std::net::TcpListener;

    /// A dashboard server that greets, then answers each line from `replies`.
    fn spawn_dashboard(replies: Vec<&'static str>) -> (u16, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let jh = std::thread::spawn(move || {
            let (sock, _) = listener.accept().unwrap();
            let mut w = sock.try_clone().unwrap();
            let mut r = StdBufReader::new(sock);
            w.write_all(b"Connected: Universal Robots Dashboard Server\n")
                .unwrap();
            let mut got = Vec::new();
            for reply in replies {
                let mut line = String::new();
                if r.read_line(&mut line).unwrap() == 0 {
                    break;
                }
                got.push(line.trim_end().to_string());
                w.write_all(format!("{reply}\n").as_bytes()).unwrap();
            }
            std::thread::sleep(Duration::from_millis(50));
            got
        });
        (port, jh)
    }

    fn client_on(port: u16) -> DashboardClient {
        let mut c = DashboardClient::new("127.0.0.1", Duration::from_millis(500));
        c.port = port;
        c.connect().unwrap();
        c
    }

    #[test]
    fn version_parse_pulls_four_part_number() {
        assert_eq!(
            parse_polyscope_version("URSoftware 5.11.3.108355 (Aug 09 2021)"),
            Some(PolyScopeVersion {
                major: 5,
                minor: 11,
                patch: 3,
                build: 108355
            })
        );
        assert_eq!(
            parse_polyscope_version("3.15.8.106339"),
            Some(PolyScopeVersion {
                major: 3,
                minor: 15,
                patch: 8,
                build: 106339
            })
        );
        assert_eq!(parse_polyscope_version("no version here"), None);
        // A shorter run of digits must not be mistaken for a version.
        assert_eq!(parse_polyscope_version("5.11.3"), None);
    }

    #[test]
    fn remote_control_query_gated_on_5_6() {
        let v = |major, minor| PolyScopeVersion {
            major,
            minor,
            ..Default::default()
        };
        assert!(!v(5, 5).supports_remote_control_query());
        assert!(v(5, 6).supports_remote_control_query());
        assert!(v(5, 11).supports_remote_control_query());
        assert!(!v(3, 15).supports_remote_control_query());
    }

    #[test]
    fn connect_consumes_the_banner_then_commands_line_up() {
        let (port, jh) = spawn_dashboard(vec!["Robotmode: RUNNING", "STOPPED", "true"]);
        let mut c = client_on(port);

        assert_eq!(c.robot_mode().unwrap(), "Robotmode: RUNNING");
        assert_eq!(c.program_state().unwrap(), "STOPPED");
        assert!(c.running().unwrap());

        let sent = jh.join().unwrap();
        assert_eq!(sent, vec!["robotmode", "programState", "running"]);
    }

    #[test]
    fn play_requires_the_success_line() {
        let (port, jh) = spawn_dashboard(vec!["Starting program"]);
        let mut c = client_on(port);
        assert!(c.play().is_ok());
        jh.join().unwrap();

        let (port, jh) = spawn_dashboard(vec!["Failed to execute: play"]);
        let mut c = client_on(port);
        let err = c.play().unwrap_err();
        assert!(
            matches!(&err, UrError::Dashboard(what) if what == "Failed to execute: play"),
            "got {err:?}"
        );
        jh.join().unwrap();
    }

    #[test]
    fn load_urp_matches_on_prefix() {
        let (port, jh) = spawn_dashboard(vec!["Loading program: /programs/demo.urp"]);
        let mut c = client_on(port);
        assert!(c.load_urp("demo.urp").is_ok());
        assert_eq!(jh.join().unwrap(), vec!["load demo.urp"]);

        let (port, jh) = spawn_dashboard(vec!["File not found: nope.urp"]);
        let mut c = client_on(port);
        assert!(c.load_urp("nope.urp").is_err());
        jh.join().unwrap();
    }

    #[test]
    fn running_is_case_insensitive_but_program_saved_is_not() {
        let (port, jh) = spawn_dashboard(vec!["Program running: True", "True demo.urp"]);
        let mut c = client_on(port);
        assert!(c.running().unwrap());
        assert!(c.is_program_saved().unwrap());
        jh.join().unwrap();

        let (port, jh) = spawn_dashboard(vec!["Program running: false", "False demo.urp"]);
        let mut c = client_on(port);
        assert!(!c.running().unwrap());
        assert!(!c.is_program_saved().unwrap());
        jh.join().unwrap();
    }

    #[test]
    fn remote_control_skips_the_wire_on_old_polyscope() {
        // Only the version is scripted: on CB3 the query must not be sent.
        let (port, jh) = spawn_dashboard(vec!["URSoftware 3.15.8.106339 (Jan 1 2023)"]);
        let mut c = client_on(port);
        assert!(!c.is_in_remote_control().unwrap());
        assert_eq!(jh.join().unwrap(), vec!["PolyscopeVersion"]);
    }

    /// A client that never saw the version (the dashboard was down at boot)
    /// still answers from the live controller.
    #[test]
    fn remote_control_asks_the_live_version() {
        let (port, jh) = spawn_dashboard(vec![
            "URSoftware 5.11.3.108355 (Aug 09 2021)",
            "Remote control: true",
        ]);
        let mut c = client_on(port);
        assert!(c.is_in_remote_control().unwrap());
        assert_eq!(
            jh.join().unwrap(),
            vec!["PolyscopeVersion", "is in remote control"]
        );
    }

    #[test]
    fn serial_number_must_be_numeric() {
        let v511 = "URSoftware 5.11.3.108355 (Aug 09 2021)";
        let (port, jh) = spawn_dashboard(vec![v511, "20195500926"]);
        let mut c = client_on(port);
        assert_eq!(c.serial_number().unwrap(), "20195500926");
        jh.join().unwrap();

        let (port, jh) = spawn_dashboard(vec![v511, "not a number"]);
        let mut c = client_on(port);
        assert!(c.serial_number().is_err());
        jh.join().unwrap();
    }

    #[test]
    fn unlock_protective_stop_checks_the_reply() {
        let (port, jh) = spawn_dashboard(vec!["Protective stop releasing"]);
        let mut c = client_on(port);
        assert!(c.unlock_protective_stop().is_ok());
        assert_eq!(jh.join().unwrap(), vec!["unlock protective stop"]);

        let (port, jh) = spawn_dashboard(vec!["Cannot unlock protective stop until 5s after"]);
        let mut c = client_on(port);
        assert_eq!(
            c.unlock_protective_stop().unwrap_err().to_string(),
            "Unlock protective stop failure: Cannot unlock protective stop until 5s after"
        );
        jh.join().unwrap();
    }
}
