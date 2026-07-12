//! URScript client: the control-script transform and the script server socket.
//!
//! Ported from `ur_rtde/src/script_client.cpp`. The control script itself
//! (`rtde_control_script.h`, `UR_SCRIPT`) is uploaded to the robot's *secondary*
//! script server on TCP 30003; the robot compiles and runs it, and it is that
//! script which reads the RTDE input registers this driver writes.
//!
//! Two text transforms run over the script before it goes on the wire:
//!
//! 1. **Version gating.** Lines that need a minimum PolyScope version carry a
//!    `$M.mm` marker (or `$M.mm|M.mm` naming an additionally-supported CB
//!    version). A line whose requirement the controller does not meet is
//!    deleted; a line that is kept has its marker blanked to spaces. The blanking
//!    width is fixed at 5 (or 10) characters rather than the marker's own length,
//!    which is what preserves the script's indentation — so it is reproduced
//!    exactly.
//! 2. **Injection.** Named anchors in the script (`# float register offset`,
//!    `# int register offset`, `# inject move path`) get text spliced in
//!    immediately after them.

use std::io::Write;
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::error::{UrError, UrResult};

/// Script server port (`ScriptClient`'s `port = 30003` default, script_client.h:28).
pub const SCRIPT_PORT: u16 = 30003;

/// The compiled-in RTDE control script (`UR_SCRIPT` in rtde_control_script.h).
///
/// The upstream ships this as a C string literal built by a generator. It is
/// vendored here verbatim as a text file next to the driver.
pub const UR_SCRIPT: &str = include_str!("rtde_control.script");

/// Anchor for the float register-offset injection.
pub const INJECT_FLOAT_OFFSET: &str = "# float register offset\n";
/// Anchor for the int register-offset injection.
pub const INJECT_INT_OFFSET: &str = "# int register offset\n";

/// A `(search, inject)` pair: `inject` is spliced in directly after `search`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Injection {
    pub search: String,
    pub inject: String,
}

/// Strip version-gated lines the controller is too old for, and blank the marker
/// on the lines that survive (`ScriptClient::removeUnsupportedFunctions`).
///
/// Upstream parses the marker with a sloppy fixed-width slice
/// (`version_str.substr(2, 4)`) and leans on `std::stoi` stopping at the first
/// non-digit; this parses the marker properly instead, which yields the same
/// numbers without depending on that.
///
/// **Upstream defect fixed here.** The PolyScopeX guard at script_client.cpp:143
/// tests `minor_version_needed == 22`, but every direct-torque line in the
/// shipped script is marked `$5.23` and no line anywhere carries `$5.22`. The
/// guard therefore never fires, and a PolyScopeX controller older than 10.9 is
/// sent direct-torque script code its comment says must be removed
/// ("Direct torque control was introduced in PolyScopeX 10.9.0. Remove if the
/// version is below."). The threshold is 23 here.
pub fn remove_unsupported_functions(script: &str, major: u32, minor: u32) -> UrResult<String> {
    /// Version-gated direct-torque lines are marked `$5.23`; PolyScopeX gained
    /// direct torque control in 10.9.
    const DIRECT_TORQUE_MARKER: (u32, u32) = (5, 23);
    const POLYSCOPE_X_DIRECT_TORQUE_MINOR: u32 = 9;

    let mut out = String::with_capacity(script.len());
    let mut rest = script;

    loop {
        let Some(pos) = rest.find('$') else {
            out.push_str(rest);
            return Ok(out);
        };
        out.push_str(&rest[..pos]);
        let marker_area = &rest[pos..];

        let Some(gate) = Gate::parse(marker_area) else {
            return Err(UrError::Script(
                "could not read the control version required from the control script".into(),
            ));
        };

        // Upstream predicate (script_client.cpp:138): the controller satisfies
        // the requirement if it is a newer major, the same major at a
        // high-enough minor, or matches the additionally-named version.
        let supported = major > gate.major
            || (major == gate.major && minor >= gate.minor)
            || gate
                .extra
                .is_some_and(|(em, en)| major == em && minor >= en);

        let strip_for_polyscope_x = major == 10
            && minor < POLYSCOPE_X_DIRECT_TORQUE_MINOR
            && (gate.major, gate.minor) == DIRECT_TORQUE_MARKER;

        if supported && !strip_for_polyscope_x {
            // Keep the line, blanking the marker to preserve indentation.
            out.push_str(&" ".repeat(gate.blank_width));
            rest = &marker_area[gate.blank_width.min(marker_area.len())..];
        } else {
            // Drop the whole line, including its newline.
            rest = match marker_area.find('\n') {
                Some(nl) => &marker_area[nl + 1..],
                None => "",
            };
        }
    }
}

/// A parsed `$M.mm` / `$M.mm|M.mm` version marker.
struct Gate {
    major: u32,
    minor: u32,
    extra: Option<(u32, u32)>,
    /// Characters upstream blanks/erases: 5 for a plain marker, 10 for a dual one.
    blank_width: usize,
}

impl Gate {
    /// `text` starts at the `$`.
    fn parse(text: &str) -> Option<Self> {
        let body = text.strip_prefix('$')?;
        let (major, minor, used) = parse_version(body)?;

        // Upstream reads the separator at a fixed offset (`version_str.at(4)`),
        // i.e. the 5th character after the `$`.
        if body.len() > 4 && body.as_bytes()[4] == b'|' {
            let (em, en, _) = parse_version(&body[5..])?;
            return Some(Self {
                major,
                minor,
                extra: Some((em, en)),
                blank_width: 10,
            });
        }

        // A single marker always costs 5 characters, even when the token itself
        // is shorter (`$5.4` -> erase(n, 5) also takes the trailing pad space).
        let _ = used;
        Some(Self {
            major,
            minor,
            extra: None,
            blank_width: 5,
        })
    }
}

/// Parse a leading `M.mm`, returning `(major, minor, chars_consumed)`.
fn parse_version(s: &str) -> Option<(u32, u32, usize)> {
    let digits = |t: &str| -> (u32, usize) {
        let n = t.bytes().take_while(u8::is_ascii_digit).count();
        (t[..n].parse().unwrap_or(0), n)
    };
    let (major, mlen) = digits(s);
    if mlen == 0 {
        return None;
    }
    let after = &s[mlen..];
    let rest = after.strip_prefix('.')?;
    let (minor, nlen) = digits(rest);
    if nlen == 0 {
        return None;
    }
    Some((major, minor, mlen + 1 + nlen))
}

/// Splice each injection's text in directly after its anchor
/// (`ScriptClient::scanAndInjectAdditionalScriptCode`). An anchor that does not
/// occur is skipped, matching upstream.
pub fn inject(script: &str, injections: &[Injection]) -> String {
    let mut out = script.to_string();
    for inj in injections {
        if let Some(pos) = out.find(&inj.search) {
            out.insert_str(pos + inj.search.len(), &inj.inject);
        } else {
            log::debug!(
                "ur-robot: script injection anchor [{}] not found",
                inj.search.trim_end()
            );
        }
    }
    out
}

/// Build the control script exactly as it goes on the wire.
///
/// `sendScript` wraps the body in `def rtde_control(): ... end`
/// (script_client.cpp:225); `getScript` (used only by the ExternalControl UR Cap
/// path, which urRobot does not use) does not.
pub fn build_control_script(major: u32, minor: u32, injections: &[Injection]) -> UrResult<String> {
    let mut s = String::from("def rtde_control():\n");
    s.push_str(UR_SCRIPT);
    s.push_str("end\n");
    let s = remove_unsupported_functions(&s, major, minor)?;
    Ok(inject(&s, injections))
}

/// Wrap a user URScript so its completion is observable.
///
/// This is urRobot's own `wrap_script` (rtde_control_driver.cpp:102): each line
/// is indented into a `custom_func`, and the last statement bumps output integer
/// register 12, which the receive driver polls to detect that the script ended.
pub fn wrap_custom_script(script: &str) -> String {
    let mut out = String::from("def custom_func():\n");
    for line in script.lines() {
        out.push('\t');
        out.push_str(line);
        out.push('\n');
    }
    out.push_str("\twrite_output_integer_register(12, read_output_integer_register(12)+1)\n");
    out.push_str("end\n");
    out
}

/// TCP connection to the robot's script server.
pub struct ScriptClient {
    hostname: String,
    port: u16,
    timeout: Duration,
    socket: Option<TcpStream>,
}

impl ScriptClient {
    pub fn new(hostname: &str, port: u16, timeout: Duration) -> Self {
        Self {
            hostname: hostname.to_string(),
            port,
            timeout,
            socket: None,
        }
    }

    pub fn connect(&mut self) -> UrResult<()> {
        let addr = (self.hostname.as_str(), self.port)
            .to_socket_addrs()
            .map_err(|e| UrError::Connect(format!("resolve {}: {e}", self.hostname)))?
            .next()
            .ok_or_else(|| UrError::Connect(format!("no address for {}", self.hostname)))?;
        let sock = TcpStream::connect_timeout(&addr, self.timeout)
            .map_err(|e| UrError::Connect(format!("script server {addr}: {e}")))?;
        sock.set_nodelay(true).ok();
        sock.set_write_timeout(Some(self.timeout)).ok();
        self.socket = Some(sock);
        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        self.socket.is_some()
    }

    pub fn disconnect(&mut self) {
        self.socket = None;
    }

    /// Send raw script text. The script server takes text and never replies.
    pub fn send(&mut self, script: &str) -> UrResult<()> {
        let sock = self
            .socket
            .as_mut()
            .ok_or_else(|| UrError::NotConnected("script server".into()))?;
        sock.write_all(script.as_bytes()).map_err(|e| {
            // A failed write leaves the connection unusable; drop it so the next
            // call reconnects rather than writing into a half-open socket.
            UrError::Io(format!("script write: {e}"))
        })?;
        sock.flush()
            .map_err(|e| UrError::Io(format!("script flush: {e}")))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_supported_line_and_blanks_marker_to_five_chars() {
        // "$5.4  while ..." on PolyScope 5.11: kept, marker blanked to 5 spaces.
        let s = "$5.4  while step_back > 0:\n";
        let out = remove_unsupported_functions(s, 5, 11).unwrap();
        assert_eq!(out, "      while step_back > 0:\n");
        // 5 blanks replace "$5.4 " (4-char token + 1 pad), the trailing pad space
        // of the original survives -> indentation is unchanged in width.
        assert_eq!(out.len(), s.len());
    }

    #[test]
    fn drops_line_when_controller_too_old() {
        let s = "keep me\n$5.23  direct_torque()\nkeep me too\n";
        let out = remove_unsupported_functions(s, 5, 11).unwrap();
        assert_eq!(out, "keep me\nkeep me too\n");
    }

    #[test]
    fn five_char_marker_blanks_exactly_five() {
        // "$5.10" is itself 5 characters.
        let s = "$5.10 foo()\n";
        let out = remove_unsupported_functions(s, 5, 11).unwrap();
        assert_eq!(out, "      foo()\n");
        assert_eq!(out.len(), s.len());
    }

    #[test]
    fn dual_version_marker_blanks_ten_and_accepts_either_line() {
        let s = "$5.10|3.15 foo()\n";
        // PolyScope 5.11 satisfies the primary requirement.
        assert_eq!(
            remove_unsupported_functions(s, 5, 11).unwrap(),
            "           foo()\n"
        );
        // CB3 3.15 satisfies the additionally-named one.
        assert_eq!(
            remove_unsupported_functions(s, 3, 15).unwrap(),
            "           foo()\n"
        );
        // CB3 3.14 satisfies neither.
        assert_eq!(remove_unsupported_functions(s, 3, 14).unwrap(), "");
    }

    #[test]
    fn newer_major_satisfies_any_marker() {
        let s = "$3.5  foo()\n";
        assert_eq!(
            remove_unsupported_functions(s, 5, 0).unwrap(),
            "      foo()\n"
        );
    }

    #[test]
    fn polyscope_x_below_10_9_strips_direct_torque_lines() {
        // Upstream defect: the guard tests $5.22, but every direct-torque line in
        // the shipped script is marked $5.23, so on PolyScopeX < 10.9 they were
        // kept -- uploading script code the controller cannot compile.
        let s = "$5.23  direct_torque()\nplain\n";
        assert_eq!(
            remove_unsupported_functions(s, 10, 8).unwrap(),
            "plain\n",
            "PolyScopeX 10.8 must not receive direct-torque script code"
        );
        // 10.9 and later do support it.
        assert_eq!(
            remove_unsupported_functions(s, 10, 9).unwrap(),
            "       direct_torque()\nplain\n"
        );
        // A non-direct-torque marker is unaffected on PolyScopeX 10.8.
        assert_eq!(
            remove_unsupported_functions("$5.4  foo()\n", 10, 8).unwrap(),
            "      foo()\n"
        );
    }

    #[test]
    fn malformed_marker_is_an_error_not_a_panic() {
        assert!(remove_unsupported_functions("$ oops\n", 5, 11).is_err());
        assert!(remove_unsupported_functions("$x.y foo\n", 5, 11).is_err());
    }

    #[test]
    fn injection_splices_after_the_anchor() {
        let script = "a\n# int register offset\nb\n";
        let out = inject(
            script,
            &[Injection {
                search: INJECT_INT_OFFSET.into(),
                inject: "24".into(),
            }],
        );
        assert_eq!(out, "a\n# int register offset\n24b\n");
    }

    #[test]
    fn missing_anchor_is_skipped() {
        let out = inject(
            "nothing here\n",
            &[Injection {
                search: "# absent\n".into(),
                inject: "X".into(),
            }],
        );
        assert_eq!(out, "nothing here\n");
    }

    #[test]
    fn control_script_builds_and_has_no_markers_left() {
        let s = build_control_script(
            5,
            11,
            &[
                Injection {
                    search: INJECT_FLOAT_OFFSET.into(),
                    inject: "0".into(),
                },
                Injection {
                    search: INJECT_INT_OFFSET.into(),
                    inject: "0".into(),
                },
            ],
        )
        .unwrap();
        assert!(s.starts_with("def rtde_control():\n"));
        assert!(s.ends_with("end\n"));
        // Every gate must be resolved: none may survive into the uploaded script.
        assert!(!s.contains('$'), "unresolved version marker in script");
        // The register offsets landed.
        assert!(s.contains("global reg_offset_int = # int register offset\n0"));
        assert!(s.contains("global reg_offset_float = # float register offset\n0"));
    }

    #[test]
    fn control_script_drops_direct_torque_on_cb3() {
        let cb3 = build_control_script(3, 15, &[]).unwrap();
        let e_series = build_control_script(5, 23, &[]).unwrap();
        assert!(!cb3.contains('$'));
        assert!(!e_series.contains('$'));
        // The $5.23 body is large; CB3 must come out materially shorter.
        assert!(
            cb3.len() < e_series.len(),
            "CB3 script ({}) should be shorter than e-Series ({})",
            cb3.len(),
            e_series.len()
        );
    }

    #[test]
    fn wrap_custom_script_indents_and_bumps_register_12() {
        let out = wrap_custom_script("movej([0,0,0,0,0,0])\ntextmsg(\"hi\")");
        assert_eq!(
            out,
            "def custom_func():\n\
             \tmovej([0,0,0,0,0,0])\n\
             \ttextmsg(\"hi\")\n\
             \twrite_output_integer_register(12, read_output_integer_register(12)+1)\n\
             end\n"
        );
    }
}
