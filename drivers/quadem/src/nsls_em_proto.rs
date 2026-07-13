//! Wire protocol of the NSLS Precision Integrator (`nslsSrc/drvNSLS_EM.cpp`).
//!
//! Three sockets: a UDP broadcast port for discovery, a TCP command port and a
//! TCP data port. Commands are single letters with one argument, answered with
//! a line that starts `OK>`; the data port streams one ASCII sample per line.

use std::time::Duration;

/// `BROADCAST_TIMEOUT` — how long discovery collects replies (`:34`).
pub const BROADCAST_TIMEOUT: Duration = Duration::from_millis(200);
/// The read timeout inside the discovery loop (`drvNSLS_EM.cpp:174`).
pub const BROADCAST_READ_TIMEOUT: Duration = Duration::from_millis(10);
/// `NSLS_EM_TIMEOUT` (`:35`).
pub const NSLS_EM_TIMEOUT: Duration = Duration::from_millis(100);

/// `COMMAND_PORT` (`:37`).
pub const COMMAND_PORT: u16 = 4747;
/// `DATA_PORT` (`:38`).
pub const DATA_PORT: u16 = 5757;
/// `BROADCAST_PORT` (`:39`).
pub const BROADCAST_PORT: u16 = 37747;

/// `MIN_INTEGRATION_TIME` (`:40`).
pub const MIN_INTEGRATION_TIME: f64 = 400e-6;
/// `FREQUENCY` (`:42`).
pub const FREQUENCY: f64 = 1e6;
/// `MAX_COUNTS` — full scale of the 20-bit ADC (`:44`).
pub const MAX_COUNTS: f64 = 1048576.0;

/// `MAX_MODULES` (`drvNSLS_EM.h:14`).
pub const MAX_MODULES: usize = 16;
/// `MAX_COMMAND_LEN` (`drvNSLS_EM.h:13`), the reply buffer size.
pub const MAX_COMMAND_LEN: usize = 256;
/// The read size of the data thread (`char ASCIIData[150]`, `:314`).
pub const DATA_BUFFER_SIZE: usize = 150;
/// The discovery read buffer (`char buffer[1024]`, `:165`).
pub const DISCOVERY_BUFFER_SIZE: usize = 1024;

/// `ranges_` — the integration capacitors in pF (`:74-81`).
pub const RANGES_PF: [f64; 8] = [12.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0];

/// `PingPongValue_t` (`:41-45`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PingPong {
    Phase0 = 0,
    Phase1 = 1,
    Both = 2,
}

impl PingPong {
    pub fn from_i32(v: i32) -> Self {
        match v {
            0 => Self::Phase0,
            1 => Self::Phase1,
            _ => Self::Both,
        }
    }
}

/// One module seen on the network.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModuleInfo {
    pub module_id: i32,
    pub ip: String,
}

/// The discovery request (`:169`).
pub const DISCOVER_COMMAND: &[u8] = b"i\n";

/// Parse the broadcast replies: every `id: <n> ip: <addr>` pair in the buffer
/// (`drvNSLS_EM.cpp:176-187`).
///
/// C++ writes each pair into `moduleInfo_[numModules_++]`, a [`MAX_MODULES`]
/// long array, with no bound on the loop: a network answering with more than
/// 16 modules writes past the end of the member. Nothing is capped here — the
/// list simply grows, so a module beyond the seventeenth is still found rather
/// than dropped or overwriting memory.
pub fn parse_discovery(buffer: &str) -> Vec<ModuleInfo> {
    let mut modules = Vec::new();
    for chunk in buffer.split("id:").skip(1) {
        let mut fields = chunk.split_whitespace();
        let Some(module_id) = fields.next().and_then(|t| t.trim_matches(',').parse().ok()) else {
            continue;
        };
        // "ip:" then the address.
        let ip = fields
            .by_ref()
            .skip_while(|t| *t != "ip:")
            .nth(1)
            .map(|t| t.trim_matches(',').to_string());
        let Some(ip) = ip else { continue };
        modules.push(ModuleInfo { module_id, ip });
    }
    modules
}

/// `setMode` (`:414-431`): bit 0 is *stopped*, bit 7 says the meter should tag
/// each sample with its phase. The phase is only meaningful at
/// `valuesPerRead == 1`, so any other value forces [`PingPong::Both`] — the
/// caller must write that back to the parameter library, as C++ does.
pub fn mode_value(acquiring: bool, ping_pong: PingPong) -> i32 {
    let mut mode = if acquiring { 0 } else { 1 };
    if ping_pong != PingPong::Both {
        mode |= 0x80;
    }
    mode
}

/// The ping-pong setting that is actually usable at this `values_per_read`
/// (`:424-427`).
pub fn effective_ping_pong(ping_pong: PingPong, values_per_read: i32) -> PingPong {
    if values_per_read != 1 {
        PingPong::Both
    } else {
        ping_pong
    }
}

pub fn cmd_mode(mode: i32) -> String {
    format!("m {mode}")
}

/// `setIntegrationTime` (`:450`): the meter takes microseconds.
pub fn cmd_integration_time(seconds: f64) -> String {
    format!("p {}", (seconds * 1e6) as i32)
}

/// `setIntegrationTime` (`:443-447`): the meter's floor.
pub fn clamp_integration_time(seconds: f64) -> f64 {
    seconds.max(MIN_INTEGRATION_TIME)
}

pub fn cmd_range(range: i32) -> String {
    format!("r {range}")
}

pub fn cmd_values_per_read(values: i32) -> String {
    format!("n {values}")
}

/// `readStatus` (`:517`).
pub const CMD_STATUS: &str = "s";

/// Every command is answered with a line that starts `OK>` (`:288`).
pub fn is_ok(reply: &str) -> bool {
    reply.starts_with("OK>")
}

/// `computeScaleFactor` (`:488-502`): counts → amps.
pub fn scale_factor(range: i32, integration_time: f64, values_per_read: i32) -> f64 {
    let capacitance = RANGES_PF
        .get(range.clamp(0, RANGES_PF.len() as i32 - 1) as usize)
        .copied()
        .unwrap_or(RANGES_PF[0]);
    capacitance * 1e-12 * FREQUENCY
        / (integration_time * 1e6)
        / MAX_COUNTS
        / values_per_read.max(1) as f64
}

/// One line from the data port: the four raw counts, and the phase when the
/// meter is tagging samples with it (`:376-381`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub phase: Option<i32>,
    pub raw: [i32; 4],
}

/// `sscanf(ASCIIData, "%d: %d %d %d %d")` when the line carries a phase,
/// `sscanf(ASCIIData, "%d %d %d %d")` when it does not.
pub fn parse_sample(line: &str) -> Option<Sample> {
    let line = line.trim();
    let (phase, rest) = match line.split_once(':') {
        Some((phase, rest)) => (Some(phase.trim().parse().ok()?), rest),
        None => (None, line),
    };
    let mut counts = rest.split_whitespace();
    let mut raw = [0i32; 4];
    for slot in raw.iter_mut() {
        *slot = counts.next()?.parse().ok()?;
    }
    Some(Sample { phase, raw })
}

/// Should this sample be published, given the meter's ping-pong setting
/// (`:382-384`)?
///
/// C++ compares against a `phase` local that the phase-less branch of the
/// `sscanf` never writes, so an untagged sample is filtered against whatever
/// was left on the stack. A line without a phase carries no phase to filter
/// on — it is always published here.
pub fn sample_wanted(sample: &Sample, ping_pong: PingPong) -> bool {
    match sample.phase {
        None => true,
        Some(_) if ping_pong == PingPong::Both => true,
        Some(0) => ping_pong == PingPong::Phase0,
        Some(1) => ping_pong == PingPong::Phase1,
        Some(_) => false,
    }
}

/// The `s` reply (`:518-519`):
/// `OK> "ip: <addr>, id: <n>, ver: <v> { m = <m>, n = <n>, r = <r>, p = <us> }"`.
#[derive(Debug, Clone, PartialEq)]
pub struct Status {
    pub ip: String,
    pub module_id: i32,
    pub firmware: String,
    pub mode: i32,
    pub values_per_read: i32,
    pub range: i32,
    /// `p` — the integration period in microseconds, as the meter reports it.
    pub period_us: f64,
}

fn labelled<'a>(reply: &'a str, label: &str) -> Option<&'a str> {
    let rest = reply.split_once(label)?.1;
    let token = rest.split_whitespace().next()?;
    Some(token.trim_matches(|c| c == ',' || c == '"'))
}

fn labelled_number<T: std::str::FromStr>(reply: &str, label: &str) -> Option<T> {
    labelled(reply, label)?.parse().ok()
}

/// C++ requires all seven items; a short reply is an error and nothing is
/// written back to the parameter library.
pub fn parse_status(reply: &str) -> Option<Status> {
    Some(Status {
        ip: labelled(reply, "ip: ")?.to_string(),
        module_id: labelled_number(reply, "id: ")?,
        firmware: labelled(reply, "ver: ")?.to_string(),
        mode: labelled_number(reply, "m = ")?,
        values_per_read: labelled_number(reply, "n = ")?,
        range: labelled_number(reply, "r = ")?,
        period_us: labelled_number(reply, "p = ")?,
    })
}

/// `readStatus` (`:539-543`): the sample time doubles when only one phase of
/// the ping-pong pair is kept.
pub fn sample_time(period_seconds: f64, values_per_read: i32, ping_pong: PingPong) -> f64 {
    let mut sample_time = period_seconds * values_per_read as f64;
    if ping_pong != PingPong::Both {
        sample_time *= 2.0;
    }
    sample_time
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovery_replies_list_the_modules() {
        let reply = "id: 3 ip: 192.168.0.5\nid: 7 ip: 192.168.0.9\n";
        assert_eq!(
            parse_discovery(reply),
            vec![
                ModuleInfo {
                    module_id: 3,
                    ip: "192.168.0.5".into()
                },
                ModuleInfo {
                    module_id: 7,
                    ip: "192.168.0.9".into()
                },
            ]
        );
        assert!(parse_discovery("nothing here").is_empty());
    }

    #[test]
    fn the_mode_byte_encodes_stopped_and_phase_tagging() {
        assert_eq!(mode_value(true, PingPong::Both), 0);
        assert_eq!(mode_value(false, PingPong::Both), 1);
        assert_eq!(mode_value(true, PingPong::Phase0), 0x80);
        assert_eq!(mode_value(false, PingPong::Phase1), 0x81);

        assert_eq!(effective_ping_pong(PingPong::Phase0, 1), PingPong::Phase0);
        assert_eq!(effective_ping_pong(PingPong::Phase0, 5), PingPong::Both);
    }

    #[test]
    fn commands_match_the_c_strings() {
        assert_eq!(cmd_mode(0x81), "m 129");
        assert_eq!(cmd_integration_time(400e-6), "p 400");
        assert_eq!(cmd_integration_time(1.0), "p 1000000");
        assert_eq!(cmd_range(3), "r 3");
        assert_eq!(cmd_values_per_read(5), "n 5");
        assert_eq!(CMD_STATUS, "s");
        assert!(is_ok("OK> 1"));
        assert!(!is_ok("ERR"));
    }

    #[test]
    fn the_integration_time_is_clamped_at_the_meters_floor() {
        assert_eq!(clamp_integration_time(1e-6), MIN_INTEGRATION_TIME);
        assert_eq!(clamp_integration_time(0.5), 0.5);
    }

    #[test]
    fn the_scale_factor_converts_counts_to_amps() {
        // range 0 = 12 pF, 1 ms integration, 1 value per read:
        // 12e-12 * 1e6 / (1e-3 * 1e6) / 2^20 / 1
        let expected = 12.0 * 1e-12 * 1e6 / (1e-3 * 1e6) / 1048576.0;
        assert!((scale_factor(0, 1e-3, 1) - expected).abs() < 1e-24);
        // valuesPerRead divides it.
        assert!((scale_factor(0, 1e-3, 5) - expected / 5.0).abs() < 1e-24);
        // range 7 = 350 pF.
        assert!((scale_factor(7, 1e-3, 1) - expected * 350.0 / 12.0).abs() < 1e-22);
    }

    #[test]
    fn data_lines_parse_with_and_without_a_phase() {
        assert_eq!(
            parse_sample("0: 100 200 300 400"),
            Some(Sample {
                phase: Some(0),
                raw: [100, 200, 300, 400]
            })
        );
        assert_eq!(
            parse_sample("100 200 300 -400"),
            Some(Sample {
                phase: None,
                raw: [100, 200, 300, -400]
            })
        );
        assert_eq!(parse_sample("100 200"), None);
        assert_eq!(parse_sample(""), None);
    }

    #[test]
    fn the_ping_pong_setting_filters_tagged_samples_only() {
        let phase0 = Sample {
            phase: Some(0),
            raw: [0; 4],
        };
        let phase1 = Sample {
            phase: Some(1),
            raw: [0; 4],
        };
        let untagged = Sample {
            phase: None,
            raw: [0; 4],
        };

        assert!(sample_wanted(&phase0, PingPong::Phase0));
        assert!(!sample_wanted(&phase0, PingPong::Phase1));
        assert!(sample_wanted(&phase1, PingPong::Phase1));
        assert!(sample_wanted(&phase0, PingPong::Both));
        assert!(sample_wanted(&phase1, PingPong::Both));
        // An untagged sample carries no phase to filter on.
        assert!(sample_wanted(&untagged, PingPong::Phase0));
        assert!(sample_wanted(&untagged, PingPong::Both));
    }

    #[test]
    fn the_status_reply_carries_seven_items() {
        let reply = "OK> \"ip: 192.168.0.5, id: 3, ver: 1.2.3 \
                     { m = 1, n = 5, r = 2, p = 400.000000 }\"";
        assert_eq!(
            parse_status(reply),
            Some(Status {
                ip: "192.168.0.5".into(),
                module_id: 3,
                firmware: "1.2.3".into(),
                mode: 1,
                values_per_read: 5,
                range: 2,
                period_us: 400.0,
            })
        );
        assert_eq!(parse_status("OK> \"ip: 192.168.0.5, id: 3\""), None);
    }

    #[test]
    fn the_sample_time_doubles_for_a_single_phase() {
        assert_eq!(sample_time(400e-6, 5, PingPong::Both), 400e-6 * 5.0);
        assert_eq!(sample_time(400e-6, 1, PingPong::Phase0), 400e-6 * 2.0);
    }
}
