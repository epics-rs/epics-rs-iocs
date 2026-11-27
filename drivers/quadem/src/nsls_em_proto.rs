//! Wire protocol of the NSLS Precision Integrator (`nslsSrc/drvNSLS_EM.cpp`).
//!
//! Three sockets: a UDP broadcast port for discovery, a TCP command port and a
//! TCP data port. Commands are single letters with one argument, answered with
//! a line that starts `OK>`; the data port streams one ASCII sample per line.

use std::cell::Cell;
use std::time::Duration;

use epics_libcom_rs::runtime::scanf::sscanf;

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

/// Parse the broadcast replies as C++ walks them (`drvNSLS_EM.cpp:184-192`):
/// `strstr` for `id:`, then `sscanf("id: %d%n")` and `sscanf(" ip: %s%n")`,
/// each from where the last ended.
///
/// C++ writes each pair into `moduleInfo_[numModules_++]`, a [`MAX_MODULES`]
/// long array, with no bound on the loop: a network answering with more than
/// 16 modules writes past the end of the member. Nothing is capped here — the
/// list simply grows, so a module beyond the seventeenth is still found rather
/// than dropped or overwriting memory. A scan that fails, which C++ steps
/// past with a stale `count` and records anyway, adds no module.
pub fn parse_discovery(buffer: &str) -> Vec<ModuleInfo> {
    let mut modules = Vec::new();
    let mut ptr = buffer.split('\0').next().unwrap_or_default().as_bytes();
    while let Some(at) = ptr.windows(3).position(|w| w == b"id:") {
        ptr = &ptr[at..];
        let (id, count) = (Cell::new(0i64), Cell::new(0i32));
        if sscanf(ptr, "id: %lld%n", &[&id, &count]) != 1 {
            ptr = &ptr[3..];
            continue;
        }
        ptr = &ptr[offset(&count)..];
        let (start, end) = (Cell::new(0i32), Cell::new(0i32));
        if sscanf(ptr, " ip: %n%s%n", &[&start, &(), &end]) != 1 {
            continue;
        }
        let ip = String::from_utf8_lossy(&ptr[offset(&start)..offset(&end)]).into_owned();
        ptr = &ptr[offset(&end)..];
        if let Ok(module_id) = i32::try_from(id.get()) {
            modules.push(ModuleInfo { module_id, ip });
        }
    }
    modules
}

/// A `%n` count as an index into the scanned text.
fn offset(count: &Cell<i32>) -> usize {
    usize::try_from(count.get()).unwrap_or(0)
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

/// `sscanf(ASCIIData, "%d: %d %d %d %d")` when the line holds a `:`,
/// `sscanf(ASCIIData, "%d %d %d %d")` when it does not (`:370-374`).
///
/// C++ ignores the count, so a short line keeps the previous line's counts;
/// that and a count C++ would wrap into `int` are `None` here.
pub fn parse_sample(line: &str) -> Option<Sample> {
    let fields: [Cell<i64>; 5] = Default::default();
    let [phase, raw @ ..] = &fields;
    let tagged = line.contains(':');
    let items = if tagged {
        sscanf(
            line,
            "%lld: %lld %lld %lld %lld",
            &[phase, &raw[0], &raw[1], &raw[2], &raw[3]],
        )
    } else {
        sscanf(
            line,
            "%lld %lld %lld %lld",
            &[&raw[0], &raw[1], &raw[2], &raw[3]],
        )
    };
    if items != 4 + i32::from(tagged) {
        return None;
    }
    let int = |v: &Cell<i64>| i32::try_from(v.get()).ok();
    let [a, b, c, d] = raw.each_ref().map(int);
    Some(Sample {
        phase: if tagged { Some(int(phase)?) } else { None },
        raw: [a?, b?, c?, d?],
    })
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

/// C++ requires all seven items; a short reply is an error and nothing is
/// written back to the parameter library.
pub fn parse_status(reply: &str) -> Option<Status> {
    match scan_status(reply) {
        (7, status) => Some(status),
        _ => None,
    }
}

/// C++'s `sscanf` of the `s` reply (`:522-532`): how many items it assigned
/// before the first miss, and the values, the unassigned ones zero — the
/// `numItems` and fields of `getStatus`'s error line. With all seven read,
/// the last byte of the address is cut, the comma C++ drops. An id or setting
/// C++ would wrap into `int` ends the items there.
pub fn scan_status(reply: &str) -> (usize, Status) {
    let spans: [Cell<i32>; 4] = Default::default();
    let ints: [Cell<i64>; 4] = Default::default();
    let period = Cell::new(0.0f64);
    let [ip_start, ip_end, ver_start, ver_end] = &spans;
    let [id, mode, values, range] = &ints;
    let scanned = sscanf(
        reply,
        "OK> \"ip: %n%s%n id: %lld, ver: %n%s%n { m = %lld, n = %lld, r = %lld, p = %lf }\"",
        &[
            ip_start,
            &(),
            ip_end,
            id,
            ver_start,
            &(),
            ver_end,
            mode,
            values,
            range,
            &period,
        ],
    );
    let scanned = usize::try_from(scanned).unwrap_or(0);
    let items = [(2, id), (4, mode), (5, values), (6, range)]
        .into_iter()
        .find(|&(item, v)| item <= scanned && i32::try_from(v.get()).is_err())
        .map_or(scanned, |(item, _)| item - 1);
    let int = |item, v: &Cell<i64>| {
        if item <= items {
            i32::try_from(v.get()).unwrap_or(0)
        } else {
            0
        }
    };
    let text = |item, start, end| {
        if item <= items {
            String::from_utf8_lossy(&reply.as_bytes()[offset(start)..offset(end)]).into_owned()
        } else {
            String::new()
        }
    };
    let mut ip = text(1, ip_start, ip_end);
    if items == 7 {
        ip.pop();
    }
    let status = Status {
        ip,
        module_id: int(2, id),
        firmware: text(3, ver_start, ver_end),
        mode: int(4, mode),
        values_per_read: int(5, values),
        range: int(6, range),
        period_us: if items == 7 { period.get() } else { 0.0 },
    };
    (items, status)
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

    /// C++ `findModules` (drvNSLS_EM.cpp:184-192): `strstr` for `id:`, then
    /// `sscanf("id: %d%n")` and `sscanf(" ip: %s%n")` from where each ended.
    /// A scan C++ steps past with a stale `count` and an unwritten module,
    /// and an id it would wrap into `int`, drop the module instead.
    #[test]
    fn discovery_replies_are_walked_as_c_sscanf() {
        let cases: [(&str, &[(i32, &str)]); 13] = [
            ("id:3 ip:10.0.0.1", &[(3, "10.0.0.1")]),
            (
                "id: 3 ip: 1.2.3.4, id: 4 ip: 5.6.7.8",
                &[(3, "1.2.3.4,"), (4, "5.6.7.8")],
            ),
            (
                "id: 3 ip: 10.0.0.1 junk id: 4 ip: 10.0.0.2",
                &[(3, "10.0.0.1"), (4, "10.0.0.2")],
            ),
            ("xid: 5 ip: 1.1.1.1", &[(5, "1.1.1.1")]),
            ("id: 3 ip: 1.2.3.4id: 4 ip: 5.6.7.8", &[(3, "1.2.3.4id:")]),
            ("id: 3\nip: 1.2.3.4", &[(3, "1.2.3.4")]),
            ("id: -2 ip: 1.1.1.1", &[(-2, "1.1.1.1")]),
            (
                "id: 3 ip: 255.255.255.255abcd",
                &[(3, "255.255.255.255abcd")],
            ),
            ("id: +0x7 ip: 1.1.1.1", &[]),
            ("id: 3000000000 ip: 1.1.1.1", &[]),
            ("id: x id: 4 ip: 1.1.1.1", &[(4, "1.1.1.1")]),
            ("id: 3", &[]),
            ("id: 3 ip: 1.1.1.1\0id: 4 ip: 2.2.2.2", &[(3, "1.1.1.1")]),
        ];
        let wrong: Vec<_> = cases
            .into_iter()
            .filter_map(|(reply, want)| {
                let want: Vec<_> = want
                    .iter()
                    .map(|&(module_id, ip)| ModuleInfo {
                        module_id,
                        ip: ip.into(),
                    })
                    .collect();
                let got = parse_discovery(reply);
                (got != want).then_some((reply, got, want))
            })
            .collect();
        assert!(wrong.is_empty(), "{wrong:#?}");
    }

    /// C++ `readThread` (drvNSLS_EM.cpp:370-374). It ignores the count, so a
    /// short line keeps the previous line's counts; that and a count C would
    /// wrap into `int` are `None` here.
    #[test]
    fn data_lines_are_scanned_as_c_sscanf() {
        let cases = [
            ("1 2 3 4x", Some((None, [1, 2, 3, 4]))),
            ("1 2 3 0x5", Some((None, [1, 2, 3, 0]))),
            ("  3: 1 2 3 4", Some((Some(3), [1, 2, 3, 4]))),
            ("3:1 2 3 4", Some((Some(3), [1, 2, 3, 4]))),
            ("+1: 2 3 4 5", Some((Some(1), [2, 3, 4, 5]))),
            ("1 2 3 4 5", Some((None, [1, 2, 3, 4]))),
            ("1\t2\n3\x0b4", Some((None, [1, 2, 3, 4]))),
            ("3 : 1 2 3 4", None),
            ("1 2 3 4 :", None),
            ("1\u{a0}2 3 4", None),
            ("1 2 3 3000000000", None),
            ("1 2 3", None),
            ("1: 2 3 4", None),
            ("", None),
        ];
        let wrong: Vec<_> = cases
            .into_iter()
            .filter_map(|(line, want)| {
                let want = want.map(|(phase, raw)| Sample { phase, raw });
                let got = parse_sample(line);
                (got != want).then_some((line, got, want))
            })
            .collect();
        assert!(wrong.is_empty(), "{wrong:#?}");
    }

    /// C++ `readStatus` (drvNSLS_EM.cpp:522-532): one `sscanf`, then the last
    /// byte of `ipAddress_` dropped once all seven items are read. An id or
    /// setting C would wrap into `int` ends the items there.
    #[test]
    fn the_status_reply_is_scanned_as_c_sscanf() {
        #[allow(clippy::too_many_arguments)]
        fn status(
            ip: &str,
            module_id: i32,
            firmware: &str,
            mode: i32,
            values_per_read: i32,
            range: i32,
            period_us: f64,
        ) -> Status {
            Status {
                ip: ip.into(),
                module_id,
                firmware: firmware.into(),
                mode,
                values_per_read,
                range,
                period_us,
            }
        }
        let tail = "ver: 1.2.3 { m = 1, n = 5, r = 2, p = 400 }\"";
        let cases = [
            (
                format!("OK>\"ip: 10.0.0.5, id: 3, {tail}"),
                7,
                status("10.0.0.5", 3, "1.2.3", 1, 5, 2, 400.0),
            ),
            (
                "OK> \"ip: 10.0.0.5, id: 3, ver: 1.2.3 { m = 1, n = 5, r = 2, p = 4e2us }\"".into(),
                7,
                status("10.0.0.5", 3, "1.2.3", 1, 5, 2, 400.0),
            ),
            (
                format!("OK> \"ip: 10.0.0.5 id: 3, {tail}"),
                7,
                status("10.0.0.", 3, "1.2.3", 1, 5, 2, 400.0),
            ),
            (
                "OK> \"ip: 10.0.0.5, id: 3, ver: \"1.2\" { m = 1, n = 5, r = -2, p = -1.5 }\""
                    .into(),
                7,
                status("10.0.0.5", 3, "\"1.2\"", 1, 5, -2, -1.5),
            ),
            (
                "OK> \"ip: 10.0.0.5, id: 3, ver: 1.2.3 { m = 0x1, n = 5, r = 2, p = 400 }\"".into(),
                4,
                status("10.0.0.5,", 3, "1.2.3", 0, 0, 0, 0.0),
            ),
            (
                "OK> \"ip: 10.0.0.5, id: 3, ver: 1.2.3 { m = 1, n = 5x, r = 2, p = 400 }\"".into(),
                5,
                status("10.0.0.5,", 3, "1.2.3", 1, 5, 0, 0.0),
            ),
            (
                "OK> \"ip: 10.0.0.5, id:3,ver:1.2.3{m=1,n=5,r=2,p=400}\"".into(),
                3,
                status("10.0.0.5,", 3, "1.2.3{m=1,n=5,r=2,p=400}\"", 0, 0, 0, 0.0),
            ),
            (
                "OK> \"ip: 10.0.0.5, id: 3\"".into(),
                2,
                status("10.0.0.5,", 3, "", 0, 0, 0, 0.0),
            ),
            (
                "OK> \"ip: 10.0.0.5,\"".into(),
                1,
                status("10.0.0.5,\"", 0, "", 0, 0, 0, 0.0),
            ),
            (
                format!("OK> \"ip: 10.0.0.5, id: 3000000000, {tail}"),
                1,
                status("10.0.0.5,", 0, "", 0, 0, 0, 0.0),
            ),
            (
                format!("OK> \"id: 3, ip: 10.0.0.5, {tail}"),
                0,
                status("", 0, "", 0, 0, 0, 0.0),
            ),
            (
                "OK \"ip: 10.0.0.5, id: 3\"".into(),
                0,
                status("", 0, "", 0, 0, 0, 0.0),
            ),
        ];
        let wrong: Vec<_> = cases
            .into_iter()
            .filter_map(|(reply, items, want)| {
                let got = scan_status(&reply);
                (got != (items, want.clone())).then_some((reply, got, (items, want)))
            })
            .collect();
        assert!(wrong.is_empty(), "{wrong:#?}");
    }
}
