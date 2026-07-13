//! Turbo PMAC ASCII protocol: status-word bit masks and response parsers.
//!
//! Ported from `pmacApp/pmacAsynMotorPortSrc/pmacController.cpp` (the
//! `PMAC_STATUS1_*` / `PMAC_STATUS2_*` / `PMAC_GSTATUS_*` constant block) and
//! the `sscanf` calls in `pmacAxis.cpp` / `pmacController.cpp` /
//! `pmacAsynCoord.c`. Keeping the parsers here — free functions over `&str` —
//! is what makes the wire formats testable without a controller.

/// Motor status word 1 (`#n ?`, first 6 hex digits).
pub const STATUS1_HOMING: u32 = 1 << 10;
pub const STATUS1_DESIRED_VELOCITY_ZERO: u32 = 1 << 13;
pub const STATUS1_OPEN_LOOP: u32 = 1 << 18;
pub const STATUS1_AMP_ENABLED: u32 = 1 << 19;
pub const STATUS1_POS_LIMIT_SET: u32 = 1 << 21;
pub const STATUS1_NEG_LIMIT_SET: u32 = 1 << 22;
pub const STATUS1_MOTOR_ON: u32 = 1 << 23;

/// Motor status word 2 (`#n ?`, second 6 hex digits).
pub const STATUS2_IN_POSITION: u32 = 1 << 0;
pub const STATUS2_ERR_FOLLOW_ERR: u32 = 1 << 2;
pub const STATUS2_AMP_FAULT: u32 = 1 << 3;
pub const STATUS2_HOME_COMPLETE: u32 = 1 << 10;
pub const STATUS2_DESIRED_STOP: u32 = 1 << 12;

/// Global status word (`???`).
const GSTATUS_MACRO_RING_ERRORCHECK: u32 = 1 << 4;
const GSTATUS_MACRO_RING_COMMS: u32 = 1 << 5;
const GSTATUS_REALTIME_INTR: u32 = 1 << 9;
const GSTATUS_FLASH_ERROR: u32 = 1 << 10;
const GSTATUS_DPRAM_ERROR: u32 = 1 << 11;
const GSTATUS_CKSUM_ERROR: u32 = 1 << 13;
const GSTATUS_WATCHDOG: u32 = 1 << 15;
const GSTATUS_SERVO_ERROR: u32 = 1 << 20;

/// Global-status bits that mean "the controller has a hardware problem"
/// (C `pmacController::PMAC_HARDWARE_PROB`).
pub const HARDWARE_PROB: u32 = GSTATUS_MACRO_RING_ERRORCHECK
    | GSTATUS_MACRO_RING_COMMS
    | GSTATUS_REALTIME_INTR
    | GSTATUS_FLASH_ERROR
    | GSTATUS_DPRAM_ERROR
    | GSTATUS_CKSUM_ERROR
    | GSTATUS_WATCHDOG
    | GSTATUS_SERVO_ERROR;

/// Axis status-2 bits that raise the motor record's PROBLEM bit
/// (C `PMAX_AXIS_GENERAL_PROB2`; `PMAX_AXIS_GENERAL_PROB1` is 0, so status-1
/// contributes nothing and is not modeled).
pub const AXIS_GENERAL_PROB2: u32 = STATUS2_DESIRED_STOP | STATUS2_AMP_FAULT;

/// The `i{n}24` bit that means "hardware limits are disabled on this axis"
/// (C `pmacAxis::getAxisStatus`, `0x20000 & limitsDisabledBit`).
pub const IX24_LIMITS_DISABLED: u32 = 0x20000;

/// Controller identifiers returned by `cid` (C `PMAC_CID_*`).
pub const CID_PMAC: i64 = 602413;
pub const CID_GEOBRICK: i64 = 603382;

/// Coordinate-system status words (`&n ??`), from `pmacAsynCoord.h`.
pub const CS_STATUS1_RUNNING_PROG: u32 = 1 << 0;
pub const CS_STATUS2_IN_POSITION: u32 = 1 << 17;
pub const CS_STATUS2_FOLLOW_ERR: u32 = 1 << 19;
pub const CS_STATUS2_AMP_FAULT: u32 = 1 << 20;
pub const CS_STATUS2_RUNTIME_ERR: u32 = 1 << 22;
pub const CS_STATUS3_LIMIT: u32 = 1 << 1;

/// Token scanner over a PMAC response. PMAC separates the values of a
/// multi-command query with CR, and C reads them with `sscanf` conversions
/// that all skip leading whitespace — so whitespace-separated tokens are an
/// exact model of the C parse for every format except the run-together
/// `%6x%6x` status pair, which [`take_status_pair`] handles.
struct Scanner<'a> {
    rest: &'a str,
}

impl<'a> Scanner<'a> {
    fn new(s: &'a str) -> Self {
        Self { rest: s }
    }

    fn token(&mut self) -> Option<&'a str> {
        let s = self.rest.trim_start();
        if s.is_empty() {
            return None;
        }
        let end = s.find(char::is_whitespace).unwrap_or(s.len());
        let (tok, rest) = s.split_at(end);
        self.rest = rest;
        Some(tok)
    }

    /// `%d` — a decimal integer.
    fn int(&mut self) -> Option<i64> {
        self.token()?.parse().ok()
    }

    /// `%lf` — a floating-point value.
    fn double(&mut self) -> Option<f64> {
        self.token()?.parse().ok()
    }

    /// `$%x` — a `$`-prefixed hex value, the form the PMAC returns for
    /// I-variables when it is in hex-reporting mode (I9=2, which the C driver
    /// assumes throughout).
    fn dollar_hex(&mut self) -> Option<u32> {
        let tok = self.token()?;
        u32::from_str_radix(tok.strip_prefix('$')?, 16).ok()
    }
}

/// Take up to `max` hex digits from the front of `s` (after leading
/// whitespace), as C's `%<max>x` does. Returns the value and the remainder.
fn take_hex(s: &str, max: usize) -> Option<(u32, &str)> {
    let s = s.trim_start();
    let end = s
        .char_indices()
        .take(max)
        .take_while(|(_, c)| c.is_ascii_hexdigit())
        .map(|(i, c)| i + c.len_utf8())
        .last()?;
    let value = u32::from_str_radix(&s[..end], 16).ok()?;
    Some((value, &s[end..]))
}

/// The run-together status pair at the head of a `#n ?` / `&n ??` response:
/// C `sscanf(response, "%6x%6x...")`. Each word is up to 6 hex digits with no
/// separator between them.
fn take_status_pair(s: &str) -> Option<(u32, u32, &str)> {
    let (s0, rest) = take_hex(s, 6)?;
    let (s1, rest) = take_hex(rest, 6)?;
    Some((s0, s1, rest))
}

/// One axis's poll response: C `pmacAxis::getAxisStatus`,
/// `sscanf(response, "%6x%6x %lf %lf", &status[0], &status[1], &position,
/// &enc_position)`. All four conversions must succeed (C checks `nvals != 4`).
///
/// For the no-encoder-axis command (`#n ? F P`) the third value is the
/// *following error* and the fourth is the actual position; the caller adds
/// them to recover the commanded position. For the encoder-axis command
/// (`#n ? P #e P`) they are the axis position and the encoder axis's position.
pub fn parse_axis_status(resp: &str) -> Option<(u32, u32, f64, f64)> {
    let (s0, s1, rest) = take_status_pair(resp)?;
    let mut sc = Scanner::new(rest);
    let a = sc.double()?;
    let b = sc.double()?;
    Some((s0, s1, a, b))
}

/// The global status word: C `pmacController::getGlobalStatus`,
/// `sscanf(response, "%6x", globalStatus)` on the `???` reply.
pub fn parse_global_status(resp: &str) -> Option<u32> {
    take_hex(resp, 6).map(|(v, _)| v)
}

/// The global feed rate: C `sscanf(response, "%d", feedrate)` on the `%` reply.
pub fn parse_feedrate(resp: &str) -> Option<i32> {
    Scanner::new(resp).int()?.try_into().ok()
}

/// The initial axis poll: C `pmacAxis::getAxisInitialStatus`,
/// `sscanf(response, "%lf %lf %lf %lf %lf", &high_limit, &low_limit, &pgain,
/// &dgain, &igain)` on the `I{n}13 I{n}14 I{n}30 I{n}31 I{n}33` reply. Note
/// the middle three land in **P, D, I** order — I{n}30 is the proportional
/// gain, I{n}31 the derivative gain and I{n}33 the integral gain.
pub struct InitialStatus {
    pub high_limit: f64,
    pub low_limit: f64,
    pub pgain: f64,
    pub dgain: f64,
    pub igain: f64,
}

pub fn parse_initial_status(resp: &str) -> Option<InitialStatus> {
    let mut sc = Scanner::new(resp);
    Some(InitialStatus {
        high_limit: sc.double()?,
        low_limit: sc.double()?,
        pgain: sc.double()?,
        dgain: sc.double()?,
        igain: sc.double()?,
    })
}

/// The `i{n}24` reply: C `sscanf(response, "$%x", &limitsDisabledBit)`. A
/// reply that is not `$`-prefixed hex leaves C's variable at 0 (the `sscanf`
/// simply fails and the value is untouched), i.e. "limits not disabled" — so
/// `None` here means the same thing to the caller.
pub fn parse_ix24(resp: &str) -> Option<u32> {
    Scanner::new(resp).dollar_hex()
}

/// The controller type from `cid`: C `sscanf(response, "%d", &controller_type)`.
pub fn parse_cid(resp: &str) -> Option<i64> {
    Scanner::new(resp).int()
}

/// The home-flag block read by `pmacAxis::home` before deciding whether it may
/// drop limits protection.
#[derive(Debug, Clone, Copy)]
pub struct HomeFlags {
    /// `Ix7n2` (Geobrick) / `ms{n},i912` (VME PMAC) — the home type.
    pub home_type: i64,
    /// `Ix7n3` / `ms{n},i913` — which limit flag the home seeks.
    pub home_flag: i64,
    /// `i{n}24` — the axis flag mode word.
    pub flag_mode: u32,
    /// `i{n}23` — the home velocity (sign carries the home direction).
    pub home_velocity: f64,
    /// `i{n}26` — the home offset.
    pub home_offset: i64,
}

/// Geobrick home-flag reply: C `sscanf(response, "%d %d $%x %lf %d", ...)`.
pub fn parse_home_flags_geobrick(resp: &str) -> Option<HomeFlags> {
    let mut sc = Scanner::new(resp);
    Some(HomeFlags {
        home_type: sc.int()?,
        home_flag: sc.int()?,
        flag_mode: sc.dollar_hex()?,
        home_velocity: sc.double()?,
        home_offset: sc.int()?,
    })
}

/// VME PMAC home-flag reply: C `sscanf(response, "$%x $%x $%x %lf %d", ...)` —
/// the two macro-station variables come back `$`-prefixed hex here, decimal on
/// the Geobrick.
pub fn parse_home_flags_pmac(resp: &str) -> Option<HomeFlags> {
    let mut sc = Scanner::new(resp);
    Some(HomeFlags {
        home_type: sc.dollar_hex()? as i64,
        home_flag: sc.dollar_hex()? as i64,
        flag_mode: sc.dollar_hex()?,
        home_velocity: sc.double()?,
        home_offset: sc.int()?,
    })
}

/// Whether limits protection may be dropped for this home (C `pmacAxis::home`,
/// the `REMOVE_LIMITS_ON_HOME` condition): the axis must be homing *onto* an
/// end limit, limits must not already be disabled, and the home velocity must
/// drive it towards the flag it homes on, with any home offset in the opposite
/// sense.
pub fn may_disable_limits(flags: &HomeFlags, home_velocity: f64) -> bool {
    flags.home_type <= 15
        && flags.home_type % 4 >= 2
        && (flags.flag_mode & IX24_LIMITS_DISABLED) == 0
        && ((home_velocity > 0.0 && flags.home_flag == 1 && flags.home_offset <= 0)
            || (home_velocity < 0.0 && flags.home_flag == 2 && flags.home_offset >= 0))
}

/// The coordinate-system status triple: C `pmacAsynCoord.c`
/// `drvPmacGetCoordStatus`, `sscanf(response, "%6x%6x%6x", &status[0],
/// &status[1], &status[2])` on the `&n ??` reply.
pub fn parse_cs_status(resp: &str) -> Option<(u32, u32, u32)> {
    let (s0, s1, rest) = take_status_pair(resp)?;
    let (s2, _) = take_hex(rest, 6)?;
    Some((s0, s1, s2))
}

/// The coordinate-system position block: C `drvPmacGetAxesStatus` walks the
/// `&n Q81 Q82 … Q89` reply with successive `strtod` calls. Returns the
/// positions actually present, in order — the C code reads exactly `count`
/// of them and treats a short reply as a (logged) parse failure while still
/// using whatever it got, so the caller checks the length.
pub fn parse_cs_positions(resp: &str, count: usize) -> Vec<f64> {
    let mut sc = Scanner::new(resp);
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        match sc.double() {
            Some(v) => out.push(v),
            None => break,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_status_parses_run_together_hex_then_two_doubles() {
        // A real `#1 ? F P` reply: 12 hex digits, then the following error and
        // the actual position, CR-separated.
        let (s0, s1, f, p) = parse_axis_status("812000\r000001\r0.5\r1000.25\r").unwrap();
        assert_eq!(s0, 0x812000);
        assert_eq!(s1, 0x000001);
        assert_eq!(f, 0.5);
        assert_eq!(p, 1000.25);
    }

    #[test]
    fn axis_status_takes_only_six_hex_digits_per_word() {
        // The words run together with no separator, so the split is positional.
        let (s0, s1, ..) = parse_axis_status("800000400000 0 0").unwrap();
        assert_eq!(s0, 0x800000);
        assert_eq!(s1, 0x400000);
    }

    #[test]
    fn axis_status_rejects_a_short_reply() {
        // C checks `nvals != 4` and logs; a truncated reply must not parse.
        assert!(parse_axis_status("812000000001 0.5").is_none());
        assert!(parse_axis_status("").is_none());
        assert!(parse_axis_status("garbage").is_none());
    }

    #[test]
    fn axis_status_accepts_negative_positions() {
        let (.., f, p) = parse_axis_status("000000000001 -0.5 -12345.5").unwrap();
        assert_eq!(f, -0.5);
        assert_eq!(p, -12345.5);
    }

    #[test]
    fn global_status_and_feedrate() {
        assert_eq!(parse_global_status("800001\r"), Some(0x800001));
        assert_eq!(parse_feedrate("100\r"), Some(100));
        assert_eq!(parse_feedrate("\r"), None);
    }

    #[test]
    fn hardware_prob_covers_the_watchdog_bit() {
        assert_ne!(parse_global_status("008000").unwrap() & HARDWARE_PROB, 0);
        // A bit outside the mask (CARD_ADDR, bit 0) is not a problem.
        assert_eq!(parse_global_status("000001").unwrap() & HARDWARE_PROB, 0);
    }

    #[test]
    fn initial_status_is_high_low_p_d_i() {
        let s = parse_initial_status("100\r-100\r2000\r1000\r500\r").unwrap();
        assert_eq!(s.high_limit, 100.0);
        assert_eq!(s.low_limit, -100.0);
        assert_eq!(s.pgain, 2000.0); // I{n}30
        assert_eq!(s.dgain, 1000.0); // I{n}31
        assert_eq!(s.igain, 500.0); // I{n}33
        assert!(parse_initial_status("100 -100 2000").is_none());
    }

    #[test]
    fn ix24_needs_the_dollar_prefix() {
        assert_eq!(parse_ix24("$20000\r"), Some(0x20000));
        // Decimal reporting mode: C's `sscanf("$%x")` fails and leaves 0.
        assert_eq!(parse_ix24("131072"), None);
    }

    #[test]
    fn home_flags_geobrick_vs_pmac_differ_in_the_first_two_fields() {
        let g = parse_home_flags_geobrick("2 1 $20000 32.0 0").unwrap();
        assert_eq!(g.home_type, 2);
        assert_eq!(g.home_flag, 1);
        assert_eq!(g.flag_mode, 0x20000);
        assert_eq!(g.home_velocity, 32.0);
        assert_eq!(g.home_offset, 0);

        let p = parse_home_flags_pmac("$2 $1 $0 -32.0 0").unwrap();
        assert_eq!(p.home_type, 2);
        assert_eq!(p.home_flag, 1);
        assert_eq!(p.flag_mode, 0);
        assert_eq!(p.home_velocity, -32.0);
    }

    #[test]
    fn may_disable_limits_requires_homing_onto_the_limit_it_seeks() {
        let base = HomeFlags {
            home_type: 2,
            home_flag: 1,
            flag_mode: 0,
            home_velocity: 0.0,
            home_offset: 0,
        };
        // Forwards onto flag 1 with a non-positive offset: allowed.
        assert!(may_disable_limits(&base, 1.0));
        // Backwards onto flag 1: the home is away from the flag — refused.
        assert!(!may_disable_limits(&base, -1.0));
        // Backwards onto flag 2 with a non-negative offset: allowed.
        let f2 = HomeFlags {
            home_flag: 2,
            ..base
        };
        assert!(may_disable_limits(&f2, -1.0));
        // A home type that is not an end-limit home (type % 4 < 2): refused.
        let t = HomeFlags {
            home_type: 1,
            ..base
        };
        assert!(!may_disable_limits(&t, 1.0));
        // Limits already disabled in the flag word: refused.
        let m = HomeFlags {
            flag_mode: IX24_LIMITS_DISABLED,
            ..base
        };
        assert!(!may_disable_limits(&m, 1.0));
        // A wrong-signed offset: refused.
        let o = HomeFlags {
            home_offset: 10,
            ..base
        };
        assert!(!may_disable_limits(&o, 1.0));
    }

    #[test]
    fn cs_status_takes_three_words() {
        let (s0, s1, s2) = parse_cs_status("000001\r020000\r000002\r").unwrap();
        assert_eq!(s0, 1);
        assert_eq!(s1, 0x020000);
        assert_eq!(s2, 2);
        assert!(parse_cs_status("000001020000").is_none());
    }

    #[test]
    fn cs_positions_reads_up_to_count() {
        let p = parse_cs_positions("1.5\r-2.5\r0\r", 9);
        assert_eq!(p, vec![1.5, -2.5, 0.0]);
        assert_eq!(parse_cs_positions("1 2 3", 2), vec![1.0, 2.0]);
    }
}
