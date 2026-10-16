//! Link-text grammar: `device cmd[:arg]` (`devGM10_common.c::gm10_parse_link`),
//! plus the shared address-family prefix grammar used by `VAL`/`UNIT`/
//! `ALARM`/etc across every `devGM10_*.c` file.
//!
//! Bound note: `K`/`W` (const/varconst) validate against 100, not the
//! `drvGM10.h` `MAX_CONST`/`MAX_VARCONST` macros (200) — those macros are
//! stale relative to the actual 100-element `constant`/`varconstant` arrays
//! in `drvGM10.c`'s `struct devqueue` (`MAX_CONST_CHAN`/`MAX_VARCONST_CHAN`
//! = 100), so `devGM10_ai.c:140,147` / `devGM10_ao.c:127,134` pass an
//! out-of-range channel straight into an out-of-bounds array index. `A`
//! (calc/math) for the `EXPR` command validates against 200, not
//! `devGM10_stringin.c:192`'s hardcoded 300 — `calc_expr` is sized
//! `MAX_CALC_CHAN` = 200 (`drvGM10.c:238`), the same over-wide-bound defect
//! family as K/W, just via a bare literal instead of a stale macro.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelFamily {
    Signal,
    Math,
    Comm,
    Const,
    VarConst,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelAddress {
    pub family: ChannelFamily,
    /// 1-based wire index, as sent/received on the wire.
    pub index: u32,
}

/// `device cmd[:arg]` split, mirroring `gm10_parse_link`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParsedLink<'a> {
    pub device: &'a str,
    pub command: &'a str,
    pub arg: Option<&'a str>,
}

pub fn parse_link(text: &str) -> Option<ParsedLink<'_>> {
    let text = text.trim_start();
    let sep = text.find(char::is_whitespace)?;
    let device = &text[..sep];
    if device.is_empty() {
        return None;
    }
    let rest = text[sep..].trim_start();
    if rest.is_empty() {
        return None;
    }
    let end = rest.find(char::is_whitespace).unwrap_or(rest.len());
    let cmd_and_arg = &rest[..end];
    let (command, arg) = match cmd_and_arg.split_once(':') {
        Some((c, a)) => (c, Some(a)),
        None => (cmd_and_arg, None),
    };
    if command.is_empty() {
        return None;
    }
    Some(ParsedLink {
        device,
        command,
        arg,
    })
}

/// C `strtol(s, &end, 10)`: optional leading whitespace and sign, then
/// decimal digits, stopping at the first non-digit. Returns the value
/// (saturated, where C `atoi` overflow is UB) and the unparsed tail, which
/// is `s` itself when there are no digits.
pub fn strtol(s: &str) -> (i64, &str) {
    let body = s.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let (neg, unsigned) = match body.as_bytes().first() {
        Some(b'-') => (true, &body[1..]),
        Some(b'+') => (false, &body[1..]),
        _ => (false, body),
    };
    let n = unsigned.bytes().take_while(u8::is_ascii_digit).count();
    if n == 0 {
        return (0, s);
    }
    let mag = unsigned[..n].bytes().fold(0i64, |acc, d| {
        acc.saturating_mul(10).saturating_add(i64::from(d - b'0'))
    });
    (if neg { -mag } else { mag }, &unsigned[n..])
}

/// Parse a family-prefixed channel address: bare digits (`Signal`, 1-999),
/// `A<n>` (`Math`, 1-200), `C<n>` (`Comm`, 1-500), `K<n>` (`Const`, 1-100),
/// `W<n>` (`VarConst`, 1-100). The number is read with C `atoi`, so
/// trailing text after the digits is ignored. Returns `None` on an
/// unknown prefix or an out-of-range number.
pub fn parse_channel_address(arg: &str) -> Option<ChannelAddress> {
    parse_channel_address_tail(arg).map(|(addr, _)| addr)
}

fn parse_channel_address_tail(arg: &str) -> Option<(ChannelAddress, &str)> {
    let first = arg.chars().next()?;
    let (family, digits, max): (_, &str, i64) = match first {
        '0'..='9' => (ChannelFamily::Signal, arg, 999),
        'A' => (ChannelFamily::Math, &arg[1..], 200),
        'C' => (ChannelFamily::Comm, &arg[1..], 500),
        'K' => (ChannelFamily::Const, &arg[1..], 100),
        'W' => (ChannelFamily::VarConst, &arg[1..], 100),
        _ => return None,
    };
    let (index, tail) = strtol(digits);
    if index <= 0 || index > max {
        return None;
    }
    Some((
        ChannelAddress {
            family,
            index: index as u32,
        },
        tail,
    ))
}

/// `ALARM:<addr>.<1-4>`: the text right after the address number must be
/// exactly `.N` (`devGM10_mbbi.c:176-183`).
pub fn parse_alarm_address(arg: &str) -> Option<(ChannelAddress, u8)> {
    let (addr, tail) = parse_channel_address_tail(arg)?;
    match tail.as_bytes() {
        [b'.', sub @ b'1'..=b'4'] => Some((addr, sub - b'0')),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_device_command_and_arg() {
        let l = parse_link("hdl VAL:0027").unwrap();
        assert_eq!(l.device, "hdl");
        assert_eq!(l.command, "VAL");
        assert_eq!(l.arg, Some("0027"));
    }

    #[test]
    fn parses_bare_command_without_arg() {
        let l = parse_link("hdl CHAN_TRIG").unwrap();
        assert_eq!(l.command, "CHAN_TRIG");
        assert_eq!(l.arg, None);
    }

    #[test]
    fn rejects_missing_command() {
        assert!(parse_link("hdl").is_none());
        assert!(parse_link("").is_none());
        assert!(parse_link("   ").is_none());
    }

    #[test]
    fn signal_address_range() {
        assert_eq!(
            parse_channel_address("0027"),
            Some(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 27
            })
        );
        assert_eq!(parse_channel_address("0000"), None);
        assert_eq!(parse_channel_address("1000"), None);
        assert!(parse_channel_address("999").is_some());
    }

    #[test]
    fn math_comm_const_varconst_prefixes() {
        assert_eq!(
            parse_channel_address("A200"),
            Some(ChannelAddress {
                family: ChannelFamily::Math,
                index: 200
            })
        );
        assert_eq!(parse_channel_address("A201"), None);
        assert_eq!(
            parse_channel_address("C500"),
            Some(ChannelAddress {
                family: ChannelFamily::Comm,
                index: 500
            })
        );
        assert_eq!(parse_channel_address("C501"), None);
        assert_eq!(
            parse_channel_address("K100"),
            Some(ChannelAddress {
                family: ChannelFamily::Const,
                index: 100
            })
        );
        // Upstream defect (devGM10_ai.c:140/147, devGM10_ao.c:127/134):
        // MAX_CONST/MAX_VARCONST=200 in drvGM10.h is stale vs the real
        // 100-element array — K101/W101 must be rejected here, not accepted.
        assert_eq!(parse_channel_address("K101"), None);
        assert_eq!(
            parse_channel_address("W100"),
            Some(ChannelAddress {
                family: ChannelFamily::VarConst,
                index: 100
            })
        );
        assert_eq!(parse_channel_address("W101"), None);
    }

    #[test]
    fn rejects_unknown_prefix_and_malformed_digits() {
        assert_eq!(parse_channel_address("Z001"), None);
        assert_eq!(parse_channel_address("A"), None);
        assert_eq!(parse_channel_address(""), None);
    }

    #[test]
    fn alarm_sub_address() {
        assert_eq!(
            parse_alarm_address("0027.2"),
            Some((
                ChannelAddress {
                    family: ChannelFamily::Signal,
                    index: 27
                },
                2
            ))
        );
        assert_eq!(parse_alarm_address("0027.0"), None);
        assert_eq!(parse_alarm_address("0027.5"), None);
        assert_eq!(parse_alarm_address("0027"), None);
        assert_eq!(parse_alarm_address("0027.12"), None);
        assert_eq!(parse_alarm_address("0027x.2"), None);
        assert_eq!(
            parse_alarm_address("A3.4").map(|(a, s)| (a.index, s)),
            Some((3, 4))
        );
    }

    #[test]
    fn address_number_reads_like_c_atoi() {
        assert_eq!(parse_channel_address("27abc").map(|a| a.index), Some(27));
        assert_eq!(parse_channel_address("A 7").map(|a| a.index), Some(7));
        assert_eq!(parse_channel_address("C+12").map(|a| a.index), Some(12));
        assert_eq!(parse_channel_address("C-12"), None);
        assert_eq!(parse_channel_address("99999999999999999999999"), None);
        assert_eq!(strtol("  -42rest"), (-42, "rest"));
        assert_eq!(strtol("x1"), (0, "x1"));
    }
}
