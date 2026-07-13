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

/// Parse a family-prefixed channel address: bare digits (`Signal`, 1-999),
/// `A<n>` (`Math`, 1-200), `C<n>` (`Comm`, 1-500), `K<n>` (`Const`, 1-100),
/// `W<n>` (`VarConst`, 1-100). Returns `None` on a malformed or
/// out-of-range address.
pub fn parse_channel_address(arg: &str) -> Option<ChannelAddress> {
    let mut chars = arg.chars();
    let first = chars.next()?;
    let (family, digits, max): (_, &str, u32) = match first {
        '0'..='9' => (ChannelFamily::Signal, arg, 999),
        'A' => (ChannelFamily::Math, arg.get(1..)?, 200),
        'C' => (ChannelFamily::Comm, arg.get(1..)?, 500),
        'K' => (ChannelFamily::Const, arg.get(1..)?, 100),
        'W' => (ChannelFamily::VarConst, arg.get(1..)?, 100),
        _ => return None,
    };
    let index: u32 = digits.parse().ok()?;
    if index == 0 || index > max {
        return None;
    }
    Some(ChannelAddress { family, index })
}

/// `ALARM:<addr>.<1-4>` — a channel address plus a mandatory `.N` sub-index.
pub fn parse_alarm_address(arg: &str) -> Option<(ChannelAddress, u8)> {
    let (addr_part, sub_part) = arg.split_once('.')?;
    let addr = parse_channel_address(addr_part)?;
    if sub_part.len() != 1 {
        return None;
    }
    let sub = sub_part.as_bytes()[0];
    if !(b'1'..=b'4').contains(&sub) {
        return None;
    }
    Some((addr, sub - b'0'))
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
    }
}
