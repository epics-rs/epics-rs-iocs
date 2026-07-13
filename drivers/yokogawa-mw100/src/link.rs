//! Link-text grammar: `device cmd[:arg]` (`devMW100_common.c::mw100_parse_link`,
//! byte-for-byte the same algorithm as GM10's `gm10_parse_link`), plus the
//! shared address-family prefix grammar used by `VAL`/`UNIT`/`ALARM`/etc
//! across every `devMW100_*.c` file.
//!
//! Only 4 address families exist for MW100 (no `VarConst`): bare digits
//! (Signal, 1-60), `A<n>` (Math, 1-300), `C<n>` (Comm, 1-300), `K<n>`
//! (Const, 1-60) — confirmed via `drvMW100.c:430-439`'s `devqueue` array
//! sizes (`ch_type[60]`, `calc_info[300]`/`comm_input[300]`,
//! `constant[60]`), and via every `devMW100_*.c::init_record`'s own
//! `switch(arg[0])` never having a `'W'` (VarConst) arm.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelFamily {
    Signal,
    Math,
    Comm,
    Const,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelAddress {
    pub family: ChannelFamily,
    /// 1-based wire index, as sent/received on the wire.
    pub index: u32,
}

/// `device cmd[:arg]` split, mirroring `mw100_parse_link`.
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

/// Parse a family-prefixed channel address: bare digits (`Signal`, 1-60),
/// `A<n>` (`Math`, 1-300), `C<n>` (`Comm`, 1-300), `K<n>` (`Const`, 1-60).
/// Returns `None` on a malformed or out-of-range address.
pub fn parse_channel_address(arg: &str) -> Option<ChannelAddress> {
    let mut chars = arg.chars();
    let first = chars.next()?;
    let (family, digits, max): (_, &str, u32) = match first {
        '0'..='9' => (ChannelFamily::Signal, arg, 60),
        'A' => (ChannelFamily::Math, arg.get(1..)?, 300),
        'C' => (ChannelFamily::Comm, arg.get(1..)?, 300),
        'K' => (ChannelFamily::Const, arg.get(1..)?, 60),
        _ => return None,
    };
    let index: u32 = digits.parse().ok()?;
    if index == 0 || index > max {
        return None;
    }
    Some(ChannelAddress { family, index })
}

/// `ALARM:<addr>.<1-4>` — a channel address plus a mandatory `.N` sub-index
/// (`devMW100_mbbi.c:1123-1133`).
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
        let l = parse_link("hdl INP_TRIG").unwrap();
        assert_eq!(l.command, "INP_TRIG");
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
            parse_channel_address("27"),
            Some(ChannelAddress {
                family: ChannelFamily::Signal,
                index: 27
            })
        );
        assert_eq!(parse_channel_address("0"), None);
        assert_eq!(parse_channel_address("61"), None);
        assert!(parse_channel_address("60").is_some());
    }

    #[test]
    fn math_comm_const_prefixes_no_varconst() {
        assert_eq!(
            parse_channel_address("A300"),
            Some(ChannelAddress {
                family: ChannelFamily::Math,
                index: 300
            })
        );
        assert_eq!(parse_channel_address("A301"), None);
        assert_eq!(
            parse_channel_address("C300"),
            Some(ChannelAddress {
                family: ChannelFamily::Comm,
                index: 300
            })
        );
        assert_eq!(parse_channel_address("C301"), None);
        assert_eq!(
            parse_channel_address("K60"),
            Some(ChannelAddress {
                family: ChannelFamily::Const,
                index: 60
            })
        );
        assert_eq!(parse_channel_address("K61"), None);
        // No VarConst family exists for MW100.
        assert_eq!(parse_channel_address("W1"), None);
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
            parse_alarm_address("27.2"),
            Some((
                ChannelAddress {
                    family: ChannelFamily::Signal,
                    index: 27
                },
                2
            ))
        );
        assert_eq!(parse_alarm_address("27.0"), None);
        assert_eq!(parse_alarm_address("27.5"), None);
        assert_eq!(parse_alarm_address("27"), None);
        assert_eq!(parse_alarm_address("27.12"), None);
    }
}
