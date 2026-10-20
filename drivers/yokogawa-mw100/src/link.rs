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

use epics_rs::base::types::c_parse;

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

/// C `isblank`, the only separator `mw100_parse_link` splits on.
fn is_blank(c: char) -> bool {
    c == ' ' || c == '\t'
}

pub fn parse_link(text: &str) -> Option<ParsedLink<'_>> {
    // C returns 1 on a leading blank, and reads past the NUL when there is
    // no blank at all.
    if text.starts_with(is_blank) {
        return None;
    }
    let sep = text.find(is_blank)?;
    let device = &text[..sep];
    let rest = text[sep..].trim_start_matches(is_blank);
    if rest.is_empty() {
        return None;
    }
    let end = rest.find(is_blank).unwrap_or(rest.len());
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
/// The number is read with C `atoi`/`strtol` (`devMW100_ai.c:105-145`), so
/// trailing text after the digits is ignored. Returns `None` on an
/// out-of-range address.
pub fn parse_channel_address(arg: &str) -> Option<ChannelAddress> {
    parse_channel_address_end(arg).map(|(addr, _)| addr)
}

/// [`parse_channel_address`] plus the byte offset `strtol` leaves its end
/// pointer at.
fn parse_channel_address_end(arg: &str) -> Option<(ChannelAddress, usize)> {
    let (family, start, max) = match *arg.as_bytes().first()? {
        b'0'..=b'9' => (ChannelFamily::Signal, 0, 60),
        b'A' => (ChannelFamily::Math, 1, 300),
        b'C' => (ChannelFamily::Comm, 1, 300),
        b'K' => (ChannelFamily::Const, 1, 60),
        _ => return None,
    };
    let (value, len) = c_parse::strtol(&arg.as_bytes()[start..], 10);
    let index = value as i32;
    if index <= 0 || index > max {
        return None;
    }
    Some((
        ChannelAddress {
            family,
            index: index as u32,
        },
        start + len,
    ))
}

/// `ALARM:<addr>.<1-4>`: the `strtol` end pointer must be followed by `.`,
/// one digit 1-4 and the end of the argument (`devMW100_mbbi.c:163-205`).
pub fn parse_alarm_address(arg: &str) -> Option<(ChannelAddress, u8)> {
    let (addr, end) = parse_channel_address_end(arg)?;
    match arg.as_bytes()[end..] {
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
    fn splits_on_c_isblank_only_and_refuses_a_leading_blank() {
        assert!(parse_link(" hdl VAL:1").is_none());
        assert!(parse_link("\thdl VAL:1").is_none());
        assert!(parse_link("hdl\u{a0}VAL:1").is_none());
        let l = parse_link("h\x0cdl\tVAL:1\x0bx").unwrap();
        assert_eq!(l.device, "h\x0cdl");
        assert_eq!(l.command, "VAL");
        assert_eq!(l.arg, Some("1\x0bx"));
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
    fn address_number_uses_c_atoi() {
        let signal = |index| ChannelAddress {
            family: ChannelFamily::Signal,
            index,
        };
        assert_eq!(parse_channel_address("1.5"), Some(signal(1)));
        assert_eq!(parse_channel_address("07abc"), Some(signal(7)));
        assert_eq!(
            parse_channel_address("C+3"),
            Some(ChannelAddress {
                family: ChannelFamily::Comm,
                index: 3
            })
        );
        assert_eq!(parse_channel_address("A-3"), None);
        // int truncation of the long: 2^32 + 5 reads back as 5.
        assert_eq!(parse_channel_address("4294967301"), Some(signal(5)));
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
        // The strtol end pointer, not the first '.', anchors the sub-index.
        assert_eq!(parse_alarm_address("27x.2"), None);
        assert_eq!(
            parse_alarm_address("A+12.4"),
            Some((
                ChannelAddress {
                    family: ChannelFamily::Math,
                    index: 12
                },
                4
            ))
        );
    }
}
