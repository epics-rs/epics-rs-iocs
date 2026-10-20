//! `parse_channel_address` reads the number as `devMW100_ai.c`'s `atoi`:
//! decimal, so a zero-padded address is not octal and `0x` stops at the `0`.

use yokogawa_mw100::link::{ChannelAddress, ChannelFamily, parse_channel_address};

#[test]
fn channel_number_is_decimal() {
    let at = |family, index| Some(ChannelAddress { family, index });
    assert_eq!(parse_channel_address("010"), at(ChannelFamily::Signal, 10));
    assert_eq!(parse_channel_address("A010"), at(ChannelFamily::Math, 10));
    assert_eq!(parse_channel_address("0x5"), None);
}
