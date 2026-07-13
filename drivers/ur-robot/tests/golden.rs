//! Byte fixtures taken from the C++ that this driver ports.
//!
//! `fixtures/ur_rtde_golden.rs` is machine-generated: `fixtures/ur_rtde_golden_gen.cpp`
//! includes ur_rtde's own `include/ur_rtde/rtde_utility.h` (pin 68ac4e18) and
//! prints what its packers (`packInt32`, `packUInt32`, `packDouble`,
//! `packVectorNd`, `packVectorNInt32`) produce. Those bytes are therefore
//! *compiled* C++ output, not a transcription.
//!
//! The frame *composition* around them — the 3-byte `[size u16 BE][cmd u8]`
//! header, the recipe-id byte prepended to a data package, and the field order
//! inside `RobotCommand` — is transcribed from `src/rtde.cpp:166-296`, which
//! cannot be compiled here (it needs boost::asio, and boost is not installed on
//! this host). The tests below therefore pin the Rust encoder against compiled
//! packer bytes laid out in the read order of `RTDE::send`.
//!
//! Regenerate with:
//! ```text
//! g++ -std=c++17 -I<stub holding rtde_export.h> -I<ur_rtde>/include \
//!     -o gen tests/fixtures/ur_rtde_golden_gen.cpp && ./gen > tests/fixtures/ur_rtde_golden.rs
//! ```

#[rustfmt::skip]
#[path = "fixtures/ur_rtde_golden.rs"]
mod golden;

use ur_robot::rtde::{
    CommandType, Payload, RobotCommand, encode_get_controller_version, encode_input_setup,
    encode_output_setup, encode_pause, encode_protocol_version, encode_start,
};

#[test]
fn the_handshake_frames_match_the_c_bytes() {
    assert_eq!(encode_protocol_version(), golden::FRAME_PROTOCOL_VERSION_2);
    assert_eq!(
        encode_get_controller_version(),
        golden::FRAME_GET_CONTROLLER_VERSION
    );
    assert_eq!(encode_start(), golden::FRAME_START);
    assert_eq!(encode_pause(), golden::FRAME_PAUSE);
}

#[test]
fn the_output_setup_frame_carries_a_big_endian_double_then_comma_terminated_names() {
    let frame = encode_output_setup(&["timestamp".into(), "actual_q".into()], 125.0);
    assert_eq!(frame, golden::FRAME_OUTPUT_SETUP_125HZ);
}

#[test]
fn the_input_setup_frame_carries_names_only() {
    let frame = encode_input_setup(&["input_int_register_23".into()]);
    assert_eq!(frame, golden::FRAME_INPUT_SETUP);
}

#[test]
fn an_asynchronous_movej_matches_the_c_bytes() {
    let val = vec![
        0.0,
        -std::f64::consts::FRAC_PI_2,
        0.0,
        -std::f64::consts::FRAC_PI_2,
        0.0,
        0.0,
        1.05,
        1.4,
    ];
    let cmd = RobotCommand::new(
        1,
        CommandType::MoveJ,
        Payload::VectorAsync {
            val,
            asynchronous: true,
        },
    );
    assert_eq!(cmd.encode(), golden::FRAME_MOVEJ_ASYNC);
}

#[test]
fn a_standard_digital_output_matches_the_c_bytes() {
    let cmd = RobotCommand::new(
        2,
        CommandType::SetStdDigitalOut,
        Payload::DigitalOut {
            mask: 1 << 3,
            value: 1 << 3,
        },
    );
    assert_eq!(cmd.encode(), golden::FRAME_SET_STD_DIGITAL_OUT_3_HIGH);
}

#[test]
fn the_input_register_commands_match_the_c_bytes() {
    let int_cmd = RobotCommand::new(
        7,
        CommandType::SetInputIntRegister,
        Payload::InputIntRegister(-5),
    );
    assert_eq!(int_cmd.encode(), golden::FRAME_SET_INPUT_INT_REGISTER_18);

    let double_cmd = RobotCommand::new(
        12,
        CommandType::SetInputDoubleRegister,
        Payload::InputDoubleRegister(0.5),
    );
    assert_eq!(
        double_cmd.encode(),
        golden::FRAME_SET_INPUT_DOUBLE_REGISTER_18
    );
}

#[test]
fn the_no_command_clear_frame_matches_the_c_bytes() {
    let cmd = RobotCommand::new(4, CommandType::NoCmd, Payload::None);
    assert_eq!(cmd.encode(), golden::FRAME_NO_CMD);
}

#[test]
fn the_speed_slider_frame_matches_the_c_bytes() {
    let cmd = RobotCommand::new(
        4,
        CommandType::SetSpeedSlider,
        Payload::SpeedSlider {
            mask: 1,
            fraction: 0.25,
        },
    );
    assert_eq!(cmd.encode(), golden::FRAME_SET_SPEED_SLIDER_25PCT);
}

/// The packers themselves: every scalar the codec writes goes out big-endian,
/// and the fixture proves the Rust `to_be_bytes` agrees with ur_rtde's manual
/// byte-swap unions.
#[test]
fn the_scalar_packers_agree_with_the_c_packers() {
    assert_eq!(1i32.to_be_bytes(), golden::PACK_INT32_1);
    assert_eq!((-2i32).to_be_bytes(), golden::PACK_INT32_MINUS_2);
    assert_eq!(i32::MAX.to_be_bytes(), golden::PACK_INT32_MAX);
    assert_eq!(0xdead_beefu32.to_be_bytes(), golden::PACK_UINT32_DEADBEEF);
    assert_eq!(0.0f64.to_be_bytes(), golden::PACK_DOUBLE_ZERO);
    assert_eq!(0.5f64.to_be_bytes(), golden::PACK_DOUBLE_HALF);
    assert_eq!(
        (-std::f64::consts::PI).to_be_bytes(),
        golden::PACK_DOUBLE_MINUS_PI
    );
    assert_eq!(125.0f64.to_be_bytes(), golden::PACK_DOUBLE_125);

    let vector: Vec<u8> = [0.0, -0.5, 1.5, 2.25, -1e-3, 3.0]
        .iter()
        .flat_map(|d: &f64| d.to_be_bytes())
        .collect();
    assert_eq!(vector, golden::PACK_VECTOR6D);

    let ints: Vec<u8> = [1i32, -1, 7].iter().flat_map(|i| i.to_be_bytes()).collect();
    assert_eq!(ints, golden::PACK_VECTOR3INT32);
}
