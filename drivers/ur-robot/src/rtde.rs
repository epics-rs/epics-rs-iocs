//! RTDE wire protocol: framing, control packages and the robot-command encoder.
//!
//! Ported from `ur_rtde/src/rtde.cpp` and `include/ur_rtde/rtde_utility.h`
//! (SDU Robotics, pin 68ac4e18). Everything on the RTDE wire is **big-endian**;
//! the C++ `packInt32`/`packDouble`/`packVectorNd` helpers byte-swap by hand and
//! `sendAll` writes the length with `htons`.
//!
//! Frame layout (`RTDE::sendAll`, rtde.cpp:303):
//!
//! ```text
//! +--------+--------+--------+= = = = = = = = =+
//! |  size (u16 BE)  |  cmd   |     payload     |
//! +--------+--------+--------+= = = = = = = = =+
//! ```
//!
//! `size` counts the 3-byte header itself.

/// `HEADER_SIZE` (rtde.cpp:20).
pub const HEADER_SIZE: usize = 3;

/// `RTDE_PROTOCOL_VERSION` (rtde.cpp:21).
pub const PROTOCOL_VERSION: u8 = 2;

/// RTDE TCP port (`port_ = 30004` in every interface constructor).
pub const RTDE_PORT: u16 = 30004;

/// `RTDE::RTDECommand` (rtde.h:179). The values are the ASCII letters the
/// controller uses as package types.
pub mod cmd {
    /// ASCII `V`
    pub const REQUEST_PROTOCOL_VERSION: u8 = 86;
    /// ASCII `v`
    pub const GET_URCONTROL_VERSION: u8 = 118;
    /// ASCII `M`
    pub const TEXT_MESSAGE: u8 = 77;
    /// ASCII `U`
    pub const DATA_PACKAGE: u8 = 85;
    /// ASCII `O`
    pub const CONTROL_PACKAGE_SETUP_OUTPUTS: u8 = 79;
    /// ASCII `I`
    pub const CONTROL_PACKAGE_SETUP_INPUTS: u8 = 73;
    /// ASCII `S`
    pub const CONTROL_PACKAGE_START: u8 = 83;
    /// ASCII `P`
    pub const CONTROL_PACKAGE_PAUSE: u8 = 80;
}

/// `RTDE::RobotCommand::Type` (rtde.h:45). Only the variants reachable from the
/// urRobot asyn drivers are given names here; the numeric values are the full
/// upstream set so an unported command cannot silently collide with a ported one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum CommandType {
    NoCmd = 0,
    MoveJ = 1,
    MoveL = 3,
    SpeedL = 10,
    SetStdDigitalOut = 13,
    SetToolDigitalOut = 14,
    SpeedStop = 15,
    TeachMode = 18,
    EndTeachMode = 19,
    SetSpeedSlider = 22,
    SetStdAnalogOut = 23,
    SetTcp = 29,
    ProtectiveStop = 31,
    StopL = 33,
    StopJ = 34,
    IsPoseWithinSafetyLimits = 36,
    IsJointsWithinSafetyLimits = 37,
    IsSteady = 47,
    SetConfDigitalOut = 48,
    SetInputIntRegister = 49,
    SetInputDoubleRegister = 50,
    StopScript = 255,
}

/// Header of an RTDE package (`RTDEUtility::readRTDEHeader`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Total package size **including** the 3 header bytes.
    pub size: u16,
    pub cmd: u8,
}

impl Header {
    /// Decode a header. Returns `None` if fewer than 3 bytes are available, or
    /// if the advertised size cannot even cover the header (a size < 3 would
    /// make `size - HEADER_SIZE` underflow — the C++ `data.resize(msg_size -
    /// HEADER_SIZE)` at rtde.cpp:363 wraps to a huge value instead).
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < HEADER_SIZE {
            return None;
        }
        let size = u16::from_be_bytes([data[0], data[1]]);
        if (size as usize) < HEADER_SIZE {
            return None;
        }
        Some(Self { size, cmd: data[2] })
    }

    /// Payload length (package size minus the header).
    pub fn payload_len(&self) -> usize {
        self.size as usize - HEADER_SIZE
    }
}

/// Build a complete RTDE frame: `[size_be16][cmd][payload]` (`RTDE::sendAll`).
pub fn encode_frame(command: u8, payload: &[u8]) -> Vec<u8> {
    let size = (HEADER_SIZE + payload.len()) as u16;
    let mut out = Vec::with_capacity(HEADER_SIZE + payload.len());
    out.extend_from_slice(&size.to_be_bytes());
    out.push(command);
    out.extend_from_slice(payload);
    out
}

/// `RTDE_REQUEST_PROTOCOL_VERSION` frame. Payload is a null byte followed by the
/// protocol version (`negotiateProtocolVersion`, rtde.cpp:115).
pub fn encode_protocol_version() -> Vec<u8> {
    encode_frame(cmd::REQUEST_PROTOCOL_VERSION, &[0, PROTOCOL_VERSION])
}

/// `RTDE_GET_URCONTROL_VERSION` frame (empty payload).
pub fn encode_get_controller_version() -> Vec<u8> {
    encode_frame(cmd::GET_URCONTROL_VERSION, &[])
}

/// `RTDE_CONTROL_PACKAGE_START` frame (empty payload).
pub fn encode_start() -> Vec<u8> {
    encode_frame(cmd::CONTROL_PACKAGE_START, &[])
}

/// `RTDE_CONTROL_PACKAGE_PAUSE` frame (empty payload).
pub fn encode_pause() -> Vec<u8> {
    encode_frame(cmd::CONTROL_PACKAGE_PAUSE, &[])
}

/// `RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS` frame: an 8-byte big-endian `double`
/// frequency followed by the comma-**terminated** variable names.
///
/// The C++ builds the frequency with `double2hexstr` + `hexToBytes`
/// (rtde.cpp:151), which formats the raw IEEE-754 bit pattern as hex and parses
/// it back into bytes — byte-for-byte identical to a plain big-endian `f64` for
/// every frequency the controller accepts. Note each name is *followed* by a
/// comma, so the payload ends with one (rtde.cpp:157).
pub fn encode_output_setup(names: &[String], frequency: f64) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.extend_from_slice(&frequency.to_be_bytes());
    for name in names {
        payload.extend_from_slice(name.as_bytes());
        payload.push(b',');
    }
    encode_frame(cmd::CONTROL_PACKAGE_SETUP_OUTPUTS, &payload)
}

/// `RTDE_CONTROL_PACKAGE_SETUP_INPUTS` frame: comma-terminated variable names,
/// no frequency (`sendInputSetup`, rtde.cpp:131).
pub fn encode_input_setup(names: &[String]) -> Vec<u8> {
    let mut payload = Vec::new();
    for name in names {
        payload.extend_from_slice(name.as_bytes());
        payload.push(b',');
    }
    encode_frame(cmd::CONTROL_PACKAGE_SETUP_INPUTS, &payload)
}

/// A robot command destined for an RTDE input recipe (`RTDE::RobotCommand`).
///
/// The C++ struct is one flat bag of fields with **no initialiser** for the PODs;
/// `RTDE::send` appends whichever of them the command type calls for. Modelling
/// the payload per command type instead makes the illegal combinations
/// unrepresentable, and removes the upstream read of uninitialised `double`s in
/// `setAnalogOutputVoltage`/`setAnalogOutputCurrent`, which always transmit both
/// analog channels but only ever assign one (rtde_io_interface.cpp:255-281).
#[derive(Debug, Clone, PartialEq)]
pub enum Payload {
    /// No extra fields (NO_CMD, TEACH_MODE, END_TEACH_MODE, PROTECTIVE_STOP,
    /// IS_STEADY, STOP_SCRIPT).
    None,
    /// `val_` vector only (SET_TCP, IS_POSE_WITHIN_SAFETY_LIMITS,
    /// IS_JOINTS_WITHIN_SAFETY_LIMITS, SPEED_STOP, SPEEDL).
    Vector(Vec<f64>),
    /// `val_` vector followed by the `async_` flag (MOVEJ, MOVEL, STOPJ, STOPL).
    VectorAsync { val: Vec<f64>, asynchronous: bool },
    /// A digital-output mask/value byte pair (SET_STD_DIGITAL_OUT,
    /// SET_CONF_DIGITAL_OUT, SET_TOOL_DIGITAL_OUT).
    DigitalOut { mask: u8, value: u8 },
    /// SET_SPEED_SLIDER: int32 mask then a double fraction.
    SpeedSlider { mask: i32, fraction: f64 },
    /// SET_STD_ANALOG_OUT: mask byte, type byte, then *both* channel doubles.
    AnalogOut {
        mask: u8,
        output_type: u8,
        ch0: f64,
        ch1: f64,
    },
    /// SET_INPUT_INT_REGISTER: one int32 (`reg_int_val_`, rtde.cpp:186).
    InputIntRegister(i32),
    /// SET_INPUT_DOUBLE_REGISTER: one double (`reg_double_val_`, rtde.cpp:193).
    InputDoubleRegister(f64),
}

/// A full robot command: the recipe it is addressed to, the command type, and
/// the type-specific payload.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotCommand {
    pub recipe_id: u8,
    pub command: CommandType,
    pub payload: Payload,
}

impl RobotCommand {
    pub fn new(recipe_id: u8, command: CommandType, payload: Payload) -> Self {
        Self {
            recipe_id,
            command,
            payload,
        }
    }

    /// Encode as an `RTDE_DATA_PACKAGE` frame.
    ///
    /// Field order is `RTDE::send` (rtde.cpp:166): the int32 command type is
    /// built first, the type-specific fields are appended, and the recipe id is
    /// finally *prepended* as a single byte (rtde.cpp:296).
    ///
    /// `WATCHDOG` in the C++ replaces the whole buffer with `NO_CMD`; that
    /// command is not reachable from urRobot and is not ported.
    pub fn encode(&self) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&(self.command as i32).to_be_bytes());

        match &self.payload {
            Payload::None => {}
            Payload::Vector(val) => push_vector_nd(&mut body, val),
            Payload::VectorAsync { val, asynchronous } => {
                // `val_` is appended before the async flag: rtde.cpp:224 runs
                // ahead of rtde.cpp:231.
                push_vector_nd(&mut body, val);
                body.extend_from_slice(&(i32::from(*asynchronous)).to_be_bytes());
            }
            Payload::DigitalOut { mask, value } => {
                body.push(*mask);
                body.push(*value);
            }
            Payload::SpeedSlider { mask, fraction } => {
                body.extend_from_slice(&mask.to_be_bytes());
                body.extend_from_slice(&fraction.to_be_bytes());
            }
            Payload::AnalogOut {
                mask,
                output_type,
                ch0,
                ch1,
            } => {
                body.push(*mask);
                body.push(*output_type);
                body.extend_from_slice(&ch0.to_be_bytes());
                body.extend_from_slice(&ch1.to_be_bytes());
            }
            Payload::InputIntRegister(v) => body.extend_from_slice(&v.to_be_bytes()),
            Payload::InputDoubleRegister(v) => body.extend_from_slice(&v.to_be_bytes()),
        }

        let mut payload = Vec::with_capacity(body.len() + 1);
        payload.push(self.recipe_id);
        payload.extend_from_slice(&body);
        encode_frame(cmd::DATA_PACKAGE, &payload)
    }
}

fn push_vector_nd(out: &mut Vec<u8>, values: &[f64]) {
    for v in values {
        out.extend_from_slice(&v.to_be_bytes());
    }
}

/// Controller version tuple returned by `RTDE_GET_URCONTROL_VERSION`
/// (four big-endian `u32`s).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControllerVersion {
    pub major: u32,
    pub minor: u32,
    pub bugfix: u32,
    pub build: u32,
}

impl ControllerVersion {
    /// Decode the body of a `RTDE_GET_URCONTROL_VERSION` reply.
    ///
    /// The C++ returns an all-zero tuple when the reply carries a different
    /// command byte (rtde.cpp:679); a body too short to hold four `u32`s is the
    /// same "no version" case here rather than an out-of-bounds read.
    pub fn decode(body: &[u8]) -> Option<Self> {
        if body.len() < 16 {
            return None;
        }
        let rd = |i: usize| u32::from_be_bytes([body[i], body[i + 1], body[i + 2], body[i + 3]]);
        Some(Self {
            major: rd(0),
            minor: rd(4),
            bugfix: rd(8),
            build: rd(12),
        })
    }
}

/// `CB3_MAJOR_VERSION` (rtde_control_interface.h:18). A controller newer than
/// this is an e-Series and runs RTDE at 500 Hz instead of 125 Hz.
pub const CB3_MAJOR_VERSION: u32 = 3;

/// Default RTDE frequency for a controller version (`frequency_ < 0` branch,
/// rtde_receive_interface.cpp:58).
pub fn default_frequency(version: ControllerVersion) -> f64 {
    if version.major > CB3_MAJOR_VERSION {
        500.0
    } else {
        125.0
    }
}

/// Reply to `RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS`: a recipe id byte followed by a
/// comma-separated list of the resolved variable types, where an unknown
/// variable comes back as `NOT_FOUND` (rtde.cpp:411).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutputSetupReply {
    pub recipe_id: u8,
    pub types: Vec<String>,
}

impl OutputSetupReply {
    pub fn decode(body: &[u8]) -> Option<Self> {
        let (&recipe_id, rest) = body.split_first()?;
        let text = String::from_utf8_lossy(rest);
        Some(Self {
            recipe_id,
            types: text.split(',').map(str::to_string).collect(),
        })
    }

    /// Names (in the order they were requested) the controller did not resolve.
    pub fn not_found<'a>(&self, requested: &'a [String]) -> Vec<&'a str> {
        self.types
            .iter()
            .enumerate()
            .filter(|(_, t)| *t == "NOT_FOUND")
            .filter_map(|(i, _)| requested.get(i).map(String::as_str))
            .collect()
    }
}

/// Reply to `RTDE_CONTROL_PACKAGE_SETUP_INPUTS`. The controller answers
/// `IN_USE` when another fieldbus already owns a requested input register
/// (rtde.cpp:402).
pub fn input_setup_in_use(body: &[u8]) -> bool {
    body.len() > 1 && String::from_utf8_lossy(&body[1..]).contains("IN_USE")
}

/// `RTDE_CONTROL_PACKAGE_START` / `..._PAUSE` reply: a single success byte.
pub fn decode_success(body: &[u8]) -> bool {
    body.first().is_some_and(|&b| b != 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Every expected byte string below was emitted by the real ur_rtde
    // `RTDEUtility` (rtde_utility.h) compiled as-is; see tests/golden.rs.

    #[test]
    fn frame_header_counts_itself() {
        // sendAll: size = HEADER_SIZE + payload.size(), big-endian.
        let f = encode_frame(cmd::CONTROL_PACKAGE_START, &[]);
        assert_eq!(f, vec![0x00, 0x03, 83]);

        let f = encode_frame(cmd::DATA_PACKAGE, &[1, 2, 3, 4]);
        assert_eq!(f, vec![0x00, 0x07, 85, 1, 2, 3, 4]);
    }

    #[test]
    fn header_decode_matches_read_rtde_header() {
        // readRTDEHeader(00 1f 55) => size=31 cmd=85, consuming 3 bytes.
        let h = Header::decode(&[0x00, 0x1f, 0x55]).unwrap();
        assert_eq!(h.size, 31);
        assert_eq!(h.cmd, cmd::DATA_PACKAGE);
        assert_eq!(h.payload_len(), 28);
    }

    #[test]
    fn header_rejects_short_and_undersized() {
        assert!(Header::decode(&[0x00]).is_none());
        // size < HEADER_SIZE would underflow `msg_size - HEADER_SIZE` in the C++.
        assert!(Header::decode(&[0x00, 0x02, 0x55]).is_none());
        assert!(Header::decode(&[0x00, 0x03, 0x55]).is_some());
    }

    #[test]
    fn protocol_version_payload() {
        // negotiateProtocolVersion pushes {0x00, version} then sendAll.
        assert_eq!(encode_protocol_version(), vec![0x00, 0x05, 86, 0x00, 0x02]);
    }

    #[test]
    fn empty_control_frames() {
        assert_eq!(encode_get_controller_version(), vec![0x00, 0x03, 118]);
        assert_eq!(encode_start(), vec![0x00, 0x03, 83]);
        assert_eq!(encode_pause(), vec![0x00, 0x03, 80]);
    }

    #[test]
    fn output_setup_frequency_is_big_endian_f64() {
        // double2hexstr(125.0) -> "405f400000000000" -> hexToBytes -> those 8 bytes.
        let f = encode_output_setup(&["timestamp".into(), "robot_mode".into()], 125.0);
        // 3 header + 8 frequency + "timestamp," (10) + "robot_mode," (11) = 32
        assert_eq!(&f[0..2], &[0x00, 0x20]);
        assert_eq!(f[2], cmd::CONTROL_PACKAGE_SETUP_OUTPUTS);
        assert_eq!(&f[3..11], &[0x40, 0x5f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]);
        // Names are comma-TERMINATED, not comma-separated.
        assert_eq!(&f[11..], b"timestamp,robot_mode,");
        assert_eq!(f.len(), u16::from_be_bytes([f[0], f[1]]) as usize);
    }

    #[test]
    fn output_setup_500hz() {
        let f = encode_output_setup(&["timestamp".into()], 500.0);
        assert_eq!(&f[3..11], &[0x40, 0x7f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn input_setup_has_no_frequency() {
        let f = encode_input_setup(&["input_int_register_0".into()]);
        assert_eq!(f[2], cmd::CONTROL_PACKAGE_SETUP_INPUTS);
        assert_eq!(&f[3..], b"input_int_register_0,");
    }

    #[test]
    fn no_cmd_clear_command() {
        // sendClearCommand: NO_CMD on recipe 4.
        let c = RobotCommand::new(4, CommandType::NoCmd, Payload::None);
        let f = c.encode();
        // 3 header + 1 recipe + 4 type
        assert_eq!(f, vec![0x00, 0x08, 85, 4, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn movej_appends_speed_accel_then_async() {
        // moveJ(q, speed, accel, async) => val_ = q ++ [speed, accel]; recipe 1.
        let q = vec![
            0.0,
            -std::f64::consts::FRAC_PI_2,
            0.0,
            -std::f64::consts::FRAC_PI_2,
            0.0,
            0.0,
        ];
        let mut val = q.clone();
        val.push(1.05);
        val.push(1.4);
        let c = RobotCommand::new(
            1,
            CommandType::MoveJ,
            Payload::VectorAsync {
                val,
                asynchronous: true,
            },
        );
        let f = c.encode();

        assert_eq!(f[2], cmd::DATA_PACKAGE);
        assert_eq!(f[3], 1); // recipe id
        assert_eq!(&f[4..8], &1i32.to_be_bytes()); // MOVEJ

        // q, packed exactly as packVectorNd(q6) emitted by the C++.
        let q_hex = "0000000000000000bff921fb54442d180000000000000000bff921fb54442d1800000000000000000000000000000000";
        assert_eq!(hex(&f[8..8 + 48]), q_hex);
        // speed 1.05, accel 1.4 (packDouble golden values)
        assert_eq!(hex(&f[56..64]), "3ff0cccccccccccd");
        assert_eq!(hex(&f[64..72]), "3ff6666666666666");
        // async flag last
        assert_eq!(&f[72..76], &1i32.to_be_bytes());
        assert_eq!(f.len(), 76);
        assert_eq!(f.len(), u16::from_be_bytes([f[0], f[1]]) as usize);
    }

    #[test]
    fn stopj_is_accel_then_async_on_recipe_19() {
        let c = RobotCommand::new(
            19,
            CommandType::StopJ,
            Payload::VectorAsync {
                val: vec![2.0],
                asynchronous: false,
            },
        );
        let f = c.encode();
        assert_eq!(f[3], 19);
        assert_eq!(&f[4..8], &34i32.to_be_bytes());
        assert_eq!(hex(&f[8..16]), "4000000000000000"); // packDouble(2.0)
        assert_eq!(&f[16..20], &0i32.to_be_bytes());
    }

    #[test]
    fn set_std_digital_out_is_mask_then_value() {
        // setStandardDigitalOut(3, true) => mask = value = 1 << 3, recipe 2.
        let c = RobotCommand::new(
            2,
            CommandType::SetStdDigitalOut,
            Payload::DigitalOut {
                mask: 1 << 3,
                value: 1 << 3,
            },
        );
        let f = c.encode();
        assert_eq!(f, vec![0x00, 0x0a, 85, 2, 0, 0, 0, 13, 0x08, 0x08]);

        // signal_level=false keeps the mask but clears the value.
        let c = RobotCommand::new(
            2,
            CommandType::SetStdDigitalOut,
            Payload::DigitalOut {
                mask: 1 << 3,
                value: 0,
            },
        );
        assert_eq!(c.encode()[8..], [0x08, 0x00]);
    }

    #[test]
    fn speed_slider_is_int32_mask_then_double() {
        // setSpeedSlider(0.5): mask is always 1, recipe 4.
        let c = RobotCommand::new(
            4,
            CommandType::SetSpeedSlider,
            Payload::SpeedSlider {
                mask: 1,
                fraction: 0.5,
            },
        );
        let f = c.encode();
        assert_eq!(f[3], 4);
        assert_eq!(&f[4..8], &22i32.to_be_bytes());
        assert_eq!(&f[8..12], &1i32.to_be_bytes());
        assert_eq!(hex(&f[12..20]), "3fe0000000000000");
    }

    #[test]
    fn analog_out_voltage_and_current_types() {
        // setAnalogOutputVoltage(1, 0.25): mask = 1<<1, type = 1<<1 (voltage).
        let c = RobotCommand::new(
            5,
            CommandType::SetStdAnalogOut,
            Payload::AnalogOut {
                mask: 1 << 1,
                output_type: 1 << 1,
                ch0: 0.0,
                ch1: 0.25,
            },
        );
        let f = c.encode();
        assert_eq!(&f[4..8], &23i32.to_be_bytes());
        assert_eq!(f[8], 0x02);
        assert_eq!(f[9], 0x02);
        assert_eq!(hex(&f[10..18]), "0000000000000000"); // ch0 defaulted, not garbage
        assert_eq!(hex(&f[18..26]), "3fd0000000000000"); // ch1 = 0.25

        // setAnalogOutputCurrent(0, 0.25): type = 0 (current).
        let c = RobotCommand::new(
            5,
            CommandType::SetStdAnalogOut,
            Payload::AnalogOut {
                mask: 1,
                output_type: 0,
                ch0: 0.25,
                ch1: 0.0,
            },
        );
        let f = c.encode();
        assert_eq!(f[8], 0x01);
        assert_eq!(f[9], 0x00);
        assert_eq!(hex(&f[10..18]), "3fd0000000000000");
    }

    #[test]
    fn input_registers_carry_one_scalar() {
        // setInputIntRegister(18, 7) -> recipe 7 (lower range).
        let c = RobotCommand::new(
            7,
            CommandType::SetInputIntRegister,
            Payload::InputIntRegister(7),
        );
        let f = c.encode();
        assert_eq!(f, vec![0x00, 0x0c, 85, 7, 0, 0, 0, 49, 0, 0, 0, 7]);

        // setInputDoubleRegister(18, 0.5) -> recipe 12.
        let c = RobotCommand::new(
            12,
            CommandType::SetInputDoubleRegister,
            Payload::InputDoubleRegister(0.5),
        );
        let f = c.encode();
        assert_eq!(f[3], 12);
        assert_eq!(&f[4..8], &50i32.to_be_bytes());
        assert_eq!(hex(&f[8..16]), "3fe0000000000000");
        assert_eq!(f.len(), 16);
    }

    #[test]
    fn controller_version_decode() {
        let mut body = Vec::new();
        for v in [5u32, 11, 3, 108355] {
            body.extend_from_slice(&v.to_be_bytes());
        }
        let v = ControllerVersion::decode(&body).unwrap();
        assert_eq!(
            v,
            ControllerVersion {
                major: 5,
                minor: 11,
                bugfix: 3,
                build: 108355
            }
        );
        assert!(ControllerVersion::decode(&body[..15]).is_none());
    }

    #[test]
    fn frequency_follows_controller_generation() {
        let cb3 = ControllerVersion {
            major: 3,
            ..Default::default()
        };
        let e_series = ControllerVersion {
            major: 5,
            ..Default::default()
        };
        assert_eq!(default_frequency(cb3), 125.0);
        assert_eq!(default_frequency(e_series), 500.0);
    }

    #[test]
    fn output_setup_reply_reports_missing_names() {
        let mut body = vec![1u8];
        body.extend_from_slice(b"DOUBLE,NOT_FOUND,UINT32");
        let reply = OutputSetupReply::decode(&body).unwrap();
        assert_eq!(reply.recipe_id, 1);
        assert_eq!(reply.types.len(), 3);

        let requested = vec![
            "timestamp".to_string(),
            "no_such_var".to_string(),
            "runtime_state".to_string(),
        ];
        assert_eq!(reply.not_found(&requested), vec!["no_such_var"]);
    }

    #[test]
    fn input_setup_detects_in_use() {
        let mut body = vec![1u8];
        body.extend_from_slice(b"IN_USE");
        assert!(input_setup_in_use(&body));

        let mut ok = vec![1u8];
        ok.extend_from_slice(b"INT32,DOUBLE");
        assert!(!input_setup_in_use(&ok));
    }

    #[test]
    fn start_pause_success_byte() {
        assert!(decode_success(&[1]));
        assert!(!decode_success(&[0]));
        assert!(!decode_success(&[]));
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }
}
