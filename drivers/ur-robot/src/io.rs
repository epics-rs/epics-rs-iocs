//! RTDE I/O interface — digital/analog outputs, speed slider, input registers.
//!
//! Ported from `ur_rtde/src/rtde_io_interface.cpp`. This interface only ever
//! *writes*: it registers 16 input recipes at connect time and then sends
//! `RTDE_DATA_PACKAGE`s on them. It never starts output synchronisation.
//!
//! Recipe numbering is positional — the controller assigns ids in registration
//! order — so the recipe table below must stay in the order upstream registers
//! it.

use std::time::Duration;

use crate::error::{UrError, UrResult, verify_within};
use crate::rtde::{CommandType, Payload, RobotCommand};
use crate::session::{DEFAULT_TIMEOUT, Session};

/// Input registers 18..=22 (lower range) or 42..=46 (upper range) are the ones
/// the I/O interface exposes; the command register is always `+23`.
const COMMAND_REGISTER: i32 = 23;
const REGISTER_SLOTS: [i32; 5] = [18, 19, 20, 21, 22];

/// Recipe ids, in the order `setupRecipes()` registers them. Recipe 1 is the
/// no-command recipe, which this interface registers but never sends on.
mod recipe {
    pub const STD_DIGITAL_OUT: u8 = 2;
    pub const TOOL_DIGITAL_OUT: u8 = 3;
    pub const SPEED_SLIDER: u8 = 4;
    pub const STD_ANALOG_OUT: u8 = 5;
    pub const CONF_DIGITAL_OUT: u8 = 6;
    /// Recipes 7..=11 set input int registers 0..=4.
    pub const INPUT_INT_BASE: u8 = 7;
    /// Recipes 12..=16 set input double registers 0..=4.
    pub const INPUT_DOUBLE_BASE: u8 = 12;
}

/// RTDE I/O interface.
pub struct IoInterface {
    session: Session,
    /// 0 for the lower register range, 24 for the upper (`register_offset_`).
    register_offset: i32,
}

impl IoInterface {
    /// Connect, negotiate and register the 16 input recipes.
    pub fn connect(hostname: &str, use_upper_range_registers: bool) -> UrResult<Self> {
        let mut me = Self {
            session: Session::new(hostname, DEFAULT_TIMEOUT),
            register_offset: if use_upper_range_registers { 24 } else { 0 },
        };
        me.reconnect()?;
        Ok(me)
    }

    /// `reconnect()` — connect, negotiate, re-register the recipes.
    pub fn reconnect(&mut self) -> UrResult<()> {
        self.session.connect()?;
        self.session.negotiate_protocol_version()?;
        self.setup_recipes()?;
        // The C++ sleeps 100 ms so the controller finishes setting the recipes up.
        std::thread::sleep(Duration::from_millis(100));
        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        self.session.is_connected()
    }

    pub fn disconnect(&mut self) {
        self.session.disconnect(false);
    }

    fn in_int_reg(&self, reg: i32) -> String {
        format!("input_int_register_{}", self.register_offset + reg)
    }

    fn in_double_reg(&self, reg: i32) -> String {
        format!("input_double_register_{}", self.register_offset + reg)
    }

    /// The register ids this interface accepts, given the configured range
    /// (`[18-22]` lower, `[42-46]` upper).
    fn register_ids(&self) -> [i32; 5] {
        REGISTER_SLOTS.map(|r| r + self.register_offset)
    }

    /// Map a user-facing register id onto its 0-based slot.
    fn register_slot(&self, input_id: i32) -> UrResult<u8> {
        let ids = self.register_ids();
        ids.iter()
            .position(|&r| r == input_id)
            .map(|i| i as u8)
            .ok_or_else(|| UrError::OutOfRange {
                value: f64::from(input_id),
                min: f64::from(ids[0]),
                max: f64::from(ids[4]),
            })
    }

    /// `setupRecipes()` — 16 input recipes, registered in a fixed order.
    fn setup_recipes(&mut self) -> UrResult<()> {
        let cmd = self.in_int_reg(COMMAND_REGISTER);
        let mut recipes: Vec<Vec<String>> = vec![
            vec![cmd.clone()],
            vec![
                cmd.clone(),
                "standard_digital_output_mask".into(),
                "standard_digital_output".into(),
            ],
            vec![
                cmd.clone(),
                "tool_digital_output_mask".into(),
                "tool_digital_output".into(),
            ],
            vec![
                cmd.clone(),
                "speed_slider_mask".into(),
                "speed_slider_fraction".into(),
            ],
            vec![
                cmd.clone(),
                "standard_analog_output_mask".into(),
                "standard_analog_output_type".into(),
                "standard_analog_output_0".into(),
                "standard_analog_output_1".into(),
            ],
            vec![
                cmd.clone(),
                "configurable_digital_output_mask".into(),
                "configurable_digital_output".into(),
            ],
        ];
        for reg in REGISTER_SLOTS {
            recipes.push(vec![cmd.clone(), self.in_int_reg(reg)]);
        }
        for reg in REGISTER_SLOTS {
            recipes.push(vec![cmd.clone(), self.in_double_reg(reg)]);
        }

        for (i, names) in recipes.iter().enumerate() {
            let assigned = self.session.send_input_setup(names)?;
            let expected = (i + 1) as u8;
            if assigned != expected {
                return Err(UrError::Protocol(format!(
                    "controller assigned recipe id {assigned} to input recipe {expected} \
                     ({names:?}); the interface addresses recipes positionally"
                )));
            }
        }
        Ok(())
    }

    /// Send one command, reconnecting once if the socket has died under us.
    ///
    /// Upstream `sendCommand` recurses into itself after a reconnect
    /// (rtde_io_interface.cpp:400); the retry is bounded to one attempt here.
    fn send(&mut self, cmd: RobotCommand) -> UrResult<()> {
        match self.session.send_command(&cmd) {
            Ok(()) => Ok(()),
            Err(first) => {
                log::warn!("ur-robot: RTDE I/O lost the connection ({first}); reconnecting");
                self.session.disconnect(false);
                self.reconnect()?;
                self.session.send_command(&cmd)
            }
        }
    }

    /// `setStandardDigitalOut(output_id, signal_level)`.
    pub fn set_standard_digital_out(&mut self, output_id: u8, level: bool) -> UrResult<()> {
        let mask = digital_mask(output_id)?;
        self.send(RobotCommand::new(
            recipe::STD_DIGITAL_OUT,
            CommandType::SetStdDigitalOut,
            Payload::DigitalOut {
                mask,
                value: if level { mask } else { 0 },
            },
        ))
    }

    /// `setConfigurableDigitalOut(output_id, signal_level)`.
    pub fn set_configurable_digital_out(&mut self, output_id: u8, level: bool) -> UrResult<()> {
        let mask = digital_mask(output_id)?;
        self.send(RobotCommand::new(
            recipe::CONF_DIGITAL_OUT,
            CommandType::SetConfDigitalOut,
            Payload::DigitalOut {
                mask,
                value: if level { mask } else { 0 },
            },
        ))
    }

    /// `setToolDigitalOut(output_id, signal_level)`.
    pub fn set_tool_digital_out(&mut self, output_id: u8, level: bool) -> UrResult<()> {
        let mask = digital_mask(output_id)?;
        self.send(RobotCommand::new(
            recipe::TOOL_DIGITAL_OUT,
            CommandType::SetToolDigitalOut,
            Payload::DigitalOut {
                mask,
                value: if level { mask } else { 0 },
            },
        ))
    }

    /// `setSpeedSlider(speed)` — the mask is always 1 (use the fraction).
    pub fn set_speed_slider(&mut self, fraction: f64) -> UrResult<()> {
        self.send(RobotCommand::new(
            recipe::SPEED_SLIDER,
            CommandType::SetSpeedSlider,
            Payload::SpeedSlider { mask: 1, fraction },
        ))
    }

    /// `setAnalogOutputVoltage(output_id, voltage_ratio)`.
    pub fn set_analog_output_voltage(&mut self, output_id: u8, ratio: f64) -> UrResult<()> {
        self.send(analog_out(output_id, ratio, AnalogKind::Voltage)?)
    }

    /// `setAnalogOutputCurrent(output_id, current_ratio)`.
    pub fn set_analog_output_current(&mut self, output_id: u8, ratio: f64) -> UrResult<()> {
        self.send(analog_out(output_id, ratio, AnalogKind::Current)?)
    }

    /// `setInputIntRegister(input_id, value)`.
    pub fn set_input_int_register(&mut self, input_id: i32, value: i32) -> UrResult<()> {
        let slot = self.register_slot(input_id)?;
        self.send(RobotCommand::new(
            recipe::INPUT_INT_BASE + slot,
            CommandType::SetInputIntRegister,
            Payload::InputIntRegister(value),
        ))
    }

    /// `setInputDoubleRegister(input_id, value)`.
    pub fn set_input_double_register(&mut self, input_id: i32, value: f64) -> UrResult<()> {
        let slot = self.register_slot(input_id)?;
        self.send(RobotCommand::new(
            recipe::INPUT_DOUBLE_BASE + slot,
            CommandType::SetInputDoubleRegister,
            Payload::InputDoubleRegister(value),
        ))
    }
}

/// Which of the two analog output modes to command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AnalogKind {
    Voltage,
    Current,
}

/// `1u << output_id`, rejecting an id that would shift out of the mask byte.
///
/// The C++ casts `1u << output_id` to `uint8_t` with no bound on `output_id`, so
/// an id of 8 or more silently becomes a mask of 0 — a command that addresses no
/// output at all (rtde_io_interface.cpp:186).
fn digital_mask(output_id: u8) -> UrResult<u8> {
    if output_id > 7 {
        return Err(UrError::OutOfRange {
            value: f64::from(output_id),
            min: 0.0,
            max: 7.0,
        });
    }
    Ok(1u8 << output_id)
}

/// Build a SET_STD_ANALOG_OUT command.
///
/// The C++ leaves the *other* channel's double uninitialised and transmits it
/// anyway (rtde_io_interface.cpp:255-281); the unaddressed channel is an
/// explicit 0.0 here. The ratio bound (`[0;1]`) is the same one
/// `RTDEIOInterface`'s callers apply via `verifyValueIsWithin`.
fn analog_out(output_id: u8, ratio: f64, kind: AnalogKind) -> UrResult<RobotCommand> {
    verify_within(ratio, 0.0, 1.0)?;
    let mask = digital_mask(output_id)?;
    let output_type = match kind {
        // Voltage sets the type bit for this channel, current clears it.
        AnalogKind::Voltage => mask,
        AnalogKind::Current => 0,
    };
    let (ch0, ch1) = match output_id {
        0 => (ratio, 0.0),
        1 => (0.0, ratio),
        _ => {
            return Err(UrError::OutOfRange {
                value: f64::from(output_id),
                min: 0.0,
                max: 1.0,
            });
        }
    };
    Ok(RobotCommand::new(
        recipe::STD_ANALOG_OUT,
        CommandType::SetStdAnalogOut,
        Payload::AnalogOut {
            mask,
            output_type,
            ch0,
            ch1,
        },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iface(upper: bool) -> IoInterface {
        IoInterface {
            session: Session::new("127.0.0.1", DEFAULT_TIMEOUT),
            register_offset: if upper { 24 } else { 0 },
        }
    }

    #[test]
    fn command_register_is_23_offset_by_the_range() {
        assert_eq!(
            iface(false).in_int_reg(COMMAND_REGISTER),
            "input_int_register_23"
        );
        assert_eq!(
            iface(true).in_int_reg(COMMAND_REGISTER),
            "input_int_register_47"
        );
    }

    #[test]
    fn register_slots_map_onto_recipes_7_to_16() {
        let lower = iface(false);
        assert_eq!(lower.register_slot(18).unwrap(), 0);
        assert_eq!(lower.register_slot(22).unwrap(), 4);
        assert!(lower.register_slot(17).is_err());
        assert!(lower.register_slot(23).is_err());
        assert!(lower.register_slot(42).is_err());

        let upper = iface(true);
        assert_eq!(upper.register_slot(42).unwrap(), 0);
        assert_eq!(upper.register_slot(46).unwrap(), 4);
        assert!(upper.register_slot(18).is_err());

        // Slot 0 -> recipe 7 (int) / 12 (double); slot 4 -> 11 / 16.
        assert_eq!(recipe::INPUT_INT_BASE, 7);
        assert_eq!(recipe::INPUT_INT_BASE + 4, 11);
        assert_eq!(recipe::INPUT_DOUBLE_BASE, 12);
        assert_eq!(recipe::INPUT_DOUBLE_BASE + 4, 16);
    }

    #[test]
    fn digital_mask_is_one_shifted_and_bounded() {
        assert_eq!(digital_mask(0).unwrap(), 0x01);
        assert_eq!(digital_mask(7).unwrap(), 0x80);
        // The C++ would produce a mask of 0 here, addressing nothing.
        assert!(digital_mask(8).is_err());
    }

    #[test]
    fn analog_voltage_sets_the_type_bit_and_zeroes_the_other_channel() {
        let cmd = analog_out(1, 0.25, AnalogKind::Voltage).unwrap();
        assert_eq!(cmd.recipe_id, recipe::STD_ANALOG_OUT);
        assert_eq!(cmd.command, CommandType::SetStdAnalogOut);
        assert_eq!(
            cmd.payload,
            Payload::AnalogOut {
                mask: 0x02,
                output_type: 0x02,
                ch0: 0.0,
                ch1: 0.25,
            }
        );
    }

    #[test]
    fn analog_current_clears_the_type_bit() {
        let cmd = analog_out(0, 0.5, AnalogKind::Current).unwrap();
        assert_eq!(
            cmd.payload,
            Payload::AnalogOut {
                mask: 0x01,
                output_type: 0x00,
                ch0: 0.5,
                ch1: 0.0,
            }
        );
    }

    #[test]
    fn analog_rejects_a_ratio_outside_zero_to_one_and_a_third_channel() {
        assert!(analog_out(0, 1.5, AnalogKind::Voltage).is_err());
        assert!(analog_out(0, -0.1, AnalogKind::Current).is_err());
        assert!(analog_out(2, 0.5, AnalogKind::Voltage).is_err());
    }
}
