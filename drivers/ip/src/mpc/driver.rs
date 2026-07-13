//! `MPC` asyn port driver — the port of `devMPC.c`'s device support.
//!
//! One port per controller. asyn address = supply (pump) index: address 0 is
//! supply 1, address 1 is supply 2, matching the `parameter` field the C link
//! carried (`@asyn(port addr)GET_PRESSURE 1`).

use std::sync::mpsc::{Sender, channel};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;

use crate::asyn_error;
use crate::mpc::protocol::{
    self, DISPLAY_STRINGS, MpcError, NUM_SETPOINT_PAIRS, NUM_SUPPLIES, cmd,
};
use crate::worker::{DeviceWorker, Transport};

/// Command timeout (`MPC_TIMEOUT`, `devMPC.c:133`).
pub const MPC_TIMEOUT: Duration = Duration::from_secs(3);

/// Commands the record write handlers hand to the worker thread.
pub enum MpcCommand {
    /// A ready-made `~ AA XX d 00` frame; the reply is only checked for `OK`.
    Write(String),
}

/// Parameter indices of the MPC port.
#[derive(Clone, Copy)]
pub struct MpcParams {
    // Inputs.
    pub status: usize,
    pub pressure: usize,
    pub pressure_egu: usize,
    pub current: usize,
    pub voltage: usize,
    pub size: usize,
    pub setpoint_value: [usize; NUM_SETPOINT_PAIRS],
    pub setpoint_state: [usize; NUM_SETPOINT_PAIRS],
    pub auto_restart: usize,
    pub tsp_status: usize,
    // Outputs.
    pub set_unit: usize,
    pub set_display: usize,
    pub set_size: usize,
    pub set_setpoint: [usize; NUM_SETPOINT_PAIRS],
    pub start_stop: usize,
    pub keyboard_lock: usize,
    pub set_auto_restart: usize,
    pub tsp_timed: usize,
    pub tsp_off: usize,
    pub tsp_filament: usize,
    pub tsp_clear: usize,
    pub tsp_auto_advance: usize,
    pub tsp_continuous: usize,
    pub tsp_sublimation: usize,
    pub tsp_degas: usize,
}

impl MpcParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        let mut setpoint_value = [0usize; NUM_SETPOINT_PAIRS];
        let mut setpoint_state = [0usize; NUM_SETPOINT_PAIRS];
        let mut set_setpoint = [0usize; NUM_SETPOINT_PAIRS];
        for (pair, slot) in setpoint_value.iter_mut().enumerate() {
            *slot = base.create_param(&format!("MPC_SP{}_VAL", pair + 1), ParamType::Float64)?;
        }
        for (pair, slot) in setpoint_state.iter_mut().enumerate() {
            *slot = base.create_param(&format!("MPC_SP{}_STATE", pair + 1), ParamType::Int32)?;
        }
        for (pair, slot) in set_setpoint.iter_mut().enumerate() {
            *slot = base.create_param(&format!("MPC_SP{}_SET", pair + 1), ParamType::Float64)?;
        }
        Ok(Self {
            status: base.create_param("MPC_STATUS", ParamType::Octet)?,
            pressure: base.create_param("MPC_PRESSURE", ParamType::Float64)?,
            pressure_egu: base.create_param("MPC_PRESSURE_EGU", ParamType::Octet)?,
            current: base.create_param("MPC_CURRENT", ParamType::Float64)?,
            voltage: base.create_param("MPC_VOLTAGE", ParamType::Float64)?,
            size: base.create_param("MPC_SIZE", ParamType::Float64)?,
            setpoint_value,
            setpoint_state,
            auto_restart: base.create_param("MPC_AUTO_RESTART", ParamType::Int32)?,
            tsp_status: base.create_param("MPC_TSP_STATUS", ParamType::Octet)?,
            set_unit: base.create_param("MPC_SET_UNIT", ParamType::Int32)?,
            set_display: base.create_param("MPC_SET_DISPLAY", ParamType::Int32)?,
            set_size: base.create_param("MPC_SET_SIZE", ParamType::Float64)?,
            set_setpoint,
            start_stop: base.create_param("MPC_START", ParamType::Int32)?,
            keyboard_lock: base.create_param("MPC_LOCK", ParamType::Int32)?,
            set_auto_restart: base.create_param("MPC_SET_AUTO_RESTART", ParamType::Int32)?,
            tsp_timed: base.create_param("MPC_TSP_TIMED", ParamType::Octet)?,
            tsp_off: base.create_param("MPC_TSP_OFF", ParamType::Int32)?,
            tsp_filament: base.create_param("MPC_TSP_FILAMENT", ParamType::Int32)?,
            tsp_clear: base.create_param("MPC_TSP_CLEAR", ParamType::Int32)?,
            tsp_auto_advance: base.create_param("MPC_TSP_AUTO_ADVANCE", ParamType::Int32)?,
            tsp_continuous: base.create_param("MPC_TSP_CONTINUOUS", ParamType::Int32)?,
            tsp_sublimation: base.create_param("MPC_TSP_SUBLIMATION", ParamType::Octet)?,
            tsp_degas: base.create_param("MPC_TSP_DEGAS", ParamType::Int32)?,
        })
    }
}

pub struct MpcDriver {
    base: PortDriverBase,
    params: MpcParams,
    address: u8,
    commands: Sender<MpcCommand>,
}

impl MpcDriver {
    /// Create the port. `address` is the controller's RS-485 address, as the
    /// `@asyn(port ADDRESS)` field of the C link was.
    pub fn new(
        port_name: &str,
        address: u8,
    ) -> AsynResult<(Self, std::sync::mpsc::Receiver<MpcCommand>)> {
        let mut base = PortDriverBase::new(
            port_name,
            NUM_SUPPLIES,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = MpcParams::create(&mut base)?;
        let (commands, rx) = channel();
        Ok((
            Self {
                base,
                params,
                address,
                commands,
            },
            rx,
        ))
    }

    pub fn params(&self) -> MpcParams {
        self.params
    }

    /// asyn address -> supply number (1 or 2).
    fn supply(&self, addr: i32) -> AsynResult<u8> {
        match addr {
            0 => Ok(1),
            1 => Ok(2),
            other => Err(asyn_error(format!(
                "MPC: asyn address {other} is not a supply (0 = supply 1, 1 = supply 2)"
            ))),
        }
    }

    fn send(&self, command: u8, data: &str) -> AsynResult<()> {
        let frame = protocol::build_command(self.address, command, data);
        self.commands
            .send(MpcCommand::Write(frame))
            .map_err(|_| asyn_error("MPC: the worker thread is gone"))
    }
}

impl PortDriver for MpcDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        let supply = self.supply(addr)?;
        self.base.params.set_int32(reason, addr, value)?;

        if reason == p.set_unit || reason == p.set_display {
            let choice = usize::try_from(value)
                .ok()
                .and_then(|i| DISPLAY_STRINGS.get(i))
                .ok_or_else(|| {
                    asyn_error(format!("MPC: display choice {value} is out of range"))
                })?;
            let command = if reason == p.set_unit {
                cmd::SET_UNIT
            } else {
                cmd::SET_DISPLAY
            };
            self.send(command, &format!("{supply}{choice}"))
        } else if reason == p.start_stop {
            // devMPC.c:663 with MPC.db's ZNAM=START / ONAM=STOP.
            let command = if value == 0 {
                cmd::PUMP_START
            } else {
                cmd::PUMP_STOP
            };
            self.send(command, &supply.to_string())
        } else if reason == p.keyboard_lock {
            let command = if value == 0 {
                cmd::KEYBOARD_LOCK
            } else {
                cmd::KEYBOARD_UNLOCK
            };
            self.send(command, &supply.to_string())
        } else if reason == p.set_auto_restart {
            self.send(cmd::SET_AUTO_RESTART, yes_no(value))
        } else if reason == p.tsp_auto_advance {
            self.send(cmd::TSP_AUTO_ADVANCE, yes_no(value))
        } else if reason == p.tsp_off {
            self.send(cmd::TSP_OFF, "")
        } else if reason == p.tsp_clear {
            self.send(cmd::TSP_CLEAR, "")
        } else if reason == p.tsp_continuous {
            self.send(cmd::TSP_CONTINUOUS, "")
        } else if reason == p.tsp_degas {
            self.send(cmd::TSP_DEGAS, "")
        } else if reason == p.tsp_filament {
            self.send(cmd::TSP_FILAMENT, &value.to_string())
        } else {
            Ok(())
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        let supply = self.supply(addr)?;
        self.base.params.set_float64(reason, addr, value)?;

        if reason == p.set_size {
            return self.send(cmd::SET_SIZE, &format!("{supply},{}", value as i64));
        }
        if let Some(pair) = p.set_setpoint.iter().position(|&r| r == reason) {
            return self.send(
                cmd::SET_SETPOINT,
                &protocol::setpoint_data(pair, supply, value),
            );
        }
        Ok(())
    }

    fn write_octet(&mut self, user: &mut AsynUser, value: &[u8]) -> AsynResult<usize> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        self.supply(addr)?;
        let text = std::str::from_utf8(value)
            .map_err(|_| asyn_error("MPC: command strings must be ASCII"))?;
        self.base
            .params
            .set_string(reason, addr, text.to_string())?;

        if reason == p.tsp_timed {
            self.send(cmd::TSP_TIMED, text)?;
        } else if reason == p.tsp_sublimation {
            self.send(cmd::TSP_SUBLIMATION, text)?;
        }
        Ok(value.len())
    }
}

/// The MPC spells booleans `YES` / `NO` (`devMPC.c:673-683`).
fn yes_no(value: i32) -> &'static str {
    if value != 0 { "YES" } else { "NO" }
}

/// Polls the controller and runs the queued commands.
pub struct MpcWorker {
    transport: Transport,
    handle: PortHandle,
    params: MpcParams,
    address: u8,
}

impl MpcWorker {
    pub fn new(transport: Transport, handle: PortHandle, params: MpcParams, address: u8) -> Self {
        Self {
            transport,
            handle,
            params,
            address,
        }
    }

    /// One `~ AA XX d 00` transaction, returning the reply payload.
    fn transact(&self, command: u8, data: &str) -> Result<String, String> {
        let frame = protocol::build_command(self.address, command, data);
        let raw = self.transport.write_read(frame.as_bytes())?;
        protocol::parse_reply(&raw).map_err(|e: MpcError| e.to_string())
    }

    fn poll_supply(&mut self, addr: i32, supply: u8) {
        let p = self.params;
        let s = supply.to_string();
        let mut values: Vec<ParamSetValue> = Vec::new();

        match self.transact(cmd::READ_STATUS, &s) {
            Ok(payload) => values.push(ParamSetValue::new(
                p.status,
                addr,
                ParamValue::Octet(payload),
            )),
            Err(e) => log::error!("MPC: read status failed: {e}"),
        }
        match self
            .transact(cmd::READ_PRESSURE, &s)
            .and_then(|payload| protocol::parse_pressure(&payload).map_err(|e| e.to_string()))
        {
            Ok(reading) => {
                values.push(ParamSetValue::new(
                    p.pressure,
                    addr,
                    ParamValue::Float64(reading.value),
                ));
                values.push(ParamSetValue::new(
                    p.pressure_egu,
                    addr,
                    ParamValue::Octet(reading.egu),
                ));
            }
            Err(e) => log::error!("MPC: read pressure failed: {e}"),
        }
        match self
            .transact(cmd::READ_CURRENT, &s)
            .and_then(|payload| protocol::parse_current(&payload).map_err(|e| e.to_string()))
        {
            Ok(reading) => values.push(ParamSetValue::new(
                p.current,
                addr,
                ParamValue::Float64(reading.value),
            )),
            Err(e) => log::error!("MPC: read current failed: {e}"),
        }
        match self
            .transact(cmd::READ_VOLTAGE, &s)
            .and_then(|payload| protocol::parse_voltage(&payload).map_err(|e| e.to_string()))
        {
            Ok(reading) => values.push(ParamSetValue::new(
                p.voltage,
                addr,
                ParamValue::Float64(reading.value),
            )),
            Err(e) => log::error!("MPC: read voltage failed: {e}"),
        }
        match self
            .transact(cmd::READ_SIZE, &s)
            .and_then(|payload| protocol::parse_size(&payload).map_err(|e| e.to_string()))
        {
            Ok(reading) => values.push(ParamSetValue::new(
                p.size,
                addr,
                ParamValue::Float64(reading.value),
            )),
            Err(e) => log::error!("MPC: read pump size failed: {e}"),
        }
        for pair in 0..NUM_SETPOINT_PAIRS {
            let number = protocol::setpoint_number(pair, supply);
            match self.transact(cmd::READ_SETPOINT, &number.to_string()) {
                Ok(payload) => {
                    match protocol::parse_setpoint_value(&payload) {
                        Ok(reading) => values.push(ParamSetValue::new(
                            p.setpoint_value[pair],
                            addr,
                            ParamValue::Float64(reading.value),
                        )),
                        Err(e) => log::error!("MPC: setpoint {number} value: {e}"),
                    }
                    match protocol::parse_setpoint_state(&payload) {
                        Ok(state) => values.push(ParamSetValue::new(
                            p.setpoint_state[pair],
                            addr,
                            ParamValue::Int32(state),
                        )),
                        Err(e) => log::error!("MPC: setpoint {number} state: {e}"),
                    }
                }
                Err(e) => log::error!("MPC: read setpoint {number} failed: {e}"),
            }
        }
        match self.transact(cmd::READ_AUTO_RESTART, "") {
            Ok(payload) => values.push(ParamSetValue::new(
                p.auto_restart,
                addr,
                ParamValue::Int32(protocol::parse_auto_restart(&payload)),
            )),
            Err(e) => log::error!("MPC: read auto-restart failed: {e}"),
        }
        match self.transact(cmd::READ_TSP_STATUS, "") {
            Ok(payload) => values.push(ParamSetValue::new(
                p.tsp_status,
                addr,
                ParamValue::Octet(payload),
            )),
            Err(e) => log::error!("MPC: read TSP status failed: {e}"),
        }

        if !values.is_empty()
            && let Err(e) = self.handle.set_params_and_notify_blocking(addr, values)
        {
            log::error!("MPC: publishing supply {supply} parameters failed: {e}");
        }
    }
}

impl DeviceWorker for MpcWorker {
    type Command = MpcCommand;

    fn poll(&mut self) {
        for supply in 1..=NUM_SUPPLIES as u8 {
            self.poll_supply(i32::from(supply) - 1, supply);
        }
    }

    fn handle(&mut self, command: Self::Command) {
        let MpcCommand::Write(frame) = command;
        match self.transport.write_read(frame.as_bytes()) {
            Ok(raw) => {
                if let Err(e) = protocol::parse_reply(&raw) {
                    log::error!("MPC: command {frame:?} rejected: {e}");
                }
            }
            Err(e) => log::error!("MPC: command {frame:?} failed: {e}"),
        }
    }
}
