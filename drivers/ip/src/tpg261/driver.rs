//! `TPG261` asyn port driver — the port of `devTPG261.c`'s device support.
//!
//! One port per controller, one asyn address per gauge (address 0 = gauge 1).

use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;

use crate::asyn_error;
use crate::tpg261::protocol::{self, ENQ, GaugeStatus, NUM_GAUGES, SETPOINTS_PER_GAUGE, TpgError};
use crate::worker::{DeviceWorker, Transport};

/// Command timeout (`TPG261_TIMEOUT`, `devTPG261.c:87`).
pub const TPG261_TIMEOUT: Duration = Duration::from_secs(2);

/// A command string to send to the controller (`SPn,...`, `SEN,...`, `UNI,n`).
pub struct TpgCommand(pub String);

#[derive(Clone, Copy)]
pub struct TpgParams {
    pub pressure: usize,
    pub gauge_status: usize,
    pub setpoint_value: [usize; SETPOINTS_PER_GAUGE],
    pub setpoint_state: [usize; SETPOINTS_PER_GAUGE],
    pub sensor: usize,
    pub units: usize,
    pub gauge_id: usize,
    pub set_setpoint: [usize; SETPOINTS_PER_GAUGE],
    pub set_sensor: usize,
    pub set_units: usize,
}

impl TpgParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        let mut setpoint_value = [0usize; SETPOINTS_PER_GAUGE];
        let mut setpoint_state = [0usize; SETPOINTS_PER_GAUGE];
        let mut set_setpoint = [0usize; SETPOINTS_PER_GAUGE];
        for (i, slot) in setpoint_value.iter_mut().enumerate() {
            *slot = base.create_param(&format!("TPG_SP{}_VAL", i + 1), ParamType::Float64)?;
        }
        for (i, slot) in setpoint_state.iter_mut().enumerate() {
            *slot = base.create_param(&format!("TPG_SP{}_STATE", i + 1), ParamType::Int32)?;
        }
        for (i, slot) in set_setpoint.iter_mut().enumerate() {
            *slot = base.create_param(&format!("TPG_SP{}_SET", i + 1), ParamType::Float64)?;
        }
        Ok(Self {
            pressure: base.create_param("TPG_PRESSURE", ParamType::Float64)?,
            gauge_status: base.create_param("TPG_GAUGE_STATUS", ParamType::Int32)?,
            setpoint_value,
            setpoint_state,
            sensor: base.create_param("TPG_SENSOR", ParamType::Int32)?,
            units: base.create_param("TPG_UNITS", ParamType::Int32)?,
            gauge_id: base.create_param("TPG_ID", ParamType::Octet)?,
            set_setpoint,
            set_sensor: base.create_param("TPG_SET_SENSOR", ParamType::Int32)?,
            set_units: base.create_param("TPG_SET_UNITS", ParamType::Int32)?,
        })
    }
}

pub struct TpgDriver {
    base: PortDriverBase,
    params: TpgParams,
    commands: Sender<TpgCommand>,
}

impl TpgDriver {
    pub fn new(port_name: &str) -> AsynResult<(Self, Receiver<TpgCommand>)> {
        let mut base = PortDriverBase::new(
            port_name,
            NUM_GAUGES,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = TpgParams::create(&mut base)?;
        let (commands, rx) = channel();
        Ok((
            Self {
                base,
                params,
                commands,
            },
            rx,
        ))
    }

    pub fn params(&self) -> TpgParams {
        self.params
    }

    /// asyn address -> gauge number (1 or 2).
    fn gauge(&self, addr: i32) -> AsynResult<u8> {
        match addr {
            0 => Ok(1),
            1 => Ok(2),
            other => Err(asyn_error(format!(
                "TPG261: asyn address {other} is not a gauge (0 = gauge 1, 1 = gauge 2)"
            ))),
        }
    }

    fn send(&self, command: String) -> AsynResult<()> {
        self.commands
            .send(TpgCommand(command))
            .map_err(|_| asyn_error("TPG261: the worker thread is gone"))
    }
}

impl PortDriver for TpgDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        let gauge = self.gauge(addr)?;
        self.base.params.set_int32(reason, addr, value)?;

        if reason == p.set_sensor {
            self.send(protocol::write_sensor(gauge, value != 0))
        } else if reason == p.set_units {
            self.send(protocol::write_units(value))
        } else {
            Ok(())
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        let gauge = self.gauge(addr)?;
        self.base.params.set_float64(reason, addr, value)?;

        if let Some(index) = p.set_setpoint.iter().position(|&r| r == reason) {
            return self.send(protocol::write_setpoint(gauge, index, value));
        }
        Ok(())
    }
}

pub struct TpgWorker {
    transport: Transport,
    handle: PortHandle,
    params: TpgParams,
}

impl TpgWorker {
    pub fn new(transport: Transport, handle: PortHandle, params: TpgParams) -> Self {
        Self {
            transport,
            handle,
            params,
        }
    }

    /// Command, ACK/NAK, `<ENQ>`, data — the two-exchange TPG26x transaction
    /// (`devTPG261.c::devTPG261Callback`).
    fn transact(&self, command: &str) -> Result<String, String> {
        let acknowledgement = self
            .transport
            .write_read(format!("{command}\r").as_bytes())?;
        protocol::check_ack(&acknowledgement).map_err(|e: TpgError| e.to_string())?;
        let data = self.transport.write_read(&[ENQ])?;
        let data = data.trim().to_string();
        if data.is_empty() {
            return Err("empty data reply".to_string());
        }
        Ok(data)
    }

    fn poll_gauge(&mut self, addr: i32, gauge: u8, states: Option<[i32; 4]>) {
        let p = self.params;
        let mut values: Vec<ParamSetValue> = Vec::new();

        match self
            .transact(&protocol::read_pressure(gauge))
            .and_then(|raw| protocol::parse_pressure(&raw).map_err(|e| e.to_string()))
        {
            Ok((status, pressure)) => {
                values.push(ParamSetValue::Float64 {
                    reason: p.pressure,
                    addr,
                    value: pressure,
                });
                let code = match status {
                    GaugeStatus::Ok => 0,
                    GaugeStatus::NoSensor => 4,
                    GaugeStatus::Error(other) => other,
                };
                values.push(ParamSetValue::Int32 {
                    reason: p.gauge_status,
                    addr,
                    value: code,
                });
            }
            Err(e) => log::error!("TPG261: read pressure of gauge {gauge} failed: {e}"),
        }

        for index in 0..SETPOINTS_PER_GAUGE {
            match self
                .transact(&protocol::read_setpoint(gauge, index))
                .and_then(|raw| protocol::parse_setpoint(&raw).map_err(|e| e.to_string()))
            {
                Ok(value) => values.push(ParamSetValue::Float64 {
                    reason: p.setpoint_value[index],
                    addr,
                    value,
                }),
                Err(e) => log::error!("TPG261: read setpoint {index} of gauge {gauge}: {e}"),
            }
            if let Some(states) = states {
                let relay = usize::from(protocol::setpoint_number(gauge, index)) - 1;
                values.push(ParamSetValue::Int32 {
                    reason: p.setpoint_state[index],
                    addr,
                    value: states[relay],
                });
            }
        }

        if !values.is_empty()
            && let Err(e) = self.handle.set_params_and_notify_blocking(addr, values)
        {
            log::error!("TPG261: publishing gauge {gauge} parameters failed: {e}");
        }
    }
}

impl DeviceWorker for TpgWorker {
    type Command = TpgCommand;

    fn poll(&mut self) {
        let p = self.params;

        // SPS, SEN, UNI and TID answer for both gauges at once, so they are read
        // once per poll and fanned out.
        let states = match self
            .transact(&protocol::read_setpoint_states())
            .and_then(|raw| protocol::parse_setpoint_states(&raw).map_err(|e| e.to_string()))
        {
            Ok(states) => Some(states),
            Err(e) => {
                log::error!("TPG261: read setpoint states failed: {e}");
                None
            }
        };
        let sensors = match self
            .transact(&protocol::read_sensor_states())
            .and_then(|raw| protocol::parse_sensor_states(&raw).map_err(|e| e.to_string()))
        {
            Ok(sensors) => Some(sensors),
            Err(e) => {
                log::error!("TPG261: read sensor states failed: {e}");
                None
            }
        };
        let units = match self
            .transact(&protocol::read_units())
            .and_then(|raw| protocol::parse_units(&raw).map_err(|e| e.to_string()))
        {
            Ok(units) => Some(units),
            Err(e) => {
                log::error!("TPG261: read units failed: {e}");
                None
            }
        };
        let ids = self.transact(&protocol::read_gauge_id()).ok();

        for gauge in 1..=NUM_GAUGES as u8 {
            let addr = i32::from(gauge) - 1;
            let mut shared: Vec<ParamSetValue> = Vec::new();
            if let Some(sensors) = sensors {
                shared.push(ParamSetValue::Int32 {
                    reason: p.sensor,
                    addr,
                    value: sensors[usize::from(gauge - 1)],
                });
            }
            if let Some(units) = units {
                shared.push(ParamSetValue::Int32 {
                    reason: p.units,
                    addr,
                    value: units,
                });
            }
            if let Some(raw) = ids.as_deref() {
                match protocol::parse_gauge_id(raw, gauge) {
                    Ok(id) => shared.push(ParamSetValue::Octet {
                        reason: p.gauge_id,
                        addr,
                        value: id,
                    }),
                    Err(e) => log::error!("TPG261: gauge {gauge} id: {e}"),
                }
            }
            if !shared.is_empty()
                && let Err(e) = self.handle.set_params_and_notify_blocking(addr, shared)
            {
                log::error!("TPG261: publishing gauge {gauge} status failed: {e}");
            }
            self.poll_gauge(addr, gauge, states);
        }
    }

    fn handle(&mut self, command: Self::Command) {
        let TpgCommand(text) = command;
        if let Err(e) = self.transact(&text) {
            log::error!("TPG261: command {text:?} failed: {e}");
        }
    }
}
