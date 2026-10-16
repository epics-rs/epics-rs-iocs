//! `TPG261` asyn port driver — the port of `devTPG261.c`'s device support.
//!
//! One port per controller, one asyn address per gauge (address 0 = gauge 1).

use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::asyn_error;
use crate::tpg261::protocol::{self, ENQ, GaugeStatus, NUM_GAUGES, SETPOINTS_PER_GAUGE, TpgError};
use crate::worker::{DeviceWorker, Transport};

/// Command timeout (`TPG261_TIMEOUT`, `devTPG261.c:87`).
pub const TPG261_TIMEOUT: Duration = Duration::from_secs(2);

/// A command string to send to the controller (`SPn,...`, `SEN,...`, `UNI,n`),
/// and the parameter that stands where C prints the record name.
pub struct TpgCommand {
    pub text: String,
    pub who: String,
}

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

    fn send(&self, text: String, who: String) -> AsynResult<()> {
        self.commands
            .send(TpgCommand { text, who })
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
            self.send(
                protocol::write_sensor(gauge, value != 0),
                format!("TPG_SET_SENSOR[{addr}]"),
            )
        } else if reason == p.set_units {
            self.send(
                protocol::write_units(value),
                format!("TPG_SET_UNITS[{addr}]"),
            )
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
            return self.send(
                protocol::write_setpoint(gauge, index, value),
                format!("TPG_SP{}_SET[{addr}]", index + 1),
            );
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
    /// (`devTPG261.c::devTPG261Callback`), with the error lines it prints.
    /// `who` stands where C prints the record name.
    fn transact(&self, command: &str, who: &str) -> Result<String, ()> {
        let acknowledgement = self.exchange(format!("{command}\r").as_bytes()).ok_or(())?;
        match protocol::check_ack(&acknowledgement) {
            Ok(()) => {}
            Err(TpgError::Nak) => {
                self.transport.print(
                    TraceMask::ERROR,
                    format_args!("devTPG261::devTPG261Callback Cmd {who} message Error "),
                );
                return Err(());
            }
            Err(_) => {
                self.print_too_small("Cmd", who);
                return Err(());
            }
        }
        // The <ENQ> goes out in the same C callback, so it always "runs".
        let data = self.exchange(&[ENQ]).unwrap_or_default();
        let data = data.trim().to_string();
        if data.is_empty() {
            self.print_too_small("Val", who);
            return Err(());
        }
        Ok(data)
    }

    fn exchange(&self, data: &[u8]) -> Option<String> {
        self.transport
            .exchange(data)
            .map(|raw| String::from_utf8_lossy(&raw).into_owned())
    }

    fn print_too_small(&self, stage: &str, who: &str) {
        self.transport.print(
            TraceMask::ERROR,
            format_args!("devTPG261::devTPG261Callback {stage} {who} message too small=0"),
        );
    }

    fn poll_gauge(&mut self, addr: i32, gauge: u8, states: Option<[i32; 4]>) {
        let p = self.params;
        let mut values: Vec<ParamSetValue> = Vec::new();

        let who = format!("TPG_PRESSURE[{addr}]");
        if let Ok((status, pressure)) = self
            .transact(&protocol::read_pressure(gauge), &who)
            .and_then(|raw| protocol::parse_pressure(&raw).map_err(|_| ()))
        {
            values.push(ParamSetValue::new(
                p.pressure,
                addr,
                ParamValue::Float64(pressure),
            ));
            let code = match status {
                GaugeStatus::Ok => 0,
                GaugeStatus::NoSensor => 4,
                GaugeStatus::Error(other) => {
                    self.transport.print(
                        TraceMask::ERROR,
                        format_args!("devTPG261::readAi Gauge error {who} status =[{other}] "),
                    );
                    other
                }
            };
            values.push(ParamSetValue::new(
                p.gauge_status,
                addr,
                ParamValue::Int32(code),
            ));
        }

        for index in 0..SETPOINTS_PER_GAUGE {
            if let Ok(value) = self
                .transact(
                    &protocol::read_setpoint(gauge, index),
                    &format!("TPG_SP{}_VAL[{addr}]", index + 1),
                )
                .and_then(|raw| protocol::parse_setpoint(&raw).map_err(|_| ()))
            {
                values.push(ParamSetValue::new(
                    p.setpoint_value[index],
                    addr,
                    ParamValue::Float64(value),
                ));
            }
            if let Some(states) = states {
                let relay = usize::from(protocol::setpoint_number(gauge, index)) - 1;
                values.push(ParamSetValue::new(
                    p.setpoint_state[index],
                    addr,
                    ParamValue::Int32(states[relay]),
                ));
            }
        }

        if !values.is_empty() {
            let _ = self.handle.set_params_and_notify_blocking(addr, values);
        }
    }
}

impl DeviceWorker for TpgWorker {
    type Command = TpgCommand;

    fn poll(&mut self) {
        let p = self.params;

        // SPS, SEN, UNI and TID answer for both gauges at once, so they are read
        // once per poll and fanned out.
        let states = self
            .transact(&protocol::read_setpoint_states(), "TPG_SP1_STATE")
            .and_then(|raw| protocol::parse_setpoint_states(&raw).map_err(|_| ()))
            .ok();
        let sensors = self
            .transact(&protocol::read_sensor_states(), "TPG_SENSOR")
            .and_then(|raw| protocol::parse_sensor_states(&raw).map_err(|_| ()))
            .ok();
        let units = self
            .transact(&protocol::read_units(), "TPG_UNITS")
            .and_then(|raw| protocol::parse_units(&raw).map_err(|_| ()))
            .ok();
        let ids = self.transact(&protocol::read_gauge_id(), "TPG_ID").ok();

        for gauge in 1..=NUM_GAUGES as u8 {
            let addr = i32::from(gauge) - 1;
            let mut shared: Vec<ParamSetValue> = Vec::new();
            if let Some(sensors) = sensors {
                shared.push(ParamSetValue::new(
                    p.sensor,
                    addr,
                    ParamValue::Int32(sensors[usize::from(gauge - 1)]),
                ));
            }
            if let Some(units) = units {
                shared.push(ParamSetValue::new(p.units, addr, ParamValue::Int32(units)));
            }
            if let Some(raw) = ids.as_deref()
                && let Ok(id) = protocol::parse_gauge_id(raw, gauge)
            {
                shared.push(ParamSetValue::new(
                    p.gauge_id,
                    addr,
                    ParamValue::Octet(id.into_bytes().into()),
                ));
            }
            if !shared.is_empty() {
                let _ = self.handle.set_params_and_notify_blocking(addr, shared);
            }
            self.poll_gauge(addr, gauge, states);
        }
    }

    fn handle(&mut self, command: Self::Command) {
        let _ = self.transact(&command.text, &command.who);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worker::test_support::{refused_transport, silent_transport, traced};

    fn params() -> TpgParams {
        TpgParams {
            pressure: 0,
            gauge_status: 1,
            setpoint_value: [2, 3],
            setpoint_state: [4, 5],
            sensor: 6,
            units: 7,
            gauge_id: 8,
            set_setpoint: [9, 10],
            set_sensor: 11,
            set_units: 12,
        }
    }

    /// A controller that never answers prints devTPG261Callback's Cmd line
    /// once per transaction, and nothing else.
    #[test]
    fn a_silent_controller_prints_cmd_message_too_small() {
        let (transport, path, runtime) = silent_transport("TPG_SILENT");
        let mut worker = TpgWorker::new(transport, runtime.port_handle().clone(), params());
        worker.poll();
        worker.handle(TpgCommand {
            text: "UNI,1".into(),
            who: "TPG_SET_UNITS[0]".into(),
        });
        let lines = traced(&path);
        let who: Vec<&str> = lines
            .iter()
            .map(|l| {
                l.split("devTPG261::devTPG261Callback Cmd ")
                    .nth(1)
                    .and_then(|rest| rest.strip_suffix(" message too small=0"))
                    .unwrap_or_else(|| panic!("{l:?}"))
            })
            .collect();
        assert_eq!(
            who,
            [
                "TPG_SP1_STATE",
                "TPG_SENSOR",
                "TPG_UNITS",
                "TPG_ID",
                "TPG_PRESSURE[0]",
                "TPG_SP1_VAL[0]",
                "TPG_SP2_VAL[0]",
                "TPG_PRESSURE[1]",
                "TPG_SP1_VAL[1]",
                "TPG_SP2_VAL[1]",
                "TPG_SET_UNITS[0]",
            ]
        );
    }

    /// A request the port refused never reached C's callback: no line.
    #[test]
    fn a_refused_request_prints_nothing() {
        let (transport, path, runtime) = refused_transport("TPG_REFUSED");
        TpgWorker::new(transport, runtime.port_handle().clone(), params()).poll();
        assert_eq!(traced(&path), Vec::<String>::new());
    }
}
