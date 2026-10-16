//! `MPC` asyn port driver — the port of `devMPC.c`'s device support.
//!
//! One port per controller. asyn address = supply (pump) index: address 0 is
//! supply 1, address 1 is supply 2, matching the `parameter` field the C link
//! carried (`@asyn(port addr)GET_PRESSURE 1`).

use std::collections::HashMap;
use std::sync::mpsc::{Sender, channel};
use std::time::Duration;

use epics_rs::asyn::error::{AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;

use crate::asyn_error;
use crate::mpc::protocol::{self, DISPLAY_STRINGS, NUM_SETPOINT_PAIRS, NUM_SUPPLIES, cmd};
use crate::worker::{DeviceWorker, Transport};

/// Command timeout (`MPC_TIMEOUT`, `devMPC.c:133`).
pub const MPC_TIMEOUT: Duration = Duration::from_secs(3);

/// Commands the record write handlers hand to the worker thread.
pub enum MpcCommand {
    /// A ready-made `~ AA XX d 00` frame; the reply is only checked for `OK`.
    /// `who` stands where C prints the record name, `record` is the C write
    /// routine (`writeAo`, `writeBo`, ...) that names it.
    Write {
        frame: String,
        who: String,
        record: &'static str,
    },
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

    /// The parameter and address that stand where C prints `pr->name`.
    fn who(&self, reason: usize, addr: i32) -> String {
        let name = self.base.params.param_name(reason).unwrap_or_default();
        format!("{name}[{addr}]")
    }

    fn send(&self, command: u8, data: &str, who: &str, record: &'static str) -> AsynResult<()> {
        let frame = protocol::build_command(self.address, command, data);
        self.commands
            .send(MpcCommand::Write {
                frame,
                who: who.to_string(),
                record,
            })
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
        let record = if [p.set_unit, p.set_display, p.tsp_filament].contains(&reason) {
            "writeMbbo"
        } else {
            "writeBo"
        };
        let who = self.who(reason, addr);
        let send = |command, data: &str| self.send(command, data, &who, record);

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
            send(command, &format!("{supply}{choice}"))
        } else if reason == p.start_stop {
            // devMPC.c:663 with MPC.db's ZNAM=START / ONAM=STOP.
            let command = if value == 0 {
                cmd::PUMP_START
            } else {
                cmd::PUMP_STOP
            };
            send(command, &supply.to_string())
        } else if reason == p.keyboard_lock {
            let command = if value == 0 {
                cmd::KEYBOARD_LOCK
            } else {
                cmd::KEYBOARD_UNLOCK
            };
            send(command, &supply.to_string())
        } else if reason == p.set_auto_restart {
            send(cmd::SET_AUTO_RESTART, yes_no(value))
        } else if reason == p.tsp_auto_advance {
            send(cmd::TSP_AUTO_ADVANCE, yes_no(value))
        } else if reason == p.tsp_off {
            send(cmd::TSP_OFF, "")
        } else if reason == p.tsp_clear {
            send(cmd::TSP_CLEAR, "")
        } else if reason == p.tsp_continuous {
            send(cmd::TSP_CONTINUOUS, "")
        } else if reason == p.tsp_degas {
            send(cmd::TSP_DEGAS, "")
        } else if reason == p.tsp_filament {
            send(cmd::TSP_FILAMENT, &value.to_string())
        } else {
            Ok(())
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let (reason, addr) = (user.reason, user.addr);
        let p = self.params;
        let supply = self.supply(addr)?;
        self.base.params.set_float64(reason, addr, value)?;
        let who = self.who(reason, addr);
        let send = |command, data: &str| self.send(command, data, &who, "writeAo");

        if reason == p.set_size {
            return send(cmd::SET_SIZE, &format!("{supply},{}", value as i64));
        }
        if let Some(pair) = p.set_setpoint.iter().position(|&r| r == reason) {
            return send(
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
        self.base.params.set_string(reason, addr, text)?;
        let who = self.who(reason, addr);
        let send = |command, data: &str| self.send(command, data, &who, "writeSo");

        if reason == p.tsp_timed {
            send(cmd::TSP_TIMED, text)?;
        } else if reason == p.tsp_sublimation {
            send(cmd::TSP_SUBLIMATION, text)?;
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
    /// C's per-record `prevReadStatus`, keyed by the name that stands for the
    /// record; a missing entry is the zeroed `asynSuccess`.
    prev_read: HashMap<String, AsynStatus>,
}

impl MpcWorker {
    pub fn new(transport: Transport, handle: PortHandle, params: MpcParams, address: u8) -> Self {
        Self {
            transport,
            handle,
            params,
            address,
            prev_read: HashMap::new(),
        }
    }

    fn print(&self, args: std::fmt::Arguments<'_>) {
        self.transport.print(TraceMask::ERROR, args);
    }

    /// `devMPCCallback`: write the frame, read the reply, print what C prints,
    /// and hand back C's `recBuf` — the payload of an `OK` reply (at least
    /// "OK"), empty for a rejected one, which is never published. `None` where C leaves `pPvt->status` failed (or the callback
    /// never ran), and the record routine stops before decoding.
    fn callback(&mut self, frame: &str, who: &str) -> Option<String> {
        match self.transport.write(frame.as_bytes()) {
            Err(e) if e.never_ran() => return None,
            Err(e) => {
                self.print(format_args!(
                    "devMPC::devMPCCallback write error, status={} error= {}",
                    e.status() as i32,
                    e.message_display()
                ));
                return None;
            }
            Ok(()) => {}
        }
        let read = self.transport.read_bytes();
        let prev = self
            .prev_read
            .get(who)
            .copied()
            .unwrap_or(AsynStatus::Success);
        let raw = match read {
            Err(e) => {
                if e.status() != prev {
                    self.print(format_args!(
                        "devMPC::devMPCCallback {who} read error, status={} error= {}",
                        e.status() as i32,
                        e.message_display()
                    ));
                    self.prev_read.insert(who.to_string(), e.status());
                }
                return None;
            }
            Ok(raw) => {
                if prev != AsynStatus::Success {
                    self.print(format_args!(
                        "devMPC::devMPCCallback {who} read status back to normal"
                    ));
                    self.prev_read.insert(who.to_string(), AsynStatus::Success);
                }
                raw
            }
        };
        if raw.len() < 4 {
            self.print(format_args!(
                "devMPC::devMPCCallback {who} message too small={}",
                raw.len()
            ));
            return None;
        }
        Some(protocol::parse_reply(&String::from_utf8_lossy(&raw)).unwrap_or_default())
    }

    /// One read routine's transaction.
    fn transact(&mut self, command: u8, data: &str, who: &str) -> Option<String> {
        let frame = protocol::build_command(self.address, command, data);
        self.callback(&frame, who)
    }

    /// `checkRtnSize`.
    fn rtn_size_ok(&self, who: &str, rec: &str, min: usize) -> bool {
        if rec.len() < min {
            self.print(format_args!(
                "devMPC, {who} rtnSize={}, should be >{min}, response={rec}",
                rec.len()
            ));
            return false;
        }
        true
    }

    /// `readSi`'s length check.
    fn si_ok(&self, who: &str, rec: &str) -> bool {
        if rec.len() > 39 {
            self.print(format_args!("devMPC::readSi message too big in {who} "));
            return false;
        }
        true
    }

    fn poll_supply(&mut self, addr: i32, supply: u8) {
        let p = self.params;
        let s = supply.to_string();
        let mut values: Vec<ParamSetValue> = Vec::new();

        let who = format!("MPC_STATUS[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_STATUS, &s, &who)
            && self.si_ok(&who, &rec)
            && !rec.is_empty()
        {
            values.push(ParamSetValue::new(
                p.status,
                addr,
                ParamValue::Octet(rec.into_bytes().into()),
            ));
        }
        let who = format!("MPC_PRESSURE[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_PRESSURE, &s, &who)
            && self.rtn_size_ok(&who, &rec, 9)
            && let Ok(reading) = protocol::parse_pressure(&rec)
        {
            values.push(ParamSetValue::new(
                p.pressure,
                addr,
                ParamValue::Float64(reading.value),
            ));
            values.push(ParamSetValue::new(
                p.pressure_egu,
                addr,
                ParamValue::Octet(reading.egu.into_bytes().into()),
            ));
        }
        let who = format!("MPC_CURRENT[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_CURRENT, &s, &who)
            && self.rtn_size_ok(&who, &rec, 7)
            && let Ok(reading) = protocol::parse_current(&rec)
        {
            values.push(ParamSetValue::new(
                p.current,
                addr,
                ParamValue::Float64(reading.value),
            ));
        }
        let who = format!("MPC_VOLTAGE[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_VOLTAGE, &s, &who)
            && self.rtn_size_ok(&who, &rec, 1)
            && let Ok(reading) = protocol::parse_voltage(&rec)
        {
            values.push(ParamSetValue::new(
                p.voltage,
                addr,
                ParamValue::Float64(reading.value),
            ));
        }
        let who = format!("MPC_SIZE[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_SIZE, &s, &who) {
            if !rec.contains('L') {
                self.print(format_args!(
                    "devMPC::readAi, {who} cannot find L, response={rec}"
                ));
            } else if let Ok(reading) = protocol::parse_size(&rec) {
                values.push(ParamSetValue::new(
                    p.size,
                    addr,
                    ParamValue::Float64(reading.value),
                ));
            }
        }
        for pair in 0..NUM_SETPOINT_PAIRS {
            let number = protocol::setpoint_number(pair, supply);
            // C reads the value (ai) and the state (bi) as two records; one
            // read serves both here, so its lines carry the value's name.
            let who = format!("MPC_SP{}_VAL[{addr}]", pair + 1);
            let Some(rec) = self.transact(cmd::READ_SETPOINT, &number.to_string(), &who) else {
                continue;
            };
            if self.rtn_size_ok(&who, &rec, 11)
                && let Ok(reading) = protocol::parse_setpoint_value(&rec)
            {
                values.push(ParamSetValue::new(
                    p.setpoint_value[pair],
                    addr,
                    ParamValue::Float64(reading.value),
                ));
            }
            if let Ok(state) = protocol::parse_setpoint_state(&rec) {
                values.push(ParamSetValue::new(
                    p.setpoint_state[pair],
                    addr,
                    ParamValue::Int32(state),
                ));
            }
        }
        let who = format!("MPC_AUTO_RESTART[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_AUTO_RESTART, "", &who)
            && !rec.is_empty()
        {
            values.push(ParamSetValue::new(
                p.auto_restart,
                addr,
                ParamValue::Int32(protocol::parse_auto_restart(&rec)),
            ));
        }
        let who = format!("MPC_TSP_STATUS[{addr}]");
        if let Some(rec) = self.transact(cmd::READ_TSP_STATUS, "", &who)
            && self.si_ok(&who, &rec)
            && !rec.is_empty()
        {
            values.push(ParamSetValue::new(
                p.tsp_status,
                addr,
                ParamValue::Octet(rec.into_bytes().into()),
            ));
        }

        if !values.is_empty() {
            let _ = self.handle.set_params_and_notify_blocking(addr, values);
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
        let MpcCommand::Write { frame, who, record } = command;
        if let Some(rec) = self.callback(&frame, &who)
            && rec.len() > 2
        {
            self.print(format_args!("devMPC::{record} message too big in {who}"));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use super::*;
    use crate::worker::test_support::{answering_transport, refused_transport, traced};

    fn params() -> MpcParams {
        let mut next = 0..;
        let mut id = || next.next().unwrap();
        MpcParams {
            status: id(),
            pressure: id(),
            pressure_egu: id(),
            current: id(),
            voltage: id(),
            size: id(),
            setpoint_value: std::array::from_fn(|_| id()),
            setpoint_state: std::array::from_fn(|_| id()),
            auto_restart: id(),
            tsp_status: id(),
            set_unit: id(),
            set_display: id(),
            set_size: id(),
            set_setpoint: std::array::from_fn(|_| id()),
            start_stop: id(),
            keyboard_lock: id(),
            set_auto_restart: id(),
            tsp_timed: id(),
            tsp_off: id(),
            tsp_filament: id(),
            tsp_clear: id(),
            tsp_auto_advance: id(),
            tsp_continuous: id(),
            tsp_sublimation: id(),
            tsp_degas: id(),
        }
    }

    /// devMPCCallback's read error prints once per record until the status
    /// changes, then "back to normal"; a short OK payload trips checkRtnSize
    /// and readAi's missing-L line.
    #[test]
    fn read_errors_print_on_change_and_short_payloads_print_rtn_size() {
        let (transport, path, runtime, answer) =
            answering_transport("MPC_DEVICE", b"05 OK 00 XX 3B");
        let mut worker = MpcWorker::new(transport, runtime.port_handle().clone(), params(), 5);
        worker.poll();
        worker.poll();
        answer.store(true, Ordering::SeqCst);
        worker.poll();
        worker.handle(MpcCommand::Write {
            frame: "~ 05 37 1 00".into(),
            who: "MPC_START[0]".into(),
            record: "writeBo",
        });
        let lines = traced(&path);
        let count = |needle: &str| lines.iter().filter(|l| l.contains(needle)).count();

        // 11 records per supply, two supplies: one error each across two polls.
        assert_eq!(count(" read error, status=1 error= "), 22, "{lines:?}");
        assert_eq!(count(" read status back to normal"), 22, "{lines:?}");
        assert!(lines.iter().any(|l| {
            l.ends_with("devMPC::devMPCCallback MPC_STATUS[0] read status back to normal")
        }));
        assert!(
            lines.iter().any(
                |l| l.ends_with("devMPC, MPC_PRESSURE[0] rtnSize=2, should be >9, response=XX")
            )
        );
        assert!(
            lines
                .iter()
                .any(|l| l.ends_with("devMPC::readAi, MPC_SIZE[1] cannot find L, response=XX"))
        );
        assert!(
            lines.iter().any(
                |l| l.ends_with("devMPC, MPC_SP1_VAL[0] rtnSize=2, should be >11, response=XX")
            )
        );
        // "XX" is not more than 2 characters, so the write prints nothing.
        assert_eq!(count("message too big"), 0, "{lines:?}");
    }

    /// A request the port refused never reached C's callback: no line.
    #[test]
    fn a_refused_request_prints_nothing() {
        let (transport, path, runtime) = refused_transport("MPC_REFUSED");
        MpcWorker::new(transport, runtime.port_handle().clone(), params(), 5).poll();
        assert_eq!(traced(&path), Vec::<String>::new());
    }
}
