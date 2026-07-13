//! Televac vacuum gauge controller (`ipApp/src/devTelevac.c`).
//!
//! The C device support is read-only: pressures, the relay ON/OFF thresholds and
//! the relay states. The port keeps that surface. asyn address = station index
//! for `TVAC_PRESSURE` and relay index for the relay parameters, both 0-based —
//! the `parameter` field of the C link (`@asyn(port addr)GET_PRESSURE 3`).

pub mod protocol;

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use crate::connect::connect_octet;
use crate::runtime::IpPortRuntime;
use crate::worker::{self, DeviceWorker, Transport};
use protocol::{MAX_RELAYS, MAX_STATIONS};

/// Command timeout (`Televac_TIMEOUT`, `devTelevac.c:59`).
pub const TELEVAC_TIMEOUT: Duration = Duration::from_secs(1);

/// The Televac port is read-only, so the worker takes no commands.
pub enum TelevacCommand {}

#[derive(Clone, Copy)]
pub struct TelevacParams {
    pub pressure: usize,
    pub relay_on: usize,
    pub relay_off: usize,
    pub relay_state: usize,
}

impl TelevacParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            pressure: base.create_param("TVAC_PRESSURE", ParamType::Float64)?,
            relay_on: base.create_param("TVAC_RELAY_ON", ParamType::Float64)?,
            relay_off: base.create_param("TVAC_RELAY_OFF", ParamType::Float64)?,
            relay_state: base.create_param("TVAC_RELAY_STATE", ParamType::Int32)?,
        })
    }
}

pub struct TelevacDriver {
    base: PortDriverBase,
    params: TelevacParams,
    /// The port has no writable parameters, but holding the sender keeps the
    /// worker's receiver open for as long as the port lives.
    _commands: std::sync::mpsc::Sender<TelevacCommand>,
}

impl TelevacDriver {
    pub fn new(
        port_name: &str,
        commands: std::sync::mpsc::Sender<TelevacCommand>,
    ) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            MAX_STATIONS,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let params = TelevacParams::create(&mut base)?;
        Ok(Self {
            base,
            params,
            _commands: commands,
        })
    }

    pub fn params(&self) -> TelevacParams {
        self.params
    }
}

impl PortDriver for TelevacDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
}

pub struct TelevacWorker {
    transport: Transport,
    handle: PortHandle,
    params: TelevacParams,
    stations: u8,
    relays: u8,
}

impl TelevacWorker {
    fn transact(&self, command: &str) -> Result<String, String> {
        self.transport.write_read(command.as_bytes())
    }

    fn publish(&self, addr: i32, values: Vec<ParamSetValue>) {
        if !values.is_empty()
            && let Err(e) = self.handle.set_params_and_notify_blocking(addr, values)
        {
            log::error!("Televac: publishing address {addr} failed: {e}");
        }
    }
}

impl DeviceWorker for TelevacWorker {
    type Command = TelevacCommand;

    fn poll(&mut self) {
        let p = self.params;

        for station in 1..=self.stations {
            let addr = i32::from(station) - 1;
            match self.transact(&protocol::read_pressure(station)) {
                Ok(raw) => self.publish(
                    addr,
                    vec![ParamSetValue::new(
                        p.pressure,
                        addr,
                        ParamValue::Float64(protocol::parse_pressure(&raw)),
                    )],
                ),
                // The C published a stale buffer on a read error; here the
                // parameter simply keeps its last good value.
                Err(e) => log::error!("Televac: read station {station} failed: {e}"),
            }
        }

        let states = match self
            .transact(&protocol::read_relay_states())
            .and_then(|raw| protocol::parse_relay_states(&raw).map_err(|e| e.to_string()))
        {
            Ok(states) => Some(states),
            Err(e) => {
                log::error!("Televac: read relay states failed: {e}");
                None
            }
        };

        for relay in 1..=self.relays {
            let addr = i32::from(relay) - 1;
            let mut values = Vec::new();
            match self.transact(&protocol::read_relay_on(relay)) {
                Ok(raw) => values.push(ParamSetValue::new(
                    p.relay_on,
                    addr,
                    ParamValue::Float64(protocol::parse_pressure(&raw)),
                )),
                Err(e) => log::error!("Televac: read relay {relay} ON failed: {e}"),
            }
            match self.transact(&protocol::read_relay_off(relay)) {
                Ok(raw) => values.push(ParamSetValue::new(
                    p.relay_off,
                    addr,
                    ParamValue::Float64(protocol::parse_pressure(&raw)),
                )),
                Err(e) => log::error!("Televac: read relay {relay} OFF failed: {e}"),
            }
            if let Some(states) = states {
                values.push(ParamSetValue::new(
                    p.relay_state,
                    addr,
                    ParamValue::Int32(i32::from(protocol::relay_is_set(states, relay))),
                ));
            }
            self.publish(addr, values);
        }
    }

    fn handle(&mut self, _command: Self::Command) {}
}

/// `TelevacConfig(port, octetPort, numStations, numRelays, [pollPeriod])`.
pub fn create_televac(
    port_name: &str,
    octet_port: &str,
    stations: u8,
    relays: u8,
    poll_period: Duration,
) -> AsynResult<IpPortRuntime> {
    if stations == 0 || usize::from(stations) > MAX_STATIONS {
        return Err(crate::asyn_error(format!(
            "Televac: numStations must be 1..{MAX_STATIONS}, got {stations}"
        )));
    }
    if usize::from(relays) > MAX_RELAYS {
        return Err(crate::asyn_error(format!(
            "Televac: numRelays must be 0..{MAX_RELAYS}, got {relays}"
        )));
    }

    let io = connect_octet(octet_port, TELEVAC_TIMEOUT).map_err(crate::asyn_error)?;
    let (commands, rx) = std::sync::mpsc::channel::<TelevacCommand>();
    let driver = TelevacDriver::new(port_name, commands)?;
    let params = driver.params();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let worker = TelevacWorker {
        transport: Transport::new(io),
        handle: runtime_handle.port_handle().clone(),
        params,
        stations,
        relays,
    };
    let thread = worker::spawn(port_name, worker, rx, poll_period);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}
