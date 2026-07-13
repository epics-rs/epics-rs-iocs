//! Heidenhain ND261 display unit (`ipApp/src/devAiHeidND261.c`).
//!
//! Read-only, single address: the C device support drove one ai record per
//! ND261, polling it at the record's SCAN rate. The port polls the readout and
//! publishes it as `ND261_POSITION`; `ND261_STATUS` says whether the last read
//! decoded, which is where the C's `VAL = 99999.0, UDF = 1` failure signal goes
//! (a port driver cannot write UDF).

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

/// Read timeout (`pPvt->timeout`, `devAiHeidND261.c:102`).
pub const ND261_TIMEOUT: Duration = Duration::from_secs(1);

/// The ND261 port is read-only, so the worker takes no commands.
pub enum Nd261Command {}

/// `ND261_STATUS`.
pub mod status {
    pub const OK: i32 = 0;
    /// The read failed or the reply did not decode — the C's
    /// `VAL = 99999.0; UDF = 1` case.
    pub const READ_ERROR: i32 = 1;
}

#[derive(Clone, Copy)]
pub struct Nd261Params {
    pub position: usize,
    pub status: usize,
}

pub struct Nd261Driver {
    base: PortDriverBase,
    params: Nd261Params,
    /// Keeps the worker's receiver open for the life of the port; the ND261 port
    /// has no writable parameters.
    _commands: std::sync::mpsc::Sender<Nd261Command>,
}

impl Nd261Driver {
    pub fn new(
        port_name: &str,
        commands: std::sync::mpsc::Sender<Nd261Command>,
    ) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        let params = Nd261Params {
            position: base.create_param("ND261_POSITION", ParamType::Float64)?,
            status: base.create_param("ND261_STATUS", ParamType::Int32)?,
        };
        Ok(Self {
            base,
            params,
            _commands: commands,
        })
    }

    pub fn params(&self) -> Nd261Params {
        self.params
    }
}

impl PortDriver for Nd261Driver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
}

pub struct Nd261Worker {
    transport: Transport,
    handle: PortHandle,
    params: Nd261Params,
}

impl DeviceWorker for Nd261Worker {
    type Command = Nd261Command;

    fn poll(&mut self) {
        let p = self.params;
        let reply = self
            .transport
            .write_read(&protocol::READ_COMMAND)
            .and_then(|reply| protocol::parse_position(&reply));

        let mut values = Vec::new();
        match reply {
            Ok(position) => {
                values.push(ParamSetValue::new(
                    p.position,
                    0,
                    ParamValue::Float64(position),
                ));
                values.push(ParamSetValue::new(
                    p.status,
                    0,
                    ParamValue::Int32(status::OK),
                ));
            }
            Err(e) => {
                log::error!("ND261: read failed: {e}");
                values.push(ParamSetValue::new(
                    p.position,
                    0,
                    ParamValue::Float64(protocol::INVALID_VALUE),
                ));
                values.push(ParamSetValue::new(
                    p.status,
                    0,
                    ParamValue::Int32(status::READ_ERROR),
                ));
            }
        }
        if let Err(e) = self.handle.set_params_and_notify_blocking(0, values) {
            log::error!("ND261: publishing failed: {e}");
        }
    }

    fn handle(&mut self, _command: Self::Command) {}
}

/// `ND261Config(port, octetPort, [pollPeriod])`.
pub fn create_nd261(
    port_name: &str,
    octet_port: &str,
    poll_period: Duration,
) -> AsynResult<IpPortRuntime> {
    let io = connect_octet(octet_port, ND261_TIMEOUT).map_err(crate::asyn_error)?;
    let (commands, rx) = std::sync::mpsc::channel::<Nd261Command>();
    let driver = Nd261Driver::new(port_name, commands)?;
    let params = driver.params();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let worker = Nd261Worker {
        transport: Transport::new(io),
        handle: runtime_handle.port_handle().clone(),
        params,
    };
    let thread = worker::spawn(port_name, worker, rx, poll_period);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}
