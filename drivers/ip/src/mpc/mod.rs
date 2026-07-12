//! MPC / Digitel ion-pump controller (`ipApp/src/devMPC.c`).

pub mod driver;
pub mod protocol;

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use crate::connect::connect_octet;
use crate::worker::{self, Transport};
use driver::{MPC_TIMEOUT, MpcDriver, MpcWorker};

pub use crate::runtime::IpPortRuntime;

/// `MPCConfig(port, octetPort, address, pollPeriodMs)` — create the MPC port on
/// top of an already-configured serial/IP octet port.
pub fn create_mpc(
    port_name: &str,
    octet_port: &str,
    address: u8,
    poll_period: Duration,
) -> AsynResult<IpPortRuntime> {
    let io = connect_octet(octet_port, MPC_TIMEOUT).map_err(crate::asyn_error)?;
    let (driver, commands) = MpcDriver::new(port_name, address)?;
    let params = driver.params();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let worker = MpcWorker::new(
        Transport::new(io),
        runtime_handle.port_handle().clone(),
        params,
        address,
    );
    let thread = worker::spawn(port_name, worker, commands, poll_period);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}
