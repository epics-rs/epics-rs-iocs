//! Pfeiffer TPG261 / TPG262 gauge controller (`ipApp/src/devTPG261.c`).

pub mod driver;
pub mod protocol;

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use crate::connect::connect_octet;
use crate::runtime::IpPortRuntime;
use crate::worker::{self, Transport};
use driver::{TPG261_TIMEOUT, TpgDriver, TpgWorker};

/// `TPG261Config(port, octetPort, [pollPeriod])`.
pub fn create_tpg261(
    port_name: &str,
    octet_port: &str,
    poll_period: Duration,
) -> AsynResult<IpPortRuntime> {
    let io = connect_octet(octet_port, TPG261_TIMEOUT).map_err(crate::asyn_error)?;
    let (driver, commands) = TpgDriver::new(port_name)?;
    let params = driver.params();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let worker = TpgWorker::new(
        Transport::new(io),
        runtime_handle.port_handle().clone(),
        params,
    );
    let thread = worker::spawn(port_name, worker, commands, poll_period);
    Ok(IpPortRuntime::new(runtime_handle, thread))
}
