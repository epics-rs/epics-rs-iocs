//! Port construction: one factory per `*Config` iocsh command.
//!
//! Each factory builds the driver, hands it to the asyn-rs port runtime and
//! starts the driver's poll thread against the resulting port handle. The IOC
//! keeps the returned [`UrPortRuntime`] alive for the life of the process.

use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};

use crate::drivers::{control, dashboard, gripper, io, receive};

/// A live port: the runtime actor plus the poll thread feeding it.
pub struct UrPortRuntime {
    pub runtime_handle: PortRuntimeHandle,
    _poller: Option<std::thread::JoinHandle<()>>,
}

impl UrPortRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// `URDashboardConfig(port, ip, poll_period)`.
pub fn create_dashboard(
    port_name: &str,
    robot_ip: &str,
    poll_period: Duration,
) -> AsynResult<UrPortRuntime> {
    let driver = dashboard::DashboardDriver::new(port_name, robot_ip)?;
    let params = driver.params();
    let client = driver.client();
    let shared = driver.shared();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let poller = dashboard::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        client,
        shared,
        poll_period,
    );
    Ok(UrPortRuntime {
        runtime_handle,
        _poller: Some(poller),
    })
}

/// `RTDEReceiveConfig(port, ip, poll_period)`.
pub fn create_receive(
    port_name: &str,
    robot_ip: &str,
    poll_period: Duration,
) -> AsynResult<UrPortRuntime> {
    let driver = receive::ReceiveDriver::new(port_name, robot_ip)?;
    let params = driver.params();
    let iface = driver.iface();
    let shared = driver.shared();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let poller = receive::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        iface,
        shared,
        poll_period,
    );
    Ok(UrPortRuntime {
        runtime_handle,
        _poller: Some(poller),
    })
}

/// `RTDEInOutConfig(port, ip, poll_period)` — the I/O driver is write-only and
/// has no poll thread (`rtde_io_driver.cpp` creates none; its `poll_period`
/// argument is accepted and ignored, as upstream does).
pub fn create_io(port_name: &str, robot_ip: &str) -> AsynResult<UrPortRuntime> {
    let driver = io::IoDriver::new(port_name, robot_ip)?;
    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    Ok(UrPortRuntime {
        runtime_handle,
        _poller: None,
    })
}

/// `RTDEControlConfig(port, dashboard_port, receive_port, poll_period)`.
pub fn create_control(
    port_name: &str,
    dashboard_port: &str,
    receive_port: &str,
    poll_period: Duration,
) -> AsynResult<UrPortRuntime> {
    let driver = control::ControlDriver::new(port_name, dashboard_port, receive_port)?;
    let params = driver.params();
    let inner = driver.inner();
    let receive = driver.receive();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let poller = control::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        inner,
        receive,
        poll_period,
    );
    Ok(UrPortRuntime {
        runtime_handle,
        _poller: Some(poller),
    })
}

/// `URGripperConfig(port, dashboard_port, poll_period)`.
pub fn create_gripper(
    port_name: &str,
    dashboard_port: &str,
    poll_period: Duration,
) -> AsynResult<UrPortRuntime> {
    let driver = gripper::GripperDriver::new(port_name, dashboard_port)?;
    let params = driver.params();
    let g = driver.gripper();
    let dash = driver.dashboard();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let poller = gripper::start_poller(
        runtime_handle.port_handle().clone(),
        params,
        g,
        dash,
        poll_period,
    );
    Ok(UrPortRuntime {
        runtime_handle,
        _poller: Some(poller),
    })
}
