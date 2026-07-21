//! An offline detector must not stop the IOC from booting.
//!
//! C's constructor issues its queries with `status |= ...` and carries on
//! (mythen.cpp:1326-1377), so `mythenConfig` always leaves a port behind and
//! `iocInit` runs. This is the regression test for the Rust port having turned
//! those queries into `?`-propagation, which aborted the whole IOC.

use std::net::TcpListener;
use std::sync::Arc;

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// An asyn IP port aimed at a TCP port nothing is listening on — the same thing
/// the driver sees when the detector is powered off.
fn dead_ip_port(name: &str) -> PortHandle {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    drop(listener);

    let driver = DrvAsynIPPort::new(name, &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    handle
}

#[test]
fn the_driver_is_created_when_the_detector_does_not_answer() {
    let det = Arc::new(Detector::new(Transport::new(dead_ip_port(
        "MYTHEN_OFFLINE_IP",
    ))));
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);

    let driver = MythenDriver::new("MYTHEN_OFFLINE", det, 0, start_tx)
        .expect("the port must be created even though every detector query failed");

    // Failed queries leave the parameters at their defaults and the detector
    // reported as disconnected, which is what tells an operator why.
    let base = &driver.ad.port_base;
    assert_eq!(
        base.get_int32_param(driver.ad.params.status, 0).unwrap(),
        ADStatus::Disconnected as i32
    );
    assert_eq!(
        base.get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        "Mythen FAILED TO CONNECT"
    );
    assert_eq!(base.get_int32_param(driver.p.nmodules, 0).unwrap(), 0);
    assert_eq!(
        base.get_string_param(driver.p.firmware_version, 0).unwrap(),
        ""
    );
    // The fixed parameters C sets regardless of the detector are still there.
    assert_eq!(
        base.get_string_param(driver.ad.params.base.manufacturer, 0)
            .unwrap(),
        "Dectris"
    );
}
