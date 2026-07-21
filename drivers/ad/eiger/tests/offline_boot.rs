//! An offline detector must not stop the IOC from booting.
//!
//! `eigerDetectorConfig` used to propagate the first failed REST request out of
//! the configure command, which left no port behind and aborted the IOC before
//! `iocInit`. C's constructor logs "Eiger FAILED TO CONNECT" and returns
//! (eigerDetector.cpp:229-235), leaving a usable port.

use std::net::TcpListener;

use epics_rs::ad_core::driver::ADStatus;

use eiger::create_eiger_detector_on_port;
use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;

/// A TCP port nothing is listening on — the same thing the driver sees when the
/// detector is powered off.
fn dead_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    port
}

#[test]
fn a_failed_version_negotiation_leaves_a_usable_client() {
    let mut rest = RestApi::new("127.0.0.1", dead_port());
    assert!(rest.negotiate_api_version().is_err());
    // The bootstrap paths stay in place, which is what lets the caller carry on.
    assert_eq!(rest.api_version(), ApiVersion::V1_6_0);
}

#[test]
fn the_driver_is_created_when_the_detector_does_not_answer() {
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _ctl_rx, _init_rx, _restart_rx) = tasks::signals();
    let cfg = EigerConfig {
        port_name: "EIGER_OFFLINE_DRV".to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };

    let driver = EigerDriver::new(cfg, rest, signals)
        .expect("the port must be created even though every REST request failed");

    // The failure has to be visible from the record layer, not just the log.
    let base = &driver.ad.port_base;
    assert_eq!(
        base.get_int32_param(driver.ad.params.status, 0).unwrap(),
        ADStatus::Disconnected as i32
    );
    assert_eq!(
        base.get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        "Eiger FAILED TO CONNECT"
    );
    // Set before the detector is first touched, so it survives the early return.
    assert_eq!(
        base.get_string_param(driver.ad.params.base.driver_version, 0)
            .unwrap(),
        env!("CARGO_PKG_VERSION")
    );
}

#[test]
fn the_configure_command_succeeds_when_the_detector_does_not_answer() {
    let runtime = create_eiger_detector_on_port("EIGER_OFFLINE_CFG", "127.0.0.1", dead_port(), 0)
        .expect("eigerDetectorConfig must leave a port behind for iocInit to use");
    // The background tasks still exist, so the driver recovers when the
    // detector comes back.
    assert!(runtime.num_tasks() > 0);
}
