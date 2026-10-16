//! What `eigerDetectorConfig` does with a detector that does not answer.
//!
//! The API version is read in C's `RestAPI` constructor, which throws when it
//! cannot, so no port is made (restApi.cpp:258-275). A detector that answered
//! that and then fails the state fetch keeps its port: C logs "Eiger FAILED TO
//! CONNECT" and returns (eigerDetector.cpp:231-235).

use std::net::TcpListener;

use epics_rs::ad_core::driver::ADStatus;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;
use eiger::{ConfigError, create_eiger_detector_on_port};

/// A TCP port nothing is listening on — the same thing the driver sees when the
/// detector is powered off.
fn dead_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    port
}

#[test]
fn a_failed_version_read_fails_the_client_with_cs_text() {
    assert_eq!(
        RestApi::open("127.0.0.1", dead_port()).err(),
        Some("unable to parse response json")
    );
    assert_eq!(
        RestApi::open("no-such-host.invalid", 80).err(),
        Some("invalid hostname")
    );
}

#[test]
fn the_driver_is_created_when_the_detector_does_not_answer() {
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _receivers) = tasks::signals();
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
        b"Eiger FAILED TO CONNECT"
    );
    // Set before the detector is first touched, so it survives the early return.
    assert_eq!(
        base.get_string_param(driver.ad.params.base.driver_version, 0)
            .unwrap(),
        env!("CARGO_PKG_VERSION").as_bytes()
    );
}

#[test]
fn the_configure_command_fails_when_the_detector_does_not_answer() {
    let result = create_eiger_detector_on_port("EIGER_OFFLINE_CFG", "127.0.0.1", dead_port(), 0);
    assert!(matches!(
        result,
        Err(ConfigError::Rest("unable to parse response json"))
    ));
}
