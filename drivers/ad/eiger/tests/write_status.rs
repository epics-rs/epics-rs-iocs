//! Statuses `eigerDetector::writeInt32` / `writeFloat64` ignore must not end
//! the write (eigerDetector.cpp:513-571, 627-654).

use std::net::TcpListener;

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::user::AsynUser;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;

fn offline_driver(name: &str) -> EigerDriver {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    let (signals, receivers) = tasks::signals();
    // The task ends must outlive the test.
    std::mem::forget(receivers);
    let cfg = EigerConfig {
        port_name: name.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    EigerDriver::new(cfg, RestApi::new("127.0.0.1", port), signals).expect("driver")
}

#[test]
fn a_stop_completes_when_the_abort_request_fails() {
    let mut driver = offline_driver("EIGER_WS_STOP");
    let status = driver.ad.params.status;
    let acquire = driver.ad.params.acquire;
    driver
        .ad
        .port_base
        .set_int32_param(status, 0, ADStatus::Acquire as i32)
        .unwrap();
    driver.ad.port_base.set_int32_param(acquire, 0, 1).unwrap();

    let mut user = AsynUser::new(acquire);
    driver
        .write_int32(&mut user, 0)
        .expect("C ignores mApi.abort()'s status");
    let base = &driver.ad.port_base;
    assert_eq!(base.get_int32_param(acquire, 0).unwrap(), 0);
    assert_eq!(
        base.get_int32_param(status, 0).unwrap(),
        ADStatus::Aborted as i32
    );
}

#[test]
fn a_photon_energy_write_reports_done_when_the_put_fails() {
    let mut driver = offline_driver("EIGER_WS_ENERGY");
    let mut user = AsynUser::new(driver.p.photon_energy);
    driver
        .write_float64(&mut user, 8000.0)
        .expect("C ignores mPhotonEnergy->put()'s status");
    assert_eq!(
        driver
            .ad
            .port_base
            .get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        b"Photon Energy set"
    );
}
