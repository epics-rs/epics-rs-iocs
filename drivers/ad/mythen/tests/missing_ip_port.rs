//! `mythenConfig` naming an IP port that does not exist.
//!
//! C's `pasynOctetSyncIO->connect` fails, the constructor prints and returns
//! before its `createParam`s (mythen.cpp:1300-1305), and the port stays
//! registered with ADDriver's parameters only — the IOC still boots.

use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::registry::claim_port;

use mythen::create_mythen_detector;

#[test]
fn a_missing_ip_port_leaves_an_addriver_port_without_mythen_params() {
    let claim = claim_port("MYTHEN_NOIP").unwrap();
    let runtime =
        create_mythen_detector(&claim, "MYTHEN_NOIP_IP", 0).expect("C leaves the port registered");

    assert!(runtime.acquisition.is_none());
    let handle = runtime.port_handle();
    assert!(
        handle
            .drv_user_create_blocking(&DrvUserRequest::new("ACQUIRE", 0))
            .is_ok()
    );
    assert!(
        handle
            .drv_user_create_blocking(&DrvUserRequest::new("SD_SETTING", 0))
            .is_err()
    );
}
