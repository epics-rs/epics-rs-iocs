//! `specsAnalyserConfig` naming a driver port that does not exist.
//!
//! C's `pasynOctetSyncIO->connect` fails inside `makeConnection`, the
//! constructor carries on (specsAnalyser.cpp:153-177) and the detector comes
//! up in error — the IOC still boots.

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::registry::claim_port;

use ad_specs_analyser::create_specs_analyser_detector;

#[test]
fn a_missing_driver_port_leaves_the_detector_in_error() {
    let claim = claim_port("SPECS_NODRV").unwrap();
    let runtime = create_specs_analyser_detector(&claim, "SPECS_NODRV_IP", 0, 0)
        .expect("C leaves the port registered");
    let status = runtime
        .port_handle()
        .read_int32_blocking(runtime.ad_params.status, 0)
        .unwrap();
    assert_eq!(status, ADStatus::Error as i32);
}
