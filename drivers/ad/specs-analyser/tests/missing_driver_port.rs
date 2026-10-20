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

/// C creates the acquisition task before makeConnection
/// (specsAnalyser.cpp:140-177), so the task's first "Waiting for the
/// acquire command" is overwritten by the failed connect's message.
#[test]
fn a_failed_connect_message_outlasts_the_task_first_pass() {
    let claim = claim_port("SPECS_NODRV_MSG").unwrap();
    let runtime = create_specs_analyser_detector(&claim, "SPECS_NODRV_MSG_IP", 0, 0)
        .expect("C leaves the port registered");
    // Room for a first pass that ran after the constructor to land.
    std::thread::sleep(std::time::Duration::from_millis(300));
    let message = runtime
        .port_handle()
        .get_string_param_blocking(runtime.ad_params.status_message, 0, 256)
        .unwrap();
    assert_eq!(
        String::from_utf8_lossy(&message),
        "Failed to initialise - check connection"
    );
}
