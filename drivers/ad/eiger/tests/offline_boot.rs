//! What `eigerDetectorConfig` does with a detector that does not answer.
//!
//! The API version is read in C's `RestAPI` constructor, which throws when it
//! cannot (restApi.cpp:258-275), after the ADDriver base registered the port,
//! so the port stays with ADDriver's parameters only. A detector that answered
//! that and then fails the state fetch keeps its full port: C logs "Eiger
//! FAILED TO CONNECT" and returns (eigerDetector.cpp:231-235).

use std::net::TcpListener;

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::error::AsynError;

use eiger::create_eiger_detector_on_port;
use eiger::driver::{EigerConfig, EigerDriver, MAX_ASYN_ADDRESS, MONITOR_ASYN_ADDRESS};
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
        b"3.6.0"
    );
}

#[test]
fn an_unanswered_version_read_leaves_the_base_port() {
    let runtime = create_eiger_detector_on_port(
        &epics_rs::asyn::registry::claim_port("EIGER_OFFLINE_CFG").unwrap(),
        "127.0.0.1",
        dead_port(),
        0,
    )
    .expect("the port outlives the exception");
    assert_eq!(runtime.thrown, Some("unable to parse response json"));
    assert_eq!(runtime.num_tasks(), 0);
}

#[test]
fn the_port_has_cs_eleven_addresses() {
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: "EIGER_MAX_ADDR".to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    let driver = EigerDriver::new(cfg, rest, signals).expect("driver");

    // ADDriver(portName, MAX_ASYN_ADDRESS, ...), eigerDetector.cpp:190.
    let base = &driver.ad.port_base;
    let counter = driver.ad.params.base.array_counter;
    // Lists past 0 are never set, so a held address reads ParamUndefined.
    assert!(matches!(
        base.get_int32_param(counter, MONITOR_ASYN_ADDRESS as i32),
        Err(AsynError::ParamUndefined(_))
    ));
    assert!(matches!(
        base.get_int32_param(counter, MAX_ASYN_ADDRESS as i32),
        Err(AsynError::ParamInvalidList(_))
    ));
}

#[test]
fn a_refused_write_prints_the_param_librarys_status() {
    use std::sync::Arc;

    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::{AsynUser, UserTrace};

    let port = "EIGER_STATUS_CODE";
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: port.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    let mut driver = EigerDriver::new(cfg, rest, signals).expect("driver");
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("eiger-status-{}.log", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    // ADDriver::writeFloat64 on an int param: setDoubleParam's asynParamWrongType.
    let reason = driver.ad.params.base.array_counter;
    let mut user = AsynUser::new(reason);
    user.trace = Some(UserTrace {
        manager,
        port: port.into(),
    });
    assert!(driver.write_float64(&mut user, 1.0).is_err());
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert!(
        text.contains(&format!(
            "eigerDetector:writeFloat64 error, status=8 function={reason}, value=1.000000"
        )),
        "{text}"
    );
}

/// eigerDetector.cpp:2108-2120: an EIG_ name drvUserCreate cannot place is
/// an ERR_ARGS line and asynError, with no errorMessage.
#[test]
fn an_unplaceable_eig_name_is_traced_with_no_message() {
    use std::sync::Arc;

    use epics_rs::asyn::port::{DrvUserRequest, PortDriver};
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;

    let port = "EIGER_DRV_USER";
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: port.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    let mut driver = EigerDriver::new(cfg, rest, signals).expect("driver");
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("eiger-drvuser-{}.log", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    for name in ["EIG_XCI_x", "EIG_DCQ_x"] {
        let e = driver
            .drv_user_create(&DrvUserRequest::new(name, 0))
            .unwrap_err();
        assert_eq!(e.c_status_code(), 3);
        assert_eq!(e.message(), "");
    }
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert!(
        text.contains(
            "eigerDetector::drvUserCreate: [EIG_XCI_x] couldn't match XC to any subsystem"
        ),
        "{text}"
    );
    assert!(
        text.contains("eigerDetector::drvUserCreate: [EIG_DCQ_x] couldn't match Q to an asyn type"),
        "{text}"
    );
}

/// A failed put prints the parameter layer's ERR_ARGS chain
/// (eigerParam.cpp:423-445, :807-808) and nothing of eigerDetector's own.
#[test]
fn a_failed_put_prints_only_the_parameter_chain() {
    use std::sync::Arc;

    use epics_rs::asyn::port::PortDriver;
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::AsynUser;

    let port = "EIGER_PUT_CHAIN";
    let rest = RestApi::new("127.0.0.1", dead_port());
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: port.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    let mut driver = EigerDriver::new(cfg, rest, signals).expect("driver");
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("eiger-put-chain-{}.log", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let mut user = AsynUser::new(driver.p.trigger_mode);
    assert!(driver.write_int32(&mut user, 0).is_err());
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    let lines: Vec<&str> = text
        .lines()
        .map(|l| l.splitn(3, ' ').nth(2).unwrap_or(l))
        .filter(|l| !l.is_empty() && !l.starts_with('['))
        .collect();
    assert!(
        lines.iter().all(|l| l.starts_with("Param[")
            || l.starts_with("eigerDetector::writeInt32: error status=")),
        "{text}"
    );
    // TRIGGER_MODE's driver-set enum values leave it unfetched, so put<int>
    // fetches first (eigerParam.cpp:807-808).
    assert!(
        lines
            .iter()
            .any(|l| l.ends_with("[param=TRIGGER_MODE] underlying baseFetch failed")),
        "{text}"
    );
}
