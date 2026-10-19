//! A failed step of `writeInt32` / `writeFloat64` does not end the write: C
//! ORs each step into `status` and still runs `getSettings` and the callbacks
//! (mythen.cpp:1121-1159, 1195-1216).

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, mpsc};

use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::user::AsynUser;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// Privileged port 1: no test can bind it, so a connect is always refused.
/// A freed ephemeral port can be taken by a concurrent test's listener.
const DEAD_ADDR: &str = "127.0.0.1:1";

/// A healthy detector whose module count is unknown (`-get nmodules` = -1), so
/// every module loop runs zero times while the `getSettings` queries still
/// succeed.
fn fake_detector(name: &str) -> (PortHandle, mpsc::Receiver<String>) {
    fake_detector_with(name, -1, None)
}

/// The same detector with `nmodules` modules, hanging up on `hang_up_at`.
fn fake_detector_with(
    name: &str,
    nmodules: i32,
    hang_up_at: Option<&'static str>,
) -> (PortHandle, mpsc::Receiver<String>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    let (sent, received) = mpsc::channel();

    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("connection");
        let mut buf = [0u8; 256];
        while let Ok(n) = stream.read(&mut buf) {
            if n == 0 {
                break;
            }
            let command = String::from_utf8_lossy(&buf[..n]).trim().to_string();
            if Some(command.as_str()) == hang_up_at {
                break;
            }
            let reply: Vec<u8> = match command.as_str() {
                "-get version" => b"M3.0.0\0".to_vec(),
                "-get nmodules" => nmodules.to_le_bytes().to_vec(),
                "-get status" => (1i32 << 16).to_le_bytes().to_vec(),
                "-get nbits" => 24i32.to_le_bytes().to_vec(),
                "-get time" => 10_000_000i32.to_le_bytes().to_vec(),
                "-get frames" => 1i32.to_le_bytes().to_vec(),
                "-get tau" => (-1.0f32).to_le_bytes().to_vec(),
                "-get kthresh" => 8.0f32.to_le_bytes().to_vec(),
                "-get energy" => 8.05f32.to_le_bytes().to_vec(),
                _ => 0i32.to_le_bytes().to_vec(),
            };
            if sent.send(command).is_err() || stream.write_all(&reply).is_err() {
                break;
            }
        }
    });

    let ip = format!("{name}_IP");
    let driver = DrvAsynIPPort::new(&ip, &addr).expect("ip port");
    let (runtime, _) =
        create_port_runtime(&claim_port(&ip).unwrap(), driver, RuntimeConfig::default())
            .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    (handle, received)
}

fn driver(name: &str) -> (MythenDriver, mpsc::Receiver<String>) {
    driver_on(name, fake_detector(name))
}

fn driver_on(
    name: &str,
    (handle, received): (PortHandle, mpsc::Receiver<String>),
) -> (MythenDriver, mpsc::Receiver<String>) {
    let det = Arc::new(Detector::new(Transport::new(handle), name));
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new(name, det, 0, start_tx).expect("driver");
    // Drop the constructor's queries.
    let _ = received.try_iter().count();
    (driver, received)
}

fn ran_get_settings(received: &mpsc::Receiver<String>) -> bool {
    received.try_iter().any(|c| c == "-get flatfieldcorrection")
}

#[test]
fn a_refused_int32_command_still_reads_the_settings_back() {
    let (mut driver, received) = driver("MYTHEN_WS_INT");
    // An int written to the float SDThreshold: setIntegerParam refuses it.
    let mut user = AsynUser::new(driver.p.threshold);

    assert!(driver.write_int32(&mut user, 1).is_err());
    assert!(
        ran_get_settings(&received),
        "getSettings did not run after the refused setIntegerParam"
    );
}

#[test]
fn a_refused_float64_command_still_reads_the_settings_back() {
    let (mut driver, received) = driver("MYTHEN_WS_F64");
    // A float written to the int SDSetting: setDoubleParam refuses it.
    let mut user = AsynUser::new(driver.p.setting);

    assert!(driver.write_float64(&mut user, 7.0).is_err());
    assert!(
        ran_get_settings(&received),
        "getSettings did not run after the refused setDoubleParam"
    );
}

#[test]
fn an_illegal_tau_still_reads_the_settings_back() {
    let (mut driver, received) = driver("MYTHEN_WS_TAU");
    let mut user = AsynUser::new(driver.p.tau);

    assert!(driver.write_float64(&mut user, -5.0).is_err());
    assert!(
        ran_get_settings(&received),
        "getSettings did not run after the illegal tau"
    );
}

/// mythen never writes `pasynUser->errorMessage`: a refused command and an
/// illegal tau reach the record as their status alone.
#[test]
fn a_failed_write_hands_the_record_no_message() {
    let (mut driver, _received) = driver("MYTHEN_WS_MESSAGE");
    let e = driver
        .write_int32(&mut AsynUser::new(driver.p.threshold), 1)
        .unwrap_err();
    assert_eq!(e.message(), "");
    let e = driver
        .write_float64(&mut AsynUser::new(driver.p.setting), 7.0)
        .unwrap_err();
    assert_eq!(e.message(), "");
    let e = driver
        .write_float64(&mut AsynUser::new(driver.p.tau), -5.0)
        .unwrap_err();
    assert_eq!(e.message(), "");
}

#[test]
fn a_failed_get_settings_is_an_error_status() {
    // Nothing listens, so the asyn port is down and refuses getSettings'
    // queries with Disconnected.
    let ip = DrvAsynIPPort::new("MYTHEN_WS_DEAD_IP", DEAD_ADDR).expect("ip port");
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_WS_DEAD_IP").unwrap(),
        ip,
        RuntimeConfig::default(),
    )
    .expect("ip port runtime");
    let det = Arc::new(Detector::new(
        Transport::new(runtime.port_handle().clone()),
        "MYTHEN_WS_DEAD",
    ));
    std::mem::forget(runtime);
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);
    let mut driver = MythenDriver::new("MYTHEN_WS_DEAD", det, 0, start_tx).expect("driver");

    // A base parameter: getSettings is the only step that can fail.
    let mut user = AsynUser::new(driver.ad.params.num_images);
    let err = driver.write_int32(&mut user, 3).unwrap_err();
    assert_eq!(err.status(), AsynStatus::Error);
}

/// Every write below FIRST_SD_PARAM goes to `ADDriver::writeInt32`
/// (mythen.cpp:1153), so PoolPreAllocBuffers copies the driver's pArrays[0].
#[test]
fn a_base_class_int32_write_reaches_the_ndarray_pool() {
    use epics_rs::ad_core::ndarray::{NDDataType, NDDimension};

    let (mut driver, _received) = driver("MYTHEN_WS_POOL");
    let image = driver
        .ad
        .pool
        .alloc(vec![NDDimension::new(4)], NDDataType::UInt32)
        .unwrap();
    driver
        .ad
        .pool
        .owner()
        .store_array(0, Some(image.share().array().clone()));
    let base = driver.ad.params.base;
    let mut user = AsynUser::new(base.pool_num_pre_alloc_buffers);
    driver.write_int32(&mut user, 2).unwrap();
    let mut user = AsynUser::new(base.pool_pre_alloc);
    driver.write_int32(&mut user, 1).unwrap();
    assert_eq!(driver.ad.pool.num_free_buffers(), 2);
}

/// ADDriver::writeFloat64 is asynPortDriver's (mythen.cpp:1210), whose
/// IO_DRIVER line precedes mythen's.
#[test]
fn a_base_class_float64_write_runs_asyn_port_drivers_write_float64() {
    use epics_rs::asyn::trace::{TraceFile, TraceMask};
    use epics_rs::asyn::user::UserTrace;
    let port = "MYTHEN_WS_F64";
    let (mut driver, _received) = driver(port);
    let manager = epics_rs::asyn::services::PortServices::global()
        .trace()
        .clone();
    let path = std::env::temp_dir().join(format!("mythen-trace-{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
    let delay = driver.ad.params.shutter_open_delay;
    let mut user = AsynUser::new(delay).with_addr(0);
    user.trace = Some(UserTrace {
        manager,
        port: port.into(),
    });
    driver.write_float64(&mut user, 2.0).unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let base = format!(
        "asynPortDriver:writeFloat64: function={delay}, name=SHUTTER_OPEN_DELAY, value=2.000000"
    );
    let own = format!("mythen:writeFloat64: function={delay}, value=2");
    let at = |line: &str| {
        text.find(line)
            .unwrap_or_else(|| panic!("{line}: {text:?}"))
    };
    assert!(at(&base) < at(&own));
}

/// mythen.cpp:1120, :1158: setIntegerParam(function, value) and
/// callParamCallbacks() are list 0 whatever the address; ADDriver's
/// getAddress then refuses addr 1 on this one-address multi-device port.
#[test]
fn a_write_at_another_address_sets_list_zero() {
    let (mut driver, _received) = driver("MYTHEN_WS_LIST0");
    let reason = driver.ad.params.base.array_counter;
    let mut user = AsynUser::new(reason).with_addr(1);
    let status = driver.write_int32(&mut user, 3);
    assert_eq!(status.unwrap_err().c_status_code(), 3);
    assert_eq!(driver.ad.port_base.get_int32_param(reason, 0).unwrap(), 3);
}

/// mythen.cpp:1182-1183: writeFloat64 returns getAddress's refusal before
/// it sets anything.
#[test]
fn a_float_write_outside_the_port_is_refused_before_the_set() {
    let (mut driver, _received) = driver("MYTHEN_WS_FADDR");
    let reason = driver.p.delay_time;
    let before = driver.ad.port_base.get_float64_param(reason, 0).ok();
    let mut user = AsynUser::new(reason).with_addr(1);
    let status = driver.write_float64(&mut user, 7.0);
    assert_eq!(status.unwrap_err().c_status_code(), 3);
    assert_eq!(
        driver.ad.port_base.get_float64_param(reason, 0).ok(),
        before
    );
}

/// mythen.cpp:1161-1164 prints the OR in asynStatus numbering:
/// asynParamWrongType (8) for an int written to ADAcquireTime.
#[test]
fn a_refused_write_prints_the_param_librarys_status() {
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::UserTrace;
    let port = "MYTHEN_WS_CODE";
    let (mut driver, _received) = driver(port);
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let reason = driver.ad.params.acquire_time;
    let mut user = AsynUser::new(reason);
    user.trace = Some(UserTrace {
        manager,
        port: port.into(),
    });
    assert!(driver.write_int32(&mut user, 1).is_err());
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert!(
        text.contains(&format!(
            "mythen:writeInt32: error, status=8 function={reason}, value=1"
        )),
        "{text}"
    );
}

/// mythen.cpp:1170: the write returns the OR of its steps. An int written to
/// the float SDThreshold fails only its own setIntegerParam, with
/// asynParamWrongType (8), not the asynError a last-status return gives.
#[test]
fn a_write_returns_the_or_of_its_steps() {
    let (mut driver, _received) = driver("MYTHEN_WS_OR");
    let mut user = AsynUser::new(driver.p.threshold);
    let e = driver.write_int32(&mut user, 1).unwrap_err();
    assert_eq!(e.c_status_code(), 8);
    assert_eq!(e.message(), "");
}

/// mythen.cpp:1076-1080: an octet written to a mythen param is only the
/// list-0 setStringParam, so an Int32 param returns its asynParamWrongType.
#[test]
fn an_octet_written_to_a_mythen_int_param_is_wrong_type() {
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    use epics_rs::asyn::user::UserTrace;
    let port = "MYTHEN_WS_OCTET";
    let (mut driver, _received) = driver(port);
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let reason = driver.p.setting;
    let mut user = AsynUser::new(reason);
    user.trace = Some(UserTrace {
        manager,
        port: port.into(),
    });
    let e = driver.write_octet(&mut user, b"x\0").unwrap_err();
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert_eq!(e.c_status_code(), 8);
    assert!(
        text.contains(&format!(
            "mythen:writeOctet: error, status=8 function={reason}, value=x"
        )),
        "{text}"
    );
}

/// mythen.cpp:361-367: a tau that is neither -1 nor positive prints the
/// refusal with outString_, the last command written.
#[test]
fn a_refused_tau_prints_the_last_command() {
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    let port = "MYTHEN_WS_TAULINE";
    let (mut driver, _received) = driver(port);
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let last = driver.det.out_string();
    assert!(last.starts_with("-get "), "{last}");
    let mut user = AsynUser::new(driver.p.tau);
    assert!(driver.write_float64(&mut user, -2.0).is_err());
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert!(
        text.contains(&format!("error check if tau -1 or >0 (outString = {last})")),
        "{text}"
    );
}

/// mythen.cpp:387-394, :413-416: a failed setKthresh or setEnergy prints its
/// refusal with outString_ and returns asynError.
#[test]
fn a_failed_threshold_or_energy_prints_the_refusal() {
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    let manager = PortServices::global().trace().clone();
    for (port, command) in [
        ("MYTHEN_WS_KTH", "-kthresh 7.000000"),
        ("MYTHEN_WS_EN", "-energy 7.000000"),
    ] {
        let (mut driver, _received) = driver_on(port, fake_detector_with(port, 1, Some(command)));
        let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let reason = if command.starts_with("-kthresh") {
            driver.p.threshold
        } else {
            driver.p.energy
        };
        let e = driver
            .write_float64(&mut AsynUser::new(reason), 7.0)
            .unwrap_err();
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        assert_eq!(e.c_status_code(), 3);
        assert!(
            text.contains(&format!("error check if value > 0 (outString = {command})")),
            "{text}"
        );
    }
}

/// mythen.cpp:377-382: C's module loop carries on past a failed command and
/// returns the last one's status, so outString_ ends on the last command.
#[test]
fn a_module_loop_runs_to_its_last_command() {
    use epics_rs::asyn::services::PortServices;
    use epics_rs::asyn::trace::TraceFile;
    let port = "MYTHEN_WS_LOOP";
    let (mut driver, _received) = driver_on(port, fake_detector_with(port, 2, Some("-module 1")));
    let manager = PortServices::global().trace().clone();
    let log = std::env::temp_dir().join(format!("{port}-{}", std::process::id()));
    let file = std::fs::File::create(&log).unwrap();
    manager.set_trace_file(
        Some(port),
        TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
    );
    let e = driver
        .write_float64(&mut AsynUser::new(driver.p.threshold), 7.0)
        .unwrap_err();
    let text = std::fs::read_to_string(&log).unwrap();
    let _ = std::fs::remove_file(&log);
    assert_eq!(e.c_status_code(), 3);
    assert!(
        text.contains("error check if value > 0 (outString = -kthresh 7.000000)"),
        "{text}"
    );
}

/// mythen.cpp:377, :406, :604, :666: with no module count C's module loops
/// run zero times and succeed, so nothing is sent and nothing refused.
#[test]
fn an_unknown_module_count_runs_the_module_loops_zero_times() {
    let (mut driver, received) = driver("MYTHEN_WS_NOMOD");
    let float = [driver.p.threshold, driver.p.energy];
    let int = [driver.p.setting, driver.p.reset];
    for reason in float {
        driver
            .write_float64(&mut AsynUser::new(reason), 7.0)
            .unwrap();
    }
    for reason in int {
        driver.write_int32(&mut AsynUser::new(reason), 1).unwrap();
    }
    let sent: Vec<String> = received.try_iter().collect();
    assert!(!sent.iter().any(|c| c.starts_with("-module")), "{sent:?}");
}
