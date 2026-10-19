//! The status of a write to a detector the network cannot reach comes from the
//! asyn port, as in C: the first command after the port reconnects fails in
//! `sendto` (asynError, drvAsynIPPort.c:740-748, which also disconnects the
//! port), and every command after it is refused by `queueRequest` with
//! asynDisconnected (asynManager.c:1549-1553) until the connect timer
//! reconnects the port 10 ms later (:2183-2185).

use std::sync::Arc;
use std::time::Duration;

use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::user::AsynUser;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// A UDP port whose every `sendto` fails at once: the broadcast address
/// without SO_BROADCAST is EACCES on every host, as an unroutable address is
/// ENETUNREACH in the cdiff network namespace.
fn unreachable_driver(name: &str) -> MythenDriver {
    let ip_name = format!("{name}_IP");
    let ip = DrvAsynIPPort::new(&ip_name, "255.255.255.255:1030 UDP").expect("ip port");
    let (runtime, _) =
        create_port_runtime(&claim_port(&ip_name).unwrap(), ip, RuntimeConfig::default())
            .expect("ip port runtime");
    let det = Arc::new(Detector::new(
        Transport::new(runtime.port_handle().clone()),
        name,
    ));
    std::mem::forget(runtime);
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);
    MythenDriver::new(name, det, 0, start_tx).expect("driver")
}

#[test]
fn the_first_write_after_a_reconnect_is_asyn_error_and_the_next_is_disconnected() {
    let mut driver = unreachable_driver("MYTHEN_FW");
    // Let the connect timer bring the port back after the constructor's
    // failed queries, as the half second before iocInit does in C.
    std::thread::sleep(Duration::from_millis(200));

    // setExposureTime's sendto fails (3); getSettings is asynError (3).
    let mut user = AsynUser::new(driver.ad.params.acquire_time);
    let first = driver.write_float64(&mut user, 1.0).unwrap_err();
    assert_eq!(first.c_status_code(), 3);

    // The port is down now: setDelayAfterTrigger is refused (4) | 3.
    let mut user = AsynUser::new(driver.p.delay_time);
    let next = driver.write_float64(&mut user, 0.1).unwrap_err();
    assert_eq!(next.c_status_code(), 7);
}
