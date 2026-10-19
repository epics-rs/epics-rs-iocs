//! C's constructor takes ADStatus from `getStatus` alone (mythen.cpp:1353); a
//! later boot query failing does not touch it.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::Arc;

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// A detector that answers every query, with a `-get flatfieldcorrection`
/// outside the 0/1 that C's getSettings accepts (mythen.cpp:698).
fn detector_with_bad_settings() -> PortHandle {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("connection");
        let mut buf = [0u8; 256];
        while let Ok(n) = stream.read(&mut buf) {
            if n == 0 {
                break;
            }
            let reply = match String::from_utf8_lossy(&buf[..n]).trim() {
                "-get version" => b"M3.0.0\0".to_vec(),
                "-get nmodules" => 1i32.to_le_bytes().to_vec(),
                // Not running, no data: idle (bit 16 set, bit 0 clear).
                "-get status" => (1i32 << 16).to_le_bytes().to_vec(),
                "-get flatfieldcorrection" => 7i32.to_le_bytes().to_vec(),
                _ => 0i32.to_le_bytes().to_vec(),
            };
            if stream.write_all(&reply).is_err() {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_BADSET_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_BADSET_IP").unwrap(),
        driver,
        RuntimeConfig::default(),
    )
    .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    handle
}

#[test]
fn a_failed_get_settings_leaves_the_status_get_status_read() {
    let det = Arc::new(Detector::new(
        Transport::new(detector_with_bad_settings()),
        "MYTHEN_BADSET",
    ));
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new("MYTHEN_BADSET", det, 0, start_tx)
        .expect("the port must be created when getSettings fails");

    assert_eq!(
        driver
            .ad
            .port_base
            .get_int32_param(driver.ad.params.status, 0)
            .unwrap(),
        ADStatus::Idle as i32
    );
}
