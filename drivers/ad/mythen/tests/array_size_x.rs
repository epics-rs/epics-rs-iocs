//! C's constructor leaves NDArraySizeX alone (mythen.cpp:1332-1345 set only
//! the sensor and region sizes and NDArraySize), so it reads 0 after boot
//! whatever `-get nmodules` answers.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::Arc;

use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// A one-module detector that answers every query.
fn one_module_detector() -> PortHandle {
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
                "-get status" => (1i32 << 16).to_le_bytes().to_vec(),
                "-get nbits" => 24i32.to_le_bytes().to_vec(),
                "-get tau" => (-1.0f32).to_le_bytes().to_vec(),
                _ => 0i32.to_le_bytes().to_vec(),
            };
            if stream.write_all(&reply).is_err() {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_ASX_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_ASX_IP").unwrap(),
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
fn the_constructor_leaves_array_size_x_at_zero() {
    let det = Arc::new(Detector::new(
        Transport::new(one_module_detector()),
        "MYTHEN_ASX",
    ));
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new("MYTHEN_ASX", det, 0, start_tx).expect("driver");

    let base = &driver.ad.port_base;
    assert_eq!(base.get_int32_param(driver.p.nmodules, 0).unwrap(), 1);
    assert_eq!(
        base.get_int32_param(driver.ad.params.base.array_size_x, 0)
            .unwrap(),
        0
    );
}
