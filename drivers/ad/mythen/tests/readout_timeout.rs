//! C re-reads the status after a readout that brought no frame, a failed one
//! too, before it reports the failure (mythen.cpp:908-917).

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, mpsc};
use std::time::Duration;

use epics_rs::ad_core::plugin::channel::NDArrayOutput;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::task::{self, Shared};
use mythen::transport::{M1K_TIMEOUT, Transport};

/// An idle one-module detector that never answers a readout, and reports every
/// command it was sent.
fn silent_readout_detector() -> (PortHandle, mpsc::Receiver<String>) {
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
            let reply = match command.as_str() {
                "-readoutraw" => None,
                "-get version" => Some(b"M3.0.0\0".to_vec()),
                "-get nmodules" => Some(1i32.to_le_bytes().to_vec()),
                // Not running, no data: idle (bit 16 set, bit 0 clear).
                "-get status" => Some((1i32 << 16).to_le_bytes().to_vec()),
                "-get nbits" => Some(24i32.to_le_bytes().to_vec()),
                "-get tau" => Some((-1.0f32).to_le_bytes().to_vec()),
                _ => Some(0i32.to_le_bytes().to_vec()),
            };
            if sent.send(command).is_err() {
                break;
            }
            if let Some(reply) = reply
                && stream.write_all(&reply).is_err()
            {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_RDTO_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_RDTO_IP").unwrap(),
        driver,
        RuntimeConfig::default(),
    )
    .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    (handle, received)
}

#[test]
fn a_timed_out_readout_re_reads_the_status() {
    let (ip, received) = silent_readout_detector();
    let det = Arc::new(Detector::new(Transport::new(ip), "MYTHEN_RDTO"));
    let (start_tx, start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new("MYTHEN_RDTO", det.clone(), 0, start_tx).unwrap();
    let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_RDTO").unwrap(),
        driver,
        RuntimeConfig::default(),
    )
    .unwrap();
    let handle = runtime.port_handle().clone();
    let _task = task::start(
        Arc::new(Shared {
            handle: handle.clone(),
            p,
            ad,
            det,
            output: Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())),
            pool,
        }),
        start_rx,
    );
    received.try_iter().for_each(drop);

    handle.write_int32_blocking(ad.acquire, 0, 1).unwrap();
    let next = |wait| received.recv_timeout(wait).ok();
    let mut sent = Vec::new();
    while sent.last().map(String::as_str) != Some("-readoutraw") {
        sent.push(next(Duration::from_secs(5)).expect("no readout was sent"));
    }
    sent.extend(next(M1K_TIMEOUT + Duration::from_secs(5)));
    assert_eq!(
        sent,
        ["-start", "-get status", "-readoutraw", "-get status"]
    );
}
