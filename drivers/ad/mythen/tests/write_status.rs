//! A failed step of `writeInt32` / `writeFloat64` does not end the write: C
//! ORs each step into `status` and still runs `getSettings` and the callbacks
//! (mythen.cpp:1121-1159, 1195-1216).

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, mpsc};

use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port::PortDriver;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::user::AsynUser;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// A healthy detector whose module count is unknown (`-get nmodules` = -1), so
/// every per-module command is refused without touching the wire while the
/// `getSettings` queries still succeed.
fn fake_detector(name: &str) -> (PortHandle, mpsc::Receiver<String>) {
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
            let reply: Vec<u8> = match command.as_str() {
                "-get version" => b"M3.0.0\0".to_vec(),
                "-get nmodules" => (-1i32).to_le_bytes().to_vec(),
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
    let (handle, received) = fake_detector(name);
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
    let mut user = AsynUser::new(driver.p.setting);

    assert!(driver.write_int32(&mut user, 1).is_err());
    assert!(
        ran_get_settings(&received),
        "getSettings did not run after the refused loadSettings"
    );
}

#[test]
fn a_refused_float64_command_still_reads_the_settings_back() {
    let (mut driver, received) = driver("MYTHEN_WS_F64");
    let mut user = AsynUser::new(driver.p.threshold);

    assert!(driver.write_float64(&mut user, 7.0).is_err());
    assert!(
        ran_get_settings(&received),
        "getSettings did not run after the refused setKthresh"
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
