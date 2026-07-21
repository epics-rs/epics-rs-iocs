//! No command reaches the socket while the detector is marked disconnected, and
//! an acquisition is what brings it back.
//!
//! Every command costs the full 5 s `M1K_TIMEOUT` against a detector that is
//! powered off but whose address still routes, and they are serialized through
//! the port's single request queue: the records that process at `iocInit` used
//! to cost minutes between them. The gate is what makes them cost nothing, and
//! the probe is what stops "nothing" from being permanent — C never re-reads the
//! module count after its constructor (mythen.cpp:1363), so a detector that was
//! off at boot stays unusable there until the IOC restarts.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::transport::Transport;

/// The module count the detector reports once it starts answering.
const NMODULES: i32 = 2;

/// A fast-fail has to be fast enough to be visibly not a timeout: `M1K_TIMEOUT`
/// is 5 s, so a second is two orders of magnitude of headroom either way.
const FAST: Duration = Duration::from_secs(1);

/// A detector that reads every command but only answers while `answering` is
/// set, and reports every command it was sent.
///
/// Silence rather than a closed port: a detector that is powered off behind a
/// switch that still routes is exactly a socket that accepts and never replies,
/// and it is the case whose per-command cost is the full timeout.
fn switchable_detector(answering: Arc<AtomicBool>) -> (PortHandle, mpsc::Receiver<String>) {
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
            if sent.send(command.clone()).is_err() {
                break;
            }
            if !answering.load(Ordering::Acquire) {
                continue;
            }
            if stream.write_all(&reply_to(&command)).is_err() {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_RECONNECT_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    (handle, received)
}

/// What a healthy firmware-3 detector answers each of the constructor's queries
/// with. Everything else is a status word of 0, which is "command accepted".
fn reply_to(command: &str) -> Vec<u8> {
    match command {
        "-get version" => b"M3.0.0\0".to_vec(),
        "-get nmodules" => NMODULES.to_le_bytes().to_vec(),
        // Not running, no data: idle (bit 16 set, bit 0 clear).
        "-get status" => (1i32 << 16).to_le_bytes().to_vec(),
        "-get nbits" => 24i32.to_le_bytes().to_vec(),
        // 100 ns units: 1 s.
        "-get time" => 10_000_000i32.to_le_bytes().to_vec(),
        "-get frames" => 1i32.to_le_bytes().to_vec(),
        // -1 is "no rate correction", the only legal non-positive tau.
        "-get tau" => (-1.0f32).to_le_bytes().to_vec(),
        "-get kthresh" => 8.0f32.to_le_bytes().to_vec(),
        "-get energy" => 8.05f32.to_le_bytes().to_vec(),
        _ => 0i32.to_le_bytes().to_vec(),
    }
}

#[test]
fn no_command_is_sent_while_the_detector_is_disconnected_and_an_acquisition_brings_it_back() {
    let answering = Arc::new(AtomicBool::new(false));
    let (handle, received) = switchable_detector(answering.clone());
    let det = Arc::new(Detector::new(Transport::new(handle)));
    let (start_tx, _start_rx) = rt::command_channel::<()>(1);

    // --- Boot against a silent detector ------------------------------------
    let mut driver = MythenDriver::new("MYTHEN_RECONNECT", det.clone(), 0, start_tx)
        .expect("the port must be created even though the detector is silent");
    assert!(!det.is_connected());
    assert_eq!(
        driver
            .ad
            .port_base
            .get_int32_param(driver.ad.params.status, 0)
            .unwrap(),
        ADStatus::Disconnected as i32
    );

    // The constructor runs four queries. Only the first may reach the wire:
    // the failure it reports is what the other three are refused on, and that
    // is the whole difference between a boot that costs one timeout and one
    // that costs four.
    let commands: Vec<String> = received.try_iter().collect();
    assert_eq!(
        commands,
        vec!["-get version"],
        "a command was sent after the detector was marked disconnected"
    );

    // --- A request while disconnected never reaches the socket -------------
    let start = Instant::now();
    let refused = det.set_frames(7, 0);
    let elapsed = start.elapsed();
    assert!(refused.is_err(), "a disconnected detector accepted a write");
    assert!(
        elapsed < FAST,
        "the write waited {elapsed:?}, so it went to the socket instead of failing at the gate"
    );
    assert_eq!(
        received.try_iter().count(),
        0,
        "a command reached the socket while the detector was disconnected"
    );

    // The module count is still unknown, so there is still no readout length —
    // which is what the probe below has to fix, not the connection alone.
    assert_eq!(det.nmodules(), None);

    // --- The detector comes back ------------------------------------------
    answering.store(true, Ordering::Release);
    driver
        .reconnect()
        .expect("the probe must succeed once the detector answers");

    assert!(det.is_connected());
    // Re-read, not merely reconnected: the readout length is derived from this.
    assert_eq!(det.nmodules(), Some(NMODULES as usize));
    let commands: Vec<String> = received.try_iter().collect();
    assert!(
        commands.contains(&"-get nmodules".to_string()),
        "the reconnect did not re-read the module count: {commands:?}"
    );

    let base = &driver.ad.port_base;
    assert_eq!(
        base.get_int32_param(driver.p.nmodules, 0).unwrap(),
        NMODULES
    );
    assert_eq!(
        base.get_string_param(driver.p.firmware_version, 0).unwrap(),
        "M3.0.0"
    );
    assert_eq!(
        base.get_int32_param(driver.ad.params.status, 0).unwrap(),
        ADStatus::Idle as i32
    );
    assert_eq!(
        base.get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        "",
        "the FAILED TO CONNECT message must be gone once the detector answers"
    );
}
