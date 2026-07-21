//! A module count the detector never reported must not read back as zero.
//!
//! `write_read` writes before it reads, so a readout whose expected length came
//! out zero sent `-readoutraw` and then read none of the reply. The frame stayed
//! in the socket and was picked up as the answer to the next command — a
//! protocol desync that lasts as long as the process. It was reachable whenever
//! `-get nmodules` failed at boot and the detector came back later, since
//! nothing re-reads it.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::mpsc;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::transport::Transport;

/// The firmware string the fake detector answers `-get version` with.
const VERSION: &[u8; 7] = b"M3.0.0\0";

/// A detector that answers `-get version`, reports a nonsense module count, and
/// reports every command it was sent.
fn fake_detector() -> (PortHandle, mpsc::Receiver<String>) {
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
                "-get version" => VERSION.to_vec(),
                // C's own "unexpected reply" case (mythen.cpp:1367).
                "-get nmodules" => (-1i32).to_le_bytes().to_vec(),
                _ => 0i32.to_le_bytes().to_vec(),
            };
            if sent.send(command).is_err() || stream.write_all(&reply).is_err() {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_UNKNOWN_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    (handle, received)
}

#[test]
fn a_readout_is_refused_while_the_module_count_is_unknown() {
    let (handle, received) = fake_detector();
    let det = Arc::new(Detector::new(Transport::new(handle)));

    // Never asked: unknown, and unknown is not zero.
    assert_eq!(det.nmodules(), None);
    assert_eq!(det.readout_len(), None);

    // Asked and refused: still unknown. A count of -1 must not become 0 either.
    assert_eq!(det.read_nmodules().expect("the reply arrives"), -1);
    assert_eq!(det.nmodules(), None);
    assert_eq!(det.readout_len(), None);

    // With no length there is no readout to issue, so the socket is untouched
    // and the next command gets its own reply rather than a stranded frame.
    assert_eq!(det.get_firmware().expect("firmware"), "M3.0.0");

    let commands: Vec<String> = received.try_iter().collect();
    assert_eq!(commands, vec!["-get nmodules", "-get version"]);
    assert!(
        !commands.iter().any(|c| c.starts_with("-readout")),
        "a readout was sent with no length to read it back at: {commands:?}"
    );
}
