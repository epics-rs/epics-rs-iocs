//! drvPCR4::exitHandler is `setAcquire(0)` (drvPCR4.cpp:693-698), run by
//! drvQuadEM's `epicsAtExit` hook: at IOC exit the meter is sent
//! `TRIGGER:STOP` and `ACQC:STOP`. Its own binary, as `call_at_exits` runs
//! every exit callback in the process.

use std::io::{BufRead, BufReader, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::registry::claim_port;
use epics_rs::base::runtime::exit::call_at_exits;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use quadem::create_pcr4;
use quadem::octet::create_ip_port;
use quadem::pcr4_proto::{CMD_ACQUIRE_STOP, CMD_TRIGGER_STOP};

/// A meter that answers every `\r`-terminated command with `ACK`.
fn ack_meter() -> (String, Arc<Mutex<Vec<String>>>) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let log = Arc::new(Mutex::new(Vec::new()));
    let seen = log.clone();
    thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut out = stream.try_clone().unwrap();
        let mut reader = BufReader::new(stream);
        let mut line = Vec::new();
        while reader.read_until(b'\r', &mut line).unwrap_or(0) > 0 {
            seen.lock()
                .unwrap()
                .push(String::from_utf8_lossy(&line).trim().to_string());
            line.clear();
            let _ = out.write_all(b"ACK\r\n");
        }
    });
    (addr, log)
}

#[test]
fn the_meter_is_stopped_at_exit() {
    let (addr, log) = ack_meter();
    let (io, _ip) = create_ip_port(
        "PCR4_EXIT_IP",
        &addr,
        Duration::from_millis(200),
        false,
        true,
        &InterruptGate::opened(),
    )
    .unwrap();
    io.set_output_eos(b"\r").unwrap();
    let _pcr4 = create_pcr4(&claim_port("PCR4_EXIT").unwrap(), "PCR4_EXIT_IP", 4, 0).unwrap();
    // What the constructor sent.
    let before = log.lock().unwrap().len();

    call_at_exits();

    let want = [CMD_TRIGGER_STOP, CMD_ACQUIRE_STOP].map(String::from);
    let deadline = Instant::now() + Duration::from_secs(3);
    while log.lock().unwrap()[before..] != want && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(log.lock().unwrap()[before..], want);
}
