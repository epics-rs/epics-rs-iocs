//! drvAHxxx, drvTetrAMM and drvPCR4 stop acquisition around a command
//! (`sendCommand`) and a status read (`readStatus`). C's `goto error` jumps
//! past the `setAcquire(1)` restart (drvAHxxx.cpp:136, :797-801), so one
//! refused command or unparsable status reply leaves the meter stopped. Here
//! the meter is restarted either way.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use quadem::octet::{OctetIo, create_ip_port};
use quadem::pcr4_proto::{CMD_ACQUIRE_START, CMD_ACQUIRE_STOP};
use quadem::{create_ahxxx, create_pcr4, create_tetramm};

type Log = Arc<Mutex<Vec<String>>>;

/// A meter that logs every `\r`-terminated command and answers `NAK` to
/// `refuse`, `ACK` to the rest. No status query parses an `ACK`.
fn meter(refuse: Option<&'static str>) -> (String, Log) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let log = Log::default();
    let seen = log.clone();
    thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut out = stream.try_clone().unwrap();
        let mut reader = BufReader::new(stream);
        let mut line = Vec::new();
        while reader.read_until(b'\r', &mut line).unwrap_or(0) > 0 {
            let command = String::from_utf8_lossy(&line).trim().to_string();
            line.clear();
            let reply: &[u8] = if Some(command.as_str()) == refuse {
                b"NAK\r\n"
            } else {
                b"ACK\r\n"
            };
            seen.lock().unwrap().push(command);
            let _ = out.write_all(reply);
        }
    });
    (addr, log)
}

fn ip_port(name: &str, addr: &str) -> (OctetIo, impl Sized) {
    let (io, ip) = create_ip_port(
        name,
        addr,
        Duration::from_millis(200),
        false,
        true,
        &InterruptGate::opened(),
    )
    .unwrap();
    io.set_output_eos(b"\r").unwrap();
    (io, ip)
}

/// The last of `start`/`stop` the meter has seen.
fn last_of(log: &Log, start: &str, stop: &str) -> Option<String> {
    let log = log.lock().unwrap();
    log.iter()
        .rev()
        .find(|c| *c == start || *c == stop)
        .cloned()
}

fn wait_for(log: &Log, start: &str, stop: &str) -> bool {
    let deadline = Instant::now() + Duration::from_secs(3);
    while last_of(log, start, stop).as_deref() != Some(start) {
        if Instant::now() > deadline {
            return false;
        }
        thread::sleep(Duration::from_millis(10));
    }
    true
}

/// Start acquiring, write `reason`, and require that the write stopped the
/// meter and left it started.
fn check(handle: &PortHandle, acquire: usize, reason: usize, log: &Log, start: &str, stop: &str) {
    handle.write_int32_blocking(acquire, 0, 1).expect("acquire");
    assert!(wait_for(log, start, stop), "never started: {log:?}");
    let before = log.lock().unwrap().len();

    // The status read fails on every meter here; the write's status is not
    // what this checks.
    let _ = handle.write_int32_blocking(reason, 0, 1);

    assert!(
        log.lock().unwrap()[before..].iter().any(|c| c == stop),
        "the write never stopped the meter: {log:?}"
    );
    assert!(wait_for(log, start, stop), "left stopped: {log:?}");
}

#[test]
fn ahxxx_restarts_after_a_failed_status_read() {
    let (addr, log) = meter(None);
    let _ip = ip_port("AH_RS_IP", &addr);
    let ah = create_ahxxx(&claim_port("AH_RS").unwrap(), "AH_RS_IP", 16, 0, "AH401D").unwrap();
    check(
        ah.port_handle(),
        ah.nd_params.acquire,
        ah.params.range,
        &log,
        "ACQ ON",
        "ACQ OFF",
    );
}

#[test]
fn ahxxx_restarts_after_a_refused_command() {
    let (addr, log) = meter(Some("RNG 1"));
    let _ip = ip_port("AH_NAK_IP", &addr);
    let ah = create_ahxxx(&claim_port("AH_NAK").unwrap(), "AH_NAK_IP", 16, 0, "AH401D").unwrap();
    check(
        ah.port_handle(),
        ah.nd_params.acquire,
        ah.params.range,
        &log,
        "ACQ ON",
        "ACQ OFF",
    );
}

#[test]
fn tetramm_restarts_after_a_failed_status_read() {
    let (addr, log) = meter(None);
    let _ip = ip_port("TETRA_RS_IP", &addr);
    let t = create_tetramm(&claim_port("TETRA_RS").unwrap(), "TETRA_RS_IP", 16, 0).unwrap();
    check(
        t.port_handle(),
        t.nd_params.acquire,
        t.params.bias_interlock,
        &log,
        "ACQ:ON",
        "ACQ:OFF",
    );
}

#[test]
fn tetramm_restarts_after_a_refused_command() {
    let (addr, log) = meter(Some("INTERLOCK:ON"));
    let _ip = ip_port("TETRA_NAK_IP", &addr);
    let t = create_tetramm(&claim_port("TETRA_NAK").unwrap(), "TETRA_NAK_IP", 16, 0).unwrap();
    check(
        t.port_handle(),
        t.nd_params.acquire,
        t.params.bias_interlock,
        &log,
        "ACQ:ON",
        "ACQ:OFF",
    );
}

#[test]
fn pcr4_restarts_after_a_failed_status_read() {
    let (addr, log) = meter(None);
    let _ip = ip_port("PCR4_RS_IP", &addr);
    let p = create_pcr4(&claim_port("PCR4_RS").unwrap(), "PCR4_RS_IP", 16, 0).unwrap();
    check(
        p.port_handle(),
        p.nd_params.acquire,
        p.params.bias_state,
        &log,
        CMD_ACQUIRE_START,
        CMD_ACQUIRE_STOP,
    );
}

#[test]
fn pcr4_restarts_after_a_refused_command() {
    let (addr, log) = meter(Some("BIAS:ON"));
    let _ip = ip_port("PCR4_NAK_IP", &addr);
    let p = create_pcr4(&claim_port("PCR4_NAK").unwrap(), "PCR4_NAK_IP", 16, 0).unwrap();
    check(
        p.port_handle(),
        p.nd_params.acquire,
        p.params.bias_state,
        &log,
        CMD_ACQUIRE_START,
        CMD_ACQUIRE_STOP,
    );
}
