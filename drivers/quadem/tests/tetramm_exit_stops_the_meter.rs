//! drvTetrAMM::exitHandler is `setAcquire(0)` (drvTetrAMM.cpp:869-874), run
//! by the `epicsAtExit` hook drvQuadEM arms (drvQuadEM.cpp:154): a meter still
//! acquiring when the IOC exits is sent `ACQ:OFF`. Its own binary, as
//! `call_at_exits` runs every exit callback in the process.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::registry::claim_port;
use epics_rs::base::runtime::exit::call_at_exits;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use quadem::create_tetramm;
use quadem::octet::create_ip_port;

#[test]
fn an_acquiring_tetramm_is_stopped_at_exit() {
    // The meter: log every command and answer each with `ACK`; it sends no
    // data, so the read thread times out and rechecks acquiring_.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let log = Arc::new(Mutex::new(Vec::<String>::new()));
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

    let (io, _ip) = create_ip_port(
        "TETRA_EXIT_IP",
        &addr,
        Duration::from_millis(200),
        false,
        true,
        &InterruptGate::opened(),
    )
    .unwrap();
    // st.cmd's asynOctetSetOutputEos; the driver sets only the input EOS.
    io.set_output_eos(b"\r").unwrap();
    let port = create_tetramm(&claim_port("TETRA_EXIT").unwrap(), "TETRA_EXIT_IP", 16, 0).unwrap();
    port.port_handle()
        .write_int32_blocking(port.nd_params.acquire, 0, 1)
        .expect("acquire");
    let deadline = Instant::now() + Duration::from_secs(10);
    while !log.lock().unwrap().iter().any(|c| c == "ACQ:ON") {
        assert!(Instant::now() < deadline, "never started");
        thread::sleep(Duration::from_millis(10));
    }

    call_at_exits();

    let log = log.lock().unwrap();
    let started = log.iter().rposition(|c| c == "ACQ:ON").unwrap();
    assert!(
        log[started..].iter().any(|c| c == "ACQ:OFF"),
        "no ACQ:OFF at exit: {log:?}"
    );
}
