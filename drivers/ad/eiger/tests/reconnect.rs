//! Every request goes to the detector while it is away, as C's do, and an
//! acquisition re-reads a detector that failed the constructor's state fetch
//! — C never re-reads one (eigerDetector.cpp:229-235).

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use epics_rs::ad_core::driver::ADStatus;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi, Sys};
use eiger::tasks;

/// A fast-fail has to be fast enough to be visibly not a timeout: the shortest
/// request timeout in the client is the 10 s probe.
const FAST: Duration = Duration::from_secs(1);

/// One JSON body that answers every GET the driver makes: a two-valued enum,
/// whose `value` doubles as the SIMPLON version string the probe reads.
const ENUM_BODY: &str = r#"{"value":"1.6.0","value_type":"string","access_mode":"rw",
                            "allowed_values":["1.6.0","enabled"]}"#;

/// What the fake detector was asked to do.
struct Detector {
    /// Set to make it answer; while clear it accepts and hangs up.
    answering: Arc<AtomicBool>,
    /// Connections accepted — the count of times the driver reached the
    /// network.
    connections: Arc<AtomicUsize>,
    port: u16,
}

/// A detector that only answers while `answering` is set.
///
/// Hanging up rather than staying silent, so a failed request costs no
/// timeout.
fn switchable_detector() -> Detector {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let port = listener.local_addr().unwrap().port();
    let answering = Arc::new(AtomicBool::new(false));
    let connections = Arc::new(AtomicUsize::new(0));

    let (a, c) = (answering.clone(), connections.clone());
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { break };
            c.fetch_add(1, Ordering::Release);
            if !a.load(Ordering::Acquire) {
                continue;
            }
            let mut reader = BufReader::new(stream.try_clone().expect("clone"));

            let mut request = String::new();
            if reader.read_line(&mut request).unwrap_or(0) == 0 {
                continue;
            }
            // Headers, then the body — the request has to be drained before the
            // socket closes or the client sees a reset instead of the reply.
            let mut length = 0usize;
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap_or(0) <= 2 {
                    break;
                }
                if let Some(v) = header.to_ascii_lowercase().strip_prefix("content-length:") {
                    length = v.trim().parse().unwrap_or(0);
                }
            }
            if length > 0 {
                let mut body = vec![0u8; length];
                let _ = reader.read_exact(&mut body);
            }

            // A PUT answers with an empty body: nothing else was invalidated.
            let response = if request.starts_with("PUT") {
                "HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".to_string()
            } else {
                format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n{ENUM_BODY}",
                    ENUM_BODY.len()
                )
            };
            let _ = stream.write_all(response.as_bytes());
        }
    });

    Detector {
        answering,
        connections,
        port,
    }
}

#[test]
fn every_request_reaches_the_detector_while_it_is_disconnected() {
    let det = switchable_detector();
    let rest = RestApi::new("127.0.0.1", det.port);

    assert!(rest.get_value(Sys::DetConfig, "description").is_err());
    assert!(!rest.is_connected());
    assert_eq!(det.connections.load(Ordering::Acquire), 1);

    // C's doRequest reconnects for every request (restApi.cpp:609-620), so
    // the next one goes to the network too, and fails fast on the hang-up.
    let start = Instant::now();
    assert!(rest.get_value(Sys::DetConfig, "description").is_err());
    assert!(start.elapsed() < FAST);
    assert_eq!(det.connections.load(Ordering::Acquire), 2);

    // The first request the detector answers marks it connected again.
    det.answering.store(true, Ordering::Release);
    assert!(rest.get_value(Sys::DetConfig, "description").is_ok());
    assert!(rest.is_connected());
    assert_eq!(rest.probe().expect("the probe"), ApiVersion::V1_6_0);
}

#[test]
fn an_acquisition_reconnects_a_detector_that_was_offline_at_boot() {
    let det = switchable_detector();
    let rest = RestApi::new("127.0.0.1", det.port);
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: "EIGER_RECONNECT".to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };

    let mut driver =
        EigerDriver::new(cfg, rest, signals).expect("the port must survive an offline detector");

    // The constructor's whole request sequence costs one connection: C
    // abandons the constructor when the state fetch fails.
    assert_eq!(
        det.connections.load(Ordering::Acquire),
        1,
        "the constructor kept talking to a detector that had already failed"
    );
    let base = &driver.ad.port_base;
    assert_eq!(
        base.get_int32_param(driver.ad.params.status, 0).unwrap(),
        ADStatus::Disconnected as i32
    );
    assert_eq!(
        base.get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        b"Eiger FAILED TO CONNECT"
    );

    // The detector comes back, and the acquisition path's probe re-reads it.
    det.answering.store(true, Ordering::Release);
    driver
        .reconnect()
        .expect("the probe must succeed once the detector answers");

    let base = &driver.ad.port_base;
    assert_eq!(
        base.get_int32_param(driver.ad.params.status, 0).unwrap(),
        ADStatus::Idle as i32
    );
    assert_eq!(
        base.get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        b"",
        "the FAILED TO CONNECT message must be gone once the detector answers"
    );
}
