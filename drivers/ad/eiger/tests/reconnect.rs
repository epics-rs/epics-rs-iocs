//! No request is made while the detector is marked disconnected, and an
//! acquisition is what brings it back.
//!
//! Every request costs the full 20 s `DEFAULT_TIMEOUT` against a detector that
//! is powered off but whose address still routes, and the driver makes one per
//! parameter: the ~80 records that process at `iocInit`, plus a fetch for every
//! `EIG_*` parameter `drv_user_create` builds, cost minutes between them. The
//! gate is what makes them cost nothing, and the probe is what stops "nothing"
//! from being permanent — C never re-reads a detector that was absent when
//! `initParams` ran (eigerDetector.cpp:229-235).

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use epics_rs::ad_core::driver::ADStatus;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi, RestError, Sys};
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
    /// network, which is what the gate has to hold at zero.
    connections: Arc<AtomicUsize>,
    port: u16,
}

/// A detector that only answers while `answering` is set.
///
/// Hanging up rather than staying silent: the failure mode under test is the
/// gate, not the timeout, and a test that waited out a 20 s timeout to reach
/// the disconnected state would be measuring the thing the gate exists to
/// avoid.
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
fn no_request_is_made_while_the_detector_is_disconnected() {
    let det = switchable_detector();
    let rest = RestApi::new("127.0.0.1", det.port);

    // The first request is the one that finds out, so it does reach the wire.
    assert!(rest.get_value(Sys::DetConfig, "description").is_err());
    assert!(!rest.is_connected());
    assert_eq!(det.connections.load(Ordering::Acquire), 1);

    // The second must not: the gate answers it.
    let start = Instant::now();
    let refused = rest.get_value(Sys::DetConfig, "description");
    let elapsed = start.elapsed();
    assert!(
        matches!(refused, Err(RestError::Disconnected(_))),
        "expected the gate to refuse the request, got {refused:?}"
    );
    assert!(
        elapsed < FAST,
        "the request waited {elapsed:?}, so it went to the network instead of failing at the gate"
    );
    assert_eq!(
        det.connections.load(Ordering::Acquire),
        1,
        "a request reached the network while the detector was disconnected"
    );

    // The probe is the one request allowed past the gate, and it is what puts
    // the client back into a state where requests are made at all.
    det.answering.store(true, Ordering::Release);
    assert_eq!(rest.probe().expect("the probe"), ApiVersion::V1_6_0);
    assert!(rest.is_connected());
    assert!(rest.get_value(Sys::DetConfig, "description").is_ok());
}

#[test]
fn an_acquisition_reconnects_a_detector_that_was_offline_at_boot() {
    let det = switchable_detector();
    let rest = RestApi::new("127.0.0.1", det.port);
    let (signals, _ctl_rx, _init_rx, _restart_rx) = tasks::signals();
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

    // The constructor's whole request sequence costs one connection: the state
    // fetch finds out, and the gate refuses everything after it.
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
        "Eiger FAILED TO CONNECT"
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
        "",
        "the FAILED TO CONNECT message must be gone once the detector answers"
    );
}
