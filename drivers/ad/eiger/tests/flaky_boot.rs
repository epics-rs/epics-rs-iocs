//! The detector answers once and then stops answering.
//!
//! The upfront `state` fetch only covers a detector that is already gone. C's
//! constructor issues its `put`s afterwards and inspects none of their return
//! codes (eigerDetector.cpp:1655-1673), so a detector that drops *between* the
//! state fetch and the writes still has to leave a usable port behind.

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use epics_rs::ad_core::driver::ADStatus;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;

/// Every GET is answered as a two-valued enum — which is what makes the
/// driver's constructor-time writes reach the wire — and every PUT fails.
fn half_dead_detector(puts: Arc<AtomicUsize>) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let port = listener.local_addr().unwrap().port();

    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { break };
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

            let response = if request.starts_with("PUT") {
                puts.fetch_add(1, Ordering::Release);
                "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\
                 Connection: close\r\n\r\n"
                    .to_string()
            } else {
                let body = r#"{"value":"disabled","value_type":"string","access_mode":"rw",
                              "allowed_values":["disabled","enabled"]}"#;
                format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                     Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                    body.len()
                )
            };
            let _ = stream.write_all(response.as_bytes());
        }
    });

    port
}

#[test]
fn the_driver_is_created_when_the_detector_stops_answering_after_the_state_fetch() {
    let puts = Arc::new(AtomicUsize::new(0));
    let rest = RestApi::new("127.0.0.1", half_dead_detector(puts.clone()));
    let (signals, _ctl_rx, _init_rx, _restart_rx) = tasks::signals();
    let cfg = EigerConfig {
        port_name: "EIGER_FLAKY".to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };

    let driver = EigerDriver::new(cfg, rest, signals)
        .expect("the port must survive a detector that fails every write");

    // The writes have to have actually reached the detector: a local encoding
    // failure would prove nothing about the `?`-propagation this covers.
    assert!(
        puts.load(Ordering::Acquire) >= 4,
        "expected the constructor's writes on the wire, saw {}",
        puts.load(Ordering::Acquire)
    );
    assert_eq!(
        driver
            .ad
            .port_base
            .get_int32_param(driver.ad.params.status, 0)
            .unwrap(),
        ADStatus::Disconnected as i32
    );
}
