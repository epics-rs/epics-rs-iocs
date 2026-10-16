//! A detector reporting `state` "na" is initialized by the constructor, then
//! armed and disarmed once (eigerDetector.cpp:239-263).

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;

/// Answers `state` with "na", fails `initialize` when `init_fails`, and
/// records every request line.
fn uninitialized_detector(init_fails: bool, log: Arc<Mutex<Vec<String>>>) -> u16 {
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
            let path = request.split_whitespace().nth(1).unwrap_or("").to_string();
            log.lock().unwrap().push(format!(
                "{} {path}",
                request.split_whitespace().next().unwrap_or("")
            ));

            let (code, body) = if path.ends_with("/initialize") && init_fails {
                ("500 Internal Server Error", String::new())
            } else if path.ends_with("/arm") {
                ("200 OK", r#"{"sequence id": 7}"#.to_string())
            } else if request.starts_with("PUT") {
                ("200 OK", "[]".to_string())
            } else if path.ends_with("/state") {
                (
                    "200 OK",
                    r#"{"value":"na","value_type":"string","access_mode":"r"}"#.to_string(),
                )
            } else {
                (
                    "200 OK",
                    r#"{"value":"disabled","value_type":"string","access_mode":"rw",
                        "allowed_values":["disabled","enabled"]}"#
                        .to_string(),
                )
            };
            let response = format!(
                "HTTP/1.1 {code}\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            let _ = stream.write_all(response.as_bytes());
        }
    });
    port
}

fn boot(name: &str, init_fails: bool) -> (EigerDriver, Vec<String>) {
    let log = Arc::new(Mutex::new(Vec::new()));
    let rest = RestApi::new("127.0.0.1", uninitialized_detector(init_fails, log.clone()));
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: name.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_size_x: 0,
        max_size_y: 0,
        max_memory: 0,
    };
    let driver = EigerDriver::new(cfg, rest, signals).expect("driver");
    let log = log.lock().unwrap().clone();
    (driver, log)
}

fn commands(log: &[String]) -> Vec<&str> {
    log.iter()
        .filter(|l| l.starts_with("PUT") && l.contains("/command/"))
        .map(|l| l.rsplit('/').next().unwrap())
        .collect()
}

#[test]
fn an_uninitialized_detector_is_initialized_armed_and_disarmed() {
    let (driver, log) = boot("EIGER_NA_OK", false);
    assert_eq!(commands(&log), ["initialize", "arm", "disarm"]);
    // The constructor goes on to read the parameters.
    assert!(log.len() > 4, "{log:?}");
    assert_eq!(
        driver
            .ad
            .port_base
            .get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        b""
    );
}

#[test]
fn a_failed_initialize_abandons_the_constructor() {
    let (driver, log) = boot("EIGER_NA_FAIL", true);
    assert_eq!(commands(&log), ["initialize"]);
    // Nothing after the initialize: C returns from the constructor.
    assert!(log.last().unwrap().ends_with("/initialize"), "{log:?}");
    assert_eq!(
        driver
            .ad
            .port_base
            .get_string_param(driver.ad.params.status_message, 0)
            .unwrap(),
        b"Eiger FAILED TO INITIALIZE"
    );
}
