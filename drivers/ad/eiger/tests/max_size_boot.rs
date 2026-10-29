//! eigerDetector.cpp:1636-1652: initParams sets MaxSize from the detector's
//! x/y_pixels_in_detector and splits its description into manufacturer and
//! model.

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;

use eiger::driver::{EigerConfig, EigerDriver};
use eiger::params::Model;
use eiger::rest::{ApiVersion, RestApi};
use eiger::tasks;

/// An initialized 1030 x 514 detector describing itself as `description`;
/// every other GET is a two-valued enum.
fn detector(description: &'static str) -> u16 {
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
            let path = request.split_whitespace().nth(1).unwrap_or("");
            let int = |v: i32| format!(r#"{{"value":{v},"value_type":"uint","access_mode":"r"}}"#);
            let body = if request.starts_with("PUT") {
                "[]".to_string()
            } else if path.ends_with("/x_pixels_in_detector") {
                int(1030)
            } else if path.ends_with("/y_pixels_in_detector") {
                int(514)
            } else if path.ends_with("/description") {
                format!(r#"{{"value":"{description}","value_type":"string","access_mode":"r"}}"#)
            } else if path.ends_with("/state") {
                r#"{"value":"idle","value_type":"string","access_mode":"r"}"#.to_string()
            } else {
                r#"{"value":"disabled","value_type":"string","access_mode":"rw",
                    "allowed_values":["disabled","enabled"]}"#
                    .to_string()
            };
            let response = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            let _ = stream.write_all(response.as_bytes());
        }
    });
    port
}

fn boot(port: &str, description: &'static str) -> EigerDriver {
    let rest = RestApi::new("127.0.0.1", detector(description));
    let (signals, _receivers) = tasks::signals();
    let cfg = EigerConfig {
        port_name: port.to_string(),
        api: ApiVersion::V1_6_0,
        model: Model::Eiger1,
        max_memory: 0,
    };
    EigerDriver::new(cfg, rest, signals).expect("driver")
}

#[test]
fn max_size_is_the_detectors_pixel_count() {
    let driver = boot("EIGER_MAX_SIZE", "Dectris EIGER");
    let (p, base) = (driver.ad.params, &driver.ad.port_base);
    assert_eq!(base.get_int32_param(p.max_size_x, 0).unwrap(), 1030);
    assert_eq!(base.get_int32_param(p.max_size_y, 0).unwrap(), 514);
}

/// C splits at the first space; with none, `space + 1` wraps npos to 0 and
/// both halves are the whole description.
#[test]
fn the_description_splits_into_manufacturer_and_model() {
    for (port, description, manufacturer, model) in [
        (
            "EIGER_MODEL_SPACE",
            "Dectris EIGER2 1M",
            "Dectris",
            "EIGER2 1M",
        ),
        ("EIGER_MODEL_NO_SPACE", "EIGER2", "EIGER2", "EIGER2"),
    ] {
        let driver = boot(port, description);
        let (p, base) = (driver.ad.params, &driver.ad.port_base);
        let text = |reason| {
            String::from_utf8_lossy(base.get_string_param(reason, 0).unwrap()).into_owned()
        };
        assert_eq!(text(p.base.manufacturer), manufacturer, "{description}");
        assert_eq!(text(p.base.model), model, "{description}");
    }
}
