//! End-to-end test of the FX4 port against a local WebSocket server that
//! speaks the meter's side of the JSON protocol.
//!
//! No FX4 hardware is available, so this stands in for it: it exercises the
//! socket thread (connect, send), the data thread (frame → `Fx4Cache` →
//! `computePositions` → parameter library) and the `get` poll loop that keeps
//! the meter reporting.

use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use serde_json::{Value, json};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

use quadem::create_fx4;
use quadem::drv_quad_em::{QE_CURRENT1, QE_CURRENT4, QE_SUM_ALL};
use quadem::fx4_proto::{ADC_PATHS, GATE_PATH};

const TIMEOUT: Duration = Duration::from_secs(10);

/// One `update` frame: one sample per channel, plus a gate transition.
fn update_frame() -> String {
    let mut data = serde_json::Map::new();
    for (i, path) in ADC_PATHS.iter().enumerate() {
        data.insert(
            (*path).to_string(),
            json!([[(i + 1) as f64, 1_000_000_000i64]]),
        );
    }
    data.insert(GATE_PATH.to_string(), json!([[true, 1_000_000_000i64]]));
    json!({ "event": "update", "data": Value::Object(data) }).to_string()
}

#[test]
fn the_port_subscribes_polls_and_publishes_what_the_meter_sends() {
    let (addr_tx, addr_rx) = mpsc::channel();
    let (received_tx, received_rx) = mpsc::channel::<String>();

    // The meter: accept one connection, record what the port sends, answer the
    // first `get` with an update frame.
    let meter = thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("meter runtime");
        rt.block_on(async move {
            let listener = TcpListener::bind("127.0.0.1:0").await.expect("bind");
            addr_tx
                .send(listener.local_addr().expect("local_addr"))
                .expect("send address");

            let (stream, _) = listener.accept().await.expect("accept");
            let ws = tokio_tungstenite::accept_async(stream)
                .await
                .expect("handshake");
            let (mut sink, mut source) = ws.split();

            // While the port is idle it polls the meter every five seconds
            // (C++ `pollThread`), so drop everything until the subscribe that
            // starts the acquisition.
            loop {
                let Some(Ok(Message::Text(text))) = source.next().await else {
                    return;
                };
                if text.contains("subscribe") {
                    if received_tx.send(text.to_string()).is_err() {
                        return;
                    }
                    break;
                }
            }

            // The get that follows the subscribe.
            let Some(Ok(Message::Text(text))) = source.next().await else {
                return;
            };
            if received_tx.send(text.to_string()).is_err() {
                return;
            }

            sink.send(Message::Text(update_frame().into()))
                .await
                .expect("send update");

            // The get the port sends once it has processed the update.
            if let Some(Ok(Message::Text(text))) = source.next().await {
                let _ = received_tx.send(text.to_string());
            }

            // Hold the socket open while the test reads the parameters back.
            tokio::time::sleep(Duration::from_secs(2)).await;
        });
    });

    let address = addr_rx.recv_timeout(TIMEOUT).expect("meter address");
    let port = create_fx4("FX4_TEST", &address.to_string(), 100, 1 << 20).expect("create_fx4");
    let handle = port.port_handle();

    // A unit current scale makes the published current the raw ADC value.
    for channel in 0..4 {
        handle
            .write_float64_blocking(port.params.current_scale, channel, 1.0)
            .expect("current scale");
    }

    handle
        .write_int32_blocking(port.nd_params.acquire, 0, 1)
        .expect("acquire");

    let subscribe: Value =
        serde_json::from_str(&received_rx.recv_timeout(TIMEOUT).expect("subscribe")).unwrap();
    assert_eq!(subscribe["event"], "subscribe");
    assert_eq!(subscribe["data"][ADC_PATHS[0]], json!(true));
    assert_eq!(subscribe["data"][GATE_PATH], json!(true));

    let get: Value =
        serde_json::from_str(&received_rx.recv_timeout(TIMEOUT).expect("get")).unwrap();
    assert_eq!(get["event"], "get");

    // The port polls again only after it has handled the update frame.
    let poll: Value =
        serde_json::from_str(&received_rx.recv_timeout(TIMEOUT).expect("second get")).unwrap();
    assert_eq!(poll["event"], "get");

    // The sample reached the parameter library: channels 1-4 carry 1.0-4.0 and
    // the sum 10.0.
    let deadline = Instant::now() + TIMEOUT;
    loop {
        let current1 = handle
            .read_float64_blocking(port.params.double_data, QE_CURRENT1 as i32)
            .expect("read Current1");
        if current1 != 0.0 || Instant::now() > deadline {
            break;
        }
        thread::sleep(Duration::from_millis(10));
    }

    for (i, expected) in [1.0, 2.0, 3.0, 4.0].iter().enumerate() {
        let value = handle
            .read_float64_blocking(port.params.double_data, (QE_CURRENT1 + i) as i32)
            .expect("read current");
        assert_eq!(value, *expected, "current {}", i + 1);
    }
    assert_eq!(QE_CURRENT4, QE_CURRENT1 + 3);
    let sum = handle
        .read_float64_blocking(port.params.double_data, QE_SUM_ALL as i32)
        .expect("read SumAll");
    assert_eq!(sum, 10.0);

    drop(port);
    meter.join().expect("meter thread");
}
