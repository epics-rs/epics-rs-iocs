//! drvQuadEM::doDataCallbacks sets each array's uniqueId and double
//! timeStamp but never its epicsTS (drvQuadEM.cpp:334-336, :352-353), which
//! the NDArray constructor zeroes (NDArray.cpp:31-32): every plugin
//! downstream reads EpicsTSSec_RBV and EpicsTSNsec_RBV as 0. Driven through
//! an FX4 port, whose meter is a local WebSocket server.

use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use serde_json::{Value, json};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

use epics_rs::ad_core::plugin::channel::ndarray_channel;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::registry::claim_port;
use quadem::create_fx4;
use quadem::drv_quad_em::{QE_ADDR_ALL, QE_CURRENT1};
use quadem::fx4_proto::{ADC_PATHS, GATE_PATH, sample_time};

const TIMEOUT: Duration = Duration::from_secs(10);

/// One `update` frame: one sample per channel.
fn update_frame() -> String {
    let mut data = serde_json::Map::new();
    for path in ADC_PATHS {
        data.insert(path.to_string(), json!([[1.0, 1_000_000_000i64]]));
    }
    data.insert(GATE_PATH.to_string(), json!([[true, 1_000_000_000i64]]));
    json!({ "event": "update", "data": Value::Object(data) }).to_string()
}

#[test]
fn published_arrays_leave_epics_ts_zero() {
    let (addr_tx, addr_rx) = mpsc::channel();

    // The meter: answer the get that follows the subscribe with one sample.
    thread::spawn(move || {
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
            while let Some(Ok(Message::Text(text))) = source.next().await {
                if text.contains("subscribe") {
                    break;
                }
            }
            let _ = source.next().await;
            sink.send(Message::Text(update_frame().into()))
                .await
                .expect("send update");
            while source.next().await.is_some() {}
        });
    });

    let address = addr_rx.recv_timeout(TIMEOUT).expect("meter address");
    let port = create_fx4(
        &claim_port("FX4_EPICS_TS").unwrap(),
        &address.to_string(),
        100,
        1 << 20,
    )
    .expect("create_fx4");
    let handle = port.port_handle();

    let (array_tx, array_rx) = mpsc::channel();
    for addr in [QE_ADDR_ALL, QE_CURRENT1] {
        let (sender, mut receiver) = ndarray_channel("EPICS_TS_SINK", 4);
        port.outputs[addr].lock().add(sender);
        let array_tx = array_tx.clone();
        thread::spawn(move || {
            while let Some(array) = receiver.blocking_recv() {
                let _ = array_tx.send((addr, array));
            }
        });
    }

    // One sample per callback: numAverage = AveragingTime / SampleTime, at
    // the default ValuesPerRead of 1.
    handle
        .write_float64_blocking(port.params.averaging_time, 0, sample_time(1))
        .expect("AveragingTime");
    handle
        .write_int32_blocking(port.nd_params.acquire, 0, 1)
        .expect("acquire");

    let mut seen = Vec::new();
    while seen.len() < 2 {
        let (addr, array) = array_rx.recv_timeout(TIMEOUT).expect("an array");
        if seen.contains(&addr) {
            continue;
        }
        assert_eq!(array.timestamp, EpicsTimestamp::default(), "address {addr}");
        assert_ne!(array.time_stamp, 0.0, "address {addr}");
        seen.push(addr);
    }
}
