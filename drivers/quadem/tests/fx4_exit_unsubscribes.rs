//! drvQuadEM arms `epicsAtExit(exitHandlerC, this)` (drvQuadEM.cpp:154), and
//! drvFX4::exitHandler is `setAcquire(0)` (drvFX4.cpp:548-553): an FX4 still
//! acquiring when the IOC exits is sent the unsubscribe before the socket
//! goes. Its own binary, as `call_at_exits` runs every exit callback in the
//! process.

use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use futures_util::StreamExt;
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::Message;

use epics_rs::asyn::registry::claim_port;
use epics_rs::base::runtime::exit::call_at_exits;
use quadem::create_fx4;

const TIMEOUT: Duration = Duration::from_secs(10);
const UNSUBSCRIBE: &str = r#"{"data":{},"event":"subscribe"}"#;

#[test]
fn an_acquiring_fx4_is_unsubscribed_at_exit() {
    let (addr_tx, addr_rx) = mpsc::channel();
    let (received_tx, received_rx) = mpsc::channel::<String>();

    // The meter: record every frame the port sends; answer nothing, so the
    // acquisition's poll stops after its first get.
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
            let mut ws = tokio_tungstenite::accept_async(stream)
                .await
                .expect("handshake");
            while let Some(Ok(Message::Text(text))) = ws.next().await {
                if received_tx.send(text.to_string()).is_err() {
                    return;
                }
            }
        });
    });

    let address = addr_rx.recv_timeout(TIMEOUT).expect("meter address");
    let port = create_fx4(
        &claim_port("FX4_EXIT").unwrap(),
        &address.to_string(),
        100,
        1 << 20,
    )
    .expect("create_fx4");
    port.port_handle()
        .write_int32_blocking(port.nd_params.acquire, 0, 1)
        .expect("acquire");
    loop {
        let frame = received_rx.recv_timeout(TIMEOUT).expect("subscribe");
        if frame.contains("/fx4/adc/channel_1/value") {
            break;
        }
    }

    call_at_exits();

    loop {
        let frame = received_rx
            .recv_timeout(Duration::from_secs(3))
            .expect("no unsubscribe at exit");
        if frame == UNSUBSCRIBE {
            break;
        }
    }
}
