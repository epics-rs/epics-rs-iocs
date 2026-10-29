//! `opcuaConnect`, `opcuaDisconnect` and the autoconnect at `iocInit` return
//! once their work is done, as C's do: `UA_Client_connect` activates the
//! session before it returns (`SessionOpen62541.cpp:577`), and
//! `UA_Client_delete` closes it (`SessionOpen62541.cpp:605-628`).

mod common;

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
use parking_lot::Mutex;
use tokio::sync::oneshot;

use common::{Server, registry};
use opcua::ioc;
use opcua::registry::Registry;
use opcua::session::Control;

/// Run the command on a thread of its own, as iocsh does, and return the
/// errlog lines it printed by the time it returned.
async fn command(registry: &Arc<Registry>, control: Control) -> Vec<String> {
    errlog_flush();
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    let registry = registry.clone();
    let (returned, finished) = oneshot::channel();
    std::thread::spawn(move || {
        ioc::control_command(&registry, Some("S1"), control);
        // What the command left queued, as iocsh flushes it before the prompt.
        errlog_flush();
        let _ = returned.send(());
    });
    tokio::time::timeout(Duration::from_secs(5), finished)
        .await
        .expect("the command returned")
        .unwrap();
    errlog_remove_listener(id);
    lines.lock().clone()
}

fn slow_server() -> Arc<Server> {
    Arc::new(Server {
        latency: Duration::from_millis(50),
        ..Default::default()
    })
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn opcua_connect_returns_with_the_session_up() {
    let server = slow_server();
    let registry = registry(&server, "autoconnect=n");
    registry.start();
    let session = registry.session("S1").unwrap();

    let lines = command(&registry, Control::Connect).await;
    assert!(session.is_connected());
    assert_eq!(
        lines,
        [
            "OPC UA session S1: connected as 'Anonymous' with security level 0 (mode=None; policy=None)\n",
            "OPC UA session S1: WARNING - this session uses *** NO SECURITY ***\n",
        ]
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn opcua_connect_returns_with_a_failed_attempt_reported() {
    let server = Arc::new(Server {
        refuse: true,
        latency: Duration::from_millis(50),
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=n");
    registry.start();

    let lines = command(&registry, Control::Connect).await;
    assert_eq!(
        lines,
        ["OPC UA session S1: connect service failed with status BadDisconnect\n"]
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn opcua_disconnect_returns_with_the_session_closed() {
    let server = slow_server();
    let registry = registry(&server, "autoconnect=n");
    registry.start();
    let session = registry.session("S1").unwrap();
    command(&registry, Control::Connect).await;

    command(&registry, Control::Disconnect).await;
    assert_eq!(server.disconnects.load(Ordering::SeqCst), 1);
    assert!(!session.is_connected());
}

/// Before `iocInit` no worker runs to carry the command out; it waits in the
/// queue and the command returns.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn opcua_connect_before_the_start_does_not_wait() {
    let server = slow_server();
    let registry = registry(&server, "autoconnect=n");

    let lines = command(&registry, Control::Connect).await;
    assert_eq!(lines, Vec::<String>::new());
    assert_eq!(server.connects.load(Ordering::SeqCst), 0);
}

/// `SessionOpen62541::initHook` connects the autoconnect sessions one after
/// another, in name order, before `iocInit` goes on
/// (`SessionOpen62541.cpp:2697-2703`).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_start_returns_with_the_autoconnect_sessions_up_in_name_order() {
    let server = slow_server();
    let registry = registry(&server, "autoconnect=y");
    ioc::session_command(
        &registry,
        Some("S0"),
        Some("opc.tcp://localhost:4999"),
        &["autoconnect=y"],
    );

    let lines = logged(|| registry.start());
    assert!(registry.session("S0").unwrap().is_connected());
    assert!(registry.session("S1").unwrap().is_connected());
    assert_eq!(
        lines,
        [
            "OPC UA: Autoconnecting sessions\n",
            "OPC UA session S0: connected as 'Anonymous' with security level 0 (mode=None; policy=None)\n",
            "OPC UA session S0: WARNING - this session uses *** NO SECURITY ***\n",
            "OPC UA session S1: connected as 'Anonymous' with security level 0 (mode=None; policy=None)\n",
            "OPC UA session S1: WARNING - this session uses *** NO SECURITY ***\n",
        ]
    );
}

/// The autoconnect is C's `connect(false)`: a failed attempt is not reported.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_start_returns_with_a_failed_autoconnect_unreported() {
    let server = Arc::new(Server {
        refuse: true,
        latency: Duration::from_millis(50),
        ..Default::default()
    });
    let registry = registry(&server, "autoconnect=y");

    let lines = logged(|| registry.start());
    assert_eq!(server.connects.load(Ordering::SeqCst), 1);
    assert_eq!(lines, ["OPC UA: Autoconnecting sessions\n"]);
}

/// Run `f` and return what it logged, flushed as iocsh flushes it.
fn logged(f: impl FnOnce()) -> Vec<String> {
    errlog_flush();
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    f();
    errlog_flush();
    errlog_remove_listener(id);
    lines.lock().clone()
}
