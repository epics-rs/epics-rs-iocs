//! Port of `quadEMApp/FX4Src/drvFX4.{h,cpp}` — the Pyramid FX4 4-channel
//! picoammeter, reached over a JSON WebSocket.
//!
//! Upstream uses ixwebsocket (C++) and nlohmann/json; this port uses
//! `tokio-tungstenite` and `serde_json`, with ixwebsocket's own opening
//! handshake ported so its failures read as C's. The protocol itself — the message
//! encoding, the per-channel sample cache and the gate/trigger state machine —
//! is in [`crate::fx4_proto`] and is unit-tested there.
//!
//! Three threads make up the port:
//!
//! * the **socket thread** owns a current-thread tokio runtime, keeps the
//!   WebSocket connected (reconnecting on its own, which is what upstream's
//!   `pollThread` open-codes) and moves frames between the socket and two
//!   channels;
//! * the **data thread** takes each received frame, runs it through
//!   [`Fx4Cache`] and publishes the samples — everything the framework needs a
//!   blocking thread for (`compute_positions`, `trigger_callbacks`) happens
//!   here, off the tokio runtime;
//! * the **callback thread** is `drvQuadEM`'s, shared with every other quadEM
//!   port.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::mpsc as std_mpsc;
use std::thread;
use std::time::{Duration, Instant};

use futures_util::{FutureExt, SinkExt, StreamExt};
use tokio::sync::mpsc as tokio_mpsc;
use tokio::time::Instant as TokioInstant;
use tokio_tungstenite::tungstenite::Message;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QeModel, QeTriggerMode, QeTriggerPolarity, QuadEmBase,
    QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::fx4_proto::{self as proto, Fx4Action, Fx4Cache, TriggerConfig};
use crate::ixwebsocket;
use crate::trace::AsynPrint;

/// C++ `onMessageEvent`'s `epicsThreadSleep(0.01)` before the next `get`: the
/// FX4 is polled at most 100 times a second.
const POLL_SLEEP: Duration = Duration::from_millis(10);

/// C++ `pollThread`'s `epicsThreadSleep(5.0)` — the reconnect and idle-`get`
/// cadence.
const POLL_INTERVAL: Duration = Duration::from_secs(5);

/// C++ `waitForConnection(5.0)` in the constructor.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// C++ `waitForConnection`'s `epicsThreadSleep(0.01)` poll.
const CONNECT_POLL: Duration = Duration::from_millis(10);

/// C++ `reconnectWebSocket`'s `epicsThreadSleep(0.1)` between stop and start.
const RESTART_PAUSE: Duration = Duration::from_millis(100);

/// How long a write waits for the reconnect it asked for: the pause and the
/// five-second wait of `reconnectWebSocket`, with a second to spare.
const RECONNECT_WAIT: Duration = Duration::from_millis(6100);

/// The bare `asynError` C returns: no `errorMessage` is set.
fn refused() -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: String::new(),
    }
}

// ===========================================================================
// WebSocket link
// ===========================================================================

/// Everything the driver and the two device threads share: the socket's
/// outbound queue, its connection state, the event cache, and the acquisition
/// settings C++ keeps in `triggerMode_` / `triggerPolarity_` / `numAverage_`.
pub struct Fx4Link {
    connected: AtomicBool,
    /// A write's request for an immediate reconnect.
    kick: tokio::sync::Notify,
    /// Reconnect attempts the socket thread has finished, for the write
    /// waiting on one.
    attempts: parking_lot::Mutex<u64>,
    attempt_done: parking_lot::Condvar,
    out_tx: tokio_mpsc::UnboundedSender<String>,
    cache: parking_lot::Mutex<Fx4Cache>,
    trigger_mode: AtomicI32,
    trigger_polarity: AtomicI32,
    num_average: AtomicI32,
}

impl Fx4Link {
    fn new(out_tx: tokio_mpsc::UnboundedSender<String>) -> Self {
        Self {
            connected: AtomicBool::new(false),
            kick: tokio::sync::Notify::new(),
            attempts: parking_lot::Mutex::new(0),
            attempt_done: parking_lot::Condvar::new(),
            out_tx,
            cache: parking_lot::Mutex::new(Fx4Cache::new()),
            trigger_mode: AtomicI32::new(0),
            trigger_polarity: AtomicI32::new(0),
            num_average: AtomicI32::new(proto::DEFAULT_NUM_AVERAGE),
        }
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }

    fn set_connected(&self, value: bool) {
        self.connected.store(value, Ordering::Release);
    }

    /// C++ `reconnectWebSocket` called from a write. The socket thread owns
    /// the socket, so the write asks it for an attempt and blocks on the
    /// outcome, as C's synchronous reconnect does.
    fn reconnect(&self) {
        let mut done = self.attempts.lock();
        let seen = *done;
        self.kick.notify_one();
        let deadline = Instant::now() + RECONNECT_WAIT;
        while *done == seen {
            if self
                .attempt_done
                .wait_until(&mut done, deadline)
                .timed_out()
            {
                break;
            }
        }
    }

    fn attempt_finished(&self) {
        *self.attempts.lock() += 1;
        self.attempt_done.notify_all();
    }

    /// C++ `sendEventData`: a message sent while the socket is down is dropped.
    fn send(&self, message: String) {
        if !self.is_connected() {
            return;
        }
        if self.out_tx.send(message).is_err() {
            epics_rs::base::stderr_println!("drvFX4: send failed: ");
        }
    }

    fn send_subscribe(&self) {
        self.send(proto::subscribe_message());
    }

    fn send_unsubscribe(&self) {
        self.send(proto::unsubscribe_message());
    }

    fn send_get(&self) {
        self.send(proto::get_message());
    }

    fn trigger_config(&self) -> TriggerConfig {
        TriggerConfig {
            mode: QeTriggerMode::from_i32(self.trigger_mode.load(Ordering::Relaxed)),
            polarity: QeTriggerPolarity::from_i32(self.trigger_polarity.load(Ordering::Relaxed)),
            num_average: self.num_average.load(Ordering::Relaxed),
        }
    }
}

// ===========================================================================
// Driver
// ===========================================================================

pub struct Fx4Driver {
    base: QuadEmBase,
    shared: Arc<QuadEmShared>,
    link: Arc<Fx4Link>,
}

impl Fx4Driver {
    fn new(
        port_name: &str,
        max_memory: usize,
        shared: Arc<QuadEmShared>,
        link: Arc<Fx4Link>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        base.port_base
            .set_int32_param(base.params.model, 0, QeModel::Fx4 as i32)?;
        shared.acq.lock().resolution = proto::RESOLUTION;

        Ok(Self { base, shared, link })
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    /// C++ `drvFX4::setAcquireParams`: reconnect first, refuse while down.
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        if !self.link.is_connected() {
            self.link.reconnect();
        }
        if !self.link.is_connected() {
            return Err(refused());
        }
        let p = self.params();
        let base = &mut self.base.port_base;

        let trigger_mode = qe::int_param(base, p.trigger_mode, 0);
        let trigger_polarity = qe::int_param(base, p.trigger_polarity, 0);
        let acquire_mode = qe::int_param(base, p.acquire_mode, 0);
        let values_per_read = qe::int_param(base, p.values_per_read, 0);
        let averaging_time = qe::double_param(base, p.averaging_time, 0);

        let sample_time = proto::sample_time(values_per_read);
        base.set_float64_param(p.sample_time, 0, sample_time)?;

        // In external-bulb mode the gate decides how many samples make up one
        // reading, so the ring buffer averages whatever arrived.
        let num_average = if QeTriggerMode::from_i32(trigger_mode) == QeTriggerMode::ExtBulb {
            0
        } else {
            num_average_from(averaging_time, sample_time)
        };
        base.set_int32_param(p.num_average, 0, num_average)?;

        {
            let mut acq = self.shared.acq.lock();
            acq.num_average = num_average;
            acq.acquire_mode = acquire_mode;
            acq.trigger_mode = trigger_mode;
            acq.values_per_read = values_per_read;
        }
        self.link
            .trigger_mode
            .store(trigger_mode, Ordering::Relaxed);
        self.link
            .trigger_polarity
            .store(trigger_polarity, Ordering::Relaxed);
        self.link.num_average.store(num_average, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// drvQuadEM virtuals
// ===========================================================================

impl QuadEmDevice for Fx4Driver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvFX4::setAcquire`.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        let acquiring = self.shared.is_acquiring();
        if (value != 0) == acquiring {
            return Ok(());
        }
        if value != 0 && !self.link.is_connected() {
            self.link.reconnect();
        }
        if value != 0 && !self.link.is_connected() {
            return Err(refused());
        }

        if value != 0 {
            self.link.cache.lock().reset();
            self.shared.set_acquiring(true);
            self.link.send_subscribe();
            self.link.send_get();
        } else {
            self.link.send_unsubscribe();
            self.shared.set_acquiring(false);
        }
        self.base_set_acquire(value);
        Ok(())
    }

    fn set_acquire_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_averaging_time(&mut self, _value: f64) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_trigger_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_values_per_read(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_num_acquire(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_trigger_polarity(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    /// C++ `drvFX4::reset` does nothing.
    fn reset(&mut self) -> AsynResult<()> {
        Ok(())
    }

    /// C++ `drvFX4::exitHandler`: an acquisition still running is
    /// unsubscribed before the IOC goes.
    fn exit_handler(&mut self) {
        let _ = self.set_acquire(0);
    }
}

// ===========================================================================
// asyn port driver
// ===========================================================================

impl PortDriver for Fx4Driver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvFX4::report`, which ends in the base report.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "drvFX4: port={} connected={}",
            self.base.port_base.port_name,
            i32::from(self.link.is_connected())
        );
        self.asyn_port_driver_report(out, level);
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.qe_write_int32(user, value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.qe_write_float64(user, value)
    }
}

// ===========================================================================
// Socket thread
// ===========================================================================

struct SocketContext {
    uri: String,
    link: Arc<Fx4Link>,
    shared: Arc<QuadEmShared>,
    out_rx: tokio_mpsc::UnboundedReceiver<String>,
    in_tx: std_mpsc::Sender<String>,
}

/// C++ `startWebSocket` + `pollThread`'s reconnect loop, on a current-thread
/// tokio runtime of its own.
fn socket_thread(ctx: SocketContext) {
    let SocketContext {
        uri,
        link,
        shared,
        mut out_rx,
        in_tx,
    } = ctx;

    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            epics_rs::base::stderr_println!("drvFX4: connection setup error: {e}");
            return;
        }
    };

    runtime.block_on(async move {
        // The constructor's startWebSocket + waitForConnection(5.0), which
        // waits the five seconds out after a failure; create_fx4 waits with
        // it and prints its own timeout. pollThread starts after it.
        let started = TokioInstant::now();
        let mut stream = attempt(&uri, &link, &shared).await;
        // When pollThread next passes its top.
        let mut pass = if stream.is_some() {
            TokioInstant::now()
        } else {
            started + CONNECT_TIMEOUT
        };
        loop {
            if let Some(s) = stream.take() {
                // Each pass while connected is the idle get.
                pass = run_session(s, pass, &mut out_rx, &in_tx, &link, &shared).await;
                link.set_connected(false);
            }
            // Down until the next pass, which reconnects at once and then
            // sleeps; a write's reconnectWebSocket leaves the passes alone.
            tokio::select! {
                () = tokio::time::sleep_until(pass) => {
                    stream = reconnect(&uri, &link, &shared).await;
                    pass = TokioInstant::now();
                    if stream.is_none() {
                        pass += POLL_INTERVAL;
                    }
                }
                () = link.kick.notified() => {
                    stream = reconnect(&uri, &link, &shared).await;
                }
            }
        }
    });
}

/// C++ `reconnectWebSocket`: stop, pause, start, then `waitForConnection(5.0)`,
/// which waits out the five seconds even after a prompt failure.
async fn reconnect(uri: &str, link: &Fx4Link, shared: &QuadEmShared) -> Option<Fx4Stream> {
    tokio::time::sleep(RESTART_PAUSE).await;
    let started = TokioInstant::now();
    let stream = attempt(uri, link, shared).await;
    if stream.is_none() {
        tokio::time::sleep_until(started + CONNECT_TIMEOUT).await;
        epics_rs::base::stderr_println!("drvFX4: reconnect timeout to {uri}");
    }
    // A write that asked during this attempt is answered by it; its stored
    // permit must not start another.
    let _ = link.kick.notified().now_or_never();
    link.attempt_finished();
    stream
}

/// One `startWebSocket` + `waitForConnection(5.0)`. An attempt still pending
/// at the deadline is stopped by the next reconnect with `wsStopping_` set,
/// so C never prints its error.
async fn attempt(uri: &str, link: &Fx4Link, shared: &QuadEmShared) -> Option<Fx4Stream> {
    match tokio::time::timeout(CONNECT_TIMEOUT, connect(uri)).await {
        Ok(Ok(stream)) => {
            link.set_connected(true);
            // reconnectWebSocket: an acquisition in progress has to be
            // re-subscribed and re-polled on the new socket.
            if shared.is_acquiring() {
                link.send_subscribe();
                link.send_get();
            }
            Some(stream)
        }
        Ok(Err(reason)) => {
            epics_rs::base::stderr_println!("drvFX4: WebSocket error: {reason}");
            None
        }
        Err(_) => None,
    }
}

type Fx4Stream = tokio_tungstenite::WebSocketStream<tokio::net::TcpStream>;

/// ixwebsocket's `WebSocket::connect`: the TCP connect, then the handshake.
/// The error is the `errorInfo.reason` drvFX4's onError prints.
async fn connect(uri: &str) -> Result<Fx4Stream, String> {
    ixwebsocket::connect_to_url(uri).await
}

/// Pump one connected socket until it fails: frames in, queued messages out,
/// and C++ `pollThread`'s idle `get` every five seconds.
async fn run_session<S>(
    stream: tokio_tungstenite::WebSocketStream<S>,
    mut pass: TokioInstant,
    out_rx: &mut tokio_mpsc::UnboundedReceiver<String>,
    in_tx: &std_mpsc::Sender<String>,
    link: &Fx4Link,
    shared: &QuadEmShared,
) -> TokioInstant
where
    S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
{
    let (mut sink, mut source) = stream.split();

    loop {
        tokio::select! {
            frame = source.next() => match frame {
                Some(Ok(Message::Text(text))) => {
                    if in_tx.send(text.to_string()).is_err() {
                        return pass;
                    }
                }
                // ixwebsocket's close codes: 1005 for a close frame with no
                // status, 1006 for a stream that ended without one.
                Some(Ok(Message::Close(frame))) => {
                    match frame {
                        Some(f) => epics_rs::base::stderr_println!(
                            "drvFX4: WebSocket closed code={} reason={}",
                            u16::from(f.code),
                            f.reason
                        ),
                        None => epics_rs::base::stderr_println!("drvFX4: WebSocket closed code=1005 reason=No status code"),
                    }
                    return pass;
                }
                None => {
                    epics_rs::base::stderr_println!("drvFX4: WebSocket closed code=1006 reason=Abnormal closure");
                    return pass;
                }
                // Ping/Pong are answered by tungstenite; the FX4 sends no
                // binary frames.
                Some(Ok(_)) => {}
                Some(Err(e)) => {
                    epics_rs::base::stderr_println!("drvFX4: WebSocket error: {e}");
                    return pass;
                }
            },
            outgoing = out_rx.recv() => match outgoing {
                Some(message) => {
                    if sink.send(Message::Text(message.into())).await.is_err() {
                        epics_rs::base::stderr_println!("drvFX4: send failed: ");
                        return pass;
                    }
                }
                None => return pass,
            },
            () = tokio::time::sleep_until(pass) => {
                // While acquiring, the data thread's `get` after every message
                // keeps the meter reporting; this is C++ pollThread's idle poll.
                if !shared.is_acquiring() {
                    link.send_get();
                }
                pass = TokioInstant::now() + POLL_INTERVAL;
            }
        }
    }
}

// ===========================================================================
// Data thread
// ===========================================================================

struct DataContext {
    in_rx: std_mpsc::Receiver<String>,
    link: Arc<Fx4Link>,
    shared: Arc<QuadEmShared>,
    handle: PortHandle,
    params: QuadEmParams,
}

/// C++ `drvFX4::onMessage` / `onMessageEvent`, minus the JSON and socket
/// handling: one received frame per iteration.
fn data_thread(ctx: DataContext) {
    while let Ok(payload) = ctx.in_rx.recv() {
        // C++ parses before onMessageEvent's acquiring check.
        let (event, data) = match proto::parse_message(&payload) {
            Ok(Some(message)) => message,
            Ok(None) => continue,
            Err(e) => {
                epics_rs::base::stderr_println!("drvFX4: JSON parse error: {e}");
                continue;
            }
        };
        if !ctx.shared.is_acquiring() {
            continue;
        }

        if event == "update" {
            let cfg = ctx.link.trigger_config();
            let (actions, trace) = {
                let mut cache = ctx.link.cache.lock();
                let actions = cache.ingest(&data, &cfg);
                (actions, cache.take_trace())
            };
            for (mask, line) in trace {
                ctx.handle.asyn_print(mask, format_args!("{line}"));
            }
            for action in actions {
                match action {
                    Fx4Action::Sample(values) => {
                        ctx.shared
                            .compute_positions(&ctx.handle, &ctx.params, &values);
                    }
                    Fx4Action::BulbTrigger => {
                        let _ = ctx.shared.trigger_callbacks(&ctx.handle);
                    }
                }
            }
        }

        // C++ `done:`: pace the poll, then ask for the next batch.
        thread::sleep(POLL_SLEEP);
        if ctx.shared.is_acquiring() {
            ctx.link.send_get();
        }
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured FX4 port.
pub struct Fx4Runtime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: epics_rs::ad_core::ndarray_pool::DriverPool,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    _socket_thread: thread::JoinHandle<()>,
    _data_thread: thread::JoinHandle<()>,
    _callback_thread: thread::JoinHandle<()>,
}

impl Fx4Runtime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// C++ `drvFX4Configure(portName, FX4_IP, ringBufferSize)`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`.
pub fn create_fx4(
    claim: &PortClaim,
    fx4_ip: &str,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<Fx4Runtime> {
    let port_name = claim.name();
    let uri = format!("ws://{fx4_ip}");
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);
    let (out_tx, out_rx) = tokio_mpsc::unbounded_channel();
    let (in_tx, in_rx) = std_mpsc::channel();
    let link = Arc::new(Fx4Link::new(out_tx));

    let driver = Fx4Driver::new(port_name, max_memory, shared.clone(), link.clone())?;
    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(claim, driver, RuntimeConfig::default())?;
    let handle = runtime_handle.port_handle().clone();

    let socket_ctx = SocketContext {
        uri: uri.clone(),
        link: link.clone(),
        shared: shared.clone(),
        out_rx,
        in_tx,
    };
    let socket_thread_handle = thread::Builder::new()
        .name("drvFX4Socket".into())
        .spawn(move || socket_thread(socket_ctx))
        .expect("failed to spawn drvFX4Socket");

    let data_ctx = DataContext {
        in_rx,
        link: link.clone(),
        shared: shared.clone(),
        handle: handle.clone(),
        params,
    };
    let data_thread_handle = thread::Builder::new()
        .name("drvFX4Task".into())
        .spawn(move || data_thread(data_ctx))
        .expect("failed to spawn drvFX4Task");

    let callback_thread = qe::start_callback_task::<Fx4Driver>(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
        pool: pool.clone(),
        shared,
        acquire_param,
    });

    // C++ `waitForConnection(5.0)` in the constructor: a meter that is not up
    // yet is reported, not fatal — the socket thread keeps retrying.
    let deadline = Instant::now() + CONNECT_TIMEOUT;
    while !link.is_connected() && Instant::now() < deadline {
        thread::sleep(CONNECT_POLL);
    }
    if !link.is_connected() {
        epics_rs::base::stderr_println!("drvFX4::drvFX4: timeout connecting to FX4 at {uri}");
    }

    Ok(Fx4Runtime {
        runtime_handle,
        params,
        nd_params,
        pool,
        outputs,
        _socket_thread: socket_thread_handle,
        _data_thread: data_thread_handle,
        _callback_thread: callback_thread,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A refused connect is found by poll, so ixwebsocket prefixes
    /// `Connect error: `; drvFX4's onError line carries the whole reason.
    #[cfg(unix)]
    #[test]
    fn a_refused_connect_reads_as_ixwebsocket_reason() {
        let port = {
            let l = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
            l.local_addr().expect("addr").port()
        };
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("runtime");
        let Err(reason) = rt.block_on(connect(&format!("ws://127.0.0.1:{port}"))) else {
            panic!("nothing listens");
        };
        let text = ixwebsocket::strerror(&std::io::Error::from_raw_os_error(libc::ECONNREFUSED));
        assert_eq!(
            reason,
            format!("Unable to connect to 127.0.0.1 on port {port}, error: Connect error: {text}")
        );
    }

    /// setAcquire / setAcquireParams reconnect before refusing: a meter that
    /// came back is reached by the write itself, not at the next poll.
    #[test]
    fn a_write_reconnects_before_refusing() {
        // Held from first connect to last: a released number can be taken
        // before the meter comes back on it.
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        let (back_tx, back_rx) = std_mpsc::channel::<()>();
        thread::spawn(move || {
            loop {
                let (sock, _) = listener.accept().expect("accept");
                if back_rx.try_recv().is_ok() {
                    let _ws = tungstenite_accept(sock);
                    thread::sleep(Duration::from_secs(10));
                    return;
                }
            }
        });
        let (shared, _trigger_rx) = QuadEmShared::new(16);
        let (out_tx, out_rx) = tokio_mpsc::unbounded_channel();
        let (in_tx, _in_rx) = std_mpsc::channel();
        let link = Arc::new(Fx4Link::new(out_tx));
        let ctx = SocketContext {
            uri: format!("ws://127.0.0.1:{port}"),
            link: link.clone(),
            shared,
            out_rx,
            in_tx,
        };
        thread::spawn(move || socket_thread(ctx));
        // The constructor's attempt is closed before its handshake.
        thread::sleep(Duration::from_millis(300));
        assert!(!link.is_connected());

        back_tx.send(()).expect("meter back");
        let started = Instant::now();
        link.reconnect();
        assert!(link.is_connected());
        // Well inside the poll sleep the socket thread would otherwise wait.
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "{:?}",
            started.elapsed()
        );
    }

    /// Start the socket thread against `port`; returns its link.
    fn spawn_socket_thread(port: u16) -> Arc<Fx4Link> {
        let (shared, _trigger_rx) = QuadEmShared::new(16);
        let (out_tx, out_rx) = tokio_mpsc::unbounded_channel();
        let (in_tx, in_rx) = std_mpsc::channel();
        let link = Arc::new(Fx4Link::new(out_tx));
        let ctx = SocketContext {
            uri: format!("ws://127.0.0.1:{port}"),
            link: link.clone(),
            shared,
            out_rx,
            in_tx,
        };
        thread::spawn(move || {
            let _in_rx = in_rx;
            socket_thread(ctx)
        });
        link
    }

    fn wait_connected(link: &Fx4Link, within: Duration) -> Option<Duration> {
        let started = Instant::now();
        while started.elapsed() < within {
            if link.is_connected() {
                return Some(started.elapsed());
            }
            thread::sleep(Duration::from_millis(10));
        }
        None
    }

    /// pollThread starts once the constructor's five seconds are up and
    /// reconnects at its first pass, however long the failed attempt hung.
    #[test]
    fn the_first_pass_after_a_hung_constructor_connect_reconnects_at_once() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        thread::spawn(move || {
            // The constructor's handshake gets no answer.
            let (_silent, _) = listener.accept().expect("accept");
            let (sock, _) = listener.accept().expect("accept");
            let _ws = tungstenite_accept(sock);
            thread::sleep(Duration::from_secs(10));
        });
        let link = spawn_socket_thread(port);
        let at = wait_connected(&link, Duration::from_secs(9)).expect("reconnected");
        // 5 s constructor + 0.1 s pause, not a further 5 s sleep.
        assert!(at < Duration::from_millis(6500), "{at:?}");
    }

    /// A session that ends mid-sleep is reconnected at pollThread's next
    /// pass, five seconds after the session's first, not five after the end.
    #[test]
    fn a_dropped_session_reconnects_at_the_next_pass() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        let (accepted_tx, accepted_rx) = std_mpsc::channel();
        thread::spawn(move || {
            let (sock, _) = listener.accept().expect("accept");
            let ws = tungstenite_accept(sock);
            thread::sleep(Duration::from_secs(3));
            drop(ws);
            let (sock, _) = listener.accept().expect("accept");
            accepted_tx.send(Instant::now()).expect("send");
            let _ws = tungstenite_accept(sock);
            thread::sleep(Duration::from_secs(10));
        });
        let link = spawn_socket_thread(port);
        wait_connected(&link, Duration::from_secs(2)).expect("connected");
        let first_pass = Instant::now();
        let again = accepted_rx
            .recv_timeout(Duration::from_secs(9))
            .expect("reconnected");
        let after = again - first_pass;
        // The pass at 5 s plus the 0.1 s pause; the drop at 3 s would give 8.1.
        assert!(after < Duration::from_millis(6500), "{after:?}");
    }

    fn tungstenite_accept(sock: std::net::TcpStream) -> impl Sized {
        tokio_tungstenite::tungstenite::accept(sock).expect("handshake")
    }
}
