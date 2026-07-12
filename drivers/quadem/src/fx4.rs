//! Port of `quadEMApp/FX4Src/drvFX4.{h,cpp}` — the Pyramid FX4 4-channel
//! picoammeter, reached over a JSON WebSocket.
//!
//! Upstream uses ixwebsocket (C++) and nlohmann/json; this port uses
//! `tokio-tungstenite` and `serde_json`. The protocol itself — the message
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

use futures_util::{SinkExt, StreamExt};
use tokio::sync::mpsc as tokio_mpsc;
use tokio_tungstenite::tungstenite::Message;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeTriggerMode,
    QeTriggerPolarity, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::fx4_proto::{self as proto, Fx4Action, Fx4Cache, TriggerConfig};

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

fn error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
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

    /// C++ `sendEventData`: a message sent while the socket is down is dropped.
    fn send(&self, message: String) {
        if !self.is_connected() {
            return;
        }
        if self.out_tx.send(message).is_err() {
            log::error!("drvFX4: the WebSocket thread is gone, dropping a message");
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

    fn acquire_param(&self) -> usize {
        self.base.nd_params.acquire
    }

    /// C++ `drvFX4::setAcquireParams`.
    ///
    /// Upstream first tries to reconnect the socket and errors out when it is
    /// still down; here the socket thread reconnects on its own, so this only
    /// refuses to compute settings the meter cannot be told about.
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        if !self.link.is_connected() {
            return Err(error("FX4: not connected"));
        }
        let p = self.params();
        let base = &mut self.base.port_base;

        let trigger_mode = base.get_int32_param(p.trigger_mode, 0)?;
        let trigger_polarity = base.get_int32_param(p.trigger_polarity, 0)?;
        let acquire_mode = base.get_int32_param(p.acquire_mode, 0)?;
        let values_per_read = base.get_int32_param(p.values_per_read, 0)?;
        let averaging_time = base.get_float64_param(p.averaging_time, 0)?;

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

    /// Mirror a parameter write into the shared position snapshot the data
    /// thread consumes.
    fn cache_position_param(&mut self, reason: usize, addr: i32, value: f64) {
        let p = self.params();
        let i = addr.clamp(0, QE_MAX_INPUTS as i32 - 1) as usize;
        let j = addr.clamp(0, 1) as usize;
        let mut pos = self.shared.pos.lock();
        if reason == p.current_offset {
            pos.current_offset[i] = value;
        } else if reason == p.current_scale {
            pos.current_scale[i] = value;
        } else if reason == p.position_offset {
            pos.position_offset[j] = value;
        } else if reason == p.position_scale {
            pos.position_scale[j] = value;
        } else if reason == p.weight_xsum {
            pos.weight_xsum[i] = value;
        } else if reason == p.weight_ysum {
            pos.weight_ysum[i] = value;
        } else if reason == p.weight_xdelta {
            pos.weight_xdelta[i] = value;
        } else if reason == p.weight_ydelta {
            pos.weight_ydelta[i] = value;
        }
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
            return Err(error("FX4: not connected"));
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
        self.base_set_acquire(value)
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

    /// C++ `drvQuadEM::writeInt32` with the FX4 overrides bound in. The
    /// meter's range, bias, resolution and read format are set through the CA
    /// links of `FX4.template`, not through this port, so those branches are
    /// `drvQuadEM`'s dummies; `readStatus` and `reset` are no-ops for the FX4.
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let channel = user.addr;
        let p = self.params();

        self.base
            .port_base
            .set_int32_param(reason, channel, value)?;

        if reason == self.acquire_param() {
            if value != 0 {
                self.shared.ring.lock().flush();
            }
            self.set_acquire(value)?;
        } else if reason == p.acquire_mode {
            if QeAcquireMode::from_i32(value) != QeAcquireMode::Continuous {
                self.set_acquire(0)?;
                self.base
                    .port_base
                    .set_int32_param(self.acquire_param(), 0, 0)?;
            }
            self.set_acquire_mode(value)?;
        } else if reason == p.geometry {
            self.shared.pos.lock().geometry = value;
        } else if reason == p.num_acquire {
            self.shared.acq.lock().num_acquire = value;
            // C++ drvFX4::setNumAcquire is setAcquireParams.
            self.set_acquire_params()?;
        } else if reason == p.trigger_mode || reason == p.trigger_polarity {
            self.set_acquire_params()?;
        } else if reason == p.values_per_read {
            self.set_values_per_read(value)?;
        } else if reason == p.read_data {
            self.shared.trigger_callbacks();
        } else if self.base.write_int32_pool(reason)? {
            // Handled by the NDArray pool controls.
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }

    /// C++ `drvQuadEM::writeFloat64` with the FX4 overrides bound in.
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let channel = user.addr;
        let p = self.params();

        self.base
            .port_base
            .set_float64_param(reason, channel, value)?;
        self.cache_position_param(reason, channel, value);

        if reason == p.averaging_time {
            self.set_averaging_time(value)?;
            self.shared.ring.lock().flush();
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
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
            log::error!("drvFX4: cannot start the WebSocket runtime: {e}");
            return;
        }
    };

    runtime.block_on(async move {
        loop {
            match tokio_tungstenite::connect_async(&uri).await {
                Ok((stream, _)) => {
                    link.set_connected(true);
                    // C++ reconnectWebSocket: an acquisition in progress has to
                    // be re-subscribed and re-polled on the new socket.
                    if shared.is_acquiring() {
                        link.send_subscribe();
                        link.send_get();
                    }
                    run_session(stream, &mut out_rx, &in_tx, &link, &shared).await;
                    link.set_connected(false);
                    log::error!("drvFX4: the WebSocket to {uri} closed");
                }
                Err(e) => log::error!("drvFX4: cannot connect to {uri}: {e}"),
            }
            tokio::time::sleep(POLL_INTERVAL).await;
        }
    });
}

/// Pump one connected socket until it fails: frames in, queued messages out,
/// and C++ `pollThread`'s idle `get` every five seconds.
async fn run_session<S>(
    stream: tokio_tungstenite::WebSocketStream<S>,
    out_rx: &mut tokio_mpsc::UnboundedReceiver<String>,
    in_tx: &std_mpsc::Sender<String>,
    link: &Fx4Link,
    shared: &QuadEmShared,
) where
    S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin,
{
    let (mut sink, mut source) = stream.split();
    let mut idle = tokio::time::interval(POLL_INTERVAL);

    loop {
        tokio::select! {
            frame = source.next() => match frame {
                Some(Ok(Message::Text(text))) => {
                    if in_tx.send(text.to_string()).is_err() {
                        return;
                    }
                }
                Some(Ok(Message::Close(_))) | None => return,
                // Ping/Pong are answered by tungstenite; the FX4 sends no
                // binary frames.
                Some(Ok(_)) => {}
                Some(Err(e)) => {
                    log::error!("drvFX4: WebSocket error: {e}");
                    return;
                }
            },
            outgoing = out_rx.recv() => match outgoing {
                Some(message) => {
                    if let Err(e) = sink.send(Message::Text(message.into())).await {
                        log::error!("drvFX4: send failed: {e}");
                        return;
                    }
                }
                None => return,
            },
            _ = idle.tick() => {
                // While acquiring, the data thread's `get` after every message
                // keeps the meter reporting; this is C++ pollThread's idle poll.
                if !shared.is_acquiring() {
                    link.send_get();
                }
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
        if !ctx.shared.is_acquiring() {
            continue;
        }
        let Some((event, data)) = proto::parse_message(&payload) else {
            log::error!("drvFX4: cannot parse the received frame");
            continue;
        };

        if event == "update" {
            let cfg = ctx.link.trigger_config();
            let actions = ctx.link.cache.lock().ingest(&data, &cfg);
            for action in actions {
                match action {
                    Fx4Action::Sample(values) => {
                        ctx.shared
                            .compute_positions(&ctx.handle, &ctx.params, &values);
                    }
                    Fx4Action::BulbTrigger => {
                        ctx.shared.trigger_callbacks();
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
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
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
    port_name: &str,
    fx4_ip: &str,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<Fx4Runtime> {
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

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
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

    let callback_thread = qe::start_callback_task(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
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
        log::warn!("drvFX4: timeout connecting to the FX4 at {uri}");
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
