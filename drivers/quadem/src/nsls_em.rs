//! Port of `quadEMApp/nslsSrc/drvNSLS_EM.{h,cpp}` — the NSLS Precision
//! Integrator, reached over Ethernet.
//!
//! Unlike the AHxxx and the TetrAMM, this driver builds its own transport: it
//! broadcasts a discovery datagram on UDP 37747, looks for the module with the
//! requested ID, and then opens a TCP command port (4747) and a TCP data port
//! (5757) to that module's address. The three sub-ports are created by the
//! driver and owned by [`NslsEmRuntime`]; they are not registered under names
//! in the asyn port registry, because nothing in the module's databases refers
//! to them (C++ names them `UDP_<port>`, `TCP_Command_<port>`, `TCP_Data_<port>`
//! only so `asynReport` can find them).
//!
//! Two C++ defects are not reproduced; both are noted at the site:
//!
//! * `readThread` (`drvNSLS_EM.cpp:376-384`) filters an untagged sample against
//!   an uninitialised `phase` local.
//! * `findModule` (`drvNSLS_EM.cpp:176-187`) writes past the end of the
//!   16-element `moduleInfo_` array when more modules answer the broadcast.

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QuadEmBase, QuadEmDevice,
    QuadEmParams, QuadEmShared, num_average_from,
};
use crate::nsls_em_proto::{self as proto, PingPong};
use crate::octet::OctetIo;

/// C++ `epicsThreadSleep(0.01)` handshake poll.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `epicsThreadSleep(1.0)` after an unexpected read error.
const READ_ERROR_BACKOFF: Duration = Duration::from_secs(1);
/// C++ `setIntegerParam(P_ValuesPerRead, 5)` in the constructor.
const DEFAULT_VALUES_PER_READ: i32 = 5;

fn is_timeout(e: &AsynError) -> bool {
    matches!(
        e,
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        }
    )
}

fn error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// What the read thread needs from the port actor: C++'s `scaleFactor_` member
/// and the ping-pong setting it latches when acquisition starts.
#[derive(Debug, Clone, Copy)]
struct NslsState {
    scale_factor: f64,
    ping_pong: PingPong,
}

type SharedState = Arc<parking_lot::Mutex<NslsState>>;

// ===========================================================================
// Transport
// ===========================================================================

/// The sockets C++ builds in its constructor. The discovery port is closed once
/// the module is found, as in C++, where nothing reads it again.
struct Transport {
    command: OctetIo,
    data: OctetIo,
    /// The runtimes of the two TCP sub-ports; dropping them closes the sockets,
    /// so [`NslsEmRuntime`] keeps them for the life of the port.
    runtimes: Vec<PortRuntimeHandle>,
}

/// Build an IP port the way `drvAsynIPPortConfigure` does, with the EOS
/// interpose installed unless the caller asks for raw datagram reads.
fn ip_port(
    port_name: &str,
    host_info: &str,
    no_auto_connect: bool,
    process_eos: bool,
) -> AsynResult<(OctetIo, PortRuntimeHandle)> {
    let mut driver = epics_rs::asyn::drivers::ip_port::DrvAsynIPPort::new(port_name, host_info)?;
    if no_auto_connect {
        driver.base_mut().auto_connect = false;
    }
    if process_eos {
        driver.push_interpose(Box::new(
            epics_rs::asyn::interpose::eos::EosInterpose::default(),
        ));
    }
    let (runtime, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let io = OctetIo::new(runtime.port_handle().clone(), proto::NSLS_EM_TIMEOUT);
    Ok((io, runtime))
}

/// C++ `drvNSLS_EM::findModule`: broadcast `i`, collect the replies for
/// `BROADCAST_TIMEOUT`, then open the two TCP ports of the requested module.
fn find_module(port_name: &str, broadcast_address: &str, module_id: i32) -> AsynResult<Transport> {
    // The discovery port carries no EOS: each reply is one datagram, and the
    // EOS interpose would keep reading past it until the read timed out.
    let (udp, udp_runtime) = ip_port(
        &format!("UDP_{port_name}"),
        &format!("{broadcast_address}:{} UDP*", proto::BROADCAST_PORT),
        false,
        false,
    )?;

    udp.write_bytes(proto::DISCOVER_COMMAND)?;

    let mut modules = Vec::new();
    let deadline = Instant::now() + proto::BROADCAST_TIMEOUT;
    let udp_reader = udp.with_timeout(proto::BROADCAST_READ_TIMEOUT);
    while Instant::now() < deadline {
        match udp_reader.read_binary(proto::DISCOVERY_BUFFER_SIZE) {
            Ok(outcome) if !outcome.data.is_empty() => {
                let text = String::from_utf8_lossy(&outcome.data);
                modules.extend(proto::parse_discovery(&text));
            }
            Ok(_) => {}
            Err(e) if is_timeout(&e) => {}
            Err(e) => return Err(e),
        }
    }
    drop(udp_runtime);

    let found = modules
        .iter()
        .find(|m| m.module_id == module_id)
        .ok_or_else(|| {
            error(format!(
                "drvNSLS_EM: module {module_id} did not answer the broadcast on \
                 {broadcast_address}; modules seen: {}",
                if modules.is_empty() {
                    "none".to_string()
                } else {
                    modules
                        .iter()
                        .map(|m| format!("{} at {}", m.module_id, m.ip))
                        .collect::<Vec<_>>()
                        .join(", ")
                }
            ))
        })?;

    // Command port: C++ passes noAutoConnect=1 and drives the connection
    // itself around every transaction.
    let (command, command_runtime) = ip_port(
        &format!("TCP_Command_{port_name}"),
        &format!("{}:{}", found.ip, proto::COMMAND_PORT),
        true,
        true,
    )?;
    command.set_input_eos(b"\r\n")?;
    command.handle().set_output_eos_blocking(b"\r")?;

    let (data, data_runtime) = ip_port(
        &format!("TCP_Data_{port_name}"),
        &format!("{}:{}", found.ip, proto::DATA_PORT),
        false,
        true,
    )?;
    data.set_input_eos(b"\n")?;

    Ok(Transport {
        command,
        data,
        runtimes: vec![command_runtime, data_runtime],
    })
}

// ===========================================================================
// Driver
// ===========================================================================

pub struct NslsEmDriver {
    base: QuadEmBase,
    command: OctetIo,
    shared: Arc<QuadEmShared>,
    state: SharedState,
}

impl NslsEmDriver {
    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    fn acquire_param(&self) -> usize {
        self.base.nd_params.acquire
    }

    /// C++ `writeReadMeter`.
    ///
    /// The meter answers a command that carries an argument only on the second
    /// write, so C++ writes those commands twice; the reply must start `OK>`.
    /// The connection is opened and closed around every transaction, as C++
    /// does with `pasynCommonSyncIO`.
    fn write_read_meter(&self, out: &str) -> AsynResult<String> {
        self.command.handle().connect_addr_blocking(0)?;
        let result = (|| -> AsynResult<String> {
            if out.len() > 1 {
                self.command.write(out)?;
            }
            let reply = self.command.write_read(out, proto::MAX_COMMAND_LEN)?;
            if !proto::is_ok(&reply) {
                log::error!("drvNSLS_EM: outString={out} expected OK>, received {reply}");
                return Err(error(format!("NSLS_EM: {out} -> {reply}, expected OK>")));
            }
            Ok(reply)
        })();
        let _ = self.command.handle().disconnect_addr_blocking(0);
        result
    }

    /// C++ `setMode`.
    fn set_mode(&mut self) -> AsynResult<()> {
        let p = self.params();
        let ping_pong = PingPong::from_i32(self.base.port_base.get_int32_param(p.ping_pong, 0)?);
        let values_per_read = self.base.port_base.get_int32_param(p.values_per_read, 0)?;

        let ping_pong = proto::effective_ping_pong(ping_pong, values_per_read);
        // C++ writes the forced value back to the parameter library.
        self.base
            .port_base
            .set_int32_param(p.ping_pong, 0, ping_pong as i32)?;
        self.state.lock().ping_pong = ping_pong;

        let mode = proto::mode_value(self.shared.is_acquiring(), ping_pong);
        // C++ discards this status.
        let _ = self.write_read_meter(&proto::cmd_mode(mode));
        Ok(())
    }

    /// C++ `computeScaleFactor`.
    fn compute_scale_factor(&mut self) -> AsynResult<()> {
        let p = self.params();
        let range = self.base.port_base.get_int32_param(p.range, 0)?;
        let values_per_read = self.base.port_base.get_int32_param(p.values_per_read, 0)?;
        let integration_time = self
            .base
            .port_base
            .get_float64_param(p.integration_time, 0)?;
        self.state.lock().scale_factor =
            proto::scale_factor(range, integration_time, values_per_read);
        Ok(())
    }

    /// C++ `readStatus`.
    fn read_status_inner(&mut self) -> AsynResult<()> {
        let reply = self.write_read_meter(proto::CMD_STATUS)?;
        let status = proto::parse_status(&reply).ok_or_else(|| {
            log::error!("drvNSLS_EM: cannot decode the status reply: {reply}");
            error(format!("NSLS_EM: bad status reply: {reply}"))
        })?;

        let p = self.params();
        let base = &mut self.base.port_base;
        base.set_int32_param(p.range, 0, status.range)?;
        base.set_int32_param(p.values_per_read, 0, status.values_per_read)?;

        let period = status.period_us / 1e6;
        base.set_float64_param(p.integration_time, 0, period)?;

        let ping_pong = PingPong::from_i32(base.get_int32_param(p.ping_pong, 0)?);
        let sample_time = proto::sample_time(period, status.values_per_read, ping_pong);
        base.set_float64_param(p.sample_time, 0, sample_time)?;
        base.set_string_param(p.firmware, 0, status.firmware)?;

        let averaging_time = base.get_float64_param(p.averaging_time, 0)?;
        let num_average = num_average_from(averaging_time, sample_time);
        base.set_int32_param(p.num_average, 0, num_average)?;

        {
            let mut acq = self.shared.acq.lock();
            acq.num_average = num_average;
            acq.values_per_read = status.values_per_read;
        }
        Ok(())
    }
}

// ===========================================================================
// drvQuadEM virtuals
// ===========================================================================

impl QuadEmDevice for NslsEmDriver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvNSLS_EM::setAcquire`.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 1 && self.shared.is_acquiring() {
            return Ok(());
        }
        if value == 0 {
            self.shared.set_acquiring(false);
            while self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
        } else {
            // The flag goes up before the signal so the read thread cannot see
            // a stale `acquiring == false` and go straight back to sleep; C++
            // orders it the other way because both sides hold the port lock.
            self.shared.set_acquiring(true);
            self.shared.acquire_start.signal();
        }
        self.base_set_acquire(value)?;
        self.set_mode()
    }

    /// C++ `drvNSLS_EM::setRange`: `"r %d"`.
    fn set_range(&mut self, value: i32) -> AsynResult<()> {
        let status = self.write_read_meter(&proto::cmd_range(value));
        self.compute_scale_factor()?;
        status.map(|_| ())
    }

    /// C++ `drvNSLS_EM::setValuesPerRead`: `"n %d"`, then the mode is re-sent
    /// because the phase tagging is only valid at one value per read.
    fn set_values_per_read(&mut self, value: i32) -> AsynResult<()> {
        // C++ discards this status.
        let _ = self.write_read_meter(&proto::cmd_values_per_read(value));
        self.compute_scale_factor()?;
        self.set_mode()
    }

    /// C++ `drvNSLS_EM::setIntegrationTime`: clamp, write back, `"p %d"` (µs).
    fn set_integration_time(&mut self, value: f64) -> AsynResult<()> {
        let clamped = proto::clamp_integration_time(value);
        if clamped != value {
            let idx = self.params().integration_time;
            self.base.port_base.set_float64_param(idx, 0, clamped)?;
        }
        let status = self.write_read_meter(&proto::cmd_integration_time(clamped));
        self.compute_scale_factor()?;
        status.map(|_| ())
    }

    /// C++ `drvNSLS_EM::setPingPong`: the setting only reaches the meter as
    /// part of the mode byte.
    fn set_ping_pong(&mut self, _value: i32) -> AsynResult<()> {
        self.set_mode()
    }

    /// C++ `drvNSLS_EM::readStatus`.
    fn read_status(&mut self) -> AsynResult<()> {
        self.read_status_inner()
    }
}

// ===========================================================================
// asyn port driver
// ===========================================================================

impl PortDriver for NslsEmDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvQuadEM::writeInt32` with the NSLS_EM overrides bound in.
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
            self.shared.acq.lock().acquire_mode = value;
            self.read_status()?;
        } else if reason == p.geometry {
            self.shared.pos.lock().geometry = value;
        } else if reason == p.range {
            // drvQuadEM::writeInt32 sends the per-channel range only when the
            // driver overrides it; drvNSLS_EM has a single meter-wide range.
            if channel == 0 {
                self.set_range(value)?;
            }
            self.read_status()?;
        } else if reason == p.values_per_read {
            self.shared.acq.lock().values_per_read = value;
            self.set_values_per_read(value)?;
            self.read_status()?;
        } else if reason == p.ping_pong {
            self.set_ping_pong(value)?;
            self.read_status()?;
        } else if reason == p.num_acquire {
            self.shared.acq.lock().num_acquire = value;
            self.read_status()?;
        } else if reason == p.read_data {
            self.shared.trigger_callbacks();
        } else if reason == p.read_status {
            if !self.shared.is_acquiring() {
                self.read_status()?;
            }
        } else if reason == p.reset {
            self.base_reset()?;
            self.read_status()?;
        } else if reason == p.trigger_mode
            || reason == p.trigger_polarity
            || reason == p.bias_state
            || reason == p.bias_interlock
            || reason == p.num_channels
            || reason == p.resolution
            || reason == p.read_format
        {
            // drvQuadEM dummies for this meter: it has no bias supply, no
            // external trigger, four fixed channels and one read format.
            self.read_status()?;
        } else if self.base.write_int32_pool(reason)? {
            // Handled by the NDArray pool controls.
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }

    /// C++ `drvQuadEM::writeFloat64` with the NSLS_EM overrides bound in.
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let channel = user.addr;
        let p = self.params();

        self.base
            .port_base
            .set_float64_param(reason, channel, value)?;
        self.cache_position_param(reason, channel, value);

        if reason == p.averaging_time {
            self.shared.ring.lock().flush();
            self.read_status()?;
        } else if reason == p.integration_time {
            self.set_integration_time(value)?;
            self.read_status()?;
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }
}

impl NslsEmDriver {
    /// Mirror a parameter write into the shared position snapshot the read
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
// Read thread
// ===========================================================================

struct ReadContext {
    data: OctetIo,
    handle: PortHandle,
    params: QuadEmParams,
    shared: Arc<QuadEmShared>,
    state: SharedState,
}

/// C++ `drvNSLS_EM::readThread`.
fn read_thread(ctx: ReadContext) {
    let mut ping_pong = PingPong::Both;

    loop {
        if !ctx.shared.is_acquiring() {
            ctx.shared.set_reading_active(false);
            ctx.shared.acquire_start.wait();
            ctx.shared.set_reading_active(true);
            let _ = ctx.data.flush();
            ping_pong = ctx.state.lock().ping_pong;
        }

        let outcome = match ctx.data.read_line(proto::DATA_BUFFER_SIZE) {
            Ok(o) => o,
            Err(e) => {
                if !is_timeout(&e) {
                    log::error!("drvNSLS_EM: unexpected error reading meter: {e}");
                    thread::sleep(READ_ERROR_BACKOFF);
                }
                continue;
            }
        };
        if !outcome.eom.contains(EomReason::EOS) {
            continue;
        }

        let Some(sample) = proto::parse_sample(&outcome.as_str()) else {
            log::error!("drvNSLS_EM: cannot decode the sample: {}", outcome.as_str());
            continue;
        };
        if !proto::sample_wanted(&sample, ping_pong) {
            continue;
        }

        let scale_factor = ctx.state.lock().scale_factor;
        let mut data = [0.0f64; QE_MAX_INPUTS];
        for (slot, raw) in data.iter_mut().zip(sample.raw) {
            *slot = raw as f64 * scale_factor;
        }
        ctx.shared
            .compute_positions(&ctx.handle, &ctx.params, &data);
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured NSLS_EM port.
pub struct NslsEmRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    /// The TCP command and data ports this driver created for itself.
    _sub_ports: Vec<PortRuntimeHandle>,
    _read_thread: thread::JoinHandle<()>,
    _callback_thread: thread::JoinHandle<()>,
}

impl NslsEmRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// C++ `drvNSLS_EMConfigure(portName, broadcastAddress, moduleID, ringBufferSize)`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`.
pub fn create_nsls_em(
    port_name: &str,
    broadcast_address: &str,
    module_id: i32,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<NslsEmRuntime> {
    let mut transport = find_module(port_name, broadcast_address, module_id)?;
    let sub_ports = std::mem::take(&mut transport.runtimes);
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);
    let state: SharedState = Arc::new(parking_lot::Mutex::new(NslsState {
        scale_factor: 0.0,
        ping_pong: PingPong::Both,
    }));

    let mut base = QuadEmBase::new(port_name, max_memory)?;
    base.port_base
        .set_int32_param(base.params.model, 0, QeModel::NslsEm as i32)?;
    base.port_base
        .set_int32_param(base.params.values_per_read, 0, DEFAULT_VALUES_PER_READ)?;
    shared.acq.lock().values_per_read = DEFAULT_VALUES_PER_READ;

    let mut driver = NslsEmDriver {
        base,
        command: transport.command,
        shared: shared.clone(),
        state: state.clone(),
    };
    // C++ leaves the meter untouched at construction: it may be offline, and
    // reset() is what the user runs once it is up. The scale factor still needs
    // a value the read thread can use before the first status read.
    driver.compute_scale_factor()?;

    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime_handle.port_handle().clone();

    let read_ctx = ReadContext {
        data: transport.data,
        handle: handle.clone(),
        params,
        shared: shared.clone(),
        state,
    };
    let read_thread_handle = thread::Builder::new()
        .name("drvNSLS_EMTask".into())
        .spawn(move || read_thread(read_ctx))
        .expect("failed to spawn drvNSLS_EMTask");

    let callback_thread = qe::start_callback_task(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
        shared,
        acquire_param,
    });

    Ok(NslsEmRuntime {
        runtime_handle,
        params,
        nd_params,
        pool,
        outputs,
        _sub_ports: sub_ports,
        _read_thread: read_thread_handle,
        _callback_thread: callback_thread,
    })
}
