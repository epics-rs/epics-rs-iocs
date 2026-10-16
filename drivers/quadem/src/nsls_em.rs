//! Port of `quadEMApp/nslsSrc/drvNSLS_EM.{h,cpp}` — the NSLS Precision
//! Integrator, reached over Ethernet.
//!
//! Unlike the AHxxx and the TetrAMM, this driver builds its own transport: it
//! broadcasts a discovery datagram on UDP 37747, looks for the module with the
//! requested ID, and then opens a TCP command port (4747) and a TCP data port
//! (5757) to that module's address. The three sub-ports are created by the
//! driver and owned by [`NslsEmRuntime`], and registered as `UDP_<port>`,
//! `TCP_Command_<port>` and `TCP_Data_<port>`, the names C++'s `NSLS_EM.cmd`
//! traces them by.
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
    self as qe, CallbackContext, QE_MAX_INPUTS, QeModel, QuadEmBase, QuadEmDevice, QuadEmParams,
    QuadEmShared, num_average_from,
};
use crate::nsls_em_proto::{self as proto, PingPong};
use crate::octet::{OctetIo, create_ip_port};
use crate::{asyn_print, trace};
use epics_rs::base::calc::engine::cvt::fmt_e;

/// C++ `epicsThreadSleep(0.01)` handshake poll.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `epicsThreadSleep(1.0)` after an unexpected read error.
const READ_ERROR_BACKOFF: Duration = Duration::from_secs(1);
/// C++ `setIntegerParam(P_ValuesPerRead, 5)` in the constructor.
const DEFAULT_VALUES_PER_READ: i32 = 5;

/// True for a timeout, including one `AsynError::PartialRead` wraps to carry
/// the bytes transferred before it. A bare `AsynError::Status` variant match
/// would miss that wrapper and misclassify every partial-transfer timeout —
/// routine on this port, since it installs the EOS interpose — as an
/// unexpected error.
fn is_timeout(e: &AsynError) -> bool {
    e.status() == AsynStatus::Timeout
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

/// What C++'s constructor leaves behind: the modules that answered the
/// broadcast, and the command/data sockets when the requested one was found.
struct Transport {
    /// `None` when the constructor returned early. C++ keeps the port then,
    /// with no meter link and no read thread.
    link: Option<(OctetIo, OctetIo)>,
    /// The UDP, command and data sub-ports; dropping them closes the sockets,
    /// so [`NslsEmRuntime`] keeps them for the life of the port.
    runtimes: Vec<PortRuntimeHandle>,
    /// C++ `moduleInfo_`: every module that answered the broadcast.
    modules: Vec<proto::ModuleInfo>,
    /// `findModule` failed, so C++ ran `report(stdout, 1)`.
    find_failed: bool,
}

/// Build an IP port the way `drvAsynIPPortConfigure` does, with the EOS
/// interpose installed unless the caller asks for raw datagram reads.
fn ip_port(
    port_name: &str,
    host_info: &str,
    no_auto_connect: bool,
    process_eos: bool,
) -> AsynResult<(OctetIo, PortRuntimeHandle)> {
    create_ip_port(
        port_name,
        host_info,
        proto::NSLS_EM_TIMEOUT,
        no_auto_connect,
        process_eos,
    )
}

/// C++ constructor from the UDP port on, then `findModule`. Every failure is
/// printed as C++ prints it and leaves `link` empty; none fails the port.
fn find_module(
    base: &PortDriverBase,
    port_name: &str,
    broadcast_address: &str,
    module_id: i32,
) -> Transport {
    let mut transport = Transport {
        link: None,
        runtimes: Vec::new(),
        modules: Vec::new(),
        find_failed: false,
    };
    // The discovery port carries no EOS: each reply is one datagram, and the
    // EOS interpose would keep reading past it until the read timed out.
    let udp_name = format!("UDP_{port_name}");
    let udp_host = format!("{broadcast_address}:{} UDP*", proto::BROADCAST_PORT);
    let udp = match ip_port(&udp_name, &udp_host, false, false) {
        Ok((udp, runtime)) => {
            transport.runtimes.push(runtime);
            udp
        }
        Err(e) => {
            asyn_print!(
                *base,
                ERROR,
                "drvNSLS_EM::drvNSLS_EM error calling drvAsynIPPortConfigure for broadcast port={udp_name}, IP={udp_host}, status={}",
                trace::status_code(&e)
            );
            return transport;
        }
    };

    match discover(base, &udp, port_name, module_id, &mut transport) {
        Ok(link) => transport.link = Some(link),
        Err(()) => {
            asyn_print!(
                *base,
                ERROR,
                "drvNSLS_EM::drvNSLS_EM error finding module, running asynReport to list modules found"
            );
            transport.find_failed = true;
        }
    }
    transport
}

/// C++ `drvNSLS_EM::findModule`: broadcast `i`, collect the replies for
/// `BROADCAST_TIMEOUT`, then open the two TCP ports of the requested module.
fn discover(
    base: &PortDriverBase,
    udp: &OctetIo,
    port_name: &str,
    module_id: i32,
    transport: &mut Transport,
) -> Result<(OctetIo, OctetIo), ()> {
    // C++ discards the write status.
    let _ = udp.write_bytes(proto::DISCOVER_COMMAND);

    let deadline = Instant::now() + proto::BROADCAST_TIMEOUT;
    let udp_reader = udp.with_timeout(proto::BROADCAST_READ_TIMEOUT);
    while Instant::now() < deadline {
        match udp_reader.read_binary(proto::DISCOVERY_BUFFER_SIZE) {
            Ok(outcome) if !outcome.data.is_empty() => {
                let text = String::from_utf8_lossy(&outcome.data);
                transport.modules.extend(proto::parse_discovery(&text));
            }
            Ok(_) => {}
            Err(e) if is_timeout(&e) => {}
            Err(_) => return Err(()),
        }
    }

    let Some(found) = transport.modules.iter().find(|m| m.module_id == module_id) else {
        asyn_print!(
            *base,
            ERROR,
            "drvNSLS_EM:findModules: cannot find requested module {module_id} on network"
        );
        return Err(());
    };
    let ip = found.ip.clone();

    let tcp_port = |name: String, port: u16, no_auto_connect: bool| {
        ip_port(&name, &format!("{ip}:{port}"), no_auto_connect, true).map_err(|e| {
            asyn_print!(
                *base,
                ERROR,
                "drvNSLS_EM:findModules: error calling drvAsyIPPortConfigure for TCP port {name}, IP={ip}, status={}",
                trace::status_code(&e)
            );
        })
    };

    // Command port: C++ passes noAutoConnect=1 and drives the connection
    // itself around every transaction.
    let (command, command_runtime) = tcp_port(
        format!("TCP_Command_{port_name}"),
        proto::COMMAND_PORT,
        true,
    )?;
    transport.runtimes.push(command_runtime);
    let _ = command.set_input_eos(b"\r\n");
    let _ = command.set_output_eos(b"\r");

    let (data, data_runtime) = tcp_port(format!("TCP_Data_{port_name}"), proto::DATA_PORT, false)?;
    transport.runtimes.push(data_runtime);
    let _ = data.set_input_eos(b"\n");

    Ok((command, data))
}

// ===========================================================================
// Driver
// ===========================================================================

pub struct NslsEmDriver {
    base: QuadEmBase,
    /// `None` when the module was not found: C++ keeps the port without one.
    command: Option<OctetIo>,
    shared: Arc<QuadEmShared>,
    state: SharedState,
    /// C++ `ipAddress_`, `moduleID_`, `firmwareVersion_` and `moduleInfo_`,
    /// printed by [`PortDriver::report`].
    ip_address: String,
    module_id: i32,
    firmware_version: String,
    modules: Vec<proto::ModuleInfo>,
}

impl NslsEmDriver {
    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    /// C++ `writeReadMeter`.
    ///
    /// The meter answers a command that carries an argument only on the second
    /// write, so C++ writes those commands twice; the reply must start `OK>`.
    /// The connection is opened and closed around every transaction, as C++
    /// does with `pasynCommonSyncIO`.
    fn write_read_meter(&self, out: &str) -> AsynResult<String> {
        // C++ dereferences a NULL asynUser here and crashes.
        let Some(command) = &self.command else {
            return Err(error("drvNSLS_EM: no module on the network"));
        };
        // C++ discards the connect and first-write statuses; the writeRead
        // that follows reports the failure.
        let _ = command.handle().connect_blocking();
        let result = (|| -> AsynResult<String> {
            if out.len() > 1 {
                let _ = command.write(out);
            }
            let reply = match command.write_read(out, proto::MAX_COMMAND_LEN) {
                Ok(reply) => reply,
                Err(e) => {
                    let partial = e.partial_read();
                    let n_read = partial.map_or(0, |p| p.data.len());
                    let eom = partial.map_or(0, |p| p.eom_reason.bits());
                    let in_string = partial
                        .map(|p| String::from_utf8_lossy(&p.data).into_owned())
                        .unwrap_or_default();
                    asyn_print!(
                        self.base.port_base,
                        ERROR,
                        "drvNSLS_EM:writeReadMeter: error calling writeRead, outString={out} status={}, nread={n_read}, eomReason={eom}, inString={in_string}",
                        trace::status_code(&e)
                    );
                    return Err(e);
                }
            };
            if !proto::is_ok(&reply) {
                asyn_print!(
                    self.base.port_base,
                    ERROR,
                    "drvNSLS_EM:writeReadMeter: error, outString={out} expected OK>, received {reply}"
                );
                return Err(error(format!("NSLS_EM: {out} -> {reply}, expected OK>")));
            }
            Ok(reply)
        })();
        let _ = command.handle().disconnect_blocking();
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
        let (items, s) = proto::scan_status(&reply);
        // C++'s sscanf stores each item it assigns, on failure too.
        if items >= 1 {
            self.ip_address.clone_from(&s.ip);
        }
        if items >= 2 {
            self.module_id = s.module_id;
        }
        if items >= 3 {
            self.firmware_version.clone_from(&s.firmware);
        }
        let status = match (items, s) {
            (7, status) => status,
            (items, s) => {
                asyn_print!(
                    self.base.port_base,
                    ERROR,
                    "drvNSLS_EM::getStatus error, numItems={items}, ipAddress={}, moduleID={}, firmwareVersion={}, mode={}, valuesPerRead={}, range={}, period={:.6}",
                    s.ip,
                    s.module_id,
                    s.firmware,
                    s.mode,
                    s.values_per_read,
                    s.range,
                    s.period_us
                );
                return Err(error(format!("NSLS_EM: bad status reply: {reply}")));
            }
        };

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
        let _ = self.base_set_acquire(value);
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

    /// C++ `drvNSLS_EM::reset` is the base class's.
    fn reset(&mut self) -> AsynResult<()> {
        self.base_reset()
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

    /// C++ `drvNSLS_EM::report`, which ends in the base report.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(out, "drvNSLS_EM: port={}", self.base.port_base.port_name);
        if level > 0 {
            let scale_factor = self.state.lock().scale_factor;
            let _ = writeln!(out, "  IP address:       {}", self.ip_address);
            let _ = writeln!(out, "  Module ID:        {}", self.module_id);
            let _ = writeln!(out, "  Firmware version: {}", self.firmware_version);
            let _ = writeln!(out, "  Scale factor:     {}", fmt_e(scale_factor, 6, false));
        }
        let _ = writeln!(
            out,
            "  Number of modules found on network={}",
            self.modules.len()
        );
        for (i, m) in self.modules.iter().enumerate() {
            let _ = writeln!(out, "    Module {i}");
            let _ = writeln!(out, "      Module ID:  {}", m.module_id);
            let _ = writeln!(out, "      IP address: {}", m.ip);
        }
        self.asyn_port_driver_report(out, level);
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.qe_write_int32(user.reason, user.addr, value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.qe_write_float64(user.reason, user.addr, value)
    }
}

impl NslsEmDriver {}

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
            let _ = ctx.data.locked(|port| port.flush());
            ping_pong = ctx.state.lock().ping_pong;
        }

        let read = ctx.data.locked_read_line(proto::DATA_BUFFER_SIZE);
        let outcome = match read {
            Ok(o) if o.eom.contains(EomReason::EOS) => o,
            _ => {
                if trace::read_result(&read).0 != AsynStatus::Timeout {
                    trace::print_read_error(&ctx.handle, "drvNSLS_EM:readThread", &read);
                    thread::sleep(READ_ERROR_BACKOFF);
                }
                continue;
            }
        };

        // C++ prints nothing for a line it cannot scan.
        let Some(sample) = proto::parse_sample(&outcome.as_str()) else {
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
    /// The `UDP_`, `TCP_Command_` and `TCP_Data_` ports this driver created for
    /// itself; C++ registers them by name, so `st.cmd` can trace them.
    pub sub_ports: Vec<PortRuntimeHandle>,
    _read_thread: Option<thread::JoinHandle<()>>,
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
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);
    let state: SharedState = Arc::new(parking_lot::Mutex::new(NslsState {
        scale_factor: 0.0,
        ping_pong: PingPong::Both,
    }));

    let mut base = QuadEmBase::new(port_name, max_memory)?;
    let transport = find_module(&base.port_base, port_name, broadcast_address, module_id);
    let (command, data) = transport.link.unzip();
    if command.is_some() {
        base.port_base
            .set_int32_param(base.params.model, 0, QeModel::NslsEm as i32)?;
        base.port_base
            .set_int32_param(base.params.values_per_read, 0, DEFAULT_VALUES_PER_READ)?;
        shared.acq.lock().values_per_read = DEFAULT_VALUES_PER_READ;
    }

    let mut driver = NslsEmDriver {
        base,
        command,
        shared: shared.clone(),
        state: state.clone(),
        ip_address: String::new(),
        module_id,
        firmware_version: String::new(),
        modules: transport.modules,
    };
    let sub_ports = transport.runtimes;
    if driver.command.is_some() {
        // C++ leaves the meter untouched at construction: it may be offline,
        // and reset() is what the user runs once it is up. The scale factor
        // still needs a value the read thread can use before the first status
        // read.
        driver.compute_scale_factor()?;
    }
    if transport.find_failed {
        // C++ `report(stdout, 1)` right after "error finding module".
        let mut out = String::new();
        driver.report(&mut out, 1);
        print!("{out}");
    }

    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default())?;
    let handle = runtime_handle.port_handle().clone();

    // C++ creates the read thread only once the module is found.
    let read_thread_handle = data.map(|data| {
        let read_ctx = ReadContext {
            data,
            handle: handle.clone(),
            params,
            shared: shared.clone(),
            state,
        };
        thread::Builder::new()
            .name("drvNSLS_EMTask".into())
            .spawn(move || read_thread(read_ctx))
            .expect("failed to spawn drvNSLS_EMTask")
    });

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
        sub_ports,
        _read_thread: read_thread_handle,
        _callback_thread: callback_thread,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interpose::PartialOctetRead;

    /// `AsynError::PartialRead` is how `asyn-rs` 0.24+ carries a timed-out
    /// read's transferred bytes (see the fix note on `is_timeout`); a bare
    /// `AsynError::Status` match would miss it entirely. Routine on this
    /// port's TCP command/data sub-ports, which install the EOS interpose.
    #[test]
    fn is_timeout_recognizes_a_partial_read_wrapped_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(PartialOctetRead {
            data: b"12.3".to_vec(),
            eom_reason: EomReason::empty(),
        });
        assert!(is_timeout(&e));
        assert_eq!(
            e.partial_read().map(|p| p.data.as_slice()),
            Some(&b"12.3"[..])
        );
    }

    #[test]
    fn report_prints_cs_block_then_the_base_report() {
        let (command, runtime) = create_ip_port(
            "QE_NSLS_REPORT_IP",
            "127.0.0.1:1",
            Duration::from_millis(50),
            true,
            true,
        )
        .unwrap();
        std::mem::forget(runtime);
        let mut drv = NslsEmDriver {
            base: QuadEmBase::new("QE_NSLS_REPORT", 0).unwrap(),
            command: Some(command),
            shared: QuadEmShared::new(16).0,
            state: Arc::new(parking_lot::Mutex::new(NslsState {
                scale_factor: 5.722e-15,
                ping_pong: PingPong::Both,
            })),
            ip_address: "10.0.0.5".into(),
            module_id: 3,
            firmware_version: "1.23".into(),
            modules: vec![proto::ModuleInfo {
                module_id: 3,
                ip: "10.0.0.5".into(),
            }],
        };
        let block = "  Number of modules found on network=1\n    Module 0\n      \
                     Module ID:  3\n      IP address: 10.0.0.5\n";
        let mut out = String::new();
        drv.report(&mut out, 0);
        assert_eq!(
            out,
            format!("drvNSLS_EM: port=QE_NSLS_REPORT\n{block}Port: QE_NSLS_REPORT\n")
        );
        let mut out = String::new();
        drv.report(&mut out, 1);
        assert!(
            out.starts_with(&format!(
                "drvNSLS_EM: port=QE_NSLS_REPORT\n  IP address:       10.0.0.5\n  \
                 Module ID:        3\n  Firmware version: 1.23\n  \
                 Scale factor:     5.722000e-15\n{block}Port: QE_NSLS_REPORT\n"
            )),
            "{out}"
        );
    }

    /// C++ keeps the port when no module answers: no link, no read thread,
    /// and the constructor runs `report(stdout, 1)`.
    #[test]
    fn a_missing_module_leaves_the_port_without_a_link() {
        let base = QuadEmBase::new("QE_NSLS_NF", 0).unwrap();
        let transport = find_module(&base.port_base, "QE_NSLS_NF", "127.0.0.3", 9);
        assert!(transport.link.is_none());
        assert!(transport.find_failed);
        assert!(transport.modules.is_empty());
    }

    #[test]
    fn is_timeout_rejects_a_real_error() {
        assert!(!is_timeout(&error("boom")));
    }
}
