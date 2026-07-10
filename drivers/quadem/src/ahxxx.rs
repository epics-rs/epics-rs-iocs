//! Port of `quadEMApp/caenSrc/drvAHxxx.{h,cpp}` — Elettra/CaenEls AHxxx
//! 4-channel picoammeters, reached over TCP or serial through an asyn octet
//! port.
//!
//! Ported models: AH401B, AH401D. The AH501 series shares this driver
//! upstream but uses a different data encoding and status block; it is not
//! ported yet, and [`create_ahxxx`] rejects its model names.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::ahxxx_proto as proto;
use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeReadFormat,
    QeTriggerMode, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::octet::{OctetIo, connect_octet};

/// Poll interval of C++'s `epicsThreadSleep(0.01)` handshake loop.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `epicsThreadSleep(1.0)` after an unexpected read error.
const READ_ERROR_BACKOFF: Duration = Duration::from_secs(1);

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

// ===========================================================================
// Driver
// ===========================================================================

pub struct AhxxxDriver {
    base: QuadEmBase,
    io: OctetIo,
    shared: Arc<QuadEmShared>,
    model: QeModel,
    /// C++ `AH401Series_`, set by `reset()` once the model is known.
    ah401_series: bool,
    firmware_version: String,
}

impl AhxxxDriver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        max_memory: usize,
        model: QeModel,
        shared: Arc<QuadEmShared>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let io = connect_octet(qe_port_name, proto::AHXXX_TIMEOUT)?;
        // C++ sets the input EOS in setAcquire; it is constant for the port's
        // whole life, so it is set once here. See `octet`.
        io.set_input_eos(b"\r\n")?;

        // C++ constructor: resolution_ = 24, model from the modelName argument.
        base.port_base
            .set_int32_param(base.params.model, 0, model as i32)?;
        {
            let mut acq = shared.acq.lock();
            acq.resolution = 24;
            acq.num_channels = 4;
        }

        let mut this = Self {
            base,
            io,
            shared,
            model,
            ah401_series: false,
            firmware_version: String::new(),
        };

        // C++ constructor calls reset() under the port lock. The meter may be
        // offline at IOC start, so a failure is logged, not fatal.
        if let Err(e) = this.reset() {
            log::warn!("drvAHxxx: reset at startup failed: {e}");
        }
        Ok(this)
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    fn acquire_param(&self) -> usize {
        self.base.nd_params.acquire
    }

    // --- I/O helpers (C++ writeReadMeter / sendCommand) ---

    fn write_read_meter(&self, out: &str) -> AsynResult<String> {
        self.io.write_read(out, proto::MAX_COMMAND_LEN)
    }

    /// C++ `sendCommand`: stop acquisition around the command, require an
    /// acknowledgement, then restart.
    ///
    /// The AH501BE answers `AK` rather than `ACK` to `HVS`; upstream accepts
    /// both for every command, so this does too.
    fn send_command(&mut self, out: &str) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            self.set_acquire(0)?;
        }
        let resp = self.write_read_meter(out)?;
        if resp != "ACK" && resp != "AK" {
            log::error!("drvAHxxx: outString={out} expected ACK, received {resp}");
            return Err(error(format!("AHxxx: {out} -> {resp}, expected ACK")));
        }
        if prev_acquiring {
            self.set_acquire(1)?;
        }
        Ok(())
    }

    /// C++ `drvAHxxx::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        self.set_acquire(0)?;
        self.firmware_version = "Unknown".into();
        if let Ok(resp) = self.write_read_meter("VER ?") {
            self.firmware_version = resp;
        }
        let p = self.params();
        self.base
            .port_base
            .set_string_param(p.firmware, 0, self.firmware_version.clone())?;

        if self.model == QeModel::Unknown {
            match proto::model_from_firmware(&self.firmware_version) {
                Some(m) => {
                    self.model = m;
                    self.base.port_base.set_int32_param(p.model, 0, m as i32)?;
                }
                None => {
                    log::error!(
                        "drvAHxxx: unknown firmware version = {}",
                        self.firmware_version
                    );
                    return Err(error(format!(
                        "AHxxx: unknown firmware version {}",
                        self.firmware_version
                    )));
                }
            }
        }
        self.ah401_series = matches!(self.model, QeModel::Ah401b | QeModel::Ah401d);
        if !self.ah401_series {
            return Err(error(format!(
                "AHxxx: model {:?} is not ported (AH501 series)",
                self.model
            )));
        }
        self.base_reset()
    }

    /// C++ `drvAHxxx::readStatus`, AH401 branch.
    ///
    /// As in `drvTetrAMM`, the `goto error` path skips the `setAcquire(1)`
    /// restore, so a status read that fails leaves the meter stopped.
    fn read_status_inner(&mut self) -> AsynResult<()> {
        let p = self.params();
        let bad = |what: &str, resp: &str| error(format!("AHxxx: bad {what} response: {resp}"));

        let resp = self.write_read_meter("RNG ?")?;
        let range = proto::parse_range(&resp).ok_or_else(|| bad("RNG", &resp))?;
        self.base.port_base.set_int32_param(p.range, 0, range)?;

        let mut sample_time = 0.0;
        if self.ah401_series {
            let resp = self.write_read_meter("HLF ?")?;
            let ping_pong = proto::parse_hlf(&resp).ok_or_else(|| bad("HLF", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.ping_pong, 0, ping_pong)?;

            let resp = self.write_read_meter("ITM ?")?;
            let integration_time = proto::parse_itm(&resp).ok_or_else(|| bad("ITM", &resp))?;
            self.base
                .port_base
                .set_float64_param(p.integration_time, 0, integration_time)?;
            sample_time = proto::sample_time_ah401(ping_pong, integration_time);
        }

        // The sample times computed above don't include valuesPerRead.
        let values_per_read = self.shared.acq.lock().values_per_read;
        sample_time *= values_per_read as f64;
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        let averaging_time = self.base.port_base.get_float64_param(p.averaging_time, 0)?;
        let num_average = num_average_from(averaging_time, sample_time);
        self.base
            .port_base
            .set_int32_param(p.num_average, 0, num_average)?;
        self.shared.acq.lock().num_average = num_average;
        Ok(())
    }
}

// ===========================================================================
// drvQuadEM virtuals
// ===========================================================================

impl QuadEmDevice for AhxxxDriver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvAHxxx::setAcquire`.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        let p = self.params();
        let trigger_mode =
            QeTriggerMode::from_i32(self.base.port_base.get_int32_param(p.trigger_mode, 0)?);
        let acquire_mode =
            QeAcquireMode::from_i32(self.base.port_base.get_int32_param(p.acquire_mode, 0)?);
        let num_average = self.base.port_base.get_int32_param(p.num_average, 0)?;
        let read_format =
            QeReadFormat::from_i32(self.base.port_base.get_int32_param(p.read_format, 0)?);

        if value == 0 {
            // A one-shot acquisition that already finished stopped the meter
            // itself; nothing left to do.
            if !self.shared.is_acquiring()
                && !self.shared.is_reading_active()
                && acquire_mode == QeAcquireMode::Single
            {
                return Ok(());
            }

            self.shared.set_acquiring(false);
            while self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
            loop {
                // Both meter families are stopped: which one is on the far end
                // is not known until the firmware string has been read.
                let _ = self.write_read_meter("ACQ OFF");
                let _ = self.write_read_meter("S");
                // TRG OFF because the meter's mode at startup is unknown.
                // C++ tests only this last command's status and gives up on
                // any failure, a timeout included.
                if let Err(e) = self.write_read_meter("TRG OFF") {
                    log::error!("drvAHxxx: error calling writeRead: {e}");
                    break;
                }
                // Flush, then read with a short timeout to drain any responses.
                let _ = self.io.flush();
                match self.io.read_line(proto::MAX_COMMAND_LEN) {
                    Ok(_) => {}
                    Err(e) if is_timeout(&e) => break,
                    Err(e) => {
                        log::error!("drvAHxxx: error draining meter responses: {e}");
                        break;
                    }
                }
            }
            self.base_set_acquire(0)?;
            return Ok(());
        }

        self.base_set_acquire(1)?;

        let result = (|| -> AsynResult<()> {
            // Put the device in the appropriate mode.
            self.write_read_meter(proto::cmd_read_format(read_format))?;

            // In one-shot mode ask the meter for a specific number of samples.
            let num_acquire = proto::naq_value(acquire_mode, num_average);
            self.write_read_meter(&proto::cmd_naq(num_acquire))?;

            if trigger_mode == QeTriggerMode::ExtTrigger || trigger_mode == QeTriggerMode::ExtGate {
                // External trigger mode: the meter waits for the trigger.
                self.write_read_meter("TRG ON")?;
            } else {
                // The AH401 series echoes an ACK after ACQ ON.
                self.write_read_meter("ACQ ON")?;
            }
            Ok(())
        })();

        {
            let mut acq = self.shared.acq.lock();
            acq.acquire_mode = acquire_mode as i32;
            acq.trigger_mode = trigger_mode as i32;
            acq.read_format = read_format as i32;
        }

        if let Err(e) = result {
            // C++ `if (status) acquiring_ = 0;`
            self.shared.set_acquiring(false);
            return Err(e);
        }

        // Setting the flag before the signal keeps the read thread from going
        // straight back to sleep; C++ orders it the other way because both
        // sides run under the port lock.
        self.shared.set_acquiring(true);
        self.shared.acquire_start.signal();
        Ok(())
    }

    /// C++ `drvAHxxx::setRange`: `"RNG %d"`.
    fn set_range(&mut self, value: i32) -> AsynResult<()> {
        self.send_command(&proto::cmd_range(value))
    }

    /// C++ `drvAHxxx::setPingPong`: `"HLF ON"`/`"HLF OFF"`.
    fn set_ping_pong(&mut self, value: i32) -> AsynResult<()> {
        self.send_command(proto::cmd_ping_pong(value))
    }

    /// C++ `drvAHxxx::setIntegrationTime`: clamp, write back, `"ITM %d"`.
    fn set_integration_time(&mut self, value: f64) -> AsynResult<()> {
        let (clamped, cmd) = proto::cmd_integration_time(value);
        if clamped != value {
            let idx = self.params().integration_time;
            self.base.port_base.set_float64_param(idx, 0, clamped)?;
        }
        self.send_command(&cmd)
    }

    /// C++ `drvAHxxx::setReadFormat`: `"BIN ON"`/`"BIN OFF"`.
    fn set_read_format(&mut self, value: i32) -> AsynResult<()> {
        self.send_command(proto::cmd_read_format(QeReadFormat::from_i32(value)))
    }

    /// C++ `drvAHxxx::readStatus`.
    fn read_status(&mut self) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            self.set_acquire(0)?;
        }
        if let Err(e) = self.read_status_inner() {
            log::error!("drvAHxxx: readStatus failed: {e}");
            return Err(e);
        }
        if prev_acquiring {
            self.set_acquire(1)?;
        }
        Ok(())
    }

    // setNumChannels, setBiasState, setBiasVoltage and setResolution all
    // return asynSuccess without touching the meter on the AH401 series, which
    // is what the trait defaults do.
}

// ===========================================================================
// asyn port driver
// ===========================================================================

impl PortDriver for AhxxxDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvQuadEM::writeInt32` with the AHxxx overrides bound in.
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
        } else if reason == p.bias_state
            || reason == p.bias_interlock
            || reason == p.num_channels
            || reason == p.resolution
            || reason == p.trigger_mode
            || reason == p.trigger_polarity
        {
            // setBiasState, setBiasInterlock, setNumChannels and setResolution
            // return asynSuccess without touching an AH401; setTriggerMode and
            // setTriggerPolarity are drvQuadEM dummies. Only readStatus runs.
            self.read_status()?;
        } else if reason == p.num_acquire {
            self.shared.acq.lock().num_acquire = value;
            self.read_status()?;
        } else if reason == p.ping_pong {
            self.set_ping_pong(value)?;
            self.read_status()?;
        } else if reason == p.range {
            // drvQuadEM::writeInt32 calls the per-channel setRange for a
            // non-zero address; drvAHxxx does not override it, so only the
            // whole-meter range reaches the wire.
            if channel == 0 {
                self.set_range(value)?;
            }
            self.read_status()?;
        } else if reason == p.read_data {
            self.shared.trigger_callbacks();
        } else if reason == p.values_per_read {
            self.shared.acq.lock().values_per_read = value;
            self.read_status()?;
        } else if reason == p.read_format {
            self.set_read_format(value)?;
            self.read_status()?;
        } else if reason == p.read_status {
            // C++ skips this while acquiring: too disruptive.
            if !self.shared.is_acquiring() {
                self.read_status()?;
            }
        } else if reason == p.reset {
            self.reset()?;
            self.read_status()?;
        } else if self.base.write_int32_pool(reason)? {
            // Handled by the NDArray pool controls.
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }

    /// C++ `drvQuadEM::writeFloat64` with the AHxxx overrides bound in.
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
        } else if reason == p.bias_voltage {
            // drvAHxxx::setBiasVoltage is a no-op on the AH401 series.
            self.read_status()?;
        } else if reason == p.integration_time {
            self.set_integration_time(value)?;
            self.read_status()?;
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }
}

impl AhxxxDriver {
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
    io: OctetIo,
    handle: PortHandle,
    params: QuadEmParams,
    shared: Arc<QuadEmShared>,
}

/// C++ `drvAHxxx::readThread`, AH401 series.
fn read_thread(ctx: ReadContext) {
    let mut read_format = QeReadFormat::Binary;

    loop {
        if !ctx.shared.is_acquiring() {
            ctx.shared.set_reading_active(false);
            ctx.shared.acquire_start.wait();
            read_format = QeReadFormat::from_i32(ctx.shared.acq.lock().read_format);
            ctx.shared.set_reading_active(true);
        }

        // C++ `if (valuesPerRead_ < 1) valuesPerRead_ = 1;` — the clamp is
        // written back to the driver's member, not to the parameter library.
        let values_per_read = {
            let mut acq = ctx.shared.acq.lock();
            if acq.values_per_read < 1 {
                acq.values_per_read = 1;
            }
            acq.values_per_read
        };

        let mut raw = [0.0f64; QE_MAX_INPUTS];

        match read_format {
            QeReadFormat::Binary => {
                // The AH401 series is fixed at 4 channels of 3 bytes.
                let n_requested = proto::ah401_read_len(values_per_read as usize);
                let outcome = match ctx.io.read_binary(n_requested) {
                    Ok(o) => o,
                    Err(e) => {
                        if !is_timeout(&e) {
                            log::error!("drvAHxxx: unexpected error reading meter: {e}");
                            thread::sleep(READ_ERROR_BACKOFF);
                        }
                        continue;
                    }
                };
                if outcome.data.len() != n_requested || !outcome.eom.contains(EomReason::CNT) {
                    log::error!(
                        "drvAHxxx: unexpected error reading meter, nRead={} expected {n_requested}, eom={:?}",
                        outcome.data.len(),
                        outcome.eom
                    );
                    thread::sleep(READ_ERROR_BACKOFF);
                    continue;
                }
                let Some(values) =
                    proto::accumulate_binary_ah401(&outcome.data, values_per_read as usize)
                else {
                    continue;
                };
                raw = values;
            }
            QeReadFormat::Ascii => {
                for _ in 0..values_per_read {
                    let outcome = match ctx.io.read_line(proto::ASCII_BUFFER_SIZE) {
                        Ok(o) => o,
                        Err(e) => {
                            if !is_timeout(&e) {
                                log::error!("drvAHxxx: unexpected error reading meter: {e}");
                                thread::sleep(READ_ERROR_BACKOFF);
                            }
                            continue;
                        }
                    };
                    if !outcome.eom.contains(EomReason::EOS) {
                        continue;
                    }
                    let values =
                        proto::parse_ascii_ah401(&outcome.as_str(), proto::AH401_NUM_CHANNELS);
                    for (acc, v) in raw.iter_mut().zip(values) {
                        *acc += v;
                    }
                }
            }
        }

        proto::average_over_values_per_read(&mut raw, proto::AH401_NUM_CHANNELS, values_per_read);
        ctx.shared.compute_positions(&ctx.handle, &ctx.params, &raw);
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured AHxxx port.
pub struct AhxxxRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    _read_thread: thread::JoinHandle<()>,
    _callback_thread: thread::JoinHandle<()>,
}

impl AhxxxRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// C++ `drvAHxxxConfigure(portName, QEPortName, ringBufferSize, modelName)`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`. `model_name` must name a ported model: `AH401B` or
/// `AH401D`.
pub fn create_ahxxx(
    port_name: &str,
    qe_port_name: &str,
    ring_buffer_size: usize,
    max_memory: usize,
    model_name: &str,
) -> AsynResult<AhxxxRuntime> {
    let model = proto::model_from_name(model_name);
    if model == QeModel::Unknown {
        return Err(error(format!(
            "drvAHxxxConfigure: model '{model_name}' is not ported; use AH401B or AH401D"
        )));
    }

    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);

    let driver = AhxxxDriver::new(port_name, qe_port_name, max_memory, model, shared.clone())?;
    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let io = driver.io.clone();
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());
    let handle = runtime_handle.port_handle().clone();

    let read_ctx = ReadContext {
        io,
        handle: handle.clone(),
        params,
        shared: shared.clone(),
    };
    let read_thread_handle = thread::Builder::new()
        .name("drvAHxxxTask".into())
        .spawn(move || read_thread(read_ctx))
        .expect("failed to spawn drvAHxxxTask");

    let callback_thread = qe::start_callback_task(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
        shared,
        acquire_param,
    });

    Ok(AhxxxRuntime {
        runtime_handle,
        params,
        nd_params,
        pool,
        outputs,
        _read_thread: read_thread_handle,
        _callback_thread: callback_thread,
    })
}
