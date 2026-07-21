//! Port of `quadEMApp/caenSrc/drvAHxxx.{h,cpp}` — Elettra/CaenEls AHxxx
//! 4-channel picoammeters, reached over TCP or serial through an asyn octet
//! port.
//!
//! Ported models: AH401B, AH401D, AH501, AH501BE, AH501C, AH501D.
//!
//! The AH501BE in external-gate trigger mode needs the partial bytes of a
//! *timed-out* binary read: upstream recognises the meter's `ACK\r\n`
//! preamble either there or at the front of a completed frame
//! (`drvAHxxx.cpp:252-283`). `asyn-rs` 0.22.1 discarded a timed-out read's
//! partial bytes, so the port used to refuse the mode outright;
//! `AsynError::partial_read` (asyn-rs 0.24.0+) carries them, so `read_thread`'s
//! binary branch now detects the preamble the same way upstream does.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
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

/// True for a timeout, including one `AsynError::PartialRead` wraps to carry
/// the bytes transferred before it. A bare `AsynError::Status` variant match
/// would miss that wrapper and misclassify every partial-transfer timeout —
/// routine on this port, since it installs the EOS interpose — as an
/// unexpected error.
fn is_timeout(e: &AsynError) -> bool {
    e.status() == AsynStatus::Timeout
}

/// `read_thread`'s binary-read error arm (C++ `drvAHxxx.cpp:252-259`): trigger
/// callbacks only when the failure was a genuine timeout. `status` must be
/// checked first — `AsynError::PartialRead` also wraps non-timeout failures
/// (an ECONNRESET, say) that could happen to carry the same 5 bytes, and C++
/// reaches this trigger only inside `status != asynTimeout`'s `else`.
fn is_ext_gate_ack_timeout(e: &AsynError, ext_gate: bool) -> bool {
    ext_gate
        && is_timeout(e)
        && e.partial_read()
            .is_some_and(|partial| proto::is_ack_preamble_only(&partial.data))
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
    /// C++ `AH401Series_` / `AH501Series_`, set by `reset()` once the model is
    /// known. Shared with the read thread, which decodes accordingly.
    ah401_series: Arc<AtomicBool>,
    firmware_version: String,
}

impl AhxxxDriver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        max_memory: usize,
        model: QeModel,
        shared: Arc<QuadEmShared>,
        ah401_series: Arc<AtomicBool>,
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
            ah401_series,
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

    /// C++ `AH401Series_`.
    fn is_ah401(&self) -> bool {
        self.ah401_series.load(Ordering::Acquire)
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
        self.ah401_series
            .store(proto::is_ah401_series(self.model), Ordering::Release);
        self.base_reset()
    }

    /// C++ `drvAHxxx::readStatus`.
    ///
    /// As in `drvTetrAMM`, the `goto error` path skips the `setAcquire(1)`
    /// restore, so a status read that fails leaves the meter stopped.
    fn read_status_inner(&mut self) -> AsynResult<()> {
        let p = self.params();
        let bad = |what: &str, resp: &str| error(format!("AHxxx: bad {what} response: {resp}"));

        let resp = self.write_read_meter("RNG ?")?;
        let range = proto::parse_range(&resp).ok_or_else(|| bad("RNG", &resp))?;
        self.base.port_base.set_int32_param(p.range, 0, range)?;

        let read_format =
            QeReadFormat::from_i32(self.base.port_base.get_int32_param(p.read_format, 0)?);
        let mut sample_time;
        if self.is_ah401() {
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
        } else {
            let resp = self.write_read_meter("CHN ?")?;
            let num_channels = proto::parse_chn(&resp).ok_or_else(|| bad("CHN", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.num_channels, 0, num_channels)?;
            self.shared.acq.lock().num_channels = num_channels;

            let resp = self.write_read_meter("RES ?")?;
            let resolution = proto::parse_res(&resp).ok_or_else(|| bad("RES", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.resolution, 0, resolution)?;
            self.shared.acq.lock().resolution = resolution;

            sample_time = proto::sample_time_ah501(read_format, resolution, num_channels);
        }

        if proto::reads_bias_status(self.model) {
            let resp = self.write_read_meter("HVS ?")?;
            match proto::parse_hvs(&resp).ok_or_else(|| bad("HVS", &resp))? {
                None => self.base.port_base.set_int32_param(p.bias_state, 0, 0)?,
                Some(volts) => {
                    self.base.port_base.set_int32_param(p.bias_state, 0, 1)?;
                    self.base
                        .port_base
                        .set_float64_param(p.bias_voltage, 0, volts)?;
                }
            }
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

        let num_acquire = proto::naq_value(acquire_mode, num_average);
        let is_ah401 = self.is_ah401();
        let model = self.model;
        let result = (|| -> AsynResult<()> {
            // Put the device in the appropriate mode.
            self.write_read_meter(proto::cmd_read_format(read_format))?;

            // In one-shot mode ask the meter for a specific number of samples.
            // On the AH501BE the NAQ command starts the acquisition and is not
            // echoed, so it is sent below instead.
            if model != QeModel::Ah501be {
                self.write_read_meter(&proto::cmd_naq(num_acquire))?;
            }

            if trigger_mode == QeTriggerMode::ExtTrigger || trigger_mode == QeTriggerMode::ExtGate {
                // External trigger mode: the meter waits for the trigger.
                self.write_read_meter("TRG ON")?;
            } else if is_ah401 {
                // The AH401 series echoes an ACK after ACQ ON; the AH501s do not.
                self.write_read_meter("ACQ ON")?;
            } else if acquire_mode == QeAcquireMode::Single && model == QeModel::Ah501be {
                self.io.write(&proto::cmd_naq(num_acquire))?;
            } else {
                self.io.write("ACQ ON")?;
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

    /// C++ `drvAHxxx::setPingPong`: `"HLF ON"`/`"HLF OFF"`, AH401 series only.
    fn set_ping_pong(&mut self, value: i32) -> AsynResult<()> {
        if !self.is_ah401() {
            return Ok(());
        }
        self.send_command(proto::cmd_ping_pong(value))
    }

    /// C++ `drvAHxxx::setIntegrationTime`: clamp, write back, `"ITM %d"`.
    /// AH401 series only.
    fn set_integration_time(&mut self, value: f64) -> AsynResult<()> {
        if !self.is_ah401() {
            return Ok(());
        }
        let (clamped, cmd) = proto::cmd_integration_time(value);
        if clamped != value {
            let idx = self.params().integration_time;
            self.base.port_base.set_float64_param(idx, 0, clamped)?;
        }
        self.send_command(&cmd)
    }

    /// C++ `drvAHxxx::setNumChannels`: `"CHN %d"`, AH501 series only.
    fn set_num_channels(&mut self, value: i32) -> AsynResult<()> {
        if self.is_ah401() {
            return Ok(());
        }
        self.send_command(&proto::cmd_num_channels(value))
    }

    /// C++ `drvAHxxx::setResolution`: `"RES %d"`, AH501 series only.
    fn set_resolution(&mut self, value: i32) -> AsynResult<()> {
        if self.is_ah401() {
            return Ok(());
        }
        self.send_command(&proto::cmd_resolution(value))
    }

    /// C++ `drvAHxxx::setBiasState`: `"HVS ON"`/`"HVS OFF"`. The AH401 series
    /// and the plain AH501 have no bias supply.
    fn set_bias_state(&mut self, value: i32) -> AsynResult<()> {
        if !proto::has_bias_supply(self.model) {
            return Ok(());
        }
        self.send_command(proto::cmd_bias_state(value != 0))
    }

    /// C++ `drvAHxxx::setBiasVoltage`: `"HVS %f"`.
    fn set_bias_voltage(&mut self, value: f64) -> AsynResult<()> {
        if !proto::has_bias_supply(self.model) {
            return Ok(());
        }
        self.send_command(&proto::cmd_bias_voltage(value))
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
        } else if reason == p.bias_state {
            self.set_bias_state(value)?;
            self.read_status()?;
        } else if reason == p.num_channels {
            self.set_num_channels(value)?;
            self.read_status()?;
        } else if reason == p.resolution {
            self.set_resolution(value)?;
            self.read_status()?;
        } else if reason == p.bias_interlock
            || reason == p.trigger_mode
            || reason == p.trigger_polarity
        {
            // setBiasInterlock, setTriggerMode and setTriggerPolarity are
            // drvQuadEM dummies for this driver: only readStatus runs.
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
            self.set_bias_voltage(value)?;
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
    ah401_series: Arc<AtomicBool>,
    /// C++ `model_ == QE_ModelAH501BE`. Fixed for the port's whole life:
    /// unlike `ah401_series`, `ahxxx_proto::model_from_firmware` has no
    /// AH501BE branch, so a meter configured as anything else can never
    /// become one through firmware rediscovery in `reset()`.
    is_ah501be: bool,
}

/// C++ `drvAHxxx::readThread`.
fn read_thread(ctx: ReadContext) {
    let mut read_format = QeReadFormat::Binary;

    loop {
        if !ctx.shared.is_acquiring() {
            ctx.shared.set_reading_active(false);
            ctx.shared.acquire_start.wait();
            read_format = QeReadFormat::from_i32(ctx.shared.acq.lock().read_format);
            ctx.shared.set_reading_active(true);
        }

        let ah401_series = ctx.ah401_series.load(Ordering::Acquire);

        // C++ `if (valuesPerRead_ < 1) valuesPerRead_ = 1;` — the clamp is
        // written back to the driver's member, not to the parameter library.
        let values_per_read = {
            let mut acq = ctx.shared.acq.lock();
            if acq.values_per_read < 1 {
                acq.values_per_read = 1;
            }
            acq.values_per_read
        };

        let (resolution, num_channels) = {
            let acq = ctx.shared.acq.lock();
            (
                acq.resolution,
                (acq.num_channels as usize).clamp(1, QE_MAX_INPUTS),
            )
        };
        // C++ forces numChannels_ = 4 for the AH401 series in the binary path.
        let num_channels = if ah401_series {
            proto::AH401_NUM_CHANNELS
        } else {
            num_channels
        };

        let mut raw = [0.0f64; QE_MAX_INPUTS];

        match read_format {
            QeReadFormat::Binary => {
                let n_requested = if ah401_series {
                    proto::ah401_read_len(values_per_read as usize)
                } else {
                    proto::ah501_read_len(resolution, num_channels, values_per_read as usize)
                };
                let ext_gate = ctx.is_ah501be
                    && QeTriggerMode::from_i32(ctx.shared.acq.lock().trigger_mode)
                        == QeTriggerMode::ExtGate;

                let mut outcome = match ctx.io.read_binary(n_requested) {
                    Ok(o) => o,
                    Err(e) => {
                        // C++ `readThread` (drvAHxxx.cpp:252-259): in Ext.
                        // Gate mode the AH501BE answers its `ACK\r\n`
                        // preamble and then goes quiet waiting for the
                        // external gate pulse, so the binary read times out
                        // after exactly those 5 bytes. Only
                        // `AsynError::partial_read` still carries them.
                        if is_ext_gate_ack_timeout(&e, ext_gate) {
                            ctx.shared.trigger_callbacks();
                        }
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

                // C++ `readThread` (drvAHxxx.cpp:265-283): a full, correctly
                // terminated read can still start with the ACK preamble if
                // the gate fired between the ACK and the rest of the frame
                // arriving. Trigger, then shift the preamble out and top the
                // buffer back up to `n_requested`. `accumulate_binary_*`
                // below already rejects a too-short buffer, so a failed or
                // short refill is caught there rather than re-checked here.
                if ext_gate && proto::starts_with_ack_preamble(&outcome.data) {
                    ctx.shared.trigger_callbacks();
                    let preamble_len = proto::ACK_PREAMBLE.len();
                    outcome.data.drain(0..preamble_len);
                    match ctx.io.read_binary(preamble_len) {
                        Ok(extra) => outcome.data.extend_from_slice(&extra.data),
                        Err(e) => log::error!(
                            "drvAHxxx: unexpected error reading additional {preamble_len} bytes from meter: {e}"
                        ),
                    }
                }

                let decoded = if ah401_series {
                    proto::accumulate_binary_ah401(&outcome.data, values_per_read as usize)
                } else {
                    proto::accumulate_binary_ah501(
                        &outcome.data,
                        resolution,
                        num_channels,
                        values_per_read as usize,
                    )
                };
                let Some(values) = decoded else {
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
                    let line = outcome.as_str();
                    let values = if ah401_series {
                        proto::parse_ascii_ah401(&line, proto::AH401_NUM_CHANNELS)
                    } else {
                        // The meter answers ACK once the requested number of
                        // trigger samples has been delivered.
                        if line.contains("ACK") {
                            break;
                        }
                        let n_expected = proto::ah501_ascii_expected_len(resolution, num_channels);
                        if outcome.data.len() != n_expected {
                            log::error!(
                                "drvAHxxx: error reading meter nRead={}, expected {n_expected}, input={line}",
                                outcome.data.len()
                            );
                            continue;
                        }
                        proto::parse_ascii_ah501(&line, resolution, num_channels)
                    };
                    for (acc, v) in raw.iter_mut().zip(values) {
                        *acc += v;
                    }
                }
            }
        }

        proto::average_over_values_per_read(&mut raw, num_channels, values_per_read);
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
/// Rust `NDArrayPool`. `model_name` is one of `AH401B`, `AH401D`, `AH501`,
/// `AH501BE`, `AH501C`, `AH501D`.
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
            "drvAHxxxConfigure: unknown model '{model_name}'; use one of \
             AH401B, AH401D, AH501, AH501BE, AH501C, AH501D"
        )));
    }

    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);
    let ah401_series = Arc::new(AtomicBool::new(proto::is_ah401_series(model)));

    let driver = AhxxxDriver::new(
        port_name,
        qe_port_name,
        max_memory,
        model,
        shared.clone(),
        ah401_series.clone(),
    )?;
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
        ah401_series,
        is_ah501be: model == QeModel::Ah501be,
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

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interpose::PartialOctetRead;

    /// `AsynError::PartialRead` is how `asyn-rs` 0.24+ carries a timed-out
    /// read's transferred bytes (see the module docs); a bare
    /// `AsynError::Status` match would miss it entirely.
    #[test]
    fn is_timeout_recognizes_a_partial_read_wrapped_timeout() {
        let e = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(PartialOctetRead {
            data: proto::ACK_PREAMBLE.to_vec(),
            eom_reason: EomReason::empty(),
        });
        assert!(is_timeout(&e));
        assert_eq!(
            e.partial_read().map(|p| p.data.as_slice()),
            Some(proto::ACK_PREAMBLE)
        );
    }

    #[test]
    fn is_timeout_rejects_a_real_error() {
        assert!(!is_timeout(&error("boom")));
    }

    /// `AsynError::PartialRead` wraps non-timeout failures too (`error.rs`
    /// cites a mid-transfer ECONNRESET). One that happens to carry exactly
    /// the 5-byte ACK preamble must NOT trigger callbacks — only a genuine
    /// timeout carrying it should.
    #[test]
    fn ext_gate_ack_trigger_requires_a_genuine_timeout() {
        let ack_payload = PartialOctetRead {
            data: proto::ACK_PREAMBLE.to_vec(),
            eom_reason: EomReason::empty(),
        };
        let non_timeout = AsynError::Status {
            status: AsynStatus::Error,
            message: "read error: ECONNRESET".into(),
        }
        .with_partial_read(ack_payload.clone());
        assert!(!is_ext_gate_ack_timeout(&non_timeout, true));

        let timeout = AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
        .with_partial_read(ack_payload);
        assert!(is_ext_gate_ack_timeout(&timeout, true));
    }
}
