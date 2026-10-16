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
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::ahxxx_proto as proto;
use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeReadFormat,
    QeTriggerMode, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::octet::{OctetIo, connect_octet};
use crate::{asyn_print, trace};

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
    /// C++ `QEPortName_`, printed by [`PortDriver::report`].
    qe_port_name: String,
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
        let (io, status) = connect_octet(qe_port_name, proto::AHXXX_TIMEOUT);
        // C++ returns from its constructor here and keeps the port, with no
        // meter link and no read thread.
        if let Err(e) = status {
            println!(
                "drvAHxxx:drvAHxxx: error calling pasynOctetSyncIO->connect, status={}, error={}",
                trace::status_code(&e),
                e.message_display()
            );
            return Ok(Self {
                base,
                io,
                shared,
                model,
                ah401_series,
                firmware_version: String::new(),
                qe_port_name: qe_port_name.to_string(),
            });
        }
        // C++ sets the input EOS only in setAcquire, where a refusal fails the
        // acquire; it is set here too so the firmware query frames on it.
        let _ = io.set_input_eos(b"\r\n");

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
            qe_port_name: qe_port_name.to_string(),
        };

        // C++ constructor calls reset() under the port lock and ignores its
        // status: the meter may be offline at IOC start.
        let _ = this.reset();
        Ok(this)
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
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
            let _ = self.set_acquire(0);
        }
        let resp = self.write_read_meter(out)?;
        if resp != "ACK" && resp != "AK" {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvAHxxx:sendCommand: error, outString={out} expected ACK, received {resp}"
            );
            return Err(error(format!("AHxxx: {out} -> {resp}, expected ACK")));
        }
        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    /// C++ `drvAHxxx::readStatus`.
    ///
    /// As in `drvTetrAMM`, the `goto error` path skips the `setAcquire(1)`
    /// restore, so a status read that fails leaves the meter stopped.
    fn read_status_inner(&mut self, last: &mut trace::StatusExchange) -> AsynResult<()> {
        let p = self.params();
        let bad = |what: &str, resp: &str| error(format!("AHxxx: bad {what} response: {resp}"));

        let resp = last.query(&self.io, "RNG ?", proto::MAX_COMMAND_LEN)?;
        let range = proto::parse_range(&resp).ok_or_else(|| bad("RNG", &resp))?;
        self.base.port_base.set_int32_param(p.range, 0, range)?;

        let read_format =
            QeReadFormat::from_i32(qe::int_param(&self.base.port_base, p.read_format, 0));
        let mut sample_time;
        if self.is_ah401() {
            let resp = last.query(&self.io, "HLF ?", proto::MAX_COMMAND_LEN)?;
            let ping_pong = proto::parse_hlf(&resp).ok_or_else(|| bad("HLF", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.ping_pong, 0, ping_pong)?;

            let resp = last.query(&self.io, "ITM ?", proto::MAX_COMMAND_LEN)?;
            let integration_time = proto::parse_itm(&resp).ok_or_else(|| bad("ITM", &resp))?;
            self.base
                .port_base
                .set_float64_param(p.integration_time, 0, integration_time)?;
            sample_time = proto::sample_time_ah401(ping_pong, integration_time);
        } else {
            let resp = last.query(&self.io, "CHN ?", proto::MAX_COMMAND_LEN)?;
            let num_channels = proto::parse_chn(&resp).ok_or_else(|| bad("CHN", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.num_channels, 0, num_channels)?;
            self.shared.acq.lock().num_channels = num_channels;

            let resp = last.query(&self.io, "RES ?", proto::MAX_COMMAND_LEN)?;
            let resolution = proto::parse_res(&resp).ok_or_else(|| bad("RES", &resp))?;
            self.base
                .port_base
                .set_int32_param(p.resolution, 0, resolution)?;
            self.shared.acq.lock().resolution = resolution;

            sample_time = proto::sample_time_ah501(read_format, resolution, num_channels);
        }

        if proto::reads_bias_status(self.model) {
            let resp = last.query(&self.io, "HVS ?", proto::MAX_COMMAND_LEN)?;
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

        let averaging_time = qe::double_param(&self.base.port_base, p.averaging_time, 0);
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
            QeTriggerMode::from_i32(qe::int_param(&self.base.port_base, p.trigger_mode, 0));
        let acquire_mode =
            QeAcquireMode::from_i32(qe::int_param(&self.base.port_base, p.acquire_mode, 0));
        let num_average = qe::int_param(&self.base.port_base, p.num_average, 0);
        let read_format =
            QeReadFormat::from_i32(qe::int_param(&self.base.port_base, p.read_format, 0));
        trace::ensure_input_eos(&self.io, &self.base.port_base, "drvAHxxx:setAcquire")?;

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
            // C++ returns the last `TRG OFF` exchange's status.
            let mut status = Ok(());
            loop {
                // Both meter families are stopped: which one is on the far end
                // is not known until the firmware string has been read.
                let _ = self.write_read_meter("ACQ OFF");
                let _ = self.write_read_meter("S");
                // TRG OFF because the meter's mode at startup is unknown.
                // C++ tests only this last command's status and gives up on
                // any failure, a timeout included.
                if let Err(e) = self.write_read_meter("TRG OFF") {
                    asyn_print!(
                        self.base.port_base,
                        ERROR,
                        "drvAHxxx:setAcquire: error calling pasynOctetSyncIO->writeRead, status={}",
                        trace::status_code(&e)
                    );
                    status = Err(e);
                    break;
                }
                // Flush, then read with a short timeout to drain any responses;
                // only a timeout that read nothing ends the loop.
                let _ = self.io.flush();
                let drained = self.io.read_line(proto::MAX_COMMAND_LEN);
                if let (AsynStatus::Timeout, 0, _) = trace::read_result(&drained) {
                    break;
                }
            }
            let _ = self.base_set_acquire(0);
            return status;
        }

        let _ = self.base_set_acquire(1);

        let num_acquire = proto::naq_value(acquire_mode, num_average);
        let model = self.model;
        // Put the device in the appropriate mode. As in C++ only the command
        // that starts the meter decides the status.
        let _ = self.write_read_meter(proto::cmd_read_format(read_format));

        // In one-shot mode ask the meter for a specific number of samples.
        // On the AH501BE the NAQ command starts the acquisition and is not
        // echoed, so it is sent below instead.
        if model != QeModel::Ah501be {
            let _ = self.write_read_meter(&proto::cmd_naq(num_acquire));
        }

        let started = if trigger_mode == QeTriggerMode::ExtTrigger
            || trigger_mode == QeTriggerMode::ExtGate
        {
            // External trigger mode: the meter waits for the trigger.
            self.write_read_meter("TRG ON").map(drop)
        } else if self.is_ah401() {
            // The AH401 series echoes an ACK after ACQ ON; the AH501s do not.
            self.write_read_meter("ACQ ON").map(drop)
        } else if acquire_mode == QeAcquireMode::Single && model == QeModel::Ah501be {
            self.io.write(&proto::cmd_naq(num_acquire)).map(drop)
        } else {
            self.io.write("ACQ ON").map(drop)
        };
        // Binary mode overwrites it with the input-EOS call's status.
        let status = match read_format {
            QeReadFormat::Binary => Ok(()),
            _ => started,
        };

        {
            let mut acq = self.shared.acq.lock();
            acq.acquire_mode = acquire_mode as i32;
            acq.trigger_mode = trigger_mode as i32;
            acq.read_format = read_format as i32;
        }

        // Setting the flag before the signal keeps the read thread from going
        // straight back to sleep; C++ orders it the other way because both
        // sides run under the port lock.
        self.shared.set_acquiring(true);
        self.shared.acquire_start.signal();
        if status.is_err() {
            self.shared.set_acquiring(false);
        }
        status
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
            let _ = self.set_acquire(0);
        }
        let mut last = trace::StatusExchange::default();
        if let Err(e) = self.read_status_inner(&mut last) {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvAHxxx:getStatus: error, outString={}, inString={}",
                last.out,
                last.inp
            );
            return Err(e);
        }
        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    /// C++ `drvAHxxx::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        let _ = self.set_acquire(0);
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
                    asyn_print!(
                        self.base.port_base,
                        ERROR,
                        "drvAHxxx:reset: unknown firmware version = {}",
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

    /// C++ `drvAHxxx::report`, which ends in the base report.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "drvAHxxx: port={}, IP port={}, firmware version={}",
            self.base.port_base.port_name, self.qe_port_name, self.firmware_version
        );
        self.asyn_port_driver_report(out, level);
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.qe_write_int32(user.reason, user.addr, value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.qe_write_float64(user.reason, user.addr, value)
    }
}

impl AhxxxDriver {}

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

                let read = ctx.io.locked_read_binary(n_requested);
                let mut outcome = match read {
                    Ok(o) if o.data.len() == n_requested && o.eom.contains(EomReason::CNT) => o,
                    Ok(_) => {
                        trace::print_read_error(&ctx.handle, "drvAHxxx:readThread", &read);
                        thread::sleep(READ_ERROR_BACKOFF);
                        continue;
                    }
                    Err(ref e) => {
                        // C++ `readThread` (drvAHxxx.cpp:252-259): in Ext.
                        // Gate mode the AH501BE answers its `ACK\r\n`
                        // preamble and then goes quiet waiting for the
                        // external gate pulse, so the binary read times out
                        // after exactly those 5 bytes. Only
                        // `AsynError::partial_read` still carries them.
                        if !is_timeout(e) {
                            trace::print_read_error(&ctx.handle, "drvAHxxx:readThread", &read);
                            thread::sleep(READ_ERROR_BACKOFF);
                        } else if is_ext_gate_ack_timeout(e, ext_gate) {
                            asyn_print!(
                                ctx.handle,
                                IO_DRIVER,
                                "drvAHxxx::readThread timeout with nRead=5, triggering callbacks"
                            );
                            ctx.shared.trigger_callbacks(&ctx.handle);
                        }
                        continue;
                    }
                };

                // C++ `readThread` (drvAHxxx.cpp:265-283): a full, correctly
                // terminated read can still start with the ACK preamble if
                // the gate fired between the ACK and the rest of the frame
                // arriving. Trigger, then shift the preamble out and top the
                // buffer back up to `n_requested`. `accumulate_binary_*`
                // below already rejects a too-short buffer, so a failed or
                // short refill is caught there rather than re-checked here.
                if ext_gate && proto::starts_with_ack_preamble(&outcome.data) {
                    asyn_print!(
                        ctx.handle,
                        IO_DRIVER,
                        "drvAHxxx::readThread found ACK\\r\\n at start of data, triggering callbacks"
                    );
                    ctx.shared.trigger_callbacks(&ctx.handle);
                    let preamble_len = proto::ACK_PREAMBLE.len();
                    outcome.data.drain(0..preamble_len);
                    let extra = ctx.io.locked_read_binary(preamble_len);
                    match &extra {
                        Ok(o) if o.data.len() == preamble_len && o.eom.contains(EomReason::CNT) => {
                        }
                        _ => {
                            let (status, n_read, eom) = trace::read_result(&extra);
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvAHxxx:readThread: unexpected error reading additional {preamble_len} bytes from meter status={}, nRead={n_read}, eomReason={eom}",
                                status as i32
                            );
                        }
                    }
                    if let Ok(o) = extra {
                        outcome.data.extend_from_slice(&o.data);
                    }
                }
                trace::print_io(
                    &ctx.handle,
                    trace::TraceMask::IO_DRIVER,
                    &outcome.data,
                    format_args!("drvAHxxx::readThread buffer read"),
                );

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
                    let read = ctx.io.locked_read_line(proto::ASCII_BUFFER_SIZE);
                    let outcome = match read {
                        Ok(o) if o.eom.contains(EomReason::EOS) => o,
                        _ => {
                            if trace::read_result(&read).0 != AsynStatus::Timeout {
                                trace::print_read_error(&ctx.handle, "drvAHxxx:readThread", &read);
                                thread::sleep(READ_ERROR_BACKOFF);
                            }
                            continue;
                        }
                    };
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
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvAHxxx:readThread: error reading meter nRead={}, expected {n_expected}, input={line}",
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
    pub pool: epics_rs::ad_core::ndarray_pool::DriverPool,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    /// `None` when the constructor found no QE port.
    _read_thread: Option<thread::JoinHandle<()>>,
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
    claim: &PortClaim,
    qe_port_name: &str,
    ring_buffer_size: usize,
    max_memory: usize,
    model_name: &str,
) -> AsynResult<AhxxxRuntime> {
    let port_name = claim.name();
    // Any other name is C++'s QE_ModelUnknown, which reset() resolves from
    // the meter's firmware version.
    let model = proto::model_from_name(model_name);

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

    let (runtime_handle, _actor) = create_port_runtime(claim, driver, RuntimeConfig::default())?;
    let handle = runtime_handle.port_handle().clone();

    let read_ctx = ReadContext {
        io,
        handle: handle.clone(),
        params,
        shared: shared.clone(),
        ah401_series,
        is_ah501be: model == QeModel::Ah501be,
    };
    let read_thread_handle = read_ctx.io.handle().is_some().then(|| {
        thread::Builder::new()
            .name("drvAHxxxTask".into())
            .spawn(move || read_thread(read_ctx))
            .expect("failed to spawn drvAHxxxTask")
    });

    let callback_thread = qe::start_callback_task(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
        pool: pool.clone(),
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
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

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

    /// C++ prints the connect failure and returns from its constructor: the
    /// port stays, with no read thread, and a meter command fails on the
    /// unbound asynUser.
    #[test]
    fn a_missing_qe_port_keeps_the_port_without_a_link() {
        let rt = create_ahxxx(
            &claim_port("QE_AH_NOQE").unwrap(),
            "QE_AH_NOQE_IP",
            4,
            0,
            "AH501",
        )
        .unwrap();
        assert!(rt._read_thread.is_none());
        let err = rt
            .port_handle()
            .submit_blocking(
                epics_rs::asyn::request::RequestOp::Int32Write { value: 1 },
                epics_rs::asyn::user::AsynUser::new(rt.params.range),
            )
            .unwrap_err();
        assert!(
            err.message()
                .contains("asynManager::queueLockPort not connected"),
            "{err}"
        );
    }

    /// C++ keeps an unrecognised model name as QE_ModelUnknown and lets
    /// reset() take the model from `VER ?`.
    #[test]
    fn an_unknown_model_name_is_resolved_from_the_firmware() {
        use std::io::{Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            let mut buf = [0u8; 256];
            while let Ok(n) = sock.read(&mut buf) {
                if n == 0 {
                    break;
                }
                let reply: &[u8] = if buf[..n].starts_with(b"VER") {
                    b"AH501D 3.0\r\n"
                } else {
                    b"ACK\r\n"
                };
                if sock.write_all(reply).is_err() {
                    break;
                }
            }
        });
        let (_io, _meter) = crate::octet::create_ip_port(
            "QE_AH_UNKMODEL_IP",
            &addr.to_string(),
            proto::AHXXX_TIMEOUT,
            false,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        let rt = create_ahxxx(
            &claim_port("QE_AH_UNKMODEL").unwrap(),
            "QE_AH_UNKMODEL_IP",
            4,
            0,
            "AH999",
        )
        .unwrap();
        let model = rt
            .port_handle()
            .read_int32_blocking(rt.params.model, 0)
            .unwrap();
        assert_eq!(model, QeModel::Ah501d as i32);
    }

    /// C++ sets the input EOS only in setAcquire (drvAHxxx.cpp:465-472): a
    /// port that refuses it keeps the driver and fails the acquire instead.
    #[test]
    fn an_input_eos_refusal_fails_the_acquire_not_the_port() {
        let (_meter, _runtime) = crate::octet::tests::eos_refusing_port("QE_AH_NOEOS_IP");
        let rt = create_ahxxx(
            &claim_port("QE_AH_NOEOS").unwrap(),
            "QE_AH_NOEOS_IP",
            4,
            0,
            "AH501D",
        )
        .unwrap();
        let err = rt
            .port_handle()
            .write_int32_blocking(rt.nd_params.acquire, 0, 1)
            .unwrap_err();
        assert_eq!(err.status(), epics_rs::asyn::error::AsynStatus::Error);
        assert_eq!(err.message(), "no input EOS");
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
