//! Port of `quadEMApp/sensicSrc/drvPCR4.{h,cpp}` — the SenSiC PCR4 4-channel
//! picoammeter, reached over TCP or serial through an asyn octet port.
//!
//! The PCR4 streams ASCII sample lines and averages internally (`SPR`), so the
//! read thread publishes one sample per line and never divides by
//! values-per-read the way the AHxxx and TetrAMM ports do.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::param::EnumEntry;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeTriggerMode,
    QeTriggerPolarity, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::octet::{OctetIo, connect_octet};
use crate::pcr4_proto::{self as proto, DataLine};

/// Poll interval of C++'s `epicsThreadSleep(0.01)` handshake loop.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `epicsThreadSleep(1.0)` after an unexpected read error.
const READ_ERROR_BACKOFF: Duration = Duration::from_secs(1);
/// C++ `reset`: `epicsThreadSleep(1.0)` between probes.
const RESET_POLL: Duration = Duration::from_secs(1);

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

// ===========================================================================
// Driver
// ===========================================================================

pub struct Pcr4Driver {
    base: QuadEmBase,
    io: OctetIo,
    shared: Arc<QuadEmShared>,
    firmware_version: String,
    /// C++ `versionNumber_`, parsed out of the `PCR4v<n>` firmware string and
    /// used only to pick the range enum.
    version_number: i32,
}

impl Pcr4Driver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        max_memory: usize,
        shared: Arc<QuadEmShared>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let io = connect_octet(qe_port_name, proto::PCR4_TIMEOUT)?;
        // C++ sets the input EOS at the top of every setAcquire; it never
        // changes, so it is set once here. See `octet`.
        io.set_input_eos(b"\r\n")?;

        base.port_base
            .set_int32_param(base.params.model, 0, QeModel::Pcr4 as i32)?;
        base.port_base.set_int32_param(
            base.params.values_per_read,
            0,
            proto::DEFAULT_VALUES_PER_READ,
        )?;
        {
            let mut acq = shared.acq.lock();
            acq.resolution = proto::RESOLUTION;
            acq.values_per_read = proto::DEFAULT_VALUES_PER_READ;
            acq.num_channels = QE_MAX_INPUTS as i32;
        }

        let mut this = Self {
            base,
            io,
            shared,
            firmware_version: String::new(),
            version_number: 0,
        };

        // C++ constructor calls getFirmwareVersion() under the port lock; the
        // meter may be offline at IOC start, so a failure is logged, not fatal.
        if let Err(e) = this.get_firmware_version() {
            log::warn!("drvPCR4: reading the firmware version at startup failed: {e}");
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

    /// C++ `sendCommand`: stop acquisition around the command, require `ACK`,
    /// then restart.
    fn send_command(&mut self, out: &str) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            self.set_acquire(0)?;
        }
        let resp = self.write_read_meter(out)?;
        if resp != proto::ACK {
            log::error!("drvPCR4: outString={out} expected ACK, received {resp}");
            return Err(error(format!("PCR4: {out} -> {resp}, expected ACK")));
        }
        if prev_acquiring {
            self.set_acquire(1)?;
        }
        Ok(())
    }

    /// C++ `getFirmwareVersion`.
    ///
    /// Upstream reads `&inString_[8]` without checking the length and calls
    /// `atoi(strstr(inString_, "PCR4v") + 5)` without checking for NULL. Here
    /// a reply that is not a version reply leaves the firmware "Unknown" and
    /// the revision 0, and `readEnum` then refuses to offer range choices —
    /// which is what upstream's `default:` branch does for an unknown revision.
    fn get_firmware_version(&mut self) -> AsynResult<()> {
        self.set_acquire(0)?;
        self.firmware_version = "Unknown".into();
        let resp = self.write_read_meter(proto::CMD_VERSION)?;
        match proto::parse_version(&resp) {
            Some((firmware, version)) => {
                self.firmware_version = firmware;
                self.version_number = version;
            }
            None => {
                log::error!("drvPCR4: unrecognised VERSION:? reply: {resp}");
                self.version_number = 0;
            }
        }
        let idx = self.params().firmware;
        self.base
            .port_base
            .set_string_param(idx, 0, self.firmware_version.clone())?;
        Ok(())
    }

    /// C++ `drvPCR4::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        self.set_acquire(0)?;
        self.send_command(proto::CMD_RESET)?;
        // Wait for the meter to start communicating again, or give up after
        // RESET_WAIT_LOOPS seconds.
        let mut back = false;
        for _ in 0..proto::RESET_WAIT_LOOPS {
            thread::sleep(RESET_POLL);
            if self.write_read_meter(proto::CMD_VERSION).is_ok() {
                back = true;
                break;
            }
        }
        if !back {
            log::error!(
                "drvPCR4: no response from the meter {} seconds after RESET",
                proto::RESET_WAIT_LOOPS
            );
        }
        self.base_reset()
    }

    /// C++ `drvPCR4::setAcquireParams`: recompute the sample time, then push
    /// every acquisition setting to the meter.
    ///
    /// As in C++ the individual `writeReadMeter` statuses are discarded: the
    /// meter is configured on a best-effort basis and the caller only learns
    /// about a failure through the next status read.
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        let p = self.params();
        let prev_acquiring = self.shared.is_acquiring();

        let range = self.base.port_base.get_int32_param(p.range, 0)?;
        let num_channels = self.base.port_base.get_int32_param(p.num_channels, 0)?;
        let trigger_mode = self.base.port_base.get_int32_param(p.trigger_mode, 0)?;
        let trigger_polarity = QeTriggerPolarity::from_i32(
            self.base.port_base.get_int32_param(p.trigger_polarity, 0)?,
        );
        let values_per_read = self.base.port_base.get_int32_param(p.values_per_read, 0)?;
        let averaging_time = self.base.port_base.get_float64_param(p.averaging_time, 0)?;

        if prev_acquiring || trigger_mode != 0 {
            self.set_acquire(0)?;
        }

        let sample_time = proto::sample_time(values_per_read);
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;
        let num_average = num_average_from(averaging_time, sample_time);
        self.base
            .port_base
            .set_int32_param(p.num_average, 0, num_average)?;
        {
            let mut acq = self.shared.acq.lock();
            acq.num_average = num_average;
            acq.num_channels = num_channels;
            acq.values_per_read = values_per_read;
            acq.trigger_mode = trigger_mode;
        }

        let _ = self.write_read_meter(&proto::cmd_range(range));
        let _ = self.write_read_meter(&proto::cmd_num_channels(num_channels));
        let _ = self.write_read_meter(&proto::cmd_values_per_read(values_per_read));
        let _ = self.write_read_meter(proto::cmd_trigger_polarity(trigger_polarity));
        let _ = self.write_read_meter(proto::cmd_trigger(trigger_mode));

        // In free-run the caller (setAcquire) starts the acquisition itself;
        // only a triggered acquisition is restarted from here.
        if prev_acquiring && trigger_mode != 0 {
            self.set_acquire(1)?;
        }
        Ok(())
    }

    /// C++ `drvPCR4::readStatus`.
    ///
    /// As in the other quadEM ports the `goto error` path skips the
    /// `setAcquire(1)` restore, so a status read that fails leaves the meter
    /// stopped.
    fn read_status_inner(&mut self) -> AsynResult<()> {
        let p = self.params();
        let bad = |what: &str, resp: &str| error(format!("PCR4: bad {what} response: {resp}"));

        let resp = self.write_read_meter(proto::CMD_RANGE_QUERY)?;
        let range = proto::parse_range(&resp).ok_or_else(|| bad("RANGE:?", &resp))?;
        self.base.port_base.set_int32_param(p.range, 0, range)?;

        let resp = self.write_read_meter(proto::CMD_CHANNELS_QUERY)?;
        let num_channels =
            proto::parse_num_channels(&resp).ok_or_else(|| bad("CHANNELS:?", &resp))?;
        self.base
            .port_base
            .set_int32_param(p.num_channels, 0, num_channels)?;
        self.shared.acq.lock().num_channels = num_channels;

        let resp = self.write_read_meter(proto::CMD_SPR_QUERY)?;
        let values_per_read =
            proto::parse_values_per_read(&resp).ok_or_else(|| bad("SPR:?", &resp))?;
        self.base
            .port_base
            .set_int32_param(p.values_per_read, 0, values_per_read)?;
        self.shared.acq.lock().values_per_read = values_per_read;

        let sample_time = proto::sample_time(values_per_read);
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        let resp = self.write_read_meter(proto::CMD_BIAS_QUERY)?;
        match proto::parse_bias_status(&resp).ok_or_else(|| bad("BIASSTATUS:?", &resp))? {
            None => self.base.port_base.set_int32_param(p.hvs_readback, 0, 0)?,
            Some(volts) => {
                self.base.port_base.set_int32_param(p.hvs_readback, 0, 1)?;
                self.base
                    .port_base
                    .set_float64_param(p.bias_voltage, 0, volts)?;
            }
        }

        // In external-bulb mode the meter delivers one sample per trigger, so
        // the ring buffer averages whatever arrives rather than a fixed count.
        let trigger_mode =
            QeTriggerMode::from_i32(self.base.port_base.get_int32_param(p.trigger_mode, 0)?);
        let averaging_time = self.base.port_base.get_float64_param(p.averaging_time, 0)?;
        let num_average = if trigger_mode == QeTriggerMode::ExtBulb {
            0
        } else {
            num_average_from(averaging_time, sample_time)
        };
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

impl QuadEmDevice for Pcr4Driver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvPCR4::setAcquire`.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 1 && self.shared.is_acquiring() {
            return Ok(());
        }

        if value == 0 {
            // Tell the read thread to stop, then wait for it to leave the read.
            self.shared.set_acquiring(false);
            while self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }

            // Stop the trigger if one is armed, then the acquisition. C++
            // discards both statuses and resynchronises on the ACK below.
            let _ = self.write_read_meter(proto::CMD_TRIGGER_STOP);
            let stopped = self.write_read_meter(proto::CMD_ACQUIRE_STOP);
            let acked = matches!(&stopped, Ok(resp) if resp == proto::ACK);
            if !acked {
                // The ACK is somewhere in the stream of samples still in
                // flight: read until a line ends in it, or until the meter
                // stops answering.
                loop {
                    match self.io.read_line(proto::MAX_COMMAND_LEN) {
                        Ok(outcome) => {
                            if outcome.eom.contains(EomReason::EOS)
                                && proto::ends_with_ack(&outcome.data)
                            {
                                break;
                            }
                            log::warn!(
                                "drvPCR4: waiting for the ACK response, nread={}",
                                outcome.data.len()
                            );
                        }
                        Err(e) => {
                            log::error!("drvPCR4: error waiting for the ACK response: {e}");
                            break;
                        }
                    }
                }
            }

            self.base_set_acquire(0)?;
            return Ok(());
        }

        self.base_set_acquire(1)?;
        // Pushing the settings also flushes any stale input.
        self.set_acquire_params()?;

        let trigger_mode = self
            .base
            .port_base
            .get_int32_param(self.params().trigger_mode, 0)?;
        if trigger_mode == 0 {
            // A triggered acquisition was already started by TRIGGER:START in
            // setAcquireParams; free-run needs the explicit start.
            if let Err(e) = self.io.write(proto::CMD_ACQUIRE_START) {
                self.shared.set_acquiring(false);
                return Err(e);
            }
        }

        // Setting the flag before the signal keeps the read thread from going
        // straight back to sleep; C++ orders it the other way because both
        // sides run under the port lock.
        self.shared.set_acquiring(true);
        self.shared.acquire_start.signal();
        while !self.shared.is_reading_active() {
            thread::sleep(HANDSHAKE_POLL);
        }
        Ok(())
    }

    fn set_acquire_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_averaging_time(&mut self, _value: f64) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_num_channels(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_range(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_trigger_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    fn set_values_per_read(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    /// C++ `drvPCR4::setBiasState`: `"BIAS:ON"`/`"BIAS:OFF"`, followed by the
    /// bias voltage — the meter rejects the voltage while the bias is off, so
    /// it has to be re-sent when the bias comes on.
    fn set_bias_state(&mut self, value: i32) -> AsynResult<()> {
        self.send_command(proto::cmd_bias_state(value != 0))?;
        if value != 0 {
            let volts = self
                .base
                .port_base
                .get_float64_param(self.params().bias_voltage, 0)?;
            self.set_bias_voltage(volts)?;
        }
        Ok(())
    }

    /// C++ `drvPCR4::setBiasVoltage`: `"SETBIAS:%f"`, skipped while the bias
    /// is off because the meter answers with an error.
    fn set_bias_voltage(&mut self, value: f64) -> AsynResult<()> {
        let bias_state = self
            .base
            .port_base
            .get_int32_param(self.params().bias_state, 0)?;
        if bias_state == 0 {
            return Ok(());
        }
        self.send_command(&proto::cmd_bias_voltage(value))
    }

    /// C++ `drvPCR4::readStatus`.
    fn read_status(&mut self) -> AsynResult<()> {
        let trigger_mode = self
            .base
            .port_base
            .get_int32_param(self.params().trigger_mode, 0)?;
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring || trigger_mode != 0 {
            self.set_acquire(0)?;
        }
        if let Err(e) = self.read_status_inner() {
            log::error!("drvPCR4: readStatus failed: {e}");
            return Err(e);
        }
        if prev_acquiring || trigger_mode != 0 {
            self.set_acquire(1)?;
        }
        Ok(())
    }
}

// ===========================================================================
// asyn port driver
// ===========================================================================

impl PortDriver for Pcr4Driver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvPCR4::readEnum`: the range choices are a function of the
    /// firmware revision. An unknown revision is an error, not a guess.
    fn read_enum(&mut self, user: &AsynUser) -> AsynResult<(usize, Arc<[EnumEntry]>)> {
        if user.reason != self.params().range {
            return Err(error("PCR4: readEnum is only defined for QE_RANGE"));
        }
        let Some(ranges) = proto::ranges_for_version(self.version_number) else {
            return Err(error(format!(
                "PCR4: no range choices for firmware revision {}",
                self.version_number
            )));
        };
        let entries: Arc<[EnumEntry]> = ranges
            .iter()
            .enumerate()
            .map(|(i, s)| EnumEntry {
                string: (*s).to_string(),
                value: i as i32,
                severity: 0,
            })
            .collect();
        Ok((entries.len(), entries))
    }

    /// C++ `drvQuadEM::writeInt32` with the PCR4 overrides bound in.
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
            self.set_acquire_mode(value)?;
            self.read_status()?;
        } else if reason == p.geometry {
            self.shared.pos.lock().geometry = value;
        } else if reason == p.bias_state {
            self.set_bias_state(value)?;
            self.read_status()?;
        } else if reason == p.num_channels {
            self.set_num_channels(value)?;
            self.read_status()?;
        } else if reason == p.trigger_mode || reason == p.trigger_polarity {
            // drvPCR4 routes both through setAcquireParams.
            self.set_acquire_params()?;
            self.read_status()?;
        } else if reason == p.num_acquire {
            self.shared.acq.lock().num_acquire = value;
            self.set_acquire_params()?;
            self.read_status()?;
        } else if reason == p.resolution || reason == p.ping_pong || reason == p.bias_interlock {
            // setResolution, setPingPong and setBiasInterlock are drvQuadEM
            // dummies for this driver: only readStatus runs.
            self.read_status()?;
        } else if reason == p.range {
            if channel == 0 {
                self.set_range(value)?;
            }
            self.read_status()?;
        } else if reason == p.read_data {
            self.shared.trigger_callbacks();
        } else if reason == p.values_per_read {
            self.set_values_per_read(value)?;
            self.read_status()?;
        } else if reason == p.read_format {
            // The PCR4 has no binary format; drvPCR4 does not override
            // setReadFormat.
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

    /// C++ `drvQuadEM::writeFloat64` with the PCR4 overrides bound in.
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
            self.set_averaging_time(value)?;
            self.read_status()?;
        } else if reason == p.bias_voltage {
            self.set_bias_voltage(value)?;
            self.read_status()?;
        } else if reason == p.integration_time {
            // The PCR4's integration time is set through SPR; drvPCR4 does not
            // override setIntegrationTime.
            self.read_status()?;
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
    }
}

impl Pcr4Driver {
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

/// C++ `drvPCR4::readThread`.
fn read_thread(ctx: ReadContext) {
    // C++ `nextExpectedEdge`, `numTrigStarts`, `numTrigEnds`: the trigger edges
    // are only counted so that a missing one can be reported.
    let mut next_expected_edge = 0;
    let mut num_trig_starts = 0u64;
    let mut num_trig_ends = 0u64;

    loop {
        if !ctx.shared.is_acquiring() {
            ctx.shared.set_reading_active(false);
            ctx.shared.acquire_start.wait();
            next_expected_edge = 0;
            num_trig_starts = 0;
            num_trig_ends = 0;
            ctx.shared.set_reading_active(true);
        }

        let outcome = match ctx.io.read_line(proto::ASCII_BUFFER_SIZE) {
            Ok(o) => o,
            Err(e) => {
                if !is_timeout(&e) {
                    log::error!("drvPCR4: unexpected error reading meter: {e}");
                    thread::sleep(READ_ERROR_BACKOFF);
                }
                continue;
            }
        };
        if !outcome.eom.contains(EomReason::EOS) {
            log::error!(
                "drvPCR4: unexpected error reading meter, nRead={}, eom={:?}",
                outcome.data.len(),
                outcome.eom
            );
            thread::sleep(READ_ERROR_BACKOFF);
            continue;
        }

        let num_channels = ctx.shared.acq.lock().num_channels;
        match proto::parse_data_line(&outcome.as_str(), num_channels) {
            DataLine::TriggerOn => {
                num_trig_starts += 1;
                if next_expected_edge != 0 {
                    log::error!(
                        "drvPCR4: extra trigger start, numTrigStarts={num_trig_starts}, \
                         numTrigEnds={num_trig_ends}"
                    );
                }
                next_expected_edge = 1;
            }
            DataLine::TriggerOff => {
                num_trig_ends += 1;
                if next_expected_edge != 1 {
                    log::error!(
                        "drvPCR4: extra trigger end, numTrigStarts={num_trig_starts}, \
                         numTrigEnds={num_trig_ends}"
                    );
                }
                next_expected_edge = 0;
            }
            DataLine::Sample(raw) => {
                // The meter has already averaged over SPR values, so the sample
                // is published as it stands.
                ctx.shared.compute_positions(&ctx.handle, &ctx.params, &raw);
            }
        }
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured PCR4 port.
pub struct Pcr4Runtime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    _read_thread: thread::JoinHandle<()>,
    _callback_thread: thread::JoinHandle<()>,
}

impl Pcr4Runtime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// C++ `drvPCR4Configure(portName, QEPortName, ringBufferSize)`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`.
pub fn create_pcr4(
    port_name: &str,
    qe_port_name: &str,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<Pcr4Runtime> {
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);

    let driver = Pcr4Driver::new(port_name, qe_port_name, max_memory, shared.clone())?;
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
        .name("drvPCR4Task".into())
        .spawn(move || read_thread(read_ctx))
        .expect("failed to spawn drvPCR4Task");

    let callback_thread = qe::start_callback_task(CallbackContext {
        trigger_rx,
        handle,
        params,
        nd_params,
        outputs: outputs.clone(),
        shared,
        acquire_param,
    });

    Ok(Pcr4Runtime {
        runtime_handle,
        params,
        nd_params,
        pool,
        outputs,
        _read_thread: read_thread_handle,
        _callback_thread: callback_thread,
    })
}
