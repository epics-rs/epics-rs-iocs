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
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeModel, QeTriggerMode, QeTriggerPolarity,
    QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared, num_average_from,
};
use crate::octet::{OctetIo, connect_octet, outcome, reply_text};
use crate::pcr4_proto::{self as proto, DataLine};
use crate::{asyn_print, trace};

/// Poll interval of C++'s `epicsThreadSleep(0.01)` handshake loop.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `epicsThreadSleep(1.0)` after an unexpected read error.
const READ_ERROR_BACKOFF: Duration = Duration::from_secs(1);
/// C++ `reset`: `epicsThreadSleep(1.0)` between probes.
const RESET_POLL: Duration = Duration::from_secs(1);

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
    /// C++ `QEPortName_`, printed by [`PortDriver::report`].
    qe_port_name: String,
    /// C++ `versionNumber_`, parsed out of the `PCR4v<n>` firmware string and
    /// used only to pick the range enum.
    version_number: i32,
}

impl Pcr4Driver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        (io, status): (OctetIo, AsynResult<()>),
        max_memory: usize,
        shared: Arc<QuadEmShared>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let unlinked = |base, io| Self {
            base,
            io,
            shared: shared.clone(),
            firmware_version: String::new(),
            qe_port_name: qe_port_name.to_string(),
            version_number: 0,
        };
        // C++ returns from its constructor here and keeps the port, with no
        // meter link and no read thread.
        if let Err(e) = status {
            println!(
                "drvPCR4::drvPCR4: error calling pasynOctetSyncIO->connect, status={}, error={}",
                e.c_status_code(),
                e.message_display()
            );
            return Ok(unlinked(base, io));
        }
        // C++ sets the input EOS only in setAcquire, where a refusal fails the
        // acquire; it is set here too so the firmware query frames on it.
        let _ = io.set_input_eos(b"\r\n");

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

        let mut this = unlinked(base, io);

        // C++ constructor calls getFirmwareVersion() under the port lock and
        // ignores its status: the meter may be offline at IOC start.
        let _ = this.get_firmware_version();
        Ok(this)
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    // --- I/O helpers (C++ writeReadMeter / sendCommand) ---

    fn write_read_meter(&self, out: &str) -> AsynResult<String> {
        let (raw, status) = self.io.write_read(out, proto::MAX_COMMAND_LEN).into_parts();
        status.map(|()| reply_text(&raw))
    }

    /// C++ `sendCommand`: stop acquisition around the command, require `ACK`,
    /// then restart.
    fn send_command(&mut self, out: &str) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            let _ = self.set_acquire(0);
        }
        let resp = self.write_read_meter(out)?;
        if resp != proto::ACK {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvPCR4::sendCommand: error, outString={out} expected ACK, received {resp}\n"
            );
            return Err(error(format!("PCR4: {out} -> {resp}, expected ACK")));
        }
        if prev_acquiring {
            let _ = self.set_acquire(1);
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
        let _ = self.set_acquire(0);
        self.firmware_version = "Unknown".into();
        let resp = self.write_read_meter(proto::CMD_VERSION)?;
        match proto::parse_version(&resp) {
            Some((firmware, version)) => {
                self.firmware_version = firmware;
                self.version_number = version;
            }
            None => self.version_number = 0,
        }
        let idx = self.params().firmware;
        self.base
            .port_base
            .set_string_param(idx, 0, self.firmware_version.clone())?;
        Ok(())
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

        let range = qe::int_param(&self.base.port_base, p.range, 0);
        let num_channels = qe::int_param(&self.base.port_base, p.num_channels, 0);
        let trigger_mode = qe::int_param(&self.base.port_base, p.trigger_mode, 0);
        let trigger_polarity =
            QeTriggerPolarity::from_i32(qe::int_param(&self.base.port_base, p.trigger_polarity, 0));
        let values_per_read = qe::int_param(&self.base.port_base, p.values_per_read, 0);
        let averaging_time = qe::double_param(&self.base.port_base, p.averaging_time, 0);

        if prev_acquiring || trigger_mode != 0 {
            let _ = self.set_acquire(0);
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
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    /// C++ `drvPCR4::readStatus`.
    ///
    /// As in the other quadEM ports the `goto error` path skips the
    /// `setAcquire(1)` restore, so a status read that fails leaves the meter
    /// stopped.
    ///
    /// A failed exchange's reply is still parsed ([`trace::Reply::parse`]);
    /// one that does not parse returns its own status. C discards each
    /// `writeReadMeter` status (drvPCR4.cpp:641-663) and reports the empty
    /// reply as asynError (:689), so a disconnected meter alarms WRITE there
    /// and COMM here. C defect, not copied.
    fn read_status_inner(&mut self, last: &mut trace::StatusExchange) -> AsynResult<()> {
        let p = self.params();
        let bad = |what: &str, resp: &str| error(format!("PCR4: bad {what} response: {resp}"));

        let range = last
            .query(&self.io, proto::CMD_RANGE_QUERY, proto::MAX_COMMAND_LEN)
            .parse(proto::parse_range, |r| bad("RANGE:?", r))?;
        self.base.port_base.set_int32_param(p.range, 0, range)?;

        let num_channels = last
            .query(&self.io, proto::CMD_CHANNELS_QUERY, proto::MAX_COMMAND_LEN)
            .parse(proto::parse_num_channels, |r| bad("CHANNELS:?", r))?;
        self.base
            .port_base
            .set_int32_param(p.num_channels, 0, num_channels)?;
        self.shared.acq.lock().num_channels = num_channels;

        let values_per_read = last
            .query(&self.io, proto::CMD_SPR_QUERY, proto::MAX_COMMAND_LEN)
            .parse(proto::parse_values_per_read, |r| bad("SPR:?", r))?;
        self.base
            .port_base
            .set_int32_param(p.values_per_read, 0, values_per_read)?;
        self.shared.acq.lock().values_per_read = values_per_read;

        let sample_time = proto::sample_time(values_per_read);
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        match last
            .query(&self.io, proto::CMD_BIAS_QUERY, proto::MAX_COMMAND_LEN)
            .parse(proto::parse_bias_status, |r| bad("BIASSTATUS:?", r))?
        {
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
            QeTriggerMode::from_i32(qe::int_param(&self.base.port_base, p.trigger_mode, 0));
        let averaging_time = qe::double_param(&self.base.port_base, p.averaging_time, 0);
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
        trace::ensure_input_eos(&self.io, &self.base.port_base, "drvPCR4::setAcquire")?;

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
            // C++ returns the status of the last exchange that looked for it.
            let mut status = stopped.map(drop);
            if !acked {
                // The ACK is somewhere in the stream of samples still in
                // flight: read until a line ends in it, or until the meter
                // stops answering.
                loop {
                    match outcome(self.io.read_line(proto::MAX_COMMAND_LEN)) {
                        (outcome, Ok(())) => {
                            if outcome.eom.contains(EomReason::EOS)
                                && proto::ends_with_ack(&outcome.data)
                            {
                                status = Ok(());
                                break;
                            }
                            asyn_print!(
                                self.base.port_base,
                                WARNING,
                                "drvPCR4::setAcquire waiting for ACK response, nread={}\n",
                                outcome.data.len()
                            );
                        }
                        (_, Err(e)) => {
                            asyn_print!(
                                self.base.port_base,
                                ERROR,
                                "drvPCR4::setAcquire error waiting for ACK response, status={}\n",
                                e.c_status_code()
                            );
                            status = Err(e);
                            break;
                        }
                    }
                }
            }

            self.base_set_acquire(0);
            return status;
        }

        self.base_set_acquire(1);
        // Pushing the settings also flushes any stale input.
        let _ = self.set_acquire_params();

        let trigger_mode = qe::int_param(&self.base.port_base, self.params().trigger_mode, 0);
        // A triggered acquisition was already started by TRIGGER:START in
        // setAcquireParams; free-run needs the explicit start.
        let status = if trigger_mode == 0 {
            self.io.write(proto::CMD_ACQUIRE_START).map(drop)
        } else {
            Ok(())
        };

        // Setting the flag before the signal keeps the read thread from going
        // straight back to sleep; C++ orders it the other way because both
        // sides run under the port lock.
        self.shared.set_acquiring(true);
        self.shared.acquire_start.signal();
        while !self.shared.is_reading_active() {
            thread::sleep(HANDSHAKE_POLL);
        }
        if status.is_err() {
            self.shared.set_acquiring(false);
        }
        status
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
    /// Both commands are sent; C++ returns the last one's status.
    fn set_bias_state(&mut self, value: i32) -> AsynResult<()> {
        let mut status = self.send_command(proto::cmd_bias_state(value != 0));
        if value != 0 {
            let volts = qe::double_param(&self.base.port_base, self.params().bias_voltage, 0);
            status = self.set_bias_voltage(volts);
        }
        status
    }

    /// C++ `drvPCR4::setBiasVoltage`: `"SETBIAS:%f"`, skipped while the bias
    /// is off because the meter answers with an error.
    fn set_bias_voltage(&mut self, value: f64) -> AsynResult<()> {
        let bias_state = qe::int_param(&self.base.port_base, self.params().bias_state, 0);
        if bias_state == 0 {
            return Ok(());
        }
        self.send_command(&proto::cmd_bias_voltage(value))
    }

    /// C++ `drvPCR4::readStatus`.
    fn read_status(&mut self) -> AsynResult<()> {
        let trigger_mode = qe::int_param(&self.base.port_base, self.params().trigger_mode, 0);
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring || trigger_mode != 0 {
            let _ = self.set_acquire(0);
        }
        let mut last = trace::StatusExchange::default();
        if let Err(e) = self.read_status_inner(&mut last) {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvPCR4::getStatus: error, outString={}, inString={}\n",
                last.out,
                last.inp
            );
            return Err(e);
        }
        if prev_acquiring || trigger_mode != 0 {
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    fn set_num_acquire(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_trigger_polarity(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    /// C++ `drvPCR4::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        let _ = self.set_acquire(0);
        // C++ ignores RESET's status: a meter that did not answer it is still
        // waited for, and reported when it never comes back.
        let _ = self.send_command(proto::CMD_RESET);
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
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvPCR4::reset: error, no response from meter after {} seconds\n",
                proto::RESET_WAIT_LOOPS
            );
        }
        self.base_reset()
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

    /// C++ `drvPCR4::report`: above details 0 the meter is stopped, asked for
    /// its NETCONFIG, and restarted after the base report. `numResync_` is
    /// never incremented by C++ (drvPCR4.cpp:78 is its only store), so it
    /// prints 0.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "drvPCR4: port={}, IP port={}, firmware version={}, numResync=0",
            self.base.port_base.port_name, self.qe_port_name, self.firmware_version
        );
        let mut prev_acquiring = false;
        if level > 0 {
            prev_acquiring = self.shared.is_acquiring();
            let _ = self.set_acquire(0);
            // C++ prints `inString_` whatever the read's status.
            let (raw, _) = self
                .io
                .write_read(proto::CMD_NETCONFIG, proto::MAX_COMMAND_LEN)
                .into_parts();
            let reply = reply_text(&raw);
            let _ = writeln!(out, "NETCONFIG response from meter:\n{reply}");
        }
        self.asyn_port_driver_report(out, level);
        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.qe_write_int32(user, value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.qe_write_float64(user, value)
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
}

impl Pcr4Driver {}

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

        let outcome = match outcome(ctx.io.locked_read_line(proto::ASCII_BUFFER_SIZE)) {
            (o, Ok(())) if o.eom.contains(EomReason::EOS) => o,
            (o, status) => {
                if trace::read_result(&o, &status).0 != AsynStatus::Timeout {
                    trace::print_read_error(&ctx.handle, "drvPCR4::readThread", &o, &status);
                    thread::sleep(READ_ERROR_BACKOFF);
                }
                continue;
            }
        };

        let num_channels = ctx.shared.acq.lock().num_channels;
        match proto::parse_data_line(&outcome.as_str(), num_channels) {
            DataLine::TriggerOn => {
                num_trig_starts += 1;
                if next_expected_edge != 0 {
                    asyn_print!(
                        ctx.handle,
                        ERROR,
                        "drvPCR4::readThread Extra trigger start, numTrigStarts={num_trig_starts}, \
                         numTrigsEnds={num_trig_ends}\n"
                    );
                }
                next_expected_edge = 1;
            }
            DataLine::TriggerOff => {
                num_trig_ends += 1;
                if next_expected_edge != 1 {
                    asyn_print!(
                        ctx.handle,
                        ERROR,
                        "drvPCR4::readThread Extra trigger end, numTrigStarts={num_trig_starts}, \
                         numTrigsEnds={num_trig_ends}\n"
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
    pub pool: epics_rs::ad_core::ndarray_pool::DriverPool,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    /// `None` when the constructor found no QE port.
    _read_thread: Option<thread::JoinHandle<()>>,
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
    claim: &PortClaim,
    qe_port_name: &str,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<Pcr4Runtime> {
    let port_name = claim.name();
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);

    let driver = Pcr4Driver::new(
        port_name,
        qe_port_name,
        connect_octet(qe_port_name, proto::PCR4_TIMEOUT),
        max_memory,
        shared.clone(),
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
    };
    let read_thread_handle = read_ctx.io.handle().is_some().then(|| {
        thread::Builder::new()
            .name("drvPCR4Task".into())
            .spawn(move || read_thread(read_ctx))
            .expect("failed to spawn drvPCR4Task")
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::octet::create_ip_port;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::base::runtime::interrupt_accept::InterruptGate;

    /// A meter that answers every `\r`-terminated command with `ACK`.
    fn ack_meter() -> (String, Arc<std::sync::Mutex<Vec<String>>>) {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let log = Arc::new(std::sync::Mutex::new(Vec::new()));
        let seen = log.clone();
        thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut out = stream.try_clone().unwrap();
            let mut reader = BufReader::new(stream);
            let mut line = Vec::new();
            while reader.read_until(b'\r', &mut line).unwrap_or(0) > 0 {
                seen.lock()
                    .unwrap()
                    .push(String::from_utf8_lossy(&line).trim().to_string());
                line.clear();
                let _ = out.write_all(b"ACK\r\n");
            }
        });
        (addr, log)
    }

    #[test]
    fn a_detailed_report_stops_the_meter_and_asks_its_netconfig() {
        let (addr, log) = ack_meter();
        let (io, runtime) = create_ip_port(
            "QE_PCR4_NET_IP",
            &addr,
            Duration::from_millis(200),
            false,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        std::mem::forget(runtime);
        io.set_output_eos(b"\r").unwrap();
        let mut drv = Pcr4Driver::new(
            "QE_PCR4_NET",
            "QE_PCR4_NET",
            (io, Ok(())),
            0,
            QuadEmShared::new(16).0,
        )
        .unwrap();
        let mut out = String::new();
        drv.report(&mut out, 1);
        assert!(
            out.contains("NETCONFIG response from meter:\nACK\n"),
            "{out}"
        );
        // The constructor's firmware query comes first.
        let stop = [
            proto::CMD_TRIGGER_STOP,
            proto::CMD_ACQUIRE_STOP,
            "NETCONFIG",
        ];
        assert!(
            log.lock().unwrap().ends_with(&stop.map(String::from)),
            "{:?}",
            log.lock().unwrap()
        );
    }

    /// C++ prints the connect failure and returns from its constructor: the
    /// port stays, with no read thread.
    #[test]
    fn a_missing_qe_port_keeps_the_port_without_a_link() {
        let rt = create_pcr4(
            &claim_port("QE_PCR4_NOQE").unwrap(),
            "QE_PCR4_NOQE_IP",
            4,
            0,
        )
        .unwrap();
        assert!(rt._read_thread.is_none());
    }

    /// A down meter link fails a write with the link's asynDisconnected, not
    /// C's asynError from parsing an empty status reply.
    #[test]
    fn a_status_read_on_a_down_meter_keeps_the_link_status() {
        let (_io, _meter) = create_ip_port(
            "QE_PCR4_DOWN_IP",
            "127.0.0.1:1",
            Duration::from_millis(200),
            false,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        let rt = create_pcr4(
            &claim_port("QE_PCR4_DOWN").unwrap(),
            "QE_PCR4_DOWN_IP",
            4,
            0,
        )
        .unwrap();
        // BiasState is 0, so only readStatus talks to the meter.
        let err = rt
            .port_handle()
            .write_float64_blocking(rt.params.bias_voltage, 0, 0.0)
            .unwrap_err();
        assert_eq!(err.status(), AsynStatus::Disconnected, "{err:?}");
    }

    /// C ends each `asynPrint` line with `\n` (drvPCR4.cpp:686-688).
    #[test]
    fn a_status_read_error_ends_its_line() {
        let (_io, _meter) = create_ip_port(
            "QE_PCR4_EOL_IP",
            "127.0.0.1:1",
            Duration::from_millis(200),
            false,
            true,
            &InterruptGate::opened(),
        )
        .unwrap();
        let rt = create_pcr4(&claim_port("QE_PCR4_EOL").unwrap(), "QE_PCR4_EOL_IP", 4, 0).unwrap();
        let handle = rt.port_handle();
        let path = std::env::temp_dir().join(format!("qe-pcr4-eol-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        handle.trace().set_trace_file(
            Some(handle.port_name()),
            epics_rs::asyn::trace::TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        let _ = handle.write_float64_blocking(rt.params.bias_voltage, 0, 0.0);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.ends_with("drvPCR4::getStatus: error, outString=RANGE:?, inString=\n"),
            "{text:?}"
        );
    }

    /// C++ sets the input EOS only in setAcquire (drvPCR4.cpp:356-363): a port
    /// that refuses it keeps the driver and fails the acquire instead.
    #[test]
    fn an_input_eos_refusal_fails_the_acquire_not_the_port() {
        let (_meter, _runtime) = crate::octet::tests::eos_refusing_port("QE_PCR4_NOEOS_IP");
        let rt = create_pcr4(
            &claim_port("QE_PCR4_NOEOS").unwrap(),
            "QE_PCR4_NOEOS_IP",
            4,
            0,
        )
        .unwrap();
        let err = rt
            .port_handle()
            .write_int32_blocking(rt.nd_params.acquire, 0, 1)
            .unwrap_err();
        assert_eq!(err.status(), epics_rs::asyn::error::AsynStatus::Error);
        assert_eq!(
            err.message(),
            format!(
                "drvQuadEM:writeInt32: status=3, function={}, name=ACQUIRE, value=1",
                rt.nd_params.acquire
            )
        );
    }
}
