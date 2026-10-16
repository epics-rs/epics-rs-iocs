//! Port of `quadEMApp/caenSrc/drvTetrAMM.{h,cpp}` — CaenEls TetrAMM
//! 4-channel picoammeter, reached over TCP or serial through an asyn octet
//! port.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::interpose::EomReason;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeReadFormat,
    QeTriggerMode, QeTriggerPolarity, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared,
};
use crate::octet::{OctetIo, ReadOutcome, connect_octet, reply_text};
use crate::tetramm_proto as proto;
use crate::{asyn_print, trace};

/// C++ `MAX_COMMAND_LEN`.
const MAX_COMMAND_LEN: usize = 256;
/// C++ `P_InterlockStatusString`.
const P_INTERLOCK_STATUS: &str = "TETRAMM_INTERLOCK_STATUS";
/// Poll interval of C++'s `epicsThreadSleep(0.01)` handshake loops.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `reset()` waits this many one-second loops for the meter to reappear.
const RESET_WAIT_LOOPS: u32 = 20;

// ===========================================================================
// Driver
// ===========================================================================

pub struct TetrAmmDriver {
    base: QuadEmBase,
    io: OctetIo,
    shared: Arc<QuadEmShared>,
    p_interlock_status: usize,
    firmware_version: String,
    /// C++ `QEPortName_`, printed by [`PortDriver::report`].
    qe_port_name: String,
    /// C++ `numResync_`: the read thread counts, the report prints.
    num_resync: Arc<AtomicU64>,
}

impl TetrAmmDriver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        (io, status): (OctetIo, AsynResult<()>),
        max_memory: usize,
        shared: Arc<QuadEmShared>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let p_interlock_status = base
            .port_base
            .create_param(P_INTERLOCK_STATUS, epics_rs::asyn::param::ParamType::Int32)?;
        let unlinked = |base, io| Self {
            base,
            io,
            shared: shared.clone(),
            p_interlock_status,
            firmware_version: String::new(),
            qe_port_name: qe_port_name.to_string(),
            num_resync: Arc::new(AtomicU64::new(0)),
        };
        // C++ returns from its constructor here and keeps the port, with no
        // meter link and no read thread.
        if let Err(e) = status {
            println!(
                "drvTetrAMM::drvTetrAMM: error calling pasynOctetSyncIO->connect, status={}, error={}",
                trace::status_code(&e),
                e.message_display()
            );
            return Ok(unlinked(base, io));
        }

        // C++ sets the input EOS only in setAcquire, where a refusal fails the
        // acquire; it is set here too so the firmware query frames on it.
        let _ = io.set_input_eos(b"\r\n");

        // C++ constructor: resolution_ = 24, model = TetrAMM, valuesPerRead = 5.
        base.port_base
            .set_int32_param(base.params.model, 0, QeModel::TetrAmm as i32)?;
        base.port_base
            .set_int32_param(base.params.values_per_read, 0, 5)?;
        base.port_base
            .set_int32_param(base.params.resolution, 0, 24)?;
        {
            let mut acq = shared.acq.lock();
            acq.resolution = 24;
            acq.values_per_read = 5;
            acq.num_channels = 4;
        }

        let mut this = unlinked(base, io);

        // C++ calls getFirmwareVersion() from the constructor, ignoring its
        // status; reset() is left for later because the meter may be offline
        // at IOC start.
        let _ = this.get_firmware_version();
        Ok(this)
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    // --- I/O helpers (C++ writeReadMeter / sendCommand) ---

    fn write_read_meter(&self, out: &str) -> AsynResult<String> {
        self.io.write_read(out, MAX_COMMAND_LEN)
    }

    /// C++ `sendCommand`: stop acquisition around the command, require `ACK`.
    /// As in C++, only the command's own status is returned.
    fn send_command(&mut self, out: &str) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            let _ = self.set_acquire(0);
        }
        let resp = self.write_read_meter(out)?;
        if resp != "ACK" {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvTetrAMM::sendCommand: error, outString={out} expected ACK, received {resp}"
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("TetrAMM: {out} -> {resp}, expected ACK"),
            });
        }
        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    fn get_firmware_version(&mut self) -> AsynResult<()> {
        self.firmware_version = "Unknown".into();
        let _ = self.set_acquire(0);
        let resp = self.write_read_meter("VER:?")?;
        self.firmware_version = proto::parse_version(&resp).to_string();
        let idx = self.params().firmware;
        self.base
            .port_base
            .set_string_param(idx, 0, self.firmware_version.clone())?;
        Ok(())
    }
}

// ===========================================================================
// drvQuadEM virtuals
// ===========================================================================

impl QuadEmDevice for TetrAmmDriver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvTetrAMM::setAcquire`. Stopping returns the status of the last
    /// exchange that looked for the `ACK`; starting returns the `ACQ:ON` write's
    /// in ASCII mode only, because C++ overwrites it in binary mode with the
    /// input-EOS call's.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 1 && self.shared.is_acquiring() {
            return Ok(());
        }
        trace::ensure_input_eos(&self.io, &self.base.port_base, "drvTetrAMM::setAcquire")?;

        if value == 0 {
            // Tell the read thread to stop, then wait for it to leave the loop.
            self.shared.set_acquiring(false);
            while self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
            let resp = self.write_read_meter("ACQ:OFF");
            let acked = matches!(&resp, Ok(r) if r == "ACK");
            let mut status = resp.map(drop);
            if !acked {
                // Drain until a response ends on EOS with a trailing ACK, or the
                // read times out.
                loop {
                    match self.io.read_line(MAX_COMMAND_LEN) {
                        Ok(outcome) => {
                            let s = outcome.as_str();
                            if outcome.eom.contains(EomReason::EOS) && s.ends_with("ACK") {
                                status = Ok(());
                                break;
                            }
                            asyn_print!(
                                self.base.port_base,
                                WARNING,
                                "drvTetrAMM::setAcquire waiting for ACK response, nread={}",
                                outcome.data.len()
                            );
                        }
                        Err(e) => {
                            asyn_print!(
                                self.base.port_base,
                                ERROR,
                                "drvTetrAMM::setAcquire error waiting for ACK response, status={}",
                                trace::status_code(&e)
                            );
                            status = Err(e);
                            break;
                        }
                    }
                }
            }
            let _ = self.base_set_acquire(0);
            status
        } else {
            let _ = self.base_set_acquire(1);
            // C++ calls setAcquireParams() here: it is needed before NAQ and
            // also flushes any stale input.
            let _ = self.set_acquire_params();
            let written = self.io.write("ACQ:ON").map(drop);
            let read_format = self
                .base
                .port_base
                .get_int32_param(self.params().read_format, 0)?;
            let status = match QeReadFormat::from_i32(read_format) {
                QeReadFormat::Binary => Ok(()),
                _ => written,
            };
            self.shared.acquire_start.signal();
            while !self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
            if status.is_err() {
                self.shared.set_acquiring(false);
            }
            status
        }
    }

    /// C++ `drvTetrAMM::setRange` writes the range to all four channels.
    fn set_range(&mut self, value: i32) -> AsynResult<()> {
        for i in 0..QE_MAX_INPUTS {
            let idx = self.params().range;
            self.base
                .port_base
                .set_int32_param(idx, i as i32 + 1, value)?;
        }
        self.set_acquire_params()
    }

    fn set_values_per_read(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_averaging_time(&mut self, _value: f64) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_trigger_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_num_channels(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_read_format(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    /// C++ `setBiasState`: reset latched faults, switch the supply, and
    /// re-send the voltage (it is rejected while the bias is off).
    /// Every command is sent; C++ returns the last one's status.
    fn set_bias_state(&mut self, value: i32) -> AsynResult<()> {
        let _ = self.send_command("STATUS:RESET");
        let mut status = self.send_command(proto::cmd_bias_state(value != 0));
        if value != 0 {
            let bv = self
                .base
                .port_base
                .get_float64_param(self.params().bias_voltage, 0)?;
            status = self.set_bias_voltage(bv);
        }
        status
    }

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

    fn set_bias_interlock(&mut self, value: i32) -> AsynResult<()> {
        self.send_command(proto::cmd_bias_interlock(value != 0))
    }

    /// C++ `drvTetrAMM::readStatus`.
    ///
    /// An unparseable response takes C++'s `goto error`, which jumps past the
    /// `setAcquire(1)` restore: a failed status read leaves the meter stopped.
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
                "drvTetrAMM::getStatus: error, outString={}, inString={}",
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

    fn set_acquire_mode(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_num_acquire(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_trigger_polarity(&mut self, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }
    fn set_channel_range(&mut self, _channel: i32, _value: i32) -> AsynResult<()> {
        self.set_acquire_params()
    }

    /// C++ `drvTetrAMM::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        let _ = self.set_acquire(0);
        let _ = self.send_command("HWRESET");
        let mut came_back = false;
        for _ in 0..RESET_WAIT_LOOPS {
            thread::sleep(Duration::from_secs(1));
            if self.write_read_meter("VER:?").is_ok() {
                came_back = true;
                break;
            }
        }
        if !came_back {
            asyn_print!(
                self.base.port_base,
                ERROR,
                "drvTetrAMM::reset: error, no response from meter after {RESET_WAIT_LOOPS} seconds"
            );
        }
        self.base_reset()
    }
}

impl TetrAmmDriver {
    /// C++ `drvTetrAMM::setAcquireParams`.
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            let _ = self.set_acquire(0);
        }

        let p = self.params();
        let base = &mut self.base.port_base;
        let num_channels = base.get_int32_param(p.num_channels, 0)?;
        let mut range = [0i32; QE_MAX_INPUTS];
        for (i, r) in range.iter_mut().enumerate() {
            *r = base.get_int32_param(p.range, i as i32 + 1)?;
        }
        let trigger_mode = QeTriggerMode::from_i32(base.get_int32_param(p.trigger_mode, 0)?);
        let trigger_polarity =
            QeTriggerPolarity::from_i32(base.get_int32_param(p.trigger_polarity, 0)?);
        let acquire_mode = QeAcquireMode::from_i32(base.get_int32_param(p.acquire_mode, 0)?);
        let values_per_read = base.get_int32_param(p.values_per_read, 0)?;
        let read_format = QeReadFormat::from_i32(base.get_int32_param(p.read_format, 0)?);
        let averaging_time = base.get_float64_param(p.averaging_time, 0)?;
        let num_acquire = base.get_int32_param(p.num_acquire, 0)?;

        // C++ computes sampleTime from the *unclamped* valuesPerRead; only the
        // NRSAMP argument gets clamped below.
        let sample_time = proto::sample_time(values_per_read);
        base.set_float64_param(p.sample_time, 0, sample_time)?;

        let num_average = proto::num_average(trigger_mode, averaging_time, sample_time);
        base.set_int32_param(p.num_average, 0, num_average)?;
        {
            let mut acq = self.shared.acq.lock();
            acq.num_average = num_average;
            acq.acquire_mode = acquire_mode as i32;
            acq.num_acquire = num_acquire;
            acq.trigger_mode = trigger_mode as i32;
            acq.read_format = read_format as i32;
            acq.num_channels = num_channels;
            acq.values_per_read = values_per_read;
        }

        for (i, r) in range.iter().enumerate() {
            let _ = self.write_read_meter(&proto::cmd_range(i as i32 + 1, *r));
        }
        let _ = self.write_read_meter(&proto::cmd_num_channels(num_channels));
        let _ = self.write_read_meter(proto::cmd_read_format(read_format));

        let nrsamp = proto::clamp_values_per_read(values_per_read, read_format);
        let _ = self.write_read_meter(&proto::cmd_nrsamp(nrsamp));
        let _ = self.write_read_meter(proto::cmd_trigger(trigger_mode));
        let _ = self.write_read_meter(proto::cmd_trigger_polarity(trigger_polarity));

        let naq = proto::naq_value(trigger_mode, acquire_mode, num_average);
        let _ = self.write_read_meter(&proto::cmd_naq(naq));
        let ntrg = proto::ntrg_value(acquire_mode, num_acquire);
        let _ = self.write_read_meter(&proto::cmd_ntrg(ntrg));

        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
        Ok(())
    }

    fn read_status_inner(&mut self, last: &mut trace::StatusExchange) -> AsynResult<()> {
        let p = self.params();
        let err = || AsynError::Status {
            status: AsynStatus::Error,
            message: "TetrAMM: unparseable status response".into(),
        };

        let resp = last.query(&self.io, "CHN:?", MAX_COMMAND_LEN)?;
        let num_channels = proto::parse_chn(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_int32_param(p.num_channels, 0, num_channels)?;
        self.shared.acq.lock().num_channels = num_channels;

        for i in 0..QE_MAX_INPUTS as i32 {
            let resp = last.query(&self.io, &proto::cmd_range_query(i + 1), MAX_COMMAND_LEN)?;
            let range = proto::parse_range(&resp).ok_or_else(err)?;
            self.base.port_base.set_int32_param(p.range, i + 1, range)?;
            if i == 0 {
                self.base.port_base.set_int32_param(p.range, 0, range)?;
            }
        }

        let resp = last.query(&self.io, "NRSAMP:?", MAX_COMMAND_LEN)?;
        let values_per_read = proto::parse_nrsamp(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_int32_param(p.values_per_read, 0, values_per_read)?;
        self.shared.acq.lock().values_per_read = values_per_read;

        let sample_time = proto::sample_time(values_per_read);
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        let resp = last.query(&self.io, "HVS:?", MAX_COMMAND_LEN)?;
        match proto::parse_hvs(&resp).ok_or_else(err)? {
            None => self.base.port_base.set_int32_param(p.hvs_readback, 0, 0)?,
            Some(volts) => {
                self.base.port_base.set_int32_param(p.hvs_readback, 0, 1)?;
                self.base
                    .port_base
                    .set_float64_param(p.bias_voltage, 0, volts)?;
            }
        }

        let resp = last.query(&self.io, "HVV:?", MAX_COMMAND_LEN)?;
        let v = proto::parse_hvv(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_float64_param(p.hvv_readback, 0, v)?;

        let resp = last.query(&self.io, "HVI:?", MAX_COMMAND_LEN)?;
        let v = proto::parse_hvi(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_float64_param(p.hvi_readback, 0, v)?;

        let resp = last.query(&self.io, "TEMP:?", MAX_COMMAND_LEN)?;
        let v = proto::parse_temp(&resp).ok_or_else(err)?;
        self.base.port_base.set_float64_param(p.temperature, 0, v)?;

        let resp = last.query(&self.io, "STATUS:?", MAX_COMMAND_LEN)?;
        let unit_status = proto::parse_status(&resp).ok_or_else(err)?;
        let interlock = proto::interlock_status(unit_status);
        self.base
            .port_base
            .set_int32_param(self.p_interlock_status, 0, interlock)?;

        let averaging_time = self.base.port_base.get_float64_param(p.averaging_time, 0)?;
        let trigger_mode =
            QeTriggerMode::from_i32(self.base.port_base.get_int32_param(p.trigger_mode, 0)?);
        let num_average = proto::num_average(trigger_mode, averaging_time, sample_time);
        self.base
            .port_base
            .set_int32_param(p.num_average, 0, num_average)?;
        self.shared.acq.lock().num_average = num_average;
        Ok(())
    }
}

impl PortDriver for TetrAmmDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvTetrAMM::report`: above details 0 the meter is stopped, asked
    /// for its network setup, and restarted after the base report.
    fn report(&mut self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "drvTetrAMM: port={}, IP port={}, firmware version={}, numResync={}",
            self.base.port_base.port_name,
            self.qe_port_name,
            self.firmware_version,
            self.num_resync.load(Ordering::Relaxed)
        );
        let mut prev_acquiring = false;
        if level > 0 {
            prev_acquiring = self.shared.is_acquiring();
            let _ = self.set_acquire(0);
            for (command, label) in [
                ("IFCONFIG", "IFCONFIG response from meter:"),
                ("IFCONFIG:LINK", "IFCONFIG:LINK: response from meter:"),
                ("IFCONFIG:ICMP", "IFCONFIG:ICMP: response from meter:"),
            ] {
                let reply = reply_text(self.write_read_meter(command));
                let _ = writeln!(out, "{label}\n{reply}");
            }
        }
        self.asyn_port_driver_report(out, level);
        if prev_acquiring {
            let _ = self.set_acquire(1);
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.qe_write_int32(user.reason, user.addr, value)
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.qe_write_float64(user.reason, user.addr, value)
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
    num_resync: Arc<AtomicU64>,
}

/// C++ `drvTetrAMM::readThread`.
fn read_thread(ctx: ReadContext) {
    let mut trigger_mode = QeTriggerMode::FreeRun;
    let mut read_format = QeReadFormat::Binary;
    let mut next_expected_edge = 0;
    let mut num_trig_starts = 0i64;
    let mut num_trig_ends = 0i64;

    loop {
        if !ctx.shared.is_acquiring() {
            ctx.shared.set_reading_active(false);
            ctx.shared.acquire_start.wait();
            ctx.shared.set_acquiring(true);
            num_trig_ends = 0;
            num_trig_starts = 0;
            next_expected_edge = 0;
            {
                let acq = ctx.shared.acq.lock();
                trigger_mode = QeTriggerMode::from_i32(acq.trigger_mode);
                read_format = QeReadFormat::from_i32(acq.read_format);
            }
            ctx.shared.set_reading_active(true);
        }

        let num_channels = (ctx.shared.acq.lock().num_channels as usize).clamp(1, QE_MAX_INPUTS);

        match read_format {
            QeReadFormat::Binary => {
                let n_requested = proto::binary_frame_len(num_channels);
                let read = ctx.io.locked_read_binary(n_requested);
                let outcome = match read {
                    Ok(o) if o.data.len() == n_requested && o.eom.contains(EomReason::CNT) => o,
                    _ => {
                        report_read_failure(&ctx.handle, &read, true);
                        continue;
                    }
                };

                let Some(frame) = proto::parse_binary_frame(&outcome.data, num_channels) else {
                    continue;
                };
                match frame.trailer {
                    proto::Trailer::Data => {
                        ctx.shared
                            .compute_positions(&ctx.handle, &ctx.params, &frame.currents);
                    }
                    proto::Trailer::TriggerStart => {
                        num_trig_starts += 1;
                        if next_expected_edge != 0 {
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvTetrAMM::readThread Extra trigger start, numTrigStarts={num_trig_starts}, numTrigsEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 1;
                    }
                    proto::Trailer::TriggerEnd => {
                        num_trig_ends += 1;
                        if trigger_mode == QeTriggerMode::ExtBulb {
                            ctx.shared.trigger_callbacks(&ctx.handle);
                        }
                        if next_expected_edge != 1 {
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvTetrAMM::readThread Extra trigger end, numTrigStarts={num_trig_starts}, numTrigsEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 0;
                    }
                    proto::Trailer::AcqDone => {
                        asyn_print!(
                            ctx.handle,
                            IO_DRIVER,
                            "drvTetrAMM::readThread: seen acq done sNaN (0xfff40003ffffffffull)"
                        );
                    }
                    proto::Trailer::LostSync => {
                        asyn_print!(
                            ctx.handle,
                            WARNING,
                            "drvTetrAMM::readThread: warning, lost sync, no NaN where expected; resynchronizing"
                        );
                        resync(&ctx, num_channels);
                    }
                }
            }
            QeReadFormat::Ascii => {
                let read = ctx.io.locked_read_line(proto::ASCII_BUFFER_SIZE);
                let outcome = match read {
                    Ok(o) if o.eom.contains(EomReason::EOS) => o,
                    _ => {
                        report_read_failure(&ctx.handle, &read, false);
                        continue;
                    }
                };
                match proto::parse_ascii_line(&outcome.as_str(), num_channels) {
                    proto::AsciiRecord::TriggerStart => {
                        num_trig_starts += 1;
                        if next_expected_edge != 0 {
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvTetrAMM::readThread Extra trigger start, numTrigStarts={num_trig_starts}, numTrigsEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 1;
                    }
                    proto::AsciiRecord::TriggerEnd => {
                        num_trig_ends += 1;
                        if trigger_mode == QeTriggerMode::ExtBulb {
                            ctx.shared.trigger_callbacks(&ctx.handle);
                        }
                        if next_expected_edge != 1 {
                            asyn_print!(
                                ctx.handle,
                                ERROR,
                                "drvTetrAMM::readThread Extra trigger end, numTrigStarts={num_trig_starts}, numTrigsEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 0;
                    }
                    proto::AsciiRecord::Data(currents) => {
                        ctx.shared
                            .compute_positions(&ctx.handle, &ctx.params, &currents);
                    }
                }
            }
        }

        let _ = ctx.handle.set_params_and_notify_blocking(0, Vec::new());
    }
}

/// C++ `readThread`'s rejected-read branch. A timeout is routine only on the
/// binary path; C++ sleeps a second after anything else, the meter being
/// probably offline.
fn report_read_failure(handle: &PortHandle, read: &AsynResult<ReadOutcome>, binary: bool) {
    if trace::read_result(read).0 == AsynStatus::Timeout {
        if binary {
            asyn_print!(
                *handle,
                WARNING,
                "drvTetrAMM::readThread: timeout reading meter"
            );
        }
        return;
    }
    trace::print_read_error(handle, "drvTetrAMM::readThread", read);
    thread::sleep(Duration::from_secs(1));
}

/// C++ resync: read two frames' worth, find the data trailer, then consume the
/// bytes that follow it so the next read lands on a frame boundary — both reads
/// under one `lockPort`. A failed read is scanned for what it did transfer, as
/// C++ scans its buffer regardless of the read status.
fn resync(ctx: &ReadContext, num_channels: usize) {
    let doubled = proto::binary_frame_len(num_channels) * 2;
    let locked = ctx.io.locked(move |port| {
        let data = match port.read_binary(doubled) {
            Ok(outcome) => outcome.data,
            Err(e) => e.partial_read().map(|p| p.data.clone()).unwrap_or_default(),
        };
        let found = proto::find_resync_offset(&data).map(|offset| {
            let remainder = proto::resync_remainder(num_channels, offset);
            let _ = port.read_binary(remainder);
            (offset, remainder)
        });
        (data, found)
    });
    let (data, found) = locked.unwrap_or_default();
    match found {
        Some((offset, remainder)) => {
            ctx.num_resync.fetch_add(1, Ordering::Relaxed);
            asyn_print!(
                ctx.handle,
                WARNING,
                "drvTetrAMM::readThread: found NaN at position {offset}, read {remainder} bytes"
            );
        }
        None => {
            asyn_print!(
                ctx.handle,
                ERROR,
                "drvTetrAMM::readThread: ERROR, did not find NaN"
            );
            trace::print_io(
                &ctx.handle,
                trace::TraceMask::ERROR,
                &data,
                format_args!("drvTetrAMM::readThread: buffer read when resynchronizing"),
            );
        }
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured TetrAMM port.
pub struct TetrAmmRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    /// `None` when the constructor found no QE port.
    _read_thread: Option<thread::JoinHandle<()>>,
    _callback_thread: thread::JoinHandle<()>,
}

impl TetrAmmRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

/// C++ `drvTetrAMMConfigure(portName, QEPortName, ringBufferSize)`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`.
pub fn create_tetramm(
    port_name: &str,
    qe_port_name: &str,
    ring_buffer_size: usize,
    max_memory: usize,
) -> AsynResult<TetrAmmRuntime> {
    let (shared, trigger_rx) = QuadEmShared::new(ring_buffer_size);

    let driver = TetrAmmDriver::new(
        port_name,
        qe_port_name,
        connect_octet(qe_port_name, proto::TETRAMM_TIMEOUT),
        max_memory,
        shared.clone(),
    )?;
    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let io = driver.io.clone();
    let num_resync = driver.num_resync.clone();
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default())?;
    let handle = runtime_handle.port_handle().clone();

    let read_ctx = ReadContext {
        io,
        handle: handle.clone(),
        params,
        shared: shared.clone(),
        num_resync,
    };
    let read_thread_handle = read_ctx.io.handle().is_some().then(|| {
        thread::Builder::new()
            .name("drvTetrAMMTask".into())
            .spawn(move || read_thread(read_ctx))
            .expect("failed to spawn drvTetrAMMTask")
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

    Ok(TetrAmmRuntime {
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
    use epics_rs::asyn::trace::TraceFile;

    /// A meter that went away is reported on the driver port's trace at
    /// `ASYN_TRACE_ERROR`, which C prints by default.
    #[test]
    fn read_failure_prints_cs_error_line_on_the_port_trace() {
        let (io, runtime) = create_ip_port(
            "QE_TETRA_TRACE",
            "127.0.0.1:1",
            Duration::from_millis(200),
            true,
            false,
        )
        .unwrap();
        let handle = runtime.port_handle();
        let path = std::env::temp_dir().join(format!("qe-tetra-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        handle.trace().set_trace_file(
            Some(handle.port_name()),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );

        let read = io.read_binary(proto::binary_frame_len(4));
        assert!(read.is_err());
        report_read_failure(handle, &read, true);

        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.ends_with(
                "drvTetrAMM::readThread: unexpected error reading meter \
                 status=4, nRead=0, eomReason=0\n"
            ),
            "{text:?}"
        );
    }

    /// A meter on a local socket: every command is logged, `nak` is answered
    /// with `NAK`, everything else with `ACK`.
    fn ack_meter(nak: &'static str) -> (String, Arc<std::sync::Mutex<Vec<String>>>) {
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
                let cmd = String::from_utf8_lossy(&line).trim().to_string();
                line.clear();
                let reply = if cmd == nak { "NAK\r\n" } else { "ACK\r\n" };
                seen.lock().unwrap().push(cmd);
                let _ = out.write_all(reply.as_bytes());
            }
        });
        (addr, log)
    }

    fn driver_on(name: &str, host: &str) -> TetrAmmDriver {
        let (io, runtime) = create_ip_port(
            &format!("{name}_IP"),
            host,
            Duration::from_millis(200),
            false,
            true,
        )
        .unwrap();
        // The port runtime must outlive the test; the driver only holds an
        // octet user on it.
        std::mem::forget(runtime);
        // st.cmd's asynOctetSetOutputEos; the driver sets only the input EOS.
        io.set_output_eos(b"\r").unwrap();
        TetrAmmDriver::new(name, name, (io, Ok(())), 0, QuadEmShared::new(16).0).unwrap()
    }

    #[test]
    fn report_prints_cs_line_then_the_base_report() {
        let mut drv = driver_on("QE_TETRA_REPORT", "127.0.0.1:1");
        drv.num_resync.fetch_add(3, Ordering::Relaxed);
        let mut out = String::new();
        drv.report(&mut out, 0);
        assert_eq!(
            out,
            "drvTetrAMM: port=QE_TETRA_REPORT, IP port=QE_TETRA_REPORT, \
             firmware version=Unknown, numResync=3\nPort: QE_TETRA_REPORT\n"
        );
    }

    #[test]
    fn a_detailed_report_stops_the_meter_and_asks_its_network_setup() {
        let (addr, log) = ack_meter("");
        let mut drv = driver_on("QE_TETRA_IFCONFIG", &addr);
        let mut out = String::new();
        drv.report(&mut out, 1);
        assert!(
            out.contains(
                "IFCONFIG response from meter:\nACK\n\
                 IFCONFIG:LINK: response from meter:\nACK\n\
                 IFCONFIG:ICMP: response from meter:\nACK\n"
            ),
            "{out}"
        );
        // The constructor's ACQ:OFF/VER:? come first.
        assert!(
            log.lock().unwrap().ends_with(
                &["ACQ:OFF", "IFCONFIG", "IFCONFIG:LINK", "IFCONFIG:ICMP"].map(String::from)
            ),
            "{:?}",
            log.lock().unwrap()
        );
    }

    #[test]
    fn stopping_a_dead_meter_fails_the_acquire_write() {
        let mut drv = driver_on("QE_TETRA_DEAD", "127.0.0.1:1");
        let acquire = drv.base.nd_params.acquire;
        assert!(drv.qe_write_int32(acquire, 0, 0).is_err());
    }

    #[test]
    fn bias_on_is_sent_after_a_refused_status_reset() {
        let (addr, log) = ack_meter("STATUS:RESET");
        let mut drv = driver_on("QE_TETRA_BIAS", &addr);
        let bias_state = drv.base.params.bias_state;
        drv.base
            .port_base
            .set_int32_param(bias_state, 0, 1)
            .unwrap();
        // The last command's status is returned: HVS:<volts> was acknowledged.
        assert!(drv.set_bias_state(1).is_ok());
        let log = log.lock().unwrap();
        let reset = log.iter().position(|c| c == "STATUS:RESET").unwrap();
        assert_eq!(log[reset + 1], "HVS:ON");
        assert!(log[reset + 2].starts_with("HVS:"), "{log:?}");
    }

    /// C++ prints the connect failure and returns from its constructor: the
    /// port stays, with no read thread.
    #[test]
    fn a_missing_qe_port_keeps_the_port_without_a_link() {
        let rt = create_tetramm("QE_TETRA_NOQE", "QE_TETRA_NOQE_IP", 4, 0).unwrap();
        assert!(rt._read_thread.is_none());
    }

    /// C++ sets the input EOS only in setAcquire (drvTetrAMM.cpp:464-471): a
    /// port that refuses it keeps the driver and fails the acquire instead.
    #[test]
    fn an_input_eos_refusal_fails_the_acquire_not_the_port() {
        let (_meter, _runtime) = crate::octet::tests::eos_refusing_port("QE_TETRA_NOEOS_IP");
        let rt = create_tetramm("QE_TETRA_NOEOS", "QE_TETRA_NOEOS_IP", 4, 0).unwrap();
        let err = rt
            .port_handle()
            .write_int32_blocking(rt.nd_params.acquire, 0, 1)
            .unwrap_err();
        assert_eq!(err.status(), AsynStatus::Error);
        assert_eq!(err.message(), "no input EOS");
    }
}
