//! Port of `quadEMApp/caenSrc/drvTetrAMM.{h,cpp}` — CaenEls TetrAMM
//! 4-channel picoammeter, reached over TCP or serial through an asyn octet
//! port.

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

use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeAcquireMode, QeModel, QeReadFormat,
    QeTriggerMode, QeTriggerPolarity, QuadEmBase, QuadEmDevice, QuadEmParams, QuadEmShared,
};
use crate::octet::{OctetIo, connect_octet};
use crate::tetramm_proto as proto;

/// C++ `MAX_COMMAND_LEN`.
const MAX_COMMAND_LEN: usize = 256;
/// C++ `P_InterlockStatusString`.
const P_INTERLOCK_STATUS: &str = "TETRAMM_INTERLOCK_STATUS";
/// Poll interval of C++'s `epicsThreadSleep(0.01)` handshake loops.
const HANDSHAKE_POLL: Duration = Duration::from_millis(10);
/// C++ `reset()` waits this many one-second loops for the meter to reappear.
const RESET_WAIT_LOOPS: u32 = 20;

fn is_timeout(e: &AsynError) -> bool {
    matches!(
        e,
        AsynError::Status {
            status: AsynStatus::Timeout,
            ..
        }
    )
}

// ===========================================================================
// Driver
// ===========================================================================

pub struct TetrAmmDriver {
    base: QuadEmBase,
    io: OctetIo,
    shared: Arc<QuadEmShared>,
    p_interlock_status: usize,
    firmware_version: String,
}

impl TetrAmmDriver {
    fn new(
        port_name: &str,
        qe_port_name: &str,
        max_memory: usize,
        shared: Arc<QuadEmShared>,
    ) -> AsynResult<Self> {
        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let p_interlock_status = base
            .port_base
            .create_param(P_INTERLOCK_STATUS, epics_rs::asyn::param::ParamType::Int32)?;

        let io = connect_octet(qe_port_name, proto::TETRAMM_TIMEOUT)?;
        // C++ `drvTetrAMM::drvTetrAMM`: setInputEos(pasynUserMeter_, "\r\n", 2).
        io.set_input_eos(b"\r\n")?;

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

        let mut this = Self {
            base,
            io,
            shared,
            p_interlock_status,
            firmware_version: String::new(),
        };

        // C++ calls getFirmwareVersion() from the constructor; reset() is left
        // for later because the meter may be offline at IOC start.
        if let Err(e) = this.get_firmware_version() {
            log::warn!("drvTetrAMM: could not read firmware version: {e}");
        }
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
    fn send_command(&mut self, out: &str) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            self.set_acquire(0)?;
        }
        let resp = self.write_read_meter(out)?;
        if resp != "ACK" {
            log::error!("drvTetrAMM: outString={out} expected ACK, received {resp}");
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("TetrAMM: {out} -> {resp}, expected ACK"),
            });
        }
        if prev_acquiring {
            self.set_acquire(1)?;
        }
        Ok(())
    }

    fn get_firmware_version(&mut self) -> AsynResult<()> {
        self.firmware_version = "Unknown".into();
        self.set_acquire(0)?;
        let resp = self.write_read_meter("VER:?")?;
        self.firmware_version = proto::parse_version(&resp).to_string();
        let idx = self.params().firmware;
        self.base
            .port_base
            .set_string_param(idx, 0, self.firmware_version.clone())?;
        Ok(())
    }

    fn acquire_param(&self) -> usize {
        self.base.nd_params.acquire
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

    /// C++ `drvTetrAMM::setAcquire`.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 1 && self.shared.is_acquiring() {
            return Ok(());
        }

        if value == 0 {
            // Tell the read thread to stop, then wait for it to leave the loop.
            self.shared.set_acquiring(false);
            while self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
            let resp = self.write_read_meter("ACQ:OFF");
            let acked = matches!(&resp, Ok(r) if r == "ACK");
            if !acked {
                // Drain until a response ends on EOS with a trailing ACK, or the
                // read times out.
                loop {
                    match self.io.read_line(MAX_COMMAND_LEN) {
                        Ok(outcome) => {
                            let s = outcome.as_str();
                            if outcome.eom.contains(EomReason::EOS) && s.ends_with("ACK") {
                                break;
                            }
                            log::warn!(
                                "drvTetrAMM: waiting for ACK response, nread={}",
                                outcome.data.len()
                            );
                        }
                        Err(e) => {
                            if !is_timeout(&e) {
                                log::error!("drvTetrAMM: error waiting for ACK response: {e}");
                            }
                            break;
                        }
                    }
                }
            }
            self.base_set_acquire(0)?;
        } else {
            self.base_set_acquire(1)?;
            // C++ calls setAcquireParams() here: it is needed before NAQ and
            // also flushes any stale input.
            self.set_acquire_params()?;
            self.io.write("ACQ:ON")?;
            self.shared.acquire_start.signal();
            while !self.shared.is_reading_active() {
                thread::sleep(HANDSHAKE_POLL);
            }
        }
        Ok(())
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
    fn set_bias_state(&mut self, value: i32) -> AsynResult<()> {
        self.send_command("STATUS:RESET")?;
        self.send_command(proto::cmd_bias_state(value != 0))?;
        if value != 0 {
            let bv = self
                .base
                .port_base
                .get_float64_param(self.params().bias_voltage, 0)?;
            self.set_bias_voltage(bv)?;
        }
        Ok(())
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
            self.set_acquire(0)?;
        }
        if let Err(e) = self.read_status_inner() {
            log::error!("drvTetrAMM: readStatus failed: {e}");
            return Err(e);
        }
        if prev_acquiring {
            self.set_acquire(1)?;
        }
        Ok(())
    }
}

impl TetrAmmDriver {
    /// C++ `drvTetrAMM::setAcquireParams`.
    fn set_acquire_params(&mut self) -> AsynResult<()> {
        let prev_acquiring = self.shared.is_acquiring();
        if prev_acquiring {
            self.set_acquire(0)?;
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
            self.set_acquire(1)?;
        }
        Ok(())
    }

    /// C++ `drvTetrAMM::reset`.
    fn reset(&mut self) -> AsynResult<()> {
        self.set_acquire(0)?;
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
            log::error!("drvTetrAMM: no response from meter after {RESET_WAIT_LOOPS} seconds");
        }
        self.base_reset()
    }

    fn read_status_inner(&mut self) -> AsynResult<()> {
        let p = self.params();
        let err = || AsynError::Status {
            status: AsynStatus::Error,
            message: "TetrAMM: unparseable status response".into(),
        };

        let resp = self.write_read_meter("CHN:?")?;
        let num_channels = proto::parse_chn(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_int32_param(p.num_channels, 0, num_channels)?;
        self.shared.acq.lock().num_channels = num_channels;

        for i in 0..QE_MAX_INPUTS as i32 {
            let resp = self.write_read_meter(&proto::cmd_range_query(i + 1))?;
            let range = proto::parse_range(&resp).ok_or_else(err)?;
            self.base.port_base.set_int32_param(p.range, i + 1, range)?;
            if i == 0 {
                self.base.port_base.set_int32_param(p.range, 0, range)?;
            }
        }

        let resp = self.write_read_meter("NRSAMP:?")?;
        let values_per_read = proto::parse_nrsamp(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_int32_param(p.values_per_read, 0, values_per_read)?;
        self.shared.acq.lock().values_per_read = values_per_read;

        let sample_time = proto::sample_time(values_per_read);
        self.base
            .port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        let resp = self.write_read_meter("HVS:?")?;
        match proto::parse_hvs(&resp).ok_or_else(err)? {
            None => self.base.port_base.set_int32_param(p.hvs_readback, 0, 0)?,
            Some(volts) => {
                self.base.port_base.set_int32_param(p.hvs_readback, 0, 1)?;
                self.base
                    .port_base
                    .set_float64_param(p.bias_voltage, 0, volts)?;
            }
        }

        let resp = self.write_read_meter("HVV:?")?;
        let v = proto::parse_hvv(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_float64_param(p.hvv_readback, 0, v)?;

        let resp = self.write_read_meter("HVI:?")?;
        let v = proto::parse_hvi(&resp).ok_or_else(err)?;
        self.base
            .port_base
            .set_float64_param(p.hvi_readback, 0, v)?;

        let resp = self.write_read_meter("TEMP:?")?;
        let v = proto::parse_temp(&resp).ok_or_else(err)?;
        self.base.port_base.set_float64_param(p.temperature, 0, v)?;

        let resp = self.write_read_meter("STATUS:?")?;
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

impl PortDriver for TetrAmmDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvQuadEM::writeInt32` with the TetrAMM overrides bound in.
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
            self.set_acquire_params()?;
            self.read_status()?;
        } else if reason == p.geometry {
            self.shared.pos.lock().geometry = value;
        } else if reason == p.bias_state {
            self.set_bias_state(value)?;
            self.read_status()?;
        } else if reason == p.bias_interlock {
            self.set_bias_interlock(value)?;
            self.read_status()?;
        } else if reason == p.num_channels {
            self.set_num_channels(value)?;
            self.read_status()?;
        } else if reason == p.num_acquire {
            self.shared.acq.lock().num_acquire = value;
            self.set_acquire_params()?;
            self.read_status()?;
        } else if reason == p.ping_pong {
            // drvQuadEM::setPingPong is a no-op on the TetrAMM.
            self.read_status()?;
        } else if reason == p.range {
            if channel == 0 {
                self.set_range(value)?;
            } else {
                self.set_acquire_params()?;
            }
            self.read_status()?;
        } else if reason == p.read_data {
            self.shared.trigger_callbacks();
        } else if reason == p.resolution {
            // drvQuadEM::setResolution is a no-op on the TetrAMM.
            self.read_status()?;
        } else if reason == p.trigger_mode {
            self.set_trigger_mode(value)?;
            self.read_status()?;
        } else if reason == p.trigger_polarity {
            self.set_acquire_params()?;
            self.read_status()?;
        } else if reason == p.values_per_read {
            self.shared.acq.lock().values_per_read = value;
            self.set_values_per_read(value)?;
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

    /// C++ `drvQuadEM::writeFloat64` with the TetrAMM overrides bound in.
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
            self.read_status()?;
        } else if reason == p.bias_voltage {
            self.set_bias_voltage(value)?;
            self.read_status()?;
        } else if reason == p.integration_time {
            // drvQuadEM::setIntegrationTime is a no-op on the TetrAMM.
            self.read_status()?;
        }

        self.base.port_base.call_param_callbacks(channel)?;
        Ok(())
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

/// C++ `drvTetrAMM::readThread`.
fn read_thread(ctx: ReadContext) {
    let mut trigger_mode = QeTriggerMode::FreeRun;
    let mut read_format = QeReadFormat::Binary;
    let mut next_expected_edge = 0;
    let mut num_trig_starts = 0i64;
    let mut num_trig_ends = 0i64;
    let mut num_resync = 0u64;

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
                let outcome = match ctx.io.read_binary(n_requested) {
                    Ok(o) => o,
                    Err(e) => {
                        if is_timeout(&e) {
                            log::warn!("drvTetrAMM: timeout reading meter");
                        } else {
                            log::error!("drvTetrAMM: unexpected error reading meter: {e}");
                            thread::sleep(Duration::from_secs(1));
                        }
                        continue;
                    }
                };
                if outcome.data.len() != n_requested || !outcome.eom.contains(EomReason::CNT) {
                    log::error!(
                        "drvTetrAMM: short read, nRead={} expected {n_requested}, eom={:?}",
                        outcome.data.len(),
                        outcome.eom
                    );
                    thread::sleep(Duration::from_secs(1));
                    continue;
                }

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
                            log::error!(
                                "drvTetrAMM: extra trigger start, numTrigStarts={num_trig_starts}, numTrigEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 1;
                    }
                    proto::Trailer::TriggerEnd => {
                        num_trig_ends += 1;
                        if trigger_mode == QeTriggerMode::ExtBulb {
                            ctx.shared.trigger_callbacks();
                        }
                        if next_expected_edge != 1 {
                            log::error!(
                                "drvTetrAMM: extra trigger end, numTrigStarts={num_trig_starts}, numTrigEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 0;
                    }
                    proto::Trailer::AcqDone => {
                        log::debug!("drvTetrAMM: seen acq-done sNaN (0xfff40003ffffffff)");
                    }
                    proto::Trailer::LostSync => {
                        log::warn!("drvTetrAMM: lost sync, no NaN where expected; resynchronizing");
                        resync(&ctx, num_channels, &mut num_resync);
                    }
                }
            }
            QeReadFormat::Ascii => {
                let outcome = match ctx.io.read_line(proto::ASCII_BUFFER_SIZE) {
                    Ok(o) => o,
                    Err(e) => {
                        if !is_timeout(&e) {
                            log::error!("drvTetrAMM: unexpected error reading meter: {e}");
                            thread::sleep(Duration::from_secs(1));
                        }
                        continue;
                    }
                };
                if !outcome.eom.contains(EomReason::EOS) {
                    continue;
                }
                match proto::parse_ascii_line(&outcome.as_str(), num_channels) {
                    proto::AsciiRecord::TriggerStart => {
                        num_trig_starts += 1;
                        if next_expected_edge != 0 {
                            log::error!(
                                "drvTetrAMM: extra trigger start, numTrigStarts={num_trig_starts}, numTrigEnds={num_trig_ends}"
                            );
                        }
                        next_expected_edge = 1;
                    }
                    proto::AsciiRecord::TriggerEnd => {
                        num_trig_ends += 1;
                        if trigger_mode == QeTriggerMode::ExtBulb {
                            ctx.shared.trigger_callbacks();
                        }
                        if next_expected_edge != 1 {
                            log::error!(
                                "drvTetrAMM: extra trigger end, numTrigStarts={num_trig_starts}, numTrigEnds={num_trig_ends}"
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

/// C++ resync: read two frames' worth, find the data trailer, then consume the
/// bytes that follow it so the next read lands on a frame boundary.
fn resync(ctx: &ReadContext, num_channels: usize, num_resync: &mut u64) {
    let doubled = proto::binary_frame_len(num_channels) * 2;
    let Ok(outcome) = ctx.io.read_binary(doubled) else {
        return;
    };
    match proto::find_resync_offset(&outcome.data) {
        Some(offset) => {
            let remainder = proto::resync_remainder(num_channels, offset);
            let _ = ctx.io.read_binary(remainder);
            *num_resync += 1;
            log::warn!("drvTetrAMM: found NaN at position {offset}, read {remainder} bytes");
        }
        None => {
            log::error!("drvTetrAMM: ERROR, did not find NaN while resynchronizing");
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
    _read_thread: thread::JoinHandle<()>,
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

    let driver = TetrAmmDriver::new(port_name, qe_port_name, max_memory, shared.clone())?;
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
        .name("drvTetrAMMTask".into())
        .spawn(move || read_thread(read_ctx))
        .expect("failed to spawn drvTetrAMMTask");

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
