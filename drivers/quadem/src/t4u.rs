//! Port of `quadEMApp/sydorSrc/drvT4U_EM.{h,cpp}` and
//! `drvT4UDirect_EM.{h,cpp}` — the Sydor T4U electrometer.
//!
//! Upstream ships the two as a copy-paste fork of one file. They speak the same
//! register language and publish the same samples; they differ only in
//! transport, and that difference is the [`T4uVariant`] here:
//!
//! | | [`T4uVariant::Middle`] (`drvT4U_EM`) | [`T4uVariant::Direct`] (`drvT4UDirect_EM`) |
//! |---|---|---|
//! | command socket | TCP `host:base` (the Qt middle layer) | TCP `host:23` (the T4U itself) |
//! | command terminator | `\n` | `\r\n` |
//! | command write | straight from the port thread | queued; the command thread drains one per idle tick |
//! | register dumps (`tr`) | binary, back over the command socket | UDP `B\x03` frames |
//! | data | TCP `host:base+1`, ASCII `read` lines or `B\x01` frames | UDP `B\x01` frames |
//! | extra parameters | — | `QE_WSMODE`, `QE_RPP` |
//! | calibration | from registers 100-107 only | plus an INI file, re-sent on each range change |
//!
//! Neither driver talks to the meter on any control path: `setAcquire`,
//! `readStatus`, `reset`, `setRange`, `setValuesPerRead`, `setPingPong` and
//! `setIntegrationTime` are all `return asynSuccess`, and the constructor puts
//! the port into a permanently-acquiring state. The device streams; the driver
//! writes registers.
//!
//! Two C++ defects are not reproduced; both are noted at the site:
//!
//! * `drvT4UDirect_EM::dataReadThread` reassembles one UDP frame with five
//!   separate `read()` calls, each of which is a `recvfrom` that discards the
//!   rest of the datagram ([`proto::parse_direct_frame`]).
//! * `drvT4U_EM::readBroadcastPayload` checks the read status but not the byte
//!   count, so a short read publishes uninitialised heap as currents.

use std::collections::VecDeque;
use std::io::Write;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriver, PortDriverBase};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::PortClaim;
use epics_rs::asyn::request::{OctetRead, ParamSetValue};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;

use crate::asyn_print;
use crate::drv_quad_em::{
    self as qe, CallbackContext, QE_MAX_INPUTS, QeModel, QuadEmBase, QuadEmDevice, QuadEmParams,
    QuadEmShared,
};
use crate::octet::{OctetIo, create_ip_port};
use crate::t4u_proto::{self as proto, CalTable, Calibration, ChannelCal, CmdName, DirectFrame};

/// C++ `epicsThreadSleep(1.0)` after an unexpected socket error.
const ERROR_BACKOFF: Duration = Duration::from_secs(1);
/// Bytes pulled from a TCP socket per read. C++ reads one byte per `read()`;
/// the framing is identical either way.
const READ_CHUNK: usize = 4096;
/// C++ `cmd_tick_count >= 12`: idle polls of the command socket before the
/// direct driver sends one queued command.
const CMD_IDLE_TICKS: i32 = 12;
/// C++ `cmd_tick_count = -40` after a `wr 1` (sample frequency): the T4U
/// restarts its sampler, so the next command waits about four seconds.
const FREQ_SETTLE_TICKS: i32 = -40;
/// C++ `cmd_tick_count = 10` after a command was parsed: check the queue on the
/// next idle poll rather than after a full timeout.
const CMD_FORCE_TICKS: i32 = 10;
/// C++ `setDoubleParam(P_SampleTime, 0.0001)` (direct driver constructor).
const DIRECT_SAMPLE_TIME: f64 = 0.0001;
/// The middle-layer driver leaves `P_SampleTime` at drvQuadEM's default.
const MIDDLE_SAMPLE_TIME: f64 = 0.1;
/// C++ `setIntegerParam(P_ValuesPerRead, 5)`.
const DEFAULT_VALUES_PER_READ: i32 = 5;
/// C++ `setStringParam(P_Firmware, "1.48")`.
const FIRMWARE: &str = "1.48";
/// C++ `setDoubleParam(P_Temperature, 1234.5)`.
const TEMPERATURE: f64 = 1234.5;
/// C++ `setIntegerParam(P_Geometry, 1)` (square).
const GEOMETRY: i32 = 1;

fn error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// Which of the two upstream drivers this port is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum T4uVariant {
    /// `drvT4U_EM`: through the Qt middle layer.
    Middle,
    /// `drvT4UDirect_EM`: straight to the T4U.
    Direct,
}

impl T4uVariant {
    fn name(&self) -> &'static str {
        match self {
            Self::Middle => "drvT4U_EM",
            Self::Direct => "drvT4UDirect_EM",
        }
    }

    /// C++ appends `\n` in the middle-layer driver and `\r\n` in the direct one.
    fn eol(&self) -> &'static str {
        match self {
            Self::Middle => "\n",
            Self::Direct => "\r\n",
        }
    }

    fn timeout(&self) -> Duration {
        match self {
            Self::Middle => proto::T4U_EM_TIMEOUT,
            Self::Direct => proto::T4U_DIRECT_TIMEOUT,
        }
    }

    /// C++ `parseCmdName`: only the middle-layer driver expects a binary `tr`
    /// dump on the command socket. The direct driver's register dumps come back
    /// as UDP `B\x03` frames, so it treats a `tr` echo as an ASCII line.
    fn tr_on_command_socket(&self) -> bool {
        *self == Self::Middle
    }
}

// ===========================================================================
// State shared with the socket threads
// ===========================================================================

/// C++ `currRange_`, `calSlope_`/`calOffset_`, and the two timing values the
/// register decoder needs. Owned here rather than in the parameter library
/// because the command and data threads run outside the port actor and both
/// read and write them.
#[derive(Debug, Clone, Copy)]
struct T4uState {
    range: i32,
    cal: ChannelCal,
    /// Direct driver only: C++ `fullSlope_`/`fullOffset_` — the table the
    /// wait-state mode currently selects, written to registers 100-107 on every
    /// range change and checked against their read-back.
    full: Option<CalTable>,
    sample_time: f64,
    averaging_time: f64,
}

impl T4uState {
    /// C++ `value != full*_[currRange_][channel]`, direct driver only. C++
    /// indexes past the table for range 3 (`RANGE_SEL_MASK` is 0x3); here
    /// that range has nothing to compare.
    fn full_differs(
        &self,
        table: impl Fn(&CalTable) -> [[f32; QE_MAX_INPUTS]; proto::NUM_RANGES],
        channel: usize,
        value: f64,
    ) -> bool {
        self.full
            .as_ref()
            .and_then(|f| table(f).get(self.range as usize).map(|r| r[channel]))
            .is_some_and(|expected| value != expected as f64)
    }
}

type SharedState = Arc<parking_lot::Mutex<T4uState>>;

// ===========================================================================
// Parameters
// ===========================================================================

/// The T4U's own parameters, created after drvQuadEM's so that
/// `reason < first` is C++'s `function < FIRST_T4U_COMMAND`.
#[derive(Clone, Copy)]
pub struct T4uParams {
    first: usize,
    bias_n_en: usize,
    bias_p_en: usize,
    bias_n_voltage: usize,
    bias_p_voltage: usize,
    pulse_bias_en: usize,
    pulse_bias_off: usize,
    pulse_bias_on: usize,
    sample_freq: usize,
    dac_mode: usize,
    pos_track_mode: usize,
    pid_en: usize,
    update_reg: usize,
    pid_cu_en: usize,
    pid_hyst_en: usize,
    pid_ctrl_pol: usize,
    pid_ctrl_ex: usize,
    /// Direct driver only (`QE_WSMODE`).
    wait_state_mode: Option<usize>,
    /// Direct driver only (`QE_RPP`).
    reads_per_packet: Option<usize>,
    /// One parameter per row of [`proto::PID_REGS`], in the same order.
    pid: [usize; proto::PID_REG_COUNT],
}

impl T4uParams {
    fn create(base: &mut PortDriverBase, variant: T4uVariant) -> AsynResult<Self> {
        let bias_n_en = base.create_param("QE_BIAS_N", ParamType::Int32)?;
        let bias_p_en = base.create_param("QE_BIAS_P", ParamType::Int32)?;
        let bias_n_voltage = base.create_param("QE_BIAS_N_VOLTAGE", ParamType::Float64)?;
        let bias_p_voltage = base.create_param("QE_BIAS_P_VOLTAGE", ParamType::Float64)?;
        let pulse_bias_en = base.create_param("QE_PULSE_BIAS", ParamType::Int32)?;
        let pulse_bias_off = base.create_param("QE_PULSE_BIAS_OFF", ParamType::Int32)?;
        let pulse_bias_on = base.create_param("QE_PULSE_BIAS_ON", ParamType::Int32)?;
        let sample_freq = base.create_param("QE_SAMPLE_FREQ", ParamType::Int32)?;
        let dac_mode = base.create_param("QE_DAC_MODE", ParamType::Int32)?;
        let pos_track_mode = base.create_param("QE_POS_TRACK_MODE", ParamType::Int32)?;
        let pid_en = base.create_param("QE_PID_EN", ParamType::Int32)?;
        let update_reg = base.create_param("QE_UPDATE_REG", ParamType::Int32)?;
        let pid_cu_en = base.create_param("QE_PID_CU_EN", ParamType::Int32)?;
        let pid_hyst_en = base.create_param("QE_PID_HYST_EN", ParamType::Int32)?;
        let pid_ctrl_pol = base.create_param("QE_PID_POL", ParamType::Int32)?;
        let pid_ctrl_ex = base.create_param("QE_PID_EXT_CTRL", ParamType::Int32)?;
        let (wait_state_mode, reads_per_packet) = if variant == T4uVariant::Direct {
            (
                Some(base.create_param("QE_WSMODE", ParamType::Int32)?),
                Some(base.create_param("QE_RPP", ParamType::Int32)?),
            )
        } else {
            (None, None)
        };

        let mut pid = [0usize; proto::PID_REG_COUNT];
        for (slot, reg) in pid.iter_mut().zip(proto::PID_REGS.iter()) {
            *slot = base.create_param(reg.param, ParamType::Float64)?;
        }

        Ok(Self {
            first: bias_n_en,
            bias_n_en,
            bias_p_en,
            bias_n_voltage,
            bias_p_voltage,
            pulse_bias_en,
            pulse_bias_off,
            pulse_bias_on,
            sample_freq,
            dac_mode,
            pos_track_mode,
            pid_en,
            update_reg,
            pid_cu_en,
            pid_hyst_en,
            pid_ctrl_pol,
            pid_ctrl_ex,
            wait_state_mode,
            reads_per_packet,
            pid,
        })
    }

    /// C++ `findRegByAsyn`.
    fn pid_index(&self, reason: usize) -> Option<usize> {
        self.pid.iter().position(|p| *p == reason)
    }
}

// ===========================================================================
// Command sink
// ===========================================================================

/// C++ `writeReadMeter`: the middle-layer driver writes the command straight to
/// the socket, the direct driver pushes it onto `cmd_queue` for the command
/// thread to drain.
#[derive(Clone)]
enum CmdSink {
    Immediate(Box<OctetIo>),
    Queued(SyncSender<String>),
}

impl CmdSink {
    fn send(&self, command: &str) -> AsynResult<()> {
        match self {
            Self::Immediate(io) => {
                io.write(command)?;
                Ok(())
            }
            Self::Queued(tx) => match tx.try_send(command.to_string()) {
                Ok(()) => Ok(()),
                Err(TrySendError::Full(_)) => Err(AsynError::Status {
                    status: AsynStatus::Overflow,
                    message: "T4U: command queue full".into(),
                }),
                Err(TrySendError::Disconnected(_)) => {
                    Err(error("T4U: the command thread has stopped"))
                }
            },
        }
    }
}

// ===========================================================================
// Driver
// ===========================================================================

pub struct T4uDriver {
    base: QuadEmBase,
    shared: Arc<QuadEmShared>,
    state: SharedState,
    t4u: T4uParams,
    variant: T4uVariant,
    cmd: CmdSink,
    /// Direct driver only: the calibration file's two tables.
    calibration: Option<Calibration>,
}

impl T4uDriver {
    fn new(
        spec: &Build,
        shared: Arc<QuadEmShared>,
        state: SharedState,
        cmd: CmdSink,
    ) -> AsynResult<Self> {
        let Build {
            port_name,
            variant,
            calibration,
            max_memory,
            ..
        } = spec;
        let (variant, calibration, max_memory) = (*variant, *calibration, *max_memory);

        let mut base = QuadEmBase::new(port_name, max_memory)?;
        let t4u = T4uParams::create(&mut base.port_base, variant)?;

        // C++ `currRange_`, set before the ports are configured.
        {
            let mut st = state.lock();
            st.range = match variant {
                T4uVariant::Middle => 0,
                T4uVariant::Direct => proto::NUM_RANGES as i32 - 1,
            };
            st.full = calibration.map(|c| c.cw);
        }

        // C++ returns from the constructor at the first port that will not
        // configure: the port stays, with drvQuadEM's defaults and no meter.
        if let Err(line) = &spec.transport.link {
            asyn_print!(base.port_base, ERROR, "{line}");
            return Ok(Self {
                base,
                shared,
                state,
                t4u,
                variant,
                cmd,
                calibration,
            });
        }

        let p = base.params;
        let sample_time = match variant {
            T4uVariant::Middle => MIDDLE_SAMPLE_TIME,
            T4uVariant::Direct => {
                base.port_base
                    .set_int32_param(p.range, 0, proto::NUM_RANGES as i32 - 1)?;
                DIRECT_SAMPLE_TIME
            }
        };
        base.port_base
            .set_int32_param(p.model, 0, QeModel::SydorEm as i32)?;
        base.port_base
            .set_int32_param(p.values_per_read, 0, DEFAULT_VALUES_PER_READ)?;
        base.port_base.set_string_param(p.firmware, 0, FIRMWARE)?;
        base.port_base
            .set_float64_param(p.temperature, 0, TEMPERATURE)?;
        base.port_base.set_int32_param(p.geometry, 0, GEOMETRY)?;
        base.port_base
            .set_float64_param(p.sample_time, 0, sample_time)?;

        shared.pos.lock().geometry = GEOMETRY;
        {
            let mut acq = shared.acq.lock();
            acq.values_per_read = DEFAULT_VALUES_PER_READ;
            acq.num_channels = QE_MAX_INPUTS as i32;
        }
        state.lock().sample_time = sample_time;

        let mut this = Self {
            base,
            shared,
            state,
            t4u,
            variant,
            cmd,
            calibration,
        };

        // C++ sets acquiring_ = 1 in the constructor and calls
        // drvQuadEM::setAcquire(1): the meter streams whether or not anyone
        // asked, so the port is always "acquiring" and the read threads never
        // wait on acquireStartEvent_.
        this.shared.set_acquiring(true);
        this.base_set_acquire(1);
        Ok(this)
    }

    fn params(&self) -> QuadEmParams {
        self.base.params
    }

    /// C++ `writeReadMeter`. As upstream, a failed send is ignored and the
    /// write still succeeds: the parameter is already in the library and the
    /// meter will be re-synchronised by the next register dump.
    fn send(&self, command: String) {
        let line = format!("{command}{}", self.variant.eol());
        let result = self.cmd.send(&line);
        // drvT4U_EM.cpp:554-561; the direct driver has these commented out.
        if self.variant == T4uVariant::Middle {
            println!("Entered writeReadMeter()");
            println!(
                "Write status {}",
                result.as_ref().map_or_else(AsynError::c_status_code, |_| 0)
            );
        }
    }

    /// The `"%s: function %i name %s"` debug line drvT4U_EM's write methods
    /// print (drvT4U_EM.cpp:291, :471).
    fn print_function_name(&self, method: &str, reason: usize) {
        if self.variant == T4uVariant::Middle {
            let name = self.base.port_base.params.param_name(reason).unwrap_or("");
            println!("{method}: function {reason} name {name}");
        }
    }

    /// C++ `P_Range` in `writeInt32`: clip, write register 3 and, in the direct
    /// driver, re-send the selected calibration table for the new range.
    fn write_range(&mut self, channel: i32, value: i32) {
        let value = value.clamp(0, proto::NUM_RANGES as i32 - 1);
        let _ = self
            .base
            .port_base
            .set_int32_param(self.params().range, channel, value);
        self.send(proto::cmd_write(proto::REG_T4U_RANGE, value));

        let full = self.state.lock().full;
        if let Some(full) = full {
            let range = value as usize;
            for channel in 0..QE_MAX_INPUTS {
                // The registers hold the float's bit pattern, so C++ type-puns
                // the float and writes the resulting int.
                let slope = full.slope[range][channel].to_bits() as i32;
                let offset = full.offset[range][channel].to_bits() as i32;
                self.send(proto::cmd_write(
                    proto::TXC_CALIB_SLOPE_BASE + channel as i32,
                    slope,
                ));
                self.send(proto::cmd_write(
                    proto::TXC_CALIB_OFFSET_BASE + channel as i32,
                    offset,
                ));
            }
        }

        self.state.lock().range = value;
    }

    /// C++ `P_WaitStateMode` in the direct driver's `writeInt32`: clear the
    /// wait-state bits, set the mode's bits, and swap the active calibration
    /// table (triggered mode uses the pulsed set).
    fn write_wait_state_mode(&mut self, value: i32) {
        let Some(cal) = self.calibration else {
            return;
        };
        self.send(proto::cmd_bits(
            false,
            proto::REG_T4U_CTRL,
            proto::WAIT_STATE_MASK,
        ));
        match value {
            proto::WAIT_STATE_MODE_INHIBIT => {
                self.send(proto::cmd_bits(
                    true,
                    proto::REG_T4U_CTRL,
                    proto::WAIT_STATE_INHIBIT_MASK,
                ));
                self.state.lock().full = Some(cal.cw);
            }
            proto::WAIT_STATE_MODE_TRIGGER => {
                self.send(proto::cmd_bits(
                    true,
                    proto::REG_T4U_CTRL,
                    proto::WAIT_STATE_TRIGGER_MASK,
                ));
                self.state.lock().full = Some(cal.pulsed);
            }
            // WAIT_STATE_MODE_NONE, and anything else: the bits stay cleared.
            _ => self.state.lock().full = Some(cal.cw),
        }
    }

    /// C++ `P_AveragingTime` in `writeFloat64`: `NumAverage = int(averagingTime
    /// / sampleTime)`. Unlike the other quadEM drivers this truncates rather
    /// than rounding, because the T4U has no `readStatus` to recompute it.
    fn write_averaging_time(&mut self, value: f64) {
        let num_average = {
            let mut st = self.state.lock();
            st.averaging_time = value;
            (value / st.sample_time) as i32
        };
        let _ = self
            .base
            .port_base
            .set_int32_param(self.params().num_average, 0, num_average);
        self.shared.acq.lock().num_average = num_average;
    }
}

// ===========================================================================
// drvQuadEM virtuals
// ===========================================================================

impl QuadEmDevice for T4uDriver {
    fn qe_base(&mut self) -> &mut QuadEmBase {
        &mut self.base
    }

    fn qe_shared(&self) -> &Arc<QuadEmShared> {
        &self.shared
    }

    /// C++ `drvT4U_EM::setAcquire` / `drvT4UDirect_EM::setAcquire`: the meter
    /// streams unconditionally, so there is nothing to start or stop.
    fn set_acquire(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }

    /// C++ `drvT4U_EM::reset` / `drvT4UDirect_EM::reset` do nothing.
    fn reset(&mut self) -> AsynResult<()> {
        Ok(())
    }
}

// ===========================================================================
// asyn port driver
// ===========================================================================

impl PortDriver for T4uDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base.port_base
    }

    /// C++ `drvT4U_EM::report` and `drvT4UDirect_EM::report` return without
    /// printing, not even the base report.
    fn report(&mut self, _out: &mut dyn std::fmt::Write, _level: i32) {}

    /// C++ `drvT4U_EM::writeInt32` / `drvT4UDirect_EM::writeInt32`.
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let channel = user.addr;
        let p = self.params();
        let t = self.t4u;

        if self.variant == T4uVariant::Middle {
            println!("writeInt32: function {reason}");
        }
        // C's `status |=` here never reaches the record: its own commands
        // return asynSuccess and drvQuadEM's restarts the status.
        let _ = self.base.port_base.set_int32_param(reason, channel, value);
        self.print_function_name("writeInt32", reason);

        if reason == t.bias_n_en {
            self.send(proto::cmd_bias_enable(value != 0, false));
        } else if reason == t.bias_p_en {
            self.send(proto::cmd_bias_enable(value != 0, true));
        } else if reason == t.pulse_bias_en {
            self.send(proto::cmd_bits(
                value != 0,
                proto::REG_T4U_CTRL,
                proto::PULSE_BIAS_EN_MASK,
            ));
        } else if reason == t.pulse_bias_off {
            self.send(proto::cmd_write(proto::PULSE_BIAS_OFF_REG, value));
        } else if reason == t.pulse_bias_on {
            self.send(proto::cmd_write(proto::PULSE_BIAS_ON_REG, value));
        } else if reason == t.sample_freq {
            self.send(proto::cmd_write(proto::REG_T4U_FREQ, value));
        } else if reason == p.range {
            self.write_range(channel, value);
        } else if reason == t.dac_mode {
            // Several functions share register 93, so the mode bits are cleared
            // before the new mode is set.
            self.send(proto::cmd_bits(
                false,
                proto::REG_OUTPUT_MODE,
                proto::OUTPUT_MODE_MASK,
            ));
            self.send(proto::cmd_bits(
                true,
                proto::REG_OUTPUT_MODE,
                value as u32 & proto::OUTPUT_MODE_MASK,
            ));
        } else if reason == t.pid_en {
            self.send(proto::cmd_bits(
                value != 0,
                proto::REG_PID_CTRL,
                proto::PID_EN_MASK,
            ));
        } else if reason == t.update_reg {
            match self.variant {
                T4uVariant::Middle => {
                    println!("Running updater.");
                    self.send(proto::cmd_read_regs(100, 107));
                }
                T4uVariant::Direct => {
                    self.send(proto::cmd_read_regs(0, 49));
                    self.send(proto::cmd_read_regs(50, 99));
                    self.send(proto::cmd_read_regs(100, 107));
                }
            }
        } else if reason == t.pid_cu_en
            || reason == t.pid_hyst_en
            || reason == t.pid_ctrl_pol
            || reason == t.pid_ctrl_ex
        {
            let mask = if reason == t.pid_cu_en {
                proto::PID_CUTOUT_EN_MASK
            } else if reason == t.pid_hyst_en {
                proto::PID_HYST_REENABLE_MASK
            } else if reason == t.pid_ctrl_pol {
                proto::PID_CTRL_POL_MASK
            } else {
                proto::PID_EXT_CTRL_MASK
            };
            self.send(proto::cmd_bits(value != 0, proto::REG_PID_CTRL, mask));
        } else if reason == t.pos_track_mode {
            let shift_mask = proto::PID_POS_TRACK_MASK << proto::PID_POS_TRACK_SHIFT;
            let shift_val =
                (value as u32 & proto::PID_POS_TRACK_MASK) << proto::PID_POS_TRACK_SHIFT;
            self.send(proto::cmd_bits(false, proto::REG_PID_CTRL, shift_mask));
            self.send(proto::cmd_bits(true, proto::REG_PID_CTRL, shift_val));
        } else if Some(reason) == t.wait_state_mode {
            self.write_wait_state_mode(value);
        } else if Some(reason) == t.reads_per_packet {
            self.send(proto::cmd_write(proto::REG_READS_PER_PACKET, value));
        }

        // C++ returns here for its own functions, without callbacks; their
        // readbacks post when the meter's register dump comes back.
        if reason < t.first {
            return self.qe_write_int32(user, value);
        }
        Ok(())
    }

    /// C++ `drvT4U_EM::writeFloat64` / `drvT4UDirect_EM::writeFloat64`.
    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = user.reason;
        let channel = user.addr;
        let p = self.params();
        let t = self.t4u;

        // As in `write_int32`, C drops this status.
        let _ = self
            .base
            .port_base
            .set_float64_param(reason, channel, value);
        self.print_function_name("writeFloat64", reason);

        if let Some(index) = t.pid_index(reason) {
            let reg = &proto::PID_REGS[index];
            let out = proto::scale_param_to_reg(value, reg, false) as i32;
            self.send(proto::cmd_write(reg.reg_num, out));
        } else if reason == t.bias_n_voltage {
            self.send(proto::cmd_write(proto::REG_BIAS_N_VOLTAGE, value as i32));
        } else if reason == t.bias_p_voltage {
            self.send(proto::cmd_write(proto::REG_BIAS_P_VOLTAGE, value as i32));
        } else if reason == p.averaging_time {
            self.write_averaging_time(value);
        }

        if self.variant == T4uVariant::Middle {
            // C prints this one without a newline (drvT4U_EM.cpp:499).
            print!("About to exit from writeFloat64");
            let _ = std::io::stdout().flush();
        }
        if reason < t.first {
            return self.qe_write_float64(user, value);
        }
        Ok(())
    }
}

// ===========================================================================
// Register updates from the device threads
// ===========================================================================

/// Applies a `(register, value)` pair from the meter to the parameter library
/// and the shared state — C++ `processRegVal`, which runs on the command thread
/// (both drivers) and on the data thread (the direct driver's `B\x03` frames).
#[derive(Clone)]
struct RegSink {
    handle: PortHandle,
    params: QuadEmParams,
    t4u: T4uParams,
    shared: Arc<QuadEmShared>,
    state: SharedState,
}

impl RegSink {
    /// False for a register C++ `processRegVal` does not handle (its `-1`).
    fn apply(&self, reg_num: i32, reg_val: u32) -> bool {
        use proto::RegUpdate::*;
        let Some(update) = proto::decode_reg(reg_num, reg_val) else {
            return false;
        };

        let set_int =
            |reason: usize, value: i32| ParamSetValue::new(reason, 0, ParamValue::Int32(value));
        let set_f64 =
            |reason: usize, value: f64| ParamSetValue::new(reason, 0, ParamValue::Float64(value));

        let values = match update {
            Pid { index, value } => vec![set_f64(self.t4u.pid[index], value)],
            Ctrl {
                bias_n,
                bias_p,
                pulse_bias,
            } => vec![
                set_int(self.t4u.bias_n_en, bias_n as i32),
                set_int(self.t4u.bias_p_en, bias_p as i32),
                set_int(self.t4u.pulse_bias_en, pulse_bias as i32),
            ],
            SampleFreq(freq) => {
                let (sample_time, num_average) = {
                    let mut st = self.state.lock();
                    st.sample_time = 1.0 / freq as f64;
                    (st.sample_time, (st.averaging_time / st.sample_time) as i32)
                };
                self.shared.acq.lock().num_average = num_average;
                vec![
                    set_int(self.t4u.sample_freq, freq as i32),
                    set_f64(self.params.sample_time, sample_time),
                    set_int(self.params.num_average, num_average),
                ]
            }
            Range(range) => {
                self.state.lock().range = range;
                vec![set_int(self.params.range, range)]
            }
            PulseBiasOff(count) => vec![set_int(self.t4u.pulse_bias_off, count)],
            PulseBiasOn(count) => vec![set_int(self.t4u.pulse_bias_on, count)],
            DacMode(mode) => vec![set_int(self.t4u.dac_mode, mode)],
            PidCtrl {
                cutout_en,
                hyst_en,
                pid_en,
                ctrl_pol,
                ext_ctrl,
                pos_track,
            } => vec![
                set_int(self.t4u.pid_cu_en, cutout_en as i32),
                set_int(self.t4u.pid_hyst_en, hyst_en as i32),
                set_int(self.t4u.pid_en, pid_en as i32),
                set_int(self.t4u.pid_ctrl_pol, ctrl_pol as i32),
                set_int(self.t4u.pid_ctrl_ex, ext_ctrl as i32),
                set_int(self.t4u.pos_track_mode, pos_track),
            ],
            CalSlope { channel, value } => {
                let mut st = self.state.lock();
                st.cal.slope[channel] = value;
                if st.full_differs(|f| f.slope, channel, value) {
                    println!("Slope calibration mismatch channel {channel}");
                }
                Vec::new()
            }
            CalOffset { channel, value } => {
                let mut st = self.state.lock();
                st.cal.offset[channel] = value;
                if st.full_differs(|f| f.offset, channel, value) {
                    println!("Offset calibration mismatch channel {channel}");
                }
                Vec::new()
            }
        };

        if !values.is_empty() {
            let _ = self.handle.set_params_and_notify_blocking(0, values);
        }
        true
    }
}

// ===========================================================================
// Byte-stream reader
// ===========================================================================

/// A buffered view of a TCP socket. C++ walks both sockets one `read()` per
/// byte; this reads in chunks and frames out of the buffer, which delivers the
/// same bytes to the same parsers.
struct StreamReader {
    io: OctetIo,
    buf: VecDeque<u8>,
}

impl StreamReader {
    fn new(io: OctetIo) -> Self {
        Self {
            io,
            buf: VecDeque::new(),
        }
    }

    /// One socket read. `false` when it timed out with nothing to show.
    fn fill(&mut self) -> AsynResult<bool> {
        let (data, status) = self.io.read_binary(READ_CHUNK).into_parts();
        match status {
            // C++ counts `nRead` whatever the status: a timeout that still
            // delivered bytes delivered them.
            Ok(()) => {}
            Err(e) if e.status() == AsynStatus::Timeout => {}
            Err(e) => return Err(e),
        }
        let got = !data.is_empty();
        self.buf.extend(data);
        Ok(got)
    }

    /// The next byte, or `None` if the socket was idle for one timeout.
    fn next_byte(&mut self) -> AsynResult<Option<u8>> {
        if let Some(b) = self.buf.pop_front() {
            return Ok(Some(b));
        }
        self.fill()?;
        Ok(self.buf.pop_front())
    }

    /// The next byte, waiting through idle timeouts as C++'s `if (nRead == 0)
    /// continue;` does.
    fn next_byte_blocking(&mut self) -> AsynResult<u8> {
        loop {
            if let Some(b) = self.next_byte()? {
                return Ok(b);
            }
        }
    }

    /// Exactly `n` bytes. `None` when the socket went idle first — C++'s
    /// `nRead != nRequest` short read, which flushes.
    fn take_exact(&mut self, n: usize) -> AsynResult<Option<Vec<u8>>> {
        while self.buf.len() < n {
            if !self.fill()? {
                return Ok(None);
            }
        }
        Ok(Some(self.buf.drain(..n).collect()))
    }

    /// Bytes up to and including the next `\n`. `None` when the line would
    /// exceed `max` (C++'s `totalBytesRead >= MAX_COMMAND_LEN - 1` flush).
    fn read_line(&mut self, max: usize) -> AsynResult<Option<Vec<u8>>> {
        loop {
            if let Some(pos) = self.buf.iter().position(|b| *b == b'\n') {
                return Ok(Some(self.buf.drain(..=pos).collect()));
            }
            if self.buf.len() >= max {
                return Ok(None);
            }
            self.fill()?;
        }
    }

    /// C++ `pasynOctetSyncIO->flush`, plus the buffered bytes this reader is
    /// holding — they belong to the same broken frame.
    fn flush(&mut self) {
        self.buf.clear();
        let _ = self.io.flush();
    }
}

// ===========================================================================
// Command thread
// ===========================================================================

struct CmdContext {
    io: OctetIo,
    regs: RegSink,
    variant: T4uVariant,
    /// Direct driver only: the outgoing command queue.
    queue: Option<Receiver<String>>,
}

/// C++ `cmdReadThread`.
fn cmd_thread(ctx: CmdContext) {
    let mut reader = StreamReader::new(ctx.io.clone());
    let mut tick = 0i32;
    loop {
        if cmd_iteration(&ctx, &mut reader, &mut tick).is_err() {
            reader.flush();
            thread::sleep(ERROR_BACKOFF);
        }
    }
}

/// C++'s outgoing-command tick: the direct driver may only write to the telnet
/// socket while it is not mid-parse, so a command waits for an idle poll.
fn service_queue(ctx: &CmdContext, tick: &mut i32) -> AsynResult<()> {
    let Some(rx) = &ctx.queue else {
        return Ok(());
    };
    // C++ increments cmd_tick_count once at the top of the poll and once more
    // when the read came back empty.
    *tick += 2;
    if *tick < CMD_IDLE_TICKS {
        return Ok(());
    }
    let Ok(command) = rx.try_recv() else {
        return Ok(());
    };
    ctx.io.write(&command)?;
    *tick = if command.starts_with("wr 1 ") {
        FREQ_SETTLE_TICKS
    } else {
        0
    };
    Ok(())
}

fn cmd_iteration(ctx: &CmdContext, reader: &mut StreamReader, tick: &mut i32) -> AsynResult<()> {
    // The verb is two non-space characters; leading whitespace is skipped.
    let first = loop {
        match reader.next_byte()? {
            Some(b) if b.is_ascii_whitespace() => continue,
            Some(b) => break b,
            None => service_queue(ctx, tick)?,
        }
    };
    let second = reader.next_byte_blocking()?;
    let verb = String::from_utf8_lossy(&[first, second]).into_owned();

    match proto::parse_cmd_name(&verb) {
        CmdName::Tr if ctx.variant.tr_on_command_socket() => {
            // A big-endian length, then that many bytes of little-endian
            // six-byte register records.
            let Some(len) = reader.take_exact(2)? else {
                reader.flush();
                return Ok(());
            };
            let tr_len = u16::from_be_bytes([len[0], len[1]]) as usize;
            let Some(payload) = reader.take_exact(tr_len)? else {
                reader.flush();
                return Ok(());
            };
            // tr_on_command_socket is the middle-layer driver, whose loop
            // prints these (drvT4U_EM.cpp:754-762).
            for (reg_num, reg_val) in proto::parse_reg_records(&payload) {
                println!("Processing reg {reg_num:3}");
                if ctx.regs.apply(reg_num, reg_val) {
                    println!(
                        "Processed reg {reg_num:3} Value: {:10} 0x{reg_val:08x}",
                        reg_val as i32
                    );
                }
            }
        }
        CmdName::Tr | CmdName::Ascii => {
            // C++'s kEXEC_ASC_CMD branch is an empty "decide if we want to
            // support this": the line is read to keep the stream framed and
            // then dropped. processReceivedCommand exists but nothing calls it.
            if reader.read_line(proto::MAX_COMMAND_LEN)?.is_none() {
                reader.flush();
            }
        }
        CmdName::Unknown => {
            reader.flush();
        }
    }

    // Check the outgoing queue on the next idle poll rather than after a full
    // timeout.
    *tick = CMD_FORCE_TICKS;
    Ok(())
}

// ===========================================================================
// Data threads
// ===========================================================================

struct DataContext {
    io: OctetIo,
    handle: PortHandle,
    params: QuadEmParams,
    shared: Arc<QuadEmShared>,
    state: SharedState,
    regs: RegSink,
}

impl DataContext {
    fn publish(&self, samples: Vec<[f64; QE_MAX_INPUTS]>) {
        for sample in samples {
            self.shared
                .compute_positions(&self.handle, &self.params, &sample);
        }
    }

    fn range_and_cal(&self) -> (i32, ChannelCal) {
        let st = self.state.lock();
        (st.range, st.cal)
    }
}

/// C++ `drvT4U_EM::dataReadThread`: a TCP stream of `read` lines and `B\x01`
/// frames.
fn middle_data_thread(ctx: DataContext) {
    let mut reader = StreamReader::new(ctx.io.clone());
    loop {
        if middle_data_iteration(&ctx, &mut reader).is_err() {
            reader.flush();
            thread::sleep(ERROR_BACKOFF);
        }
    }
}

fn middle_data_iteration(ctx: &DataContext, reader: &mut StreamReader) -> AsynResult<()> {
    let Some(kind) = reader.next_byte()? else {
        return Ok(());
    };
    match kind {
        b'r' => {
            let Some(rest) = reader.read_line(proto::MAX_COMMAND_LEN)? else {
                reader.flush();
                return Ok(());
            };
            let line = format!("r{}", String::from_utf8_lossy(&rest));
            match proto::parse_read_line(&line) {
                Some(sample) => ctx.publish(vec![sample]),
                None => reader.flush(),
            }
        }
        b'B' => {
            // The rest of the header: the frame type, then the units flag and
            // the payload length, both big-endian.
            let Some(header) = reader.take_exact(5)? else {
                reader.flush();
                return Ok(());
            };
            if header[0] != 1 {
                reader.flush();
                return Ok(());
            }
            let Some(bc) = proto::parse_broadcast_header(&header[1..]) else {
                reader.flush();
                return Ok(());
            };
            // C++ checks only the read status here, not the byte count, so a
            // short read publishes whatever the uninitialised heap buffer held.
            let payload = match reader.take_exact(bc.payload_len) {
                Ok(Some(payload)) => payload,
                failed => {
                    // No payload within the timeout is C's asynTimeout.
                    let status =
                        failed.map_or_else(|e| e.c_status_code(), |_| AsynStatus::Timeout as i32);
                    println!("Error reading data payload: {status}.");
                    reader.flush();
                    return Ok(());
                }
            };
            let (range, cal) = ctx.range_and_cal();
            ctx.publish(proto::decode_samples(&payload, bc.units, range, &cal));
        }
        _ => reader.flush(),
    }
    Ok(())
}

/// What one datagram read leaves [`direct_data_thread`] to do.
#[derive(Debug, PartialEq)]
enum Datagram {
    Frame(Vec<u8>),
    /// Nothing arrived within the read timeout: read again at once.
    Idle,
    /// The socket failed: back off before reading again.
    Failed,
}

/// C++ reads its header `while (nRead != 2)` whatever the status, so bytes
/// beside a timeout are a frame; a timeout with none is an idle line.
fn datagram(read: OctetRead) -> Datagram {
    match read.into_parts() {
        (data, Ok(())) if data.is_empty() => Datagram::Idle,
        (data, Ok(())) => Datagram::Frame(data),
        (data, Err(e)) if e.status() == AsynStatus::Timeout => {
            if data.is_empty() {
                Datagram::Idle
            } else {
                Datagram::Frame(data)
            }
        }
        (_, Err(_)) => Datagram::Failed,
    }
}

/// C++ `drvT4UDirect_EM::dataReadThread`: one UDP datagram per frame.
fn direct_data_thread(ctx: DataContext) {
    loop {
        let datagram = match datagram(ctx.io.read_binary(proto::MAX_PACKET_SIZE)) {
            Datagram::Frame(data) => data,
            Datagram::Idle => continue,
            Datagram::Failed => {
                thread::sleep(ERROR_BACKOFF);
                continue;
            }
        };

        match proto::parse_direct_frame(&datagram) {
            Ok(DirectFrame::Data { metadata, image }) => {
                let (range, cal) = ctx.range_and_cal();
                ctx.publish(proto::decode_samples(image, metadata.units, range, &cal));
            }
            Ok(DirectFrame::Registers(records)) => {
                for (reg_num, reg_val) in records {
                    ctx.regs.apply(reg_num, reg_val);
                }
            }
            Err(_) => {
                let _ = ctx.io.flush();
            }
        }
    }
}

// ===========================================================================
// Construction
// ===========================================================================

/// Live handles to a configured T4U port.
pub struct T4uRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: QuadEmParams,
    pub nd_params: epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams,
    pub pool: epics_rs::ad_core::ndarray_pool::DriverPool,
    pub outputs: Vec<Arc<parking_lot::Mutex<epics_rs::ad_core::plugin::channel::NDArrayOutput>>>,
    /// The sub-ports the driver built; dropping them closes the sockets. C++
    /// registers them by name, so `st.cmd` can trace them.
    pub sub_ports: Vec<PortRuntimeHandle>,
    /// The meter threads; absent when a port would not configure.
    _meter_threads: Option<[thread::JoinHandle<()>; 2]>,
    _callback_thread: thread::JoinHandle<()>,
}

impl T4uRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }
}

struct Transport {
    /// The command and data sockets, or the `asynPrint` line C++ gives for
    /// the port that would not configure.
    link: Result<(OctetIo, OctetIo), String>,
    runtimes: Vec<PortRuntimeHandle>,
}

/// C++ `drvAsynIPPortConfigure` of one sub-port, with the constructor's error
/// line when it fails (`status` is the configure call's -1).
fn sub_port(
    name: &str,
    host: &str,
    timeout: Duration,
    owner: &Arc<InterruptGate>,
    runtimes: &mut Vec<PortRuntimeHandle>,
    describe: impl FnOnce(&str, &str) -> String,
) -> Result<OctetIo, String> {
    match create_ip_port(name, host, timeout, false, false, owner) {
        Ok((io, runtime)) => {
            runtimes.push(runtime);
            Ok(io)
        }
        Err(_) => Err(describe(name, host)),
    }
}

/// Everything the two `create_*` entry points differ in.
struct Build {
    port_name: String,
    variant: T4uVariant,
    transport: Transport,
    /// Direct driver only: the receiving end of the command queue.
    queue: Option<Receiver<String>>,
    /// Direct driver only: the parsed calibration file.
    calibration: Option<Calibration>,
    ring_buffer_size: usize,
    max_memory: usize,
}

/// The command port, with the line both C++ constructors print after
/// `drvAsynIPPortConfigure` whatever it returned.
fn command_port(
    port_name: &str,
    host: &str,
    variant: T4uVariant,
    owner: &Arc<InterruptGate>,
    runtimes: &mut Vec<PortRuntimeHandle>,
) -> Result<OctetIo, String> {
    let name = format!("TCP_Command_{port_name}");
    let port = sub_port(
        &name,
        host,
        variant.timeout(),
        owner,
        runtimes,
        |name, host| {
            format!(
                "{}::drvT4U_EM error calling drvAsynIPPortConfigure for command port={name}, \
             IP={host}, status=-1",
                variant.name()
            )
        },
    );
    println!(
        "Attempted command port: {name} connection to: {host}\nStatus: {}",
        if port.is_ok() { 0 } else { -1 }
    );
    port
}

/// C++ `drvT4U_EMConfigure(portName, qtHostAddress, ringBufferSize,
/// base_port_num)`: the Qt middle layer listens for commands on `base_port_num`
/// and streams data on `base_port_num + 1`.
///
/// `max_memory` has no C++ analogue (the C++ pool is unbounded); it bounds the
/// Rust `NDArrayPool`.
pub fn create_t4u_em(
    claim: &PortClaim,
    qt_host_address: &str,
    ring_buffer_size: usize,
    base_port_num: u16,
    max_memory: usize,
    owner: &Arc<InterruptGate>,
) -> AsynResult<T4uRuntime> {
    let port_name = claim.name();
    let variant = T4uVariant::Middle;
    let mut runtimes = Vec::new();
    let link = command_port(
        port_name,
        &format!("{qt_host_address}:{base_port_num}"),
        variant,
        owner,
        &mut runtimes,
    )
    .and_then(|cmd_io| {
        let data_io = sub_port(
            &format!("TCP_Data_{port_name}"),
            &format!("{qt_host_address}:{}", u32::from(base_port_num) + 1),
            variant.timeout(),
            owner,
            &mut runtimes,
            |name, host| {
                format!(
                    "drvT4U_EM::drvT4U_EM error calling drvAsynIPPortConfigure for data \
                     port={name}, IP={host}, status=-1"
                )
            },
        )?;
        Ok((cmd_io, data_io))
    });
    // An unlinked port has no socket to write to; C++ dereferences a null
    // asynUser there.
    let cmd = match &link {
        Ok((cmd_io, _)) => cmd_io.clone(),
        Err(_) => OctetIo::unbound(variant.timeout()),
    };

    build(
        Build {
            port_name: port_name.to_string(),
            variant,
            transport: Transport { link, runtimes },
            queue: None,
            calibration: None,
            ring_buffer_size,
            max_memory,
        },
        CmdSink::Immediate(Box::new(cmd)),
        claim,
    )
}

/// Why [`create_t4u_direct_em`] built no port.
#[derive(Debug)]
pub enum T4uDirectError {
    /// The `std::logic_error` inicpp throws out of the constructor on a
    /// malformed calibration file; its `what()`. C iocsh catches it and keeps
    /// running (iocsh.cpp:1275-1284).
    Thrown(String),
    Asyn(AsynError),
}

impl From<AsynError> for T4uDirectError {
    fn from(e: AsynError) -> Self {
        Self::Asyn(e)
    }
}

impl std::fmt::Display for T4uDirectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Thrown(what) => f.write_str(what),
            Self::Asyn(e) => e.fmt(f),
        }
    }
}

/// C++ `drvT4UDirect_EMConfigure(portName, T4U_Address, ringBufferSize,
/// base_port_num, cfgFileName)`: commands go to the T4U's telnet port, data
/// arrives as UDP datagrams on `base_port_num`.
pub fn create_t4u_direct_em(
    claim: &PortClaim,
    t4u_address: &str,
    ring_buffer_size: usize,
    base_port_num: u16,
    cfg_file_name: &str,
    max_memory: usize,
    owner: &Arc<InterruptGate>,
) -> Result<T4uRuntime, T4uDirectError> {
    let port_name = claim.name();
    let variant = T4uVariant::Direct;

    // inicpp reads a file that will not open as an empty one, and throws out
    // of the constructor on a malformed line.
    let text = std::fs::read(cfg_file_name).unwrap_or_default();
    let ini = proto::parse_ini(&String::from_utf8_lossy(&text)).map_err(T4uDirectError::Thrown)?;
    let Some(calibration) = proto::parse_calibration(&ini) else {
        println!("Error in calibration file.  Aborting.");
        // C++ `exit(1)`, which runs the epicsAtExit list through atexit().
        epics_rs::base::runtime::exit::call_at_exits();
        std::process::exit(1);
    };

    let mut runtimes = Vec::new();
    let link = command_port(
        port_name,
        &format!("{t4u_address}:{}", proto::T4U_CMD_PORT),
        variant,
        owner,
        &mut runtimes,
    )
    .and_then(|cmd_io| {
        // The C++ host spec is "127.0.0.1:base-1:base UDP": the driver binds
        // the local port the T4U streams to, and the remote endpoint only
        // gives the socket a peer. `base_port_num` is unsigned in C++, so 0
        // gives a remote port that will not parse.
        let data_io = sub_port(
            &format!("UDP_Data_{port_name}"),
            &format!(
                "127.0.0.1:{}:{base_port_num} UDP",
                u32::from(base_port_num).wrapping_sub(1)
            ),
            variant.timeout(),
            owner,
            &mut runtimes,
            |name, host| {
                format!(
                    "drvT4UDirect_EM::drvT4U_EM error calling drvAsynIPPortConfigure for UDP \
                     data port={name}, IP={host}, status=-1"
                )
            },
        )?;
        Ok((cmd_io, data_io))
    });

    let (cmd_tx, cmd_rx) = sync_channel::<String>(proto::CMD_QUEUE_LEN);

    Ok(build(
        Build {
            port_name: port_name.to_string(),
            variant,
            transport: Transport { link, runtimes },
            queue: Some(cmd_rx),
            calibration: Some(calibration),
            ring_buffer_size,
            max_memory,
        },
        CmdSink::Queued(cmd_tx),
        claim,
    )?)
}

fn build(spec: Build, cmd: CmdSink, claim: &PortClaim) -> AsynResult<T4uRuntime> {
    let variant = spec.variant;
    let (shared, trigger_rx) = QuadEmShared::new(spec.ring_buffer_size);
    let state: SharedState = Arc::new(parking_lot::Mutex::new(T4uState {
        range: 0,
        cal: ChannelCal::default(),
        full: None,
        sample_time: MIDDLE_SAMPLE_TIME,
        averaging_time: 0.0,
    }));

    let driver = T4uDriver::new(&spec, shared.clone(), state.clone(), cmd)?;
    let Build {
        transport, queue, ..
    } = spec;
    let params = driver.base.params;
    let nd_params = driver.base.nd_params;
    let pool = driver.base.pool.clone();
    let outputs = driver.base.outputs.clone();
    let t4u = driver.t4u;
    let acquire_param = nd_params.acquire;

    let (runtime_handle, _actor) = create_port_runtime(claim, driver, RuntimeConfig::default())?;
    let handle = runtime_handle.port_handle().clone();

    let regs = RegSink {
        handle: handle.clone(),
        params,
        t4u,
        shared: shared.clone(),
        state: state.clone(),
    };

    let meter_threads = transport.link.ok().map(|(cmd_io, data_io)| {
        let cmd_ctx = CmdContext {
            io: cmd_io,
            regs: regs.clone(),
            variant,
            queue,
        };
        let cmd_thread_handle = thread::Builder::new()
            .name(format!("{}_Cmd_Task", variant.name()))
            .spawn(move || cmd_thread(cmd_ctx))
            .expect("failed to spawn the T4U command thread");

        let data_ctx = DataContext {
            io: data_io,
            handle: handle.clone(),
            params,
            shared: shared.clone(),
            state,
            regs,
        };
        let data_thread_handle = thread::Builder::new()
            .name(format!("{}_Data_Task", variant.name()))
            .spawn(move || match variant {
                T4uVariant::Middle => middle_data_thread(data_ctx),
                T4uVariant::Direct => direct_data_thread(data_ctx),
            })
            .expect("failed to spawn the T4U data thread");
        [cmd_thread_handle, data_thread_handle]
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

    Ok(T4uRuntime {
        runtime_handle,
        params,
        nd_params,
        pool,
        outputs,
        sub_ports: transport.runtimes,
        _meter_threads: meter_threads,
        _callback_thread: callback_thread,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::port::{DrvUserRequest, InterfaceMask, PortFlags};
    use epics_rs::asyn::registry::claim_port;

    /// An octet port whose every read times out after delivering `abc`.
    struct PartialThenTimeout(PortDriverBase);

    impl PortDriver for PartialThenTimeout {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            buf[..3].copy_from_slice(b"abc");
            OctetReadResult {
                status: Err(timeout()),
                ..OctetReadResult::ok(3, EomReason::empty())
            }
        }
    }

    fn timeout() -> AsynError {
        AsynError::Status {
            status: AsynStatus::Timeout,
            message: "read timeout".into(),
        }
    }

    fn partial_then_timeout_port(name: &str) -> PortRuntimeHandle {
        let claim = claim_port(name).unwrap();
        let (runtime, _actor) = create_port_runtime(
            &claim,
            PartialThenTimeout(PortDriverBase::new(
                name,
                1,
                PortFlags::default(),
                InterfaceMask::UNMIGRATED_ALL,
                InterfaceMask::UNMIGRATED_INTERRUPTS,
            )),
            RuntimeConfig::default(),
        )
        .unwrap();
        runtime
    }

    /// C++ compares a read-back calibration register with the selected table
    /// at `currRange_`; the middle driver has no table and never complains.
    #[test]
    fn a_calibration_read_back_is_checked_against_the_selected_range() {
        let mut table = CalTable::default();
        table.slope[1][2] = 0.75;
        let mut st = T4uState {
            range: 1,
            cal: ChannelCal::default(),
            full: Some(table),
            sample_time: DIRECT_SAMPLE_TIME,
            averaging_time: 0.0,
        };
        assert!(!st.full_differs(|f| f.slope, 2, 0.75));
        assert!(st.full_differs(|f| f.slope, 2, 0.5));
        st.range = 3;
        assert!(!st.full_differs(|f| f.slope, 2, 0.5));
        st.full = None;
        st.range = 1;
        assert!(!st.full_differs(|f| f.slope, 2, 0.5));
    }

    /// C++ returns from the constructor when `drvAsynIPPortConfigure` fails:
    /// the port exists with drvQuadEM's defaults, and no meter thread runs.
    /// inicpp throws out of the constructor on a malformed line; C iocsh
    /// catches it, so the IOC carries on and only this port is missing.
    #[test]
    fn a_malformed_calibration_file_is_thrown_not_fatal() {
        let path = std::env::temp_dir().join(format!("t4u-bad-{}.ini", std::process::id()));
        std::fs::write(&path, "[config\n").unwrap();
        let err = create_t4u_direct_em(
            &claim_port("T4U_BADINI").unwrap(),
            "127.0.0.1",
            4,
            5000,
            path.to_str().unwrap(),
            0,
            &InterruptGate::opened(),
        )
        .err()
        .expect("no port");
        std::fs::remove_file(&path).unwrap();
        assert!(
            matches!(&err, T4uDirectError::Thrown(what)
                if what == "l.1: ini parsing failed, section not closed"),
            "{err:?}"
        );
    }

    #[test]
    fn a_command_port_that_will_not_configure_keeps_the_port() {
        let rt = create_t4u_em(
            &claim_port("QE_T4U_BADHOST").unwrap(),
            "[bad",
            4,
            5000,
            0,
            &InterruptGate::opened(),
        )
        .unwrap();
        assert!(rt._meter_threads.is_none());
        assert!(rt.sub_ports.is_empty());
        // drvQuadEM never sets P_Model; only the linked constructor does.
        let model = rt.port_handle().read_int32_blocking(rt.params.model, 0);
        assert!(
            matches!(&model, Err(AsynError::ParamAccess { source, .. })
                if matches!(**source, AsynError::ParamUndefined(_))),
            "{model:?}"
        );
        // writeReadMeter's status is ignored, as on a linked port.
        rt.port_handle()
            .write_int32_blocking(rt.params.range, 0, 1)
            .unwrap();
    }

    /// drvT4U_EM's own functions return asynSuccess whatever
    /// `setIntegerParam`/`setDoubleParam` returned (drvT4U_EM.cpp:444-449,
    /// :500-506), so a list the port lacks leaves the record no error.
    #[test]
    fn a_t4u_write_ignores_its_set_param_status() {
        let rt = create_t4u_em(
            &claim_port("QE_T4U_SETPARAM").unwrap(),
            "[bad",
            4,
            5000,
            0,
            &InterruptGate::opened(),
        )
        .unwrap();
        let h = rt.port_handle();
        let reason = |name: &str| {
            h.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
                .unwrap()
                .reason
        };
        let bad_list = crate::drv_quad_em::QE_MAX_DATA as i32 + 1;
        h.write_int32_blocking(reason("QE_BIAS_N"), bad_list, 1)
            .unwrap();
        h.write_float64_blocking(reason("QE_BIAS_N_VOLTAGE"), bad_list, 1.0)
            .unwrap();
    }

    /// C++ `readResponse` appends `nRead` whatever the status, so a socket
    /// read that timed out after some bytes still advances the frame.
    #[test]
    fn a_read_that_times_out_after_bytes_keeps_them() {
        let runtime = partial_then_timeout_port("QE_PARTIAL_TIMEOUT");
        let io = OctetIo::new(runtime.port_handle().clone(), Duration::from_millis(100));
        let mut reader = StreamReader::new(io);
        assert!(reader.fill().unwrap());
        assert_eq!(reader.buf.iter().copied().collect::<Vec<_>>(), b"abc");
    }

    /// The direct meter's datagram read: an empty timeout is an idle line read
    /// again at once, not the one-second backoff a socket failure takes.
    #[test]
    fn a_datagram_timeout_is_idle_and_any_other_failure_backs_off() {
        let read = |data: &[u8], status| OctetRead::new(data.to_vec(), EomReason::empty(), status);
        assert_eq!(datagram(read(b"", Err(timeout()))), Datagram::Idle);
        assert_eq!(datagram(read(b"", Ok(()))), Datagram::Idle);
        assert_eq!(
            datagram(read(b"B\x01", Err(timeout()))),
            Datagram::Frame(b"B\x01".to_vec())
        );
        let reset = AsynError::Status {
            status: AsynStatus::Error,
            message: "ECONNRESET".into(),
        };
        assert_eq!(datagram(read(b"", Err(reset))), Datagram::Failed);
    }
}
