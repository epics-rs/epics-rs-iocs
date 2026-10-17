//! Port of `quadEMApp/quadEMSrc/drvQuadEM.{h,cpp}` — the shared electrometer
//! base.
//!
//! The C++ class derives from `asynNDArrayDriver` with `maxAddr =
//! QE_MAX_DATA+1` (12) and `ASYN_MULTIDEVICE`: addresses 0-10 carry one data
//! item each (4 currents, 2 sums, sum-all, 2 diffs, 2 positions) and address
//! 11 carries the full 2-D array. This module reproduces that layout on top of
//! `asyn_rs::PortDriverBase` plus `ad_core_rs`'s `NDArrayDriverParams` and
//! `DriverPool`, because `NDArrayDriverBase::new` hard-codes `maxAddr = 1`.
//!
//! Sample flow, unchanged from C++:
//!
//! 1. A device read thread produces `raw[4]` and calls
//!    [`QuadEmShared::compute_positions`], which applies offsets/scales,
//!    derives sums/diffs/positions per the geometry, and pushes one
//!    `[f64; QE_MAX_DATA]` sample into the ring buffer.
//! 2. Once `raw_count >= num_average` the read thread triggers callbacks,
//!    handing the accumulated sample count to the callback task.
//! 3. The callback task drains that many samples and publishes them as
//!    NDArrays: one `[QE_MAX_DATA, numRead]` array on address 11 and one
//!    `[numRead]` array on each of addresses 0-10.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use crate::asyn_print;
use crate::trace::AsynPrint;
use epics_rs::ad_core::driver::ndarray_driver::{ArrayOwner, handle_write_int32};
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, NDArrayOutput, NDArraySender};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::user::AsynUser;

/// Number of data items published per sample (C++ `QE_MAX_DATA`).
pub const QE_MAX_DATA: usize = 11;
/// Number of physical electrometer inputs (C++ `QE_MAX_INPUTS`).
pub const QE_MAX_INPUTS: usize = 4;
/// C++ `QE_DEFAULT_RING_BUFFER_SIZE`.
pub const QE_DEFAULT_RING_BUFFER_SIZE: usize = 2048;

/// Offsets into a published sample (C++ `QEData_t`).
pub const QE_CURRENT1: usize = 0;
pub const QE_CURRENT2: usize = 1;
pub const QE_CURRENT3: usize = 2;
pub const QE_CURRENT4: usize = 3;
pub const QE_SUM_X: usize = 4;
pub const QE_SUM_Y: usize = 5;
pub const QE_SUM_ALL: usize = 6;
pub const QE_DIFF_X: usize = 7;
pub const QE_DIFF_Y: usize = 8;
pub const QE_POSITION_X: usize = 9;
pub const QE_POSITION_Y: usize = 10;

/// asyn address carrying the full 2-D array (C++ `doCallbacksGenericPointer(…,
/// QE_MAX_DATA)`).
pub const QE_ADDR_ALL: usize = QE_MAX_DATA;

/// C++ `QEModel_t`. Discriminants are the wire-visible `QE_MODEL` values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeModel {
    Unknown = 0,
    Ah401b = 1,
    Ah401d = 2,
    Ah501 = 3,
    /// Elettra build of the AH501B — different firmware from the CaenEls unit.
    Ah501be = 4,
    Ah501c = 5,
    Ah501d = 6,
    TetrAmm = 7,
    NslsEm = 8,
    Nsls2Em = 9,
    Nsls2Ic = 10,
    Pcr4 = 11,
    SoftDevice = 12,
    SydorEm = 13,
    Fx4 = 14,
}

/// C++ `QEGeometry_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeGeometry {
    Diamond = 0,
    Square = 1,
    SquareCc = 2,
    Custom = 3,
}

impl QeGeometry {
    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => Self::Square,
            2 => Self::SquareCc,
            3 => Self::Custom,
            _ => Self::Diamond,
        }
    }
}

/// C++ `QEAcquireMode_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeAcquireMode {
    Continuous = 0,
    Multiple = 1,
    Single = 2,
}

impl QeAcquireMode {
    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => Self::Multiple,
            2 => Self::Single,
            _ => Self::Continuous,
        }
    }
}

/// C++ `QETriggerMode_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeTriggerMode {
    FreeRun = 0,
    Software = 1,
    ExtTrigger = 2,
    ExtBulb = 3,
    ExtGate = 4,
}

impl QeTriggerMode {
    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => Self::Software,
            2 => Self::ExtTrigger,
            3 => Self::ExtBulb,
            4 => Self::ExtGate,
            _ => Self::FreeRun,
        }
    }
}

/// C++ `QETriggerPolarity_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeTriggerPolarity {
    Positive = 0,
    Negative = 1,
}

impl QeTriggerPolarity {
    pub fn from_i32(v: i32) -> Self {
        if v == 1 {
            Self::Negative
        } else {
            Self::Positive
        }
    }
}

/// C++ `QEReadFormat_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum QeReadFormat {
    Binary = 0,
    Ascii = 1,
}

impl QeReadFormat {
    pub fn from_i32(v: i32) -> Self {
        if v == 1 { Self::Ascii } else { Self::Binary }
    }
}

// ===========================================================================
// Parameters
// ===========================================================================

/// The `QE_*` parameter indices created by C++ `drvQuadEM::drvQuadEM`.
#[derive(Clone, Copy)]
pub struct QuadEmParams {
    pub acquire_mode: usize,
    pub weight_xsum: usize,
    pub weight_ysum: usize,
    pub weight_xdelta: usize,
    pub weight_ydelta: usize,
    pub current_offset: usize,
    pub current_scale: usize,
    pub position_offset: usize,
    pub position_scale: usize,
    pub geometry: usize,
    pub double_data: usize,
    pub int_array_data: usize,
    pub ring_overflows: usize,
    pub read_data: usize,
    pub ping_pong: usize,
    pub integration_time: usize,
    pub sample_time: usize,
    pub range: usize,
    pub reset: usize,
    pub trigger_mode: usize,
    pub trigger_polarity: usize,
    pub num_channels: usize,
    pub bias_state: usize,
    pub bias_voltage: usize,
    pub bias_interlock: usize,
    pub hvs_readback: usize,
    pub hvv_readback: usize,
    pub hvi_readback: usize,
    pub temperature: usize,
    pub read_status: usize,
    pub resolution: usize,
    pub values_per_read: usize,
    pub num_acquire: usize,
    pub num_acquired: usize,
    pub read_format: usize,
    pub averaging_time: usize,
    pub num_average: usize,
    pub num_averaged: usize,
    pub model: usize,
    pub firmware: usize,
}

impl QuadEmParams {
    /// Creates the parameters in the same order as C++ `drvQuadEM::drvQuadEM`.
    pub fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        Ok(Self {
            acquire_mode: base.create_param("QE_ACQUIRE_MODE", ParamType::Int32)?,
            weight_xsum: base.create_param("QE_WEIGHT_XSUM", ParamType::Float64)?,
            weight_ysum: base.create_param("QE_WEIGHT_YSUM", ParamType::Float64)?,
            weight_xdelta: base.create_param("QE_WEIGHT_XDELTA", ParamType::Float64)?,
            weight_ydelta: base.create_param("QE_WEIGHT_YDELTA", ParamType::Float64)?,
            current_offset: base.create_param("QE_CURRENT_OFFSET", ParamType::Float64)?,
            current_scale: base.create_param("QE_CURRENT_SCALE", ParamType::Float64)?,
            position_offset: base.create_param("QE_POSITION_OFFSET", ParamType::Float64)?,
            position_scale: base.create_param("QE_POSITION_SCALE", ParamType::Float64)?,
            geometry: base.create_param("QE_GEOMETRY", ParamType::Int32)?,
            double_data: base.create_param("QE_DOUBLE_DATA", ParamType::Float64)?,
            int_array_data: base.create_param("QE_INT_ARRAY_DATA", ParamType::Int32Array)?,
            ring_overflows: base.create_param("QE_RING_OVERFLOWS", ParamType::Int32)?,
            read_data: base.create_param("QE_READ_DATA", ParamType::Int32)?,
            ping_pong: base.create_param("QE_PING_PONG", ParamType::Int32)?,
            integration_time: base.create_param("QE_INTEGRATION_TIME", ParamType::Float64)?,
            sample_time: base.create_param("QE_SAMPLE_TIME", ParamType::Float64)?,
            range: base.create_param("QE_RANGE", ParamType::Int32)?,
            reset: base.create_param("QE_RESET", ParamType::Int32)?,
            trigger_mode: base.create_param("QE_TRIGGER_MODE", ParamType::Int32)?,
            trigger_polarity: base.create_param("QE_TRIGGER_POLARITY", ParamType::Int32)?,
            num_channels: base.create_param("QE_NUM_CHANNELS", ParamType::Int32)?,
            bias_state: base.create_param("QE_BIAS_STATE", ParamType::Int32)?,
            bias_voltage: base.create_param("QE_BIAS_VOLTAGE", ParamType::Float64)?,
            bias_interlock: base.create_param("QE_BIAS_INTERLOCK", ParamType::Int32)?,
            hvs_readback: base.create_param("QE_HVS_READBACK", ParamType::Int32)?,
            hvv_readback: base.create_param("QE_HVV_READBACK", ParamType::Float64)?,
            hvi_readback: base.create_param("QE_HVI_READBACK", ParamType::Float64)?,
            temperature: base.create_param("QE_TEMPERATURE", ParamType::Float64)?,
            read_status: base.create_param("QE_READ_STATUS", ParamType::Int32)?,
            resolution: base.create_param("QE_RESOLUTION", ParamType::Int32)?,
            values_per_read: base.create_param("QE_VALUES_PER_READ", ParamType::Int32)?,
            num_acquire: base.create_param("QE_NUM_ACQUIRE", ParamType::Int32)?,
            num_acquired: base.create_param("QE_NUM_ACQUIRED", ParamType::Int32)?,
            read_format: base.create_param("QE_READ_FORMAT", ParamType::Int32)?,
            averaging_time: base.create_param("QE_AVERAGING_TIME", ParamType::Float64)?,
            num_average: base.create_param("QE_NUM_AVERAGE", ParamType::Int32)?,
            num_averaged: base.create_param("QE_NUM_AVERAGED", ParamType::Int32)?,
            model: base.create_param("QE_MODEL", ParamType::Int32)?,
            firmware: base.create_param("QE_FIRMWARE", ParamType::Octet)?,
        })
    }
}

// ===========================================================================
// Position computation
// ===========================================================================

/// Everything `drvQuadEM::computePositions` reads out of the parameter library.
///
/// C++ re-reads these per sample under the port lock. The read thread here is
/// outside the port actor, so the driver mirrors each write into this snapshot
/// (the only writers are `writeInt32`/`writeFloat64`, exactly as in C++, so the
/// values seen by a sample are the same ones C++ would have read).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionConfig {
    pub geometry: i32,
    pub current_offset: [f64; QE_MAX_INPUTS],
    pub current_scale: [f64; QE_MAX_INPUTS],
    pub position_offset: [f64; 2],
    pub position_scale: [f64; 2],
    pub weight_xsum: [f64; QE_MAX_INPUTS],
    pub weight_ysum: [f64; QE_MAX_INPUTS],
    pub weight_xdelta: [f64; QE_MAX_INPUTS],
    pub weight_ydelta: [f64; QE_MAX_INPUTS],
}

impl Default for PositionConfig {
    fn default() -> Self {
        Self {
            geometry: QeGeometry::Diamond as i32,
            current_offset: [0.0; QE_MAX_INPUTS],
            current_scale: [0.0; QE_MAX_INPUTS],
            position_offset: [0.0; 2],
            position_scale: [0.0; 2],
            weight_xsum: [0.0; QE_MAX_INPUTS],
            weight_ysum: [0.0; QE_MAX_INPUTS],
            weight_xdelta: [0.0; QE_MAX_INPUTS],
            weight_ydelta: [0.0; QE_MAX_INPUTS],
        }
    }
}

/// Port of `drvQuadEM::computePositions`'s pure arithmetic: scale/offset the
/// four raw currents, derive the sums and diffs for the configured geometry,
/// then normalise the diffs into positions.
pub fn compute_positions(raw: &[f64; QE_MAX_INPUTS], cfg: &PositionConfig) -> [f64; QE_MAX_DATA] {
    let mut d = [0.0f64; QE_MAX_DATA];

    for i in 0..QE_MAX_INPUTS {
        d[i] = raw[i] * cfg.current_scale[i] - cfg.current_offset[i];
    }

    d[QE_SUM_ALL] = d[QE_CURRENT1] + d[QE_CURRENT2] + d[QE_CURRENT3] + d[QE_CURRENT4];

    match QeGeometry::from_i32(cfg.geometry) {
        QeGeometry::Square => {
            d[QE_SUM_X] = d[QE_SUM_ALL];
            d[QE_SUM_Y] = d[QE_SUM_ALL];
            d[QE_DIFF_X] = (d[QE_CURRENT2] + d[QE_CURRENT3]) - (d[QE_CURRENT1] + d[QE_CURRENT4]);
            d[QE_DIFF_Y] = (d[QE_CURRENT1] + d[QE_CURRENT2]) - (d[QE_CURRENT3] + d[QE_CURRENT4]);
        }
        QeGeometry::SquareCc => {
            d[QE_SUM_X] = d[QE_SUM_ALL];
            d[QE_SUM_Y] = d[QE_SUM_ALL];
            d[QE_DIFF_X] = (d[QE_CURRENT3] + d[QE_CURRENT4]) - (d[QE_CURRENT1] + d[QE_CURRENT2]);
            d[QE_DIFF_Y] = (d[QE_CURRENT1] + d[QE_CURRENT4]) - (d[QE_CURRENT2] + d[QE_CURRENT3]);
        }
        QeGeometry::Diamond => {
            d[QE_SUM_X] = d[QE_CURRENT1] + d[QE_CURRENT2];
            d[QE_SUM_Y] = d[QE_CURRENT3] + d[QE_CURRENT4];
            d[QE_DIFF_X] = d[QE_CURRENT2] - d[QE_CURRENT1];
            d[QE_DIFF_Y] = d[QE_CURRENT4] - d[QE_CURRENT3];
        }
        QeGeometry::Custom => {
            let c = [
                d[QE_CURRENT1],
                d[QE_CURRENT2],
                d[QE_CURRENT3],
                d[QE_CURRENT4],
            ];
            let dot =
                |w: &[f64; QE_MAX_INPUTS]| w[0] * c[0] + w[1] * c[1] + w[2] * c[2] + w[3] * c[3];
            d[QE_SUM_X] = dot(&cfg.weight_xsum);
            d[QE_SUM_Y] = dot(&cfg.weight_ysum);
            d[QE_DIFF_X] = dot(&cfg.weight_xdelta);
            d[QE_DIFF_Y] = dot(&cfg.weight_ydelta);
        }
    }

    let denom_x = if d[QE_SUM_X] == 0.0 { 1.0 } else { d[QE_SUM_X] };
    let denom_y = if d[QE_SUM_Y] == 0.0 { 1.0 } else { d[QE_SUM_Y] };
    d[QE_POSITION_X] = (cfg.position_scale[0] * d[QE_DIFF_X] / denom_x) - cfg.position_offset[0];
    d[QE_POSITION_Y] = (cfg.position_scale[1] * d[QE_DIFF_Y] / denom_y) - cfg.position_offset[1];

    d
}

/// `getIntegerParam` as the quadEM drivers call it, status ignored: a param
/// nothing has set yet reads as zero, as C's getter stores it
/// (asynPortDriver.cpp:304).
pub fn int_param(base: &PortDriverBase, index: usize, addr: i32) -> i32 {
    base.get_int32_param(index, addr).unwrap_or(0)
}

/// `getDoubleParam` with its status ignored; see [`int_param`].
pub fn double_param(base: &PortDriverBase, index: usize, addr: i32) -> f64 {
    base.get_float64_param(index, addr).unwrap_or(0.0)
}

/// C++ `numAverage = (int)((averagingTime / sampleTime) + 0.5)`.
///
/// Reproduces the C truncating cast (including the negative and non-finite
/// cases, where C++ would produce an implementation-defined result and this
/// yields 0).
pub fn num_average_from(averaging_time: f64, sample_time: f64) -> i32 {
    let n = (averaging_time / sample_time) + 0.5;
    if n.is_finite() { n as i32 } else { 0 }
}

// ===========================================================================
// Ring buffer
// ===========================================================================

/// The C++ `epicsRingBytes` of `ringBufferSize * QE_MAX_DATA * sizeof(double)`
/// bytes, expressed in whole samples.
#[derive(Debug)]
pub struct RingState {
    buf: VecDeque<[f64; QE_MAX_DATA]>,
    capacity: usize,
    /// C++ `ringCount_`: samples written but not yet drained by a callback.
    pub ring_count: i32,
    /// C++ `rawCount_`: samples accumulated since the last trigger.
    pub raw_count: i32,
    /// C++ `P_RingOverflows`.
    pub ring_overflows: i32,
}

impl RingState {
    pub fn new(capacity: usize) -> Self {
        let capacity = if capacity == 0 {
            QE_DEFAULT_RING_BUFFER_SIZE
        } else {
            capacity
        };
        Self {
            buf: VecDeque::with_capacity(capacity),
            capacity,
            ring_count: 0,
            raw_count: 0,
            ring_overflows: 0,
        }
    }

    /// Push one sample. When the ring is full the oldest sample is discarded
    /// and `ring_overflows` incremented, mirroring the C++ pre-write check
    /// `epicsRingBytesFreeBytes(...) < sizeof(doubleData)`.
    ///
    /// Returns `true` when an overflow occurred.
    pub fn push(&mut self, sample: [f64; QE_MAX_DATA]) -> bool {
        let overflowed = self.buf.len() >= self.capacity;
        if overflowed {
            self.buf.pop_front();
            self.ring_count -= 1;
            self.raw_count -= 1;
            self.ring_overflows += 1;
        }
        self.buf.push_back(sample);
        self.ring_count += 1;
        self.raw_count += 1;
        overflowed
    }

    /// C++ `epicsRingBytesUsedBytes / sampleSize`.
    pub fn used(&self) -> usize {
        self.buf.len()
    }

    /// Drain `n` samples, oldest first. `None` when fewer than `n` are
    /// buffered (C++ `doDataCallbacks` bails out with `asynError`).
    pub fn drain(&mut self, n: usize) -> Option<Vec<[f64; QE_MAX_DATA]>> {
        if self.buf.len() < n {
            return None;
        }
        let out: Vec<_> = self.buf.drain(..n).collect();
        self.ring_count -= n as i32;
        Some(out)
    }

    /// C++ `epicsRingBytesFlush` plus the counter reset that always accompanies
    /// it in `writeInt32`/`writeFloat64`.
    pub fn flush(&mut self) {
        self.buf.clear();
        self.ring_count = 0;
        self.raw_count = 0;
    }

    /// Take `raw_count` and zero it (C++ `triggerCallbacks`). `None` when there
    /// is nothing to publish.
    pub fn take_raw_count(&mut self) -> Option<i32> {
        if self.raw_count < 1 {
            return None;
        }
        let n = self.raw_count;
        self.raw_count = 0;
        Some(n)
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

// ===========================================================================
// Acquisition state shared between the port actor and the device threads
// ===========================================================================

/// The cached values C++ keeps as `resolution_`, `numChannels_`,
/// `valuesPerRead_`, `acquiring_`, `numAcquired_` plus the acquire-mode
/// parameters read by `computePositions` / `callbackTask`.
#[derive(Debug, Clone, Copy)]
pub struct AcqState {
    pub num_average: i32,
    pub acquire_mode: i32,
    pub num_acquire: i32,
    pub num_acquired: i32,
    pub trigger_mode: i32,
    pub read_format: i32,
    pub num_channels: i32,
    pub values_per_read: i32,
    pub resolution: i32,
}

impl Default for AcqState {
    fn default() -> Self {
        Self {
            num_average: 0,
            acquire_mode: QeAcquireMode::Continuous as i32,
            num_acquire: 0,
            num_acquired: 0,
            trigger_mode: QeTriggerMode::FreeRun as i32,
            read_format: QeReadFormat::Binary as i32,
            num_channels: 4,
            values_per_read: 1,
            resolution: 16,
        }
    }
}

/// A one-shot latch with the semantics of `epicsEvent`: `signal` wakes exactly
/// one `wait`, and a signal delivered before the wait is remembered.
#[derive(Default)]
pub struct Event {
    signalled: parking_lot::Mutex<bool>,
    cv: parking_lot::Condvar,
}

impl Event {
    pub fn signal(&self) {
        *self.signalled.lock() = true;
        self.cv.notify_one();
    }

    /// Block until signalled, then consume the signal.
    pub fn wait(&self) {
        let mut g = self.signalled.lock();
        while !*g {
            self.cv.wait(&mut g);
        }
        *g = false;
    }
}

/// State the port actor, the device read thread and the callback task all
/// touch.
pub struct QuadEmShared {
    /// C++ `acquiring_`.
    pub acquiring: AtomicBool,
    /// C++ `readingActive_`.
    pub reading_active: AtomicBool,
    /// C++ `acquireStartEvent_`.
    pub acquire_start: Event,
    pub ring: parking_lot::Mutex<RingState>,
    pub pos: parking_lot::Mutex<PositionConfig>,
    pub acq: parking_lot::Mutex<AcqState>,
    /// C++ `msgQId_`: carries `rawCount_` from `triggerCallbacks` to
    /// `callbackTask`.
    trigger_tx: rt::CommandSender<i32>,
    /// Mirrors `NDArrayCounter` so the callback task need not round-trip the
    /// actor to increment it.
    pub array_counter: AtomicI32,
}

/// C's `int status; status |= ...`: the OR of every step's `asynStatus`, so
/// a timeout and an overflow return asynError's 3 and a param-library
/// refusal its own code.
#[derive(Default)]
struct StatusOr(i32);

impl StatusOr {
    fn add<T>(&mut self, r: AsynResult<T>) {
        if let Err(e) = r {
            self.0 |= e.c_status_code();
        }
    }

    /// C's `status = ...`, which drops what the earlier steps returned.
    fn set<T>(&mut self, r: AsynResult<T>) {
        self.0 = 0;
        self.add(r);
    }

    /// C's `return (asynStatus)status` with the errorMessage it formats.
    fn into_result(self, message: impl FnOnce(i32) -> String) -> AsynResult<()> {
        if self.0 == 0 {
            return Ok(());
        }
        AsynError::from_c_status_code(self.0, message(self.0)).map_or(Ok(()), Err)
    }
}

/// The tail C's `writeInt32`/`writeFloat64` share: the errorMessage on a
/// failure, the `ASYN_TRACEIO_DRIVER` line otherwise.
fn qe_write_result(
    base: &PortDriverBase,
    status: StatusOr,
    function_name: &str,
    reason: usize,
    value: &str,
) -> AsynResult<()> {
    let name = base.params.param_name(reason).unwrap_or("");
    if status.0 == 0 {
        asyn_print!(
            *base,
            IO_DRIVER,
            "drvQuadEM:{function_name}: function={reason}, name={name}, value={value}\n"
        );
    }
    status.into_result(|code| {
        format!(
            "drvQuadEM:{function_name}: status={code}, function={reason}, name={name}, value={value}"
        )
    })
}

impl QuadEmShared {
    /// The read threads' and callback task's copy of an integer parameter C
    /// reads back with `getIntegerParam` (or `valuesPerRead_`, which
    /// `drvQuadEM::writeInt32` assigns): the written value, raw.
    fn shadow_int32(&self, p: &QuadEmParams, reason: usize, value: i32) {
        if reason == p.geometry {
            self.pos.lock().geometry = value;
        } else if reason == p.acquire_mode {
            self.acq.lock().acquire_mode = value;
        } else if reason == p.num_acquire {
            self.acq.lock().num_acquire = value;
        } else if reason == p.values_per_read {
            self.acq.lock().values_per_read = value;
        }
    }

    /// As [`Self::shadow_int32`], for the per-input position settings.
    fn shadow_float64(&self, p: &QuadEmParams, reason: usize, addr: i32, value: f64) {
        let i = addr.clamp(0, QE_MAX_INPUTS as i32 - 1) as usize;
        let j = addr.clamp(0, 1) as usize;
        let mut pos = self.pos.lock();
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

    pub fn new(ring_buffer_size: usize) -> (Arc<Self>, rt::CommandReceiver<i32>) {
        let capacity = if ring_buffer_size == 0 {
            QE_DEFAULT_RING_BUFFER_SIZE
        } else {
            ring_buffer_size
        };
        let (trigger_tx, trigger_rx) = rt::command_channel::<i32>(capacity);
        let shared = Arc::new(Self {
            acquiring: AtomicBool::new(false),
            reading_active: AtomicBool::new(false),
            acquire_start: Event::default(),
            ring: parking_lot::Mutex::new(RingState::new(capacity)),
            pos: parking_lot::Mutex::new(PositionConfig::default()),
            acq: parking_lot::Mutex::new(AcqState::default()),
            trigger_tx,
            array_counter: AtomicI32::new(0),
        });
        (shared, trigger_rx)
    }

    pub fn is_acquiring(&self) -> bool {
        self.acquiring.load(Ordering::SeqCst)
    }

    pub fn set_acquiring(&self, v: bool) {
        self.acquiring.store(v, Ordering::SeqCst);
    }

    pub fn is_reading_active(&self) -> bool {
        self.reading_active.load(Ordering::SeqCst)
    }

    pub fn set_reading_active(&self, v: bool) {
        self.reading_active.store(v, Ordering::SeqCst);
    }

    /// C++ `drvQuadEM::triggerCallbacks`. Returns `false` when nothing was
    /// pending (`rawCount_ < 1`, e.g. a Read press while idle). `port` is the
    /// driver port the failure is reported on.
    pub fn trigger_callbacks(&self, port: &dyn AsynPrint) -> AsynResult<()> {
        let Some(n) = self.ring.lock().take_raw_count() else {
            return Ok(());
        };
        if self.trigger_tx.try_send(n).is_err() {
            asyn_print!(
                *port,
                ERROR,
                "drvQuadEM::triggerCallbacks, error calling epicsMessageQueueSend\n"
            );
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: String::new(),
            });
        }
        Ok(())
    }

    /// C++ `drvQuadEM::computePositions`: derive the sample, ring it, trigger
    /// callbacks once `numAverage` samples have accumulated, and publish the
    /// per-address `QE_DOUBLE_DATA` / `QE_INT_ARRAY_DATA` parameters.
    ///
    /// Called from the device read thread, so parameter updates are enqueued on
    /// the port actor rather than written directly.
    pub fn compute_positions(
        &self,
        handle: &PortHandle,
        params: &QuadEmParams,
        raw: &[f64; QE_MAX_INPUTS],
    ) {
        let cfg = *self.pos.lock();
        let sample = compute_positions(raw, &cfg);

        let (overflowed, ring_overflows) = {
            let mut ring = self.ring.lock();
            let overflowed = ring.push(sample);
            (overflowed, ring.ring_overflows)
        };
        if overflowed {
            asyn_print!(
                *handle,
                WARNING,
                "drvQuadEM::computePositions warning ring buffer overflow\n"
            );
            let _ = handle.set_params_and_notify_blocking(
                0,
                vec![ParamSetValue::new(
                    params.ring_overflows,
                    0,
                    ParamValue::Int32(ring_overflows),
                )],
            );
        }

        let num_average = self.acq.lock().num_average;
        if num_average > 0 && self.ring.lock().raw_count >= num_average {
            let _ = self.trigger_callbacks(handle);
        }

        // C++ sets QE_DOUBLE_DATA and fires callbacks on each of the 11
        // addresses, then does one Int32Array callback of the truncated values.
        for (addr, value) in sample.iter().enumerate() {
            let _ = handle.set_params_and_notify_blocking(
                addr as i32,
                vec![ParamSetValue::new(
                    params.double_data,
                    addr as i32,
                    ParamValue::Float64(*value),
                )],
            );
        }
        let int_data: Vec<i32> = sample.iter().map(|v| *v as i32).collect();
        let _ = handle.set_params_and_notify_blocking(
            0,
            vec![ParamSetValue::new(
                params.int_array_data,
                0,
                ParamValue::Int32Array(int_data.into()),
            )],
        );
    }
}

// ===========================================================================
// Device hooks
// ===========================================================================

/// The `virtual` surface of `drvQuadEM` that each electrometer overrides.
///
/// The defaults are C++'s "dummy implementations of set functions ... called
/// when a derived class does not implement a function" (`drvQuadEM.cpp`), so a
/// device only implements what its meter supports. [`Self::base_reset`] and
/// [`Self::base_set_acquire`] are `drvQuadEM::reset` and
/// `drvQuadEM::setAcquire`.
pub trait QuadEmDevice {
    fn qe_base(&mut self) -> &mut QuadEmBase;
    fn qe_shared(&self) -> &Arc<QuadEmShared>;

    /// `drvQuadEM::setAcquire` is pure virtual — every device implements it.
    fn set_acquire(&mut self, value: i32) -> AsynResult<()>;

    fn set_range(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_values_per_read(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_averaging_time(&mut self, _value: f64) -> AsynResult<()> {
        Ok(())
    }
    fn set_trigger_mode(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_num_channels(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_bias_state(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_bias_interlock(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_bias_voltage(&mut self, _value: f64) -> AsynResult<()> {
        Ok(())
    }
    fn set_resolution(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_read_format(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_integration_time(&mut self, _value: f64) -> AsynResult<()> {
        Ok(())
    }
    fn set_ping_pong(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_acquire_mode(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_trigger_polarity(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn set_num_acquire(&mut self, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    /// `drvQuadEM::setRange(int chan, epicsInt32 value)`.
    fn set_channel_range(&mut self, _channel: i32, _value: i32) -> AsynResult<()> {
        Ok(())
    }
    fn read_status(&mut self) -> AsynResult<()> {
        Ok(())
    }
    /// `drvQuadEM::reset` is pure virtual; [`Self::base_reset`] is its body.
    fn reset(&mut self) -> AsynResult<()>;

    /// C++ `drvQuadEM::writeInt32`. Every step runs whatever an earlier one
    /// returned, as C's `status |=` does; the first failure is what the record
    /// sees.
    fn qe_write_int32(&mut self, user: &AsynUser, value: i32) -> AsynResult<()> {
        let (reason, channel) = (user.reason, user.addr);
        let p = self.qe_base().params;
        let acquire = self.qe_base().nd_params.acquire;
        let shared = self.qe_shared().clone();
        let mut status = StatusOr::default();

        status.add(
            self.qe_base()
                .port_base
                .set_int32_param(reason, channel, value),
        );
        shared.shadow_int32(&p, reason, value);

        if reason == acquire {
            if value != 0 {
                shared.ring.lock().flush();
            }
            status.add(self.set_acquire(value));
        } else if reason == p.acquire_mode {
            if QeAcquireMode::from_i32(value) != QeAcquireMode::Continuous {
                status.add(self.set_acquire(0));
                let _ = self.qe_base().port_base.set_int32_param(acquire, 0, 0);
            }
            status.add(self.set_acquire_mode(value));
            status.add(self.read_status());
        } else if reason == p.bias_state {
            status.add(self.set_bias_state(value));
            status.add(self.read_status());
        } else if reason == p.bias_interlock {
            status.add(self.set_bias_interlock(value));
            status.add(self.read_status());
        } else if reason == p.num_channels {
            status.add(self.set_num_channels(value));
            status.add(self.read_status());
        } else if reason == p.num_acquire {
            status.add(self.set_num_acquire(value));
            status.add(self.read_status());
        } else if reason == p.ping_pong {
            status.add(self.set_ping_pong(value));
            status.add(self.read_status());
        } else if reason == p.range {
            if channel == 0 {
                status.add(self.set_range(value));
            } else {
                status.add(self.set_channel_range(channel, value));
            }
            status.add(self.read_status());
        } else if reason == p.read_data {
            status.add(shared.trigger_callbacks(&self.qe_base().port_base));
        } else if reason == p.resolution {
            status.add(self.set_resolution(value));
            status.add(self.read_status());
        } else if reason == p.trigger_mode {
            status.add(self.set_trigger_mode(value));
            status.add(self.read_status());
        } else if reason == p.trigger_polarity {
            status.add(self.set_trigger_polarity(value));
            status.add(self.read_status());
        } else if reason == p.values_per_read {
            status.add(self.set_values_per_read(value));
            status.add(self.read_status());
        } else if reason == p.read_format {
            status.add(self.set_read_format(value));
            status.add(self.read_status());
        } else if reason == p.read_status {
            // Not while acquiring: too disruptive.
            if !shared.is_acquiring() {
                status.add(self.read_status());
            }
        } else if reason == p.reset {
            status.add(self.reset());
            status.add(self.read_status());
        } else if reason < p.acquire_mode {
            // FIRST_QE_COMMAND is P_AcquireMode (drvQuadEM.h:151).
            let b = self.qe_base();
            let template = b.owner.array(0);
            status.set(handle_write_int32(
                &mut b.port_base,
                &b.nd_params,
                &b.pool,
                template.as_deref(),
                user,
                value,
            ));
        }

        // C's callParamCallbacks() posts address 0 only; the other addresses
        // post from computePositions while data flows.
        status.add(self.qe_base().port_base.call_param_callbacks(0));
        qe_write_result(
            &self.qe_base().port_base,
            status,
            "writeInt32",
            reason,
            &value.to_string(),
        )
    }

    /// C++ `drvQuadEM::writeFloat64`, with [`Self::qe_write_int32`]'s status rule.
    fn qe_write_float64(&mut self, user: &AsynUser, value: f64) -> AsynResult<()> {
        let (reason, channel) = (user.reason, user.addr);
        let p = self.qe_base().params;
        let shared = self.qe_shared().clone();
        let mut status = StatusOr::default();

        status.add(
            self.qe_base()
                .port_base
                .set_float64_param(reason, channel, value),
        );
        shared.shadow_float64(&p, reason, channel, value);

        if reason == p.averaging_time {
            status.add(self.set_averaging_time(value));
            shared.ring.lock().flush();
            status.add(self.read_status());
        } else if reason == p.bias_voltage {
            status.add(self.set_bias_voltage(value));
            status.add(self.read_status());
        } else if reason == p.integration_time {
            status.add(self.set_integration_time(value));
            status.add(self.read_status());
        }

        status.add(self.qe_base().port_base.call_param_callbacks(0));
        qe_write_result(
            &self.qe_base().port_base,
            status,
            "writeFloat64",
            reason,
            &format!("{value:.6}"),
        )
    }

    /// `drvQuadEM::setAcquire`: starting an acquisition clears `NumAcquired`.
    /// C discards both statuses and returns asynSuccess.
    fn base_set_acquire(&mut self, value: i32) {
        if value == 1 {
            self.qe_shared().acq.lock().num_acquired = 0;
            let idx = self.qe_base().params.num_acquired;
            let _ = self.qe_base().port_base.set_int32_param(idx, 0, 0);
            let _ = self.qe_base().port_base.call_param_callbacks(0);
        }
    }

    /// `drvQuadEM::reset`: push every cached EPICS setting back to the meter,
    /// re-read the status, then restore the acquire state. C++ discards every
    /// status, the last `setAcquire`'s included, and returns success.
    fn base_reset(&mut self) -> AsynResult<()> {
        let p = self.qe_base().params;
        let acquire_param = self.qe_base().nd_params.acquire;
        // C's getters zero `iValue` / `dValue` before an unset param fails
        // (asynPortDriver.cpp:304,386).
        let int = |s: &mut Self, index| int_param(&s.qe_base().port_base, index, 0);
        let double = |s: &mut Self, index| double_param(&s.qe_base().port_base, index, 0);

        let v = int(self, p.range);
        let _ = self.set_range(v);
        let v = int(self, p.values_per_read);
        let _ = self.set_values_per_read(v);
        let v = double(self, p.averaging_time);
        let _ = self.set_averaging_time(v);
        let v = int(self, p.trigger_mode);
        let _ = self.set_trigger_mode(v);
        let v = int(self, p.num_channels);
        let _ = self.set_num_channels(v);
        let v = int(self, p.bias_state);
        let _ = self.set_bias_state(v);
        let v = int(self, p.bias_interlock);
        let _ = self.set_bias_interlock(v);
        let v = double(self, p.bias_voltage);
        let _ = self.set_bias_voltage(v);
        let v = int(self, p.resolution);
        let _ = self.set_resolution(v);
        let v = int(self, p.read_format);
        let _ = self.set_read_format(v);
        let v = double(self, p.integration_time);
        let _ = self.set_integration_time(v);
        let _ = self.read_status();
        let v = int(self, acquire_param);
        let _ = self.set_acquire(v);
        Ok(())
    }
}

// ===========================================================================
// Port base
// ===========================================================================

/// The `asynNDArrayDriver` slice of `drvQuadEM`: a 12-address multi-device port
/// plus the NDArray pool and one fan-out per address.
pub struct QuadEmBase {
    pub port_base: PortDriverBase,
    pub nd_params: NDArrayDriverParams,
    pub params: QuadEmParams,
    pub pool: DriverPool,
    /// One output per asyn address: 0-10 single-value streams, 11 the full
    /// 2-D array.
    pub outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    /// Keeps the pool's owner alive; the pool holds it weakly.
    pub owner: Arc<ArrayOwner>,
}

impl QuadEmBase {
    pub fn new(port_name: &str, max_memory: usize) -> AsynResult<Self> {
        let mut port_base = PortDriverBase::new(
            port_name,
            QE_MAX_DATA + 1,
            PortFlags {
                can_block: true,
                multi_device: true,
                ..Default::default()
            },
        );

        let (nd_params, owner) = NDArrayDriverParams::create(&mut port_base)?;
        let params = QuadEmParams::create(&mut port_base)?;

        port_base.set_int32_param(nd_params.array_callbacks, 0, 1)?;
        port_base.set_float64_param(
            nd_params.pool_max_memory,
            0,
            max_memory as f64 / 1_048_576.0,
        )?;

        // C++ drvQuadEM constructor defaults.
        port_base.set_int32_param(params.ring_overflows, 0, 0)?;
        port_base.set_int32_param(params.ping_pong, 0, 0)?;
        port_base.set_float64_param(params.integration_time, 0, 0.0)?;
        // Defined so drvEpidFast does not error on an undefined SampleTime.
        port_base.set_float64_param(params.sample_time, 0, 0.1)?;
        port_base.set_int32_param(params.range, 0, 0)?;
        port_base.set_int32_param(params.trigger_mode, 0, 0)?;
        port_base.set_int32_param(params.num_channels, 0, 4)?;
        port_base.set_int32_param(params.bias_state, 0, 0)?;
        port_base.set_float64_param(params.bias_voltage, 0, 0.0)?;
        port_base.set_int32_param(params.resolution, 0, 16)?;
        port_base.set_int32_param(params.values_per_read, 0, 1)?;
        port_base.set_int32_param(params.read_format, 0, 0)?;
        for addr in 0..QE_MAX_DATA {
            port_base.set_float64_param(params.double_data, addr as i32, 0.0)?;
        }

        let pool = DriverPool::for_driver(max_memory, &owner);

        let outputs = (0..=QE_MAX_DATA)
            .map(|_| Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())))
            .collect();

        Ok(Self {
            port_base,
            nd_params,
            params,
            pool,
            outputs,
            owner,
        })
    }

    /// Attach a downstream plugin to one asyn address' NDArray stream.
    pub fn connect_downstream(&self, addr: usize, sender: NDArraySender) {
        let Some(out) = self.outputs.get(addr) else {
            asyn_print!(
                self.port_base,
                ERROR,
                "drvQuadEM::connect_downstream: address {addr} out of range\n"
            );
            return;
        };
        out.lock().add(sender);
    }
}

// ===========================================================================
// Callback task
// ===========================================================================

/// Everything the callback task needs; mirrors C++ `drvQuadEM::callbackTask`.
pub struct CallbackContext {
    pub trigger_rx: rt::CommandReceiver<i32>,
    pub handle: PortHandle,
    pub params: QuadEmParams,
    pub nd_params: NDArrayDriverParams,
    pub outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    pub pool: DriverPool,
    pub shared: Arc<QuadEmShared>,
    /// Index of `ADAcquire` on this port, written to stop a finished
    /// multiple/single acquisition.
    pub acquire_param: usize,
}

/// C++ `drvQuadEM::doDataCallbacks`.
async fn do_data_callbacks(ctx: &CallbackContext, num_read: usize) -> bool {
    let drained = {
        let mut ring = ctx.shared.ring.lock();
        ring.drain(num_read)
            .ok_or((ring.used(), ring.raw_count, ring.ring_count))
    };
    let samples = match drained {
        Ok(samples) => samples,
        Err((ring_size, raw_count, ring_count)) => {
            asyn_print!(
                ctx.handle,
                ERROR,
                "drvQuadEM::doDataCallbacks: not enough bytes in ring buffer, expecting {num_read} actually {ring_size}, rawCount_={raw_count}, ringCount_={ring_count}\n"
            );
            return false;
        }
    };

    let counter = ctx.shared.array_counter.fetch_add(1, Ordering::SeqCst) + 1;
    let ts = EpicsTimestamp::now();
    let time_stamp = ts.as_f64();

    // Address 11: the whole [QE_MAX_DATA, numRead] array, sample-major on disk
    // exactly as the C++ ring layout.
    let mut flat = Vec::with_capacity(num_read * QE_MAX_DATA);
    for s in &samples {
        flat.extend_from_slice(s);
    }
    // C dereferences a refused alloc unchecked (drvQuadEM.cpp:333, :351); here
    // that array is dropped after the pool's limit line.
    if let Ok(mut all) = ctx.pool.wrap(
        vec![NDDimension::new(QE_MAX_DATA), NDDimension::new(num_read)],
        NDDataBuffer::F64(flat),
    ) {
        all.unique_id = counter;
        all.timestamp = ts;
        all.time_stamp = time_stamp;
        ArrayPublisher::new(ctx.outputs[QE_ADDR_ALL].clone())
            .publish(all)
            .await;
    }

    // Addresses 0-10: one [numRead] array per data item.
    for item in 0..QE_MAX_DATA {
        let column: Vec<f64> = samples.iter().map(|s| s[item]).collect();
        let Ok(mut single) = ctx
            .pool
            .wrap(vec![NDDimension::new(num_read)], NDDataBuffer::F64(column))
        else {
            continue;
        };
        single.unique_id = counter;
        single.timestamp = ts;
        single.time_stamp = time_stamp;
        ArrayPublisher::new(ctx.outputs[item].clone())
            .publish(single)
            .await;
    }

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::new(ctx.nd_params.array_counter, 0, ParamValue::Int32(counter)),
                ParamSetValue::new(
                    ctx.params.num_averaged,
                    0,
                    ParamValue::Int32(num_read as i32),
                ),
                ParamSetValue::new(ctx.params.ring_overflows, 0, ParamValue::Int32(0)),
            ],
        )
        .await;
    ctx.shared.ring.lock().ring_overflows = 0;
    true
}

/// C++ `drvQuadEM::callbackTask`.
pub async fn callback_loop(mut ctx: CallbackContext) {
    while let Some(num_read) = ctx.trigger_rx.recv().await {
        if num_read < 1 {
            continue;
        }
        let (acquire_mode, mut num_acquire, num_acquired) = {
            let a = ctx.shared.acq.lock();
            (a.acquire_mode, a.num_acquire, a.num_acquired)
        };
        if QeAcquireMode::from_i32(acquire_mode) == QeAcquireMode::Single {
            num_acquire = 1;
        }

        if QeAcquireMode::from_i32(acquire_mode) == QeAcquireMode::Continuous {
            if !do_data_callbacks(&ctx, num_read as usize).await {
                continue;
            }
            let n = {
                let mut a = ctx.shared.acq.lock();
                a.num_acquired += 1;
                a.num_acquired
            };
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ctx.params.num_acquired,
                        0,
                        ParamValue::Int32(n),
                    )],
                )
                .await;
        } else if num_acquired < num_acquire {
            if !do_data_callbacks(&ctx, num_read as usize).await {
                continue;
            }
            let n = {
                let mut a = ctx.shared.acq.lock();
                a.num_acquired += 1;
                a.num_acquired
            };
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ctx.params.num_acquired,
                        0,
                        ParamValue::Int32(n),
                    )],
                )
                .await;
            if n == num_acquire {
                // C++ calls setAcquire(0) then setIntegerParam(ADAcquire, 0);
                // routing through ADAcquire does both on the port actor.
                let _ = ctx.handle.write_int32(ctx.acquire_param, 0, 0).await;
            }
        }
    }
}

/// Spawn the callback task on its own thread (C++ `drvQuadEMCallbackTask`).
pub fn start_callback_task(ctx: CallbackContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("drvQuadEMCallbackTask", move || callback_loop(ctx))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::TraceMask;

    fn unit_cfg(geometry: QeGeometry) -> PositionConfig {
        PositionConfig {
            geometry: geometry as i32,
            current_scale: [1.0; 4],
            position_scale: [1.0; 2],
            ..Default::default()
        }
    }

    #[test]
    fn diamond_geometry_matches_cpp() {
        let cfg = unit_cfg(QeGeometry::Diamond);
        let d = compute_positions(&[1.0, 2.0, 4.0, 8.0], &cfg);
        assert_eq!(d[QE_SUM_ALL], 15.0);
        assert_eq!(d[QE_SUM_X], 3.0); // c1 + c2
        assert_eq!(d[QE_SUM_Y], 12.0); // c3 + c4
        assert_eq!(d[QE_DIFF_X], 1.0); // c2 - c1
        assert_eq!(d[QE_DIFF_Y], 4.0); // c4 - c3
        assert_eq!(d[QE_POSITION_X], 1.0 / 3.0);
        assert_eq!(d[QE_POSITION_Y], 4.0 / 12.0);
    }

    #[test]
    fn square_geometry_matches_cpp() {
        let cfg = unit_cfg(QeGeometry::Square);
        let d = compute_positions(&[1.0, 2.0, 4.0, 8.0], &cfg);
        assert_eq!(d[QE_SUM_X], 15.0);
        assert_eq!(d[QE_SUM_Y], 15.0);
        assert_eq!(d[QE_DIFF_X], (2.0 + 4.0) - (1.0 + 8.0));
        assert_eq!(d[QE_DIFF_Y], (1.0 + 2.0) - (4.0 + 8.0));
    }

    #[test]
    fn square_cc_geometry_matches_cpp() {
        let cfg = unit_cfg(QeGeometry::SquareCc);
        let d = compute_positions(&[1.0, 2.0, 4.0, 8.0], &cfg);
        assert_eq!(d[QE_DIFF_X], (4.0 + 8.0) - (1.0 + 2.0));
        assert_eq!(d[QE_DIFF_Y], (1.0 + 8.0) - (2.0 + 4.0));
    }

    #[test]
    fn custom_geometry_applies_weights() {
        let cfg = PositionConfig {
            geometry: QeGeometry::Custom as i32,
            current_scale: [1.0; 4],
            position_scale: [1.0; 2],
            weight_xsum: [1.0, 1.0, 0.0, 0.0],
            weight_ysum: [0.0, 0.0, 1.0, 1.0],
            weight_xdelta: [-1.0, 1.0, 0.0, 0.0],
            weight_ydelta: [0.0, 0.0, -1.0, 1.0],
            ..Default::default()
        };
        let d = compute_positions(&[1.0, 2.0, 4.0, 8.0], &cfg);
        assert_eq!(d[QE_SUM_X], 3.0);
        assert_eq!(d[QE_SUM_Y], 12.0);
        assert_eq!(d[QE_DIFF_X], 1.0);
        assert_eq!(d[QE_DIFF_Y], 4.0);
    }

    #[test]
    fn zero_sum_denominator_becomes_one() {
        let cfg = unit_cfg(QeGeometry::Diamond);
        // c1 = -c2 and c3 = -c4 make both sums zero.
        let d = compute_positions(&[1.0, -1.0, 2.0, -2.0], &cfg);
        assert_eq!(d[QE_SUM_X], 0.0);
        assert_eq!(d[QE_SUM_Y], 0.0);
        assert_eq!(d[QE_POSITION_X], -2.0); // diffX / 1.0
        assert_eq!(d[QE_POSITION_Y], -4.0); // diffY / 1.0
    }

    #[test]
    fn offsets_and_scales_apply_before_geometry() {
        let cfg = PositionConfig {
            geometry: QeGeometry::Diamond as i32,
            current_scale: [2.0, 2.0, 2.0, 2.0],
            current_offset: [1.0, 1.0, 1.0, 1.0],
            position_scale: [10.0, 10.0],
            position_offset: [0.5, 0.5],
            ..Default::default()
        };
        let d = compute_positions(&[1.0, 2.0, 3.0, 4.0], &cfg);
        // raw*2 - 1 => 1, 3, 5, 7
        assert_eq!(d[QE_CURRENT1], 1.0);
        assert_eq!(d[QE_CURRENT4], 7.0);
        assert_eq!(d[QE_SUM_X], 4.0);
        assert_eq!(d[QE_DIFF_X], 2.0);
        assert_eq!(d[QE_POSITION_X], 10.0 * 2.0 / 4.0 - 0.5);
    }

    #[test]
    fn num_average_rounds_like_c_cast() {
        assert_eq!(num_average_from(1.0, 0.001), 1000);
        // 0.0015 / 0.001 = 1.5 → +0.5 = 2.0 → 2
        assert_eq!(num_average_from(0.0015, 0.001), 2);
        // 0.0014 / 0.001 = 1.4 → +0.5 = 1.9 → truncates to 1
        assert_eq!(num_average_from(0.0014, 0.001), 1);
        assert_eq!(num_average_from(0.0, 0.001), 0);
        // sampleTime 0 gives inf in C++ too; we clamp to 0 instead of UB.
        assert_eq!(num_average_from(1.0, 0.0), 0);
    }

    #[test]
    fn ring_overflow_drops_oldest_and_counts() {
        let mut ring = RingState::new(2);
        assert!(!ring.push([1.0; QE_MAX_DATA]));
        assert!(!ring.push([2.0; QE_MAX_DATA]));
        assert!(ring.push([3.0; QE_MAX_DATA]));
        assert_eq!(ring.ring_overflows, 1);
        assert_eq!(ring.len(), 2);
        assert_eq!(ring.ring_count, 2);
        // raw_count: +1 +1 (+1 -1) = 2
        assert_eq!(ring.raw_count, 2);
        let drained = ring.drain(2).expect("two samples buffered");
        assert_eq!(drained[0][0], 2.0);
        assert_eq!(drained[1][0], 3.0);
        assert_eq!(ring.ring_count, 0);
    }

    #[test]
    fn ring_drain_more_than_buffered_fails() {
        let mut ring = RingState::new(4);
        ring.push([1.0; QE_MAX_DATA]);
        assert!(ring.drain(2).is_none());
        assert_eq!(ring.len(), 1);
    }

    #[test]
    fn take_raw_count_zeroes_and_refuses_empty() {
        let mut ring = RingState::new(4);
        assert_eq!(ring.take_raw_count(), None);
        ring.push([1.0; QE_MAX_DATA]);
        ring.push([2.0; QE_MAX_DATA]);
        assert_eq!(ring.take_raw_count(), Some(2));
        assert_eq!(ring.raw_count, 0);
        assert_eq!(ring.take_raw_count(), None);
        // Draining still works: ring_count is untouched by take_raw_count.
        assert_eq!(ring.ring_count, 2);
    }

    #[test]
    fn flush_clears_ring_and_counters() {
        let mut ring = RingState::new(4);
        ring.push([1.0; QE_MAX_DATA]);
        ring.ring_overflows = 3;
        ring.flush();
        assert!(ring.is_empty());
        assert_eq!(ring.ring_count, 0);
        assert_eq!(ring.raw_count, 0);
        // C++ epicsRingBytesFlush does not reset P_RingOverflows.
        assert_eq!(ring.ring_overflows, 3);
    }

    /// A meter whose setters log their call and fail when told to.
    struct FakeMeter {
        base: QuadEmBase,
        shared: Arc<QuadEmShared>,
        calls: Vec<&'static str>,
        failing: &'static [(&'static str, AsynStatus)],
        bias_interlocks: Vec<i32>,
    }

    impl FakeMeter {
        fn new(failing: &'static [(&'static str, AsynStatus)]) -> Self {
            Self {
                base: QuadEmBase::new("QE_FAKE_DISPATCH", 0).unwrap(),
                shared: QuadEmShared::new(16).0,
                calls: Vec::new(),
                failing,
                bias_interlocks: Vec::new(),
            }
        }

        fn step(&mut self, name: &'static str) -> AsynResult<()> {
            self.calls.push(name);
            if let Some(&(_, status)) = self.failing.iter().find(|(n, _)| *n == name) {
                return Err(AsynError::Status {
                    status,
                    message: name.into(),
                });
            }
            Ok(())
        }
    }

    impl QuadEmDevice for FakeMeter {
        fn qe_base(&mut self) -> &mut QuadEmBase {
            &mut self.base
        }
        fn qe_shared(&self) -> &Arc<QuadEmShared> {
            &self.shared
        }
        fn set_acquire(&mut self, _value: i32) -> AsynResult<()> {
            self.step("set_acquire")
        }
        fn set_acquire_mode(&mut self, _value: i32) -> AsynResult<()> {
            self.step("set_acquire_mode")
        }
        fn set_range(&mut self, _value: i32) -> AsynResult<()> {
            self.step("set_range")
        }
        fn set_channel_range(&mut self, _channel: i32, _value: i32) -> AsynResult<()> {
            self.step("set_channel_range")
        }
        fn set_bias_voltage(&mut self, _value: f64) -> AsynResult<()> {
            self.step("set_bias_voltage")
        }
        fn set_bias_interlock(&mut self, value: i32) -> AsynResult<()> {
            self.bias_interlocks.push(value);
            Ok(())
        }
        fn read_status(&mut self) -> AsynResult<()> {
            self.step("read_status")
        }
        fn reset(&mut self) -> AsynResult<()> {
            self.step("reset")
        }
    }

    /// The status C's write returns, in its numbering, and its errorMessage.
    fn failed_with(r: AsynResult<()>) -> (i32, String) {
        let e = r.unwrap_err();
        (e.c_status_code(), e.message())
    }

    #[test]
    fn a_failed_setter_still_reads_status_and_fails_the_write() {
        let mut m = FakeMeter::new(&[("set_range", AsynStatus::Disconnected)]);
        let range = m.base.params.range;
        assert_eq!(
            failed_with(m.qe_write_int32(&AsynUser::new(range).with_addr(0), 3)),
            (
                4,
                format!("drvQuadEM:writeInt32: status=4, function={range}, name=QE_RANGE, value=3")
            )
        );
        assert_eq!(m.calls, ["set_range", "read_status"]);
        assert_eq!(m.base.port_base.get_int32_param(range, 0).unwrap(), 3);
    }

    #[test]
    fn a_failed_status_read_fails_an_accepted_setting() {
        let mut m = FakeMeter::new(&[("read_status", AsynStatus::Timeout)]);
        let bias_voltage = m.base.params.bias_voltage;
        assert_eq!(
            failed_with(m.qe_write_float64(&AsynUser::new(bias_voltage).with_addr(0), 50.0)),
            (
                1,
                format!(
                    "drvQuadEM:writeFloat64: status=1, function={bias_voltage}, \
                     name=QE_BIAS_VOLTAGE, value=50.000000"
                )
            )
        );
        assert_eq!(m.calls, ["set_bias_voltage", "read_status"]);
    }

    /// C ORs the codes: Timeout(1) | Overflow(2) is asynError(3).
    #[test]
    fn the_step_statuses_are_ored() {
        let mut m = FakeMeter::new(&[
            ("set_acquire", AsynStatus::Timeout),
            ("set_acquire_mode", AsynStatus::Overflow),
        ]);
        let (acquire, acquire_mode) = (m.base.nd_params.acquire, m.base.params.acquire_mode);
        m.base.port_base.set_int32_param(acquire, 0, 1).unwrap();
        let single = QeAcquireMode::Single as i32;
        assert_eq!(
            failed_with(m.qe_write_int32(&AsynUser::new(acquire_mode).with_addr(0), single)),
            (
                3,
                format!(
                    "drvQuadEM:writeInt32: status=3, function={acquire_mode}, \
                     name=QE_ACQUIRE_MODE, value={single}"
                )
            )
        );
        assert_eq!(m.calls, ["set_acquire", "set_acquire_mode", "read_status"]);
        assert_eq!(m.base.port_base.get_int32_param(acquire, 0).unwrap(), 0);
        assert_eq!(m.shared.acq.lock().acquire_mode, single);
    }

    /// drvQuadEM never fills pArrays[0], so asynNDArrayDriver::preAllocateBuffers
    /// prints its error line (asynNDArrayDriver.cpp:716-721).
    #[test]
    fn a_pre_alloc_prints_cs_no_template_line() {
        use epics_rs::asyn::trace::TraceFile;
        let mut m = FakeMeter::new(&[]);
        let path = std::env::temp_dir().join(format!("qe-prealloc-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        epics_rs::asyn::services::PortServices::global()
            .trace()
            .set_trace_file(
                Some("QE_FAKE_DISPATCH"),
                TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
            );
        let pre_alloc = m.base.nd_params.pool_pre_alloc;
        m.qe_write_int32(&AsynUser::new(pre_alloc).with_addr(0), 1)
            .unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(
            text.contains(
                "asynNDArrayDriver::preAllocateBuffers: ERROR, must collect an array to get dimensions first"
            ),
            "{text:?}"
        );
        assert_eq!(m.base.port_base.get_int32_param(pre_alloc, 0).unwrap(), 0);
    }

    /// drvQuadEM.cpp:512-513 runs asynNDArrayDriver::writeInt32, and both
    /// writes close with drvQuadEM's own line (:524-526, :579-581).
    #[test]
    fn writes_print_the_c_io_driver_lines() {
        use epics_rs::asyn::trace::TraceFile;
        use epics_rs::asyn::user::UserTrace;
        let port = "QE_FAKE_DISPATCH";
        let mut m = FakeMeter::new(&[]);
        let manager = epics_rs::asyn::services::PortServices::global()
            .trace()
            .clone();
        let path = std::env::temp_dir().join(format!("qe-trace-{}", std::process::id()));
        let file = std::fs::File::create(&path).unwrap();
        manager.set_trace_file(
            Some(port),
            TraceFile::File(Arc::new(std::sync::Mutex::new(file))),
        );
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::IO_DRIVER);
        let traced = |reason| {
            let mut user = AsynUser::new(reason);
            user.trace = Some(UserTrace {
                manager: manager.clone(),
                port: port.into(),
            });
            user
        };
        let poll = m.base.nd_params.pool_poll_stats;
        let bias = m.base.params.bias_voltage;
        m.qe_write_int32(&traced(poll), 1).unwrap();
        m.qe_write_float64(&traced(bias), 50.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        for line in [
            format!(
                "asynNDArrayDriver:writeInt32: function={poll}, paramName=POOL_POLL_STATS, value=1"
            ),
            format!("drvQuadEM:writeInt32: function={poll}, name=POOL_POLL_STATS, value=1"),
            format!(
                "drvQuadEM:writeFloat64: function={bias}, name=QE_BIAS_VOLTAGE, value=50.000000"
            ),
        ] {
            assert!(text.contains(&line), "{line}: {text:?}");
        }
    }

    #[test]
    fn a_reset_sends_zero_for_an_unset_setting_not_the_previous_read() {
        let mut m = FakeMeter::new(&[]);
        // QE_BIAS_INTERLOCK has no constructor default and is read right
        // after QE_BIAS_STATE.
        let bias_state = m.base.params.bias_state;
        m.base.port_base.set_int32_param(bias_state, 0, 1).unwrap();
        m.base_reset().unwrap();
        assert_eq!(m.bias_interlocks, [0]);
    }

    /// Overflow(2) | Disconnected(4) is 6, which no asynStatus names: the
    /// message and the returned status both carry C's 6.
    #[test]
    fn an_or_past_asyn_disabled_keeps_its_code() {
        let mut m = FakeMeter::new(&[
            ("set_range", AsynStatus::Overflow),
            ("read_status", AsynStatus::Disconnected),
        ]);
        let range = m.base.params.range;
        let (status, message) =
            failed_with(m.qe_write_int32(&AsynUser::new(range).with_addr(0), 1));
        assert_eq!(status, 6);
        assert!(message.contains("status=6,"), "{message}");
    }

    /// drvQuadEM.cpp:424 ORs setIntegerParam's asynParamInvalidList (11) for
    /// an address past the port's lists.
    #[test]
    fn a_refused_param_set_carries_cs_param_status() {
        let mut m = FakeMeter::new(&[]);
        let geometry = m.base.params.geometry;
        let list = QE_MAX_DATA as i32 + 1;
        assert_eq!(
            failed_with(m.qe_write_int32(&AsynUser::new(geometry).with_addr(list), 1)),
            (
                11,
                format!(
                    "drvQuadEM:writeInt32: status=11, function={geometry}, \
                     name=QE_GEOMETRY, value=1"
                )
            )
        );
    }

    fn acquire_busy(m: &FakeMeter) -> i32 {
        let busy = m.base.nd_params.acquire_busy;
        m.base.port_base.get_int32_param(busy, 0).unwrap()
    }

    /// `asynNDArrayDriver::setIntegerParam`: `ADAcquireBusy` follows `ADAcquire`.
    #[test]
    fn acquire_busy_follows_acquire_without_wait_for_plugins() {
        let mut m = FakeMeter::new(&[]);
        let acquire = m.base.nd_params.acquire;
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 1)
            .unwrap();
        assert_eq!(acquire_busy(&m), 1);
        m.base.owner.queued().increment();
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 0)
            .unwrap();
        assert_eq!(acquire_busy(&m), 0);
    }

    #[test]
    fn a_stop_with_wait_for_plugins_stays_busy_while_arrays_are_queued() {
        let mut m = FakeMeter::new(&[]);
        let (acquire, wait) = (m.base.nd_params.acquire, m.base.nd_params.wait_for_plugins);
        m.base.port_base.set_int32_param(wait, 0, 1).unwrap();
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 1)
            .unwrap();
        m.base.owner.queued().increment();
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 0)
            .unwrap();
        assert_eq!(acquire_busy(&m), 1);

        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 1)
            .unwrap();
        m.base.owner.queued().decrement();
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 0)
            .unwrap();
        assert_eq!(acquire_busy(&m), 0);
    }

    #[test]
    fn leaving_continuous_mode_clears_acquire_busy() {
        let mut m = FakeMeter::new(&[]);
        let (acquire, acquire_mode) = (m.base.nd_params.acquire, m.base.params.acquire_mode);
        m.qe_write_int32(&AsynUser::new(acquire).with_addr(0), 1)
            .unwrap();
        m.qe_write_int32(
            &AsynUser::new(acquire_mode).with_addr(0),
            QeAcquireMode::Single as i32,
        )
        .unwrap();
        assert_eq!(acquire_busy(&m), 0);
    }

    #[test]
    fn a_channel_range_goes_to_the_per_channel_setter() {
        let mut m = FakeMeter::new(&[]);
        let range = m.base.params.range;
        m.qe_write_int32(&AsynUser::new(range).with_addr(0), 1)
            .unwrap();
        m.qe_write_int32(&AsynUser::new(range).with_addr(2), 1)
            .unwrap();
        assert_eq!(
            m.calls,
            [
                "set_range",
                "read_status",
                "set_channel_range",
                "read_status"
            ]
        );
    }
}
