//! Port of `quadEMApp/quadEMSrc/drvQuadEM.{h,cpp}` — the shared electrometer
//! base.
//!
//! The C++ class derives from `asynNDArrayDriver` with `maxAddr =
//! QE_MAX_DATA+1` (12) and `ASYN_MULTIDEVICE`: addresses 0-10 carry one data
//! item each (4 currents, 2 sums, sum-all, 2 diffs, 2 positions) and address
//! 11 carries the full 2-D array. This module reproduces that layout on top of
//! `asyn_rs::PortDriverBase` plus `ad_core_rs`'s `NDArrayDriverParams` and
//! `NDArrayPool`, because `NDArrayDriverBase::new` hard-codes `maxAddr = 1`.
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

use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;
use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::{PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

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

impl QuadEmShared {
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
    /// pending (`rawCount_ < 1`, e.g. a Read press while idle).
    pub fn trigger_callbacks(&self) -> bool {
        let Some(n) = self.ring.lock().take_raw_count() else {
            return false;
        };
        if self.trigger_tx.try_send(n).is_err() {
            log::error!("quadEM: trigger message queue full, dropping {n} samples");
            return false;
        }
        true
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
            log::warn!("quadEM: ring buffer overflow");
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
            self.trigger_callbacks();
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
    fn read_status(&mut self) -> AsynResult<()> {
        Ok(())
    }

    /// `drvQuadEM::setAcquire`: starting an acquisition clears `NumAcquired`.
    fn base_set_acquire(&mut self, value: i32) -> AsynResult<()> {
        if value == 1 {
            self.qe_shared().acq.lock().num_acquired = 0;
            let idx = self.qe_base().params.num_acquired;
            self.qe_base().port_base.set_int32_param(idx, 0, 0)?;
            self.qe_base().port_base.call_param_callbacks(0)?;
        }
        Ok(())
    }

    /// `drvQuadEM::reset`: push every cached EPICS setting back to the meter,
    /// re-read the status, then restore the acquire state. C++ discards each
    /// setter's status, so the port keeps resetting even when the meter
    /// rejects one command.
    fn base_reset(&mut self) -> AsynResult<()> {
        let p = self.qe_base().params;
        let acquire_param = self.qe_base().nd_params.acquire;
        let base = &mut self.qe_base().port_base;

        let range = base.get_int32_param(p.range, 0)?;
        let values_per_read = base.get_int32_param(p.values_per_read, 0)?;
        let averaging_time = base.get_float64_param(p.averaging_time, 0)?;
        let trigger_mode = base.get_int32_param(p.trigger_mode, 0)?;
        let num_channels = base.get_int32_param(p.num_channels, 0)?;
        let bias_state = base.get_int32_param(p.bias_state, 0)?;
        let bias_interlock = base.get_int32_param(p.bias_interlock, 0)?;
        let bias_voltage = base.get_float64_param(p.bias_voltage, 0)?;
        let resolution = base.get_int32_param(p.resolution, 0)?;
        let read_format = base.get_int32_param(p.read_format, 0)?;
        let integration_time = base.get_float64_param(p.integration_time, 0)?;
        let acquire = base.get_int32_param(acquire_param, 0)?;

        let _ = self.set_range(range);
        let _ = self.set_values_per_read(values_per_read);
        let _ = self.set_averaging_time(averaging_time);
        let _ = self.set_trigger_mode(trigger_mode);
        let _ = self.set_num_channels(num_channels);
        let _ = self.set_bias_state(bias_state);
        let _ = self.set_bias_interlock(bias_interlock);
        let _ = self.set_bias_voltage(bias_voltage);
        let _ = self.set_resolution(resolution);
        let _ = self.set_read_format(read_format);
        let _ = self.set_integration_time(integration_time);
        let _ = self.read_status();
        self.set_acquire(acquire)
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
    pub pool: Arc<epics_rs::ad_core::ndarray_pool::NDArrayPool>,
    /// One output per asyn address: 0-10 single-value streams, 11 the full
    /// 2-D array.
    pub outputs: Vec<Arc<parking_lot::Mutex<NDArrayOutput>>>,
    pub queued: Vec<Arc<QueuedArrayCounter>>,
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

        let nd_params = NDArrayDriverParams::create(&mut port_base)?;
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

        let pool = Arc::new(epics_rs::ad_core::ndarray_pool::NDArrayPool::new(
            max_memory,
        ));

        let outputs = (0..=QE_MAX_DATA)
            .map(|_| Arc::new(parking_lot::Mutex::new(NDArrayOutput::new())))
            .collect();
        let queued = (0..=QE_MAX_DATA)
            .map(|_| Arc::new(QueuedArrayCounter::new()))
            .collect();

        Ok(Self {
            port_base,
            nd_params,
            params,
            pool,
            outputs,
            queued,
        })
    }

    /// Pool branch of `asynNDArrayDriver::writeInt32`, which `drvQuadEM`
    /// reaches through its `function < FIRST_QE_COMMAND` fallthrough.
    ///
    /// `NDPoolPreAllocBuffers` needs a template array (C++ `pArrays[0]`);
    /// `drvQuadEM` never stores one, so pre-allocation only resets the request
    /// parameter, exactly as it does upstream.
    ///
    /// Returns `true` when `param_index` named a pool control.
    pub fn write_int32_pool(&mut self, param_index: usize) -> AsynResult<bool> {
        let p = &self.nd_params;
        if param_index == p.pool_empty_free_list {
            self.pool.empty_free_list();
            self.refresh_pool_stats()?;
            Ok(true)
        } else if param_index == p.pool_poll_stats {
            self.refresh_pool_stats()?;
            Ok(true)
        } else if param_index == p.pool_pre_alloc {
            self.port_base.set_int32_param(p.pool_pre_alloc, 0, 0)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn refresh_pool_stats(&mut self) -> AsynResult<()> {
        const MEGABYTE: f64 = 1_048_576.0;
        let p = self.nd_params;
        self.port_base.set_float64_param(
            p.pool_max_memory,
            0,
            self.pool.max_memory() as f64 / MEGABYTE,
        )?;
        self.port_base.set_float64_param(
            p.pool_used_memory,
            0,
            self.pool.allocated_bytes() as f64 / MEGABYTE,
        )?;
        self.port_base.set_int32_param(
            p.pool_alloc_buffers,
            0,
            self.pool.num_alloc_buffers() as i32,
        )?;
        self.port_base.set_int32_param(
            p.pool_free_buffers,
            0,
            self.pool.num_free_buffers() as i32,
        )?;
        Ok(())
    }

    /// Attach a downstream plugin to one asyn address' NDArray stream.
    pub fn connect_downstream(&self, addr: usize, mut sender: NDArraySender) {
        let Some(out) = self.outputs.get(addr) else {
            log::error!("quadEM: connect_downstream: address {addr} out of range");
            return;
        };
        sender.set_queued_counter(self.queued[addr].clone());
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
    pub shared: Arc<QuadEmShared>,
    /// Index of `ADAcquire` on this port, written to stop a finished
    /// multiple/single acquisition.
    pub acquire_param: usize,
}

/// C++ `drvQuadEM::doDataCallbacks`.
async fn do_data_callbacks(ctx: &CallbackContext, num_read: usize) -> bool {
    let Some(samples) = ctx.shared.ring.lock().drain(num_read) else {
        log::error!("quadEM: not enough samples in ring buffer, expected {num_read}");
        return false;
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
    let mut all = NDArray::with_data(
        vec![NDDimension::new(QE_MAX_DATA), NDDimension::new(num_read)],
        NDDataBuffer::F64(flat),
    );
    all.unique_id = counter;
    all.timestamp = ts;
    all.time_stamp = time_stamp;
    ArrayPublisher::new(ctx.outputs[QE_ADDR_ALL].clone())
        .publish(Arc::new(all))
        .await;

    // Addresses 0-10: one [numRead] array per data item.
    for item in 0..QE_MAX_DATA {
        let column: Vec<f64> = samples.iter().map(|s| s[item]).collect();
        let mut single =
            NDArray::with_data(vec![NDDimension::new(num_read)], NDDataBuffer::F64(column));
        single.unique_id = counter;
        single.timestamp = ts;
        single.time_stamp = time_stamp;
        ArrayPublisher::new(ctx.outputs[item].clone())
            .publish(Arc::new(single))
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
}
