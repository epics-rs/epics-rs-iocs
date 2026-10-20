use std::time::SystemTime;

use epics_libcom_rs::cfmt::CFloat;
use meascomp::counter::CounterScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::general_time::EPICS_EPOCH_UNIX_SECS;

use crate::params::*;
use crate::scaler::print_get_status;
use crate::trace::SelfTrace;

/// MCS (Multi-Channel Scaler) acquisition state.
pub struct McsState {
    /// C `MCSRunning_`.
    pub running: bool,
    pub num_counters_enabled: usize,
    pub max_points: usize,
    pub current_point: usize,
    pub start_time: f64,
    pub preset_real_time: f64,

    /// Scan buffer for ulDaqInScan (f64 values).
    pub scan_buffer: Vec<f64>,
    /// Per-counter MCS data `[counter][point]`.
    pub mcs_buffers: Vec<Vec<i32>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
    /// Time waveform per point.
    pub time_buffer: Vec<f32>,
    /// Which counters are enabled (bitmask).
    pub counter_enable: u32,
    /// Mapping from scan channel index to counter number.
    pub chan_map: Vec<usize>,
    /// C `MCSPoint0Skip`: the scan's first point is not stored.
    pub point0_skip: bool,
    /// C `MCSErased_`, which only `report` reads.
    pub erased: bool,
}

impl McsState {
    pub fn new(max_points: usize) -> Self {
        let mut mcs_buffers = Vec::with_capacity(MAX_MCS_COUNTERS);
        for _ in 0..MAX_MCS_COUNTERS {
            mcs_buffers.push(vec![0i32; max_points]);
        }
        Self {
            running: false,
            num_counters_enabled: 0,
            max_points,
            current_point: 0,
            start_time: 0.0,
            preset_real_time: 0.0,
            scan_buffer: Vec::new(),
            mcs_buffers,
            abs_time_buffer: vec![0.0; max_points],
            time_buffer: vec![0.0; max_points],
            counter_enable: 0x1FF, // all 9 enabled by default
            chan_map: Vec::new(),
            point0_skip: false,
            erased: false,
        }
    }
}

impl McsState {
    /// Seconds since `start_mcs` armed the scan.
    pub fn elapsed_secs(&self) -> f64 {
        (current_time_secs() - self.start_time).max(0.0)
    }
}

/// Clear the first `num_points` of every spectrum and restart the elapsed
/// clock, which an acquisition in progress keeps counting from. C
/// `USBCTR::eraseMCS`; the time bases are left alone.
pub fn erase_mcs(state: &mut McsState, num_points: usize) {
    state.erased = true;
    for buf in &mut state.mcs_buffers {
        let n = num_points.min(buf.len());
        buf[..n].fill(0);
    }
    state.current_point = 0;
    state.start_time = current_time_secs();
}

/// Fill the first `num_points` of the time base with `i * dwell` and return
/// them. C `USBCTR::computeMCSTimes`.
pub fn compute_times(state: &mut McsState, num_points: usize, dwell: f64) -> &[f32] {
    let n = num_points.min(state.time_buffer.len());
    for (i, t) in state.time_buffer[..n].iter_mut().enumerate() {
        *t = (i as f64 * dwell) as f32;
    }
    &state.time_buffer[..n]
}

/// C `MCSPoint0Action_t`.
pub const MCS_POINT0_NO_CLEAR: i32 = 1;
pub const MCS_POINT0_SKIP: i32 = 2;

/// C `SINGLEIO_THRESHOLD_TIME`: dwell times from here up read one sample at
/// a time.
const SINGLEIO_THRESHOLD_TIME: f64 = 0.01;

/// Acquisition settings for [`start_mcs`], read from the MCA/MCS records.
#[derive(Clone, Copy, Debug)]
pub struct McsScan {
    /// C `numCounters_`.
    pub num_counters: usize,
    pub num_points: usize,
    pub dwell_time: f64,
    pub counter_enable: u32,
    pub ch_advance_source: i32,
    /// C `mcaPrescale_`: with external channel advance, divide the advance
    /// pulses by this on `prescale_counter`.
    pub prescale: i32,
    /// C `MCSPrescaleCounter_`.
    pub prescale_counter: i32,
    /// C `MCSPoint0Action_`.
    pub point0_action: i32,
}

/// Start MCS acquisition using DaqInScan and return the dwell time the
/// hardware clock gave. C `USBCTR::startMCS` (drvUSBCTR.cpp:520), which
/// marks the scan running even when ulDaqInScan fails: readMCS then finds it
/// idle and stops it.
pub fn start_mcs(
    device: &DaqDevice,
    state: &mut McsState,
    scan: &McsScan,
    trace: &SelfTrace,
) -> f64 {
    let McsScan {
        num_counters,
        num_points,
        dwell_time,
        counter_enable,
        ch_advance_source,
        prescale,
        prescale_counter,
        point0_action,
    } = *scan;
    state.counter_enable = counter_enable;

    let mut chan_descs = Vec::new();
    let mut chan_map = Vec::new();
    for i in 0..num_counters.min(MAX_COUNTERS) {
        if counter_enable & (1 << i) == 0 {
            continue;
        }
        let mode = CMM_OUTPUT_ON
            | CMM_CLEAR_ON_READ
            | CMM_GATING_ON
            | CMM_INVERT_GATE
            | CMM_OUTPUT_INITIAL_STATE_HIGH;
        if let Err(e) = device.counter_config_scan(
            i as i32,
            &CounterScanConfig {
                measurement_type: CMT_COUNT,
                measurement_mode: mode,
                edge_detection: CED_RISING_EDGE,
                tick_size: CTS_TICK_20PT83ns,
                debounce_mode: CDM_NONE,
                debounce_time: CDT_DEBOUNCE_0ns,
                flags: CF_DEFAULT,
            },
        ) {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "USBCTR::startMCS error calling cbCConfigScan, counter={i}, mode=0x{mode:x}, status={}, error={}\n",
                    e.code, e.message()
                ),
            );
        }
        chan_descs.push(DaqInChanDescriptor {
            channel: i as i32,
            chan_type: DAQI_CTR32,
            range: BIP10VOLTS,
            ..DaqInChanDescriptor::default()
        });
        chan_map.push(i);
    }
    if counter_enable & (1 << DIGITAL_IO_COUNTER) != 0 {
        chan_descs.push(DaqInChanDescriptor {
            channel: AUXPORT,
            chan_type: DAQI_DIGITAL,
            range: BIP10VOLTS,
            ..DaqInChanDescriptor::default()
        });
        chan_map.push(DIGITAL_IO_COUNTER);
    }

    // Clear counter 0 output registers to prevent scaler presets from
    // interfering with MCS acquisition. C checks only the second load.
    let _ = device.counter_load(0, CRT_OUTPUT_VAL0, 0);
    if let Err(e) = device.counter_load(0, CRT_OUTPUT_VAL1, 0xFFFFFFFF) {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::startMCS error calling cbCLoad32, reg=OUTPUTVALREG, value=0, status={}, error={}\n",
                e.code, e.message()
            ),
        );
    }

    if ch_advance_source != 0 && prescale > 1 {
        // C keeps only the configScan status.
        let limit = (prescale - 1) as u64;
        let _ = device.counter_clear(prescale_counter);
        let _ = device.counter_load(prescale_counter, CRT_OUTPUT_VAL0, 0);
        let _ = device.counter_load(prescale_counter, CRT_OUTPUT_VAL1, limit);
        let _ = device.counter_load(prescale_counter, CRT_MAX_LIMIT, limit);
        let mode = CMM_OUTPUT_ON | CMM_RANGE_LIMIT_ON;
        if let Err(e) = device.counter_config_scan(
            prescale_counter,
            &CounterScanConfig {
                measurement_type: CMT_COUNT,
                measurement_mode: mode,
                edge_detection: CED_RISING_EDGE,
                tick_size: CTS_TICK_20PT83ns,
                debounce_mode: CDM_NONE,
                debounce_time: CDT_DEBOUNCE_0ns,
                flags: CF_DEFAULT,
            },
        ) {
            trace.print(
                TraceMask::ERROR,
                format_args!(
                    "USBCTR::startMCS error calling cbCConfigScan, counter={prescale_counter}, mode=0x{mode:x}, status={}, error={}\n",
                    e.code, e.message()
                ),
            );
        }
    }

    // A skipped point 0 is acquired, then dropped by read_mcs.
    let num_points = num_points + usize::from(point0_action == MCS_POINT0_SKIP);
    let mut rate = 1.0 / dwell_time;
    let mut options = SO_DEFAULTIO;
    if ch_advance_source != 0 {
        options |= SO_EXTCLOCK;
    }
    if dwell_time >= SINGLEIO_THRESHOLD_TIME {
        options |= SO_SINGLEIO;
    }
    options |= SO_EXTTRIGGER;
    let mut flags = DAQINSCAN_FF_DEFAULT;
    if point0_action == MCS_POINT0_NO_CLEAR {
        flags |= DAQINSCAN_FF_NOCLEAR;
    }

    state.num_counters_enabled = chan_descs.len();
    state.chan_map = chan_map;
    state.point0_skip = point0_action == MCS_POINT0_SKIP;
    state
        .scan_buffer
        .resize(state.num_counters_enabled * num_points, 0.0);

    if let Err(e) = device.daq_in_scan(
        &chan_descs,
        num_points as i32,
        &mut rate,
        options,
        flags,
        &mut state.scan_buffer,
    ) {
        // C passes `options, status, flags` to `options=, flags=, status=`.
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::startMCS error calling ulDaqInScan, numChans={}, numPoints={num_points}, rate={rate:.6}, options=0x{options:x}, flags=0x{:x}, status={flags}, error={}\n",
                chan_descs.len(),
                e.code,
                e.message(), rate = CFloat(rate)
            ),
        );
    }

    state.running = true;
    state.current_point = 0;
    state.start_time = current_time_secs();
    1.0 / rate
}

/// Number of complete scan points behind `current_index`; see
/// `usb_2408::wave_dig::points_transferred` for the C original. Kept here
/// rather than shared so the two drivers stay independent crates.
pub fn points_transferred(current_index: i64, n_chans: usize, max_points: usize) -> usize {
    if current_index < 0 || n_chans == 0 {
        return 0;
    }
    (current_index as usize / n_chans + 1).min(max_points)
}

/// What one [`read_mcs`] pass leaves for the driver to publish.
#[derive(Clone, Copy, Debug)]
pub struct McsPass {
    /// C `MCSCurrentPoint_`.
    pub current_point: usize,
    /// C `mcaElapsedRealTime_`/`LiveTime_`, set on every counter address.
    pub elapsed: f64,
    /// The scan ended this pass: C sets `mcaAcquiring_` 0 on each counter.
    pub stopped: bool,
}

impl McsPass {
    /// The param sets C readMCS makes for this pass, for whichever side
    /// (poller or port driver) applies them.
    pub fn updates(
        &self,
        params: &CtrParams,
        num_counters: usize,
    ) -> Vec<(usize, i32, ParamValue)> {
        let mut updates = vec![(
            params.mcs_current_point,
            0,
            ParamValue::Int32(self.current_point as i32),
        )];
        for addr in 0..MAX_MCS_COUNTERS as i32 {
            updates.push((
                params.mca_elapsed_real,
                addr,
                ParamValue::Float64(self.elapsed),
            ));
            updates.push((
                params.mca_elapsed_live,
                addr,
                ParamValue::Float64(self.elapsed),
            ));
        }
        if self.stopped {
            for addr in 0..num_counters as i32 {
                updates.push((params.mca_acquiring, addr, ParamValue::Int32(0)));
            }
        }
        updates
    }
}

/// Copy the points the scan has transferred and stop it once the hardware
/// is idle or the preset real time has passed. C `USBCTR::readMCS`.
pub fn read_mcs(device: &DaqDevice, state: &mut McsState, trace: &SelfTrace) -> McsPass {
    // C reads the state and index whatever the call returned
    // (drvUSBCTR.cpp:743-754): a scan that ended on a transfer error
    // reports SS_IDLE and its last index beside the error.
    let polled = device.daq_in_scan_status();
    print_get_status(trace, "readMCS", &polled);
    let current_index = polled.xfer.current_index;
    if polled.status == SS_IDLE {
        state.running = false;
    }

    let n_chans = state.num_counters_enabled;
    if current_index >= 0 && n_chans > 0 {
        let skip = usize::from(state.point0_skip);
        let last_point = points_transferred(current_index, n_chans, state.max_points + skip);
        let now = current_time_secs();
        // A skipped point 0 shifts the scan buffer by one.
        let mut in_ptr = state.current_point + skip;
        while in_ptr < last_point && state.current_point < state.max_points {
            let buf_offset = in_ptr * n_chans;
            for (scan_idx, &ctr_idx) in state.chan_map.iter().enumerate() {
                state.mcs_buffers[ctr_idx][state.current_point] =
                    state.scan_buffer[buf_offset + scan_idx] as i32;
            }
            state.abs_time_buffer[state.current_point] = now;
            state.current_point += 1;
            in_ptr += 1;
        }
    }

    let elapsed = state.elapsed_secs();
    if state.running && state.preset_real_time > 0.0 && elapsed >= state.preset_real_time {
        state.running = false;
    }
    let stopped = !state.running;
    if stopped {
        stop_mcs(device, state, trace);
    }
    McsPass {
        current_point: state.current_point,
        elapsed,
        stopped,
    }
}

/// C `USBCTR::stopMCS`: a running scan is marked stopped and given a final
/// [`read_mcs`] pass, which comes back here to stop the hardware.
pub fn stop_mcs(device: &DaqDevice, state: &mut McsState, trace: &SelfTrace) -> Option<McsPass> {
    if state.running {
        state.running = false;
        return Some(read_mcs(device, state, trace));
    }
    if let Err(e) = device.daq_in_scan_stop() {
        trace.print(
            TraceMask::ERROR,
            format_args!(
                "USBCTR::stopMCS ERROR calling cbStopBackground(CTRFUNCTION), status={}, error={}\n",
                e.code, e.message()
            ),
        );
    }
    None
}

/// Seconds past the EPICS epoch, as C stamps each absolute-time point with
/// `now.secPastEpoch + now.nsec/1.e9`.
fn current_time_secs() -> f64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
        - EPICS_EPOCH_UNIX_SECS as f64
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use super::*;

    /// libuldaq's ERR_OVERRUN (uldaq.h:152), a transfer error.
    const ERR_OVERRUN: UlError = 18;

    thread_local! {
        /// What the mocked `ulDaqInScanStatus` reports: code, scan state,
        /// total count and index.
        static DAQ_IN_STATUS: Cell<Option<(UlError, i32, u64, i64)>> = const { Cell::new(None) };
    }

    /// The test binary's definition wins over libuldaq's. Unset, it answers
    /// as libuldaq does for the null handle: ERR_BAD_DEV_HANDLE (2), nothing
    /// filled.
    #[unsafe(no_mangle)]
    extern "C" fn ulDaqInScanStatus(
        _handle: uldaq_sys::DaqDeviceHandle,
        status: *mut i32,
        xfer: *mut TransferStatus,
    ) -> UlError {
        let Some((code, scan, count, index)) = DAQ_IN_STATUS.get() else {
            return 2;
        };
        unsafe {
            *status = scan;
            (*xfer).current_total_count = count;
            (*xfer).current_index = index;
        }
        code
    }

    /// C reads a failed status's state and index and copies the last
    /// points (drvUSBCTR.cpp:743-754).
    #[test]
    fn a_scan_that_ended_on_a_transfer_error_keeps_its_last_points() {
        let mut state = McsState::new(4);
        state.running = true;
        state.num_counters_enabled = 2;
        state.chan_map = vec![0, 1];
        state.scan_buffer = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        DAQ_IN_STATUS.set(Some((ERR_OVERRUN, SS_IDLE, 6, 5)));

        let pass = read_mcs(&DaqDevice::unconnected(), &mut state, &SelfTrace::default());
        assert!(pass.stopped);
        assert_eq!(pass.current_point, 3);
        assert_eq!(state.mcs_buffers[0][..3], [1, 3, 5]);
        assert_eq!(state.mcs_buffers[1][..3], [2, 4, 6]);
    }

    #[test]
    fn absolute_time_counts_from_the_epics_epoch() {
        let unix = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let offset = unix - current_time_secs();
        assert!((offset - 631_152_000.0).abs() < 1.0, "offset {offset}");
    }

    #[test]
    fn a_completed_scan_counts_every_point() {
        // 9 MCS channels x 100 points: the last sample written is index 899.
        assert_eq!(points_transferred(899, 9, 2048), 100);
    }

    #[test]
    fn the_first_sample_of_a_point_already_counts_it() {
        assert_eq!(points_transferred(0, 9, 2048), 1);
        assert_eq!(points_transferred(8, 9, 2048), 1);
        assert_eq!(points_transferred(9, 9, 2048), 2);
    }

    #[test]
    fn no_transfer_yet_is_zero_points() {
        assert_eq!(points_transferred(-1, 9, 2048), 0);
        assert_eq!(points_transferred(10, 0, 2048), 0);
    }
}
