use std::time::SystemTime;

use epics_rs::base::runtime::general_time::EPICS_EPOCH_UNIX_SECS;

use meascomp::counter::CounterScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// MCS (Multi-Channel Scaler) acquisition state.
pub struct McsState {
    pub running: bool,
    pub num_counters_enabled: usize,
    pub max_points: usize,
    pub current_point: usize,
    pub start_time: f64,
    pub dwell_time: f64,

    /// Scan buffer for ulDaqInScan (f64 values).
    pub scan_buffer: Vec<f64>,
    /// Per-counter MCS data [counter][point].
    pub mcs_buffers: Vec<Vec<i32>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
    /// Time waveform per point.
    pub time_buffer: Vec<f32>,
    /// Which counters are enabled (bitmask).
    pub counter_enable: u32,
    /// Mapping from scan channel index to counter number.
    pub chan_map: Vec<usize>,
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
            dwell_time: 0.001,
            scan_buffer: Vec::new(),
            mcs_buffers,
            abs_time_buffer: vec![0.0; max_points],
            time_buffer: vec![0.0; max_points],
            counter_enable: 0x1FF, // all 9 enabled by default
            chan_map: Vec::new(),
        }
    }
}

impl McsState {
    /// Seconds since `start_mcs` armed the scan.
    pub fn elapsed_secs(&self) -> f64 {
        (current_time_secs() - self.start_time).max(0.0)
    }
}

/// Erase all MCS buffers.
///
/// C `eraseMCS`: the spectra go back to zero and the elapsed-time clock
/// restarts -- also mid-scan, so an erase during a run restarts PresetReal.
/// The time bases are left as they are; they depend on the dwell, not on
/// the data. The caller publishes MCS_CURRENT_POINT and the zeroed elapsed
/// times.
pub fn erase_mcs(state: &mut McsState) {
    for buf in &mut state.mcs_buffers {
        buf.iter_mut().for_each(|v| *v = 0);
    }
    state.current_point = 0;
    state.start_time = current_time_secs();
}

/// C `computeMCSTimes`: the relative time base `i * dwell` over the
/// `num_points` the scan is set to (bounded by the buffer). Returns how many
/// points it wrote, which is also how many the MCS_TIME_WF callback carries.
pub fn compute_times(state: &mut McsState, num_points: usize, dwell: f64) -> usize {
    let n = num_points.min(state.time_buffer.len());
    for (i, t) in state.time_buffer[..n].iter_mut().enumerate() {
        *t = (i as f64 * dwell) as f32;
    }
    n
}

/// Acquisition settings for [`start_mcs`], read from the MCA/MCS records.
#[derive(Clone, Copy, Debug)]
pub struct McsScan {
    pub num_points: usize,
    pub dwell_time: f64,
    pub counter_enable: u32,
    pub ch_advance_source: i32,
    /// Accepted but not implemented (matches C++ drvUSBCTR, which ignores
    /// prescale for MCS).
    pub prescale: i32,
    pub ext_trigger: bool,
    pub point0_no_clear: bool,
}

/// Dwell at and above which C reads the scan one sample at a time
/// (drvUSBCTR.cpp:101); below it libuldaq picks block transfers, which a
/// short-dwell, high-rate scan needs to keep up.
pub const SINGLEIO_THRESHOLD_TIME: f64 = 0.01;

/// The transfer and clock options C `startMCS` gives `ulDaqInScan`
/// (drvUSBCTR.cpp:674-679).
pub fn scan_options(dwell_time: f64, external_advance: bool) -> i32 {
    let mut options = SO_DEFAULTIO;
    if external_advance {
        options |= SO_EXTCLOCK;
    }
    if dwell_time >= SINGLEIO_THRESHOLD_TIME {
        options |= SO_SINGLEIO;
    }
    options
}

/// Start MCS acquisition using DaqInScan.
///
/// C `startMCS`: every uldaq failure is logged and the start carries on, and
/// the scan is marked running whatever `ulDaqInScan` returned. A scan that
/// did not start is idle from the first `read_mcs`, which ends it like any
/// finished one -- so MCA_ACQUIRING always comes back to 0. Returns the last
/// failure, for LAST_ERROR_MESSAGE.
pub fn start_mcs(
    device: &DaqDevice,
    state: &mut McsState,
    scan: &McsScan,
    num_counters: usize,
) -> Option<String> {
    let mut failure: Option<String> = None;
    let mut fail = |msg: String| {
        log::error!("{msg}");
        failure = Some(msg);
    };
    let McsScan {
        num_points,
        dwell_time,
        counter_enable,
        ch_advance_source,
        prescale: _,
        ext_trigger,
        point0_no_clear,
    } = *scan;
    state.dwell_time = dwell_time;
    state.counter_enable = counter_enable;
    // C sets startTime_ before starting the hardware.
    state.start_time = current_time_secs();
    let max_pts = state.max_points.min(num_points);

    // Build channel descriptor list from enabled counters
    let mut chan_descs = Vec::new();
    let mut chan_map = Vec::new();

    // The board's counters, then the digital I/O channel: C scans only the
    // counters the model has (numCounters_), whatever the enable mask says.
    let channels = (0..num_counters).chain(std::iter::once(DIGITAL_IO_COUNTER));
    for i in channels {
        if counter_enable & (1 << i) != 0 {
            // Configure counter for MCS (matches C++ drvUSBCTR startMCS)
            let mode = CMM_OUTPUT_ON
                | CMM_OUTPUT_INITIAL_STATE_HIGH
                | CMM_CLEAR_ON_READ
                | CMM_GATING_ON
                | CMM_INVERT_GATE;
            if i < num_counters
                && let Err(e) = device.counter_config_scan(
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
                )
            {
                fail(format!("counter_config_scan({i}) error: {e}"));
            }

            let (channel, chan_type) = if i == DIGITAL_IO_COUNTER {
                (AUXPORT, DAQI_DIGITAL)
            } else {
                (i as i32, DAQI_CTR32)
            };

            chan_descs.push(DaqInChanDescriptor {
                channel,
                chan_type,
                range: 0,
                ..DaqInChanDescriptor::default()
            });
            chan_map.push(i);
        }
    }

    state.num_counters_enabled = chan_descs.len();
    state.chan_map = chan_map;

    // Allocate scan buffer
    let total_samples = state.num_counters_enabled * max_pts;
    state.scan_buffer.resize(total_samples, 0.0);

    // C passes 1/dwell as is; a zero or negative dwell is libuldaq's to
    // reject, not a cue to run at some other rate.
    let mut rate = 1.0 / dwell_time;

    let mut options = scan_options(dwell_time, ch_advance_source != 0);
    if ext_trigger {
        options |= SO_EXTTRIGGER;
    }

    let mut flags = DAQINSCAN_FF_DEFAULT;
    if point0_no_clear {
        flags |= DAQINSCAN_FF_NOCLEAR;
    }

    // Clear counter 0 output registers to prevent scaler presets from
    // interfering with MCS acquisition (matches C++ drvUSBCTR.cpp).
    for (register, value) in [(CRT_OUTPUT_VAL0, 0), (CRT_OUTPUT_VAL1, 0xFFFF_FFFF)] {
        if let Err(e) = device.counter_load(0, register, value) {
            fail(format!("counter_load({register}) error: {e}"));
        }
    }

    let started = device.daq_in_scan(
        &chan_descs,
        max_pts as i32,
        &mut rate,
        options,
        flags,
        &mut state.scan_buffer,
    );
    // The clock divides the dwell down to what it can do; C writes that
    // actual dwell back to MCA_DWELL_TIME whether or not the scan started.
    state.dwell_time = 1.0 / rate;
    match started {
        Ok(()) => log::info!(
            "MCS started: {} counters, {} points, dwell={dwell_time:.6}s, rate={rate:.0}",
            state.num_counters_enabled,
            max_pts
        ),
        Err(e) => fail(format!("daq_in_scan error: {e}")),
    }

    state.running = true;
    state.current_point = 0;
    failure
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

/// What one poll of the scan found, for the poller to publish (C `readMCS`
/// sets these parameters itself; here the owner of the parameters applies
/// them).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct McsReadout {
    pub current_point: usize,
    /// Seconds since the scan was started or last erased.
    pub elapsed: f64,
    /// The scan ended during this read: MCA_ACQUIRING goes back to 0.
    pub finished: bool,
}

/// C `readMCS`: copy every point transferred since the last read, then end
/// the scan if the hardware has gone idle or PresetReal has run out.
///
/// Both end conditions are checked on every read, whether or not the scan
/// has transferred anything yet -- a scan still waiting for its external
/// trigger or clock must still stop at PresetReal, and one that libuldaq
/// rejected at start is idle from the first read.
pub fn read_mcs(device: &DaqDevice, state: &mut McsState, preset_real: f64) -> McsReadout {
    match device.daq_in_scan_status() {
        Ok((status, xfer)) => {
            if status == SS_IDLE {
                state.running = false;
            }
            copy_transferred_points(state, xfer.current_index);
        }
        Err(e) => log::warn!("MCS scan status error: {e}"),
    }

    let elapsed = state.elapsed_secs();
    if state.running && preset_real > 0.0 && elapsed >= preset_real {
        state.running = false;
    }
    let finished = !state.running;
    if finished {
        scan_stop(device);
    }
    McsReadout {
        current_point: state.current_point,
        elapsed,
        finished,
    }
}

/// Copy the points behind `current_index` that have not been copied yet into
/// the per-counter spectra, stamping each with the time it was read.
fn copy_transferred_points(state: &mut McsState, current_index: i64) {
    let n_chans = state.num_counters_enabled;
    let last_point = points_transferred(current_index, n_chans, state.max_points);
    let now = current_time_secs();
    while state.current_point < last_point {
        let buf_offset = state.current_point * n_chans;
        for (scan_idx, &ctr_idx) in state.chan_map.iter().enumerate() {
            state.mcs_buffers[ctr_idx][state.current_point] =
                state.scan_buffer[buf_offset + scan_idx] as i32;
        }
        state.abs_time_buffer[state.current_point] = now;
        state.current_point += 1;
    }
}

fn scan_stop(device: &DaqDevice) {
    if let Err(e) = device.daq_in_scan_stop() {
        log::warn!("MCS daq_in_scan_stop error: {e}");
    }
}

/// C `stopMCS` on a forced stop: the scan is marked stopped first, then one
/// last [`read_mcs`] collects every point transferred since the previous
/// poll and stops the hardware. `None` if no scan was running.
pub fn stop_mcs(device: &DaqDevice, state: &mut McsState) -> Option<McsReadout> {
    if !state.running {
        return None;
    }
    state.running = false;
    Some(read_mcs(device, state, 0.0))
}

/// Seconds past the EPICS epoch (1990-01-01), as C's
/// `now.secPastEpoch + now.nsec/1.e9` stamps each absolute-time point.
fn current_time_secs() -> f64 {
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
        - EPICS_EPOCH_UNIX_SECS as f64
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn transferred_points_are_copied_once_per_counter() {
        let mut st = McsState::new(4);
        st.num_counters_enabled = 2;
        st.chan_map = vec![0, 8];
        st.scan_buffer = vec![10.0, 1.0, 20.0, 2.0, 30.0, 3.0];
        // Index 3 is the last sample of point 1: two points are complete.
        copy_transferred_points(&mut st, 3);
        assert_eq!(st.current_point, 2);
        assert_eq!(&st.mcs_buffers[0][..2], &[10, 20]);
        assert_eq!(&st.mcs_buffers[8][..2], &[1, 2]);
        // A later read picks up where this one stopped.
        copy_transferred_points(&mut st, 5);
        assert_eq!(st.current_point, 3);
        assert_eq!(st.mcs_buffers[0][2], 30);
    }

    #[test]
    fn nothing_transferred_copies_nothing() {
        let mut st = McsState::new(4);
        st.num_counters_enabled = 2;
        copy_transferred_points(&mut st, -1);
        assert_eq!(st.current_point, 0);
    }

    #[test]
    fn an_erase_clears_the_spectra_but_keeps_the_time_base() {
        let mut st = McsState::new(4);
        compute_times(&mut st, 4, 0.5);
        st.mcs_buffers[0][2] = 7;
        st.current_point = 3;
        st.start_time = 0.0;
        erase_mcs(&mut st);
        assert_eq!(st.mcs_buffers[0], vec![0; 4]);
        assert_eq!(st.current_point, 0);
        assert_eq!(&st.time_buffer[..], &[0.0, 0.5, 1.0, 1.5]);
        assert!(st.elapsed_secs() < 1.0, "the elapsed clock restarted");
    }

    #[test]
    fn the_time_base_follows_the_dwell_over_the_scan_length() {
        let mut st = McsState::new(8);
        assert_eq!(compute_times(&mut st, 4, 0.5), 4);
        assert_eq!(&st.time_buffer[..4], &[0.0, 0.5, 1.0, 1.5]);
        // Never past the buffer, however many points are asked for.
        assert_eq!(compute_times(&mut st, 100, 1.0), 8);
    }

    #[test]
    fn a_long_dwell_is_read_one_sample_at_a_time() {
        assert_eq!(scan_options(0.01, false), SO_SINGLEIO);
        assert_eq!(scan_options(1.0, false), SO_SINGLEIO);
    }

    #[test]
    fn a_short_dwell_leaves_the_transfer_mode_to_libuldaq() {
        assert_eq!(scan_options(0.001, false), SO_DEFAULTIO);
        assert_eq!(scan_options(1e-6, true), SO_DEFAULTIO | SO_EXTCLOCK);
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
