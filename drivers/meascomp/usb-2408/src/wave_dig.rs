use std::time::SystemTime;

use epics_rs::base::runtime::general_time::EPICS_EPOCH_UNIX_SECS;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::request::ParamSetValue;
use meascomp::analog_in::AInScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::params::*;

/// Waveform digitizer state.
pub struct WaveDigState {
    pub running: bool,
    pub num_chans: usize,
    pub first_chan: usize,
    pub num_points: usize,
    pub current_point: usize,
    /// Scan buffer (f64, allocated by ulAInScan).
    pub scan_buffer: Vec<f64>,
    /// Per-channel waveform data [channel][point], in volts.
    pub channel_buffers: Vec<Vec<f64>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
    /// Time waveform per point.
    pub time_buffer: Vec<f32>,
    pub dwell_actual: f64,
    /// Bumped by every start, so a report that an earlier scan ended can
    /// never end a later one.
    pub generation: u64,
}

impl WaveDigState {
    pub fn new(max_points: usize) -> Self {
        let mut channel_buffers = Vec::with_capacity(MAX_ANALOG_IN);
        for _ in 0..MAX_ANALOG_IN {
            channel_buffers.push(vec![0.0f64; max_points]);
        }
        Self {
            running: false,
            num_chans: MAX_ANALOG_IN,
            first_chan: 0,
            num_points: max_points,
            current_point: 0,
            scan_buffer: Vec::new(),
            channel_buffers,
            abs_time_buffer: vec![0.0; max_points],
            time_buffer: vec![0.0; max_points],
            dwell_actual: 0.001,
            generation: 0,
        }
    }
}

/// Digitizer settings for [`start_wave_dig`], read from the WaveDig records.
#[derive(Clone, Copy, Debug)]
pub struct WaveDigScan {
    pub first_chan: usize,
    pub num_chans: usize,
    pub num_points: usize,
    pub dwell: f64,
    pub input_mode: i32,
    /// Each channel's ANALOG_IN_RANGE, by absolute channel.
    pub ranges: [i32; MAX_ANALOG_IN],
    pub ext_trigger: bool,
    pub ext_clock: bool,
    pub continuous: bool,
    pub retrigger: bool,
    pub burst_mode: bool,
}

/// C's WAVEDIG_DWELL_ACTUAL for a scan the device refused its rate for.
pub const BAD_RATE_DWELL: f64 = -9999.0;

/// A refused start. `dwell_actual` is `Some` when `ulAInScan` itself ran:
/// C publishes the dwell it ended with, or [`BAD_RATE_DWELL`], after any
/// outcome of that call, but nothing when the queue load failed before it.
#[derive(Debug)]
pub struct WaveDigStartError {
    pub message: String,
    pub dwell_actual: Option<f64>,
}

/// Start the waveform digitizer (analog input scan). Returns the actual
/// dwell the device runs at.
pub fn start_wave_dig(
    device: &DaqDevice,
    state: &mut WaveDigState,
    scan: &WaveDigScan,
) -> Result<f64, WaveDigStartError> {
    let WaveDigScan {
        first_chan,
        num_chans,
        num_points,
        dwell,
        input_mode,
        ranges,
        ext_trigger,
        ext_clock,
        continuous,
        retrigger,
        burst_mode,
    } = *scan;
    state.first_chan = first_chan;
    state.num_chans = num_chans;
    state.num_points = num_points;
    state.current_point = 0;

    let total_samples = num_chans * num_points;
    state.scan_buffer.resize(total_samples, 0.0);

    // C startWaveDig: the queue gives every scanned channel its own range
    // (drvMultiFunction.cpp:1787-1802).
    let queue = scan_queue(first_chan, num_chans, input_mode, &ranges);
    device
        .analog_in_load_queue(&queue)
        .map_err(|e| WaveDigStartError {
            message: format!("analog_in_load_queue error: {e}"),
            dwell_actual: None,
        })?;

    let mut rate = if dwell > 0.0 { 1.0 / dwell } else { 1000.0 };

    let mut options = SO_DEFAULTIO;
    if ext_trigger {
        options |= SO_EXTTRIGGER;
    }
    if ext_clock {
        options |= SO_EXTCLOCK;
    }
    if continuous {
        options |= SO_CONTINUOUS;
    }
    if retrigger {
        options |= SO_RETRIGGER;
    }
    if burst_mode {
        options |= SO_BURSTMODE;
    }

    let scanned = device.analog_in_scan(
        &AInScanConfig {
            low_chan: first_chan as i32,
            high_chan: (first_chan + num_chans - 1) as i32,
            input_mode,
            // The loaded queue sets each channel's range; C passes
            // BIP10VOLTS here.
            range: BIP10VOLTS,
            samples_per_chan: num_points as i32,
            options,
            flags: AINSCAN_FF_DEFAULT,
        },
        &mut rate,
        &mut state.scan_buffer,
    );
    // C drvMultiFunction.cpp:1836-1846: the dwell the rate came back as, or
    // -9999 when the device rejected the rate outright.
    let dwell_actual = match &scanned {
        Err(e) if e.code == ERR_BAD_RATE => BAD_RATE_DWELL,
        _ => 1.0 / rate,
    };
    if let Err(e) = scanned {
        return Err(WaveDigStartError {
            message: format!("analog_in_scan error: {e}"),
            dwell_actual: Some(dwell_actual),
        });
    }

    state.dwell_actual = dwell_actual;
    state.running = true;
    state.generation = state.generation.wrapping_add(1);

    log::info!(
        "WaveDig started: ch{first_chan}-{}, {num_points} pts, rate={rate:.0} Hz",
        first_chan + num_chans - 1
    );
    Ok(dwell_actual)
}

/// The `ulAInLoadQueue` entries for `num_chans` channels from `first_chan`,
/// each with its own range.
fn scan_queue(
    first_chan: usize,
    num_chans: usize,
    input_mode: i32,
    ranges: &[i32; MAX_ANALOG_IN],
) -> Vec<AiQueueElement> {
    (first_chan..first_chan + num_chans)
        .map(|chan| AiQueueElement {
            channel: chan as i32,
            input_mode,
            range: ranges.get(chan).copied().unwrap_or(BIP10VOLTS),
            ..AiQueueElement::default()
        })
        .collect()
}

/// Number of complete scan points behind `current_index`.
///
/// ulAInScanStatus reports `currentIndex` as the position of the LAST sample
/// written, so the count is `index / chans + 1` -- C drvMultiFunction.cpp's
/// `lastPoint = aiIndex / numWaveDigChans_ + 1`. Clamped to `num_points`
/// because the per-channel buffers are sized once at construction.
pub fn points_transferred(current_index: i64, n_chans: usize, num_points: usize) -> usize {
    if current_index < 0 || n_chans == 0 {
        return 0;
    }
    (current_index as usize / n_chans + 1).min(num_points)
}

/// C pollerThread's digitizer block: copy the points transferred since the
/// last poll, and tell whether the scan has gone idle -- which it also is
/// when libuldaq reports a transfer error with the status. Ending the scan
/// (Run back to 0, the data delivered, the scan stopped, an auto-restart)
/// is the driver's one transition, not this read's.
pub fn read_wave_dig(device: &DaqDevice, state: &mut WaveDigState) -> bool {
    let report = device.analog_in_scan_status();
    if let Some(e) = &report.error {
        log::warn!("WaveDig scan status error: {e}");
    }
    let n_chans = state.num_chans;
    let last_point = points_transferred(report.xfer.current_index, n_chans, state.num_points);
    let now = current_time_secs();
    while state.current_point < last_point {
        let buf_offset = state.current_point * n_chans;
        for j in 0..n_chans {
            let ch = state.first_chan + j;
            if ch < MAX_ANALOG_IN {
                state.channel_buffers[ch][state.current_point] = state.scan_buffer[buf_offset + j];
            }
        }
        state.abs_time_buffer[state.current_point] = now;
        state.current_point += 1;
    }
    report.status == SS_IDLE
}

/// Stop the waveform digitizer.
pub fn stop_wave_dig(device: &DaqDevice, state: &mut WaveDigState) {
    if state.running {
        if let Err(e) = device.analog_in_scan_stop() {
            log::warn!("WaveDig scan stop error: {e}");
        }
        state.running = false;
    }
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

/// Array callbacks carrying the digitized data: WAVEDIG_VOLT_WF for each
/// scanned channel plus the absolute time base, both truncated to the points
/// actually acquired. C `MultiFunction::readWaveDig`.
///
/// Returned rather than applied so the actor thread (a WAVEDIG_READ_WF write)
/// and the poller thread can each push them through their own owner.
pub fn waveform_updates(params: &MultiFunctionParams, state: &WaveDigState) -> Vec<ParamSetValue> {
    let n = state.current_point.min(state.num_points);
    let mut updates = Vec::with_capacity(state.num_chans + 1);
    for j in 0..state.num_chans {
        let ch = state.first_chan + j;
        if ch < MAX_ANALOG_IN {
            updates.push(ParamSetValue::new(
                params.wave_dig_volt_wf,
                ch as i32,
                ParamValue::Float64Array(state.channel_buffers[ch][..n].into()),
            ));
        }
    }
    updates.push(ParamSetValue::new(
        params.wave_dig_abs_time_wf,
        0,
        ParamValue::Float64Array(state.abs_time_buffer[..n].into()),
    ));
    updates
}

/// C `computeWaveDigTimes` / `computeWaveGenTimes`: the relative time base
/// `i * dwell` over `num_points`, bounded by the buffer. Returns the number
/// of points written, which is also the length of the array callback.
pub fn compute_times(buffer: &mut [f32], num_points: usize, dwell: f64) -> usize {
    let n = num_points.min(buffer.len());
    for (i, t) in buffer[..n].iter_mut().enumerate() {
        *t = (i as f64 * dwell) as f32;
    }
    n
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
    fn every_queued_channel_keeps_its_own_range() {
        let mut ranges = [BIP10VOLTS; MAX_ANALOG_IN];
        ranges[2] = BIP1VOLTS;
        let queue = scan_queue(1, 3, AI_DIFFERENTIAL, &ranges);
        let got: Vec<_> = queue.iter().map(|q| (q.channel, q.range)).collect();
        assert_eq!(got, vec![(1, BIP10VOLTS), (2, BIP1VOLTS), (3, BIP10VOLTS)]);
    }

    #[test]
    fn the_time_base_follows_the_dwell_over_the_scan_length() {
        let mut buf = [0.0f32; 8];
        assert_eq!(compute_times(&mut buf, 3, 0.25), 3);
        assert_eq!(&buf[..3], &[0.0, 0.25, 0.5]);
        assert_eq!(compute_times(&mut buf, 100, 1.0), 8);
    }

    #[test]
    fn a_completed_scan_counts_every_point() {
        // 2 channels x 50 points: the last sample written is index 98.
        assert_eq!(points_transferred(98, 2, 50), 50);
        // 1 channel x 20 points: index 19.
        assert_eq!(points_transferred(19, 1, 20), 20);
    }

    #[test]
    fn the_first_sample_of_a_point_already_counts_it() {
        // C counts a point as soon as its first channel lands, so index 0 and
        // index 1 of a 2-channel scan are both "point 1".
        assert_eq!(points_transferred(0, 2, 50), 1);
        assert_eq!(points_transferred(1, 2, 50), 1);
        assert_eq!(points_transferred(2, 2, 50), 2);
    }

    #[test]
    fn an_index_past_the_buffer_is_clamped() {
        assert_eq!(points_transferred(4096, 2, 50), 50);
    }

    #[test]
    fn no_transfer_yet_is_zero_points() {
        assert_eq!(points_transferred(-1, 2, 50), 0);
        assert_eq!(points_transferred(10, 0, 50), 0);
    }
}
