use std::time::SystemTime;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::runtime::general_time::EPICS_EPOCH_UNIX_SECS;
use meascomp::analog_in::AInScanConfig;
use meascomp::device::DaqDevice;
use uldaq_sys::*;

use crate::board_enums::map_range;
use crate::params::*;
use crate::trace::{DRIVER_NAME, SelfTrace};

/// Waveform digitizer state.
pub struct WaveDigState {
    pub running: bool,
    pub num_chans: usize,
    pub first_chan: usize,
    pub num_points: usize,
    pub current_point: usize,
    /// Scan buffer (f64, allocated by ulAInScan).
    pub scan_buffer: Vec<f64>,
    /// Per-channel waveform data `[channel][point]`, in volts.
    pub channel_buffers: Vec<Vec<f64>>,
    /// Absolute time per point.
    pub abs_time_buffer: Vec<f64>,
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
        }
    }
}

/// Digitizer settings for [`start_wave_dig`], read from the WaveDig records.
#[derive(Clone, Debug)]
pub struct WaveDigScan {
    pub first_chan: usize,
    pub num_chans: usize,
    pub num_points: usize,
    pub dwell: f64,
    pub input_mode: i32,
    /// C `gainArray`: each scanned channel's own ANALOG_IN_RANGE.
    pub ranges: Vec<i32>,
    pub ext_trigger: bool,
    pub ext_clock: bool,
    pub continuous: bool,
    pub retrigger: bool,
    pub burst_mode: bool,
}

impl WaveDigScan {
    /// C `startWaveDig`'s parameter reads, through whichever owner of the
    /// store the caller is: the port actor directly, the poller by request.
    pub fn read(
        params: &MultiFunctionParams,
        input_mode: i32,
        int: impl Fn(usize, i32) -> i32,
        float: impl Fn(usize, i32) -> f64,
    ) -> Self {
        let first_chan = int(params.wave_dig_first_chan, 0).max(0) as usize;
        let num_chans = int(params.wave_dig_num_chans, 0).max(0) as usize;
        let ranges = (first_chan..first_chan + num_chans)
            .map(|chan| int(params.analog_in_range, chan as i32))
            .collect();
        Self {
            first_chan,
            num_chans,
            // C hands a non-positive count to ulAInScan, which refuses it.
            num_points: int(params.wave_dig_num_points, 0).max(0) as usize,
            dwell: float(params.wave_dig_dwell, 0),
            input_mode,
            ranges,
            ext_trigger: int(params.wave_dig_ext_trigger, 0) != 0,
            ext_clock: int(params.wave_dig_ext_clock, 0) != 0,
            continuous: int(params.wave_dig_continuous, 0) != 0,
            retrigger: int(params.wave_dig_retrigger, 0) != 0,
            burst_mode: int(params.wave_dig_burst_mode, 0) != 0,
        }
    }

    /// What `stopWaveDig` restarts with: the current settings, read when the
    /// scan ends, if WAVEDIG_AUTO_RESTART is set.
    pub fn read_restart(
        params: &MultiFunctionParams,
        input_mode: i32,
        int: impl Fn(usize, i32) -> i32,
        float: impl Fn(usize, i32) -> f64,
    ) -> Option<Self> {
        (int(params.wave_dig_auto_restart, 0) != 0)
            .then(|| Self::read(params, input_mode, int, float))
    }

    /// C `rate = 1./dwell` (drvMultiFunction.cpp:1826), unguarded: uldaq
    /// refuses a dwell <= 0 with ERR_BAD_RATE.
    pub fn rate(&self) -> f64 {
        1.0 / self.dwell
    }
}

/// Start the waveform digitizer (analog input scan). C `startWaveDig`:
/// the uldaq status it reported, and the parameters it set on the way, which
/// include DWELL_ACTUAL even when AInScan refused.
pub fn start_wave_dig(
    device: &DaqDevice,
    state: &mut WaveDigState,
    params: &MultiFunctionParams,
    scan: &WaveDigScan,
    trace: &SelfTrace,
) -> (UlError, Vec<ParamSetValue>) {
    const FUNCTION: &str = "startWaveDig";
    let WaveDigScan {
        first_chan,
        num_chans,
        num_points,
        dwell: _,
        input_mode,
        ref ranges,
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

    // Load input queue for multi-channel scanning (required by uldaq)
    let mut queue = Vec::with_capacity(num_chans);
    for (i, &gain) in ranges.iter().enumerate() {
        queue.push(AiQueueElement {
            channel: (first_chan + i) as i32,
            input_mode,
            range: map_range(trace, gain),
            ..AiQueueElement::default()
        });
    }
    let status = trace.report(
        &device.analog_in_load_queue(&queue),
        FUNCTION,
        "Calling ALoadQueue",
    );
    if status != 0 {
        return (status, Vec::new());
    }

    let mut rate = scan.rate();

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

    let result = device.analog_in_scan(
        &AInScanConfig {
            low_chan: first_chan as i32,
            high_chan: first_chan as i32 + num_chans as i32 - 1,
            input_mode,
            // The queue sets each channel's range.
            range: BIP10VOLTS,
            samples_per_chan: num_points as i32,
            options,
            flags: AINSCAN_FF_DEFAULT,
        },
        &mut rate,
        &mut state.scan_buffer,
    );
    // C converts the rate back to the dwell whatever the scan returned, and
    // shows a rate the device refused as -9999.
    let dwell = 1.0 / rate;
    let dwell_actual = ParamSetValue::new(
        params.wave_dig_dwell_actual,
        0,
        ParamValue::Float64(match result {
            Err(ref e) if e.code == ERR_BAD_RATE => -9999.0,
            _ => dwell,
        }),
    );
    let status = trace.report(&result, FUNCTION, "Calling AInScan");
    if status != 0 {
        return (status, vec![dwell_actual]);
    }

    state.running = true;
    let last_chan = first_chan as i32 + num_chans as i32 - 1;
    trace.print(
        TraceMask::FLOW,
        format_args!(
            "{DRIVER_NAME}:{FUNCTION}: called cbAInScan, firstChan={first_chan}, lastChan={last_chan}, numPoints={num_points}, dwell={:.6}, options=0x{options:x}\n",
            CFloat(dwell)
        ),
    );

    (
        0,
        vec![
            ParamSetValue::new(params.wave_dig_current_point, 0, ParamValue::Int32(0)),
            dwell_actual,
            ParamSetValue::new(params.wave_dig_run, 0, ParamValue::Int32(1)),
            ParamSetValue::new(
                params.wave_dig_total_time,
                0,
                ParamValue::Float64(dwell * num_points as f64),
            ),
        ],
    )
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

/// The digitizer half of C `pollerThread`: copy the points transferred
/// since the last poll and, once the scan is idle, run C `stopWaveDig`.
/// `Err` is the failed `ulAInScanStatus` status, for the poller's
/// once-per-outage report; `Ok(Some(..))` the array callbacks of a scan
/// that just ended.
pub fn read_wave_dig(
    device: &DaqDevice,
    state: &mut WaveDigState,
    params: &MultiFunctionParams,
    restart: Option<&WaveDigScan>,
    trace: &SelfTrace,
) -> Result<Option<Vec<ParamSetValue>>, UlError> {
    let (status, xfer) = device.analog_in_scan_status().map_err(|e| e.code)?;
    trace.print(
        TraceMask::IO_DRIVER,
        format_args!(
            "{DRIVER_NAME}::pollerThread waveform digitizer status, aiStatus={status}, aiCount={}, aiIndex={}\n",
            xfer.current_total_count, xfer.current_index
        ),
    );

    let n_chans = state.num_chans;
    if xfer.current_total_count != 0 && n_chans != 0 && xfer.current_index >= 0 {
        let last_point = points_transferred(xfer.current_index, n_chans, state.num_points);
        let now = current_time_secs();
        while state.current_point < last_point {
            let buf_offset = state.current_point * n_chans;
            for j in 0..n_chans {
                let ch = state.first_chan + j;
                if ch < MAX_ANALOG_IN {
                    state.channel_buffers[ch][state.current_point] =
                        state.scan_buffer[buf_offset + j];
                }
            }
            state.abs_time_buffer[state.current_point] = now;
            state.current_point += 1;
        }
    }

    if status == SS_IDLE {
        return Ok(Some(stop_wave_dig(device, state, params, restart, trace)));
    }
    Ok(None)
}

/// C `stopWaveDig`: clear RUN, hand the acquired points to the records
/// (`readWaveDig`), stop the scan, and under AUTO_RESTART start it again
/// with `restart`, the settings read as it ends.
pub fn stop_wave_dig(
    device: &DaqDevice,
    state: &mut WaveDigState,
    params: &MultiFunctionParams,
    restart: Option<&WaveDigScan>,
    trace: &SelfTrace,
) -> Vec<ParamSetValue> {
    state.running = false;
    let mut updates = vec![ParamSetValue::new(
        params.wave_dig_run,
        0,
        ParamValue::Int32(0),
    )];
    updates.extend(waveform_updates(params, state, trace));
    trace.report(
        &device.analog_in_scan_stop(),
        "stopWaveDig",
        "Stopping AIn scan",
    );
    if let Some(scan) = restart {
        updates.extend(start_wave_dig(device, state, params, scan, trace).1);
    }
    updates
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

/// Array callbacks carrying the digitized data: WAVEDIG_VOLT_WF for each
/// scanned channel plus the absolute time base, both truncated to the points
/// actually acquired. C `MultiFunction::readWaveDig`.
///
/// Returned rather than applied so the actor thread (a WAVEDIG_READ_WF write)
/// and the poller thread can each push them through their own owner.
pub fn waveform_updates(
    params: &MultiFunctionParams,
    state: &WaveDigState,
    trace: &SelfTrace,
) -> Vec<ParamSetValue> {
    let n = state.current_point.min(state.num_points);
    let mut updates = Vec::with_capacity(state.num_chans + 1);
    for j in 0..state.num_chans {
        let ch = state.first_chan + j;
        if ch < MAX_ANALOG_IN {
            trace.print(
                TraceMask::FLOW,
                format_args!(
                    "{DRIVER_NAME}:readWaveDig:, doing callbacks on input {ch}, first value={:.6}\n",
                    CFloat(state.channel_buffers[ch][0])
                ),
            );
            updates.push(ParamSetValue::array(
                params.wave_dig_volt_wf,
                ch as i32,
                ParamValue::Float64Array(state.channel_buffers[ch][..n].into()),
            ));
        }
    }
    updates.push(ParamSetValue::array(
        params.wave_dig_abs_time_wf,
        0,
        ParamValue::Float64Array(state.abs_time_buffer[..n].into()),
    ));
    updates
}

/// C `computeWaveDigTimes`: WAVEDIG_TIME_WF is `i * dwell` over the
/// requested point count and dwell, posted when either is written.
pub fn compute_wave_dig_times(
    params: &MultiFunctionParams,
    num_points: i32,
    dwell: f64,
) -> ParamSetValue {
    ParamSetValue::array(
        params.wave_dig_time_wf,
        0,
        ParamValue::Float32Array(time_axis(num_points, dwell).into()),
    )
}

/// The `i * dwell` time buffers of C `computeWaveDigTimes`/`computeWaveGenTimes`.
pub fn time_axis(num_points: i32, dwell: f64) -> Vec<f32> {
    (0..num_points.max(0))
        .map(|i| (f64::from(i) * dwell) as f32)
        .collect()
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
    fn inputs_9_to_16_get_their_waveforms() {
        use epics_rs::asyn::port::{InterfaceMask, PortDriverBase, PortFlags};
        let mut base = PortDriverBase::new(
            "MFWAVEDIG",
            MAX_SIGNALS,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        );
        let params = MultiFunctionParams::create(&mut base).unwrap();
        let mut state = WaveDigState::new(4);
        state.first_chan = 14;
        state.num_chans = 2;
        state.current_point = 3;
        let addrs: Vec<_> = waveform_updates(&params, &state, &SelfTrace::default())
            .into_iter()
            .filter_map(|u| match u {
                ParamSetValue::Array { reason, addr, .. } if reason == params.wave_dig_volt_wf => {
                    Some(addr)
                }
                _ => None,
            })
            .collect();
        assert_eq!(addrs, [14, 15]);
    }

    /// readWaveDig's trace is C's %f (drvMultiFunction.cpp:1897).
    #[test]
    fn a_nan_first_value_prints_as_cs_percent_f() {
        use crate::driver::MultiFunctionDriver;
        use epics_rs::asyn::registry::claim_port;
        use epics_rs::asyn::runtime::config::RuntimeConfig;
        use epics_rs::asyn::runtime::port::create_port_runtime;
        use epics_rs::asyn::trace::TraceFile;
        use std::sync::{Arc, Mutex};
        let port = "MFNANWF";
        let drv = MultiFunctionDriver::new(port, DaqDevice::unconnected(), 4, 4).unwrap();
        let params = drv.params;
        let (rt, _) =
            create_port_runtime(&claim_port(port).unwrap(), drv, RuntimeConfig::default()).unwrap();
        let trace = SelfTrace::default();
        trace.bind(rt.port_handle().clone());
        let manager = rt.port_handle().trace();
        let log = std::env::temp_dir().join(format!("usb-2408-nanwf-{}.log", std::process::id()));
        let file = std::fs::File::create(&log).unwrap();
        manager.set_trace_file(Some(port), TraceFile::File(Arc::new(Mutex::new(file))));
        manager.set_trace_mask(Some(port), TraceMask::ERROR | TraceMask::FLOW);
        let mut state = WaveDigState::new(4);
        state.channel_buffers[0][0] = f64::NAN;
        state.current_point = 1;
        waveform_updates(&params, &state, &trace);
        let text = std::fs::read_to_string(&log).unwrap();
        let _ = std::fs::remove_file(&log);
        assert!(
            text.contains(
                "MultiFunction:readWaveDig:, doing callbacks on input 0, first value=nan\n"
            ),
            "{text}"
        );
    }

    #[test]
    fn the_scan_rate_is_cs_reciprocal_dwell() {
        let scan = |dwell| WaveDigScan {
            first_chan: 0,
            num_chans: 1,
            num_points: 1,
            dwell,
            input_mode: 0,
            ranges: vec![0],
            ext_trigger: false,
            ext_clock: false,
            continuous: false,
            retrigger: false,
            burst_mode: false,
        };
        let rates = [1e-3, 0.0, -0.5].map(|dwell| scan(dwell).rate());
        assert_eq!(rates, [1000.0, f64::INFINITY, -2.0]);
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
