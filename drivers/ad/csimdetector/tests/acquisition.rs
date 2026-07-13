//! End-to-end acquisition test: start the port runtime and the simulation task,
//! write `ADAcquire = 1`, and check what lands on the nine NDArray addresses.

use std::time::Duration;

use ad_csimdetector::{MAX_SIGNALS, create_c_sim_detector};
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};
use epics_rs::ad_core::plugin::channel::ndarray_channel;

const NUM_TIME_POINTS: usize = 16;

fn f64s(data: &NDDataBuffer) -> Vec<f64> {
    match data {
        NDDataBuffer::F64(v) => v.clone(),
        other => panic!("expected F64, got {:?}", other.data_type()),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn acquisition_publishes_the_2d_array_and_all_eight_signals() {
    let rt = create_c_sim_detector(
        "CSIM_E2E",
        NUM_TIME_POINTS as i32,
        NDDataType::Float64,
        0,
        0,
    )
    .unwrap();
    let handle = rt.port_handle().clone();
    let sim = rt.sim_params;
    let nd = rt.nd_params;

    // One receiver per address: 0 = the 2-D array, 1..=8 = the 1-D signals.
    let mut receivers = Vec::new();
    for addr in 0..=MAX_SIGNALS {
        let (sender, receiver) = ndarray_channel(&format!("SINK{addr}"), 8);
        rt.connect_downstream(addr, sender);
        receivers.push(receiver);
    }

    // Signal 0 is a unit sine of period 4 s with a 1 s time step, so its four
    // samples per cycle are 0, 1, 0, -1. Every other signal keeps the
    // constructor's zero amplitude at its own address.
    handle.write_float64(sim.time_step, 0, 1.0).await.unwrap();
    handle.write_float64(sim.period, 0, 4.0).await.unwrap();
    handle.write_float64(sim.amplitude, 0, 1.0).await.unwrap();
    // A period of 0 would make `1/period` infinite and every sample NaN.
    for addr in 1..MAX_SIGNALS as i32 {
        handle.write_float64(sim.period, addr, 1.0).await.unwrap();
    }

    handle.write_int32(nd.acquire, 0, 1).await.unwrap();
    assert_eq!(handle.read_int32(nd.acquire_busy, 0).await.unwrap(), 1);

    // The 2-D array: [MAX_SIGNALS, NUM_TIME_POINTS], signal index fastest.
    let image = tokio::time::timeout(Duration::from_secs(5), receivers[0].recv())
        .await
        .expect("timed out waiting for the 2-D array")
        .expect("2-D output closed");
    assert_eq!(image.dims.len(), 2);
    assert_eq!(image.dims[0].size, MAX_SIGNALS);
    assert_eq!(image.dims[1].size, NUM_TIME_POINTS);
    assert_eq!(image.unique_id, 0, "uniqueId_ starts at 0");

    let samples = f64s(&image.data);
    assert_eq!(samples.len(), MAX_SIGNALS * NUM_TIME_POINTS);
    for i in 0..NUM_TIME_POINTS {
        let want = ((i as f64 / 4.0) * 2.0 * std::f64::consts::PI).sin();
        assert!(
            (samples[i * MAX_SIGNALS] - want).abs() < 1e-12,
            "time point {i}: {} != {want}",
            samples[i * MAX_SIGNALS]
        );
    }

    // Each per-signal address gets the matching 1-D slice.
    for j in 0..MAX_SIGNALS {
        let array = tokio::time::timeout(Duration::from_secs(5), receivers[j + 1].recv())
            .await
            .unwrap_or_else(|_| panic!("timed out waiting for signal {j}"))
            .unwrap_or_else(|| panic!("signal {j} output closed"));
        assert_eq!(array.dims.len(), 1, "signal {j} must be 1-D");
        assert_eq!(array.dims[0].size, NUM_TIME_POINTS);
        assert_eq!(array.unique_id, 0, "signal {j} keeps the parent uniqueId");

        let got = f64s(&array.data);
        let want: Vec<f64> = (0..NUM_TIME_POINTS)
            .map(|i| samples[i * MAX_SIGNALS + j])
            .collect();
        assert_eq!(got, want, "signal {j}");
    }

    // `NDArrayCounter` is incremented once per frame.
    assert!(handle.read_int32(nd.array_counter, 0).await.unwrap() >= 1);

    handle.write_int32(nd.acquire, 0, 0).await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_acquire_time_limit_stops_the_acquisition_by_itself() {
    let rt = create_c_sim_detector(
        "CSIM_E2E_STOP",
        NUM_TIME_POINTS as i32,
        NDDataType::Float64,
        0,
        0,
    )
    .unwrap();
    let handle = rt.port_handle().clone();
    let sim = rt.sim_params;
    let nd = rt.nd_params;

    let (sender, mut receiver) = ndarray_channel("SINK0", 8);
    rt.connect_downstream(0, sender);

    // 1 s per point, stop once elapsed > 4 s: 5 of the 16 points are computed.
    handle.write_float64(sim.time_step, 0, 1.0).await.unwrap();
    handle
        .write_float64(sim.acquire_time, 0, 4.0)
        .await
        .unwrap();
    // A flat DC signal 0: offset only, so the computed head is exactly 5.0.
    handle.write_float64(sim.offset, 0, 5.0).await.unwrap();
    handle.write_float64(sim.amplitude, 0, 0.0).await.unwrap();
    for addr in 0..MAX_SIGNALS as i32 {
        handle.write_float64(sim.period, addr, 1.0).await.unwrap();
    }

    handle.write_int32(nd.acquire, 0, 1).await.unwrap();

    let image = tokio::time::timeout(Duration::from_secs(5), receiver.recv())
        .await
        .expect("timed out waiting for the 2-D array")
        .expect("2-D output closed");
    let samples = f64s(&image.data);

    // The computed head carries the offset; the tail is the memset zero.
    for i in 0..5 {
        assert_eq!(samples[i * MAX_SIGNALS], 5.0, "time point {i}");
    }
    for i in 5..NUM_TIME_POINTS {
        assert_eq!(samples[i * MAX_SIGNALS], 0.0, "time point {i}");
    }

    // The task cleared ADAcquire and ADAcquireBusy on its own.
    for _ in 0..50 {
        if handle.read_int32(nd.acquire, 0).await.unwrap() == 0 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert_eq!(handle.read_int32(nd.acquire, 0).await.unwrap(), 0);
    assert_eq!(handle.read_int32(nd.acquire_busy, 0).await.unwrap(), 0);
    assert_eq!(handle.read_float64(sim.elapsed_time, 0).await.unwrap(), 5.0);
}
