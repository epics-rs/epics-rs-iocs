//! End-to-end injection test: start the port runtime and publisher task, wire a
//! downstream receiver, then push dimensions and array data through the asyn
//! port exactly as the `Dimensions` and `ArrayIn` waveform records would.

use std::time::Duration;

use ad_std_arrays_driver::create_nd_std_arrays;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};
use epics_rs::ad_core::plugin::channel::ndarray_channel;

fn f64s(data: &NDDataBuffer) -> Vec<f64> {
    match data {
        NDDataBuffer::F64(v) => v.clone(),
        other => panic!("expected F64, got {:?}", other.data_type()),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_waveform_write_is_published_as_an_ndarray() {
    let rt = create_nd_std_arrays("NDSA_E2E", 0, 0).unwrap();
    let handle = rt.port_handle().clone();
    let params = rt.params;
    let ndsa = rt.ndsa;

    let (sender, mut receiver) = ndarray_channel("SINK", 8);
    rt.connect_downstream(sender);

    // Configure a 1-D Float64 array of length 8, non-append, OnUpdate.
    handle
        .write_int32(params.base.data_type, 0, NDDataType::Float64 as u8 as i32)
        .await
        .unwrap();
    handle
        .write_int32(params.base.n_dimensions, 0, 1)
        .await
        .unwrap();
    handle
        .write_int32_array(params.base.array_dimensions, 0, vec![8])
        .await
        .unwrap();

    // Start acquiring, then inject the waveform.
    handle.write_int32(params.acquire, 0, 1).await.unwrap();
    let data: Vec<f64> = (0..8).map(|i| i as f64 * 1.5).collect();
    handle
        .write_float64_array(ndsa.array_data, 0, data.clone())
        .await
        .unwrap();

    let array = tokio::time::timeout(Duration::from_secs(5), receiver.recv())
        .await
        .expect("timed out waiting for the injected array")
        .expect("output closed");

    assert_eq!(array.dims.len(), 1);
    assert_eq!(array.dims[0].size, 8);
    assert_eq!(f64s(&array.data), data);
    assert_eq!(array.unique_id, 1);

    // Single image mode: acquisition self-clears after the complete array.
    for _ in 0..50 {
        if handle.read_int32(params.acquire, 0).await.unwrap() == 0 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(handle.read_int32(params.acquire, 0).await.unwrap(), 0);
    assert_eq!(handle.read_int32(params.acquire_busy, 0).await.unwrap(), 0);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn append_mode_publishes_a_growing_array_on_each_write() {
    let rt = create_nd_std_arrays("NDSA_E2E_APPEND", 0, 0).unwrap();
    let handle = rt.port_handle().clone();
    let params = rt.params;
    let ndsa = rt.ndsa;

    let (sender, mut receiver) = ndarray_channel("SINK", 8);
    rt.connect_downstream(sender);

    handle
        .write_int32(params.base.data_type, 0, NDDataType::Float64 as u8 as i32)
        .await
        .unwrap();
    handle
        .write_int32(params.base.n_dimensions, 0, 1)
        .await
        .unwrap();
    handle
        .write_int32_array(params.base.array_dimensions, 0, vec![6])
        .await
        .unwrap();
    handle.write_int32(ndsa.append_mode, 0, 1).await.unwrap();
    // Continuous image mode so the array never self-completes.
    handle.write_int32(params.image_mode, 0, 2).await.unwrap();
    // Fill so the not-yet-written tail is identifiable.
    handle
        .write_float64(ndsa.fill_value, 0, -1.0)
        .await
        .unwrap();

    handle.write_int32(params.acquire, 0, 1).await.unwrap();

    // First append: elements 0..2.
    handle
        .write_float64_array(ndsa.array_data, 0, vec![1.0, 2.0, 3.0])
        .await
        .unwrap();
    let first = tokio::time::timeout(Duration::from_secs(5), receiver.recv())
        .await
        .expect("timed out")
        .expect("closed");
    assert_eq!(f64s(&first.data), vec![1.0, 2.0, 3.0, -1.0, -1.0, -1.0]);

    // Second append: elements 3..5.
    handle
        .write_float64_array(ndsa.array_data, 0, vec![4.0, 5.0, 6.0])
        .await
        .unwrap();
    let second = tokio::time::timeout(Duration::from_secs(5), receiver.recv())
        .await
        .expect("timed out")
        .expect("closed");
    assert_eq!(f64s(&second.data), vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    assert_eq!(second.unique_id, 2, "NDArrayCounter increments per publish");
}
