//! Port of `ADCSimDetector::simTask` (ADCSimDetector.cpp:270-346).

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use epics_rs::ad_core::error::ADResult;
use epics_rs::ad_core::ndarray::{NDArray, NDDimension};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ndarray_driver::NDArrayDriverParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::driver::LastArray;
use crate::params::CSimParams;
use crate::rng::Rng;
use crate::signals::compute_arrays;
use crate::types::{MAX_SIGNALS, Signal};

pub(crate) struct CSimTaskContext {
    pub start_rx: rt::CommandReceiver<Signal>,
    pub stop_rx: rt::CommandReceiver<Signal>,
    /// C `setAcquire(0)` from inside `computeArraysT`'s acquire-time break.
    pub stop_tx: rt::CommandSender<Signal>,
    pub acquiring: Arc<AtomicBool>,
    pub handle: PortHandle,
    /// Index 0 publishes the 2-D array; `1..=MAX_SIGNALS` the 1-D signals.
    pub publishers: Vec<ArrayPublisher>,
    pub queued: Arc<QueuedArrayCounter>,
    pub pool: Arc<NDArrayPool>,
    pub last_array: LastArray,
    pub nd: NDArrayDriverParams,
    pub sim: CSimParams,
}

pub(crate) fn start_sim_task(ctx: CSimTaskContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("SimDetTask", move || sim_task(ctx))
}

fn int32(reason: usize, addr: i32, value: i32) -> ParamSetValue {
    ParamSetValue::new(reason, addr, ParamValue::Int32(value))
}

fn float64(reason: usize, addr: i32, value: f64) -> ParamSetValue {
    ParamSetValue::new(reason, addr, ParamValue::Float64(value))
}

impl CSimTaskContext {
    async fn notify(&self, addr: i32, updates: Vec<ParamSetValue>) {
        if let Err(e) = self.handle.set_params_and_notify(addr, updates).await {
            log::error!("ADCSimDetector: parameter update failed: {e}");
        }
    }

    /// The `ADAcquire` half of `asynNDArrayDriver::setIntegerParam`, for the
    /// task's own `setIntegerParam(ADAcquire, 0)` (ADCSimDetector.cpp:206).
    async fn acquire_off_updates(&self) -> Vec<ParamSetValue> {
        let wait_for_plugins = self
            .handle
            .read_int32(self.nd.wait_for_plugins, 0)
            .await
            .unwrap_or(0)
            != 0;
        let mut updates = Vec::with_capacity(2);
        if !wait_for_plugins || self.queued.get() == 0 {
            updates.push(int32(self.nd.acquire_busy, 0, 0));
        }
        updates.push(int32(self.nd.acquire, 0, 0));
        updates
    }
}

/// Slice signal `i` out of the `[MAX_SIGNALS, numTimePoints]` array and reshape
/// the `[1, numTimePoints]` result to `[numTimePoints]`
/// (ADCSimDetector.cpp:314-321).
fn extract_signal(pool: &NDArrayPool, image: &NDArray, i: usize) -> ADResult<NDArray> {
    let dims_out = vec![
        NDDimension {
            size: 1,
            offset: i,
            binning: 1,
            reverse: false,
        },
        image.dims[1].clone(),
    ];
    let mut array = pool.convert(image, &dims_out, image.data.data_type())?;
    // `NDArrayPool::convert` copies the timestamps and attributes but not
    // `uniqueId`, which C's does (NDArrayPool.cpp:658-660).
    array.unique_id = image.unique_id;
    // `pArray->ndims = 1; pArray->dims[0] = pArray->dims[1];`
    array.dims = vec![array.dims[1].clone()];
    Ok(array)
}

async fn sim_task(mut ctx: CSimTaskContext) {
    let mut rng = Rng::from_entropy();
    // C `uniqueId_`, `elapsedTime_`, `acquiring_`.
    let mut unique_id: i32 = 0;
    let mut elapsed_time = 0.0f64;
    let mut acquiring = false;

    loop {
        // "Has acquisition been stopped?"
        if ctx.stop_rx.try_recv().is_ok() {
            acquiring = false;
            ctx.acquiring.store(false, Ordering::Release);
        }

        if !acquiring {
            // Release the lock while we wait for the start event. `None` = the
            // driver has been dropped.
            if ctx.start_rx.recv().await.is_none() {
                return;
            }
            acquiring = true;
            ctx.acquiring.store(true, Ordering::Release);
            elapsed_time = 0.0;
        }

        // `computeArrays` (ADCSimDetector.cpp:216-253).
        let cfg = ctx.sim.read_config(&ctx.handle, &ctx.nd).await;
        // DEVIATION: C would allocate a zero-length NDArray and then hand a
        // zero-size dimension to `NDArrayPool::convert`. `ad-core-rs` rejects a
        // zero-size dimension, so the frame is skipped instead.
        if cfg.num_time_points == 0 {
            log::error!("ADCSimDetector: NumTimePoints is 0, skipping frame");
            rt::sleep(Duration::from_secs_f64(0.001)).await;
            continue;
        }

        let result = compute_arrays(&cfg, elapsed_time, &mut rng);
        elapsed_time = result.elapsed_time;

        // `setAcquire(0)` in the acquire-time break, before the array callbacks.
        if result.acquire_finished {
            let _ = ctx.stop_tx.try_send(Signal);
        }

        let dims = vec![
            NDDimension::new(MAX_SIGNALS),
            NDDimension::new(cfg.num_time_points),
        ];
        let mut image = match ctx.pool.alloc(dims, cfg.data_type) {
            Ok(image) => image,
            Err(e) => {
                log::error!("ADCSimDetector: error allocating array: {e}");
                continue;
            }
        };
        image.data = result.data;

        image.unique_id = unique_id;
        unique_id += 1;

        let array_counter = ctx
            .handle
            .read_int32(ctx.nd.array_counter, 0)
            .await
            .unwrap_or(0)
            + 1;

        let now = EpicsTimestamp::now();
        image.time_stamp = now.sec as f64 + now.nsec as f64 / 1.0e9;
        image.timestamp = now;
        // DEVIATION: C calls `getAttributes(pImage->pAttributeList)` here. The
        // `NDAttributesFile` machinery lives in `NDArrayDriverBase`, which this
        // driver cannot use (it hardcodes `max_addr = 1`), so no attributes are
        // attached.

        let image = Arc::new(image);
        *ctx.last_array.lock() = Some(image.clone());

        // `doCallbacksGenericPointer(pImage, NDArrayData, 0)` — unconditional:
        // ADCSimDetector never tests `NDArrayCallbacks`.
        ctx.publishers[0].publish(image.clone()).await;

        // Per-signal 1-D arrays.
        for i in 0..MAX_SIGNALS {
            match extract_signal(&ctx.pool, &image, i) {
                Ok(array) => {
                    ctx.publishers[i + 1].publish(Arc::new(array)).await;
                }
                Err(e) => log::error!("ADCSimDetector: error converting signal {i}: {e}"),
            }

            // `callParamCallbacks(i)` — parameter list `i`, not `i+1`.
            let addr = i as i32;
            let mut updates = vec![float64(ctx.sim.frequency, addr, result.frequencies[i])];
            if i == 0 {
                updates.push(float64(ctx.sim.elapsed_time, 0, elapsed_time));
                updates.push(int32(ctx.nd.array_counter, 0, array_counter));
                if result.acquire_finished {
                    updates.extend(ctx.acquire_off_updates().await);
                }
            }
            ctx.notify(addr, updates).await;
        }

        // "Sleep for the acquire period"; the parameters are re-read because a
        // client may have changed them during the frame.
        let num_time_points = ctx
            .handle
            .read_int32(ctx.sim.num_time_points, 0)
            .await
            .unwrap_or(0)
            .max(0) as f64;
        let time_step = ctx
            .handle
            .read_float64(ctx.sim.time_step, 0)
            .await
            .unwrap_or(0.0);
        let delay = num_time_points * time_step;
        if delay > 0.0 {
            rt::sleep(Duration::from_secs_f64(delay)).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};

    /// A `[MAX_SIGNALS, n]` array whose element `(i, j)` is `100*i + j`, so a
    /// correct extraction of signal `j` is `[j, 100+j, 200+j, ...]`.
    fn fixture(pool: &NDArrayPool, n: usize) -> NDArray {
        let mut image = pool
            .alloc(
                vec![NDDimension::new(MAX_SIGNALS), NDDimension::new(n)],
                NDDataType::Float64,
            )
            .unwrap();
        let data: Vec<f64> = (0..n)
            .flat_map(|i| (0..MAX_SIGNALS).map(move |j| (100 * i + j) as f64))
            .collect();
        image.data = NDDataBuffer::F64(data);
        image.unique_id = 42;
        image
    }

    #[test]
    fn each_signal_is_taken_with_a_stride_of_max_signals() {
        let pool = NDArrayPool::new(0);
        let image = fixture(&pool, 5);
        for j in 0..MAX_SIGNALS {
            let array = extract_signal(&pool, &image, j).unwrap();
            let got = match &array.data {
                NDDataBuffer::F64(v) => v.clone(),
                other => panic!("expected F64, got {:?}", other.data_type()),
            };
            let want: Vec<f64> = (0..5).map(|i| (100 * i + j) as f64).collect();
            assert_eq!(got, want, "signal {j}");
        }
    }

    #[test]
    fn the_extracted_signal_is_one_dimensional_with_the_time_point_count() {
        let pool = NDArrayPool::new(0);
        let image = fixture(&pool, 5);
        let array = extract_signal(&pool, &image, 3).unwrap();
        assert_eq!(array.dims.len(), 1);
        assert_eq!(array.dims[0].size, 5);
    }

    #[test]
    fn the_extracted_signal_carries_the_parent_unique_id() {
        // `NDArrayPool::convert` does not copy `unique_id`; `extract_signal`
        // restores C's behaviour.
        let pool = NDArrayPool::new(0);
        let image = fixture(&pool, 4);
        assert_eq!(extract_signal(&pool, &image, 0).unwrap().unique_id, 42);
    }

    #[test]
    fn the_extracted_signal_keeps_the_source_data_type() {
        let pool = NDArrayPool::new(0);
        let mut image = fixture(&pool, 3);
        image.data = NDDataBuffer::I16(vec![7; MAX_SIGNALS * 3]);
        let array = extract_signal(&pool, &image, 2).unwrap();
        assert_eq!(array.data.data_type(), NDDataType::Int16);
    }
}
