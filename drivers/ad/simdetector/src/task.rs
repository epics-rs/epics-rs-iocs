//! Port of `simDetector::simTask` (simDetector.cpp:729-883).

use std::sync::Arc;
use std::time::{Duration, Instant};

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDArray;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::image::{Layout, SimEngine};
use crate::params::SimParams;
use crate::rng::Rng;
use crate::shutter::{ShutterOp, shutter_op};
use crate::types::Signal;

/// `MIN_DELAY` (simDetector.cpp:30) — the exposure wait is never zero, so the
/// stop event always gets a chance to be seen.
const MIN_DELAY: f64 = 1e-5;

/// DEVIATION: C's `ADAcquireBusy` is cleared asynchronously by
/// `asynNDArrayDriver::setIntegerParam(NDNumQueuedArrays, 0)`. The Rust task
/// owns no driver reference, so it blocks on the queued-array counter instead.
/// This bounds that block; `ad-core-rs` offers no unbounded variant.
const PLUGIN_DRAIN_TIMEOUT: Duration = Duration::from_secs(5);

pub(crate) struct SimTaskContext {
    pub start_rx: rt::CommandReceiver<Signal>,
    pub stop_rx: rt::CommandReceiver<Signal>,
    pub handle: PortHandle,
    pub publisher: ArrayPublisher,
    pub queued: Arc<QueuedArrayCounter>,
    pub pool: Arc<NDArrayPool>,
    pub ad: ADBaseParams,
    pub sim: SimParams,
}

pub(crate) fn start_sim_task(ctx: SimTaskContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("SimDetTask", move || sim_task(ctx))
}

/// Outcome of a timed wait on the stop event.
enum Wait {
    Stopped,
    TimedOut,
    Shutdown,
}

async fn wait_for_stop(rx: &mut rt::CommandReceiver<Signal>, delay: f64) -> Wait {
    match rt::timeout(Duration::from_secs_f64(delay), rx.recv()).await {
        Ok(Some(Signal)) => Wait::Stopped,
        Ok(None) => Wait::Shutdown,
        Err(rt::Elapsed) => Wait::TimedOut,
    }
}

async fn notify(handle: &PortHandle, updates: Vec<ParamSetValue>) {
    if let Err(e) = handle.set_params_and_notify(0, updates).await {
        log::error!("simDetector: parameter update failed: {e}");
    }
}

fn int32(reason: usize, value: i32) -> ParamSetValue {
    ParamSetValue::Int32 {
        reason,
        addr: 0,
        value,
    }
}

fn octet(reason: usize, value: &str) -> ParamSetValue {
    ParamSetValue::Octet {
        reason,
        addr: 0,
        value: value.to_string(),
    }
}

impl SimTaskContext {
    async fn read_i32(&self, reason: usize) -> i32 {
        self.handle.read_int32(reason, 0).await.unwrap_or(0)
    }

    /// `simDetector::setShutter` reimplemented over the parameter port, because
    /// the task thread holds no `&mut SimDetector`.
    async fn set_shutter(&self, open: bool) {
        let mode = self.read_i32(self.ad.shutter_mode).await;
        match shutter_op(mode, open) {
            ShutterOp::Nothing => {}
            ShutterOp::DetectorStatus(v) => {
                notify(&self.handle, vec![int32(self.ad.shutter_status, v)]).await;
            }
            ShutterOp::EpicsControl(v) => {
                let open_delay = self
                    .handle
                    .read_float64(self.ad.shutter_open_delay, 0)
                    .await
                    .unwrap_or(0.0);
                let close_delay = self
                    .handle
                    .read_float64(self.ad.shutter_close_delay, 0)
                    .await
                    .unwrap_or(0.0);
                notify(&self.handle, vec![int32(self.ad.shutter_control_epics, v)]).await;
                // `ADDriver::setShutter`: epicsThreadSleep(openDelay - closeDelay).
                let delay = open_delay - close_delay;
                if delay > 0.0 {
                    rt::sleep(Duration::from_secs_f64(delay)).await;
                }
            }
        }
    }

    /// The `ADStatus` written when a stop event lands mid-wait
    /// (simDetector.cpp:790-796 and 872-878).
    async fn stopped_status(&self, image_mode: ImageMode) {
        let status = if image_mode == ImageMode::Continuous {
            ADStatus::Idle
        } else {
            ADStatus::Aborted
        };
        notify(&self.handle, vec![int32(self.ad.status, status as i32)]).await;
    }

    /// Clear `ADAcquireBusy`, honouring `ADWaitForPlugins`. Also clears
    /// `ADAcquire` when the task itself is the one ending the acquisition.
    async fn clear_acquire_busy(&self, also_clear_acquire: bool) {
        if self.read_i32(self.ad.wait_for_plugins).await != 0 {
            self.queued.wait_until_zero(PLUGIN_DRAIN_TIMEOUT);
        }
        let mut updates = vec![int32(self.ad.acquire_busy, 0)];
        if also_clear_acquire {
            updates.push(int32(self.ad.acquire, 0));
        }
        notify(&self.handle, updates).await;
    }

    /// `simDetector::computeImage` (simDetector.cpp:505-718).
    async fn compute_image(&self, engine: &mut SimEngine) -> Result<NDArray, String> {
        let mut cfg = self
            .sim
            .read_config(&self.handle, &self.ad)
            .await
            .map_err(|e| format!("error getting parameters: {e}"))?;

        // "Make sure parameters are consistent, fix them if they are not" — and
        // write the corrections back, as C does with setIntegerParam.
        let before = cfg.geometry;
        cfg.geometry.clamp();
        let g = cfg.geometry;
        let mut updates = Vec::new();
        for (changed, reason, value) in [
            (g.min_x != before.min_x, self.ad.min_x, g.min_x),
            (g.min_y != before.min_y, self.ad.min_y, g.min_y),
            (g.size_x != before.size_x, self.ad.size_x, g.size_x),
            (g.size_y != before.size_y, self.ad.size_y, g.size_y),
            (g.bin_x != before.bin_x, self.ad.bin_x, g.bin_x),
            (g.bin_y != before.bin_y, self.ad.bin_y, g.bin_y),
        ] {
            if changed {
                updates.push(int32(reason, value));
            }
        }

        let reset = self.read_i32(self.sim.reset_image).await != 0;
        let image = engine
            .compute_image(&cfg, &self.pool, reset)
            .map_err(|e| format!("error computing image: {e}"))?;

        let layout = Layout::for_color_mode(cfg.color_mode);
        let elements: usize = image.dims.iter().map(|d| d.size).product();
        let total_bytes = elements * cfg.data_type.element_size();
        updates.extend([
            int32(
                self.ad.base.array_size,
                i32::try_from(total_bytes).unwrap_or(i32::MAX),
            ),
            int32(
                self.ad.base.array_size_x,
                image.dims[layout.x_dim].size as i32,
            ),
            int32(
                self.ad.base.array_size_y,
                image.dims[layout.y_dim].size as i32,
            ),
            int32(self.sim.reset_image, 0),
        ]);
        notify(&self.handle, updates).await;

        Ok(image)
    }
}

async fn sim_task(mut ctx: SimTaskContext) {
    let mut engine = SimEngine::new(Rng::from_entropy());
    let mut acquire = false;

    loop {
        if !acquire {
            // Wait for the start event. `None` = the driver has been dropped.
            if ctx.start_rx.recv().await.is_none() {
                return;
            }
            acquire = true;
            notify(
                &ctx.handle,
                vec![
                    octet(ctx.ad.status_message, "Acquiring data"),
                    int32(ctx.ad.num_images_counter, 0),
                ],
            )
            .await;
        }

        let start_time = Instant::now();
        let image_mode = ImageMode::from_i32(ctx.read_i32(ctx.ad.image_mode).await);
        let acquire_time = ctx
            .handle
            .read_float64(ctx.ad.acquire_time, 0)
            .await
            .unwrap_or(0.0);
        let acquire_period = ctx
            .handle
            .read_float64(ctx.ad.acquire_period, 0)
            .await
            .unwrap_or(0.0);

        notify(
            &ctx.handle,
            vec![int32(ctx.ad.status, ADStatus::Acquire as i32)],
        )
        .await;
        ctx.set_shutter(true).await;

        let mut image = match ctx.compute_image(&mut engine).await {
            Ok(image) => image,
            Err(e) => {
                // C: `status = computeImage(); if (status) continue;` — the task
                // stays in the acquiring state and retries on the next pass.
                log::error!("simDetector: {e}");
                continue;
            }
        };

        // Simulate being busy during the exposure.
        let mut delay = acquire_time - start_time.elapsed().as_secs_f64();
        if delay <= 0.0 {
            delay = MIN_DELAY;
        }
        match wait_for_stop(&mut ctx.stop_rx, delay).await {
            Wait::Shutdown => return,
            Wait::Stopped => {
                acquire = false;
                ctx.stopped_status(image_mode).await;
            }
            Wait::TimedOut => {}
        }

        ctx.set_shutter(false).await;

        if !acquire {
            // `writeInt32` already wrote ADAcquire=0; only ADAcquireBusy may
            // still be pending on the plugin queue.
            ctx.clear_acquire_busy(false).await;
            continue;
        }

        notify(
            &ctx.handle,
            vec![int32(ctx.ad.status, ADStatus::Readout as i32)],
        )
        .await;

        let image_counter = ctx.read_i32(ctx.ad.base.array_counter).await + 1;
        let num_images = ctx.read_i32(ctx.ad.num_images).await;
        let num_images_counter = ctx.read_i32(ctx.ad.num_images_counter).await + 1;
        let array_callbacks = ctx.read_i32(ctx.ad.base.array_callbacks).await != 0;
        notify(
            &ctx.handle,
            vec![
                int32(ctx.ad.base.array_counter, image_counter),
                int32(ctx.ad.num_images_counter, num_images_counter),
            ],
        )
        .await;

        image.unique_id = image_counter;
        // `updateTimeStamps` only stamps the NDArray; C's simDetector never
        // writes the NDTimeStamp / NDEpicsTSSec parameters.
        image.timestamp = EpicsTimestamp::now();

        if array_callbacks {
            ctx.publisher.publish(Arc::new(image)).await;
        }

        // See if acquisition is done.
        if image_mode == ImageMode::Single
            || (image_mode == ImageMode::Multiple && num_images_counter >= num_images)
        {
            notify(
                &ctx.handle,
                vec![
                    octet(ctx.ad.status_message, "Waiting for acquisition"),
                    int32(ctx.ad.status, ADStatus::Idle as i32),
                ],
            )
            .await;
            acquire = false;
            ctx.clear_acquire_busy(true).await;
        }

        // Sleep for the acquire period minus the elapsed time.
        if acquire {
            let delay = acquire_period - start_time.elapsed().as_secs_f64();
            if delay >= 0.0 {
                notify(
                    &ctx.handle,
                    vec![int32(ctx.ad.status, ADStatus::Waiting as i32)],
                )
                .await;
                match wait_for_stop(&mut ctx.stop_rx, delay).await {
                    Wait::Shutdown => return,
                    Wait::Stopped => {
                        acquire = false;
                        ctx.stopped_status(image_mode).await;
                        ctx.clear_acquire_busy(false).await;
                    }
                    Wait::TimedOut => {}
                }
            }
        }
    }
}
