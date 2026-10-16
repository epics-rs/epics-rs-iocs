//! Port of `simDetector::simTask` (simDetector.cpp:729-883).

use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::trace::TraceMask;

use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;

use crate::image::{Layout, SimEngine};
use crate::params::SimParams;
use crate::rng::Rng;
use crate::shutter::{ShutterOp, shutter_op};
use crate::types::Signal;

/// `MIN_DELAY` (simDetector.cpp:30) — the exposure wait is never zero, so the
/// stop event always gets a chance to be seen.
const MIN_DELAY: f64 = 1e-5;

pub(crate) struct SimTaskContext {
    pub start_rx: rt::CommandReceiver<Signal>,
    pub stop_rx: rt::CommandReceiver<Signal>,
    pub handle: PortHandle,
    pub publisher: ArrayPublisher,
    pub pool: DriverPool,
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
    let _ = handle.set_params_and_notify(0, updates).await;
}

fn int32(reason: usize, value: i32) -> ParamSetValue {
    ParamSetValue::new(reason, 0, ParamValue::Int32(value))
}

fn octet(reason: usize, value: &str) -> ParamSetValue {
    ParamSetValue::new(reason, 0, ParamValue::Octet(value.as_bytes().into()))
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

    /// The task ending the acquisition (C `setIntegerParam(ADAcquire, 0)`);
    /// ADAcquireBusy follows that store, WaitForPlugins included
    /// (asynNDArrayDriver.cpp:636-663).
    async fn end_acquisition(&self) {
        notify(&self.handle, vec![int32(self.ad.acquire, 0)]).await;
    }

    /// `simDetector::computeImage` (simDetector.cpp:505-718).
    /// The error is the text of `computeImage`'s own asynPrint.
    async fn compute_image(&self, engine: &mut SimEngine) -> Result<DriverArray, &'static str> {
        let (mut cfg, read_ok) = self.sim.read_config(&self.handle, &self.ad).await;
        if !read_ok {
            self.handle.trace().output_device(
                self.handle.port_name(),
                Some(0),
                0,
                TraceMask::ERROR,
                "simDetector:computeImage: error getting parameters",
            );
        }

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
        let image = engine.compute_image(&cfg, &self.pool, reset)?;

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
        self.handle
            .set_params_and_notify(0, updates)
            .await
            .map_err(|_| "error setting parameters")?;

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
                // FIXED (doc/upstream-c-defects.md #2): upstream does
                // `status = computeImage(); if (status) continue;` with ADAcquire
                // still set, so a persistently failing computeImage spins the
                // task hot, recomputing with no delay. Instead report the failure
                // via ADStatusError and end the acquisition, so the task returns
                // to waiting for the next start event rather than busy-looping.
                ctx.handle.trace().output_device(
                    ctx.handle.port_name(),
                    Some(0),
                    0,
                    TraceMask::ERROR,
                    &format!("simDetector:computeImage: {e}"),
                );
                notify(
                    &ctx.handle,
                    vec![
                        int32(ctx.ad.status, ADStatus::Error as i32),
                        octet(ctx.ad.status_message, "Image computation failed"),
                    ],
                )
                .await;
                ctx.set_shutter(false).await;
                acquire = false;
                ctx.end_acquisition().await;
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
            // `writeInt32` already stored ADAcquire=0.
            ctx.pool
                .owner()
                .store_array(0, Some(image.share().array().clone()));
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
        image.update_time_stamps_now();

        let image = image.share();
        if array_callbacks {
            ctx.publisher.publish(image.clone()).await;
        }
        ctx.pool.owner().store_array(0, Some(image.array().clone()));

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
            ctx.end_acquisition().await;
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
                    }
                    Wait::TimedOut => {}
                }
            }
        }
    }
}
