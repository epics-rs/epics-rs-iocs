use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::{ADStatus, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::NDArray;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::decode::decode_image;
use crate::fetch::fetch_bytes;
use crate::params::URLParams;
use crate::types::AcqCommand;

/// C++ `char URLString[MAX_FILENAME_LEN]` (ADCore `NDArray.h`, `MAX_FILENAME_LEN == 256`).
const MAX_FILENAME_LEN: usize = 256;

/// Bundled state for the acquisition task thread.
pub(crate) struct AcquisitionContext {
    pub acq_rx: rt::CommandReceiver<AcqCommand>,
    pub handle: PortHandle,
    pub publisher: ArrayPublisher,
    pub queued: Arc<QueuedArrayCounter>,
    pub ad: ADBaseParams,
    pub url: URLParams,
}

impl AcquisitionContext {
    async fn end_acquisition(&self, wait_for_plugins: bool) {
        if wait_for_plugins {
            self.queued.wait_until_zero(Duration::from_secs(5));
        }
        let _ = self
            .handle
            .set_params_and_notify(
                0,
                vec![
                    ParamSetValue::Int32 {
                        reason: self.ad.acquire_busy,
                        addr: 0,
                        value: 0,
                    },
                    ParamSetValue::Int32 {
                        reason: self.ad.status,
                        addr: 0,
                        value: ADStatus::Idle as i32,
                    },
                    ParamSetValue::Int32 {
                        reason: self.ad.acquire,
                        addr: 0,
                        value: 0,
                    },
                ],
            )
            .await;
    }
}

/// Start the acquisition task thread via the `rt` facade.
pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("URLTask", move || acquisition_loop_async(ctx))
}

/// Async equivalent of C++ `ADDriver::setShutter()`. Mirrors
/// `ADDriverBase::set_shutter` (which the task cannot call directly — it
/// only has an async `PortHandle`, not synchronous access to the port's
/// `ADDriverBase`).
async fn set_shutter_via_handle(handle: &PortHandle, ad: &ADBaseParams, open: bool) {
    let mode = ShutterMode::from_i32(handle.read_int32(ad.shutter_mode, 0).await.unwrap_or(0));
    match mode {
        Some(ShutterMode::None) | None => {}
        Some(ShutterMode::DetectorOnly) => {}
        Some(ShutterMode::EpicsOnly) => {
            let open_delay = handle
                .read_float64(ad.shutter_open_delay, 0)
                .await
                .unwrap_or(0.0);
            let close_delay = handle
                .read_float64(ad.shutter_close_delay, 0)
                .await
                .unwrap_or(0.0);
            let _ = handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Int32 {
                        reason: ad.shutter_control_epics,
                        addr: 0,
                        value: if open { 1 } else { 0 },
                    }],
                )
                .await;
            // C++: epicsThreadSleep(shutterOpenDelay - shutterCloseDelay), applied
            // identically on open and close (an upstream ADCore quirk, not specific
            // to this driver).
            let delay = open_delay - close_delay;
            if delay > 0.0 {
                rt::sleep(Duration::from_secs_f64(delay)).await;
            }
        }
    }
}

/// Fetch + decode one image and publish it as an `NDArray`.
///
/// Returns `Ok(())` on success (image published if array callbacks are
/// enabled), `Err(())` on any fetch/decode failure — mirrors C++
/// `readImage()`'s `asynStatus` return, collapsed to success/failure since
/// the caller (matching `URLTask`) only branches on that.
async fn acquire_one_image(
    ctx: &AcquisitionContext,
    start_time: EpicsTimestamp,
    image_counter: i32,
    num_images_counter: i32,
) -> Result<(), ()> {
    let url_bytes = ctx
        .handle
        .read_octet(ctx.url.url_name, 0, MAX_FILENAME_LEN)
        .await
        .unwrap_or_default();
    let url_string = String::from_utf8_lossy(&url_bytes).into_owned();

    let decoded = fetch_bytes(&url_string)
        .map_err(|e| log::error!("ad-url: fetch {url_string}: {e}"))
        .ok()
        .and_then(|bytes| {
            decode_image(&bytes)
                .map_err(|e| log::error!("ad-url: decode {url_string}: {e}"))
                .ok()
        });

    let Some(decoded) = decoded else {
        return Err(());
    };

    // C++ readImage(): ADSizeX/NDArraySizeX use dims[ndims-2] (X), ADSizeY/
    // NDArraySizeY use dims[ndims-1] (Y) — true for both the 2-D Mono
    // ([x, y]) and 3-D RGB1 ([3, x, y]) layouts.
    let ndims = decoded.dims.len();
    let ncols = decoded.dims[ndims - 2].size as i32;
    let nrows = decoded.dims[ndims - 1].size as i32;
    let total_bytes: i64 = decoded.data.total_bytes() as i64;
    let array_size = total_bytes.min(i32::MAX as i64) as i32;
    let data_type = decoded.data.data_type();

    let array_callbacks = ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        != 0;

    let color_mode = decoded.color_mode;
    let mut attributes = NDAttributeList::new();
    attributes.add(NDAttribute::new_static(
        "ColorMode",
        "Color mode",
        NDAttrSource::Driver,
        NDAttrValue::Int32(color_mode as i32),
    ));

    let end_ts = EpicsTimestamp::now();
    let array = NDArray {
        unique_id: image_counter,
        timestamp: end_ts,
        time_stamp: start_time.as_f64(),
        data_size: decoded.data.total_bytes(),
        pool_id: 0,
        dims: decoded.dims,
        data: decoded.data,
        attributes,
        codec: None,
    };

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::Int32 {
                    reason: ctx.ad.size_x,
                    addr: 0,
                    value: ncols,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.array_size_x,
                    addr: 0,
                    value: ncols,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.size_y,
                    addr: 0,
                    value: nrows,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.array_size_y,
                    addr: 0,
                    value: nrows,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.array_size,
                    addr: 0,
                    value: array_size,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.data_type,
                    addr: 0,
                    value: data_type as u8 as i32,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.color_mode,
                    addr: 0,
                    value: color_mode as i32,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.base.array_counter,
                    addr: 0,
                    value: image_counter,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad.num_images_counter,
                    addr: 0,
                    value: num_images_counter,
                },
            ],
        )
        .await;

    if array_callbacks {
        ctx.publisher.publish(Arc::new(array)).await;
    }

    Ok(())
}

async fn acquisition_loop_async(mut ctx: AcquisitionContext) {
    loop {
        let _ = ctx
            .handle
            .set_params_and_notify(
                0,
                vec![ParamSetValue::Int32 {
                    reason: ctx.ad.status,
                    addr: 0,
                    value: ADStatus::Idle as i32,
                }],
            )
            .await;
        let _ = ctx.handle.call_param_callbacks(0).await;

        match ctx.acq_rx.recv().await {
            Some(AcqCommand::Start) => {}
            Some(AcqCommand::Stop) => continue,
            None => break,
        }

        let _ = ctx
            .handle
            .set_params_and_notify(
                0,
                vec![
                    ParamSetValue::Int32 {
                        reason: ctx.ad.num_images_counter,
                        addr: 0,
                        value: 0,
                    },
                    ParamSetValue::Int32 {
                        reason: ctx.ad.acquire_busy,
                        addr: 0,
                        value: 1,
                    },
                ],
            )
            .await;

        let mut image_counter = ctx
            .handle
            .read_int32(ctx.ad.base.array_counter, 0)
            .await
            .unwrap_or(0);
        let mut num_images_counter = 0i32;

        loop {
            let start_time = EpicsTimestamp::now();
            let acquire_period = ctx
                .handle
                .read_float64(ctx.ad.acquire_period, 0)
                .await
                .unwrap_or(1.0);

            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Int32 {
                        reason: ctx.ad.status,
                        addr: 0,
                        value: ADStatus::Acquire as i32,
                    }],
                )
                .await;

            set_shutter_via_handle(&ctx.handle, &ctx.ad, true).await;
            let _ = ctx.handle.call_param_callbacks(0).await;

            image_counter += 1;
            num_images_counter += 1;
            let image_result =
                acquire_one_image(&ctx, start_time, image_counter, num_images_counter).await;

            if image_result.is_err() {
                // Failed fetch/decode: this iteration's counters didn't advance
                // (matches C++, which only increments NDArrayCounter/
                // ADNumImagesCounter inside the `imageStatus == asynSuccess` branch).
                image_counter -= 1;
                num_images_counter -= 1;
            }

            set_shutter_via_handle(&ctx.handle, &ctx.ad, false).await;
            let _ = ctx.handle.call_param_callbacks(0).await;

            // C++ short-circuits on `imageStatus != asynSuccess` before reading
            // ImageMode/NDNumImages, so a failed fetch always stops acquisition
            // regardless of mode (no retry/backoff, unlike hardware-camera ports).
            let stop = if image_result.is_err() {
                true
            } else {
                let image_mode = ImageMode::from_i32(
                    ctx.handle
                        .read_int32(ctx.ad.image_mode, 0)
                        .await
                        .unwrap_or(0),
                );
                let num_images = ctx
                    .handle
                    .read_int32(ctx.ad.num_images, 0)
                    .await
                    .unwrap_or(1);
                image_mode == ImageMode::Single
                    || (image_mode == ImageMode::Multiple && num_images_counter >= num_images)
            };

            if stop {
                let _ = ctx
                    .handle
                    .set_params_and_notify(
                        0,
                        vec![ParamSetValue::Int32 {
                            reason: ctx.ad.acquire,
                            addr: 0,
                            value: 0,
                        }],
                    )
                    .await;
            }

            let _ = ctx.handle.call_param_callbacks(0).await;

            let acquire = ctx.handle.read_int32(ctx.ad.acquire, 0).await.unwrap_or(0);
            if acquire == 0 {
                break;
            }

            let elapsed = EpicsTimestamp::now().as_f64() - start_time.as_f64();
            let mut delay = acquire_period - elapsed;
            if delay <= 0.0 {
                delay = 0.001;
            }

            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Int32 {
                        reason: ctx.ad.status,
                        addr: 0,
                        value: ADStatus::Waiting as i32,
                    }],
                )
                .await;
            let _ = ctx.handle.call_param_callbacks(0).await;

            match rt::timeout(Duration::from_secs_f64(delay), ctx.acq_rx.recv()).await {
                Ok(Some(AcqCommand::Stop)) | Ok(None) => break,
                _ => {}
            }
        }

        let wait_for_plugins = ctx
            .handle
            .read_int32(ctx.ad.base.wait_for_plugins, 0)
            .await
            .unwrap_or(0)
            != 0;
        ctx.end_acquisition(wait_for_plugins).await;
    }
}
