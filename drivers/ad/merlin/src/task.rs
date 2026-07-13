//! The two background tasks (C `merlinTask` and `merlinStatus`).
//!
//! The data task owns the data socket and never touches the command socket:
//! it waits for frames, decodes them and publishes NDArrays. The status task
//! only kicks the driver's startup sequence and refreshes the idle message.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::{ChannelError, MpxConnection};
use crate::driver::SharedState;
use crate::image;
use crate::params::MerlinParams;
use crate::protocol;
use crate::types::*;

/// Everything the data task needs.
pub(crate) struct AcquisitionContext {
    pub data: MpxConnection,
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    #[allow(dead_code)] // held so plugins can be back-pressured on the queue
    pub queued: Arc<QueuedArrayCounter>,
    pub ad_params: ADBaseParams,
    pub params: MerlinParams,
    pub init_param: usize,
    pub det_type: DetectorType,
    pub shared: Arc<SharedState>,
}

/// Spawn the data task and the status task.
pub(crate) fn start_acquisition_task(
    ctx: AcquisitionContext,
) -> (std::thread::JoinHandle<()>, std::thread::JoinHandle<()>) {
    let status_handle = ctx.handle.clone();
    let status_params = ctx.ad_params;
    let init_param = ctx.init_param;

    let data = rt::run_thread_named("MerlinDataTask", move || data_loop(ctx));
    let status = rt::run_thread_named("MerlinStatusTask", move || {
        status_loop(status_handle, status_params, init_param)
    });
    (data, status)
}

/// C waited 4 s for the startup script and autosave to finish before touching
/// the device, so that the settings it pushes are the restored ones.
const STARTUP_DELAY: Duration = Duration::from_secs(4);
const STATUS_POLL: Duration = Duration::from_secs(4);
/// A socket error would otherwise spin this loop as fast as the kernel can
/// fail the read.
const ERROR_BACKOFF: Duration = Duration::from_secs(5);

async fn status_loop(handle: PortHandle, ad: ADBaseParams, init_param: usize) {
    rt::sleep(STARTUP_DELAY).await;

    // Run the startup sequence on the port actor: it owns the command channel,
    // so this is the only way to touch the device without a second owner.
    let sync = SyncIOHandle::from_handle(handle.clone(), 0, Duration::from_secs(60));
    if let Err(e) = sync.write_int32(init_param, 1) {
        log::error!("merlin: startup sequence failed: {e}");
    }

    loop {
        rt::sleep(STATUS_POLL).await;
        let Ok(status) = sync.read_int32(ad.status) else {
            continue;
        };
        if status == ADStatus::Idle as i32 {
            let _ = handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Octet {
                        reason: ad.status_message,
                        addr: 0,
                        value: "Waiting for acquire command".into(),
                    }],
                )
                .await;
        }
    }
}

async fn data_loop(ctx: AcquisitionContext) {
    // Do not read the data channel before the IOC is up: the server can be
    // mid-flight with frames from a previous run.
    while ctx.shared.starting_up.load(Ordering::Acquire) {
        rt::sleep(Duration::from_millis(500)).await;
    }

    let sync = SyncIOHandle::from_handle(ctx.handle.clone(), 0, Duration::from_secs(5));
    let timeout = Duration::from_secs_f64(DATA_READ_TIMEOUT_SEC);
    // The most recent acquisition header, attached to every frame that follows
    // it (string attributes are global in the HDF5 plugin, so the file gets the
    // header of the acquisition that wrote it).
    let mut acquisition_header = String::new();

    loop {
        let body = match ctx.data.read_frame(timeout) {
            Ok(b) => b,
            // An idle detector sends nothing; that is not an error.
            Err(ChannelError::Timeout) => continue,
            Err(e) => {
                log::error!("merlin: error on the Labview data channel: {e}");
                let _ = ctx
                    .handle
                    .set_params_and_notify(
                        0,
                        vec![ParamSetValue::Octet {
                            reason: ctx.ad_params.status_message,
                            addr: 0,
                            value: "Error in Labview data channel response".into(),
                        }],
                    )
                    .await;
                rt::sleep(ERROR_BACKOFF).await;
                continue;
            }
        };

        let start = EpicsTimestamp::now();
        let header = protocol::data_header(&body);

        // The acquisition header is not an image, so it advances no counter.
        let mut array_counter = sync
            .read_int32(ctx.ad_params.base.array_counter)
            .unwrap_or(0);
        if header != DataHeader::Acquisition {
            let images = sync
                .read_int32(ctx.ad_params.num_images_counter)
                .unwrap_or(0)
                + 1;
            array_counter += 1;
            let _ = ctx.shared.images_remaining.fetch_update(
                Ordering::AcqRel,
                Ordering::Acquire,
                |n| if n > 0 { Some(n - 1) } else { None },
            );
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Int32 {
                        reason: ctx.ad_params.num_images_counter,
                        addr: 0,
                        value: images,
                    }],
                )
                .await;
        }

        let array_callbacks = sync
            .read_int32(ctx.ad_params.base.array_callbacks)
            .unwrap_or(1)
            != 0;

        if array_callbacks {
            match header {
                DataHeader::Acquisition => {
                    let text = String::from_utf8_lossy(&body);
                    let end = text.len().min(MPX_ACQUISITION_HEADER_LEN);
                    acquisition_header = text[..end].to_string();
                }
                DataHeader::QuadData => {
                    publish_image(&ctx, &body, array_counter, start, &acquisition_header).await;
                }
                DataHeader::Profile => {
                    publish_profiles(&ctx, &body, array_counter, start, &acquisition_header).await;
                }
                DataHeader::Unknown => {
                    log::error!(
                        "merlin: unknown data frame type '{}'",
                        String::from_utf8_lossy(&body[..body.len().min(MPX_MSG_DATATYPE_LEN)])
                    );
                }
            }
        }

        // A software trigger is one-shot: clear it once its frame arrives.
        let trigger = sync.read_int32(ctx.ad_params.trigger_mode).unwrap_or(0);
        if trigger == TriggerMode::SoftwareTrigger as i32 {
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Int32 {
                        reason: ctx.params.software_trigger,
                        addr: 0,
                        value: 0,
                    }],
                )
                .await;
        }

        if ctx.shared.images_remaining.load(Ordering::Acquire) == 0 {
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![
                        ParamSetValue::Int32 {
                            reason: ctx.ad_params.acquire,
                            addr: 0,
                            value: 0,
                        },
                        ParamSetValue::Int32 {
                            reason: ctx.ad_params.status,
                            addr: 0,
                            value: ADStatus::Idle as i32,
                        },
                    ],
                )
                .await;
        }
    }
}

fn build_attributes(parsed: &[(String, NDAttrValue)], acquisition_header: &str) -> NDAttributeList {
    let mut attrs = NDAttributeList::new();
    for (name, value) in parsed {
        attrs.add(NDAttribute {
            name: name.clone(),
            description: String::new(),
            source: NDAttrSource::Driver,
            value: value.clone(),
            source_impl: None,
        });
    }
    attrs.add(NDAttribute {
        name: "Acquisition Header".into(),
        description: String::new(),
        source: NDAttrSource::Driver,
        value: NDAttrValue::String(acquisition_header.to_string()),
        source_impl: None,
    });
    attrs
}

async fn publish_image(
    ctx: &AcquisitionContext,
    body: &[u8],
    array_counter: i32,
    start: EpicsTimestamp,
    acquisition_header: &str,
) {
    let header = match protocol::parse_mq_header(body) {
        Ok(h) => h,
        Err(e) => {
            log::error!("merlin: bad MQ1 frame header: {e}");
            return;
        }
    };
    let data = match image::decode_image(
        body,
        header.offset,
        header.x_size,
        header.y_size,
        header.pixel_depth,
        ctx.det_type.swaps_pixels(),
    ) {
        Ok(d) => d,
        Err(e) => {
            log::error!(
                "merlin: cannot decode a {}-bit {}x{} frame: {e}",
                header.pixel_depth,
                header.x_size,
                header.y_size
            );
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::Octet {
                        reason: ctx.ad_params.status_message,
                        addr: 0,
                        value: "Error: unsupported frame".into(),
                    }],
                )
                .await;
            return;
        }
    };

    let array = NDArray {
        unique_id: array_counter,
        timestamp: start,
        time_stamp: start.as_f64(),
        dims: vec![
            NDDimension::new(header.x_size),
            NDDimension::new(header.y_size),
        ],
        data_size: data.total_bytes(),
        pool_id: 0,
        data,
        attributes: build_attributes(&header.attrs, acquisition_header),
        codec: None,
    };
    publish(ctx, array).await;
}

async fn publish_profiles(
    ctx: &AcquisitionContext,
    body: &[u8],
    array_counter: i32,
    start: EpicsTimestamp,
    acquisition_header: &str,
) {
    let header = match protocol::parse_mq_header(body) {
        Ok(h) => h,
        Err(e) => {
            log::error!("merlin: bad PR1 frame header: {e}");
            return;
        }
    };
    // The device echoes the PROFILES mask it acquired with; we can only lay
    // out a frame that carries both profiles and the sum.
    let expected = MPXPROFILES_XPROFILE | MPXPROFILES_YPROFILE | MPXPROFILES_SUM;
    if header.profile_select != expected {
        log::error!(
            "merlin: unsupported PROFILES mode {}",
            header.profile_select
        );
        return;
    }

    let profiles = match image::decode_profiles(
        body,
        header.offset,
        header.x_size,
        header.y_size,
        ctx.det_type.swaps_pixels(),
    ) {
        Ok(p) => p,
        Err(e) => {
            log::error!("merlin: cannot decode a profile frame: {e}");
            return;
        }
    };

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::Int32Array {
                    reason: ctx.params.profile_x,
                    addr: 0,
                    value: profiles.x,
                },
                ParamSetValue::Int32Array {
                    reason: ctx.params.profile_y,
                    addr: 0,
                    value: profiles.y,
                },
            ],
        )
        .await;

    let stride = header.x_size.max(header.y_size);
    let NDDataBuffer::U32(_) = &profiles.image else {
        unreachable!("decode_profiles always returns U32")
    };
    let array = NDArray {
        unique_id: array_counter,
        timestamp: start,
        time_stamp: start.as_f64(),
        dims: vec![NDDimension::new(stride), NDDimension::new(2)],
        data_size: profiles.image.total_bytes(),
        pool_id: 0,
        data: profiles.image,
        attributes: build_attributes(&header.attrs, acquisition_header),
        codec: None,
    };
    publish(ctx, array).await;
}

/// Update the NDArray metadata parameters and hand the array to the plugins.
async fn publish(ctx: &AcquisitionContext, array: NDArray) {
    let base = &ctx.ad_params.base;
    let ts = array.timestamp;
    let size_x = array.dims[0].size as i32;
    let size_y = array.dims[1].size as i32;
    let data_type = array.data.data_type();
    let num_elements: i64 = array.dims.iter().map(|d| d.size as i64).product();
    let array_size = num_elements
        .saturating_mul(data_type.element_size() as i64)
        .min(i32::MAX as i64) as i32;

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::Int32 {
                    reason: base.array_counter,
                    addr: 0,
                    value: array.unique_id,
                },
                ParamSetValue::Float64 {
                    reason: base.timestamp_rbv,
                    addr: 0,
                    value: ts.as_f64(),
                },
                ParamSetValue::Int32 {
                    reason: base.epics_ts_sec,
                    addr: 0,
                    value: ts.sec as i32,
                },
                ParamSetValue::Int32 {
                    reason: base.epics_ts_nsec,
                    addr: 0,
                    value: ts.nsec as i32,
                },
                ParamSetValue::Int32 {
                    reason: base.array_size_x,
                    addr: 0,
                    value: size_x,
                },
                ParamSetValue::Int32 {
                    reason: base.array_size_y,
                    addr: 0,
                    value: size_y,
                },
                ParamSetValue::Int32 {
                    reason: base.array_size,
                    addr: 0,
                    value: array_size,
                },
                ParamSetValue::Int32 {
                    reason: base.n_dimensions,
                    addr: 0,
                    value: 2,
                },
                ParamSetValue::Int32 {
                    reason: base.color_mode,
                    addr: 0,
                    value: NDColorMode::Mono as i32,
                },
                ParamSetValue::Int32 {
                    reason: base.data_type,
                    addr: 0,
                    value: data_type as u8 as i32,
                },
            ],
        )
        .await;

    ctx.output.publish(Arc::new(array)).await;
}
