//! The two background tasks (C `merlinTask` and `merlinStatus`).
//!
//! The data task owns the data socket and never touches the command socket:
//! it waits for frames, decodes them and publishes NDArrays. The status task
//! only kicks the driver's startup sequence and refreshes the idle message.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::port_handle::{DriverHold, PortHandle};
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDimension};
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::{ChannelError, MpxConnection};
use crate::driver::{MerlinDetector, SharedState};
use crate::image;
use crate::params::MerlinParams;
use crate::protocol;
use crate::types::*;

/// Everything the data task needs.
pub(crate) struct AcquisitionContext {
    pub data: MpxConnection,
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
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
    let _ = sync.write_int32(init_param, 1);

    loop {
        rt::sleep(STATUS_POLL).await;
        let Ok(status) = sync.read_int32(ad.status) else {
            continue;
        };
        if status == ADStatus::Idle as i32 {
            let _ = handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ad.status_message,
                        0,
                        ParamValue::Octet("Waiting for acquire command".as_bytes().into()),
                    )],
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
                ctx.data.trace(&format!(
                    "merlinDetector:merlinTask: error in Labview data channel response, status={}\n",
                    e.code()
                ));
                if let Ok(mut held) = ctx.handle.hold_driver_blocking() {
                    set_status_message(
                        &mut held,
                        ctx.ad_params,
                        "Error in Labview data channel response",
                    );
                }
                rt::sleep(ERROR_BACKOFF).await;
                continue;
            }
        };

        handle_frame(&ctx, &body, &mut acquisition_header).await;
    }
}

/// The driver lock merlinTask holds through a frame's pass.
type Held = DriverHold<MerlinDetector>;

/// [`DriverHold::run`] on the port's param library; a stopped port yields
/// `R::default()`.
fn step<R: Default + Send + 'static>(
    held: &mut Held,
    f: impl FnOnce(&mut PortDriverBase) -> R + Send + 'static,
) -> R {
    held.run(move |d| f(&mut d.ad.port_base))
        .unwrap_or_default()
}

/// One frame of C `merlinTask`, under the driver lock C holds from the
/// read's end to the next read (merlinDetector.cpp:137-305). A data frame
/// that yields no NDArray ends the pass there, as C's `continue` does, so it
/// neither clears the software trigger nor completes the acquisition, and
/// posts nothing it stored.
pub(crate) async fn handle_frame(
    ctx: &AcquisitionContext,
    body: &[u8],
    acquisition_header: &mut String,
) {
    let Ok(mut held) = ctx.handle.hold_driver_blocking::<MerlinDetector>() else {
        return;
    };
    let start = EpicsTimestamp::now();
    let header = protocol::data_header(body);
    let (ad, params) = (ctx.ad_params, ctx.params);

    // The acquisition header is not an image, so it advances no counter.
    let counted = header != DataHeader::Acquisition;
    let (array_counter, array_callbacks) = step(&mut held, move |base| {
        let mut counter = base.get_int32_param(ad.base.array_counter, 0).unwrap_or(0);
        if counted {
            let images = base.get_int32_param(ad.num_images_counter, 0).unwrap_or(0) + 1;
            let _ = base.set_int32_param(ad.num_images_counter, 0, images);
            counter += 1;
            let _ = base.set_int32_param(ad.base.array_counter, 0, counter);
        }
        let callbacks = base
            .get_int32_param(ad.base.array_callbacks, 0)
            .unwrap_or(1);
        (counter, callbacks != 0)
    });
    if counted {
        let _ = ctx
            .shared
            .images_remaining
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                if n > 0 { Some(n - 1) } else { None }
            });
    }

    if array_callbacks {
        let published = match header {
            DataHeader::Acquisition => {
                let text = String::from_utf8_lossy(body);
                let end = text.len().min(MPX_ACQUISITION_HEADER_LEN);
                *acquisition_header = text[..end].to_string();
                true
            }
            DataHeader::QuadData => {
                publish_image(
                    ctx,
                    &mut held,
                    body,
                    array_counter,
                    start,
                    acquisition_header,
                )
                .await
            }
            DataHeader::Profile => {
                publish_profiles(
                    ctx,
                    &mut held,
                    body,
                    array_counter,
                    start,
                    acquisition_header,
                )
                .await
            }
            DataHeader::Unknown => {
                trace(&ctx.handle, "Unknown header type 3\n");
                true
            }
        };
        if !published {
            return;
        }
    }

    let done = ctx.shared.images_remaining.load(Ordering::Acquire) == 0;
    step(&mut held, move |base| {
        // A software trigger is one-shot: clear it once its frame arrives.
        if base.get_int32_param(ad.trigger_mode, 0).unwrap_or(0)
            == TriggerMode::SoftwareTrigger as i32
        {
            let _ = base.set_int32_param(params.software_trigger, 0, 0);
        }
        if done {
            let _ = base.set_int32_param(ad.acquire, 0, 0);
            let _ = base.set_int32_param(ad.status, 0, ADStatus::Idle as i32);
        }
        let _ = base.call_param_callbacks(0);
    });
}

/// C prints these on `pasynUserSelf`, the detector port.
fn trace(handle: &PortHandle, msg: &str) {
    handle
        .trace()
        .output_device(handle.port_name(), Some(0), 0, TraceMask::ERROR, msg);
}

/// C `setStringParam(ADStatusMessage, ...)` without `callParamCallbacks`:
/// the pass's one post is at its end (merlinDetector.cpp:305), and a pass
/// that ends early posts nothing, so the message waits for the next driver
/// callback.
fn set_status_message(held: &mut Held, ad: ADBaseParams, msg: &'static str) {
    step(held, move |base| {
        let _ = base.set_string_param(ad.status_message, 0, msg);
    });
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

/// False when no NDArray went out (C's `pImage == NULL`).
async fn publish_image(
    ctx: &AcquisitionContext,
    held: &mut Held,
    body: &[u8],
    array_counter: i32,
    start: EpicsTimestamp,
    acquisition_header: &str,
) -> bool {
    let header = match protocol::parse_mq_header(body) {
        Ok(h) => h,
        Err(_) => return false,
    };
    if !matches!(header.pixel_depth, 8 | 16 | 32) {
        trace(
            &ctx.handle,
            &format!("Unsupported bit depth {}\n", header.pixel_depth),
        );
        set_status_message(held, ctx.ad_params, "Error: Unsupported bit depth");
        return false;
    }
    let data = match image::decode_image(
        body,
        header.offset,
        header.x_size,
        header.y_size,
        header.pixel_depth,
        ctx.det_type.swaps_pixels(),
    ) {
        Ok(d) => d,
        Err(_) => {
            set_status_message(held, ctx.ad_params, "Error: unsupported frame");
            return false;
        }
    };

    let Ok(mut array) = ctx.pool.alloc_with(
        vec![
            NDDimension::new(header.x_size),
            NDDimension::new(header.y_size),
        ],
        data,
    ) else {
        // merlinDetector.cpp:436-442 (copyToNDArray8, and its 16/32 twins).
        trace(
            &ctx.handle,
            &format!(
                "merlinDetector:copyToNDArray{}: unable to allocate NDArray from pool\n",
                header.pixel_depth
            ),
        );
        set_status_message(
            held,
            ctx.ad_params,
            "Error: run out of buffers in detector driver",
        );
        return false;
    };
    array.unique_id = array_counter;
    // C never calls updateTimeStamp, so epicsTS stays zero
    // (merlinDetector.cpp:255-258).
    array.time_stamp = start.as_f64();
    array.attributes = build_attributes(&header.attrs, acquisition_header);
    // C stores no array size, type or time-stamp param per frame; the
    // constructor's values stand (merlinDetector.cpp:1494-1497).
    ctx.output.publish(array).await;
    true
}

/// False when no NDArray went out (C's `pImage == NULL`).
async fn publish_profiles(
    ctx: &AcquisitionContext,
    held: &mut Held,
    body: &[u8],
    array_counter: i32,
    start: EpicsTimestamp,
    acquisition_header: &str,
) -> bool {
    let header = match protocol::parse_mq_header(body) {
        Ok(h) => h,
        Err(_) => return false,
    };
    // The device echoes the PROFILES mask it acquired with; we can only lay
    // out a frame that carries both profiles and the sum.
    let expected = MPXPROFILES_XPROFILE | MPXPROFILES_YPROFILE | MPXPROFILES_SUM;
    if header.profile_select != expected {
        trace(
            &ctx.handle,
            &format!(
                "merlinDetector:merlinTask: unsupported PROFILES mode {}\n",
                header.profile_select
            ),
        );
        return false;
    }

    let profiles = match image::decode_profiles(
        body,
        header.offset,
        header.x_size,
        header.y_size,
        ctx.det_type.swaps_pixels(),
    ) {
        Ok(p) => p,
        Err(_) => return false,
    };

    let stride = header.x_size.max(header.y_size);
    let NDDataBuffer::U32(_) = &profiles.image else {
        unreachable!("decode_profiles always returns U32")
    };
    // C fills the profile waveforms only once the array is allocated
    // (merlinDetector.cpp:384-391).
    let Ok(mut array) = ctx.pool.alloc_with(
        vec![NDDimension::new(stride), NDDimension::new(2)],
        profiles.image,
    ) else {
        ctx.data
            .trace("merlinDetector:copyProfileToNDArray32: unable to allocate NDArray from pool\n");
        set_status_message(
            held,
            ctx.ad_params,
            "Error: run out of buffers in detector driver",
        );
        return false;
    };

    // doCallbacksInt32Array, Y then X (merlinDetector.cpp:415-416).
    let (x, y) = (ctx.params.profile_x, ctx.params.profile_y);
    step(held, move |base| {
        let _ = base.do_callbacks_array(y, 0, ParamValue::Int32Array(profiles.y.into()));
        let _ = base.do_callbacks_array(x, 0, ParamValue::Int32Array(profiles.x.into()));
    });

    array.unique_id = array_counter;
    // C never calls updateTimeStamp, so epicsTS stays zero
    // (merlinDetector.cpp:255-258).
    array.time_stamp = start.as_f64();
    array.attributes = build_attributes(&header.attrs, acquisition_header);
    // C stores no array size, type or time-stamp param per frame; the
    // constructor's values stand (merlinDetector.cpp:1494-1497).
    ctx.output.publish(array).await;
    true
}
