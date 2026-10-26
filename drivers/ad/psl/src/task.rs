//! The acquisition task (C `PSLTask`).
//!
//! It polls the server for a new frame, reads it and hands it to the plugins.
//! Every parameter it touches goes through the port actor; the server socket is
//! taken through [`PslServer`], and never while the actor is being waited on
//! (see the invariant in [`crate::connection`]).

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDimension;
use epics_rs::ad_core::ndarray_pool::DriverPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::{PslServer, ServerError, ServerResult};
use crate::driver::{SharedState, TaskCommand};
use crate::image;
use crate::protocol::{self, ImageHeader};
use crate::types::{MAX_MESSAGE_SIZE, POLL_INTERVAL};

pub(crate) struct AcquisitionContext {
    pub server: PslServer,
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    pub pool: DriverPool,
    pub ad_params: ADBaseParams,
    pub shared: Arc<SharedState>,
    pub commands: rt::CommandReceiver<TaskCommand>,
}

pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PSLDetTask", move || task_loop(ctx))
}

/// Why an acquisition ended before it had taken every frame.
pub(crate) enum Abort {
    /// Acquire was set to 0.
    Stopped,
    /// The socket failed, or the server sent a frame that cannot be read.
    Error(String),
}

async fn task_loop(mut ctx: AcquisitionContext) {
    let sync = SyncIOHandle::from_handle(ctx.handle.clone(), 0, Duration::from_secs(30));

    loop {
        set_params(
            &ctx,
            vec![
                ParamSetValue::new(
                    ctx.ad_params.status,
                    0,
                    ParamValue::Int32(ADStatus::Idle as i32),
                ),
                ParamSetValue::new(
                    ctx.ad_params.status_message,
                    0,
                    ParamValue::Octet("Waiting for acquire command".as_bytes().into()),
                ),
            ],
        )
        .await;

        let Some(TaskCommand::Start) = ctx.commands.recv().await else {
            return;
        };
        // The task is the sole clearer of the stop flag, the actor its sole
        // setter: a stop that arrived while idle cannot abort the next run.
        ctx.shared.stop_requested.store(false, Ordering::Release);

        acquire(&ctx, &sync).await;
    }
}

async fn acquire(ctx: &AcquisitionContext, sync: &SyncIOHandle) {
    set_params(
        ctx,
        vec![
            ParamSetValue::new(ctx.ad_params.num_images_counter, 0, ParamValue::Int32(0)),
            ParamSetValue::new(
                ctx.ad_params.status,
                0,
                ParamValue::Int32(ADStatus::Acquire as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.status_message,
                0,
                ParamValue::Octet("Waiting for the detector".as_bytes().into()),
            ),
        ],
    )
    .await;

    let mut images_taken = 0;

    loop {
        // C read these once per frame, so a change during a Continuous
        // acquisition takes effect on the next frame.
        let image_mode =
            ImageMode::from_i32(sync.read_int32(ctx.ad_params.image_mode).unwrap_or(0));
        let num_images = sync
            .read_int32(ctx.ad_params.num_images)
            .unwrap_or(1)
            .max(1);
        let array_callbacks = sync
            .read_int32(ctx.ad_params.base.array_callbacks)
            .unwrap_or(1)
            != 0;

        // C only waited for the frame when ArrayCallbacks was on; with it off
        // the loop counted frames as fast as the CPU allowed and never touched
        // the detector. The wait is the acquisition; only the readout is
        // optional.
        if let Err(abort) = wait_for_new_data(ctx).await {
            finish(ctx, Some(abort)).await;
            return;
        }

        if array_callbacks {
            let unique_id = sync
                .read_int32(ctx.ad_params.base.array_counter)
                .unwrap_or(0);
            match read_frame(&ctx.server, &ctx.handle) {
                Ok((header, data)) => publish_frame(ctx, &header, data, unique_id).await,
                Err(abort) => {
                    finish(ctx, Some(abort)).await;
                    return;
                }
            }
        }

        let array_counter = sync
            .read_int32(ctx.ad_params.base.array_counter)
            .unwrap_or(0)
            + 1;
        images_taken += 1;
        set_params(
            ctx,
            vec![
                ParamSetValue::new(
                    ctx.ad_params.base.array_counter,
                    0,
                    ParamValue::Int32(array_counter),
                ),
                ParamSetValue::new(
                    ctx.ad_params.num_images_counter,
                    0,
                    ParamValue::Int32(images_taken),
                ),
            ],
        )
        .await;

        let done = match image_mode {
            ImageMode::Single => true,
            ImageMode::Multiple => images_taken >= num_images,
            ImageMode::Continuous => false,
        };
        if done {
            finish(ctx, None).await;
            return;
        }
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            finish(ctx, Some(Abort::Stopped)).await;
            return;
        }
    }
}

/// One `HasNewData` poll. C compares `fromServer_` to "True" whatever the
/// read's status (PSL.cpp:697-698), so a reply a timeout cut still counts.
pub(crate) fn has_new_data(server: &PslServer) -> ServerResult<bool> {
    let (reply, status) = server.exchange("HasNewData");
    if reply.is_some_and(|reply| reply.trim() == "True") {
        return Ok(true);
    }
    match status {
        Err(ServerError::Timeout) => Ok(false),
        status => status.map(|()| false),
    }
}

/// Poll `HasNewData` until the server has a frame (C's inner `while (1)` loop).
async fn wait_for_new_data(ctx: &AcquisitionContext) -> Result<(), Abort> {
    loop {
        match has_new_data(&ctx.server) {
            Ok(true) => return Ok(()),
            Ok(false) => {}
            Err(e) => return Err(Abort::Error(format!("HasNewData failed: {e}"))),
        }
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(Abort::Stopped);
        }
        rt::sleep(POLL_INTERVAL).await;
    }
}

/// Read one frame off the socket (C `getImage`).
///
/// The session is held for the whole read — header and every payload block —
/// so no other command can slip between them; it is dropped before the caller
/// goes back to the parameter library.
pub(crate) fn read_frame(
    server: &PslServer,
    handle: &PortHandle,
) -> Result<(ImageHeader, Vec<u8>), Abort> {
    let mut session = server.session();

    // C parses a cut read's bytes too. A failed write leaves HasNewData's
    // "True" in fromServer_, which C parses as a header: a C defect, not copied.
    let (first, _) = session
        .request_image()
        .map_err(|e| Abort::Error(format!("GetImage failed: {e}")))?;
    let header = protocol::parse_image_header(&first)
        .map_err(|e| Abort::Error(format!("cannot read the image header: {e}")))?;

    let mut payload = Vec::with_capacity(header.data_len);
    payload.extend_from_slice(&first[header.header_len..]);

    while payload.len() < header.data_len {
        let want = (header.data_len - payload.len()).min(MAX_MESSAGE_SIZE);
        let (block, status) = session.read_image_block(want);
        if let Err(e) = status {
            handle.trace().output_device(
                handle.port_name(),
                Some(0),
                0,
                TraceMask::ERROR,
                &format!(
                    "PSL:getImage: error reading image, status={}, dataLen={}, nCopied={}, \
                     maxRead={want}, nRead={}\n",
                    e.status(),
                    header.data_len,
                    payload.len(),
                    block.len()
                ),
            );
            return Err(Abort::Error(format!("error reading the image: {e}")));
        }
        // C added `nRead_` to its copy counter without checking it: a read that
        // returned nothing left the loop spinning forever on a half-written
        // image.
        if block.is_empty() {
            return Err(Abort::Error(format!(
                "the server stopped sending after {} of {} image bytes",
                payload.len(),
                header.data_len
            )));
        }
        payload.extend_from_slice(&block);
    }
    // C trusted the server's byte count while sizing the array from the
    // geometry; a server that sends more than it announced overran the buffer.
    payload.truncate(header.data_len);

    Ok((header, payload))
}

async fn publish_frame(
    ctx: &AcquisitionContext,
    header: &ImageHeader,
    payload: Vec<u8>,
    unique_id: i32,
) {
    if image::check_payload(header, &payload).is_err() {
        return;
    }

    // C reads the clock for timeStamp, then takes the port's moved stamp as
    // epicsTS (PSL.cpp:606-609).
    let now = EpicsTimestamp::now();
    let ts: EpicsTimestamp = ctx
        .handle
        .update_time_stamp()
        .await
        .unwrap_or_else(|_| std::time::SystemTime::now())
        .into();
    let dims: Vec<NDDimension> = header.dims().into_iter().map(NDDimension::new).collect();
    let mut attributes = epics_rs::ad_core::attributes::NDAttributeList::new();
    attributes.add(NDAttribute {
        name: "ColorMode".into(),
        description: "Color Mode".into(),
        source: NDAttrSource::Driver,
        value: NDAttrValue::Int32(header.color_mode as i32),
        source_impl: None,
    });

    // C dereferences a refused alloc unchecked (PSL.cpp:573); here the frame
    // is dropped after the pool's limit line.
    let Ok(mut array) = ctx.pool.alloc(dims, header.data_type) else {
        return;
    };
    let mut frame = array.fill();
    image::decode_into(&payload, frame.data_mut().view_mut());
    frame.set_unique_id(unique_id);
    frame.set_timestamp(ts);
    frame.set_time_stamp(now.as_f64());
    *frame.attributes_mut() = attributes;

    // C stores DataType and the sizes per frame, and no other array param
    // (PSL.cpp:555-577).
    set_params(
        ctx,
        vec![
            ParamSetValue::new(
                ctx.ad_params.base.array_size_x,
                0,
                ParamValue::Int32(header.width as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size_y,
                0,
                ParamValue::Int32(header.height as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size,
                0,
                ParamValue::Int32(array.data_size() as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.data_type,
                0,
                ParamValue::Int32(header.data_type as u8 as i32),
            ),
        ],
    )
    .await;

    ctx.output.publish(array).await;
}

/// The one exit path of an acquisition: Acquire cleared, whatever went wrong
/// reported.
async fn finish(ctx: &AcquisitionContext, abort: Option<Abort>) {
    let mut updates = vec![
        ParamSetValue::new(ctx.ad_params.acquire, 0, ParamValue::Int32(0)),
        ParamSetValue::new(
            ctx.ad_params.status,
            0,
            ParamValue::Int32(match abort {
                Some(Abort::Error(_)) => ADStatus::Error as i32,
                _ => ADStatus::Idle as i32,
            }),
        ),
    ];
    match &abort {
        None => {}
        Some(Abort::Stopped) => updates.push(ParamSetValue::new(
            ctx.ad_params.status_message,
            0,
            ParamValue::Octet("Acquisition aborted".as_bytes().into()),
        )),
        Some(Abort::Error(message)) => {
            updates.push(ParamSetValue::new(
                ctx.ad_params.status_message,
                0,
                ParamValue::Octet(message.clone().into_bytes().into()),
            ));
        }
    }
    set_params(ctx, updates).await;
}

async fn set_params(ctx: &AcquisitionContext, values: Vec<ParamSetValue>) {
    let _ = ctx.handle.set_params_and_notify(0, values).await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::create_psl_detector;
    use epics_rs::ad_core::color::NDColorMode;
    use epics_rs::ad_core::ndarray::{NDDataType, NDDataView};
    use epics_rs::ad_core::plugin::channel::{NDArrayOutput, ndarray_channel};
    use epics_rs::asyn::interfaces::InterfaceType;
    use epics_rs::asyn::registry::claim_port;
    use std::sync::atomic::AtomicBool;

    /// PSL::readImage stores DataType and the array sizes per frame and no
    /// other array param (PSL.cpp:555-577, :603-618).
    #[test]
    fn a_frame_stores_only_the_array_params_c_stores() {
        let server = || {
            SyncIOHandle::connect_port("PSL_FRAME_IP", 0, Duration::ZERO, InterfaceType::Octet).0
        };
        let det = create_psl_detector(&claim_port("PSL_FRAME").unwrap(), server(), 0).unwrap();
        let handle = det.port_handle().clone();
        let base = det.ad_params.base;
        let (sink, mut arrays) = ndarray_channel("PSL_FRAME_SINK", 4);
        let mut output = NDArrayOutput::new();
        output.add(sink);
        let (_tx, commands) = rt::command_channel::<TaskCommand>(4);
        let ctx = AcquisitionContext {
            server: PslServer::new(server()),
            handle: handle.clone(),
            output: ArrayPublisher::new(Arc::new(parking_lot::Mutex::new(output))),
            pool: DriverPool::standalone(0),
            ad_params: det.ad_params,
            shared: Arc::new(SharedState {
                stop_requested: AtomicBool::new(false),
            }),
            commands,
        };
        let header = ImageHeader {
            data_type: NDDataType::UInt16,
            color_mode: NDColorMode::Mono,
            width: 2,
            height: 1,
            data_len: 4,
            header_len: 0,
        };
        rt::run_thread(move || async move {
            publish_frame(&ctx, &header, vec![1, 0, 2, 0], 1).await;
        })
        .join()
        .unwrap();
        let array = arrays.blocking_recv().unwrap();
        std::mem::forget(det);
        assert!(
            matches!(array.data().view(), NDDataView::U16(p) if p == [1, 2]),
            "the published pixels are the payload's"
        );

        let int = |reason| handle.read_int32_blocking(reason, 0).unwrap();
        assert_eq!(int(base.array_size_x), 2);
        assert_eq!(int(base.array_size_y), 1);
        assert_eq!(int(base.array_size), 4);
        assert_eq!(int(base.data_type), NDDataType::UInt16 as u8 as i32);
        assert_eq!(int(base.n_dimensions), 0);
        assert_eq!(int(base.epics_ts_sec), 0);
        assert_eq!(int(base.epics_ts_nsec), 0);
        assert_eq!(
            handle.read_float64_blocking(base.timestamp_rbv, 0).unwrap(),
            0.0
        );
    }
}
