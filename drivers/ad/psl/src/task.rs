//! The acquisition task (C `PSLTask`).
//!
//! It polls the server for a new frame, reads it and hands it to the plugins.
//! Every parameter it touches goes through the port actor; the server socket is
//! taken through [`PslServer`], and never while the actor is being waited on
//! (see the invariant in [`crate::connection`]).

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::{PslServer, ServerError};
use crate::driver::{SharedState, TaskCommand};
use crate::image;
use crate::protocol::{self, ImageHeader};
use crate::types::{MAX_MESSAGE_SIZE, POLL_INTERVAL};

pub(crate) struct AcquisitionContext {
    pub server: PslServer,
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    #[allow(dead_code)] // held so plugins can be back-pressured on the queue
    pub queued: Arc<QueuedArrayCounter>,
    pub ad_params: ADBaseParams,
    pub shared: Arc<SharedState>,
    pub commands: rt::CommandReceiver<TaskCommand>,
}

pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PSLDetTask", move || task_loop(ctx))
}

/// Why an acquisition ended before it had taken every frame.
enum Abort {
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
                ParamSetValue::Int32 {
                    reason: ctx.ad_params.status,
                    addr: 0,
                    value: ADStatus::Idle as i32,
                },
                ParamSetValue::Octet {
                    reason: ctx.ad_params.status_message,
                    addr: 0,
                    value: "Waiting for acquire command".into(),
                },
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
            ParamSetValue::Int32 {
                reason: ctx.ad_params.num_images_counter,
                addr: 0,
                value: 0,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.status,
                addr: 0,
                value: ADStatus::Acquire as i32,
            },
            ParamSetValue::Octet {
                reason: ctx.ad_params.status_message,
                addr: 0,
                value: "Waiting for the detector".into(),
            },
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
            match read_frame(ctx) {
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
                ParamSetValue::Int32 {
                    reason: ctx.ad_params.base.array_counter,
                    addr: 0,
                    value: array_counter,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad_params.num_images_counter,
                    addr: 0,
                    value: images_taken,
                },
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

/// Poll `HasNewData` until the server has a frame (C's inner `while (1)` loop).
async fn wait_for_new_data(ctx: &AcquisitionContext) -> Result<(), Abort> {
    loop {
        match ctx.server.command("HasNewData") {
            Ok(reply) if reply.trim() == "True" => return Ok(()),
            Ok(_) => {}
            Err(ServerError::Timeout) => {}
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
fn read_frame(ctx: &AcquisitionContext) -> Result<(ImageHeader, Vec<u8>), Abort> {
    let mut session = ctx.server.session();

    let first = session
        .request_image()
        .map_err(|e| Abort::Error(format!("GetImage failed: {e}")))?;
    let header = protocol::parse_image_header(&first)
        .map_err(|e| Abort::Error(format!("cannot read the image header: {e}")))?;

    let mut payload = Vec::with_capacity(header.data_len);
    payload.extend_from_slice(&first[header.header_len..]);

    while payload.len() < header.data_len {
        let want = (header.data_len - payload.len()).min(MAX_MESSAGE_SIZE);
        let block = session
            .read_image_block(want)
            .map_err(|e| Abort::Error(format!("error reading the image: {e}")))?;
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
    let data = match image::decode_payload(header, &payload) {
        Ok(data) => data,
        Err(e) => {
            log::error!("psl: cannot decode the frame: {e}");
            return;
        }
    };

    let ts = EpicsTimestamp::now();
    let dims: Vec<NDDimension> = header.dims().into_iter().map(NDDimension::new).collect();
    let n_dims = dims.len();
    let mut attributes = epics_rs::ad_core::attributes::NDAttributeList::new();
    attributes.add(NDAttribute {
        name: "ColorMode".into(),
        description: "Color Mode".into(),
        source: NDAttrSource::Driver,
        value: NDAttrValue::Int32(header.color_mode as i32),
        source_impl: None,
    });

    let array = NDArray {
        unique_id,
        timestamp: ts,
        time_stamp: ts.as_f64(),
        dims,
        data_size: data.total_bytes(),
        pool_id: 0,
        data,
        attributes,
        codec: None,
    };

    set_params(
        ctx,
        vec![
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_x,
                addr: 0,
                value: header.width as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_y,
                addr: 0,
                value: header.height as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size,
                addr: 0,
                value: array.data_size as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.n_dimensions,
                addr: 0,
                value: n_dims as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.data_type,
                addr: 0,
                value: header.data_type as u8 as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.color_mode,
                addr: 0,
                value: header.color_mode as i32,
            },
            ParamSetValue::Float64 {
                reason: ctx.ad_params.base.timestamp_rbv,
                addr: 0,
                value: ts.as_f64(),
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.epics_ts_sec,
                addr: 0,
                value: ts.sec as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.epics_ts_nsec,
                addr: 0,
                value: ts.nsec as i32,
            },
        ],
    )
    .await;

    ctx.output.publish(Arc::new(array)).await;
}

/// The one exit path of an acquisition: Acquire cleared, whatever went wrong
/// reported.
async fn finish(ctx: &AcquisitionContext, abort: Option<Abort>) {
    let mut updates = vec![
        ParamSetValue::Int32 {
            reason: ctx.ad_params.acquire,
            addr: 0,
            value: 0,
        },
        ParamSetValue::Int32 {
            reason: ctx.ad_params.status,
            addr: 0,
            value: match abort {
                Some(Abort::Error(_)) => ADStatus::Error as i32,
                _ => ADStatus::Idle as i32,
            },
        },
    ];
    match &abort {
        None => {}
        Some(Abort::Stopped) => updates.push(ParamSetValue::Octet {
            reason: ctx.ad_params.status_message,
            addr: 0,
            value: "Acquisition aborted".into(),
        }),
        Some(Abort::Error(message)) => {
            log::error!("psl: {message}");
            updates.push(ParamSetValue::Octet {
                reason: ctx.ad_params.status_message,
                addr: 0,
                value: message.clone(),
            });
        }
    }
    set_params(ctx, updates).await;
}

async fn set_params(ctx: &AcquisitionContext, values: Vec<ParamSetValue>) {
    let _ = ctx.handle.set_params_and_notify(0, values).await;
}
