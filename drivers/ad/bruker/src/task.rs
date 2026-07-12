//! The two background threads: the acquisition task (C `BISTask`) and the
//! status task (C `statusTask`).
//!
//! Neither of them touches a parameter or a socket itself: both reach the
//! detector port through its actor. Each runs on a current-thread runtime of
//! its own, so every port call here is an `await` on the asynchronous
//! `PortHandle` API — the blocking API (`SyncIOHandle`, `submit_blocking`) uses
//! `block_in_place`, which panics outside a multi-threaded runtime.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender, TryRecvError};
use std::time::{Duration, Instant, SystemTime};

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::{ParamSetValue, RequestOp};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::driver::{ADStatus, ImageMode, ShutterMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::driver::{SharedState, TaskCommand};
use crate::params::BrukerParams;
use crate::protocol;
use crate::sfrm;
use crate::types::*;

pub(crate) struct AcquisitionContext {
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    #[allow(dead_code)] // held so plugins can be back-pressured on the queue
    pub queued: Arc<QueuedArrayCounter>,
    pub ad_params: ADBaseParams,
    pub params: BrukerParams,
    pub shared: Arc<SharedState>,
    pub commands: rt::CommandReceiver<TaskCommand>,
    /// BIS has finished processing a frame (C's `readoutEventId`).
    pub readout: Receiver<()>,
}

pub(crate) struct StatusContext {
    /// The `drvAsynIPPort` BIS broadcasts its status on.
    pub status: PortHandle,
    pub handle: PortHandle,
    pub ad_params: ADBaseParams,
    pub params: BrukerParams,
    pub readout: SyncSender<()>,
}

pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("BISDetTask", move || acquisition_loop(ctx))
}

pub(crate) fn start_status_task(ctx: StatusContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("BISStatusTask", move || status_loop(ctx))
}

/// Why an acquisition ended before it had taken every frame.
enum Abort {
    /// Acquire was set to 0.
    Stopped,
    /// BIS never finished, or the frame file cannot be read.
    Error(String),
}

/// What this exposure is, read once it has been started (C's `getIntegerParam`
/// block at the top of `BISTask`).
struct Exposure {
    frame_type: FrameType,
    acquire_time: f64,
    shutter_mode: Option<ShutterMode>,
    sfrm_timeout: f64,
    file_name: String,
}

async fn acquisition_loop(mut ctx: AcquisitionContext) {
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

        acquire(&ctx).await;
    }
}

async fn acquire(ctx: &AcquisitionContext) {
    set_params(
        ctx,
        vec![ParamSetValue::Int32 {
            reason: ctx.ad_params.num_images_counter,
            addr: 0,
            value: 0,
        }],
    )
    .await;

    let mut images_taken = 0;

    loop {
        let exposure = match expose(ctx).await {
            Ok(exposure) => exposure,
            Err(abort) => return finish(ctx, Some(abort)).await,
        };

        let array_counter = read_i32(ctx, ctx.ad_params.base.array_counter).await + 1;
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

        let array_callbacks = ctx
            .handle
            .read_int32(ctx.ad_params.base.array_callbacks, 0)
            .await
            .unwrap_or(1)
            != 0;
        // A dark is BIS's business: it writes no frame file for the driver.
        if array_callbacks
            && exposure.frame_type != FrameType::Dark
            && let Err(abort) = publish_frame(ctx, &exposure, array_counter).await
        {
            return finish(ctx, Some(abort)).await;
        }

        // C read these once per frame, so a change during a Continuous
        // acquisition takes effect on the next frame.
        let image_mode = ImageMode::from_i32(read_i32(ctx, ctx.ad_params.image_mode).await);
        let num_images = read_i32(ctx, ctx.ad_params.num_images).await.max(1);
        let done = match image_mode {
            ImageMode::Single => true,
            ImageMode::Multiple => images_taken >= num_images,
            ImageMode::Continuous => false,
        };
        if done {
            return finish(ctx, None).await;
        }
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return finish(ctx, Some(Abort::Stopped)).await;
        }
    }
}

/// Start the scan, count the exposure down, and wait for BIS to say it has
/// processed the frame.
async fn expose(ctx: &AcquisitionContext) -> Result<Exposure, Abort> {
    // A frame BIS finished before this exposure started is not this exposure's.
    while ctx.readout.try_recv() != Err(TryRecvError::Empty) {}

    // The actor names the file and sends the scan command: it owns both the
    // parameters the command is built from and the socket it goes out on.
    ctx.handle
        .write_int32(ctx.params.start_scan, 0, 1)
        .await
        .map_err(|e| Abort::Error(format!("cannot start the scan: {e}")))?;

    let exposure = Exposure {
        frame_type: FrameType::from_i32(read_i32(ctx, ctx.ad_params.frame_type).await)
            .unwrap_or(FrameType::Normal),
        acquire_time: read_f64(ctx, ctx.ad_params.acquire_time).await,
        shutter_mode: ShutterMode::from_i32(read_i32(ctx, ctx.ad_params.shutter_mode).await),
        sfrm_timeout: read_f64(ctx, ctx.params.sfrm_timeout).await,
        file_name: read_string(ctx, ctx.ad_params.base.full_file_name).await,
    };

    set_params(
        ctx,
        vec![ParamSetValue::Octet {
            reason: ctx.ad_params.status_message,
            addr: 0,
            value: "Waiting for Acquisition".into(),
        }],
    )
    .await;

    // BIS drives the shutter itself when it is wired to the detector; an EPICS
    // shutter is ours to open.
    let epics_shutter = exposure.shutter_mode == Some(ShutterMode::EpicsOnly);
    if epics_shutter {
        let _ = ctx.handle.write_int32(ctx.params.epics_shutter, 0, 1).await;
    }

    let start = Instant::now();
    let exposure_time = Duration::from_secs_f64(exposure.acquire_time.max(0.0));
    let mut stopped = false;
    while start.elapsed() < exposure_time {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            stopped = true;
            break;
        }
        let remaining = (exposure_time - start.elapsed()).as_secs_f64();
        set_params(
            ctx,
            vec![ParamSetValue::Float64 {
                reason: ctx.ad_params.time_remaining,
                addr: 0,
                value: remaining,
            }],
        )
        .await;
        rt::sleep(BIS_POLL_DELAY).await;
    }

    if epics_shutter {
        let _ = ctx.handle.write_int32(ctx.params.epics_shutter, 0, 0).await;
    }
    set_params(
        ctx,
        vec![ParamSetValue::Float64 {
            reason: ctx.ad_params.time_remaining,
            addr: 0,
            value: 0.0,
        }],
    )
    .await;

    // C could not tell a Stop from the exposure timer's expiry — both signalled
    // the same event — so a Stop went on to wait for the readout and read the
    // frame file as if the exposure had run its course. A Stop ends the
    // acquisition here. BIS is not told to abort: no abort command appears
    // anywhere in the driver it came from.
    if stopped {
        return Err(Abort::Stopped);
    }

    // Blocking this thread is safe: the acquisition task is the only thing on
    // its runtime, and it has nothing else to do until BIS answers.
    match ctx.readout.recv_timeout(READOUT_TIMEOUT) {
        Ok(()) => Ok(exposure),
        Err(RecvTimeoutError::Timeout) => Err(Abort::Error(
            "timeout waiting for the readout to complete".into(),
        )),
        Err(RecvTimeoutError::Disconnected) => {
            Err(Abort::Error("the status task is not running".into()))
        }
    }
}

/// Wait for the frame file, read it and hand it to the plugins (C `readSFRM`
/// plus the `doCallbacksGenericPointer` that followed it).
async fn publish_frame(
    ctx: &AcquisitionContext,
    exposure: &Exposure,
    unique_id: i32,
) -> Result<(), Abort> {
    set_params(
        ctx,
        vec![ParamSetValue::Octet {
            reason: ctx.ad_params.status_message,
            addr: 0,
            value: format!("Reading from File {}", exposure.file_name),
        }],
    )
    .await;

    let deadline = Duration::from_secs_f64(exposure.acquire_time.max(0.0))
        + Duration::from_secs_f64(exposure.sfrm_timeout.max(0.0));
    let path = Path::new(&exposure.file_name);
    let written_after = SystemTime::now() - CLOCK_SKEW_ALLOWANCE;
    wait_for_file(ctx, path, written_after, deadline).await?;

    let bytes = std::fs::read(path)
        .map_err(|e| Abort::Error(format!("cannot read {}: {e}", path.display())))?;
    let image = sfrm::decode(&bytes)
        .map_err(|e| Abort::Error(format!("cannot read {}: {e}", path.display())))?;

    // C sized the array from ADSizeX/ADSizeY — the geometry BIS reported on the
    // status socket — and then let readSFRM write NROWS*NCOLS pixels into it: a
    // frame file bigger than the reported geometry ran off the end of the
    // buffer. The array is the size of the frame that was actually read.
    let ts = EpicsTimestamp::now();
    let data_size = image.data.len() * 4;
    let array = NDArray {
        unique_id,
        timestamp: ts,
        time_stamp: ts.as_f64(),
        dims: vec![NDDimension::new(image.cols), NDDimension::new(image.rows)],
        data_size,
        pool_id: 0,
        data: NDDataBuffer::U32(image.data),
        attributes: epics_rs::ad_core::attributes::NDAttributeList::new(),
        codec: None,
    };

    set_params(
        ctx,
        vec![
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_x,
                addr: 0,
                value: image.cols as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_y,
                addr: 0,
                value: image.rows as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size,
                addr: 0,
                value: data_size as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.n_dimensions,
                addr: 0,
                value: 2,
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
    Ok(())
}

/// Wait until the frame file is there and is this exposure's, not the last
/// one's (C's `stat` loop, which allowed the file server's clock to be up to
/// ten seconds behind ours).
async fn wait_for_file(
    ctx: &AcquisitionContext,
    path: &Path,
    written_after: SystemTime,
    deadline: Duration,
) -> Result<(), Abort> {
    let start = Instant::now();
    let mut stale = false;

    loop {
        match std::fs::metadata(path).and_then(|m| m.modified()) {
            Ok(modified) if modified >= written_after => return Ok(()),
            Ok(_) => stale = true,
            Err(_) => {}
        }
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(Abort::Stopped);
        }
        if start.elapsed() > deadline {
            return Err(Abort::Error(if stale {
                format!(
                    "{} is more than {} seconds old: the clocks may not agree",
                    path.display(),
                    CLOCK_SKEW_ALLOWANCE.as_secs()
                )
            } else {
                format!("timeout waiting for {} to be written", path.display())
            }));
        }
        rt::sleep(FILE_READ_DELAY).await;
    }
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
            log::error!("bruker: {message}");
            updates.push(ParamSetValue::Octet {
                reason: ctx.ad_params.status_message,
                addr: 0,
                value: message.clone(),
            });
        }
    }
    set_params(ctx, updates).await;
}

/// Read what BIS broadcasts and publish it (C `statusTask`).
async fn status_loop(ctx: StatusContext) {
    loop {
        let message = match read_status_message(&ctx).await {
            Ok(message) => message,
            // A quiet detector is not an error: BIS speaks when it has
            // something to say.
            Err(_) => continue,
        };
        if message.is_empty() {
            continue;
        }

        let report = protocol::parse_status(&message);
        let mut updates = vec![ParamSetValue::Octet {
            reason: ctx.params.status,
            addr: 0,
            value: message,
        }];
        if let Some(temperature) = report.temperature {
            updates.push(ParamSetValue::Float64 {
                reason: ctx.ad_params.temperature,
                addr: 0,
                value: temperature,
            });
        }
        if let Some(frame_size) = report.frame_size {
            // The detector is square.
            updates.push(ParamSetValue::Int32 {
                reason: ctx.ad_params.size_x,
                addr: 0,
                value: frame_size,
            });
            updates.push(ParamSetValue::Int32 {
                reason: ctx.ad_params.size_y,
                addr: 0,
                value: frame_size,
            });
        }
        if let Some(open) = report.shutter_open {
            updates.push(ParamSetValue::Int32 {
                reason: ctx.ad_params.shutter_status,
                addr: 0,
                value: open,
            });
        }
        let _ = ctx.handle.set_params_and_notify(0, updates).await;

        let acquiring = ctx
            .handle
            .read_int32(ctx.ad_params.acquire, 0)
            .await
            .unwrap_or(0)
            != 0;
        if report.processing_done && acquiring {
            // The acquisition task takes this when it is done exposing; if it
            // is not waiting yet, the slot holds it until it is.
            let _ = ctx.readout.try_send(());
        }
    }
}

async fn read_status_message(ctx: &StatusContext) -> AsynResult<String> {
    let user = AsynUser::new(0)
        .with_addr(0)
        .with_timeout(STATUS_READ_TIMEOUT);
    let op = RequestOp::OctetRead {
        buf_size: MAX_MESSAGE_SIZE,
    };
    let result = ctx.status.submit_async(op, user).await?;
    let bytes = result.data.unwrap_or_default();
    Ok(String::from_utf8_lossy(&bytes).trim_end().to_string())
}

async fn read_i32(ctx: &AcquisitionContext, reason: usize) -> i32 {
    ctx.handle.read_int32(reason, 0).await.unwrap_or(0)
}

async fn read_f64(ctx: &AcquisitionContext, reason: usize) -> f64 {
    ctx.handle.read_float64(reason, 0).await.unwrap_or(0.0)
}

async fn read_string(ctx: &AcquisitionContext, reason: usize) -> String {
    ctx.handle
        .read_octet(reason, 0, MAX_FILENAME_LEN)
        .await
        .map(|bytes| {
            String::from_utf8_lossy(&bytes)
                .trim_end_matches('\0')
                .to_string()
        })
        .unwrap_or_default()
}

async fn set_params(ctx: &AcquisitionContext, values: Vec<ParamSetValue>) {
    let _ = ctx.handle.set_params_and_notify(0, values).await;
}
