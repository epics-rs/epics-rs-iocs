//! The acquisition task (C `PhotonIITask`).
//!
//! It owns nothing the actor owns: every parameter it reads or writes goes
//! through the port actor, and every p2util exchange goes through the p2util
//! port's actor.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime};

use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDimension;
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::{ChannelError, P2Util};
use crate::driver::{SharedState, TaskCommand};
use crate::params::PhotonIIParams;
use crate::protocol;
use crate::raw::{self, FileState};
use crate::types::*;

pub(crate) struct AcquisitionContext {
    pub p2: P2Util,
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    pub pool: Arc<NDArrayPool>,
    pub ad_params: ADBaseParams,
    pub params: PhotonIIParams,
    pub shared: Arc<SharedState>,
    pub commands: rt::CommandReceiver<TaskCommand>,
}

pub(crate) fn start_acquisition_task(ctx: AcquisitionContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PhotonIIDetTask", move || task_loop(ctx))
}

/// How the parameters a single acquisition needs looked when it started.
struct AcquireSetup {
    frame_type: FrameType,
    acquire_time: f64,
    /// How many frames p2util was asked for, and therefore how many frame
    /// messages and files this acquisition will see.
    count: i32,
    file_path: String,
    file_name: String,
    file_number: i32,
    array_callbacks: bool,
}

/// Why an acquisition ended before all of its frames arrived.
enum Abort {
    /// Acquire was set to 0.
    Stopped,
    /// p2util never announced the frame, or the file never appeared.
    Timeout(String),
    /// The socket or the file system failed.
    Error(String),
}

async fn task_loop(mut ctx: AcquisitionContext) {
    let sync = SyncIOHandle::from_handle(ctx.handle.clone(), 0, Duration::from_secs(10));

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

        // The only command the actor sends is Start; a closed channel means the
        // driver is gone.
        let Some(TaskCommand::Start) = ctx.commands.recv().await else {
            return;
        };
        // The task is the sole clearer of the stop flag; the actor is the sole
        // setter. A stop that arrived while idle cannot abort the next run.
        ctx.shared.stop_requested.store(false, Ordering::Release);

        acquire(&ctx, &sync).await;
    }
}

/// Read the parameters this acquisition runs with.
fn read_setup(ctx: &AcquisitionContext, sync: &SyncIOHandle) -> AcquireSetup {
    let ad = &ctx.ad_params;
    let frame_type_raw = sync.read_int32(ad.frame_type).unwrap_or(0);
    let frame_type = FrameType::from_i32(frame_type_raw).unwrap_or(FrameType::Normal);

    let image_mode = ImageMode::from_i32(sync.read_int32(ad.image_mode).unwrap_or(0));
    let mut num_images = sync.read_int32(ad.num_images).unwrap_or(1);
    if image_mode == ImageMode::Single {
        num_images = 1;
    }
    let num_darks = sync.read_int32(ctx.params.num_darks).unwrap_or(1);

    // C sent `--count numDarks` for a dark frame but then waited for
    // `numImages` frame messages, so a dark acquisition with NumDarks !=
    // NumImages either hung until the read timeout or left messages unread for
    // the next acquisition. The count that was requested is the count that is
    // awaited.
    let count = match frame_type {
        FrameType::Normal => num_images,
        FrameType::Dark | FrameType::Adc0 => num_darks,
    }
    .max(1);

    AcquireSetup {
        frame_type,
        acquire_time: sync.read_float64(ad.acquire_time).unwrap_or(0.0),
        count,
        file_path: read_string(ctx, ad.base.file_path),
        file_name: read_string(ctx, ad.base.file_name),
        file_number: sync.read_int32(ad.base.file_number).unwrap_or(0),
        array_callbacks: sync.read_int32(ad.base.array_callbacks).unwrap_or(1) != 0,
    }
}

async fn acquire(ctx: &AcquisitionContext, sync: &SyncIOHandle) {
    let setup = read_setup(ctx, sync);

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
                ParamValue::Octet("Starting exposure".as_bytes().into()),
            ),
        ],
    )
    .await;

    // Commands go through the actor so that StringToServer / StringFromServer
    // and the socket stay in one owner's hands.
    // writePhotonII prints its own failures.
    let _ = sync.write_octet(
        ctx.params.util,
        protocol::set_run_number(setup.file_number).as_bytes(),
    );
    let grab = protocol::grab(
        setup.frame_type,
        &setup.file_path,
        &setup.file_name,
        setup.count,
    );
    if let Err(e) = sync.write_octet(ctx.params.util, grab.as_bytes()) {
        finish(ctx, sync, Some(Abort::Error(format!("grab failed: {e}")))).await;
        return;
    }

    let start = SystemTime::now();
    let start_instant = Instant::now();
    let start_ts = EpicsTimestamp::now();
    let expected = Duration::from_secs_f64((setup.count as f64 * setup.acquire_time).max(0.0));

    // set_shutter is a no-op unless ShutterMode is EPICS, so the mode test C
    // spelled out at both ends lives in one place.
    let _ = sync.write_int32(ctx.params.shutter, 1);

    set_params(
        ctx,
        vec![ParamSetValue::new(
            ctx.ad_params.status_message,
            0,
            ParamValue::Octet("Waiting for Acquisition".as_bytes().into()),
        )],
    )
    .await;

    for _ in 0..setup.count {
        let file_name = match wait_for_frame(ctx, &setup, start_instant, expected).await {
            Ok(name) => name,
            Err(abort) => {
                finish(ctx, sync, Some(abort)).await;
                return;
            }
        };

        let array_counter = sync
            .read_int32(ctx.ad_params.base.array_counter)
            .unwrap_or(0)
            + 1;
        let images = sync
            .read_int32(ctx.ad_params.num_images_counter)
            .unwrap_or(0)
            + 1;
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
                    ParamValue::Int32(images),
                ),
            ],
        )
        .await;

        // A dark or ADC0 frame is a calibration measurement kept inside the
        // detector server; C published neither, and neither do we.
        if setup.array_callbacks
            && setup.frame_type == FrameType::Normal
            && let Err(abort) =
                publish_frame(ctx, &setup, &file_name, start, start_ts, array_counter).await
        {
            finish(ctx, sync, Some(abort)).await;
            return;
        }
    }

    finish(ctx, sync, None).await;
}

/// Poll p2util until it announces the next written frame.
async fn wait_for_frame(
    ctx: &AcquisitionContext,
    setup: &AcquireSetup,
    start: Instant,
    expected: Duration,
) -> Result<String, Abort> {
    let deadline = Duration::from_secs_f64(setup.acquire_time.max(0.0)) + FILE_READ_TIMEOUT;
    let poll_start = Instant::now();

    loop {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(Abort::Stopped);
        }

        let remaining = expected.saturating_sub(start.elapsed()).as_secs_f64();
        set_params(
            ctx,
            vec![ParamSetValue::new(
                ctx.ad_params.time_remaining,
                0,
                ParamValue::Float64(remaining),
            )],
        )
        .await;

        match ctx.p2.read(FILE_READ_DELAY) {
            Ok(line) if protocol::is_file_written(&line) => {
                return protocol::parse_file_written(&line)
                    .map(|s| s.to_string())
                    .map_err(|e| {
                        Abort::Error(format!("cannot read the file name p2util sent: {e}"))
                    });
            }
            // Any other chatter from p2util is progress output; keep waiting.
            Ok(_) => {}
            Err(ChannelError::Timeout) => {}
            Err(ChannelError::Transport(e)) => {
                let status = e.status() as i32;
                trace_error(ctx, &format!("PhotonII::readPhotonII, status={status}"));
                trace_error(
                    ctx,
                    &format!(
                        "PhotonII::PhotonIITask: error reading file written message status={status}"
                    ),
                );
                return Err(Abort::Error(format!("error reading from p2util: {e}")));
            }
        }

        if poll_start.elapsed() > deadline {
            trace_error(
                ctx,
                "PhotonII::PhotonIITask: timeout waiting for file written message",
            );
            return Err(Abort::Timeout(
                "timeout waiting for the file-written message".into(),
            ));
        }
        rt::sleep(FILE_READ_DELAY).await;
    }
}

/// Wait for the frame file to be complete, read it and hand it to the plugins.
async fn publish_frame(
    ctx: &AcquisitionContext,
    setup: &AcquireSetup,
    file_name: &str,
    start: SystemTime,
    start_ts: EpicsTimestamp,
    array_counter: i32,
) -> Result<(), Abort> {
    set_params(
        ctx,
        vec![
            ParamSetValue::new(
                ctx.ad_params.status_message,
                0,
                ParamValue::Octet(format!("Reading from File {file_name}").into_bytes().into()),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.full_file_name,
                0,
                ParamValue::Octet(file_name.as_bytes().into()),
            ),
        ],
    )
    .await;

    let path = Path::new(file_name);
    let expected = raw::frame_bytes();
    let deadline = Duration::from_secs_f64(setup.acquire_time.max(0.0)) + FILE_READ_TIMEOUT;
    let poll_start = Instant::now();
    // C's task prints this after every readRaw failure, a stop included.
    let read_raw_failed = |abort: Abort| {
        trace_error(
            ctx,
            &format!(
                "PhotonII::PhotonIITask, error calling readRaw(), status={}",
                AsynStatus::Error as i32
            ),
        );
        abort
    };
    let mut file_exists = false;

    loop {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(read_raw_failed(Abort::Stopped));
        }
        match raw::check_raw_file(path, start, expected) {
            FileState::Ready => break,
            FileState::Missing => {}
            FileState::Stale | FileState::Incomplete { .. } => file_exists = true,
        }
        if poll_start.elapsed() > deadline {
            trace_error(
                ctx,
                &format!("PhotonII::readRaw timeout waiting for file to be created {file_name}"),
            );
            if file_exists {
                trace_error(ctx, "  file exists but is either old or the wrong size");
            }
            return Err(read_raw_failed(Abort::Timeout(format!(
                "timeout waiting for the frame file {file_name}"
            ))));
        }
        rt::sleep(FILE_READ_DELAY).await;
    }

    let bytes = raw::read_raw_file(path)
        .map_err(|e| read_raw_failed(Abort::Error(format!("cannot read {file_name}: {e}"))))?;
    let data = raw::decode_raw(&bytes)
        .map_err(|e| read_raw_failed(Abort::Error(format!("cannot decode {file_name}: {e}"))))?;

    let ts = EpicsTimestamp::now();
    let data_type = data.data_type();
    // C dereferences a refused alloc unchecked (PhotonII.cpp:416); here the frame
    // is dropped after the pool's limit line.
    let Ok(mut array) = ctx.pool.wrap(
        vec![NDDimension::new(PII_SIZE_X), NDDimension::new(PII_SIZE_Y)],
        data,
    ) else {
        return Ok(());
    };
    array.unique_id = array_counter;
    array.timestamp = ts;
    // C stamped every frame of an acquisition with the acquisition's start
    // time, not the frame's; keep that, it is the exposure start.
    array.time_stamp = start_ts.as_f64();

    set_params(
        ctx,
        vec![
            ParamSetValue::new(
                ctx.ad_params.base.timestamp_rbv,
                0,
                ParamValue::Float64(ts.as_f64()),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.epics_ts_sec,
                0,
                ParamValue::Int32(ts.sec as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.epics_ts_nsec,
                0,
                ParamValue::Int32(ts.nsec as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size_x,
                0,
                ParamValue::Int32(PII_SIZE_X as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size_y,
                0,
                ParamValue::Int32(PII_SIZE_Y as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size,
                0,
                ParamValue::Int32(raw::frame_bytes() as i32),
            ),
            ParamSetValue::new(ctx.ad_params.base.n_dimensions, 0, ParamValue::Int32(2)),
            ParamSetValue::new(
                ctx.ad_params.base.color_mode,
                0,
                ParamValue::Int32(NDColorMode::Mono as i32),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.data_type,
                0,
                ParamValue::Int32(data_type as u8 as i32),
            ),
        ],
    )
    .await;

    ctx.output.publish(Arc::new(array)).await;
    Ok(())
}

/// The one exit path of an acquisition: shutter closed, Acquire cleared,
/// TimeRemaining zeroed, whatever went wrong reported.
async fn finish(ctx: &AcquisitionContext, sync: &SyncIOHandle, abort: Option<Abort>) {
    let _ = sync.write_int32(ctx.params.shutter, 0);

    let message = match &abort {
        None => None,
        Some(Abort::Stopped) => Some("Acquisition aborted".to_string()),
        Some(Abort::Timeout(m)) | Some(Abort::Error(m)) => Some(m.clone()),
    };

    let mut updates = vec![
        ParamSetValue::new(ctx.ad_params.acquire, 0, ParamValue::Int32(0)),
        ParamSetValue::new(ctx.ad_params.time_remaining, 0, ParamValue::Float64(0.0)),
        ParamSetValue::new(
            ctx.ad_params.status,
            0,
            ParamValue::Int32(match abort {
                Some(Abort::Error(_)) | Some(Abort::Timeout(_)) => ADStatus::Error as i32,
                _ => ADStatus::Idle as i32,
            }),
        ),
    ];
    if let Some(m) = message {
        updates.push(ParamSetValue::new(
            ctx.ad_params.status_message,
            0,
            ParamValue::Octet(m.into_bytes().into()),
        ));
    }
    set_params(ctx, updates).await;
}

/// C `asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, ...)`.
fn trace_error(ctx: &AcquisitionContext, message: &str) {
    ctx.handle
        .trace()
        .output(ctx.handle.port_name(), TraceMask::ERROR, message);
}

async fn set_params(ctx: &AcquisitionContext, values: Vec<ParamSetValue>) {
    let _ = ctx.handle.set_params_and_notify(0, values).await;
}

fn read_string(ctx: &AcquisitionContext, reason: usize) -> String {
    match ctx
        .handle
        .get_string_param_blocking(reason, 0, MAX_MESSAGE_SIZE)
    {
        Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
        Err(_) => String::new(),
    }
}
