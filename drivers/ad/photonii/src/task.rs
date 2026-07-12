//! The acquisition task (C `PhotonIITask`).
//!
//! It owns nothing the actor owns: every parameter it reads or writes goes
//! through the port actor, and every p2util exchange goes through the p2util
//! port's actor.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime};

use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::color::NDColorMode;
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
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
    #[allow(dead_code)] // held so plugins can be back-pressured on the queue
    pub queued: Arc<QueuedArrayCounter>,
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
    let frame_type = FrameType::from_i32(frame_type_raw).unwrap_or_else(|| {
        log::error!("photonii: unknown FrameType {frame_type_raw}, acquiring a normal frame");
        FrameType::Normal
    });

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
        file_path: read_string(sync, ad.base.file_path),
        file_name: read_string(sync, ad.base.file_name),
        file_number: sync.read_int32(ad.base.file_number).unwrap_or(0),
        array_callbacks: sync.read_int32(ad.base.array_callbacks).unwrap_or(1) != 0,
    }
}

async fn acquire(ctx: &AcquisitionContext, sync: &SyncIOHandle) {
    let setup = read_setup(ctx, sync);

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
                value: "Starting exposure".into(),
            },
        ],
    )
    .await;

    // Commands go through the actor so that StringToServer / StringFromServer
    // and the socket stay in one owner's hands.
    if let Err(e) = sync.write_octet(
        ctx.params.util,
        protocol::set_run_number(setup.file_number).as_bytes(),
    ) {
        log::error!("photonii: could not set the run number: {e}");
    }
    let grab = protocol::grab(
        setup.frame_type,
        &setup.file_path,
        &setup.file_name,
        setup.count,
    );
    if let Err(e) = sync.write_octet(ctx.params.util, grab.as_bytes()) {
        log::error!("photonii: could not start the acquisition: {e}");
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
        vec![ParamSetValue::Octet {
            reason: ctx.ad_params.status_message,
            addr: 0,
            value: "Waiting for Acquisition".into(),
        }],
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
                ParamSetValue::Int32 {
                    reason: ctx.ad_params.base.array_counter,
                    addr: 0,
                    value: array_counter,
                },
                ParamSetValue::Int32 {
                    reason: ctx.ad_params.num_images_counter,
                    addr: 0,
                    value: images,
                },
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
            vec![ParamSetValue::Float64 {
                reason: ctx.ad_params.time_remaining,
                addr: 0,
                value: remaining,
            }],
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
            Err(e) => {
                return Err(Abort::Error(format!("error reading from p2util: {e}")));
            }
        }

        if poll_start.elapsed() > deadline {
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
            ParamSetValue::Octet {
                reason: ctx.ad_params.status_message,
                addr: 0,
                value: format!("Reading from File {file_name}"),
            },
            ParamSetValue::Octet {
                reason: ctx.ad_params.base.full_file_name,
                addr: 0,
                value: file_name.to_string(),
            },
        ],
    )
    .await;

    let path = Path::new(file_name);
    let expected = raw::frame_bytes();
    let deadline = Duration::from_secs_f64(setup.acquire_time.max(0.0)) + FILE_READ_TIMEOUT;
    let poll_start = Instant::now();

    loop {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(Abort::Stopped);
        }
        match raw::check_raw_file(path, start, expected) {
            FileState::Ready => break,
            FileState::Missing | FileState::Stale | FileState::Incomplete { .. } => {}
        }
        if poll_start.elapsed() > deadline {
            return Err(Abort::Timeout(format!(
                "timeout waiting for the frame file {file_name}"
            )));
        }
        rt::sleep(FILE_READ_DELAY).await;
    }

    let bytes = raw::read_raw_file(path)
        .map_err(|e| Abort::Error(format!("cannot read {file_name}: {e}")))?;
    let data = raw::decode_raw(&bytes)
        .map_err(|e| Abort::Error(format!("cannot decode {file_name}: {e}")))?;

    let ts = EpicsTimestamp::now();
    let data_type = data.data_type();
    let array = NDArray {
        unique_id: array_counter,
        timestamp: ts,
        // C stamped every frame of an acquisition with the acquisition's start
        // time, not the frame's; keep that, it is the exposure start.
        time_stamp: start_ts.as_f64(),
        dims: vec![NDDimension::new(PII_SIZE_X), NDDimension::new(PII_SIZE_Y)],
        data_size: data.total_bytes(),
        pool_id: 0,
        data,
        attributes: Default::default(),
        codec: None,
    };

    set_params(
        ctx,
        vec![
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
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_x,
                addr: 0,
                value: PII_SIZE_X as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size_y,
                addr: 0,
                value: PII_SIZE_Y as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.array_size,
                addr: 0,
                value: raw::frame_bytes() as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.n_dimensions,
                addr: 0,
                value: 2,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.color_mode,
                addr: 0,
                value: NDColorMode::Mono as i32,
            },
            ParamSetValue::Int32 {
                reason: ctx.ad_params.base.data_type,
                addr: 0,
                value: data_type as u8 as i32,
            },
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
        Some(Abort::Timeout(m)) | Some(Abort::Error(m)) => {
            log::error!("photonii: {m}");
            Some(m.clone())
        }
    };

    let mut updates = vec![
        ParamSetValue::Int32 {
            reason: ctx.ad_params.acquire,
            addr: 0,
            value: 0,
        },
        ParamSetValue::Float64 {
            reason: ctx.ad_params.time_remaining,
            addr: 0,
            value: 0.0,
        },
        ParamSetValue::Int32 {
            reason: ctx.ad_params.status,
            addr: 0,
            value: match abort {
                Some(Abort::Error(_)) | Some(Abort::Timeout(_)) => ADStatus::Error as i32,
                _ => ADStatus::Idle as i32,
            },
        },
    ];
    if let Some(m) = message {
        updates.push(ParamSetValue::Octet {
            reason: ctx.ad_params.status_message,
            addr: 0,
            value: m,
        });
    }
    set_params(ctx, updates).await;
}

async fn set_params(ctx: &AcquisitionContext, values: Vec<ParamSetValue>) {
    let _ = ctx.handle.set_params_and_notify(0, values).await;
}

fn read_string(sync: &SyncIOHandle, reason: usize) -> String {
    match sync.read_octet(reason, MAX_MESSAGE_SIZE) {
        Ok(bytes) => String::from_utf8_lossy(&bytes)
            .trim_end_matches('\0')
            .to_string(),
        Err(e) => {
            log::error!("photonii: cannot read a string parameter: {e}");
            String::new()
        }
    }
}
