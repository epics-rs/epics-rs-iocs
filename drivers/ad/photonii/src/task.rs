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

use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::NDDimension;
use epics_rs::ad_core::ndarray_pool::DriverPool;
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
    pub pool: DriverPool,
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
    // C's expectedTime is signed, so TimeRemaining goes below zero past it
    // or on a negative AcquireTime (PhotonII.cpp:350, :363-364).
    let expected = f64::from(setup.count) * setup.acquire_time;

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

        // A dark frame stays inside the detector server (PhotonII.cpp:412).
        if setup.array_callbacks
            && setup.frame_type != FrameType::Dark
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
    expected: f64,
) -> Result<String, Abort> {
    let deadline = setup.acquire_time + FILE_READ_TIMEOUT.as_secs_f64();
    let poll_start = Instant::now();

    loop {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(Abort::Stopped);
        }

        let remaining = expected - start.elapsed().as_secs_f64();
        set_params(
            ctx,
            vec![ParamSetValue::new(
                ctx.ad_params.time_remaining,
                0,
                ParamValue::Float64(remaining),
            )],
        )
        .await;

        // readPhotonII posts every read's line, a timeout's included
        // (PhotonII.cpp:259-260).
        let (line, status) = ctx.p2.read(FILE_READ_DELAY);
        if let Some(line) = &line {
            set_params(
                ctx,
                vec![ParamSetValue::new(
                    ctx.ad_params.string_from_server,
                    0,
                    ParamValue::Octet(line.as_bytes().into()),
                )],
            )
            .await;
        }
        match status.map(|()| line.unwrap_or_default()) {
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
                trace_error(ctx, &format!("PhotonII::readPhotonII, status={status}\n"));
                trace_error(
                    ctx,
                    &format!(
                        "PhotonII::PhotonIITask: error reading file written message status={status}\n"
                    ),
                );
                return Err(Abort::Error(format!("error reading from p2util: {e}")));
            }
        }

        if poll_start.elapsed().as_secs_f64() > deadline {
            trace_error(
                ctx,
                "PhotonII::PhotonIITask: timeout waiting for file written message\n",
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
    let timeout = setup.acquire_time + FILE_READ_TIMEOUT.as_secs_f64();
    // C's task prints this after every readRaw failure, a stop included.
    let read_raw_failed = |abort: Abort| {
        trace_error(
            ctx,
            &format!(
                "PhotonII::PhotonIITask, error calling readRaw(), status={}\n",
                AsynStatus::Error as i32
            ),
        );
        abort
    };
    let mut file_exists = false;
    let mut ready = false;

    // C `while (deltaTime <= timeout)`: a negative timeout never looks for
    // the file (PhotonII.cpp:165-198).
    let poll_start = Instant::now();
    let mut delta = 0.0;
    while delta <= timeout {
        if ctx.shared.stop_requested.load(Ordering::Acquire) {
            return Err(read_raw_failed(Abort::Stopped));
        }
        match raw::check_raw_file(path, start, expected) {
            FileState::Ready => {
                ready = true;
                break;
            }
            FileState::Missing => {}
            FileState::Stale | FileState::Incomplete { .. } => file_exists = true,
        }
        rt::sleep(FILE_READ_DELAY).await;
        delta = poll_start.elapsed().as_secs_f64();
    }
    if !ready {
        trace_error(
            ctx,
            &format!("PhotonII::readRaw timeout waiting for file to be created {file_name}\n"),
        );
        if file_exists {
            trace_error(ctx, "  file exists but is either old or the wrong size\n");
        }
        return Err(read_raw_failed(Abort::Timeout(format!(
            "timeout waiting for the frame file {file_name}"
        ))));
    }

    let bytes = raw::read_raw_file(path)
        .map_err(|e| read_raw_failed(Abort::Error(format!("cannot read {file_name}: {e}"))))?;
    let data = raw::decode_raw(&bytes)
        .map_err(|e| read_raw_failed(Abort::Error(format!("cannot decode {file_name}: {e}"))))?;

    // C updateTimeStamp (PhotonII.cpp:435): the port's stamp, moved now.
    let ts: EpicsTimestamp = ctx
        .handle
        .update_time_stamp()
        .await
        .unwrap_or_else(|_| SystemTime::now())
        .into();
    // C dereferences a refused alloc unchecked (PhotonII.cpp:416); here the frame
    // is dropped after the pool's limit line.
    let Ok(mut array) = ctx.pool.alloc(
        vec![NDDimension::new(PII_SIZE_X), NDDimension::new(PII_SIZE_Y)],
        data.data_type(),
    ) else {
        return Ok(());
    };
    array
        .fill()
        .data_mut()
        .as_u8_slice_mut()
        .copy_from_slice(data.as_u8_slice());
    array.unique_id = array_counter;
    array.timestamp = ts;
    // C stamped every frame of an acquisition with the acquisition's start
    // time, not the frame's; keep that, it is the exposure start.
    array.time_stamp = start_ts.as_f64();

    // C stores no array size, type or time-stamp param per frame; the
    // constructor's values stand (PhotonII.cpp:398-440).
    ctx.output.publish(array).await;
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
    ctx.handle.trace().output_device(
        ctx.handle.port_name(),
        Some(0),
        0,
        TraceMask::ERROR,
        message,
    );
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::create_photonii_detector;
    use epics_rs::ad_core::plugin::channel::NDArrayOutput;
    use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use std::sync::atomic::AtomicBool;

    /// A task context whose p2util socket connects and never speaks.
    fn silent_ctx(name: &str) -> (AcquisitionContext, std::net::TcpListener) {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let ip = format!("{name}_IP");
        let addr = listener.local_addr().unwrap().to_string();
        let (ip_rt, _) = create_port_runtime(
            &claim_port(&ip).unwrap(),
            DrvAsynIPPort::new(&ip, &addr).unwrap(),
            RuntimeConfig::default(),
        )
        .unwrap();
        let sync = || SyncIOHandle::from_handle(ip_rt.port_handle().clone(), 0, Duration::ZERO);
        let det = create_photonii_detector(&claim_port(name).unwrap(), sync(), 0).unwrap();
        let p2 = P2Util::new(sync());
        // Until the socket is up a read fails rather than times out.
        for _ in 0..100 {
            if matches!(p2.read(FILE_READ_DELAY).1, Err(ChannelError::Timeout)) {
                break;
            }
        }
        let (_tx, commands) = rt::command_channel::<TaskCommand>(1);
        let ctx = AcquisitionContext {
            p2,
            handle: det.port_handle().clone(),
            output: ArrayPublisher::new(Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()))),
            pool: det.pool().clone(),
            ad_params: det.ad_params,
            params: det.params,
            shared: Arc::new(SharedState {
                stop_requested: AtomicBool::new(false),
            }),
            commands,
        };
        std::mem::forget(ip_rt);
        std::mem::forget(det);
        (ctx, listener)
    }

    fn setup(acquire_time: f64) -> AcquireSetup {
        AcquireSetup {
            frame_type: FrameType::Normal,
            acquire_time,
            count: 1,
            file_path: String::new(),
            file_name: String::new(),
            file_number: 0,
            array_callbacks: true,
        }
    }

    /// wait_for_frame on its own thread; what it ended with, how long it took
    /// and the last TimeRemaining it posted.
    fn wait(
        ctx: AcquisitionContext,
        acquire_time: f64,
        stop_after: Option<Duration>,
    ) -> (bool, Duration, f64) {
        if let Some(after) = stop_after {
            let shared = ctx.shared.clone();
            std::thread::spawn(move || {
                std::thread::sleep(after);
                shared.stop_requested.store(true, Ordering::Release);
            });
        }
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = ctx.handle.clone();
        let remaining = ctx.ad_params.time_remaining;
        rt::run_thread(move || async move {
            let setup = setup(acquire_time);
            let start = Instant::now();
            let timed_out = matches!(
                wait_for_frame(
                    &ctx,
                    &setup,
                    start,
                    f64::from(setup.count) * setup.acquire_time
                )
                .await,
                Err(Abort::Timeout(_))
            );
            let _ = tx.send((timed_out, start.elapsed()));
        })
        .join()
        .unwrap();
        let (timed_out, took) = rx.recv().unwrap();
        (
            timed_out,
            took,
            handle.read_float64_blocking(remaining, 0).unwrap(),
        )
    }

    /// C TimeRemaining is expectedTime - elapsed, unclamped: past the
    /// expected time it goes below zero (PhotonII.cpp:350, :363-364).
    #[test]
    fn time_remaining_goes_below_zero_past_the_expected_time() {
        let (ctx, _listener) = silent_ctx("PII_REMAINING");
        let (_, _, remaining) = wait(ctx, 0.05, Some(Duration::from_millis(300)));
        assert!(remaining < -0.1, "{remaining}");
    }

    /// A negative AcquireTime gives C a negative expected time and a deadline
    /// already past (PhotonII.cpp:350, :390).
    #[test]
    fn a_negative_acquire_time_times_out_at_once() {
        let (ctx, _listener) = silent_ctx("PII_NEGATIVE");
        let (timed_out, took, remaining) = wait(ctx, -10.0, Some(Duration::from_secs(5)));
        assert!(timed_out);
        assert!(took < Duration::from_secs(1), "{took:?}");
        assert!(remaining < -9.9, "{remaining}");
    }

    /// readRaw's loop never runs on a negative timeout, so even a ready file
    /// times out (PhotonII.cpp:169, :417-421).
    #[test]
    fn read_raw_with_a_negative_timeout_never_looks_for_the_file() {
        let (ctx, _listener) = silent_ctx("PII_RAW_NEGATIVE");
        let path = std::env::temp_dir().join(format!("pii-neg-{}.raw", std::process::id()));
        std::fs::write(&path, vec![0u8; raw::frame_bytes()]).unwrap();
        let file = path.to_string_lossy().into_owned();
        let (tx, rx) = std::sync::mpsc::channel();
        rt::run_thread(move || async move {
            let start = SystemTime::now() - Duration::from_secs(5);
            let outcome =
                publish_frame(&ctx, &setup(-10.0), &file, start, EpicsTimestamp::now(), 1).await;
            let _ = tx.send(matches!(outcome, Err(Abort::Timeout(_))));
        })
        .join()
        .unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(rx.recv().unwrap());
    }

    /// PhotonIITask stores no array size, dimension, colour, type or time
    /// stamp param per frame (PhotonII.cpp:398-440).
    #[test]
    fn a_frame_leaves_the_array_params_alone() {
        let (ctx, _listener) = silent_ctx("PII_FRAME_PARAMS");
        let handle = ctx.handle.clone();
        let base = ctx.ad_params.base;
        let path = std::env::temp_dir().join(format!("pii-frame-{}.raw", std::process::id()));
        std::fs::write(&path, vec![0u8; raw::frame_bytes()]).unwrap();
        let file = path.to_string_lossy().into_owned();
        let (tx, rx) = std::sync::mpsc::channel();
        rt::run_thread(move || async move {
            let start = SystemTime::now() - Duration::from_secs(5);
            let outcome =
                publish_frame(&ctx, &setup(0.0), &file, start, EpicsTimestamp::now(), 1).await;
            let _ = tx.send(outcome.is_ok());
        })
        .join()
        .unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(rx.recv().unwrap());
        let int = |reason| handle.read_int32_blocking(reason, 0).unwrap();
        assert_eq!(int(base.n_dimensions), 0);
        assert_eq!(int(base.epics_ts_sec), 0);
        assert_eq!(int(base.epics_ts_nsec), 0);
        assert_eq!(
            handle.read_float64_blocking(base.timestamp_rbv, 0).unwrap(),
            0.0
        );
    }
}
