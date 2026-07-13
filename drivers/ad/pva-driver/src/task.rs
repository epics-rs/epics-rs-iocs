use std::sync::Arc;

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;

use epics_rs::ad_core::driver::ImageMode;
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;
use epics_rs::pva::client::PvaClient;
use epics_rs::pva::client_native::context::ConnectHandle;
use epics_rs::pva::client_native::ops_v2::SubscriptionHandle;
use epics_rs::pva::{PvField, PvaResult};

use crate::convert::decode_nt_nd_array;
use crate::params::PvaParams;
use crate::types::PvaCommand;

/// Bridges the PVA client's synchronous callbacks (`pvmonitor_handle`'s data
/// callback and `connect().on_connect`/`on_disconnect`, none of which are
/// `async`) into this task's event loop, which needs to `.await` on
/// `PortHandle` I/O to react to them. Not a wire concept — internal plumbing
/// only, distinct from `PvaCommand` (driver writes -> task).
enum PvaEvent {
    Connected,
    Disconnected,
    Frame(PvField),
}

/// Bundled state for the PVA monitor task thread.
pub(crate) struct MonitorContext {
    pub cmd_rx: rt::CommandReceiver<PvaCommand>,
    pub handle: PortHandle,
    pub publisher: ArrayPublisher,
    pub ad: ADBaseParams,
    pub pva: PvaParams,
    pub initial_pv_name: String,
}

/// Start the monitor task thread via the `rt` facade.
pub(crate) fn start_pva_task(ctx: MonitorContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PVATask", move || pva_loop_async(ctx))
}

/// One live PV connection's adopted state: the connect-watcher (drives
/// `PVAPvConnectionStatus`) and the pausable data subscription.
struct Connection {
    connect: ConnectHandle,
    subscription: SubscriptionHandle,
}

/// Mirrors C++ `pvaDriver::connectPv`: builds a brand new channel watcher +
/// monitor for `pv_name` before touching any existing connection, so a
/// failed attempt leaves the caller's current [`Connection`] completely
/// untouched (`connectPv`'s tentative `channel`/`monitor` locals are simply
/// dropped on a thrown exception, leaving `m_channel`/`m_monitor` alone).
///
/// The new subscription is paused immediately after creation. C++'s monitor
/// is created but never `start()`-ed by `connectPv()` itself — only an
/// explicit `ADAcquire=1` write calls `m_monitor->start()` — so a freshly
/// built [`SubscriptionHandle`] here (which begins unpaused) must be paused
/// right away to reproduce that same silent-until-`ADAcquire=1` behavior,
/// regardless of whether this is the initial connect or a later reconnect.
async fn try_connect(
    client: &PvaClient,
    pv_name: &str,
    event_tx: rt::CommandSender<PvaEvent>,
) -> PvaResult<Connection> {
    let connect_tx = event_tx.clone();
    let disconnect_tx = event_tx.clone();
    let connect = client
        .connect(pv_name)
        .on_connect(move || {
            let _ = connect_tx.try_send(PvaEvent::Connected);
        })
        .on_disconnect(move || {
            let _ = disconnect_tx.try_send(PvaEvent::Disconnected);
        })
        .exec()
        .await?;

    let frame_tx = event_tx;
    let subscription = match client
        .pvmonitor_handle(pv_name, move |_desc, value| {
            let _ = frame_tx.try_send(PvaEvent::Frame(value.clone()));
        })
        .await
    {
        Ok(sub) => sub,
        Err(e) => {
            connect.wait().await;
            return Err(e);
        }
    };
    subscription.pause().await;

    Ok(Connection {
        connect,
        subscription,
    })
}

async fn pva_loop_async(mut ctx: MonitorContext) {
    let client = match PvaClient::new() {
        Ok(c) => c,
        Err(e) => {
            log::error!("ad-pva-driver: failed to create PVA client: {e}");
            return;
        }
    };

    let (event_tx, mut event_rx) = rt::command_channel::<PvaEvent>(64);

    // C++'s constructor calls `connectPv(pvName)` unconditionally and
    // discards its return status; a failed initial connect just leaves the
    // driver disconnected — there is no earlier "known-good" name to fall
    // back to, so (unlike a later failed `Reconnect`) nothing is reverted.
    let mut current_pv_name = ctx.initial_pv_name.clone();
    let mut conn = match try_connect(&client, &current_pv_name, event_tx.clone()).await {
        Ok(c) => Some(c),
        Err(e) => {
            log::error!("ad-pva-driver: initial connect to '{current_pv_name}' failed: {e}");
            None
        }
    };

    loop {
        tokio::select! {
            biased;
            cmd = ctx.cmd_rx.recv() => {
                let Some(cmd) = cmd else { break };
                handle_command(&mut ctx, &client, &event_tx, &mut conn, &mut current_pv_name, cmd).await;
            }
            ev = event_rx.recv() => {
                let Some(ev) = ev else { continue };
                handle_event(&ctx, conn.as_ref(), ev).await;
            }
        }
    }
}

async fn handle_command(
    ctx: &mut MonitorContext,
    client: &PvaClient,
    event_tx: &rt::CommandSender<PvaEvent>,
    conn: &mut Option<Connection>,
    current_pv_name: &mut String,
    cmd: PvaCommand,
) {
    match cmd {
        // C++ `writeInt32(ADAcquire, 1)`: `m_monitor->start()`. No null
        // check on `m_monitor` in C++ (a driver whose initial `connectPv`
        // failed would crash there) — `conn` being `None` here is the safe
        // Rust equivalent of that same broken-PV-name state, so this is a
        // deliberate no-op rather than a reproduced crash.
        PvaCommand::Start => {
            if let Some(c) = conn.as_ref() {
                c.subscription.resume().await;
            }
        }
        // C++ `writeInt32(ADAcquire, 0)`: `m_monitor->stop()`.
        PvaCommand::Stop => {
            if let Some(c) = conn.as_ref() {
                c.subscription.pause().await;
            }
        }
        PvaCommand::Reconnect(new_name) => {
            match try_connect(client, &new_name, event_tx.clone()).await {
                Ok(new_conn) => {
                    if let Some(old) = conn.take() {
                        old.subscription.stop_sync().await;
                        old.connect.wait().await;
                    }
                    *conn = Some(new_conn);
                    *current_pv_name = new_name;
                }
                Err(e) => {
                    log::error!("ad-pva-driver: reconnect to '{new_name}' failed: {e}");
                    // Mirrors `writeOctet`'s revert-on-failure: `connectPv()`
                    // left `m_pvName` untouched, so C++ reverts `PVAPvName` back
                    // to it.
                    let _ = ctx
                        .handle
                        .set_params_and_notify(
                            0,
                            vec![ParamSetValue::new(
                                ctx.pva.pv_name,
                                0,
                                ParamValue::Octet(current_pv_name.clone()),
                            )],
                        )
                        .await;
                }
            }
        }
    }
}

async fn handle_event(ctx: &MonitorContext, conn: Option<&Connection>, ev: PvaEvent) {
    match ev {
        // C++ `channelStateChange`: `setIntegerParam(PVAPvConnectionStatus,
        // state == Channel::CONNECTED); callParamCallbacks();`.
        PvaEvent::Connected => {
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ctx.pva.pv_connection_status,
                        0,
                        ParamValue::Int32(1),
                    )],
                )
                .await;
        }
        PvaEvent::Disconnected => {
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ctx.pva.pv_connection_status,
                        0,
                        ParamValue::Int32(0),
                    )],
                )
                .await;
        }
        PvaEvent::Frame(value) => process_frame(ctx, conn, value).await,
    }
}

/// One `monitor->poll()` iteration's worth of work from C++ `monitorEvent`.
async fn process_frame(ctx: &MonitorContext, conn: Option<&Connection>, raw: PvField) {
    // Overrun check — always, unconditionally, before the acquire gate
    // (matches C++'s `!update->overrunBitSet->isEmpty()` check, which
    // precedes the `ADAcquire` read). `stats(true)` resets `n_srv_squash` on
    // read; this task's monitor loop is the only reader/resetter and runs on
    // a single-threaded `current_thread` tokio runtime (`rt::run_thread_named`),
    // so no other task can interleave a read between the subscription's
    // internal per-frame stats update and this call — the delta read here is
    // exactly this frame's "did the server report a squashed update" bit,
    // the same per-update granularity C++ has.
    if let Some(c) = conn {
        let squash = c.subscription.stats(true).n_srv_squash;
        if squash > 0 {
            // C++ re-reads `PVAOverrunCounter` fresh from the param cache on
            // every check (not a driver-local running total) so a manual
            // `caput` to this param is preserved instead of being clobbered.
            let overrun = ctx
                .handle
                .read_int32(ctx.pva.overrun_counter, 0)
                .await
                .unwrap_or(0);
            let _ = ctx
                .handle
                .set_params_and_notify(
                    0,
                    vec![ParamSetValue::new(
                        ctx.pva.overrun_counter,
                        0,
                        ParamValue::Int32(overrun + squash as i32),
                    )],
                )
                .await;
        }
    }

    let acquire = ctx.handle.read_int32(ctx.ad.acquire, 0).await.unwrap_or(0);
    if acquire == 0 {
        return;
    }

    let array = match decode_nt_nd_array(&raw) {
        Ok(a) => a,
        Err(e) => {
            log::error!("ad-pva-driver: failed to convert NTNDArray into NDArray: {e}");
            return;
        }
    };

    let info = array.info();
    // C++ indexes `pImage->dims[info.x/y/color.dim]` directly rather than
    // using any pre-derived size fields — for a 2-D Mono array `color.dim`
    // defaults to the same index as `x.dim` (0), so `NDArraySizeZ` ends up
    // reporting the X size instead of 0. This is a "shared C-parity gap"
    // (see `NDArray::info()`'s doc comment) reproduced deliberately, not a
    // bug introduced here.
    let (Some(x), Some(y), Some(color)) = (
        array.dims.get(info.x_dim),
        array.dims.get(info.y_dim),
        array.dims.get(info.color_dim),
    ) else {
        log::error!("ad-pva-driver: NTNDArray dimension count too small for its own layout info");
        return;
    };
    let (x_size, y_size, color_size) = (x.size as i32, y.size as i32, color.size as i32);
    let (min_x, min_y) = (x.offset as i32, y.offset as i32);
    let (bin_x, bin_y) = (x.binning as i32, y.binning as i32);
    let (reverse_x, reverse_y) = (x.reverse as i32, y.reverse as i32);
    let data_type = array.data.data_type() as u8 as i32;
    let color_mode = info.color_mode as i32;
    let total_bytes = info.total_bytes as i32;

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::new(ctx.ad.max_size_x, 0, ParamValue::Int32(x_size)),
                ParamSetValue::new(ctx.ad.max_size_y, 0, ParamValue::Int32(y_size)),
                ParamSetValue::new(ctx.ad.size_x, 0, ParamValue::Int32(x_size)),
                ParamSetValue::new(ctx.ad.size_y, 0, ParamValue::Int32(y_size)),
                ParamSetValue::new(ctx.ad.base.array_size_x, 0, ParamValue::Int32(x_size)),
                ParamSetValue::new(ctx.ad.base.array_size_y, 0, ParamValue::Int32(y_size)),
                ParamSetValue::new(ctx.ad.base.array_size_z, 0, ParamValue::Int32(color_size)),
                ParamSetValue::new(ctx.ad.min_x, 0, ParamValue::Int32(min_x)),
                ParamSetValue::new(ctx.ad.min_y, 0, ParamValue::Int32(min_y)),
                ParamSetValue::new(ctx.ad.bin_x, 0, ParamValue::Int32(bin_x)),
                ParamSetValue::new(ctx.ad.bin_y, 0, ParamValue::Int32(bin_y)),
                ParamSetValue::new(ctx.ad.reverse_x, 0, ParamValue::Int32(reverse_x)),
                ParamSetValue::new(ctx.ad.reverse_y, 0, ParamValue::Int32(reverse_y)),
                ParamSetValue::new(ctx.ad.base.array_size, 0, ParamValue::Int32(total_bytes)),
                ParamSetValue::new(ctx.ad.base.data_type, 0, ParamValue::Int32(data_type)),
                ParamSetValue::new(ctx.ad.base.color_mode, 0, ParamValue::Int32(color_mode)),
            ],
        )
        .await;

    let array_callbacks = ctx
        .handle
        .read_int32(ctx.ad.base.array_callbacks, 0)
        .await
        .unwrap_or(0)
        != 0;
    if array_callbacks {
        ctx.publisher.publish(Arc::new(array)).await;
    }

    // Counters are updated after the plugin callback, matching C++'s own
    // ordering (`doCallbacksGenericPointer` then `NDArrayCounter`/
    // `ADNumImagesCounter`). Both are re-read fresh from the param cache
    // rather than tracked as a driver-local running total, matching C++'s
    // `getIntegerParam` calls here.
    let array_counter = ctx
        .handle
        .read_int32(ctx.ad.base.array_counter, 0)
        .await
        .unwrap_or(0)
        + 1;
    let num_images_counter = ctx
        .handle
        .read_int32(ctx.ad.num_images_counter, 0)
        .await
        .unwrap_or(0)
        + 1;
    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::new(
                    ctx.ad.base.array_counter,
                    0,
                    ParamValue::Int32(array_counter),
                ),
                ParamSetValue::new(
                    ctx.ad.num_images_counter,
                    0,
                    ParamValue::Int32(num_images_counter),
                ),
            ],
        )
        .await;

    // C++: `if (imageMode == ADImageMultiple) { ...; if (imageCounter >=
    // numImages) setIntegerParam(ADAcquire, 0); } if (imageMode ==
    // ADImageSingle) setIntegerParam(ADAcquire, 0);` — note this sets the
    // param directly, NOT through `writeInt32`, so (matching C++) the
    // subscription is deliberately left unpaused: later frames are just
    // silently discarded by the acquire-gate check above until the next
    // explicit `ADAcquire=1` write actually resumes it.
    let image_mode = ImageMode::from_i32(
        ctx.handle
            .read_int32(ctx.ad.image_mode, 0)
            .await
            .unwrap_or(0),
    );
    let stop = match image_mode {
        ImageMode::Single => true,
        ImageMode::Multiple => {
            let num_images = ctx
                .handle
                .read_int32(ctx.ad.num_images, 0)
                .await
                .unwrap_or(1);
            num_images_counter >= num_images
        }
        ImageMode::Continuous => false,
    };
    if stop {
        let _ = ctx
            .handle
            .set_params_and_notify(
                0,
                vec![ParamSetValue::new(ctx.ad.acquire, 0, ParamValue::Int32(0))],
            )
            .await;
    }
}
