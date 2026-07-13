//! The three threads behind the driver (C `udpDataListenerTask`, `dataTask`
//! and `statusTask`).

use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::{Receiver, SyncSender};
use std::time::{Duration, Instant};

use epics_rs::asyn::param::ParamValue;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::ParamSetValue;
use epics_rs::asyn::sync_io::SyncIOHandle;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute, NDAttributeList};
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};
use epics_rs::ad_core::params::ADBaseParams;
use epics_rs::ad_core::plugin::channel::{ArrayPublisher, QueuedArrayCounter};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::connection::PixiradServer;
use crate::decode::Decoder;
use crate::driver::SharedState;
use crate::params::PixiradParams;
use crate::protocol;
use crate::types::{FrameType, MAX_UDP_PACKET_LEN, Sensor};
use crate::udp::{Frame, FrameAssembler};

/// The box sends the colours in the order thresh2, thresh1, thresh4, thresh3
/// (C `colorOffsetMap`).
const COLOR_OFFSET_MAP: [usize; 4] = [1, 0, 3, 2];

// ───────────────────────────── UDP data listener ─────────────────────────────

/// Read packets and hand complete frames on (C `udpDataListenerTask`).
pub(crate) fn start_udp_listener(
    socket: UdpSocket,
    sensor: Sensor,
    frames: SyncSender<Frame>,
    shared: Arc<SharedState>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("PixiradUDPDataTask".into())
        .spawn(move || {
            let mut assembler = FrameAssembler::new(sensor);
            let mut packet = vec![0u8; MAX_UDP_PACKET_LEN];
            let mut started_at: Option<Instant> = None;

            loop {
                let n = match socket.recv(&mut packet) {
                    Ok(n) => n,
                    Err(e) => {
                        log::error!("pixirad: error reading the UDP data port: {e}");
                        continue;
                    }
                };
                let started = *started_at.get_or_insert_with(Instant::now);

                let Some(frame) = assembler.accept(&packet[..n]) else {
                    continue;
                };

                let seconds = started.elapsed().as_secs_f64();
                if seconds > 0.0 {
                    let bytes = frame.payload.len() as f64;
                    shared.set_speed(bytes / (seconds * 1024.0 * 1024.0));
                }
                started_at = None;

                shared.udp_buffers_read.fetch_add(1, Ordering::AcqRel);
                shared.queued_frames.fetch_add(1, Ordering::AcqRel);
                if frames.send(frame).is_err() {
                    // The data task is gone; so is the point of reading.
                    return;
                }
            }
        })
        .expect("failed to spawn the Pixirad UDP thread")
}

// ─────────────────────────────────── data ────────────────────────────────────

pub(crate) struct DataContext {
    pub handle: PortHandle,
    pub output: ArrayPublisher,
    #[allow(dead_code)] // held so plugins can be back-pressured on the queue
    pub queued: Arc<QueuedArrayCounter>,
    pub ad_params: ADBaseParams,
    pub params: PixiradParams,
    pub sensor: Sensor,
    pub shared: Arc<SharedState>,
    pub frames: Receiver<Frame>,
}

pub(crate) fn start_data_task(ctx: DataContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PixiradDataTask", move || data_loop(ctx))
}

/// The image being built out of one frame per colour.
struct Image {
    pixels: Vec<u16>,
    colors: usize,
    timestamp: EpicsTimestamp,
}

async fn data_loop(ctx: DataContext) {
    let sync = SyncIOHandle::from_handle(ctx.handle.clone(), 0, Duration::from_secs(30));
    let mut decoder = Decoder::new(ctx.sensor);
    let pixels_per_color = ctx.sensor.image_pixels();
    // C allocated the NDArray only when ColorsCollected was 0 and memcpy'd into
    // it whatever the count was, so a first frame that arrived with a stale
    // non-zero count wrote through a null pointer. The image being assembled is
    // owned here instead: there is no path on which a frame has nowhere to go.
    let mut image: Option<Image> = None;

    loop {
        // A blocking receive on this thread's own runtime: nothing else runs on
        // it, and the decode below blocks for far longer anyway.
        let Ok(frame) = ctx.frames.recv() else {
            return;
        };
        ctx.shared.queued_frames.fetch_sub(1, Ordering::AcqRel);

        if frame.align_errors {
            log::error!("pixirad: frame has alignment errors");
        }

        let auto_calibrate = sync.read_int32(ctx.params.auto_calibrate).unwrap_or(0) != 0;
        let frame_type =
            FrameType::from_i32(sync.read_int32(ctx.ad_params.frame_type).unwrap_or(0))
                .unwrap_or(FrameType::OneColorLow);
        // An autocalibration run sends one frame, whatever the frame type says.
        let num_colors = if auto_calibrate {
            1
        } else {
            frame_type.num_colors()
        };
        let mut colors_collected = sync
            .read_int32(ctx.params.colors_collected)
            .unwrap_or(0)
            .max(0) as usize;
        if colors_collected >= num_colors {
            // The frame type changed under a part-built image.
            colors_collected = 0;
            image = None;
        }

        let decoded = match decoder.decode(frame.is_autocal, &frame.payload) {
            Ok(pixels) => pixels,
            Err(e) => {
                log::error!("pixirad: cannot decode the frame: {e}");
                continue;
            }
        };

        let image = image.get_or_insert_with(|| Image {
            pixels: vec![0u16; pixels_per_color * num_colors],
            colors: num_colors,
            timestamp: EpicsTimestamp::now(),
        });
        if image.colors != num_colors {
            image.pixels = vec![0u16; pixels_per_color * num_colors];
            image.colors = num_colors;
            image.timestamp = EpicsTimestamp::now();
        }

        let offset = if num_colors == 1 {
            0
        } else {
            COLOR_OFFSET_MAP[colors_collected] * pixels_per_color
        };
        image.pixels[offset..offset + pixels_per_color].copy_from_slice(decoded);
        colors_collected += 1;

        let udp_updates = vec![
            ParamSetValue::new(
                ctx.params.udp_buffers_read,
                0,
                ParamValue::Int32(ctx.shared.udp_buffers_read.load(Ordering::Acquire)),
            ),
            ParamSetValue::new(
                ctx.params.udp_buffers_free,
                0,
                ParamValue::Int32(
                    ctx.shared.max_buffers - ctx.shared.queued_frames.load(Ordering::Acquire),
                ),
            ),
            ParamSetValue::new(
                ctx.params.udp_speed,
                0,
                ParamValue::Float64(ctx.shared.speed()),
            ),
            ParamSetValue::new(
                ctx.params.colors_collected,
                0,
                ParamValue::Int32((colors_collected % num_colors) as i32),
            ),
        ];
        let _ = ctx.handle.set_params_and_notify(0, udp_updates).await;

        if colors_collected < num_colors {
            continue;
        }

        // Every colour is in: this is an image.
        let unique_id = sync
            .read_int32(ctx.ad_params.base.array_counter)
            .unwrap_or(0)
            + 1;
        let images_taken = sync
            .read_int32(ctx.ad_params.num_images_counter)
            .unwrap_or(0)
            + 1;
        let num_images = sync.read_int32(ctx.ad_params.num_images).unwrap_or(1);
        let array_callbacks = sync
            .read_int32(ctx.ad_params.base.array_callbacks)
            .unwrap_or(1)
            != 0;

        let ts = image.timestamp;
        let taken = image.pixels.clone();
        let colors = image.colors;

        let mut updates = vec![
            ParamSetValue::new(
                ctx.ad_params.base.array_counter,
                0,
                ParamValue::Int32(unique_id),
            ),
            ParamSetValue::new(
                ctx.ad_params.num_images_counter,
                0,
                ParamValue::Int32(images_taken),
            ),
            ParamSetValue::new(
                ctx.ad_params.base.array_size,
                0,
                ParamValue::Int32((taken.len() * 2) as i32),
            ),
        ];
        if images_taken >= num_images {
            updates.push(ParamSetValue::new(
                ctx.ad_params.acquire,
                0,
                ParamValue::Int32(0),
            ));
        }
        if auto_calibrate {
            updates.push(ParamSetValue::new(
                ctx.params.auto_calibrate,
                0,
                ParamValue::Int32(0),
            ));
        }
        let _ = ctx.handle.set_params_and_notify(0, updates).await;

        if array_callbacks {
            publish(&ctx, taken, colors, unique_id, ts).await;
        }
    }
}

async fn publish(
    ctx: &DataContext,
    pixels: Vec<u16>,
    colors: usize,
    unique_id: i32,
    ts: EpicsTimestamp,
) {
    let mut dims = vec![
        NDDimension::new(ctx.sensor.rows),
        NDDimension::new(ctx.sensor.cols * ctx.sensor.modules),
    ];
    if colors > 1 {
        dims.push(NDDimension::new(colors));
    }
    let n_dims = dims.len();

    let mut attributes = NDAttributeList::new();
    attributes.add(NDAttribute {
        name: "ColorsCollected".into(),
        description: "Colours in this image".into(),
        source: NDAttrSource::Driver,
        value: NDAttrValue::Int32(colors as i32),
        source_impl: None,
    });

    let data = NDDataBuffer::U16(pixels);
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

    let _ = ctx
        .handle
        .set_params_and_notify(
            0,
            vec![
                ParamSetValue::new(
                    ctx.ad_params.base.n_dimensions,
                    0,
                    ParamValue::Int32(n_dims as i32),
                ),
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
            ],
        )
        .await;

    ctx.output.publish(Arc::new(array)).await;
}

// ────────────────────────────────── status ───────────────────────────────────

pub(crate) struct StatusContext {
    pub socket: UdpSocket,
    pub server: PixiradServer,
    pub handle: PortHandle,
    pub ad_params: ADBaseParams,
    pub params: PixiradParams,
}

pub(crate) fn start_status_task(ctx: StatusContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("PixiradStatusTask", move || status_loop(ctx))
}

/// Read the environmental broadcast, work out the dew point, and switch the
/// cooling off if the box is in trouble (C `statusTask`).
async fn status_loop(ctx: StatusContext) {
    let sync = SyncIOHandle::from_handle(ctx.handle.clone(), 0, Duration::from_secs(30));
    // C blocked in `recvfrom` for ever; a timeout keeps the loop alive when the
    // box stops broadcasting.
    let _ = ctx.socket.set_read_timeout(Some(Duration::from_secs(5)));
    let mut buffer = [0u8; 256];

    loop {
        rt::sleep(Duration::from_secs(1)).await;

        let n = match ctx.socket.recv(&mut buffer) {
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                log::error!("pixirad: error reading the status broadcast: {e}");
                continue;
            }
        };
        let message = String::from_utf8_lossy(&buffer[..n]).to_string();

        let readings = [
            ("READ_TCOLD", ctx.ad_params.temperature_actual),
            ("READ_THOT", ctx.params.hot_temperature),
            ("READ_BOX_TEMP", ctx.params.box_temperature),
            ("READ_BOX_HUM", ctx.params.box_humidity),
            ("READ_PELTIER_PWR", ctx.params.peltier_power),
            ("READ_HV", ctx.params.hv_actual),
            ("READ_HV_CURRENT", ctx.params.hv_current),
        ];

        let mut updates = Vec::new();
        let mut cold_temp = None;
        let mut hot_temp = None;
        let mut box_temp = None;
        let mut humidity = None;
        for (key, reason) in readings {
            let Some(value) = protocol::status_value(&message, key) else {
                log::error!("pixirad: no {key} in the status broadcast '{message}'");
                continue;
            };
            match key {
                "READ_TCOLD" => cold_temp = Some(value),
                "READ_THOT" => hot_temp = Some(value),
                "READ_BOX_TEMP" => box_temp = Some(value),
                "READ_BOX_HUM" => humidity = Some(value),
                _ => {}
            }
            updates.push(ParamSetValue::new(reason, 0, ParamValue::Float64(value)));
        }

        // C computed the dew point from whatever was last in the parameter
        // library, so a broadcast that was missing the humidity produced a dew
        // point from a stale reading and could switch the cooling off on it.
        let (Some(cold_temp), Some(hot_temp), Some(box_temp), Some(humidity)) =
            (cold_temp, hot_temp, box_temp, humidity)
        else {
            let _ = ctx.handle.set_params_and_notify(0, updates).await;
            continue;
        };

        let dew_point = protocol::dew_point(humidity, box_temp);
        let cooling_status = protocol::cooling_status(cold_temp, hot_temp, dew_point);
        updates.push(ParamSetValue::new(
            ctx.params.dew_point,
            0,
            ParamValue::Float64(dew_point),
        ));
        updates.push(ParamSetValue::new(
            ctx.params.cooling_status,
            0,
            ParamValue::Int32(cooling_status as i32),
        ));
        if cooling_status.is_error() {
            updates.push(ParamSetValue::new(
                ctx.params.cooling_state,
                0,
                ParamValue::Int32(0),
            ));
        }
        let _ = ctx.handle.set_params_and_notify(0, updates).await;

        if !cooling_status.is_error() {
            continue;
        }

        // Switch the cooling off at the box. Every parameter it needs is read
        // before the command socket is taken — see the invariant in
        // `crate::connection`.
        log::error!("pixirad: {cooling_status:?}, switching the cooling off");
        let temperature = sync.read_float64(ctx.ad_params.temperature).unwrap_or(0.0);
        let hv_value = sync.read_float64(ctx.params.hv_value).unwrap_or(0.0);
        let hv_state = sync.read_int32(ctx.params.hv_state).unwrap_or(0);
        let command = protocol::init(temperature, 0, hv_value, hv_state);
        let exchange = ctx.server.command(&command);
        if let Err(e) = exchange.result {
            log::error!("pixirad: '{command}': {e}");
        }
    }
}
