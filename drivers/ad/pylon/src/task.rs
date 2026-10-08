//! Frame delivery. Pylon's grab-loop thread turns each grab result into an
//! NDArray and the parameter updates that go with it (C `processFrame`), and
//! hands both to the image task, which owns every exchange with the port
//! (C `imageGrabTask`).
//!
//! The grab-loop thread never waits on the port: the port actor may be inside
//! `Camera::stop`, which waits for that thread to leave its callback.

use std::sync::Arc;
use std::sync::atomic::{AtomicI32, AtomicI64, AtomicU64, Ordering};

use asyn_rs::error::AsynResult;
use asyn_rs::param::ParamValue;
use asyn_rs::port_handle::PortHandle;
use asyn_rs::request::{ParamSetValue, RequestOp};
use asyn_rs::user::AsynUser;
use parking_lot::RwLock;

use ad_core_rs::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use ad_core_rs::color::{NDBayerPattern, NDColorMode};
use ad_core_rs::driver::{ADStatus, ImageMode};
use ad_core_rs::ndarray::{NDArray, NDDataBuffer, NDDataType, NDDimension};
use ad_core_rs::ndarray_pool::NDArrayPool;
use ad_core_rs::params::ADBaseParams;
use ad_core_rs::plugin::channel::ArrayPublisher;
use ad_core_rs::runtime as rt;

use ad_genicam::{FeatureIndex, FeatureValue};

use crate::camera::{CameraHandler, ChunkValue, Frame, NodeKind, NodeMap, Nodes, PixelFormat};
use crate::driver::ADPylon;
use crate::node::TL_STATISTICS_FEATURE_NAMES;
use crate::trace::Trace;

/// Frames the image task may fall behind by before the grab thread drops one.
const QUEUE_DEPTH: usize = 16;

pub const TIME_STAMP_CAMERA: i32 = 0;
pub const UNIQUE_ID_CAMERA: i32 = 0;

pub(crate) enum Msg {
    /// One grab result, of any outcome.
    Result {
        tag: GrabTag,
        outcome: Outcome,
    },
    /// Camera event data.
    Params(Vec<ParamSetValue>),
    Removed,
}

/// Where a grab result stands in the grab that delivered it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GrabTag {
    /// The grab, numbered by `GrabShared::begin_grab`.
    pub grab: u64,
    /// The result after which pylon stops a counted grab; pylon counts
    /// failed results too.
    pub last: bool,
}

pub(crate) enum Outcome {
    Frame {
        array: NDArray,
        /// The `ColorMode` and `BayerPattern` attributes, which C adds after
        /// the `NDAttributesFile` set so that they win over a same-named entry.
        layout: (NDColorMode, NDBayerPattern),
        updates: Vec<ParamSetValue>,
    },
    /// The grab failed or the frame could not be represented.
    Failed { updates: Vec<ParamSetValue> },
    /// The NDArrayPool is exhausted; the acquisition cannot go on.
    NoBuffer { updates: Vec<ParamSetValue> },
}

/// Why a grab result produced no NDArray.
enum NoArray {
    Failed,
    PoolExhausted,
}

/// NDArray layout of a pylon pixel type: C `pix_lookup`, plus the BayerRG
/// formats that table leaves out.
fn pixel_layout(pixel: PixelFormat) -> (NDColorMode, NDDataType, NDBayerPattern) {
    use NDBayerPattern::*;
    use NDColorMode::*;
    use NDDataType::{UInt8, UInt16};
    match pixel {
        PixelFormat::Mono8 => (Mono, UInt8, BGGR),
        PixelFormat::Mono16 => (Mono, UInt16, BGGR),
        PixelFormat::Rgb8 => (RGB1, UInt8, BGGR),
        PixelFormat::Rgb16 => (RGB1, UInt16, BGGR),
        PixelFormat::BayerBG8 => (Bayer, UInt8, BGGR),
        PixelFormat::BayerGB8 => (Bayer, UInt8, GBRG),
        PixelFormat::BayerGR8 => (Bayer, UInt8, GRBG),
        PixelFormat::BayerRG8 => (Bayer, UInt8, RGGB),
        PixelFormat::BayerBG16 => (Bayer, UInt16, BGGR),
        PixelFormat::BayerGB16 => (Bayer, UInt16, GBRG),
        PixelFormat::BayerGR16 => (Bayer, UInt16, GRBG),
        PixelFormat::BayerRG16 => (Bayer, UInt16, RGGB),
    }
}

/// State the grab-loop thread shares with the driver.
pub(crate) struct GrabShared {
    pub ad: ADBaseParams,
    pub pool: Arc<NDArrayPool>,
    pub index: FeatureIndex,
    /// Per `EventSelector` entry, the event data features to read when that
    /// event fires.
    pub events: RwLock<Vec<Vec<String>>>,
    pub time_stamp_mode: AtomicI32,
    pub unique_id_mode: AtomicI32,
    pub unique_id: AtomicI32,
    /// The grab now running, or the last one to have run.
    pub grab: AtomicU64,
    /// Grab results pylon has yet to deliver before a counted grab stops on
    /// its own; negative while grabbing until stopped.
    pub results_left: AtomicI64,
    pub tx: rt::CommandSender<Msg>,
    pub trace: Trace,
}

impl GrabShared {
    /// Number a new grab and arm its result count, before
    /// `StartGrabbing(count)`. The previous grab has stopped, so no result
    /// of it is being tagged.
    pub fn begin_grab(&self, count: Option<u64>) {
        let left = count.map_or(-1, |c| c as i64);
        self.results_left.store(left, Ordering::Relaxed);
        self.grab.fetch_add(1, Ordering::Relaxed);
    }

    pub fn current_grab(&self) -> u64 {
        self.grab.load(Ordering::Relaxed)
    }

    /// Count one grab result of the running grab.
    fn take_result(&self) -> GrabTag {
        let last = self
            .results_left
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |left| {
                (left > 0).then(|| left - 1)
            })
            .is_ok_and(|left| left == 1);
        GrabTag {
            grab: self.current_grab(),
            last,
        }
    }

    fn send(&self, msg: Msg) {
        if self.tx.try_send(msg).is_err() {
            self.trace.error(&format!(
                "ADPylon: image task is {QUEUE_DEPTH} messages behind, dropping one"
            ));
        }
    }

    fn int32(&self, index: usize, value: i32) -> ParamSetValue {
        ParamSetValue::new(index, 0, ParamValue::Int32(value))
    }

    /// The update that stores `value` in the parameter bound to `feature`.
    fn feature_update(&self, feature: &str, value: FeatureValue) -> Option<ParamSetValue> {
        let binding = *self.index.read().get(feature)?;
        let value = value.to_param_value(binding.asyn_type)?;
        Some(ParamSetValue::new(binding.asyn_index, 0, value))
    }

    fn read_node(&self, nodes: &Nodes, map: NodeMap, name: &str) -> Option<FeatureValue> {
        match nodes.kind(map, name)? {
            NodeKind::Integer => nodes.get_int(map, name).ok().map(FeatureValue::Integer),
            NodeKind::Enum => nodes.get_enum(map, name).ok().map(FeatureValue::Integer),
            NodeKind::Boolean => nodes
                .get_bool(map, name)
                .ok()
                .map(|b| FeatureValue::Integer(b as i64)),
            NodeKind::Float => nodes.get_float(map, name).ok().map(FeatureValue::Double),
            NodeKind::String => nodes.get_string(map, name).ok().map(FeatureValue::String),
            NodeKind::Command | NodeKind::Other => None,
        }
    }

    fn statistics(&self, nodes: &Nodes) -> Vec<ParamSetValue> {
        TL_STATISTICS_FEATURE_NAMES
            .iter()
            .filter(|name| self.index.read().contains_key(**name))
            .filter_map(|name| {
                let value = self.read_node(nodes, NodeMap::Stream, name)?;
                self.feature_update(name, value)
            })
            .collect()
    }

    /// C `processFrame` up to the point where the array is handed on.
    fn build(
        &self,
        frame: &Frame<'_>,
        updates: &mut Vec<ParamSetValue>,
    ) -> Result<(NDArray, (NDColorMode, NDBayerPattern)), NoArray> {
        let failed = |text: String| {
            self.trace.error(&format!("ADPylon::processFrame {text}"));
            NoArray::Failed
        };
        if let Some(e) = &frame.convert_error {
            self.trace.error(&format!(
                "ADPylon::processFrame error converting, input pixel type=0x{:x}: {e}",
                frame.raw_pixel_type
            ));
        }
        let Some(pixel) = frame.pixel else {
            return Err(failed(format!(
                "unsupported pixel type=0x{:x}",
                frame.raw_pixel_type
            )));
        };
        let (color_mode, data_type, bayer) = pixel_layout(pixel);

        let dims = match color_mode {
            NDColorMode::RGB1 => vec![3, frame.width, frame.height],
            _ => vec![frame.width, frame.height],
        };
        let num_bytes = dims.iter().product::<usize>() * data_type.element_size();
        let Some(bytes) = frame.data.get(..num_bytes) else {
            return Err(failed("ERROR: image is invalid!".to_string()));
        };

        let dims = dims.into_iter().map(NDDimension::new).collect();
        let Ok(mut array) = self.pool.alloc(dims, data_type) else {
            self.trace.error(
                "ADPylon::processFrame ERROR: Serious problem: not enough buffers left! \
                 Aborting acquisition!",
            );
            return Err(NoArray::PoolExhausted);
        };
        match &mut array.data {
            NDDataBuffer::U8(v) => {
                v.clear();
                v.extend_from_slice(bytes);
            }
            NDDataBuffer::U16(v) => {
                v.clear();
                v.extend(
                    bytes
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .map(|b| u16::from_ne_bytes(*b)),
                );
            }
            _ => unreachable!("pixel_layout yields only UInt8 and UInt16"),
        }

        let driver_id = self.unique_id.fetch_add(1, Ordering::Relaxed);
        array.unique_id = if self.unique_id_mode.load(Ordering::Relaxed) == UNIQUE_ID_CAMERA {
            frame.id as i32
        } else {
            driver_id
        };
        array.update_time_stamps_now();
        if self.time_stamp_mode.load(Ordering::Relaxed) == TIME_STAMP_CAMERA {
            array.time_stamp = frame.timestamp as f64 / 1e9;
        }

        let nd = &self.ad.base;
        updates.extend([
            self.int32(nd.array_size_x, frame.width as i32),
            self.int32(nd.array_size_y, frame.height as i32),
            self.int32(nd.array_size, num_bytes as i32),
            self.int32(nd.data_type, data_type as i32),
            self.int32(nd.color_mode, color_mode as i32),
            self.int32(nd.bayer_pattern, bayer as i32),
        ]);

        // Chunk data becomes NDArray attributes, and the value of whichever
        // records are bound to the chunk features.
        for chunk in frame.chunks() {
            let (attr, value) = match chunk.value {
                ChunkValue::Integer(v) => (NDAttrValue::Int64(v), FeatureValue::Integer(v)),
                ChunkValue::Float(v) => (NDAttrValue::Float64(v), FeatureValue::Double(v)),
                ChunkValue::Boolean(v) => {
                    (NDAttrValue::UInt8(v as u8), FeatureValue::Integer(v as i64))
                }
                ChunkValue::String(v) => (NDAttrValue::String(v.clone()), FeatureValue::String(v)),
            };
            array.attributes.add(NDAttribute::new_static(
                chunk.name.as_str(),
                chunk.display_name,
                NDAttrSource::Driver,
                attr,
            ));
            updates.extend(self.feature_update(&chunk.name, value));
        }
        Ok((array, (color_mode, bayer)))
    }
}

pub(crate) struct GrabHandler(pub Arc<GrabShared>);

impl CameraHandler for GrabHandler {
    fn on_frame(&self, nodes: &Nodes, frame: Result<Frame<'_>, String>) {
        let shared = &self.0;
        let tag = shared.take_result();
        let mut updates = Vec::new();
        let built = match &frame {
            Ok(frame) => shared.build(frame, &mut updates),
            Err(e) => {
                shared
                    .trace
                    .error(&format!("ADPylon::processFrame error in grabbing: {e}"));
                Err(NoArray::Failed)
            }
        };
        let statistics = shared.statistics(nodes);
        let outcome = match built {
            Ok((array, layout)) => {
                updates.extend(statistics);
                Outcome::Frame {
                    array,
                    layout,
                    updates,
                }
            }
            Err(NoArray::PoolExhausted) => Outcome::NoBuffer {
                updates: statistics,
            },
            Err(NoArray::Failed) => Outcome::Failed {
                updates: statistics,
            },
        };
        shared.send(Msg::Result { tag, outcome });
    }

    fn on_event(&self, nodes: &Nodes, event_id: i32) {
        let shared = &self.0;
        let names = match shared.events.read().get(event_id as usize) {
            Some(names) => names.clone(),
            None => return,
        };
        let updates: Vec<ParamSetValue> = names
            .iter()
            .filter_map(|name| {
                let value = shared.read_node(nodes, NodeMap::Camera, name)?;
                shared.feature_update(name, value)
            })
            .collect();
        if !updates.is_empty() {
            shared.send(Msg::Params(updates));
        }
    }

    fn on_removed(&self) {
        self.0.send(Msg::Removed);
    }
}

pub(crate) fn channel() -> (rt::CommandSender<Msg>, rt::CommandReceiver<Msg>) {
    rt::command_channel(QUEUE_DEPTH)
}

pub(crate) struct ImageTask {
    pub rx: rt::CommandReceiver<Msg>,
    pub shared: Arc<GrabShared>,
    pub port: PortHandle,
    pub publisher: ArrayPublisher,
    pub ad: ADBaseParams,
    pub trace: Trace,
}

impl ImageTask {
    pub fn start(self) -> std::thread::JoinHandle<()> {
        rt::run_thread_named("PylonImageTask", move || self.run())
    }

    async fn run(mut self) {
        while let Some(msg) = self.rx.recv().await {
            if let Err(e) = self.handle(msg).await {
                if self.port.is_closed() {
                    // The IOC is shutting down.
                    break;
                }
                self.trace.error(&format!("ADPylon::imageGrabTask {e}"));
            }
        }
    }

    /// A parameter nothing has set yet reads as 0.
    async fn int32(&self, index: usize) -> i32 {
        self.port.read_int32(index, 0).await.unwrap_or(0)
    }

    async fn handle(&self, msg: Msg) -> asyn_rs::error::AsynResult<()> {
        match msg {
            Msg::Result { tag, outcome } => self.grab_result(tag, outcome).await?,
            Msg::Params(updates) => {
                self.port.set_params_and_notify(0, updates).await?;
            }
            Msg::Removed => {
                // The driver's disconnect closes the camera; the port's
                // auto-connect then keeps trying to reopen it.
                self.port
                    .submit_async(RequestOp::Disconnect, AsynUser::new(0).with_addr(-1))
                    .await?;
            }
        }
        Ok(())
    }

    /// C `processFrame` from the attributes on, then `imageGrabTask`'s check
    /// for the end of the acquisition. C finishes every result of a grab
    /// before `StopGrabbing` returns; here a result of an earlier grab can
    /// still be queued when the next one starts. It is published and
    /// counted as an array, but its acquisition is over: it leaves
    /// `NumImagesCounter`, `Acquire` and `Status` to the running one.
    async fn grab_result(&self, tag: GrabTag, outcome: Outcome) -> AsynResult<()> {
        let ad = &self.ad;
        let current = tag.grab == self.shared.current_grab();
        let done = match outcome {
            Outcome::Frame {
                array,
                layout,
                mut updates,
            } => {
                // C `getAttributes(pRaw->pAttributeList)`, between the chunk
                // data and the layout attributes.
                let mut array = Arc::new(array);
                let mut array = self
                    .port
                    .with_driver(move |driver: &mut ADPylon| {
                        driver.ad.attach_attributes(&mut array);
                        array
                    })
                    .await?;
                let attributes = &mut Arc::make_mut(&mut array).attributes;
                attributes.add(NDAttribute::new_static(
                    "ColorMode",
                    "Color mode",
                    NDAttrSource::Driver,
                    NDAttrValue::Int32(layout.0 as i32),
                ));
                attributes.add(NDAttribute::new_static(
                    "BayerPattern",
                    "Bayer Pattern",
                    NDAttrSource::Driver,
                    NDAttrValue::Int32(layout.1 as i32),
                ));

                let image_counter = self.int32(ad.base.array_counter).await + 1;
                updates.push(ParamSetValue::new(
                    ad.base.array_counter,
                    0,
                    ParamValue::Int32(image_counter),
                ));
                let num_images_counter = self.int32(ad.num_images_counter).await + 1;
                if current {
                    updates.push(ParamSetValue::new(
                        ad.num_images_counter,
                        0,
                        ParamValue::Int32(num_images_counter),
                    ));
                }
                if self.int32(ad.base.array_callbacks).await != 0 {
                    self.publisher.publish(array).await;
                }
                self.port.set_params_and_notify(0, updates).await?;

                // See if acquisition is done in single or multiple mode.
                let image_mode = ImageMode::from_i32(self.int32(ad.image_mode).await);
                let num_images = self.int32(ad.num_images).await;
                tag.last
                    || image_mode == ImageMode::Single
                    || (image_mode == ImageMode::Multiple && num_images_counter >= num_images)
            }
            Outcome::Failed { updates } => {
                self.port.set_params_and_notify(0, updates).await?;
                // No more results are coming.
                tag.last
            }
            Outcome::NoBuffer { mut updates } => {
                if current {
                    updates.push(ParamSetValue::new(
                        ad.status,
                        0,
                        ParamValue::Int32(ADStatus::Aborting as i32),
                    ));
                }
                self.port.set_params_and_notify(0, updates).await?;
                true
            }
        };
        if current && done {
            self.port.write_int32(ad.acquire, 0, 0).await?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bayer_pattern_follows_the_first_two_pixels() {
        assert_eq!(pixel_layout(PixelFormat::BayerRG8).2, NDBayerPattern::RGGB);
        assert_eq!(pixel_layout(PixelFormat::BayerGB16).2, NDBayerPattern::GBRG);
        assert_eq!(pixel_layout(PixelFormat::BayerGR8).2, NDBayerPattern::GRBG);
        assert_eq!(pixel_layout(PixelFormat::BayerBG16).2, NDBayerPattern::BGGR);
    }

    #[test]
    fn sixteen_bit_formats_are_uint16() {
        for p in [
            PixelFormat::Mono16,
            PixelFormat::Rgb16,
            PixelFormat::BayerRG16,
        ] {
            assert_eq!(pixel_layout(p).1, NDDataType::UInt16);
        }
        assert_eq!(pixel_layout(PixelFormat::Rgb8).0, NDColorMode::RGB1);
    }
}
