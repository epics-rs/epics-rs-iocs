//! Port of `NDDriverStdArrays` (NDDriverStdArrays.cpp).
//!
//! `NDDriverStdArrays` derives from `ADDriver` with `maxAddr = 1`,
//! `ASYN_CANBLOCK = 0` and `ASYN_MULTIDEVICE = 0`. Rather than an acquisition
//! thread it assembles NDArrays from waveform-record writes: each
//! `asynXXXArrayOut` write lands in `writeXXXArray`, which copies the samples
//! into `pArrays[0]` and (depending on the callback/append mode) publishes it.

use std::sync::Arc;

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use epics_rs::ad_core::attributes::{NDAttrSource, NDAttrValue, NDAttribute};
use epics_rs::ad_core::driver::{ADStatus, ImageMode};
use epics_rs::ad_core::ndarray::{NDArray, NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::NDArrayPool;
use epics_rs::ad_core::params::ad_driver::ADDriverParams;
use epics_rs::ad_core::plugin::channel::{
    ArrayPublisher, NDArrayOutput, NDArraySender, QueuedArrayCounter,
};
use epics_rs::ad_core::runtime as rt;
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::convert::{self, ND_ARRAY_MAX_DIMS};
use crate::params::{CallbackMode, NdsaParams};
use crate::task::{PublisherContext, start_publisher_task};

/// `DRIVER_VERSION.DRIVER_REVISION.DRIVER_MODIFICATION` (NDDriverStdArrays.h:18-20).
pub const DRIVER_VERSION: &str = "1.3.0";

/// Depth of the driver → publisher-task hand-off channel. C `doCallbacks`
/// delivers synchronously; this queue only buffers the brief window between the
/// write handler and the publisher task draining it. Sized generously so it is
/// never the drop point under the non-blocking (default) plugin callbacks; see
/// the deviation note in the crate docs.
const PUBLISH_QUEUE: usize = 1024;

const MEGABYTE: f64 = 1_048_576.0;

fn pool_err(e: impl std::fmt::Display) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: e.to_string(),
    }
}

pub struct NdStdArraysDriver {
    pub port_base: PortDriverBase,
    pub params: ADDriverParams,
    pub ndsa: NdsaParams,
    pool: Arc<NDArrayPool>,
    queued_counter: Arc<QueuedArrayCounter>,
    publish_tx: rt::CommandSender<Arc<NDArray>>,
    /// C `pArrays[0]` — the working buffer that waveform writes append into.
    current: Option<NDArray>,
    /// C `arrayDimensions_` — the requested dimensions, written via the
    /// `NDDimensions` (`ARRAY_DIMENSIONS`) waveform.
    array_dimensions: [usize; ND_ARRAY_MAX_DIMS],
    /// C `dimProd_` — the cumulative dimension product, recomputed on alloc.
    dim_prod: [usize; ND_ARRAY_MAX_DIMS],
}

impl NdStdArraysDriver {
    fn new(
        port_name: &str,
        max_memory: usize,
        pool: Arc<NDArrayPool>,
        queued_counter: Arc<QueuedArrayCounter>,
        publish_tx: rt::CommandSender<Arc<NDArray>>,
    ) -> AsynResult<Self> {
        // ASYN_CANBLOCK = 0, ASYN_MULTIDEVICE = 0, maxAddr = 1
        // (NDDriverStdArrays.cpp:47-52).
        let mut port_base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                can_block: false,
                multi_device: false,
                ..Default::default()
            },
        );

        let params = ADDriverParams::create(&mut port_base)?;
        let ndsa = NdsaParams::create(&mut port_base)?;

        // asynNDArrayDriver constructor defaults.
        port_base.set_int32_param(params.base.array_callbacks, 0, 1)?;
        port_base.set_int32_param(params.base.data_type, 0, NDDataType::UInt8 as u8 as i32)?;
        port_base.set_int32_param(params.base.color_mode, 0, 0)?; // NDColorMode::Mono
        port_base.set_int32_param(params.base.array_size_x, 0, 0)?;
        port_base.set_int32_param(params.base.array_size_y, 0, 0)?;
        port_base.set_int32_param(params.base.array_size_z, 0, 0)?;
        port_base.set_int32_param(params.base.array_size, 0, 0)?;
        port_base.set_int32_param(params.base.array_counter, 0, 0)?;
        port_base.set_float64_param(
            params.base.pool_max_memory,
            0,
            max_memory as f64 / MEGABYTE,
        )?;
        port_base.set_int32_param(params.status, 0, ADStatus::Idle as i32)?;

        // NDDriverStdArrays constructor (NDDriverStdArrays.cpp:70-86).
        port_base.set_string_param(params.base.manufacturer, 0, "NDDriverStdArrays".into())?;
        port_base.set_string_param(params.base.model, 0, "Software Detector".into())?;
        port_base.set_string_param(params.base.driver_version, 0, DRIVER_VERSION.into())?;
        port_base.set_string_param(params.base.sdk_version, 0, DRIVER_VERSION.into())?;
        port_base.set_string_param(params.base.serial_number, 0, "No serial number".into())?;
        port_base.set_string_param(params.base.firmware_version, 0, "No firmware".into())?;
        port_base.set_int32_param(params.image_mode, 0, ImageMode::Single as i32)?;
        port_base.set_int32_param(params.num_images, 0, 100)?;
        port_base.set_int32_param(ndsa.callback_mode, 0, CallbackMode::OnUpdate as i32)?;
        port_base.set_int32_param(ndsa.num_elements, 0, 0)?;
        port_base.set_int32_param(ndsa.next_element, 0, 0)?;
        port_base.set_int32_param(ndsa.stride, 0, 1)?;
        port_base.set_float64_param(ndsa.fill_value, 0, 0.0)?;
        port_base.set_int32_param(ndsa.new_array, 0, 1)?;
        port_base.set_int32_param(ndsa.array_complete, 0, 0)?;

        Ok(Self {
            port_base,
            params,
            ndsa,
            pool,
            queued_counter,
            publish_tx,
            current: None,
            array_dimensions: [0; ND_ARRAY_MAX_DIMS],
            dim_prod: [0; ND_ARRAY_MAX_DIMS],
        })
    }

    fn get_i32(&self, reason: usize, default: i32) -> i32 {
        self.port_base.get_int32_param(reason, 0).unwrap_or(default)
    }

    /// `asynNDArrayDriver::setIntegerParam` (asynNDArrayDriver.cpp:636-663):
    /// writes to `ADAcquire`/`NDNumQueuedArrays` are intercepted to drive
    /// `ADAcquireBusy`.
    fn set_integer_param(&mut self, addr: i32, reason: usize, value: i32) -> AsynResult<()> {
        if reason == self.params.acquire {
            if value == 0 {
                let wait_for_plugins = self
                    .port_base
                    .get_int32_param(self.params.wait_for_plugins, addr)
                    .unwrap_or(0)
                    != 0;
                if !wait_for_plugins || self.queued_counter.get() == 0 {
                    self.port_base
                        .set_int32_param(self.params.acquire_busy, addr, 0)?;
                }
            } else {
                self.port_base
                    .set_int32_param(self.params.acquire_busy, addr, 1)?;
            }
        } else if reason == self.params.base.num_queued_arrays
            && value == 0
            && self
                .port_base
                .get_int32_param(self.params.acquire, addr)
                .unwrap_or(0)
                == 0
        {
            self.port_base
                .set_int32_param(self.params.acquire_busy, addr, 0)?;
        }
        self.port_base.params.set_int32(reason, addr, value)
    }

    /// The allocation half of `writeXXXArray` (NDDriverStdArrays.cpp:153-218):
    /// release the previous array, allocate a fresh one sized by
    /// `arrayDimensions_`, publish its size parameters, recompute `dimProd_`,
    /// and fill it when required.
    fn allocate_array(
        &mut self,
        num_dimensions: usize,
        data_type: NDDataType,
        color_mode: i32,
        append_mode: i32,
        n_elements: usize,
    ) -> AsynResult<()> {
        self.port_base.set_int32_param(self.ndsa.new_array, 0, 0)?;

        let ndims = num_dimensions.min(ND_ARRAY_MAX_DIMS);
        let dims: Vec<NDDimension> = (0..ndims)
            .map(|i| NDDimension::new(self.array_dimensions[i]))
            .collect();
        let mut array = self.pool.alloc(dims, data_type).map_err(pool_err)?;
        // pArray->pAttributeList->add("ColorMode", ...) (NDDriverStdArrays.cpp:167).
        array.attributes.add(NDAttribute::new_static(
            "ColorMode",
            "Color Mode",
            NDAttrSource::Driver,
            NDAttrValue::Int32(color_mode),
        ));
        let info = array.info();
        self.port_base
            .set_int32_param(self.params.base.array_size, 0, info.total_bytes as i32)?;
        self.port_base
            .set_int32_param(self.ndsa.num_elements, 0, info.num_elements as i32)?;
        self.port_base.set_int32_param(
            self.params.max_size_x,
            0,
            self.array_dimensions[0] as i32,
        )?;
        self.port_base.set_int32_param(
            self.params.max_size_y,
            0,
            self.array_dimensions[1] as i32,
        )?;
        self.port_base
            .set_int32_param(self.params.base.array_size_x, 0, 0)?;
        self.port_base
            .set_int32_param(self.params.base.array_size_y, 0, 0)?;
        self.port_base
            .set_int32_param(self.params.base.array_size_z, 0, 0)?;

        self.dim_prod = convert::dim_prod(&self.array_dimensions, num_dimensions);

        // NDDriverStdArrays.cpp:184-214.
        if append_mode == 1 || (append_mode == 0 && info.num_elements < n_elements) {
            let fill = self
                .port_base
                .get_float64_param(self.ndsa.fill_value, 0)
                .unwrap_or(0.0);
            convert::fill_buffer(&mut array.data, fill);
        }
        if append_mode == 0 {
            self.port_base
                .set_int32_param(self.ndsa.next_element, 0, 0)?;
        }

        self.current = Some(array);
        Ok(())
    }

    /// `writeXXXArray` (NDDriverStdArrays.cpp:120-279) with the input samples
    /// already promoted to `f64`.
    fn write_array_data(&mut self, input: &[f64]) -> AsynResult<()> {
        let n_elements = input.len();

        // NDDriverStdArrays.cpp:141-142: silently ignore writes when not
        // acquiring.
        if self.get_i32(self.params.acquire, 0) == 0 {
            return Ok(());
        }

        let data_type_code =
            self.get_i32(self.params.base.data_type, NDDataType::UInt8 as u8 as i32);
        let data_type = match u8::try_from(data_type_code)
            .ok()
            .and_then(NDDataType::from_ordinal)
        {
            Some(dt) => dt,
            None => {
                // C's `switch (dataType)` has no default; there is no Rust
                // equivalent of allocating an array of an unknown type, so the
                // write is dropped.
                log::error!("NDDriverStdArrays: invalid NDDataType {data_type_code}");
                return Ok(());
            }
        };
        let color_mode = self.get_i32(self.params.base.color_mode, 0);
        let callback_mode = self.get_i32(self.ndsa.callback_mode, 0);
        let append_mode = self.get_i32(self.ndsa.append_mode, 0);
        let num_dimensions = self.get_i32(self.params.base.n_dimensions, 0).max(0) as usize;
        let new_array = self.get_i32(self.ndsa.new_array, 0);
        let array_callbacks = self.get_i32(self.params.base.array_callbacks, 0);

        if append_mode == 0 || (append_mode == 1 && new_array != 0) {
            self.allocate_array(
                num_dimensions,
                data_type,
                color_mode,
                append_mode,
                n_elements,
            )?;
        }

        // arrayInfo_.nElements is the current buffer's element count.
        let total = self.current.as_ref().map(|a| a.data.len()).unwrap_or(0);
        let next_element = self.get_i32(self.ndsa.next_element, 0).max(0) as usize;

        // NDDriverStdArrays.cpp:221-223: clamp the write to the buffer tail.
        let mut n = n_elements;
        if next_element + n >= total {
            n = total.saturating_sub(next_element);
        }
        let stride = self.get_i32(self.ndsa.stride, 1).max(0) as usize;

        if let Some(array) = self.current.as_mut() {
            convert::copy_buffer(&mut array.data, next_element, stride, &input[..n]);
        }

        let new_next = next_element + n;
        self.port_base
            .set_int32_param(self.ndsa.next_element, 0, new_next as i32)?;

        // NDDriverStdArrays.cpp:256-266: post the current write index back onto
        // the NDDimensions (ARRAY_DIMENSIONS) parameter.
        let current_index = convert::current_index(
            new_next as i32,
            num_dimensions,
            &self.dim_prod,
            &self.array_dimensions,
        );
        self.port_base.params.set_int32_array(
            self.params.base.array_dimensions,
            0,
            current_index.to_vec(),
        )?;

        if append_mode == 0 {
            self.set_array_complete()?;
        }
        if array_callbacks != 0
            && append_mode == 1
            && callback_mode == CallbackMode::OnUpdate as i32
        {
            self.do_callbacks()?;
        }

        self.port_base.call_param_callbacks(0)?;
        Ok(())
    }

    /// `setArrayComplete` (NDDriverStdArrays.cpp:281-308).
    fn set_array_complete(&mut self) -> AsynResult<()> {
        let num_images = self.get_i32(self.params.num_images, 0);
        let num_images_counter = self.get_i32(self.params.num_images_counter, 0) + 1;
        let callback_mode = self.get_i32(self.ndsa.callback_mode, 0);
        let image_mode = self.get_i32(self.params.image_mode, 0);
        let array_callbacks = self.get_i32(self.params.base.array_callbacks, 0);

        self.port_base
            .set_int32_param(self.params.num_images_counter, 0, num_images_counter)?;

        if array_callbacks != 0 && callback_mode != CallbackMode::OnCommand as i32 {
            self.do_callbacks()?;
        }

        if image_mode == ImageMode::Single as i32
            || (image_mode == ImageMode::Multiple as i32 && num_images_counter >= num_images)
        {
            // setIntegerParam(ADAcquire, 0) routes through the intercept so
            // ADAcquireBusy is cleared.
            self.set_integer_param(0, self.params.acquire, 0)?;
            self.port_base
                .set_int32_param(self.params.status, 0, ADStatus::Idle as i32)?;
        }
        Ok(())
    }

    /// `doCallbacks` (NDDriverStdArrays.cpp:310-329): bump `NDArrayCounter`,
    /// stamp identity and timestamps, and hand the array to the publisher task.
    fn do_callbacks(&mut self) -> AsynResult<()> {
        if self.current.is_none() {
            return Ok(());
        }
        let counter = self.get_i32(self.params.base.array_counter, 0) + 1;
        self.port_base
            .set_int32_param(self.params.base.array_counter, 0, counter)?;

        // DEVIATION: C stamps `pArrays[0]` itself and hands the live pointer to
        // the plugin, so append-mode re-publishes share one growing buffer.
        // This port snapshots the buffer per publish, giving each queued frame
        // an independent copy (identical observable data, no shared-buffer race).
        let mut snapshot = self.current.as_ref().unwrap().clone();
        snapshot.unique_id = counter;
        let now = EpicsTimestamp::now();
        snapshot.time_stamp = now.sec as f64 + now.nsec as f64 / 1.0e9;
        snapshot.timestamp = now;
        // DEVIATION: C calls getAttributes(pArray->pAttributeList) here; the
        // NDAttributesFile machinery is not wired for this driver, so only the
        // ColorMode attribute added at allocation is attached.

        if self.publish_tx.try_send(Arc::new(snapshot)).is_err() {
            log::warn!("NDDriverStdArrays: publish queue full or closed; frame dropped");
        }
        Ok(())
    }

    /// The pool branch of `asynNDArrayDriver::writeInt32`
    /// (asynNDArrayDriver.cpp:684-694), reproduced over the public
    /// `NDArrayPool` API. `pArrays[0]` is the pre-allocation template.
    fn write_int32_pool(&mut self, reason: usize) -> AsynResult<bool> {
        if reason == self.params.base.pool_empty_free_list {
            self.pool.empty_free_list();
            self.refresh_pool_stats()?;
            Ok(true)
        } else if reason == self.params.base.pool_poll_stats {
            self.refresh_pool_stats()?;
            Ok(true)
        } else if reason == self.params.base.pool_pre_alloc {
            if let Some(template) = self.current.clone() {
                let count = self
                    .get_i32(self.params.base.pool_num_pre_alloc_buffers, 0)
                    .max(0) as usize;
                self.pool
                    .pre_allocate_buffers(&template, count)
                    .map_err(pool_err)?;
                self.refresh_pool_stats()?;
            }
            self.port_base
                .set_int32_param(self.params.base.pool_pre_alloc, 0, 0)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn refresh_pool_stats(&mut self) -> AsynResult<()> {
        let max_memory = self.pool.max_memory() as f64 / MEGABYTE;
        let used = self.pool.allocated_bytes() as f64 / MEGABYTE;
        let alloc = self.pool.num_alloc_buffers() as i32;
        let free = self.pool.num_free_buffers() as i32;
        self.port_base
            .set_float64_param(self.params.base.pool_max_memory, 0, max_memory)?;
        self.port_base
            .set_float64_param(self.params.base.pool_used_memory, 0, used)?;
        self.port_base
            .set_int32_param(self.params.base.pool_alloc_buffers, 0, alloc)?;
        self.port_base
            .set_int32_param(self.params.base.pool_free_buffers, 0, free)?;
        Ok(())
    }

    fn write_array_input(&mut self, input: Vec<f64>) -> AsynResult<()> {
        self.write_array_data(&input)
    }
}

impl PortDriver for NdStdArraysDriver {
    fn base(&self) -> &PortDriverBase {
        &self.port_base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.port_base
    }

    /// `NDDriverStdArrays::writeInt32` (NDDriverStdArrays.cpp:336-384).
    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let reason = user.reason;
        let addr = user.addr;

        // setIntegerParam(function, value) (NDDriverStdArrays.cpp:346) — the
        // ADAcquire/NDNumQueuedArrays intercept lives here.
        self.set_integer_param(addr, reason, value)?;

        if reason == self.params.acquire {
            if value == 1 {
                self.port_base
                    .set_int32_param(self.params.num_images_counter, 0, 0)?;
            }
        } else if reason == self.ndsa.do_callbacks {
            self.do_callbacks()?;
        } else if reason == self.ndsa.new_array {
            self.port_base
                .set_int32_param(self.ndsa.next_element, 0, 0)?;
        } else if reason == self.ndsa.array_complete {
            if self.get_i32(self.ndsa.append_mode, 0) != 0 {
                self.set_array_complete()?;
            }
        } else if self.ndsa.belongs_to_base(reason) {
            // ADDriver::writeInt32 — only the pool operations act.
            self.write_int32_pool(reason)?;
        }

        self.port_base.call_param_callbacks(addr)?;
        Ok(())
    }

    /// `writeInt32Array` (NDDriverStdArrays.cpp:396-411): dispatches between the
    /// `NDDimensions` dimension write and `NDSA_ArrayData_` array data.
    fn write_int32_array(&mut self, user: &AsynUser, data: &[i32]) -> AsynResult<()> {
        if user.reason == self.params.base.array_dimensions {
            for (slot, &v) in self
                .array_dimensions
                .iter_mut()
                .zip(data.iter())
                .take(ND_ARRAY_MAX_DIMS)
            {
                *slot = v.max(0) as usize;
            }
            Ok(())
        } else if user.reason == self.ndsa.array_data {
            self.write_array_input(data.iter().map(|&v| v as f64).collect())
        } else {
            Ok(())
        }
    }

    fn write_int8_array(&mut self, _user: &AsynUser, data: &[i8]) -> AsynResult<()> {
        self.write_array_input(data.iter().map(|&v| v as f64).collect())
    }

    fn write_int16_array(&mut self, _user: &AsynUser, data: &[i16]) -> AsynResult<()> {
        self.write_array_input(data.iter().map(|&v| v as f64).collect())
    }

    fn write_float32_array(&mut self, _user: &AsynUser, data: &[f32]) -> AsynResult<()> {
        self.write_array_input(data.iter().map(|&v| v as f64).collect())
    }

    fn write_float64_array(&mut self, _user: &AsynUser, data: &[f64]) -> AsynResult<()> {
        self.write_array_input(data.to_vec())
    }
}

/// Handles kept by the IOC after the driver has been moved into its runtime.
pub struct NdStdArraysRuntime {
    pub runtime_handle: PortRuntimeHandle,
    pub params: ADDriverParams,
    pub ndsa: NdsaParams,
    pool: Arc<NDArrayPool>,
    output: Arc<parking_lot::Mutex<NDArrayOutput>>,
    queued_counter: Arc<QueuedArrayCounter>,
    /// Owns the publisher thread for the lifetime of the IOC.
    pub task_handle: std::thread::JoinHandle<()>,
}

impl NdStdArraysRuntime {
    pub fn port_handle(&self) -> &PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn pool(&self) -> &Arc<NDArrayPool> {
        &self.pool
    }

    /// The single `NDArrayOutput` (asyn address 0) fed to downstream plugins.
    pub fn array_output(&self) -> &Arc<parking_lot::Mutex<NDArrayOutput>> {
        &self.output
    }

    pub fn connect_downstream(&self, mut sender: NDArraySender) {
        sender.set_queued_counter(self.queued_counter.clone());
        self.output.lock().add(sender);
    }
}

/// `NDDriverStdArraysConfig` (NDDriverStdArrays.cpp:424-431).
///
/// DEVIATION: C's `maxBuffers` caps the number of NDArrays the pool may
/// allocate. `ad-core-rs` 0.22.1's `NDArrayPool` is bounded by `maxMemory`
/// only, so `max_buffers` is accepted for iocsh signature parity and ignored.
pub fn create_nd_std_arrays(
    port_name: &str,
    _max_buffers: i32,
    max_memory: usize,
) -> AsynResult<NdStdArraysRuntime> {
    let pool = Arc::new(NDArrayPool::new(max_memory));
    let queued_counter = Arc::new(QueuedArrayCounter::new());
    let (publish_tx, publish_rx) = rt::command_channel::<Arc<NDArray>>(PUBLISH_QUEUE);

    let driver = NdStdArraysDriver::new(
        port_name,
        max_memory,
        pool.clone(),
        queued_counter.clone(),
        publish_tx,
    )?;
    let params = driver.params;
    let ndsa = driver.ndsa;

    let (runtime_handle, _) = create_port_runtime(driver, RuntimeConfig::default());

    let output = Arc::new(parking_lot::Mutex::new(NDArrayOutput::new()));
    let publisher = ArrayPublisher::new(output.clone());
    let task_handle = start_publisher_task(PublisherContext {
        rx: publish_rx,
        publisher,
    });

    Ok(NdStdArraysRuntime {
        runtime_handle,
        params,
        ndsa,
        pool,
        output,
        queued_counter,
        task_handle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::NDDataBuffer;

    struct Fixture {
        det: NdStdArraysDriver,
        publish_rx: rt::CommandReceiver<Arc<NDArray>>,
    }

    impl Fixture {
        fn new() -> Self {
            let pool = Arc::new(NDArrayPool::new(0));
            let queued_counter = Arc::new(QueuedArrayCounter::new());
            let (publish_tx, publish_rx) = rt::command_channel::<Arc<NDArray>>(PUBLISH_QUEUE);
            let det =
                NdStdArraysDriver::new("NDSATEST", 0, pool, queued_counter, publish_tx).unwrap();
            Self { det, publish_rx }
        }

        fn set_i32(&mut self, reason: usize, value: i32) {
            self.det
                .port_base
                .set_int32_param(reason, 0, value)
                .unwrap();
        }

        fn get_i32(&self, reason: usize) -> i32 {
            self.det.port_base.get_int32_param(reason, 0).unwrap()
        }

        fn write_i32(&mut self, reason: usize, value: i32) {
            let mut user = AsynUser::new(reason);
            self.det.write_int32(&mut user, value).unwrap();
        }

        fn write_dims(&mut self, dims: &[i32]) {
            let user = AsynUser::new(self.det.params.base.array_dimensions);
            self.det.write_int32_array(&user, dims).unwrap();
        }

        fn write_f64_array(&mut self, data: &[f64]) {
            let user = AsynUser::new(self.det.ndsa.array_data);
            self.det.write_float64_array(&user, data).unwrap();
        }

        fn drain_publish(&mut self) -> Vec<Arc<NDArray>> {
            let mut out = Vec::new();
            while let Ok(a) = self.publish_rx.try_recv() {
                out.push(a);
            }
            out
        }
    }

    fn f64s(a: &NDArray) -> Vec<f64> {
        match &a.data {
            NDDataBuffer::F64(v) => v.clone(),
            other => panic!("expected F64, got {:?}", other.data_type()),
        }
    }

    #[test]
    fn constructor_defaults_match_the_c_constructor() {
        let f = Fixture::new();
        assert_eq!(f.get_i32(f.det.params.image_mode), ImageMode::Single as i32);
        assert_eq!(f.get_i32(f.det.params.num_images), 100);
        assert_eq!(
            f.get_i32(f.det.ndsa.callback_mode),
            CallbackMode::OnUpdate as i32
        );
        assert_eq!(f.get_i32(f.det.ndsa.next_element), 0);
        assert_eq!(f.get_i32(f.det.ndsa.stride), 1);
        assert_eq!(f.get_i32(f.det.ndsa.new_array), 1);
        assert_eq!(f.get_i32(f.det.ndsa.array_complete), 0);
        assert_eq!(
            f.det
                .port_base
                .get_float64_param(f.det.ndsa.fill_value, 0)
                .unwrap(),
            0.0
        );
        assert_eq!(
            f.det
                .port_base
                .get_string_param(f.det.params.base.driver_version, 0)
                .unwrap(),
            "1.3.0"
        );
        assert_eq!(
            f.det
                .port_base
                .get_string_param(f.det.params.base.manufacturer, 0)
                .unwrap(),
            "NDDriverStdArrays"
        );
    }

    #[test]
    fn array_writes_are_ignored_until_acquire_is_set() {
        let mut f = Fixture::new();
        f.set_i32(
            f.det.params.base.data_type,
            NDDataType::Float64 as u8 as i32,
        );
        f.set_i32(f.det.params.base.n_dimensions, 1);
        f.write_dims(&[4]);
        // Not acquiring: the write is a no-op and nothing is published.
        f.write_f64_array(&[1.0, 2.0, 3.0, 4.0]);
        assert!(f.det.current.is_none());
        assert!(f.drain_publish().is_empty());
    }

    #[test]
    fn a_non_append_write_allocates_and_publishes_one_array() {
        let mut f = Fixture::new();
        f.set_i32(
            f.det.params.base.data_type,
            NDDataType::Float64 as u8 as i32,
        );
        f.set_i32(f.det.params.base.n_dimensions, 1);
        f.set_i32(f.det.ndsa.append_mode, 0);
        // A non-zero fill value that must NOT appear: in non-append mode C only
        // fills when the array is smaller than the incoming data
        // (arrayInfo_.nElements < nElements), which is not the case here.
        f.det
            .port_base
            .set_float64_param(f.det.ndsa.fill_value, 0, -1.0)
            .unwrap();
        f.write_dims(&[6]);
        f.write_i32(f.det.params.acquire, 1);

        // Only four samples for a six-element array: the array (6) is larger
        // than the data (4), so no fill runs and the tail keeps the pool zero.
        f.write_f64_array(&[10.0, 11.0, 12.0, 13.0]);

        let published = f.drain_publish();
        assert_eq!(published.len(), 1, "non-append publishes once per write");
        let arr = &published[0];
        assert_eq!(arr.dims.len(), 1);
        assert_eq!(arr.dims[0].size, 6);
        assert_eq!(f64s(arr), vec![10.0, 11.0, 12.0, 13.0, 0.0, 0.0]);
        assert_eq!(arr.unique_id, 1, "NDArrayCounter starts at 1");

        // Single image mode: acquisition self-clears after one complete array.
        assert_eq!(f.get_i32(f.det.params.acquire), 0);
        assert_eq!(f.get_i32(f.det.params.status), ADStatus::Idle as i32);
    }

    #[test]
    fn append_mode_fills_the_buffer_before_the_first_write() {
        let mut f = Fixture::new();
        f.set_i32(
            f.det.params.base.data_type,
            NDDataType::Float64 as u8 as i32,
        );
        f.set_i32(f.det.params.base.n_dimensions, 1);
        f.set_i32(f.det.ndsa.append_mode, 1);
        f.set_i32(f.det.params.image_mode, ImageMode::Continuous as i32);
        // Append mode always fills the freshly allocated buffer, so the tail
        // not yet written carries the fill value.
        f.det
            .port_base
            .set_float64_param(f.det.ndsa.fill_value, 0, -7.0)
            .unwrap();
        f.write_dims(&[6]);
        f.write_i32(f.det.params.acquire, 1);

        f.write_f64_array(&[1.0, 2.0, 3.0]);
        let published = f.drain_publish();
        assert_eq!(published.len(), 1);
        assert_eq!(f64s(&published[0]), vec![1.0, 2.0, 3.0, -7.0, -7.0, -7.0]);
    }

    #[test]
    fn append_mode_accumulates_across_writes_into_one_array() {
        let mut f = Fixture::new();
        f.set_i32(
            f.det.params.base.data_type,
            NDDataType::Float64 as u8 as i32,
        );
        f.set_i32(f.det.params.base.n_dimensions, 1);
        f.set_i32(f.det.ndsa.append_mode, 1);
        f.set_i32(f.det.params.image_mode, ImageMode::Continuous as i32);
        f.write_dims(&[6]);
        f.write_i32(f.det.params.acquire, 1);

        // First append allocates (NewArray defaults to 1) and writes 0..3.
        f.write_f64_array(&[1.0, 2.0, 3.0]);
        assert_eq!(f.get_i32(f.det.ndsa.next_element), 3);
        // Second append continues at element 3.
        f.write_f64_array(&[4.0, 5.0, 6.0]);
        assert_eq!(f.get_i32(f.det.ndsa.next_element), 6);

        let published = f.drain_publish();
        // OnUpdate callback mode: one publish per write.
        assert_eq!(published.len(), 2);
        assert_eq!(f64s(&published[1]), vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        // Continuous mode never self-clears acquire.
        assert_eq!(f.get_i32(f.det.params.acquire), 1);
    }

    #[test]
    fn on_command_append_publishes_only_on_the_do_callbacks_command() {
        let mut f = Fixture::new();
        f.set_i32(
            f.det.params.base.data_type,
            NDDataType::Float64 as u8 as i32,
        );
        f.set_i32(f.det.params.base.n_dimensions, 1);
        f.set_i32(f.det.ndsa.append_mode, 1);
        f.set_i32(f.det.ndsa.callback_mode, CallbackMode::OnCommand as i32);
        f.set_i32(f.det.params.image_mode, ImageMode::Continuous as i32);
        f.write_dims(&[4]);
        f.write_i32(f.det.params.acquire, 1);

        f.write_f64_array(&[1.0, 2.0]);
        f.write_f64_array(&[3.0, 4.0]);
        assert!(
            f.drain_publish().is_empty(),
            "OnCommand suppresses update callbacks"
        );

        // The explicit DoCallbacks command publishes the assembled array.
        f.write_i32(f.det.ndsa.do_callbacks, 1);
        let published = f.drain_publish();
        assert_eq!(published.len(), 1);
        assert_eq!(f64s(&published[0]), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn new_array_command_resets_the_write_cursor() {
        let mut f = Fixture::new();
        f.set_i32(f.det.ndsa.next_element, 7);
        f.write_i32(f.det.ndsa.new_array, 1);
        assert_eq!(f.get_i32(f.det.ndsa.next_element), 0);
    }

    #[test]
    fn dimension_writes_are_stored_but_do_not_touch_the_param_cache() {
        let mut f = Fixture::new();
        f.write_dims(&[3, 5]);
        assert_eq!(f.det.array_dimensions[0], 3);
        assert_eq!(f.det.array_dimensions[1], 5);
    }

    #[test]
    fn starting_acquisition_raises_busy_and_resets_the_image_counter() {
        let mut f = Fixture::new();
        f.set_i32(f.det.params.num_images_counter, 9);
        f.write_i32(f.det.params.acquire, 1);
        assert_eq!(f.get_i32(f.det.params.acquire_busy), 1);
        assert_eq!(f.get_i32(f.det.params.num_images_counter), 0);
    }
}
