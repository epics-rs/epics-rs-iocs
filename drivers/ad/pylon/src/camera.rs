//! Safe wrapper over the pylon shim: one `CBaslerUniversalInstantCamera` with
//! its image, configuration and camera-event handlers.

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};

use crate::ffi;

pub type PylonResult<T> = Result<T, String>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeMap {
    Camera,
    /// The stream grabber's node map, home of the transport statistics.
    Stream,
}

impl NodeMap {
    fn raw(self) -> c_int {
        match self {
            Self::Camera => ffi::MAP_CAMERA,
            Self::Stream => ffi::MAP_STREAM,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeKind {
    Integer,
    Boolean,
    Enum,
    Float,
    String,
    Command,
    Other,
}

impl NodeKind {
    fn from_raw(raw: c_int) -> Option<Self> {
        Some(match raw {
            ffi::NODE_ABSENT => return None,
            ffi::NODE_INTEGER => Self::Integer,
            ffi::NODE_BOOLEAN => Self::Boolean,
            ffi::NODE_ENUM => Self::Enum,
            ffi::NODE_FLOAT => Self::Float,
            ffi::NODE_STRING => Self::String,
            ffi::NODE_COMMAND => Self::Command,
            _ => Self::Other,
        })
    }
}

/// Pixel formats the driver can turn into an NDArray; the discriminants are
/// the shim's `SHIM_PIXEL_*`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum PixelFormat {
    Mono8 = 0,
    Mono16,
    Rgb8,
    Rgb16,
    BayerBG8,
    BayerGB8,
    BayerGR8,
    BayerRG8,
    BayerBG16,
    BayerGB16,
    BayerGR16,
    BayerRG16,
}

impl PixelFormat {
    const ALL: [Self; 12] = [
        Self::Mono8,
        Self::Mono16,
        Self::Rgb8,
        Self::Rgb16,
        Self::BayerBG8,
        Self::BayerGB8,
        Self::BayerGR8,
        Self::BayerRG8,
        Self::BayerBG16,
        Self::BayerGB16,
        Self::BayerGR16,
        Self::BayerRG16,
    ];

    fn from_raw(raw: c_int) -> Option<Self> {
        Self::ALL.into_iter().find(|p| *p as c_int == raw)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ChunkValue {
    Integer(i64),
    Float(f64),
    Boolean(bool),
    String(String),
}

#[derive(Clone, Debug)]
pub struct Chunk {
    pub name: String,
    pub display_name: String,
    pub value: ChunkValue,
}

/// A grabbed image, valid for the duration of `CameraHandler::on_frame`.
pub struct Frame<'a> {
    pub width: usize,
    pub height: usize,
    /// `None` when the pixel type has no NDArray representation.
    pub pixel: Option<PixelFormat>,
    pub raw_pixel_type: i64,
    pub data: &'a [u8],
    pub id: i64,
    pub timestamp: u64,
    /// The requested pixel conversion failed; `data` is unconverted.
    pub convert_error: Option<String>,
    chunks: *mut c_void,
}

impl Frame<'_> {
    pub fn chunks(&self) -> Vec<Chunk> {
        extern "C" fn sink(
            user: *mut c_void,
            name: *const c_char,
            display: *const c_char,
            kind: c_int,
            ival: i64,
            dval: f64,
            sval: *const c_char,
        ) {
            let out = unsafe { &mut *(user as *mut Vec<Chunk>) };
            let value = match kind {
                ffi::NODE_FLOAT => ChunkValue::Float(dval),
                ffi::NODE_BOOLEAN => ChunkValue::Boolean(ival != 0),
                ffi::NODE_STRING => ChunkValue::String(lossy(sval)),
                _ => ChunkValue::Integer(ival),
            };
            out.push(Chunk {
                name: lossy(name),
                display_name: lossy(display),
                value,
            });
        }
        let mut out: Vec<Chunk> = Vec::new();
        unsafe { ffi::pylon_frame_chunks(self.chunks, sink, &mut out as *mut _ as *mut c_void) };
        out
    }
}

/// Camera callbacks. `on_frame` and `on_event` run on pylon's grab-loop
/// thread, which `Camera::stop` and `Camera::close` wait for: a handler that
/// blocks on whoever is calling those deadlocks.
pub trait CameraHandler: Send + Sync + 'static {
    fn on_frame(&self, nodes: &Nodes, frame: PylonResult<Frame<'_>>);
    fn on_event(&self, nodes: &Nodes, event_id: i32);
    fn on_removed(&self);
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub friendly_name: String,
    pub model: String,
    pub serial: String,
    pub interface_id: String,
}

fn lossy(s: *const c_char) -> String {
    if s.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(s) }.to_string_lossy().into_owned()
}

fn last_error() -> String {
    lossy(unsafe { ffi::pylon_shim_last_error() })
}

fn check(rc: c_int) -> PylonResult<()> {
    if rc == 0 { Ok(()) } else { Err(last_error()) }
}

fn cstring(s: &str) -> PylonResult<CString> {
    CString::new(s).map_err(|_| format!("embedded NUL in {s:?}"))
}

extern "C" fn string_sink(user: *mut c_void, s: *const c_char) {
    unsafe { *(user as *mut String) = lossy(s) };
}

pub fn sdk_version() -> String {
    let mut out = String::new();
    unsafe { ffi::pylon_shim_version(string_sink, &mut out as *mut _ as *mut c_void) };
    out
}

/// Node access on an open camera.
pub struct Nodes {
    raw: *mut ffi::PylonCam,
}

// SAFETY: the pylon instant camera and GenApi node maps are thread-safe.
unsafe impl Send for Nodes {}
unsafe impl Sync for Nodes {}

impl Nodes {
    /// `None` when the node does not exist or the camera is not open.
    pub fn kind(&self, map: NodeMap, name: &str) -> Option<NodeKind> {
        let name = cstring(name).ok()?;
        NodeKind::from_raw(unsafe { ffi::pylon_node_type(self.raw, map.raw(), name.as_ptr()) })
    }

    /// (available, readable, writable); all false for an absent node.
    pub fn access(&self, map: NodeMap, name: &str) -> (bool, bool, bool) {
        let Ok(name) = cstring(name) else {
            return (false, false, false);
        };
        let (mut a, mut r, mut w) = (0, 0, 0);
        unsafe {
            ffi::pylon_node_access(self.raw, map.raw(), name.as_ptr(), &mut a, &mut r, &mut w)
        };
        (a != 0, r != 0, w != 0)
    }

    pub fn get_int(&self, map: NodeMap, name: &str) -> PylonResult<i64> {
        let name = cstring(name)?;
        let mut v = 0;
        let null = std::ptr::null_mut();
        check(unsafe {
            ffi::pylon_node_get_int(self.raw, map.raw(), name.as_ptr(), &mut v, null, null, null)
        })?;
        Ok(v)
    }

    /// (min, max, increment)
    pub fn int_range(&self, map: NodeMap, name: &str) -> PylonResult<(i64, i64, i64)> {
        let name = cstring(name)?;
        let (mut min, mut max, mut inc) = (0, 0, 0);
        check(unsafe {
            ffi::pylon_node_get_int(
                self.raw,
                map.raw(),
                name.as_ptr(),
                std::ptr::null_mut(),
                &mut min,
                &mut max,
                &mut inc,
            )
        })?;
        Ok((min, max, inc))
    }

    pub fn set_int(&self, map: NodeMap, name: &str, value: i64) -> PylonResult<()> {
        let name = cstring(name)?;
        check(unsafe { ffi::pylon_node_set_int(self.raw, map.raw(), name.as_ptr(), value) })
    }

    pub fn get_float(&self, map: NodeMap, name: &str) -> PylonResult<f64> {
        let name = cstring(name)?;
        let mut v = 0.0;
        let null = std::ptr::null_mut();
        check(unsafe {
            ffi::pylon_node_get_float(self.raw, map.raw(), name.as_ptr(), &mut v, null, null)
        })?;
        Ok(v)
    }

    /// (min, max)
    pub fn float_range(&self, map: NodeMap, name: &str) -> PylonResult<(f64, f64)> {
        let name = cstring(name)?;
        let (mut min, mut max) = (0.0, 0.0);
        check(unsafe {
            ffi::pylon_node_get_float(
                self.raw,
                map.raw(),
                name.as_ptr(),
                std::ptr::null_mut(),
                &mut min,
                &mut max,
            )
        })?;
        Ok((min, max))
    }

    pub fn set_float(&self, map: NodeMap, name: &str, value: f64) -> PylonResult<()> {
        let name = cstring(name)?;
        check(unsafe { ffi::pylon_node_set_float(self.raw, map.raw(), name.as_ptr(), value) })
    }

    pub fn get_bool(&self, map: NodeMap, name: &str) -> PylonResult<bool> {
        let name = cstring(name)?;
        let mut v = 0;
        check(unsafe { ffi::pylon_node_get_bool(self.raw, map.raw(), name.as_ptr(), &mut v) })?;
        Ok(v != 0)
    }

    pub fn set_bool(&self, map: NodeMap, name: &str, value: bool) -> PylonResult<()> {
        let name = cstring(name)?;
        check(unsafe {
            ffi::pylon_node_set_bool(self.raw, map.raw(), name.as_ptr(), value as c_int)
        })
    }

    /// The enumeration's integer value, not an index into its entries.
    pub fn get_enum(&self, map: NodeMap, name: &str) -> PylonResult<i64> {
        let name = cstring(name)?;
        let mut v = 0;
        check(unsafe { ffi::pylon_node_get_enum(self.raw, map.raw(), name.as_ptr(), &mut v) })?;
        Ok(v)
    }

    pub fn set_enum(&self, map: NodeMap, name: &str, value: i64) -> PylonResult<()> {
        let name = cstring(name)?;
        check(unsafe { ffi::pylon_node_set_enum(self.raw, map.raw(), name.as_ptr(), value) })
    }

    pub fn get_enum_symbolic(&self, map: NodeMap, name: &str) -> PylonResult<String> {
        let name = cstring(name)?;
        let mut out = String::new();
        check(unsafe {
            ffi::pylon_node_get_enum_symbolic(
                self.raw,
                map.raw(),
                name.as_ptr(),
                string_sink,
                &mut out as *mut _ as *mut c_void,
            )
        })?;
        Ok(out)
    }

    pub fn enum_entries(
        &self,
        map: NodeMap,
        name: &str,
        settable_only: bool,
    ) -> PylonResult<Vec<(String, i64)>> {
        extern "C" fn sink(user: *mut c_void, symbolic: *const c_char, value: i64) {
            let out = unsafe { &mut *(user as *mut Vec<(String, i64)>) };
            out.push((lossy(symbolic), value));
        }
        let name = cstring(name)?;
        let mut out: Vec<(String, i64)> = Vec::new();
        check(unsafe {
            ffi::pylon_node_enum_entries(
                self.raw,
                map.raw(),
                name.as_ptr(),
                settable_only as c_int,
                sink,
                &mut out as *mut _ as *mut c_void,
            )
        })?;
        Ok(out)
    }

    pub fn get_string(&self, map: NodeMap, name: &str) -> PylonResult<String> {
        let name = cstring(name)?;
        let mut out = String::new();
        check(unsafe {
            ffi::pylon_node_get_string(
                self.raw,
                map.raw(),
                name.as_ptr(),
                string_sink,
                &mut out as *mut _ as *mut c_void,
            )
        })?;
        Ok(out)
    }

    pub fn set_string(&self, map: NodeMap, name: &str, value: &str) -> PylonResult<()> {
        let name = cstring(name)?;
        let value = cstring(value)?;
        check(unsafe {
            ffi::pylon_node_set_string(self.raw, map.raw(), name.as_ptr(), value.as_ptr())
        })
    }

    pub fn execute(&self, map: NodeMap, name: &str) -> PylonResult<()> {
        let name = cstring(name)?;
        check(unsafe { ffi::pylon_node_execute(self.raw, map.raw(), name.as_ptr()) })
    }
}

struct CallbackCtx {
    handler: Box<dyn CameraHandler>,
    raw: std::sync::atomic::AtomicPtr<ffi::PylonCam>,
}

impl CallbackCtx {
    fn nodes(&self) -> Nodes {
        Nodes {
            raw: self.raw.load(std::sync::atomic::Ordering::Acquire),
        }
    }
}

extern "C" fn on_frame(user: *mut c_void, frame: *const ffi::ShimFrame) {
    let ctx = unsafe { &*(user as *const CallbackCtx) };
    let f = unsafe { &*frame };
    let result = if f.ok != 0 {
        Ok(Frame {
            width: f.width as usize,
            height: f.height as usize,
            pixel: PixelFormat::from_raw(f.pixel),
            raw_pixel_type: f.raw_pixel_type,
            data: unsafe { std::slice::from_raw_parts(f.data as *const u8, f.size) },
            id: f.id,
            timestamp: f.timestamp,
            convert_error: (!f.convert_error.is_null()).then(|| lossy(f.convert_error)),
            chunks: f.chunks,
        })
    } else {
        Err(lossy(f.error))
    };
    ctx.handler.on_frame(&ctx.nodes(), result);
}

extern "C" fn on_event(user: *mut c_void, event_id: c_int) {
    let ctx = unsafe { &*(user as *const CallbackCtx) };
    ctx.handler.on_event(&ctx.nodes(), event_id);
}

extern "C" fn on_removed(user: *mut c_void) {
    let ctx = unsafe { &*(user as *const CallbackCtx) };
    ctx.handler.on_removed();
}

pub struct Camera {
    nodes: Nodes,
    // Referenced by the shim for as long as `nodes.raw` lives.
    _ctx: Box<CallbackCtx>,
}

impl std::ops::Deref for Camera {
    type Target = Nodes;

    fn deref(&self) -> &Nodes {
        &self.nodes
    }
}

impl Camera {
    pub fn new(handler: Box<dyn CameraHandler>) -> Self {
        unsafe { ffi::pylon_shim_initialize() };
        let ctx = Box::new(CallbackCtx {
            handler,
            raw: std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()),
        });
        let callbacks = ffi::ShimCallbacks {
            user: &*ctx as *const CallbackCtx as *mut c_void,
            on_frame,
            on_event,
            on_removed,
        };
        let raw = unsafe { ffi::pylon_cam_new(&callbacks) };
        ctx.raw.store(raw, std::sync::atomic::Ordering::Release);
        Self {
            nodes: Nodes { raw },
            _ctx: ctx,
        }
    }

    pub fn enumerate() -> PylonResult<Vec<DeviceInfo>> {
        extern "C" fn sink(
            user: *mut c_void,
            friendly: *const c_char,
            model: *const c_char,
            serial: *const c_char,
            interface_id: *const c_char,
        ) {
            let out = unsafe { &mut *(user as *mut Vec<DeviceInfo>) };
            out.push(DeviceInfo {
                friendly_name: lossy(friendly),
                model: lossy(model),
                serial: lossy(serial),
                interface_id: lossy(interface_id),
            });
        }
        let mut out: Vec<DeviceInfo> = Vec::new();
        check(unsafe { ffi::pylon_shim_enumerate(sink, &mut out as *mut _ as *mut c_void) })?;
        Ok(out)
    }

    /// `camera_id` of fewer than 4 digits is an index into the enumeration,
    /// anything else a serial number.
    pub fn open(&self, camera_id: &str) -> PylonResult<()> {
        let id = cstring(camera_id)?;
        check(unsafe { ffi::pylon_cam_open(self.nodes.raw, id.as_ptr()) })
    }

    /// Stops any grab in progress, waiting for the grab-loop thread.
    pub fn close(&self) {
        unsafe { ffi::pylon_cam_close(self.nodes.raw) };
    }

    pub fn is_open(&self) -> bool {
        unsafe { ffi::pylon_cam_is_open(self.nodes.raw) != 0 }
    }

    pub fn sfnc_major(&self) -> i32 {
        unsafe { ffi::pylon_cam_sfnc_major(self.nodes.raw) }
    }

    /// Deliver `CameraHandler::on_event(event_id)` whenever the event data
    /// node `node_name` updates. The first registration replaces all earlier
    /// ones.
    pub fn register_event(&self, node_name: &str, event_id: i32, append: bool) -> PylonResult<()> {
        let name = cstring(node_name)?;
        check(unsafe {
            ffi::pylon_cam_register_event(self.nodes.raw, name.as_ptr(), event_id, append as c_int)
        })
    }

    /// Start pylon's own grab loop; `count` of `None` grabs until stopped.
    pub fn start(&self, count: Option<u64>) -> PylonResult<()> {
        let count = count.map_or(-1, |c| c as i64);
        check(unsafe { ffi::pylon_cam_start(self.nodes.raw, count) })
    }

    /// Waits for the grab-loop thread to leave its current callback.
    pub fn stop(&self) -> PylonResult<()> {
        check(unsafe { ffi::pylon_cam_stop(self.nodes.raw) })
    }

    /// Convert every frame to `pixel` before it reaches `on_frame`; `None`
    /// delivers frames as the camera sent them.
    pub fn set_convert(&self, pixel: Option<PixelFormat>, bit_align: i32, shift_bits: i32) {
        let pixel = pixel.map_or(-1, |p| p as c_int);
        unsafe { ffi::pylon_cam_set_convert(self.nodes.raw, pixel, bit_align, shift_bits) };
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        unsafe {
            ffi::pylon_cam_free(self.nodes.raw);
            ffi::pylon_shim_terminate();
        }
    }
}
