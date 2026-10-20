//! Declarations for `shim/pylon_shim.h`.
#![allow(non_camel_case_types)]

use std::os::raw::{c_char, c_int, c_void};

#[repr(C)]
pub struct PylonCam {
    _private: [u8; 0],
}

pub const MAP_CAMERA: c_int = 0;
pub const MAP_STREAM: c_int = 1;

pub const NODE_ABSENT: c_int = -1;
pub const NODE_INTEGER: c_int = 0;
pub const NODE_BOOLEAN: c_int = 1;
pub const NODE_ENUM: c_int = 2;
pub const NODE_FLOAT: c_int = 3;
pub const NODE_STRING: c_int = 4;
pub const NODE_COMMAND: c_int = 5;

#[repr(C)]
pub struct ShimFrame {
    pub ok: c_int,
    pub error: *const c_char,
    pub convert_error: *const c_char,
    pub width: u32,
    pub height: u32,
    pub pixel: c_int,
    pub raw_pixel_type: i64,
    pub data: *const c_void,
    pub size: usize,
    pub id: i64,
    pub timestamp: u64,
    pub chunks: *mut c_void,
}

#[repr(C)]
pub struct ShimCallbacks {
    pub user: *mut c_void,
    pub on_frame: extern "C" fn(*mut c_void, *const ShimFrame),
    pub on_event: extern "C" fn(*mut c_void, c_int),
    pub on_removed: extern "C" fn(*mut c_void),
}

pub type StringSink = extern "C" fn(*mut c_void, *const c_char);
pub type EnumSink = extern "C" fn(*mut c_void, *const c_char, i64);
pub type DeviceSink =
    extern "C" fn(*mut c_void, *const c_char, *const c_char, *const c_char, *const c_char);
pub type ChunkSink =
    extern "C" fn(*mut c_void, *const c_char, *const c_char, c_int, i64, f64, *const c_char);

unsafe extern "C" {
    pub fn pylon_shim_last_error() -> *const c_char;
    pub fn pylon_shim_initialize();
    pub fn pylon_shim_terminate();
    pub fn pylon_shim_version(sink: StringSink, user: *mut c_void);
    pub fn pylon_shim_enumerate(sink: DeviceSink, user: *mut c_void) -> c_int;

    pub fn pylon_cam_new(callbacks: *const ShimCallbacks) -> *mut PylonCam;
    pub fn pylon_cam_free(cam: *mut PylonCam);
    pub fn pylon_cam_open(cam: *mut PylonCam, camera_id: *const c_char) -> c_int;
    pub fn pylon_cam_close(cam: *mut PylonCam);
    pub fn pylon_cam_is_open(cam: *mut PylonCam) -> c_int;
    pub fn pylon_cam_sfnc_major(cam: *mut PylonCam) -> c_int;
    pub fn pylon_cam_register_event(
        cam: *mut PylonCam,
        node_name: *const c_char,
        event_id: c_int,
        append: c_int,
    ) -> c_int;
    pub fn pylon_cam_start(cam: *mut PylonCam, count: i64) -> c_int;
    pub fn pylon_cam_stop(cam: *mut PylonCam) -> c_int;
    pub fn pylon_cam_set_convert(cam: *mut PylonCam, pixel: c_int, bit_align: c_int, shift: c_int);

    pub fn pylon_node_type(cam: *mut PylonCam, map: c_int, name: *const c_char) -> c_int;
    pub fn pylon_node_access(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        available: *mut c_int,
        readable: *mut c_int,
        writable: *mut c_int,
    ) -> c_int;
    pub fn pylon_node_get_int(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        value: *mut i64,
        min: *mut i64,
        max: *mut i64,
        inc: *mut i64,
    ) -> c_int;
    pub fn pylon_node_set_int(cam: *mut PylonCam, map: c_int, name: *const c_char, v: i64)
    -> c_int;
    pub fn pylon_node_get_float(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        value: *mut f64,
        min: *mut f64,
        max: *mut f64,
    ) -> c_int;
    pub fn pylon_node_set_float(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        v: f64,
    ) -> c_int;
    pub fn pylon_node_get_bool(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        value: *mut c_int,
    ) -> c_int;
    pub fn pylon_node_set_bool(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        v: c_int,
    ) -> c_int;
    pub fn pylon_node_get_enum(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        value: *mut i64,
    ) -> c_int;
    pub fn pylon_node_set_enum(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        v: i64,
    ) -> c_int;
    pub fn pylon_node_get_enum_symbolic(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        sink: StringSink,
        user: *mut c_void,
    ) -> c_int;
    pub fn pylon_node_enum_entries(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        settable_only: c_int,
        sink: EnumSink,
        user: *mut c_void,
    ) -> c_int;
    pub fn pylon_node_get_string(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        sink: StringSink,
        user: *mut c_void,
    ) -> c_int;
    pub fn pylon_node_set_string(
        cam: *mut PylonCam,
        map: c_int,
        name: *const c_char,
        value: *const c_char,
    ) -> c_int;
    pub fn pylon_node_execute(cam: *mut PylonCam, map: c_int, name: *const c_char) -> c_int;

    pub fn pylon_frame_chunks(chunks: *mut c_void, sink: ChunkSink, user: *mut c_void);
}
