//! Driver tests against pylon's camera emulation transport layer
//! (`PYLON_CAMEMU`), which needs the SDK but no hardware. Each test runs in
//! its own process under nextest, so each gets its own emulated camera.

use std::time::{Duration, Instant};

use ad_core_rs::driver::{ADStatus, ImageMode};
use ad_core_rs::plugin::channel::NDArrayOutput;
use ad_pylon::{PylonRuntime, create_pylon};
use asyn_rs::port::DrvUserRequest;

const CAMERA_ID: &str = "0815-0000";

fn emulated(port: &str) -> PylonRuntime {
    // SAFETY: set before any other thread exists in this test process.
    unsafe { std::env::set_var("PYLON_CAMEMU", "1") };
    let rt = create_pylon(port, CAMERA_ID, 0, NDArrayOutput::new()).unwrap();
    // As at iocInit: the first record to bind maps the ADDriver parameters
    // onto GenICam features, ImageMode onto AcquisitionMode among them.
    bind(&rt, "IMAGE_MODE");
    rt
}

fn bind(rt: &PylonRuntime, drv_info: &str) -> usize {
    rt.port_handle()
        .drv_user_create_blocking(&DrvUserRequest::new(drv_info, 0))
        .unwrap()
        .reason
}

fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn standard_parameters_take_the_camera_values() {
    let rt = emulated("EMU_STD");
    let h = rt.port_handle();
    let p = rt.ad_params;
    assert!(h.read_int32_blocking(p.max_size_x, 0).unwrap() > 0);
    assert!(h.read_int32_blocking(p.size_x, 0).unwrap() > 0);
    assert!(h.read_float64_blocking(p.acquire_time, 0).unwrap() > 0.0);
}

#[test]
fn feature_drv_info_creates_a_parameter_bound_to_the_node() {
    let rt = emulated("EMU_FEAT");
    let h = rt.port_handle();
    let width = bind(&rt, "GC_I_Width");
    assert_eq!(bind(&rt, "GC_I_Width"), width);
    let size_x = h.read_int32_blocking(rt.ad_params.size_x, 0).unwrap();
    h.write_int32_blocking(rt.ad_params.size_x, 0, size_x / 2)
        .unwrap();
    // A write reads every feature back, so the second parameter bound to
    // Width follows.
    assert_eq!(
        h.read_int32_blocking(rt.ad_params.size_x, 0).unwrap(),
        size_x / 2
    );
}

#[test]
fn single_mode_stops_after_one_frame() {
    let rt = emulated("EMU_SINGLE");
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Single as i32)
        .unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("Acquire to return to 0", || {
        h.read_int32_blocking(p.acquire, 0).unwrap() == 0
    });
    assert_eq!(h.read_int32_blocking(p.base.array_counter, 0).unwrap(), 1);
    assert_eq!(
        h.read_int32_blocking(p.status, 0).unwrap(),
        ADStatus::Idle as i32
    );

    // and a second acquisition starts from there
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("the second frame", || {
        h.read_int32_blocking(p.base.array_counter, 0).unwrap() == 2
    });
}

#[test]
fn multiple_mode_stops_at_num_images() {
    let rt = emulated("EMU_MULTI");
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Multiple as i32)
        .unwrap();
    h.write_int32_blocking(p.num_images, 0, 3).unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("Acquire to return to 0", || {
        h.read_int32_blocking(p.acquire, 0).unwrap() == 0
    });
    assert_eq!(h.read_int32_blocking(p.num_images_counter, 0).unwrap(), 3);
}

#[test]
fn continuous_mode_runs_until_stopped() {
    let rt = emulated("EMU_CONT");
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Continuous as i32)
        .unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("three frames", || {
        h.read_int32_blocking(p.base.array_counter, 0).unwrap() >= 3
    });
    assert_eq!(
        h.read_int32_blocking(p.status, 0).unwrap(),
        ADStatus::Waiting as i32
    );
    h.write_int32_blocking(p.acquire, 0, 0).unwrap();
    assert_eq!(
        h.read_int32_blocking(p.status, 0).unwrap(),
        ADStatus::Idle as i32
    );
    let stopped_at = h.read_int32_blocking(p.base.array_counter, 0).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    // Frames already queued for the image task may still land; the grab
    // itself is over.
    let settled = h.read_int32_blocking(p.base.array_counter, 0).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        h.read_int32_blocking(p.base.array_counter, 0).unwrap(),
        settled
    );
    assert!(settled >= stopped_at);
}

#[test]
fn a_write_notifies_the_readback_subscribers() {
    use asyn_rs::interrupt::{InterruptFilter, InterruptValue};
    use asyn_rs::param::ParamValue;
    use std::sync::{Arc, Mutex};

    // No IOC raises the gate here; until it is up a flush keeps its flags.
    epics_rs::base::runtime::interrupt_accept::set_interrupts_accepted(true);
    let rt = emulated("EMU_INTR");
    let h = rt.port_handle();
    let size_x = rt.ad_params.size_x;
    let seen: Arc<Mutex<Vec<i32>>> = Arc::new(Mutex::new(Vec::new()));
    let sink = seen.clone();
    let _sub = h.interrupts().register_sync_callback(
        InterruptFilter {
            reason: Some(size_x),
            addr: Some(0),
            uint32_mask: None,
            iface: None,
        },
        move |iv: &InterruptValue| {
            if let ParamValue::Int32(v) = &iv.value {
                sink.lock().unwrap().push(*v);
            }
        },
    );
    h.write_int32_blocking(size_x, 0, 512).unwrap();
    assert_eq!(seen.lock().unwrap().last(), Some(&512));
}

#[test]
fn a_string_feature_write_reaches_the_camera() {
    use asyn_rs::request::RequestOp;
    use asyn_rs::user::AsynUser;

    let rt = emulated("EMU_STR");
    let h = rt.port_handle();
    let user_id = bind(&rt, "GC_S_DeviceUserID");
    h.submit_blocking(
        RequestOp::OctetWrite {
            data: b"beamline-7".to_vec(),
        },
        AsynUser::new(user_id).with_addr(0),
    )
    .unwrap();
    // Any write reads every feature back from the camera, so the parameter
    // now holds what the camera holds.
    let size_x = rt.ad_params.size_x;
    h.write_int32_blocking(size_x, 0, 512).unwrap();
    let read = h
        .submit_blocking(
            RequestOp::OctetRead { buf_size: 64 },
            AsynUser::new(user_id).with_addr(0),
        )
        .unwrap();
    assert_eq!(read.data.as_deref(), Some(&b"beamline-7"[..]));
}

/// C `processFrame`: `getAttributes` puts the `NDAttributesFile` set on every
/// frame, and the driver's own `ColorMode` is added after it, so it wins over a
/// same-named entry in the file.
#[test]
fn a_frame_carries_the_attributes_file_set() {
    use ad_core_rs::attributes::NDAttrValue;
    use ad_core_rs::plugin::channel::ndarray_channel;
    use asyn_rs::request::RequestOp;
    use asyn_rs::user::AsynUser;

    let rt = emulated("EMU_ATTR");
    let (sender, mut frames) = ndarray_channel("EMU_ATTR", 4);
    rt.array_output().lock().add(sender);
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.base.array_callbacks, 0, 1)
        .unwrap();

    let xml = r#"<Attributes>
        <Attribute name="Mode" type="PARAM" source="IMAGE_MODE" datatype="INT"/>
        <Attribute name="ColorMode" type="CONST" source="from the file"/>
    </Attributes>"#;
    h.submit_blocking(
        RequestOp::OctetWrite {
            data: xml.as_bytes().to_vec(),
        },
        AsynUser::new(p.base.attributes_file).with_addr(0),
    )
    .unwrap();
    assert_eq!(
        h.read_int32_blocking(p.base.attributes_status, 0).unwrap(),
        0
    );

    h.write_int32_blocking(p.image_mode, 0, ImageMode::Single as i32)
        .unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    let frame = frames.blocking_recv().expect("a frame");
    assert_eq!(
        frame.attributes.get("Mode").unwrap().value,
        NDAttrValue::Int32(ImageMode::Single as i32)
    );
    assert!(matches!(
        frame.attributes.get("ColorMode").unwrap().value,
        NDAttrValue::Int32(_)
    ));
}

/// Puts the emulated camera in a pixel format with no NDArray layout, so that
/// every grab result is delivered as a failed frame.
fn unrepresentable_frames(rt: &PylonRuntime) {
    const PFNC_MONO12: i32 = 0x0110_0005;
    let pixel_format = bind(rt, "GC_E_PixelFormat");
    rt.port_handle()
        .write_int32_blocking(pixel_format, 0, PFNC_MONO12)
        .unwrap();
}

/// pylon counts a failed result towards `StartGrabbing(count)`; the
/// acquisition ends with the grab instead of waiting for a frame that is not
/// coming.
#[test]
fn a_failed_frame_ends_a_single_acquisition() {
    let rt = emulated("EMU_FAIL_SINGLE");
    unrepresentable_frames(&rt);
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Single as i32)
        .unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("Acquire to return to 0", || {
        h.read_int32_blocking(p.acquire, 0).unwrap() == 0
    });
    assert_eq!(h.read_int32_blocking(p.base.array_counter, 0).unwrap(), 0);
    assert_eq!(
        h.read_int32_blocking(p.status, 0).unwrap(),
        ADStatus::Idle as i32
    );
}

#[test]
fn failed_frames_end_a_multiple_acquisition_with_the_grab() {
    let rt = emulated("EMU_FAIL_MULTI");
    unrepresentable_frames(&rt);
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Multiple as i32)
        .unwrap();
    h.write_int32_blocking(p.num_images, 0, 3).unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    wait_for("Acquire to return to 0", || {
        h.read_int32_blocking(p.acquire, 0).unwrap() == 0
    });
    assert_eq!(h.read_int32_blocking(p.num_images_counter, 0).unwrap(), 0);
}

#[test]
fn failed_frames_do_not_end_a_continuous_acquisition() {
    let rt = emulated("EMU_FAIL_CONT");
    unrepresentable_frames(&rt);
    let h = rt.port_handle();
    let p = rt.ad_params;
    h.write_int32_blocking(p.image_mode, 0, ImageMode::Continuous as i32)
        .unwrap();
    h.write_int32_blocking(p.acquire, 0, 1).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(h.read_int32_blocking(p.acquire, 0).unwrap(), 1);
    h.write_int32_blocking(p.acquire, 0, 0).unwrap();
    assert_eq!(
        h.read_int32_blocking(p.status, 0).unwrap(),
        ADStatus::Idle as i32
    );
}
