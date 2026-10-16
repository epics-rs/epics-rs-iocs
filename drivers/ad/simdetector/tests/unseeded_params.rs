//! An IOC without the PINI records for SimNoise, SimPeakHeightVariation and
//! the sine params leaves them undefined; C reads them as 0 and still produces
//! frames (simDetector.cpp:57-60,251-265,364-386).

use std::sync::mpsc;
use std::time::Duration;

use epics_rs::ad_core::driver::ImageMode;
use epics_rs::ad_core::ndarray::NDDataType;
use epics_rs::ad_core::plugin::channel::ndarray_channel;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::sync_io::SyncIOHandle;

use ad_simdetector::create_sim_detector;

#[test]
fn a_port_with_unseeded_sim_params_still_produces_a_frame() {
    let detector = create_sim_detector(
        &claim_port("SIM_UNSEEDED").unwrap(),
        16,
        8,
        NDDataType::UInt8,
        0,
        0,
    )
    .expect("the detector port");

    let (sender, mut receiver) = ndarray_channel("SIM_UNSEEDED_PLUGIN", 4);
    detector.connect_downstream(sender);
    let (published, arrays) = mpsc::channel();
    std::thread::spawn(move || {
        while let Some(array) = receiver.blocking_recv() {
            if published.send(array).is_err() {
                break;
            }
        }
    });

    let ad = detector.ad_params;
    let sync = SyncIOHandle::from_handle(detector.port_handle().clone(), 0, Duration::from_secs(5));
    sync.write_int32(ad.image_mode, ImageMode::Single as i32)
        .expect("image mode");
    sync.write_int32(ad.acquire, 1).expect("acquire");

    let array = arrays
        .recv_timeout(Duration::from_secs(10))
        .expect("no frame from a port whose sim params were never written");
    assert_eq!(array.dims[0].size, 16);
    assert_eq!(array.dims[1].size, 8);
}
