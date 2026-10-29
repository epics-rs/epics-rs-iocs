//! C decodes each readout straight into the pool array it publishes
//! (mythen.cpp:973-977), masking the bits above a 24-bit count.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::Arc;

use epics_rs::ad_core::plugin::channel::{NDArrayOutput, ndarray_channel};
use epics_rs::ad_core::runtime as rt;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use mythen::detector::Detector;
use mythen::driver::MythenDriver;
use mythen::task::{self, Shared};
use mythen::transport::Transport;

/// Word `i` of the one-module readout, with a stray bit above its count.
fn word(i: u32) -> u32 {
    (i * 3 + 1) | 0x0100_0000
}

/// An idle one-module 24-bit detector that answers one readout.
fn one_readout_detector() -> PortHandle {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("connection");
        let mut buf = [0u8; 256];
        while let Ok(n) = stream.read(&mut buf) {
            if n == 0 {
                break;
            }
            let reply = match String::from_utf8_lossy(&buf[..n]).trim() {
                "-readoutraw" => (0..1280).flat_map(|i| word(i).to_le_bytes()).collect(),
                "-get version" => b"M3.0.0\0".to_vec(),
                "-get nmodules" => 1i32.to_le_bytes().to_vec(),
                // Not running, no data: idle (bit 16 set, bit 0 clear).
                "-get status" => (1i32 << 16).to_le_bytes().to_vec(),
                "-get nbits" => 24i32.to_le_bytes().to_vec(),
                "-get tau" => (-1.0f32).to_le_bytes().to_vec(),
                _ => 0i32.to_le_bytes().to_vec(),
            };
            if stream.write_all(&reply).is_err() {
                break;
            }
        }
    });

    let driver = DrvAsynIPPort::new("MYTHEN_PIX_IP", &addr).expect("ip port");
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_PIX_IP").unwrap(),
        driver,
        RuntimeConfig::default(),
    )
    .expect("ip port runtime");
    let handle = runtime.port_handle().clone();
    // The port actor must outlive the test.
    std::mem::forget(runtime);
    handle
}

#[test]
fn a_readout_publishes_its_decoded_channels() {
    let det = Arc::new(Detector::new(
        Transport::new(one_readout_detector()),
        "MYTHEN_PIX",
    ));
    let (start_tx, start_rx) = rt::command_channel::<()>(1);
    let driver = MythenDriver::new("MYTHEN_PIX", det.clone(), 0, start_tx).unwrap();
    let (p, ad, pool) = (driver.p, driver.ad.params, driver.ad.pool.clone());
    let (runtime, _) = create_port_runtime(
        &claim_port("MYTHEN_PIX").unwrap(),
        driver,
        RuntimeConfig::default(),
    )
    .unwrap();
    let handle = runtime.port_handle().clone();
    let (sender, mut receiver) = ndarray_channel("MYTHEN_PIX_SINK", 4);
    let mut output = NDArrayOutput::new();
    output.add(sender);
    let _task = task::start(
        Arc::new(Shared {
            handle: handle.clone(),
            p,
            ad,
            det,
            output: Arc::new(parking_lot::Mutex::new(output)),
            pool,
        }),
        start_rx,
    );

    handle
        .write_int32_blocking(ad.base.array_callbacks, 0, 1)
        .unwrap();
    handle.write_int32_blocking(ad.acquire, 0, 1).unwrap();
    let frame = receiver.blocking_recv().expect("no frame was published");
    let expected: Vec<u32> = (0..1280).map(|i| word(i) & 0x00ff_ffff).collect();
    assert_eq!(frame.data().as_slice::<u32>(), Some(&expected[..]));
}
