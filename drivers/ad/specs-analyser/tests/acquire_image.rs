//! C sums each acquisition's data points straight into the pool array it
//! publishes (specsAnalyser.cpp:454, :584-588), against a fake SPECS server
//! that acquires one 2x2 iteration.

use std::io::{Read, Write};
use std::net::TcpListener;

use epics_rs::ad_core::plugin::channel::ndarray_channel;
use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use ad_specs_analyser::create_specs_analyser_detector;

/// Two non-energy channels of two energy channels each, row by row.
const DATA: [f64; 4] = [1.5, 2.5, 3.5, 4.5];

fn reply(command: &str) -> String {
    match command.split(' ').next().unwrap_or("") {
        "Connect" => "OK ServerName:\"fake\" ProtocolVersion:1.11".into(),
        "GetAnalyzerVisibleName" => "OK AnalyzerVisibleName:\"Phoibos\"".into(),
        "GetAllAnalyzerParameterNames" => "OK ParameterNames:[]".into(),
        "GetSpectrumParameterInfo" => "OK Values:[\"a\"]".into(),
        "GetAnalyzerParameterValue" => "OK Value:2".into(),
        "ValidateSpectrum" => "OK StartEnergy:1 EndEnergy:2 StepWidth:1 Samples:2 \
                               DwellTime:0.01 PassEnergy:5 LensMode:\"a\" ScanRange:\"a\""
            .into(),
        "GetAcquisitionStatus" => "OK ControllerState:finished NumberOfAcquiredPoints:2".into(),
        "GetAcquisitionData" => format!("OK Data:[{}]", DATA.map(|v| v.to_string()).join(",")),
        _ => "OK".into(),
    }
}

#[test]
fn an_acquisition_publishes_the_data_it_summed() {
    let name = "SPECS_IMAGE";
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
    std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("connection");
        let mut buf = [0u8; 4096];
        // No EOS on the port: one read is one request.
        while let Ok(n) = stream.read(&mut buf) {
            if n == 0 {
                break;
            }
            let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
            let (counter, command) = line.trim_start_matches('?').split_at(4);
            let frame = format!("!{counter} {}", reply(command.trim()));
            if stream.write_all(frame.as_bytes()).is_err() {
                break;
            }
        }
    });

    let ip = format!("{name}_IP");
    let driver = DrvAsynIPPort::new(&ip, &addr).expect("ip port");
    let ip_claim = claim_port(&ip).unwrap();
    let (ip_runtime, _) =
        create_port_runtime(&ip_claim, driver, RuntimeConfig::default()).expect("ip port runtime");
    ip_claim
        .publish(ip_runtime.port_handle().clone(), &open_owner())
        .unwrap();
    std::mem::forget(ip_runtime);

    let runtime =
        create_specs_analyser_detector(&claim_port(name).unwrap(), &ip, 0, 0).expect("detector");
    let (sender, mut receiver) = ndarray_channel(&format!("{name}_SINK"), 4);
    runtime.connect_downstream(sender);
    let (h, ad) = (runtime.port_handle(), runtime.ad_params);
    h.write_int32_blocking(ad.base.array_callbacks, 0, 1)
        .unwrap();
    h.write_int32_blocking(ad.num_exposures, 0, 1).unwrap();
    h.write_int32_blocking(ad.acquire, 0, 1).unwrap();

    let frame = receiver.blocking_recv().expect("no frame was published");
    assert_eq!(
        frame.dims.iter().map(|d| d.size).collect::<Vec<_>>(),
        [2, 2]
    );
    assert_eq!(frame.data().as_slice::<f64>(), Some(&DATA[..]));
    std::mem::forget(runtime);
}
