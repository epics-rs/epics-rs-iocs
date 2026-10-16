//! `writeInt32`/`writeFloat64` return the status of the step they ran
//! (specsAnalyser.cpp:798-835, 865-902, 941-955), against a fake SPECS
//! server that refuses the commands a test names.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::mpsc;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;

use ad_specs_analyser::{SpecsAnalyserRuntime, create_specs_analyser_detector};

/// The reply payload a healthy server gives `command`.
fn healthy(command: &str) -> String {
    let name = command
        .split("ParameterName:\"")
        .nth(1)
        .and_then(|rest| rest.split('"').next())
        .unwrap_or("");
    match command.split(' ').next().unwrap_or("") {
        "Connect" => "OK ServerName:\"fake\" ProtocolVersion:1.11".into(),
        "GetAnalyzerVisibleName" => "OK AnalyzerVisibleName:\"Phoibos\"".into(),
        "GetAllAnalyzerParameterNames" => "OK ParameterNames:[\"Bias\",\"Turns\",\"Odd\"]".into(),
        "GetAnalyzerParameterInfo" if name == "Bias" => "OK ValueType:double".into(),
        "GetAnalyzerParameterInfo" => "OK ValueType:integer".into(),
        "GetAnalyzerParameterValue" if name == "Bias" => "OK Value:1.5".into(),
        "GetAnalyzerParameterValue" if name == "Odd" => "OK Value:x".into(),
        "GetAnalyzerParameterValue" => "OK Value:3".into(),
        "GetSpectrumParameterInfo" => "OK Values:[\"a\",\"b\"]".into(),
        _ => "OK".into(),
    }
}

/// A detector on a fake server that answers `ERROR <code> refused` to every
/// command starting with one of the prefixes last passed to `refuse`.
struct Fake {
    runtime: SpecsAnalyserRuntime,
    arm: mpsc::Sender<(u32, Vec<&'static str>)>,
}

impl Fake {
    fn start(name: &str) -> Fake {
        Self::start_refusing(name, 0, &[])
    }

    /// Refusing from the constructor's first command on.
    fn start_refusing(name: &str, code: u32, refuse: &[&'static str]) -> Fake {
        let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let (arm, armed) = mpsc::channel::<(u32, Vec<&'static str>)>();
        arm.send((code, refuse.to_vec())).unwrap();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("connection");
            let mut refuse = (0, Vec::new());
            let mut buf = [0u8; 4096];
            // No EOS on the port: one read is one request.
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
                refuse = armed.try_iter().last().unwrap_or(refuse);
                let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
                let (counter, command) = line.trim_start_matches('?').split_at(4);
                let command = command.trim().to_string();
                let reply = if refuse.1.iter().any(|r| command.starts_with(r)) {
                    format!("ERROR {} refused", refuse.0)
                } else {
                    healthy(&command)
                };
                let counter: u32 = counter.parse().unwrap();
                // One write, so one read takes the whole reply.
                let frame = format!("!{counter:04} {reply}");
                if stream.write_all(frame.as_bytes()).is_err() {
                    break;
                }
            }
        });

        let ip = format!("{name}_IP");
        let driver = DrvAsynIPPort::new(&ip, &addr).expect("ip port");
        let ip_claim = claim_port(&ip).unwrap();
        let (ip_runtime, _) = create_port_runtime(&ip_claim, driver, RuntimeConfig::default())
            .expect("ip port runtime");
        // What drvAsynIPPortConfigure's registerPort does.
        ip_claim
            .publish(ip_runtime.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(ip_runtime);

        let claim = claim_port(name).unwrap();
        let runtime = create_specs_analyser_detector(&claim, &ip, 0, 0).expect("detector");
        Fake { runtime, arm }
    }

    fn refuse(&self, code: u32, commands: &[&'static str]) {
        self.arm.send((code, commands.to_vec())).unwrap();
    }

    fn reason(&self, name: &str) -> usize {
        self.runtime
            .port_handle()
            .drv_user_create_blocking(&DrvUserRequest::new(name, 0))
            .unwrap()
            .reason
    }

    fn write_int32(&self, reason: usize, value: i32) -> Result<(), AsynStatus> {
        self.runtime
            .port_handle()
            .write_int32_blocking(reason, 0, value)
            .map_err(|e| e.status())
    }
}

#[test]
fn a_refused_define_is_an_error() {
    let fake = Fake::start("SPECS_WS_DEFINE");
    fake.refuse(7, &["DefineSpectrum"]);
    let define = fake.runtime.specs_params.define;
    assert_eq!(fake.write_int32(define, 1), Err(AsynStatus::Error));
}

#[test]
fn a_refused_validate_is_an_error() {
    let fake = Fake::start("SPECS_WS_VALIDATE");
    fake.refuse(7, &["ValidateSpectrum"]);
    let validate = fake.runtime.specs_params.validate;
    assert_eq!(fake.write_int32(validate, 1), Err(AsynStatus::Error));
}

#[test]
fn a_refused_spectrum_parameter_read_fails_the_connect() {
    // Code 3 drops SPECSConnected_ (specsAnalyser.cpp:2021-2024), so the
    // Connect write below is not skipped.
    let fake = Fake::start_refusing("SPECS_WS_CONNECT", 3, &["Connect"]);
    fake.refuse(7, &["GetSpectrumParameterInfo"]);
    let connect = fake.runtime.specs_params.connect;
    assert_eq!(fake.write_int32(connect, 1), Err(AsynStatus::Error));
}

#[test]
fn a_refused_non_energy_channels_set_is_an_error_and_reverts() {
    let fake = Fake::start("SPECS_WS_NEC");
    fake.refuse(7, &["SetAnalyzerParameterValue"]);
    let nec = fake.runtime.specs_params.non_energy_channels;
    let before = fake
        .runtime
        .port_handle()
        .read_int32_blocking(nec, 0)
        .unwrap();
    assert_eq!(fake.write_int32(nec, before + 4), Err(AsynStatus::Error));
    let after = fake
        .runtime
        .port_handle()
        .read_int32_blocking(nec, 0)
        .unwrap();
    assert_eq!(after, before);
}

#[test]
fn a_refused_stored_int_parameter_is_an_error() {
    let fake = Fake::start("SPECS_WS_TURNS");
    fake.refuse(7, &["SetAnalyzerParameterValue"]);
    let turns = fake.reason("TURNS");
    assert_eq!(fake.write_int32(turns, 9), Err(AsynStatus::Error));
}

#[test]
fn a_refused_stored_double_parameter_is_an_error_and_reverts() {
    let fake = Fake::start("SPECS_WS_BIAS");
    fake.refuse(7, &["SetAnalyzerParameterValue"]);
    let bias = fake.reason("BIAS");
    let handle = fake.runtime.port_handle();
    let err = handle.write_float64_blocking(bias, 0, 9.0).unwrap_err();
    assert_eq!(err.status(), AsynStatus::Error);
    assert_eq!(handle.read_float64_blocking(bias, 0).unwrap(), 1.5);
}

/// `readIntegerData` fails on a value that does not decode
/// (specsAnalyser.cpp:1635-1647), so the confirm read is an error.
#[test]
fn an_undecodable_confirm_is_an_error() {
    let fake = Fake::start("SPECS_WS_ODD");
    let odd = fake.reason("ODD");
    assert_eq!(fake.write_int32(odd, 9), Err(AsynStatus::Error));
}
