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

/// Commands starting with a prefix get `ERROR <code> refused`, or no reply at
/// all for code 0.
type Rules = Vec<(&'static str, u32)>;

/// A detector on a fake server that answers by the rules last armed; `heard`
/// gets each request's counter.
struct Fake {
    runtime: SpecsAnalyserRuntime,
    arm: mpsc::Sender<Rules>,
    heard: mpsc::Receiver<u32>,
}

impl Fake {
    fn start(name: &str) -> Fake {
        Self::start_refusing(name, 0, &[])
    }

    /// Refusing from the constructor's first command on.
    fn start_refusing(name: &str, code: u32, refuse: &[&'static str]) -> Fake {
        let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let (arm, armed) = mpsc::channel::<Rules>();
        arm.send(refuse.iter().map(|&p| (p, code)).collect())
            .unwrap();
        let (tell, heard) = mpsc::channel();
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("connection");
            let mut rules = Rules::new();
            let mut buf = [0u8; 4096];
            // No EOS on the port: one read is one request.
            while let Ok(n) = stream.read(&mut buf) {
                if n == 0 {
                    break;
                }
                rules = armed.try_iter().last().unwrap_or(rules);
                let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
                let (counter, command) = line.trim_start_matches('?').split_at(4);
                let command = command.trim().to_string();
                let counter: u32 = counter.parse().unwrap();
                let _ = tell.send(counter);
                let reply = match rules.iter().find(|(p, _)| command.starts_with(p)) {
                    Some((_, 0)) => continue,
                    Some((_, code)) => format!("ERROR {code} refused"),
                    None => healthy(&command),
                };
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
        let fake = Fake {
            runtime,
            arm,
            heard,
        };
        // The task's first post would overwrite a test's ADStatusMessage, and
        // an Acquire it reads before its wait is never started.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while fake.status_message() != "Waiting for the acquire command" {
            assert!(
                std::time::Instant::now() < deadline,
                "the task never waited"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        fake
    }

    fn refuse(&self, code: u32, commands: &[&'static str]) {
        self.answer(&commands.iter().map(|&p| (p, code)).collect::<Rules>());
    }

    fn answer(&self, rules: &[(&'static str, u32)]) {
        self.arm.send(rules.to_vec()).unwrap();
    }

    fn reason(&self, name: &str) -> usize {
        self.runtime
            .port_handle()
            .drv_user_create_blocking(&DrvUserRequest::new(name, 0))
            .unwrap()
            .reason
    }

    fn status_message(&self) -> String {
        let bytes = self
            .runtime
            .port_handle()
            .get_string_param_blocking(self.runtime.ad_params.status_message, 0, 256)
            .unwrap();
        String::from_utf8_lossy(&bytes).into_owned()
    }

    fn msg_counter(&self) -> i32 {
        self.runtime
            .port_handle()
            .read_int32_blocking(self.runtime.specs_params.msg_counter, 0)
            .unwrap()
    }

    /// The counter of the last request the server has seen.
    fn last_heard(&self) -> u32 {
        self.heard.try_iter().last().unwrap_or(0)
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

/// writeInt32 ORs the Connect step statuses (specsAnalyser.cpp:803-813): a
/// timeout (1) then a refusal (3) is asynError, not the first step's timeout.
#[test]
fn connect_returns_the_or_of_the_step_statuses() {
    let fake = Fake::start_refusing("SPECS_WS_CONNOR", 3, &["Connect"]);
    fake.answer(&[
        ("GetSpectrumParameterInfo ParameterName:\"LensMode\"", 0),
        ("GetSpectrumParameterInfo", 7),
    ]);
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

/// sendSimpleCommand puts an ERROR reply's code and message in
/// ADStatusMessage (specsAnalyser.cpp:1324-1331); SetSafeState's status is
/// ignored (:879).
#[test]
fn a_refused_safe_state_reports_the_code_and_message() {
    let fake = Fake::start("SPECS_WS_SAFE");
    fake.refuse(7, &["SetSafeState"]);
    let safe = fake.runtime.specs_params.safe_state;
    assert_eq!(fake.write_int32(safe, 1), Ok(()));
    assert_eq!(fake.status_message(), "7: refused");
}

#[test]
fn a_refused_validate_reports_the_code_and_message() {
    let fake = Fake::start("SPECS_WS_VALMSG");
    fake.refuse(7, &["ValidateSpectrum"]);
    let validate = fake.runtime.specs_params.validate;
    assert_eq!(fake.write_int32(validate, 1), Err(AsynStatus::Error));
    assert_eq!(fake.status_message(), "7: refused");
}

/// asynWriteRead posts SPECSMsgCounter for every exchange
/// (specsAnalyser.cpp:2056-2062).
#[test]
fn a_write_posts_the_message_counter() {
    let fake = Fake::start("SPECS_WS_COUNT");
    let define = fake.runtime.specs_params.define;
    fake.write_int32(define, 1).unwrap();
    let last = fake.last_heard();
    assert!(last > 0);
    assert_eq!(fake.msg_counter(), last as i32);
}

/// The acquisition task's exchanges post it too.
#[test]
fn the_acquisition_task_posts_the_message_counter() {
    let fake = Fake::start("SPECS_WS_TASKCOUNT");
    let before = fake.last_heard();
    // Refusing the first command of an acquisition ends it at once.
    fake.refuse(7, &["GetAnalyzerParameterValue"]);
    let acquire = fake.runtime.ad_params.acquire;
    fake.write_int32(acquire, 1).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut last = before;
    loop {
        last = fake.heard.try_iter().last().unwrap_or(last);
        if last > before && fake.msg_counter() == last as i32 {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "counter {} after the task sent {last}",
            fake.msg_counter()
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}
