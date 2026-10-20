//! `writeInt32`/`writeFloat64` return the status of the step they ran
//! (specsAnalyser.cpp:798-835, 865-902, 941-955), against a fake SPECS
//! server that refuses the commands a test names.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::mpsc;

use epics_rs::asyn::drivers::ip_port::DrvAsynIPPort;
use epics_rs::asyn::error::AsynStatus;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::interrupt::{ENUM_ENTRY_BYTES, InterruptFilter};
use epics_rs::asyn::param::ParamValue;
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

/// How the server answers a command starting with a rule's prefix.
#[derive(Clone, Copy)]
enum Act {
    /// `ERROR <code> refused`.
    Refuse(u32),
    /// No reply.
    Mute,
    /// The healthy reply, this many milliseconds late.
    Slow(u64),
    /// This reply, this many milliseconds late.
    Say(&'static str, u64),
}

type Rules = Vec<(&'static str, Act)>;

/// A detector on a fake server that answers by the rules last armed; `heard`
/// gets each request's counter and command.
struct Fake {
    runtime: SpecsAnalyserRuntime,
    arm: mpsc::Sender<Rules>,
    heard: mpsc::Receiver<(u32, String)>,
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
        arm.send(refuse.iter().map(|&p| (p, Act::Refuse(code))).collect())
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
                let _ = tell.send((counter, command.clone()));
                let reply = match rules.iter().find(|(p, _)| command.starts_with(p)) {
                    Some((_, Act::Mute)) => continue,
                    Some((_, Act::Refuse(code))) => format!("ERROR {code} refused"),
                    Some((_, Act::Slow(ms))) => {
                        std::thread::sleep(std::time::Duration::from_millis(*ms));
                        healthy(&command)
                    }
                    Some((_, Act::Say(reply, ms))) => {
                        std::thread::sleep(std::time::Duration::from_millis(*ms));
                        reply.to_string()
                    }
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
        // Open, as after iocInit, so the posts reach a subscriber.
        claim
            .publish(runtime.port_handle().clone(), &open_owner())
            .unwrap();
        // The open's flush runs on its own thread; flushing here leaves it
        // nothing to post to a test's subscriber.
        runtime
            .port_handle()
            .call_param_callbacks_blocking(0)
            .unwrap();
        Fake {
            runtime,
            arm,
            heard,
        }
    }

    fn refuse(&self, code: u32, commands: &[&'static str]) {
        self.answer(
            &commands
                .iter()
                .map(|&p| (p, Act::Refuse(code)))
                .collect::<Rules>(),
        );
    }

    fn answer(&self, rules: &[(&'static str, Act)]) {
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

    fn string(&self, reason: usize) -> String {
        let bytes = self
            .runtime
            .port_handle()
            .get_string_param_blocking(reason, 0, 256)
            .unwrap();
        String::from_utf8_lossy(&bytes).into_owned()
    }

    fn int32(&self, reason: usize) -> i32 {
        self.runtime
            .port_handle()
            .read_int32_blocking(reason, 0)
            .unwrap()
    }

    /// The counter of the last request the server has seen.
    fn last_heard(&self) -> u32 {
        self.heard
            .try_iter()
            .last()
            .map_or(0, |(counter, _)| counter)
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
        (
            "GetSpectrumParameterInfo ParameterName:\"LensMode\"",
            Act::Mute,
        ),
        ("GetSpectrumParameterInfo", Act::Refuse(7)),
    ]);
    let connect = fake.runtime.specs_params.connect;
    assert_eq!(fake.write_int32(connect, 1), Err(AsynStatus::Error));
}

/// readSpectrumParameter publishes each table on asynEnum
/// (specsAnalyser.cpp:1724-1735), so the mbbo states follow a reconnect.
#[test]
fn a_connect_posts_the_spectrum_tables_on_asyn_enum() {
    let fake = Fake::start_refusing("SPECS_WS_ENUMPOST", 3, &["Connect"]);
    let p = &fake.runtime.specs_params;
    let subscribe = |reason| {
        fake.runtime
            .port_handle()
            .interrupts()
            .subscribe_async_filtered(
                InterruptFilter {
                    reason: Some(reason),
                    iface: Some(InterfaceType::Enum),
                    ..Default::default()
                },
                16,
                16 * ENUM_ENTRY_BYTES,
            )
            .unwrap()
    };
    let (mut lens, mut range) = (subscribe(p.lens_mode), subscribe(p.scan_range));
    fake.answer(&[]);
    fake.write_int32(p.connect, 1).unwrap();
    for posts in [&mut lens, &mut range] {
        let mut tables = Vec::new();
        while let Ok(v) = posts.try_recv() {
            if let ParamValue::Enum { choices, .. } = v.value {
                tables.push(choices.iter().map(|e| e.string.clone()).collect::<Vec<_>>());
            }
        }
        assert_eq!(tables, [["a", "b"]]);
    }
}

/// connect() records the connection and asynPortConnect the server's
/// answer (specsAnalyser.cpp:236-238, 1861-1868), so a Connect write while
/// connected sends nothing (:801-803).
#[test]
fn a_connect_while_connected_is_skipped() {
    let fake = Fake::start("SPECS_WS_CONNECTED");
    let p = &fake.runtime.specs_params;
    assert_eq!(fake.int32(p.connected), 1);
    assert_eq!(fake.string(p.server_name), "fake");
    assert_eq!(fake.string(p.protocol_version), "1.11");
    let before = fake.last_heard();
    fake.write_int32(p.connect, 1).unwrap();
    assert_eq!(fake.heard.try_iter().last(), None, "after {before}");
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

/// An ERROR 3 reply disconnects (specsAnalyser.cpp:2021-2024, 1882).
#[test]
fn a_not_connected_reply_clears_the_connected_param() {
    let fake = Fake::start("SPECS_WS_ERR3");
    fake.refuse(3, &["SetSafeState"]);
    let p = &fake.runtime.specs_params;
    assert_eq!(fake.write_int32(p.safe_state, 1), Ok(()));
    assert_eq!(fake.int32(p.connected), 0);
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

/// The post comes before each exchange, not once per write: a stored
/// parameter's set and confirm read post two counters.
#[test]
fn a_write_posts_the_counter_for_each_exchange() {
    let fake = Fake::start("SPECS_WS_EACH");
    let turns = fake.reason("TURNS");
    let filter = InterruptFilter {
        reason: Some(fake.runtime.specs_params.msg_counter),
        ..Default::default()
    };
    let mut posts = fake
        .runtime
        .port_handle()
        .interrupts()
        .subscribe_async_filtered(filter, 16, 0)
        .unwrap();
    fake.write_int32(turns, 9).unwrap();
    let mut posted = Vec::new();
    while let Ok(v) = posts.try_recv() {
        if let ParamValue::Int32(n) = v.value {
            posted.push(n);
        }
    }
    let last = fake.last_heard() as i32;
    assert_eq!(posted, [last - 1, last]);
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
        last = fake
            .heard
            .try_iter()
            .last()
            .map_or(last, |(counter, _)| counter);
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

/// The task holds the driver lock from its start wait to its first poll
/// sleep (C this->lock(), specsAnalyser.cpp:323-501), so a write issued
/// while ValidateSpectrum is out lands after the iteration's
/// GetAcquisitionStatus.
#[test]
fn a_write_waits_for_the_task_sequence() {
    let fake = Fake::start("SPECS_WS_HOLD");
    fake.answer(&[("ValidateSpectrum", Act::Slow(300))]);
    let turns = fake.reason("TURNS");
    fake.write_int32(fake.runtime.ad_params.acquire, 1).unwrap();
    let mut heard = Vec::new();
    while !heard.iter().any(|c: &String| c == "ValidateSpectrum") {
        let (_, command) = fake
            .heard
            .recv_timeout(std::time::Duration::from_secs(5))
            .expect("the task never validated");
        heard.push(command);
    }
    fake.write_int32(turns, 9).unwrap();
    heard.extend(fake.heard.try_iter().map(|(_, command)| command));
    let set = heard
        .iter()
        .position(|c| c.starts_with("SetAnalyzerParameterValue"))
        .expect("the write was sent");
    let status = heard
        .iter()
        .position(|c| c == "GetAcquisitionStatus")
        .unwrap_or(heard.len());
    assert!(status < set, "{heard:?}");
}

/// The constructor's connect follows the task's first pass
/// (specsAnalyser.cpp:140-177), so a boot ends on its message.
#[test]
fn a_boot_ends_on_the_connect_message() {
    let fake = Fake::start("SPECS_WS_BOOTMSG");
    // Room for a first pass that ran after the constructor to land.
    std::thread::sleep(std::time::Duration::from_millis(300));
    assert_eq!(fake.status_message(), "Connected to SPECS");
}

/// An Acquire written as the constructor returns starts the acquisition
/// with C's function-scope locals (specsAnalyser.cpp:270-294, 300-303).
#[test]
fn an_acquire_after_the_constructor_starts_the_acquisition() {
    let fake = Fake::start("SPECS_WS_EARLY");
    fake.write_int32(fake.runtime.ad_params.acquire, 1).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let left = deadline.saturating_duration_since(std::time::Instant::now());
        let (_, command) = fake
            .heard
            .recv_timeout(left)
            .expect("the acquisition never started");
        if command.starts_with("Start") {
            break;
        }
    }
}

/// The task releases the driver around each update's callParamCallbacks
/// (specsAnalyser.cpp:617-619), so an Acquire=0 written during a poll lands
/// before the task re-reads ADAcquire and no further poll follows the Abort.
#[test]
fn an_abort_during_a_poll_lands_at_the_update_callbacks() {
    let fake = Fake::start("SPECS_WS_UPDATE");
    fake.answer(&[
        ("ValidateSpectrum", Act::Say("OK Samples:2", 0)),
        (
            "GetAcquisitionStatus",
            Act::Say("OK ControllerState:running NumberOfAcquiredPoints:1", 300),
        ),
        ("GetAcquisitionData", Act::Say("OK Data:[1,2,3]", 0)),
    ]);
    let acquire = fake.runtime.ad_params.acquire;
    fake.write_int32(acquire, 1).unwrap();
    let mut heard = Vec::new();
    while heard
        .iter()
        .filter(|c: &&String| *c == "GetAcquisitionStatus")
        .count()
        < 2
    {
        let (_, command) = fake
            .heard
            .recv_timeout(std::time::Duration::from_secs(5))
            .expect("the task never polled");
        heard.push(command);
    }
    fake.write_int32(acquire, 0).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while fake.status_message() != "Waiting for the acquire command" {
        assert!(
            std::time::Instant::now() < deadline,
            "the task never stopped"
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    heard.extend(fake.heard.try_iter().map(|(_, command)| command));
    let abort = heard.iter().position(|c| c == "Abort").expect("aborted");
    assert!(
        !heard[abort..].iter().any(|c| c == "GetAcquisitionStatus"),
        "{heard:?}"
    );
}

/// ADImageMode is re-read after the iteration loop (specsAnalyser.cpp:650-651),
/// so a Continuous run switched to Single mid-image ends after that image.
#[test]
fn an_image_mode_written_mid_image_ends_the_run() {
    let fake = Fake::start("SPECS_WS_MODE");
    fake.answer(&[
        ("ValidateSpectrum", Act::Say("OK Samples:2", 0)),
        (
            "GetAcquisitionStatus",
            Act::Say("OK ControllerState:finished NumberOfAcquiredPoints:2", 300),
        ),
        ("GetAcquisitionData", Act::Say("OK Data:[1,2,3,4,5,6]", 0)),
    ]);
    let ad = &fake.runtime.ad_params;
    fake.write_int32(ad.image_mode, 2).unwrap();
    fake.write_int32(ad.acquire, 1).unwrap();
    let mut heard = Vec::new();
    while !heard.iter().any(|c: &String| c == "GetAcquisitionStatus") {
        let (_, command) = fake
            .heard
            .recv_timeout(std::time::Duration::from_secs(5))
            .expect("the task never polled");
        heard.push(command);
    }
    fake.write_int32(ad.image_mode, 0).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while fake.int32(ad.acquire) != 0 {
        assert!(
            std::time::Instant::now() < deadline,
            "the run never ended: {:?}",
            fake.heard.try_iter().collect::<Vec<_>>()
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    heard.extend(fake.heard.try_iter().map(|(_, command)| command));
    let starts = heard.iter().filter(|c| c.starts_with("Start")).count();
    assert_eq!(starts, 1, "{heard:?}");
}
