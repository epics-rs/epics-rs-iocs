//! C `phytronIO` (phytronIOctrl.cpp) and the controller poll that reads it
//! (phytronAxisMotor.cpp:921-922, 976-980).

#![cfg(target_os = "linux")]

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::runtime::task::sleep;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandDef};
use motor_common::testing::{LineRig, port_report, run_command};

/// What the mock answers each command: the ACKed payload, or a NAK.
type Answers = Arc<Mutex<HashMap<String, Option<String>>>>;

/// A controller at address 0 that answers each command of a frame from
/// `answers` and ACKs any other; a phyMOTION's frames carry the CRC. An
/// output write (`AG1S<bits>`, `AG2=<v>`) is what its read answers next.
fn reply(answers: &Answers, frame: &str) -> String {
    let body = &frame[2..frame.len() - 1];
    let (body, crc) = match body.rsplit_once(':') {
        Some((b, _)) => (b, ":XX"),
        None => (body, ""),
    };
    let mut answers = answers.lock().unwrap();
    let mut out = String::from("\x02");
    for c in body.split(' ') {
        let write = c.split_once('=').or_else(|| c.split_once('S'));
        if let Some((card, v)) = write.filter(|_| c.starts_with("AG")) {
            answers.insert(format!("{card}R"), Some(v.to_owned()));
        }
        match answers.get(c) {
            Some(None) => out.push('\x15'),
            Some(Some(a)) => {
                out.push('\x06');
                out.push_str(a);
            }
            None => out.push('\x06'),
        }
    }
    out.push_str(crc);
    out.push('\x03');
    out
}

struct Rig {
    line: LineRig,
    _io: SyncIOHandle,
    answers: Answers,
    /// The command set whose registry holds the controller.
    commands: Vec<CommandDef>,
}

impl Rig {
    /// The frames since the last call but an MCC poll's SUI.
    fn sent(&self) -> Vec<String> {
        self.frames().into_iter().filter(|f| f != "SUI").collect()
    }

    /// The frames since the last call, each its commands.
    fn frames(&self) -> Vec<String> {
        self.line
            .sent()
            .iter()
            .map(|f| {
                let body = &f[2..f.len() - 1];
                body.rsplit_once(':').map_or(body, |(b, _)| b).to_owned()
            })
            .collect()
    }

    /// The commands since the last call, one per frame command.
    fn wire(&self) -> Vec<String> {
        self.frames()
            .iter()
            .flat_map(|f| f.split(' ').map(str::to_owned).collect::<Vec<_>>())
            .collect()
    }

    fn answer(&self, cmd: &str, a: Option<&str>) {
        self.answers
            .lock()
            .unwrap()
            .insert(cmd.to_owned(), a.map(str::to_owned));
    }

    /// `phytronCreateDigital` (or `phytronCreateAnalog`) `io` on controller
    /// `ctrl`, card `card`, channel `channel`.
    async fn create_io(
        &self,
        io: &str,
        ctrl: &str,
        analog: bool,
        card: i64,
        channel: i64,
        init: &str,
    ) {
        let name = if analog {
            "phytronCreateAnalog"
        } else {
            "phytronCreateDigital"
        };
        let create = self.commands.iter().find(|c| c.name == name).unwrap();
        let s = |v: &str| ArgValue::String(v.into());
        let args = vec![
            s(io),
            s(ctrl),
            ArgValue::Int(card),
            ArgValue::Int(channel),
            s(init),
        ];
        run_command(create.clone(), args).await;
    }

    /// Wait for a poll that starts after this call to end.
    async fn polled(&self, last: &str) {
        self.wire();
        for _ in 0..500 {
            if self.wire().iter().any(|c| c == last) {
                // The poll posts after it reads the last reply.
                sleep(Duration::from_millis(50)).await;
                return;
            }
            sleep(Duration::from_millis(10)).await;
        }
        panic!("no poll sent {last}");
    }
}

/// Controller `name` (phyMOTION when `phymotion`, else an MCC at address
/// 0) on a mock answering `answers`, polled every 20 ms, and no axes.
async fn controller(
    name: &str,
    phymotion: bool,
    answers: &[(&'static str, Option<&'static str>)],
) -> Rig {
    let answers = answers
        .iter()
        .map(|&(c, a)| (c.to_owned(), a.map(str::to_owned)));
    let answers: Answers = Arc::new(Mutex::new(answers.collect()));
    let io_port = format!("{name}_IO");
    let (line, io) = LineRig::new(&io_port, {
        let answers = answers.clone();
        move |frame| reply(&answers, frame)
    });
    let commands = motor_phytron::ioc::phytron_commands();
    let s = |v: &str| ArgValue::String(v.into());
    let mut args = vec![s(name), s(&io_port)];
    let create = if phymotion {
        commands[0].clone()
    } else {
        args.push(ArgValue::Int(0));
        commands[1].clone()
    };
    args.extend([
        ArgValue::Int(20),
        ArgValue::Int(20),
        ArgValue::Double(1000.0),
        ArgValue::Int(1),
    ]);
    run_command(create, args).await;
    line.sent();
    Rig {
        line,
        _io: io,
        answers,
        commands,
    }
}

/// Run `f` on a thread of its own, as a record's port thread does.
async fn off_runtime<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
    let thread = std::thread::spawn(f);
    while !thread.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    thread.join().unwrap()
}

fn port(name: &str) -> PortHandle {
    PortManager::global().find_port_handle(name).unwrap()
}

fn reason(port: &PortHandle, name: &str) -> usize {
    port.drv_user_create_blocking(&DrvUserRequest::new(name, 0))
        .unwrap()
        .reason
}

/// IN and OUT as a record reads them.
async fn in_out(io: &str) -> (i32, i32) {
    let port = port(io);
    off_runtime(move || {
        let read = |p| port.read_int32_blocking(reason(&port, p), 0).unwrap();
        (read("IN"), read("OUT"))
    })
    .await
}

/// A write of `value` to `param`.
async fn write(io: &str, param: &'static str, value: i32) -> bool {
    let port = port(io);
    off_runtime(move || {
        port.write_int32_blocking(reason(&port, param), 0, value)
            .is_ok()
    })
    .await
}

const MCC1: [(&str, Option<&str>); 5] = [
    ("IAR", Some("1")),
    ("1P48R", Some("0")),
    ("SUI", Some("I=00000000")),
    ("AG1R", Some("10000001")),
    ("EG1R", Some("01100000")),
];

/// An MCC's digital card is in and out, read as bit strings, first bit 0
/// (phytronIOctrl.cpp:361-372, 466-480); the poll reads it after SUI and
/// posts what it read (:334-344).
#[epics_rs::base::epics_test]
async fn an_mcc_digital_channel_reads_its_bits_at_creation_and_in_each_poll() {
    let rig = controller("phyIoMccD", false, &MCC1).await;
    rig.create_io("phyIoMccDio", "phyIoMccD", false, 1, 1, "")
        .await;
    assert_eq!(rig.sent()[..2], ["AG1R", "EG1R"], "one command a frame");
    assert_eq!(in_out("phyIoMccDio").await, (6, 0x81));
    rig.answer("EG1R", Some("11110000"));
    rig.polled("EG1R").await;
    assert_eq!(in_out("phyIoMccDio").await, (15, 0x81));
    let report = off_runtime(|| port_report("phyIoMccDio", 0)).await;
    assert!(
        report.contains("MCC-1 DIO: card=1 channel=1 in=15 out=129"),
        "{report}"
    );
    let report = off_runtime(|| port_report("phyIoMccDio", 2)).await;
    assert!(report.contains("name=IN, value=15"), "{report}");
}

/// An MCC writes its outputs as a bit string and an MCC-1 its direction
/// with `EAS` (phytronIOctrl.cpp:185-205), which leaves the outputs.
#[epics_rs::base::epics_test]
async fn an_mcc_1_writes_out_with_ag_and_dir_with_eas() {
    let rig = controller("phyIoMccW", false, &MCC1).await;
    rig.create_io("phyIoMccWio", "phyIoMccW", false, 1, 1, "")
        .await;
    rig.wire();
    assert!(write("phyIoMccWio", "OUT", 5).await);
    assert!(rig.wire().contains(&"AG1S10100000".to_owned()));
    assert_eq!(in_out("phyIoMccWio").await.1, 5);
    assert!(write("phyIoMccWio", "DIR", 3).await);
    assert!(rig.wire().contains(&"EAS1S11000000".to_owned()));
    let port = port("phyIoMccWio");
    let dir = off_runtime(move || port.read_int32_blocking(reason(&port, "DIR"), 0).unwrap()).await;
    assert_eq!((dir, in_out("phyIoMccWio").await.1), (3, 5));
    assert!(!write("phyIoMccWio", "IN", 1).await);
}

/// An MCC's analog card is an input read by channel (:368, :394-396),
/// which has nothing to write.
#[epics_rs::base::epics_test]
async fn an_mcc_analog_channel_is_an_input() {
    let rig = controller("phyIoMccA", false, &[("AD2R", Some("100"))]).await;
    rig.create_io("phyIoMccAin", "phyIoMccA", true, 1, 2, "")
        .await;
    assert_eq!(rig.sent()[0], "AD2R");
    assert_eq!(in_out("phyIoMccAin").await, (100, 0));
    assert!(!write("phyIoMccAin", "OUT", 1).await);
    let report = off_runtime(|| port_report("phyIoMccAin", 0)).await;
    assert!(
        report.contains(" AIN: card=1 channel=2 in=100 out=0"),
        "{report}"
    );
}

/// A second MCC card does not exist (:363-364): no port is created.
#[epics_rs::base::epics_test]
async fn an_mcc_has_no_second_card() {
    let rig = controller("phyIoMccC", false, &MCC1).await;
    rig.create_io("phyIoMccCio", "phyIoMccC", false, 2, 1, "")
        .await;
    assert!(
        PortManager::global()
            .find_port_handle("phyIoMccCio")
            .is_err()
    );
}

const PHYMOTION: [(&str, Option<&str>); 5] = [
    ("IM0", Some("phyMOTION")),
    ("AG2R", Some("12")),
    ("EG2R", Some("34")),
    ("DA1.1", None),
    ("AD1.1", Some("300")),
];

/// A guessed phyMOTION card is what it ACKs (:427-444): card 2 is in and
/// out, card 1 an analog input C reads from the NAK to its output request
/// and so never creates (:489-495). The init commands share the line.
#[epics_rs::base::epics_test]
async fn a_phymotion_card_is_what_its_requests_ack() {
    let rig = controller("phyIoPhy", true, &PHYMOTION).await;
    rig.create_io("phyIoPhyDio", "phyIoPhy", false, 2, 1, "XY")
        .await;
    assert_eq!(rig.frames()[0], "AG2R EG2R XY");
    rig.create_io("phyIoPhyAin", "phyIoPhy", true, 1, 1, "")
        .await;
    assert_eq!(in_out("phyIoPhyDio").await, (34, 12));
    assert_eq!(in_out("phyIoPhyAin").await, (300, 0));
    assert!(write("phyIoPhyDio", "OUT", 7).await);
    assert!(!write("phyIoPhyAin", "OUT", 7).await);
    assert!(rig.wire().contains(&"AG2=7".to_owned()));
}

/// The poll reads every channel on one line; a channel whose replies do
/// not parse fails the poll and leaves the next channel its own replies.
#[epics_rs::base::epics_test]
async fn a_channel_that_fails_to_parse_leaves_the_next_its_replies() {
    let rig = controller("phyIoPhyPoll", true, &PHYMOTION).await;
    rig.create_io("phyIoPollDio", "phyIoPhyPoll", false, 2, 1, "")
        .await;
    rig.create_io("phyIoPollAin", "phyIoPhyPoll", true, 1, 1, "")
        .await;
    rig.answer("AG2R", Some("zz"));
    rig.answer("AD1.1", Some("301"));
    rig.polled("AD1.1").await;
    assert_eq!(in_out("phyIoPollAin").await, (301, 0));
    assert_eq!(in_out("phyIoPollDio").await, (34, 12));
}
