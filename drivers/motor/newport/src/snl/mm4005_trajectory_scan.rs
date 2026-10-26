//! `MM4005_trajectoryScan.st`, compiled by snc-rs from `build.rs` and run on
//! `epics-seq-rs`.
//!
//! The escaped C follows: the `%{ }%` functions as Rust, on the octet user
//! `pasynOctetSyncIO->connect` made, then one `seqg_escaped!` arm per
//! fragment, its text as cpp hands it to snc. `getMotorPositions` is only
//! ever handed `pVar->motorCurrent`, and fills it here.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use epics_rs::asyn::error::OctetStep;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_seq_rs::c::*;
use epics_seq_rs::{ChId, CharPtr, CompType, SeqVar, StateSet};

type Env = StateSet<seqg::seqg_vars>;

/// The size of `stringIn` and of each command buffer.
const MAX_MM4000_STRING: usize = 256;
const POLL_INTERVAL: f64 = 0.1;

/// The user each `pasynOctetSyncIO->connect` made, by the `PORT` it named:
/// `pVar->pasynUser` is a `char *` here, which cannot hold one.
static USERS: Mutex<BTreeMap<String, Arc<SyncIOHandle>>> = Mutex::new(BTreeMap::new());

fn text(s: impl CSrc) -> String {
    s.with_c_bytes(|b| String::from_utf8_lossy(b).into_owned())
}

/// C `time(NULL)`.
fn time(_tloc: i32) -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs() as i64)
}

/// C `difftime(t1, t0)`, `t0` the `unsigned long` startTime holds.
fn difftime(t1: i64, t0: u64) -> f64 {
    t1 as f64 - t0 as i64 as f64
}

/// `numAxes` as a loop bound.
fn axes(v: &seqg::seqg_vars) -> std::ops::Range<usize> {
    0..usize::try_from(v.numAxes.get()).unwrap_or(0)
}

/// The C `int dir` of axis `j`: 1 when its motor record's `DIR` is Pos.
fn dir(v: &seqg::seqg_vars, j: usize) -> f64 {
    if v.epicsMotorDir[j].get() == 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// `pasynOctetSyncIO->connect(port, 0, &pasynUser, NULL)`: the status.
fn connect(port: CharPtr) -> i32 {
    // C's connectDevice refuses a NULL name (asynManager.c:1331) with the
    // status of a name it cannot find.
    let name = if port.is_null() {
        String::new()
    } else {
        text(&port)
    };
    let (user, status) =
        SyncIOHandle::connect_port(&name, 0, Duration::from_secs(1), InterfaceType::Octet);
    USERS.lock().unwrap().insert(name, Arc::new(user));
    status.map_or_else(|e| e.status() as i32, |()| 0)
}

/// The user `connect` made for this instance's `PORT`.
fn user(v: &seqg::seqg_vars) -> Arc<SyncIOHandle> {
    let port = v.asynPort.get();
    let name = if port.is_null() {
        String::new()
    } else {
        text(&port)
    };
    USERS
        .lock()
        .unwrap()
        .get(&name)
        .cloned()
        .expect("state init connects PORT")
}

/// `command` and its `\r`, as `strncpy(buffer, command, 253)` and `strcat`
/// leave them.
fn terminated(command: impl CSrc) -> Vec<u8> {
    // C's strncpy leaves a command of 253 bytes or more unterminated before
    // its strcat, which then runs past the buffer; it is cut there instead.
    let mut buffer = command.to_c_vec();
    buffer.truncate(MAX_MM4000_STRING - 3);
    buffer.push(b'\r');
    buffer
}

/// C `writeOnly`: the asyn status.
fn write_only(v: &seqg::seqg_vars, command: impl CSrc) -> i32 {
    user(v)
        .with_timeout(Duration::from_secs(1))
        .write_octet(0, &terminated(command))
        .map_or_else(|e| e.status() as i32, |_| 0)
}

/// C `writeRead`: the asyn status, and `stringIn` emptied, then the reply
/// as the port's read leaves it there. An exchange that failed before its
/// read leaves it empty.
fn write_read(v: &seqg::seqg_vars, command: impl CSrc) -> i32 {
    v.stringIn.with_c_dst(|b| b.fill(0));
    let (read, status) = user(v)
        .with_timeout(Duration::from_secs(30))
        .write_read(0, &terminated(command), MAX_MM4000_STRING)
        .into_parts();
    let read_ran = match &status {
        Ok(()) => true,
        Err(e) => e.octet_step() == Some(OctetStep::Read),
    };
    if read_ran {
        v.stringIn.with_c_dst(|b| {
            let n = read.len().min(b.len());
            b[..n].copy_from_slice(&read[..n]);
        });
    }
    status.map_or_else(|e| e.status() as i32, |()| 0)
}

/// C `getMotorPositions(ssId, pVar, pVar->motorCurrent)`: the `TP`
/// positions, in user units, for as many axes as the reply has.
fn get_motor_positions(v: &seqg::seqg_vars) -> i32 {
    write_read(v, "TP");
    // 1TP2.65,2TP3.1 ...
    let tok_save = SeqVar::new(CharPtr::default());
    let mut p = epicsStrtok_r(&v.stringIn, ",", &tok_save);
    for j in axes(v) {
        if p.is_null() {
            break;
        }
        v.motorCurrent[j].set(atof(p.offset(3)) * dir(v, j) + v.epicsMotorOff[j].get());
        p = epicsStrtok_r(CharPtr::default(), ",", &tok_save);
    }
    0
}

/// C `getMotorMoving`: a bit per axis whose `MS` status byte has its
/// moving bit set.
fn get_motor_moving(v: &seqg::seqg_vars) -> i32 {
    write_read(v, "MS");
    // 1MSA,2MS@ ...
    let tok_save = SeqVar::new(CharPtr::default());
    let mut p = epicsStrtok_r(&v.stringIn, ",", &tok_save);
    let mut result = 0;
    for j in axes(v) {
        if p.is_null() {
            break;
        }
        if p.offset(3).get() & 0x01 != 0 {
            result |= 1 << j;
        }
        p = epicsStrtok_r(CharPtr::default(), ",", &tok_save);
    }
    result
}

/// C `getEpicsMotorMoving`: a bit per axis whose motor record moves.
fn get_epics_motor_moving(env: &Env) -> i32 {
    let v = env.var();
    let mut result = 0;
    for j in axes(v) {
        env.pv_get(
            ChId(v.epicsMotorDoneIndex.at(j).get() as u32),
            CompType::Default,
        );
        if v.epicsMotorDone[j].get() == 0.0 {
            result |= 1 << j;
        }
    }
    result
}

/// C `waitEpicsMotors`: post each motor record's position as
/// `motorCurrent` until none moves.
fn wait_epics_motors(env: &Env) {
    let v = env.var();
    let post = || {
        for j in axes(v) {
            v.motorCurrent[j].set(v.epicsMotorPos[j].get());
            env.pv_put(
                ChId(v.motorCurrentIndex.at(j).get() as u32),
                CompType::Default,
            );
        }
    };
    while get_epics_motor_moving(env) != 0 {
        post();
        epicsThreadSleep(POLL_INTERVAL);
    }
    post();
}

macro_rules! seqg_escaped {
    (r#"#include <stdlib.h>"#) => {};
    (r#"#include <string.h>"#) => {};
    (r#"#include <stdio.h>"#) => {};
    (r#"#include <epicsString.h>"#) => {};
    (r#"#include <asynOctetSyncIO.h>"#) => {};
    (r#"static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command);"#) => {};
    (r#"static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command);"#) => {};
    (r#"static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);"#) => {};
    (r#"static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command) { asynStatus status; size_t nwrite; char buffer[256]; strncpy(buffer, command, 256 -3); strcat(buffer, "\r"); status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer, strlen(buffer), 1.0, &nwrite); return(status); } static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command) { asynStatus status; size_t nwrite, nread; int eomReason; char buffer[256]; strncpy(buffer, command, 256 -3); strcat(buffer, "\r"); memset(pVar->stringIn, 0, 256); status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer, strlen(buffer), pVar->stringIn, 256, 30.0, &nwrite, &nread, &eomReason); return(status); } static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos) { char *p, *tok_save; int j; int dir; writeRead(ssId, pVar, "TP"); tok_save = 0; p = epicsStrtok_r(pVar->stringIn, ",", &tok_save); for (j=0; (j<pVar->numAxes && p!=0); j++) { if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1; pos[j] = atof(p+3)*dir + pVar->epicsMotorOff[j]; p = epicsStrtok_r(0, ",", &tok_save); } return(0); } static int getMotorMoving(SS_ID ssId, struct UserVar *pVar) { char *p, *tok_save; int j; int result=0, mask=0x01; writeRead(ssId, pVar, "MS"); tok_save = 0; p = epicsStrtok_r(pVar->stringIn, ",", &tok_save); for (j=0; (j<pVar->numAxes && p!=0); j++) { if (*(p+3) & 0x01) result |= mask; mask = mask << 1; p = epicsStrtok_r(0, ",", &tok_save); } return(result); } static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar) { int j; int result=0, mask=0x01; for (j=0; j<pVar->numAxes; j++) { seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0); if (pVar->epicsMotorDone[j] == 0) result |= mask; mask = mask << 1; } return(result); } static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar) { int j; while(getEpicsMotorMoving(ssId, pVar)) { for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } epicsThreadSleep(0.1); } for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } return(0); }"#) => {};
    ($env:ident, r#"pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0,"#, r#"(asynUser **)&pVar->pasynUser,"#, r#"NULL);"#) => {
        $env.var().status.set(connect($env.var().asynPort.get()))
    };
    ($env:ident, r#"writeOnly(ssId, pVar, pVar->stringOut);"#) => {
        write_only($env.var(), &$env.var().stringOut)
    };
    ($env:ident, r#"writeOnly(ssId, pVar, "AB");"#) => {
        write_only($env.var(), "AB")
    };
    ($env:ident, r#"writeRead(ssId, pVar, pVar->stringOut);"#) => {
        write_read($env.var(), &$env.var().stringOut)
    };
    ($env:ident, r#"writeRead(ssId, pVar, "TB");"#) => {
        write_read($env.var(), "TB")
    };
    ($env:ident, r#"writeRead(ssId, pVar, "XC1");"#) => {
        write_read($env.var(), "XC1")
    };
    ($env:ident, r#"writeRead(ssId, pVar, "NQ");"#) => {
        write_read($env.var(), "NQ")
    };
    ($env:ident, r#"waitEpicsMotors(ssId, pVar);"#) => {
        wait_epics_motors($env)
    };
    ($env:ident, r#"getMotorPositions(ssId, pVar, pVar->motorCurrent);"#) => {
        get_motor_positions($env.var())
    };
    ($env:ident, r#"pVar->anyMoving = getMotorMoving(ssId, pVar);"#) => {
        $env.var().anyMoving.set(get_motor_moving($env.var()))
    };
}

epics_seq_rs::include_program!("MM4005_trajectoryScan");

#[cfg(test)]
mod tests {
    use epics_seq_rs::{ProgOptions, ProgramVars, VarModel};
    use motor_common::testing::LineRig;

    use super::*;

    /// MM4005_trajectoryScan.st:1, with `option +r;` (:23).
    #[test]
    fn the_program_is_the_st_one() {
        assert_eq!(MM4005_trajectoryScan.validate(), Ok(()));
        assert_eq!(
            MM4005_trajectoryScan.params,
            "P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1"
        );
        assert_eq!(
            MM4005_trajectoryScan.options,
            ProgOptions::new(ProgOptions::CONN | ProgOptions::NEWEF)
        );
        assert!(matches!(MM4005_trajectoryScan.vars, VarModel::Reentrant));
    }

    /// One axis's variables on a [`LineRig`] port `name` answering `reply`.
    fn vars(
        name: &str,
        reply: impl Fn(&str) -> String + Send + Sync + 'static,
    ) -> (seqg::seqg_vars, LineRig) {
        let (rig, _) = LineRig::new(name, reply);
        let v = seqg::seqg_vars::zeroed();
        v.asynPort.set(CharPtr::from(name));
        assert_eq!(connect(v.asynPort.get()), 0);
        v.numAxes.set(1);
        (v, rig)
    }

    /// writeOnly ends the command with `\r` and waits a second; writeRead
    /// waits thirty.
    #[test]
    fn commands_are_terminated_and_timed_as_c() {
        let (v, rig) = vars("mm4005_terminated", |_| "TB@ no error".into());
        assert_eq!(write_only(&v, "NC"), 0);
        assert_eq!(write_read(&v, "TB"), 0);
        assert_eq!(rig.sent(), ["NC\r", "TB\r"]);
        assert_eq!(
            rig.timeouts(),
            [Some(Duration::from_secs(1)), Some(Duration::from_secs(30))]
        );
        assert_eq!(text(&v.stringIn), "TB@ no error");
        let long = "x".repeat(300);
        assert_eq!(write_only(&v, long.as_str()), 0);
        assert_eq!(rig.sent(), [format!("{}\r", &long[..253])]);
    }

    /// The reply lands in an emptied `stringIn`, none of the last behind it.
    #[test]
    fn a_reply_replaces_the_last() {
        let (v, _rig) = vars("mm4005_over", |cmd| match cmd {
            "1GC?\r" => "1GC0.25".into(),
            _ => "XC12".into(),
        });
        strcpy(&v.stringOut, "1GC?");
        write_read(&v, &v.stringOut);
        write_read(&v, "XC1");
        v.stringIn
            .with_c_dst(|b| assert_eq!(&b[..8], b"XC12\0\0\0\0"));
    }

    /// A refused write leaves `stringIn` empty: C's writeRead returns before
    /// its read, after the memset.
    #[test]
    fn a_failed_write_empties_the_reply() {
        let (rig, _) = LineRig::failing(
            "mm4005_refused",
            |_| "TB@ no error".into(),
            |cmd| cmd == "XC1\r",
        );
        let v = seqg::seqg_vars::zeroed();
        v.asynPort.set(CharPtr::from("mm4005_refused"));
        connect(v.asynPort.get());
        assert_eq!(write_read(&v, "TB"), 0);
        assert_ne!(write_read(&v, "XC1"), 0);
        assert_eq!(text(&v.stringIn), "");
        assert_eq!(rig.sent(), ["TB\r", "XC1\r"]);
    }

    /// C's connectDevice refuses a port nobody made, and the user it
    /// leaves fails each transfer.
    #[test]
    fn an_unknown_port_connects_to_nothing() {
        let v = seqg::seqg_vars::zeroed();
        v.asynPort.set(CharPtr::from("mm4005_nowhere"));
        assert_ne!(connect(v.asynPort.get()), 0);
        assert_ne!(write_only(&v, "NC"), 0);
        assert_ne!(write_read(&v, "TB"), 0);
        assert_eq!(text(&v.stringIn), "");
    }

    /// getMotorPositions converts `TP` dial positions to user units, axis
    /// by axis while the reply lasts.
    #[test]
    fn motor_positions_are_user_units() {
        let (v, rig) = vars("mm4005_tp", |_| "1TP2.5,2TP-3".into());
        v.numAxes.set(3);
        v.epicsMotorDir[1].set(1.0);
        v.epicsMotorOff[0].set(10.0);
        v.motorCurrent[2].set(7.0);
        assert_eq!(get_motor_positions(&v), 0);
        let got: Vec<f64> = (0..3).map(|j| v.motorCurrent[j].get()).collect();
        assert_eq!(got, [12.5, 3.0, 7.0]);
        assert_eq!(rig.sent(), ["TP\r"]);
    }

    /// getMotorMoving reads the moving bit of each `MS` status byte.
    #[test]
    fn moving_is_the_low_bit_of_each_status() {
        let (v, rig) = vars("mm4005_ms", |_| "1MSA,2MS@,3MSC".into());
        v.numAxes.set(3);
        assert_eq!(get_motor_moving(&v), 0b101);
        assert_eq!(rig.sent(), ["MS\r"]);
    }

    /// `startTime` is the `unsigned long` of a signed `time_t`.
    #[test]
    fn difftime_is_seconds_since_start() {
        assert_eq!(difftime(105, 100), 5.0);
        assert!(time(0) > 1_700_000_000);
    }

    mod program {
        use std::collections::HashMap;
        use std::sync::{Arc, Mutex};
        use std::time::Duration;

        use epics_rs::asyn::interfaces::motor::AsynMotor;
        use epics_rs::base::runtime::task::sleep;
        use epics_rs::base::server::database::PvDatabase;
        use epics_rs::base::server::ioc_builder::IocBuilder;
        use epics_rs::base::server::scan::ScanOwner;
        use epics_rs::base::types::EpicsValue;
        use epics_rs::motor::builder::{AxisSetup, MotorBuilder};
        use epics_rs::motor::sim_motor::SimMotor;
        use epics_rs::motor::trace::TraceUser;
        use epics_seq_rs::{SeqThreadId, seq, seq_stop};
        use motor_common::testing::LineRig;

        use super::super::*;

        const DB: &str = include_str!("../../../../../iocs/motor/mm4000-ioc/db/trajectoryScan.db");

        /// m1 on a simulated axis at dial 0, user offset `$(OFF)`.
        const MOTOR: &str = r#"record(motor, "T:m1") { field(DTYP, "asynMotor")
            field(OUT, "@asyn($(PORT)m1,0)") field(MRES, "0.001") field(VELO, "1")
            field(VMAX, "1") field(ACCL, "0.2") field(DHLM, "100") field(DLLM, "-100")
            field(OFF, "$(OFF)") }"#;

        /// What the fake MM4005 answers.
        struct Mm4005 {
            /// `MS` polls still to answer as moving.
            moving: u32,
            /// The `TB` reply.
            tb: String,
            /// The dial position `TP` answers.
            position: f64,
            /// The points `NQ` says were traced.
            traced: usize,
            /// Whether `1TQ` answers no axis and `2TQ` a cut one.
            short: bool,
            /// Whether the `TB` that reads `VC`'s result is refused.
            refuse_verify: bool,
            /// Whether `VC` was the last write.
            verifying: bool,
            /// Whether `XC1` takes 300 ms to answer.
            slow_xc: bool,
            /// Whether `TB` takes 300 ms to answer.
            slow_tb: bool,
            /// Whether `AB` was written, which stops the axes.
            aborted: bool,
        }

        type Shared = Arc<Mutex<Mm4005>>;

        fn reply(mm: &Shared, cmd: &str) -> String {
            let cmd = cmd.trim_end_matches('\r');
            let mut m = mm.lock().unwrap();
            match cmd {
                "TB" if m.slow_tb => {
                    let tb = m.tb.clone();
                    drop(m);
                    std::thread::sleep(Duration::from_millis(300));
                    tb
                }
                "TB" => m.tb.clone(),
                "TP" => format!("1TP{}", m.position),
                "XC1" if m.slow_xc => {
                    drop(m);
                    std::thread::sleep(Duration::from_millis(300));
                    "XC4".into()
                }
                "XC1" => "XC4".into(),
                "MS" if m.moving > 0 && !m.aborted => {
                    m.moving -= 1;
                    "1MSA".into()
                }
                "MS" => "1MS@".into(),
                "NQ" => format!("NQ{}", m.traced),
                "1GC?" => "1GC0.25".into(),
                _ => match cmd.split_once("RC") {
                    // 1RCn: element n of RC1, RC3, RC5, the value of the others
                    Some((axis, n)) => {
                        format!(
                            "{axis}RC{}",
                            ["7", "0.5", "8", "1.5", "9", "2.5"][n.parse::<usize>().unwrap() - 1]
                        )
                    }
                    // nTQ: point n, theory then actual
                    None => match cmd.strip_suffix("TQ") {
                        Some(n) => match (m.short, n) {
                            (true, "1") => "1TQ".into(),
                            (true, "2") => "2TQ,1TH2,1T".into(),
                            _ => {
                                let n: f64 = n.parse().unwrap();
                                format!("{n}TQ,1TH{},1TP{}", n, n + 0.25)
                            }
                        },
                        None => String::new(),
                    },
                },
            }
        }

        /// The fake's side of the write of `cmd`: whether it is refused.
        fn written(mm: &Shared, cmd: &str) -> bool {
            let mut m = mm.lock().unwrap();
            m.aborted |= cmd == "AB\r";
            let verify = std::mem::replace(&mut m.verifying, cmd == "VC\r");
            m.refuse_verify && verify && cmd == "TB\r"
        }

        /// `port`'s axis m1 of one simulated motor.
        fn sim_axis(port: &str) {
            let motor: Arc<Mutex<dyn AsynMotor>> =
                Arc::new(Mutex::new(SimMotor::new().with_limits(-1e9, 1e9)));
            let name = format!("{port}m1");
            let AxisSetup {
                axis, poll_loop, ..
            } = MotorBuilder::new(motor)
                .trace_user(TraceUser::Asyn {
                    port: name.clone(),
                    addr: 0,
                })
                .poll_interval(Duration::from_millis(10))
                .build_axis();
            epics_rs::motor::devices::register_controller_port(
                &epics_rs::asyn::registry::open_owner(),
                &name,
                1,
                (),
            )
            .unwrap();
            epics_rs::motor::devices::register_axis(axis).unwrap();
            poll_loop.start();
        }

        async fn put(db: &PvDatabase, pv: &str, v: EpicsValue) {
            db.put_record_field_from_ca(pv, "VAL", v).await.unwrap();
        }

        fn value(db: &PvDatabase, pv: &str) -> f64 {
            db.get_pv(pv).unwrap().to_f64().unwrap()
        }

        fn array(db: &PvDatabase, pv: &str, n: usize) -> Vec<f64> {
            match db.get_pv(pv).unwrap() {
                EpicsValue::DoubleArray(a) => a.as_slice()[..n].to_vec(),
                v => panic!("{pv}: {v:?}"),
            }
        }

        fn string(db: &PvDatabase, pv: &str) -> String {
            match db.get_pv(pv).unwrap() {
                EpicsValue::String(s) => s.as_str_lossy().into_owned(),
                v => panic!("{pv}: {v:?}"),
            }
        }

        /// Poll until every `(pv, value)` holds; on timeout, name what each
        /// read.
        async fn expect(db: &PvDatabase, want: &[(&str, f64)]) {
            for _ in 0..500 {
                if want.iter().all(|&(pv, v)| value(db, pv) == v) {
                    return;
                }
                sleep(Duration::from_millis(10)).await;
            }
            let got: Vec<_> = want.iter().map(|&(pv, _)| (pv, value(db, pv))).collect();
            panic!("want {want:?}, got {got:?}");
        }

        struct Program {
            db: Arc<PvDatabase>,
            mm: Shared,
            rig: LineRig,
            /// The commands sent, each less its `\r`.
            log: Mutex<Vec<String>>,
            tid: SeqThreadId,
            _scan: ScanOwner,
        }

        impl Drop for Program {
            fn drop(&mut self) {
                seq_stop(self.tid);
            }
        }

        impl Program {
            /// The commands sent since the last call.
            fn sent(&self) -> Vec<String> {
                let mut log = self.log.lock().unwrap();
                log.extend(
                    self.rig
                        .sent()
                        .into_iter()
                        .map(|c| c.trim_end_matches('\r').to_string()),
                );
                std::mem::take(&mut *log)
            }

            /// Poll until `cmd` is sent, keeping what came before it for
            /// [`Self::sent`].
            async fn until(&self, cmd: &str) {
                for _ in 0..500 {
                    let sent = self.sent();
                    let seen = sent.iter().any(|c| c == cmd);
                    self.log.lock().unwrap().splice(0..0, sent);
                    if seen {
                        return;
                    }
                    sleep(Duration::from_millis(10)).await;
                }
                panic!("never sent {cmd}: {:?}", self.sent());
            }

            async fn build(&self) {
                put(&self.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
                expect(&self.db, &[("T:traj1:Build", 0.0)]).await;
            }
        }

        /// trajectoryScan.db on NumAxes 1, T:m1 on a simulated axis, and the
        /// program on a fake MM4005 port named `port`, once init has read
        /// the axis's MDVS and posted it.
        async fn boot(port: &str, set: &[(&str, &str)]) -> Program {
            let mm = Arc::new(Mutex::new(Mm4005 {
                moving: 0,
                tb: "TB@ no error".into(),
                position: 0.5,
                traced: 0,
                short: false,
                refuse_verify: false,
                verifying: false,
                slow_xc: false,
                slow_tb: false,
                aborted: false,
            }));
            let (shared, writes) = (mm.clone(), mm.clone());
            let (rig, _) = LineRig::failing(
                port,
                move |cmd| reply(&shared, cmd),
                move |cmd| written(&writes, cmd),
            );
            sim_axis(port);
            let mut macros: HashMap<String, String> = [
                ("P", "T:"),
                ("R", "traj1:"),
                ("NAXES", "1"),
                ("NELM", "2000"),
                ("NPULSE", "2000"),
                ("PORT", port),
                ("OFF", "0"),
            ]
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .into();
            macros.extend(set.iter().map(|&(k, v)| (k.to_string(), v.to_string())));
            let (motor, factory) = epics_rs::motor::motor_record_factory();
            let (db, _) = IocBuilder::new()
                .register_record_type(motor, factory)
                .register_dynamic_device_support(
                    &epics_rs::motor::devices::devices(),
                    epics_rs::motor::devices::device_support_factory(),
                )
                .register_record_type("busy", || Box::new(epics_rs::busy::BusyRecord::default()))
                .db_string(DB, &macros)
                .unwrap()
                .db_string(MOTOR, &macros)
                .unwrap()
                .build_served()
                .await
                .unwrap();
            // seq holds each state set in its first state until PINI is done.
            let scan = ScanOwner::start(db.clone());
            let tid = seq(
                &MM4005_trajectoryScan,
                &format!("P=T:,R=traj1:,M1=m1,PORT={port}"),
                0,
                &db,
            )
            .expect("MM4005_trajectoryScan starts");
            let p = Program {
                db,
                mm,
                rig,
                log: Mutex::default(),
                tid,
                _scan: scan,
            };
            p.until("1GC?").await;
            expect(&p.db, &[("T:traj1:M1MDVS", 0.25)]).await;
            p
        }

        /// ss trajectoryScan state build: the elements, pulses and verify
        /// sent, and the dynamic parameters read back.
        #[epics_rs::base::epics_test]
        async fn build_defines_and_verifies_the_trajectory() {
            let p = boot("mm4005Build", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(4)).await;
            // monitor_inputs moves EndPulses to it.
            expect(&p.db, &[("T:traj1:EndPulses", 4.0)]).await;
            p.sent();
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 1.0)]).await;
            let mut want = vec!["NC".to_string(), "1DC1".into(), "UC0.500000".into()];
            for i in 1..=4 {
                want.extend([format!("{i}DT2.500000"), "1DX0.000000".into()]);
            }
            want.extend(["MB1,ME4,MN11", "VC", "TB"].map(String::from));
            want.extend((1..=6).map(|n| format!("1RC{n}")));
            assert_eq!(p.sent(), want);
            let got = ["MDVE", "MDVA", "MVE", "MVA", "MAE", "MAA"]
                .map(|f| value(&p.db, &format!("T:traj1:M1{f}")));
            assert_eq!(got, [7.0, 0.5, 8.0, 1.5, 9.0, 2.5]);
            assert_eq!(string(&p.db, "T:traj1:BuildMessage"), " ");
        }

        /// ss trajectoryScan state init reads NumAxes once, so a put past
        /// MAX_AXES after it leaves the build's per-axis loops on its one
        /// axis.
        #[epics_rs::base::epics_test]
        async fn a_num_axes_put_after_init_leaves_the_build_on_its_axes() {
            let p = boot("mm4005NumAxesPut", &[]).await;
            put(&p.db, "T:traj1:NumAxes", EpicsValue::Long(9)).await;
            p.sent();
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 1.0)]).await;
            let sent = p.sent();
            let axes: Vec<_> = sent.iter().filter(|c| c.contains("DC")).collect();
            assert_eq!(axes, ["1DC1"]);
            assert_eq!(sent.iter().filter(|c| c.contains("RC")).count(), 6);
        }

        /// The element times `DT` sent since the last [`Program::sent`].
        fn elements(p: &Program) -> usize {
            p.sent().iter().filter(|c| c.contains("DT")).count()
        }

        /// ss trajectoryScan state build bounds each build where its arrays
        /// end: MAX_ELEMENTS relative, and absolute MAX_ELEMENTS-3 with a moved
        /// axis, whose padding writes past the last element, else
        /// MAX_ELEMENTS. An absolute build posts its padded count, as C.
        #[epics_rs::base::epics_test]
        async fn each_build_is_bounded_where_its_arrays_end() {
            let p = boot("mm4005Bounds", &[]).await;
            // (M1Move, MoveMode, Nelements put, Nelements after, DT, DX)
            for (axis, mode, n, after, dt, dx) in [
                (1, 0, 2000, 2000.0, 2000, 2000),
                (1, 0, 2001, 2000.0, 2000, 2000),
                (0, 0, 2000, 2000.0, 2000, 0),
                (0, 0, 2001, 2000.0, 2000, 0),
                (1, 1, 1997, 1997.0, 1996, 1996),
                (1, 1, 1998, 1997.0, 1996, 1996),
                (0, 1, 2000, 2001.0, 2000, 0),
                (0, 1, 2001, 2001.0, 2000, 0),
            ] {
                put(&p.db, "T:traj1:M1Move", EpicsValue::Enum(axis)).await;
                put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(mode)).await;
                put(&p.db, "T:traj1:Nelements", EpicsValue::Long(n)).await;
                p.sent();
                p.build().await;
                expect(
                    &p.db,
                    &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", after)],
                )
                .await;
                let sent = p.sent();
                let count = |s: &str| sent.iter().filter(|c| c.contains(s)).count();
                assert_eq!((count("DT"), count("DX")), (dt, dx), "{axis} {mode} {n}");
            }
        }

        /// ss trajectoryScan state build sends an absolute trajectory of
        /// 1998 elements with no axis moved as C does: padded to 2001, with
        /// no motorTrajectory to fit.
        #[epics_rs::base::epics_test]
        async fn an_absolute_build_with_no_moved_axis_is_sent_whole() {
            let p = boot("mm4005AbsoluteNoAxis", &[]).await;
            put(&p.db, "T:traj1:M1Move", EpicsValue::Enum(0)).await;
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(1998)).await;
            expect(&p.db, &[("T:traj1:EndPulses", 1998.0)]).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", 2001.0)],
            )
            .await;
            let mut want: Vec<String> = ["NC", "1DC0", "UC0.500000"].map(String::from).into();
            for i in 1..=2000 {
                let t = if i <= 1997 { 10.0 / 1998.0 } else { 0.1 };
                want.push(format!("{i}DT{t:.6}"));
                if i % 20 == 0 {
                    want.push("TB".into());
                }
            }
            want.extend(["MB1,ME1998,MN11", "VC", "TB"].map(String::from));
            assert_eq!(p.sent(), want);
        }

        /// ss trajectoryScan state build declares, pads and moves the axes
        /// moveAxis named when it began: an M1Move put while it waits on the
        /// twentieth element's TB leaves its DX set as declared, either way.
        /// The dynamic-parameter reads after VC read moveAxis live, as C.
        #[epics_rs::base::epics_test]
        async fn a_move_axis_put_mid_build_leaves_the_builds_axes() {
            let p = boot("mm4005MoveAxisMidBuild", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(40)).await;
            expect(&p.db, &[("T:traj1:EndPulses", 40.0)]).await;
            p.mm.lock().unwrap().slow_tb = true;
            for (declared, then, dx, rc) in [("1DC1", 0, 40, 0), ("1DC0", 1, 0, 6)] {
                p.sent();
                put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
                p.until("TB").await;
                put(&p.db, "T:traj1:M1Move", EpicsValue::Enum(then)).await;
                expect(
                    &p.db,
                    &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
                )
                .await;
                let sent = p.sent();
                assert!(sent.iter().any(|c| c == declared), "{sent:?}");
                assert_eq!(sent.iter().filter(|c| c.contains("DT")).count(), 40);
                assert_eq!(sent.iter().filter(|c| c.contains("DX")).count(), dx);
                assert_eq!(sent.iter().filter(|c| c.contains("RC")).count(), rc);
            }
        }

        /// ss trajectoryScan state build sends a relative trajectory of no
        /// elements as C does: no element, no padding, then the pulses and
        /// the verify.
        #[epics_rs::base::epics_test]
        async fn a_relative_build_of_no_elements_sends_none() {
            let p = boot("mm4005RelativeEmpty", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(0)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", 0.0)],
            )
            .await;
            let mut want: Vec<String> = ["NC", "1DC1", "UC0.500000", "MB0,ME0,MN11", "VC", "TB"]
                .map(String::from)
                .into();
            want.extend((1..=6).map(|n| format!("1RC{n}")));
            assert_eq!(p.sent(), want);
        }

        /// ss trajectoryScan state build sends an absolute trajectory of one
        /// element, which has no move, as C does: no element.
        #[epics_rs::base::epics_test]
        async fn an_absolute_build_of_one_element_sends_none() {
            let p = boot("mm4005AbsoluteEmpty", &[]).await;
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(1)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", 1.0)],
            )
            .await;
            assert_eq!(elements(&p), 0);
        }

        /// ss trajectoryScan state build keeps a count below those that C does
        /// not pad, a point count that is a multiple of 4: no element.
        #[epics_rs::base::epics_test]
        async fn a_build_below_its_least_elements_c_does_not_pad_is_kept() {
            let p = boot("mm4005BelowUnpadded", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(-4)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", -4.0)],
            )
            .await;
            let mut want: Vec<String> = ["NC", "1DC1", "UC0.500000", "MB-4,ME-4,MN11", "VC", "TB"]
                .map(String::from)
                .into();
            want.extend((1..=6).map(|n| format!("1RC{n}")));
            assert_eq!(p.sent(), want);
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(-3)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", -3.0)],
            )
            .await;
            assert_eq!(elements(&p), 0);
        }

        /// ss trajectoryScan state build raises a count below those that C
        /// pads, from before the arrays, to them.
        #[epics_rs::base::epics_test]
        async fn a_build_below_its_least_elements_builds_the_least() {
            let p = boot("mm4005BelowLeast", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(-1)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", 0.0)],
            )
            .await;
            assert_eq!(elements(&p), 0);
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(0)).await;
            p.sent();
            p.build().await;
            expect(
                &p.db,
                &[("T:traj1:BuildStatus", 1.0), ("T:traj1:Nelements", 1.0)],
            )
            .await;
            assert_eq!(elements(&p), 0);
        }

        /// ss trajectoryScan state build: a TB error after VC fails the
        /// build with the controller's message.
        #[epics_rs::base::epics_test]
        async fn a_tb_error_fails_the_build() {
            let p = boot("mm4005BuildError", &[]).await;
            p.mm.lock().unwrap().tb = "TBE trajectory error".into();
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 2.0)]).await;
            assert_eq!(
                string(&p.db, "T:traj1:BuildMessage"),
                "TBE trajectory error"
            );
        }

        /// ss trajectoryScan state build: a refused TB after VC fails the
        /// build, though the TB before it, at the twentieth element, answered
        /// no error.
        #[epics_rs::base::epics_test]
        async fn a_refused_verify_read_fails_the_build() {
            let p = boot("mm4005VerifyRefused", &[]).await;
            p.mm.lock().unwrap().refuse_verify = true;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(20)).await;
            p.sent();
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 2.0)]).await;
            assert_eq!(p.sent().iter().filter(|c| *c == "TB").count(), 2);
            assert_eq!(string(&p.db, "T:traj1:BuildMessage"), "");
        }

        /// ss trajectoryScan states execute and wait_execute: the run polled
        /// until the axes stop, then the flyback to the controller's
        /// position.
        #[epics_rs::base::epics_test]
        async fn execute_runs_until_the_axes_stop() {
            let p = boot("mm4005Execute", &[]).await;
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 1.0)]).await;
            p.mm.lock().unwrap().moving = 2;
            p.sent();
            put(&p.db, "T:traj1:Execute", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Execute", 0.0),
                    ("T:traj1:ExecState", 0.0),
                    ("T:traj1:ExecStatus", 1.0),
                ],
            )
            .await;
            let poll = ["TP", "XC1", "MS", "TB"];
            let mut want = vec!["LS,0EC1.000000"];
            for _ in 0..3 {
                want.extend(poll);
            }
            want.push("TP");
            assert_eq!(p.sent(), want);
            assert_eq!(value(&p.db, "T:m1"), 0.5);
            assert_eq!(value(&p.db, "T:traj1:M1Current"), 0.5);
            assert_eq!(string(&p.db, "T:traj1:ExecMessage"), " ");
        }

        /// ss trajectoryScan state wait_execute: an abort while an EXECUTING
        /// pass polls ends the run aborted, though that pass then finds the
        /// axes stopped.
        #[epics_rs::base::epics_test]
        async fn an_abort_during_a_poll_ends_the_run_aborted() {
            let p = boot("mm4005AbortPoll", &[]).await;
            p.build().await;
            expect(&p.db, &[("T:traj1:BuildStatus", 1.0)]).await;
            {
                let mut m = p.mm.lock().unwrap();
                m.moving = u32::MAX;
                m.slow_xc = true;
            }
            p.sent();
            put(&p.db, "T:traj1:Execute", EpicsValue::Enum(1)).await;
            p.until("XC1").await;
            put(&p.db, "T:traj1:Abort", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Execute", 0.0), ("T:traj1:ExecState", 0.0)],
            )
            .await;
            assert_eq!(value(&p.db, "T:traj1:ExecStatus"), 3.0);
            assert_eq!(string(&p.db, "T:traj1:ExecMessage"), "Motion aborted");
        }

        /// ss trajectoryScan state execute: an abort during the move to an
        /// absolute trajectory's start sends no execute command, and the run
        /// ends aborted with AB sent after it.
        #[epics_rs::base::epics_test]
        async fn an_abort_during_the_move_to_the_start_executes_nothing() {
            let p = boot("mm4005AbortStart", &[]).await;
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(
                &p.db,
                "T:traj1:M1Traj",
                EpicsValue::DoubleArray(vec![1.0; 5].into()),
            )
            .await;
            p.sent();
            put(&p.db, "T:traj1:Execute", EpicsValue::Enum(1)).await;
            expect(&p.db, &[("T:traj1:ExecState", 1.0), ("T:m1.DMOV", 0.0)]).await;
            put(&p.db, "T:traj1:Abort", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Execute", 0.0),
                    ("T:traj1:ExecState", 0.0),
                    ("T:traj1:ExecStatus", 3.0),
                ],
            )
            .await;
            assert_eq!(value(&p.db, "T:m1"), 1.0);
            assert_eq!(p.sent(), ["AB", "AB"]);
        }

        /// ss trajectoryScan state readback: each traced point's actual
        /// position in user units, and its error from the theory.
        #[epics_rs::base::epics_test]
        async fn readback_reads_each_traced_point() {
            let p = boot("mm4005Readback", &[("OFF", "10")]).await;
            p.mm.lock().unwrap().traced = 2;
            p.sent();
            put(&p.db, "T:traj1:Readback", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Readback", 0.0),
                    ("T:traj1:ReadStatus", 1.0),
                    ("T:traj1:Nactual", 2.0),
                ],
            )
            .await;
            assert_eq!(p.sent(), ["NQ", "1TQ", "2TQ"]);
            assert_eq!(array(&p.db, "T:traj1:M1Actual", 3), [11.25, 12.25, 0.0]);
            assert_eq!(array(&p.db, "T:traj1:M1Error", 3), [0.25, 0.25, 0.0]);
        }

        /// ss trajectoryScan state readback: a point whose reply stops short
        /// of an axis, or cuts its token, leaves that axis 0.
        #[epics_rs::base::epics_test]
        async fn readback_stops_a_point_at_a_short_reply() {
            let p = boot("mm4005ReadbackShort", &[("OFF", "10")]).await;
            {
                let mut m = p.mm.lock().unwrap();
                m.traced = 3;
                m.short = true;
            }
            put(&p.db, "T:traj1:Readback", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Readback", 0.0), ("T:traj1:ReadStatus", 1.0)],
            )
            .await;
            assert_eq!(array(&p.db, "T:traj1:M1Actual", 3), [0.0, 0.0, 13.25]);
            assert_eq!(array(&p.db, "T:traj1:M1Error", 3), [0.0, 0.0, 0.25]);
        }
    }
}
