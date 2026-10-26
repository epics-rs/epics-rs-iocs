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

/// C `writeRead`: the asyn status, the reply in `stringIn` as the port's
/// read leaves it there, a NUL after it when there is room. An exchange
/// that failed before its read leaves `stringIn` as it was.
fn write_read(v: &seqg::seqg_vars, command: impl CSrc) -> i32 {
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
            if let Some(end) = b.get_mut(n) {
                *end = 0;
            }
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
    (r#"static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command) { asynStatus status; size_t nwrite; char buffer[256]; strncpy(buffer, command, 256 -3); strcat(buffer, "\r"); status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer, strlen(buffer), 1.0, &nwrite); return(status); } static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command) { asynStatus status; size_t nwrite, nread; int eomReason; char buffer[256]; strncpy(buffer, command, 256 -3); strcat(buffer, "\r"); status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer, strlen(buffer), pVar->stringIn, 256, 30.0, &nwrite, &nread, &eomReason); return(status); } static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos) { char *p, *tok_save; int j; int dir; writeRead(ssId, pVar, "TP"); tok_save = 0; p = epicsStrtok_r(pVar->stringIn, ",", &tok_save); for (j=0; (j<pVar->numAxes && p!=0); j++) { if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1; pos[j] = atof(p+3)*dir + pVar->epicsMotorOff[j]; p = epicsStrtok_r(0, ",", &tok_save); } return(0); } static int getMotorMoving(SS_ID ssId, struct UserVar *pVar) { char *p, *tok_save; int j; int result=0, mask=0x01; writeRead(ssId, pVar, "MS"); tok_save = 0; p = epicsStrtok_r(pVar->stringIn, ",", &tok_save); for (j=0; (j<pVar->numAxes && p!=0); j++) { if (*(p+3) & 0x01) result |= mask; mask = mask << 1; p = epicsStrtok_r(0, ",", &tok_save); } return(result); } static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar) { int j; int result=0, mask=0x01; for (j=0; j<pVar->numAxes; j++) { seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0); if (pVar->epicsMotorDone[j] == 0) result |= mask; mask = mask << 1; } return(result); } static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar) { int j; while(getEpicsMotorMoving(ssId, pVar)) { for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } epicsThreadSleep(0.1); } for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } return(0); }"#) => {};
    ($env:ident, r#"pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0,"#, r#"(asynUser **)&pVar->pasynUser,"#, r#"NULL);"#) => {
        $env.var().status.set(connect($env.var().asynPort.get()))
    };
    ($env:ident, r#"writeOnly(ssId, pVar, pVar->stringOut);"#) => {
        write_only($env.var(), &$env.var().stringOut)
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

    /// The reply lands as the port's read leaves it: a NUL after it, the
    /// rest of the last reply behind that.
    #[test]
    fn a_reply_is_terminated_over_the_last() {
        let (v, _rig) = vars("mm4005_over", |cmd| match cmd {
            "1GC?\r" => "1GC0.25".into(),
            _ => "XC12".into(),
        });
        strcpy(&v.stringOut, "1GC?");
        write_read(&v, &v.stringOut);
        write_read(&v, "XC1");
        v.stringIn
            .with_c_dst(|b| assert_eq!(&b[..8], b"XC12\x0025\x00"));
    }

    /// A refused write leaves `stringIn` as the last reply left it, as C's
    /// writeRead returns before its read.
    #[test]
    fn a_failed_write_leaves_the_last_reply() {
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
        assert_eq!(text(&v.stringIn), "TB@ no error");
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
}
