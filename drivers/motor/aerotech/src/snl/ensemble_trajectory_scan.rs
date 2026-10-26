//! `EnsembleTrajectoryScan.st`, compiled by snc-rs from `build.rs` and run
//! on `epics-seq-rs`.
//!
//! The escaped C follows: the `%{ }%` functions the program reaches, as
//! Rust, then one `seqg_escaped!` arm per fragment, its text as cpp hands
//! it to snc. `writeOnly`, `userToRaw` and `trajEval` with its helpers are
//! reached by nothing cpp leaves (`USE_SCOPE` is 0) and have no Rust.

use std::collections::BTreeMap;
use std::ffi::c_char;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::general_time;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_seq_rs::c::*;
use epics_seq_rs::{ChId, CharPtr, CompType, Ptr, SeqVar, StateSet};

type Env = StateSet<seqg::seqg_vars>;

/// A C `char [MAX_MESSAGE_STRING]`, every command and reply buffer.
type Message = SeqVar<[c_char; MAX_MESSAGE_STRING]>;

const MAX_MESSAGE_STRING: usize = 100;
const MAX_AXES: usize = 8;
const MAX_PULSES: i32 = 8000;
const MAX_PSO_PULSES: i32 = 8192;
/// The first IGLOBAL of the PSO pulse table.
const IGLOBALINDEXSTART: i32 = 50;
/// `iFirstTrajDGLOBAL`: the first DGLOBAL of the PVT points.
const I_FIRST_TRAJ_DGLOBAL: i32 = 10;
/// The IGLOBALs doCommand.ab takes its command and arguments from.
const CMD_ACK_VAR: i32 = 41;
const PVT_WAIT_MS_VAR: i32 = 42;
const CMD_VAR: i32 = 45;
const IARG1_VAR: i32 = 46;
const IARG2_VAR: i32 = 47;
const IARG3_VAR: i32 = 48;
const IARG4_VAR: i32 = 49;
const VELOCITY_COMMAND_THRESHOLD_PARM_NUM: i32 = 35;
const REVERSE_MOTION_DIRECTION_PARM_NUM: i32 = 1;
const HOME_POSITION_SET_PARM_NUM: i32 = 166;
const PULSE_MODE_FIXED: i32 = 0;
const PULSE_MODE_ARRAY: i32 = 1;
const PULSE_MODE_TRAJPTS: i32 = 2;
const PULSE_MODE_NONE: i32 = 3;
const PULSE_POSITIONS_LOADED_ARRAY: i32 = 1;
const PULSE_POSITIONS_LOADED_TRAJ: i32 = 2;
const ABORT_STATE_NONE: i32 = 0;
const ABORT_STATE_SENT: i32 = 2;
/// `sendReceiveCommand`'s wait for doCommand.ab's ack.
const WAITLOOPS: i32 = 300;
const WAITTIME: f64 = 0.01;
/// The timeout every octet call here passes.
const TIMEOUT: Duration = Duration::from_secs(30);
const ASCII_ACK_CHAR: c_char = b'%' as c_char;
const ASCII_NAK_CHAR: c_char = b'!' as c_char;
const PROGRAM_RUN: &str = r#"PROGRAM RUN 1, "doCommand.bcx""#;

/// The user each `pasynOctetSyncIO->connect` made, by the `PORT` it named:
/// `pVar->pasynUser` is a `char *` here, which cannot hold one.
static USERS: Mutex<BTreeMap<String, Arc<SyncIOHandle>>> = Mutex::new(BTreeMap::new());

fn message() -> Message {
    SeqVar::new([0; MAX_MESSAGE_STRING])
}

fn text(s: impl CSrc) -> String {
    s.with_c_bytes(|b| String::from_utf8_lossy(b).into_owned())
}

/// `numAxes` as a loop bound; init caps it at `MAX_AXES`.
fn axes(v: &seqg::seqg_vars) -> std::ops::Range<usize> {
    0..usize::try_from(v.numAxes.get()).unwrap_or(0)
}

/// The first axis `moveAxis` selects, or `MAX_AXES`.
fn first_move_axis(v: &seqg::seqg_vars) -> usize {
    (0..MAX_AXES)
        .find(|&j| v.moveAxis[j].get() != 0)
        .unwrap_or(MAX_AXES)
}

/// C `NINT`.
fn nint(f: f64) -> i32 {
    (if f > 0.0 { f + 0.5 } else { f - 0.5 }) as i32
}

/// C `rawToUser`.
fn raw_to_user(raw: i32, off: f64, dir: i32, res: f64) -> f64 {
    f64::from(raw) * res * f64::from(dir) + off
}

/// C `timeNow`: seconds past the EPICS epoch.
fn time_now() -> f64 {
    let now = general_time::get_current();
    now.unix_secs() as f64 - general_time::EPICS_EPOCH_UNIX_SECS as f64
        + f64::from(now.subsec_nanos()) / 1e9
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
    let (user, status) = SyncIOHandle::connect_port(&name, 0, TIMEOUT, InterfaceType::Octet);
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

/// `memset(reply, 0, MAX_MESSAGE_STRING)` and the octet `writeRead` of
/// `command` into it: the asyn status.
fn exchange(v: &seqg::seqg_vars, command: &[u8], reply: &Message) -> i32 {
    let (read, status) = user(v)
        .write_read(0, command, MAX_MESSAGE_STRING)
        .into_parts();
    reply.with_c_dst(|b| {
        b.fill(0);
        let n = read.len().min(b.len());
        b[..n].copy_from_slice(&read[..n]);
    });
    status.map_or_else(|e| e.status() as i32, |()| 0)
}

/// C `writeRead`: `command` to the controller, unless `simMode`, and its
/// reply into `reply`.
fn write_read(env: &Env, command: impl CSrc, reply: &Message) -> i32 {
    let v = env.var();
    let mut buffer = command.to_c_vec();
    buffer.truncate(MAX_MESSAGE_STRING - 3);
    let mut status = 0;
    if v.simMode.get() == 0 {
        status = exchange(v, &buffer, reply);
        if status != 0 {
            printf(
                "writeRead: pasynOctetSyncIO->writeRead returned %d\n",
                &[status.into()],
            );
        }
    } else {
        reply.with_c_dst(|b| b.fill(0));
    }
    if v.simMode.get() != 0 || v.debugLevel.get() >= 3 {
        if buffer.last() == Some(&b'\n') {
            buffer.pop();
        }
        printf(
            "    writeRead:command='%s', reply='%s'\n",
            &[CArg::from(&buffer[..]), CArg::from(reply)],
        );
    }
    status
}

/// A `double *pos` `getMotorPositions` fills.
trait Positions {
    fn set_pos(&self, j: usize, pos: f64);
    fn pos(&self, j: usize) -> f64;
}

impl Positions for SeqVar<[f64; MAX_AXES]> {
    fn set_pos(&self, j: usize, pos: f64) {
        self.at(j).set(pos);
    }

    fn pos(&self, j: usize) -> f64 {
        self.at(j).get()
    }
}

impl Positions for [SeqVar<f64>; MAX_AXES] {
    fn set_pos(&self, j: usize, pos: f64) {
        self[j].set(pos);
    }

    fn pos(&self, j: usize) -> f64 {
        self[j].get()
    }
}

/// C `getMotorPositions`: each axis's `PFBKPROG` in user units. It asks
/// the controller even in `simMode`, as C does.
fn get_motor_positions(env: &Env, pos: &impl Positions) {
    let v = env.var();
    let input = message();
    for j in axes(v) {
        let status = exchange(v, format!("PFBKPROG(@{j})").as_bytes(), &input);
        if status != 0 {
            printf(
                "getMotorPositions: pasynOctetSyncIO->writeRead returned %d\n",
                &[status.into()],
            );
        }
        let raw_f = if char_at(&input, 0) == ASCII_ACK_CHAR {
            atof(tail(&input, 1))
        } else {
            0.0
        };
        let mres = v.epicsMotorMres[j].get();
        let raw_p = nint(raw_f / fabs(mres));
        let dir = if v.epicsMotorDir[j].get() == 0 { 1 } else { -1 };
        pos.set_pos(j, raw_to_user(raw_p, v.epicsMotorOff[j].get(), dir, mres));
    }
    if v.debugLevel.get() > 2 {
        printf("pos[0]=%.2f\n", &[pos.pos(0).into()]);
    }
}

/// C `getMotorMoving`: 1 when `PLANESTATUS(0)` says the plane moves. C's
/// `MAX_DONE_TRY` is 1, so it asks once, and never reads its `movingMask`.
fn get_motor_moving(env: &Env) -> i32 {
    let v = env.var();
    let input = message();
    let status = exchange(v, b"PLANESTATUS(0)", &input);
    if status != 0 {
        printf(
            "getMotorMoving: pasynOctetSyncIO->writeRead returned %d\n",
            &[status.into()],
        );
    }
    if strlen(&input) > 3 {
        printf(
            "getMotorMoving: PLANESTATUS(0) returned '%s'\n",
            &[(&input).into()],
        );
    }
    if v.debugLevel.get() > 2 {
        printf("getMotorMoving: inputBuff='%s'\n", &[(&input).into()]);
    }
    i32::from(atoi(tail(&input, 1)) != 0)
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
        epicsThreadSleep(1.0 / 5.0);
    }
    post();
}

/// C `buildTrajectory` for axis `j`: its PVT velocities into `velocity[j]`,
/// checked against the controller's velocity threshold. C's other
/// arguments are `pVar` fields, read here, or unused.
fn build_trajectory(env: &Env, j: usize) -> i32 {
    let v = env.var();
    let reply = message();
    write_read(
        env,
        &*format!("GETPARM({VELOCITY_COMMAND_THRESHOLD_PARM_NUM})"),
        &reply,
    );
    if char_at(&reply, 0) != ASCII_ACK_CHAR {
        printf(
            "buildTrajectory: Can't read velocityCommandThreshold parameter\n",
            &[],
        );
        return -1;
    }
    let max_v = atof(tail(&reply, 1));
    write_int_and_check(env, IARG1_VAR, 0);
    send_receive_command(env, Command::Version, true);
    write_read(env, &*format!("IGLOBAL({IARG1_VAR})"), &reply);
    let version_major = atol(tail(&reply, 1)) as i32;
    write_read(env, &*format!("IGLOBAL({IARG2_VAR})"), &reply);
    let version_minor = atol(tail(&reply, 1)) as i32;
    if version_major < 1 || version_minor < 2 {
        printf(
            "buildTrajectory: I need version 1.2 or higher of doCommand.ab (currV=%d.%d)\n",
            &[version_major.into(), version_minor.into()],
        );
        return -1;
    }

    let npoints = usize::try_from(v.npoints.get()).unwrap_or(0);
    let time = v.realTimeTrajectory.get();
    let traj = v.motorTrajectory[j].get();
    let mut velocity = v.velocity.at(j).get();
    let debug = v.debugLevel.get();
    for i in 0..npoints {
        velocity[i] = if i == 0 {
            (traj[1] - traj[0]) / (time[1] - time[0])
        } else {
            2.0 * (traj[i] - traj[i - 1]) / (time[i] - time[i - 1]) - velocity[i - 1]
        };
        if debug > 2 {
            printf(
                "point %d: pos=%f, vel=%f, time=%f\n",
                &[i.into(), traj[i].into(), velocity[i].into(), time[i].into()],
            );
        }
    }
    v.velocity.at(j).set(velocity);
    for (i, vel) in velocity[..npoints].iter().enumerate() {
        if !vel.is_finite() || fabs(*vel) > max_v {
            printf(
                "velocity > limit %f at point %d: vel=%f\n",
                &[max_v.into(), i.into(), (*vel).into()],
            );
            return -1;
        }
    }
    0
}

/// The doCommand.ab commands this program runs through
/// `sendReceiveCommand`; C's parser also takes twenty it never sends.
#[derive(Clone, Copy)]
enum Command {
    VelocityOff,
    StartAbort,
    DoStoredTraj,
    Version,
}

impl Command {
    /// The text C's parser matched.
    fn text(self) -> &'static str {
        match self {
            Command::VelocityOff => "VELOCITY OFF",
            Command::StartAbort => "STARTABORT",
            Command::DoStoredTraj => "DOSTOREDTRAJ",
            Command::Version => "VERSION",
        }
    }

    /// Its `cmd*` number, put in `IGLOBAL(cmdVar)`.
    fn number(self) -> i32 {
        match self {
            Command::VelocityOff => 2,
            Command::StartAbort => 12,
            Command::DoStoredTraj => 26,
            Command::Version => 27,
        }
    }
}

/// C `sendReceiveCommand`: `cmd` through doCommand.ab, which
/// `check_program` (re)starts first, then its ack awaited. C's
/// `callerReply` is never NULL and never written.
fn send_receive_command(env: &Env, cmd: Command, check_program: bool) -> i32 {
    let v = env.var();
    let reply = message();
    if v.debugLevel.get() > 1 {
        printf("sendReceiveCommand: command='%s'\n", &[cmd.text().into()]);
    }
    if check_program {
        write_read(env, "TASKSTATE(1)", &reply);
        let state = atoi(tail(&reply, 1));
        if char_at(&reply, 0) == ASCII_NAK_CHAR {
            printf("sendReceiveCommand: 'TASKSTATE(1)' returned error.\n", &[]);
            return -1;
        }
        match state {
            0 if strlen(&reply) == 1 => {
                printf(
                    "sendReceiveCommand: TASKSTATE(1)' returned '%s', which I'm ignoring\n",
                    &[(&reply).into()],
                );
            }
            0 => {
                printf(
                    "sendReceiveCommand: TASKSTATE(1)' returned '%s'\n",
                    &[(&reply).into()],
                );
                return -1;
            }
            1 | 2 | 4 | 5 => {
                write_read(env, PROGRAM_RUN, &reply);
                epicsThreadSleep(0.1);
            }
            6 => {
                write_read(env, "PROGRAM STOP 1", &reply);
                epicsThreadSleep(0.1);
                write_read(env, PROGRAM_RUN, &reply);
                epicsThreadSleep(0.1);
            }
            _ => {}
        }
    }

    let save_debug = v.debugLevel.get();
    v.debugLevel.set((v.debugLevel.get() - 1).max(0));
    write_read_int_and_check(env, CMD_VAR, cmd.number(), &reply);
    let mut i = 0;
    // C waits for no ack of DOTRAJECTORY or DOSTOREDTRAJ.
    if !matches!(cmd, Command::DoStoredTraj) {
        let ack = format!("IGLOBAL({CMD_ACK_VAR})");
        while i < WAITLOOPS {
            write_read(env, &*ack, &reply);
            if char_at(&reply, 0) == ASCII_ACK_CHAR {
                if v.debugLevel.get() >= 10 {
                    printf(
                        "atoi(&reply[1])=%d, cmdNum=%d\n",
                        &[atoi(tail(&reply, 1)).into(), cmd.number().into()],
                    );
                }
                if atoi(tail(&reply, 1)) == cmd.number() {
                    break;
                }
            }
            epicsThreadSleep(WAITTIME);
            i += 1;
        }
    }
    v.debugLevel.set(save_debug);
    if i > WAITLOOPS - 1 {
        printf(
            "sendReceiveCommand: timeout (%.2f s) executing '%s'; reply='%s'\n",
            &[
                (f64::from(WAITLOOPS) * WAITTIME).into(),
                cmd.text().into(),
                (&reply).into(),
            ],
        );
        printf("... reply='%s'\n", &[(&reply).into()]);
        write_int_and_check(env, CMD_VAR, 0);
        return -1;
    }
    0
}

/// C `writeDoubleAndCheck`: `DGLOBAL(n) = d`, read back until it holds,
/// at most ten times.
fn write_double_and_check(env: &Env, n: i32, d: f64) -> i32 {
    let reply = message();
    for _ in 0..10 {
        if write_read(env, &*format!("DGLOBAL({n}) = {:.6}", CFloat(d)), &reply) != 0 {
            return -1;
        }
        if write_read(env, &*format!("DGLOBAL({n})"), &reply) != 0 {
            return -1;
        }
        let check = atof(tail(&reply, 1));
        if fabs(d - check) < 0.0001 {
            return 0;
        }
        printf(
            "writeDoubleAndCheck: fix dglobal %d (%f != %f)\n",
            &[n.into(), d.into(), check.into()],
        );
    }
    -1
}

/// C `writeIntAndCheck`: `IGLOBAL(n) = ival`, read back until it holds, at
/// most ten times.
fn write_int_and_check(env: &Env, n: i32, ival: i32) -> i32 {
    let reply = message();
    for _ in 0..10 {
        if write_read(env, &*format!("IGLOBAL({n}) = {ival}"), &reply) != 0 {
            return -1;
        }
        if write_read(env, &*format!("IGLOBAL({n})"), &reply) != 0 {
            return -1;
        }
        let check = atol(tail(&reply, 1)) as i32;
        if check == ival {
            return 0;
        }
        printf(
            "writeIntAndCheck: fix iglobal %d (%d != %d)\n",
            &[n.into(), ival.into(), check.into()],
        );
    }
    -1
}

/// C `writeReadIntAndCheck`: [`write_int_and_check`] after clearing
/// `cmdAckVar`, the write's reply into `caller_reply`.
fn write_read_int_and_check(env: &Env, n: i32, ival: i32, caller_reply: &Message) -> i32 {
    let reply = message();
    write_int_and_check(env, CMD_ACK_VAR, 0);
    for _ in 0..10 {
        if write_read(env, &*format!("IGLOBAL({n}) = {ival}"), caller_reply) != 0 {
            return -1;
        }
        if write_read(env, &*format!("IGLOBAL({n})"), &reply) != 0 {
            return -1;
        }
        let check = atol(tail(&reply, 1)) as i32;
        if check == ival {
            return 0;
        }
        // doCommand.ab acks before it zeroes cmdVar: a 0 with the ack set
        // ran, and writing it again runs it again.
        if check == 0 {
            if write_read(env, &*format!("IGLOBAL({CMD_ACK_VAR})"), &reply) != 0 {
                return -1;
            }
            if atol(tail(&reply, 1)) as i32 == ival {
                return 0;
            }
        }
        printf(
            "writeReadIntAndCheck: fix iglobal %d (%d != %d)\n",
            &[n.into(), ival.into(), check.into()],
        );
    }
    -1
}

/// The PSO pulse table from `position`s along axis 0, as distances in
/// steps from the last pulse, into the IGLOBALs from `IGLOBALINDEXSTART`.
/// The count loaded.
fn load_pulses(env: &Env, positions: impl Iterator<Item = f64>, max_pulses: i32) -> i32 {
    let v = env.var();
    let mut position_last = v.motorStart[0].get();
    let mut np = 0;
    let positions = positions.take(usize::try_from(max_pulses).unwrap_or(0));
    for (i_global_index, position) in (IGLOBALINDEXSTART..).zip(positions) {
        np += 1;
        let mres = v.epicsMotorMres[0].get();
        let int_position = nint((position - position_last) / mres);
        write_int_and_check(env, i_global_index, int_position.saturating_abs());
        position_last += f64::from(int_position) * mres;
    }
    np
}

/// C `loadTrajectory`: the pulse table and the PVT points of the first
/// axis `moveAxis` selects into the controller's globals, doCommand.ab
/// restarted; `*pstatus` is -1 on failure.
fn load_trajectory(env: &Env, pstatus: Ptr<i32>) {
    let v = env.var();
    let reply = message();
    v.loadingTrajectory.set(1);
    pstatus.set(0);
    write_read(
        env,
        &*format!("getparm(@0,{REVERSE_MOTION_DIRECTION_PARM_NUM})"),
        &reply,
    );
    v.encoder_runs_backwards.set(atoi(tail(&reply, 1)));
    if v.debugLevel.get() > 2 {
        printf(
            "loadTrajectory: encoder_runs_backwards=%d\n",
            &[v.encoder_runs_backwards.get().into()],
        );
    }
    write_read(
        env,
        &*format!("getparm(@0,{HOME_POSITION_SET_PARM_NUM})"),
        &reply,
    );
    let home_position_set = atof(tail(&reply, 1));
    if v.debugLevel.get() > 2 {
        printf(
            "loadTrajectory: home_position_set=%f\n",
            &[home_position_set.into()],
        );
    }
    write_int_and_check(env, PVT_WAIT_MS_VAR, 50);
    for j in 0..MAX_AXES {
        write_read(env, &*format!("pfbkprog(@{j})"), &reply);
        let posfbkprog = atof(tail(&reply, 1));
        write_read(env, &*format!("pfbkcal(@{j})"), &reply);
        let posfbkcal = atof(tail(&reply, 1));
        v.motorPosOffset.at(j).set(posfbkprog - posfbkcal);
    }

    let pulse_mode = v.pulseMode.get();
    let globals = v.numGlobalIntegers.get();
    let max_array_pulses = MAX_PSO_PULSES.min(globals.saturating_sub(IGLOBALINDEXSTART));
    if pulse_mode == PULSE_MODE_ARRAY
        && v.pulsePositionsLoaded.get() != PULSE_POSITIONS_LOADED_ARRAY
    {
        if v.debugLevel.get() > 2 {
            printf("loadTrajectory: load PULSE_MODE_ARRAY array\n", &[]);
        }
        if v.debugLevel.get() != 0 {
            printf(
                "loadTrajectory: motorStart=%f, pVar->pulsePositions[0]=%f\n",
                &[
                    v.motorStart[0].get().into(),
                    v.pulsePositions.at(0).get().into(),
                ],
            );
        }
        let mut n_pulses = v.numPulsePositions.get();
        if i64::from(globals) < i64::from(n_pulses) + i64::from(IGLOBALINDEXSTART) {
            printf(
                "EnsembleTrajectoryScan: The Ensemble only has enough global integers for %d pulses\n",
                &[globals.saturating_sub(IGLOBALINDEXSTART).into()],
            );
            n_pulses = globals.saturating_sub(IGLOBALINDEXSTART);
        }
        if n_pulses > MAX_PSO_PULSES {
            printf(
                "EnsembleTrajectoryScan: The Ensemble can't load more than %d PSOARRAY pulses\n",
                &[MAX_PSO_PULSES.into()],
            );
            n_pulses = MAX_PSO_PULSES;
        }
        n_pulses = n_pulses.min(MAX_PULSES);
        v.numPulsePositions.set(n_pulses);
        let count = usize::try_from(n_pulses).unwrap_or(0);
        let positions = (0..count).map(|i| v.pulsePositions.at(i).get());
        let np = load_pulses(env, positions, max_array_pulses);
        v.pulsePositionsLoaded.set(PULSE_POSITIONS_LOADED_ARRAY);
        v.numPulsePositionsLoaded.set(np);
    } else if pulse_mode == PULSE_MODE_TRAJPTS
        && v.pulsePositionsLoaded.get() != PULSE_POSITIONS_LOADED_TRAJ
    {
        if v.debugLevel.get() > 2 {
            printf("loadTrajectory: load PULSE_MODE_TRAJPTS array\n", &[]);
        }
        let npoints = v.npoints.get();
        if i64::from(globals) < i64::from(npoints) + i64::from(IGLOBALINDEXSTART) {
            printf(
                "EnsembleTrajectoryScan: The Ensemble only has enough global integers for %d pulses\n",
                &[globals.saturating_sub(IGLOBALINDEXSTART).into()],
            );
        }
        if npoints > MAX_PSO_PULSES {
            printf(
                "EnsembleTrajectoryScan: The Ensemble can't load more than %d PSOARRAY pulses\n",
                &[MAX_PSO_PULSES.into()],
            );
        }
        let count = usize::try_from(npoints).unwrap_or(0);
        let positions = (0..count)
            .filter(|&i| {
                let i = i as i32;
                i >= v.startPulses.get() && i <= v.endPulses.get()
            })
            .map(|i| v.motorTrajectory[0].at(i).get());
        let np = load_pulses(env, positions, max_array_pulses);
        v.pulsePositionsLoaded.set(PULSE_POSITIONS_LOADED_TRAJ);
        v.numPulsePositionsLoaded.set(np);
    }

    get_motor_positions(env, &v.motorCurrent);
    write_read(env, "TASKSTATE(1)", &reply);
    let state = atoi(tail(&reply, 1));
    if char_at(&reply, 0) == ASCII_NAK_CHAR || !matches!(state, 1 | 3 | 5) {
        write_read(env, "PROGRAM STOP 1", &reply);
        write_read(env, "TASKSTATE(1)", &reply);
    }
    write_int_and_check(env, CMD_VAR, 0);
    write_read(env, PROGRAM_RUN, &reply);
    epicsThreadSleep(0.1);

    let j = first_move_axis(v);
    let npoints = usize::try_from(v.npoints.get()).unwrap_or(0);
    if j == MAX_AXES || npoints < 2 || load_points(env, j, npoints) != 0 {
        load_failed(env, &reply);
        pstatus.set(-1);
    }
}

/// `loadTrajectory`'s PVT points for axis `j`, from `iFirstTrajDGLOBAL`:
/// an acceleration point, the trajectory, a point past its end, and two
/// deceleration points. -1 when a write fails.
fn load_points(env: &Env, j: usize, npoints: usize) -> i32 {
    let v = env.var();
    let traj = v.motorTrajectory[j].get();
    let velocity = v.velocity.at(j).get();
    let time = v.realTimeTrajectory.get();
    let accel_time = v.epicsMotorACCL[j].get();
    let decel_time = accel_time;
    let decel_dist = (velocity[npoints - 1] * accel_time) / 2.0;
    let last = npoints - 1;

    let mut points = vec![(traj[0], velocity[0], accel_time)];
    points.extend((1..npoints).map(|i| (traj[i], velocity[i], accel_time + time[i])));
    let mut p = 2.0 * traj[last] - traj[last - 1];
    let mut t = accel_time + 2.0 * time[last] - time[last - 1];
    points.push((p, velocity[last], t));
    p += decel_dist * 0.9;
    t += decel_time * 0.9;
    points.push((p, velocity[last] * 0.1, t));
    p += decel_dist * 0.1;
    t += decel_time * 0.1;
    points.push((p, 0.0, t));

    let mut n = I_FIRST_TRAJ_DGLOBAL;
    for (p, vel, t) in points {
        let mut status = 0;
        for d in [p, vel, t] {
            status += write_double_and_check(env, n, d);
            n += 1;
        }
        if status != 0 {
            return -1;
        }
    }
    v.actualNumTrajectoryPoints
        .set((n - I_FIRST_TRAJ_DGLOBAL) / 3);
    0
}

/// `loadTrajectory`'s `bad:` exit, short of `*pstatus`.
fn load_failed(env: &Env, reply: &Message) {
    printf("loadTrajectory: error\n", &[]);
    for command in ["TASKSTATE(1)", "TASKERROR(1)", "PLANESTATUS(0)"] {
        write_read(env, command, reply);
        printf("%s returns '%s'\n", &[command.into(), reply.into()]);
    }
    write_read(env, "AXISSTATUS(@0)", reply);
    printf(
        "AXISSTATUS(@0) returns '%s' (%x)\n",
        &[reply.into(), (atoi(tail(reply, 1)) as u32).into()],
    );
    env.var().loadingTrajectory.set(0);
    printf("loadTrajectory: aborting because of error\n", &[]);
}

/// C `executeTrajectory`: the PSO output armed, the stored trajectory
/// started and awaited; -2 when it did not start, -1 when aborted.
fn execute_trajectory(env: &Env) -> i32 {
    let v = env.var();
    let reply = message();
    let pulse_mode = v.pulseMode.get();
    let array = pulse_mode == PULSE_MODE_ARRAY || pulse_mode == PULSE_MODE_TRAJPTS;
    if pulse_mode != PULSE_MODE_NONE {
        write_read(env, "PSOCONTROL @0 RESET", &reply);
        if array {
            let loaded = v.numPulsePositionsLoaded.get();
            write_read(
                env,
                &*format!("PSOARRAY @0,{IGLOBALINDEXSTART},{loaded}"),
                &reply,
            );
            if v.debugLevel.get() > 2 {
                printf(
                    "executeTrajectory: copied PSOARRAY (%d elements) to drive\n",
                    &[loaded.into()],
                );
            }
            write_read(env, "PSODISTANCE @0 ARRAY", &reply);
        }
        let len = v.pulseLenUS.get();
        write_read(
            env,
            &*format!(
                "PSOPULSE @0 TIME {:.6},{:.6}",
                CFloat(len * 1.5),
                CFloat(len)
            ),
            &reply,
        );
        write_read(env, "PSOOUTPUT @0 PULSE", &reply);
        write_read(
            env,
            &*format!("PSOTRACK @0 INPUT {}", v.pulseSrc.get()),
            &reply,
        );
        write_read(
            env,
            &*format!("PSOTRACK @0 DIRECTION {}", v.pulseDir.get()),
            &reply,
        );
        if array {
            write_read(env, "PSODISTANCE @0 ARRAY", &reply);
        } else if pulse_mode == PULSE_MODE_FIXED {
            // A built trajectory has two points or more.
            let last = usize::try_from(v.npoints.get() - 1).unwrap_or(0);
            let traj = &v.motorTrajectory[0];
            let dp = fabs(traj.at(last).get() - traj.at(0).get()) / f64::from(v.npulses.get());
            write_read(
                env,
                &*format!("PSODISTANCE @0 FIXED {:.6} UNITS", CFloat(dp)),
                &reply,
            );
        }
        write_read(env, "PSOCONTROL @0 ARM", &reply);
        if v.debugLevel.get() > 2 {
            printf("executeTrajectory: sent PSO commands\n", &[]);
        }
    }

    v.startTime.set(time_now());
    write_int_and_check(env, IARG1_VAR, first_move_axis(v) as i32);
    write_int_and_check(env, IARG2_VAR, v.actualNumTrajectoryPoints.get());
    write_int_and_check(env, IARG3_VAR, I_FIRST_TRAJ_DGLOBAL);
    write_int_and_check(env, IARG4_VAR, -1);
    send_receive_command(env, Command::DoStoredTraj, false);
    if v.abortState.get() != ABORT_STATE_NONE {
        printf("executeTrajectory: aborted\n", &[]);
        send_receive_command(env, Command::VelocityOff, true);
        send_receive_command(env, Command::StartAbort, true);
        write_read(env, &v.abortCommand, &v.stringIn);
        v.abortState.set(ABORT_STATE_SENT);
        v.loadingTrajectory.set(0);
        if pulse_mode != PULSE_MODE_NONE {
            write_read(env, "PSOCONTROL @0 OFF", &reply);
        }
        return -1;
    }
    write_read(env, "TASKSTATE(1)", &reply);
    if v.debugLevel.get() != 0 {
        printf("TASKSTATE = '%s', ", &[(&reply).into()]);
    }
    write_read(env, "TASKERROR(1)", &reply);
    if v.debugLevel.get() != 0 {
        printf("TASKERROR = '%s', ", &[(&reply).into()]);
    }
    let point = format!("IGLOBAL({IARG4_VAR})");
    for _ in 0..50 {
        write_read(env, &*point, &reply);
        let curr_point = atol(tail(&reply, 1)) as i32;
        if v.debugLevel.get() != 0 {
            printf("traj point = %d\n", &[curr_point.into()]);
        }
        if curr_point > 20 {
            break;
        }
        epicsThreadSleep(0.1);
    }
    let mut i = 0;
    while i < 50 {
        write_read(env, "PLANESTATUS(0)", &reply);
        if char_at(&reply, 1) == b'3' as c_char {
            break;
        }
        epicsThreadSleep(0.1);
        i += 1;
    }
    if i >= 50 {
        printf(
            "executeTrajectory: trajectory didn't start. (PLANESTATUS = '%s')\n",
            &[(&reply).into()],
        );
        send_receive_command(env, Command::VelocityOff, true);
        write_read(env, &v.abortCommand, &v.stringIn);
        return -2;
    }
    if v.debugLevel.get() != 0 {
        printf("PLANESTATUS = '%s'\n", &[(&reply).into()]);
    }
    v.loadingTrajectory.set(0);
    if v.debugLevel.get() != 0 {
        printf(
            "executeTrajectory: normal exit after %.1f s wait\n",
            &[(f64::from(i) / 10.0).into()],
        );
    }
    0
}

macro_rules! seqg_escaped {
    (r#"#include <stdlib.h>"#) => {};
    (r#"#include <string.h>"#) => {};
    (r#"#include <ctype.h>"#) => {};
    (r#"#include <stdio.h>"#) => {};
    (r#"#include <math.h>"#) => {};
    (r#"#include <epicsString.h>"#) => {};
    (r#"#include <epicsStdio.h>"#) => {};
    (r#"#include <asynOctetSyncIO.h>"#) => {};
    (r#"epicsTimeStamp pvtTime;"#) => {};
    (r#"epicsTimeStamp lastPvtTime;"#) => {};
    (r#"static double timeNow(void);"#) => {};
    (r#"static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command);"#) => {};
    (r#"static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply);"#) => {};
    (r#"static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);"#) => {};
    (r#"static int getMotorMoving(SS_ID ssId, struct UserVar *pVar, int movingMask);"#) => {};
    (r#"static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (
        r#"static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *timeTrajectory,"#,
        r#"double *motorTrajectory, int epicsMotorDir, int moveMode, int npoints, int npulses, double motorOffset,"#,
        r#"double motorResolution, double *velocity);"#
    ) => {};
    (r#"static int loadTrajectory(SS_ID ssId, struct UserVar *pVar, int *pstatus);"#) => {};
    (r#"static int executeTrajectory(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int userToRaw(double user, double off, int dir, double res);"#) => {};
    (r#"static double rawToUser(int raw, double off, int dir, double res);"#) => {};
    (r#"static int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply, int checkProgram);"#) => {};
    (
        r#"static double trajEval(SS_ID ssId, struct UserVar *pVar, double *motorReadbacks, int scopeDataIntervalMS, int nScopeDataPoints,"#,
        r#"double *motorTrajectory, double *realTimeTrajectory, int npoints, double *motorError);"#
    ) => {};
    (r#"int writeDoubleAndCheck(SS_ID ssId, struct UserVar *pVar, int n, double d);"#) => {};
    (r#"int writeIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival);"#) => {};
    (r#"int writeReadIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival, char *callerReply);"#) => {};
    (r#"static double timeNow(void) { epicsTimeStamp now; epicsTimeGetCurrent(&now); return now.secPastEpoch + now.nsec/1e9; } static int writeOnly(SS_ID ssId, struct UserVar *pVar, char *command) { asynStatus status=0; int debug_out=0; size_t nwrite; char buffer[100]; strncpy(buffer, command, 100 -3); strcat(buffer, "\n"); if (!(pVar->simMode)) { status = pasynOctetSyncIO->write((asynUser *)pVar->pasynUser, buffer, strlen(buffer), 1.0, &nwrite); } if (pVar->execState==2) debug_out = (pVar->debugLevel >= 7); else debug_out = (pVar->debugLevel >= 2); if (pVar->simMode || debug_out) printf("    writeOnly:command='%s'\n", command); return(status); } static int writeRead(SS_ID ssId, struct UserVar *pVar, char *command, char *reply) { asynStatus status=0; char buffer[100]; size_t nwrite, nread; int eomReason; strncpy(buffer, command, 100 -3); memset(reply, 0, 100); if (!(pVar->simMode)) { status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, buffer, strlen(buffer), reply, 100, 30.0, &nwrite, &nread, &eomReason); if (status) printf("writeRead: pasynOctetSyncIO->writeRead returned %d\n", status); } if (pVar->simMode || (pVar->debugLevel >= 3)) { if ((strlen(buffer) > 0) && (buffer[strlen(buffer)-1] == '\n')) buffer[strlen(buffer)-1] = '\0'; printf("    writeRead:command='%s', reply='%s'\n", buffer, reply); } return(status); } static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos) { int j, status; int dir, rawP; double rawF; char inputBuff[100], outputBuff[100]; size_t nwrite, nread; int eomReason; for (j=0; j<pVar->numAxes; j++) { sprintf(outputBuff, "PFBKPROG(@%d)", j); memset(inputBuff, 0, 100); status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, outputBuff, strlen(outputBuff), inputBuff, 100, 30.0, &nwrite, &nread, &eomReason); if (status) printf("getMotorPositions: pasynOctetSyncIO->writeRead returned %d\n", status); if (inputBuff[0] != '%') rawF = 0; else rawF = atof(&inputBuff[1]); rawP = (int)((rawF / fabs(pVar->epicsMotorMres[j]))>0 ? (rawF / fabs(pVar->epicsMotorMres[j]))+0.5 : (rawF / fabs(pVar->epicsMotorMres[j]))-0.5); if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1; pos[j] = rawToUser(rawP, pVar->epicsMotorOff[j], dir, pVar->epicsMotorMres[j]); } if (pVar->debugLevel > 2) { printf("pos[0]=%.2f\n", pos[0]); } return(0); } static int getMotorMoving(SS_ID ssId, struct UserVar *pVar, int movingMask) { char inputBuff[100], outputBuff[100]; size_t nwrite, nread; int eomReason; int move_active; int i, count, status; sprintf(outputBuff, "PLANESTATUS(0)"); for (i=0, count=0; i<1; i++) { memset(inputBuff, 0, 100); status = pasynOctetSyncIO->writeRead((asynUser *)pVar->pasynUser, outputBuff, strlen(outputBuff), inputBuff, 100, 30.0, &nwrite, &nread, &eomReason); if (status) printf("getMotorMoving: pasynOctetSyncIO->writeRead returned %d\n", status); if (strlen(inputBuff) > 3) printf("getMotorMoving: PLANESTATUS(0) returned '%s'\n", inputBuff); if (pVar->debugLevel > 2) { printf("getMotorMoving: inputBuff='%s'\n", inputBuff); } move_active = atoi(&inputBuff[1]); if (move_active) { return(1); } else { count++; if (count>2) break; } } return(0); } static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar) { int j; int result=0, mask=0x01; for (j=0; j<pVar->numAxes; j++) { seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0); if (pVar->epicsMotorDone[j] == 0) result |= mask; mask = mask << 1; } return(result); } static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar) { int j; while (getEpicsMotorMoving(ssId, pVar)) { for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } epicsThreadSleep((1/5.)); } for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } return(0); } static int buildTrajectory(SS_ID ssId, struct UserVar *pVar, double *realTimeTrajectory, double *motorTrajectory, int epicsMotorDir, int moveMode, int npoints, int npulses, double motorOffset, double motorResolution, double *velocity) { double dp, dt, maxV=0; int i, status; char stringOut[100], reply[100]; int version_major, version_minor; sprintf(stringOut, "GETPARM(%d)", 35); status = writeRead(ssId, pVar, stringOut, reply); if (reply[0] == '%') { maxV = atof(&reply[1]); } else { printf("buildTrajectory: Can't read velocityCommandThreshold parameter\n"); return(-1); } status = writeIntAndCheck(ssId, pVar, 46, 0); strcpy(stringOut, "VERSION"); status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1); sprintf(stringOut, "IGLOBAL(%d)", 46); status = writeRead(ssId, pVar, stringOut, reply); version_major = atol(&reply[1]); sprintf(stringOut, "IGLOBAL(%d)", 47); status = writeRead(ssId, pVar, stringOut, reply); version_minor = atol(&reply[1]); if (version_major < 1 || version_minor < 2) { printf("buildTrajectory: I need version 1.2 or higher of doCommand.ab (currV=%d.%d)\n", version_major, version_minor); return(-1); } for (i=0; i<npoints; i++) { if (i==0) { dp = motorTrajectory[1] - motorTrajectory[0]; dt = realTimeTrajectory[1] - realTimeTrajectory[0]; velocity[i] = dp/dt; } else { dp = motorTrajectory[i] - motorTrajectory[i-1]; dt = realTimeTrajectory[i] - realTimeTrajectory[i-1]; velocity[i] = 2*dp/dt - velocity[i-1]; } if (pVar->debugLevel > 2) { printf("point %d: pos=%f, vel=%f, time=%f\n", i, motorTrajectory[i], velocity[i], realTimeTrajectory[i]); } } for (i=0; i<npoints; i++) { if (!isfinite(velocity[i]) || (fabs(velocity[i]) > maxV)) { printf("velocity > limit %f at point %d: vel=%f\n", maxV, i, velocity[i]); return(-1); } } return(0); } static int userToRaw(double user, double off, int dir, double res) { return ((int)(((user-off)*dir/res)>0 ? ((user-off)*dir/res)+0.5 : ((user-off)*dir/res)-0.5)); } static double rawToUser(int raw, double off, int dir, double res) { return (raw*res*dir+off); } int sendReceiveCommand(SS_ID ssId, struct UserVar *pVar, char *cmd, char *callerReply, int checkProgram) { int i, i1, i2, i3, i4, cmdNum; double d1, d2, d3; char stringOut[100], reply[100]; int status, saveDebug; if (pVar->debugLevel > 1) { printf("sendReceiveCommand: command='%s'\n", cmd); } if (checkProgram) { strcpy(stringOut, "TASKSTATE(1)"); status = writeRead(ssId, pVar, stringOut, reply); status = atoi(&reply[1]); if (reply[0] == '!') { printf("sendReceiveCommand: 'TASKSTATE(1)' returned error.\n"); return(-1); } switch (status) { case 0: if (strlen(reply) == 1) { printf("sendReceiveCommand: TASKSTATE(1)' returned '%s', which I'm ignoring\n", reply); } else { printf("sendReceiveCommand: TASKSTATE(1)' returned '%s'\n", reply); return(-1); } break; case 3: break; case 1: case 2: case 4: case 5: strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\""); status = writeRead(ssId, pVar, stringOut, reply); epicsThreadSleep(.1); break; case 6: strcpy(stringOut, "PROGRAM STOP 1"); status = writeRead(ssId, pVar, stringOut, reply); epicsThreadSleep(.1); strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\""); status = writeRead(ssId, pVar, stringOut, reply); epicsThreadSleep(.1); break; default: break; } } saveDebug = pVar->debugLevel; pVar->debugLevel = ((0) > (pVar->debugLevel - 1) ? (0) : (pVar->debugLevel - 1)); cmdNum = 0; if (strncmp(cmd, "VELOCITY ON", strlen("VELOCITY ON")) == 0) { cmdNum = 1; status = writeReadIntAndCheck(ssId, pVar, 45, 1, reply); } else if (strncmp(cmd, "VERSION", strlen("VERSION")) == 0) { cmdNum = 27; status = writeReadIntAndCheck(ssId, pVar, 45, 27, reply); } else if (strncmp(cmd, "DOTRAJECTORY", strlen("DOTRAJECTORY")) == 0) { cmdNum = 25; sprintf(stringOut, "IGLOBAL(%d) = %d", 45, 25); status = writeRead(ssId, pVar, stringOut, reply); } else if (strncmp(cmd, "DOSTOREDTRAJ", strlen("DOSTOREDTRAJ")) == 0) { cmdNum = 26; status = writeReadIntAndCheck(ssId, pVar, 45, 26, reply); } else if (strncmp(cmd, "VELOCITY OFF", strlen("VELOCITY OFF")) == 0) { cmdNum = 2; status = writeReadIntAndCheck(ssId, pVar, 45, 2, reply); } else if (strncmp(cmd, "HALT", strlen("HALT")) == 0) { cmdNum = 3; status = writeReadIntAndCheck(ssId, pVar, 45, 3, reply); } else if (strncmp(cmd, "STARTABORT", strlen("STARTABORT")) == 0) { cmdNum = 12; status = writeReadIntAndCheck(ssId, pVar, 45, 12, reply); } else if (strncmp(cmd, "START", strlen("START")) == 0) { cmdNum = 4; status = writeReadIntAndCheck(ssId, pVar, 45, 4, reply); } else if (strncmp(cmd, "PVT INIT TIME ABS", strlen("PVT INIT TIME ABS")) == 0) { cmdNum = 5; status = writeReadIntAndCheck(ssId, pVar, 45, 5, reply); } else if (strncmp(cmd, "PVT INIT TIME INC", strlen("PVT INIT TIME INC")) == 0) { cmdNum = 6; status = writeReadIntAndCheck(ssId, pVar, 45, 6, reply); } else if (strncmp(cmd, "PVT1", strlen("PVT1")) == 0) { cmdNum = 7; sscanf(cmd, "PVT1 %d %lf,%lf TIME %lf", &i1, &d1, &d2, &d3); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeDoubleAndCheck(ssId, pVar, 1, d1); status = writeDoubleAndCheck(ssId, pVar, 2, d2); status = writeDoubleAndCheck(ssId, pVar, 3, d3); status = writeReadIntAndCheck(ssId, pVar, 45, 7, reply); } else if (strncmp(cmd, "ABORT", strlen("ABORT")) == 0) { cmdNum = 11; i1 = i2 = i3 = i4 = -1; i = sscanf(cmd, "ABORT @%d @%d @%d @%d", &i1, &i2, &i3, &i4); status = writeIntAndCheck(ssId, pVar, 44, i); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeIntAndCheck(ssId, pVar, 48, i3); status = writeIntAndCheck(ssId, pVar, 49, i4); status = writeReadIntAndCheck(ssId, pVar, 45, 11, reply); } else if (strncmp(cmd, "SCOPEBUFFER", strlen("SCOPEBUFFER")) == 0) { cmdNum = 13; i1 = 0; i = sscanf(cmd, "SCOPEBUFFER %d", &i1); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeReadIntAndCheck(ssId, pVar, 45, 13, reply); } else if (strncmp(cmd, "SCOPEDATA", strlen("SCOPEDATA")) == 0) { cmdNum = 14; i1 = i2 = 0; i = sscanf(cmd, "SCOPEDATA %d %d", &i1, &i2); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeReadIntAndCheck(ssId, pVar, 45, 14, reply); } else if (strncmp(cmd, "SCOPESTATUS", strlen("SCOPESTATUS")) == 0) { cmdNum = 15; status = writeReadIntAndCheck(ssId, pVar, 45, 15, reply); } else if (strncmp(cmd, "SCOPETRIG STOP", strlen("SCOPETRIG STOP")) == 0) { cmdNum = 16; status = writeIntAndCheck(ssId, pVar, 46, 1); status = writeReadIntAndCheck(ssId, pVar, 45, 16, reply); } else if (strncmp(cmd, "SCOPETRIGPERIOD", strlen("SCOPETRIGPERIOD")) == 0) { cmdNum = 17; i1 = 0; i = sscanf(cmd, "SCOPETRIGPERIOD %d", &i1); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeReadIntAndCheck(ssId, pVar, 45, 17, reply); } else if (strncmp(cmd, "SCOPETRIG", strlen("SCOPETRIG")) == 0) { cmdNum = 16; status = writeIntAndCheck(ssId, pVar, 46, 0); status = writeReadIntAndCheck(ssId, pVar, 45, 16, reply); } else if (strncmp(cmd, "LINEAR", strlen("LINEAR")) == 0) { cmdNum = 19; i1 = 0; d1 = d2 = 0; i = sscanf(cmd, "LINEAR @%d %lf F%lf", &i1, &d1, &d2); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeDoubleAndCheck(ssId, pVar, 1, d1); status = writeDoubleAndCheck(ssId, pVar, 2, d2); status = writeReadIntAndCheck(ssId, pVar, 45, 19, reply); } else if (strncmp(cmd, "DATAACQ", strlen("DATAACQ")) == 0) { if (strncmp(cmd, "DATAACQ_TRIG", strlen("DATAACQ_TRIG")) == 0) { cmdNum = 20; i1 = i2 = 0; i = sscanf(cmd, "DATAACQ_TRIG @%d %d", &i1, &i2); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeReadIntAndCheck(ssId, pVar, 45, 20, reply); } else if (strncmp(cmd, "DATAACQ_INP", strlen("DATAACQ_INP")) == 0) { cmdNum = 21; i1 = i2 = 0; i = sscanf(cmd, "DATAACQ_INP @%d %d", &i1, &i2); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeReadIntAndCheck(ssId, pVar, 45, 21, reply); } else if (strncmp(cmd, "DATAACQ_ON", strlen("DATAACQ_ON")) == 0) { cmdNum = 22; i1 = i2 = 0; i = sscanf(cmd, "DATAACQ_ON @%d %d", &i1, &i2); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeReadIntAndCheck(ssId, pVar, 45, 22, reply); } else if (strncmp(cmd, "DATAACQ_OFF", strlen("DATAACQ_OFF")) == 0) { cmdNum = 23; i1 = 0; i = sscanf(cmd, "DATAACQ_OFF @%d", &i1); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeReadIntAndCheck(ssId, pVar, 45, 23, reply); } else if (strncmp(cmd, "DATAACQ_READ", strlen("DATAACQ_READ")) == 0) { cmdNum = 24; i1 = i2 = i3 = 0; i = sscanf(cmd, "DATAACQ_READ @%d %d %d", &i1, &i2, &i3); status = writeIntAndCheck(ssId, pVar, 46, i1); status = writeIntAndCheck(ssId, pVar, 47, i2); status = writeIntAndCheck(ssId, pVar, 48, i3); status = writeReadIntAndCheck(ssId, pVar, 45, 24, reply); } else { cmdNum = -1; printf("sendReceiveCommand: unexpected command: '%s'\n", cmd); return(-1); } } i = 0; if ((cmdNum != 25) && (cmdNum != 26)) { sprintf(stringOut, "IGLOBAL(%d)", 41); for (i=0; i<300; i++) { status = writeRead(ssId, pVar, stringOut, reply); if (reply[0] == '%') { if (pVar->debugLevel >= 10) printf("atoi(&reply[1])=%d, cmdNum=%d\n", atoi(&reply[1]), cmdNum); if (atoi(&reply[1]) == cmdNum) break; } epicsThreadSleep(0.01); } } pVar->debugLevel = saveDebug; if (i>300 -1) { printf("sendReceiveCommand: timeout (%.2f s) executing '%s'; reply='%s'\n", 300*0.01, cmd, reply); if (callerReply != NULL) { printf("... reply='%s'\n", reply); } status = writeIntAndCheck(ssId, pVar, 45, 0); return(-1); } return(0); } int writeDoubleAndCheck(SS_ID ssId, struct UserVar *pVar, int n, double d) { char stringOut[100], reply[100]; double dCheck = -d; int i, status; for (i=0; i<10; i++) { sprintf(stringOut, "DGLOBAL(%d) = %f", n, d); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); sprintf(stringOut, "DGLOBAL(%d)", n); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); dCheck = atof(&reply[1]); if (fabs(d-dCheck) < .0001) break; printf("writeDoubleAndCheck: fix dglobal %d (%f != %f)\n", n, d, dCheck); } if (i==10) return(-1); return(0); } int writeIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival) { char stringOut[100], reply[100]; int iCheck = -ival; int i, status; for (i=0; i<10; i++) { sprintf(stringOut, "IGLOBAL(%d) = %d", n, ival); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); sprintf(stringOut, "IGLOBAL(%d)", n); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); iCheck = atol(&reply[1]); if (ival == iCheck) break; printf("writeIntAndCheck: fix iglobal %d (%d != %d)\n", n, ival, iCheck); } if (i==10) return(-1); return(0); } int writeReadIntAndCheck(SS_ID ssId, struct UserVar *pVar, int n, int ival, char *callerReply) { char stringOut[100], reply[100]; int iCheck = -ival; int i, status; status = writeIntAndCheck(ssId, pVar, 41, 0); for (i=0; i<10; i++) { sprintf(stringOut, "IGLOBAL(%d) = %d", n, ival); status = writeRead(ssId, pVar, stringOut, callerReply); if (status) return(-1); sprintf(stringOut, "IGLOBAL(%d)", n); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); iCheck = atol(&reply[1]); if (ival == iCheck) break; if (iCheck == 0) { sprintf(stringOut, "IGLOBAL(%d)", 41); status = writeRead(ssId, pVar, stringOut, reply); if (status) return(-1); if (atol(&reply[1]) == ival) break; } printf("writeReadIntAndCheck: fix iglobal %d (%d != %d)\n", n, ival, iCheck); } if (i==10) return(-1); return(0); } static int loadTrajectory(SS_ID ssId, struct UserVar *pVar, int *pstatus) { int i, j, n, status; char stringOut[100], reply[100]; double position, p, v, t; int iGlobalIndex; int intPosition; double accelDist, accelTime, decelDist, decelTime, positionLast; double home_position_set=0.; double posfbkprog, posfbkcal; pVar->loadingTrajectory = 1; *pstatus = 0; sprintf(stringOut, "getparm(@0,%d)", 1); status = writeRead(ssId, pVar, stringOut, reply); pVar->encoder_runs_backwards = atoi(&reply[1]); if (pVar->debugLevel > 2) printf("loadTrajectory: encoder_runs_backwards=%d\n", pVar->encoder_runs_backwards); sprintf(stringOut, "getparm(@0,%d)", 166); status = writeRead(ssId, pVar, stringOut, reply); home_position_set = atof(&reply[1]); if (pVar->debugLevel > 2) printf("loadTrajectory: home_position_set=%f\n", home_position_set); status = writeIntAndCheck(ssId, pVar, 42, 50); for (j=0; j<8; j++) { sprintf(stringOut, "pfbkprog(@%d)", j); status = writeRead(ssId, pVar, stringOut, reply); posfbkprog = atof(&reply[1]); sprintf(stringOut, "pfbkcal(@%d)", j); status = writeRead(ssId, pVar, stringOut, reply); posfbkcal = atof(&reply[1]); pVar->motorPosOffset[j] = posfbkprog - posfbkcal; } if ((pVar->pulseMode == 1) || (pVar->pulseMode == 2)) { int np, nPulses; int maxArrayPulses = ((8192) > (pVar->numGlobalIntegers - 50) ? (pVar->numGlobalIntegers - 50) : (8192)); if ((pVar->pulseMode == 1) && (pVar->pulsePositionsLoaded != 1)) { if (pVar->debugLevel > 2) printf("loadTrajectory: load PULSE_MODE_ARRAY array\n"); j = 0; positionLast = pVar->motorStart[j]; if (pVar->debugLevel) printf("loadTrajectory: motorStart=%f, pVar->pulsePositions[0]=%f\n", pVar->motorStart[j], pVar->pulsePositions[0]); iGlobalIndex = 50; nPulses = pVar->numPulsePositions; if (pVar->numGlobalIntegers < (nPulses + 50)) { printf("EnsembleTrajectoryScan: The Ensemble only has enough global integers for %d pulses\n", pVar->numGlobalIntegers - 50); nPulses = pVar->numGlobalIntegers - 50; } if (nPulses > 8192) { printf("EnsembleTrajectoryScan: The Ensemble can't load more than %d PSOARRAY pulses\n", 8192); nPulses = 8192; } if (nPulses > 8000) nPulses = 8000; pVar->numPulsePositions = nPulses; for (i=0, np=0; i<nPulses && np<maxArrayPulses; i++) { np++; position = pVar->pulsePositions[i]; intPosition = (int)(((position - positionLast)/pVar->epicsMotorMres[j])>0 ? ((position - positionLast)/pVar->epicsMotorMres[j])+0.5 : ((position - positionLast)/pVar->epicsMotorMres[j])-0.5); status = writeIntAndCheck(ssId, pVar, iGlobalIndex++, abs(intPosition)); positionLast += intPosition*pVar->epicsMotorMres[j]; } pVar->pulsePositionsLoaded = 1; pVar->numPulsePositionsLoaded = np; } else if ((pVar->pulseMode == 2) && (pVar->pulsePositionsLoaded != 2)) { if (pVar->debugLevel > 2) printf("loadTrajectory: load PULSE_MODE_TRAJPTS array\n"); j = 0; positionLast = pVar->motorStart[j]; iGlobalIndex = 50; if (pVar->numGlobalIntegers < (pVar->npoints + 50)) { printf("EnsembleTrajectoryScan: The Ensemble only has enough global integers for %d pulses\n", pVar->numGlobalIntegers - 50); } if (pVar->npoints > 8192) { printf("EnsembleTrajectoryScan: The Ensemble can't load more than %d PSOARRAY pulses\n", 8192); } for (i=0, np=0; i<pVar->npoints && np<maxArrayPulses; i++) { if (i >= pVar->startPulses && i <= pVar->endPulses) { np++; position = pVar->motorTrajectory[j][i]; intPosition = (int)(((position - positionLast)/pVar->epicsMotorMres[j])>0 ? ((position - positionLast)/pVar->epicsMotorMres[j])+0.5 : ((position - positionLast)/pVar->epicsMotorMres[j])-0.5); status = writeIntAndCheck(ssId, pVar, iGlobalIndex++, abs(intPosition)); positionLast += intPosition*pVar->epicsMotorMres[j]; } } pVar->pulsePositionsLoaded = 2; pVar->numPulsePositionsLoaded = np; } } getMotorPositions(ssId, pVar, pVar->motorCurrent); strcpy(stringOut, "TASKSTATE(1)"); status = writeRead(ssId, pVar, stringOut, reply); status = atoi(&reply[1]); if ((reply[0] == '!') || ((status != 1) && (status != 3) && (status != 5))) { strcpy(stringOut, "PROGRAM STOP 1"); status = writeRead(ssId, pVar, stringOut, reply); strcpy(stringOut, "TASKSTATE(1)"); status = writeRead(ssId, pVar, stringOut, reply); } status = writeIntAndCheck(ssId, pVar, 45, 0); strcpy(stringOut, "PROGRAM RUN 1, \"doCommand.bcx\""); status = writeRead(ssId, pVar, stringOut, reply); epicsThreadSleep(.1); for (j=0; j<8; j++) { if (pVar->moveAxis[j]) break; } if (j == 8) goto bad; if (pVar->npoints < 2) goto bad; accelTime = pVar->epicsMotorACCL[j]; accelDist = (pVar->velocity[j][0] * pVar->epicsMotorACCL[j]) / 2; decelTime = pVar->epicsMotorACCL[j]; decelDist = (pVar->velocity[j][pVar->npoints - 1] * pVar->epicsMotorACCL[j]) / 2; n = 10; status = 0; status += writeDoubleAndCheck(ssId, pVar, n++, pVar->motorTrajectory[j][0]); status += writeDoubleAndCheck(ssId, pVar, n++, pVar->velocity[j][0]); status += writeDoubleAndCheck(ssId, pVar, n++, accelTime); if (status) goto bad; for (i=1; i<pVar->npoints; i++) { p = pVar->motorTrajectory[j][i]; v = pVar->velocity[j][i]; t = accelTime + pVar->realTimeTrajectory[i]; status += writeDoubleAndCheck(ssId, pVar, n++, p); status += writeDoubleAndCheck(ssId, pVar, n++, v); status += writeDoubleAndCheck(ssId, pVar, n++, t); if (status) goto bad; } p = 2*pVar->motorTrajectory[j][pVar->npoints-1] - pVar->motorTrajectory[j][pVar->npoints-2]; v = pVar->velocity[j][pVar->npoints-1]; t = accelTime + 2*pVar->realTimeTrajectory[pVar->npoints-1] - pVar->realTimeTrajectory[pVar->npoints-2]; status += writeDoubleAndCheck(ssId, pVar, n++, p); status += writeDoubleAndCheck(ssId, pVar, n++, v); status += writeDoubleAndCheck(ssId, pVar, n++, t); if (status) goto bad; p = p + decelDist*.9; v = pVar->velocity[j][pVar->npoints-1]*.1; t = t + decelTime*.9; status += writeDoubleAndCheck(ssId, pVar, n++, p); status += writeDoubleAndCheck(ssId, pVar, n++, v); status += writeDoubleAndCheck(ssId, pVar, n++, t); if (status) goto bad; p = p + decelDist*.1; v = 0.; t = t + decelTime*.1; status += writeDoubleAndCheck(ssId, pVar, n++, p); status += writeDoubleAndCheck(ssId, pVar, n++, v); status += writeDoubleAndCheck(ssId, pVar, n++, t); if (status) goto bad; pVar->actualNumTrajectoryPoints = (n-10)/3; return(0); bad: printf("loadTrajectory: error\n"); strcpy(stringOut, "TASKSTATE(1)"); status = writeRead(ssId, pVar, stringOut, reply); printf("TASKSTATE(1) returns '%s'\n", reply); strcpy(stringOut, "TASKERROR(1)"); status = writeRead(ssId, pVar, stringOut, reply); printf("TASKERROR(1) returns '%s'\n", reply); strcpy(stringOut, "PLANESTATUS(0)"); status = writeRead(ssId, pVar, stringOut, reply); printf("PLANESTATUS(0) returns '%s'\n", reply); strcpy(stringOut, "AXISSTATUS(@0)"); status = writeRead(ssId, pVar, stringOut, reply); printf("AXISSTATUS(@0) returns '%s' (%x)\n", reply, atoi(&reply[1])); pVar->loadingTrajectory = 0; printf("loadTrajectory: aborting because of error\n"); *pstatus = -1; return(-1); } static int executeTrajectory(SS_ID ssId, struct UserVar *pVar) { int i, status, currPoint; char stringOut[100], reply[100]; double dp; if (pVar->pulseMode != 3) { strcpy(stringOut, "PSOCONTROL @0 RESET"); status = writeRead(ssId, pVar, stringOut, reply); if ((pVar->pulseMode == 1) || (pVar->pulseMode == 2)) { sprintf(stringOut, "PSOARRAY @0,%d,%d", 50, pVar->numPulsePositionsLoaded); status = writeRead(ssId, pVar, stringOut, reply); if (pVar->debugLevel > 2) { printf("executeTrajectory: copied PSOARRAY (%d elements) to drive\n", pVar->numPulsePositionsLoaded); } strcpy(stringOut, "PSODISTANCE @0 ARRAY"); status = writeRead(ssId, pVar, stringOut, reply); } sprintf(stringOut, "PSOPULSE @0 TIME %f,%f", pVar->pulseLenUS*1.5, pVar->pulseLenUS); status = writeRead(ssId, pVar, stringOut, reply); strcpy(stringOut, "PSOOUTPUT @0 PULSE"); status = writeRead(ssId, pVar, stringOut, reply); sprintf(stringOut, "PSOTRACK @0 INPUT %d", pVar->pulseSrc); status = writeRead(ssId, pVar, stringOut, reply); sprintf(stringOut, "PSOTRACK @0 DIRECTION %d", pVar->pulseDir); status = writeRead(ssId, pVar, stringOut, reply); if ((pVar->pulseMode == 1) || (pVar->pulseMode == 2)) { strcpy(stringOut, "PSODISTANCE @0 ARRAY"); status = writeRead(ssId, pVar, stringOut, reply); } else if (pVar->pulseMode == 0) { dp = fabs(pVar->motorTrajectory[0][pVar->npoints-1] - pVar->motorTrajectory[0][0]) / pVar->npulses; sprintf(stringOut, "PSODISTANCE @0 FIXED %f UNITS", dp); status = writeRead(ssId, pVar, stringOut, reply); } strcpy(stringOut, "PSOCONTROL @0 ARM"); status = writeRead(ssId, pVar, stringOut, reply); if (pVar->debugLevel > 2) { printf("executeTrajectory: sent PSO commands\n"); } } pVar->startTime = timeNow(); for (i=0; i<8; i++) { if (pVar->moveAxis[i]) break; } status = writeIntAndCheck(ssId, pVar, 46, i); status = writeIntAndCheck(ssId, pVar, 47, pVar->actualNumTrajectoryPoints); status = writeIntAndCheck(ssId, pVar, 48, 10); status = writeIntAndCheck(ssId, pVar, 49, -1); sprintf(stringOut, "DOSTOREDTRAJ"); status = sendReceiveCommand(ssId, pVar, stringOut, reply, 0); if (pVar->abortState != 0) goto abort; strcpy(stringOut, "TASKSTATE(1)"); status = writeRead(ssId, pVar, stringOut, reply); if (pVar->debugLevel) printf("TASKSTATE = '%s', ", reply); strcpy(stringOut, "TASKERROR(1)"); status = writeRead(ssId, pVar, stringOut, reply); if (pVar->debugLevel) printf("TASKERROR = '%s', ", reply); for (i=0; i<50; i++) { sprintf(stringOut, "IGLOBAL(%d)", 49); status = writeRead(ssId, pVar, stringOut, reply); currPoint = atol(&reply[1]); if (pVar->debugLevel) printf("traj point = %d\n", currPoint); if (currPoint > 20) break; epicsThreadSleep(.1); } for (i=0; i<50; i++) { strcpy(stringOut, "PLANESTATUS(0)"); status = writeRead(ssId, pVar, stringOut, reply); if (reply[1] == '3') break; epicsThreadSleep(.1); } if (i>=50) { printf("executeTrajectory: trajectory didn't start. (PLANESTATUS = '%s')\n", reply); strcpy(stringOut, "VELOCITY OFF"); status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1); status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn); return(-2); } if (pVar->debugLevel) printf("PLANESTATUS = '%s'\n", reply); pVar->loadingTrajectory = 0; if (pVar->debugLevel) printf("executeTrajectory: normal exit after %.1f s wait\n", i/10.); return(0); abort: printf("executeTrajectory: aborted\n"); strcpy(stringOut, "VELOCITY OFF"); status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1); strcpy(stringOut, "STARTABORT"); status = sendReceiveCommand(ssId, pVar, stringOut, reply, 1); status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn); pVar->abortState = 2; pVar->loadingTrajectory = 0; if (pVar->pulseMode != 3) { strcpy(stringOut, "PSOCONTROL @0 OFF"); status = writeRead(ssId, pVar, stringOut, reply); } return(-1); } double eval_fStart(double fStart, double *motorReadbacks, double scopeDataInterval, int nScopeDataPoints, double *motorTrajectory, double *realTimeTrajectory, int npoints); int parabola(double *x, double *y, int npts, double *xFit); static double trajEval(SS_ID ssId, struct UserVar *pVar, double *motorReadbacks, int scopeDataIntervalMS, int nScopeDataPoints, double *motorTrajectory, double *realTimeTrajectory, int npoints, double *motorError) { int i, rStart, lower, iThis; double fStart, fThis, frac, interp, scopeDataInterval = scopeDataIntervalMS*.001; double p; double fTry, x[10], y[10]; int nTry; lower = motorReadbacks[0] < motorTrajectory[0]; for (rStart=0; rStart < nScopeDataPoints/2; rStart++) { if (pVar->debugLevel>5) printf("trajEval: motorReadbacks[rStart]=%f, motorTrajectory[0]=%f\n", motorReadbacks[rStart], motorTrajectory[0]); if ((motorReadbacks[rStart] > motorTrajectory[0]) == lower) break; } if (rStart>1 && fabs(motorReadbacks[rStart] - motorReadbacks[rStart-1]) > 1.e-3) { frac = (motorTrajectory[0] - motorReadbacks[rStart-1]) / (motorReadbacks[rStart] - motorReadbacks[rStart-1]); } else { frac = 0.; } fStart = 0.; frac = 0.; if (rStart>0) { fStart = (rStart-1) + frac; } if (pVar->debugLevel>5) printf("trajEval: rStart=%d, frac=%f, fStart=%f\n", rStart, frac, fStart); fTry = ((0) > (fStart-2) ? (0) : (fStart-2)); nTry = 5; for (i=0; i<nTry; i++, fTry += 1) { x[i] = fTry; y[i] = eval_fStart(fTry, motorReadbacks, scopeDataInterval, nScopeDataPoints, motorTrajectory, realTimeTrajectory, npoints); } i = parabola(x, y, nTry, &fTry); if (i==0) fStart = fTry; if (pVar->debugLevel>5) printf("trajEval: fStart=%f\n", fStart); for (i=0; i<npoints; i++) { fThis = fStart + (realTimeTrajectory[i] - realTimeTrajectory[0])/scopeDataInterval; iThis = floor(fThis); if (iThis+1 < nScopeDataPoints) { if (pVar->debugLevel>5) printf("trajEval: iThis=%d, motorReadbacks[iThis]=%f\n", iThis, motorReadbacks[iThis]); p = fThis-iThis; if ((iThis > 0) && (iThis+2 < nScopeDataPoints)) { interp = (-p*(p-1)*(p-2)/6)*motorReadbacks[iThis-1] + ((p*p-1)*(p-2)/2)*motorReadbacks[iThis] + (-p*(p+1)*(p-2)/2)*motorReadbacks[iThis+1] + (p*(p*p-1)/6)*motorReadbacks[iThis+2]; } else if ((iThis > 0) && (iThis+1 < nScopeDataPoints)) { interp = (p*(p-1)/2)*motorReadbacks[iThis-1] + (1-p*p)*motorReadbacks[iThis] + (p*(p+1)/2)*motorReadbacks[iThis+1]; } else { interp = motorReadbacks[iThis] + (motorReadbacks[iThis+1] - motorReadbacks[iThis])*p; } if (pVar->debugLevel>5) printf("trajEval: fThis=%f, interp=%f, motorTrajectory[%d]=%f, \n", fThis, interp, i, motorTrajectory[i]); motorReadbacks[i] = interp; motorError[i] = interp - motorTrajectory[i]; } else { motorReadbacks[i] = 0; motorError[i] = 0; } } for(; i<nScopeDataPoints; i++) { motorReadbacks[i] = 0; motorError[i] = 0; } return(0.); } double eval_fStart(double fStart, double *motorReadbacks, double scopeDataInterval, int nScopeDataPoints, double *motorTrajectory, double *realTimeTrajectory, int npoints) { double fThis, p, interp, chisq = 0; int i, iThis, numChi = 0; for (i=0; i<npoints; i++) { fThis = fStart + (realTimeTrajectory[i] - realTimeTrajectory[0])/scopeDataInterval; iThis = floor(fThis); if (iThis+1 < nScopeDataPoints) { p = fThis-iThis; if ((iThis > 0) && (iThis+2 < nScopeDataPoints)) { interp = (-p*(p-1)*(p-2)/6)*motorReadbacks[iThis-1] + ((p*p-1)*(p-2)/2)*motorReadbacks[iThis] + (-p*(p+1)*(p-2)/2)*motorReadbacks[iThis+1] + (p*(p*p-1)/6)*motorReadbacks[iThis+2]; } else if ((iThis > 0) && (iThis+1 < nScopeDataPoints)) { interp = (p*(p-1)/2)*motorReadbacks[iThis-1] + (1-p*p)*motorReadbacks[iThis] + (p*(p+1)/2)*motorReadbacks[iThis+1]; } else { interp = motorReadbacks[iThis] + (motorReadbacks[iThis+1] - motorReadbacks[iThis])*p; } chisq += (interp - motorTrajectory[i]) * (interp - motorTrajectory[i]); numChi++; } } return(chisq/numChi); } int parabola(double *x, double *y, int npts, double *xFit) { double xf, yf; double xx, yy, xi, xi2, xi3, xi4, yi, yixi, yixi2; int i; double numer, denom; double xmin=1e10, xmax=-1e10; xi = yi = yixi = xi2 = yixi2 = xi3 = xi4 = 0.0; for (i=0; i<npts; i++) { xmin = ((xmin) > (x[i]) ? (x[i]) : (xmin)); xmax = ((xmax) > (x[i]) ? (xmax) : (x[i])); xi += (xx = x[i]); yi += (yy = y[i]); yixi += yy * xx; xi2 += (xx *=x[i]); yixi2 += yy * xx; xi3 += (xx *= x[i]); xi4 += xx * x[i]; } xi /= npts; xi2 /= npts; xi3 /= npts; xi4 /= npts; yi /= npts; yixi /= npts; yixi2 /= npts; numer = yi*(xi2*xi3 - xi*xi4) + yixi*(xi4 - xi2*xi2) + yixi2*(xi*xi2 - xi3); denom = 2*(yi*(xi2*xi2 - xi*xi3) + yixi*(xi3 - xi*xi2) + yixi2*(xi*xi - xi2)); if (fabs(denom) < 1e-8) return(-1); xf = numer / denom; if ((xf < xmin) || (xf > xmax)) return(-1); numer = (xf*xf - 2*xf*xi + xi2) * (yixi2 - yi*xi2); denom = 2*xf*(xi*xi2 - xi3) - xi2*xi2 + xi4; yf = yi - numer/denom; if (fabs(denom) < 1e-8) return(-1); *xFit = xf; return(0); }"#) => {};
    ($env:ident, r#"pVar->status = pasynOctetSyncIO->connect(pVar->asynPort, 0, (asynUser **)&pVar->pasynUser, NULL);"#) => {
        $env.var().status.set(connect($env.var().asynPort.get()))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, "getparm(125)", pVar->stringIn);"#) => {
        $env.var()
            .status
            .set(write_read($env, "getparm(125)", &$env.var().stringIn))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, "getparm(@0,125)", pVar->stringIn);"#) => {
        $env.var()
            .status
            .set(write_read($env, "getparm(@0,125)", &$env.var().stringIn))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, "getparm(124)", pVar->stringIn);"#) => {
        $env.var()
            .status
            .set(write_read($env, "getparm(124)", &$env.var().stringIn))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, "getparm(@0,124)", pVar->stringIn);"#) => {
        $env.var()
            .status
            .set(write_read($env, "getparm(@0,124)", &$env.var().stringIn))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, pVar->abortCommand, pVar->stringIn);"#) => {
        $env.var().status.set(write_read(
            $env,
            &$env.var().abortCommand,
            &$env.var().stringIn,
        ))
    };
    ($env:ident, r#"pVar->status = writeRead(ssId, pVar, pVar->stringOut, pVar->stringIn);"#) => {
        $env.var().status.set(write_read(
            $env,
            &$env.var().stringOut,
            &$env.var().stringIn,
        ))
    };
    (
        $env:ident,
        r#"pVar->status = buildTrajectory(ssId, pVar, pVar->realTimeTrajectory, pVar->motorTrajectory[pVar->j],"#,
        r#"pVar->epicsMotorDir[pVar->j], pVar->moveMode, pVar->npoints, pVar->npulses,"#,
        r#"pVar->epicsMotorOff[pVar->j], pVar->epicsMotorMres[pVar->j], pVar->velocity[pVar->j]);"#
    ) => {
        $env.var()
            .status
            .set(build_trajectory($env, $env.var().j.get() as usize))
    };
    ($env:ident, r#"waitEpicsMotors(ssId, pVar);"#) => {
        wait_epics_motors($env)
    };
    ($env:ident, r#"loadTrajectory(ssId, pVar, pVar->pstatus);"#) => {
        load_trajectory($env, $env.var().pstatus.get())
    };
    ($env:ident, r#"getMotorPositions(ssId, pVar, pVar->motorCurr);"#) => {
        get_motor_positions($env, &$env.var().motorCurr)
    };
    ($env:ident, r#"getMotorPositions(ssId, pVar, pVar->motorStart);"#) => {
        get_motor_positions($env, &$env.var().motorStart)
    };
    ($env:ident, r#"getMotorPositions(ssId, pVar, pVar->motorCurrent);"#) => {
        get_motor_positions($env, &$env.var().motorCurrent)
    };
    ($env:ident, r#"pVar->lastPollTime = timeNow();"#) => {
        $env.var().lastPollTime.set(time_now())
    };
    ($env:ident, r#"pVar->startTime = timeNow();"#) => {
        $env.var().startTime.set(time_now())
    };
    ($env:ident, r#"pVar->currTime = timeNow();"#) => {
        $env.var().currTime.set(time_now())
    };
    ($env:ident, r#"pVar->status = executeTrajectory(ssId, pVar);"#) => {
        $env.var().status.set(execute_trajectory($env))
    };
    ($env:ident, r#"pVar->anyMoving = getMotorMoving(ssId, pVar, pVar->movingMask);"#) => {
        $env.var().anyMoving.set(get_motor_moving($env))
    };
}

epics_seq_rs::include_program!("EnsembleTrajectoryScan");

#[cfg(test)]
mod tests {
    use epics_seq_rs::{ProgOptions, ProgramVars, VarModel};
    use motor_common::testing::LineRig;

    use super::*;

    /// EnsembleTrajectoryScan.st:1, with `option +r;` (:42).
    #[test]
    fn the_program_is_the_st_one() {
        assert_eq!(EnsembleTrajectoryScan.validate(), Ok(()));
        assert_eq!(
            EnsembleTrajectoryScan.params,
            "P=13IDC:,R=traj1,M1=M1,M2=M2,M3=M3,M4=M4,M5=M5,M6=M6,M7=M7,M8=M8,PORT=serial1"
        );
        assert_eq!(
            EnsembleTrajectoryScan.options,
            ProgOptions::new(ProgOptions::CONN | ProgOptions::NEWEF)
        );
        assert!(matches!(EnsembleTrajectoryScan.vars, VarModel::Reentrant));
    }

    /// `NINT` rounds half away from zero.
    #[test]
    fn nint_rounds_half_away_from_zero() {
        assert_eq!([2.5, 2.49, -2.5, -2.49, 0.0].map(nint), [3, 2, -3, -2, 0]);
    }

    /// `epicsTimeDiffInSeconds` is signed.
    /// The `cmd*` numbers of EnsembleTrajectoryScan.h:363-388.
    #[test]
    fn command_numbers_are_do_command_ab_ones() {
        let numbers = [
            Command::VelocityOff,
            Command::StartAbort,
            Command::DoStoredTraj,
            Command::Version,
        ]
        .map(Command::number);
        assert_eq!(numbers, [2, 12, 26, 27]);
    }

    /// writeRead clears the reply before the read, so a short reply keeps
    /// nothing of the last one.
    #[test]
    fn exchange_clears_the_reply_before_the_read() {
        let (rig, user) = LineRig::new("ens_snl_exchange", |cmd| match cmd {
            "TASKSTATE(1)" => "%3".into(),
            _ => "!".into(),
        });
        USERS
            .lock()
            .unwrap()
            .insert("ens_snl_exchange".into(), Arc::new(user));
        let v = seqg::seqg_vars::zeroed();
        v.asynPort.set(CharPtr::from("ens_snl_exchange"));
        let reply = message();
        strcpy(&reply, "%123456789");
        assert_eq!(exchange(&v, b"TASKSTATE(1)", &reply), 0);
        assert_eq!(text(&reply), "%3");
        assert_eq!(rig.sent(), ["TASKSTATE(1)"]);
    }

    /// C's connectDevice refuses a NULL port name, and every transfer
    /// through that user is then an asynError.
    #[test]
    fn a_null_port_connects_to_nothing() {
        assert_ne!(connect(CharPtr::default()), 0);
        let v = seqg::seqg_vars::zeroed();
        let reply = message();
        assert_eq!(exchange(&v, b"TASKSTATE(1)", &reply), 3);
        assert_eq!(text(&reply), "");
    }

    mod program {
        use std::collections::HashMap;
        use std::time::Instant;

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

        use super::*;

        const DB: &str =
            include_str!("../../../../../iocs/motor/aerotech-ioc/db/trajectoryScan.db");

        /// `$(M1)` on a simulated axis, within ±100 and at most 1 per second.
        const MOTOR: &str = r#"record(motor, "T:$(M1)") { field(DTYP, "asynMotor")
            field(OUT, "@asyn($(PORT)m1,0)") field(MRES, "0.001") field(VELO, "1")
            field(VMAX, "1") field(ACCL, "0.2") field(DHLM, "100") field(DLLM, "-100") }"#;

        /// The controller doCommand.bcx runs on: globals read back as
        /// written, a `cmdVar` run, acked and zeroed at once as doCommand.ab
        /// does (VERSION as 1.2, DOSTOREDTRAJ through its last point with
        /// the plane left moving), 100 globals of each kind, and `%0` to
        /// anything else.
        fn ensemble(globals: &Mutex<HashMap<String, String>>, cmd: &str) -> String {
            let mut g = globals.lock().unwrap();
            if let Some((global, value)) = cmd.split_once(" = ") {
                if global == format!("IGLOBAL({CMD_VAR})") && value != "0" {
                    if value == Command::Version.number().to_string() {
                        g.insert(format!("IGLOBAL({IARG1_VAR})"), "1".into());
                        g.insert(format!("IGLOBAL({IARG2_VAR})"), "2".into());
                    }
                    if value == Command::DoStoredTraj.number().to_string() {
                        let points: i32 = g[&format!("IGLOBAL({IARG2_VAR})")].parse().unwrap();
                        g.insert(format!("IGLOBAL({IARG4_VAR})"), (points - 1).to_string());
                        g.insert("PLANESTATUS(0)".into(), "3".into());
                    }
                    g.insert(format!("IGLOBAL({CMD_ACK_VAR})"), value.into());
                    g.insert(global.into(), "0".into());
                } else {
                    g.insert(global.into(), value.into());
                }
                return "%".into();
            }
            match cmd {
                "getparm(124)" | "getparm(125)" => "%100".into(),
                "TASKSTATE(1)" => "%3".into(),
                _ if cmd == format!("GETPARM({VELOCITY_COMMAND_THRESHOLD_PARM_NUM})") => {
                    "%100".into()
                }
                _ => format!("%{}", g.get(cmd).map_or("0", String::as_str)),
            }
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

        struct Program {
            db: Arc<PvDatabase>,
            tid: SeqThreadId,
            _scan: Arc<ScanOwner>,
            _ensemble: (LineRig, SyncIOHandle),
        }

        /// The program on NumAxes 1 and an Ensemble at `port`, m1 to move
        /// along `traj` in a second, once init is done.
        async fn boot(port: &str, traj: [f64; 2]) -> Program {
            boot_each(&[("traj1:", "m1", port)], traj).await.remove(0)
        }

        /// [`boot`] for each `(R, M1, port)`, in one database as in an IOC:
        /// seq resolves every instance's pvs through the first one's.
        async fn boot_each(instances: &[(&str, &str, &str)], traj: [f64; 2]) -> Vec<Program> {
            let (motor, factory) = epics_rs::motor::motor_record_factory();
            let mut ioc = IocBuilder::new()
                .register_record_type(motor, factory)
                .register_dynamic_device_support(
                    &epics_rs::motor::devices::devices(),
                    epics_rs::motor::devices::device_support_factory(),
                )
                .register_record_type("busy", || Box::new(epics_rs::busy::BusyRecord::default()));
            let mut rigs = Vec::new();
            for &(r, m1, port) in instances {
                let globals = Mutex::default();
                rigs.push(LineRig::new(port, move |cmd| {
                    ensemble(&globals, cmd.trim())
                }));
                sim_axis(port);
                let macros: HashMap<String, String> = [
                    ("P", "T:"),
                    ("R", r),
                    ("M1", m1),
                    ("NAXES", "1"),
                    ("NELM", "1000"),
                    ("NPULSE", "1000"),
                    ("PORT", port),
                ]
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .into();
                ioc = ioc
                    .db_string(DB, &macros)
                    .unwrap()
                    .db_string(MOTOR, &macros)
                    .unwrap();
            }
            let (db, _) = ioc.build_served().await.unwrap();
            for &(r, ..) in instances {
                put(&db, &format!("T:{r}Nelements"), EpicsValue::Long(2)).await;
                put(&db, &format!("T:{r}Time"), EpicsValue::Double(1.0)).await;
                put(
                    &db,
                    &format!("T:{r}M1Traj"),
                    EpicsValue::DoubleArray(traj.into()),
                )
                .await;
            }
            // seq holds each state set in its first state until PINI is done.
            let scan = Arc::new(ScanOwner::start(db.clone()));
            let mut programs = Vec::new();
            for (&(r, m1, port), (rig, io)) in instances.iter().zip(rigs) {
                let macros = format!("P=T:,R={r},M1={m1},PORT={port}");
                let tid = seq(&EnsembleTrajectoryScan, &macros, 0, &db)
                    .expect("EnsembleTrajectoryScan starts");
                // init clears buildMon before its last reads.
                let mut sent = Vec::new();
                for _ in 0..500 {
                    sent.extend(rig.sent());
                    if sent.iter().any(|c| c.trim() == "getparm(124)") {
                        break;
                    }
                    sleep(Duration::from_millis(10)).await;
                }
                assert!(
                    sent.iter().any(|c| c.trim() == "getparm(124)"),
                    "init never read the global integers: {sent:?}"
                );
                programs.push(Program {
                    db: db.clone(),
                    tid,
                    _scan: scan.clone(),
                    _ensemble: (rig, io),
                });
            }
            programs
        }

        async fn put(db: &PvDatabase, pv: &str, v: EpicsValue) {
            db.put_record_field_from_ca(pv, "VAL", v).await.unwrap();
        }

        fn value(db: &PvDatabase, pv: &str) -> f64 {
            db.get_pv(pv).unwrap().to_f64().unwrap()
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

        /// EnsembleTrajectoryScan.st:421: a build past VMAX fails whichever
        /// way the motor goes.
        #[epics_rs::base::epics_test]
        async fn a_build_past_vmax_backwards_fails() {
            let p = boot("ensVlimit", [0.0, -5.0]).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 2.0)],
            )
            .await;
            assert_eq!(
                p.db.get_pv("T:traj1:BuildMessage").unwrap(),
                EpicsValue::String("V limit: m1 at pt. 1 (-5.000000)".into())
            );
            seq_stop(p.tid);
        }

        /// EnsembleTrajectoryScan.st:187: init sizes the program to NumAxes
        /// once, so one put past MAX_AXES after it leaves a build as it was.
        #[epics_rs::base::epics_test]
        async fn a_num_axes_put_after_init_leaves_the_build_alone() {
            let p = boot("ensNumAxes", [0.0, 0.5]).await;
            put(&p.db, "T:traj1:NumAxes", EpicsValue::Long(9)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
            seq_stop(p.tid);
        }

        /// EnsembleTrajectoryScan.st:1033: a flat one-point build divides 0
        /// by 0 for its velocity, which fails the build before m1 is sent
        /// to a start position computed from it.
        #[epics_rs::base::epics_test]
        async fn a_nan_velocity_fails_the_build() {
            let p = boot("ensNaN", [0.5, 0.5]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(1)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 2.0)],
            )
            .await;
            assert_eq!(
                p.db.get_pv("T:traj1:BuildMessage").unwrap(),
                EpicsValue::String("Can't build m1".into())
            );
            assert_eq!(value(&p.db, "T:m1"), 0.0);
            seq_stop(p.tid);
        }

        /// EnsembleTrajectoryScan.st:1404: a command doCommand.ab has run
        /// and zeroed is not written again, for it to run again.
        #[epics_rs::base::epics_test]
        async fn a_command_do_command_ran_is_written_once() {
            let p = boot("ensCmdOnce", [0.0, 0.5]).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
            let version = format!("IGLOBAL({CMD_VAR}) = {}", Command::Version.number());
            let sent = p._ensemble.0.sent();
            assert_eq!(sent.iter().filter(|c| c.trim() == version).count(), 1);
            seq_stop(p.tid);
        }

        /// EnsembleTrajectoryScan.st:113: an execute times out at twice its
        /// expected 1 s from its own start, however late another instance
        /// starts one.
        #[epics_rs::base::epics_test]
        async fn an_execute_times_out_from_its_own_start() {
            let programs = boot_each(
                &[("trajA:", "m1", "ensTimeA"), ("trajB:", "m2", "ensTimeB")],
                [0.0, 0.0],
            )
            .await;
            for (p, r) in programs.iter().zip(["trajA:", "trajB:"]) {
                // Past 20 points, executeTrajectory stops waiting for the plane.
                put(&p.db, &format!("T:{r}Nelements"), EpicsValue::Long(25)).await;
                let flat = EpicsValue::DoubleArray(vec![0.0; 25].into());
                put(&p.db, &format!("T:{r}M1Traj"), flat).await;
                put(&p.db, &format!("T:{r}Build"), EpicsValue::Enum(1)).await;
                expect(
                    &p.db,
                    &[
                        (&format!("T:{r}Build"), 0.0),
                        (&format!("T:{r}BuildStatus"), 1.0),
                    ],
                )
                .await;
            }
            let (a, b) = (&programs[0], &programs[1]);
            put(&a.db, "T:trajA:Execute", EpicsValue::Enum(1)).await;
            expect(&a.db, &[("T:trajA:ExecState", 2.0)]).await;
            let start = Instant::now();
            while value(&a.db, "T:trajA:ElapsedTime") < 1.4 {
                assert!(
                    start.elapsed() < Duration::from_secs(2),
                    "trajA never polled"
                );
                sleep(Duration::from_millis(10)).await;
            }
            put(&b.db, "T:trajB:Execute", EpicsValue::Enum(1)).await;
            expect(&a.db, &[("T:trajA:ExecStatus", 4.0)]).await;
            let timeout = start.elapsed();
            assert!(
                timeout < Duration::from_secs(3),
                "trajA timed out after {timeout:?}"
            );
            expect(&b.db, &[("T:trajB:ExecStatus", 4.0)]).await;
            seq_stop(a.tid);
            seq_stop(b.tid);
        }
    }
}
