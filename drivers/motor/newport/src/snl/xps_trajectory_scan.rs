//! `XPS_trajectoryScan.st`, compiled by snc-rs from `build.rs` and run on
//! `epics-seq-rs`.
//!
//! The escaped C follows: the `%{ }%` functions as Rust, on the
//! XPS_C8_drivers.cpp calls of `xps_c8` and the xps_ftp.c ones of
//! [`crate::xps::ftp`], then one `seqg_escaped!` arm per fragment, its text
//! as cpp hands it to snc. `getMotorPositions` is only ever handed
//! `pVar->motorCurrent`, and fills it here.

use std::io::Write;
use std::time::{SystemTime, UNIX_EPOCH};

use epics_libcom_rs::cfmt::CFloat;
use epics_libcom_rs::runtime::general_time;
use epics_rs::base::server::builtin_devices::timestamp::epics_time_to_strftime;
use epics_seq_rs::c::*;
use epics_seq_rs::{ChId, CharPtr, CompType, Place, SeqVar, StateSet};

use super::xps_c8::{c_int, socket, tcp_connect_to_server, text};
use crate::xps::ftp;

type Env = StateSet<seqg::seqg_vars>;

const MAX_AXES: usize = 8;
const MAX_PULSES: i32 = 2000;
const MOVE_MODE_RELATIVE: i32 = 0;
const STATUS_SUCCESS: i32 = 1;
const STATUS_FAILURE: i32 = 2;
const POLL_INTERVAL: f64 = 0.1;
const NUM_GATHERING_ITEMS: i32 = 2;
const TRAJECTORY_DIRECTORY: &str = "/Admin/public/Trajectories";
/// XPS_C8_drivers.cpp's list buffer and each name in it.
const SIZE_NOMINAL: usize = 1024;
const SIZE_NAME: usize = 100;

use positioner_user_travel_limits_get as PositionerUserTravelLimitsGet;
use print_message as printMessage;

/// C `printMessage`: the time, then `pformat`, on stdout.
fn print_message(pformat: &str, args: &[CArg<'_>]) {
    let now = epics_time_to_strftime("%Y/%m/%d %H:%M:%S.%03f", general_time::get_current());
    printf("%s ", &[now.as_str().into()]);
    printf(pformat, args);
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

/// `axisName[j]`, the `GROUP.Pn` init made.
fn axis_name(v: &seqg::seqg_vars, j: usize) -> String {
    text(v.axisName.at(j).get())
}

/// The C `int dir` of axis `j`: 1 when its motor record's `DIR` is Pos.
fn dir(v: &seqg::seqg_vars, j: usize) -> f64 {
    if v.epicsMotorDir[j].get() == 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// The first `n` names of a list as XPS_C8_drivers.cpp's `strtok_r` cuts
/// it on `" \t;"`, from its first `SIZE_NOMINAL - 1` bytes, each name to
/// `SIZE_NAME - 1`.
fn split_list(list: &str, n: i32) -> Vec<String> {
    let list = &list.as_bytes()[..list.len().min(SIZE_NOMINAL - 1)];
    list.split(|c| b" \t;".contains(c))
        .filter(|t| !t.is_empty())
        .take(usize::try_from(n).unwrap_or(0))
        .map(|t| String::from_utf8_lossy(&t[..t.len().min(SIZE_NAME - 1)]).into_owned())
        .collect()
}

/// C `PositionerUserTravelLimitsGet(socket, positioner, &low, &high)`;
/// as C, both keep their values when the call fails.
fn positioner_user_travel_limits_get(
    index: i32,
    positioner: CharPtr,
    low: &impl Place<f64>,
    high: &impl Place<f64>,
) -> i32 {
    match socket(index).positioner_user_travel_limits_get(&text(&positioner)) {
        Ok((l, h)) => {
            low.modify(|v| *v = l);
            high.modify(|v| *v = h);
            0
        }
        Err(e) => e.code(),
    }
}

/// C `getMotorPositions(ssId, pVar, pVar->motorCurrent)`: the group's
/// positions, in user units.
fn get_motor_positions(v: &seqg::seqg_vars) -> i32 {
    let mut pos: Vec<f64> = axes(v).map(|j| v.motorCurrent[j].get()).collect();
    let status = c_int(
        socket(v.positionSocket.get()).group_positions_current_get(&text(&v.groupName), &mut pos),
    );
    if status != 0 {
        print_message(
            "Error performing GroupPositionCurrentGet%i\n",
            &[status.into()],
        );
        return status;
    }
    for (j, p) in pos.into_iter().enumerate() {
        v.motorCurrent[j].set(p * dir(v, j) + v.epicsMotorOff[j].get());
    }
    status
}

/// C `GroupStatusGet` into the `int groupStatus` of `getMotorMoving` and
/// `getGroupStatus`, 0 when the call fails.
fn group_status(v: &seqg::seqg_vars, error: &str) -> i32 {
    match socket(v.pollSocket.get()).group_status_get(&text(&v.groupName)) {
        Ok(status) => status,
        Err(e) => {
            print_message(error, &[e.code().into()]);
            0
        }
    }
}

/// C `getMotorMoving`: 1 while the group status says it moves.
fn get_motor_moving(v: &seqg::seqg_vars) -> i32 {
    i32::from(group_status(v, "Error performing GroupStatusGet %i\n") > 42)
}

/// C `getGroupStatus`.
fn get_group_status(v: &seqg::seqg_vars) -> i32 {
    group_status(v, "Error performing GroupStatusGet, status=%d\n")
}

/// `seq_pvPut` of each `motorCurrent`.
fn put_motor_current(env: &Env) {
    let v = env.var();
    for j in axes(v) {
        env.pv_put(
            ChId(v.motorCurrentIndex.at(j).get() as u32),
            CompType::Default,
        );
    }
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

/// C `waitMotors`: post the group's positions until it stops.
fn wait_motors(env: &Env) {
    let v = env.var();
    while get_motor_moving(v) != 0 {
        get_motor_positions(v);
        put_motor_current(env);
        epicsThreadSleep(POLL_INTERVAL);
    }
    get_motor_positions(v);
    put_motor_current(env);
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

/// C `getSocket`.
fn get_socket(env: &Env, timeout: f64) -> i32 {
    let v = env.var();
    let sock = tcp_connect_to_server(env, &v.xpsAddress, v.xpsPort.get(), timeout);
    if sock < 0 {
        print_message("Error TCP_ConnectToServer, status=%d\n", &[sock.into()]);
    }
    sock
}

/// C `trajectoryExecute`: gathering and the pulse event set up, then the
/// trajectory run, which returns when it ends, and both taken down.
fn trajectory_execute(v: &seqg::seqg_vars) {
    let debug = v.debugLevel.get() > 0;
    let poll_socket = v.pollSocket.get();
    let poll = socket(poll_socket);
    let group = text(&v.groupName);
    if debug {
        print_message("XPS_trajectoryScan: trajectoryExecute: entry\n", &[]);
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling GatheringReset(%d)\n",
            &[poll_socket.into()],
        );
    }
    let status = c_int(poll.gathering_reset());
    if status != 0 {
        print_message(
            "Error performing GatheringReset, status=%d\n",
            &[status.into()],
        );
        return;
    }

    let mut buffer = String::new();
    for j in axes(v) {
        let name = axis_name(v, j);
        buffer += &format!("{name}.SetpointPosition;{name}.CurrentPosition;");
    }
    let n = v.numAxes.get() * NUM_GATHERING_ITEMS;
    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling GatheringConfigurationSet(%d, %d, %s)\n",
            &[poll_socket.into(), n.into(), buffer.as_str().into()],
        );
    }
    let status = c_int(poll.gathering_configuration_set(&split_list(&buffer, n)));
    if status != 0 {
        print_message(
            "Error performing GatheringConfigurationSet, status=%d, buffer=%p\n",
            &[status.into(), buffer.as_ptr().addr().into()],
        );
    }

    let (start, end, period) = (
        v.startPulses.get() + 1,
        v.endPulses.get() + 1,
        v.pulsePeriod.get(),
    );
    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling MultipleAxesPVTPulseOutputSet(%d, %s, %d, %d, %f)\n",
            &[
                poll_socket.into(),
                group.as_str().into(),
                start.into(),
                end.into(),
                period.into(),
            ],
        );
    }
    // C keeps no status of this one.
    let _ = poll.multiple_axes_pvt_pulse_output_set(&group, start, end, period);

    let buffer = format!("Always;{group}.PVT.TrajectoryPulse");
    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling EventExtendedConfigurationTriggerSet(%d, %d, %s, %s, %s, %s, %s)\n",
            &[
                poll_socket.into(),
                2.into(),
                buffer.as_str().into(),
                "".into(),
                "".into(),
                "".into(),
                "".into(),
            ],
        );
    }
    let status = c_int(poll.event_extended_configuration_trigger_set(&split_list(&buffer, 2)));
    if status != 0 {
        print_message(
            "Error performing EventExtendedConfigurationTriggerSet, status=%d, buffer=%s\n",
            &[status.into(), buffer.as_str().into()],
        );
        return;
    }

    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling EventExtendedConfigurationActionSet(%d, %d, %s, %s, %s, %s, %s)\n",
            &[
                poll_socket.into(),
                1.into(),
                "GatheringOneData".into(),
                "".into(),
                "".into(),
                "".into(),
                "".into(),
            ],
        );
    }
    let status =
        c_int(poll.event_extended_configuration_action_set(&split_list("GatheringOneData", 1)));
    if status != 0 {
        print_message(
            "Error performing EventExtendedConfigurationActionSet, status=%d\n",
            &[status.into()],
        );
        return;
    }

    let mut event_id = 0;
    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling EventExtendedStart(%d, %p)\n",
            &[poll_socket.into(), (&raw const event_id).addr().into()],
        );
    }
    match poll.event_extended_start() {
        Ok(id) => event_id = id,
        Err(e) => {
            print_message(
                "Error performing EventExtendedStart, status=%d\n",
                &[e.code().into()],
            );
            return;
        }
    }

    let file = text(&v.trajectoryFile);
    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling MultipleAxesPVTExecution(%d, %s, %s, %d)\n",
            &[
                v.driveSocket.get().into(),
                group.as_str().into(),
                file.as_str().into(),
                1.into(),
            ],
        );
    }
    let status = c_int(socket(v.driveSocket.get()).multiple_axes_pvt_execution(&group, &file, 1));
    // -27: the trajectory was aborted.
    if status != 0 && status != -27 {
        print_message(
            "Error performing MultipleAxesPVTExecution, status=%d\n",
            &[status.into()],
        );
    }

    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling EventExtendedRemove(%d, %d)\n",
            &[poll_socket.into(), event_id.into()],
        );
    }
    let status = c_int(poll.event_extended_remove(event_id));
    if status != 0 {
        print_message(
            "Error performing ExtendedEventRemove, status=%d\n",
            &[status.into()],
        );
        return;
    }

    if debug {
        print_message(
            "XPS_trajectoryScan: trajectoryExecute: calling GatheringStop(%d)\n",
            &[poll_socket.into()],
        );
    }
    let status = c_int(poll.gathering_stop());
    // -30: aborted before the end of the first element, so never started.
    if status != 0 && status != -30 {
        print_message(
            "Error performing GatheringStop, status=%d\n",
            &[status.into()],
        );
    }
}

/// C `buildAndVerify`: the trajectory file, with an acceleration element
/// before it and a deceleration one after it at each axis's maximum
/// acceleration, written, sent to the XPS and verified there.
fn build_and_verify(v: &seqg::seqg_vars) {
    let debug = v.debugLevel.get() > 0;
    let poll_socket = v.pollSocket.get();
    let poll = socket(poll_socket);
    if debug {
        print_message("XPS_trajectoryScan: buildAndVerify: entry\n", &[]);
    }
    let relative = v.buildMode.get() == MOVE_MODE_RELATIVE;
    let nelements = v.buildElements.get();
    let time_trajectory = v.timeTrajectory.get();
    let trajectory: Vec<[f64; 2000]> = axes(v).map(|j| v.motorTrajectory[j].get()).collect();
    let last = (nelements - 1) as usize;

    let (mut pre_time_max, mut post_time_max) = (0.0, 0.0);
    let mut pre_velocity = [0.0; MAX_AXES];
    let mut post_velocity = [0.0; MAX_AXES];
    for j in axes(v) {
        if v.buildMoveAxis.at(j).get() == 0 {
            continue;
        }
        let max_acceleration = match poll.positioner_sgamma_parameters_get(&axis_name(v, j)) {
            Ok((_, accel, _, _)) => accel,
            Err(e) => {
                print_message(
                    "Error calling positionerSGammaParametersSet, status=%d\n",
                    &[e.code().into()],
                );
                0.0
            }
        };
        // 10% off for the roundoff of the ASCII the XPS is sent.
        let max_acceleration = 0.9 * max_acceleration;
        let traj = &trajectory[j];
        let distance = if relative { traj[0] } else { traj[1] - traj[0] };
        pre_velocity[j] = distance / time_trajectory[0];
        let time = fabs(pre_velocity[j]) / max_acceleration;
        if time > pre_time_max {
            pre_time_max = time;
        }
        let distance = if relative {
            traj[last]
        } else {
            traj[last] - traj[(nelements - 2) as usize]
        };
        post_velocity[j] = distance / time_trajectory[last];
        let time = fabs(post_velocity[j]) / max_acceleration;
        if time > post_time_max {
            post_time_max = time;
        }
    }

    // In user units: the EPICS moves before and after the scan use them.
    for j in axes(v) {
        v.preDistance
            .at(j)
            .set(0.5 * pre_velocity[j] * pre_time_max);
        v.postDistance
            .at(j)
            .set(0.5 * post_velocity[j] * post_time_max);
    }

    let file_name = text(&v.trajectoryFile);
    let Ok(mut traj_file) = std::fs::File::create(&file_name) else {
        print_message(
            "Error calling fopen, file=%s\n",
            &[file_name.as_str().into()],
        );
        return;
    };
    let dir: Vec<f64> = axes(v).map(|j| dir(v, j)).collect();
    let mut out = format!("{}", CFloat(pre_time_max));
    for j in axes(v) {
        out += &format!(
            ", {}, {}",
            CFloat(v.preDistance.at(j).get() * dir[j]),
            CFloat(pre_velocity[j] * dir[j])
        );
    }
    out += "\n";

    let npoints = if relative { nelements } else { nelements - 1 };
    for i in 0..npoints {
        let iu = i as usize;
        let t0 = time_trajectory[iu];
        let t1 = if i < npoints - 1 {
            time_trajectory[iu + 1]
        } else {
            t0
        };
        for j in axes(v) {
            let traj = &trajectory[j];
            let (mut d0, d1);
            if relative {
                d0 = traj[iu] * dir[j];
                d1 = if i < npoints - 1 {
                    traj[iu + 1] * dir[j]
                } else {
                    d0
                };
            } else {
                d0 = traj[iu + 1] * dir[j] - traj[iu] * dir[j];
                d1 = if i < npoints - 1 {
                    traj[iu + 2] * dir[j] - traj[iu + 1] * dir[j]
                } else {
                    d0
                };
            }
            let mut traj_vel = (d0 + d1) / (t0 + t1);
            if v.buildMoveAxis.at(j).get() == 0 {
                d0 = 0.0;
                traj_vel = 0.0;
            }
            if j == 0 {
                out += &format!("{}", CFloat(time_trajectory[iu]));
            }
            out += &format!(", {}, {}", CFloat(d0), CFloat(traj_vel));
            if j + 1 == axes(v).end {
                out += "\n";
            }
        }
    }

    // Its final velocity must be 0.
    out += &format!("{}", CFloat(post_time_max));
    for j in axes(v) {
        out += &format!(
            ", {}, {}",
            CFloat(v.postDistance.at(j).get() * dir[j]),
            CFloat(0.0)
        );
    }
    out += "\n";
    // C ignores what fprintf and fclose return.
    let _ = traj_file.write_all(out.as_bytes());
    drop(traj_file);

    // C leaks ftpSocket on each of these returns; it closes here.
    let mut ftp_socket =
        match ftp::ftp_connect(&text(&v.xpsAddress), &text(&v.userName), &text(&v.password)) {
            Ok(sock) => sock,
            Err(status) => {
                print_message("Error calling ftpConnect, status=%d\n", &[status.into()]);
                return;
            }
        };
    let status = ftp::ftp_change_dir(&mut ftp_socket, TRAJECTORY_DIRECTORY);
    if status != 0 {
        print_message("Error calling ftpChangeDir, status=%d\n", &[status.into()]);
        return;
    }
    let status = ftp::ftp_store_file(&mut ftp_socket, &file_name);
    if status != 0 {
        print_message("Error calling ftpStoreFile, status=%d\n", &[status.into()]);
        return;
    }
    let status = ftp::ftp_disconnect(ftp_socket);
    if status != 0 {
        print_message("Error calling ftpDisconnect, status=%d\n", &[status.into()]);
        return;
    }

    let group = text(&v.groupName);
    if debug {
        print_message(
            "XPS_trajectoryScan: buildAndVerify: calling MultipleAxesPVTVerification(%d, %s, %s)\n",
            &[
                poll_socket.into(),
                group.as_str().into(),
                file_name.as_str().into(),
            ],
        );
    }
    let status = c_int(poll.multiple_axes_pvt_verification(&group, &file_name));
    v.buildStatus.set(STATUS_FAILURE);
    if status == 0 {
        strcpy(&v.buildMessage, " ");
        v.buildStatus.set(STATUS_SUCCESS);
    } else if status == -69 {
        strcpy(&v.buildMessage, "Acceleration Too High");
    } else if status == -68 {
        strcpy(&v.buildMessage, "Velocity Too High");
    } else if status == -70 {
        strcpy(&v.buildMessage, "Final Velocity Non Zero");
    } else if status == -75 {
        strcpy(&v.buildMessage, "Negative or Null Delta Time");
    } else {
        sprintf(
            &v.buildMessage,
            "Unknown trajectory verify error=%d",
            &[status.into()],
        );
    }

    for j in axes(v) {
        let name = axis_name(v, j);
        match poll.multiple_axes_pvt_verification_result_get(&name) {
            Ok((_, min_pos, max_pos, max_velocity, max_acceleration)) => {
                v.motorMinPos.at(j).set(min_pos);
                v.motorMaxPos.at(j).set(max_pos);
                v.motorMVA[j].set(max_velocity);
                v.motorMAA[j].set(max_acceleration);
            }
            Err(e) => print_message(
                "Error performing MultipleAxesPVTVerificationResultGet for axis %s, status=%d\n",
                &[name.as_str().into(), e.code().into()],
            ),
        }
    }
}

/// C `currentElement`: the element the running trajectory is in, 0 when
/// the call fails.
fn current_element(v: &seqg::seqg_vars) -> i32 {
    let (mut file_name, mut number) = (String::new(), 0);
    let status = c_int(socket(v.pollSocket.get()).multiple_axes_pvt_parameters_get(
        &text(&v.groupName),
        &mut file_name,
        &mut number,
    ));
    if status != 0 {
        print_message(
            "Error performing MultipleAxesPVTParametersGet, status=%d\n",
            &[status.into()],
        );
    }
    number
}

/// C `readGathering`: the setpoint and actual position of each axis at
/// each pulse, as `motorReadbacks` in user units and `motorError`.
fn read_gathering(v: &seqg::seqg_vars) {
    let debug = v.debugLevel.get() > 0;
    let poll = socket(v.pollSocket.get());
    if debug {
        print_message("XPS_trajectoryScan: readGathering: entry\n", &[]);
    }
    let current_samples = match poll.gathering_current_number_get() {
        Ok((current, _)) => current,
        Err(e) => {
            print_message(
                "Error calling GatherCurrentNumberGet, status=%d\n",
                &[e.code().into()],
            );
            return;
        }
    };
    if current_samples != v.npulses.get() {
        print_message(
            "readGathering: error, npulses=%d, currentSamples=%d\n",
            &[v.npulses.get().into(), current_samples.into()],
        );
        return;
    }
    let current_samples = current_samples.min(MAX_PULSES);
    let mut num_read = 0;
    'done: while num_read < current_samples {
        // All that is left, halved until the XPS can send it.
        let mut buffer = String::new();
        let mut status = -1;
        let mut num_in_buffer = current_samples - num_read;
        while status != 0 && num_in_buffer > 0 {
            status = match poll.gathering_data_multiple_lines_get(num_read, num_in_buffer) {
                Ok(lines) => {
                    buffer = lines;
                    0
                }
                Err(e) => e.code(),
            };
            if debug {
                print_message(
                    "readGathering: GatheringDataMultipleLinesGet, status=%d, numInBuffer=%d\n",
                    &[status.into(), num_in_buffer.into()],
                );
            }
            if status != 0 {
                num_in_buffer /= 2;
            }
        }
        if num_in_buffer == 0 {
            print_message("Error reading gathering data, numInBuffer = 0\n", &[]);
            break 'done;
        }
        let mut bptr = buffer.as_bytes();
        for _ in 0..num_in_buffer {
            // C ends the line at its \n, and steps past a reply short of
            // lines into memory not its own; an empty rest fails the scan.
            let (line, rest) = match bptr.iter().position(|&c| c == b'\n') {
                Some(n) => (&bptr[..n], &bptr[n + 1..]),
                None => (bptr, &[][..]),
            };
            let mut field = line;
            for j in axes(v) {
                let (setpoint, actual, num_chars) =
                    (SeqVar::new(0.0), SeqVar::new(0.0), SeqVar::new(0));
                let nitems = sscanf(field, "%lf;%lf%n", &[&setpoint, &actual, &num_chars]);
                if nitems != NUM_GATHERING_ITEMS {
                    print_message(
                        "Error reading Gathering.dat file, nitems=%d, should be %d\n",
                        &[nitems.into(), NUM_GATHERING_ITEMS.into()],
                    );
                    break 'done;
                }
                field = field
                    .get(num_chars.get() as usize + 1..)
                    .unwrap_or_default();
                let at = num_read as usize;
                v.motorError[j].at(at).set(actual.get() - setpoint.get());
                v.motorReadbacks[j]
                    .at(at)
                    .set(actual.get() * dir(v, j) + v.epicsMotorOff[j].get());
            }
            num_read += 1;
            v.nactual.set(num_read);
            bptr = rest;
        }
    }
    if debug {
        print_message("XPS_trajectoryScan: readGathering: exit\n", &[]);
    }
}

/// C `trajectoryAbort`, on the abort socket; its 0 no caller reads.
fn trajectory_abort(v: &seqg::seqg_vars) {
    let status = c_int(socket(v.abortSocket.get()).group_move_abort(&text(&v.groupName)));
    if status != 0 {
        print_message(
            "Error performing GroupMoveAbort, status=%d\n",
            &[status.into()],
        );
    }
}

macro_rules! seqg_escaped {
    (r#"#include <string.h>"#) => {};
    (r#"#include <stdio.h>"#) => {};
    (r#"#include <math.h>"#) => {};
    (r#"#include <stdarg.h>"#) => {};
    (r#"#define epicsExportSharedSymbols"#) => {};
    (r#"#include <shareLib.h>"#) => {};
    (r#"#include "XPS_C8_drivers.h""#) => {};
    (r#"#include "XPS_C8_errors.h""#) => {};
    (r#"#include "Socket.h""#) => {};
    (r#"#include "xps_ftp.h""#) => {};
    (r#"static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);"#) => {};
    (r#"static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int waitMotors(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int getSocket(SS_ID ssId, struct UserVar *pVar, double timeout);"#) => {};
    (r#"static void trajectoryExecute(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static void buildAndVerify(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int currentElement(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int getGroupStatus(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static void readGathering(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar);"#) => {};
    (r#"static void printMessage(const char *pformat, ...);"#) => {};
    (r#"static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos) { int status; int j; int dir; status = GroupPositionCurrentGet(pVar->positionSocket, pVar->groupName,pVar->numAxes,pos); if (status != 0) { printMessage("Error performing GroupPositionCurrentGet%i\n", status); return(status); } for (j=0; j<pVar->numAxes; j++) { if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1; pos[j] = pos[j]*dir + pVar->epicsMotorOff[j]; } return(status); } static int getMotorMoving(SS_ID ssId, struct UserVar *pVar) { int status; int moving=0; int groupStatus=0; status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus); if (status != 0) printMessage("Error performing GroupStatusGet %i\n",status); if (groupStatus > 42) moving = 1; return(moving); } static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar) { int j; int result=0, mask=0x01; for (j=0; j<pVar->numAxes; j++) { seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0); if (pVar->epicsMotorDone[j] == 0) result |= mask; mask = mask << 1; } return(result); } static int waitMotors(SS_ID ssId, struct UserVar *pVar) { int j; while(getMotorMoving(ssId, pVar)) { getMotorPositions(ssId, pVar, pVar->motorCurrent); for (j=0; j<pVar->numAxes; j++) { seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } epicsThreadSleep(0.1); } getMotorPositions(ssId, pVar, pVar->motorCurrent); for (j=0; j<pVar->numAxes; j++) { seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } return(0); } static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar) { int j; while(getEpicsMotorMoving(ssId, pVar)) { for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } epicsThreadSleep(0.1); } for (j=0; j<pVar->numAxes; j++) { pVar->motorCurrent[j] = pVar->epicsMotorPos[j]; seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0); } return(0); } static int getSocket(SS_ID ssId, struct UserVar *pVar, double timeout) { int sock = 0; sock = TCP_ConnectToServer(pVar->xpsAddress, pVar->xpsPort, timeout); if (sock < 0) printMessage("Error TCP_ConnectToServer, status=%d\n",sock); return (sock); } static void trajectoryExecute(SS_ID ssId, struct UserVar *pVar) { int status; int j; char buffer[60 * 2 * 8]; int eventId; if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " entry\n"); } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling GatheringReset(%d)\n", pVar->pollSocket); } status = GatheringReset(pVar->pollSocket); if (status != 0) { printMessage("Error performing GatheringReset, status=%d\n",status); return; } strcpy(buffer, ""); for (j=0; j<pVar->numAxes; j++) { strcat (buffer, pVar->axisName[j]); strcat (buffer, ".SetpointPosition;"); strcat (buffer, pVar->axisName[j]); strcat (buffer, ".CurrentPosition;"); } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling GatheringConfigurationSet(%d, %d, %s)\n", pVar->pollSocket, pVar->numAxes*2, buffer); } status = GatheringConfigurationSet(pVar->pollSocket, pVar->numAxes*2, buffer); if (status != 0) printMessage("Error performing GatheringConfigurationSet, status=%d, buffer=%p\n", status, buffer); if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling MultipleAxesPVTPulseOutputSet(%d, %s, %d, %d, %f)\n", pVar->pollSocket, pVar->groupName, pVar->startPulses+1, pVar->endPulses+1, pVar->pulsePeriod); } status = MultipleAxesPVTPulseOutputSet(pVar->pollSocket, pVar->groupName, pVar->startPulses+1, pVar->endPulses+1, pVar->pulsePeriod); sprintf(buffer, "Always;%s.PVT.TrajectoryPulse", pVar->groupName); if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling EventExtendedConfigurationTriggerSet(%d, %d, %s, %s, %s, %s, %s)\n", pVar->pollSocket, 2, buffer, "", "", "", ""); } status = EventExtendedConfigurationTriggerSet(pVar->pollSocket, 2, buffer, "", "", "", ""); if (status != 0) { printMessage("Error performing EventExtendedConfigurationTriggerSet, status=%d, buffer=%s\n", status, buffer); return; } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling EventExtendedConfigurationActionSet(%d, %d, %s, %s, %s, %s, %s)\n", pVar->pollSocket, 1, "GatheringOneData", "", "", "", ""); } status = EventExtendedConfigurationActionSet(pVar->pollSocket, 1, "GatheringOneData", "", "", "", ""); if (status != 0) { printMessage("Error performing EventExtendedConfigurationActionSet, status=%d\n", status); return; } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling EventExtendedStart(%d, %p)\n", pVar->pollSocket, &eventId); } status= EventExtendedStart(pVar->pollSocket, &eventId); if (status != 0) { printMessage("Error performing EventExtendedStart, status=%d\n",status); return; } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling MultipleAxesPVTExecution(%d, %s, %s, %d)\n", pVar->driveSocket, pVar->groupName, pVar->trajectoryFile, 1); } status = MultipleAxesPVTExecution(pVar->driveSocket, pVar->groupName, pVar->trajectoryFile, 1); if ((status != 0) && (status != -27)) printMessage("Error performing MultipleAxesPVTExecution, status=%d\n", status); if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling EventExtendedRemove(%d, %d)\n", pVar->pollSocket, eventId); } status = EventExtendedRemove(pVar->pollSocket, eventId); if (status != 0) { printMessage("Error performing ExtendedEventRemove, status=%d\n",status); return; } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: trajectoryExecute:" " calling GatheringStop(%d)\n", pVar->pollSocket); } status = GatheringStop(pVar->pollSocket); if ((status != 0) && (status != -30)) printMessage("Error performing GatheringStop, status=%d\n", status); return; } static void buildAndVerify(SS_ID ssId, struct UserVar *pVar) { FILE *trajFile; int i, j, status; int npoints; double trajVel; double D0, D1, T0, T1; SOCKET ftpSocket; char fileName[100]; double distance; double maxVelocity[8], maxAcceleration[8]; double minJerkTime[8], maxJerkTime[8]; double preTimeMax, postTimeMax; double preVelocity[8], postVelocity[8]; double time; int dir[8]; if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: buildAndVerify:" " entry\n"); } preTimeMax = 0.; postTimeMax = 0.; for (j=0; j<pVar->numAxes; j++) { preVelocity[j] = 0.; postVelocity[j] = 0.; } for (j=0; j<pVar->numAxes; j++) { if (!pVar->buildMoveAxis[j]) continue; status = PositionerSGammaParametersGet(pVar->pollSocket, pVar->axisName[j], &maxVelocity[j], &maxAcceleration[j], &minJerkTime[j], &maxJerkTime[j]); if (status != 0) { printMessage("Error calling positionerSGammaParametersSet, status=%d\n", status); maxAcceleration[j] = 0.; } maxAcceleration[j] = 0.9 * maxAcceleration[j]; if (pVar->buildMode == 0) { distance = pVar->motorTrajectory[j][0]; } else { distance = pVar->motorTrajectory[j][1] - pVar->motorTrajectory[j][0]; } preVelocity[j] = distance/pVar->timeTrajectory[0]; time = fabs(preVelocity[j]) / maxAcceleration[j]; if (time > preTimeMax) preTimeMax = time; if (pVar->buildMode == 0) { distance = pVar->motorTrajectory[j][pVar->buildElements-1]; } else { distance = pVar->motorTrajectory[j][pVar->buildElements-1] - pVar->motorTrajectory[j][pVar->buildElements-2]; } postVelocity[j] = distance/pVar->timeTrajectory[pVar->buildElements-1]; time = fabs(postVelocity[j]) / maxAcceleration[j]; if (time > postTimeMax) postTimeMax = time; } for (j=0; j<pVar->numAxes; j++) { pVar->preDistance[j] = 0.5 * preVelocity[j] * preTimeMax; pVar->postDistance[j] = 0.5 * postVelocity[j] * postTimeMax; } trajFile = fopen (pVar->trajectoryFile, "w"); if (trajFile == NULL) { printMessage("Error calling fopen, file=%s\n", pVar->trajectoryFile); return; } for (j=0; j<pVar->numAxes; j++) { if (pVar->epicsMotorDir[j] == 0) dir[j]=1; else dir[j]=-1; } fprintf(trajFile,"%f", preTimeMax); for (j=0; j<pVar->numAxes; j++) fprintf(trajFile,", %f, %f", pVar->preDistance[j]*dir[j], preVelocity[j]*dir[j]); fprintf(trajFile,"\n"); if (pVar->buildMode == 0) npoints = pVar->buildElements; else npoints = pVar->buildElements-1; for (i=0; i<npoints; i++) { T0 = pVar->timeTrajectory[i]; if (i < npoints-1) T1 = pVar->timeTrajectory[i+1]; else T1 = T0; for (j=0; j<pVar->numAxes; j++) { if (pVar->buildMode == 0) { D0 = pVar->motorTrajectory[j][i] * dir[j]; if (i < npoints-1) D1 = pVar->motorTrajectory[j][i+1] * dir[j]; else D1 = D0; } else { D0 = pVar->motorTrajectory[j][i+1] *dir[j] - pVar->motorTrajectory[j][i] * dir[j]; if (i < npoints-1) D1 = pVar->motorTrajectory[j][i+2] * dir[j] - pVar->motorTrajectory[j][i+1] * dir[j]; else D1 = D0; } trajVel = ((D0 + D1) / (T0 + T1)); if (!(pVar->buildMoveAxis[j])) { D0 = 0.0; trajVel = 0.0; } if (j == 0) fprintf(trajFile,"%f", pVar->timeTrajectory[i]); fprintf(trajFile,", %f, %f",D0,trajVel); if (j == (pVar->numAxes-1)) fprintf(trajFile,"\n"); } } fprintf(trajFile,"%f", postTimeMax); for (j=0; j<pVar->numAxes; j++) fprintf(trajFile,", %f, %f", pVar->postDistance[j]*dir[j], 0.); fprintf(trajFile,"\n"); fclose (trajFile); status = ftpConnect(pVar->xpsAddress, pVar->userName, pVar->password, &ftpSocket); if (status != 0) { printMessage("Error calling ftpConnect, status=%d\n", status); return; } status = ftpChangeDir(ftpSocket, "/Admin/public/Trajectories"); if (status != 0) { printMessage("Error calling ftpChangeDir, status=%d\n", status); return; } status = ftpStoreFile(ftpSocket, pVar->trajectoryFile); if (status != 0) { printMessage("Error calling ftpStoreFile, status=%d\n", status); return; } status = ftpDisconnect(ftpSocket); if (status != 0) { printMessage("Error calling ftpDisconnect, status=%d\n", status); return; } if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: buildAndVerify:" " calling MultipleAxesPVTVerification(%d, %s, %s)\n", pVar->pollSocket, pVar->groupName, pVar->trajectoryFile); } status = MultipleAxesPVTVerification(pVar->pollSocket, pVar->groupName, pVar->trajectoryFile); pVar->buildStatus = 2; if (status == 0) { strcpy(pVar->buildMessage, " "); pVar->buildStatus = 1; } else if (status == -69) strcpy(pVar->buildMessage, "Acceleration Too High"); else if (status == -68) strcpy(pVar->buildMessage, "Velocity Too High"); else if (status == -70) strcpy(pVar->buildMessage, "Final Velocity Non Zero"); else if (status == -75) strcpy(pVar->buildMessage, "Negative or Null Delta Time"); else sprintf(pVar->buildMessage, "Unknown trajectory verify error=%d", status); if (status == 0){ pVar->buildStatus = 1; } if (1) { for (j=0; j<pVar->numAxes; j++) { status = MultipleAxesPVTVerificationResultGet(pVar->pollSocket, pVar->axisName[j], fileName, &pVar->motorMinPos[j], &pVar->motorMaxPos[j], &pVar->motorMVA[j], &pVar->motorMAA[j]); if (status != 0) { printMessage("Error performing MultipleAxesPVTVerificationResultGet for axis %s, status=%d\n", pVar->axisName[j], status); } } } else { for (j=0; j<pVar->numAxes; j++) { pVar->motorMVA[j] = 0; pVar->motorMAA[j] = 0; } } return; } static int currentElement(SS_ID ssId, struct UserVar *pVar) { int status; int number=0; char fileName[100]; status = MultipleAxesPVTParametersGet(pVar->pollSocket, pVar->groupName, fileName, &number); if (status != 0) printMessage("Error performing MultipleAxesPVTParametersGet, status=%d\n", status); return (number); } static int getGroupStatus(SS_ID ssId, struct UserVar *pVar) { int status; int groupStatus=0; status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus); if (status != 0) printMessage("Error performing GroupStatusGet, status=%d\n", status); return(groupStatus); } static void readGathering(SS_ID ssId, struct UserVar *pVar) { char *buffer, *bptr, *tptr; int currentSamples, maxSamples; double setpointPosition, actualPosition; int status; int dir; int i, j; int nitems; int numRead, numInBuffer, numChars; if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: readGathering:" " entry\n"); } status = GatheringCurrentNumberGet(pVar->pollSocket, &currentSamples, &maxSamples); if (status != 0) { printMessage("Error calling GatherCurrentNumberGet, status=%d\n", status); return; } if (currentSamples != pVar->npulses) { printMessage("readGathering: error, npulses=%d, currentSamples=%d\n", pVar->npulses, currentSamples); return; } if (currentSamples > 2000) currentSamples = 2000; buffer = (char *)calloc(65536, sizeof(char)); numInBuffer = 0; for (numRead=0; numRead<currentSamples;) { status = -1; numInBuffer = currentSamples - numRead; while (status && (numInBuffer > 0)) { status = GatheringDataMultipleLinesGet(pVar->pollSocket, numRead, numInBuffer, buffer); if (pVar->debugLevel > 0) { printMessage("readGathering: GatheringDataMultipleLinesGet, status=%d, numInBuffer=%d\n", status, numInBuffer); } if (status) numInBuffer /= 2; } if (numInBuffer == 0) { printMessage("Error reading gathering data, numInBuffer = 0\n"); goto done; } bptr = buffer; for (i=0; i<numInBuffer; i++) { tptr = strstr(bptr, "\n"); if (tptr) *tptr = 0; for (j=0; j<pVar->numAxes; j++) { nitems = sscanf(bptr, "%lf;%lf%n", &setpointPosition, &actualPosition, &numChars); bptr += numChars+1; if (nitems != 2) { printMessage("Error reading Gathering.dat file, nitems=%d, should be %d\n", nitems, 2); goto done; } pVar->motorError[j][numRead] = actualPosition - setpointPosition; if (pVar->epicsMotorDir[j] == 0) dir=1; else dir=-1; pVar->motorReadbacks[j][numRead] = actualPosition * dir + pVar->epicsMotorOff[j]; } numRead++; pVar->nactual = numRead; bptr = tptr + 1; } } done: free(buffer); if (pVar->debugLevel > 0) { printMessage("XPS_trajectoryScan: readGathering:" " exit\n"); } return; } static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar) { int status; status = GroupMoveAbort(pVar->abortSocket,pVar->groupName); if (status != 0) printMessage("Error performing GroupMoveAbort, status=%d\n", status); return (0); } static void printMessage(const char *pformat, ...) { va_list pvar; FILE *fp = stdout; epicsTimeStamp now; char nowText[40]; epicsTimeGetCurrent(&now); nowText[0] = 0; epicsTimeToStrftime(nowText,sizeof(nowText), "%Y/%m/%d %H:%M:%S.%03f",&now); fprintf(fp,"%s ",nowText); va_start(pvar, pformat); vfprintf(fp,pformat,pvar); va_end(pvar); fflush(fp); }"#) => {};
    ($env:ident, r#"pVar->pollSocket = getSocket(ssId, pVar, 1.0);"#) => {
        $env.var().pollSocket.set(get_socket($env, 1.0))
    };
    ($env:ident, r#"pVar->driveSocket = getSocket(ssId, pVar, 100000.);"#) => {
        $env.var().driveSocket.set(get_socket($env, 100000.))
    };
    ($env:ident, r#"pVar->abortSocket = getSocket(ssId, pVar, 10.);"#) => {
        $env.var().abortSocket.set(get_socket($env, 10.))
    };
    ($env:ident, r#"pVar->positionSocket = getSocket(ssId, pVar, 1.0);"#) => {
        $env.var().positionSocket.set(get_socket($env, 1.0))
    };
    ($env:ident, r#"buildAndVerify(ssId, pVar);"#) => {
        build_and_verify($env.var())
    };
    ($env:ident, r#"waitMotors(ssId, pVar);"#) => {
        wait_motors($env)
    };
    ($env:ident, r#"waitEpicsMotors(ssId, pVar);"#) => {
        wait_epics_motors($env)
    };
    ($env:ident, r#"pVar->xpsStatus = getGroupStatus(ssId, pVar);"#) => {
        $env.var().xpsStatus.set(get_group_status($env.var()))
    };
    ($env:ident, r#"getMotorPositions(ssId, pVar, pVar->motorCurrent);"#) => {
        get_motor_positions($env.var())
    };
    ($env:ident, r#"pVar->ncomplete = currentElement(ssId, pVar);"#) => {
        $env.var().ncomplete.set(current_element($env.var()))
    };
    ($env:ident, r#"readGathering(ssId, pVar);"#) => {
        read_gathering($env.var())
    };
    ($env:ident, r#"trajectoryAbort(ssId, pVar);"#) => {
        trajectory_abort($env.var())
    };
    ($env:ident, r#"trajectoryExecute(ssId, pVar);"#) => {
        trajectory_execute($env.var())
    };
}

epics_seq_rs::include_program!("XPS_trajectoryScan");

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::Ordering;

    use epics_seq_rs::{ProgOptions, ProgramVars, VarModel};

    use super::super::xps_c8::SOCKETS;
    use super::*;
    use crate::xps::SocketMode;
    use crate::xps::fake::FakeLine;

    /// XPS_trajectoryScan.st:1, with `option +r;` (:34).
    #[test]
    fn the_program_is_the_st_one() {
        assert_eq!(XPS_trajectoryScan.validate(), Ok(()));
        assert_eq!(
            XPS_trajectoryScan.params,
            "P=13BMC:,R=traj1,IPADDR=164.54.160.34,PORT=5001,\
             USERNAME=Administrator,PASSWORD=Administrator,\
             M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8,\
             GROUP=g1,\
             P1=p1,P2=p2,P3=p3,P4=p4,P5=p5,P6=p6,P7=p7,P8=p8"
        );
        assert_eq!(
            XPS_trajectoryScan.options,
            ProgOptions::new(ProgOptions::CONN | ProgOptions::NEWEF)
        );
        assert!(matches!(XPS_trajectoryScan.vars, VarModel::Reentrant));
    }

    /// The lists XPS_C8_drivers.cpp cuts: `" \t;"` separators, `n` names,
    /// the first 1023 bytes and each name to 99.
    #[test]
    fn lists_are_cut_as_strtok_r() {
        assert_eq!(
            split_list("Always;G1.PVT.TrajectoryPulse", 2),
            ["Always", "G1.PVT.TrajectoryPulse"]
        );
        assert_eq!(split_list(";a \t b;;c;", 2), ["a", "b"]);
        assert_eq!(split_list(&"x".repeat(150), 1), ["x".repeat(99)]);
        let list = format!("{};tail", "y".repeat(1021));
        assert_eq!(split_list(&list, 2), ["y".repeat(99), "t".to_string()]);
        assert!(split_list("a", 0).is_empty());
    }

    /// A program's variables for one axis `G9.P1` of group `G9`, on fake
    /// XPS sockets `base` (poll), `base + 1` (drive) and `base + 2`
    /// (position) of one line.
    fn vars(base: i32) -> (seqg::seqg_vars, FakeLine) {
        let v = seqg::seqg_vars::zeroed();
        let line = FakeLine::new(&format!("xps_traj_{base}"), "");
        let mut sockets = SOCKETS.lock().unwrap();
        for i in 0..3 {
            sockets.insert(base + i, Arc::new(line.socket(SocketMode::Query)));
        }
        v.pollSocket.set(base);
        v.driveSocket.set(base + 1);
        v.positionSocket.set(base + 2);
        v.numAxes.set(1);
        strcpy(&v.groupName, "G9");
        v.axisName.at(0).set(malloc(100));
        strcpy(v.axisName.at(0).get(), "G9.P1");
        (v, line)
    }

    /// getMotorPositions converts XPS dial positions to user units.
    #[test]
    fn motor_positions_are_user_units() {
        let (v, line) = vars(910);
        v.epicsMotorDir[0].set(1.0);
        v.epicsMotorOff[0].set(10.0);
        assert_eq!(get_motor_positions(&v), 0);
        assert_eq!(v.motorCurrent[0].get(), 8.5);
        assert_eq!(line.take_log()[0], "GroupPositionCurrentGet (G9,double *)");
    }

    /// getMotorPositions leaves the positions as they were when the read
    /// fails.
    #[test]
    fn a_failed_position_read_leaves_the_positions() {
        let (v, line) = vars(915);
        *line.fail.lock().unwrap() = "GroupPositionCurrentGet";
        v.epicsMotorOff[0].set(10.0);
        v.motorCurrent[0].set(8.5);
        assert_eq!(get_motor_positions(&v), -3);
        assert_eq!(v.motorCurrent[0].get(), 8.5);
    }

    /// readGathering: each line's setpoint and actual position as the
    /// following error and the user-unit readback.
    #[test]
    fn gathering_is_read_as_readbacks_and_errors() {
        let (v, line) = vars(920);
        v.npulses.set(2);
        v.epicsMotorOff[0].set(1.0);
        read_gathering(&v);
        assert_eq!(v.nactual.get(), 2);
        assert_eq!(v.motorError[0].get()[..2], [0.5, 0.5]);
        assert_eq!(v.motorReadbacks[0].get()[..2], [2.5, 3.5]);
        assert_eq!(
            line.take_log(),
            [
                "GatheringCurrentNumberGet (int *,int *)",
                "GatheringDataMultipleLinesGet (0,2,char *)"
            ]
        );
        // A count other than npulses reads nothing.
        v.npulses.set(3);
        v.nactual.set(0);
        read_gathering(&v);
        assert_eq!(v.nactual.get(), 0);
    }

    /// trajectoryExecute: gathering and the pulse event on the poll
    /// socket around the trajectory run on the drive socket.
    #[test]
    fn execute_runs_the_trajectory_between_gathering_calls() {
        let (v, line) = vars(930);
        v.startPulses.set(1);
        v.endPulses.set(4);
        v.pulsePeriod.set(0.25);
        strcpy(&v.trajectoryFile, "traj.trj");
        trajectory_execute(&v);
        let log: Vec<String> = line
            .take_log()
            .iter()
            .map(|c| c.split(" (").next().unwrap().to_string())
            .collect();
        assert_eq!(
            log,
            [
                "GatheringReset",
                "GatheringConfigurationSet",
                "MultipleAxesPVTPulseOutputSet",
                "EventExtendedConfigurationTriggerSet",
                "EventExtendedConfigurationActionSet",
                "EventExtendedStart",
                "MultipleAxesPVTExecution",
                "EventExtendedRemove",
                "GatheringStop",
            ]
        );
        // A failed reset stops it there.
        *line.fail.lock().unwrap() = "GatheringReset";
        trajectory_execute(&v);
        assert_eq!(line.take_log().len(), 1);
    }

    /// buildAndVerify: a trajectory file that cannot be made ends the
    /// build there.
    #[test]
    fn an_unopenable_trajectory_file_ends_the_build() {
        let (v, line) = vars(945);
        v.buildElements.set(2);
        v.buildMode.set(MOVE_MODE_RELATIVE);
        v.buildMoveAxis.at(0).set(1);
        v.timeTrajectory
            .modify(|t| t[..2].copy_from_slice(&[1.0, 1.0]));
        strcpy(&v.trajectoryFile, "/nonexistent/xps.trj");
        build_and_verify(&v);
        assert_eq!(v.buildStatus.get(), 0);
        assert_eq!(
            line.take_log(),
            ["PositionerSGammaParametersGet (G9.P1,double *,double *,double *,double *)"]
        );
    }

    /// buildAndVerify: the trajectory file C writes, sent over FTP, then
    /// verified and its limits read.
    #[test]
    fn build_writes_sends_and_verifies_the_file() {
        let (v, line) = vars(940);
        v.buildElements.set(2);
        v.buildMode.set(MOVE_MODE_RELATIVE);
        v.buildMoveAxis.at(0).set(1);
        v.timeTrajectory.set({
            let mut t = [0.0; 2000];
            t[..2].copy_from_slice(&[1.0, 1.0]);
            t
        });
        v.motorTrajectory[0].modify(|t| t[..2].copy_from_slice(&[1.0, 2.0]));
        let file = std::env::temp_dir().join(format!("xps{}.trj", std::process::id()));
        let file = file.to_str().unwrap();
        strcpy(&v.trajectoryFile, file);
        strcpy(&v.xpsAddress, "127.0.0.1");
        strcpy(&v.userName, "Administrator");
        strcpy(&v.password, "pw");
        let (addr, ftp_log) = ftp::tests::server("331 password\r\n");
        ftp::FTP_PORT.store(addr.port(), Ordering::Relaxed);

        build_and_verify(&v);

        // 0.9 * the 40 PositionerSGammaParametersGet reads.
        let contents = "0.027778, 0.013889, 1.000000\n\
                        1.000000, 1.000000, 1.500000\n\
                        1.000000, 2.000000, 2.000000\n\
                        0.055556, 0.055556, 0.000000\n";
        assert_eq!(std::fs::read_to_string(file).unwrap(), contents);
        let ftp_log: Vec<String> = ftp_log.iter().collect();
        assert_eq!(ftp_log[4], "CWD /Admin/public/Trajectories");
        assert_eq!(
            ftp_log[6..],
            [format!("STOR {file}"), contents.into(), "CLOSE".into()]
        );
        assert_eq!(v.buildStatus.get(), STATUS_SUCCESS);
        assert_eq!(text(&v.buildMessage), " ");
        assert_eq!(
            (v.motorMinPos.get()[0], v.motorMaxPos.get()[0]),
            (-0.5, 2.5)
        );
        assert_eq!((v.motorMVA[0].get(), v.motorMAA[0].get()), (1.2, 3.4));
        let log: Vec<String> = line
            .take_log()
            .iter()
            .map(|c| c.split(" (").next().unwrap().to_string())
            .collect();
        assert_eq!(
            log,
            [
                "PositionerSGammaParametersGet",
                "MultipleAxesPVTVerification",
                "MultipleAxesPVTVerificationResultGet"
            ]
        );
        let _ = std::fs::remove_file(file);
    }

    /// buildAndVerify: an axis moved after state build bounded a relative
    /// trajectory of no element as moving none is not moved in it.
    #[test]
    fn an_axis_moved_after_the_bound_is_not_built() {
        let (v, line) = vars(935);
        // The build's snapshot, zeroed, moves no axis.
        v.buildElements.set(0);
        v.buildMode.set(MOVE_MODE_RELATIVE);
        v.moveAxis[0].set(1);
        let file = std::env::temp_dir().join(format!("xpsMoved{}.trj", std::process::id()));
        let file = file.to_str().unwrap();
        strcpy(&v.trajectoryFile, file);
        strcpy(&v.xpsAddress, "127.0.0.1");
        let (addr, _ftp_log) = ftp::tests::server("331 password\r\n");
        ftp::FTP_PORT.store(addr.port(), Ordering::Relaxed);

        build_and_verify(&v);

        assert_eq!(
            std::fs::read_to_string(file).unwrap(),
            "0.000000, 0.000000, 0.000000\n".repeat(2)
        );
        let log: Vec<String> = line
            .take_log()
            .iter()
            .map(|c| c.split(" (").next().unwrap().to_string())
            .collect();
        assert_eq!(
            log,
            [
                "MultipleAxesPVTVerification",
                "MultipleAxesPVTVerificationResultGet"
            ]
        );
        let _ = std::fs::remove_file(file);
    }

    /// currentElement and getGroupStatus read the poll socket.
    #[test]
    fn element_and_group_status_are_read() {
        let (v, line) = vars(950);
        assert_eq!(current_element(&v), 42);
        *line.group_status.lock().unwrap() = 45;
        assert_eq!(get_group_status(&v), 45);
        assert_eq!(get_motor_moving(&v), 1);
        *line.group_status.lock().unwrap() = 12;
        assert_eq!(get_motor_moving(&v), 0);
    }

    /// The program against trajectoryScan.db, as xps-ioc loads it, a motor
    /// record `T:m1`, and an XPS on 127.0.0.1 that runs a trajectory until
    /// `GroupMoveAbort`.
    mod program {
        use std::collections::HashMap;
        use std::io::{Read, Write};
        use std::net::{TcpListener, TcpStream};
        use std::sync::{Arc, Condvar, Mutex};
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

        use super::*;
        use crate::xps::XpsSocket;

        const DB: &str = include_str!("../../../../../iocs/motor/xps-ioc/db/trajectoryScan.db");

        /// m1 on a simulated axis at dial 0, user offset `$(OFF)`.
        const MOTOR: &str = r#"record(motor, "T:m1") { field(DTYP, "asynMotor")
            field(OUT, "@asyn($(PORT)m1,0)") field(MRES, "0.001") field(VELO, "1")
            field(VMAX, "1") field(ACCL, "0.2") field(DHLM, "100") field(DLLM, "-100")
            field(OFF, "$(OFF)") }"#;

        /// What the fake XPS has seen and holds.
        #[derive(Default)]
        struct Xps {
            log: Vec<String>,
            /// A MultipleAxesPVTExecution is running.
            running: bool,
            aborted: bool,
            /// The running trajectory is to end as done.
            released: bool,
            /// The lines gathering holds.
            gathered: usize,
        }

        type Shared = Arc<(Mutex<Xps>, Condvar)>;

        fn reply(xps: &Shared, cmd: &str) -> String {
            let (state, cv) = &**xps;
            let name = cmd.split(" (").next().unwrap_or_default().to_string();
            let mut x = state.lock().unwrap();
            x.log.push(name.clone());
            let lines;
            let values = match name.as_str() {
                "GroupStatusGet" if x.running => "0,45",
                "GroupStatusGet" => "0,12",
                "GroupPositionCurrentGet" => "0,0",
                "PositionerUserTravelLimitsGet" => "0,-5,5",
                "PositionerSGammaParametersGet" => "0,10,40,0.02,0.02",
                "MultipleAxesPVTVerificationResultGet" => "0,traj.trj,-0.5,2.5,1.2,3.4",
                "MultipleAxesPVTParametersGet" => "0,traj.trj,1",
                "EventExtendedStart" => "0,1",
                "GatheringCurrentNumberGet" => {
                    lines = format!("0,{},100000", x.gathered);
                    &lines
                }
                "GatheringDataMultipleLinesGet" => {
                    // (indexPoint,numberOfLines,char *)
                    let n: usize = cmd.split([',', '(']).nth(2).unwrap().parse().unwrap();
                    lines = format!("0,{}", vec!["1;1.5"; n].join("\n"));
                    &lines
                }
                "GroupMoveAbort" => {
                    x.aborted = true;
                    cv.notify_all();
                    "0"
                }
                "MultipleAxesPVTExecution" => {
                    x.running = true;
                    x = cv.wait_while(x, |x| !x.aborted && !x.released).unwrap();
                    x.running = false;
                    // XPS error -27: the trajectory was aborted.
                    if x.aborted { "-27" } else { "0" }
                }
                _ => "0",
            };
            format!("{values},EndOfAPI")
        }

        /// The XPS, each connection one socket; a command ends at its `)`.
        fn xps() -> (u16, Shared) {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let port = listener.local_addr().unwrap().port();
            let shared = Shared::default();
            let xps = shared.clone();
            std::thread::spawn(move || {
                for sock in listener.incoming() {
                    let (mut sock, xps): (TcpStream, _) = (sock.unwrap(), xps.clone());
                    std::thread::spawn(move || {
                        let mut cmd = Vec::new();
                        let mut buf = [0; 1024];
                        while let Ok(n @ 1..) = sock.read(&mut buf) {
                            cmd.extend_from_slice(&buf[..n]);
                            if cmd.ends_with(b")") {
                                let r = reply(&xps, &String::from_utf8_lossy(&cmd));
                                cmd.clear();
                                if sock.write_all(r.as_bytes()).is_err() {
                                    break;
                                }
                            }
                        }
                    });
                }
            });
            (port, shared)
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

        /// The trajectory database on NumAxes 1 and T:m1 on the simulated
        /// axis of asyn port `port`, its user offset 0, unless `set` says.
        async fn ioc(port: &str, set: &[(&str, &str)]) -> (Arc<PvDatabase>, ScanOwner) {
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
            (db, scan)
        }

        /// Poll until `holds` holds of the fake XPS.
        async fn until(xps: &Shared, what: &str, holds: impl Fn(&Xps) -> bool) {
            for _ in 0..500 {
                if holds(&xps.0.lock().unwrap()) {
                    return;
                }
                sleep(Duration::from_millis(10)).await;
            }
            panic!("the XPS never saw {what}: {:?}", xps.0.lock().unwrap().log);
        }

        /// The program's macros for one axis on the XPS at `port`.
        fn macros(port: u16) -> String {
            format!(
                "P=T:,R=traj1:,M1=m1,IPADDR=127.0.0.1,PORT={port},GROUP=G1,P1=P1,\
                 USERNAME=Administrator,PASSWORD=pw"
            )
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

        struct Program {
            db: Arc<PvDatabase>,
            xps: Shared,
            tid: SeqThreadId,
            file: String,
            _scan: ScanOwner,
            _ftp_log: std::sync::mpsc::Receiver<String>,
        }

        impl Drop for Program {
            fn drop(&mut self) {
                seq_stop(self.tid);
                let _ = std::fs::remove_file(&self.file);
            }
        }

        /// The program on its own XPS and FTP server, T:m1 on asyn port
        /// `port` as `ioc` sets it, Nelements 2, once init is done.
        async fn boot(port: &str, set: &[(&str, &str)]) -> Program {
            let (xps_port, xps) = xps();
            let (ftp_addr, ftp_log) = ftp::tests::server("331 password\r\n");
            ftp::FTP_PORT.store(ftp_addr.port(), Ordering::Relaxed);
            let (db, scan) = ioc(port, set).await;
            let file = std::env::temp_dir().join(format!("{port}{}.trj", std::process::id()));
            let file = file.to_str().unwrap().to_string();
            put(
                &db,
                "T:traj1:TrajectoryFile",
                EpicsValue::String(file.clone().into()),
            )
            .await;
            put(&db, "T:traj1:Nelements", EpicsValue::Long(2)).await;
            let tid = seq(&XPS_trajectoryScan, &macros(xps_port), 0, &db)
                .expect("XPS_trajectoryScan starts");
            // init clears buildMon as it ends; xpsTrajectoryPosition's
            // first poll comes a second after.
            until(&xps, "a position poll", |x| {
                x.log.iter().any(|c| c == "GroupPositionCurrentGet")
            })
            .await;
            Program {
                db,
                xps,
                tid,
                file,
                _scan: scan,
                _ftp_log: ftp_log,
            }
        }

        /// Execute, the run let end as done once wait_execute reads the
        /// element, as the start loop ends only on 45; a failed execute
        /// never gets that far.
        async fn execute_to_the_end(p: &Program) {
            put(&p.db, "T:traj1:Execute", EpicsValue::Enum(1)).await;
            for _ in 0..500 {
                let started = {
                    let x = p.xps.0.lock().unwrap();
                    x.running && x.log.iter().any(|c| c == "MultipleAxesPVTParametersGet")
                };
                if started || value(&p.db, "T:traj1:ExecStatus") == 2.0 {
                    break;
                }
                sleep(Duration::from_millis(10)).await;
            }
            p.xps.0.lock().unwrap().released = true;
            p.xps.1.notify_all();
        }

        /// init's positionSocket: the position polls ask this program's XPS,
        /// though another one holds socket 0.
        #[epics_rs::base::epics_test]
        async fn positions_are_read_from_its_own_xps() {
            let (decoy_port, decoy) = xps();
            let (port, xps) = xps();
            let (db, _scan) = ioc("xpsPositions", &[]).await;
            let (index, handle) =
                crate::ioc::connect_to_server(&db, "127.0.0.1", decoy_port.into()).unwrap();
            let name = handle.port_name().to_string();
            let decoy_socket = XpsSocket::new(
                handle,
                index,
                &name,
                SocketMode::Query,
                Duration::from_secs(1),
            );
            SOCKETS
                .lock()
                .unwrap()
                .insert(index, Arc::new(decoy_socket));
            let tid =
                seq(&XPS_trajectoryScan, &macros(port), 0, &db).expect("XPS_trajectoryScan starts");
            until(&xps, "a position poll", |x| {
                x.log.iter().any(|c| c == "GroupPositionCurrentGet")
            })
            .await;
            assert_eq!(decoy.0.lock().unwrap().log, Vec::<String>::new());
            seq_stop(tid);
        }

        /// ss xpsTrajectoryScan state init: NumAxes past MAX_AXES assigns
        /// MAX_AXES motors, and the program runs on.
        #[epics_rs::base::epics_test]
        async fn num_axes_past_max_axes_is_clamped_before_init_assigns() {
            boot("xpsNumAxes", &[("NAXES", "9")]).await;
        }

        /// ss xpsTrajectoryScan state init reads NumAxes once, so a put past
        /// MAX_AXES after it leaves a build as it was.
        #[epics_rs::base::epics_test]
        async fn a_num_axes_put_after_init_leaves_the_build_alone() {
            let p = boot("xpsNumAxesPut", &[]).await;
            put(&p.db, "T:traj1:NumAxes", EpicsValue::Long(9)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
        }

        /// ss xpsTrajectoryScan state build: Nelements past MAX_ELEMENTS
        /// builds MAX_ELEMENTS.
        #[epics_rs::base::epics_test]
        async fn nelements_past_max_elements_builds_max_elements() {
            let p = boot("xpsNelmMax", &[]).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(2001)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Build", 0.0),
                    ("T:traj1:BuildStatus", 1.0),
                    ("T:traj1:Nelements", 2000.0),
                ],
            )
            .await;
        }

        /// ss xpsTrajectoryScan state build: an absolute trajectory of one
        /// element builds two, the fewest it has a move in.
        #[epics_rs::base::epics_test]
        async fn one_absolute_element_builds_two() {
            let p = boot("xpsNelmAbs", &[]).await;
            // MOVE_MODE_ABSOLUTE
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(1)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(1)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Build", 0.0),
                    ("T:traj1:BuildStatus", 1.0),
                    ("T:traj1:Nelements", 2.0),
                ],
            )
            .await;
        }

        /// A build of `n` elements in `mode` moving no axis, on its own XPS:
        /// it reads no element, so it builds Nelements as put, the
        /// acceleration elements alone when that is none.
        async fn build_moving_no_axis(port: &str, mode: u16, n: i32) -> Program {
            let p = boot(port, &[]).await;
            put(&p.db, "T:traj1:M1Move", EpicsValue::Enum(0)).await;
            put(&p.db, "T:traj1:MoveMode", EpicsValue::Enum(mode)).await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(n)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Build", 0.0),
                    ("T:traj1:BuildStatus", 1.0),
                    ("T:traj1:Nelements", n.into()),
                ],
            )
            .await;
            assert_eq!(
                std::fs::read_to_string(&p.file).unwrap(),
                "0.000000, 0.000000, 0.000000\n".repeat(2)
            );
            p
        }

        /// ss xpsTrajectoryScan state build: an absolute trajectory of one
        /// element that moves no axis builds one.
        #[epics_rs::base::epics_test]
        async fn one_absolute_element_moving_no_axis_builds_one() {
            // MOVE_MODE_ABSOLUTE
            build_moving_no_axis("xpsNoMoveAbs", 1, 1).await;
        }

        /// ss xpsTrajectoryScan states build and execute: a relative
        /// trajectory of no element that moves no axis builds none, and
        /// executes pulsing over none.
        #[epics_rs::base::epics_test]
        async fn no_relative_element_moving_no_axis_builds_and_executes_none() {
            // MOVE_MODE_RELATIVE
            let p = build_moving_no_axis("xpsNoMoveRel", 0, 0).await;
            execute_to_the_end(&p).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Execute", 0.0),
                    ("T:traj1:ExecStatus", 1.0),
                    ("T:traj1:StartPulses", 0.0),
                    ("T:traj1:EndPulses", 0.0),
                ],
            )
            .await;
        }

        /// ss xpsTrajectoryScan state execute: pulses stay within the
        /// trajectory built, whatever Nelements was put after it.
        #[epics_rs::base::epics_test]
        async fn nelements_put_after_the_build_leaves_pulses_in_it() {
            let p = boot("xpsNelmPut", &[]).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
            put(&p.db, "T:traj1:Nelements", EpicsValue::Long(3000)).await;
            // monitor_inputs moves EndPulses to it.
            expect(&p.db, &[("T:traj1:EndPulses", 3000.0)]).await;
            execute_to_the_end(&p).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Execute", 0.0),
                    ("T:traj1:ExecStatus", 1.0),
                    ("T:traj1:EndPulses", 2.0),
                ],
            )
            .await;
        }

        /// ss xpsTrajectoryScan state readback: of more lines than
        /// MAX_PULSES gathered, MAX_PULSES are read.
        #[epics_rs::base::epics_test]
        async fn gathering_past_max_pulses_reads_max_pulses() {
            let p = boot("xpsGather", &[]).await;
            put(&p.db, "T:traj1:Npulses", EpicsValue::Long(2001)).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
            execute_to_the_end(&p).await;
            expect(
                &p.db,
                &[("T:traj1:Execute", 0.0), ("T:traj1:ExecStatus", 1.0)],
            )
            .await;
            p.xps.0.lock().unwrap().gathered = 2001;
            put(&p.db, "T:traj1:Readback", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[
                    ("T:traj1:Readback", 0.0),
                    ("T:traj1:ReadStatus", 1.0),
                    ("T:traj1:Nactual", 2000.0),
                ],
            )
            .await;
        }

        /// ss xpsTrajectoryScan state execute: T:m1 at user 10, dial 0, is
        /// within the XPS's -5..5 once its offset of 10 is taken off.
        #[epics_rs::base::epics_test]
        async fn soft_limits_are_checked_in_dial() {
            let p = boot("xpsLimits", &[("OFF", "10")]).await;
            put(&p.db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(
                &p.db,
                &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)],
            )
            .await;
            execute_to_the_end(&p).await;
            expect(
                &p.db,
                &[("T:traj1:Execute", 0.0), ("T:traj1:ExecStatus", 1.0)],
            )
            .await;
        }

        /// ss xpsTrajectoryAbort: an Abort while the trajectory runs sends
        /// GroupMoveAbort on the abort socket, so the execution ends.
        /// The xpsTrajectoryRun state set is in MultipleAxesPVTExecution
        /// then, as C's is, and xpsTrajectoryAbort must still see abortMon.
        #[ignore = "epics-seq-rs EventGate holds abortMon while xpsTrajectoryRun \
                    blocks in MultipleAxesPVTExecution (seqrt, BIG for 0.33)"]
        #[epics_rs::base::epics_test]
        async fn abort_during_execute_aborts_the_trajectory() {
            let p = boot("xpsAbort", &[]).await;
            let (db, xps) = (&p.db, &p.xps);
            put(db, "T:traj1:Build", EpicsValue::Enum(1)).await;
            expect(db, &[("T:traj1:Build", 0.0), ("T:traj1:BuildStatus", 1.0)]).await;
            put(db, "T:traj1:Execute", EpicsValue::Enum(1)).await;
            // xpsTrajectoryScan in wait_execute, which alone reads the
            // element: its start loop (:421) ends only on 45 or FAILURE.
            expect(db, &[("T:traj1:ExecState", 2.0)]).await;
            until(xps, "the element read", |x| {
                x.running && x.log.iter().any(|c| c == "MultipleAxesPVTParametersGet")
            })
            .await;

            put(db, "T:traj1:Abort", EpicsValue::Enum(1)).await;
            expect(
                db,
                &[
                    ("T:traj1:Abort", 0.0),
                    ("T:traj1:ExecStatus", 3.0),
                    ("T:traj1:ExecState", 0.0),
                    ("T:traj1:Execute", 0.0),
                ],
            )
            .await;
            let log = xps.0.lock().unwrap().log.clone();
            let run = log
                .iter()
                .position(|c| c == "MultipleAxesPVTExecution")
                .unwrap();
            assert!(log[run..].contains(&"GroupMoveAbort".to_string()));
        }
    }
}
