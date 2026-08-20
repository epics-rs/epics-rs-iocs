//! Port of `urRobotApp/src/calibrate_gripper.cpp` (the C PROD): connect to
//! the gripper URCap, activate if inactive, auto-calibrate, print the
//! native position range for the `MIN_POS`/`MAX_POS` db macros.
//!
//! The C program labels min "closed" and max "open"; the device unit runs
//! 0 = fully open to 255 = fully closed and `autoCalibrate` records min
//! at the open sweep (`robotiq_gripper.h:101`), so the labels here are
//! the corrected ones — `doc/upstream-c-defects.md` #222.

use std::process::ExitCode;
use std::time::Duration;

use ur_robot::UrResult;
use ur_robot::gripper::RobotiqGripper;

/// ur_rtde's `RobotiqGripper` socket timeout, as in the URGripper driver.
const GRIPPER_TIMEOUT: Duration = Duration::from_secs(2);

fn run(ip: &str) -> UrResult<()> {
    let mut gripper = RobotiqGripper::new(ip, GRIPPER_TIMEOUT);
    if let Err(e) = gripper.connect() {
        eprintln!("Error connecting to gripper, check that IP address is correct");
        return Err(e);
    }
    if !gripper.is_active()? {
        gripper.activate(false)?;
    }

    println!("Auto calibrating gripper...");
    gripper.auto_calibrate(None)?;
    println!("Gripper calibrated");
    println!();

    let (min, max) = gripper.native_position_range();
    println!("Min (open)   = {min}");
    println!("Max (closed) = {max}");
    Ok(())
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let (Some(ip), None) = (args.next(), args.next()) else {
        eprintln!("Please provide robot IP");
        return ExitCode::FAILURE;
    };
    match run(&ip) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
