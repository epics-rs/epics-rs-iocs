//! XPS positioner corrector (PID) parameters.
//!
//! The XPS stores servo-loop gains in one of four corrector "types", each with
//! a different parameter set. Setting a single gain (P/I/D) is a read-modify-
//! write: read the corrector type, read all of that type's parameters, replace
//! the one gain, and write them all back. This mirrors
//! `XPSAxis::setPID`/`getPID`/`setPIDValue` and the `PositionerCorrector*`
//! wrappers.
//!
//! Note the C behavior faithfully reproduced here: on a `PIPosition` corrector
//! (which has no derivative term) a D-gain write updates the struct's `kd` but
//! the `PIPositionSet` command never sends it, so it has no wire effect.

use epics_rs::asyn::interfaces::motor::PidGainKind;

use super::rpc::{XpsError, XpsResult, XpsSocket, format_g};

/// `%.13g` double format used on the wire.
fn g(value: f64) -> String {
    format_g(value, 13)
}

/// The XPS corrector type in use for a positioner (`XPSController.cpp`
/// `CorrectorTypes`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CorrectorType {
    /// `PositionerCorrectorPIPosition`.
    PiPosition,
    /// `PositionerCorrectorPIDFFVelocity`.
    PidFfVelocity,
    /// `PositionerCorrectorPIDFFAcceleration`.
    PidFfAcceleration,
    /// `PositionerCorrectorPIDDualFFVoltage`.
    PidDualFfVoltage,
    /// `NoCorrector` — PID cannot be set.
    NoCorrector,
    /// Any other/unrecognized corrector type string.
    Other,
}

impl CorrectorType {
    fn parse(s: &str) -> Self {
        match s {
            "PositionerCorrectorPIPosition" => CorrectorType::PiPosition,
            "PositionerCorrectorPIDFFVelocity" => CorrectorType::PidFfVelocity,
            "PositionerCorrectorPIDFFAcceleration" => CorrectorType::PidFfAcceleration,
            "PositionerCorrectorPIDDualFFVoltage" => CorrectorType::PidDualFfVoltage,
            "NoCorrector" => CorrectorType::NoCorrector,
            _ => CorrectorType::Other,
        }
    }
}

/// All corrector parameters across the four types (`xpsCorrectorInfo_t`).
/// Unused fields for a given type stay at their read/default values.
#[derive(Clone, Copy, Debug, Default)]
pub struct XpsCorrectorInfo {
    pub closed_loop: bool,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub ks: f64,
    pub integration_time: f64,
    pub derivative_filter_cutoff: f64,
    pub gkp: f64,
    pub gki: f64,
    pub gkd: f64,
    pub kform: f64,
    pub ff_velocity: f64,
    pub ff_acceleration: f64,
    pub friction: f64,
}

/// Read the corrector type for `positioner` (`PositionerCorrectorTypeGet`).
pub fn corrector_type(sock: &XpsSocket, positioner: &str) -> XpsResult<CorrectorType> {
    let cmd = format!("PositionerCorrectorTypeGet ({positioner},char *)");
    let r = sock.exec(&cmd)?.require_ok()?;
    Ok(CorrectorType::parse(r.string(1)))
}

/// Read the full PID/corrector parameter set for `positioner`, dispatching on
/// its corrector type (`XPSAxis::getPID`). Returns an error for `NoCorrector`
/// or an unrecognized type, matching C.
pub fn get_pid(sock: &XpsSocket, positioner: &str) -> XpsResult<XpsCorrectorInfo> {
    let mut info = XpsCorrectorInfo::default();
    read_into(
        sock,
        positioner,
        corrector_type(sock, positioner)?,
        &mut info,
    )?;
    Ok(info)
}

/// Apply a single P/I/D gain to `positioner` via read-modify-write
/// (`XPSAxis::setPID` + `setPIDValue`).
pub fn set_pid(sock: &XpsSocket, positioner: &str, kind: PidGainKind, gain: f64) -> XpsResult<()> {
    let ct = corrector_type(sock, positioner)?;
    let mut info = XpsCorrectorInfo::default();
    read_into(sock, positioner, ct, &mut info)?;
    match kind {
        PidGainKind::Proportional => info.kp = gain,
        PidGainKind::Integral => info.ki = gain,
        PidGainKind::Derivative => info.kd = gain,
    }
    write_from(sock, positioner, ct, &info)
}

/// Read the type-specific parameters into `info`.
fn read_into(
    sock: &XpsSocket,
    positioner: &str,
    ct: CorrectorType,
    info: &mut XpsCorrectorInfo,
) -> XpsResult<()> {
    match ct {
        CorrectorType::PiPosition => {
            let cmd = format!(
                "PositionerCorrectorPIPositionGet ({positioner},bool *,double *,double *,double *)"
            );
            let r = sock.exec(&cmd)?.require_ok()?;
            info.closed_loop = r.int(1) != 0;
            info.kp = r.double(2);
            info.ki = r.double(3);
            info.integration_time = r.double(4);
            Ok(())
        }
        CorrectorType::PidFfVelocity => {
            let cmd = format!(
                "PositionerCorrectorPIDFFVelocityGet ({positioner},bool *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *)"
            );
            let r = sock.exec(&cmd)?.require_ok()?;
            read_pidff_common(&r, info);
            info.ff_velocity = r.double(12);
            Ok(())
        }
        CorrectorType::PidFfAcceleration => {
            let cmd = format!(
                "PositionerCorrectorPIDFFAccelerationGet ({positioner},bool *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *)"
            );
            let r = sock.exec(&cmd)?.require_ok()?;
            read_pidff_common(&r, info);
            info.ff_acceleration = r.double(12);
            Ok(())
        }
        CorrectorType::PidDualFfVoltage => {
            let cmd = format!(
                "PositionerCorrectorPIDDualFFVoltageGet ({positioner},bool *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *,double *)"
            );
            let r = sock.exec(&cmd)?.require_ok()?;
            read_pidff_common(&r, info);
            info.ff_velocity = r.double(12);
            info.ff_acceleration = r.double(13);
            info.friction = r.double(14);
            Ok(())
        }
        CorrectorType::NoCorrector | CorrectorType::Other => Err(XpsError::Api(0)),
    }
}

/// Fields 1..=11 shared by the three PIDFF* correctors (closed, KP, KI, KD, KS,
/// IntegrationTime, DerivativeFilterCutOff, GKP, GKI, GKD, KForm).
fn read_pidff_common(r: &super::rpc::XpsReply, info: &mut XpsCorrectorInfo) {
    info.closed_loop = r.int(1) != 0;
    info.kp = r.double(2);
    info.ki = r.double(3);
    info.kd = r.double(4);
    info.ks = r.double(5);
    info.integration_time = r.double(6);
    info.derivative_filter_cutoff = r.double(7);
    info.gkp = r.double(8);
    info.gki = r.double(9);
    info.gkd = r.double(10);
    info.kform = r.double(11);
}

/// Write the type-specific parameters back (`PositionerCorrector*Set`).
fn write_from(
    sock: &XpsSocket,
    positioner: &str,
    ct: CorrectorType,
    info: &XpsCorrectorInfo,
) -> XpsResult<()> {
    let closed = i32::from(info.closed_loop);
    let cmd = match ct {
        CorrectorType::PiPosition => format!(
            "PositionerCorrectorPIPositionSet ({positioner},{closed},{},{},{})",
            g(info.kp),
            g(info.ki),
            g(info.integration_time),
        ),
        CorrectorType::PidFfVelocity => format!(
            "PositionerCorrectorPIDFFVelocitySet ({positioner},{closed},{},{},{},{},{},{},{},{},{},{},{})",
            g(info.kp),
            g(info.ki),
            g(info.kd),
            g(info.ks),
            g(info.integration_time),
            g(info.derivative_filter_cutoff),
            g(info.gkp),
            g(info.gki),
            g(info.gkd),
            g(info.kform),
            g(info.ff_velocity),
        ),
        CorrectorType::PidFfAcceleration => format!(
            "PositionerCorrectorPIDFFAccelerationSet ({positioner},{closed},{},{},{},{},{},{},{},{},{},{},{})",
            g(info.kp),
            g(info.ki),
            g(info.kd),
            g(info.ks),
            g(info.integration_time),
            g(info.derivative_filter_cutoff),
            g(info.gkp),
            g(info.gki),
            g(info.gkd),
            g(info.kform),
            g(info.ff_acceleration),
        ),
        CorrectorType::PidDualFfVoltage => format!(
            "PositionerCorrectorPIDDualFFVoltageSet ({positioner},{closed},{},{},{},{},{},{},{},{},{},{},{},{},{})",
            g(info.kp),
            g(info.ki),
            g(info.kd),
            g(info.ks),
            g(info.integration_time),
            g(info.derivative_filter_cutoff),
            g(info.gkp),
            g(info.gki),
            g(info.gkd),
            g(info.kform),
            g(info.ff_velocity),
            g(info.ff_acceleration),
            g(info.friction),
        ),
        CorrectorType::NoCorrector | CorrectorType::Other => return Err(XpsError::Api(0)),
    };
    sock.exec(&cmd)?.require_ok()?;
    Ok(())
}
