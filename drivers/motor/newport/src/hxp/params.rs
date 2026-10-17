//! `HXPController`'s part of the controller port: the parameters
//! HXPDriver.h:15-52 creates and the `writeInt32` that runs `moveAll`,
//! `readAllCS` and `setCS` (HXPDriver.cpp:202-370).

use std::sync::{Arc, Mutex, MutexGuard};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::PortDriverBase;
use epics_rs::asyn::user::AsynUser;
use epics_rs::motor::controller_port::{ControllerParams, ControllerReport, ParamWrite};

use super::axis::{MoveLine, xps_status};
use super::controller::{HXP_GROUP, HxpController, MoveCoordSys};

/// The parameter indices, in C's createParam order.
#[derive(Clone, Copy, Debug)]
pub struct HxpParams {
    pub move_coord_sys: usize,
    pub status: usize,
    pub error: usize,
    pub error_desc: usize,
    pub move_all: usize,
    pub move_all_target: [usize; 6],
    pub coord_sys_read_all: usize,
    pub tool: [usize; 6],
    pub work: [usize; 6],
    pub base: [usize; 6],
    pub coord_sys_set: usize,
    pub coord_sys_to_set: usize,
    pub coord_sys_set_value: [usize; 6],
}

/// The six float64 parameters `<prefix>X` .. `<prefix>W`.
fn six(base: &mut PortDriverBase, prefix: &str) -> AsynResult<[usize; 6]> {
    let mut out = [0; 6];
    for (slot, axis) in out.iter_mut().zip(["X", "Y", "Z", "U", "V", "W"]) {
        *slot = base.create_param(&format!("{prefix}{axis}"), ParamType::Float64)?;
    }
    Ok(out)
}

impl HxpParams {
    fn create(base: &mut PortDriverBase) -> AsynResult<Self> {
        let int32 = |base: &mut PortDriverBase, name| base.create_param(name, ParamType::Int32);
        Ok(Self {
            move_coord_sys: int32(base, "HXP_MOVE_COORD_SYS")?,
            status: int32(base, "HXP_STATUS")?,
            error: int32(base, "HXP_ERROR")?,
            error_desc: base.create_param("HXP_ERROR_DESC", ParamType::Octet)?,
            move_all: int32(base, "HXP_MOVE_ALL")?,
            move_all_target: six(base, "HXP_MOVE_ALL_TARGET_")?,
            coord_sys_read_all: int32(base, "HXP_COORD_SYS_READ_ALL")?,
            tool: six(base, "HXP_COORD_SYS_TOOL_")?,
            work: six(base, "HXP_COORD_SYS_WORK_")?,
            base: six(base, "HXP_COORD_SYS_BASE_")?,
            coord_sys_set: int32(base, "HXP_COORD_SYS_SET")?,
            coord_sys_to_set: int32(base, "HXP_COORD_SYS_TO_SET")?,
            coord_sys_set_value: six(base, "HXP_COORD_SYS_SET_")?,
        })
    }
}

pub struct HxpControllerParams {
    controller: Arc<Mutex<HxpController>>,
    /// C's `movingPollPeriod_`/`idlePollPeriod_`, in seconds.
    moving_poll_s: f64,
    idle_poll_s: f64,
    /// The address-0 `HXP_MOVE_COORD_SYS` as written, which C's report
    /// reads. C prints an uninitialised int before the first write; 0
    /// stands for it.
    coord_sys: i32,
}

impl HxpControllerParams {
    pub fn new(
        controller: Arc<Mutex<HxpController>>,
        moving_poll_ms: u64,
        idle_poll_ms: u64,
    ) -> Self {
        Self {
            controller,
            moving_poll_s: moving_poll_ms as f64 / 1000.0,
            idle_poll_s: idle_poll_ms as f64 / 1000.0,
            coord_sys: 0,
        }
    }

    fn lock(&self) -> MutexGuard<'_, HxpController> {
        self.controller.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Six address-0 float64 parameters; one never written reads 0.
fn read_six(base: &PortDriverBase, reasons: &[usize; 6]) -> [f64; 6] {
    reasons.map(|reason| base.get_float64_param(reason, 0).unwrap_or(0.0))
}

/// C `HXPController::postError`: the error at address 0 and its callbacks.
fn post_error(
    base: &mut PortDriverBase,
    p: &HxpParams,
    line: &MoveLine,
    status: i32,
) -> AsynResult<()> {
    let (code, desc) = line.error_values(status);
    base.set_int32_param(p.error, 0, code)?;
    base.set_string_param(p.error_desc, 0, desc)?;
    base.call_param_callbacks(0)
}

impl ControllerParams for HxpControllerParams {
    /// C `HXPController::report` (HXPDriver.cpp:143-154).
    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "Newport hexapod motor driver {}, numAxes={}, moving poll period={:.6}, idle poll \
             period={:.6}, coordSys={}",
            ctx.port_name(),
            ctx.num_axes(),
            self.moving_poll_s,
            self.idle_poll_s,
            self.coord_sys
        );
        ctx.report(out, level);
    }

    fn create_params(&mut self, base: &mut PortDriverBase) -> AsynResult<()> {
        let params = HxpParams::create(base)?;
        self.lock().params = Some(params);
        Ok(())
    }

    fn write_int32(
        &mut self,
        base: &mut PortDriverBase,
        user: &AsynUser,
        value: i32,
    ) -> ParamWrite {
        let Some(p) = self.lock().params else {
            return ParamWrite::PassToBase(Ok(()));
        };
        let reason = user.reason;
        if reason == p.move_coord_sys && user.addr == 0 {
            self.coord_sys = value;
            // C `move()` reads it at address 0: 0 is Work, anything else Tool.
            let cs = if value == 0 {
                MoveCoordSys::Work
            } else {
                MoveCoordSys::Tool
            };
            self.lock().set_move_coord_sys(cs);
        }
        if ![p.move_all, p.coord_sys_read_all, p.coord_sys_set].contains(&reason) {
            return ParamWrite::PassToBase(Ok(()));
        }
        // C getAxis(pasynUser): the record's axis, whose move socket runs it.
        let Some(line) = self.lock().line(user.addr) else {
            return ParamWrite::Handled(Err(AsynError::Status {
                status: AsynStatus::Error,
                message: format!("HXP has no axis {}", user.addr),
            }));
        };
        ParamWrite::Handled(write_own(base, &p, &line, user, value))
    }
}

/// The three functions C `HXPController::writeInt32` keeps from the base.
fn write_own(
    base: &mut PortDriverBase,
    p: &HxpParams,
    line: &MoveLine,
    user: &AsynUser,
    value: i32,
) -> AsynResult<()> {
    base.set_int32_param(user.reason, user.addr, value)?;
    if value == 1 {
        if user.reason == p.move_all {
            move_all(base, p, line)?;
        } else if user.reason == p.coord_sys_read_all {
            read_all_cs(base, p, line)?;
        } else {
            set_cs(base, p, line)?;
        }
    }
    base.call_param_callbacks(user.addr)
}

/// C `HXPController::moveAll`.
fn move_all(base: &mut PortDriverBase, p: &HxpParams, line: &MoveLine) -> AsynResult<()> {
    let targets = read_six(base, &p.move_all_target);
    let status = xps_status(
        &line
            .sock()
            .hexapod_move_absolute(HXP_GROUP, "Work", &targets),
    );
    post_error(base, p, line, status)
}

/// C `HXPController::readAllCS`.
fn read_all_cs(base: &mut PortDriverBase, p: &HxpParams, line: &MoveLine) -> AsynResult<()> {
    for (cs, reasons) in [("Tool", &p.tool), ("Work", &p.work), ("Base", &p.base)] {
        let result = line.sock().hexapod_coordinate_system_get(HXP_GROUP, cs);
        // C sets its uninitialised locals when the read fails or the move
        // socket's 0.1 s read times out (HXPDriver.cpp:283); rs sets 0.
        let origin = result.as_ref().map_or([0.0; 6], |o| *o);
        for (reason, value) in reasons.iter().zip(origin) {
            base.set_float64_param(*reason, 0, value)?;
        }
        post_error(base, p, line, xps_status(&result.map(drop)))?;
    }
    Ok(())
}

/// C `HXPController::setCS`: 1 Work, 2 Tool, 3 Base, anything else nothing.
fn set_cs(base: &mut PortDriverBase, p: &HxpParams, line: &MoveLine) -> AsynResult<()> {
    let cs = base.get_int32_param(p.coord_sys_to_set, 0).unwrap_or(0);
    let origin = read_six(base, &p.coord_sys_set_value);
    let name = match cs {
        1 => Some("Work"),
        2 => Some("Tool"),
        3 => Some("Base"),
        _ => None,
    };
    let status = name.map_or(0, |name| {
        xps_status(
            &line
                .sock()
                .hexapod_coordinate_system_set(HXP_GROUP, name, &origin),
        )
    });
    post_error(base, p, line, status)
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::port_handle::PortHandle;
    use epics_rs::asyn::registry::open_owner;
    use epics_rs::motor::devices::register_controller_port;
    use motor_common::{Controller, asyn_user};

    use super::*;
    use crate::hxp::HxpAxis;
    use crate::xps::fake::FakeLine;
    use crate::xps::rpc::SocketMode;

    struct Rig {
        line: FakeLine,
        port: PortHandle,
        p: HxpParams,
        controller: Arc<Mutex<HxpController>>,
    }

    /// An HXP whose axes 0 and 1 have move lines, on `name`'s fake.
    fn rig(name: &str) -> Rig {
        let line = FakeLine::new(&format!("{name}_line"), "");
        let controller = Arc::new(Mutex::new(HxpController::new(
            line.socket(SocketMode::Query),
        )));
        let params = HxpControllerParams::new(controller.clone(), 100, 1000);
        let port = register_controller_port(&open_owner(), name, 6, params).unwrap();
        controller.lock().unwrap().attach_port(port.clone());
        let axes = Controller::new();
        for axis_no in 0..2 {
            let move_line = Arc::new(MoveLine::new(line.socket(SocketMode::Fire)));
            controller
                .lock()
                .unwrap()
                .add_line(axis_no, move_line.clone());
            let motor = Arc::new(Mutex::new(HxpAxis::new(
                controller.clone(),
                move_line,
                axis_no,
            )));
            axes.install(asyn_user(name, axis_no as i32), motor)
                .unwrap();
        }
        let p = controller.lock().unwrap().params.unwrap();
        line.take_log();
        Rig {
            line,
            port,
            p,
            controller,
        }
    }

    impl Rig {
        fn error(&self) -> (i32, Vec<u8>) {
            (
                self.port.read_int32_blocking(self.p.error, 0).unwrap(),
                self.port
                    .get_string_param_blocking(self.p.error_desc, 0, 80)
                    .unwrap(),
            )
        }
    }

    /// HXPDriver.cpp:267-285: the targets at address 0 go out in one
    /// HexapodMoveAbsolute, and a success clears the error.
    #[test]
    fn move_all_sends_the_address_0_targets() {
        let r = rig("hxpMoveAll");
        for (i, reason) in r.p.move_all_target.iter().enumerate() {
            r.port.write_float64_blocking(*reason, 0, i as f64).unwrap();
        }
        r.port.write_int32_blocking(r.p.move_all, 0, 0).unwrap();
        assert!(r.line.take_log().is_empty());
        r.port.write_int32_blocking(r.p.move_all, 0, 1).unwrap();
        assert_eq!(
            r.line.take_log(),
            ["HexapodMoveAbsolute (HEXAPOD,Work,0,1,2,3,4,5)"]
        );
        assert_eq!(r.error(), (0, Vec::new()));
    }

    /// HXPDriver.cpp:350-371: a failure posts its code and the controller's
    /// text for it, cut to 39 bytes.
    #[test]
    fn a_failed_write_posts_the_error_string() {
        let r = rig("hxpPostError");
        *r.line.fail.lock().unwrap() = "HexapodMoveAbsolute";
        r.port.write_int32_blocking(r.p.move_all, 0, 1).unwrap();
        let (code, desc) = r.error();
        assert_eq!(code, -3);
        assert_eq!(desc, b"Error in the command parameters of a he");
    }

    /// HXPDriver.cpp:292-327: Tool, Work and Base, each read on the move
    /// socket into its six parameters.
    #[test]
    fn read_all_cs_fills_the_three_systems() {
        let r = rig("hxpReadCs");
        r.port
            .write_int32_blocking(r.p.coord_sys_read_all, 0, 1)
            .unwrap();
        let log = r.line.take_log();
        let systems: Vec<_> = log
            .iter()
            .filter_map(|c| c.strip_prefix("HexapodCoordinateSystemGet (HEXAPOD,"))
            .map(|c| &c[..4])
            .collect();
        assert_eq!(systems, ["Tool", "Work", "Base"]);
        for reasons in [r.p.tool, r.p.work, r.p.base] {
            let read: Vec<_> = reasons
                .iter()
                .map(|reason| r.port.read_float64_blocking(*reason, 0).unwrap())
                .collect();
            assert_eq!(read, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        }
    }

    /// HXPDriver.cpp:335-348: 1 Work, 2 Tool, 3 Base; 0 sends nothing.
    #[test]
    fn set_cs_follows_the_to_set_menu() {
        let r = rig("hxpSetCs");
        r.port
            .write_float64_blocking(r.p.coord_sys_set_value[2], 0, 10.0)
            .unwrap();
        r.port
            .write_int32_blocking(r.p.coord_sys_set, 0, 1)
            .unwrap();
        assert!(r.line.take_log().is_empty());
        r.port
            .write_int32_blocking(r.p.coord_sys_to_set, 0, 2)
            .unwrap();
        r.port
            .write_int32_blocking(r.p.coord_sys_set, 0, 1)
            .unwrap();
        assert_eq!(
            r.line.take_log(),
            ["HexapodCoordinateSystemSet (HEXAPOD,Tool,0,0,10,0,0,0)"]
        );
    }

    /// HXPDriver.cpp:690-712: an axis's failed move sets the error at
    /// address 0, and its next good move clears it.
    #[test]
    fn an_axis_move_sets_the_address_0_error() {
        use epics_rs::asyn::interfaces::motor::AsynMotor;

        let r = rig("hxpAxisError");
        let line = r.controller.lock().unwrap().line(1).unwrap();
        let mut axis = HxpAxis::new(r.controller.clone(), line, 1);
        let user = AsynUser::new(0);
        *r.line.fail.lock().unwrap() = "HexapodMoveIncremental";
        assert!(axis.move_relative(&user, 1.0, 0.0, 1.0, 1.0).is_err());
        assert_eq!(
            r.error(),
            (-3, b"Error in the command parameters of a he".to_vec())
        );
        *r.line.fail.lock().unwrap() = "";
        axis.move_relative(&user, 1.0, 0.0, 1.0, 1.0).unwrap();
        assert_eq!(r.error(), (0, Vec::new()));
    }

    /// HXPDriver.cpp:603: moves read HXP_MOVE_COORD_SYS at address 0.
    #[test]
    fn the_address_0_coordinate_system_selects_tool_moves() {
        let r = rig("hxpMoveCs");
        r.port
            .write_int32_blocking(r.p.move_coord_sys, 1, 1)
            .unwrap();
        assert_eq!(
            r.controller.lock().unwrap().move_coord_sys(),
            MoveCoordSys::Work
        );
        r.port
            .write_int32_blocking(r.p.move_coord_sys, 0, 1)
            .unwrap();
        assert_eq!(
            r.controller.lock().unwrap().move_coord_sys(),
            MoveCoordSys::Tool
        );
    }
}
