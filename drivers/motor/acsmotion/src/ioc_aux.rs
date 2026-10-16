//! `AcsMotionAuxIOConfig(auxIOPortName, asynPortName, numChannels,
//! pollPeriod)`, SPiiPlusAuxDriver.cpp's iocsh command; the period is in
//! seconds.

use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_double_opt, arg_int_opt, arg_str_req, opt_double, req_string};
use motor_common::model1::c_int;

pub fn acsmotion_aux_io_config_command() -> CommandDef {
    CommandDef::new(
        "AcsMotionAuxIOConfig",
        vec![
            arg_str_req("Aux IO port name"),
            arg_str_req("Asyn port name"),
            arg_int_opt("Max I/O channels"),
            arg_double_opt("Poll period (s)"),
        ],
        "AcsMotionAuxIOConfig(auxIOPortName, asynPortName, numChannels, pollPeriod)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port = req_string(args, 0, "Aux IO port name")?;
            let asyn_port = req_string(args, 1, "Asyn port name")?;
            let period = opt_double(args, 3, 0.0, "Poll period (s)")?;
            let owner = ctx.db().scan_control().interrupt_gate();
            crate::aux_io::config(owner, &port, &asyn_port, c_int(args, 2), period)?;
            Ok(CommandOutcome::Continue)
        },
    )
}
