//! `SPiiPlusComm(commPortName, asynPortName, numChannels)`,
//! SPiiPlusCommDriver.cpp's iocsh command: a comm on octet port
//! `asynPortName` as asyn port `commPortName`.

use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_int_opt, arg_str_req, req_string};
use motor_common::model1::c_int;

use crate::comm::{SpiiPlusComm, register_comm_port};

pub fn spiiplus_comm_command() -> CommandDef {
    CommandDef::new(
        "SPiiPlusComm",
        vec![
            arg_str_req("Comm port name"),
            arg_str_req("Asyn port name"),
            arg_int_opt("Num channels"),
        ],
        "SPiiPlusComm(commPortName, asynPortName, numChannels)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let comm_port = req_string(args, 0, "Comm port name")?;
            let asyn_port = req_string(args, 1, "Asyn port name")?;
            SpiiPlusComm::connect(&comm_port, &asyn_port)?;
            let owner = ctx.db().scan_control().interrupt_gate();
            register_comm_port(owner, &comm_port, c_int(args, 2))?;
            Ok(CommandOutcome::Continue)
        },
    )
}
