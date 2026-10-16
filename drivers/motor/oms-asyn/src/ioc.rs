//! `omsMAXnetConfig` / `omsMXAConfig`: the controller and its axes, which
//! records reach through `DTYP "asynMotor"` and `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_int_req, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::{asyn_user, install, register_port};

use crate::oms::{OmsAxis, OmsController};

/// C `OMS_MAX_AXES`.
const OMS_MAX_AXES: usize = 10;

/// The constructor's `timeout = 2.0`.
const TIMEOUT: Duration = Duration::from_secs(2);

/// `omsMAXnetConfig` and `omsMXAConfig`.
pub fn oms_commands() -> Vec<CommandDef> {
    vec![
        oms_config_command("omsMAXnetConfig", "MAXnet"),
        oms_config_command("omsMXAConfig", "MXA"),
    ]
}

/// `<cmd_name>(portName, numAxes, serialPortName, movingPollPeriod,
/// idlePollPeriod, initString)`. As C's constructor, a failure past the
/// port's creation is printed and leaves the port without axes.
fn oms_config_command(cmd_name: &'static str, controller_type: &'static str) -> CommandDef {
    CommandDef::new(
        cmd_name,
        vec![
            arg_str_req("asyn motor port name"),
            arg_int_req("number of axes"),
            arg_str_req("asyn serial/tcp port name"),
            arg_int_req("moving poll rate"),
            arg_int_req("idle poll rate"),
            arg_str_req("initstring"),
        ],
        format!(
            "{cmd_name}(portName, numAxes, serialPortName, movingPollPeriod, \
             idlePollPeriod, initString)"
        ),
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "asyn motor port name")?;
            let num_axes = c_int(args, 1);
            let serial_port = req_string(args, 2, "asyn serial/tcp port name")?;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 3, 4)?;
            let init_string = req_string(args, 5, "initstring").unwrap_or_default();
            let who = format!("{controller_type}Config");

            register_port(ctx, &port_name, num_axes)?;
            let Some(handle) = connect_with_default_eos(&serial_port, TIMEOUT, &who) else {
                println!(
                    "{who}: can't connect to port {serial_port}: port {serial_port} not found"
                );
                return Ok(CommandOutcome::Continue);
            };
            let controller = OmsController::new(handle, &serial_port, controller_type);
            if let Err(e) = controller.firmware_version() {
                println!("{who}: unable to talk to controller at {serial_port}: {e}");
                return Ok(CommandOutcome::Continue);
            }
            controller.init(&init_string);

            // omsBaseController::Init: the axes are the fields of `AA RP;`.
            let total_axes = match controller.axis_count() {
                Ok(n) => n,
                Err(e) => {
                    println!("{who}: unable to talk to controller at {serial_port}: {e}");
                    return Ok(CommandOutcome::Continue);
                }
            };
            if num_axes as usize > OMS_MAX_AXES {
                eprintln!(
                    "omsBaseController:Init: number of axes={num_axes} exceeds allowed maximum"
                );
                println!("{who}: unable to talk to controller at {serial_port}");
                return Ok(CommandOutcome::Continue);
            }
            if total_axes > num_axes.max(0) as usize {
                eprintln!(
                    "omsBaseController:Init: actual number of axes={total_axes} > MIN(numAxes={num_axes}, OMS_MAX_AXES)"
                );
                println!("{who}: unable to talk to controller at {serial_port}");
                return Ok(CommandOutcome::Continue);
            }

            let controller = Arc::new(Mutex::new(controller));
            for axis in 0..total_axes {
                let axis_dev = OmsAxis::new(controller.clone(), axis)
                    .map_err(|e| format!("{cmd_name}: axis {axis}: {e}"))?;
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_dev));
                install(
                    ctx,
                    asyn_user(&port_name, axis as i32),
                    motor,
                    moving_poll_ms,
                    idle_poll_ms,
                )?;
            }
            Ok(CommandOutcome::Continue)
        },
    )
}

/// Connect to the octet port, giving it the OMS EOS (input `"\n\r"`, output
/// `"\n"`) only where st.cmd left that direction empty, as
/// `omsMAXnet`/`omsMXA` do after `getInputEos`/`getOutputEos`. As there, a
/// refused get skips that direction and a refused set is printed, never fatal
/// (omsMAXnet.cpp:134-142, omsMXA.cpp:129-137).
fn connect_with_default_eos(port_name: &str, timeout: Duration, who: &str) -> Option<SyncIOHandle> {
    let port = get_port(port_name)?;
    let user = || AsynUser::new(0).with_addr(0);
    let handle = &port.handle;
    if handle
        .get_input_eos_blocking(user())
        .is_ok_and(|eos| eos.is_empty())
        && let Err(e) = handle.set_input_eos_blocking(user(), b"\n\r")
    {
        println!("{who}: unable to set InputEOS {port_name}: {e}");
    }
    if handle
        .get_output_eos_blocking(user())
        .is_ok_and(|eos| eos.is_empty())
        && let Err(e) = handle.set_output_eos_blocking(user(), b"\n")
    {
        println!("{who}: unable to set OutputEOS {port_name}: {e}");
    }
    Some(SyncIOHandle::from_handle(handle.clone(), 0, timeout))
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    /// A port with no EOS support of its own: both sets are refused.
    struct Bare(PortDriverBase);

    impl PortDriver for Bare {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    #[test]
    fn a_port_refusing_the_default_eos_still_connects() {
        let name = "omsEosRefused";
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Bare(base), RuntimeConfig::default()).unwrap();
        let port = rt.port_handle().clone();
        assert!(
            port.set_input_eos_blocking(AsynUser::new(0), b"\n\r")
                .is_err()
        );
        claim.publish(port, &open_owner()).unwrap();
        assert!(connect_with_default_eos(name, Duration::from_secs(1), "MAXnetConfig").is_some());
    }
}
