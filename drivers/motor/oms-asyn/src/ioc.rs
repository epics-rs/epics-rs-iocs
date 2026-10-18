//! `omsMAXnetConfig` / `omsMXAConfig`: the controller and its axes, which
//! records reach through `DTYP "asynMotor"` and `@asyn(portName,axis)`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::iocsh::{arg_int_req, arg_str_req, poll_intervals, req_string};
use motor_common::model1::c_int;
use motor_common::{Controller, asyn_user, register_port_with};

use crate::oms::{FirmwareVersion, OmsAxis, OmsController, OmsParams};

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
    // The constructors' minFwMajor, minFwMinor, minFwRevision.
    let min_fw = FirmwareVersion {
        major: 1,
        minor: if controller_type == "MAXnet" { 30 } else { 0 },
        revision: 0,
    };
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

            let (params, report) = OmsParams::new(controller_type);
            register_port_with(ctx, &port_name, num_axes, params)?;
            // On each early return C still runs startPoller on the half-built
            // controller and crashes (a C bug); the port keeps no axes here.
            let Some(handle) = connect_with_default_eos(&serial_port, TIMEOUT, &who) else {
                println!(
                    "{who}: can't connect to port {serial_port}: asynManager:connectDevice port \
                     {serial_port} not found"
                );
                return Ok(CommandOutcome::Continue);
            };
            let mut controller =
                OmsController::new(handle, &port_name, &serial_port, controller_type);
            if let Err(e) = controller.watch_connection() {
                println!("{who}: can't set exceptionCallback for {serial_port}: {e}");
                return Ok(CommandOutcome::Continue);
            }
            let fw = match controller.firmware_version() {
                Ok(version) => version,
                Err(e) => {
                    println!(
                        "{who}: unable to talk to controller at {serial_port}: {}",
                        e.message_display()
                    );
                    return Ok(CommandOutcome::Continue);
                }
            };
            let _ = report.fw.set(fw);
            controller.set_firmware(fw, min_fw);
            if fw.minor < min_fw.minor {
                println!(
                    "{}",
                    unsupported_firmware(controller_type, fw.major, fw.minor)
                );
            }
            controller.init(&init_string);

            // omsBaseController::Init: the axes are the fields of `AA RP;`.
            let total_axes = controller.axis_count();
            // Init leaves errorMessage untouched, so C prints it empty.
            if num_axes as usize > OMS_MAX_AXES {
                errlog_vprintf(format_args!(
                    "omsBaseController:Init: number of axes={num_axes} exceeds allowed maximum\n"
                ));
                println!("{who}: unable to talk to controller at {serial_port}: ");
                return Ok(CommandOutcome::Continue);
            }
            if total_axes > num_axes.max(0) as usize {
                errlog_vprintf(format_args!(
                    "omsBaseController:Init: actual number of axes={total_axes} > \
                     MIN(numAxes={num_axes}, OMS_MAX_AXES)\n"
                ));
                println!("{who}: unable to talk to controller at {serial_port}: ");
                return Ok(CommandOutcome::Continue);
            }

            let shared = controller.poll_shared().clone();
            let controller = Arc::new(Mutex::new(controller));
            let _ = report.controller.set(controller.clone());
            let axes = (0..total_axes)
                .map(|axis| {
                    OmsAxis::new(controller.clone(), axis)
                        .map_err(|e| format!("{cmd_name}: axis {axis}: {e}"))
                })
                .collect::<Result<Vec<_>, _>>()?;
            let _ = report.axes.set(shared);
            crate::poller::init_positions(&controller);
            let mut posters = Vec::with_capacity(total_axes);
            // No timed pass: the OMS poller posts (zero periods wait on
            // events alone).
            let axes_poller = Controller::new();
            for (axis, axis_dev) in axes.into_iter().enumerate() {
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis_dev));
                posters.push(axes_poller.install(asyn_user(&port_name, axis as i32), motor)?);
            }
            axes_poller.start(ctx, &port_name, 0, 0, 0);
            crate::poller::start(
                &controller,
                posters,
                Duration::from_millis(moving_poll_ms),
                Duration::from_millis(idle_poll_ms),
                fw.minor,
            );
            Ok(CommandOutcome::Continue)
        },
    )
}

/// The constructors' unsupported-firmware line; only MXA's names the minimum
/// from its fields.
fn unsupported_firmware(controller_type: &str, major: i32, minor: i32) -> String {
    let mandatory = if controller_type == "MAXnet" {
        "1.30"
    } else {
        "1.0"
    };
    format!(
        "This Controllers Firmware Version {major}.{minor} is not supported, version \
         {mandatory} or higher is mandatory"
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
    use epics_rs::asyn::port::InterfaceMask;
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
        let base = PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        );
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
