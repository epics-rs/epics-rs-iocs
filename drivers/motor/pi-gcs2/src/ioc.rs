//! `PI_GCS2_CreateController` iocsh command for the PI GCS2 driver.
//!
//! Mirrors the C `PIasynController` constructor / `PI_GCS2_CreateController`
//! iocsh function: create controller port `portName`, connect the serial/TCP
//! octet port, identify and probe the controller (which auto-discovers
//! connected axis names via `SAI?`), then give `portName` the first
//! `numAxes` of those axes in discovery order, addressed by records as
//! `@asyn(portName,axis)`. Single-step, matching upstream — see
//! `gcs2.rs`'s "Config" doc section for why this is not a two-step
//! registry/explicit-`CreateAxis` API like `motor-phytron`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::interfaces::motor::AsynMotor;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosConnectError, EosRefusal, connect_with_eos};
use motor_common::iocsh::{
    arg_int_opt, arg_int_req, arg_str_req, poll_intervals, req_int, req_string,
};
use motor_common::{Controller, asyn_user, register_port};

use crate::gcs2::{PIGCS2Axis, PIGCS2Controller, is_known_controller};

/// Command timeout (C `PIInterface::TIMEOUT` is 5.0 s).
const PIGCS2_TIMEOUT: Duration = Duration::from_secs(5);

/// Build the `PI_GCS2_CreateController(portName, asynPort, numAxes, priority,
/// stackSize, movingPollingRate, idlePollingRate)` command, rates in ms.
/// `priority` and `stackSize` size C's port thread, which has no counterpart
/// here.
pub fn pigcs2_config_command() -> CommandDef {
    CommandDef::new(
        "PI_GCS2_CreateController",
        vec![
            arg_str_req("Port name"),
            arg_str_req("asyn Port name"),
            arg_int_req("Number of axes"),
            arg_int_opt("priority"),
            arg_int_opt("stackSize"),
            arg_int_opt("moving polling time [msec]"),
            arg_int_opt("idle polling time [msec]"),
        ],
        "PI_GCS2_CreateController(portName, asynPort, numAxes, priority, stackSize, \
         movingPollingRate, idlePollingRate)",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = req_string(args, 0, "Port name")?;
            let asyn_port = req_string(args, 1, "asyn Port name")?;
            // C: `if (numAxes < 1 ) numAxes = 1;`
            let num_axes = req_int(args, 2, "Number of axes")?.max(1) as usize;
            let (moving_poll_ms, idle_poll_ms) = poll_intervals(args, 5, 6)?;

            // C's asynMotorController base creates the port before any of the
            // constructor's early returns.
            register_port(ctx, &port_name, num_axes as i32)?;
            let Some(handle) = connect(&asyn_port) else {
                return Ok(CommandOutcome::Continue);
            };
            let mut controller = PIGCS2Controller::new(handle, &asyn_port);
            let idn = controller.read_idn().to_string();
            controller.print(format_args!("read from {asyn_port}: {idn}\n"));
            let idn = idn.to_ascii_uppercase();
            if !is_known_controller(&idn) {
                controller.print(format_args!(
                    "PIasynController: unknown controller type {asyn_port}: {idn}\n"
                ));
                return Ok(CommandOutcome::Continue);
            }
            controller.init();
            let found: Vec<String> = controller.axis_names().to_vec();
            if found.len() < num_axes {
                controller.print(format_args!(
                    "PIasynController: requested number of axes ({num_axes}) out of range, \
                     only {} axis/axes supported\n",
                    found.len()
                ));
                return Ok(CommandOutcome::Continue);
            }
            let controller = Arc::new(Mutex::new(controller));
            // C startPoller, PIasynController.cpp:192. Its poll()
            // (PIasynController.cpp:579) reads nothing on a motor controller:
            // getGlobalState is PIGCSController's no-op, and the pivot,
            // coordinate-system and last-error parameters it posts are not
            // modeled here.
            let poller = Controller::new();
            for (index, axis_name) in found.into_iter().take(num_axes).enumerate() {
                let axis = PIGCS2Axis::new(controller.clone(), axis_name, index, &port_name);
                let motor: Arc<Mutex<dyn AsynMotor>> = Arc::new(Mutex::new(axis));
                poller.install(asyn_user(&port_name, index as i32), motor)?;
            }
            poller.start(ctx, moving_poll_ms, idle_poll_ms, 10);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// C's `pasynOctetSyncIO->connect` and EOS sets on `pAsynCom`; each refusal is
/// printed on that user and leaves the controller without axes.
fn connect(asyn_port: &str) -> Option<SyncIOHandle> {
    let e = match connect_with_eos(asyn_port, PIGCS2_TIMEOUT, b"\n", b"", EosRefusal::BothFatal) {
        Ok(handle) => return Some(handle),
        Err(e) => e,
    };
    let manager = PortManager::global();
    let trace = manager.trace_manager();
    match e {
        // An unconnected user prints on the global trace.
        EosConnectError::PortNotFound(_) => trace.print(
            None,
            None,
            0,
            TraceMask::ERROR,
            format_args!("echoHandler: unable to connect to port {asyn_port}\n"),
        ),
        EosConnectError::InputEos(_, e) => trace.print(
            Some(asyn_port),
            Some(0),
            0,
            TraceMask::ERROR,
            format_args!(
                "echoHandler: unable to set input EOS on {asyn_port}: {}\n",
                e.message_display()
            ),
        ),
        EosConnectError::OutputEos(_, e) => trace.print(
            Some(asyn_port),
            Some(0),
            0,
            TraceMask::ERROR,
            format_args!(
                "echoHandler: unable to set output EOS on {asyn_port}: {}\n",
                e.message_display()
            ),
        ),
    }
    None
}

#[cfg(all(test, target_os = "linux"))]
mod one_poller {
    use super::*;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;
    use motor_common::testing::pollers_after;

    /// A C-863 with axes `1` and `2`. GCS sends a command and its `\n` as two
    /// writes, so a read answers the last write before the `\n`.
    struct C863 {
        base: PortDriverBase,
        command: String,
        sai_left: bool,
    }

    impl PortDriver for C863 {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn set_input_eos(&mut self, user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
            self.base.set_input_eos(user.addr, eos);
            Ok(())
        }
        fn set_output_eos(&mut self, user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
            self.base.set_output_eos(user.addr, eos);
            Ok(())
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            if data != b"\n" {
                self.command = String::from_utf8_lossy(data).into_owned();
                self.sai_left = self.command == "SAI?";
            }
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            let reply = match self.command.as_str() {
                "*IDN?" => "(c)2026 Physik Instrumente (PI) GmbH & Co. KG, C-863.11, 0, 1.0",
                "SAI?" if self.sai_left => {
                    self.sai_left = false;
                    "1 "
                }
                "SAI?" => "2",
                _ => "1=0",
            };
            buf[..reply.len()].copy_from_slice(reply.as_bytes());
            Ok(reply.len())
        }
    }

    /// C PIasynController's one asynMotorController poller for all its axes.
    #[epics_rs::base::epics_test]
    async fn pi_gcs2_create_controller_polls_its_axes_on_one_poller() {
        let line = C863 {
            base: PortDriverBase::new("c863Line", 1, PortFlags::default()),
            command: String::new(),
            sai_left: false,
        };
        let claim = claim_port("c863Line").unwrap();
        let (rt, _actor) = create_port_runtime(&claim, line, RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        let args = vec![
            ArgValue::String("c863".into()),
            ArgValue::String("c863Line".into()),
            ArgValue::Int(2),
            ArgValue::Int(0),
            ArgValue::Int(0),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ];
        let cmd = pigcs2_config_command();
        assert_eq!(pollers_after(cmd, args).await, 1);
    }
}
