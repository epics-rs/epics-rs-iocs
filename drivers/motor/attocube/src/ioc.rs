//! `ANC150AsynSetup` / `ANC150AsynConfig` iocsh commands for the attocube
//! ANC150 stepper, a C model-2 driver: `ANC150AsynConfig` builds card `card`
//! of [`MOTOR_ANC150`], and `drvAsynMotorConfigure(port, "motorANC150", card,
//! numAxes)` opens its axes as `@asyn(port,0..)`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::base::server::iocsh::registry::*;

use motor_common::connect::{EosRefusal, connect_with_eos};
use motor_common::iocsh::{arg_int_req, arg_str_req, req_string};
use motor_common::model1::c_int;
use motor_common::model2::{Model2, Model2Axis};

use crate::anc150::{Anc150Axis, Anc150Controller, INPUT_EOS, OUTPUT_EOS};

/// Serial command timeout (C `TIMEOUT` 2.0 s).
const ANC150_TIMEOUT: Duration = Duration::from_secs(2);

/// `ANC150_MAX_AXES` (drvANC150Asyn.cc).
const ANC150_MAX_AXES: i32 = 6;

/// drvANC150Asyn.cc's `motorAxisDrvSET_t`.
/// Its poller forces none (drvANC150Asyn.cc:580).
pub static MOTOR_ANC150: Model2 = Model2::new("motorANC150", 0);

/// `numANC150Controllers`, which `ANC150AsynSetup` sets.
static NUM_CONTROLLERS: Mutex<i32> = Mutex::new(0);

/// `ANC150AsynSetup(num_controllers)`, printing what C's does.
pub fn anc150_setup_command() -> CommandDef {
    CommandDef::new(
        "ANC150AsynSetup",
        vec![arg_int_req("Maximum # of controllers")],
        "ANC150AsynSetup(num_controllers)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let num_controllers = c_int(args, 0);
            if num_controllers < 1 {
                println!("ANC150Setup, num_controllers must be > 0");
                return Ok(CommandOutcome::Continue);
            }
            *NUM_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner()) = num_controllers;
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `ANC150AsynConfig(card, portName, numAxes, movingPollPeriod,
/// idlePollPeriod)`, printing what C's does; a controller that never answers
/// `ver` fails silently, as in C.
pub fn anc150_config_command() -> CommandDef {
    CommandDef::new(
        "ANC150AsynConfig",
        vec![
            arg_int_req("Card# being configured"),
            arg_str_req("asyn port name"),
            arg_int_req("Number of Axes"),
            arg_int_req("Moving poll rate"),
            arg_int_req("Idle poll rate"),
        ],
        "ANC150AsynConfig(card, portName, numAxes, movingPollPeriod, idlePollPeriod)",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let card = c_int(args, 0);
            let port_name = req_string(args, 1, "asyn port name")?;
            let num_axes = c_int(args, 2);
            let moving_poll_ms = c_int(args, 3).max(0) as u64;
            let idle_poll_ms = c_int(args, 4).max(0) as u64;

            let num_controllers = *NUM_CONTROLLERS.lock().unwrap_or_else(|e| e.into_inner());
            if num_controllers < 1 {
                println!("ANC150Config: no ANC150 controllers allocated, call ANC150Setup first");
                return Ok(CommandOutcome::Continue);
            }
            if !(0..num_controllers).contains(&card) {
                println!(
                    "ANC150Config: card must in range 0 to {}",
                    num_controllers - 1
                );
                return Ok(CommandOutcome::Continue);
            }
            if !(1..=ANC150_MAX_AXES).contains(&num_axes) {
                println!("ANC150Config: numAxes must in range 1 to {ANC150_MAX_AXES}");
                return Ok(CommandOutcome::Continue);
            }

            let Ok(handle) = connect_with_eos(
                &port_name,
                0,
                ANC150_TIMEOUT,
                INPUT_EOS,
                OUTPUT_EOS,
                EosRefusal::Ignored,
            ) else {
                println!("ANC150AsynConfig: cannot connect to asyn port {port_name}");
                return Ok(CommandOutcome::Continue);
            };
            // UPSTREAM DEFECT (drvANC150Asyn.cc:260, :295): C returns here with
            // pAxis never allocated, so motorAxisOpen's &pAxis[axis] segfaults
            // in motorAxisSetCallback. No card is registered here instead.
            let Ok(controller) = Anc150Controller::new(handle, &port_name) else {
                return Ok(CommandOutcome::Continue);
            };
            let controller = Arc::new(Mutex::new(controller));
            let axes = (0..num_axes as usize)
                .map(|axis| {
                    let ax = Anc150Axis::new(controller.clone(), card, axis)
                        .map_err(|e| format!("ANC150AsynConfig: axis {axis}: {e}"))?;
                    let (getter, log) = (ax.params(), ax.log());
                    Ok(Model2Axis::new(Arc::new(Mutex::new(ax)), getter, log))
                })
                .collect::<Result<Vec<_>, String>>()?;
            MOTOR_ANC150.start_card(card, axes, moving_poll_ms, idle_poll_ms)?;
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;
    use motor_common::testing::{LineRig, run_command};

    /// drvANC150Asyn.cc:784-788: ANC150AsynConfig starts ANC150Poller, whose
    /// `getm` polls the axes before any drvAsynMotorConfigure.
    #[epics_rs::base::epics_test]
    async fn anc150_config_starts_its_poller() {
        run_command(anc150_setup_command(), vec![ArgValue::Int(1)]).await;
        let (line, _io) = LineRig::new("ancPollLine", |last| match last {
            "ver" => "ver\r\nattocube ANC150\r\nOK\r\n".into(),
            c if c.starts_with("getf") => format!("{c}\r\nfrequency = 1000\r\nOK\r\n"),
            c => format!("{c}\r\nmode = stp\r\nOK\r\n"),
        });
        let args = vec![
            ArgValue::Int(0),
            ArgValue::String("ancPollLine".into()),
            ArgValue::Int(1),
            ArgValue::Int(100),
            ArgValue::Int(1000),
        ];
        run_command(anc150_config_command(), args).await;
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut sent = line.sent();
        while !sent.iter().any(|c| c == "getm 1") && Instant::now() < deadline {
            epics_rs::base::runtime::task::sleep(Duration::from_millis(10)).await;
            sent.extend(line.sent());
        }
        assert!(sent.iter().any(|c| c == "getm 1"), "{sent:?}");
    }
}
