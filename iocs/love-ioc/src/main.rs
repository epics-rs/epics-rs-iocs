//! Love PID controller IOC — RS-485 multi-drop, generic asyn Int32/
//! UInt32Digital device support.
//!
//! Usage:
//!   cargo run -p love-ioc -- iocs/love-ioc/st.cmd

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use love::connect::{follow_serial_port, init_serial_port, set_default_eos};
use love::driver::{INPUT_EOS, LoveDriver, OUTPUT_EOS};
use love::registry::{K_INSTRMAX, Model};

/// C `#define K_COMTMO (1.0)` (`drvLove.c:116`) — the `pasynUser->timeout`
/// used for every Love command transaction.
const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: love-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("LOVE", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // Standard asyn iocsh commands — provides drvAsynSerialPortConfigure /
    // drvAsynIPPortConfigure / asynOctetSetInputEos / asynOctetSetOutputEos.
    let port_manager = Arc::new(epics_rs::asyn::manager::PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // C drvLoveInit(lovPort,serPort,serAddr)
    {
        app = app.register_startup_command(CommandDef::new(
            "drvLoveInit",
            vec![
                ArgDesc {
                    name: "lovPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "serAddr",
                    arg_type: ArgType::Int,
                },
            ],
            "drvLoveInit lovPort serPort serAddr",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let lov_port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("lovPort required".into()),
                };
                let ser_port = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("serPort required".into()),
                };
                let ser_addr = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("serAddr required".into()),
                };

                // C drvLoveInit prints each failure and returns -1 without
                // iocshSetError (drvLove.c:354-381).
                let port = match init_serial_port(&ser_port, ser_addr) {
                    Ok(port) => port,
                    Err(line) => {
                        ctx.println(&line);
                        ctx.println(&format!(
                            "drvLoveInit::failure to initialize serial port {ser_port}"
                        ));
                        return Ok(CommandOutcome::Continue);
                    }
                };

                let models = Arc::new(Mutex::new([Model::default(); K_INSTRMAX]));
                let registered = claim_port(&lov_port).and_then(|claim| {
                    let driver = LoveDriver::new(
                        &lov_port,
                        &ser_port,
                        port.clone(),
                        ser_addr,
                        COMMAND_TIMEOUT,
                        models.clone(),
                    )?;
                    let (runtime_handle, _actor_jh) =
                        create_port_runtime(&claim, driver, RuntimeConfig::default())?;
                    let love = runtime_handle.port_handle().clone();
                    claim.publish(love.clone(), ctx.db().scan_control().interrupt_gate())?;
                    Ok(love)
                });
                let Ok(love) = registered else {
                    ctx.println(&format!(
                        "drvLoveInit::failure to register love port {lov_port}"
                    ));
                    return Ok(CommandOutcome::Continue);
                };
                // C `exceptionCallbackAdd(pser->pasynUser,exceptCallback)` (drvLove.c:442).
                let follow = follow_serial_port(&port, ser_addr, &love);
                love::registry::register(&lov_port, models, follow);

                // C `setDefaultEos` (drvLove.c:640-660).
                let (lines, ok) = set_default_eos(&port, ser_addr, INPUT_EOS, OUTPUT_EOS);
                for line in &lines {
                    ctx.println(line);
                }
                if !ok {
                    ctx.println(&format!("drvLoveInit::failure to set {lov_port} EOS"));
                }

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // C drvLoveConfig(lovPort,addr,model)
    {
        app = app.register_startup_command(CommandDef::new(
            "drvLoveConfig",
            vec![
                ArgDesc {
                    name: "lovPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "addr",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "model",
                    arg_type: ArgType::String,
                },
            ],
            "drvLoveConfig lovPort addr model",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let lov_port = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("lovPort required".into()),
                };
                let addr = match &args[1] {
                    ArgValue::Int(n) => *n as i32,
                    _ => return Err("addr required".into()),
                };
                let model_str = match &args[2] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("model required".into()),
                };

                // C drvLoveConfig prints and returns -1 without iocshSetError
                // (drvLove.c:458-480).
                let Some(table) = love::registry::lookup(&lov_port) else {
                    ctx.println(&format!("drvLoveConfig::failure to locate port {lov_port}"));
                    return Ok(CommandOutcome::Continue);
                };
                let Some(model) = Model::parse(&model_str) else {
                    ctx.println(&format!("drvLoveConfig::unsupported model \"{model_str}\""));
                    return Ok(CommandOutcome::Continue);
                };
                // C indexes instr[addr-1] unchecked.
                if !(1..=K_INSTRMAX as i32).contains(&addr) {
                    ctx.println(&format!("drvLoveConfig: addr {addr} out of range"));
                    return Ok(CommandOutcome::Continue);
                }
                table.lock().unwrap()[(addr - 1) as usize] = model;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
