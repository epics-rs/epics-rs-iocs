use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use ad_core_rs::ioc::GenericDriverContext;
use ad_core_rs::plugin::channel::NDArrayOutput;

use ad_pylon::{ADPylon, PylonRuntime, create_pylon};

/// Register `ADPylonConfig` on an `AdIoc`.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADPYLON", env!("CARGO_MANIFEST_DIR"));

    let runtimes: Arc<std::sync::Mutex<Vec<PylonRuntime>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));

    let mgr = ioc.mgr().clone();
    let rt = runtimes.clone();
    ioc.register_startup_command(CommandDef::new(
        "ADPylonConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "cameraId",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "priority",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "stackSize",
                arg_type: ArgType::Int,
            },
        ],
        "ADPylonConfig portName cameraId [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let string = |i: usize, name: &str| match args.get(i) {
                Some(ArgValue::String(s)) => Ok(s.clone()),
                // A bare serial number or index parses as an integer.
                Some(ArgValue::Int(n)) => Ok(n.to_string()),
                _ => Err(format!("{name} required")),
            };
            let port_name = string(0, "portName")?;
            let camera_id = string(1, "cameraId")?;
            let max_memory = match args.get(2) {
                Some(ArgValue::Int(n)) => *n as usize,
                _ => 0,
            };
            // priority and stackSize size an EPICS port thread; the port
            // here is an actor and has neither.

            let runtime = create_pylon(&port_name, &camera_id, max_memory, NDArrayOutput::new())
                .map_err(|e| format!("failed to create ADPylon: {e}"))?;

            // The CA client behind the `EPICS_PV` entries of an
            // NDAttributesFile; C's PVAttribute uses the IOC's CA context.
            let (client, handle) = ctx.block_on(async {
                (
                    epics_rs::ca::client::CaClient::new().await,
                    tokio::runtime::Handle::current(),
                )
            });
            let client = Arc::new(client.map_err(|e| format!("CA client: {e}"))?);
            runtime
                .port_handle()
                .with_driver_blocking(move |driver: &mut ADPylon| {
                    driver.ad.attributes.set_ca_client(client, handle)
                })
                .map_err(|e| e.to_string())?;

            asyn_rs::asyn_record::register_port(&port_name, runtime.port_handle().clone())
                .map_err(|e| e.to_string())?;

            mgr.set_driver(Arc::new(GenericDriverContext::new(
                runtime.pool().clone(),
                runtime.array_output().clone(),
                &port_name,
                mgr.wiring(),
            )));

            rt.lock().unwrap().push(runtime);
            Ok(CommandOutcome::Continue)
        },
    ));

    ioc.register_startup_command(CommandDef::new(
        "genicamShowFeature",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "featureName",
                arg_type: ArgType::String,
            },
        ],
        "genicamShowFeature portName featureName",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let (Some(ArgValue::String(port_name)), Some(ArgValue::String(feature_name))) =
                (args.first(), args.get(1))
            else {
                return Err("portName and featureName required".to_string());
            };
            let Some(port) = asyn_rs::registry::get_port(port_name) else {
                ctx.println(&format!(
                    "ADGenICam::showFeature cannot find port {port_name}"
                ));
                return Ok(CommandOutcome::Continue);
            };
            let feature_name = feature_name.clone();
            let text = port
                .handle
                .with_driver_blocking(move |driver: &mut ADPylon| {
                    driver.show_feature(&feature_name)
                })
                .map_err(|e| e.to_string())?;
            ctx.println(text.trim_end_matches('\n'));
            Ok(CommandOutcome::Continue)
        },
    ));

    // Keep the runtimes alive for the IOC's lifetime.
    ioc.keep_alive(runtimes);
}
