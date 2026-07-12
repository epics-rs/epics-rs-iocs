use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use photonii::{PII_UTIL, PhotonIIRuntime, create_photonii_detector};

/// Register `PhotonIIConfig` and `p2util` on an `AdIoc`.
///
/// C parity: `PhotonIIConfig(portName, commandPort, maxBuffers, maxMemory,
/// priority, stackSize)` and `p2util(portName, command)`. `maxBuffers`,
/// `priority` and `stackSize` are accepted for startup-script compatibility and
/// ignored: the Rust pool is bounded by memory alone and the runtime schedules
/// the port actors itself.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADPHOTONII", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<PhotonIIRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "PhotonIIConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "commandPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxBuffers",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "maxMemory",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "priority",
                    arg_type: ArgType::Int,
                    optional: true,
                },
                ArgDesc {
                    name: "stackSize",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "PhotonIIConfig portName commandPort [maxBuffers] [maxMemory] [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) if !s.is_empty() => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let command_port = match &args[1] {
                    ArgValue::String(s) if !s.is_empty() => s.clone(),
                    _ => return Err("commandPort required".into()),
                };
                // 0 means "unlimited" in the C startup scripts.
                let max_memory = match args.get(3) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                let cmd = get_port(&command_port).ok_or_else(|| {
                    format!(
                        "command port '{command_port}' not found \
                         (call drvAsynIPPortConfigure first)"
                    )
                })?;

                println!("PhotonIIConfig: port={port_name}, p2util={command_port}");

                let rt = create_photonii_detector(&port_name, cmd.handle.clone(), max_memory)
                    .map_err(|e| format!("failed to create the PhotonII detector: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    rt.port_handle().clone(),
                    trace.clone(),
                );

                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    rt.pool().clone(),
                    rt.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                *rt_slot.lock().unwrap() = Some(rt);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // p2util(portName, command): send one raw command line to p2util.
    //
    // C looked the driver up with findAsynPortDriver() and called through the
    // returned pointer without checking it for NULL, so a typo in the port name
    // crashed the IOC at startup. Here an unknown port is an error the startup
    // script reports.
    ioc.register_startup_command(CommandDef::new(
        "p2util",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "command",
                arg_type: ArgType::String,
                optional: false,
            },
        ],
        "p2util portName command",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("portName required".into()),
            };
            let command = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("command required".into()),
            };

            let port = get_port(&port_name).ok_or_else(|| {
                format!("port '{port_name}' not found (call PhotonIIConfig first)")
            })?;
            let sync = SyncIOHandle::from_handle(port.handle.clone(), 0, Duration::from_secs(5));
            let reason = sync
                .drv_user_create(PII_UTIL)
                .map_err(|e| format!("port '{port_name}' is not a PhotonII driver: {e}"))?;
            sync.write_octet(reason, command.as_bytes())
                .map_err(|e| format!("p2util '{command}' failed: {e}"))?;
            Ok(CommandOutcome::Continue)
        },
    ));

    ioc.keep_alive(runtime);
}
