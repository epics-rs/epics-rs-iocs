use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use pixirad::{PixiradRuntime, create_pixirad_detector};

fn int_arg(args: &[ArgValue], index: usize, default: i32) -> i32 {
    match args.get(index) {
        Some(ArgValue::Int(n)) => *n as i32,
        _ => default,
    }
}

/// Register `pixiradConfig` and `pixiradAutoCal` on an `AdIoc`.
///
/// C parity: `pixiradConfig(portName, commandPort, dataPortNumber,
/// statusPortNumber, maxDataPortBuffers, maxSizeX, maxSizeY, maxBuffers,
/// maxMemory, priority, stackSize)`. `maxBuffers`, `priority` and `stackSize`
/// are accepted for startup-script compatibility and ignored: the Rust pool is
/// bounded by memory alone and the runtime schedules the port actors itself.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADPIXIRAD", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<PixiradRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    let mgr = ioc.mgr().clone();
    let trace = ioc.trace().clone();
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "pixiradConfig",
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
                name: "dataPortNumber",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "statusPortNumber",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxDataPortBuffers",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxSizeX",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "maxSizeY",
                arg_type: ArgType::Int,
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
        "pixiradConfig portName commandPort dataPortNumber statusPortNumber \
         maxDataPortBuffers maxSizeX maxSizeY [maxBuffers] [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("portName required".into()),
            };
            let command_port = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("commandPort required".into()),
            };
            let data_port = int_arg(args, 2, 0);
            let status_port = int_arg(args, 3, 0);
            let max_data_port_buffers = int_arg(args, 4, 0);
            let max_size_x = int_arg(args, 5, 0);
            let max_size_y = int_arg(args, 6, 0);
            // 0 means "unlimited" in the C startup scripts.
            let max_memory = match args.get(8) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };

            let data_port: u16 = data_port
                .try_into()
                .map_err(|_| format!("dataPortNumber {data_port} is not a UDP port"))?;
            let status_port: u16 = status_port
                .try_into()
                .map_err(|_| format!("statusPortNumber {status_port} is not a UDP port"))?;
            if max_data_port_buffers <= 0 {
                return Err("maxDataPortBuffers must be at least 1".into());
            }

            let command = get_port(&command_port).ok_or_else(|| {
                format!(
                    "command port '{command_port}' not found \
                     (call drvAsynIPPortConfigure first)"
                )
            })?;

            println!(
                "pixiradConfig: port={port_name}, command={command_port}, \
                 data=UDP {data_port}, status=UDP {status_port}, \
                 size={max_size_x}x{max_size_y}"
            );

            let rt = create_pixirad_detector(
                &port_name,
                command.handle.clone(),
                data_port,
                status_port,
                max_data_port_buffers as usize,
                max_size_x,
                max_size_y,
                max_memory,
            )
            .map_err(|e| format!("failed to create the Pixirad detector: {e}"))?;

            epics_rs::asyn::asyn_record::register_port(
                &port_name,
                rt.port_handle().clone(),
                trace.clone(),
            )
            .map_err(|e| e.to_string())?;

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

    // C's `pixiradAutoCal` looked the driver up by port name and called into it
    // directly. Here the settings are written to an internal parameter, so the
    // port actor stays the only thing that talks to the box.
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "pixiradAutoCal",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
                optional: false,
            },
            ArgDesc {
                name: "ofs0",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "fs0",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "ofs2",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "fs1",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "fs2",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "Ibias",
                arg_type: ArgType::Int,
                optional: false,
            },
            ArgDesc {
                name: "vbgMcalDAC",
                arg_type: ArgType::Int,
                optional: false,
            },
        ],
        "pixiradAutoCal portName ofs0 fs0 ofs2 fs1 fs2 Ibias vbgMcalDAC",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.clone(),
                _ => return Err("portName required".into()),
            };

            let guard = rt_slot.lock().unwrap();
            let rt = guard
                .as_ref()
                .ok_or_else(|| format!("port '{port_name}' has no Pixirad detector"))?;

            let settings = (1..=7)
                .map(|i| int_arg(args, i, 0).to_string())
                .collect::<Vec<_>>()
                .join(" ");

            let sync =
                SyncIOHandle::from_handle(rt.port_handle().clone(), 0, Duration::from_secs(10));
            sync.write_octet(rt.params.autocal_conf, settings.as_bytes())
                .map_err(|e| format!("pixiradAutoCal failed: {e}"))?;
            Ok(CommandOutcome::Continue)
        },
    ));

    ioc.keep_alive(runtime);
}
