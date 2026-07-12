use std::sync::Arc;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use merlin::{DetectorType, MerlinRuntime, create_merlin_detector};

/// Register `merlinDetectorConfig` on an `AdIoc`.
///
/// C parity: `merlinDetectorConfig(portName, LabviewCmdPort, LabviewDataPort,
/// maxSizeX, maxSizeY, detectorType, maxBuffers, maxMemory, priority,
/// stackSize)`. `maxBuffers`, `priority` and `stackSize` are accepted for
/// startup-script compatibility and ignored: the Rust pool is bounded by
/// memory alone and the runtime schedules the port actors itself.
///
/// C declared this command with an argument *count* of 9 while passing a
/// 10-entry argument array and reading `args[9]`, so `stackSize` was read
/// past the end of iocsh's parsed argument buffer.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADMERLIN", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<MerlinRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "merlinDetectorConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "LabviewCmdPort",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "LabviewDataPort",
                    arg_type: ArgType::String,
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
                    name: "detectorType",
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
            "merlinDetectorConfig portName cmdPort dataPort maxSizeX maxSizeY detectorType \
             [maxBuffers] [maxMemory] [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) if !s.is_empty() => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let cmd_port = match &args[1] {
                    ArgValue::String(s) if !s.is_empty() => s.clone(),
                    _ => return Err("LabviewCmdPort required".into()),
                };
                let data_port = match &args[2] {
                    ArgValue::String(s) if !s.is_empty() => s.clone(),
                    _ => return Err("LabviewDataPort required".into()),
                };
                let size_x = match args.get(3) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => return Err("maxSizeX required".into()),
                };
                let size_y = match args.get(4) {
                    Some(ArgValue::Int(n)) => *n as i32,
                    _ => return Err("maxSizeY required".into()),
                };
                let det_type = match args.get(5) {
                    Some(ArgValue::Int(n)) => DetectorType::from_i32(*n as i32)
                        .ok_or_else(|| format!("unknown detectorType {n}"))?,
                    _ => return Err("detectorType required".into()),
                };
                // 0 means "unlimited" in the C startup scripts.
                let max_memory = match args.get(7) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                let cmd = get_port(&cmd_port).ok_or_else(|| {
                    format!(
                        "command port '{cmd_port}' not found (call drvAsynIPPortConfigure first)"
                    )
                })?;
                let data = get_port(&data_port).ok_or_else(|| {
                    format!("data port '{data_port}' not found (call drvAsynIPPortConfigure first)")
                })?;

                println!(
                    "merlinDetectorConfig: port={port_name}, cmd={cmd_port}, data={data_port}, \
                     size={size_x}x{size_y}, type={det_type:?}"
                );

                let rt = create_merlin_detector(
                    &port_name,
                    cmd.handle.clone(),
                    data.handle.clone(),
                    size_x,
                    size_y,
                    det_type,
                    max_memory,
                )
                .map_err(|e| format!("failed to create the Merlin detector: {e}"))?;

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

    ioc.keep_alive(runtime);
}
