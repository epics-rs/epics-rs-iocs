use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::interfaces::InterfaceType;
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
    let rt_slot = runtime.clone();
    ioc.register_startup_command(CommandDef::new(
        "pixiradConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "commandPort",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "dataPortNumber",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "statusPortNumber",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxDataPortBuffers",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxSizeX",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxSizeY",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "maxBuffers",
                arg_type: ArgType::Int,
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
        "pixiradConfig portName commandPort dataPortNumber statusPortNumber \
         maxDataPortBuffers maxSizeX maxSizeY [maxBuffers] [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let command_port = match &args[1] {
                ArgValue::String(s) if !s.is_empty() => s.to_str()?.to_owned(),
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

            let claim = match epics_rs::asyn::manager::claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            // C ignores the connect status (pixirad.cpp:471-472): a missing
            // command port leaves a user whose every command fails.
            let (command, _) =
                SyncIOHandle::connect_port(&command_port, 0, Duration::ZERO, InterfaceType::Octet);

            let rt = create_pixirad_detector(
                &claim,
                command,
                data_port,
                status_port,
                max_data_port_buffers as usize,
                max_size_x,
                max_size_y,
                max_memory,
            )
            .map_err(|e| format!("failed to create the Pixirad detector: {e}"))?;

            claim
                .publish(
                    rt.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
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

    ioc.register_startup_command(autocal_command(runtime.clone()));

    ioc.keep_alive(runtime);
}

// C's `pixiradAutoCal` looked the driver up by port name and called into it
// directly. Here the settings are written to an internal parameter, so the
// port actor stays the only thing that talks to the box.
fn autocal_command(rt_slot: Arc<std::sync::Mutex<Option<PixiradRuntime>>>) -> CommandDef {
    CommandDef::new(
        "pixiradAutoCal",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "ofs0",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "fs0",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "ofs2",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "fs1",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "fs2",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "Ibias",
                arg_type: ArgType::Int,
            },
            ArgDesc {
                name: "vbgMcalDAC",
                arg_type: ArgType::Int,
            },
        ],
        "pixiradAutoCal portName ofs0 fs0 ofs2 fs1 fs2 Ibias vbgMcalDAC",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            // C returns 0 whatever happens: a port findAsynPortDriver does not
            // find is skipped, and a refusal is only writeReadServer's trace.
            let port_name = match &args[0] {
                ArgValue::String(s) => s.to_str().unwrap_or_default(),
                _ => "",
            };
            let guard = rt_slot.lock().unwrap();
            let Some(rt) = guard
                .as_ref()
                .filter(|rt| rt.port_handle().port_name() == port_name)
            else {
                return Ok(CommandOutcome::Continue);
            };

            let settings = (1..=7)
                .map(|i| int_arg(args, i, 0).to_string())
                .collect::<Vec<_>>()
                .join(" ");

            // The connect status is dropped, as C's (pixirad.cpp:472).
            let (sync, _) = SyncIOHandle::connect_port(
                rt.port_handle().port_name(),
                0,
                Duration::from_secs(10),
                InterfaceType::Octet,
            );
            let _ = sync.write_octet(rt.params.autocal_conf, settings.as_bytes());
            Ok(CommandOutcome::Continue)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// pixirad.cpp:1522-1529: `pixiradAutoCal` returns 0 for a port it does
    /// not find and for a box that refuses; only writeReadServer's trace
    /// reports the refusal.
    #[epics_rs::base::epics_test]
    async fn auto_cal_never_fails_the_line() {
        use epics_rs::base::runtime::task::BlockingBridge;
        use epics_rs::base::server::database::PvDatabase;

        let (command, _) =
            SyncIOHandle::connect_port("PIXCAL_IP", 0, Duration::ZERO, InterfaceType::Octet);
        let rt = create_pixirad_detector(
            &epics_rs::asyn::registry::claim_port("PIXCAL").unwrap(),
            command,
            0,
            0,
            1,
            402,
            512,
            1 << 20,
        )
        .unwrap();
        let cmd = autocal_command(Arc::new(std::sync::Mutex::new(Some(rt))));
        let bridge = BlockingBridge::capture();
        std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            for port in ["PIXCAL", "NOPIXCAL", ""] {
                let mut args = vec![ArgValue::String(port.into())];
                args.extend((1..=7).map(ArgValue::Int));
                match cmd.handler.call(&args, &ctx) {
                    Ok(CommandOutcome::Continue) => {}
                    Err(e) => panic!("{port:?}: the command refused: {e}"),
                    Ok(_) => panic!("{port:?}: the command failed the line"),
                }
            }
        })
        .join()
        .unwrap();
    }
}
