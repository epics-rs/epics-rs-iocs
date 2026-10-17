//! `eigerDetectorConfig` — the iocsh command that creates an Eiger port
//! (C `eigerDetectorConfig`, eigerDetector.cpp:2200).

use std::sync::Arc;

use epics_rs::ad_core::ioc::{GenericDriverContext, PluginManager};
use epics_rs::base::server::iocsh::registry::*;

use eiger::{EigerRuntime, create_eiger_detector};

/// NDArray fan-out names.
///
/// C publishes the same NDArray on several asyn *addresses* of one port:
/// address 0 (every frame), address `threshold + 1` (per-threshold streams) and
/// address 10 (the monitor image). epics-rs routes NDArrays by *port name*
/// through the plugin wiring registry — `NDArrayAddr` is parsed but never
/// consulted — so each of C's addresses becomes its own named output:
///
/// | C asyn address | port name here  |
/// |----------------|-----------------|
/// | 0              | `$(PORT)`       |
/// | 1..4           | `$(PORT)_TH1..4`|
/// | 10             | `$(PORT)_MON`   |
fn threshold_port(port: &str, n: usize) -> String {
    format!("{port}_TH{n}")
}

fn monitor_port(port: &str) -> String {
    format!("{port}_MON")
}

/// Register the Eiger configure command on an `AdIoc`.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADEIGER", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Vec<EigerRuntime>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));
    ioc.register_startup_command(config_command(ioc.mgr().clone(), runtime.clone()));

    // Keep the runtimes (and their tasks) alive for the IOC's lifetime.
    ioc.keep_alive(runtime);
}

/// `eigerDetectorConfig`, keeping each runtime it makes in `rt`.
fn config_command(
    mgr: Arc<PluginManager>,
    rt: Arc<std::sync::Mutex<Vec<EigerRuntime>>>,
) -> CommandDef {
    CommandDef::new(
        "eigerDetectorConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "serverHostname",
                arg_type: ArgType::String,
            },
            // C also takes maxBuffers, priority and stackSize. The Rust
            // NDArray pool is not buffer-count limited and the tasks are
            // OS threads with the runtime's own priority, so only
            // maxMemory carries over.
            ArgDesc {
                name: "maxMemory",
                arg_type: ArgType::Int,
            },
        ],
        "eigerDetectorConfig portName serverHostname [maxMemory]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let port_name = match &args[0] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
                _ => return Err("portName required".into()),
            };
            let hostname = match &args[1] {
                ArgValue::String(s) => s.to_str()?.to_owned(),
                _ => return Err("serverHostname required".into()),
            };
            let max_memory = match args.get(2) {
                Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                _ => 0,
            };

            let claim = match epics_rs::asyn::manager::claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };

            let runtime = create_eiger_detector(&claim, &hostname, max_memory)
                .map_err(|e| format!("failed to create the Eiger detector: {e}"))?;

            claim
                .publish(
                    runtime.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

            // Address 0: the driver context (pool + main fan-out).
            mgr.set_driver(Arc::new(GenericDriverContext::new(
                runtime.pool.clone(),
                runtime.array_output().clone(),
                &port_name,
                mgr.wiring(),
            )));

            // Addresses 1..4 and 10.
            for (i, output) in runtime.outputs.thresholds.iter().enumerate() {
                mgr.wiring()
                    .register_output(&threshold_port(&port_name, i + 1), output.clone());
            }
            mgr.wiring()
                .register_output(&monitor_port(&port_name), runtime.outputs.monitor.clone());

            let thrown = runtime.thrown;
            rt.lock().unwrap().push(runtime);
            Ok(match thrown {
                Some(what) => ctx.cxx_error(what),
                None => CommandOutcome::Continue,
            })
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// eigerDetector.cpp:190-211, restApi.cpp:244: `mApi` throws after the
    /// ADDriver base registered the port, which stays for the plugins.
    #[epics_rs::base::epics_test]
    async fn a_thrown_constructor_leaves_the_port_registered() {
        use epics_rs::base::runtime::task::BlockingBridge;
        use epics_rs::base::server::database::PvDatabase;

        let mgr = PluginManager::new();
        let rt = Arc::new(std::sync::Mutex::new(Vec::new()));
        let cmd = config_command(mgr.clone(), rt.clone());
        let args = vec![
            ArgValue::String("EIGTHROWN".into()),
            ArgValue::String("no-such-host.invalid".into()),
        ];
        let bridge = BlockingBridge::capture();
        let outcome = std::thread::spawn(move || {
            let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
            cmd.handler.call(&args, &ctx)
        })
        .join()
        .unwrap();
        match outcome {
            Ok(CommandOutcome::Failed) => {}
            Err(e) => panic!("the command refused: {e}"),
            Ok(_) => panic!("the command did not fail the line"),
        }
        assert!(mgr.driver().is_ok());
        assert!(
            epics_rs::asyn::manager::PortManager::global()
                .find_port_handle("EIGTHROWN")
                .is_ok()
        );
        assert_eq!(rt.lock().unwrap()[0].thrown, Some("invalid hostname"));
    }

    #[test]
    fn the_fan_out_names_follow_the_c_addresses() {
        assert_eq!(threshold_port("EIG1", 1), "EIG1_TH1");
        assert_eq!(threshold_port("EIG1", 4), "EIG1_TH4");
        assert_eq!(monitor_port("EIG1"), "EIG1_MON");
    }
}
