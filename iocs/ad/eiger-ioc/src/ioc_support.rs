//! `eigerDetectorConfig` — the iocsh command that creates an Eiger port
//! (C `eigerDetectorConfig`, eigerDetector.cpp:2200).

use std::sync::Arc;

use epics_rs::ad_core::ioc::GenericDriverContext;
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

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "eigerDetectorConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "serverHostname",
                    arg_type: ArgType::String,
                    optional: false,
                },
                // C also takes maxBuffers, priority and stackSize. The Rust
                // NDArray pool is not buffer-count limited and the tasks are
                // OS threads with the runtime's own priority, so only
                // maxMemory carries over.
                ArgDesc {
                    name: "maxMemory",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "eigerDetectorConfig portName serverHostname [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let hostname = match &args[1] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("serverHostname required".into()),
                };
                let max_memory = match args.get(2) {
                    Some(ArgValue::Int(n)) if *n > 0 => *n as usize,
                    _ => 0,
                };

                println!(
                    "eigerDetectorConfig: port={port_name}, host={hostname}, maxMemory={max_memory}"
                );

                let runtime = create_eiger_detector(&port_name, &hostname, max_memory)
                    .map_err(|e| format!("failed to create the Eiger detector: {e}"))?;

                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime.port_handle().clone(),
                    trace.clone(),
                );

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

                rt.lock().unwrap().push(runtime);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Keep the runtimes (and their tasks) alive for the IOC's lifetime.
    ioc.keep_alive(runtime);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_fan_out_names_follow_the_c_addresses() {
        assert_eq!(threshold_port("EIG1", 1), "EIG1_TH1");
        assert_eq!(threshold_port("EIG1", 4), "EIG1_TH4");
        assert_eq!(monitor_port("EIG1"), "EIG1_MON");
    }
}
