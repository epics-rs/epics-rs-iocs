use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use ad_url::{URLRuntime, create_url_detector};

/// Register the URL driver configure command on an `AdIoc`.
///
/// After calling this, `URLDriverConfig(...)` can be used in st.cmd to
/// create a URL detector — same command name as C++ `ADURL`'s
/// `URLDriverConfig` iocshFunc, for drop-in st.cmd compatibility.
///
/// `maxBuffers`, `priority` and `stackSize` are accepted (matching the C++
/// signature) but unused: `ADDriverBase::new` takes only a memory budget (no
/// buffer-count cap), and the acquisition task always runs on its own
/// `rt::run_thread_named` OS thread at default priority/stack size — the
/// same framework limitation the rest of this workspace's AD ports (e.g.
/// d435i) already accept.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADURLIOC", env!("CARGO_MANIFEST_DIR"));

    let runtime: Arc<std::sync::Mutex<Option<URLRuntime>>> = Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let rt_slot = runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "URLDriverConfig",
            vec![
                ArgDesc {
                    name: "portName",
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
            "URLDriverConfig portName [maxBuffers] [maxMemory] [priority] [stackSize]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let max_memory = match args.get(2) {
                    Some(ArgValue::Int(n)) => *n as usize,
                    _ => 0,
                };

                println!("URLDriverConfig: port={port_name}, maxMemory={max_memory}");

                let det_runtime = create_url_detector(&port_name, max_memory)
                    .map_err(|e| format!("failed to create URL detector: {e}"))?;

                let port_handle = det_runtime.port_handle().clone();
                epics_rs::asyn::asyn_record::register_port(&port_name, port_handle, trace.clone())
                    .map_err(|e| e.to_string())?;

                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    det_runtime.pool().clone(),
                    det_runtime.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                *rt_slot.lock().unwrap() = Some(det_runtime);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    ioc.keep_alive(runtime);
}
