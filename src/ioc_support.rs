use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use epics_rs::ad_core::ioc::GenericDriverContext;

use crate::driver::{D435iColorRuntime, D435iDepthRuntime, create_d435i_detector};

/// Register the D435i configure command on an `AdIoc`.
///
/// After calling this, `d435iConfig(...)` can be used in st.cmd to create
/// a D435i detector. All records use standard asyn DTYPs handled by
/// the universal asyn device support factory.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default("ADD435I", env!("CARGO_MANIFEST_DIR"));

    let color_runtime: Arc<std::sync::Mutex<Option<D435iColorRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));
    let depth_runtime: Arc<std::sync::Mutex<Option<D435iDepthRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    {
        let mgr = ioc.mgr().clone();
        let trace = ioc.trace().clone();
        let c_rt = color_runtime.clone();
        let d_rt = depth_runtime.clone();
        ioc.register_startup_command(CommandDef::new(
            "d435iConfig",
            vec![
                ArgDesc { name: "portName", arg_type: ArgType::String, optional: false },
                ArgDesc { name: "serial", arg_type: ArgType::String, optional: true },
                ArgDesc { name: "maxSizeX", arg_type: ArgType::Int, optional: true },
                ArgDesc { name: "maxSizeY", arg_type: ArgType::Int, optional: true },
                ArgDesc { name: "maxMemory", arg_type: ArgType::Int, optional: true },
            ],
            "d435iConfig portName [serial] [maxSizeX] [maxSizeY] [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let serial = match args.get(1) {
                    Some(ArgValue::String(s)) => s.clone(),
                    _ => String::new(),
                };
                let size_x = match args.get(2) { Some(ArgValue::Int(n)) => *n as i32, _ => 1920 };
                let size_y = match args.get(3) { Some(ArgValue::Int(n)) => *n as i32, _ => 1080 };
                let max_memory = match args.get(4) { Some(ArgValue::Int(n)) => *n as usize, _ => 100_000_000 };

                let depth_port_name = format!("{port_name}_DEPTH");
                let pc_port_name = format!("{port_name}_PC");

                println!("d435iConfig: port={port_name}, serial={serial}, size={size_x}x{size_y}, maxMemory={max_memory}");

                let (color_rt, depth_rt) = create_d435i_detector(&port_name, &serial, size_x, size_y, max_memory)
                    .map_err(|e| format!("failed to create D435i detector: {e}"))?;

                let c_port_handle = color_rt.port_handle().clone();
                let d_port_handle = depth_rt.port_handle().clone();

                epics_rs::asyn::asyn_record::register_port(&port_name, c_port_handle, trace.clone());
                epics_rs::asyn::asyn_record::register_port(&depth_port_name, d_port_handle, trace.clone());

                // Register color port as the primary driver context (pool + fan-out).
                mgr.set_driver(Arc::new(GenericDriverContext::new(
                    color_rt.pool().clone(),
                    color_rt.array_output().clone(),
                    &port_name,
                    mgr.wiring(),
                )));

                // Expose depth port + PC as additional NDArray sources so plugins can
                // wire to them via NDARRAY_PORT=RS1_DEPTH / RS1_PC.
                mgr.wiring().register_output(&depth_port_name, depth_rt.array_output().clone());
                mgr.wiring().register_output(&pc_port_name, color_rt.pc_output().clone());

                *c_rt.lock().unwrap() = Some(color_rt);
                *d_rt.lock().unwrap() = Some(depth_rt);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Keep runtimes alive for the IOC's lifetime.
    ioc.keep_alive(color_runtime);
    ioc.keep_alive(depth_runtime);
}
