//! Amptek DP5/PX5/DP5G/MCA8000D/TB5/DP5-X MCA IOC, modeled on `mca-ioc` but
//! driving a real (network-transport) vendor MCA instead of the demo signal
//! source: `mca` record ↔ `devMcaAsyn` ↔ `mca_amptek::driver::AmptekDriver`
//! over UDP.
//!
//! Usage:
//!   cargo run -p mca-amptek-ioc -- iocs/mca-amptek-ioc/st.cmd
//!
//! iocsh commands:
//! * `drvAmptekConfigure(portName, interface, addressInfo, directMode)` — C
//!   `drvAmptekConfigure` (`drvAmptek.cpp:1095-1098`). `interface` must be
//!   `0` (Ethernet); USB (`1`) is feasibility-gated (see the `mca-amptek`
//!   crate doc) and rejected by [`mca_amptek::driver::AmptekDriver::new`].
//!   `directMode` is optional, defaulting to `0` (broadcast-discovery
//!   connect) like the upstream iocsh call, which omits it entirely in
//!   `st_base.cmd`.

use std::sync::{Arc, Mutex};

use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca::dev_mca_asyn;
use mca::interface::ASYN_MCA_DTYP;
use mca_amptek::driver::AmptekDriver;

/// Every asyn port this IOC creates, kept alive for the life of the process
/// (mirrors `mca-ioc`'s `Ports`; see its `main.rs`).
type Ports = Arc<Mutex<Vec<PortRuntimeHandle>>>;

fn arg_string(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Some(s.clone()),
        _ => None,
    }
}

fn arg_int(args: &[ArgValue], i: usize) -> Option<i64> {
    match args.get(i) {
        Some(ArgValue::Int(n)) => Some(*n),
        _ => None,
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: mca-amptek-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MCA_AMPTEK_IOC", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    let (mca_name, mca_factory) = mca_rs::mca_record_factory();
    app = app.register_record_type(mca_name, move || mca_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // drvAmptekConfigure(portName, interface, addressInfo, directMode)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "drvAmptekConfigure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "interface",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "addressInfo",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "directMode",
                    arg_type: ArgType::Int,
                    optional: true,
                },
            ],
            "drvAmptekConfigure portName interface addressInfo [directMode] - \
             create an Amptek DP5 asyn port (interface must be 0, Ethernet)",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = arg_string(args, 0).ok_or("portName required")?;
                let interface = arg_int(args, 1).ok_or("interface required")? as i32;
                let address_info = arg_string(args, 2).ok_or("addressInfo required")?;
                let direct_mode = arg_int(args, 3).unwrap_or(0) != 0;

                let driver = AmptekDriver::new(&port_name, interface, &address_info, direct_mode)
                    .map_err(|e| format!("drvAmptekConfigure: {e}"))?;
                let (runtime_handle, _actor) =
                    create_port_runtime(driver, RuntimeConfig::default());
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime_handle.port_handle().clone(),
                    trace.clone(),
                )
                .map_err(|e| format!("drvAmptekConfigure: {e}"))?;
                ports
                    .lock()
                    .expect("port list poisoned")
                    .push(runtime_handle);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Binds every mca record with DTYP "asynMCA" to the asyn port its own
    // INP link (`@asyn(portName,addr)`) names.
    app = app.register_dynamic_device_support(|ctx: &DeviceSupportContext| {
        if ctx.dtyp != ASYN_MCA_DTYP {
            return None;
        }
        let dev = dev_mca_asyn::connect(ctx.inp)?;
        Some(Box::new(dev) as Box<dyn DeviceSupport>)
    });

    app.startup_script(&script)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
