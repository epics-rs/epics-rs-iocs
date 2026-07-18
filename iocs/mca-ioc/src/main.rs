//! MCA (multichannel analyzer) demo IOC.
//!
//! Registers the `mca` record type (`mca-rs`) and this workspace's
//! `drivers/mca` asyn device support (`mca::dev_mca_asyn::DevMcaAsyn`, DTYP
//! `"asynMCA"`), then boots a `DemoSourceConfig` signal source and an
//! `initFastSweep` software MCA sweeping it — proving the `mca` record ↔
//! `DevMcaAsyn` ↔ asyn MCA driver path end-to-end.
//!
//! Usage:
//!   cargo run -p mca-ioc -- iocs/mca-ioc/st.cmd
//!
//! iocsh commands:
//! * `DemoSourceConfig(portName, maxSignals, period)` — not an upstream
//!   command; this IOC's own stand-in signal source (see
//!   [`demo_source::connect`]'s doc). `period` is in seconds.
//! * `initFastSweep(portName, inputName, maxSignals, maxPoints, dataString,
//!   intervalString)` — C `initFastSweep` (`drvFastSweep.cpp:57`).
//!   `dataString`/`intervalString` are optional (default `"DATA"`/
//!   `"SCAN_PERIOD"`).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::{PortRuntimeHandle, create_port_runtime};
use epics_rs::asyn::trace::TraceManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::{DeviceSupportContext, IocApplication};

use mca::dev_mca_asyn;
use mca::fastsweep::FastSweepDriver;
use mca::interface::ASYN_MCA_DTYP;

mod demo_source;

/// Every asyn port this IOC creates, kept alive for the life of the process
/// (mirrors `ur-robot-ioc`'s `Ports`; see its `main.rs`).
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

fn arg_double(args: &[ArgValue], i: usize) -> Option<f64> {
    match args.get(i) {
        Some(ArgValue::Double(v)) => Some(*v),
        Some(ArgValue::Int(v)) => Some(*v as f64),
        _ => None,
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: mca-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MCA_IOC", env!("CARGO_MANIFEST_DIR"));

    let trace = Arc::new(TraceManager::new());
    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    let (mca_name, mca_factory) = mca_rs::mca_record_factory();
    app = app.register_record_type(mca_name, move || mca_factory());
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);

    // DemoSourceConfig(portName, maxSignals, period)
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "DemoSourceConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxSignals",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "period",
                    arg_type: ArgType::Double,
                    optional: false,
                },
            ],
            "DemoSourceConfig portName maxSignals period - start the demo signal source",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = arg_string(args, 0).ok_or("portName required")?;
                let max_signals = arg_int(args, 1).ok_or("maxSignals required")? as usize;
                let period = arg_double(args, 2).ok_or("period required")?;

                let runtime_handle =
                    demo_source::connect(&port_name, max_signals, Duration::from_secs_f64(period))
                        .map_err(|e| format!("DemoSourceConfig: {e}"))?;
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime_handle.port_handle().clone(),
                    trace.clone(),
                )
                .map_err(|e| format!("DemoSourceConfig: {e}"))?;
                ports
                    .lock()
                    .expect("port list poisoned")
                    .push(runtime_handle);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // initFastSweep(portName,inputName,maxSignals,maxPoints,dataString,intervalString)
    // -- C `initFastSweep` (`drvFastSweep.cpp:57`).
    {
        let ports = ports.clone();
        let trace = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "initFastSweep",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "inputName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "maxSignals",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "maxPoints",
                    arg_type: ArgType::Int,
                    optional: false,
                },
                ArgDesc {
                    name: "dataString",
                    arg_type: ArgType::String,
                    optional: true,
                },
                ArgDesc {
                    name: "intervalString",
                    arg_type: ArgType::String,
                    optional: true,
                },
            ],
            "initFastSweep portName inputName maxSignals maxPoints [dataString] [intervalString]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = arg_string(args, 0).ok_or("portName required")?;
                let input_name = arg_string(args, 1).ok_or("inputName required")?;
                let max_signals = arg_int(args, 2).ok_or("maxSignals required")? as usize;
                let max_points = arg_int(args, 3).ok_or("maxPoints required")? as usize;
                let data_string = arg_string(args, 4).unwrap_or_default();
                let interval_string = arg_string(args, 5).unwrap_or_default();

                let (driver, subscriptions) = FastSweepDriver::connect(
                    &port_name,
                    &input_name,
                    max_signals,
                    max_points,
                    &data_string,
                    &interval_string,
                )
                .map_err(|e| format!("initFastSweep: {e}"))?;
                let (runtime_handle, _actor) =
                    create_port_runtime(driver, RuntimeConfig::default());
                mca::fastsweep::start(runtime_handle.port_handle().clone(), subscriptions);
                epics_rs::asyn::asyn_record::register_port(
                    &port_name,
                    runtime_handle.port_handle().clone(),
                    trace.clone(),
                )
                .map_err(|e| format!("initFastSweep: {e}"))?;
                ports
                    .lock()
                    .expect("port list poisoned")
                    .push(runtime_handle);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Binds every mca record with DTYP "asynMCA" to the asyn port its own
    // INP link (`@asyn(portName,addr)`) names -- unlike scalerRecord
    // (scaler974-ioc), mcaRecord's INP is now mirrored into
    // DeviceSupportContext (epics-base-rs 0.24.0), so no single-instance
    // registry workaround is needed here.
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
