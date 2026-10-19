//! MicroEpsilon capaNCDT6200 IOC — L0 config port (upstream
//! `capaNCDT6200.proto` on stream-rs) + L1 native data port
//! (`capaNCDT6200Sup.c`, ported byte-for-byte).
//!
//! Usage:
//!   cargo run -p microepsilon-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::asyn::asyn_record;
use epics_rs::asyn::manager::PortManager;
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use microepsilon::data_driver::configure as data_configure;

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: microepsilon-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("MICROEPSILON", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    let (asyn_name, asyn_factory) = asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // asyn device support, plus `stream` and its iocsh commands.
    app = stream_rs::dev::register_app(app);

    let port_manager = Arc::new(PortManager::new());
    app = epics_rs::asyn::iocsh::register_asyn_commands(app, port_manager);

    // capaNCDT6200Configure(portName, IPaddress, IPport) -- C
    // `capaNCDT6200Configure` (`capaNCDT6200Sup.c:682-791`). Thin wrapper:
    // `microepsilon::data_driver::configure` builds both the outer L1 port
    // and its internal `_RBK` transport port, and retains both runtimes
    // itself.
    {
        app = app.register_startup_command(CommandDef::new(
            "capaNCDT6200Configure",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "IPaddress",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "IPport",
                    arg_type: ArgType::String,
                },
            ],
            "capaNCDT6200Configure portName IPaddress IPport",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("portName required".into()),
                };
                let ip_address = match &args[1] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("IPaddress required".into()),
                };
                let ip_port = match &args[2] {
                    ArgValue::String(s) => s.to_str()?.to_owned(),
                    _ => return Err("IPport required".into()),
                };

                data_configure(
                    &port_name,
                    &ip_address,
                    &ip_port,
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| format!("capaNCDT6200Configure: {e}"))?;

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
