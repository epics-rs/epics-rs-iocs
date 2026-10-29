use std::sync::Arc;

use epics_rs::base::server::iocsh::registry::*;

use ad_core_rs::ioc::GenericDriverContext;
use ad_core_rs::plugin::channel::NDArrayOutput;

use ad_pylon::{ADPylon, PylonRuntime, create_pylon};

/// Register `ADPylonConfig` on an `AdIoc`.
pub fn register(ioc: &mut epics_rs::ad_plugins::ioc::AdIoc) {
    epics_rs::base::runtime::env::set_default_path("ADPYLON", env!("CARGO_MANIFEST_DIR"));

    let runtimes: Arc<std::sync::Mutex<Vec<PylonRuntime>>> =
        Arc::new(std::sync::Mutex::new(Vec::new()));

    let mgr = ioc.mgr().clone();
    let rt = runtimes.clone();
    ioc.register_startup_command(CommandDef::new(
        "ADPylonConfig",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "cameraId",
                arg_type: ArgType::String,
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
        "ADPylonConfig portName cameraId [maxMemory] [priority] [stackSize]",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let string = |i: usize, name: &str| match args.get(i) {
                Some(ArgValue::String(s)) => {
                    s.to_str().map(str::to_owned).map_err(|e| e.to_string())
                }
                // A bare serial number or index parses as an integer.
                Some(ArgValue::Int(n)) => Ok(n.to_string()),
                _ => Err(format!("{name} required")),
            };
            let port_name = string(0, "portName")?;
            let camera_id = string(1, "cameraId")?;
            let max_memory = match args.get(2) {
                Some(ArgValue::Int(n)) => *n as usize,
                _ => 0,
            };
            // priority and stackSize size an EPICS port thread; the port
            // here is an actor and has neither.

            let claim = match epics_rs::asyn::manager::claim_port_driver(&port_name) {
                Ok(claim) => claim,
                Err(e) => return Ok(ctx.cxx_error(e)),
            };
            let runtime = create_pylon(&claim, &camera_id, max_memory, NDArrayOutput::new())
                .map_err(|e| format!("failed to create ADPylon: {e}"))?;

            // The CA client behind the `EPICS_PV` entries of an
            // NDAttributesFile; C's PVAttribute uses the IOC's CA context.
            let (client, handle) = ctx.block_on(async {
                (
                    epics_rs::ca::client::CaClient::new().await,
                    tokio::runtime::Handle::current(),
                )
            });
            let client = Arc::new(client.map_err(|e| format!("CA client: {e}"))?);
            runtime
                .port_handle()
                .with_driver_blocking(move |driver: &mut ADPylon| {
                    driver
                        .ad
                        .owner
                        .attributes()
                        .lock()
                        .set_ca_client(client, handle)
                })
                .map_err(|e| e.to_string())?;

            claim
                .publish(
                    runtime.port_handle().clone(),
                    ctx.db().scan_control().interrupt_gate(),
                )
                .map_err(|e| e.to_string())?;

            mgr.set_driver(Arc::new(GenericDriverContext::new(
                runtime.pool().clone(),
                runtime.array_output().clone(),
                &port_name,
                mgr.wiring(),
            )));

            rt.lock().unwrap().push(runtime);
            Ok(CommandOutcome::Continue)
        },
    ));

    ioc.register_startup_command(CommandDef::new(
        "genicamShowFeature",
        vec![
            ArgDesc {
                name: "portName",
                arg_type: ArgType::String,
            },
            ArgDesc {
                name: "featureName",
                arg_type: ArgType::String,
            },
        ],
        "genicamShowFeature portName featureName",
        move |args: &[ArgValue], ctx: &CommandContext| {
            let (Some(ArgValue::String(port_name)), Some(ArgValue::String(feature_name))) =
                (args.first(), args.get(1))
            else {
                return Err("portName and featureName required".to_string());
            };
            let text = show_feature(port_name.to_str()?, feature_name.to_str()?);
            ctx.println(text.trim_end_matches('\n'));
            Ok(CommandOutcome::Continue)
        },
    ));

    // Keep the runtimes alive for the IOC's lifetime.
    ioc.keep_alive(runtimes);
}

/// C `genicamShowFeature`: `findDerivedAsynPortDriver<ADGenICam>` is a
/// `dynamic_cast`, so a port that is not this driver reads as no port at all
/// (ADGenICam.cpp:655-659). It checks no interface, so the bare registry
/// lookup is the equivalent.
fn show_feature(port_name: &str, feature_name: &str) -> String {
    let feature_name = feature_name.to_owned();
    asyn_rs::registry::get_port(port_name)
        .and_then(|port| {
            port.handle
                .with_driver_blocking(move |driver: &mut ADPylon| {
                    driver.show_feature(&feature_name)
                })
                .ok()
        })
        .unwrap_or_else(|| format!("ADGenICam::showFeature cannot find port {port_name}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};

    struct Other(PortDriverBase);

    impl PortDriver for Other {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    /// A registered port that is not an ADPylon is not found, as C's
    /// `dynamic_cast` gives no driver; it is not a command error.
    #[test]
    fn a_port_of_another_driver_is_not_found() {
        let name = "pylonShowOther";
        let base = PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::OCTET,
            InterfaceMask::OCTET,
        );
        let (_rt, _actor) = create_port_runtime(
            &claim_port(name).unwrap(),
            Other(base),
            RuntimeConfig::default(),
        )
        .unwrap();
        assert_eq!(
            show_feature(name, "Width"),
            format!("ADGenICam::showFeature cannot find port {name}")
        );
    }
}
