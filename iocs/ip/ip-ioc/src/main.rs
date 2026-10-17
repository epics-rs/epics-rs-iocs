//! IOC for the serial devices of the EPICS `ip` module.
//!
//! Usage:
//!   cargo run -p ip-ioc -- iocs/ip/ip-ioc/st.cmd
//!
//! Each device family gets one iocsh command that puts an asyn port on top of an
//! octet port created earlier by `drvAsynSerialPortConfigure` /
//! `drvAsynIPPortConfigure`:
//!
//! ```text
//! TelevacConfig(port, octetPort, numStations, numRelays, [pollPeriod])
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::asyn::registry::{PortClaim, claim_port};
use epics_rs::base::error::CaResult;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::ca::server::ioc_app::IocApplication;

use ip_devices::runtime::IpPortRuntime;
use ip_devices::televac::create_televac;

/// `ipSupport.dbd`'s `devAiMKS` (`"HPS SensaVac 937"`), `dev<Type>MPC`
/// (`"asyn MPC"`), `dev<Type>TPG261` (`"asyn TPG261"`), `dev<Xx>StrParm`
/// (`"asyn <xx> stringParm"`), `dev<Xx>Eurotherm` (`"asyn <xx> Eurotherm"`)
/// and `devAiHeidND261` (`"asyn ai HeidND261"`) rows; their dsets
/// are `{6, 0, 0, init, 0, read/write, convert}`, with no convert on the
/// non-analog types.
static DEVICES: &[(&str, &[DevSup])] = &[
    (
        "ai",
        &[
            DevSup {
                name: "devAiMKS",
                choice: ip_devices::mks::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAiMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAiTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAiHeidND261",
                choice: ip_devices::nd261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devAiStrParm",
                choice: "asyn ai stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
        ],
    ),
    (
        "ao",
        &[
            DevSup {
                name: "devAoMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAoTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAoStrParm",
                choice: "asyn ao stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4, 5], false),
            },
            DevSup {
                name: "devAoEurotherm",
                choice: "asyn ao Eurotherm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "bi",
        &[
            DevSup {
                name: "devBiMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devBiTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devBiStrParm",
                choice: "asyn bi stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "bo",
        &[
            DevSup {
                name: "devBoMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devBoTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devBoStrParm",
                choice: "asyn bo stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "mbbi",
        &[DevSup {
            name: "devMbbiTPG261",
            choice: ip_devices::tpg261::DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }],
    ),
    (
        "mbbo",
        &[
            DevSup {
                name: "devMbboMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devMbboTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "stringin",
        &[
            DevSup {
                name: "devSiMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devSiTPG261",
                choice: ip_devices::tpg261::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devSiStrParm",
                choice: "asyn si stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "stringout",
        &[
            DevSup {
                name: "devSoMPC",
                choice: ip_devices::mpc::DTYP,
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devSoStrParm",
                choice: "asyn so stringParm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
            DevSup {
                name: "devSoEurotherm",
                choice: "asyn so Eurotherm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
    (
        "longin",
        &[DevSup {
            name: "devLiStrParm",
            choice: "asyn li stringParm",
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }],
    ),
    (
        "longout",
        &[DevSup {
            name: "devLoStrParm",
            choice: "asyn lo stringParm",
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[2, 4], false),
        }],
    ),
];

/// Every port the startup script creates, kept alive for the life of the IOC.
type Ports = Arc<Mutex<Vec<IpPortRuntime>>>;

fn string_arg(args: &[ArgValue], i: usize) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.to_str()?.to_owned()),
        _ => Err(format!("argument {i} must be a string")),
    }
}

fn int_arg(args: &[ArgValue], i: usize) -> Result<i64, String> {
    match args.get(i) {
        Some(ArgValue::Int(v)) => Ok(*v),
        _ => Err(format!("argument {i} must be an integer")),
    }
}

/// Poll period in seconds; the C device support polled at the record's SCAN
/// rate, so this is the port's equivalent knob.
fn poll_arg(args: &[ArgValue], i: usize, default: f64) -> Result<Duration, String> {
    let seconds = match args.get(i) {
        Some(ArgValue::Double(v)) => *v,
        Some(ArgValue::Int(v)) => *v as f64,
        None => default,
        _ => return Err(format!("argument {i} must be a number")),
    };
    if seconds <= 0.0 {
        return Err(format!("the poll period must be positive, got {seconds}"));
    }
    Ok(Duration::from_secs_f64(seconds))
}

fn register(
    ports: &Ports,
    claim: PortClaim,
    owner: &Arc<InterruptGate>,
    port: IpPortRuntime,
) -> Result<(), String> {
    claim
        .publish(port.port_handle().clone(), owner)
        .map_err(|e| e.to_string())?;
    ports.lock().expect("port list poisoned").push(port);
    Ok(())
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: ip-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("IP", env!("CARGO_MANIFEST_DIR"));

    let ports: Ports = Arc::new(Mutex::new(Vec::new()));

    let mut app = IocApplication::new();
    let (asyn_name, asyn_factory) = epics_rs::asyn::asyn_record::asyn_record_factory();
    app = app.register_record_type(asyn_name, move || asyn_factory());
    // scalcout is opt-in in epics-rs (dropped from the default
    // registry with the stdRecords.dbd manifest); the db files this IOC
    // loads use it, as a C IOC links the owning module's .dbd.
    app = app.register_record_type("scalcout", || {
        Box::new(epics_rs::base::server::records::scalcout::ScalcoutRecord::default())
    });
    app = epics_rs::asyn::adapter::register_asyn_device_support(app);
    // devAiMKS, devMPC, devTPG261, devXxStrParm, devXxEurotherm and
    // devAiHeidND261: per-record device
    // support, bound by DTYP.
    let mks = ip_devices::mks::device_support_factory();
    let mpc = ip_devices::mpc::device_support_factory();
    let tpg = ip_devices::tpg261::device_support_factory();
    let strparm = ip_devices::strparm::device_support_factory();
    let eurotherm = ip_devices::eurotherm::device_support_factory();
    let nd261 = ip_devices::nd261::device_support_factory();
    app = app.register_dynamic_device_support(DEVICES, move |ctx| {
        mks(ctx)
            .or_else(|| mpc(ctx))
            .or_else(|| tpg(ctx))
            .or_else(|| strparm(ctx))
            .or_else(|| eurotherm(ctx))
            .or_else(|| nd261(ctx))
    });

    // TelevacConfig(port, octetPort, numStations, numRelays, [pollPeriod])
    {
        let ports = ports.clone();
        app = app.register_startup_command(CommandDef::new(
            "TelevacConfig",
            vec![
                ArgDesc {
                    name: "portName",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "octetPort",
                    arg_type: ArgType::String,
                },
                ArgDesc {
                    name: "numStations",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "numRelays",
                    arg_type: ArgType::Int,
                },
                ArgDesc {
                    name: "pollPeriod",
                    arg_type: ArgType::Double,
                },
            ],
            "TelevacConfig portName octetPort numStations numRelays [pollPeriod] - \
             Televac vacuum gauge controller",
            move |args: &[ArgValue], ctx: &CommandContext| {
                let name = string_arg(args, 0)?;
                let octet = string_arg(args, 1)?;
                let stations = int_arg(args, 2)?;
                let relays = int_arg(args, 3)?;
                let stations = u8::try_from(stations)
                    .map_err(|_| format!("TelevacConfig: numStations {stations} is not 1..9"))?;
                let relays = u8::try_from(relays)
                    .map_err(|_| format!("TelevacConfig: numRelays {relays} is not 0..8"))?;
                let poll = poll_arg(args, 4, 1.0)?;
                let claim = claim_port(&name).map_err(|e| format!("TelevacConfig failed: {e}"))?;
                let port = create_televac(&claim, &octet, stations, relays, poll)
                    .map_err(|e| format!("TelevacConfig failed: {e}"))?;
                register(
                    &ports,
                    claim,
                    ctx.db().scan_control().interrupt_gate(),
                    port,
                )?;
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
