//! IOC for the serial devices of the EPICS `ip` module.
//!
//! Usage:
//!   cargo run -p ip-ioc -- iocs/ip/ip-ioc/st.cmd

use epics_rs::base::error::CaResult;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use epics_rs::ca::server::ioc_app::IocApplication;

/// `ipSupport.dbd`'s `devAiMKS` (`"HPS SensaVac 937"`), `dev<Type>MPC`
/// (`"asyn MPC"`), `dev<Type>TPG261` (`"asyn TPG261"`), `dev<Xx>Eurotherm`
/// (`"asyn <xx> Eurotherm"`), `devAiHeidND261` (`"asyn ai HeidND261"`) and
/// `dev<Type>Televac` (`"asyn Televac"`) rows; their dsets
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
                name: "devAiTelevac",
                choice: ip_devices::televac::DTYP,
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
                name: "devBiTelevac",
                choice: ip_devices::televac::DTYP,
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
                name: "devSoEurotherm",
                choice: "asyn so Eurotherm",
                link_type: DbLinkType::InstIo,
                dset: Dset::new(6, &[2, 4], false),
            },
        ],
    ),
];

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        epics_rs::base::stderr_println!("Usage: ip-ioc <st.cmd>");
        std::process::exit(1);
    };

    epics_rs::base::runtime::env::set_default("IP", ip_devices::IP_DIR);

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
    // ipSupport.dbd's registrar() lines: the Federal, Keithley and Oxford
    // programs and the shell's `seq` command.
    ip_devices::register_ip_dbd();
    // devXxStrParm's rows come from this crate, ahead of the rest as
    // ipSupport.dbd lists them.
    app = app.register_dynamic_device_support(
        &ip_devices::strparm::DEVICE_ROWS,
        ip_devices::strparm::device_support_factory(),
    );
    // devAiMKS, devMPC, devTPG261, devXxEurotherm, devAiHeidND261 and
    // devTelevac: per-record device support, bound by DTYP.
    let mks = ip_devices::mks::device_support_factory();
    let mpc = ip_devices::mpc::device_support_factory();
    let tpg = ip_devices::tpg261::device_support_factory();
    let eurotherm = ip_devices::eurotherm::device_support_factory();
    let nd261 = ip_devices::nd261::device_support_factory();
    let televac = ip_devices::televac::device_support_factory();
    app = app.register_dynamic_device_support(DEVICES, move |ctx| {
        mks(ctx)
            .or_else(|| mpc(ctx))
            .or_else(|| tpg(ctx))
            .or_else(|| eurotherm(ctx))
            .or_else(|| nd261(ctx))
            .or_else(|| televac(ctx))
    });

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}
