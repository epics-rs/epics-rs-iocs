//! OPC UA IOC binary — `epics-modules/opcua`, on the pure-Rust `async-opcua`
//! client (`devOpcuaSup/iocshIntegration.cpp`).
//!
//! Wires three things the driver crate cannot wire by itself:
//!
//! * the `opcuaItem` record type (`opcuaItemRecord.dbd` in the C module),
//! * the dynamic device support that every record's `@opcua(...)` link binds
//!   through (the C's `opcua` DTYP, one dset per record type),
//! * the iocsh commands, and the after-`iocInit` hook that starts the session
//!   workers — the C connects from an `initHookAfterIocRunning` hook too, so
//!   that every record has added its item to the session before the first read.
//!
//! Not ported: the C's deprecated command set (`opcuaCreateSession`,
//! `opcuaSetOption`, `opcuaCreateSubscription`, `opcuaShowSession`,
//! `opcuaDebugSession`, `opcuaShowSubscription`, `opcuaDebugSubscription`,
//! `opcuaShowData`, `iocshIntegration.cpp:1099-1107`). Each of them is an alias
//! for, or a subset of, `opcuaSession` / `opcuaSubscription` / `opcuaOptions` /
//! `opcuaShow`, and the C prints a deprecation warning for every one.
//!
//! Usage:
//!   cargo run -p opcua-ioc -- st.cmd

use std::sync::Arc;

use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::*;
use epics_rs::ca::server::ioc_app::IocApplication;

use epics_rs::base::server::record::{DbLinkType, DevSup, Dset};
use opcua::client::AsyncOpcuaConnector;
use opcua::defaults;
use opcua::device_support;
use opcua::ioc::{self, SharedRegistry};
use opcua::registry::Registry;
use opcua::session::Control;

/// `devOpcuaSup/10_devOpcuaInt64.dbd` and `20_devOpcuaAll.dbd`'s `device()` lines.
static DEVICES: &[(&str, &[DevSup])] = &[
    (
        "longin",
        &[row("devLiOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "longout",
        &[row("devLoOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    ("bi", &[row("devBiOpcua", device_support::DTYP, 6, &[3, 4])]),
    ("bo", &[row("devBoOpcua", device_support::DTYP, 6, &[3, 4])]),
    (
        "mbbi",
        &[row("devMbbiOpcua", device_support::DTYP, 6, &[2, 3, 4])],
    ),
    (
        "mbbo",
        &[row("devMbboOpcua", device_support::DTYP, 6, &[2, 3, 4])],
    ),
    (
        "mbbiDirect",
        &[row(
            "devMbbiDirectOpcua",
            device_support::DTYP,
            6,
            &[2, 3, 4],
        )],
    ),
    (
        "mbboDirect",
        &[row(
            "devMbboDirectOpcua",
            device_support::DTYP,
            6,
            &[2, 3, 4],
        )],
    ),
    ("ai", &[row("devAiOpcua", device_support::DTYP, 6, &[3, 4])]),
    ("ao", &[row("devAoOpcua", device_support::DTYP, 6, &[3, 4])]),
    (
        "stringin",
        &[row("devSiOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "stringout",
        &[row("devSoOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "lsi",
        &[row("devLsiOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "lso",
        &[row("devLsoOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "waveform",
        &[row("devWfOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "aai",
        &[row("devAaiOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "aao",
        &[row("devAaoOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "int64in",
        &[row("devInt64inOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "int64out",
        &[row("devInt64outOpcua", device_support::DTYP, 6, &[3, 4])],
    ),
    (
        "opcuaItem",
        &[DevSup {
            name: "devItemOpcua",
            choice: device_support::DTYP,
            link_type: DbLinkType::InstIo,
            dset: Dset::new(6, &[3, 4], false),
        }],
    ),
];

/// An opcua row whose `init` is `opcua_init`, which calls `devExtend` at
/// pass 0 (`devOpcua.cpp:139`).
const fn row(name: &'static str, choice: &'static str, number: u8, funcs: &'static [u8]) -> DevSup {
    DevSup {
        name,
        choice,
        link_type: DbLinkType::InstIo,
        dset: Dset::new(number, funcs, true).with_init(device_support::opcua_init),
    }
}

/// C's `sval`: `None` when the line ended before the argument, `""` for an
/// empty token.
fn sval(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Some(s.to_str_lossy().into_owned()),
        _ => None,
    }
}

/// C's `ival` / `dval`: 0 when the argument is absent or empty.
fn ival(args: &[ArgValue], i: usize) -> i64 {
    match args.get(i) {
        Some(ArgValue::Int(v)) => *v,
        _ => 0,
    }
}

fn dval(args: &[ArgValue], i: usize) -> f64 {
    match args.get(i) {
        Some(ArgValue::Double(v)) => *v,
        _ => 0.0,
    }
}

/// An `iocshArgArgv` option tail, without the C's leading `av[0]`.
fn argv(args: &[ArgValue], i: usize) -> Vec<String> {
    match args.get(i) {
        Some(ArgValue::Argv(tokens)) => tokens
            .iter()
            .map(|t| t.to_str_lossy().into_owned())
            .collect(),
        _ => Vec::new(),
    }
}

fn strs(tokens: &[String]) -> Vec<&str> {
    tokens.iter().map(String::as_str).collect()
}

fn arg(name: &'static str, arg_type: ArgType) -> ArgDesc {
    ArgDesc { name, arg_type }
}

fn commands(
    mut app: IocApplication,
    registry: &SharedRegistry,
    connector: &Arc<AsyncOpcuaConnector>,
) -> IocApplication {
    // None of the C's command functions sets an error status: each prints its
    // own lines and the script goes on.
    app = app.register_startup_command(CommandDef::new(
        "opcuaSession",
        vec![
            arg("name", ArgType::String),
            arg("URL", ArgType::String),
            arg("[options]", ArgType::Argv),
        ],
        ioc::SESSION_USAGE,
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let options = argv(args, 2);
                ioc::session_command(
                    &registry,
                    sval(args, 0).as_deref(),
                    sval(args, 1).as_deref(),
                    &strs(&options),
                );
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    app = app.register_startup_command(CommandDef::new(
        "opcuaSubscription",
        vec![
            arg("name", ArgType::String),
            arg("session", ArgType::String),
            arg("publishing interval [ms]", ArgType::Double),
            arg("[options]", ArgType::Argv),
        ],
        ioc::SUBSCRIPTION_USAGE,
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let options = argv(args, 3);
                ioc::subscription_command(
                    &registry,
                    sval(args, 0).as_deref(),
                    sval(args, 1).as_deref(),
                    dval(args, 2),
                    &strs(&options),
                );
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    app = app.register_startup_command(CommandDef::new(
        "opcuaOptions",
        vec![
            arg("pattern", ArgType::String),
            arg("[options]", ArgType::Argv),
        ],
        ioc::OPTIONS_USAGE,
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let options = argv(args, 1);
                ioc::options_command(&registry, sval(args, 0).as_deref(), &strs(&options));
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // A runtime command as well as a startup one, as it is in the C.
    let show = CommandDef::new(
        "opcuaShow",
        vec![
            arg("pattern", ArgType::String),
            arg("verbosity", ArgType::Int),
        ],
        ioc::SHOW_USAGE,
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let _ = ioc::show(
                    &registry,
                    sval(args, 0).as_deref(),
                    ival(args, 1) as i32,
                    // Unlocked: the errlog thread prints to stderr while this
                    // waits for it.
                    &mut std::io::stdout(),
                    &mut std::io::stderr(),
                );
                Ok(CommandOutcome::Continue)
            }
        },
    );
    app = app
        .register_startup_command(show.clone())
        .register_shell_command(show);

    for (name, arg_name, control, usage) in [
        (
            "opcuaConnect",
            "session",
            Control::Connect,
            ioc::CONNECT_USAGE,
        ),
        (
            "opcuaDisconnect",
            "session name",
            Control::Disconnect,
            ioc::DISCONNECT_USAGE,
        ),
    ] {
        let command = CommandDef::new(name, vec![arg(arg_name, ArgType::String)], usage, {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                ioc::control_command(&registry, sval(args, 0).as_deref(), control);
                Ok(CommandOutcome::Continue)
            }
        });
        app = app
            .register_startup_command(command.clone())
            .register_shell_command(command);
    }

    app = app.register_startup_command(CommandDef::new(
        "opcuaMapNamespace",
        vec![
            arg("session", ArgType::String),
            arg("namespace index", ArgType::Int),
            arg("namespace URI", ArgType::String),
        ],
        ioc::MAP_NAMESPACE_USAGE,
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                ioc::map_namespace_command(
                    &registry,
                    sval(args, 0).as_deref(),
                    ival(args, 1),
                    sval(args, 2).as_deref(),
                );
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    let show_security = CommandDef::new(
        "opcuaShowSecurity",
        vec![arg("session name [\"\"=client]", ArgType::String)],
        ioc::SHOW_SECURITY_USAGE,
        {
            let registry = registry.clone();
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                print!(
                    "{}",
                    ioc::show_security(&registry, &connector.security(), sval(args, 0).as_deref())
                );
                Ok(CommandOutcome::Continue)
            }
        },
    );
    app = app
        .register_startup_command(show_security.clone())
        .register_shell_command(show_security);

    app = app.register_startup_command(CommandDef::new(
        "opcuaClientCertificate",
        vec![
            arg("certificate (public key) file", ArgType::String),
            arg("private key file", ArgType::String),
        ],
        ioc::CLIENT_CERTIFICATE_USAGE,
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let (certificate, key) = (sval(args, 0), sval(args, 1));
                if let Some((certificate, key)) =
                    ioc::client_certificate_args(certificate.as_deref(), key.as_deref())
                {
                    connector.set_client_certificate(certificate, key);
                }
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // Accepted-and-documented: only the C's one-argument form ("a standard
    // directory structure under the specified location") has an equivalent.
    // `async-opcua` lays the four PKI locations out beneath one root itself, so
    // the C's four-argument form — trusted certs, server CRLs, issuer certs,
    // issuer CRLs, each somewhere different — cannot be honoured. It is refused
    // rather than silently reduced to its first argument.
    app = app.register_startup_command(CommandDef::new(
        "opcuaSetupPKI",
        ioc::SETUP_PKI_ARGS
            .iter()
            .map(|name| arg(name, ArgType::String))
            .collect(),
        ioc::SETUP_PKI_USAGE,
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let args: [Option<String>; 4] = std::array::from_fn(|i| sval(args, i));
                match ioc::setup_pki_args(args.each_ref().map(|a| a.as_deref())) {
                    Some([root, "", "", ""]) => connector.set_pki_root(root),
                    Some(_) => eprintln!(
                        "opcuaSetupPKI: this port's client (async-opcua) derives trusted/, \
                         rejected/ and issuers/ from one PKI root; the four-argument form, which \
                         names each location separately, has no equivalent. Use \
                         'opcuaSetupPKI <root>'."
                    ),
                    None => {}
                }
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // Accepted-and-documented: rejected server certificates are saved, but the
    // location is `<PKI root>/rejected`, which `async-opcua` fixes. A script that
    // asks for a different one is warned, not failed — the certificates it wants
    // kept are being kept, just elsewhere.
    app = app.register_startup_command(CommandDef::new(
        "opcuaSaveRejected",
        vec![arg("location for saving rejected certs", ArgType::String)],
        ioc::SAVE_REJECTED_USAGE,
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let pki = connector
                    .security()
                    .pki_root
                    .unwrap_or_else(|| opcua::client::DEFAULT_PKI_DIR.to_string());
                match sval(args, 0).filter(|s| !s.is_empty()) {
                    Some(location) => eprintln!(
                        "opcuaSaveRejected: '{location}' ignored — this port's client saves \
                         rejected server certificates under '{pki}/rejected', which it fixes."
                    ),
                    None => eprintln!(
                        "opcuaSaveRejected: rejected server certificates are saved under \
                         '{pki}/rejected'."
                    ),
                }
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // var(name, value) — the C's iocsh variables come from the .dbd's
    // `variable()` entries (`opcua.dbd`, `iocshVariables.h`), which iocsh's own
    // built-in `var` command then sets.
    //
    // Framework gap: `epics_rs` has neither a dbd `variable()` entry nor a `var`
    // command, so the ten `opcua_Default*` variables are unreachable from an
    // st.cmd unless the IOC provides the command. This one is that command; it
    // sets only this module's variables (`defaults::set_variable`).
    app = app.register_startup_command(CommandDef::new(
        "var",
        vec![
            arg("variable", ArgType::String),
            arg("value", ArgType::String),
        ],
        "var opcua_<Variable> value",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let required = |i, name| {
                sval(args, i)
                    .filter(|s| !s.is_empty())
                    .ok_or_else(|| format!("argument '{name}' is required"))
            };
            let name = required(0, "variable")?;
            let value = required(1, "value")?;
            if !defaults::set_variable(&name, &value) {
                return Err(format!(
                    "var: '{name}' is not one of this IOC's variables, or '{value}' is not a \
                     value it takes"
                ));
            }
            Ok(CommandOutcome::Continue)
        },
    ));

    app
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();
    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: opcua-ioc <st.cmd>");
        std::process::exit(1);
    };

    let connector = Arc::new(AsyncOpcuaConnector::new());
    let registry: SharedRegistry = Registry::new(connector.clone());

    epics_rs::base::runtime::env::set_default("OPCUA", env!("CARGO_MANIFEST_DIR"));

    let mut app = IocApplication::new();

    // The `opcuaItem` record type (`opcuaItemRecord.cpp`).
    let (name, factory) = ioc::item_record_factory();
    app = app.register_record_type(name, factory);

    // Every record type's `@opcua(...)` link binds through this.
    app = app.register_dynamic_device_support(DEVICES, device_support::factory(registry.clone()));

    app = commands(app, &registry, &connector);

    // The sessions connect once every record has added its item, so that the
    // initial read covers all of them (`SessionOpen62541::initHook`).
    app = app.register_after_init({
        let registry = registry.clone();
        move || registry.start()
    });

    epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc_app(app.startup_script(&script)).await
}

#[cfg(test)]
mod tests {
    use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
    use std::sync::Mutex;

    use super::*;

    /// `opcua_init` calls `devExtend(&opcua_dsxt)` at pass 0
    /// (`devOpcua.cpp:139`); devItemOpcua has no init, so no `dsxt`.
    #[test]
    fn every_dset_but_item_extends() {
        for (record_type, rows) in DEVICES {
            for row in *rows {
                assert_eq!(
                    row.has_dsxt(),
                    row.name != "devItemOpcua",
                    "{record_type} {}",
                    row.name
                );
            }
        }
    }

    /// initDevSup's init(0) then finishDevSup's init(1) over every row: the
    /// opcua_init banner prints once, and devItemOpcua (SUPI, devOpcua.cpp:1346)
    /// has no init.
    #[test]
    fn the_banner_prints_once_across_the_dsets() {
        let lines = Arc::new(Mutex::new(Vec::<String>::new()));
        let id = errlog_add_listener({
            let lines = lines.clone();
            move |line: &str| lines.lock().unwrap().push(line.to_string())
        });
        let opcua_init: fn(bool) = device_support::opcua_init;
        for after in [false, true] {
            for (record_type, rows) in DEVICES {
                for row in *rows {
                    let init = row.dset.init();
                    assert_eq!(
                        init.map(|f| f as usize),
                        (row.name != "devItemOpcua").then_some(opcua_init as usize),
                        "{record_type} {}",
                        row.name
                    );
                    if let Some(init) = init {
                        init(after);
                    }
                }
            }
        }
        errlog_flush();
        errlog_remove_listener(id);
        let lines = lines.lock().unwrap();
        let banners: Vec<_> = lines
            .iter()
            .filter(|l| l.starts_with("OPC UA Client Device Support "))
            .collect();
        assert_eq!(banners, [device_support::BANNER], "{lines:#?}");
    }
}
