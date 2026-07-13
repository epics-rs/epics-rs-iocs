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

use opcua::client::AsyncOpcuaConnector;
use opcua::defaults;
use opcua::device_support;
use opcua::ioc::{self, SharedRegistry};
use opcua::registry::Registry;
use opcua::session::{Control, SessionConfig};
use opcua::subscription::SubscriptionConfig;

/// How many `key=value` option slots each option-taking command declares.
///
/// Framework gap: `epics_rs`'s iocsh has no variadic argument type (`ArgType` is
/// String, Int or Double), where the C's option argument is an `iocshArgArgv`
/// tail of any length (`iocshIntegration.cpp:82`). A fixed number of optional
/// String slots is the closest shape; six covers every option `SessionConfig`
/// and `SubscriptionConfig` accept.
const OPTION_SLOTS: usize = 6;

fn string(args: &[ArgValue], i: usize) -> Option<String> {
    match args.get(i) {
        Some(ArgValue::String(s)) if !s.is_empty() => Some(s.clone()),
        _ => None,
    }
}

fn required(args: &[ArgValue], i: usize, name: &str) -> Result<String, String> {
    string(args, i).ok_or_else(|| format!("argument '{name}' is required"))
}

/// The `key=value` tail of a command, from argument `from` on.
fn options(args: &[ArgValue], from: usize) -> Result<Vec<(String, String)>, String> {
    let tokens: Vec<String> = (from..args.len()).filter_map(|i| string(args, i)).collect();
    ioc::parse_options(tokens.iter().map(String::as_str))
}

fn arg(name: &'static str, arg_type: ArgType, optional: bool) -> ArgDesc {
    ArgDesc {
        name,
        arg_type,
        optional,
    }
}

/// `[options]`, as `OPTION_SLOTS` optional String arguments.
fn option_args() -> Vec<ArgDesc> {
    (0..OPTION_SLOTS)
        .map(|_| arg("[key=value]", ArgType::String, true))
        .collect()
}

fn commands(
    mut app: IocApplication,
    registry: &SharedRegistry,
    connector: &Arc<AsyncOpcuaConnector>,
) -> IocApplication {
    // opcuaSession(name, URL, [options]) — iocshIntegration.cpp:88-140.
    let mut args = vec![
        arg("name", ArgType::String, false),
        arg("URL", ArgType::String, false),
    ];
    args.extend(option_args());
    app = app.register_startup_command(CommandDef::new(
        "opcuaSession",
        args,
        "opcuaSession name URL [key=value ...]",
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = required(args, 0, "name")?;
                let url = required(args, 1, "URL")?;
                if name.contains(char::is_whitespace) {
                    return Err(format!("session name '{name}' must not contain spaces"));
                }
                let mut config = SessionConfig::new(&name, &url);
                for (key, value) in options(args, 2)? {
                    config
                        .set_option(&key, &value)
                        .map_err(|e| format!("opcuaSession {name}: {e}"))?;
                }
                registry
                    .add_session(config)
                    .map_err(|e| format!("opcuaSession: {e}"))?;
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaSubscription(name, session, interval, [options]) —
    // iocshIntegration.cpp:159-201.
    let mut args = vec![
        arg("name", ArgType::String, false),
        arg("session", ArgType::String, false),
        arg("publishing interval [ms]", ArgType::Double, true),
    ];
    args.extend(option_args());
    app = app.register_startup_command(CommandDef::new(
        "opcuaSubscription",
        args,
        "opcuaSubscription name session [interval_ms] [key=value ...]",
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let name = required(args, 0, "name")?;
                let session = required(args, 1, "session")?;
                if name.contains(char::is_whitespace) {
                    return Err(format!(
                        "subscription name '{name}' must not contain spaces"
                    ));
                }
                let mut config = SubscriptionConfig::new(&name, &session);
                // A missing or non-positive interval keeps the default, as in the
                // C (`iocshIntegration.cpp:176-181`).
                if let Some(ArgValue::Double(interval)) = args.get(2)
                    && *interval > 0.0
                {
                    config.publishing_interval = *interval;
                }
                for (key, value) in options(args, 3)? {
                    config
                        .set_option(&key, &value)
                        .map_err(|e| format!("opcuaSubscription {name}: {e}"))?;
                }
                registry
                    .add_subscription(config)
                    .map_err(|e| format!("opcuaSubscription: {e}"))?;
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaOptions(pattern, [options]) — iocshIntegration.cpp:212-260.
    let mut args = vec![arg("session/subscription", ArgType::String, false)];
    args.extend(option_args());
    app = app.register_startup_command(CommandDef::new(
        "opcuaOptions",
        args,
        "opcuaOptions pattern [key=value ...]",
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let pattern = required(args, 0, "session/subscription")?;
                let options = options(args, 1)?;
                ioc::set_options(&registry, &pattern, &options)
                    .map_err(|e| format!("opcuaOptions: {e}"))?;
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaShow(pattern, verbosity) — iocshIntegration.cpp:379-410. A runtime
    // command as well as a startup one, as it is in the C.
    let show = CommandDef::new(
        "opcuaShow",
        vec![
            arg("pattern", ArgType::String, true),
            arg("verbosity", ArgType::Int, true),
        ],
        "opcuaShow [pattern] [verbosity]",
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let pattern = string(args, 0).unwrap_or_else(|| "*".to_string());
                let level = match args.get(1) {
                    Some(ArgValue::Int(v)) => *v as i32,
                    _ => 0,
                };
                print!("{}", ioc::show(&registry, &pattern, level));
                Ok(CommandOutcome::Continue)
            }
        },
    );
    app = app
        .register_startup_command(show.clone())
        .register_shell_command(show);

    // opcuaConnect / opcuaDisconnect — iocshIntegration.cpp:419-490.
    for (name, control, usage) in [
        ("opcuaConnect", Control::Connect, "opcuaConnect session"),
        (
            "opcuaDisconnect",
            Control::Disconnect,
            "opcuaDisconnect session",
        ),
    ] {
        let command = CommandDef::new(name, vec![arg("session", ArgType::String, false)], usage, {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let session = required(args, 0, "session")?;
                registry
                    .control(&session, control)
                    .map_err(|e| format!("{name}: {e}"))?;
                Ok(CommandOutcome::Continue)
            }
        });
        app = app
            .register_startup_command(command.clone())
            .register_shell_command(command);
    }

    // opcuaMapNamespace(session, index, URI) — iocshIntegration.cpp:452-490.
    app = app.register_startup_command(CommandDef::new(
        "opcuaMapNamespace",
        vec![
            arg("session", ArgType::String, false),
            arg("namespace index", ArgType::Int, false),
            arg("namespace URI", ArgType::String, false),
        ],
        "opcuaMapNamespace session index URI",
        {
            let registry = registry.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let session = required(args, 0, "session")?;
                let index = match args.get(1) {
                    Some(ArgValue::Int(v)) if (0..=i64::from(u16::MAX)).contains(v) => *v as u16,
                    _ => return Err("namespace index must be 0..65535".into()),
                };
                let uri = required(args, 2, "namespace URI")?;
                registry
                    .map_namespace(&session, index, &uri)
                    .map_err(|e| format!("opcuaMapNamespace: {e}"))?;
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaShowSecurity(session) — iocshIntegration.cpp:573-590.
    let show_security = CommandDef::new(
        "opcuaShowSecurity",
        vec![arg("session", ArgType::String, true)],
        "opcuaShowSecurity [session]",
        {
            let registry = registry.clone();
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let session = string(args, 0).unwrap_or_else(|| "*".to_string());
                print!(
                    "{}",
                    ioc::show_security(&registry, &connector.security(), &session)
                );
                Ok(CommandOutcome::Continue)
            }
        },
    );
    app = app
        .register_startup_command(show_security.clone())
        .register_shell_command(show_security);

    // opcuaClientCertificate(cert, key) — iocshIntegration.cpp:603-620.
    app = app.register_startup_command(CommandDef::new(
        "opcuaClientCertificate",
        vec![
            arg("client public cert file", ArgType::String, false),
            arg("client private key file", ArgType::String, false),
        ],
        "opcuaClientCertificate certfile keyfile",
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let certificate = required(args, 0, "client public cert file")?;
                let key = required(args, 1, "client private key file")?;
                connector.set_client_certificate(&certificate, &key);
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaSetupPKI(location, ...) — iocshIntegration.cpp:638-690.
    //
    // Accepted-and-documented: only the C's one-argument form ("a standard
    // directory structure under the specified location") has an equivalent.
    // `async-opcua` lays the four PKI locations out beneath one root itself, so
    // the C's four-argument form — trusted certs, server CRLs, issuer certs,
    // issuer CRLs, each somewhere different — cannot be honoured. It is refused
    // rather than silently reduced to its first argument.
    app = app.register_startup_command(CommandDef::new(
        "opcuaSetupPKI",
        vec![
            arg("PKI / server certs location", ArgType::String, false),
            arg("server revocation lists location", ArgType::String, true),
            arg("issuer certs location", ArgType::String, true),
            arg("issuer revocation lists location", ArgType::String, true),
        ],
        "opcuaSetupPKI pki_root",
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let root = required(args, 0, "PKI / server certs location")?;
                if (1..4).any(|i| string(args, i).is_some()) {
                    return Err("opcuaSetupPKI: this port's client (async-opcua) derives \
                         trusted/, rejected/ and issuers/ from one PKI root; the four-argument \
                         form, which names each location separately, has no equivalent. Use \
                         'opcuaSetupPKI <root>'."
                        .into());
                }
                connector.set_pki_root(&root);
                Ok(CommandOutcome::Continue)
            }
        },
    ));

    // opcuaSaveRejected(location) — iocshIntegration.cpp:701-740.
    //
    // Accepted-and-documented: rejected server certificates are saved, but the
    // location is `<PKI root>/rejected`, which `async-opcua` fixes. A script that
    // asks for a different one is warned, not failed — the certificates it wants
    // kept are being kept, just elsewhere.
    app = app.register_startup_command(CommandDef::new(
        "opcuaSaveRejected",
        vec![arg("location", ArgType::String, true)],
        "opcuaSaveRejected [location]",
        {
            let connector = connector.clone();
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let pki = connector
                    .security()
                    .pki_root
                    .unwrap_or_else(|| opcua::client::DEFAULT_PKI_DIR.to_string());
                match string(args, 0) {
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
            arg("variable", ArgType::String, false),
            arg("value", ArgType::String, false),
        ],
        "var opcua_<Variable> value",
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let name = required(args, 0, "variable")?;
            let value = required(args, 1, "value")?;
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

    let mut app = IocApplication::new();

    // The `opcuaItem` record type (`opcuaItemRecord.cpp`).
    let (name, factory) = ioc::item_record_factory();
    app = app.register_record_type(name, factory);

    // Every record type's `@opcua(...)` link binds through this.
    app = app.register_dynamic_device_support(device_support::factory(registry.clone()));

    app = commands(app, &registry, &connector);

    // The sessions connect once every record has added its item, so that the
    // initial read covers all of them (`SessionOpen62541::initHook`).
    app = app.register_after_init({
        let registry = registry.clone();
        move || {
            let started = registry.start();
            eprintln!("opcua: {started} session(s) started");
        }
    });

    app.startup_script(&script)
        .register_link_set_installer(epics_rs::ca::calink::calink_link_set_install)
        .register_link_set_installer(epics_rs::bridge::qsrv::pvalink_link_set_install)
        .run(epics_rs::bridge::qsrv::run_ca_pva_qsrv_ioc)
        .await
}
