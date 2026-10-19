//! What an IOC binary needs to wire this driver up: the record-type factory, the
//! device-support factory, and the bodies of the iocsh commands
//! (`iocshIntegration.cpp`).
//!
//! The command *definitions* live in the IOC crate, because that is where the
//! framework's `CommandDef` is assembled; what each one does lives here, so it
//! can be tested without an IOC.

use std::sync::Arc;

use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::RecordFactory;

use crate::client::ClientSecurity;
use crate::defaults;
use crate::link::split_escaped;
use crate::queue::ConnectionStatus;
use crate::record::OpcuaItemRecord;
use crate::registry::Registry;
use crate::session::{Control, SessionConfig, strtoul};
use crate::subscription::SubscriptionConfig;

/// The `opcuaItem` record-type factory, for
/// `IocApplication::register_record_type`.
pub fn item_record_factory() -> (&'static str, RecordFactory) {
    (
        "opcuaItem",
        Box::new(|| Box::new(OpcuaItemRecord::default())),
    )
}

/// `opcuaSession`'s usage text (`iocshIntegration.cpp:108-114`).
pub const SESSION_USAGE: &str = concat!(
    "Configures a new OPC UA session, assigning it a name and the URL ",
    "of the OPC UA server.\nMust be called before iocInit.\n\n",
    "name       session name (no spaces)\n",
    "URL        URL of the OPC UA server (e.g. opc.tcp://192.168.1.23:4840)\n",
    "[options]  list of options in 'key=value' format\n",
    "           (see 'help opcuaOptions' for a list of valid options)\n",
);

/// `opcuaSubscription`'s usage text (`iocshIntegration.cpp:195-202`).
pub const SUBSCRIPTION_USAGE: &str = concat!(
    "Configures a new OPC UA subscription, assigning it a name and creating it under an existing ",
    "session.\nMust be called before iocInit.\n\n",
    "name                 subscription name (no spaces)\n",
    "session              name of the existing OPC UA session for the new subscription\n",
    "publishing interval  publishing interval for the new subscription (in ms)\n",
    "[options]            list of options in 'key=value' format\n",
    "                     (see 'help opcuaOptions' for a list of valid options)\n",
);

/// `opcuaOptions`'s usage text: `Session::optionUsage` followed by
/// `Subscription::optionUsage` (`Session.cpp:159-176`, `Subscription.cpp:56-60`).
///
/// Upstream C defect fixed at source: the C lists the identity file option as
/// `ident-file`, which `SessionOpen62541::setOption` does not know; the option
/// it reads is `sec-id`.
pub const OPTIONS_USAGE: &str = "Sets options for existing OPC UA sessions or subscriptions.\n\n\
pattern    pattern for session or subscription names (* and ? supported)\n\
[options]  list of options in 'key=value' format\n\n\
Valid session options are:\n\
debug              debug level [default 0 = no debug]\n\
autoconnect        automatically connect sessions [default y]\n\
nodes-max          max. nodes per service call [0 = no limit]\n\
read-nodes-max     max. nodes per read service call [0 = no limit]\n\
read-timeout-min   min. timeout (holdoff) after read service call [ms]\n\
read-timeout-max   timeout (holdoff) after read service call w/ max elements [ms]\n\
write-nodes-max    max. nodes per write service call [0 = no limit]\n\
write-timeout-min  min. timeout (holdoff) after write service call [ms]\n\
write-timeout-max  timeout (holdoff) after write service call w/ max elements [ms]\n\
sec-mode           requested security mode\n\
sec-policy         requested security policy\n\
sec-id             file to read identity credentials from\n\n\
Valid subscription options are:\n\
debug              debug level [default 0 = no debug]\n\
priority           priority level [default 0(lowest) .. 255]\n";

/// `opcuaShow`'s usage text (`iocshIntegration.cpp:372-375`).
pub const SHOW_USAGE: &str = "Prints information about sessions, subscriptions, items and their \
related data elements.\n\n\
pattern    glob pattern (supports * and ?) for session, subscription, record names\n\
verbosity  amount of printed information (default 0 = sparse)\n";

/// `opcuaConnect`'s usage text (`iocshIntegration.cpp:424-427`).
pub const CONNECT_USAGE: &str = "Attempts to connect sessions to the configured OPC UA server.\n\
For sessions with configured autoconnect option, the autoconnector is started.\n\n\
session  glob pattern (supports * and ?) for session names\n";

/// `opcuaDisconnect`'s usage text (`iocshIntegration.cpp:464-467`).
pub const DISCONNECT_USAGE: &str = "Gracefully disconnects sessions from the configured server.\n\
For sessions with configured autoconnect option, the autoconnector is stopped.\n\n\
session  glob pattern (supports * and ?) for session names\n";

/// `opcuaMapNamespace`'s usage text (`iocshIntegration.cpp:508-515`).
pub const MAP_NAMESPACE_USAGE: &str = "Adds a namespace mapping to the mapping table of the \
specified session.\nThe specified numerical namespace index (used in the loaded databases) will \
be mapped to the\nspecified namespace URI (on the server).\nThis allows to automatically adapt \
to servers that use volatile namespace indices.\n\n\
session          existing session name\n\
namespace index  numerical namespace as used in the database files\n\
namespace URI    full URI identification of that namespace\n";

/// `opcuaShowSecurity`'s usage text (`iocshIntegration.cpp:566-568`).
pub const SHOW_SECURITY_USAGE: &str = "Prints information about the security setup of a sepcific \
session or the IOC client.\n\n\
session name  name of the session to report on (empty string for client report)\n";

/// `opcuaClientCertificate`'s usage text (`iocshIntegration.cpp:599-602`).
pub const CLIENT_CERTIFICATE_USAGE: &str = "Sets up the OPC UA client certificates to use for the IOC client.\n\n\
certificate file  path to the file containing the certificate (public key)\n\
private key file  path to the file containing the private key\n";

/// `opcuaSetupPKI`'s usage text (`iocshIntegration.cpp:640-651`).
pub const SETUP_PKI_USAGE: &str = "Sets up the PKI file store of the IOC client, where \
certificates and revocation lists are stored.\n\
The first form (single parameter) expects a standard directory structure under the specified \
location.\n\
The second form (four parameters) explicitly defines the specific locations.\n\n\
PKI / server certs location       path to the PKI structure / to the location of trusted server \
certs\n\
server revocation lists location  path to the location of server revocation lists\n\
issuer certs location             path to the location of issuer certificates\n\
issuer revocation lists location  path to the location of issuer revocation lists\n";

/// `opcuaSetupPKI`'s argument names, which its missing-argument lines quote.
pub const SETUP_PKI_ARGS: [&str; 4] = [
    "PKI / server certs location",
    "server revocation lists location",
    "issuer certs location",
    "issuer revocation lists location",
];

/// `opcuaSaveRejected`'s usage text (`iocshIntegration.cpp:713-715`).
pub const SAVE_REJECTED_USAGE: &str = "Sets the location where the client will save rejected certificates.\n\n\
rejected certs location  where to save rejected certificates\n";

/// Walk an iocsh option tail (`iocshIntegration.cpp:150-163`): each argument
/// holds `:`-separated options, and one that is not exactly `key=value` is
/// reported and ignored as the walk reaches it. `apply` stopping the walk is
/// the C's `catch` around the whole command.
fn walk_options(
    tokens: &[&str],
    mut apply: impl FnMut(&str, &str) -> Result<(), String>,
) -> Result<(), String> {
    for token in tokens {
        for opt in split_escaped(token, ':') {
            if opt.is_empty() {
                continue;
            }
            match <[String; 2]>::try_from(split_escaped(&opt, '=')) {
                Ok([key, value]) => apply(&key, &value)?,
                Err(_) => errlog_vprintf(format_args!(
                    "option '{opt}' must follow 'key=value' format - ignored\n"
                )),
            }
        }
    }
    Ok(())
}

/// The `key=value` pairs of an option tail, parsed up front as `opcuaSession`
/// and `opcuaSubscription` do before they create anything.
pub fn parse_options(tokens: &[&str]) -> Vec<(String, String)> {
    let mut options = Vec::new();
    let _ = walk_options(tokens, |key, value| {
        options.push((key.to_string(), value.to_string()));
        Ok(())
    });
    options
}

/// The last `debug=` of an option tail is nonzero (`std::strtol(...)`, tested
/// as `if (s && debug)`).
fn debug_requested(options: &[(String, String)]) -> bool {
    options
        .iter()
        .rfind(|(key, _)| key == "debug")
        .is_some_and(|(_, value)| strtoul(value) != 0)
}

/// `opcuaSession(name, URL, [options])` (`iocshIntegration.cpp:124-183`).
pub fn session_command(
    registry: &Registry,
    name: Option<&str>,
    url: Option<&str>,
    options: &[&str],
) {
    let mut ok = true;
    match name {
        None => {
            errlog_vprintf(format_args!("missing argument #1 (session name)\n"));
            ok = false;
        }
        Some(name) if name.contains(' ') => {
            errlog_vprintf(format_args!(
                "invalid argument #1 (session name) '{name}'\n"
            ));
            ok = false;
        }
        Some(name) if registry.name_in_use(name) => {
            errlog_vprintf(format_args!("session name {name} already in use\n"));
            ok = false;
        }
        Some(_) => {}
    }
    if url.is_none() {
        errlog_vprintf(format_args!("missing argument #2 (server URL)\n"));
        ok = false;
    }
    let options = parse_options(options);

    let (true, Some(name), Some(url)) = (ok, name, url) else {
        errlog_vprintf(format_args!("ERROR - no session created\n"));
        return;
    };
    let Some(session) = registry.add_session(SessionConfig::new(name, url)) else {
        return;
    };
    if debug_requested(&options) {
        errlog_vprintf(format_args!(
            "opcuaSession: successfully created session '{name}'\n"
        ));
    }
    for (key, value) in &options {
        if let Err(e) = session.set_option(key, value) {
            eprintln!("ERROR : {e}");
            return;
        }
    }
}

/// `opcuaSubscription(name, session, publishing interval, [options])`
/// (`iocshIntegration.cpp:213-288`).
pub fn subscription_command(
    registry: &Registry,
    name: Option<&str>,
    session: Option<&str>,
    interval: f64,
    options: &[&str],
) {
    let mut ok = true;
    match name {
        None => {
            errlog_vprintf(format_args!("missing argument #1 (subscription name)\n"));
            ok = false;
        }
        Some(name) if name.contains(' ') => {
            errlog_vprintf(format_args!(
                "invalid argument #1 (subscription name) '{name}'\n"
            ));
            ok = false;
        }
        Some(name) if registry.has_subscription(name) => {
            errlog_vprintf(format_args!("subscription name {name} already in use\n"));
            ok = false;
        }
        Some(_) => {}
    }
    match session {
        None => {
            errlog_vprintf(format_args!("missing argument #2 (session name)\n"));
            ok = false;
        }
        Some(session) if registry.session(session).is_err() => {
            errlog_vprintf(format_args!("session {session} does not exist\n"));
            ok = false;
        }
        Some(_) => {}
    }
    let interval = if interval < 0.0 {
        errlog_vprintf(format_args!(
            "invalid argument #3 (publishing interval) '{interval:.6}' - ignored\n"
        ));
        defaults::DEFAULT_PUBLISH_INTERVAL.get()
    } else if interval == 0.0 {
        defaults::DEFAULT_PUBLISH_INTERVAL.get()
    } else {
        interval
    };
    let options = parse_options(options);

    let (true, Some(name), Some(session)) = (ok, name, session) else {
        errlog_vprintf(format_args!("ERROR - no subscription created\n"));
        return;
    };
    let mut config = SubscriptionConfig::new(name, session);
    config.publishing_interval = interval;
    // A name a session holds passes the checks above and is refused here, with
    // no line, as `Subscription::createSubscription` refuses it.
    if !registry.add_subscription(config) {
        return;
    }
    if debug_requested(&options) {
        errlog_vprintf(format_args!(
            "opcuaSubscription: successfully created subscription '{name}'\n"
        ));
    }
    for (key, value) in &options {
        registry.set_subscription_option(name, key, value);
    }
}

/// `opcuaConnect(session)` / `opcuaDisconnect(session name)`
/// (`iocshIntegration.cpp:437-458`, `:477-498`): every session the pattern
/// matches, and nothing printed when none does.
pub fn control_command(registry: &Registry, pattern: Option<&str>, control: Control) {
    let Some(pattern) = pattern else {
        let missing = match control {
            Control::Connect => "session name pattern",
            Control::Disconnect => "session name",
        };
        errlog_vprintf(format_args!("ERROR : missing argument #1 ({missing})\n"));
        return;
    };
    for session in registry.glob_sessions(pattern) {
        session.control(control);
    }
}

/// `opcuaMapNamespace(session, namespace index, namespace URI)`
/// (`iocshIntegration.cpp:527-560`).
///
/// Upstream C defect fixed at source: the C reports an out-of-range index and
/// then maps namespace 0, the OPC UA namespace, to the URI; here the mapping
/// is not made.
pub fn map_namespace_command(
    registry: &Registry,
    session: Option<&str>,
    index: i64,
    uri: Option<&str>,
) {
    let mut ok = true;
    let handle = match session {
        None => {
            errlog_vprintf(format_args!("missing argument #1 (session name)\n"));
            ok = false;
            None
        }
        Some(name) => {
            let handle = registry.session(name).ok();
            if handle.is_none() {
                errlog_vprintf(format_args!("'{name}' - no such session\n"));
                ok = false;
            }
            handle
        }
    };
    let index = match u16::try_from(index) {
        Ok(index) => Some(index),
        Err(_) => {
            errlog_vprintf(format_args!(
                "invalid argument #2 (namespace index) '{}'\n",
                index as i32
            ));
            None
        }
    };
    if uri.is_none() {
        errlog_vprintf(format_args!("missing argument #3 (namespace URI)\n"));
        ok = false;
    }
    if let (true, Some(handle), Some(index), Some(uri)) = (ok, handle, index, uri) {
        handle.add_namespace_mapping(index, uri);
    }
}

/// `opcuaClientCertificate`'s argument check (`iocshIntegration.cpp:614-633`):
/// both files, or the C's lines for the missing ones.
pub fn client_certificate_args<'a>(
    certificate: Option<&'a str>,
    key: Option<&'a str>,
) -> Option<(&'a str, &'a str)> {
    let certificate = certificate.filter(|s| !s.is_empty());
    let key = key.filter(|s| !s.is_empty());
    if certificate.is_none() {
        errlog_vprintf(format_args!("missing argument #1 (certificate file)\n"));
    }
    if key.is_none() {
        errlog_vprintf(format_args!("missing argument #2 (private key file)\n"));
    }
    certificate.zip(key)
}

/// `opcuaSetupPKI`'s argument check (`iocshIntegration.cpp:666-697`): the
/// one-argument form is a first argument and no second; otherwise all four,
/// or the C's line for each missing one.
pub fn setup_pki_args(args: [Option<&str>; 4]) -> Option<[&str; 4]> {
    if let [Some(root), None, _, _] = args {
        return Some([root, "", "", ""]);
    }
    let mut ok = true;
    for (i, arg) in args.iter().enumerate() {
        if arg.is_none_or(str::is_empty) {
            errlog_vprintf(format_args!(
                "missing argument #{} - {}\n",
                i + 1,
                SETUP_PKI_ARGS[i]
            ));
            ok = false;
        }
    }
    ok.then(|| args.map(|a| a.unwrap_or_default()))
}

/// `opcuaOptions(pattern, [options])` (`iocshIntegration.cpp:305-365`): every
/// option goes to each session the pattern matches, or, when it matches none,
/// to each subscription it matches.
pub fn options_command(registry: &Registry, pattern: Option<&str>, options: &[&str]) {
    let Some(pattern) = pattern.filter(|p| !p.is_empty()) else {
        errlog_vprintf(format_args!("missing argument #1 (pattern for name)\n"));
        return;
    };
    if pattern == "help" {
        println!("{OPTIONS_USAGE}");
        return;
    }
    if options.is_empty() {
        errlog_vprintf(format_args!("missing argument #2 (options)\n"));
        return;
    }
    let sessions = registry.glob_sessions(pattern);
    let applied = if !sessions.is_empty() {
        walk_options(options, |key, value| {
            sessions.iter().try_for_each(|s| s.set_option(key, value))
        })
    } else {
        let subscriptions = registry.glob_subscriptions(pattern);
        if subscriptions.is_empty() {
            errlog_vprintf(format_args!("No matches for pattern '{pattern}'\n"));
            return;
        }
        walk_options(options, |key, value| {
            for name in &subscriptions {
                registry.set_subscription_option(name, key, value);
            }
            Ok(())
        })
    };
    if let Err(e) = applied {
        eprintln!("ERROR : {e}");
    }
}

/// `opcuaShow(pattern, verbosity)` (`iocshIntegration.cpp:379-410`): what every
/// matching session and subscription is, and — from verbosity 1 — the items on
/// it and the records bound to them.
pub fn show(registry: &Registry, pattern: &str, level: i32) -> String {
    let pattern = if pattern.is_empty() { "*" } else { pattern };
    let mut out = String::new();
    let mut sessions = registry.sessions();
    sessions.sort_by(|a, b| a.name.cmp(&b.name));

    for session in sessions {
        let subscriptions = session.subscriptions.lock().clone();
        let items = session.items.lock().clone();
        let matched = glob_match(pattern, &session.name);
        let matched_subscriptions: Vec<_> = subscriptions
            .iter()
            .filter(|s| glob_match(pattern, &s.name))
            .collect();
        if !matched && matched_subscriptions.is_empty() {
            continue;
        }

        if matched {
            let config = session.config.lock().clone();
            out.push_str(&format!(
                "session {} ({}) {} items={} subscriptions={} status={}\n",
                session.name,
                config.url,
                if config.autoconnect {
                    "autoconnect"
                } else {
                    "manual"
                },
                items.len(),
                subscriptions.len(),
                status_of(*session.status.lock()),
            ));
        }
        for subscription in matched_subscriptions {
            out.push_str(&format!(
                "subscription {} on session {} interval={}ms priority={}\n",
                subscription.name,
                session.name,
                subscription.publishing_interval,
                subscription.priority,
            ));
        }
        if level < 1 || !matched {
            continue;
        }
        for item in &items {
            let item = item.lock();
            out.push_str(&format!(
                "  item {} status={} records={}\n",
                item.node_id,
                status_of(item.state),
                item.leaves.len(),
            ));
            if level < 2 {
                continue;
            }
            for leaf in &item.leaves {
                let leaf = leaf.lock();
                out.push_str(&format!(
                    "    record {} element='{}'\n",
                    leaf.record, leaf.link.element,
                ));
            }
        }
    }
    out
}

/// `opcuaShowSecurity` (`iocshIntegration.cpp:576-590`), as far as it is
/// derivable here: no name, or an empty one, is the client report; a name is
/// that session's, and nothing when there is no such session.
///
/// Accepted-and-documented: the C also asks the server for its endpoints and
/// prints, per endpoint, whether the client would accept it. That is a discovery
/// service call on a live connection; the client boundary here
/// ([`crate::client::UaConnection`]) has no `GetEndpoints`, so what is shown is
/// what the client and the session are configured with, not what the server
/// offers.
pub fn show_security(
    registry: &Registry,
    client: &ClientSecurity,
    session: Option<&str>,
) -> String {
    let mut out = String::new();
    let Some(name) = session.filter(|s| !s.is_empty()) else {
        out.push_str(&format!(
            "client certificate: {}\nclient private key: {}\nPKI store: {}\n",
            client.certificate_path.as_deref().unwrap_or("(none)"),
            client.private_key_path.as_deref().unwrap_or("(none)"),
            client.pki_root.as_deref().unwrap_or("(default)"),
        ));
        // The rejected-certificate directory is fixed by `async-opcua`'s PKI
        // layout, where the C's `opcuaSaveRejected` makes it settable.
        out.push_str("rejected certificates: <PKI store>/rejected\n");
        return out;
    };
    if let Ok(handle) = registry.session(name) {
        let config = handle.config.lock().clone();
        out.push_str(&format!(
            "session {}: mode={:?} policy={} identity={}\n",
            handle.name,
            config.security_mode,
            config.security_policy.as_deref().unwrap_or("(best)"),
            config.identity_file.as_deref().unwrap_or("anonymous"),
        ));
    }
    out
}

fn status_of(status: ConnectionStatus) -> &'static str {
    match status {
        ConnectionStatus::Down => "down",
        ConnectionStatus::InitialRead => "initial read",
        ConnectionStatus::InitialWrite => "initial write",
        ConnectionStatus::Up => "up",
    }
}

/// `epicsStrGlobMatch` — `*` matches any run of characters, `?` matches one.
pub fn glob_match(pattern: &str, name: &str) -> bool {
    let (p, n): (Vec<char>, Vec<char>) = (pattern.chars().collect(), name.chars().collect());
    // The classic two-cursor glob: on a `*`, remember where to resume if the
    // rest of the pattern fails, and grow what the `*` swallowed.
    let (mut pi, mut ni) = (0usize, 0usize);
    let (mut star, mut resume) = (None, 0usize);
    while ni < n.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == n[ni]) {
            pi += 1;
            ni += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            resume = ni;
            pi += 1;
        } else if let Some(s) = star {
            pi = s + 1;
            resume += 1;
            ni = resume;
        } else {
            return false;
        }
    }
    p[pi..].iter().all(|c| *c == '*')
}

/// The registry an IOC hands to every record's device support and to every
/// command.
pub type SharedRegistry = Arc<Registry>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_argument_carries_colon_separated_options() {
        let pairs = |v: &[(&str, &str)]| -> Vec<(String, String)> {
            v.iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect()
        };
        assert_eq!(
            parse_options(&["sec-mode=Sign:debug=1", "batch-nodes=10"]),
            pairs(&[("sec-mode", "Sign"), ("debug", "1"), ("batch-nodes", "10")])
        );
        // An escaped colon stays in the value; empty options are skipped.
        assert_eq!(
            parse_options(&["sec-id=C\\:/creds::"]),
            pairs(&[("sec-id", "C:/creds")])
        );
        // Not exactly one `=`: ignored, the rest still taken.
        assert_eq!(
            parse_options(&["debug:a=b=c:=1:nodes-max=5"]),
            pairs(&[("", "1"), ("nodes-max", "5")])
        );
    }

    #[test]
    fn a_glob_matches_the_way_iocsh_globs_do() {
        assert!(glob_match("*", "anything"));
        assert!(glob_match("S*", "Sess1"));
        assert!(glob_match("*1", "Sess1"));
        assert!(glob_match("S?ss1", "Sess1"));
        assert!(glob_match("Sess1", "Sess1"));
        assert!(!glob_match("S?s1", "Sess1"));
        assert!(!glob_match("Sess2", "Sess1"));
        assert!(glob_match("*ss*", "Sess1"));
        assert!(!glob_match("", "Sess1"));
        assert!(glob_match("", ""));
    }
}
