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
use crate::link::split_escaped;
use crate::queue::ConnectionStatus;
use crate::record::OpcuaItemRecord;
use crate::registry::Registry;

/// The `opcuaItem` record-type factory, for
/// `IocApplication::register_record_type`.
pub fn item_record_factory() -> (&'static str, RecordFactory) {
    (
        "opcuaItem",
        Box::new(|| Box::new(OpcuaItemRecord::default())),
    )
}

/// The `key=value` options of an iocsh option tail (`iocshIntegration.cpp:150-163`):
/// each argument holds `:`-separated options, and one that is not exactly
/// `key=value` is reported and ignored.
///
/// Framework gap: `epics_rs`'s iocsh has no variadic argument type
/// ([`ArgType`](epics_rs::base::server::iocsh::registry::ArgType) is String,
/// Int or Double), so the IOC declares a fixed number of optional option slots
/// where the C takes an `iocshArgArgv` tail.
pub fn parse_options<'a>(tokens: impl IntoIterator<Item = &'a str>) -> Vec<(String, String)> {
    let mut options = Vec::new();
    for token in tokens {
        for opt in split_escaped(token, ':') {
            if opt.is_empty() {
                continue;
            }
            match <[String; 2]>::try_from(split_escaped(&opt, '=')) {
                Ok([key, value]) => options.push((key, value)),
                Err(_) => errlog_vprintf(format_args!(
                    "option '{opt}' must follow 'key=value' format - ignored\n"
                )),
            }
        }
    }
    options
}

/// `opcuaOptions(pattern, [options])` — the pattern selects sessions and
/// subscriptions by name (`iocshIntegration.cpp:299-360`).
///
/// Returns how many names it applied the options to; none is an error, as it is
/// in the C ("no matching session or subscription").
pub fn set_options(
    registry: &Registry,
    pattern: &str,
    options: &[(String, String)],
) -> Result<usize, String> {
    let names: Vec<String> = registry
        .names()
        .into_iter()
        .filter(|name| glob_match(pattern, name))
        .collect();
    if names.is_empty() {
        return Err(format!("no session or subscription matches '{pattern}'"));
    }
    for name in &names {
        for (key, value) in options {
            // The C's catch: report, and apply nothing further.
            if let Err(e) = registry.set_option(name, key, value) {
                eprintln!("ERROR : {e}");
                return Ok(names.len());
            }
        }
    }
    Ok(names.len())
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

/// `opcuaShowSecurity` (`iocshIntegration.cpp:573-590`), as far as it is
/// derivable here.
///
/// Accepted-and-documented: the C also asks the server for its endpoints and
/// prints, per endpoint, whether the client would accept it. That is a discovery
/// service call on a live connection; the client boundary here
/// ([`crate::client::UaConnection`]) has no `GetEndpoints`, so what is shown is
/// what the client and the session are configured with, not what the server
/// offers.
pub fn show_security(registry: &Registry, client: &ClientSecurity, session: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "client certificate: {}\nclient private key: {}\nPKI store: {}\n",
        client.certificate_path.as_deref().unwrap_or("(none)"),
        client.private_key_path.as_deref().unwrap_or("(none)"),
        client.pki_root.as_deref().unwrap_or("(default)"),
    ));
    // The rejected-certificate directory is fixed by `async-opcua`'s PKI layout,
    // where the C's `opcuaSaveRejected` makes it settable.
    out.push_str("rejected certificates: <PKI store>/rejected\n");

    let mut sessions = registry.sessions();
    sessions.sort_by(|a, b| a.name.cmp(&b.name));
    for handle in sessions {
        if !session.is_empty() && !glob_match(session, &handle.name) {
            continue;
        }
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
            parse_options(["sec-mode=Sign:debug=1", "batch-nodes=10"]),
            pairs(&[("sec-mode", "Sign"), ("debug", "1"), ("batch-nodes", "10")])
        );
        // An escaped colon stays in the value; empty options are skipped.
        assert_eq!(
            parse_options(["sec-id=C\\:/creds::"]),
            pairs(&[("sec-id", "C:/creds")])
        );
        // Not exactly one `=`: ignored, the rest still taken.
        assert_eq!(
            parse_options(["debug:a=b=c:=1:nodes-max=5"]),
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
