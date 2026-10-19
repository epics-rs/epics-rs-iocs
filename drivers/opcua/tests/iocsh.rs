//! The iocsh command bodies (`iocshIntegration.cpp`): the console lines each
//! prints, captured from the errlog stream. The expected lines are what the C
//! `opcuaIoc` (opcua 0.12.0-dev, open62541 v1.2-29-g875d33a9) prints for the
//! same command line.

use std::sync::Arc;

use async_trait::async_trait;
use parking_lot::Mutex;

use epics_rs::base::runtime::log::{errlog_add_listener, errlog_flush, errlog_remove_listener};
use opcua::client::{Identity, UaConnection, UaConnector};
use opcua::ioc;
use opcua::registry::Registry;
use opcua::session::{Control, SessionConfig};

struct Offline;

#[async_trait]
impl UaConnector for Offline {
    async fn connect(
        &self,
        _config: &SessionConfig,
        _identity: &Identity,
    ) -> Result<Arc<dyn UaConnection>, String> {
        Err("no server".to_string())
    }
}

fn registry() -> Arc<Registry> {
    Registry::new(Arc::new(Offline))
}

/// The errlog lines `f` prints.
fn lines(f: impl FnOnce()) -> Vec<String> {
    let lines = Arc::new(Mutex::new(Vec::<String>::new()));
    let id = errlog_add_listener({
        let lines = lines.clone();
        move |line: &str| lines.lock().push(line.to_string())
    });
    f();
    errlog_flush();
    errlog_remove_listener(id);
    lines.lock().clone()
}

#[test]
fn a_session_without_arguments_is_refused_with_both_lines() {
    let registry = registry();
    assert_eq!(
        lines(|| ioc::session_command(&registry, None, None, &[])),
        [
            "missing argument #1 (session name)\n",
            "missing argument #2 (server URL)\n",
            "ERROR - no session created\n",
        ]
    );
    assert!(registry.sessions().is_empty());
}

#[test]
fn a_session_name_with_a_space_is_refused() {
    let registry = registry();
    assert_eq!(
        lines(|| ioc::session_command(&registry, Some("a b"), Some("opc.tcp://h:4840"), &[])),
        [
            "invalid argument #1 (session name) 'a b'\n",
            "ERROR - no session created\n",
        ]
    );
}

#[test]
fn a_session_reports_bad_options_before_it_is_created() {
    let registry = registry();
    assert_eq!(
        lines(|| ioc::session_command(
            &registry,
            Some("S1"),
            Some("opc.tcp://h:4840"),
            &["debug=1:bad", "nodes-max=5"]
        )),
        [
            "option 'bad' must follow 'key=value' format - ignored\n",
            "opcuaSession: successfully created session 'S1'\n",
        ]
    );
    let session = registry.session("S1").unwrap();
    assert_eq!(session.config.lock().debug, 1);
    // Without a nonzero debug option there is no success line.
    assert_eq!(
        lines(|| ioc::session_command(
            &registry,
            Some("S2"),
            Some("opc.tcp://h:4841"),
            &["debug=0"]
        )),
        Vec::<String>::new()
    );
    assert!(registry.session("S2").is_ok());
}

/// Sessions and subscriptions share `RegistryKeyNamespace::global`.
#[test]
fn a_name_any_session_or_subscription_holds_is_in_use() {
    let registry = registry();
    ioc::session_command(&registry, Some("S1"), Some("opc.tcp://h:4840"), &[]);
    ioc::subscription_command(&registry, Some("SUB1"), Some("S1"), 0.0, &[]);
    for name in ["S1", "SUB1"] {
        assert_eq!(
            lines(|| ioc::session_command(&registry, Some(name), Some("opc.tcp://x"), &[])),
            [
                format!("session name {name} already in use\n"),
                "ERROR - no session created\n".to_string(),
            ]
        );
    }
    // A subscription named like a session passes the subscription checks and is
    // refused by `createSubscription`, with no line.
    assert_eq!(
        lines(|| ioc::subscription_command(&registry, Some("S1"), Some("S1"), 100.0, &[])),
        Vec::<String>::new()
    );
    assert_eq!(registry.glob_subscriptions("*"), ["SUB1"]);
}

#[test]
fn a_subscription_without_arguments_is_refused_with_both_lines() {
    let registry = registry();
    assert_eq!(
        lines(|| ioc::subscription_command(&registry, None, None, 0.0, &[])),
        [
            "missing argument #1 (subscription name)\n",
            "missing argument #2 (session name)\n",
            "ERROR - no subscription created\n",
        ]
    );
}

#[test]
fn a_subscription_checks_its_name_session_and_interval() {
    let registry = registry();
    ioc::session_command(&registry, Some("S1"), Some("opc.tcp://h:4840"), &[]);
    assert_eq!(
        lines(|| ioc::subscription_command(
            &registry,
            Some("SUB1"),
            Some("S1"),
            -5.0,
            &["debug=1"]
        )),
        [
            "invalid argument #3 (publishing interval) '-5.000000' - ignored\n",
            "opcuaSubscription: successfully created subscription 'SUB1'\n",
        ]
    );
    let session = registry.session("S1").unwrap();
    let subscriptions = session.subscriptions.lock().clone();
    assert_eq!(subscriptions[0].publishing_interval, 100.0);
    assert_eq!(subscriptions[0].debug, 1);
    assert_eq!(
        lines(|| ioc::subscription_command(&registry, Some("SUB1"), Some("S1"), 0.0, &[])),
        [
            "subscription name SUB1 already in use\n",
            "ERROR - no subscription created\n",
        ]
    );
    assert_eq!(
        lines(|| ioc::subscription_command(&registry, Some("a b"), Some("NOPE"), 0.0, &[])),
        [
            "invalid argument #1 (subscription name) 'a b'\n",
            "session NOPE does not exist\n",
            "ERROR - no subscription created\n",
        ]
    );
}

#[test]
fn connect_and_disconnect_name_their_missing_argument() {
    let registry = registry();
    assert_eq!(
        lines(|| ioc::control_command(&registry, None, Control::Connect)),
        ["ERROR : missing argument #1 (session name pattern)\n"]
    );
    assert_eq!(
        lines(|| ioc::control_command(&registry, None, Control::Disconnect)),
        ["ERROR : missing argument #1 (session name)\n"]
    );
    // A pattern that matches nothing prints nothing.
    assert_eq!(
        lines(|| ioc::control_command(&registry, Some("NOPE"), Control::Connect)),
        Vec::<String>::new()
    );
}

#[test]
fn a_namespace_mapping_checks_each_argument() {
    let registry = registry();
    ioc::session_command(&registry, Some("S1"), Some("opc.tcp://h:4840"), &[]);
    assert_eq!(
        lines(|| ioc::map_namespace_command(&registry, None, 0, None)),
        [
            "missing argument #1 (session name)\n",
            "missing argument #3 (namespace URI)\n",
        ]
    );
    assert_eq!(
        lines(|| ioc::map_namespace_command(&registry, Some("NOPE"), 2, Some("urn:x"))),
        ["'NOPE' - no such session\n"]
    );
    assert_eq!(
        lines(|| ioc::map_namespace_command(&registry, Some("S1"), -1, Some("urn:x"))),
        ["invalid argument #2 (namespace index) '-1'\n"]
    );
    // The C goes on to map namespace 0; that is not copied.
    let session = registry.session("S1").unwrap();
    assert!(session.config.lock().namespace_map.is_empty());
}

#[test]
fn client_certificate_and_pki_name_each_missing_argument() {
    assert_eq!(
        lines(|| assert_eq!(ioc::client_certificate_args(None, Some("")), None)),
        [
            "missing argument #1 (certificate file)\n",
            "missing argument #2 (private key file)\n",
        ]
    );
    assert_eq!(
        lines(|| assert_eq!(ioc::setup_pki_args([None; 4]), None)),
        [
            "missing argument #1 - PKI / server certs location\n",
            "missing argument #2 - server revocation lists location\n",
            "missing argument #3 - issuer certs location\n",
            "missing argument #4 - issuer revocation lists location\n",
        ]
    );
    // An empty first argument with a second is the four-argument form.
    assert_eq!(
        lines(|| assert_eq!(ioc::setup_pki_args([Some(""), Some("b"), None, None]), None)),
        [
            "missing argument #1 - PKI / server certs location\n",
            "missing argument #3 - issuer certs location\n",
            "missing argument #4 - issuer revocation lists location\n",
        ]
    );
    assert_eq!(
        lines(|| assert_eq!(
            ioc::setup_pki_args([Some("/pki"), None, None, None]),
            Some(["/pki", "", "", ""])
        )),
        Vec::<String>::new()
    );
}

#[test]
fn the_usage_texts_are_the_c_ones() {
    assert!(ioc::SESSION_USAGE.ends_with(
        "[options]  list of options in 'key=value' format\n           (see 'help opcuaOptions' for a list of valid options)\n"
    ));
    assert!(ioc::SUBSCRIPTION_USAGE.ends_with(
        "\n                     (see 'help opcuaOptions' for a list of valid options)\n"
    ));
    assert!(ioc::OPTIONS_USAGE.ends_with(
        "Valid subscription options are:\ndebug              debug level [default 0 = no debug]\npriority           priority level [default 0(lowest) .. 255]\n"
    ));
}

#[test]
fn options_name_their_missing_arguments() {
    let registry = registry();
    for pattern in [None, Some("")] {
        assert_eq!(
            lines(|| ioc::options_command(&registry, pattern, &["debug=1"])),
            ["missing argument #1 (pattern for name)\n"]
        );
    }
    assert_eq!(
        lines(|| ioc::options_command(&registry, Some("S1"), &[])),
        ["missing argument #2 (options)\n"]
    );
    assert_eq!(
        lines(|| ioc::options_command(&registry, Some("NOPE"), &["debug=1"])),
        ["No matches for pattern 'NOPE'\n"]
    );
}

/// A pattern that matches a session never reaches a subscription; one that
/// matches none falls back to the subscriptions.
#[test]
fn options_go_to_the_matching_sessions_before_any_subscription() {
    let registry = registry();
    ioc::session_command(&registry, Some("S1"), Some("opc.tcp://h:4840"), &[]);
    ioc::subscription_command(&registry, Some("SUB1"), Some("S1"), 0.0, &[]);
    let session = registry.session("S1").unwrap();
    let subscription_debug = || session.subscriptions.lock()[0].debug;

    assert!(lines(|| ioc::options_command(&registry, Some("S*"), &["debug=2"])).is_empty());
    assert_eq!(session.config.lock().debug, 2);
    assert_eq!(subscription_debug(), 0);

    assert!(lines(|| ioc::options_command(&registry, Some("SU*"), &["debug=3"])).is_empty());
    assert_eq!(subscription_debug(), 3);
    assert_eq!(session.config.lock().debug, 2);
}

/// Each option goes to every matching session before the next option, and an
/// option a session refuses ends the command (the C's catch).
#[test]
fn an_option_reaches_every_session_before_the_next_one() {
    let registry = registry();
    for (name, url) in [("S1", "opc.tcp://h:4840"), ("S2", "opc.tcp://h:4841")] {
        ioc::session_command(&registry, Some(name), Some(url), &[]);
    }
    assert_eq!(
        lines(|| ioc::options_command(
            &registry,
            Some("S?"),
            &["nodes-max=3:bad", "autoconnect=q", "nodes-max=9"]
        )),
        ["option 'bad' must follow 'key=value' format - ignored\n"]
    );
    for name in ["S1", "S2"] {
        assert_eq!(
            registry.session(name).unwrap().config.lock().nodes_max,
            3,
            "{name}"
        );
    }
}
