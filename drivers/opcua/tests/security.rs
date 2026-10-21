//! `Session::securityPolicyString`, as the "connected as" line prints it.

use opcua::client::security_policy_string;

#[cfg(unix)]
mod console;

#[test]
fn a_policy_uri_prints_as_c_names_it() {
    assert_eq!(security_policy_string(""), "None");
    assert_eq!(
        security_policy_string("http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256"),
        "Basic256Sha256"
    );
    assert_eq!(
        security_policy_string("http://example.org/SecurityPolicy#Rot13"),
        "Rot13 (unsupported)"
    );
    assert_eq!(security_policy_string("nonsense"), "Invalid");
}

mod setup_security {
    use async_opcua::types::{
        EndpointDescription, MessageSecurityMode, StatusCode, UserTokenPolicy, UserTokenType,
    };
    use opcua::client::{AsyncOpcuaConnector, SetupError, UaConnector};
    use opcua::session::SessionConfig;

    const NONE: &str = "http://opcfoundation.org/UA/SecurityPolicy#None";
    const SHA256: &str = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";

    /// Privileged port 1: no test can bind it, so a connect is always refused.
    /// A freed ephemeral port can be taken by a concurrent test's listener.
    const DEAD_PORT: u16 = 1;

    fn endpoint(
        url: &str,
        mode: MessageSecurityMode,
        policy: &str,
        level: u8,
    ) -> EndpointDescription {
        EndpointDescription {
            endpoint_url: url.into(),
            security_mode: mode,
            security_policy_uri: policy.into(),
            security_level: level,
            ..Default::default()
        }
    }

    fn offered() -> Vec<EndpointDescription> {
        vec![
            endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 0),
            endpoint("opc.tcp://srv:4840", MessageSecurityMode::Sign, SHA256, 3),
            endpoint(
                "opc.tcp://srv:4840",
                MessageSecurityMode::SignAndEncrypt,
                SHA256,
                7,
            ),
            endpoint(
                "https://srv:4843",
                MessageSecurityMode::SignAndEncrypt,
                SHA256,
                9,
            ),
            endpoint(
                "opc.tcp://srv:4840",
                MessageSecurityMode::SignAndEncrypt,
                "http://example.org/SecurityPolicy#Rot13",
                8,
            ),
        ]
    }

    #[test]
    fn best_takes_the_highest_supported_opc_tcp_level_at_the_session_url() {
        let config = SessionConfig::new("s", "opc.tcp://10.0.0.1:4840");
        let (number, e) = config.select_endpoint(&offered()).unwrap();
        assert_eq!(number, 2);
        assert_eq!(e.security_level, 7);
        assert_eq!(e.endpoint_url.as_ref(), "opc.tcp://10.0.0.1:4840");
    }

    #[test]
    fn a_requested_mode_and_policy_narrow_the_pick() {
        let mut config = SessionConfig::new("s", "opc.tcp://srv:4840");
        config.set_option("sec-mode", "Sign").unwrap();
        assert_eq!(
            config.select_endpoint(&offered()).unwrap().1.security_level,
            3
        );
        // A requested None policy matches any policy, as C's find("#None").
        config.set_option("sec-policy", "None").unwrap();
        assert_eq!(
            config.select_endpoint(&offered()).unwrap().1.security_level,
            3
        );
        config.set_option("sec-policy", "Basic128Rsa15").unwrap();
        assert!(config.select_endpoint(&offered()).is_none());
    }

    const BINARY: &str = "http://opcfoundation.org/UA-Profile/Transport/uatcp-uasc-uabinary";

    fn token(id: &str, token_type: UserTokenType, policy: &str) -> UserTokenPolicy {
        UserTokenPolicy {
            policy_id: id.into(),
            token_type,
            security_policy_uri: policy.into(),
            ..Default::default()
        }
    }

    fn with_tokens(
        mut e: EndpointDescription,
        tokens: Vec<UserTokenPolicy>,
    ) -> EndpointDescription {
        e.user_identity_tokens = Some(tokens);
        e
    }

    /// `responseGetEndpoints` (open62541.c:45155-45320) takes the first endpoint
    /// in the server's order, not the most secure, and the first token policy on
    /// it for the identity.
    #[test]
    fn a_none_session_takes_the_first_none_endpoint_with_a_token_for_it() {
        let config = SessionConfig::new("s", "opc.tcp://10.0.0.1:4840");
        let anonymous = token("anon", UserTokenType::Anonymous, "");
        let user = token("user", UserTokenType::UserName, SHA256);
        let mut https = with_tokens(
            endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 9),
            vec![anonymous.clone()],
        );
        https.transport_profile_uri =
            "http://opcfoundation.org/UA-Profile/Transport/https-uabinary".into();
        let mut binary = with_tokens(
            endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 1),
            vec![user.clone(), anonymous.clone()],
        );
        binary.transport_profile_uri = BINARY.into();
        let offered = [
            with_tokens(
                endpoint("opc.tcp://srv:4840", MessageSecurityMode::Sign, SHA256, 5),
                vec![anonymous.clone()],
            ),
            with_tokens(
                endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 8),
                vec![user.clone()],
            ),
            https,
            binary,
            with_tokens(
                endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 7),
                vec![token("later", UserTokenType::Anonymous, "")],
            ),
        ];
        let e = config
            .connect_endpoint(&offered, None, UserTokenType::Anonymous)
            .unwrap();
        assert_eq!(e.endpoint_url.as_ref(), "opc.tcp://10.0.0.1:4840");
        assert_eq!(e.user_identity_tokens, Some(vec![anonymous]));
    }

    /// The level the session reports is `setupSecurity`'s: 0 without security
    /// (`SessionOpen62541.cpp:1028`), whatever the server advertises.
    #[test]
    fn a_none_session_reports_security_level_zero() {
        let config = SessionConfig::new("s", "opc.tcp://srv:4840");
        let offered = [with_tokens(
            endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 1),
            vec![token("anon", UserTokenType::Anonymous, "")],
        )];
        let e = config
            .connect_endpoint(&offered, None, UserTokenType::Anonymous)
            .unwrap();
        assert_eq!(e.security_level, 0);
    }

    /// A non-anonymous token policy whose own security policy the client lacks
    /// is passed over.
    #[test]
    fn a_token_policy_on_an_unsupported_policy_is_passed_over() {
        let config = SessionConfig::new("s", "opc.tcp://srv:4840");
        let rot13 = token(
            "rot13",
            UserTokenType::UserName,
            "http://example.org/SecurityPolicy#Rot13",
        );
        let user = token("user", UserTokenType::UserName, SHA256);
        let offered = [with_tokens(
            endpoint(
                "opc.tcp://srv:4840",
                MessageSecurityMode::SignAndEncrypt,
                SHA256,
                7,
            ),
            vec![rot13, user.clone()],
        )];
        let e = config
            .connect_endpoint(
                &offered,
                Some(&endpoint(
                    "opc.tcp://srv:4840",
                    MessageSecurityMode::SignAndEncrypt,
                    SHA256,
                    6,
                )),
                UserTokenType::UserName,
            )
            .unwrap();
        assert_eq!(e.user_identity_tokens, Some(vec![user]));
    }

    /// No endpoint, and an endpoint with no token policy for the identity, are
    /// both BadInternalError in open62541.
    #[test]
    fn no_endpoint_or_no_token_is_bad_internal_error() {
        let config = SessionConfig::new("s", "opc.tcp://srv:4840");
        let offered = [
            with_tokens(
                endpoint("opc.tcp://srv:4840", MessageSecurityMode::None, NONE, 0),
                vec![token("anon", UserTokenType::Anonymous, "")],
            ),
            with_tokens(
                endpoint("opc.tcp://srv:4840", MessageSecurityMode::Sign, SHA256, 3),
                vec![token("anon", UserTokenType::Anonymous, "")],
            ),
        ];
        assert_eq!(
            config.connect_endpoint(&offered, None, UserTokenType::UserName),
            Err(StatusCode::BadInternalError)
        );
        assert_eq!(
            config.connect_endpoint(
                &offered,
                Some(&endpoint(
                    "opc.tcp://srv:4840",
                    MessageSecurityMode::Sign,
                    NONE,
                    6
                )),
                UserTokenType::Anonymous
            ),
            Err(StatusCode::BadInternalError)
        );
        // A secured session reports the level of `setupSecurity`'s pick.
        assert_eq!(
            config
                .connect_endpoint(
                    &offered,
                    Some(&endpoint(
                        "opc.tcp://srv:4840",
                        MessageSecurityMode::Sign,
                        SHA256,
                        6
                    )),
                    UserTokenType::Anonymous
                )
                .unwrap()
                .security_level,
            6
        );
    }

    #[tokio::test]
    async fn a_dead_discovery_url_is_cant_connect() {
        let pki = std::env::temp_dir().join(format!("opcua-cantconnect-{}", std::process::id()));
        let mut config = SessionConfig::new("s", format!("opc.tcp://127.0.0.1:{DEAD_PORT}"));
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        let err = AsyncOpcuaConnector::new()
            .setup(&config)
            .await
            .err()
            .unwrap();
        let _ = std::fs::remove_dir_all(&pki);
        assert_eq!(
            err,
            SetupError {
                status: Some(StatusCode::BadDisconnect),
                message: "security discovery and setup failed with status cantConnect".into(),
            }
        );
    }

    /// `SessionOpen62541::setupSecurity` discovers only over `opc.tcp`, and says
    /// so whatever the debug level (`SessionOpen62541.cpp:1155-1158`).
    #[tokio::test]
    async fn a_secured_session_on_another_scheme_is_fatal() {
        use std::sync::Arc;

        use epics_rs::base::runtime::log::{
            errlog_add_listener, errlog_flush, errlog_remove_listener,
        };
        use parking_lot::Mutex;

        let pki = std::env::temp_dir().join(format!("opcua-fatal-{}", std::process::id()));
        let mut config = SessionConfig::new("s", "http://127.0.0.1:4840");
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        let lines = Arc::new(Mutex::new(Vec::<String>::new()));
        let id = errlog_add_listener({
            let lines = lines.clone();
            move |line: &str| lines.lock().push(line.to_string())
        });
        let err = AsyncOpcuaConnector::new()
            .setup(&config)
            .await
            .err()
            .unwrap();
        errlog_flush();
        errlog_remove_listener(id);
        let _ = std::fs::remove_dir_all(&pki);
        assert_eq!(
            err,
            SetupError::new("security discovery and setup failed with status fatal")
        );
        assert_eq!(
            *lines.lock(),
            ["OPC UA session s: fatal - only URLs of type 'opc.tcp' supported\n"]
        );
    }

    async fn connect_without_security(url: &str, tag: &str) -> StatusCode {
        let pki = std::env::temp_dir().join(format!("opcua-{tag}-{}", std::process::id()));
        let mut config = SessionConfig::new("s", url);
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        config.set_option("sec-mode", "None").unwrap();
        let Ok(mut client) = AsyncOpcuaConnector::new().setup(&config).await else {
            panic!("the setup without security reads nothing");
        };
        let connect = client.connect();
        let err = tokio::time::timeout(std::time::Duration::from_secs(20), connect)
            .await
            .expect("a failed connect returns")
            .err()
            .unwrap();
        let _ = std::fs::remove_dir_all(&pki);
        err
    }

    /// C `SessionOpen62541::connect` prints `UA_Client_connect`'s status on
    /// every failed connect (`SessionOpen62541.cpp:586-589`); a refused TCP
    /// connect is BadDisconnect (`UA_ClientConnectionTCP_poll`).
    #[tokio::test]
    async fn a_dead_session_url_is_a_connect_failure_with_its_status() {
        let err =
            connect_without_security(&format!("opc.tcp://127.0.0.1:{DEAD_PORT}"), "deadsession")
                .await;
        assert_eq!(err, StatusCode::BadDisconnect);
    }

    /// open62541's `initConnect` closes a connection whose host does not
    /// resolve before any socket exists.
    #[tokio::test]
    async fn a_session_url_that_does_not_resolve_is_connection_closed() {
        let err = connect_without_security("opc.tcp://no-such-host.invalid:4840", "nohost").await;
        assert_eq!(err, StatusCode::BadConnectionClosed);
    }

    /// The session's own lines from `setupClientSecurityInfo`, the PKI set-up
    /// and `setupSecurity`, at debug 1, after setting it up on `url`.
    #[cfg(unix)]
    async fn setup_lines(url: &str, options: &[(&str, &str)], tag: &str) -> Vec<String> {
        let pki = std::env::temp_dir().join(format!("opcua-{tag}-{}", std::process::id()));
        let mut config = SessionConfig::new("s", url);
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        for (name, value) in options {
            config.set_option(name, value).unwrap();
        }
        config.set_option("debug", "1").unwrap();
        let console = crate::console::Console::capture();
        let _ = AsyncOpcuaConnector::new().setup(&config).await;
        let lines = console.lines();
        let _ = std::fs::remove_dir_all(&pki);
        lines
            .into_iter()
            .filter(|l| l.starts_with("Session s:"))
            .collect()
    }

    /// `SessionOpen62541.cpp:1029-1040`, after the client security info
    /// (`:1219-1225`) and the PKI provider (`:516-522`).
    #[cfg(unix)]
    #[tokio::test]
    async fn a_session_without_security_prints_its_setup() {
        let lines = setup_lines(
            "opc.tcp://127.0.0.1:4840",
            &[("sec-mode", "None")],
            "dbgnone",
        )
        .await;
        assert_eq!(
            lines,
            [
                "Session s: (setupClientSecurityInfo) no client certificate configured",
                "Session s: (connect) setting up PKI provider",
                "Session s: (setupIdentity) setting Anonymous token",
                "Session s: (setupSecurity) no security configured ",
            ]
        );
    }

    /// `SessionOpen62541.cpp:1043-1059`.
    #[cfg(unix)]
    #[tokio::test]
    async fn a_dead_discovery_url_prints_the_failed_get_endpoints() {
        let url = format!("opc.tcp://127.0.0.1:{DEAD_PORT}");
        let lines = setup_lines(&url, &[], "dbgdead").await;
        assert_eq!(
            lines,
            [
                "Session s: (setupClientSecurityInfo) no client certificate configured".to_string(),
                "Session s: (connect) setting up PKI provider".to_string(),
                "Session s: (setupIdentity) setting Anonymous token".to_string(),
                format!("Session s: (setupSecurity) reading endpoints from {url}"),
                format!(
                    "Session s: (setupSecurity) UaDiscovery::getEndpoints from {url} failed \
                     with status BadDisconnect"
                ),
            ]
        );
    }

    /// `SessionOpen62541.cpp:1172-1201`: C only checks that each file reads.
    #[cfg(unix)]
    #[tokio::test]
    async fn a_client_certificate_prints_its_files() {
        let files = [(); 2].map(|_| {
            let file = tempfile::NamedTempFile::new().unwrap();
            std::fs::write(file.path(), "not checked").unwrap();
            file
        });
        let [cert, key] = files
            .each_ref()
            .map(|f| f.path().to_string_lossy().into_owned());
        let lines = setup_lines(
            "opc.tcp://127.0.0.1:4840",
            &[("sec-mode", "None"), ("cert", &cert), ("key", &key)],
            "dbgcert",
        )
        .await;
        assert_eq!(
            lines[..2],
            [
                format!("Session s: (initClientSecurity) loading client certificate {cert}"),
                format!("Session s: (initClientSecurity) loading client private key {key}"),
            ]
        );
    }
}
