//! `Session::securityPolicyString`, as the "connected as" line prints it.

use opcua::client::security_policy_string;

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
    use opcua::client::{AsyncOpcuaConnector, ConnectError, Identity, UaConnector};
    use opcua::session::SessionConfig;

    const NONE: &str = "http://opcfoundation.org/UA/SecurityPolicy#None";
    const SHA256: &str = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";

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
        let e = config.select_endpoint(&offered()).unwrap();
        assert_eq!(e.security_level, 7);
        assert_eq!(e.endpoint_url.as_ref(), "opc.tcp://10.0.0.1:4840");
    }

    #[test]
    fn a_requested_mode_and_policy_narrow_the_pick() {
        let mut config = SessionConfig::new("s", "opc.tcp://srv:4840");
        config.set_option("sec-mode", "Sign").unwrap();
        assert_eq!(
            config.select_endpoint(&offered()).unwrap().security_level,
            3
        );
        // A requested None policy matches any policy, as C's find("#None").
        config.set_option("sec-policy", "None").unwrap();
        assert_eq!(
            config.select_endpoint(&offered()).unwrap().security_level,
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
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let pki = std::env::temp_dir().join(format!("opcua-cantconnect-{}", std::process::id()));
        let mut config = SessionConfig::new("s", format!("opc.tcp://127.0.0.1:{port}"));
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        let err = AsyncOpcuaConnector::new()
            .connect(&config, &Identity::Anonymous)
            .await
            .err()
            .unwrap();
        let _ = std::fs::remove_dir_all(&pki);
        assert_eq!(
            err.message,
            "security discovery and setup failed with status cantConnect"
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
            .connect(&config, &Identity::Anonymous)
            .await
            .err()
            .unwrap();
        errlog_flush();
        errlog_remove_listener(id);
        let _ = std::fs::remove_dir_all(&pki);
        assert_eq!(
            err,
            ConnectError::setup("security discovery and setup failed with status fatal")
        );
        assert_eq!(
            *lines.lock(),
            ["OPC UA session s: fatal - only URLs of type 'opc.tcp' supported\n"]
        );
    }

    /// C `SessionOpen62541::connect` prints `UA_Client_connect`'s status on
    /// every failed connect (`SessionOpen62541.cpp:586-589`).
    #[tokio::test]
    async fn a_dead_session_url_is_a_connect_failure_with_its_status() {
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let pki = std::env::temp_dir().join(format!("opcua-deadsession-{}", std::process::id()));
        let mut config = SessionConfig::new("s", format!("opc.tcp://127.0.0.1:{port}"));
        config.pki_dir = Some(pki.to_string_lossy().into_owned());
        config.set_option("sec-mode", "None").unwrap();
        let connector = AsyncOpcuaConnector::new();
        let connect = connector.connect(&config, &Identity::Anonymous);
        let err = tokio::time::timeout(std::time::Duration::from_secs(20), connect)
            .await
            .expect("a failed connect returns")
            .err()
            .unwrap();
        let _ = std::fs::remove_dir_all(&pki);
        assert!(
            err.message
                .starts_with("connect service failed with status Bad"),
            "{err}"
        );
        assert!(err.status.is_some_and(|s| s.is_bad()), "{err:?}");
    }
}
