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
    use async_opcua::types::{EndpointDescription, MessageSecurityMode};
    use opcua::client::{AsyncOpcuaConnector, Identity, UaConnector};
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
            err,
            "security discovery and setup failed with status cantConnect"
        );
    }
}
