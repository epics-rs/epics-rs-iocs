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
