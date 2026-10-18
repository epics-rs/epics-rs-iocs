//! The Pilatus database, as `dbLoadRecords` builds it from pilatus.template.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("db")
}

fn load() -> Vec<DbRecordDef> {
    let macros: HashMap<String, String> = [
        ("P", "13PIL1:"),
        ("R", "cam1:"),
        ("PORT", "PIL"),
        ("ADDR", "0"),
        ("TIMEOUT", "1"),
        ("CAMSERVER_PORT", "camserver"),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();
    let config = DbLoadConfig {
        include_paths: vec![
            db_dir(),
            Path::new(epics_rs::ad_core::AD_CORE_DIR).join("db"),
        ],
        ..Default::default()
    };
    parse_db_file(&db_dir().join("pilatus.template"), &macros, &config)
        .expect("pilatus.template failed to parse")
}

fn disa(defs: &[DbRecordDef], name: &str) -> Option<String> {
    defs.iter()
        .filter(|d| d.name == name)
        .flat_map(|d| d.fields.iter())
        .filter(|f| f.name == "DISA")
        .map(|f| f.value.to_string())
        .next_back()
}

/// pilatus.template:544-547 disables DataType only; DataType_RBV keeps
/// ADBase's record as it is.
#[test]
fn only_data_type_is_disabled_as_in_c() {
    let defs = load();
    assert_eq!(disa(&defs, "13PIL1:cam1:DataType").as_deref(), Some("1"));
    assert_eq!(disa(&defs, "13PIL1:cam1:DataType_RBV"), None);
}
