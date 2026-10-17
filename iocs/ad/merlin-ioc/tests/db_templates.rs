//! The Merlin database, as `dbLoadRecords` builds it from merlin.template.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("db")
}

fn load() -> Vec<DbRecordDef> {
    let macros: HashMap<String, String> = [
        ("P", "13ML1:"),
        ("R", "cam1:"),
        ("PORT", "ML"),
        ("ADDR", "0"),
        ("TIMEOUT", "1"),
        ("XSIZE", "512"),
        ("YSIZE", "512"),
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
    parse_db_file(&db_dir().join("merlin.template"), &macros, &config)
        .expect("merlin.template failed to parse")
}

/// C's merlin.template declares FileFormat and FileFormat_RBV with no device
/// support (merlin.template:54-68), so the C IOC serves both.
#[test]
fn file_format_records_exist_as_in_c() {
    let defs = load();
    for (name, rtype) in [
        ("13ML1:cam1:FileFormat", "mbbo"),
        ("13ML1:cam1:FileFormat_RBV", "mbbi"),
    ] {
        let def = defs
            .iter()
            .find(|d| d.name == name)
            .unwrap_or_else(|| panic!("{name} missing"));
        assert_eq!(def.record_type, rtype, "{name}");
    }
}
