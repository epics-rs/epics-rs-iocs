//! The Mythen database must parse and load through the framework's db loader,
//! and every `drvInfo` in it must name a parameter the driver creates.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

/// Every parameter `MythenParams::create` makes (drivers/ad/mythen/src/driver.rs).
const CREATED: &[&str] = &[
    "SD_SETTING",
    "SD_DELAY_TIME",
    "SD_THRESHOLD",
    "SD_ENERGY",
    "SD_USE_FLATFIELD",
    "SD_USE_COUNTRATE",
    "SD_USE_BADCHANNEL_INTRPL",
    "SD_BIT_DEPTH",
    "SD_USE_GATES",
    "SD_NUM_GATES",
    "SD_NUM_FRAMES",
    "SD_TRIGGER",
    "SD_RESET",
    "SD_TAU",
    "SD_NMODULES",
    "SD_FIRMWARE_VERSION",
    "SD_READ_MODE",
];

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../drivers/ad/mythen/db")
}

fn ad_core_db_dir() -> PathBuf {
    Path::new(epics_rs::ad_core::AD_CORE_DIR).join("db")
}

fn load() -> Vec<DbRecordDef> {
    let macros: HashMap<String, String> = [
        ("P", "dp_mythen1K:"),
        ("R", "cam1:"),
        ("PORT", "SD1"),
        ("ADDR", "0"),
        ("TIMEOUT", "1"),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();

    let config = DbLoadConfig {
        include_paths: vec![db_dir(), ad_core_db_dir()],
        ..Default::default()
    };
    parse_db_file(&db_dir().join("mythen.template"), &macros, &config)
        .expect("mythen.template failed to parse")
}

#[test]
fn the_template_loads() {
    let defs = load();
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    // One record from the Mythen set, one from the included ADBase.template.
    assert!(names.contains(&"dp_mythen1K:cam1:BitDepth"));
    assert!(names.contains(&"dp_mythen1K:cam1:Acquire"));
}

/// `dbLoadRecords` merges a re-declared record into the existing instance only
/// when the record *type* matches, so the `ImageMode` mbbo the template re-opens
/// to drop the "Continuous" choice must stay an mbbo.
#[test]
fn redeclared_records_keep_their_type() {
    let mut types: HashMap<String, String> = HashMap::new();
    for def in load() {
        match types.get(&def.name) {
            Some(first) => assert_eq!(
                *first, def.record_type,
                "{} is re-declared as {} but was first declared as {first}",
                def.name, def.record_type
            ),
            None => {
                types.insert(def.name.clone(), def.record_type.clone());
            }
        }
    }
    assert!(types.contains_key("dp_mythen1K:cam1:ImageMode"));
}

#[test]
fn every_sd_drv_info_is_one_the_driver_creates() {
    let mut unserved = Vec::new();
    for def in load() {
        for (field, value) in &def.fields {
            if field != "INP" && field != "OUT" {
                continue;
            }
            let Some(rest) = value.strip_prefix("@asyn(") else {
                continue;
            };
            let Some((_, drv_info)) = rest.split_once(')') else {
                continue;
            };
            let drv_info = drv_info.trim();
            // Base-class parameters (ADBase.template) are the framework's.
            if !drv_info.starts_with("SD_") {
                continue;
            }
            if !CREATED.contains(&drv_info) {
                unserved.push(format!("{} -> {drv_info}", def.name));
            }
        }
    }
    assert!(unserved.is_empty(), "unserved drvInfo: {unserved:#?}");
}
