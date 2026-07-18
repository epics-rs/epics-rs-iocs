//! Every TimePix3 template must parse through the framework's db loader, and
//! every `TPX3_*` drvInfo in them must name a parameter the driver creates.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

/// The templates st.cmd loads, with the macros it loads them under.
const TEMPLATES: &[(&str, &[(&str, &str)])] = &[
    ("TimePix3Base.template", &[]),
    ("ADTimePix3.template", &[]),
    ("File.template", &[]),
    ("Server.template", &[("MAX_PIXELS", "262144")]),
    ("Measurement.template", &[]),
    ("Dashboard.template", &[("S", "Stats5:")]),
    (
        "MaskBPC.template",
        &[("TYPE", "Int32"), ("FTVL", "LONG"), ("NELEMENTS", "262144")],
    ),
    ("Chips.template", &[("C", "CHIP0")]),
    ("OperatingVoltage.template", &[("C", "Pwr0")]),
];

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../drivers/ad/timepix3/db")
}

fn ad_core_db_dir() -> PathBuf {
    Path::new(epics_rs::ad_core::AD_CORE_DIR).join("db")
}

fn load(template: &str, extra: &[(&str, &str)]) -> Vec<DbRecordDef> {
    let mut macros: HashMap<String, String> = [
        ("P", "TPX3-TEST:"),
        ("R", "cam1:"),
        ("PORT", "TPX3"),
        ("ADDR", "0"),
        ("TIMEOUT", "1"),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();
    for (k, v) in extra {
        macros.insert((*k).to_string(), (*v).to_string());
    }

    let config = DbLoadConfig {
        include_paths: vec![db_dir(), ad_core_db_dir()],
        ..Default::default()
    };
    parse_db_file(&db_dir().join(template), &macros, &config)
        .unwrap_or_else(|e| panic!("{template} failed to parse: {e}"))
}

fn load_all() -> Vec<DbRecordDef> {
    TEMPLATES
        .iter()
        .flat_map(|(t, m)| load(t, m))
        .collect::<Vec<_>>()
}

#[test]
fn every_template_loads() {
    let defs = load_all();
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    // One record from each of the four shapes: the base class the driver's own
    // template pulls in, a Serval scalar, a per-chip DAC, and a rail.
    assert!(names.contains(&"TPX3-TEST:cam1:Acquire"));
    assert!(names.contains(&"TPX3-TEST:cam1:WriteData"));
    assert!(names.contains(&"TPX3-TEST:cam1:CHIP0_Ikrum_RBV"));
    assert!(names.contains(&"TPX3-TEST:cam1:Pwr0_VDD_RBV"));
}

/// `dbLoadRecords` merges a re-declared record into the existing instance only
/// when the record *type* matches, so a record that a later template re-opens
/// (TimePix3Base.template re-declares ADBase records) must keep its type.
#[test]
fn redeclared_records_keep_their_type() {
    let mut types: HashMap<String, String> = HashMap::new();
    for def in load_all() {
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
    assert!(types.contains_key("TPX3-TEST:cam1:TriggerMode"));
}

#[test]
fn every_tpx3_drv_info_is_one_the_driver_creates() {
    let mut unserved = Vec::new();
    for def in load_all() {
        for (field, value) in &def.fields {
            if field != "INP" && field != "OUT" {
                continue;
            }
            let value = value.as_str_lossy();
            let Some(rest) = value.strip_prefix("@asyn(") else {
                continue;
            };
            let Some((_, drv_info)) = rest.split_once(')') else {
                continue;
            };
            let drv_info = drv_info.trim();
            // Base-class parameters (ADBase.template) are the framework's.
            if !drv_info.starts_with("TPX3_") {
                continue;
            }
            if !timepix3::params::DRV_INFO.contains(&drv_info) {
                unserved.push(format!("{} -> {drv_info}", def.name));
            }
        }
    }
    assert!(unserved.is_empty(), "unserved drvInfo: {unserved:#?}");
}
