//! The Eiger databases must parse and load through the framework's db loader.
//!
//! `dbLoadRecords` merges a re-declared record into the existing instance only
//! when the record *type* matches (C `dbLexRoutines.c:1170-1188`), which is what
//! the ADCore convention of re-opening `TriggerMode` / `DataType` / … to
//! override menu choices relies on. These tests pin both properties for the
//! four Eiger templates: they parse with the same macro set st.cmd passes, and
//! every re-declared record keeps its original record type.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../drivers/ad/eiger/db")
}

fn ad_core_db_dir() -> PathBuf {
    Path::new(epics_rs::ad_core::AD_CORE_DIR).join("db")
}

fn load(template: &str) -> Vec<DbRecordDef> {
    let macros: HashMap<String, String> = [
        ("P", "13EIG2:"),
        ("R", "cam1:"),
        ("PORT", "EIG"),
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
    parse_db_file(&db_dir().join(template), &macros, &config)
        .unwrap_or_else(|e| panic!("{template} failed to parse: {e}"))
}

/// Re-declaring a record with a *different* type is fatal in `dbLoadRecords`,
/// so an override that changed the type would take the IOC down at boot.
fn assert_redeclarations_keep_their_type(template: &str) {
    let defs = load(template);
    let mut types: HashMap<&str, &str> = HashMap::new();
    for def in &defs {
        match types.get(def.name.as_str()) {
            Some(first) => assert_eq!(
                *first, def.record_type,
                "{template}: {} is re-declared as {} but was first declared as {first}",
                def.name, def.record_type
            ),
            None => {
                types.insert(&def.name, &def.record_type);
            }
        }
    }
    assert!(!defs.is_empty(), "{template}: no records");
}

#[test]
fn eiger_base_loads() {
    let defs = load("eigerBase.template");
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    // One record from the Eiger set, one from the included ADBase.template.
    assert!(names.contains(&"13EIG2:cam1:ThresholdEnergy"));
    assert!(names.contains(&"13EIG2:cam1:Acquire"));
    assert_redeclarations_keep_their_type("eigerBase.template");
}

#[test]
fn eiger1_loads() {
    let defs = load("eiger1.template");
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    assert!(names.contains(&"13EIG2:cam1:Link0_RBV"));
    assert!(names.contains(&"13EIG2:cam1:FWClear"));
    assert_redeclarations_keep_their_type("eiger1.template");
}

#[test]
fn eiger2_loads() {
    let defs = load("eiger2.template");
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    assert!(names.contains(&"13EIG2:cam1:Threshold2Energy"));
    assert!(names.contains(&"13EIG2:cam1:ExtGateMode"));
    assert_redeclarations_keep_their_type("eiger2.template");
}

#[test]
fn pilatus4_loads() {
    let defs = load("pilatus4.template");
    let names: Vec<&str> = defs.iter().map(|d| d.name.as_str()).collect();
    assert!(names.contains(&"13EIG2:cam1:Threshold4Energy"));
    assert_redeclarations_keep_their_type("pilatus4.template");
}

/// Every `@asyn(...)DRV_INFO` in the templates must name a parameter the driver
/// can serve: either one it creates up front (`params::create`) or one it
/// creates on demand from the `EIG_XYZ_name` encoding (`drv_user_create`).
#[test]
fn every_drv_info_is_one_the_driver_can_serve() {
    let created: Vec<&str> = vec![
        // Eiger-specific, params::create.
        "STATE",
        "DATA_SOURCE",
        "AUTO_REMOVE",
        "TRIGGER",
        "MANUAL_TRIGGER",
        "ARMED",
        "SEQ_ID",
        "PENDING_FILES",
        "SAVE_FILES",
        "FILE_OWNER",
        "FILE_OWNER_GROUP",
        "FILE_PERMISSIONS",
        "MONITOR_TIMEOUT",
        "RESTART",
        "INITIALIZE",
        "STREAM_DECOMPRESS",
        "WAVELENGTH_EPSILON",
        "ENERGY_EPSILON",
        "SIGNED_DATA",
        "STREAM_AS_TIMESTAMP_SOURCE",
        "DESCRIPTION",
        "WAVELENGTH",
        "PHOTON_ENERGY",
        "THRESHOLD",
        "NUM_TRIGGERS",
        "COMPRESSION_ALGO",
        "ROI_MODE",
        "AUTO_SUMMATION",
        "ERROR",
        "TH_TEMP_0",
        "TH_HUMID_0",
        "FW_ENABLE",
        "COMPRESSION",
        "NAME_PATTERN",
        "NIMAGES_PER_FILE",
        "FW_IMG_NUM_START",
        "FW_STATE",
        "FW_FREE",
        "MONITOR_ENABLE",
        "MONITOR_BUF_SIZE",
        "MONITOR_STATE",
        "STREAM_ENABLE",
        "STREAM_STATE",
        "STREAM_DROPPED",
        "STREAM_VERSION",
        "LINK_0",
        "LINK_1",
        "LINK_2",
        "LINK_3",
        "DCU_BUF_FREE",
        "CLEAR",
        "THRESHOLD1_ENABLE",
        "TRIGGER_START_DELAY",
        "THRESHOLD2",
        "THRESHOLD2_ENABLE",
        "THRESHOLD_DIFF_ENABLE",
        "HV_STATE",
        "HV_RESET_TIME",
        "HV_RESET",
        "FWHDF5_FORMAT",
        "EXT_GATE_MODE",
        "THRESHOLD3",
        "THRESHOLD3_ENABLE",
        "THRESHOLD4",
        "THRESHOLD4_ENABLE",
    ];

    let mut unserved = Vec::new();
    for template in [
        "eigerBase.template",
        "eiger1.template",
        "eiger2.template",
        "pilatus4.template",
    ] {
        for def in load(template) {
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
                if drv_info.is_empty()
                    || drv_info.starts_with("EIG_")
                    || created.contains(&drv_info)
                {
                    continue;
                }
                // Anything left must be an ADDriver / NDArrayDriver base
                // parameter, which the framework creates. Those are exactly the
                // ones the ADCore templates use, so only flag a name that is
                // neither ours nor plausibly a base parameter: a name we do not
                // create and that no ADCore template declares.
                if !ad_core_declares(drv_info) {
                    unserved.push(format!("{template}: {} -> {drv_info}", def.name));
                }
            }
        }
    }
    assert!(unserved.is_empty(), "unserved drvInfo: {unserved:#?}");
}

/// Does any ADCore template use this drvInfo? (A cheap proxy for "the framework
/// creates this base-class parameter".)
fn ad_core_declares(drv_info: &str) -> bool {
    let needle = format!("){drv_info}\"");
    std::fs::read_dir(ad_core_db_dir())
        .expect("ad-core db/ directory")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().is_some_and(|x| x == "template"))
        .any(|e| {
            std::fs::read_to_string(e.path())
                .map(|s| s.contains(&needle))
                .unwrap_or(false)
        })
}
