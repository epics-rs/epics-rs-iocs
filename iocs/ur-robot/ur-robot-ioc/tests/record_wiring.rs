//! Record-name ↔ asyn-parameter wiring, machine-checked.
//!
//! Upstream urRobot's `rtde_receive.db` wired `TargetJointMoments` to
//! `TARGET_JOINT_CURRENTS` (INP and DESC both copied from the record
//! above it — defect register #161): the moments PV served the currents
//! data, and nothing could notice, because the db is the only place the
//! pairing exists. Two tests close the two halves of that failure:
//!
//! - existence, all dbs: every `@asyn(...)DRVINFO` names a parameter the
//!   port's driver actually creates, so a typo'd or stale drvInfo fails
//!   here instead of soft-failing at iocInit;
//! - identity, receive waveforms: the parameter is derived from the
//!   record name, so a copy-pasted but *existing* drvInfo (the #161
//!   shape) fails too. Only the `Receive:Target*`/`Actual*` family has a
//!   naming convention strict enough to support this.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::path::PathBuf;

struct AsynLink {
    record_type: String,
    record: String,
    drv_info: String,
}

/// Every `field(INP|OUT, "@asyn(...)DRVINFO")` in a db file, with the
/// record it belongs to. calc `INPA`-style PV links don't start with
/// `@asyn` and fall through.
fn asyn_links(db_file: &str) -> Vec<AsynLink> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("db")
        .join(db_file);
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("a readable {db_file}: {e}"));

    let mut links = Vec::new();
    let mut current: Option<(String, String)> = None;
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("record(") {
            let record_type = rest
                .split(',')
                .next()
                .unwrap_or_default()
                .trim()
                .to_string();
            let record = rest.split('"').nth(1).unwrap_or_default().to_string();
            current = Some((record_type, record));
        } else if let Some((record_type, record)) = &current
            && (line.starts_with("field(INP") || line.starts_with("field(OUT"))
            && let Some(link) = line.split('"').nth(1)
            && link.starts_with("@asyn")
            && let Some(idx) = link.rfind(')')
        {
            links.push(AsynLink {
                record_type: record_type.clone(),
                record: record.clone(),
                drv_info: link[idx + 1..].to_string(),
            });
        }
    }
    links
}

/// db file → the driver kind st.cmd wires its `PORT` macro to.
const DB_TO_DRIVER: &[(&str, &str)] = &[
    ("dashboard.db", "dashboard"),
    ("dashboard_ctrl.db", "dashboard"),
    ("rtde_receive.db", "receive"),
    ("rtde_io.db", "io"),
    ("rtde_control.db", "control"),
    ("rtde_control_jog.db", "control"),
    ("robotiq_gripper.db", "gripper"),
];

#[test]
fn every_asyn_link_names_a_parameter_its_driver_creates() {
    // A db file this map doesn't know is a db file this test doesn't
    // check — refuse the silent gap.
    let mapped: BTreeSet<&str> = DB_TO_DRIVER.iter().map(|&(db, _)| db).collect();
    let on_disk: BTreeSet<String> =
        std::fs::read_dir(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("db"))
            .expect("a readable db/ directory")
            .filter_map(|e| {
                let name = e
                    .expect("a readable db/ entry")
                    .file_name()
                    .into_string()
                    .ok()?;
                name.ends_with(".db").then_some(name)
            })
            .collect();
    assert_eq!(
        on_disk.iter().map(String::as_str).collect::<BTreeSet<_>>(),
        mapped,
        "db/ and DB_TO_DRIVER disagree; map the new or renamed file to its driver"
    );

    let tables: HashMap<&str, HashSet<String>> = ur_robot::drivers::created_param_names()
        .into_iter()
        .map(|(kind, names)| (kind, names.into_iter().collect()))
        .collect();

    let mut offenders = Vec::new();
    for &(db, kind) in DB_TO_DRIVER {
        let params = &tables[kind];
        let links = asyn_links(db);
        assert!(
            !links.is_empty(),
            "{db}: parsed no @asyn links; the parser or the db went stale"
        );
        for link in links {
            if !params.contains(&link.drv_info) {
                offenders.push(format!(
                    "{db}: {} wires {}, which the {kind} driver never creates",
                    link.record, link.drv_info
                ));
            }
        }
    }
    assert!(
        offenders.is_empty(),
        "records wired to nonexistent parameters:\n  {}",
        offenders.join("\n  ")
    );
}

/// `TargetJointMoments` → `TARGET_JOINT_MOMENTS`, `ActualTCPPose` →
/// `ACTUAL_TCP_POSE`. An uppercase run stays one word until its last
/// letter starts a lowercase tail.
fn camel_to_screaming_snake(name: &str) -> String {
    let chars: Vec<char> = name.chars().collect();
    let mut out = String::new();
    for (i, &c) in chars.iter().enumerate() {
        let boundary = i > 0
            && c.is_ascii_uppercase()
            && (chars[i - 1].is_ascii_lowercase()
                || (i + 1 < chars.len() && chars[i + 1].is_ascii_lowercase()));
        if boundary {
            out.push('_');
        }
        out.push(c.to_ascii_uppercase());
    }
    out
}

/// The pairings where the parameter deliberately differs from the record
/// name, each with the reason the deviation exists.
const EXCEPTIONS: &[(&str, &str)] = &[
    // The `_ARR` suffix distinguishes the whole-vector parameter from the
    // per-address scalar (`ACTUAL_JOINT_POS` at ADDR 0..5) on the same data.
    ("ActualJointPositions", "ACTUAL_JOINT_POS_ARR"),
    ("ActualTCPPose", "ACTUAL_TCP_POSE_ARR"),
    // Upstream abbreviates the parameter; kept for driver parity.
    ("ActualToolAccelerometer", "ACTUAL_TOOL_ACCEL"),
];

#[test]
fn every_target_and_actual_waveform_names_its_own_parameter() {
    let mut checked = 0usize;
    let mut offenders = Vec::new();
    for link in asyn_links("rtde_receive.db") {
        if !link.record_type.starts_with("waveform") {
            continue;
        }
        let name = link.record.trim_start_matches("$(P)Receive:");
        if !name.starts_with("Target") && !name.starts_with("Actual") {
            continue;
        }
        checked += 1;
        let expected = EXCEPTIONS
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, p)| (*p).to_string())
            .unwrap_or_else(|| camel_to_screaming_snake(name));
        if link.drv_info != expected {
            offenders.push(format!(
                "{name}: INP names {}, expected {expected}",
                link.drv_info
            ));
        }
    }
    assert!(
        offenders.is_empty(),
        "miswired waveform records:\n  {}",
        offenders.join("\n  ")
    );
    assert_eq!(
        checked, 15,
        "expected 15 Target*/Actual* waveforms; a changed count means a \
         record was added or removed and this test should see it"
    );
}
