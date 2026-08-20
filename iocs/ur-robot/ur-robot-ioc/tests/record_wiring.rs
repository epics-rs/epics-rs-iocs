//! Record-name ↔ asyn-parameter wiring, machine-checked.
//!
//! Upstream urRobot's `rtde_receive.db` wired `TargetJointMoments` to
//! `TARGET_JOINT_CURRENTS` (INP and DESC both copied from the record
//! above it — defect register #161): the moments PV served the currents
//! data, and nothing could notice, because the db is the only place the
//! pairing exists. This test derives the expected parameter from the
//! record name for every `Receive:Target*`/`Receive:Actual*` waveform, so
//! the same copy-paste miswiring fails a test instead of shipping.

use std::path::PathBuf;

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

/// `(record name without "$(P)Receive:", asyn parameter)` for every
/// waveform record in rtde_receive.db.
fn waveform_wirings() -> Vec<(String, String)> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("db/rtde_receive.db");
    let text = std::fs::read_to_string(path).expect("a readable rtde_receive.db");

    let mut wirings = Vec::new();
    let mut current: Option<String> = None;
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("record(") {
            let is_waveform = rest.starts_with("waveform");
            let name = rest
                .split('"')
                .nth(1)
                .unwrap_or_default()
                .trim_start_matches("$(P)Receive:")
                .to_string();
            current = is_waveform.then_some(name);
        } else if let Some(name) = &current
            && let Some(rest) = line.strip_prefix("field(INP")
            && let Some(link) = rest.split('"').nth(1)
            && link.starts_with("@asyn")
            && let Some(idx) = link.rfind(')')
        {
            wirings.push((name.clone(), link[idx + 1..].to_string()));
        }
    }
    wirings
}

#[test]
fn every_target_and_actual_waveform_names_its_own_parameter() {
    let mut checked = 0usize;
    let mut offenders = Vec::new();
    for (name, param) in waveform_wirings() {
        if !name.starts_with("Target") && !name.starts_with("Actual") {
            continue;
        }
        checked += 1;
        let expected = EXCEPTIONS
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, p)| (*p).to_string())
            .unwrap_or_else(|| camel_to_screaming_snake(&name));
        if param != expected {
            offenders.push(format!("{name}: INP names {param}, expected {expected}"));
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
