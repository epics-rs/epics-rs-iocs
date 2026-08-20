//! Every momentary command record must declare `field(VAL, 1)`.
//!
//! The driver ignores a zero write to a command parameter (`is_command` in
//! `ur_robot::drivers`), so a command record only reaches the robot if
//! processing it sends a one. A client writing `1` does that on its own; the
//! paths that do not are the ones this test exists for, because they process
//! the record without supplying a value:
//!
//! - `Control:jog_watchdog:trigger` fires `Control:JogStop.PROC` when the
//!   client stops kicking the watchdog — the timeout that stops a jog,
//! - the `Control:Stop` dfanout fires `Control:stopL.PROC`,
//!   and `Control:stop_on_safety` fans out to `Control:Stop` / `Dashboard:Stop`,
//! - the `J*CmdU` / `Pose*CmdU` records FLNK `Control:moveJ.PROC` /
//!   `Control:moveL.PROC` after writing the target,
//! - `RobotiqGripper:MinPosition` / `MaxPosition` FLNK to
//!   `SetPositionRange.PROC`,
//! - `RobotiqGripper:Activate` carries `PINI="$(AUTO_ACTIVATE=YES)"`.
//!
//! All of those send `VAL`. Leave `VAL` at its default zero and they become
//! silent no-ops — a jog that never stops on timeout, a gripper that never
//! auto-activates. Adding a command record without `field(VAL, 1)` is the
//! regression this catches.

use std::collections::BTreeMap;
use std::path::PathBuf;

/// The `@asyn(...)` parameter names the driver treats as momentary commands.
/// Deliberately spelled out rather than imported: this is the database's copy
/// of the classification, and the test is what holds the two in agreement.
const COMMANDS: &[&str] = &[
    "ACTIVATE",
    "AUTO_CALIBRATE",
    "BRAKE_RELEASE",
    "CLOSE",
    "CLOSE_POPUP",
    "CLOSE_SAFETY_POPUP",
    "CONNECT",
    "DISCONNECT",
    "JOG_START",
    "JOG_STOP",
    "MOVEJ",
    "MOVEL",
    "OPEN",
    "PAUSE",
    "PLAY",
    "POWER_OFF",
    "POWER_ON",
    "RECONNECT",
    "RESTART_SAFETY",
    "REUPLOAD_CONTROL_SCRIPT",
    "RUN_CUSTOM_SCRIPT_FILE",
    "SET_POSITION_RANGE",
    "SHUTDOWN",
    "STOP",
    "STOPJ",
    "STOPL",
    "STOP_CONTROL_SCRIPT",
    "TRIGGER_PROT_STOP",
    "UNLOCK_PROTECTIVE_STOP",
];

/// One record: its name, the asyn parameter its `OUT` link names, and whether
/// it declares `VAL`.
struct Record {
    name: String,
    out_param: Option<String>,
    val: Option<String>,
}

/// Split a `.db` file into records on the `record(type, "name")` headers.
///
/// A full grammar is not needed: every record in this database opens with that
/// header and the fields that matter here are one per line.
fn parse(text: &str) -> Vec<Record> {
    let mut records: Vec<Record> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("record(") {
            let name = rest
                .split('"')
                .nth(1)
                .unwrap_or_default()
                .trim_start_matches("$(P)")
                .to_string();
            records.push(Record {
                name,
                out_param: None,
                val: None,
            });
            continue;
        }
        let Some(current) = records.last_mut() else {
            continue;
        };
        if let Some(rest) = line.strip_prefix("field(OUT") {
            // "@asyn($(PORT),$(ADDR=0),$(TIMEOUT=1))PARAM" — the parameter is
            // whatever follows the last ')' of the asyn address.
            if let Some(link) = rest.split('"').nth(1)
                && link.starts_with("@asyn")
                && let Some(idx) = link.rfind(')')
            {
                current.out_param = Some(link[idx + 1..].to_string());
            }
        } else if let Some(rest) = line.strip_prefix("field(VAL") {
            let value = rest
                .trim_start_matches(|c: char| c == ',' || c.is_whitespace())
                .trim_end_matches(')')
                .trim()
                .trim_matches('"')
                .to_string();
            current.val = Some(value);
        }
    }
    records
}

fn db_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("db")
}

#[test]
fn every_command_record_declares_val_one() {
    let mut checked = 0usize;
    let mut offenders: Vec<String> = Vec::new();

    for entry in std::fs::read_dir(db_dir()).expect("the db directory is readable") {
        let path = entry.expect("a readable dir entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("db") {
            continue;
        }
        let text = std::fs::read_to_string(&path).expect("a readable .db file");
        let file = path.file_name().unwrap().to_string_lossy().to_string();
        for record in parse(&text) {
            let Some(param) = record.out_param.as_deref() else {
                continue;
            };
            if !COMMANDS.contains(&param) {
                continue;
            }
            checked += 1;
            if record.val.as_deref() != Some("1") {
                offenders.push(format!(
                    "{file}: {} ({param}) has VAL={}",
                    record.name,
                    record.val.as_deref().unwrap_or("<unset>")
                ));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "command records that would send zero when processed:\n  {}",
        offenders.join("\n  ")
    );
    assert_eq!(
        checked, 33,
        "expected 33 command records; a changed count means a command was \
         added or removed and this test's COMMANDS list needs the same edit"
    );
}

/// The parser has to see the same records the IOC does; a silent zero-match
/// would make the test above vacuous.
#[test]
fn the_parser_finds_every_record_in_every_db_file() {
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for entry in std::fs::read_dir(db_dir()).expect("the db directory is readable") {
        let path = entry.expect("a readable dir entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("db") {
            continue;
        }
        let text = std::fs::read_to_string(&path).expect("a readable .db file");
        let file = path.file_name().unwrap().to_string_lossy().to_string();
        let expected = text
            .lines()
            .filter(|l| l.trim_start().starts_with("record("))
            .count();
        let parsed = parse(&text);
        assert_eq!(parsed.len(), expected, "{file}: record count");
        assert!(
            parsed.iter().all(|r| !r.name.is_empty()),
            "{file}: every record parsed a name"
        );
        counts.insert(file, parsed.len());
    }
    assert_eq!(counts.len(), 7, "seven .db files: {counts:?}");
}

/// The A4 split invariant: everything that drives the robot through the
/// dashboard lives in dashboard_ctrl.db and nothing that does lives in
/// dashboard.db, so an IOC loading only dashboard.db exposes no way to
/// write those commands. Connect/Disconnect are the deliberate exceptions:
/// link management a monitoring IOC still needs.
#[test]
fn the_dashboard_control_surface_lives_only_in_the_ctrl_file() {
    let read = |name: &str| std::fs::read_to_string(db_dir().join(name)).expect("a readable db");
    let out_params = |text: &str| -> Vec<String> {
        parse(text)
            .into_iter()
            .filter_map(|r| r.out_param)
            .collect()
    };

    let status_outs = out_params(&read("dashboard.db"));
    assert_eq!(
        status_outs,
        vec!["CONNECT".to_string(), "DISCONNECT".to_string()],
        "dashboard.db may carry no writable surface beyond the link pair"
    );

    let ctrl_outs = out_params(&read("dashboard_ctrl.db"));
    for param in [
        "PLAY",
        "STOP",
        "PAUSE",
        "SHUTDOWN",
        "CLOSE_POPUP",
        "CLOSE_SAFETY_POPUP",
        "POWER_ON",
        "POWER_OFF",
        "BRAKE_RELEASE",
        "UNLOCK_PROTECTIVE_STOP",
        "RESTART_SAFETY",
        "POPUP",
        "LOAD_URP",
    ] {
        assert!(
            ctrl_outs.iter().any(|p| p == param),
            "{param} must be in dashboard_ctrl.db"
        );
    }
    assert_eq!(
        ctrl_outs.len(),
        13,
        "the ctrl file is exactly the 13 commands"
    );
}
