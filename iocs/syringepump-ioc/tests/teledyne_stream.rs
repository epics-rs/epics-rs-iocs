//! Teledyne D/H run on StreamDevice, as C does: SPApp/Db's templates and
//! teled_d.proto/teled_h.proto, loaded with the st.cmd's macros.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("db")
}

fn load(template: &str, macros: &str) -> Vec<DbRecordDef> {
    let macros: HashMap<String, String> = macros
        .split(',')
        .map(|kv| kv.split_once('=').unwrap())
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    parse_db_file(&db_dir().join(template), &macros, &DbLoadConfig::default()).unwrap()
}

fn records() -> Vec<DbRecordDef> {
    let mut defs = load(
        "teledynePumpD.template",
        "P=SP:,PUMP=D1:,s=SP,ta=Teledyne,ss=D1,PORT=TelD1_tty,ADDR=0",
    );
    defs.extend(load(
        "teledynePumpH.template",
        "s=SP,ta=Teledyne,ss=H1,PORT=TelH1_tty",
    ));
    defs
}

fn field(defs: &[DbRecordDef], record: &str, name: &str) -> Option<String> {
    let def = defs
        .iter()
        .find(|d| d.name == record)
        .unwrap_or_else(|| panic!("{record} not loaded"));
    def.fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.value.as_str_lossy().into_owned())
}

#[test]
fn device_records_use_the_teled_protocols() {
    let defs = records();
    for (record, link, value) in [
        (
            "SP:Teledyne:D1:Vol",
            "INP",
            "@teled_d.proto getVol(6,A) TelD1_tty",
        ),
        (
            "SP:Teledyne:D1:PressSend",
            "OUT",
            "@teled_d.proto setPress(6,A) TelD1_tty",
        ),
        (
            "SP:Teledyne:H1:Refill",
            "OUT",
            "@teled_h.proto setRefill(6,A) TelH1_tty",
        ),
        (
            "SP:Teledyne:H1:SetDigitalCalc1",
            "OUT",
            "@teled_h.proto setDigital(6,A) TelH1_tty",
        ),
        (
            "SP:Teledyne:H1:CmdReply",
            "INP",
            "@teled_h.proto sendCmd(6) TelH1_tty",
        ),
    ] {
        assert_eq!(
            field(&defs, record, "DTYP").as_deref(),
            Some("stream"),
            "{record}"
        );
        assert_eq!(
            field(&defs, record, link).as_deref(),
            Some(value),
            "{record}"
        );
    }
}

/// teledynePumpD.template's `setRun($(u=6),$(p=$(PUMP)) $(PORT)` never
/// closes setRun's parenthesis, so C's Run record fails stream init.
#[test]
fn d_run_keeps_the_unclosed_set_run_link() {
    let defs = records();
    assert_eq!(
        field(&defs, "SP:D1:Run", "OUT").as_deref(),
        Some("@teled_d.proto setRun(6,D1: TelD1_tty")
    );
}

/// Records whose links name an undefined PV or an unregistered aSub load
/// as in C; only the approved PressSeq/MaxFlowSeq repoint differs.
#[test]
fn upstream_records_are_kept() {
    let defs = records();
    for ss in ["D1", "H1"] {
        let alarm = format!("SP:Teledyne:{ss}:AlarmI");
        assert_eq!(
            field(&defs, &alarm, "CALC").as_deref(),
            Some("(A==3)||B||C||D")
        );
        let command = format!("SP:Teledyne:{ss}:Command");
        let cmd_proc = format!("SP:Teledyne:{ss}:CmdProc");
        assert_eq!(
            field(&defs, &command, "FLNK").as_deref(),
            Some(cmd_proc.as_str())
        );
        assert_eq!(
            field(&defs, &cmd_proc, "SNAM").as_deref(),
            Some("formatCmd")
        );
    }
    assert!(field(&defs, "SP:D1:FlowRateTweak", "VAL").is_some());
    assert_eq!(
        field(&defs, "SP:Teledyne:D1:PressSeq", "LNK2").as_deref(),
        Some("SP:D1:Run.PROC CA")
    );
}

#[test]
fn protocol_files_ship_with_the_db() {
    for proto in ["teled_d.proto", "teled_h.proto"] {
        assert!(db_dir().join(proto).is_file(), "{proto}");
    }
}
