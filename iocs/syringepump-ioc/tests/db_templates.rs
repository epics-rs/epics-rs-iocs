//! The ISCO/Vindum register maps must load the way C's `dbLoadTemplate` loads
//! them against `$(MODBUS)/db`.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use epics_rs::base::server::db_loader::{
    DbLoadConfig, DbRecordDef, SubstitutionEvent, parse_db_file, parse_substitutions,
};

const SUBSTITUTIONS: &[(&str, &str)] = &[
    ("ISCOBinaryIn.substitutions", "P=ISCO1:"),
    ("ISCOBinaryOut.substitutions", "P=ISCO1:"),
    ("ISCOAnalogIn.substitutions", "P=ISCO1:"),
    ("ISCOAnalogOut.substitutions", "P=ISCO1:"),
    (
        "VindumReadCoils.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_ReadCoils",
    ),
    (
        "VindumWriteCoils.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_WriteCoils",
    ),
    (
        "VindumReadContacts.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_ReadContacts",
    ),
    (
        "VindumReadInputRegisters.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_ReadInputRegs",
    ),
    (
        "VindumReadHoldingRegisters.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_ReadHoldingRegs",
    ),
    (
        "VindumWriteHoldingRegisters.substitutions",
        "P=VINDUM1:P1:,PORT=SP2_WriteHoldingRegs",
    ),
];

fn db_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("db")
}

/// One substitution row: the template it loads and the macros it sets.
struct Row {
    template: PathBuf,
    macros: HashMap<String, String>,
}

fn rows() -> Vec<Row> {
    let mut rows = Vec::new();
    for (file, global) in SUBSTITUTIONS {
        let text = std::fs::read(db_dir().join(file)).unwrap();
        let subs = parse_substitutions(&text);
        assert!(subs.stopped.is_none(), "{file}: {:?}", subs.stopped);
        for event in subs.events {
            let SubstitutionEvent::Load(load) = event else {
                panic!("{file}: unexpected {event:?}");
            };
            let name = load
                .file
                .strip_prefix("$(SYRINGEPUMP)/db/")
                .unwrap_or_else(|| panic!("{file}: {}", load.file));
            let mut macros: HashMap<String, String> = global
                .split(',')
                .map(|kv| kv.split_once('=').unwrap())
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect();
            for m in load.macros {
                macros.insert(
                    String::from_utf8(m.name).unwrap(),
                    String::from_utf8(m.value).unwrap(),
                );
            }
            rows.push(Row {
                template: db_dir().join(name),
                macros,
            });
        }
    }
    rows
}

fn records() -> Vec<DbRecordDef> {
    let config = DbLoadConfig::default();
    rows()
        .into_iter()
        .flat_map(|row| {
            parse_db_file(&row.template, &row.macros, &config)
                .unwrap_or_else(|e| panic!("{}: {e}", row.template.display()))
        })
        .collect()
}

fn field<'a>(def: &'a DbRecordDef, name: &str) -> Option<std::borrow::Cow<'a, str>> {
    def.fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.value.as_str_lossy())
}

fn record<'a>(defs: &'a [DbRecordDef], name: &str) -> &'a DbRecordDef {
    defs.iter()
        .find(|d| d.name == name)
        .unwrap_or_else(|| panic!("{name} not loaded"))
}

/// C's modbus templates carry DESC on every register record and PINI on
/// aoFloat64; the ISCO rows set the one and the Vindum outputs the other.
#[test]
fn row_macros_set_the_fields_c_sets() {
    let defs = records();
    let ai = record(&defs, "ISCO1:A:PressureSP_RBV");
    assert_eq!(field(ai, "DESC").as_deref(), Some("Register"));
    let ao = record(&defs, "ISCO1:A:PressureSP");
    assert_eq!(field(ao, "DESC").as_deref(), Some("Register"));
    let ao = record(&defs, "VINDUM1:P1:A:SetFlowRate");
    assert_eq!(field(ao, "PINI").as_deref(), Some("YES"));
}

/// modbus `mbbi.template`/`mbbo.template` bind through asynInt32 at the
/// port's default data type.
#[test]
fn vindum_enumerations_bind_as_c_mbbi_template() {
    let defs = records();
    let mut seen = 0;
    for def in defs
        .iter()
        .filter(|d| d.record_type == "mbbi" || d.record_type == "mbbo")
    {
        seen += 1;
        assert_eq!(
            field(def, "DTYP").as_deref(),
            Some("asynInt32"),
            "{}",
            def.name
        );
        let link = field(def, "INP").or_else(|| field(def, "OUT")).unwrap();
        assert!(link.starts_with("@asyn(SP2_"), "{}: {link}", def.name);
        assert!(link.ends_with(')'), "{}: {link}", def.name);
    }
    assert_eq!(seen, 8);
}
