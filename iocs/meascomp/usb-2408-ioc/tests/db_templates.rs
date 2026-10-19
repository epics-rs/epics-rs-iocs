//! The usb-2408 records bind the parameters C's measComp templates bind.

use std::collections::HashMap;
use std::path::Path;

use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};

fn load(template: &str, macros: &[(&str, &str)]) -> Vec<DbRecordDef> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../db")
        .join(template);
    let macros: HashMap<String, String> = macros
        .iter()
        .map(|&(k, v)| (k.to_owned(), v.to_owned()))
        .collect();
    parse_db_file(&path, &macros, &DbLoadConfig::default())
        .unwrap_or_else(|e| panic!("{template}: {e}"))
}

fn field(def: &DbRecordDef, name: &str) -> Option<String> {
    def.fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.value.as_str_lossy().into_owned())
}

/// measCompAnalogIn.template:1-12 with USB2408.substitutions' Ai1 row. C
/// reads a 2408 input as raw counts only and never sets VOLTAGE_IN_VALUE on
/// this model.
#[test]
fn an_analog_input_averages_its_raw_counts() {
    let defs = load(
        "measCompAnalogIn.template",
        &[
            ("P", "T:"),
            ("R", "Ai1"),
            ("PORT", "MF1"),
            ("ADDR", "0"),
            ("EGUL", "-10."),
            ("LOPR", "-10."),
            ("EGUF", "10."),
            ("HOPR", "10."),
            ("RANGE", "3"),
            ("SCAN", "1 second"),
            ("PREC", "4"),
        ],
    );
    let ai = defs.iter().find(|d| d.name == "T:Ai1").unwrap();
    assert_eq!(field(ai, "DTYP").as_deref(), Some("asynInt32Average"));
    assert_eq!(
        field(ai, "INP").as_deref(),
        Some("@asyn(MF1,0)ANALOG_IN_VALUE")
    );
    assert_eq!(field(ai, "LINR").as_deref(), Some("LINEAR"));
    assert_eq!(field(ai, "SCAN").as_deref(), Some("1 second"));
}
