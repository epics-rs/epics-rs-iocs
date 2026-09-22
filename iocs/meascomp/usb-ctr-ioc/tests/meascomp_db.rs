//! Contract checks on the databases of the meascomp IOC family (usb-ctr-ioc
//! and usb-2408-ioc): each st.cmd is expanded the way dbLoadRecords expands
//! it, and the loaded records are checked against the rules the drivers and
//! autosave depend on.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

const IOCS: [&str; 2] = ["usb-ctr-ioc", "usb-2408-ioc"];

struct Record {
    rtype: String,
    fields: HashMap<String, String>,
}

impl Record {
    fn field(&self, name: &str) -> Option<&str> {
        self.fields.get(name).map(String::as_str)
    }
}

struct Ioc {
    records: HashMap<String, Record>,
    /// Record names (and `record.FIELD` entries) auto_settings.req restores.
    restored: Vec<String>,
}

fn family_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// `$(NAME)` / `$(NAME=default)` substitution, as msi/dbLoadRecords do it.
fn expand(text: &str, macros: &HashMap<String, String>) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(start) = rest.find("$(") {
        out.push_str(&rest[..start]);
        let tail = &rest[start + 2..];
        let end = tail.find(')').expect("unterminated macro");
        let spec = &tail[..end];
        let (name, default) = match spec.split_once('=') {
            Some((n, d)) => (n, Some(d)),
            None => (spec, None),
        };
        match macros.get(name) {
            Some(v) => out.push_str(v),
            None => out.push_str(default.unwrap_or_else(|| panic!("undefined macro {name}"))),
        }
        rest = &tail[end + 1..];
    }
    out.push_str(rest);
    out
}

fn parse_macros(spec: &str, into: &mut HashMap<String, String>) {
    for pair in spec.split(',').filter(|p| !p.trim().is_empty()) {
        let (k, v) = pair.split_once('=').expect("macro without '='");
        into.insert(k.trim().to_string(), v.trim().to_string());
    }
}

/// The quoted arguments of `name("a", "b", ...)` on one st.cmd line.
fn quoted_args(line: &str) -> Vec<String> {
    line.split('"')
        .enumerate()
        .filter(|(i, _)| i % 2 == 1)
        .map(|(_, s)| s.to_string())
        .collect()
}

fn parse_records(db: &str, into: &mut HashMap<String, Record>) {
    let mut rest = db;
    while let Some(start) = rest.find("record(") {
        let head_end = rest[start..].find(')').unwrap() + start;
        let head = &rest[start + "record(".len()..head_end];
        let (rtype, name) = head.split_once(',').unwrap();
        let name = name.trim().trim_matches('"').to_string();
        let body_start = rest[head_end..].find('{').unwrap() + head_end;
        let body_end = rest[body_start..].find('}').unwrap() + body_start;
        let mut fields = HashMap::new();
        for field in rest[body_start + 1..body_end].split("field(").skip(1) {
            let close = field.find(')').unwrap();
            if let Some((k, v)) = field[..close].split_once(',') {
                fields.insert(k.trim().to_string(), v.trim().trim_matches('"').to_string());
            }
        }
        into.insert(
            name,
            Record {
                rtype: rtype.trim().to_string(),
                fields,
            },
        );
        rest = &rest[body_end + 1..];
    }
}

fn load(ioc: &str) -> Ioc {
    let ioc_dir = family_dir().join(ioc);
    let st = std::fs::read_to_string(ioc_dir.join("st.cmd")).unwrap();
    let mut env = HashMap::new();
    env.insert(
        "MEASCOMP".to_string(),
        family_dir().to_string_lossy().into_owned(),
    );
    let mut records = HashMap::new();
    for line in st.lines().map(str::trim).filter(|l| !l.starts_with('#')) {
        let args = quoted_args(line);
        if line.starts_with("epicsEnvSet") {
            env.insert(args[0].clone(), args[1].clone());
        } else if line.starts_with("dbLoadRecords") && args[0].starts_with("$(MEASCOMP)") {
            let path = expand(&args[0], &env);
            let mut macros = env.clone();
            parse_macros(&expand(&args[1], &env), &mut macros);
            let db = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
            parse_records(&expand(&db, &macros), &mut records);
        }
    }
    let prefix = env.get("PREFIX").expect("PREFIX").clone();
    let req = std::fs::read_to_string(ioc_dir.join("auto_settings.req")).unwrap();
    let restored = req
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|l| l.replace("$(P)", &prefix))
        .collect();
    Ioc { records, restored }
}

/// Autosave pass 1 writes VAL without processing the record, so a restored
/// setting reaches the driver only through the record's PINI. Soft records
/// (no DTYP) hold the value themselves and are exempt.
#[test]
fn every_restored_driver_setting_is_written_at_init() {
    let mut missing = Vec::new();
    for ioc in IOCS {
        let loaded = load(ioc);
        for name in loaded.restored.iter().filter(|n| !n.contains('.')) {
            let Some(record) = loaded.records.get(name) else {
                if !name.contains("scaler1") {
                    missing.push(format!("{ioc}: {name} is restored but not loaded"));
                }
                continue;
            };
            let output = matches!(
                record.rtype.as_str(),
                "ao" | "bo" | "mbbo" | "mbboDirect" | "longout"
            );
            if output && record.field("DTYP").is_some() && record.field("PINI") != Some("YES") {
                missing.push(format!("{ioc}: {name} has no PINI"));
            }
        }
    }
    assert!(missing.is_empty(), "{}", missing.join("\n"));
}

/// Record-writing link fields: an output link names the record whose value
/// it sets (`FLNK` and a fanout's `LNKn` only process, `INP*`/`DOL*` read).
fn is_output_link(rtype: &str, field: &str) -> bool {
    field == "OUT"
        || (field.len() == 4 && field.starts_with("OUT"))
        || (rtype != "fanout" && field.len() == 4 && field.starts_with("LNK"))
}

/// A soft record that another record writes holds derived state: at init the
/// writer recomputes it from the driver-bound source and the restored value
/// is lost (C's measCompPulseGen_settings.req restored Width, which CalcWidth
/// overwrites from DutyCycle). The setting to restore is the source.
#[test]
fn no_restored_soft_record_is_written_by_another_record() {
    let mut derived = Vec::new();
    for ioc in IOCS {
        let loaded = load(ioc);
        for name in loaded.restored.iter().filter(|n| !n.contains('.')) {
            let Some(record) = loaded.records.get(name) else {
                continue;
            };
            if record.field("DTYP").is_some() {
                continue;
            }
            for (writer, other) in &loaded.records {
                for (field, link) in &other.fields {
                    let target = link.split_whitespace().next().unwrap_or("");
                    let target = target.split('.').next().unwrap_or("");
                    if is_output_link(&other.rtype, field) && target == name {
                        derived.push(format!(
                            "{ioc}: {name} is restored but {writer}.{field} writes it"
                        ));
                    }
                }
            }
        }
    }
    assert!(derived.is_empty(), "{}", derived.join("\n"));
}
