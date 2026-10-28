//! A shipped db against the port's real interface masks: the check C makes
//! at record init, where `findInterface` fails a record whose DTYP interface
//! the port lacks and `getInterruptPvt` fails one that registers for
//! callbacks on an interface outside the interrupt mask.

use std::collections::HashMap;
use std::path::Path;

use epics_rs::asyn::interfaces::InterfaceMask;
use epics_rs::asyn::interfaces::InterfaceType;
use epics_rs::base::server::db_loader::{parse_db_file, DbLoadConfig, DbRecordDef, MacroDefs};

/// The interface a record's DTYP binds (devAsyn*.c `initCommon`), `None` for
/// a DTYP that is not an asyn one.
pub fn dtyp_interface(dtyp: &str) -> Option<InterfaceType> {
    use InterfaceType as I;
    let array = |t: &str| match t {
        "Int8" => Some(I::Int8Array),
        "Int16" => Some(I::Int16Array),
        "Int32" => Some(I::Int32Array),
        "Int64" => Some(I::Int64Array),
        "Float32" => Some(I::Float32Array),
        "Float64" => Some(I::Float64Array),
        _ => None,
    };
    let rest = dtyp.strip_prefix("asyn")?;
    if let Some(at) = rest.find("Array") {
        return array(&rest[..at]);
    }
    if rest.starts_with("Octet") {
        Some(I::Octet)
    } else if rest.starts_with("UInt32Digital") {
        Some(I::UInt32Digital)
    } else if rest.starts_with("Int32") {
        Some(I::Int32)
    } else if rest.starts_with("Int64") {
        Some(I::Int64)
    } else if rest.starts_with("Float64") {
        Some(I::Float64)
    } else {
        None
    }
}

/// What a record needs of its port.
#[derive(Debug, PartialEq, Eq)]
pub struct Need {
    pub record: String,
    pub interface: InterfaceType,
    /// `SCAN="I/O Intr"`, an `asyn:READBACK` output or an averaging input:
    /// the record registers an interrupt user on `interface`.
    pub interrupt: bool,
}

/// The records of a db that has no `include` and sets no macro a DTYP or
/// SCAN depends on: every macro the file names is defined, so the names
/// expand.
pub fn parse(db: &Path) -> Vec<DbRecordDef> {
    let text = std::fs::read(db).unwrap_or_else(|e| panic!("{}: {e}", db.display()));
    let mut macros = HashMap::new();
    for open in text
        .windows(2)
        .enumerate()
        .filter_map(|(i, w)| (w[0] == b'$' && (w[1] == b'(' || w[1] == b'{')).then_some(i + 2))
    {
        let name: Vec<u8> = text[open..]
            .iter()
            .copied()
            .take_while(|b| b.is_ascii_alphanumeric() || *b == b'_')
            .collect();
        macros.insert(name, b"m".to_vec());
    }
    parse_db_file(
        db,
        MacroDefs::from_byte_map(&macros),
        &DbLoadConfig::default(),
    )
    .unwrap_or_else(|e| panic!("{}: {e}", db.display()))
}

/// Every asyn record of `defs`, with what it needs of its port.
pub fn needs(defs: &[DbRecordDef]) -> Vec<Need> {
    defs.iter()
        .filter_map(|def| {
            let field = |name: &str| {
                def.fields
                    .iter()
                    .find(|f| f.name == name)
                    .map(|f| String::from_utf8_lossy(f.value.as_bytes()).into_owned())
            };
            let dtyp = field("DTYP")?;
            let interface = dtyp_interface(&dtyp)?;
            let readback = def
                .info_tags
                .iter()
                .any(|(tag, value)| tag == "asyn:READBACK" && value == "1");
            let interrupt = field("SCAN").is_some_and(|s| s == "I/O Intr")
                || readback
                || dtyp.ends_with("Average");
            Some(Need {
                record: def.name.to_str_lossy().into_owned(),
                interface,
                interrupt,
            })
        })
        .collect()
}

/// The records of `defs` the port refuses: an interface outside
/// `interfaces`, or an interrupt user outside `interrupts`.
pub fn refused(
    defs: &[DbRecordDef],
    interfaces: InterfaceMask,
    interrupts: InterfaceMask,
) -> Vec<String> {
    let mut out = Vec::new();
    for need in needs(defs) {
        let bit = InterfaceMask::of(need.interface).expect("DTYP interfaces have a mask bit");
        if !interfaces.contains(bit) {
            out.push(format!(
                "{}: {} is not in the interface mask",
                need.record, need.interface
            ));
        } else if need.interrupt && !interrupts.contains(bit) {
            out.push(format!(
                "{}: {} is not in the interrupt mask",
                need.record, need.interface
            ));
        }
    }
    out
}

/// Panics when the port refuses a record of `defs`, or when `defs` holds no
/// asyn record at all (a loader gone stale finds none).
pub fn assert_wired(defs: &[DbRecordDef], interfaces: InterfaceMask, interrupts: InterfaceMask) {
    assert!(!needs(defs).is_empty(), "no asyn record in the db");
    let refusals = refused(defs, interfaces, interrupts);
    assert!(
        refusals.is_empty(),
        "record init would refuse:\n{}",
        refusals.join("\n")
    );
}
