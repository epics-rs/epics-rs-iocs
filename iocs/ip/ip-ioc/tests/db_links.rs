//! The MPC and TPG261 databases are ipApp/Db's, whose device records carry
//! the per-record links devMPC.c and devTPG261.c parse.

use std::path::Path;

/// `(record type, DTYP, INP/OUT)` of every record in `file` that names a DTYP.
fn device_records(file: &str) -> Vec<(String, String, String)> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("db").join(file);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
    let mut out = Vec::new();
    for block in text.split("record(").skip(1) {
        let rtyp = block.split(',').next().unwrap().trim().to_owned();
        let field = |name: &str| {
            block
                .split("field(")
                .skip(1)
                .find_map(|f| f.strip_prefix(name)?.trim_start().strip_prefix(','))
                .map(|v| v.split('"').nth(1).unwrap_or("").to_owned())
        };
        if let Some(dtyp) = field("DTYP") {
            let link = field("INP").or_else(|| field("OUT")).unwrap_or_default();
            out.push((rtyp, dtyp, link));
        }
    }
    out
}

#[test]
fn mpc_and_tsp_records_use_dev_mpc() {
    for (file, count, supply) in [("MPC.db", 23, "$(PN)"), ("TSP.db", 9, "1")] {
        let records = device_records(file);
        assert_eq!(records.len(), count, "{file}: {records:?}");
        for (rtyp, dtyp, link) in records {
            assert_eq!(dtyp, "asyn MPC", "{file} {rtyp} {link}");
            let command = link
                .strip_prefix("@asyn($(PORT) $(PA))")
                .unwrap_or_else(|| panic!("{file} {rtyp}: {link}"));
            assert!(command.ends_with(&format!(" {supply}")), "{file}: {link}");
        }
    }
}

#[test]
fn tpg261_records_use_dev_tpg261() {
    let records = device_records("TPG261.db");
    assert_eq!(records.len(), 12, "{records:?}");
    for (rtyp, dtyp, link) in records {
        assert_eq!(dtyp, "asyn TPG261", "{rtyp} {link}");
        assert!(link.starts_with("@asyn($(PORT) $(ADDR))"), "{rtyp}: {link}");
    }
}
