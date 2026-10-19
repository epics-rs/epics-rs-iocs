//! The MPC, TPG261, Eurotherm and heidND261 databases are ipApp/Db's, whose
//! device records carry the per-record links devMPC.c, devTPG261.c,
//! devXxStrParm.c and devAiHeidND261.c parse.

use std::path::Path;

/// `(record type, DTYP, INP/OUT)` of every record in `file` that names a DTYP.
fn device_records(file: &str) -> Vec<(String, String, String)> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("db").join(file);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
    let text: String = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .map(|l| format!("{l}\n"))
        .collect();
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

/// ipApp/Db's Eurotherm.db: the controller is read through devXxStrParm and
/// written through devXxEurotherm. Upstream ships the writers with DTYP
/// `<none>` (:47, :57, :106, :116, :130), which no base accepts.
#[test]
fn eurotherm_records_read_through_dev_xx_str_parm() {
    let records = device_records("Eurotherm.db");
    assert_eq!(records.len(), 13, "{records:?}");
    let count = |dtyp: &str| records.iter().filter(|(_, d, _)| d == dtyp).count();
    assert_eq!(count("asyn ai stringParm"), 6, "{records:?}");
    assert_eq!(count("asyn si stringParm"), 2, "{records:?}");
    assert_eq!(count("asyn ao Eurotherm"), 2, "{records:?}");
    assert_eq!(count("asyn so Eurotherm"), 3, "{records:?}");
}

/// ipApp/Db's heidND261.db: one devAiHeidND261 ai feeding two calcs.
#[test]
fn heid_nd261_reads_through_dev_ai_heid_nd261() {
    let records = device_records("heidND261.db");
    assert_eq!(
        records,
        [(
            "ai".to_owned(),
            "asyn ai HeidND261".to_owned(),
            "@asyn($(PORT))".to_owned()
        )]
    );
}

/// devTPG261.c:12-13: the gauge address is 1 or 2; any other makes readBi
/// index `vals[]` outside the array.
#[test]
fn st_cmd_tpg261_gauges_are_1_and_2() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("st.cmd");
    let text = std::fs::read_to_string(&path).unwrap();
    let addrs: Vec<&str> = text
        .lines()
        .filter(|l| l.starts_with("dbLoadRecords(") && l.contains("/TPG261.db"))
        .map(|l| l.split("ADDR=").nth(1).unwrap().trim_end_matches("\")"))
        .collect();
    assert_eq!(addrs, ["1", "2"]);
}
