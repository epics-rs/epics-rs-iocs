//! Each startup script boots with the records C's iocPhytron loads for its
//! controller, on the script's prefix.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_fields, assert_same_records, booted_records, read_db,
    read_dbl, renamed, renamed_fields, script_fields,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_phytron-ioc"))
}

// C's I/O and controller rows carry no prefix; this IOC's are under its P.
const RENAMES: [(&str, &str); 5] = [
    ("TEST:", "phytron:"),
    ("phytron-", "phytron:-"),
    ("phyA", "phytron:phyA"),
    ("phyD", "phytron:phyD"),
    ("phy:", "phytron:phy:"),
];

/// C's st.cmd.phytron with motor.substitutions.phytron's phyMOTION blocks
/// (see st.cmd).
#[test]
fn st_cmd_loads_the_c_phymotion_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.phytron.dbl")),
        &RENAMES,
        &[],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

/// st.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_cmd_sets_the_c_phymotion_fields() {
    let c = renamed_fields(
        &read_db(&dir().join("tests/st.cmd.phytron.db")),
        &RENAMES,
        &[],
    );
    assert_same_fields(&script_fields(&dir().join("st.cmd")), &c);
}

#[test]
fn st_mcc_cmd_loads_the_c_mcc_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.mcc.dbl")),
        &RENAMES,
        &[],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.mcc.cmd")), &c);
}

/// st.mcc.cmd's loads and dbpf lines leave every field as C's leave it.
#[test]
fn st_mcc_cmd_sets_the_c_mcc_fields() {
    let mut c = renamed_fields(&read_db(&dir().join("tests/st.cmd.mcc.db")), &RENAMES, &[]);
    // C's STEP_RES "1/16" makes DOL7 a link to a PV of that name and its
    // LNK8 names a PS-MONITOR_SET Phytron_MCC.db lacks; st.mcc.cmd passes
    // the state index and its db leaves LNK8 out.
    for m in ["m1", "m2"] {
        let init = &mut c
            .get_mut(&format!("phytron:{m}-INIT-AXIS-2"))
            .unwrap()
            .fields;
        init.insert("DOL7".into(), "5".into());
        init.remove("DOL8");
        init.remove("LNK8");
    }
    assert_same_fields(&script_fields(&dir().join("st.mcc.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocPhytron does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_phytron-ioc")));
}
