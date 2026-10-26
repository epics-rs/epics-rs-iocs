//! Each startup script boots with the records C's iocPhytron loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, booted_records, read_dbl, renamed};

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
        &["TEST:m2"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_mcc_cmd_loads_the_c_mcc_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.mcc.dbl")),
        &RENAMES,
        &["TEST:m2"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.mcc.cmd")), &c);
}
