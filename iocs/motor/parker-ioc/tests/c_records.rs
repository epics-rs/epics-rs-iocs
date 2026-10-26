//! Each startup script loads the records C's iocParker loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn st_cmd_loads_the_c_oem_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.oem.dbl")),
        &[("IOC:", "OEM:")],
        &["IOC:m2", "IOC:m3"],
    );
    assert_same_records(&script_records(&dir().join("st.cmd")), &c);
}

#[test]
fn st_acr_cmd_loads_the_c_acr_records() {
    // C's asyn2 sits on ARIES1 beside asyn1, its ARIES2 port's records on
    // the second controller this IOC does not configure.
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.acr.dbl")),
        &[("IOC:ACR1:", "ACR:PARKER1:"), ("IOC:", "ACR:")],
        &["IOC:m2", "IOC:ACR2:", "IOC:asyn2"],
    );
    assert_same_records(&script_records(&dir().join("st.acr.cmd")), &c);
}

#[test]
fn st_pc6k_cmd_loads_the_c_pc6k_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.dbl")),
        &[("parker:", "PC6K:")],
        &["parker:m2"],
    );
    assert_same_records(&script_records(&dir().join("st-pc6k.cmd")), &c);
}
