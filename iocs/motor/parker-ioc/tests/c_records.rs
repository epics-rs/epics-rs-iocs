//! Each startup script boots with the records C's iocParker loads for its
//! controller, on the script's prefix and axes.

use std::path::Path;

use ioc_conventions::{
    assert_inits_motor_util, assert_links_asyn_dbd, assert_same_fields, assert_same_records,
    booted_records, read_db, read_dbl, renamed, renamed_fields, script_fields,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_parker-ioc"))
}

/// `script`'s loads and dbpf lines leave every field as C's `db` leaves it.
fn assert_sets_c_fields(script: &str, db: &str, renames: &[(&str, &str)], drops: &[&str]) {
    let c = renamed_fields(&read_db(&dir().join("tests").join(db)), renames, drops);
    assert_same_fields(&script_fields(&dir().join(script)), &c);
}

const OEM_RENAMES: &[(&str, &str)] = &[("IOC:", "OEM:")];
const OEM_DROPS: &[&str] = &["IOC:m2", "IOC:m3"];
// C's asyn2 sits on ARIES1 beside asyn1, its ARIES2 port's records on the
// second controller this IOC does not configure.
const ACR_RENAMES: &[(&str, &str)] = &[("IOC:", "ACR:")];
const ACR_DROPS: &[&str] = &["IOC:m2", "IOC:ACR2:", "IOC:asyn2"];
const PC6K_RENAMES: &[(&str, &str)] = &[("parker:", "PC6K:")];
const PC6K_DROPS: &[&str] = &["parker:m2"];

#[test]
fn st_cmd_loads_the_c_oem_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.oem.dbl")),
        OEM_RENAMES,
        OEM_DROPS,
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_cmd_sets_the_c_oem_fields() {
    assert_sets_c_fields("st.cmd", "st.cmd.oem.db", OEM_RENAMES, OEM_DROPS);
}

#[test]
fn st_acr_cmd_loads_the_c_acr_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.acr.dbl")),
        ACR_RENAMES,
        ACR_DROPS,
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.acr.cmd")), &c);
}

#[test]
fn st_acr_cmd_sets_the_c_acr_fields() {
    assert_sets_c_fields("st.acr.cmd", "st.cmd.acr.db", ACR_RENAMES, ACR_DROPS);
}

#[test]
fn st_pc6k_cmd_loads_the_c_pc6k_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/st.cmd.dbl")),
        PC6K_RENAMES,
        PC6K_DROPS,
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st-pc6k.cmd")), &c);
}

#[test]
fn st_pc6k_cmd_sets_the_c_pc6k_fields() {
    assert_sets_c_fields("st-pc6k.cmd", "st.cmd.db", PC6K_RENAMES, PC6K_DROPS);
}

/// The binary links asyn.dbd, as C's iocParker does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_parker-ioc")));
}

/// st-pc6k.cmd starts motorUtil after iocInit, as C's iocParker st.cmd does.
#[test]
fn st_pc6k_cmd_inits_motor_util() {
    assert_inits_motor_util(ioc(), &dir().join("st-pc6k.cmd"));
}
