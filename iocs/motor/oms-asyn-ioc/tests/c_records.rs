//! Each startup script boots with the records C's iocOmsAsyn st.cmd loads
//! for its controller, on this IOC's prefix and axes.

use std::path::Path;

use ioc_conventions::{
    assert_links_asyn_dbd, assert_same_records, booted_records, read_dbl, renamed,
};

fn dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn ioc() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_oms-asyn-ioc"))
}

/// C's st.cmd names the MAXnet's axis m2 and the MAXv's m1; there is no MAXv.
#[test]
fn st_cmd_loads_the_c_maxnet_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/iocOmsAsyn.dbl")),
        &[("omsAsyn:", "oms:"), ("omsAsyn:m2", "oms:m1")],
        &["omsAsyn:m1"],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.cmd")), &c);
}

#[test]
fn st_mxa_cmd_loads_the_c_mxa_records() {
    let c = renamed(
        &read_dbl(&dir().join("tests/MXA.dbl")),
        &[("omsAsyn:", "oms:")],
        &[],
    );
    assert_same_records(&booted_records(ioc(), &dir().join("st.mxa.cmd")), &c);
}

/// The binary links asyn.dbd, as C's iocOmsAsyn does.
#[test]
fn the_ioc_links_asyn_dbd() {
    assert_links_asyn_dbd(Path::new(env!("CARGO_BIN_EXE_oms-asyn-ioc")));
}
