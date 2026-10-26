//! st.cmd boots with the records C's st.cmd.AGAP_CONEX loads, on the
//! script's prefix.

use std::path::Path;

use ioc_conventions::{assert_same_records, booted_records, read_dbl, renamed};

#[test]
fn st_cmd_loads_the_c_agap_conex_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let c = renamed(
        &read_dbl(&dir.join("tests/iocNewport.AGAP_CONEX.dbl")),
        &[("IOC:", "AGAP:")],
        &[],
    );
    let ioc = Path::new(env!("CARGO_BIN_EXE_agap-conex-ioc"));
    assert_same_records(&booted_records(ioc, &dir.join("st.cmd")), &c);
}
