//! st.cmd loads the records C's iocMotorSim st.cmd loads, on this IOC's
//! prefix and its two axes.

use std::path::Path;

use ioc_conventions::{assert_same_records, read_dbl, renamed, script_records};

#[test]
fn st_cmd_loads_the_c_records() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut c = renamed(
        &read_dbl(&dir.join("tests/motorSim.dbl")),
        &[("IOC:", "SIM:")],
        &["IOC:m3", "IOC:m4", "IOC:m5", "IOC:m6", "IOC:m7", "IOC:m8"],
    );
    // motorUtil.db, which C's st.cmd runs motorUtilInit without.
    c.extend(
        ["allstop", "alldone", "moving", "alldoneBlink", "movingDiff"].map(|r| format!("SIM:{r}")),
    );
    assert_same_records(&script_records(&dir.join("st.cmd")), &c);
}
