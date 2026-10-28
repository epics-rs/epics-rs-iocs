//! Every shipped record's DTYP names an interface the ADS port registers,
//! and every `I/O Intr` record one it posts interrupts for.

use std::path::PathBuf;

use ioc_conventions::wiring;
use twincat_ads::driver::{INTERFACES, INTERRUPTS};

#[test]
fn every_record_binds_an_interface_the_ads_port_registers() {
    let db = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("db/adsTestAsyn.db");
    wiring::assert_wired(&wiring::parse(&db), INTERFACES, INTERRUPTS);
}
