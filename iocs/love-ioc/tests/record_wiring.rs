//! Every shipped record's DTYP names an interface the love port registers.

use std::path::PathBuf;

use ioc_conventions::wiring;
use love::driver::{INTERFACES, INTERRUPTS};

#[test]
fn every_record_binds_an_interface_the_love_port_registers() {
    for file in ["LoveController.template", "LoveControllerControl.template"] {
        let db = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("db")
            .join(file);
        wiring::assert_wired(&wiring::parse(&db), INTERFACES, INTERRUPTS);
    }
}
