//! quadem/saveRestore.cmd, shared by every quadEM IOC, may read only macros
//! the rs IOC defines; any other prints macLib "<name> is undefined" at boot.

use std::path::Path;

/// st.cmd's PREFIX, ioc_support's QUADEM, and the ADCORE and AUTOSAVE that
/// ad-core-rs and epics-base-rs export.
const DEFINED: &[&str] = &["PREFIX", "QUADEM", "ADCORE", "AUTOSAVE"];

#[test]
fn save_restore_reads_only_defined_macros() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../saveRestore.cmd");
    let text = std::fs::read_to_string(&path).unwrap();
    let undefined: Vec<&str> = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .flat_map(|l| l.split("$(").skip(1))
        .map(|m| {
            m.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .next()
                .unwrap()
        })
        .filter(|m| !DEFINED.contains(m))
        .collect();
    assert!(
        undefined.is_empty(),
        "{} reads {undefined:?}",
        path.display()
    );
}
