//! quadem/saveRestore.cmd, shared by every quadEM IOC, may read only macros
//! the rs IOC defines; any other prints macLib "<name> is undefined" at boot.
//! It runs after quadEM_Plugins.cmd's commonPlugins.cmd, whose autosave block
//! already does the rest of what C's saveRestore.cmd does.

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

/// What commonPlugins.cmd's autosave block (C EXAMPLE_commonPlugins.cmd:165-173)
/// already sets for every quadEM IOC.
const COMMON_PLUGINS_SETS: &[&str] = &[
    "set_requestfile_path(\"./\")",
    "set_requestfile_path(\"$(ADCORE)/db\")",
    "set_savefile_path(",
    "set_pass0_restoreFile(",
    "set_pass1_restoreFile(",
    "save_restoreSet_status_prefix(",
    "save_restoreStatus.db",
];

#[test]
fn save_restore_repeats_nothing_common_plugins_sets() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../saveRestore.cmd");
    let text = std::fs::read_to_string(&path).unwrap();
    let repeated: Vec<&str> = text
        .lines()
        .filter(|l| COMMON_PLUGINS_SETS.iter().any(|c| l.contains(c)))
        .collect();
    assert!(
        repeated.is_empty(),
        "{} repeats {repeated:?}",
        path.display()
    );
}
