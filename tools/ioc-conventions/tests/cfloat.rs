//! epics-rs's `cfloat-gate` over this workspace: in production source, a
//! format placeholder with a precision or the `e` / `E` type formats a
//! `CFloat`, the one owner of C's `%f` / `%e` spelling. The rule and its
//! reasons live in that crate; this file supplies the crates and the
//! exemptions, which are this workspace's own.
//!
//! The crates are every workspace member, so a new driver or IOC is inside
//! the gate on creation.

use std::path::Path;

/// `(file, placeholder, argument, reason)`, matched exactly; an entry that
/// matches no site fails the gate.
const EXEMPT: &[(&str, &str, &str, &str)] = &[];

/// The root manifest's `[workspace] members` that have a `src`.
fn members(root: &Path) -> Vec<String> {
    let manifest = std::fs::read_to_string(root.join("Cargo.toml")).expect("root Cargo.toml");
    let (_, rest) = manifest
        .split_once("\nmembers = [")
        .expect("[workspace] members");
    let (list, _) = rest.split_once("\n]").expect("members list closes");
    list.lines()
        .map(|l| l.split('#').next().unwrap().trim().trim_end_matches(','))
        .filter_map(|l| l.strip_prefix('"')?.strip_suffix('"'))
        .filter(|m| root.join(m).join("src").is_dir())
        .map(str::to_string)
        .collect()
}

#[test]
fn every_float_placeholder_formats_a_cfloat() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("workspace root");
    cfloat_gate::check(root, &members(root), EXEMPT);
}
