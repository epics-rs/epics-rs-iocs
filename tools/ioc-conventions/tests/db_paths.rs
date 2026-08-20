//! Every db load in `iocs/**/*.cmd` follows the one canonical form.
//!
//! Rule: `dbLoadRecords`/`dbLoadTemplate` paths are `$(MACRO)/db/<file>`
//! (no cwd-relative, no repo-root-relative, no `..` traversal, no
//! `EPICS_DB_INCLUDE_PATH` search), where the IOC's own Rust sources give
//! `MACRO` a `set_default` pointing at the owning directory. Crates that
//! ship their templates ($(ADCORE) from ad-core-rs, $(SCALER) from
//! scaler-rs) follow the same form; for those only the form is checked
//! here — existence is the dependency's concern.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Macros whose owning directory lives in a dependency crate.
const EXTERNAL: &[&str] = &["ADCORE", "SCALER"];

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("tools/ioc-conventions sits two levels below the root")
        .to_path_buf()
}

fn collect_cmd_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).expect("readable dir") {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            if path.file_name().is_some_and(|n| n == "target") {
                continue;
            }
            collect_cmd_files(&path, out);
        } else if path.extension().is_some_and(|e| e == "cmd") {
            out.push(path);
        }
    }
}

/// What a `set_default` in the IOC's sources points the macro at.
enum MacroTarget {
    /// `env!("CARGO_MANIFEST_DIR")` — the directory holding the .cmd file.
    IocDir,
    /// `concat!(env!("CARGO_MANIFEST_DIR"), "/..")` — the family directory.
    FamilyDir,
    /// A crate-exported const (e.g. `epics_rs::scaler::SCALER_DB_DIR`).
    ExternalConst,
}

/// Scan `<ioc>/src/**.rs` (and nested crates') for `set_default("NAME", ...)`.
fn macro_table(ioc_dir: &Path) -> HashMap<String, MacroTarget> {
    let mut table = HashMap::new();
    let mut rs_files = Vec::new();
    fn walk_rs(dir: &Path, out: &mut Vec<PathBuf>) {
        for entry in std::fs::read_dir(dir).expect("readable dir") {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                if path.file_name().is_some_and(|n| n == "target") {
                    continue;
                }
                walk_rs(&path, out);
            } else if path.extension().is_some_and(|e| e == "rs") {
                out.push(path);
            }
        }
    }
    walk_rs(ioc_dir, &mut rs_files);
    for rs in rs_files {
        let text = std::fs::read_to_string(&rs).expect("readable source");
        // rustfmt may wrap the call across lines, so tolerate whitespace
        // between `set_default(` and the name literal.
        let mut rest = text.as_str();
        while let Some(pos) = rest.find("set_default(") {
            rest = rest[pos + "set_default(".len()..].trim_start();
            let Some(stripped) = rest.strip_prefix('"') else {
                continue;
            };
            rest = stripped;
            let Some(end) = rest.find('"') else { break };
            let name = &rest[..end];
            let call_tail = &rest[end..rest.len().min(end + 200)];
            let target = if call_tail.contains("\"/..\"") {
                MacroTarget::FamilyDir
            } else if call_tail.contains("CARGO_MANIFEST_DIR") {
                MacroTarget::IocDir
            } else {
                MacroTarget::ExternalConst
            };
            table.insert(name.to_string(), target);
        }
    }
    table
}

/// `epicsEnvSet("NAME", "literal")` pairs in one .cmd file, for expanding
/// macros used inside the filename (e.g. quadEM's `$(TEMPLATE).template`).
fn env_sets(text: &str) -> HashMap<String, String> {
    let mut envs = HashMap::new();
    for line in text.lines() {
        let Some(rest) = line.trim_start().strip_prefix("epicsEnvSet(\"") else {
            continue;
        };
        let Some((name, rest)) = rest.split_once('"') else {
            continue;
        };
        let Some(pos) = rest.find('"') else { continue };
        let Some(end) = rest[pos + 1..].find('"') else {
            continue;
        };
        let value = &rest[pos + 1..pos + 1 + end];
        if !value.contains('$') {
            envs.insert(name.to_string(), value.to_string());
        }
    }
    envs
}

/// The one sanctioned db search path, needed only for `include` lines
/// *inside* templates (ADBase.template and friends ship with ad-core-rs).
const CANONICAL_INCLUDE_PATH: &str = r#"epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")"#;

/// Does `dir/file` (transitively, within `dir`) include a template that is
/// not present in `dir`? Such an include only resolves through the search
/// path, so the loading st.cmd must declare [`CANONICAL_INCLUDE_PATH`].
fn has_external_include(dir: &Path, file: &str, seen: &mut Vec<String>) -> bool {
    if seen.iter().any(|s| s == file) {
        return false;
    }
    seen.push(file.to_string());
    let Ok(text) = std::fs::read_to_string(dir.join(file)) else {
        return false;
    };
    for line in text.lines() {
        let Some(rest) = line.trim_start().strip_prefix("include") else {
            continue;
        };
        let Some(inc) = rest
            .trim_start()
            .strip_prefix('"')
            .and_then(|r| r.split('"').next())
        else {
            continue;
        };
        if !dir.join(inc).exists() || has_external_include(dir, inc, seen) {
            return true;
        }
    }
    false
}

fn expand(name_part: &str, envs: &HashMap<String, String>) -> Option<String> {
    let mut out = String::new();
    let mut rest = name_part;
    while let Some(pos) = rest.find("$(") {
        out.push_str(&rest[..pos]);
        let tail = &rest[pos + 2..];
        let end = tail.find(')')?;
        out.push_str(envs.get(&tail[..end])?);
        rest = &tail[end + 1..];
    }
    out.push_str(rest);
    Some(out)
}

#[test]
fn every_db_load_is_macro_form_and_resolvable() {
    let root = workspace_root();
    let mut cmd_files = Vec::new();
    collect_cmd_files(&root.join("iocs"), &mut cmd_files);
    cmd_files.sort();
    assert!(!cmd_files.is_empty(), "no .cmd files found under iocs/");

    let mut failures = Vec::new();
    let mut checked = 0usize;

    for cmd in &cmd_files {
        let ioc_dir = cmd.parent().expect("cmd file has a parent");
        let macros = macro_table(ioc_dir);
        let text = std::fs::read_to_string(cmd).expect("readable .cmd");
        let envs = env_sets(&text);
        let mut needs_include_path = false;

        for (lineno, line) in text.lines().enumerate() {
            let trimmed = line.trim_start();
            if trimmed.starts_with('#') {
                continue;
            }
            for keyword in ["dbLoadRecords(\"", "dbLoadTemplate(\""] {
                let Some(pos) = line.find(keyword) else {
                    continue;
                };
                let arg = &line[pos + keyword.len()..];
                let path = arg.split(['"', ',']).next().unwrap_or("");
                checked += 1;
                let at = format!(
                    "{}:{}",
                    cmd.strip_prefix(&root).unwrap_or(cmd).display(),
                    lineno + 1
                );

                if path.contains("..") {
                    failures.push(format!("{at}: path traversal in {path:?}"));
                    continue;
                }
                let Some(rest) = path.strip_prefix("$(") else {
                    failures.push(format!("{at}: not macro-form: {path:?}"));
                    continue;
                };
                let Some((name, rest)) = rest.split_once(')') else {
                    failures.push(format!("{at}: unclosed macro in {path:?}"));
                    continue;
                };
                let external = EXTERNAL.contains(&name);
                let Some(file) = rest.strip_prefix("/db/") else {
                    failures.push(format!("{at}: not $({name})/db/<file>: {path:?}"));
                    continue;
                };
                if file.contains('/') {
                    failures.push(format!("{at}: nested path after db/: {path:?}"));
                    continue;
                }
                let Some(file) = expand(file, &envs) else {
                    failures.push(format!("{at}: unexpandable macro in filename {path:?}"));
                    continue;
                };
                let local_db = match macros.get(name) {
                    Some(MacroTarget::IocDir) => Some(ioc_dir.join("db")),
                    Some(MacroTarget::FamilyDir) => {
                        Some(ioc_dir.parent().expect("ioc dir has a parent").join("db"))
                    }
                    Some(MacroTarget::ExternalConst) => None,
                    None if external => None,
                    None => {
                        failures.push(format!(
                            "{at}: $({name}) has no set_default in {}",
                            ioc_dir.strip_prefix(&root).unwrap_or(ioc_dir).display()
                        ));
                        None
                    }
                };
                if let Some(db_dir) = local_db {
                    if !db_dir.join(&file).exists() {
                        failures.push(format!("{at}: $({name})/db/{file} missing"));
                    } else if has_external_include(&db_dir, &file, &mut Vec::new()) {
                        needs_include_path = true;
                    }
                }
            }
        }

        // Template-internal `include "..."` lines resolve through the db
        // search path, not through dbLoad's explicit path. An IOC whose
        // templates include an ADCore base template must declare exactly
        // the canonical single-entry search path — anything else is the
        // multi-convention drift this test exists to stop.
        let declares = text.contains(CANONICAL_INCLUDE_PATH);
        if text.contains("EPICS_DB_INCLUDE_PATH") && !declares {
            failures.push(format!(
                "{}: EPICS_DB_INCLUDE_PATH set to something other than {CANONICAL_INCLUDE_PATH:?}",
                cmd.strip_prefix(&root).unwrap_or(cmd).display()
            ));
        }
        if needs_include_path && !declares {
            failures.push(format!(
                "{}: templates have external includes but the file does not set {CANONICAL_INCLUDE_PATH:?}",
                cmd.strip_prefix(&root).unwrap_or(cmd).display()
            ));
        }
    }

    assert!(
        checked > 300,
        "suspiciously few db loads checked: {checked}"
    );
    assert!(
        failures.is_empty(),
        "non-canonical or unresolvable db loads:\n{}",
        failures.join("\n")
    );
}
