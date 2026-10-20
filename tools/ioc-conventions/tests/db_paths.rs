//! Every db load in `iocs/**/*.cmd` resolves the way the IOC resolves it,
//! and is written in the one canonical form.
//!
//! Each startup script is run as iocsh runs it, from its own directory:
//! the IOC's `set_default` macros and the framework's envPaths
//! (`$(ADCORE)`, `$(ASYN)`, `$(AUTOSAVE)`, ... as the pinned epics-rs
//! exports them) first, then every line macro-expanded in order,
//! `epicsEnvSet` updating the environment and `<` running the fragment
//! with it, so a fragment's `$(TEMPLATE)` is the including st.cmd's.
//! A load then opens its file as `dbOpenFile` does: a path with a `/`
//! as is, a bare name through `EPICS_DB_INCLUDE_PATH`, and every
//! template `include` through that path too.
//!
//! Form: a path is `$(MACRO)/db/<file>` (no cwd-relative, no
//! repo-root-relative, no `..` traversal) or a bare name found through a
//! declared `EPICS_DB_INCLUDE_PATH`.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

/// envPaths macros the framework sets, and where each points in the crate
/// that sets it.
const FRAMEWORK: &[(&str, &str, &str)] = &[
    ("ADCORE", "ad-core-rs", ""),
    ("ASYN", "asyn-rs", ""),
    ("AUTOSAVE", "epics-base-rs", "autosave"),
    ("CALC", "epics-base-rs", "calc"),
];

/// The crates whose sources export [`FRAMEWORK`] macros.
const FRAMEWORK_CRATES: &[&str] = &["ad-plugins-rs", "asyn-rs", "epics-base-rs"];

/// Macros an IOC sets from a dependency crate's exported const.
const EXTERNAL: &[(&str, &str)] = &[("MCA", "mca"), ("SCALER", "scaler-rs")];

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("tools/ioc-conventions sits two levels below the root")
        .to_path_buf()
}

fn walk(dir: &Path, ext: &str, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).expect("readable dir") {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            if path.file_name().is_some_and(|n| n == "target") {
                continue;
            }
            walk(&path, ext, out);
        } else if path.extension().is_some_and(|e| e == ext) {
            out.push(path);
        }
    }
}

/// The directory of every package in the resolved dependency graph.
fn package_dirs(root: &Path) -> HashMap<String, PathBuf> {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    let out = std::process::Command::new(cargo)
        .args(["metadata", "--format-version", "1", "--offline"])
        .current_dir(root)
        .output()
        .expect("cargo metadata runs");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let json = String::from_utf8(out.stdout).expect("UTF-8 metadata");
    let mut dirs = HashMap::new();
    // A package object opens with its name and version (a dependency entry
    // has no version) and names its manifest path further on.
    let mut rest = json.as_str();
    while let Some(pos) = rest.find(r#"{"name":""#) {
        rest = &rest[pos + r#"{"name":""#.len()..];
        if !rest
            .find('"')
            .is_some_and(|end| rest[end..].starts_with(r#"","version":""#))
        {
            continue;
        }
        let Some(end) = rest.find('"') else { break };
        let name = rest[..end].to_string();
        let Some(m) = rest.find(r#""manifest_path":""#) else {
            break;
        };
        let tail = &rest[m + r#""manifest_path":""#.len()..];
        let Some(mend) = tail.find('"') else { break };
        let manifest = PathBuf::from(&tail[..mend]);
        if let Some(dir) = manifest.parent() {
            dirs.entry(name).or_insert_with(|| dir.to_path_buf());
        }
    }
    dirs
}

/// `set_default("NAME", ...)` calls in the `.rs` files under `dir`, with the
/// text after the name.
fn set_defaults(dir: &Path) -> Vec<(String, String)> {
    let mut rs_files = Vec::new();
    walk(dir, "rs", &mut rs_files);
    let mut found = Vec::new();
    for rs in rs_files {
        let text = std::fs::read_to_string(&rs).expect("readable source");
        // rustfmt may wrap the call across lines.
        let mut rest = text.as_str();
        while let Some(pos) = rest.find("set_default(") {
            rest = rest[pos + "set_default(".len()..].trim_start();
            let Some(stripped) = rest.strip_prefix('"') else {
                continue;
            };
            rest = stripped;
            let Some(end) = rest.find('"') else { break };
            let tail = &rest[end..rest.len().min(end + 200)];
            found.push((rest[..end].to_string(), tail.to_string()));
        }
    }
    found
}

/// The envPaths macros the pinned framework exports.
fn framework_env(packages: &HashMap<String, PathBuf>) -> HashMap<String, String> {
    let exported: HashSet<String> = FRAMEWORK_CRATES
        .iter()
        .filter_map(|c| packages.get(*c))
        .flat_map(|dir| set_defaults(&dir.join("src")))
        .map(|(name, _)| name)
        .collect();
    FRAMEWORK
        .iter()
        .filter(|(name, _, _)| exported.contains(*name))
        .filter_map(|(name, krate, sub)| {
            let dir = packages.get(*krate)?;
            let dir = if sub.is_empty() {
                dir.clone()
            } else {
                dir.join(sub)
            };
            Some((name.to_string(), dir.to_str()?.to_string()))
        })
        .collect()
}

/// The IOC's own `set_default` macros: its crate dir, its family dir, or a
/// dependency crate's dir.
fn ioc_env(ioc_dir: &Path, packages: &HashMap<String, PathBuf>) -> HashMap<String, String> {
    let mut env = HashMap::new();
    for (name, tail) in set_defaults(ioc_dir) {
        let dir = if tail.contains("\"/..\"") {
            ioc_dir.parent().map(Path::to_path_buf)
        } else if tail.contains("CARGO_MANIFEST_DIR") {
            Some(ioc_dir.to_path_buf())
        } else {
            EXTERNAL
                .iter()
                .find(|(n, _)| *n == name)
                .and_then(|(_, krate)| packages.get(*krate).cloned())
        };
        if let Some(dir) = dir.and_then(|d| d.to_str().map(str::to_string)) {
            env.insert(name, dir);
        }
    }
    env
}

/// macLib expansion of `$(NAME)`, `${NAME}` and `$(NAME=default)`; an
/// undefined macro stays as written.
fn expand(text: &str, env: &HashMap<String, String>) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(pos) = rest.find('$') {
        out.push_str(&rest[..pos]);
        let tail = &rest[pos + 1..];
        let close = match tail.chars().next() {
            Some('(') => ')',
            Some('{') => '}',
            _ => {
                out.push('$');
                rest = tail;
                continue;
            }
        };
        let Some(end) = tail.find(close) else {
            out.push_str(&rest[pos..]);
            return out;
        };
        let body = &tail[1..end];
        let (name, default) = match body.split_once('=') {
            Some((n, d)) => (n, Some(d)),
            None => (body, None),
        };
        match env.get(name).map(String::as_str).or(default) {
            Some(v) => out.push_str(v),
            None => out.push_str(&rest[pos..pos + 2 + end]),
        }
        rest = &tail[end + 1..];
    }
    out.push_str(rest);
    out
}

/// iocsh's argument split: blanks, commas and parentheses separate,
/// double quotes group.
fn args(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut quoted = false;
    let mut has = false;
    for c in line.chars() {
        match c {
            '"' => {
                quoted = !quoted;
                has = true;
            }
            c if !quoted && (c.is_whitespace() || matches!(c, ',' | '(' | ')')) => {
                if has {
                    out.push(std::mem::take(&mut cur));
                    has = false;
                }
            }
            c => {
                cur.push(c);
                has = true;
            }
        }
    }
    if has {
        out.push(cur);
    }
    out
}

/// `dbOpenFile`: a name with a `/` opens as is, from the IOC's directory;
/// a bare name is looked for along `EPICS_DB_INCLUDE_PATH`, or `.`.
fn open_db(file: &str, cwd: &Path, env: &HashMap<String, String>) -> Option<PathBuf> {
    if file.contains('/') {
        let path = cwd.join(file);
        return path.is_file().then_some(path);
    }
    let search = env.get("EPICS_DB_INCLUDE_PATH").map_or(".", String::as_str);
    search
        .split(':')
        .map(|d| cwd.join(if d.is_empty() { "." } else { d }).join(file))
        .find(|p| p.is_file())
}

/// Every `include` of `path`, transitively, opens through the search path.
fn check_includes(
    path: &Path,
    cwd: &Path,
    env: &HashMap<String, String>,
    seen: &mut HashSet<PathBuf>,
    missing: &mut Vec<String>,
) {
    if !seen.insert(path.to_path_buf()) {
        return;
    }
    let Ok(text) = std::fs::read_to_string(path) else {
        return;
    };
    for line in text.lines() {
        let line = line.trim_start();
        let Some(rest) = line.strip_prefix("include") else {
            continue;
        };
        let Some(inc) = rest
            .trim_start()
            .strip_prefix('"')
            .and_then(|r| r.split('"').next())
        else {
            continue;
        };
        match open_db(inc, cwd, env) {
            Some(found) => check_includes(&found, cwd, env, seen, missing),
            None => missing.push(format!("{} includes {inc:?}", path.display())),
        }
    }
}

/// `EPICS_DB_INCLUDE_PATH`'s sanctioned form: a colon list whose every
/// component is `$(MACRO)/db`.
fn is_canonical_include_path(value: &str) -> bool {
    !value.is_empty()
        && value.split(':').all(|c| {
            c.strip_prefix("$(")
                .and_then(|c| c.split_once(')'))
                .is_some_and(|(name, tail)| {
                    !name.is_empty()
                        && name
                            .chars()
                            .all(|ch| ch.is_ascii_uppercase() || ch.is_ascii_digit() || ch == '_')
                        && tail == "/db"
                })
        })
}

struct Run<'a> {
    root: &'a Path,
    failures: Vec<String>,
    checked: usize,
    visited: HashSet<PathBuf>,
    /// The startup script being run, which a fragment's line is run from.
    top: PathBuf,
}

impl Run<'_> {
    fn at(&self, script: &Path, lineno: usize) -> String {
        let rel = |p: &Path| p.strip_prefix(self.root).unwrap_or(p).display().to_string();
        let mut at = format!("{}:{}", rel(script), lineno + 1);
        if script != self.top {
            at += &format!(" (from {})", rel(&self.top));
        }
        at
    }

    /// Run `script` from `cwd` as iocsh does, checking each db load.
    fn script(&mut self, script: &Path, cwd: &Path, env: &mut HashMap<String, String>, depth: u32) {
        assert!(depth < 16, "{}: `<` nests too deep", script.display());
        let script = script
            .canonicalize()
            .unwrap_or_else(|_| script.to_path_buf());
        self.visited.insert(script.clone());
        let text = std::fs::read_to_string(&script).expect("readable .cmd");
        for (lineno, raw) in text.lines().enumerate() {
            let raw = raw.trim_start();
            if raw.is_empty() || raw.starts_with('#') {
                continue;
            }
            let line = expand(raw, env);
            if let Some(file) = line.strip_prefix('<') {
                let file = cwd.join(file.trim());
                if file.is_file() {
                    self.script(&file, cwd, env, depth + 1);
                } else {
                    let at = self.at(&script, lineno);
                    self.failures
                        .push(format!("{at}: `<` finds no {}", file.display()));
                }
                continue;
            }
            let words = args(&line);
            let Some(command) = words.first() else {
                continue;
            };
            match command.as_str() {
                "epicsEnvSet" if words.len() >= 3 => {
                    if words[1] == "EPICS_DB_INCLUDE_PATH" {
                        let declared = args(raw).get(2).cloned().unwrap_or_default();
                        if !is_canonical_include_path(&declared) {
                            let at = self.at(&script, lineno);
                            self.failures.push(format!(
                                "{at}: EPICS_DB_INCLUDE_PATH is not a colon list of \
                                 $(MACRO)/db components: {declared:?}"
                            ));
                        }
                    }
                    env.insert(words[1].clone(), words[2].clone());
                }
                "dbLoadRecords" | "dbLoadTemplate" => {
                    let written = args(raw).get(1).cloned().unwrap_or_default();
                    let file = words.get(1).cloned().unwrap_or_default();
                    let at = self.at(&script, lineno);
                    self.load(&at, command == "dbLoadTemplate", &written, &file, cwd, env);
                }
                _ => {}
            }
        }
    }

    fn load(
        &mut self,
        at: &str,
        template: bool,
        written: &str,
        file: &str,
        cwd: &Path,
        env: &HashMap<String, String>,
    ) {
        self.checked += 1;
        if written.contains("..") {
            self.failures
                .push(format!("{at}: path traversal in {written:?}"));
            return;
        }
        if written.contains('/') {
            let canonical = written
                .strip_prefix("$(")
                .and_then(|r| r.split_once(')'))
                .and_then(|(_, r)| r.strip_prefix("/db/"))
                .is_some_and(|f| !f.is_empty() && !f.contains('/'));
            if !canonical {
                self.failures
                    .push(format!("{at}: not $(MACRO)/db/<file>: {written:?}"));
                return;
            }
        } else if !env.contains_key("EPICS_DB_INCLUDE_PATH") {
            self.failures.push(format!(
                "{at}: bare {written:?} without EPICS_DB_INCLUDE_PATH opens from the cwd"
            ));
            return;
        }
        if file.contains("$(") || file.contains("${") {
            self.failures.push(format!(
                "{at}: {written:?} expands to {file:?}: macro undefined"
            ));
            return;
        }
        // dbLoadTemplate tries the name from the cwd before the path.
        let found = if template && cwd.join(file).is_file() {
            Some(cwd.join(file))
        } else {
            open_db(file, cwd, env)
        };
        let Some(found) = found else {
            self.failures
                .push(format!("{at}: {written:?} ({file}) does not open"));
            return;
        };
        if !template {
            let mut missing = Vec::new();
            check_includes(&found, cwd, env, &mut HashSet::new(), &mut missing);
            for m in missing {
                self.failures
                    .push(format!("{at}: {m}, which does not open"));
            }
        }
    }
}

#[test]
fn every_db_load_is_macro_form_and_resolvable() {
    let root = workspace_root();
    let packages = package_dirs(&root);
    let framework = framework_env(&packages);
    let mut cmd_files = Vec::new();
    walk(&root.join("iocs"), "cmd", &mut cmd_files);
    cmd_files.sort();
    assert!(!cmd_files.is_empty(), "no .cmd files found under iocs/");
    // Startup scripts first; a fragment is checked from the scripts that
    // include it, and on its own only when none does.
    let (scripts, fragments): (Vec<_>, Vec<_>) = cmd_files.into_iter().partition(|p| {
        p.file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("st"))
    });

    let mut run = Run {
        root: &root,
        failures: Vec::new(),
        checked: 0,
        visited: HashSet::new(),
        top: PathBuf::new(),
    };
    for cmd in scripts.iter().chain(&fragments) {
        let canonical = cmd.canonicalize().unwrap_or_else(|_| cmd.clone());
        if run.visited.contains(&canonical) {
            continue;
        }
        let ioc_dir = cmd.parent().expect("cmd file has a parent");
        let mut env = framework.clone();
        env.extend(ioc_env(ioc_dir, &packages));
        run.top = canonical;
        run.script(cmd, ioc_dir, &mut env, 0);
    }

    assert!(
        run.checked > 300,
        "suspiciously few db loads checked: {}",
        run.checked
    );
    assert!(
        run.failures.is_empty(),
        "non-canonical or unresolvable db loads:\n{}",
        run.failures.join("\n")
    );
}

#[test]
fn a_fragment_sees_the_including_scripts_macros() {
    let mut env = HashMap::from([("TEMPLATE".to_string(), "AH401B".to_string())]);
    assert_eq!(
        expand("$(QUADEM)/db/$(TEMPLATE).template", &env),
        "$(QUADEM)/db/AH401B.template",
        "an undefined macro stays as written"
    );
    env.insert("QUADEM".into(), "/q".into());
    assert_eq!(expand("${QUADEM}/$(X=d)", &env), "/q/d");
}

#[test]
fn a_bare_name_opens_through_the_include_path_only() {
    let dir = std::env::temp_dir().join(format!("db-paths-{}", std::process::id()));
    std::fs::create_dir_all(dir.join("db")).unwrap();
    std::fs::write(dir.join("db/a.template"), "include \"b.template\"\n").unwrap();
    let mut env = HashMap::new();
    assert_eq!(open_db("a.template", &dir, &env), None);
    env.insert("EPICS_DB_INCLUDE_PATH".to_string(), "x:db".to_string());
    let found = open_db("a.template", &dir, &env).expect("found along the path");
    let mut missing = Vec::new();
    check_includes(&found, &dir, &env, &mut HashSet::new(), &mut missing);
    assert_eq!(missing.len(), 1, "{missing:?}");
    std::fs::write(dir.join("db/b.template"), "").unwrap();
    missing.clear();
    check_includes(&found, &dir, &env, &mut HashSet::new(), &mut missing);
    assert!(missing.is_empty(), "{missing:?}");
    std::fs::remove_dir_all(&dir).unwrap();
}
