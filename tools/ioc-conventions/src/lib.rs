//! Workspace conventions enforced by test, not by review.
//!
//! `tests/db_paths.rs` holds the single rule for EPICS database loading:
//! every `dbLoadRecords`/`dbLoadTemplate` path in `iocs/**/*.cmd` is
//! `$(MACRO)/db/<file>`, the macro has a `set_default` or `set_default_path`
//! in the owning IOC's Rust sources, and the file exists where the macro
//! points.
//!
//! `tests/cfloat.rs` runs epics-rs's `cfloat-gate` over every member: a
//! precision or exponent float placeholder formats a `CFloat`, C's spelling.
//!
//! The startup-script walker those rules run on is this library, so an
//! IOC's own tests can ask it which records a script loads
//! ([`script_records`]), or which its IOC binary makes running it
//! ([`booted_records`]), and compare them with the C IOC's `dbl`
//! ([`read_dbl`], [`renamed`], [`assert_same_records`]).
//!
//! A script is run as iocsh runs it, from its own directory: the IOC's
//! `set_default`/`set_default_path` macros and the framework's envPaths
//! (`$(ADCORE)`, `$(ASYN)`, `$(AUTOSAVE)`, ... as the pinned epics-rs
//! exports them) first, then every line macro-expanded in order,
//! `epicsEnvSet` updating the environment and `<` running the fragment with
//! it, so a fragment's `$(TEMPLATE)` is the including st.cmd's. A load then
//! opens its file as `dbOpenFile` does: a path with a `/` as is, a bare name
//! through `EPICS_DB_INCLUDE_PATH`, and every template `include` through
//! that path too.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};

use epics_rs::base::runtime::mac_lib::macro_defn_pairs_bytes;
use epics_rs::base::server::db_loader::{
    parse_db_file, parse_substitutions, DbLoadConfig, MacroDefs, SubstitutionEvent,
};

/// envPaths macros the framework sets, and where each points in the crate
/// that sets it.
const FRAMEWORK: &[(&str, &str, &str)] = &[
    ("ADCORE", "ad-core-rs", ""),
    ("ASYN", "asyn-rs", ""),
    ("AUTOSAVE", "epics-base-rs", "autosave"),
    ("CALC", "epics-base-rs", "calc"),
    ("MOTOR", "motor-rs", ""),
];

/// The crates whose sources export [`FRAMEWORK`] macros.
const FRAMEWORK_CRATES: &[&str] = &["ad-plugins-rs", "asyn-rs", "epics-base-rs", "motor-rs"];

/// Macros an IOC sets from a dependency crate's exported const.
const EXTERNAL: &[(&str, &str)] = &[("MCA", "mca"), ("SCALER", "scaler-rs")];

pub fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("tools/ioc-conventions sits two levels below the root")
        .to_path_buf()
}

pub fn walk(dir: &Path, ext: &str, out: &mut Vec<PathBuf>) {
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
pub fn package_dirs(root: &Path) -> HashMap<String, PathBuf> {
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

/// `set_default("NAME", ...)` and `set_default_path("NAME", ...)` calls in
/// the `.rs` files under `dir`, with the text after the name. epics-rs
/// exports its envPaths through `set_default_path`, the `set_default` for a
/// directory; an IOC may call either.
fn set_defaults(dir: &Path) -> Vec<(String, String)> {
    let mut rs_files = Vec::new();
    walk(dir, "rs", &mut rs_files);
    let mut found = Vec::new();
    for rs in rs_files {
        let text = std::fs::read_to_string(&rs).expect("readable source");
        found.extend(set_default_calls(&text));
    }
    found
}

pub fn set_default_calls(text: &str) -> Vec<(String, String)> {
    let mut found = Vec::new();
    let mut rest = text;
    while let Some(pos) = rest.find("set_default") {
        rest = &rest[pos + "set_default".len()..];
        let call = rest.strip_prefix("_path").unwrap_or(rest);
        let Some(call) = call.strip_prefix('(') else {
            continue;
        };
        // rustfmt may wrap the call across lines.
        rest = call.trim_start();
        let Some(stripped) = rest.strip_prefix('"') else {
            continue;
        };
        rest = stripped;
        let Some(end) = rest.find('"') else { break };
        let tail = &rest[end..rest.len().min(end + 200)];
        found.push((rest[..end].to_string(), tail.to_string()));
    }
    found
}

/// The envPaths macros the pinned framework exports.
pub fn framework_env(packages: &HashMap<String, PathBuf>) -> HashMap<String, String> {
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

/// The IOC's own `set_default`/`set_default_path` macros: its crate dir,
/// its family dir, or a dependency crate's dir.
pub fn ioc_env(ioc_dir: &Path, packages: &HashMap<String, PathBuf>) -> HashMap<String, String> {
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
pub fn expand(text: &str, env: &HashMap<String, String>) -> String {
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
pub fn args(line: &str) -> Vec<String> {
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

/// `EPICS_DB_INCLUDE_PATH`, or `.`, as directories under `cwd`.
fn include_path(cwd: &Path, env: &HashMap<String, String>) -> Vec<PathBuf> {
    let search = env.get("EPICS_DB_INCLUDE_PATH").map_or(".", String::as_str);
    search
        .split(':')
        .map(|d| cwd.join(if d.is_empty() { "." } else { d }))
        .collect()
}

/// `dbOpenFile`: a name with a `/` opens as is, from the IOC's directory;
/// a bare name is looked for along `EPICS_DB_INCLUDE_PATH`, or `.`.
pub fn open_db(file: &str, cwd: &Path, env: &HashMap<String, String>) -> Option<PathBuf> {
    if file.contains('/') {
        let path = cwd.join(file);
        return path.is_file().then_some(path);
    }
    include_path(cwd, env)
        .into_iter()
        .map(|d| d.join(file))
        .find(|p| p.is_file())
}

/// Every `include` of `path`, transitively, opens through the search path.
pub fn check_includes(
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
pub fn is_canonical_include_path(value: &str) -> bool {
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

/// C `macParseDefns` of a load's macro argument; a name without a value
/// defines nothing.
fn macro_args(text: &str) -> HashMap<Vec<u8>, Vec<u8>> {
    macro_defn_pairs_bytes(text.as_bytes())
        .into_iter()
        .filter(|(name, _)| !name.is_empty())
        .filter_map(|(name, value)| Some((name, value?)))
        .collect()
}

pub struct Run<'a> {
    pub root: &'a Path,
    pub failures: Vec<String>,
    pub checked: usize,
    pub visited: HashSet<PathBuf>,
    /// The startup script being run, which a fragment's line is run from.
    pub top: PathBuf,
    /// When set, every load is parsed and the record names it creates,
    /// aliases included as C `dbl` lists them, are kept here.
    pub records: Option<BTreeSet<String>>,
    /// When set, every `epicsEnvSet`, `dbLoadRecords` and `dbLoadTemplate`
    /// line is kept here as written, each `<` fragment's in its place.
    pub lines: Option<Vec<String>>,
}

impl<'a> Run<'a> {
    pub fn new(root: &'a Path) -> Self {
        Run {
            root,
            failures: Vec::new(),
            checked: 0,
            visited: HashSet::new(),
            top: PathBuf::new(),
            records: None,
            lines: None,
        }
    }

    fn keep(&mut self, raw: &str) {
        if let Some(lines) = self.lines.as_mut() {
            lines.push(raw.to_string());
        }
    }

    fn at(&self, script: &Path, lineno: usize) -> String {
        let rel = |p: &Path| p.strip_prefix(self.root).unwrap_or(p).display().to_string();
        let mut at = format!("{}:{}", rel(script), lineno + 1);
        if script != self.top {
            at += &format!(" (from {})", rel(&self.top));
        }
        at
    }

    /// Run `script` from `cwd` as iocsh does, checking each db load.
    pub fn script(
        &mut self,
        script: &Path,
        cwd: &Path,
        env: &mut HashMap<String, String>,
        depth: u32,
    ) {
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
                    self.keep(raw);
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
                    self.keep(raw);
                    let written = args(raw).get(1).cloned().unwrap_or_default();
                    let file = words.get(1).cloned().unwrap_or_default();
                    let macros = words.get(2).cloned().unwrap_or_default();
                    let at = self.at(&script, lineno);
                    let template = command == "dbLoadTemplate";
                    if let Some(found) = self.load(&at, template, &written, &file, cwd, env) {
                        if template {
                            self.collect_template(&at, &found, &macros, cwd, env);
                        } else {
                            self.collect(&at, &found, macro_args(&macros), cwd, env);
                        }
                    }
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
    ) -> Option<PathBuf> {
        self.checked += 1;
        if written.contains("..") {
            self.failures
                .push(format!("{at}: path traversal in {written:?}"));
            return None;
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
                return None;
            }
        } else if !env.contains_key("EPICS_DB_INCLUDE_PATH") {
            self.failures.push(format!(
                "{at}: bare {written:?} without EPICS_DB_INCLUDE_PATH opens from the cwd"
            ));
            return None;
        }
        if file.contains("$(") || file.contains("${") {
            self.failures.push(format!(
                "{at}: {written:?} expands to {file:?}: macro undefined"
            ));
            return None;
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
            return None;
        };
        if !template {
            let mut missing = Vec::new();
            check_includes(&found, cwd, env, &mut HashSet::new(), &mut missing);
            for m in missing {
                self.failures
                    .push(format!("{at}: {m}, which does not open"));
            }
        }
        Some(found)
    }

    /// One `dbLoadRecords` of `path`, parsed as the IOC parses it.
    fn collect(
        &mut self,
        at: &str,
        path: &Path,
        macros: HashMap<Vec<u8>, Vec<u8>>,
        cwd: &Path,
        env: &HashMap<String, String>,
    ) {
        let Some(records) = self.records.as_mut() else {
            return;
        };
        let config = DbLoadConfig {
            include_paths: include_path(cwd, env),
            ..DbLoadConfig::default()
        };
        match parse_db_file(path, MacroDefs::from_byte_map(&macros), &config) {
            Ok(defs) => {
                for def in defs {
                    let names =
                        std::iter::once(&def.name).chain(def.aliases.iter().map(|a| &a.name));
                    for name in names {
                        let name = name.to_str_lossy().into_owned();
                        if name.contains("$(") || name.contains("${") {
                            self.failures.push(format!(
                                "{at}: {} names {name:?}: macro undefined",
                                path.display()
                            ));
                        }
                        records.insert(name);
                    }
                }
            }
            Err(e) => self
                .failures
                .push(format!("{at}: {} does not parse: {e}", path.display())),
        }
    }

    /// `dbLoadTemplate`: one `dbLoadRecords` per row, the row's values over
    /// the globals over the command's own macros (`dbLoadTemplate.y:301-323`,
    /// `:387-389`), the template name env-expanded as `dbReadCOM` does.
    fn collect_template(
        &mut self,
        at: &str,
        subs: &Path,
        macros: &str,
        cwd: &Path,
        env: &HashMap<String, String>,
    ) {
        if self.records.is_none() {
            return;
        }
        let text = std::fs::read(subs).expect("readable substitutions");
        let parsed = parse_substitutions(&text);
        for event in parsed.events {
            let load = match event {
                SubstitutionEvent::Load(load) => load,
                SubstitutionEvent::Fault(fault) => {
                    self.failures
                        .push(format!("{at}: {}: {fault:?}", subs.display()));
                    continue;
                }
                SubstitutionEvent::Notice(_) => continue,
            };
            let file = expand(&load.file, env);
            let Some(found) = open_db(&file, cwd, env) else {
                self.failures.push(format!(
                    "{at}: {} line {} names {file:?}, which does not open",
                    subs.display(),
                    load.line
                ));
                continue;
            };
            let mut merged = macro_args(macros);
            merged.extend(load.macros.into_iter().map(|m| (m.name, m.value)));
            self.collect(at, &found, merged, cwd, env);
        }
        if let Some(fault) = parsed.stopped {
            self.failures
                .push(format!("{at}: {} stops: {fault:?}", subs.display()));
        }
    }
}

/// Run `script` from its own directory with its IOC's macros as the IOC
/// runs it. Panics naming every load that fails.
fn run_script(run: &mut Run, script: &Path) {
    let packages = package_dirs(run.root);
    let ioc_dir = script.parent().expect("script has a parent");
    let mut env = framework_env(&packages);
    env.extend(ioc_env(ioc_dir, &packages));
    run.top = script.canonicalize().unwrap_or_else(|_| script.into());
    run.script(script, ioc_dir, &mut env, 0);
    assert!(
        run.failures.is_empty(),
        "{} loads:\n{}",
        script.display(),
        run.failures.join("\n")
    );
}

/// The records `script` loads: every name C `dbl` would list, aliases
/// included.
pub fn script_records(script: &Path) -> BTreeSet<String> {
    let root = workspace_root();
    let mut run = Run::new(&root);
    run.records = Some(BTreeSet::new());
    run_script(&mut run, script);
    run.records.unwrap_or_default()
}

/// The IOC's servers on loopback, beaconing and searching only there. PVA
/// binds at startup, before the script runs, and takes port 0 as the
/// kernel's choice; CA refuses port 0 (`envGetInetPortConfigParam`), so it
/// is given a free one, though its server starts only at `iocInit`, which a
/// booted run never reaches.
const LOOPBACK_ENV: &[(&str, &str)] = &[
    ("EPICS_PVAS_INTF_ADDR_LIST", "127.0.0.1"),
    ("EPICS_PVAS_SERVER_PORT", "0"),
    ("EPICS_PVAS_BROADCAST_PORT", "0"),
    ("EPICS_PVAS_BEACON_ADDR_LIST", "127.0.0.1"),
    ("EPICS_PVAS_AUTO_BEACON_ADDR_LIST", "NO"),
    ("EPICS_PVA_ADDR_LIST", "127.0.0.1"),
    ("EPICS_PVA_AUTO_ADDR_LIST", "NO"),
    ("EPICS_CAS_INTF_ADDR_LIST", "127.0.0.1"),
    ("EPICS_CAS_BEACON_ADDR_LIST", "127.0.0.1"),
    ("EPICS_CAS_AUTO_BEACON_ADDR_LIST", "NO"),
    ("EPICS_CA_ADDR_LIST", "127.0.0.1"),
    ("EPICS_CA_AUTO_ADDR_LIST", "NO"),
];

/// The records `binary`, the IOC, makes running `script`: its
/// `epicsEnvSet`, `dbLoadRecords` and `dbLoadTemplate` lines, each `<`
/// fragment's in its place, run by the IOC from the script's directory,
/// then `dbl`. Driver configuration needs the hardware, so it is left out,
/// as it was where the C `dbl` was captured. Panics on a failed walk
/// ([`script_records`]), a failed exit, or any line on the IOC's stderr,
/// where every load refusal goes.
pub fn booted_records(binary: &Path, script: &Path) -> BTreeSet<String> {
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    let root = workspace_root();
    let mut run = Run::new(&root);
    run.lines = Some(Vec::new());
    run_script(&mut run, script);
    let tmp = std::env::temp_dir().join(format!(
        "ioc-conventions-{}-{}",
        std::process::id(),
        RUNS.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&tmp).expect("temp dir");
    let (cmd, dbl) = (tmp.join("booted.cmd"), tmp.join("booted.dbl"));
    let mut lines = run.lines.unwrap_or_default();
    lines.push(format!("dbl > {}\n", dbl.display()));
    std::fs::write(&cmd, lines.join("\n")).expect("write the booted script");
    let ca_port = TcpListener::bind("127.0.0.1:0")
        .and_then(|l| l.local_addr())
        .expect("a free loopback port")
        .port()
        .to_string();
    let out = Command::new(binary)
        .arg(&cmd)
        .current_dir(script.parent().expect("script has a parent"))
        .envs(LOOPBACK_ENV.iter().copied())
        .env("EPICS_CA_SERVER_PORT", &ca_port)
        .env("EPICS_CAS_SERVER_PORT", &ca_port)
        .stdin(Stdio::null())
        .output()
        .unwrap_or_else(|e| panic!("{}: {e}", binary.display()));
    let records = dbl.is_file().then(|| read_dbl(&dbl));
    std::fs::remove_dir_all(&tmp).expect("remove the temp dir");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success() && stderr.trim().is_empty(),
        "{} running {} exits {}:\n{stderr}",
        binary.display(),
        script.display(),
        out.status
    );
    records.unwrap_or_else(|| panic!("{} wrote no dbl", binary.display()))
}

/// A C IOC's `dbl` output: one record name per line. `#` lines say where
/// it was captured.
pub fn read_dbl(path: &Path) -> BTreeSet<String> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(str::to_string)
        .collect()
}

/// Whether `name` starts with the record-name prefix `prefix`. A prefix
/// ending in a digit takes no further digit, so `m1` is not `m10`'s.
fn has_prefix(name: &str, prefix: &str) -> bool {
    name.strip_prefix(prefix).is_some_and(|rest| {
        !(prefix.ends_with(|c: char| c.is_ascii_digit())
            && rest.starts_with(|c: char| c.is_ascii_digit()))
    })
}

/// C's record names as an rs st.cmd names them. Each name takes the
/// longest prefix that matches it from `renames` (C prefix, rs prefix) or
/// `drops`; a drop (a C axis or record the rs script does not configure)
/// removes the name, a rename replaces the prefix, and a name no prefix
/// matches is kept as it is.
pub fn renamed(c: &BTreeSet<String>, renames: &[(&str, &str)], drops: &[&str]) -> BTreeSet<String> {
    c.iter()
        .filter_map(|name| {
            let rename = renames
                .iter()
                .filter(|(from, _)| has_prefix(name, from))
                .max_by_key(|(from, _)| from.len());
            let drop = drops
                .iter()
                .filter(|from| has_prefix(name, from))
                .max_by_key(|from| from.len());
            match (rename, drop) {
                (_, Some(d)) if rename.is_none_or(|(from, _)| d.len() >= from.len()) => None,
                (Some((from, to)), _) => Some(format!("{to}{}", &name[from.len()..])),
                _ => Some(name.clone()),
            }
        })
        .collect()
}

/// `rs` and `c` hold the same names; the panic lists each side's extras.
pub fn assert_same_records(rs: &BTreeSet<String>, c: &BTreeSet<String>) {
    let only_rs: Vec<_> = rs.difference(c).map(String::as_str).collect();
    let only_c: Vec<_> = c.difference(rs).map(String::as_str).collect();
    assert!(
        only_rs.is_empty() && only_c.is_empty(),
        "record sets differ\nonly in rs ({}):\n  {}\nonly in C ({}):\n  {}",
        only_rs.len(),
        only_rs.join("\n  "),
        only_c.len(),
        only_c.join("\n  ")
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(list: &[&str]) -> BTreeSet<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn a_digit_prefix_does_not_take_a_longer_axis() {
        let c = names(&["n:m1", "n:m1_able", "n:m10", "n:m2", "n:allstop"]);
        let rs = renamed(&c, &[("n:", "e:"), ("n:m1", "e:m3")], &["n:m2"]);
        assert_eq!(rs, names(&["e:m3", "e:m3_able", "e:m10", "e:allstop"]));
    }

    #[test]
    fn the_longer_of_a_drop_and_a_rename_wins() {
        let c = names(&["n:m1", "n:m1_able", "n:x"]);
        assert_eq!(
            renamed(&c, &[("n:m1", "e:m1")], &["n:"]),
            names(&["e:m1", "e:m1_able"])
        );
        assert_eq!(renamed(&c, &[("n:", "e:")], &["n:m1"]), names(&["e:x"]));
    }

    #[test]
    fn a_booted_script_keeps_the_record_and_macro_lines_in_order() {
        let dir = std::env::temp_dir().join(format!("ioc-conv-lines-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("db")).unwrap();
        std::fs::write(dir.join("db/m.db"), "record(ao, \"$(P)x\") {}\n").unwrap();
        std::fs::write(
            dir.join("frag.cmd"),
            "drvAsynIPPortConfigure(\"L0\", \"h:1\")\n\
             dbLoadRecords(\"$(TOP)/db/m.db\", \"P=$(P)\")\n",
        )
        .unwrap();
        std::fs::write(
            dir.join("st.cmd"),
            "epicsEnvSet(\"P\", \"c:\")\n\
             < frag.cmd\n\
             iocInit()\n",
        )
        .unwrap();
        let root = dir.clone();
        let mut run = Run::new(&root);
        run.lines = Some(Vec::new());
        let mut env = HashMap::from([("TOP".to_string(), dir.to_str().unwrap().to_string())]);
        run.script(&dir.join("st.cmd"), &dir, &mut env, 0);
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(run.failures.is_empty(), "{:?}", run.failures);
        assert_eq!(
            run.lines.unwrap(),
            [
                "epicsEnvSet(\"P\", \"c:\")",
                "dbLoadRecords(\"$(TOP)/db/m.db\", \"P=$(P)\")"
            ]
        );
    }

    #[test]
    fn a_template_row_overrides_globals_over_the_commands_macros() {
        let dir = std::env::temp_dir().join(format!("ioc-conv-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("db")).unwrap();
        std::fs::write(
            dir.join("db/m.db"),
            "record(motor, \"$(P)$(M)\") { alias(\"$(P)$(M)_a\") }\n\
             record(ao, \"$(P)$(M)_$(S=s)\") {}\n",
        )
        .unwrap();
        std::fs::write(
            dir.join("db/m.substitutions"),
            "global { S = g }\n\
             file \"$(TOP)/db/m.db\" {\n\
             pattern { M, S }\n\
             { m1, r }\n\
             { m2 }\n}\n",
        )
        .unwrap();
        std::fs::write(
            dir.join("st.cmd"),
            "dbLoadRecords(\"$(TOP)/db/m.db\", \"P=c:,M=x\")\n\
             dbLoadTemplate(\"$(TOP)/db/m.substitutions\", \"P=c:,S=cmd\")\n",
        )
        .unwrap();
        let root = dir.clone();
        let mut run = Run::new(&root);
        run.records = Some(BTreeSet::new());
        let mut env = HashMap::from([("TOP".to_string(), dir.to_str().unwrap().to_string())]);
        run.script(&dir.join("st.cmd"), &dir, &mut env, 0);
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(run.failures.is_empty(), "{:?}", run.failures);
        assert_eq!(
            run.records.unwrap(),
            names(&[
                "c:x", "c:x_a", "c:x_s", "c:m1", "c:m1_a", "c:m1_r", "c:m2", "c:m2_a", "c:m2_g"
            ])
        );
    }
}
