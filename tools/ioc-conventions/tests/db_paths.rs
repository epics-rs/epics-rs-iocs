//! Every db load in `iocs/**/*.cmd` resolves the way the IOC resolves it,
//! and is written in the one canonical form.
//!
//! Each startup script is run by the crate's walker ([`Run`]), as iocsh
//! runs it from its own directory.
//!
//! Form: a path is `$(MACRO)/db/<file>` (no cwd-relative, no
//! repo-root-relative, no `..` traversal) or a bare name found through a
//! declared `EPICS_DB_INCLUDE_PATH`.

use std::collections::{HashMap, HashSet};

use epics_rs::base::server::db_loader::PATH_LIST_SEPARATOR;

use ioc_conventions::{
    check_includes, expand, framework_env, ioc_env, open_db, package_dirs, set_default_calls, walk,
    workspace_root, Run,
};

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

    let mut run = Run::new(&root);
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
fn both_setters_default_a_macro() {
    let names: Vec<_> = set_default_calls(
        "env::set_default(\"A\", x);\n\
         env::set_default_path(\n    \"B\",\n    y,\n);\n\
         tracing::subscriber::set_default(subscriber);\n\
         set_defaults(&dir);\n",
    )
    .into_iter()
    .map(|(name, _)| name)
    .collect();
    assert_eq!(names, ["A", "B"]);
}

#[test]
fn a_bare_name_opens_through_the_include_path_only() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().to_path_buf();
    std::fs::create_dir_all(dir.join("db")).unwrap();
    std::fs::write(dir.join("db/a.template"), "include \"b.template\"\n").unwrap();
    let mut env = HashMap::new();
    assert_eq!(open_db("a.template", &dir, &env), None);
    env.insert(
        "EPICS_DB_INCLUDE_PATH".to_string(),
        format!("x{PATH_LIST_SEPARATOR}db"),
    );
    let found = open_db("a.template", &dir, &env).expect("found along the path");
    let mut missing = Vec::new();
    check_includes(&found, &dir, &env, &mut HashSet::new(), &mut missing);
    assert_eq!(missing.len(), 1, "{missing:?}");
    std::fs::write(dir.join("db/b.template"), "").unwrap();
    missing.clear();
    check_includes(&found, &dir, &env, &mut HashSet::new(), &mut missing);
    assert!(missing.is_empty(), "{missing:?}");
}
