//! The two strings opcua_init's banner takes from the build: the
//! `EPICS_OPCUA_GIT_COMMIT` devOpcuaSup/Makefile:31-35 sets, and the
//! client library opcuaGetDriverName names (open62541/Session.cpp:212-223).

use std::path::Path;
use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn main() {
    let commit = if git(&["rev-parse", "--is-inside-work-tree"]).as_deref() == Some("true") {
        // A commit or checkout rewrites one of these.
        for path in ["HEAD", "index"] {
            if let Some(p) = git(&["rev-parse", "--git-path", path]) {
                println!("cargo:rerun-if-changed={p}");
            }
        }
        git(&["describe", "--long", "--dirty"]).unwrap_or_default()
    } else {
        "-".to_string()
    };
    println!("cargo:rustc-env=OPCUA_GIT_COMMIT={commit}");

    // The resolved async-opcua, from the lock of the workspace building us.
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let lock = Path::new(&manifest)
        .ancestors()
        .map(|dir| dir.join("Cargo.lock"))
        .find(|lock| lock.exists());
    let version = lock.and_then(|lock| {
        println!("cargo:rerun-if-changed={}", lock.display());
        let text = std::fs::read_to_string(lock).ok()?;
        let mut lines = text.lines();
        lines.find(|l| *l == "name = \"async-opcua\"")?;
        let v = lines
            .next()?
            .strip_prefix("version = \"")?
            .strip_suffix('"')?;
        Some(format!(" v{v}"))
    });
    println!(
        "cargo:rustc-env=OPCUA_CLIENT_API=async-opcua Client API{}",
        version.unwrap_or_default()
    );
}
