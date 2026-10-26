//! Writes a C IOC's expansion, the `tests/*.db` an IOC's own tests compare
//! its field values with ([`ioc_conventions::assert_same_fields`]).
//!
//! ```text
//! c-expansion --dbl <dbl> --ioc <iocBoot dir> --out <expansion>.db
//!             [--db <Db dir>]... [--env NAME=VALUE]...
//!             [--swap FRAGMENT=SCRIPT]... <script>...
//! ```
//!
//! The scripts run in turn from the iocBoot directory, as C's iocsh would
//! with `--env` for the envPaths its build writes, each db file opened by
//! name from the source `--db` directories the build installs from, each
//! `< FRAGMENT` or substitutions file read as `SCRIPT` (a fragment skipped
//! when that is empty). The
//! names they load must be the `dbl` the C IOC listed, which is how the
//! expansion is known to load what the C IOC loaded.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use ioc_conventions::{assert_same_records, c_expansion, read_db, read_dbl};

fn main() {
    let mut args = std::env::args().skip(1);
    let (mut dbl, mut ioc, mut out) = (None, None, None);
    let (mut db_dirs, mut scripts) = (Vec::new(), Vec::new());
    let (mut env, mut swap) = (HashMap::new(), HashMap::new());
    while let Some(arg) = args.next() {
        let mut value = || args.next().unwrap_or_else(|| panic!("{arg} takes a value"));
        match arg.as_str() {
            "--dbl" => dbl = Some(PathBuf::from(value())),
            "--ioc" => ioc = Some(PathBuf::from(value())),
            "--out" => out = Some(PathBuf::from(value())),
            "--db" => db_dirs.push(PathBuf::from(value())),
            "--env" | "--swap" => {
                let pair = value();
                let (name, v) = pair.split_once('=').expect("NAME=VALUE");
                let map = if arg == "--env" { &mut env } else { &mut swap };
                map.insert(name.to_string(), v.to_string());
            }
            _ => scripts.push(PathBuf::from(arg)),
        }
    }
    let dbl = dbl.expect("--dbl");
    let ioc = ioc.expect("--ioc");
    let out = out.expect("--out");
    let scripts: Vec<PathBuf> = scripts.iter().map(|s| ioc.join(s)).collect();
    let swap_text = swap.clone();
    let (names, records, refused) = c_expansion(&scripts, &ioc, env, db_dirs, swap);
    assert_same_records(&names, &read_dbl(&dbl));

    let mut ran: Vec<String> = scripts
        .iter()
        .map(|s| s.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    let mut swaps: Vec<_> = swap_text.iter().collect();
    swaps.sort();
    ran.extend(swaps.into_iter().map(|(from, to)| match to.as_str() {
        "" => format!("no {from}"),
        to => {
            let to = Path::new(to).file_name().unwrap().to_string_lossy();
            format!("{to} for {from}")
        }
    }));
    let mut text = format!(
        "# c-expansion of {} over the C sources, the records of {}:\n",
        ran.join(", "),
        dbl.file_name().unwrap().to_string_lossy()
    );
    let header = std::fs::read_to_string(&dbl).expect("readable dbl");
    for line in header.lines().take_while(|l| l.starts_with('#')) {
        text += &format!("{line}\n");
    }
    let mut refused = refused;
    // One load refused per substitutions row is one refusal.
    refused.dedup();
    for load in refused {
        text += &format!("# C refuses {load}\n");
    }
    for (name, record) in &records {
        text += &format!("record({}, \"{}\") {{\n", record.rtyp, escape(name));
        for (field, value) in &record.fields {
            text += &format!("    field({field}, \"{}\")\n", escape(value));
        }
        text += "}\n";
    }
    std::fs::write(&out, text).unwrap_or_else(|e| panic!("{}: {e}", out.display()));
    assert!(
        read_db(&out) == records,
        "{} reads back other values",
        out.display()
    );
}

/// A value as a .db string: `"` and `\` escaped.
fn escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}
