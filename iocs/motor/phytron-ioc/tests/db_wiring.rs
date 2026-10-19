//! st.cmd and st.mcc.cmd load the C module's aux dbs the way C's iocPhytron
//! does, and every record they load reaches a port the script creates, a
//! parameter that port creates, and a record the script loads.

use std::collections::{BTreeSet, HashMap};
use std::path::{Path, PathBuf};

use epics_rs::asyn::interfaces::InterfaceMask;
use epics_rs::asyn::port::{PortDriverBase, PortFlags};
use epics_rs::base::server::db_loader::{DbLoadConfig, DbRecordDef, parse_db_file};
use motor_common::ControllerParams;
use motor_phytron::phytron::PhytronParams;

fn ioc_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).to_path_buf()
}

/// The ports a script creates, by the driver behind them.
enum Port {
    Controller,
    Io,
}

struct Script {
    loaded: BTreeSet<String>,
    ports: HashMap<String, Port>,
    records: Vec<DbRecordDef>,
}

/// The `"..."` arguments of an iocsh call.
fn args(line: &str) -> Vec<String> {
    let inner = line.split_once('(').unwrap().1.rsplit_once(')').unwrap().0;
    inner
        .split('"')
        .skip(1)
        .step_by(2)
        .map(str::to_string)
        .collect()
}

fn expand(s: &str, env: &HashMap<String, String>) -> String {
    let mut out = s.to_string();
    for (k, v) in env {
        out = out.replace(&format!("$({k})"), v);
    }
    out
}

fn run(script: &str) -> Script {
    let text = std::fs::read_to_string(ioc_dir().join(script)).unwrap();
    let mut env = HashMap::from([("PHYTRON_IOC".to_string(), ioc_dir().display().to_string())]);
    let mut out = Script {
        loaded: BTreeSet::new(),
        ports: HashMap::new(),
        records: Vec::new(),
    };
    for line in text.lines().map(str::trim) {
        let Some((cmd, _)) = line.split_once('(') else {
            continue;
        };
        if cmd.starts_with('#') {
            continue;
        }
        let a: Vec<String> = args(line).iter().map(|s| expand(s, &env)).collect();
        match cmd {
            "epicsEnvSet" => {
                env.insert(a[0].clone(), a[1].clone());
            }
            "phytronCreateController" | "phytronCreateMCC" => {
                out.ports.insert(a[0].clone(), Port::Controller);
            }
            "phytronCreateAnalog" | "phytronCreateDigital" => {
                out.ports.insert(a[0].clone(), Port::Io);
            }
            "dbLoadRecords" => {
                let path = PathBuf::from(&a[0]);
                out.loaded
                    .insert(path.file_name().unwrap().to_string_lossy().into_owned());
                let macros: HashMap<String, String> = a[1]
                    .split(',')
                    .map(|kv| kv.split_once('=').unwrap())
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect();
                out.records.extend(
                    parse_db_file(&path, &macros, &DbLoadConfig::default())
                        .unwrap_or_else(|e| panic!("{script}: {}: {e}", path.display())),
                );
            }
            _ => {}
        }
    }
    out
}

/// The parameters phytronController creates.
fn controller_params() -> PortDriverBase {
    let mut base = PortDriverBase::new(
        "probe",
        1,
        PortFlags::default(),
        InterfaceMask::INT32,
        InterfaceMask::INT32,
    );
    PhytronParams::new(0.1, 1.0)
        .create_params(&mut base)
        .unwrap();
    base
}

/// phytronIO's parameters (C `g_aParameters`, phytronIOctrl.cpp:30-35) less
/// DIR, which only an MCC-1 creates (:73-79).
const IO_PARAMS: [&str; 2] = ["IN", "OUT"];

fn field(def: &DbRecordDef, name: &str) -> Option<String> {
    def.fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.value.as_str_lossy().into_owned())
}

/// Every `@asyn(port,...)drvInfo` names a port the script creates and a
/// parameter that port's driver creates.
fn asyn_links_resolve(script: &Script) {
    let controller = controller_params();
    let mut bad = Vec::new();
    for def in &script.records {
        for link in ["INP", "OUT"].iter().filter_map(|f| field(def, f)) {
            let Some(rest) = link.strip_prefix("@asyn(") else {
                continue;
            };
            let (inside, drv_info) = rest.split_once(')').unwrap();
            let port = inside.split(',').next().unwrap().trim();
            let found = match script.ports.get(port) {
                // The motor record's link carries no drvInfo.
                _ if field(def, "DTYP").as_deref() == Some("asynMotor") => true,
                Some(Port::Controller) => controller.find_param(drv_info).is_some(),
                Some(Port::Io) => IO_PARAMS.contains(&drv_info),
                None => false,
            };
            if !found {
                bad.push(format!("{}: {link}", def.name));
            }
        }
    }
    assert!(
        bad.is_empty(),
        "unresolved asyn links:\n  {}",
        bad.join("\n  ")
    );
}

/// Every database link names a record the script loads, except `dangling`.
fn db_links_resolve(script: &Script, dangling: &[&str]) {
    let names: BTreeSet<&[u8]> = script.records.iter().map(|d| d.name.as_bytes()).collect();
    let mut bad = Vec::new();
    for def in &script.records {
        for f in &def.fields {
            let n = f.name.as_str();
            let is_link = n == "FLNK"
                || n == "SDIS"
                || n == "SELL"
                || n.starts_with("INP")
                || n.starts_with("DOL")
                || n.starts_with("LNK")
                || n.starts_with("OUT");
            let value = f.value.as_str_lossy();
            let target = value.split_whitespace().next().unwrap_or("");
            if !is_link || target.is_empty() || target.starts_with('@') {
                continue;
            }
            if target.parse::<f64>().is_ok() {
                continue;
            }
            let record = target.split('.').next().unwrap();
            if !names.contains(record.as_bytes()) && !dangling.contains(&record) {
                bad.push(format!("{}.{n}: {value}", def.name));
            }
        }
    }
    assert!(
        bad.is_empty(),
        "links to no record:\n  {}",
        bad.join("\n  ")
    );
}

/// C's iocPhytron example (`st.cmd.phytron`, motor.substitutions.phytron):
/// the MCM01 controller status, an I1AM01 per axis, AIOM01 and DIOM01
/// channels and the direct command.
#[test]
fn st_cmd_loads_the_phymotion_aux_records() {
    let script = run("st.cmd");
    let want = [
        "phytron.template",
        "Phytron_MCM01.db",
        "Phytron_I1AM01.db",
        "Phytron_AIO.db",
        "Phytron_DIO.db",
        "Phytron_CMD.db",
    ];
    assert_eq!(script.loaded, want.map(String::from).into());
    asyn_links_resolve(&script);
    // I1AM01's REINIT_ follows MCM01's RESET_ by P (motor.substitutions.phytron).
    db_links_resolve(&script, &[]);
}

/// C's MCC example (`st.cmd.mcc`): an MCC axis, two analog inputs, the
/// digital channel and the direct command; it loads no MCM01, so the
/// axis REINIT_ link to the controller RESET_ dangles there too.
#[test]
fn st_mcc_cmd_loads_the_mcc_aux_records() {
    let script = run("st.mcc.cmd");
    let want = [
        "phytron.template",
        "Phytron_MCC.db",
        "Phytron_AIO_MCC.db",
        "Phytron_DIO.db",
        "Phytron_CMD.db",
    ];
    assert_eq!(script.loaded, want.map(String::from).into());
    asyn_links_resolve(&script);
    db_links_resolve(&script, &["phytron:-RESET_"]);
}

/// Every db the IOC ships is one a script loads, or Phytron_DIO_MCC1.db,
/// whose DIR only an MCC-1 creates.
#[test]
fn every_shipped_db_is_loaded() {
    let mut loaded = run("st.cmd").loaded;
    loaded.extend(run("st.mcc.cmd").loaded);
    loaded.insert("Phytron_DIO_MCC1.db".into());
    let shipped: BTreeSet<String> = std::fs::read_dir(ioc_dir().join("db"))
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(shipped, loaded);
}
