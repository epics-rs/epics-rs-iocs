//! The motorUtil and autosave lines of the startup scripts, in the order
//! C's iocSmarAct st.cmd and AutoSave.cmd give them, and the request files
//! their monitor sets name, resolved against the records the script loads.
//! Each script runs through the iocsh with every command but `epicsEnvSet`
//! recording the arguments it was given.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::autosave::macros::MacroContext;
use epics_rs::base::server::autosave::request::load_request_file_with_search_paths;
use epics_rs::base::server::autosave::startup::AutosaveStartupConfig;
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::db_loader::parse_db;
use epics_rs::base::server::iocsh::IocShell;
use epics_rs::base::server::iocsh::registry::{
    ArgDesc, ArgType, ArgValue, CommandContext, CommandDef, CommandOutcome,
};
use epics_rs::ca::server::ioc_app::IocApplication;

/// The lines the shell ran: the command, then its arguments.
static LINES: Mutex<Vec<Vec<String>>> = Mutex::new(Vec::new());
/// One script at a time: the command table and [`LINES`] are the process's.
static ONE_SCRIPT: Mutex<()> = Mutex::new(());

const SCRIPTS: [&str; 3] = ["st.cmd", "st.mcs.cmd", "st.scu.cmd"];

/// The commands [`pinned`] keeps, with the motorUtil.db and
/// save_restoreStatus.db loads.
const PINNED: [&str; 9] = [
    "epicsThreadSleep",
    "set_requestfile_path",
    "set_savefile_path",
    "set_pass0_restoreFile",
    "set_pass1_restoreFile",
    "save_restoreSet_status_prefix",
    "iocInit",
    "motorUtilInit",
    "create_monitor_set",
];

/// Every command `text` runs but `epicsEnvSet`, whose macros the others
/// are expanded with.
fn commands(text: &str) -> BTreeSet<String> {
    text.lines()
        .map(str::trim_start)
        .filter(|line| !line.starts_with('#'))
        .filter_map(|line| {
            line.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .next()
        })
        .filter(|name| !name.is_empty() && *name != "epicsEnvSet")
        .map(str::to_owned)
        .collect()
}

fn recorder(name: String) -> CommandDef {
    let args = vec![ArgDesc {
        name: "args",
        arg_type: ArgType::Argv,
    }];
    CommandDef::new(
        name.clone(),
        args,
        "",
        move |args: &[ArgValue], _: &CommandContext| {
            let mut line = vec![name.clone()];
            if let [ArgValue::Argv(argv)] = args {
                line.extend(argv.iter().map(|a| a.to_str_lossy().into_owned()));
            }
            LINES.lock().unwrap().push(line);
            Ok(CommandOutcome::Continue)
        },
    )
}

/// `script`'s lines as the shell ran them, under the IOC's `$(SMARACT_IOC)`,
/// `$(MOTOR)`, `$(ASYN)` and `$(AUTOSAVE)`.
async fn run(script: &str) -> Vec<Vec<String>> {
    // As main.rs sets them.
    epics_rs::base::runtime::env::set_default("SMARACT_IOC", env!("CARGO_MANIFEST_DIR"));
    let _ = epics_rs::motor::motor_record_factory();
    let _ = epics_rs::asyn::asyn_record::asyn_record_factory();
    let _ =
        IocApplication::new().autosave_startup(Arc::new(Mutex::new(AutosaveStartupConfig::new())));

    let path = format!("{}/{script}", env!("CARGO_MANIFEST_DIR"));
    let names = commands(&std::fs::read_to_string(&path).unwrap());
    let bridge = BlockingBridge::capture();
    let shell = std::thread::spawn(move || {
        let _one = ONE_SCRIPT.lock().unwrap_or_else(|e| e.into_inner());
        let shell = IocShell::new(Arc::new(PvDatabase::new()), bridge);
        for name in names {
            shell.register(recorder(name));
        }
        shell.execute_script(&path).unwrap();
        std::mem::take(&mut *LINES.lock().unwrap())
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    shell.join().unwrap()
}

/// The motorUtil, autosave and sleep lines of `lines`, in order, each
/// directory written back as its macro.
fn pinned(lines: &[Vec<String>]) -> Vec<String> {
    let dirs: Vec<(String, String)> = ["SMARACT_IOC", "MOTOR", "AUTOSAVE"]
        .map(|name| (std::env::var(name).unwrap(), format!("$({name})")))
        .into();
    let unexpand = |arg: &str| {
        let arg = dirs
            .iter()
            .find_map(|(dir, name)| {
                arg.strip_prefix(dir.as_str())
                    .map(|rest| name.clone() + rest)
            })
            .unwrap_or_else(|| arg.to_owned());
        format!("{arg:?}")
    };
    lines
        .iter()
        .filter(|line| match line[0].as_str() {
            "dbLoadRecords" => {
                line[1].ends_with("/motorUtil.db") || line[1].ends_with("/save_restoreStatus.db")
            }
            name => PINNED.contains(&name),
        })
        .map(|line| {
            let args = line[1..].iter().map(|arg| unexpand(arg));
            std::iter::once(line[0].clone())
                .chain(args)
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect()
}

/// Check every monitor set of `lines`: its request file resolves through
/// the request paths to PVs of records the script loads, with the same
/// fields of every motor, DVAL among them, and its .sav is one the script
/// restores and the only one. Returns the .sav names.
async fn check_requests(lines: &[Vec<String>]) -> BTreeSet<String> {
    let of = |command: &'static str| lines.iter().filter(move |line| line[0] == command);
    let search: Vec<PathBuf> = of("set_requestfile_path")
        .map(|line| PathBuf::from(&line[1]).join(&line[2]))
        .collect();
    let mut records = BTreeMap::new();
    for line in of("dbLoadRecords") {
        let db = std::fs::read_to_string(&line[1]).unwrap();
        for record in parse_db(&db, MacroContext::parse_defs(&line[2])).unwrap() {
            records.insert(record.name.to_str().unwrap().to_owned(), record.record_type);
        }
    }

    let mut saved = BTreeSet::new();
    let mut motor_fields: BTreeMap<&str, BTreeSet<String>> = BTreeMap::new();
    for line in of("create_monitor_set") {
        let (req, macros) = (&line[1], &line[3]);
        let list = load_request_file_with_search_paths(
            req,
            &search,
            &MacroContext::from_macrostring(macros),
        )
        .await
        .unwrap();
        assert!(!list.include_unopened, "{req}: an include is not found");
        assert!(!list.entries.is_empty(), "{req} names no PV");
        for entry in &list.entries {
            let (record, field) = entry.pv_name.split_once('.').unwrap();
            let Some((record, kind)) = records.get_key_value(record) else {
                panic!("{req}: {} is not a record the script loads", entry.pv_name);
            };
            if kind == "motor" {
                motor_fields
                    .entry(record)
                    .or_default()
                    .insert(field.to_owned());
            }
        }
        saved.insert(req.replace(".req", ".sav"));
    }

    let motors: Vec<&str> = records
        .iter()
        .filter(|(_, kind)| *kind == "motor")
        .map(|(name, _)| name.as_str())
        .collect();
    let first = motor_fields.get(motors[0]).expect(motors[0]);
    assert!(first.contains("DVAL"), "{first:?}");
    for motor in &motors {
        assert_eq!(motor_fields.get(motor), Some(first), "{motor}");
    }

    let restored: BTreeSet<String> = of("set_pass0_restoreFile")
        .chain(of("set_pass1_restoreFile"))
        .map(|line| line[1].clone())
        .collect();
    assert_eq!(restored, saved);
    saved
}

#[epics_rs::base::epics_test]
async fn st_cmd() {
    let lines = run("st.cmd").await;
    assert_eq!(
        pinned(&lines),
        [
            r#"dbLoadRecords "$(MOTOR)/db/motorUtil.db" "P=MCS2:""#,
            r#"set_requestfile_path "$(SMARACT_IOC)" """#,
            r#"set_requestfile_path "$(MOTOR)" "db""#,
            r#"set_savefile_path "$(SMARACT_IOC)" "autosave""#,
            r#"set_pass0_restoreFile "auto_positions.sav""#,
            r#"set_pass0_restoreFile "auto_settings_mcs2.sav""#,
            r#"set_pass1_restoreFile "auto_settings_mcs2.sav""#,
            r#"save_restoreSet_status_prefix "MCS2:""#,
            r#"dbLoadRecords "$(AUTOSAVE)/db/save_restoreStatus.db" "P=MCS2:""#,
            "iocInit",
            r#"motorUtilInit "MCS2:""#,
            r#"create_monitor_set "auto_positions.req" "5" "P=MCS2:""#,
            r#"create_monitor_set "auto_settings_mcs2.req" "30" "P=MCS2:""#,
        ]
    );
    check_requests(&lines).await;
}

#[epics_rs::base::epics_test]
async fn st_mcs_cmd() {
    let lines = run("st.mcs.cmd").await;
    assert_eq!(
        pinned(&lines),
        [
            r#"dbLoadRecords "$(MOTOR)/db/motorUtil.db" "P=MCS:""#,
            r#"set_requestfile_path "$(SMARACT_IOC)" """#,
            r#"set_requestfile_path "$(MOTOR)" "db""#,
            r#"set_savefile_path "$(SMARACT_IOC)" "autosave""#,
            r#"set_pass0_restoreFile "auto_positions_mcs.sav""#,
            r#"set_pass0_restoreFile "auto_settings_mcs.sav""#,
            r#"set_pass1_restoreFile "auto_settings_mcs.sav""#,
            r#"save_restoreSet_status_prefix "MCS:""#,
            r#"dbLoadRecords "$(AUTOSAVE)/db/save_restoreStatus.db" "P=MCS:""#,
            "iocInit",
            r#"motorUtilInit "MCS:""#,
            r#"create_monitor_set "auto_positions_mcs.req" "5" "P=MCS:""#,
            r#"create_monitor_set "auto_settings_mcs.req" "30" "P=MCS:""#,
        ]
    );
    check_requests(&lines).await;
}

#[epics_rs::base::epics_test]
async fn st_scu_cmd() {
    let lines = run("st.scu.cmd").await;
    assert_eq!(
        pinned(&lines),
        [
            r#"dbLoadRecords "$(MOTOR)/db/motorUtil.db" "P=SCU:""#,
            r#"epicsThreadSleep "1.0""#,
            r#"set_requestfile_path "$(SMARACT_IOC)" """#,
            r#"set_requestfile_path "$(MOTOR)" "db""#,
            r#"set_savefile_path "$(SMARACT_IOC)" "autosave""#,
            r#"set_pass0_restoreFile "auto_positions_scu.sav""#,
            r#"set_pass0_restoreFile "auto_settings_scu.sav""#,
            r#"set_pass1_restoreFile "auto_settings_scu.sav""#,
            r#"save_restoreSet_status_prefix "SCU:""#,
            r#"dbLoadRecords "$(AUTOSAVE)/db/save_restoreStatus.db" "P=SCU:""#,
            "iocInit",
            r#"motorUtilInit "SCU:""#,
            r#"create_monitor_set "auto_positions_scu.req" "5" "P=SCU:""#,
            r#"create_monitor_set "auto_settings_scu.req" "30" "P=SCU:""#,
        ]
    );
    check_requests(&lines).await;
}

/// The scripts save to one directory, so no two may share a .sav.
#[epics_rs::base::epics_test]
async fn no_two_scripts_share_a_save_file() {
    let mut owner = BTreeMap::new();
    for script in SCRIPTS {
        let lines = run(script).await;
        let dir = lines
            .iter()
            .find(|line| line[0] == "set_savefile_path")
            .map(|line| PathBuf::from(&line[1]).join(&line[2]))
            .unwrap();
        for sav in check_requests(&lines).await {
            if let Some(other) = owner.insert((dir.clone(), sav.clone()), script) {
                panic!("{script} and {other} both save {sav}");
            }
        }
    }
}
