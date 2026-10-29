//! The Alarm1-4 mbbi records load with THSV=MAJOR and no refusal. Upstream
//! writes "MAJR", which is not a menuAlarmSevr choice; C refuses it at load
//! (dbLexRoutines.c:1410) and leaves THSV at NO_ALARM.

#![cfg(unix)]

use std::sync::Arc;

use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::IocShell;
use epics_rs::base::server::record::{DbLinkType, DevSup, Dset, register_device_menu};
use epics_rs::base::types::EpicsValue;
use yokogawa_mw100::device_support::DTYP;

/// The bare shell has no `mw100Support.dbd`; give the template's record
/// types the DTYP choice so a DTYP refusal cannot hide in the capture.
static ROW: &[DevSup] = &[DevSup {
    name: "devMW100_test",
    choice: DTYP,
    link_type: DbLinkType::InstIo,
    dset: Dset::new(5, &[], false),
}];

/// Run iocsh `line` with fd 2 pointed at a file and return what it wrote.
fn stderr_of(line: &str, db: Arc<PvDatabase>) -> String {
    let sink = tempfile::NamedTempFile::new().expect("capture file");
    let saved = unsafe { libc::dup(2) };
    assert!(saved >= 0, "dup(2) failed");
    let fd = std::os::fd::AsRawFd::as_raw_fd(sink.as_file());
    assert!(unsafe { libc::dup2(fd, 2) } >= 0, "dup2 onto fd 2 failed");
    let bridge = epics_rs::base::runtime::task::BlockingBridge::capture();
    let owned = line.to_string();
    let ran = std::thread::spawn(move || {
        epics_rs::base::runtime::task::set_thread_ok_to_block(true);
        IocShell::new(db, bridge).execute_line(&owned)
    })
    .join();
    assert!(unsafe { libc::dup2(saved, 2) } >= 0, "restore fd 2 failed");
    unsafe { libc::close(saved) };
    ran.expect("the shell thread").ok();
    std::fs::read_to_string(sink.path()).expect("read the capture")
}

#[epics_rs::base::epics_test]
async fn alarm_mbbis_load_with_thsv_major() {
    for rtyp in ["bi", "longin", "mbbi", "stringout"] {
        register_device_menu(rtyp, ROW);
    }
    let dir = env!("CARGO_MANIFEST_DIR");
    for (template, address) in [("mw100_mx114_channel", 11), ("mw100_mx115_channel", 21)] {
        let db = Arc::new(PvDatabase::new());
        let got = stderr_of(
            &format!(
                "dbLoadRecords(\"{dir}/db/{template}.template\", \
                 \"P=t:,DAU=d,HANDLE=h,ADDRESS={address}\")"
            ),
            db.clone(),
        );
        assert_eq!(got, "", "{template} must load with nothing on stderr");
        for n in 1..=4 {
            let pv = format!("t:d:{address}:Alarm{n}.THSV");
            let thsv = db.get_pv_blocking(&pv).expect(&pv);
            assert_eq!(
                thsv,
                EpicsValue::Short(2),
                "{pv} must read MAJOR (menuAlarmSevr 2), got {thsv:?}"
            );
        }
    }
}
