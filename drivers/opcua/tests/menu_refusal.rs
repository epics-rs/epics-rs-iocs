//! A `.db` label outside an `opcuaItem` menu is refused with the menu's
//! name, as `dbPutString` prints it (`dbMsgPrint(pdbentry, "using menu %s",
//! pdbMenu->name)`): the names come from `FieldDesc::menu_name`, and
//! without them the loader falls back to the field name.

#![cfg(unix)]

use std::os::fd::AsRawFd;

use epics_rs::base::server::ioc_builder::IocBuilder;

const DB: &str = "record(opcuaItem, \"I1\") { field(DEFACTN, \"bogus\") }\n\
                  record(opcuaItem, \"I2\") { field(BINI, \"bogus\") }\n\
                  record(opcuaItem, \"I3\") { field(WOC, \"bogus\") }\n";

#[test]
fn a_bad_label_names_its_menu() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("item.db");
    std::fs::write(&path, DB).unwrap();

    // The loader prints to fd 2; this binary's only test owns it.
    let sink = tempfile::NamedTempFile::new().unwrap();
    let saved = unsafe { libc::dup(2) };
    assert!(unsafe { libc::dup2(sink.as_file().as_raw_fd(), 2) } >= 0);
    let p = path.to_string_lossy().into_owned();
    let ran = std::thread::spawn(move || {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(async {
                let (name, factory) = opcua::ioc::item_record_factory();
                IocBuilder::new()
                    .register_record_type(name, factory)
                    .db_file(&p, &Default::default())?
                    .build()
                    .await
                    .map(|_| ())
            })
    })
    .join();
    unsafe {
        libc::dup2(saved, 2);
        libc::close(saved);
    }
    assert!(ran.expect("builder thread").is_err());

    let got = std::fs::read_to_string(sink.path()).unwrap();
    for (rec, field, menu) in [
        ("I1", "DEFACTN", "menuDefAction"),
        ("I2", "BINI", "menuBini"),
        ("I3", "WOC", "menuWoc"),
    ] {
        let want =
            format!("Can't set '{rec}.{field}' to 'bogus' using menu {menu} : Illegal choice");
        assert!(got.contains(&want), "no {want:?} in:\n{got}");
    }
}
