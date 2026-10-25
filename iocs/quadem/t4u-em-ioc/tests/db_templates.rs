//! Every record in the template st.cmd loads must bind to a parameter the
//! drvT4U_EM port creates. C's iocT4U_EM loads T4U_EM.template, whose
//! WaitStateMode and ReadsPerPacket records name QE_WSMODE and QE_RPP, which
//! only drvT4UDirect_EM creates (drvT4UDirect_EM.cpp:182-183).

use std::collections::HashMap;
use std::net::TcpListener;
use std::path::{Path, PathBuf};

use epics_rs::asyn::port::DrvUserRequest;
use epics_rs::asyn::registry::claim_port;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use epics_rs::base::server::db_loader::{DbLoadConfig, parse_db_file};
use quadem::create_t4u_em;

fn ioc_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// st.cmd's `epicsEnvSet("TEMPLATE", ...)`.
fn st_cmd_template() -> String {
    let st = std::fs::read_to_string(ioc_dir().join("st.cmd")).unwrap();
    let line = st
        .lines()
        .find(|l| l.starts_with("epicsEnvSet(\"TEMPLATE\""))
        .expect("st.cmd sets TEMPLATE");
    line.split('"').nth(3).unwrap().to_string()
}

/// `(record, addr, drvInfo)` of every `@asyn(PORT ...)` link in the template.
fn links(port: &str) -> Vec<(String, i32, String)> {
    let db = ioc_dir().join("../db");
    let macros: HashMap<String, String> = [
        ("P", "QE1_"),
        ("R", "T4U_EM_"),
        ("PORT", port),
        ("ADDR", "0"),
        ("TIMEOUT", "1"),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v.to_string()))
    .collect();
    let config = DbLoadConfig {
        include_paths: vec![
            db.clone(),
            Path::new(epics_rs::ad_core::AD_CORE_DIR).join("db"),
        ],
        ..Default::default()
    };
    let template = db.join(format!("{}.template", st_cmd_template()));
    let mut links = Vec::new();
    for def in parse_db_file(&template, &macros, &config).unwrap() {
        for fd in &def.fields {
            if fd.name != "INP" && fd.name != "OUT" {
                continue;
            }
            let value = fd.value.as_str_lossy();
            let Some((spec, drv_info)) = value
                .strip_prefix("@asyn(")
                .and_then(|rest| rest.split_once(')'))
            else {
                continue;
            };
            let mut spec = spec.split([' ', ',']).filter(|s| !s.is_empty());
            if spec.next() != Some(port) {
                continue;
            }
            let addr = spec.next().and_then(|a| a.parse().ok()).unwrap_or(0);
            links.push((def.name.to_string(), addr, drv_info.trim().to_string()));
        }
    }
    links
}

#[test]
fn every_template_link_binds() {
    let port = "T4U_EM_TPL";
    let links = links(port);
    assert!(!links.is_empty());

    // No middle layer listens: the port is built unlinked, as C's is.
    let dead = TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let rt = create_t4u_em(
        &claim_port(port).unwrap(),
        "127.0.0.1",
        1000,
        dead,
        0,
        &InterruptGate::opened(),
    )
    .unwrap();

    let unbound: Vec<String> = links
        .iter()
        .filter(|(_, addr, drv_info)| {
            rt.port_handle()
                .drv_user_create_blocking(&DrvUserRequest::new(drv_info, *addr))
                .is_err()
        })
        .map(|(record, _, drv_info)| format!("{record} -> {drv_info}"))
        .collect();
    assert!(unbound.is_empty(), "unbound: {unbound:#?}");
}
