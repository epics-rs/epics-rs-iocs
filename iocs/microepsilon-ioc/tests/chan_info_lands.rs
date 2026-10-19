//! The L0 records run capaNCDT6200.proto: the measDataM -> chanNInfoM chain's
//! `$CHIn` replies land in chanN{ANO,NAM,SNO,OFS,RNG,UNT}, chanNRNG drives
//! measRangeChanNC, and the L1 displacements scale by it as C does
//! (capaNCDT6200Sup.c: disp * range / 16777215).

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

const RNG: [i32; 4] = [50, 100, 200, 500];
const DISP: [u32; 4] = [0x40_0000, 0x80_0000, 0xC0_0000, 0x20_0000];

/// One loopback address per test process: capaNCDT6200Configure always
/// dials port 10001, so the address is what keeps runs apart.
fn host() -> String {
    format!(
        "127.0.{}.{}",
        1 + std::process::id() % 200,
        2 + std::process::id() % 250
    )
}

/// The config port: a welcome line on connect (grabWelcomeText), then one
/// CR LF reply per CR-terminated command, shaped as the proto's `in` reads it.
fn serve_config(mut c: TcpStream) {
    let _ = c.write_all(b"capaNCDT 6200\r\n");
    let mut r = BufReader::new(c.try_clone().unwrap());
    let mut line = Vec::new();
    while r.read_until(b'\r', &mut line).is_ok_and(|n| n > 0) {
        let cmd = String::from_utf8_lossy(&line).trim().to_string();
        line.clear();
        let reply = match cmd
            .strip_prefix("$CHI")
            .and_then(|n| n.parse::<usize>().ok())
        {
            Some(n @ 1..=4) => format!(
                "$CHI{n}:ANO{n},NAMCS{n:02},SNO{},OFS0,RNG{},UNTumOK",
                1000 + n,
                RNG[n - 1]
            ),
            _ => format!("{cmd}OK"),
        };
        if c.write_all(format!("{reply}\r\n").as_bytes()).is_err() {
            return;
        }
    }
}

/// The data port: one packet per send, preamble bytes "SAEM", all four
/// channels, measValueCounter +1 per packet.
fn serve_data(mut c: TcpStream) {
    for n in 1u32.. {
        let mut p = b"SAEM".to_vec();
        for v in [6200u32, 1234, 15, 0, 0] {
            p.extend(v.to_le_bytes());
        }
        p.extend((n as u16).to_le_bytes());
        p.extend(16u16.to_le_bytes());
        p.extend(n.to_le_bytes());
        for d in DISP {
            p.extend(d.to_le_bytes());
        }
        if c.write_all(&p).is_err() {
            return;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn listen(addr: &str, serve: fn(TcpStream)) -> u16 {
    let l = TcpListener::bind(addr).expect(addr);
    let port = l.local_addr().unwrap().port();
    std::thread::spawn(move || {
        for c in l.incoming().flatten() {
            std::thread::spawn(move || serve(c));
        }
    });
    port
}

struct Ioc {
    child: Child,
    lines: mpsc::Receiver<String>,
    st: std::path::PathBuf,
}

impl Drop for Ioc {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_file(&self.st);
    }
}

impl Ioc {
    /// `dbgf <pv>`'s value text, read off the shell's stdout.
    fn dbgf(&mut self, pv: &str) -> Option<String> {
        while self.lines.try_recv().is_ok() {}
        let stdin = self.child.stdin.as_mut().unwrap();
        writeln!(stdin, "dbgf {pv}").ok()?;
        let deadline = Instant::now() + Duration::from_secs(2);
        while let Ok(l) = self
            .lines
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
        {
            if let Some((_, v)) = l.split_once("DBF_").and_then(|(_, r)| r.split_once(": ")) {
                // `DBF_LONG:           3 = 0x3`, `DBF_STRING: "CS01"`
                return v
                    .split_whitespace()
                    .next()
                    .map(|t| t.trim_matches('"').to_string());
            }
        }
        None
    }
}

#[test]
fn chan_info_and_ranges_land_and_scale_the_displacements() {
    let host = host();
    let cfg = listen(&format!("{host}:0"), serve_config);
    listen(&format!("{host}:10001"), serve_data);

    let dir = env!("CARGO_MANIFEST_DIR");
    let tmp = std::env::temp_dir().join(format!("microepsilon-st-{}.cmd", std::process::id()));
    std::fs::write(
        &tmp,
        format!(
            "epicsEnvSet(\"STREAM_PROTOCOL_PATH\", \"{dir}/db\")\n\
             drvAsynIPPortConfigure(\"L0\", \"{host}:{cfg}\", 0, 0, 0)\n\
             capaNCDT6200Configure(\"L1\", \"{host}\", \"10001\")\n\
             dbLoadRecords(\"{dir}/db/xxCapaNCDT6200.template\", \"dev=t,Link=L0\")\n\
             dbLoadRecords(\"{dir}/db/xxCapaMeas.template\", \"dev=t,PORT=L1\")\n\
             iocInit()\n"
        ),
    )
    .unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_microepsilon-ioc"))
        .arg(&tmp)
        .env("EPICS_CAS_INTF_ADDR_LIST", &host)
        .env("EPICS_PVAS_INTF_ADDR_LIST", &host)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("microepsilon-ioc");
    let out = BufReader::new(child.stdout.take().unwrap());
    let (tx, lines) = mpsc::channel();
    std::thread::spawn(move || {
        for l in out.lines().map_while(Result::ok) {
            let _ = tx.send(l);
        }
    });
    let mut ioc = Ioc {
        child,
        lines,
        st: tmp,
    };

    let deadline = Instant::now() + Duration::from_secs(20);
    let expect: Vec<(String, f64)> = (1..=4)
        .flat_map(|n| {
            let rng = RNG[n - 1] as f64;
            [
                (format!("t:chan{n}RNG"), rng),
                (format!("t:measRangeChan{n}C"), rng),
                (
                    format!("t:dispChan{n}M"),
                    DISP[n - 1] as f64 * rng / 16_777_215.0,
                ),
            ]
        })
        .collect();
    let mut last = Vec::new();
    while Instant::now() < deadline {
        last = expect
            .iter()
            .map(|(pv, _)| ioc.dbgf(pv).and_then(|v| v.parse::<f64>().ok()))
            .collect::<Vec<_>>();
        if last
            .iter()
            .zip(&expect)
            .all(|(got, (_, want))| got.is_some_and(|g| (g - want).abs() < 1e-3))
        {
            assert_eq!(ioc.dbgf("t:chan1NAM").as_deref(), Some("CS01"));
            assert_eq!(ioc.dbgf("t:chan4UNT").as_deref(), Some("um"));
            assert_eq!(ioc.dbgf("t:chan3SNO").as_deref(), Some("1003"));
            return;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    panic!(
        "never landed: {:?}",
        expect.iter().map(|(p, _)| p).zip(last).collect::<Vec<_>>()
    );
}
