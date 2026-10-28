//! A SPiiPlus on an octet port for the tests: it answers the ASCII queries
//! and the binary array reads and writes from one variable store, and logs
//! every command it is sent.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::InterfaceMask;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::user::AsynUser;

use crate::bincomm::{ACKNOWLEDGE, FRAME_START, REPLY_END, REPLY_START};

/// The controller's state, shared with the test.
#[derive(Default)]
pub struct Controller {
    /// `var(i)(j)`; an absent element reads 0.
    pub vars: HashMap<String, HashMap<(i32, i32), f64>>,
    /// Commands answered with an error, a binary read as `BIN ??var(..)(..)`.
    pub refused: HashSet<String>,
    /// Binary reads left unanswered, which the read times out on.
    pub dropped: HashSet<String>,
    /// Every command, a binary one as `BIN ??var(..)(..)` or
    /// `BIN >>var(..)(..)`.
    pub sent: Vec<String>,
}

impl Controller {
    pub fn set(&mut self, var: &str, values: &[f64]) {
        let cells = self.vars.entry(var.to_string()).or_default();
        for (i, v) in values.iter().enumerate() {
            cells.insert((i as i32, 0), *v);
        }
    }

    fn get(&self, var: &str, i: i32, j: i32) -> f64 {
        self.vars
            .get(var)
            .and_then(|cells| cells.get(&(i, j)))
            .copied()
            .unwrap_or(0.0)
    }

    fn ascii(&mut self, cmd: &str) -> Vec<u8> {
        self.sent.push(cmd.to_string());
        if self.refused.contains(cmd) {
            return b"?1001\r".to_vec();
        }
        let reply = if cmd == "?VR" {
            "v3.13".to_string()
        } else if let Some(query) = cmd.strip_prefix('?') {
            match query.split_once('(') {
                Some((var, rest)) => {
                    let i = rest.trim_end_matches(')').parse().unwrap_or(0);
                    format!("{}", self.get(var, i, 0))
                }
                None if self.vars.contains_key(query) => "0".to_string(),
                None => "?1064".to_string(),
            }
        } else {
            ":".to_string()
        };
        format!("{reply}\r").into_bytes()
    }

    fn binary(&mut self, frame: &[u8]) -> Vec<u8> {
        let id = frame[1];
        let body = &frame[4..frame.len() - 1];
        let text = String::from_utf8_lossy(body);
        // A read is `[%slice%]%??<size>var(..)(..)`, a write
        // `[%slice]%>>\x08var(..)(..)/%<doubles>`.
        if let Some(at) = text.find("%??") {
            let size = body[at + 3] as usize;
            let name = &text[at + 4..];
            let sent = format!("BIN ??{name}");
            self.sent.push(sent.clone());
            if self.dropped.contains(&sent) {
                return Vec::new();
            }
            if self.refused.contains(&sent) {
                let mut reply = vec![REPLY_START, id, 6, 0];
                reply.extend_from_slice(b"?1001\r");
                reply.push(REPLY_END);
                return reply;
            }
            let (var, (a, b), (c, d)) = parse_span(name);
            let mut data = Vec::new();
            for i in a..=b {
                for j in c..=d {
                    let v = self.get(&var, i, j);
                    if size == 8 {
                        data.extend_from_slice(&v.to_le_bytes());
                    } else {
                        data.extend_from_slice(&(v as i32).to_le_bytes());
                    }
                }
            }
            let mut reply = vec![REPLY_START, id, data.len() as u8, (data.len() >> 8) as u8];
            reply.extend(data);
            reply.push(REPLY_END);
            return reply;
        }
        let at = text.find("%>>").expect("a binary read or write");
        let end = text.find("/%").expect("a write's data");
        let name = text[at + 4..end].to_string();
        self.sent.push(format!("BIN >>{name}"));
        let (var, (a, b), (c, d)) = parse_span(&name);
        let mut values = body[end + 2..]
            .as_chunks::<8>()
            .0
            .iter()
            .map(|b| f64::from_le_bytes(*b));
        let cells = self.vars.entry(var).or_default();
        for i in a..=b {
            for j in c..=d {
                if let Some(v) = values.next() {
                    cells.insert((i, j), v);
                }
            }
        }
        vec![ACKNOWLEDGE, id]
    }
}

/// `var(a,b)(c,d)`.
fn parse_span(name: &str) -> (String, (i32, i32), (i32, i32)) {
    let (var, rest) = name.split_once('(').unwrap();
    let nums: Vec<i32> = rest
        .split(['(', ')', ','])
        .filter(|s| !s.is_empty())
        .map(|s| s.parse().unwrap())
        .collect();
    (var.to_string(), (nums[0], nums[1]), (nums[2], nums[3]))
}

struct Line {
    base: PortDriverBase,
    controller: Arc<Mutex<Controller>>,
    reply: Vec<u8>,
}

impl PortDriver for Line {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let mut ctrl = self.controller.lock().unwrap();
        self.reply = if data.first() == Some(&FRAME_START) {
            ctrl.binary(data)
        } else {
            let cmd = String::from_utf8_lossy(data);
            ctrl.ascii(cmd.trim_end_matches('\r'))
        };
        Ok(data.len())
    }
    fn io_read_octet(
        &mut self,
        _: &AsynUser,
        buf: &mut [u8],
    ) -> epics_rs::asyn::interpose::OctetReadResult {
        if self.reply.is_empty() {
            return epics_rs::asyn::interpose::OctetReadResult::failed(AsynError::Status {
                status: AsynStatus::Timeout,
                message: "no reply".into(),
            });
        }
        let n = self.reply.len().min(buf.len());
        buf[..n].copy_from_slice(&self.reply[..n]);
        self.reply.drain(..n);
        epics_rs::asyn::interpose::OctetReadResult::ok(
            n,
            epics_rs::asyn::interpose::EomReason::empty(),
        )
    }
}

/// Register octet port `name` answered by a fresh controller.
pub fn line(name: &str) -> Arc<Mutex<Controller>> {
    let controller = Arc::new(Mutex::new(Controller::default()));
    let driver = Line {
        base: PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::OCTET_TRANSPORT,
            InterfaceMask::OCTET,
        ),
        controller: controller.clone(),
        reply: Vec::new(),
    };
    let claim = claim_port(name).unwrap();
    let (runtime, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
    claim
        .publish(runtime.port_handle().clone(), &open_owner())
        .unwrap();
    controller
}
