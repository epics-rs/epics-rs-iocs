//! The scan-list / PLC driver layer -- port of `drvEtherIP.c`.
//!
//! One scan thread per PLC. Each PLC owns a set of scan lists (one per scan
//! period); each list owns the tags scanned at that period. A scan pass packs
//! as many tag reads/writes as fit under the transfer-buffer limit into one CIP
//! MultiRequest, sends it, and fans the results back out to the tags.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use parking_lot::Mutex;
use tokio::sync::mpsc::Sender;

use crate::cip::{self, CipType, ParsedTag};
use crate::connection::{Connection, EipError};
use crate::encap::{self, ETHERIP_PORT};
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::server::records::printf::format_g;

/// `EIP_TIMEOUT` -- default connect / response timeout.
pub const DEFAULT_TIMEOUT_MS: u32 = 5000;

/// `epicsThreadSleepQuantum()` on POSIX: one clock tick, 1/100 s.
const SLEEP_QUANTUM: f64 = 0.01;

/// `EIP_MIN_TIMEOUT` -- the idle delay when no scan list is due.
const MIN_TIMEOUT: Duration = Duration::from_millis(100);

/// Seconds between the UNIX epoch and the EPICS epoch (1990-01-01).
const EPICS_EPOCH_OFFSET: f64 = 631_152_000.0;

// ---------------------------------------------------------------------------
// Global driver options (the iocsh knobs)
// ---------------------------------------------------------------------------

/// `EIP_timeout` (ms).
pub static TIMEOUT_MS: AtomicU32 = AtomicU32::new(DEFAULT_TIMEOUT_MS);
/// `EIP_buffer_limit`.
pub static BUFFER_LIMIT: AtomicU32 = AtomicU32::new(encap::DEFAULT_BUFFER_LIMIT as u32);
/// `drvEtherIP_default_rate` (seconds), scaled by 1000 to stay in an atomic.
pub static DEFAULT_RATE_MS: AtomicU32 = AtomicU32::new(0);

fn timeout() -> Duration {
    Duration::from_millis(TIMEOUT_MS.load(Ordering::Relaxed) as u64)
}

// ---------------------------------------------------------------------------
// Tags
// ---------------------------------------------------------------------------

/// The mutable half of a tag, guarded by one lock (C `TagInfo.data_lock`).
#[derive(Default)]
pub struct TagData {
    /// Raw `[type][data]`, exactly as the PLC returned it.
    pub buf: Vec<u8>,
    /// 0 means "no valid value" -- the record goes INVALID.
    pub valid_data_size: usize,
    /// Device support asked for a write; the next scan pass performs it.
    pub do_write: bool,
    /// A write for this tag is in flight in the current scan pass.
    pub is_writing: bool,
    /// Round-trip time of the transfer this tag last took part in.
    pub transfer_time: f64,
    /// Sizes learned from the probe read at connect time; 0 = unusable tag.
    pub r_request: usize,
    pub r_response: usize,
    pub w_request: usize,
    pub w_response: usize,
}

impl TagData {
    pub fn has_value(&self) -> bool {
        self.valid_data_size > 0
    }
}

pub struct TagInfo {
    pub string_tag: String,
    pub tag: ParsedTag,
    /// How many elements to request; grows as records register interest.
    pub elements: Mutex<usize>,
    pub data: Mutex<TagData>,
    /// I/O Intr pulses, one per bound record (C `TagInfo.callbacks`).
    listeners: Mutex<Vec<Sender<()>>>,
    /// The stats of the list this tag lives in -- the statistics device
    /// support reads them through the tag (C `TagInfo.scanlist`).
    pub list_stats: Arc<Mutex<ListStats>>,
}

impl TagInfo {
    /// Notify every bound record that this tag's value (or validity) changed.
    ///
    /// Never blocks: a full channel already has a pending pulse, and the record
    /// reads the *current* value when it processes, so coalescing is correct.
    fn notify(&self) {
        let listeners = self.listeners.lock();
        for tx in listeners.iter() {
            let _ = tx.try_send(());
        }
    }

    pub fn add_listener(&self, tx: Sender<()>) {
        self.listeners.lock().push(tx);
    }

    /// Device support side of the write handshake: stage the value in the tag
    /// buffer, then flag it. The scan thread clears `do_write` and sets
    /// `is_writing` when it picks the write up.
    pub fn request_write(&self, data: &mut TagData, record: &str) {
        if data.do_write {
            crate::eip_printf!(6, "'{record}': already writing\n");
        } else {
            data.do_write = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Scan lists
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct ListStats {
    pub list_errors: u32,
    pub last_scan_time: f64,
    pub min_scan_time: f64,
    pub max_scan_time: f64,
    /// Seconds past the EPICS epoch when the list was last scanned.
    pub scan_time: f64,
}

pub struct ScanList {
    pub period: f64,
    pub enabled: AtomicBool,
    pub tags: Mutex<Vec<Arc<TagInfo>>>,
    pub stats: Arc<Mutex<ListStats>>,
    /// When this list is next due. Monotonic -- see `PlcHandle::scan_loop`.
    scheduled: Mutex<Instant>,
}

// ---------------------------------------------------------------------------
// PLCs
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct PlcStats {
    pub plc_errors: u32,
    pub slow_scans: u32,
}

pub struct Plc {
    pub name: String,
    pub ip_addr: String,
    pub slot: u8,
    pub scanlists: Mutex<Vec<Arc<ScanList>>>,
    pub stats: Mutex<PlcStats>,
    pub connected: AtomicBool,
    pub identity: Mutex<Option<crate::connection::IdentityInfo>>,
    stop: AtomicBool,
    started: AtomicBool,
}

impl Plc {
    fn new(name: &str, ip_addr: &str, slot: u8) -> Plc {
        Plc {
            name: name.to_string(),
            ip_addr: ip_addr.to_string(),
            slot,
            scanlists: Mutex::new(Vec::new()),
            stats: Mutex::new(PlcStats::default()),
            connected: AtomicBool::new(false),
            identity: Mutex::new(None),
            stop: AtomicBool::new(false),
            started: AtomicBool::new(false),
        }
    }

    /// Register a tag for scanning at `period`, requesting at least `elements`
    /// elements. If the tag already exists in *any* list of this PLC, the
    /// existing one is reused and its element count maximized -- the C
    /// (`drvEtherIP_add_tag`) does the same, so the first period a tag is
    /// registered at wins.
    pub fn add_tag(&self, period: f64, string_tag: &str, elements: usize) -> Option<Arc<TagInfo>> {
        let mut lists = self.scanlists.lock();

        for list in lists.iter() {
            let tags = list.tags.lock();
            if let Some(info) = tags.iter().find(|t| t.string_tag == string_tag) {
                let mut n = info.elements.lock();
                *n = (*n).max(elements);
                return Some(info.clone());
            }
        }

        let list = match lists.iter().find(|l| l.period == period) {
            Some(l) => l.clone(),
            None => {
                let l = Arc::new(ScanList {
                    period,
                    enabled: AtomicBool::new(true),
                    tags: Mutex::new(Vec::new()),
                    stats: Arc::new(Mutex::new(ListStats::default())),
                    scheduled: Mutex::new(Instant::now()),
                });
                lists.push(l.clone());
                l
            }
        };

        let tag = ParsedTag::parse(string_tag)?;
        let info = Arc::new(TagInfo {
            string_tag: string_tag.to_string(),
            tag,
            elements: Mutex::new(elements),
            data: Mutex::new(TagData::default()),
            listeners: Mutex::new(Vec::new()),
            list_stats: list.stats.clone(),
        });
        list.tags.lock().push(info.clone());
        Some(info)
    }

    /// Drop every tag's value and clear the write flags, then tell the records.
    ///
    /// C `invalidate_PLC_tags`: after an error we must not write stale data on
    /// reconnect.
    fn invalidate_tags(&self) {
        let lists: Vec<_> = self.scanlists.lock().clone();
        for list in lists {
            let tags: Vec<_> = list.tags.lock().clone();
            for info in tags {
                {
                    let mut d = info.data.lock();
                    d.is_writing = false;
                    d.do_write = false;
                    d.valid_data_size = 0;
                }
                info.notify();
            }
        }
    }

    pub fn reset_statistics(&self) {
        *self.stats.lock() = PlcStats::default();
        for list in self.scanlists.lock().iter() {
            let mut s = list.stats.lock();
            s.list_errors = 0;
            s.min_scan_time = 0.0;
            s.max_scan_time = 0.0;
        }
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

static REGISTRY: OnceLock<Mutex<HashMap<String, Arc<Plc>>>> = OnceLock::new();

fn registry() -> &'static Mutex<HashMap<String, Arc<Plc>>> {
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}

/// `drvEtherIP_define_PLC`.
pub fn define_plc(name: &str, ip_addr: &str, slot: u8) -> Arc<Plc> {
    let mut reg = registry().lock();
    reg.entry(name.to_string())
        .or_insert_with(|| Arc::new(Plc::new(name, ip_addr, slot)))
        .clone()
}

/// `drvEtherIP_find_PLC`.
pub fn find_plc(name: &str) -> Option<Arc<Plc>> {
    registry().lock().get(name).cloned()
}

pub fn all_plcs() -> Vec<Arc<Plc>> {
    registry().lock().values().cloned().collect()
}

/// `drvEtherIP_reset_statistics`.
pub fn reset_statistics() {
    for plc in all_plcs() {
        plc.reset_statistics();
    }
}

/// Start one scan thread per defined PLC. Idempotent: a PLC whose thread is
/// already running is skipped (C `drvEtherIP_restart` checks `scan_task_id`).
pub fn start_scan_tasks() -> usize {
    let mut started = 0;
    for plc in all_plcs() {
        if plc.started.swap(true, Ordering::SeqCst) {
            continue;
        }
        let name = format!("EIP{}", plc.name);
        let p = plc.clone();
        match std::thread::Builder::new()
            .name(name)
            .spawn(move || scan_loop(p))
        {
            Ok(_) => {
                crate::eip_printf!(5, "drvEtherIP: launch scan task for PLC '{}'\n", plc.name);
                started += 1;
            }
            Err(_) => plc.started.store(false, Ordering::SeqCst),
        }
    }
    started
}

/// Stop every scan thread (used by the IOC shutdown path and by tests).
pub fn stop_scan_tasks() {
    for plc in all_plcs() {
        plc.stop();
    }
}

// ---------------------------------------------------------------------------
// The scan loop
// ---------------------------------------------------------------------------

fn scan_loop(plc: Arc<Plc>) {
    let mut conn: Option<Connection> = None;

    while !plc.stop.load(Ordering::Relaxed) {
        // -- connect ---------------------------------------------------------
        if conn.is_none() {
            let limit = BUFFER_LIMIT.load(Ordering::Relaxed) as usize;
            crate::eip_printf_time!(4, "EIP connecting {}\n", plc.name);
            match Connection::startup(&plc.ip_addr, ETHERIP_PORT, plc.slot, timeout(), limit) {
                Ok(mut c) => {
                    *plc.identity.lock() = Some(c.identity.clone());
                    if !complete_tag_infos(&plc, &mut c) {
                        errlog_vprintf(format_args!(
                            "EIP error during scan list completion for {}:{ETHERIP_PORT}\n",
                            plc.ip_addr
                        ));
                        crate::eip_printf_time!(4, "EIP disconnecting {}\n", plc.name);
                        c.shutdown();
                        crate::eip_printf_time!(
                            2,
                            "drvEtherIP: PLC '{}' is disconnected\n",
                            plc.name
                        );
                        plc.stats.lock().plc_errors += 1;
                        std::thread::sleep(timeout());
                        continue;
                    }
                    plc.connected.store(true, Ordering::Relaxed);
                    conn = Some(c);
                }
                Err(_) => {
                    errlog_vprintf(format_args!(
                        "EIP connection failed for {}:{ETHERIP_PORT}\n",
                        plc.ip_addr
                    ));
                    crate::eip_printf_time!(2, "drvEtherIP: PLC '{}' is disconnected\n", plc.name);
                    plc.connected.store(false, Ordering::Relaxed);
                    plc.invalidate_tags();
                    std::thread::sleep(timeout());
                    continue;
                }
            }
        }
        let c = conn.as_mut().expect("connected above");
        crate::eip_printf_time!(10, "drvEtherIP scan PLC '{}'\n", plc.name);

        // -- scan every list that is due --------------------------------------
        let lists: Vec<_> = plc.scanlists.lock().clone();
        let start = Instant::now();
        let mut next_due: Option<Instant> = None;
        let mut failed = false;

        for list in &lists {
            if !list.enabled.load(Ordering::Relaxed) {
                continue;
            }
            let due = *list.scheduled.lock();
            if due <= start {
                let t0 = Instant::now();
                let ok = process_scan_list(c, list);
                let elapsed = t0.elapsed().as_secs_f64();

                {
                    let mut s = list.stats.lock();
                    s.scan_time = epics_now();
                    s.last_scan_time = elapsed;
                    if elapsed > s.max_scan_time {
                        s.max_scan_time = elapsed;
                    }
                    if elapsed < s.min_scan_time || s.min_scan_time == 0.0 {
                        s.min_scan_time = elapsed;
                    }
                    if !ok {
                        s.list_errors += 1;
                    }
                }

                if ok {
                    // Re-schedule exactly, from the start of this scan.
                    *list.scheduled.lock() = t0 + Duration::from_secs_f64(list.period);
                } else {
                    *list.scheduled.lock() = Instant::now() + timeout();
                    plc.stats.lock().plc_errors += 1;
                    failed = true;
                    break;
                }
            }
            let due = *list.scheduled.lock();
            next_due = Some(match next_due {
                Some(n) if n <= due => n,
                _ => due,
            });
        }

        if failed {
            // Drop the session and reconnect on the next turn.
            crate::eip_printf_time!(4, "EIP disconnecting {}\n", plc.name);
            if let Some(mut c) = conn.take() {
                c.shutdown();
            }
            plc.connected.store(false, Ordering::Relaxed);
            plc.invalidate_tags();
            continue;
        }

        // -- sleep until the next list is due ---------------------------------
        //
        // UPSTREAM FIX: the C (`drvEtherIP.c:909-930`) computes this delay from
        // a wall-clock `epicsTimeStamp`, so a backwards clock step yields a
        // huge delay; the recovery branch then dereferences `list`, which the
        // preceding `for` loop has already walked to NULL -- a guaranteed crash
        // exactly when the clock misbehaves. A monotonic `Instant` cannot step
        // backwards, so the whole failure mode is gone by construction, along
        // with the `sched_errors` counter that only ever counted it (it is not
        // reachable from the device support, so no PV disappears).
        let now = Instant::now();
        match next_due {
            None => std::thread::sleep(MIN_TIMEOUT),
            Some(due) if due > now => std::thread::sleep(due - now),
            Some(due) => {
                let delay = -(now - due).as_secs_f64();
                if delay <= -SLEEP_QUANTUM {
                    crate::eip_printf!(
                        8,
                        "drvEtherIP scan task slow, {} sec delay\n",
                        format_g(delay, 6)
                    );
                    plc.stats.lock().slow_scans += 1;
                }
            }
        }
    }

    if let Some(mut c) = conn {
        c.shutdown();
    }
    plc.connected.store(false, Ordering::Relaxed);
    plc.started.store(false, Ordering::SeqCst);
}

fn epics_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64() - EPICS_EPOCH_OFFSET)
        .unwrap_or(0.0)
}

/// C `complete_PLC_ScanList_TagInfos`: probe every tag with a single read so we
/// learn its request/response sizes (and hence how many fit in a MultiRequest).
/// A tag that cannot be read is disabled until the next reconnect.
///
/// Returns true if at least one tag answered, or if there are no tags at all.
fn complete_tag_infos(plc: &Arc<Plc>, c: &mut Connection) -> bool {
    let mut tried = 0usize;
    let mut ok = 0usize;

    crate::eip_printf!(5, "complete_PLC_ScanList_TagInfos PLC '{}':\n", plc.name);
    let lists: Vec<_> = plc.scanlists.lock().clone();
    for list in lists {
        crate::eip_printf!(5, "- Scanlist {:.2}\n", list.period);
        let tags: Vec<_> = list.tags.lock().clone();
        for info in tags {
            tried += 1;
            let elements = *info.elements.lock() as u16;
            match c.read_tag(&info.tag, elements) {
                Ok(data) => {
                    ok += 1;
                    let mut d = info.data.lock();
                    // The probe response is `4 (MR header) + type + data`, and
                    // that is exactly what the scan pass will get back.
                    d.r_request = cip::read_data_size(&info.tag);
                    d.r_response = 4 + data.len();
                    // Estimate the write sizes from the read, as the C does:
                    // a write request is the read request plus the type+data
                    // block, and its response is a bare MR header.
                    let type_and_data = d.r_response - 4;
                    d.w_request = d.r_request + type_and_data;
                    d.w_response = 4;
                    crate::eip_printf!(
                        5,
                        "  tag '{}': req {}, resp {} bytes\n",
                        info.string_tag,
                        d.r_request,
                        d.r_response
                    );

                    if data.len() < encap::BUFFER_SIZE {
                        d.buf = data;
                        d.valid_data_size = d.buf.len();
                    } else {
                        print_rejected(&info.string_tag, data.len());
                        d.valid_data_size = 0;
                    }
                }
                Err(_) => {
                    crate::eip_printf!(3, "tag '{}': Cannot read!\n", info.string_tag);
                    let mut d = info.data.lock();
                    d.r_request = 0;
                    d.r_response = 0;
                    d.w_request = 0;
                    d.w_response = 0;
                    d.valid_data_size = 0;
                }
            }
            info.notify();
        }
    }

    crate::eip_printf!(
        5,
        "complete_PLC_ScanList_TagInfos PLC '{}': tried {tried} tags, got {ok} tags\n",
        plc.name
    );
    ok > 0 || tried == 0
}

/// A tag's contribution to the current MultiRequest.
struct Item {
    info: Arc<TagInfo>,
    /// The write half of the handshake was taken for this pass.
    writing: bool,
    request: Vec<u8>,
    /// The response size this item was budgeted at.
    response_size: usize,
}

/// C `determine_MultiRequest_count` + the build half of `process_ScanList`,
/// fused: walk the tags from `start`, taking each one's read-or-write request
/// until the next one would push the request or the estimated response past the
/// buffer limit.
///
/// Returns the items and the index to resume the walk from. The two are
/// returned together because a skipped (unusable) tag makes `items.len()` an
/// invalid stride -- the C keeps a separate list cursor for the same reason.
fn build_multi_request(limit: usize, tags: &[Arc<TagInfo>], start: usize) -> (Vec<Item>, usize) {
    let mut items: Vec<Item> = Vec::new();
    let mut requests_size = 0usize;
    let mut responses_size = 0usize;
    let mut at = start;

    while at < tags.len() {
        let info = &tags[at];
        let built = {
            let mut d = info.data.lock();
            if d.r_request == 0 || d.w_request == 0 {
                // A tag the PLC would not give us: skip it, permanently.
                at += 1;
                continue;
            }
            let elements = *info.elements.lock() as u16;

            // Take the write half of the handshake here, under the same lock
            // that stages the data, so a record's write cannot be lost between
            // "value staged" and "request built".
            if d.do_write {
                d.do_write = false;
                d.is_writing = true;
            }
            if d.is_writing {
                match cip::typecode(&d.buf) {
                    Some(cip_type) => {
                        let mut request = Vec::with_capacity(d.w_request);
                        let raw = &d.buf[cip::TYPECODE_SIZE..];
                        if cip_type == CipType::Struct {
                            cip::encode_write_string(&mut request, &info.tag, elements, raw, limit);
                        } else {
                            cip::encode_write_data(
                                &mut request,
                                &info.tag,
                                cip_type,
                                elements,
                                raw,
                            );
                        }
                        (true, d.w_request, d.w_response, request)
                    }
                    None => {
                        // No value staged -> nothing to write. Fall back to a
                        // read so the tag still gets a value.
                        d.is_writing = false;
                        let mut request = Vec::with_capacity(d.r_request);
                        cip::encode_read_data(&mut request, &info.tag, elements);
                        (false, d.r_request, d.r_response, request)
                    }
                }
            } else {
                let mut request = Vec::with_capacity(d.r_request);
                cip::encode_read_data(&mut request, &info.tag, elements);
                (false, d.r_request, d.r_response, request)
            }
        };
        let (writing, req, resp, request) = built;

        let try_req = requests_size + req;
        let try_resp = responses_size + resp;
        let multi_req = cip::multi_request_size(items.len() + 1, try_req);
        let multi_resp = cip::multi_response_size(items.len() + 1, try_resp);

        if multi_req > limit || multi_resp > limit {
            crate::eip_printf!(
                8,
                " Skipping tag '{}', reached buffer limit at req/resp: {}, {}\n",
                info.string_tag,
                cip::multi_request_size(items.len(), requests_size),
                cip::multi_response_size(items.len(), responses_size)
            );
            if items.is_empty() {
                crate::eip_printf!(
                    2,
                    "Tag '{}' can never be read because it alone exceeds buffer limit of {limit} bytes,\n",
                    info.string_tag
                );
                crate::eip_printf!(3, " Request   size: {try_req:>10} bytes\n");
                crate::eip_printf!(3, " Response  size: {try_resp:>10} bytes\n");
                // Give the write handshake back -- this pass will never send it.
                if writing {
                    info.data.lock().is_writing = false;
                }
            }
            break;
        }

        requests_size = try_req;
        responses_size = try_resp;
        items.push(Item {
            info: info.clone(),
            writing,
            request,
            response_size: resp,
        });
        at += 1;
    }
    if at == tags.len() {
        crate::eip_printf!(
            8,
            " End of list, total req/resp: {}, {}\n",
            cip::multi_request_size(items.len(), requests_size),
            cip::multi_response_size(items.len(), responses_size)
        );
    }
    (items, at)
}

/// C `process_ScanList`: transfer every tag of one list, in as few
/// MultiRequests as the buffer limit allows.
fn process_scan_list(c: &mut Connection, list: &Arc<ScanList>) -> bool {
    let tags: Vec<_> = list.tags.lock().clone();
    let limit = c.buffer_limit();
    let mut start = 0usize;

    while start < tags.len() {
        let (items, next) = build_multi_request(limit, &tags, start);
        if items.is_empty() {
            // Nothing from here on fits (or the rest are unusable tags).
            return true;
        }

        let requests: Vec<Vec<u8>> = items.iter().map(|i| i.request.clone()).collect();
        let mut multi = Vec::new();
        cip::encode_multi_request(&mut multi, &requests);

        let t0 = Instant::now();
        let response = match c.send_routed(&multi) {
            Ok(r) => Some(r),
            Err(EipError::Send(_) | EipError::TooLarge { .. }) => {
                crate::eip_printf_time!(2, "EIP process_ScanList: Error while sending request\n");
                release_writes(&items);
                return false;
            }
            Err(EipError::TransactionMismatch { got, want }) => {
                crate::eip_printf_time!(2, "EIP process_ScanList: Mismatch in transaction ID\n");
                crate::eip_printf!(2, "expected {want} received {got}\n");
                release_writes(&items);
                return false;
            }
            Err(EipError::Malformed | EipError::EncapStatus { .. }) => None,
            Err(_) => {
                crate::eip_printf_time!(2, "EIP process_ScanList: No response\n");
                release_writes(&items);
                return false;
            }
        };
        let transfer_time = t0.elapsed().as_secs_f64();

        let Some(response) = response.filter(|r| cip::check_multi_request_response(r)) else {
            crate::eip_printf_time!(2, "EIP process_ScanList: Error in response\n");
            for (i, item) in items.iter().enumerate() {
                crate::eip_printf!(2, "Tag {i}: '{}'\n", item.info.string_tag);
            }
            crate::eip_printf!(
                2,
                "Request size: {} bytes, response size: {}, buffer limit: {limit}\n",
                cip::multi_request_size(items.len(), items.iter().map(|i| i.request.len()).sum()),
                cip::multi_response_size(items.len(), items.iter().map(|i| i.response_size).sum())
            );
            release_writes(&items);
            return false;
        };

        let n = response.len();
        for (i, item) in items.iter().enumerate() {
            let Some(sub) = cip::get_multi_request_response(&response, n, i) else {
                release_writes(&items);
                return false;
            };
            apply_response(item, sub, transfer_time);
            item.info.notify();
        }

        start = next;
    }
    true
}

/// Hand a failed pass's write handshakes back so the next pass retries them.
///
/// C leaves `is_writing` set on a transfer error and relies on
/// `invalidate_PLC_tags` to clear it on reconnect -- which also *drops* the
/// staged value. Re-arming `do_write` here would re-send data the operator may
/// have superseded, so we match the C's "drop the write" outcome, but we do it
/// in one place instead of leaving the flag set across an error path.
fn release_writes(items: &[Item]) {
    for item in items {
        if item.writing {
            item.info.data.lock().is_writing = false;
        }
    }
}

/// `reserve_tag_data`'s refusal of a value that would not fit `EIP_BUFFER_SIZE`.
fn print_rejected(tag: &str, size: usize) {
    crate::eip_printf!(
        2,
        "EIP reserve_tag_data: rejecting tag '{tag}' data size of {size} bytes\n"
    );
}

fn apply_response(item: &Item, sub: &[u8], transfer_time: f64) {
    let mut d = item.info.data.lock();
    d.transfer_time = transfer_time;

    if item.writing {
        d.is_writing = false;
        if !cip::check_write_data_response(sub) {
            crate::eip_printf_time!(0, "EIP: CIPWrite failed for '{}'\n", item.info.string_tag);
            d.valid_data_size = 0;
        }
        return;
    }

    match cip::check_read_data_response(sub, sub.len()) {
        Some(data) if !data.is_empty() => {
            if d.do_write {
                // A record staged a write while this read was in flight; keep
                // its data, the next pass sends it (C process_ScanList).
                crate::eip_printf!(
                    8,
                    "EIP '{}': Device support requested write in middle of read cycle.\n",
                    item.info.string_tag
                );
            } else if data.len() < encap::BUFFER_SIZE {
                d.buf.clear();
                d.buf.extend_from_slice(data);
                d.valid_data_size = data.len();
            } else {
                print_rejected(&item.info.string_tag, data.len());
                d.valid_data_size = 0;
            }
        }
        _ => d.valid_data_size = 0,
    }
}

// ---------------------------------------------------------------------------
// Diagnostics (`drvEtherIP_report`)
// ---------------------------------------------------------------------------

pub fn report(level: i32) {
    for plc in all_plcs() {
        let ident = plc.identity.lock().clone();
        println!(
            "PLC '{}': IP {}, slot {}, {}",
            plc.name,
            plc.ip_addr,
            plc.slot,
            if plc.connected.load(Ordering::Relaxed) {
                "connected"
            } else {
                "disconnected"
            }
        );
        if let Some(id) = ident {
            println!(
                "  identity: '{}' vendor 0x{:04X} type 0x{:04X} rev 0x{:04X} serial 0x{:08X}",
                id.name, id.vendor, id.device_type, id.revision, id.serial_number
            );
        }
        {
            let s = plc.stats.lock();
            println!("  errors: {}, slow scans: {}", s.plc_errors, s.slow_scans);
        }
        if level <= 0 {
            continue;
        }
        for list in plc.scanlists.lock().iter() {
            let s = list.stats.lock();
            println!(
                "  scanlist {:.4} s: errors {}, scan time last/min/max {:.5}/{:.5}/{:.5} s",
                list.period, s.list_errors, s.last_scan_time, s.min_scan_time, s.max_scan_time
            );
            if level <= 1 {
                continue;
            }
            for info in list.tags.lock().iter() {
                let d = info.data.lock();
                println!(
                    "    tag '{}': {} elements, {} valid bytes, req/resp r {}/{} w {}/{}",
                    info.string_tag,
                    *info.elements.lock(),
                    d.valid_data_size,
                    d.r_request,
                    d.r_response,
                    d.w_request,
                    d.w_response
                );
            }
        }
    }
}

/// `drvEtherIP_read_tag` -- a one-shot round-trip test, outside the scan tasks.
pub fn read_tag_once(
    ip: &str,
    slot: u8,
    tag: &str,
    elements: u16,
    timeout_ms: u32,
) -> Result<Vec<u8>, EipError> {
    let parsed = ParsedTag::parse(tag).ok_or(EipError::Malformed)?;
    let mut c = Connection::startup(
        ip,
        ETHERIP_PORT,
        slot,
        Duration::from_millis(timeout_ms as u64),
        BUFFER_LIMIT.load(Ordering::Relaxed) as usize,
    )?;
    c.read_tag(&parsed, elements)
}
