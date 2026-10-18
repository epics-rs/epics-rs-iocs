//! The scan-list / PLC driver layer -- port of `drvEtherIP.c`.
//!
//! One scan thread per PLC. Each PLC owns a set of scan lists (one per scan
//! period); each list owns the tags scanned at that period. A scan pass packs
//! as many tag reads/writes as fit under the transfer-buffer limit into one CIP
//! MultiRequest, sends it, and fans the results back out to the tags.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use parking_lot::Mutex;
use tokio::sync::mpsc::Sender;

use crate::cip::{self, CipType, ParsedTag};
use crate::connection::{Connection, EipError};
use crate::encap::{self, ETHERIP_PORT};
use chrono::TimeZone;
use epics_libcom_rs::cfmt::CFloat;
use epics_rs::base::runtime::log::errlog_vprintf;
use epics_rs::base::runtime::task::CallbackPriority;
use epics_rs::base::server::database::DeviceDone;
use epics_rs::base::server::ioc_app::InitHookState;

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
/// `drvEtherIP_default_rate` (seconds), as f64 bits.
static DEFAULT_RATE_BITS: AtomicU64 = AtomicU64::new(0);

/// `drvEtherIP_default_rate`.
pub fn default_rate() -> f64 {
    f64::from_bits(DEFAULT_RATE_BITS.load(Ordering::Relaxed))
}

pub fn set_default_rate(secs: f64) {
    DEFAULT_RATE_BITS.store(secs.to_bits(), Ordering::Relaxed);
}

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

/// A bound record's end of a tag's callbacks.
pub struct Listener {
    tx: Sender<()>,
    /// Set while the record's write is PACT; the next transfer completes it.
    pub write_done: Mutex<Option<(DeviceDone, CallbackPriority)>>,
}

pub struct TagInfo {
    pub string_tag: String,
    pub tag: ParsedTag,
    /// How many elements to request; grows as records register interest.
    pub elements: Mutex<usize>,
    pub data: Mutex<TagData>,
    /// One per bound record (C `TagInfo.callbacks`).
    listeners: Mutex<Vec<Arc<Listener>>>,
    /// The stats of the list this tag lives in -- the statistics device
    /// support reads them through the tag (C `TagInfo.scanlist`).
    /// Swapped when `add_tag` moves the tag to a faster list.
    list_stats: Mutex<Arc<Mutex<ListStats>>>,
}

impl TagInfo {
    /// The stats of the list this tag lives in -- the statistics device
    /// support reads them through the tag (C `TagInfo.scanlist`).
    pub fn list_stats(&self) -> Arc<Mutex<ListStats>> {
        self.list_stats.lock().clone()
    }

    /// C's callback loop after every transfer: a record whose write is
    /// pending ends that write's cycle (`if (rec->pact) process`), every
    /// other record gets an I/O Intr pulse.
    ///
    /// Never blocks: a full channel already has a pending pulse, and the record
    /// reads the *current* value when it processes, so coalescing is correct.
    pub(crate) fn notify(&self) {
        let listeners = self.listeners.lock();
        for l in listeners.iter() {
            match l.write_done.lock().take() {
                Some((done, prio)) => {
                    let _ = done.complete(prio, Ok(()));
                }
                None => {
                    let _ = l.tx.try_send(());
                }
            }
        }
    }

    pub fn add_listener(&self, tx: Sender<()>) -> Arc<Listener> {
        let l = Arc::new(Listener {
            tx,
            write_done: Mutex::new(None),
        });
        self.listeners.lock().push(l.clone());
        l
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

#[derive(Clone, Default)]
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
    /// `ip_addr` and `slot`; `drvEtherIP_define_PLC` may redefine them.
    pub addr: Mutex<(String, u8)>,
    pub scanlists: Mutex<Vec<Arc<ScanList>>>,
    pub stats: Mutex<PlcStats>,
    pub connected: AtomicBool,
    pub identity: Mutex<Option<crate::connection::IdentityInfo>>,
    /// C's `plc->connection` under `plc->lock`: the scan thread holds it for
    /// a pass, iocsh diagnostics borrow it between passes.
    conn: Mutex<Option<Connection>>,
    stop: AtomicBool,
    started: AtomicBool,
}

impl Plc {
    fn new(name: &str, ip_addr: &str, slot: u8) -> Plc {
        Plc {
            name: name.to_string(),
            addr: Mutex::new((ip_addr.to_string(), slot)),
            scanlists: Mutex::new(Vec::new()),
            stats: Mutex::new(PlcStats::default()),
            connected: AtomicBool::new(false),
            identity: Mutex::new(None),
            conn: Mutex::new(None),
            stop: AtomicBool::new(false),
            started: AtomicBool::new(false),
        }
    }

    /// `drvEtherIP_add_tag`: register a tag for scanning at `period`,
    /// requesting at least `elements` elements. A tag already in a slower
    /// list moves to the list at `period`; the list it leaves stays.
    pub fn add_tag(&self, period: f64, string_tag: &str, elements: usize) -> Option<Arc<TagInfo>> {
        let mut lists = self.scanlists.lock();
        let get_list =
            |lists: &mut Vec<Arc<ScanList>>| match lists.iter().find(|l| l.period == period) {
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

        let found = lists.iter().find_map(|list| {
            let tags = list.tags.lock();
            let info = tags.iter().find(|t| t.string_tag == string_tag)?.clone();
            Some((list.clone(), info))
        });
        if let Some((list, info)) = found {
            if list.period > period {
                list.tags.lock().retain(|t| !Arc::ptr_eq(t, &info));
                let faster = get_list(&mut lists);
                *info.list_stats.lock() = faster.stats.clone();
                faster.tags.lock().push(info.clone());
            }
            let mut n = info.elements.lock();
            *n = (*n).max(elements);
            drop(n);
            return Some(info);
        }

        let list = get_list(&mut lists);
        let tag = ParsedTag::parse(string_tag)?;
        let info = Arc::new(TagInfo {
            string_tag: string_tag.to_string(),
            tag,
            elements: Mutex::new(elements),
            data: Mutex::new(TagData::default()),
            listeners: Mutex::new(Vec::new()),
            list_stats: Mutex::new(list.stats.clone()),
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
        // C `reset_ScanList`; a zero `scheduled_time` is due at once.
        for list in self.scanlists.lock().iter() {
            list.enabled.store(true, Ordering::Relaxed);
            *list.stats.lock() = ListStats::default();
            *list.scheduled.lock() = Instant::now();
        }
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

/// C's PLC list: definition order, which every diagnostic walks in.
static REGISTRY: Mutex<Vec<Arc<Plc>>> = Mutex::new(Vec::new());

/// `drvEtherIP_define_PLC`: a second definition replaces the address.
pub fn define_plc(name: &str, ip_addr: &str, slot: u8) -> Arc<Plc> {
    let mut reg = REGISTRY.lock();
    if let Some(plc) = reg.iter().find(|p| p.name == name) {
        crate::eip_printf!(1, "Redefining IP address of PLC {name}?\n");
        *plc.addr.lock() = (ip_addr.to_string(), slot);
        return plc.clone();
    }
    let plc = Arc::new(Plc::new(name, ip_addr, slot));
    reg.push(plc.clone());
    plc
}

/// `drvEtherIP_find_PLC`.
pub fn find_plc(name: &str) -> Option<Arc<Plc>> {
    REGISTRY.lock().iter().find(|p| p.name == name).cloned()
}

pub fn all_plcs() -> Vec<Arc<Plc>> {
    REGISTRY.lock().clone()
}

/// `drvEtherIP_reset_statistics`.
pub fn reset_statistics() {
    for plc in all_plcs() {
        plc.reset_statistics();
    }
}

/// `drvEtherIP_private.lock` being set: `drvEtherIP_init` has run.
static INITIALIZED: AtomicBool = AtomicBool::new(false);
/// `databaseIsReady`: set at `initHookAfterScanInit`.
static DATABASE_READY: AtomicBool = AtomicBool::new(false);

/// `drvEtherIP_init`.
pub fn init() {
    if INITIALIZED.swap(true, Ordering::SeqCst) {
        crate::eip_printf!(0, "drvEtherIP_init called more than once!\n");
    }
}

/// `drvEtherIP_initialized`.
pub fn initialized() -> bool {
    INITIALIZED.load(Ordering::SeqCst)
}

/// The init hooks C's driver and device support act on: devEtherIP's
/// `init(1)` (finishDevSup, before `initHookAfterFinishDevSup`) and
/// `drvEtherIP_initHook` at `initHookAfterScanInit`. Both call
/// `drvEtherIP_restart`; only the second finds the database ready.
pub fn init_hook(state: InitHookState) {
    match state {
        InitHookState::AfterFinishDevSup => {
            restart();
        }
        InitHookState::AfterScanInit if initialized() => {
            DATABASE_READY.store(true, Ordering::SeqCst);
            restart();
        }
        _ => {}
    }
}

/// `drvEtherIP_restart`: drop every PLC's connection (its scan thread
/// reconnects) and start the scan threads not running yet. Returns how many
/// it started; nothing happens before `drvEtherIP_init` or while the
/// database is not ready.
pub fn restart() -> usize {
    if !initialized() {
        return 0;
    }
    if !DATABASE_READY.load(Ordering::SeqCst) {
        for plc in all_plcs() {
            crate::eip_printf!(
                4,
                "drvEtherIP: Delaying launch of scan task for PLC '{}' until database ready\n",
                plc.name
            );
        }
        return 0;
    }
    let mut started = 0;
    for plc in all_plcs() {
        // Held like C's plc->lock: a running pass finishes first.
        let mut conn = plc.conn.lock();
        disconnect(&plc, &mut conn);
        if plc.started.swap(true, Ordering::SeqCst) {
            continue;
        }
        let name: String = format!("EIP{}", plc.name.chars().take(16).collect::<String>());
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

/// C `disconnect_PLC`: close a live connection and invalidate the tags.
fn disconnect(plc: &Plc, conn: &mut Option<Connection>) {
    if let Some(c) = conn.take() {
        crate::eip_printf_time!(4, "EIP disconnecting {}\n", plc.name);
        c.shutdown();
        plc.connected.store(false, Ordering::Relaxed);
        plc.invalidate_tags();
    }
}

/// `drvEtherIP_describe`: describe `type_id` on every PLC, over the
/// connection the scan thread holds between passes.
pub fn describe(type_id: u16) {
    for plc in all_plcs() {
        match plc.conn.lock().as_mut() {
            Some(c) => {
                c.describe_type(type_id);
            }
            // C sends on the closed socket and fails there.
            None => {
                crate::eip_printf!(9, "EIP describe type 0x{type_id:04X}\n");
                crate::eip_printf!(1, "EIP_describe_type: send failed\n");
            }
        }
    }
}

/// `drvEtherIP_read_tag`: a one-off connection that reads `tag_name` and
/// dumps it. -1 when the PLC does not start up, else 0.
pub fn read_tag(ip_addr: &str, slot: u8, tag_name: &str, elements: u16, timeout_ms: u64) -> i32 {
    let limit = BUFFER_LIMIT.load(Ordering::Relaxed) as usize;
    let timeout = Duration::from_millis(timeout_ms);
    let Ok(mut c) = Connection::startup(ip_addr, ETHERIP_PORT, slot, timeout, limit) else {
        return -1;
    };
    if let Some(tag) = ParsedTag::parse(tag_name)
        && let Ok(data) = c.read_tag(&tag, elements)
    {
        crate::eip_printf!(
            0,
            "{}",
            cip::dump_raw_cip_data(&data, usize::from(elements))
        );
    }
    c.shutdown();
    0
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
    while !plc.stop.load(Ordering::Relaxed) {
        // The pass holds the connection the way C's PLC_scan_task holds
        // plc->lock; the sleep comes after it is released.
        let pause = scan_pass(&plc, &mut plc.conn.lock());
        if let Some(pause) = pause {
            std::thread::sleep(pause);
        }
    }

    if let Some(c) = plc.conn.lock().take() {
        c.shutdown();
    }
    plc.connected.store(false, Ordering::Relaxed);
    plc.started.store(false, Ordering::SeqCst);
}

/// One turn of C's `scan_loop:` label. Returns how long to sleep.
fn scan_pass(plc: &Arc<Plc>, conn: &mut Option<Connection>) -> Option<Duration> {
    // -- connect ---------------------------------------------------------
    if conn.is_none() {
        let limit = BUFFER_LIMIT.load(Ordering::Relaxed) as usize;
        crate::eip_printf_time!(4, "EIP connecting {}\n", plc.name);
        let (ip_addr, slot) = plc.addr.lock().clone();
        match Connection::startup(&ip_addr, ETHERIP_PORT, slot, timeout(), limit) {
            Ok(mut c) => {
                *plc.identity.lock() = Some(c.identity.clone());
                if !complete_tag_infos(plc, &mut c) {
                    errlog_vprintf(format_args!(
                        "EIP error during scan list completion for {ip_addr}:{ETHERIP_PORT}\n"
                    ));
                    // C disconnect_PLC: the session is open, so the tags
                    // are invalidated.
                    *conn = Some(c);
                    disconnect(plc, conn);
                    crate::eip_printf_time!(2, "drvEtherIP: PLC '{}' is disconnected\n", plc.name);
                    plc.stats.lock().plc_errors += 1;
                    return Some(timeout());
                }
                plc.connected.store(true, Ordering::Relaxed);
                *conn = Some(c);
            }
            Err(_) => {
                errlog_vprintf(format_args!(
                    "EIP connection failed for {ip_addr}:{ETHERIP_PORT}\n"
                ));
                // C's EIP_startup leaves no socket, so disconnect_PLC
                // never runs and the records keep their state.
                crate::eip_printf_time!(2, "drvEtherIP: PLC '{}' is disconnected\n", plc.name);
                plc.connected.store(false, Ordering::Relaxed);
                return Some(timeout());
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
        disconnect(plc, conn);
        return None;
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
        None => Some(MIN_TIMEOUT),
        Some(due) if due > now => Some(due - now),
        Some(due) => {
            let delay = -(now - due).as_secs_f64();
            if delay <= -SLEEP_QUANTUM {
                crate::eip_printf!(
                    8,
                    "drvEtherIP scan task slow, {} sec delay\n",
                    CFloat(delay).g()
                );
                plc.stats.lock().slow_scans += 1;
            }
            None
        }
    }
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
        crate::eip_printf!(5, "- Scanlist {:.2}\n", CFloat(list.period));
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

/// `ETHERIP_MAYOR` / `ETHERIP_MINOR` (drvEtherIP.h).
const ETHERIP_MAYOR: u32 = 3;
const ETHERIP_MINOR: u32 = 10;

/// `epicsTimeToStrftime(.., "%Y/%m/%d %H:%M:%S.%04f", ..)` of seconds past
/// the EPICS epoch, in local time.
fn epics_strftime(epics_secs: f64) -> String {
    let unix = epics_secs + EPICS_EPOCH_OFFSET;
    let secs = unix.floor();
    let nanos = ((unix - secs) * 1e9) as u32;
    match chrono::Local.timestamp_opt(secs as i64, nanos).single() {
        Some(t) => format!(
            "{}.{:04}",
            t.format("%Y/%m/%d %H:%M:%S"),
            t.timestamp_subsec_nanos() / 100_000
        ),
        None => String::new(),
    }
}

/// `dump_TagInfo`.
fn dump_tag_info(info: &TagInfo, list: &ScanList, level: i32) {
    println!(
        "*** Tag '{}' @ 0x{:X}:",
        info.string_tag, info as *const TagInfo as usize
    );
    let d = info.data.lock();
    let elements = *info.elements.lock();
    if level > 3 {
        println!(
            "  scanlist            : 0x{:X}",
            list as *const ScanList as usize
        );
        println!(
            "  compiled tag        : '{}', {elements} elements",
            info.tag
        );
        println!("  cip read requ./resp.: {} / {}", d.r_request, d.r_response);
        println!("  cip write req./resp.: {} / {}", d.w_request, d.w_response);
        println!(
            "  data_lock ID        : 0x{:X}",
            &info.data as *const Mutex<TagData> as usize
        );
        println!(
            "  data_size / valid   : {} / {}",
            d.buf.len(),
            d.valid_data_size
        );
        println!(
            "  do_write/is_writing : {} / {}",
            if d.do_write { "yes" } else { "no" },
            if d.is_writing { "yes" } else { "no" }
        );
        crate::eip_printf!(0, "  data                : ");
    }
    if d.valid_data_size > 0 {
        crate::eip_printf!(0, "{}", cip::dump_raw_cip_data(&d.buf, elements));
    } else {
        println!("-no data-");
    }
    if level > 3 {
        println!(
            "  transfer time       : {} secs",
            CFloat(d.transfer_time).g()
        );
    }
}

/// `dump_ScanList`.
fn dump_scan_list(list: &ScanList, level: i32) {
    let s = list.stats.lock().clone();
    println!(
        "Scanlist {} secs @ 0x{:X}:",
        CFloat(list.period).g(),
        list as *const ScanList as usize
    );
    println!(
        "  Status        : {}",
        if list.enabled.load(Ordering::Relaxed) {
            "enabled"
        } else {
            "DISABLED"
        }
    );
    println!("  Last scan     : {}", epics_strftime(s.scan_time));
    if level > 4 {
        let now = Instant::now();
        let due = *list.scheduled.lock();
        let next = if due >= now {
            epics_now() + (due - now).as_secs_f64()
        } else {
            epics_now() - (now - due).as_secs_f64()
        };
        println!("  Errors        : {}", s.list_errors);
        // The monotonic schedule cannot step back, so C's schedule-error
        // count stays 0 (see `scan_pass`).
        println!("  Schedule Errs : 0");
        println!("  Next scan     : {}", epics_strftime(next));
        println!("  Min. scan time: {} secs", CFloat(s.min_scan_time).g());
        println!("  Max. scan time: {} secs", CFloat(s.max_scan_time).g());
        println!("  Last scan time: {} secs", CFloat(s.last_scan_time).g());
    }
    if level > 5 {
        for info in list.tags.lock().iter() {
            dump_tag_info(info, list, level);
        }
    }
}

/// `drvEtherIP_report`, also the driver's `dbior` report.
pub fn report(level: i32) {
    if level <= 0 {
        println!(
            "drvEtherIP V{ETHERIP_MAYOR}.{ETHERIP_MINOR} - ControlLogix 5000 PLC via EtherNet/IP"
        );
        return;
    }
    println!("drvEtherIP V{ETHERIP_MAYOR}.{ETHERIP_MINOR} report, -*- outline -*-");
    if !initialized() {
        println!(" drvEtherIP lock is 0, did you call drvEtherIP_init?");
        return;
    }
    if level > 1 {
        println!("  Mutex lock: 0x{:X}", &REGISTRY as *const _ as usize);
    }
    for plc in all_plcs() {
        let (ip_addr, _) = plc.addr.lock().clone();
        println!("* PLC '{}', IP '{ip_addr}'", plc.name);
        if level > 1 {
            let ident = plc.identity.lock().clone().unwrap_or_default();
            let s = plc.stats.lock();
            println!("  Interface name        : {}", ident.name);
            println!("  Interface vendor      : 0x{:X}", ident.vendor);
            println!("  Interface type        : 0x{:X}", ident.device_type);
            println!("  Interface revision    : 0x{:X}", ident.revision);
            println!("  Interface serial      : 0x{:X}", ident.serial_number);
            println!("  scan thread slow count: {}", s.slow_scans);
            println!("  connection errors     : {}", s.plc_errors);
        }
        if level > 2 {
            println!(
                "  Mutex lock            : 0x{:X}",
                &plc.conn as *const Mutex<Option<Connection>> as usize
            );
            if plc.started.load(Ordering::SeqCst) {
                println!(
                    "  scan task ID          : 0x{:X} (running)",
                    Arc::as_ptr(&plc) as usize
                );
            } else {
                println!("  scan task ID          : 0x0 (-dead-)");
            }
            println!("  Now                   : {}", epics_strftime(epics_now()));
            if level > 3 {
                print!("** ");
                Connection::dump(
                    plc.conn.lock().as_ref(),
                    BUFFER_LIMIT.load(Ordering::Relaxed) as usize,
                    TIMEOUT_MS.load(Ordering::Relaxed),
                );
            }
            if level > 4 {
                for list in plc.scanlists.lock().iter() {
                    print!("** ");
                    dump_scan_list(list, level);
                }
            }
        }
    }
    println!();
}

/// `drvEtherIP_dump`: every tag and its value.
pub fn dump() {
    for plc in all_plcs() {
        println!("PLC {}", plc.name);
        for list in plc.scanlists.lock().iter() {
            for info in list.tags.lock().iter() {
                crate::eip_printf!(0, "{} ", info.string_tag);
                let d = info.data.lock();
                if d.valid_data_size > 0 {
                    crate::eip_printf!(
                        0,
                        "{}",
                        cip::dump_raw_cip_data(&d.buf, *info.elements.lock())
                    );
                } else {
                    println!(" - no data -");
                }
            }
        }
    }
    println!();
}

/// `drvEtherIP_list`: every tag each PLC publishes, over the connection
/// the scan thread holds between passes.
pub fn list() {
    for plc in all_plcs() {
        let (ip_addr, slot) = plc.addr.lock().clone();
        println!("Tags on PLC '{}', IP {ip_addr}, slot {slot}", plc.name);
        match plc.conn.lock().as_mut() {
            Some(c) => {
                c.list_tags();
            }
            // C sends on the closed socket and fails there.
            None => {
                crate::eip_printf!(9, "EIP list tags\n");
                crate::eip_printf!(1, "EIP_list_tags: send failed\n");
            }
        }
    }
}

/// `drvEtherIP_help`.
pub fn help() {
    let verbosity = crate::EIP_VERBOSITY.load(Ordering::Relaxed);
    let timeout = TIMEOUT_MS.load(Ordering::Relaxed);
    let limit = BUFFER_LIMIT.load(Ordering::Relaxed);
    let default_limit = encap::DEFAULT_BUFFER_LIMIT;
    print!(
        "drvEtherIP V{ETHERIP_MAYOR}.{ETHERIP_MINOR} diagnostics routines:
    EIP_verbosity(0-10)
    -  define logging detail, currently set to {verbosity}
    -  10: Dump all protocol details
        9: Hexdump each sent/received buffer
        6: show driver details
        5: show write-related operations
        4: DEFAULT: show basic startup plus error messages
        2: show more error info
        1: show severe error messages
        0: keep quiet
    EIP_timeout(<milliseconds>)
    -  define the default timeout for connecting to PLC and reading responses
       (default: {timeout} ms)
    drvEtherIP_default_rate(<seconds>)
    -  define the default scan rate
       (if neither SCAN nor INP/OUT provide one)
    EIP_buffer_limit(<bytes>)
    -  Set buffer limit enforced by driver.
       Currently {limit}, default: {default_limit}
       The actual PLC limit is unknown, it might depend on the PLC or ENET model.
       Can only be set before driver starts up.
    drvEtherIP_define_PLC(<name>, <ip_addr>, <slot>)
    -  define a PLC name (used by EPICS records) as IP
       (DNS name or dot-notation) and slot (0...)
    drvEtherIP_read_tag(<ip>, <slot>, <tag>, <elm.>, <timeout>)
    -  call to test a round-trip single tag read
       ip: IP address (numbers or name known by IOC
       slot: Slot of the PLC controller (not ENET). 0, 1, ...
       timeout: milliseconds
    drvEtherIP_report(<level>)
    -  level = 0..10
    drvEtherIP_dump
    -  dump all tags and values; short version of ..._report
    drvEtherIP_list
    -  list all tags that the PLC publishes
    drvEtherIP_describe(<type ID>)
    -  describe the tag type, used to inspect custom structures
    drvEtherIP_reset_statistics
    -  reset error counts and min/max scan times
    drvEtherIP_restart
    -  in case of communication errors, driver will restart,
       so calling this one directly shouldn't be necessary
       but is possible

"
    );
}

#[cfg(test)]
mod registry_tests {
    use super::*;

    /// drvEtherIP.c:494-501: a PLC that never connects leaves no socket, so
    /// disconnect_PLC does not run and no record is told its tag is invalid.
    #[test]
    fn a_failed_connect_leaves_the_records_alone() {
        let plc = Arc::new(Plc::new("noconnect", "", 0));
        let tag = plc.add_tag(1.0, "T", 1).unwrap();
        tag.data.lock().valid_data_size = 4;
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        tag.add_listener(tx);
        assert!(scan_pass(&plc, &mut None).is_some());
        assert_eq!(tag.data.lock().valid_data_size, 4);
        assert!(rx.try_recv().is_err(), "no record was processed");
    }

    /// drvEtherIP.c:1234: a redefinition keeps the PLC and its place in
    /// the list but takes the new address.
    #[test]
    fn redefining_a_plc_keeps_its_place_and_takes_the_new_address() {
        define_plc("regorder_b", "10.0.0.1", 0);
        define_plc("regorder_a", "10.0.0.2", 0);
        let again = define_plc("regorder_b", "10.0.0.3", 4);
        assert_eq!(*again.addr.lock(), ("10.0.0.3".to_string(), 4));
        let names: Vec<_> = all_plcs()
            .into_iter()
            .map(|p| p.name.clone())
            .filter(|n| n.starts_with("regorder_"))
            .collect();
        assert_eq!(names, ["regorder_b", "regorder_a"]);
    }

    /// drvEtherIP.c:1278: a tag registered again at a shorter period moves
    /// to that list, the list it left stays, and a longer period leaves it.
    #[test]
    fn a_faster_period_moves_the_tag_and_keeps_the_old_list() {
        let plc = Plc::new("addtag_move", "10.0.0.9", 0);
        let tag = plc.add_tag(1.0, "T", 1).unwrap();
        assert!(Arc::ptr_eq(&plc.add_tag(0.5, "T", 4).unwrap(), &tag));
        assert!(Arc::ptr_eq(&plc.add_tag(2.0, "T", 2).unwrap(), &tag));
        let lists = plc.scanlists.lock();
        let shape: Vec<_> = lists
            .iter()
            .map(|l| (l.period, l.tags.lock().len()))
            .collect();
        assert_eq!(shape, [(1.0, 0), (0.5, 1)]);
        assert!(Arc::ptr_eq(&tag.list_stats(), &lists[1].stats));
        assert_eq!(*tag.elements.lock(), 4);
    }
}
