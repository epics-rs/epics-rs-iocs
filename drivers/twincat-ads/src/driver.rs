//! The asyn port driver — the `adsAsynPortDriver.cpp` half.
//!
//! One driver instance owns one TCP connection to one PLC (one AMS Net Id) and
//! serves every AMS port on it (851, 852, 501, …). Parameters are created on
//! demand, when a record binds its `drvInfo`:
//!
//! * reason 0 is `Default access`, the asynOctet ASCII command channel the
//!   motor record and StreamDevice talk to (see [`crate::octet`]);
//! * every other reason is one PLC variable, addressed either symbolically or
//!   through `.ADR.`, and fed either by a device notification (the PLC pushes on
//!   change) or by the sum-up bulk read (`POLL_RATE=`, one round trip for up to
//!   500 variables).
//!
//! Values decoded off the wire are cached in [`ParamState`] and pushed to
//! records as I/O Intr interrupts tagged with the record's own asyn interface,
//! so a `REAL` array reaches an `asynFloat32ArrayIn` waveform as `f32`s and a
//! `LREAL` reaches an `ai` as `f64` — the pairing matrix lives in
//! [`crate::convert`].
//!
//! ## Deviations from the C driver, all fixed at source
//!
//! Each is cited at the C line it comes from; none is a behaviour change the
//! PLC can observe, they are memory-safety and availability bugs.
//!
//! * `adsAsynPortDriver.cpp:552` — `cyclicThread` calls `exit(-1)` when a
//!   previously-connected AMS port drops, killing the whole IOC (every other
//!   device support on it included) because one PLC rebooted. Here the
//!   supervisor drops the socket, alarms the parameters and reconnects.
//! * `adsAsynPortDriver.cpp:3223` — `writeFloat64Array` passes
//!   `nElements * nElements * sizeof(epicsFloat64)` as the byte count; a
//!   2-element waveform writes 32 bytes from a 16-byte buffer. Rust sizes the
//!   write from the slice.
//! * `adsAsynPortDriver.cpp:4716-4790` — `fireCallbacks` passes
//!   `lastCallbackSize` (a **byte** count) as the **element** count to
//!   `doCallbacksInt16Array` / `Int32Array` / `Float32Array` / `Float64Array`,
//!   so a 100-element `LREAL` array is published as 800 elements read from a
//!   100-element buffer. Rust decodes to a typed `Vec` whose length is the
//!   element count by construction.
//! * `adsAsynPortDriver.cpp:306` — the constructor `memset`s the parameter
//!   table with `sizeof(*pAdsParamArray_)`, which is the size of *one pointer*,
//!   leaving the rest of the table uninitialised; `:496` then frees it with
//!   `delete` instead of `delete[]`. A `Vec` has neither problem.
//! * `adsAsynPortDriver.cpp:1260` — the array-allocation failure path in
//!   `updateParamInfoWithPLCInfo` calls `unlock()` without a matching `lock()`.
//! * `adsAsynPortDriver.cpp:3800-3806` — `adsReleaseSymbolicHandle` zeroes the
//!   handle *before* the error message that prints it, so the log always says
//!   `0xffffffff`.
//! * `adsAsynPortDriverUtils.cpp:409-419` — `adsTypeSize` returns `-1` as a
//!   `size_t` (i.e. `SIZE_MAX`) for `WSTRING`/`BIGTYPE`/unknown ids; every
//!   caller treats it as a length. [`AdsType::element_size`] returns `None`.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime};

use parking_lot::Mutex;

use asyn_rs::error::{AsynError, AsynResult, AsynStatus};
use asyn_rs::interfaces::InterfaceType;
use asyn_rs::interrupt::{InterruptManager, InterruptValue};
use asyn_rs::param::{ParamType, ParamValue};
use asyn_rs::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use asyn_rs::runtime::config::RuntimeConfig;
use asyn_rs::runtime::port::{PortRuntimeHandle, create_port_runtime};
use asyn_rs::user::AsynUser;

use epics_rs::base::server::recgbl::alarm_status;

use crate::ads::defs::{ADSIGRP_MEMORY_BYTE, ADSIGRP_SYM_VALBYHND, LOCAL_PORT_BASE};
use crate::ads::sumup::{self, SumEntry};
use crate::ads::{AdsClient, AdsError, AdsState, AdsType, AmsAddr, AmsNetId, NotificationSample};
use crate::convert::{self, WriteValue};
use crate::drvinfo::{DataSource, DrvInfo, DrvInfoDefaults, PlcAddress, TimeBase};
use crate::octet::{self, Command, OctetError};
use crate::time::{filetime_from_dwords, windows_to_system_time};

/// `epicsSevInvalid` — the severity every ADS failure raises.
const INVALID_ALARM: u16 = 3;
/// `epicsSevNone`.
const NO_ALARM_SEV: u16 = 0;

/// The AMS port of the first TC3 PLC runtime.
pub const DEFAULT_AMS_PORT: u16 = 851;

/// C `cyclicThread`'s `sampleTime` (adsAsynPortDriver.cpp:510).
const CYCLIC_PERIOD: Duration = Duration::from_millis(500);

/// C `BULKSIZ` (adsAsynPortDriver.h:224) — sub-requests per sum-up read, minus
/// the two timestamp entries every group starts with.
const BULK_GROUP_MAX: usize = 500;

/// C `updateParamInfoWithPLCInfo` (adsAsynPortDriver.cpp:1288): a variable
/// bigger than this is subscribed even when it asked for the bulk read, because
/// one such variable would fill the sum-up response on its own.
const BULK_MAX_VARIABLE_BYTES: u32 = 1024 * 1024;

/// The PLC symbols the bulk read timestamps its samples from
/// (C `adsFindBulkTimeStamp`, adsAsynPortDriver.cpp:1409-1411).
const TS_SYMBOL_HI: &str = "MAIN.fbSystemTime.timeHiDW";
const TS_SYMBOL_LO: &str = "MAIN.fbSystemTime.timeLoDW";

/// Everything `adsAsynPortDriverConfigure` takes.
#[derive(Debug, Clone)]
pub struct AdsConfig {
    pub port_name: String,
    /// PLC host name or IPv4 address (the AMS router listens on 48898).
    pub ip_addr: String,
    /// The PLC's AMS Net Id.
    pub remote_net_id: AmsNetId,
    /// Default AMS port for a drvInfo that carries no `ADSPORT=`.
    pub ams_port: u16,
    pub auto_connect: bool,
    pub default_sample_time_ms: f64,
    pub default_max_delay_time_ms: f64,
    pub ads_timeout_ms: u64,
    pub default_time_base: TimeBase,
}

impl AdsConfig {
    pub fn new(port_name: &str, ip_addr: &str, remote_net_id: AmsNetId, ams_port: u16) -> Self {
        Self {
            port_name: port_name.to_string(),
            ip_addr: ip_addr.to_string(),
            remote_net_id,
            ams_port,
            auto_connect: true,
            default_sample_time_ms: 100.0,
            default_max_delay_time_ms: 100.0,
            ads_timeout_ms: 5000,
            default_time_base: TimeBase::Plc,
        }
    }

    fn drvinfo_defaults(&self) -> DrvInfoDefaults {
        DrvInfoDefaults {
            ams_port: self.ams_port,
            sample_time_ms: self.default_sample_time_ms,
            max_delay_time_ms: self.default_max_delay_time_ms,
            time_base: self.default_time_base,
        }
    }

    fn timeout(&self) -> Duration {
        Duration::from_millis(self.ads_timeout_ms)
    }

    /// C constructor (adsAsynPortDriver.cpp:417-421): the bulk loop runs at
    /// 1 Hz unless the default sample time is slower than that.
    fn bulk_period(&self) -> Duration {
        let ms = self.default_sample_time_ms.max(1000.0);
        Duration::from_millis(ms as u64)
    }
}

/// Where one parameter's samples come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Feed {
    /// Not an input (write-only record), or the AMS state, which the supervisor
    /// pushes.
    None,
    /// The PLC pushes on change (`ADD_DEVICE_NOTIFICATION`).
    Notification,
    /// The bulk poller reads it (`ADSIGRP_SUMUP_READ`); `(group, slot)`.
    Bulk(usize, usize),
}

/// One asyn parameter: a PLC variable plus the last sample of it.
struct ParamState {
    info: DrvInfo,
    addr: i32,
    /// The interface the bound record reads this on. `None` for a port-level
    /// resolve (sync_io), where the value is fired untyped.
    iface: Option<InterfaceType>,
    param_type: ParamType,
    plc_type: AdsType,
    plc_size: u32,
    is_array: bool,
    /// `SYM_HNDBYNAME` handle; `None` for `.ADR.` and before the first connect.
    handle: Option<u32>,
    notification: Option<u32>,
    feed: Feed,
    /// Last sample, exactly as decoded for `param_type`.
    value: ParamValue,
    /// Last sample's raw PLC bytes (what an octet read of a `STRING` returns).
    raw: Vec<u8>,
    timestamp: SystemTime,
    alarm: (u16, u16),
    /// Set when the PLC info (size/type/handle) must be re-read: at creation,
    /// and again after every reconnect, because handles do not survive one.
    refresh_needed: bool,
}

impl ParamState {
    /// The absolute address a read/write of this parameter goes to.
    fn wire_address(&self) -> Option<(u32, u32)> {
        match &self.info.address {
            PlcAddress::Absolute {
                index_group,
                index_offset,
                ..
            } => Some((*index_group, *index_offset)),
            PlcAddress::Symbolic(_) => self.handle.map(|h| (ADSIGRP_SYM_VALBYHND, h)),
            PlcAddress::AmsPortState => None,
        }
    }

    fn symbol_name(&self) -> Option<&str> {
        match &self.info.address {
            PlcAddress::Symbolic(name) => Some(name),
            _ => None,
        }
    }
}

/// One sum-up read: up to `BULK_GROUP_MAX` variables on one AMS port, fetched
/// in a single `READ_WRITE`. Slots 0 and 1 are always the PLC clock.
struct BulkGroup {
    ams_port: u16,
    entries: Vec<SumEntry>,
    /// Reason for each slot; slots 0 and 1 hold `None` (the clock).
    reasons: Vec<Option<usize>>,
}

/// One AMS port on the PLC, and what we last knew about it.
struct AmsPortState {
    ams_port: u16,
    state: AdsState,
    connected: bool,
    /// The `.AMSPORTSTATE.` parameter, when a record bound one.
    state_param: Option<usize>,
    /// Handles of the PLC clock symbols, once resolved.
    ts_handles: Option<(u32, u32)>,
}

/// Driver state the actor thread and the background threads share.
///
/// The actor owns `PortDriverBase` (parameter names, lifecycle); everything the
/// notification reader, the bulk poller and the connection supervisor also touch
/// lives here.
pub struct AdsShared {
    cfg: AdsConfig,
    /// `None` while disconnected. Held in an `Arc` so a caller can clone it out
    /// and do its I/O without holding the lock.
    client: Mutex<Option<Arc<AdsClient>>>,
    params: Mutex<Vec<Option<ParamState>>>,
    /// Notification handle → reason. The PLC's push carries only the handle.
    notifications: Mutex<HashMap<u32, usize>>,
    bulk: Mutex<Vec<BulkGroup>>,
    ports: Mutex<Vec<AmsPortState>>,
    /// The ASCII command channel's reply buffer (reason 0).
    octet_buffer: Mutex<String>,
    interrupts: InterruptManager,
    running: AtomicBool,
    /// Wall time of the last completed bulk cycle, for `adsPollInfo`.
    bulk_elapsed: Mutex<Duration>,
}

impl AdsShared {
    fn client(&self) -> Result<Arc<AdsClient>, AdsError> {
        self.client.lock().clone().ok_or(AdsError::NotConnected)
    }

    fn is_connected(&self) -> bool {
        self.client
            .lock()
            .as_ref()
            .map(|c| c.is_connected())
            .unwrap_or(false)
    }

    /// Register an AMS port the first time a drvInfo names it
    /// (C `addNewAmsPortToList`).
    fn ensure_ams_port(&self, ams_port: u16) {
        let mut ports = self.ports.lock();
        if ports.iter().any(|p| p.ams_port == ams_port) {
            return;
        }
        ports.push(AmsPortState {
            ams_port,
            state: AdsState::Unknown,
            connected: false,
            state_param: None,
            ts_handles: None,
        });
    }

    /// Publish one sample: decode it for the bound record's interface, cache it,
    /// and fire the I/O Intr.
    ///
    /// `plc_time_raw` is the sample's PLC timestamp in FILETIME ticks; `0` means
    /// "the PLC did not stamp this one", which sends the parameter to the IOC
    /// clock exactly as C `refreshParamTime` does (adsAsynPortDriver.cpp:4176).
    fn publish(&self, reason: usize, data: &[u8], plc_time_raw: u64) {
        let fire = {
            let mut params = self.params.lock();
            let Some(p) = params.get_mut(reason).and_then(Option::as_mut) else {
                log::error!("ADS: sample for unknown parameter {reason}");
                return;
            };

            let decoded = if p.is_array {
                convert::decode_array(p.plc_type, data, p.param_type)
            } else {
                convert::decode_scalar(p.plc_type, data, p.param_type)
            };

            match decoded {
                Ok(value) => {
                    p.raw = data.to_vec();
                    p.value = value;
                    p.timestamp = match windows_to_system_time(plc_time_raw) {
                        Some(t) if p.info.time_base == TimeBase::Plc => t,
                        _ => SystemTime::now(),
                    };
                    p.alarm = (alarm_status::NO_ALARM, NO_ALARM_SEV);
                }
                Err(e) => {
                    log::error!("ADS: {}: {e}", p.info.raw);
                    p.alarm = (alarm_status::READ_ALARM, INVALID_ALARM);
                    p.timestamp = SystemTime::now();
                }
            }
            interrupt_for(reason, p)
        };
        if let Some(iv) = fire {
            self.interrupts.notify(iv);
        }
    }

    /// Raise an alarm on one parameter without a new value (C `setAlarmParam`).
    fn alarm(&self, reason: usize, status: u16, severity: u16) {
        let fire = {
            let mut params = self.params.lock();
            let Some(p) = params.get_mut(reason).and_then(Option::as_mut) else {
                return;
            };
            if p.alarm == (status, severity) {
                return;
            }
            p.alarm = (status, severity);
            p.timestamp = SystemTime::now();
            interrupt_for(reason, p)
        };
        if let Some(iv) = fire {
            self.interrupts.notify(iv);
        }
    }

    /// Alarm every parameter of one AMS port and mark it for refresh — the PLC
    /// dropped, so its symbol handles are gone (C `invalidateParams`).
    fn invalidate_ams_port(&self, ams_port: u16) {
        let reasons: Vec<usize> = {
            let mut params = self.params.lock();
            let mut hit = Vec::new();
            for (reason, slot) in params.iter_mut().enumerate() {
                let Some(p) = slot.as_mut() else { continue };
                if p.info.ams_port != ams_port || p.info.data_source != DataSource::Plc {
                    continue;
                }
                p.handle = None;
                p.notification = None;
                p.refresh_needed = true;
                hit.push(reason);
            }
            hit
        };
        // A dead connection's notification handles mean nothing on the next one.
        self.notifications
            .lock()
            .retain(|_, reason| !reasons.contains(reason));
        for reason in reasons {
            self.alarm(reason, alarm_status::COMM_ALARM, INVALID_ALARM);
        }
    }

    /// Re-resolve every parameter of one AMS port that needs it
    /// (C `refreshParams`).
    fn refresh_ams_port(&self, ams_port: u16) {
        let pending: Vec<usize> = self
            .params
            .lock()
            .iter()
            .enumerate()
            .filter_map(|(reason, slot)| {
                let p = slot.as_ref()?;
                (p.refresh_needed
                    && p.info.ams_port == ams_port
                    && p.info.data_source == DataSource::Plc)
                    .then_some(reason)
            })
            .collect();

        for reason in pending {
            if let Err(e) = self.refresh_param(reason) {
                log::warn!("ADS: refresh of parameter {reason} failed: {e}");
            }
        }
    }

    /// Read the PLC's own view of one variable (size, type, address), subscribe
    /// or enrol it in the bulk read, and take a first sample.
    ///
    /// C `updateParamInfoWithPLCInfo` (adsAsynPortDriver.cpp:1199).
    fn refresh_param(&self, reason: usize) -> Result<(), AdsError> {
        let client = self.client()?;

        let (ams_port, symbol, mut plc_type, mut plc_size, has_input, is_bulk) = {
            let params = self.params.lock();
            let p = param_ref(&params, reason)?;
            (
                p.info.ams_port,
                p.symbol_name().map(str::to_string),
                p.plc_type,
                p.plc_size,
                p.info.has_input,
                p.info.is_bulk_read,
            )
        };

        // Symbolic: the PLC tells us the type and size; `.ADR.` carries both.
        let mut handle = None;
        if let Some(name) = &symbol {
            let entry = client.get_symbol_info(ams_port, name)?;
            plc_type = entry.data_type;
            plc_size = entry.size;

            // Give the old handle back before asking for a new one, or the PLC
            // leaks one per refresh (C `adsReleaseSymbolicHandle`, :1279). C
            // zeroes the handle *before* the message that reports the failure to
            // release it (:3800-3806), so its log always reads `0xffffffff`.
            let stale = self.params.lock()[reason]
                .as_mut()
                .and_then(|p| p.handle.take());
            if let Some(stale) = stale
                && let Err(e) = client.release_symbol_handle(ams_port, stale)
            {
                log::debug!("ADS: releasing handle 0x{stale:x} of '{name}' failed: {e}");
            }
            handle = Some(client.get_symbol_handle(ams_port, name)?);
        }

        // C `updateParamInfoWithPLCInfo` (:1224-1246): a variable is an array
        // when it is larger than one element of its own type. STRING is the
        // exception — always an array of bytes, however short.
        let is_array = match plc_type {
            AdsType::String | AdsType::WString => true,
            AdsType::Void | AdsType::BigType | AdsType::Unknown(_) => false,
            other => other.element_size().is_some_and(|e| plc_size as usize > e),
        };

        let feed = {
            let mut params = self.params.lock();
            let p = param_mut(&mut params, reason)?;
            p.handle = handle;
            p.plc_type = plc_type;
            p.plc_size = plc_size;
            p.is_array = is_array;
            p.feed
        };

        // Feed it: a notification, unless the record asked for the bulk read and
        // the variable is small enough to share a sum-up response with 500 more.
        if has_input {
            if is_bulk && plc_size <= BULK_MAX_VARIABLE_BYTES {
                self.add_to_bulk(reason)?;
            } else {
                if let Feed::Notification = feed {
                    self.remove_notification(reason);
                }
                self.add_notification(reason)?;
            }
        }

        // First sample, so a record has a value before the first push.
        if has_input {
            let (group, offset, size) = {
                let params = self.params.lock();
                let p = param_ref(&params, reason)?;
                let (g, o) = p.wire_address().ok_or(AdsError::NotConnected)?;
                (g, o, p.plc_size)
            };
            let data = client.read(ams_port, group, offset, size)?;
            self.publish(reason, &data, 0);
        }

        if let Some(p) = self.params.lock().get_mut(reason).and_then(Option::as_mut) {
            p.refresh_needed = false;
        }
        Ok(())
    }

    fn add_notification(&self, reason: usize) -> Result<(), AdsError> {
        let client = self.client()?;
        let (ams_port, group, offset, size, cycle, delay) = {
            let params = self.params.lock();
            let p = param_ref(&params, reason)?;
            let (g, o) = p.wire_address().ok_or(AdsError::NotConnected)?;
            (
                p.info.ams_port,
                g,
                o,
                p.plc_size,
                Duration::from_secs_f64(p.info.sample_time_ms / 1000.0),
                Duration::from_secs_f64(p.info.max_delay_time_ms / 1000.0),
            )
        };
        let handle = client.add_notification(ams_port, group, offset, size, cycle, delay)?;
        self.notifications.lock().insert(handle, reason);
        let mut params = self.params.lock();
        let p = param_mut(&mut params, reason)?;
        p.notification = Some(handle);
        p.feed = Feed::Notification;
        Ok(())
    }

    /// Best-effort unsubscribe — a dead socket cannot carry the cancel, and the
    /// PLC drops its notifications with the connection anyway.
    fn remove_notification(&self, reason: usize) {
        let (ams_port, handle) = {
            let mut params = self.params.lock();
            let Ok(p) = param_mut(&mut params, reason) else {
                return;
            };
            let Some(handle) = p.notification.take() else {
                return;
            };
            p.feed = Feed::None;
            (p.info.ams_port, handle)
        };
        self.notifications.lock().remove(&handle);
        if let Ok(client) = self.client()
            && let Err(e) = client.del_notification(ams_port, handle)
        {
            log::debug!("ADS: cancelling notification 0x{handle:x} failed: {e}");
        }
    }

    /// Put one variable in a sum-up group, or update its slot if it already has
    /// one (C `adsAddToBulkRead`, adsAsynPortDriver.cpp:1341).
    ///
    /// A slot is never given up: the handle behind it changes on reconnect, the
    /// slot does not.
    fn add_to_bulk(&self, reason: usize) -> Result<(), AdsError> {
        let (ams_port, group, offset, size, existing) = {
            let params = self.params.lock();
            let p = param_ref(&params, reason)?;
            let (g, o) = p.wire_address().ok_or(AdsError::NotConnected)?;
            let existing = match p.feed {
                Feed::Bulk(gi, si) => Some((gi, si)),
                _ => None,
            };
            (p.info.ams_port, g, o, p.plc_size, existing)
        };

        let entry = SumEntry {
            index_group: group,
            index_offset: offset,
            size,
        };

        let mut groups = self.bulk.lock();
        if let Some((gi, si)) = existing {
            groups[gi].entries[si] = entry;
            return Ok(());
        }

        let gi = match groups
            .iter()
            .position(|g| g.ams_port == ams_port && g.entries.len() < BULK_GROUP_MAX)
        {
            Some(gi) => gi,
            None => {
                let ts = self.timestamp_entries(ams_port);
                groups.push(BulkGroup {
                    ams_port,
                    entries: ts.to_vec(),
                    reasons: vec![None, None],
                });
                groups.len() - 1
            }
        };
        let si = groups[gi].entries.len();
        groups[gi].entries.push(entry);
        groups[gi].reasons.push(Some(reason));
        drop(groups);

        let mut params = self.params.lock();
        param_mut(&mut params, reason)?.feed = Feed::Bulk(gi, si);
        Ok(())
    }

    /// The two slots every sum-up group starts with: the PLC's clock, high dword
    /// first (C `adsFindBulkTimeStamp`, adsAsynPortDriver.cpp:1400).
    ///
    /// A PLC without `MAIN.fbSystemTime` gets the `%M` fallback C uses, whose
    /// sub-requests are harmless and whose results the poller ignores — the
    /// samples then carry the IOC's clock.
    fn timestamp_entries(&self, ams_port: u16) -> [SumEntry; 2] {
        let mut handles = None;
        if let Ok(client) = self.client() {
            let hi = client.get_symbol_handle(ams_port, TS_SYMBOL_HI);
            let lo = client.get_symbol_handle(ams_port, TS_SYMBOL_LO);
            if let (Ok(hi), Ok(lo)) = (hi, lo) {
                handles = Some((hi, lo));
            }
        }

        if let Some(port) = self
            .ports
            .lock()
            .iter_mut()
            .find(|p| p.ams_port == ams_port)
        {
            port.ts_handles = handles;
        }

        match handles {
            Some((hi, lo)) => [
                SumEntry {
                    index_group: ADSIGRP_SYM_VALBYHND,
                    index_offset: hi,
                    size: 4,
                },
                SumEntry {
                    index_group: ADSIGRP_SYM_VALBYHND,
                    index_offset: lo,
                    size: 4,
                },
            ],
            None => {
                log::info!(
                    "ADS: AMS port {ams_port} has no {TS_SYMBOL_HI}/{TS_SYMBOL_LO}; \
                     bulk-read samples will carry the IOC timestamp"
                );
                [SumEntry {
                    index_group: ADSIGRP_MEMORY_BYTE,
                    index_offset: 0,
                    size: 4,
                }; 2]
            }
        }
    }

    /// One pass of the sum-up reads (C `bulkReadThread`, :617).
    fn bulk_cycle(&self) {
        let Ok(client) = self.client() else { return };

        // Snapshot: a drv_user_create on the actor thread may add a slot while
        // we are on the wire, and the response we are decoding predates it.
        let groups: Vec<(u16, Vec<SumEntry>, Vec<Option<usize>>)> = self
            .bulk
            .lock()
            .iter()
            .map(|g| (g.ams_port, g.entries.clone(), g.reasons.clone()))
            .collect();

        for (ams_port, entries, reasons) in groups {
            if entries.len() <= 2 {
                continue; // clock only
            }
            let resp = match client.sum_up_read(ams_port, &entries) {
                Ok(r) => r,
                Err(e) => {
                    log::warn!("ADS: sum-up read on AMS port {ams_port} failed: {e}");
                    continue;
                }
            };
            let (results, _layout) = match sumup::decode_response(&resp, &entries) {
                Ok(r) => r,
                Err(e) => {
                    log::warn!("ADS: sum-up response on AMS port {ams_port} is malformed: {e}");
                    continue;
                }
            };

            // Slots 0/1 are the PLC clock, high dword then low. Either failing
            // (an `%M` fallback group always does) leaves the samples on the IOC
            // clock.
            let plc_time = match (&results[0], &results[1]) {
                (Ok(hi), Ok(lo)) if hi.len() == 4 && lo.len() == 4 => filetime_from_dwords(
                    u32::from_le_bytes(lo[..4].try_into().unwrap()),
                    u32::from_le_bytes(hi[..4].try_into().unwrap()),
                ),
                _ => 0,
            };

            for (slot, result) in results.iter().enumerate().skip(2) {
                let Some(reason) = reasons[slot] else {
                    continue;
                };
                match result {
                    Ok(data) => self.publish(reason, data, plc_time),
                    Err(e) => {
                        log::warn!("ADS: bulk read of parameter {reason} failed: {e}");
                        self.alarm(reason, alarm_status::READ_ALARM, INVALID_ALARM);
                    }
                }
            }
        }
    }

    /// One sample pushed by the PLC (C `adsDataCallback`, :157).
    fn on_notification(&self, sample: NotificationSample) {
        let Some(&reason) = self.notifications.lock().get(&sample.handle) else {
            // A cancel and a sample in flight cross; C logs and drops it too.
            log::debug!("ADS: notification for unknown handle 0x{:x}", sample.handle);
            return;
        };
        self.publish(reason, &sample.data, sample.timestamp);
    }
}

/// The interrupt one parameter's current value fires, or `None` when it has
/// never held one (nothing to publish, and the record keeps its UDF).
fn interrupt_for(reason: usize, p: &ParamState) -> Option<InterruptValue> {
    if matches!(p.value, ParamValue::Undefined) {
        return None;
    }
    Some(InterruptValue {
        reason,
        addr: p.addr,
        value: p.value.clone(),
        timestamp: p.timestamp,
        uint32_changed_mask: 0,
        aux_status: if p.alarm.0 == alarm_status::NO_ALARM {
            AsynStatus::Success
        } else {
            AsynStatus::Error
        },
        alarm_status: p.alarm.0,
        alarm_severity: p.alarm.1,
        iface: p.iface,
    })
}

fn param_ref(params: &[Option<ParamState>], reason: usize) -> Result<&ParamState, AdsError> {
    params
        .get(reason)
        .and_then(Option::as_ref)
        .ok_or(AdsError::NotConnected)
}

fn param_mut(
    params: &mut [Option<ParamState>],
    reason: usize,
) -> Result<&mut ParamState, AdsError> {
    params
        .get_mut(reason)
        .and_then(Option::as_mut)
        .ok_or(AdsError::NotConnected)
}

/// The asyn parameter type a record's interface asks for.
///
/// C reaches the same table through the record's DTYP string
/// (`dtypStringToAsynType`, adsAsynPortDriverUtils.cpp:429): the interfaces it
/// leaves out — `asynUInt32Digital`, `asynEnum`, `asynGenericPointer` — have no
/// arm in the conversion matrix either, so a record binding one is refused here
/// rather than silently mistyped.
fn param_type_for(iface: InterfaceType) -> Option<ParamType> {
    Some(match iface {
        InterfaceType::Int32 => ParamType::Int32,
        InterfaceType::Int64 => ParamType::Int64,
        InterfaceType::Float64 => ParamType::Float64,
        InterfaceType::Octet => ParamType::Octet,
        InterfaceType::Int8Array => ParamType::Int8Array,
        InterfaceType::Int16Array => ParamType::Int16Array,
        InterfaceType::Int32Array => ParamType::Int32Array,
        InterfaceType::Int64Array => ParamType::Int64Array,
        InterfaceType::Float32Array => ParamType::Float32Array,
        InterfaceType::Float64Array => ParamType::Float64Array,
        _ => return None,
    })
}

fn asyn_err(msg: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: msg.into(),
    }
}

/// The TwinCAT ADS asyn port driver.
pub struct AdsPortDriver {
    base: PortDriverBase,
    shared: Arc<AdsShared>,
}

impl AdsPortDriver {
    pub fn new(cfg: AdsConfig) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            &cfg.port_name,
            1,
            PortFlags {
                multi_device: false,
                // Every parameter bind and every write goes to the PLC over TCP.
                can_block: true,
                destructible: true,
            },
        );

        // Reason 0 — the ASCII command channel (C constructor, :372).
        let default_access = base.create_param("Default access", ParamType::Octet)?;
        if default_access != 0 {
            return Err(asyn_err("the ASCII command channel must be reason 0"));
        }

        let shared = Arc::new(AdsShared {
            interrupts: InterruptManager::from_shared_state(base.interrupts.shared_state()),
            client: Mutex::new(None),
            params: Mutex::new(vec![None]),
            notifications: Mutex::new(HashMap::new()),
            bulk: Mutex::new(Vec::new()),
            ports: Mutex::new(Vec::new()),
            octet_buffer: Mutex::new(String::new()),
            running: AtomicBool::new(true),
            bulk_elapsed: Mutex::new(Duration::ZERO),
            cfg,
        });
        shared.ensure_ams_port(shared.cfg.ams_port);

        Ok(Self { base, shared })
    }

    pub fn shared(&self) -> Arc<AdsShared> {
        Arc::clone(&self.shared)
    }

    /// EPICS scalar → PLC. C `writeInt32` (:2539) / `writeInt64` / `writeFloat64`.
    fn write_scalar(&mut self, user: &AsynUser, value: WriteValue) -> AsynResult<()> {
        let reason = user.reason;
        let (source, ams_port, plc_type, plc_size, wire) = {
            let params = self.shared.params.lock();
            let p = params
                .get(reason)
                .and_then(Option::as_ref)
                .ok_or_else(|| asyn_err(format!("no ADS parameter for reason {reason}")))?;
            (
                p.info.data_source,
                p.info.ams_port,
                p.plc_type,
                p.plc_size,
                p.wire_address(),
            )
        };

        // `.AMSPORTSTATE.` is a command to the runtime, not a variable
        // (C `writeInt32`, :2556).
        if source == DataSource::AmsState {
            let state = match value {
                WriteValue::Int(v) => v as u16,
                WriteValue::Float(v) => v as u16,
            };
            let client = self.shared.client().map_err(|e| asyn_err(e.to_string()))?;
            return client.write_control(ams_port, state, 0, &[]).map_err(|e| {
                self.shared
                    .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
                asyn_err(format!("ADS write state: {e}"))
            });
        }

        let bytes = convert::encode_scalar(plc_type, value).map_err(|e| {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            asyn_err(e.to_string())
        })?;

        // C (:2657) refuses a write whose width is not the PLC variable's own.
        if bytes.len() != plc_size as usize {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            return Err(asyn_err(format!(
                "{}: {} is {} bytes in the PLC, the write is {}",
                self.shared.params.lock()[reason]
                    .as_ref()
                    .map(|p| p.info.raw.clone())
                    .unwrap_or_default(),
                plc_type.as_str(),
                plc_size,
                bytes.len()
            )));
        }

        let (group, offset) = wire.ok_or_else(|| {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            asyn_err("PLC variable has no address yet (not connected)")
        })?;
        let client = self.shared.client().map_err(|e| asyn_err(e.to_string()))?;
        client.write(ams_port, group, offset, &bytes).map_err(|e| {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            asyn_err(format!("ADS write: {e}"))
        })?;

        // The written value is the parameter's value now — the PLC will confirm
        // it on the next notification.
        self.shared.publish(reason, &bytes, 0);
        Ok(())
    }

    /// EPICS array → PLC. C `adsGenericArrayWrite` (:2912).
    ///
    /// `allowed` is the PLC type this asyn array flavour pairs with; C refuses
    /// any other, and so do we — a `waveform` of `SHORT` bound to an array of
    /// `LREAL` is a link error, not a conversion.
    fn write_array(
        &mut self,
        user: &AsynUser,
        allowed: &[AdsType],
        bytes: &[u8],
    ) -> AsynResult<()> {
        let reason = user.reason;
        let (ams_port, plc_type, plc_size, wire) = {
            let params = self.shared.params.lock();
            let p = params
                .get(reason)
                .and_then(Option::as_ref)
                .ok_or_else(|| asyn_err(format!("no ADS parameter for reason {reason}")))?;
            (p.info.ams_port, p.plc_type, p.plc_size, p.wire_address())
        };

        if !allowed.contains(&plc_type) {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            return Err(asyn_err(format!(
                "array write to a PLC {} is not supported",
                plc_type.as_str()
            )));
        }

        // C caps the write at the PLC variable's size (:3012).
        let n = bytes.len().min(plc_size as usize);
        let (group, offset) = wire.ok_or_else(|| {
            self.shared
                .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
            asyn_err("PLC variable has no address yet (not connected)")
        })?;
        let client = self.shared.client().map_err(|e| asyn_err(e.to_string()))?;
        client
            .write(ams_port, group, offset, &bytes[..n])
            .map_err(|e| {
                self.shared
                    .alarm(reason, alarm_status::WRITE_ALARM, INVALID_ALARM);
                asyn_err(format!("ADS write: {e}"))
            })?;

        self.shared.publish(reason, &bytes[..n], 0);
        Ok(())
    }

    /// Serve an array read from the last sample (C `adsGenericArrayRead`, :2845).
    fn read_array<T: Copy>(
        &self,
        user: &AsynUser,
        buf: &mut [T],
        extract: impl Fn(&ParamValue) -> Option<&[T]>,
    ) -> AsynResult<usize> {
        let params = self.shared.params.lock();
        let p = params
            .get(user.reason)
            .and_then(Option::as_ref)
            .ok_or_else(|| asyn_err(format!("no ADS parameter for reason {}", user.reason)))?;
        let src = extract(&p.value).ok_or_else(|| {
            asyn_err(format!(
                "{}: no sample of the requested array type yet",
                p.info.raw
            ))
        })?;
        let n = src.len().min(buf.len());
        buf[..n].copy_from_slice(&src[..n]);
        Ok(n)
    }

    /// One ASCII command from the motor record / StreamDevice.
    ///
    /// The reply text is appended to the buffer a following `readOctet` drains,
    /// exactly as C's `octetCmdBuf_printf` does; a failing command answers
    /// `Error: <reason>` and the line keeps going (C `octetCmdHandleInputLine`,
    /// :2134).
    fn octet_command(&mut self, cmd: &str) -> Result<String, AdsError> {
        let parsed = match octet::parse_command(cmd, self.shared.cfg.ams_port) {
            Ok(c) => c,
            Err(e) => return Ok(format!("Error: {e}")),
        };

        let client = self.shared.client()?;
        let reply = match parsed {
            // C answers `ads;stv1` when it was built with DUT_AxisStatus support,
            // which this port always has (see `octet::axis_status_to_ascii`).
            Command::Features => "ads;stv1".to_string(),
            Command::Bad => "Error: Bad command".to_string(),
            Command::ReadSymbol { ams_port, name } => {
                let info = client.get_symbol_info(ams_port, &name)?;
                let data = client.read(ams_port, info.index_group, info.index_offset, info.size)?;
                octet_reply(octet::binary_to_ascii(
                    &data,
                    info.data_type,
                    info.size as usize,
                    &name,
                    &info.type_name,
                ))
            }
            Command::WriteSymbol {
                ams_port,
                name,
                value,
            } => {
                let info = client.get_symbol_info(ams_port, &name)?;
                match octet::ascii_to_binary(&value, info.data_type, info.size as usize) {
                    Ok(bytes) => {
                        let n = bytes.len().min(info.size as usize);
                        client.write(ams_port, info.index_group, info.index_offset, &bytes[..n])?;
                        "OK".to_string()
                    }
                    Err(e) => format!("Error: {e}"),
                }
            }
            Command::ReadAdr {
                ams_port,
                index_group,
                index_offset,
                length,
                data_type,
            } => {
                let data = client.read(ams_port, index_group, index_offset, length)?;
                octet_reply(octet::binary_to_ascii(
                    &data,
                    data_type,
                    length as usize,
                    "",
                    "",
                ))
            }
            Command::WriteAdr {
                ams_port,
                index_group,
                index_offset,
                length,
                data_type,
                value,
            } => match octet::ascii_to_binary(&value, data_type, length as usize) {
                Ok(bytes) => {
                    let n = bytes.len().min(length as usize);
                    client.write(ams_port, index_group, index_offset, &bytes[..n])?;
                    "OK".to_string()
                }
                Err(e) => format!("Error: {e}"),
            },
        };
        Ok(reply)
    }
}

fn octet_reply(r: Result<String, OctetError>) -> String {
    match r {
        Ok(text) => text,
        Err(e) => format!("Error: {e}"),
    }
}

impl PortDriver for AdsPortDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    /// Bind one record's `drvInfo` (C `drvUserCreate`, :1050).
    ///
    /// The record's asyn interface decides the parameter's type — that is what
    /// C reads out of the DTYP field by scanning the static database. When the
    /// request carries no interface (a port-level `sync_io` resolve), the PLC's
    /// own type decides, which is C's `asynParamNotDefined` default with the
    /// guesswork removed.
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        if let Some(reason) = self.base.find_param(&req.drv_info) {
            return Ok(DrvUserInfo::from_reason(reason));
        }

        let info = DrvInfo::parse(&req.drv_info, self.shared.cfg.drvinfo_defaults())
            .map_err(|e| asyn_err(format!("drvInfo '{}': {e}", req.drv_info)))?;
        self.shared.ensure_ams_port(info.ams_port);

        // What the record will read the parameter as.
        let param_type = match req.iface {
            Some(iface) => param_type_for(iface).ok_or_else(|| {
                asyn_err(format!(
                    "drvInfo '{}': asyn interface {iface:?} has no PLC conversion",
                    req.drv_info
                ))
            })?,
            None => self.natural_param_type(&info)?,
        };

        let reason = self.base.create_param(&req.drv_info, param_type)?;

        let state = ParamState {
            addr: req.addr,
            iface: req.iface,
            param_type,
            // A symbolic address has no type until the PLC tells us; VOID is the
            // "nothing readable here yet" placeholder `refresh_param` overwrites.
            plc_type: info
                .declared_type()
                .map(|(t, _)| t)
                .unwrap_or(AdsType::Void),
            plc_size: info.declared_type().map(|(_, s)| s).unwrap_or(0),
            is_array: false,
            handle: None,
            notification: None,
            feed: Feed::None,
            value: ParamValue::Undefined,
            raw: Vec::new(),
            timestamp: SystemTime::now(),
            alarm: (alarm_status::NO_ALARM, NO_ALARM_SEV),
            refresh_needed: info.data_source == DataSource::Plc,
            info,
        };

        {
            let mut params = self.shared.params.lock();
            if params.len() <= reason {
                params.resize_with(reason + 1, || None);
            }
            params[reason] = Some(state);
        }

        // The AMS state is the driver's own; the supervisor pushes it.
        let source = self.shared.params.lock()[reason]
            .as_ref()
            .map(|p| p.info.data_source)
            .unwrap_or(DataSource::Plc);
        if source == DataSource::AmsState {
            let ams_port = self.shared.params.lock()[reason]
                .as_ref()
                .map(|p| p.info.ams_port)
                .unwrap_or(self.shared.cfg.ams_port);
            if let Some(port) = self
                .shared
                .ports
                .lock()
                .iter_mut()
                .find(|p| p.ams_port == ams_port)
            {
                port.state_param = Some(reason);
            }
            return Ok(DrvUserInfo::from_reason(reason));
        }

        // Resolve it against the PLC now if we can; the supervisor picks up
        // whatever is still `refresh_needed` when the link comes up.
        if self.shared.is_connected()
            && let Err(e) = self.shared.refresh_param(reason)
        {
            log::warn!("ADS: '{}' not resolved yet: {e}", req.drv_info);
        }

        Ok(DrvUserInfo::from_reason(reason))
    }

    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let params = self.shared.params.lock();
        let p = param_ref(&params, user.reason).map_err(|e| asyn_err(e.to_string()))?;
        match p.value {
            ParamValue::Int32(v) => Ok(v),
            _ => Err(asyn_err(format!("{}: no int32 sample yet", p.info.raw))),
        }
    }

    fn read_int64(&mut self, user: &AsynUser) -> AsynResult<i64> {
        let params = self.shared.params.lock();
        let p = param_ref(&params, user.reason).map_err(|e| asyn_err(e.to_string()))?;
        match p.value {
            ParamValue::Int64(v) => Ok(v),
            _ => Err(asyn_err(format!("{}: no int64 sample yet", p.info.raw))),
        }
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let params = self.shared.params.lock();
        let p = param_ref(&params, user.reason).map_err(|e| asyn_err(e.to_string()))?;
        match p.value {
            ParamValue::Float64(v) => Ok(v),
            _ => Err(asyn_err(format!("{}: no float64 sample yet", p.info.raw))),
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.write_scalar(user, WriteValue::Int(value as i64))
    }

    fn write_int64(&mut self, user: &mut AsynUser, value: i64) -> AsynResult<()> {
        self.write_scalar(user, WriteValue::Int(value))
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.write_scalar(user, WriteValue::Float(value))
    }

    /// Reason 0 drains the ASCII command channel's reply buffer; a `STRING`
    /// parameter returns its last sample (C `readOctet`, :1919).
    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        if user.reason != 0 {
            let params = self.shared.params.lock();
            let p = param_ref(&params, user.reason).map_err(|e| asyn_err(e.to_string()))?;
            let text = match &p.value {
                ParamValue::Octet(s) => s.as_bytes(),
                _ => return Err(asyn_err(format!("{}: no string sample yet", p.info.raw))),
            };
            let n = text.len().min(buf.len());
            buf[..n].copy_from_slice(&text[..n]);
            return Ok(n);
        }

        let mut reply = self.shared.octet_buffer.lock();
        let bytes = reply.as_bytes();
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        // C `octetCMDreadIt` (:1985) removes exactly what it handed out.
        *reply = String::from_utf8_lossy(&bytes[n..]).into_owned();
        Ok(n)
    }

    /// Reason 0 is the ASCII command channel; a `STRING` parameter is written to
    /// the PLC, NUL-padded to its declared size (C `writeOctet`, :2033).
    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        if user.reason != 0 {
            let (plc_size, plc_type) = {
                let params = self.shared.params.lock();
                let p = param_ref(&params, user.reason).map_err(|e| asyn_err(e.to_string()))?;
                (p.plc_size as usize, p.plc_type)
            };
            if plc_type != AdsType::String {
                return Err(asyn_err(format!(
                    "octet write to a PLC {} is not supported",
                    plc_type.as_str()
                )));
            }
            // The PLC's STRING slot is NUL-terminated: keep the last byte for it.
            let keep = plc_size.saturating_sub(1).min(data.len());
            let mut buf = vec![0u8; plc_size];
            buf[..keep].copy_from_slice(&data[..keep]);
            self.write_array(user, &[AdsType::String], &buf)?;
            return Ok(keep);
        }

        let line = String::from_utf8_lossy(data);
        // The terminator is echoed back after the replies, as C does (:2118).
        let body = line.trim_end_matches('\n');
        let had_lf = body.len() != line.len();
        let body = body.trim_end_matches('\r');
        let had_cr = body.len() != line.trim_end_matches('\n').len();

        let mut out = String::new();
        for (cmd, sep) in octet::split_commands(body) {
            match self.octet_command(&cmd) {
                Ok(reply) => out.push_str(&reply),
                // A dead link is an asyn error, not a reply: C returns asynError
                // for every code at or above ADSERR_CLIENT_ERROR (:2085).
                Err(e) => return Err(asyn_err(format!("ADS: {e}"))),
            }
            out.push_str(sep);
        }
        if had_cr {
            out.push('\r');
        }
        if had_lf {
            out.push('\n');
        }
        self.shared.octet_buffer.lock().push_str(&out);
        Ok(data.len())
    }

    fn read_int8_array(&mut self, user: &AsynUser, buf: &mut [i8]) -> AsynResult<usize> {
        self.read_array(user, buf, |v| match v {
            ParamValue::Int8Array(a) => Some(a),
            _ => None,
        })
    }

    fn write_int8_array(&mut self, user: &AsynUser, data: &[i8]) -> AsynResult<()> {
        let bytes: Vec<u8> = data.iter().map(|&v| v as u8).collect();
        // C `writeInt8Array` (:3017) accepts an int8 array against a PLC
        // ARRAY OF SINT, a STRING or an ARRAY OF BOOL.
        self.write_array(
            user,
            &[AdsType::Int8, AdsType::String, AdsType::Bit],
            &bytes,
        )
    }

    fn read_int16_array(&mut self, user: &AsynUser, buf: &mut [i16]) -> AsynResult<usize> {
        self.read_array(user, buf, |v| match v {
            ParamValue::Int16Array(a) => Some(a),
            _ => None,
        })
    }

    fn write_int16_array(&mut self, user: &AsynUser, data: &[i16]) -> AsynResult<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|v| v.to_le_bytes()).collect();
        self.write_array(user, &[AdsType::Int16], &bytes)
    }

    fn read_int32_array(&mut self, user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        self.read_array(user, buf, |v| match v {
            ParamValue::Int32Array(a) => Some(a),
            _ => None,
        })
    }

    fn write_int32_array(&mut self, user: &AsynUser, data: &[i32]) -> AsynResult<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|v| v.to_le_bytes()).collect();
        self.write_array(user, &[AdsType::Int32], &bytes)
    }

    fn read_float32_array(&mut self, user: &AsynUser, buf: &mut [f32]) -> AsynResult<usize> {
        self.read_array(user, buf, |v| match v {
            ParamValue::Float32Array(a) => Some(a),
            _ => None,
        })
    }

    fn write_float32_array(&mut self, user: &AsynUser, data: &[f32]) -> AsynResult<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|v| v.to_le_bytes()).collect();
        self.write_array(user, &[AdsType::Real32], &bytes)
    }

    fn read_float64_array(&mut self, user: &AsynUser, buf: &mut [f64]) -> AsynResult<usize> {
        self.read_array(user, buf, |v| match v {
            ParamValue::Float64Array(a) => Some(a),
            _ => None,
        })
    }

    fn write_float64_array(&mut self, user: &AsynUser, data: &[f64]) -> AsynResult<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|v| v.to_le_bytes()).collect();
        self.write_array(user, &[AdsType::Real64], &bytes)
    }

    fn report(&self, _level: i32) {
        let cfg = &self.shared.cfg;
        eprintln!(
            "twincat-ads port {}: PLC {} ({}), default AMS port {}, {}",
            cfg.port_name,
            cfg.ip_addr,
            cfg.remote_net_id,
            cfg.ams_port,
            if self.shared.is_connected() {
                "connected"
            } else {
                "disconnected"
            }
        );
        for port in self.shared.ports.lock().iter() {
            eprintln!(
                "  AMS port {}: {} ({})",
                port.ams_port,
                port.state.as_str(),
                if port.connected {
                    "connected"
                } else {
                    "disconnected"
                }
            );
        }
        for (reason, slot) in self.shared.params.lock().iter().enumerate() {
            let Some(p) = slot else { continue };
            eprintln!(
                "  [{reason}] {} — PLC {} ({} bytes){}",
                p.info.raw,
                p.plc_type.as_str(),
                p.plc_size,
                match p.feed {
                    Feed::Notification => ", notification",
                    Feed::Bulk(_, _) => ", bulk read",
                    Feed::None => "",
                }
            );
        }
    }
}

impl AdsPortDriver {
    /// The parameter type a bind with no interface gets: the PLC's own.
    ///
    /// `.ADR.` carries the type in the link, so it needs no PLC round trip;
    /// a symbol needs `SYM_INFOBYNAMEEX`, which needs the PLC. Rather than
    /// guessing a type that would then silently mis-decode every sample, an
    /// unresolvable bind is refused with the reason.
    fn natural_param_type(&mut self, info: &DrvInfo) -> AsynResult<ParamType> {
        if info.data_source == DataSource::AmsState {
            return Ok(ParamType::Int32);
        }

        let (plc_type, plc_size) = match info.declared_type() {
            Some(t) => t,
            None => {
                let name = match &info.address {
                    PlcAddress::Symbolic(n) => n.clone(),
                    _ => return Err(asyn_err("drvInfo has no PLC address")),
                };
                let client = self.shared.client().map_err(|_| {
                    asyn_err(format!(
                        "'{}' binds no asyn interface (no DTYP) and the PLC is not \
                         connected, so its type cannot be read from the symbol table",
                        info.raw
                    ))
                })?;
                let entry = client
                    .get_symbol_info(info.ams_port, &name)
                    .map_err(|e| asyn_err(format!("symbol info for '{name}': {e}")))?;
                (entry.data_type, entry.size)
            }
        };

        let is_array = match plc_type {
            AdsType::String | AdsType::WString => true,
            AdsType::Void | AdsType::BigType | AdsType::Unknown(_) => false,
            other => other.element_size().is_some_and(|e| plc_size as usize > e),
        };

        convert::natural_param_type(plc_type, is_array).ok_or_else(|| {
            asyn_err(format!(
                "'{}': PLC type {} has no asyn parameter type",
                info.raw,
                plc_type.as_str()
            ))
        })
    }
}

/// A running twincat-ads port: the actor plus its two background threads.
pub struct AdsRuntime {
    runtime_handle: PortRuntimeHandle,
    shared: Arc<AdsShared>,
    threads: Vec<JoinHandle<()>>,
}

impl AdsRuntime {
    pub fn port_handle(&self) -> &asyn_rs::port_handle::PortHandle {
        self.runtime_handle.port_handle()
    }

    pub fn shared(&self) -> &Arc<AdsShared> {
        &self.shared
    }
}

impl Drop for AdsRuntime {
    fn drop(&mut self) {
        self.shared.running.store(false, Ordering::SeqCst);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

/// Open the port: create the driver, start the supervisor and the bulk poller,
/// and try the first connection.
///
/// C's constructor (adsAsynPortDriver.cpp:465-479) loops on `connect()` until it
/// succeeds, so an IOC whose PLC is off never finishes `iocInit`. Here a failed
/// first connect is logged and left to the supervisor: the IOC starts, the
/// records sit in COMM/INVALID, and they come alive when the PLC does.
pub fn create_ads_port(cfg: AdsConfig) -> AsynResult<AdsRuntime> {
    let driver = AdsPortDriver::new(cfg)?;
    let shared = driver.shared();

    let (runtime_handle, _actor) = create_port_runtime(driver, RuntimeConfig::default());

    if let Err(e) = connect(&shared) {
        log::warn!(
            "ADS: first connection to {} failed ({e}); the supervisor will retry",
            shared.cfg.ip_addr
        );
    }

    let cyclic = {
        let shared = Arc::clone(&shared);
        thread::Builder::new()
            .name("ads-cyclic".into())
            .spawn(move || cyclic_thread(&shared))
            .map_err(|e| asyn_err(format!("cannot start the ADS supervisor: {e}")))?
    };
    let bulk = {
        let shared = Arc::clone(&shared);
        thread::Builder::new()
            .name("ads-bulk".into())
            .spawn(move || bulk_thread(&shared))
            .map_err(|e| asyn_err(format!("cannot start the ADS bulk reader: {e}")))?
    };

    register_port(&shared);

    Ok(AdsRuntime {
        runtime_handle,
        shared,
        threads: vec![cyclic, bulk],
    })
}

/// Open the TCP connection (C `adsConnect`, :3651).
fn connect(shared: &Arc<AdsShared>) -> Result<(), AdsError> {
    let notify_target = Arc::downgrade(shared);
    let on_notification = Arc::new(move |sample: NotificationSample| {
        if let Some(shared) = notify_target.upgrade() {
            shared.on_notification(sample);
        }
    });
    let on_disconnect = Arc::new(|| log::warn!("ADS: the PLC closed the connection"));

    let local = AmsAddr {
        // Zero = derive it from our own IP, as Beckhoff's router does when
        // `adsSetLocalAddress` was never called.
        net_id: local_net_id(),
        port: LOCAL_PORT_BASE,
    };

    let client = AdsClient::connect(
        &shared.cfg.ip_addr,
        local,
        shared.cfg.remote_net_id,
        shared.cfg.timeout(),
        on_notification,
        on_disconnect,
    )?;
    log::info!(
        "ADS: connected to {} ({}) as {}",
        shared.cfg.ip_addr,
        shared.cfg.remote_net_id,
        client.local_addr().net_id
    );
    *shared.client.lock() = Some(Arc::new(client));
    Ok(())
}

/// Drop the connection and alarm everything that fed off it.
fn disconnect(shared: &Arc<AdsShared>) {
    if shared.client.lock().take().is_none() {
        return;
    }
    // Handles and subscriptions died with the socket; drop the bookkeeping so a
    // reconnect re-resolves rather than reusing a handle the PLC forgot.
    shared.notifications.lock().clear();
    shared.bulk.lock().clear();
    let ams_ports: Vec<u16> = shared.ports.lock().iter().map(|p| p.ams_port).collect();
    for ams_port in ams_ports {
        shared.invalidate_ams_port(ams_port);
    }
    for port in shared.ports.lock().iter_mut() {
        port.connected = false;
        port.state = AdsState::Invalid;
        port.ts_handles = None;
    }
}

/// Supervise the connection (C `cyclicThread`, :508).
///
/// C `exit(-1)`s here when a port that was connected drops (:552), taking the
/// IOC — and every other device on it — down with one PLC reboot. This
/// reconnects instead.
fn cyclic_thread(shared: &Arc<AdsShared>) {
    while shared.running.load(Ordering::SeqCst) {
        thread::sleep(CYCLIC_PERIOD);
        if !shared.running.load(Ordering::SeqCst) {
            break;
        }

        if !shared.is_connected() {
            disconnect(shared);
            if !shared.cfg.auto_connect {
                continue;
            }
            if let Err(e) = connect(shared) {
                log::debug!("ADS: reconnect to {} failed: {e}", shared.cfg.ip_addr);
                continue;
            }
        }

        let Ok(client) = shared.client() else {
            continue;
        };
        let ams_ports: Vec<u16> = shared.ports.lock().iter().map(|p| p.ams_port).collect();
        let mut any_connected = false;

        for ams_port in ams_ports {
            let (state, connected) = match client.read_state(ams_port) {
                Ok((state, _device)) => (state, state == AdsState::Run),
                Err(e) => {
                    log::debug!("ADS: read state of AMS port {ams_port} failed: {e}");
                    (AdsState::Invalid, false)
                }
            };
            any_connected |= connected;

            let (was_connected, state_changed, state_param) = {
                let mut ports = shared.ports.lock();
                let Some(port) = ports.iter_mut().find(|p| p.ams_port == ams_port) else {
                    continue;
                };
                let was = port.connected;
                let changed = port.state != state;
                port.connected = connected;
                port.state = state;
                (was, changed, port.state_param)
            };

            // `.AMSPORTSTATE.` records see the runtime's state, whatever it is.
            if state_changed {
                if let Some(reason) = state_param {
                    shared.publish(reason, &(state as u16).to_le_bytes(), 0);
                }
                log::info!("ADS: AMS port {ams_port} is now {}", state.as_str());
            }

            if was_connected && !connected {
                shared.invalidate_ams_port(ams_port);
            }
            if connected {
                shared.refresh_ams_port(ams_port);
            }
        }

        // Not one runtime answered: the socket is up but the PLC behind it is
        // not talking. Drop it so the next pass reconnects from scratch.
        if !any_connected {
            disconnect(shared);
        }
    }
}

/// Poll every sum-up group (C `bulkReadThread`, :617).
fn bulk_thread(shared: &Arc<AdsShared>) {
    let period = shared.cfg.bulk_period();
    while shared.running.load(Ordering::SeqCst) {
        let start = Instant::now();
        shared.bulk_cycle();
        let elapsed = start.elapsed();
        *shared.bulk_elapsed.lock() = elapsed;

        // Sleep the remainder of the period, in slices, so a stop is prompt.
        let mut left = period.saturating_sub(elapsed);
        while left > Duration::ZERO && shared.running.load(Ordering::SeqCst) {
            let slice = left.min(CYCLIC_PERIOD);
            thread::sleep(slice);
            left -= slice;
        }
    }
}

// --- iocsh support ---------------------------------------------------------

/// The local AMS Net Id `adsSetLocalAddress` set, if any. Zero means "derive it
/// from the socket", which is what Beckhoff's router does.
static LOCAL_NET_ID: Mutex<AmsNetId> = Mutex::new(AmsNetId([0; 6]));

/// Every port created so far, for `adsPollInfo` (C keeps one global
/// `adsAsynPortObj`; this driver allows several PLCs in one IOC).
static PORTS: Mutex<Vec<(String, Arc<AdsShared>)>> = Mutex::new(Vec::new());

/// iocsh `adsSetLocalAddress` — the AMS Net Id this IOC answers to.
pub fn set_local_address(net_id: AmsNetId) {
    *LOCAL_NET_ID.lock() = net_id;
}

fn local_net_id() -> AmsNetId {
    *LOCAL_NET_ID.lock()
}

fn register_port(shared: &Arc<AdsShared>) {
    PORTS
        .lock()
        .push((shared.cfg.port_name.clone(), Arc::clone(shared)));
}

/// iocsh `adsPollInfo` — what the bulk reader is polling (C `poll_info`, :1307).
///
/// `filter` is matched against the PLC address; empty prints every variable.
pub fn poll_info(filter: &str) {
    for (port_name, shared) in PORTS.lock().iter() {
        let period = shared.cfg.bulk_period().as_secs_f64();
        let elapsed = shared.bulk_elapsed.lock().as_secs_f64();
        println!("{port_name}: bulk read loop: period {period:.3} s, last cycle {elapsed:.3} s");

        let params = shared.params.lock();
        for (gi, group) in shared.bulk.lock().iter().enumerate() {
            println!(
                "  bulk read #{gi} (AMS port {}, {} variables):",
                group.ams_port,
                group.entries.len().saturating_sub(2)
            );
            if filter.is_empty() {
                println!(
                    "      0: {TS_SYMBOL_HI} (G=0x{:x}, O=0x{:x}, S={})",
                    group.entries[0].index_group,
                    group.entries[0].index_offset,
                    group.entries[0].size
                );
                println!(
                    "      1: {TS_SYMBOL_LO} (G=0x{:x}, O=0x{:x}, S={})",
                    group.entries[1].index_group,
                    group.entries[1].index_offset,
                    group.entries[1].size
                );
            }
            for (slot, reason) in group.reasons.iter().enumerate().skip(2) {
                let Some(reason) = reason else { continue };
                let Some(p) = params.get(*reason).and_then(Option::as_ref) else {
                    continue;
                };
                if !filter.is_empty() && !p.info.address_str.contains(filter) {
                    continue;
                }
                let e = &group.entries[slot];
                println!(
                    "  {slot:5}: {} (G=0x{:x}, O=0x{:x}, S={})",
                    p.info.address_str, e.index_group, e.index_offset, e.size
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use asyn_rs::interrupt::InterruptFilter;

    use super::*;

    fn driver() -> AdsPortDriver {
        let cfg = AdsConfig::new("ADS1", "10.0.0.1", AmsNetId([10, 0, 0, 1, 1, 1]), 851);
        AdsPortDriver::new(cfg).unwrap()
    }

    /// Bind one record. `.ADR.` links carry the PLC type and size in the link
    /// itself, so they resolve with no PLC on the other end.
    fn bind(d: &mut AdsPortDriver, drv_info: &str, iface: InterfaceType) -> usize {
        d.drv_user_create(&DrvUserRequest::new(drv_info, 0).with_iface(iface))
            .unwrap()
            .reason
    }

    fn user(reason: usize) -> AsynUser {
        AsynUser {
            reason,
            ..Default::default()
        }
    }

    #[test]
    fn an_adr_link_carries_its_own_plc_type_and_size() {
        let mut d = driver();
        // 16#4020 = %M memory, 4 bytes, type 5 = REAL64.
        let reason = bind(
            &mut d,
            "ADSPORT=851/.ADR.16#4020,16#0,8,5?",
            InterfaceType::Float64,
        );
        let params = d.shared.params.lock();
        let p = params[reason].as_ref().unwrap();
        assert_eq!(p.plc_type, AdsType::Real64);
        assert_eq!(p.plc_size, 8);
        assert_eq!(p.param_type, ParamType::Float64);
        assert!(p.info.has_input);
    }

    #[test]
    fn a_sample_lands_in_the_cache_and_fires_one_typed_interrupt() {
        let mut d = driver();
        let reason = bind(
            &mut d,
            "ADSPORT=851/.ADR.16#4020,16#0,8,5?",
            InterfaceType::Float64,
        );

        let (tx, rx) = mpsc::channel();
        let _sub = d.shared.interrupts.register_sync_callback(
            InterruptFilter {
                reason: Some(reason),
                ..Default::default()
            },
            move |iv| {
                let _ = tx.send((iv.value.clone(), iv.iface, iv.alarm_severity));
            },
        );

        d.shared.publish(reason, &2.5f64.to_le_bytes(), 0);

        let (value, iface, severity) = rx.try_recv().unwrap();
        assert!(matches!(value, ParamValue::Float64(v) if v == 2.5));
        // The interrupt is tagged with the record's own interface, so a second
        // record on another interface cannot pick this sample up.
        assert_eq!(iface, Some(InterfaceType::Float64));
        assert_eq!(severity, NO_ALARM_SEV);
        assert_eq!(d.read_float64(&user(reason)).unwrap(), 2.5);
    }

    #[test]
    fn a_sample_of_the_wrong_width_alarms_rather_than_decoding_garbage() {
        let mut d = driver();
        let reason = bind(
            &mut d,
            "ADSPORT=851/.ADR.16#4020,16#0,8,5?",
            InterfaceType::Float64,
        );
        d.shared.publish(reason, &[1, 2, 3], 0); // 3 bytes for an 8-byte LREAL

        let params = d.shared.params.lock();
        let p = params[reason].as_ref().unwrap();
        assert_eq!(p.alarm, (alarm_status::READ_ALARM, INVALID_ALARM));
        assert!(matches!(p.value, ParamValue::Undefined));
    }

    #[test]
    fn timebase_plc_stamps_the_sample_with_the_plc_clock() {
        let mut d = driver();
        let plc = bind(
            &mut d,
            "TIMEBASE=PLC/.ADR.16#4020,16#0,4,3?",
            InterfaceType::Int32,
        );
        let epics = bind(
            &mut d,
            "TIMEBASE=EPICS/.ADR.16#4020,16#0,4,3?",
            InterfaceType::Int32,
        );

        // 2020-01-01T00:00:00Z in FILETIME ticks.
        let ticks =
            (1_577_836_800u64 + crate::time::SEC_TO_UNIX_EPOCH) * crate::time::WINDOWS_TICK_PER_SEC;
        let now = SystemTime::now();
        d.shared.publish(plc, &7i32.to_le_bytes(), ticks);
        d.shared.publish(epics, &7i32.to_le_bytes(), ticks);

        let params = d.shared.params.lock();
        assert_eq!(
            params[plc].as_ref().unwrap().timestamp,
            windows_to_system_time(ticks).unwrap()
        );
        // The EPICS-based one ignores the PLC's stamp and takes the IOC clock.
        assert!(params[epics].as_ref().unwrap().timestamp >= now);
    }

    #[test]
    fn an_array_sample_is_served_element_wise_not_byte_wise() {
        let mut d = driver();
        // 5 REAL64 = 40 bytes. C `fireCallbacks` (:4790) hands the *byte* count
        // to doCallbacksFloat64Array as the element count; this publishes 5.
        let reason = bind(
            &mut d,
            ".ADR.16#4020,16#0,40,5?",
            InterfaceType::Float64Array,
        );
        let bytes: Vec<u8> = (0..5).flat_map(|i| (i as f64).to_le_bytes()).collect();
        {
            let mut params = d.shared.params.lock();
            params[reason].as_mut().unwrap().is_array = true;
        }
        d.shared.publish(reason, &bytes, 0);

        let mut buf = [0f64; 8];
        let n = d.read_float64_array(&user(reason), &mut buf).unwrap();
        assert_eq!(n, 5);
        assert_eq!(&buf[..5], &[0.0, 1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn the_command_channel_hands_out_its_reply_once() {
        let mut d = driver();
        d.shared.octet_buffer.lock().push_str("1.500000;OK");

        let mut buf = [0u8; 4];
        let n = d.read_octet(&user(0), &mut buf).unwrap();
        assert_eq!(&buf[..n], b"1.50");
        // What was handed out is gone; the rest waits for the next read.
        let mut rest = [0u8; 32];
        let n = d.read_octet(&user(0), &mut rest).unwrap();
        assert_eq!(&rest[..n], b"0000;OK");
        assert_eq!(d.read_octet(&user(0), &mut rest).unwrap(), 0);
    }

    #[test]
    fn the_ams_port_state_is_a_driver_parameter_not_a_plc_variable() {
        let mut d = driver();
        let reason = bind(&mut d, "ADSPORT=852/.AMSPORTSTATE.?", InterfaceType::Int32);
        {
            let params = d.shared.params.lock();
            let p = params[reason].as_ref().unwrap();
            assert_eq!(p.info.data_source, DataSource::AmsState);
            // Nothing to resolve against the PLC — it never needs a refresh.
            assert!(!p.refresh_needed);
        }
        // The supervisor's push is what a record sees.
        d.shared
            .publish(reason, &(AdsState::Run as u16).to_le_bytes(), 0);
        assert_eq!(d.read_int32(&user(reason)).unwrap(), AdsState::Run as i32);

        // ...and binding it registered AMS port 852 with the supervisor.
        let ports = d.shared.ports.lock();
        let port = ports.iter().find(|p| p.ams_port == 852).unwrap();
        assert_eq!(port.state_param, Some(reason));
    }

    #[test]
    fn a_dropped_connection_invalidates_every_handle_of_that_ams_port() {
        let mut d = driver();
        let reason = bind(
            &mut d,
            "ADSPORT=851/.ADR.16#4020,16#0,4,3?",
            InterfaceType::Int32,
        );
        {
            let mut params = d.shared.params.lock();
            let p = params[reason].as_mut().unwrap();
            p.handle = Some(0x1234);
            p.notification = Some(7);
            p.refresh_needed = false;
        }
        d.shared.notifications.lock().insert(7, reason);

        d.shared.invalidate_ams_port(851);

        let params = d.shared.params.lock();
        let p = params[reason].as_ref().unwrap();
        // A handle from the dead socket means nothing on the next one.
        assert_eq!(p.handle, None);
        assert_eq!(p.notification, None);
        assert!(p.refresh_needed);
        assert_eq!(p.alarm, (alarm_status::COMM_ALARM, INVALID_ALARM));
        assert!(d.shared.notifications.lock().is_empty());
    }

    #[test]
    fn a_bulk_read_group_starts_with_the_two_plc_clock_slots() {
        let mut d = driver();
        let a = bind(
            &mut d,
            "POLL_RATE=1.0/.ADR.16#4020,16#0,4,3?",
            InterfaceType::Int32,
        );
        let b = bind(
            &mut d,
            "POLL_RATE=1.0/.ADR.16#4020,16#4,8,5?",
            InterfaceType::Float64,
        );
        // No PLC to resolve the clock symbols against, so the group falls back
        // to the %M slots C uses, and the samples carry the IOC clock.
        d.shared.add_to_bulk(a).unwrap();
        d.shared.add_to_bulk(b).unwrap();

        let groups = d.shared.bulk.lock();
        assert_eq!(groups.len(), 1);
        let g = &groups[0];
        assert_eq!(g.entries.len(), 4);
        assert_eq!(g.entries[0].index_group, ADSIGRP_MEMORY_BYTE);
        assert_eq!(g.entries[1].index_group, ADSIGRP_MEMORY_BYTE);
        assert_eq!(g.reasons, vec![None, None, Some(a), Some(b)]);
        assert_eq!(g.entries[3].size, 8);
    }

    #[test]
    fn re_binding_the_same_drvinfo_reuses_the_parameter() {
        let mut d = driver();
        let first = bind(&mut d, ".ADR.16#4020,16#0,4,3?", InterfaceType::Int32);
        let second = bind(&mut d, ".ADR.16#4020,16#0,4,3?", InterfaceType::Int32);
        assert_eq!(first, second);
    }

    #[test]
    fn interface_maps_to_the_c_dtyp_table() {
        assert_eq!(param_type_for(InterfaceType::Int32), Some(ParamType::Int32));
        assert_eq!(
            param_type_for(InterfaceType::Float32Array),
            Some(ParamType::Float32Array)
        );
        assert_eq!(param_type_for(InterfaceType::Octet), Some(ParamType::Octet));
        // C's dtypStringToAsynType has no arm for these, and neither does the
        // conversion matrix (adsAsynPortDriverUtils.cpp:465-467).
        assert_eq!(param_type_for(InterfaceType::UInt32Digital), None);
        assert_eq!(param_type_for(InterfaceType::Enum), None);
        assert_eq!(param_type_for(InterfaceType::GenericPointer), None);
    }

    #[test]
    fn bulk_period_never_polls_faster_than_1hz() {
        let mut cfg = AdsConfig::new("ADS", "10.0.0.1", AmsNetId([10, 0, 0, 1, 1, 1]), 851);
        cfg.default_sample_time_ms = 100.0;
        assert_eq!(cfg.bulk_period(), Duration::from_millis(1000));
        cfg.default_sample_time_ms = 2500.0;
        assert_eq!(cfg.bulk_period(), Duration::from_millis(2500));
    }

    #[test]
    fn a_port_driver_reserves_reason_0_for_the_command_channel() {
        let cfg = AdsConfig::new("ADS1", "10.0.0.1", AmsNetId([10, 0, 0, 1, 1, 1]), 851);
        let driver = AdsPortDriver::new(cfg).unwrap();
        assert_eq!(driver.base.find_param("Default access"), Some(0));
    }
}
