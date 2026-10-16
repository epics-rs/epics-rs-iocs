//! `RontecDriver` — an asyn MCA port driver for the Rontec X-ray detector,
//! ported from `drvMcaRontec.c` (`mcaApp/RontecSrc`). Implements
//! [`crate` docs](crate)'s asyn MCA contract directly against
//! [`mca::interface::McaReason`] (the same 21 `mcaCommand` drvInfo strings
//! `drvFastSweep` registers) over a plain serial ASCII/binary protocol
//! against an already-configured octet port (`drvAsynSerialPortConfigure`),
//! found the same way `love`/`microepsilon` find theirs.
//!
//! # Architecture vs. C
//! C's `drvMcaRontec` implements `asynInt32`/`asynFloat64`/`asynInt32Array`
//! *directly* (not via the `asynPortDriver` C++ base's `paramList`), so
//! [`RontecDriver`] does not touch [`epics_rs::asyn::port::PortDriverBase`]'s
//! own param cache either -- every value is served from this struct's own
//! fields, exactly like `love::driver::LoveDriver`.
//! [`mca::interface::McaReason::create_params`] is called only to give each
//! command a reason index, which `drv_user_create` hands out for a
//! case-insensitive drvInfo match and `RontecDriver::reason_of` maps back;
//! the param slots themselves are otherwise unused.
//!
//! There is no `Init`/`Config` iocsh split here (unlike `love`/`microepsilon`):
//! `RontecConfig(portName,serialPort,serialPortAddress)` does everything C's
//! `RontecConfig` does in one call (`drvMcaRontec.c:165-257`), so no
//! crate-local name-keyed registry is needed.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **A write handler always returns success.** C `RontecWrite`
//!   (`drvMcaRontec.c:272-379`) has a single `return(asynSuccess)` reached
//!   from every switch arm, including the illegal-command default -- no
//!   internal I/O failure (a failed `sendMessage`) is ever surfaced to the
//!   asyn caller, only logged. `RontecDriver::dispatch_write` reproduces
//!   this: every `send_message` result inside it is discarded once
//!   `send_message` has printed C's `ASYN_TRACE_ERROR` line for it.
//! - **`mcaElapsedLiveTime` always mirrors `mcaElapsedRealTime`.** C's
//!   `mcaReadStatus` handler (`drvMcaRontec.c:307-317`) sets `elive` from
//!   `atoi(&response[4])` on the **same** `response` buffer the preceding
//!   `"$MS"` query filled for `ereal` -- the `"$LS"` query that would supply
//!   a real live-time reading is commented out ("LS seems not to be
//!   supported - track this down"). No spec for the correct live-time query
//!   is available from the C source alone, so this is preserved verbatim
//!   rather than guessed at (`unfixable-without-spec`).
//! - **`mcaPresetRealTime`/`mcaPresetLiveTime`'s comment says
//!   "centiseconds"; the code stores milliseconds.** C: `pPvt->preal = (int)
//!   (1000. * dvalue)` (`drvMcaRontec.c:359-366`) -- `*1000` is
//!   seconds-to-milliseconds, and every consumer (`"$MT %d"`/`"$LT %d"` on
//!   `mcaStartAcquire`, `/1000.` on `mcaElapsedRealTime`/
//!   `mcaElapsedLiveTime` read) agrees the unit is milliseconds. The stale
//!   comment is not reproduced; the millisecond arithmetic is.
//! - **`mcaPresetLowChannel`/`mcaPresetHighChannel`/`mcaDwellTime`/
//!   `mcaAcquireMode`/`mcaSequence`/`mcaPrescale`/`mcaPresetSweeps`/
//!   `mcaChannelAdvanceSource`/`mcaPresetCounts` are accepted no-ops.**
//!   Matches C exactly -- none of these ever reach the wire for Rontec.
//!   `mcaPresetCounts`'s C arm assigns a dead `ptotal` field (never read
//!   anywhere in the file); this port does not carry that dead field, which
//!   changes no observable behavior.
//!
//! # Fixed (not reproduced) upstream defects
//! - **Missing `break` / switch fallthrough** (`drvMcaRontec.c:317-318`):
//!   `case mcaReadStatus` has no `break` before `case mcaChannelAdvanceSource`,
//!   so the latter's body runs as an unintended continuation of the former.
//!   Currently harmless only because `mcaChannelAdvanceSource`'s own body is
//!   itself `break;` -- a latent hazard for any future edit. Rust `match`
//!   arms cannot fall through, so `RontecDriver::dispatch_write` closes
//!   this structurally rather than by convention.
//! - **Integer division by zero in `mcaNumChannels`** (`drvMcaRontec.c:322-330`):
//!   `pPvt->binning = pPvt->maxChans / pPvt->nchans;` is unguarded --
//!   `NumChannels(0)` (or negative) is a client-reachable write (the mca
//!   record's `NUSE` field has no lower bound of its own) that divides by
//!   zero, undefined behavior in C and a `SIGFPE` crash in practice on
//!   Linux/x86. `RontecDriver::set_num_channels` only recomputes
//!   `binning` when the (post-cache) channel count is positive.
//! - **Uninitialized-read on I/O failure in `mcaReadStatus`**
//!   (`drvMcaRontec.c:307-317`): `sendMessage`'s `asynStatus` return is
//!   discarded, and `response[4]`/`atoi(&response[4])` are read
//!   unconditionally from a *local, stack-allocated* buffer -- on a timeout
//!   or disconnected line, `response` was never written and its bytes are
//!   indeterminate, so `acquiring`/`elive`/`ereal` are corrupted with
//!   garbage rather than left at their last known value.
//!   `RontecDriver::read_status` applies each parsed field only when its
//!   `send_message` call actually succeeded.
//! - **`int32ArrayRead` buffer overflow** (`drvMcaRontec.c:436-478`):
//!   `nread = 4 + 4*maxChans` is written into the fixed
//!   `pPvt->mcaBuffer[RONTEC_MAXCHANS*4+4]` with no check that the caller's
//!   `maxChans` (ultimately the mca record's client-writable `NUSE`, never
//!   clamped on this path) is `<= RONTEC_MAXCHANS` -- an oversized `NUSE`
//!   overflows the fixed buffer. `RontecDriver::read_spectrum` clamps the
//!   request to `RONTEC_MAXCHANS`. A short reply is not an error, as in C:
//!   it is logged and the persistent buffer is parsed anyway.

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{DrvUserInfo, DrvUserRequest, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::registry::claim_port;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::runtime::config::RuntimeConfig;
use epics_rs::asyn::runtime::port::create_port_runtime;
use epics_rs::asyn::trace::TraceMask;
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::interrupt_accept::InterruptGate;
use epics_rs::base::runtime::log::errlog_printf;

use mca::interface::McaReason;

/// C `#define RONTEC_MAXCHANS 4096` (`drvMcaRontec.c:28`).
const RONTEC_MAXCHANS: usize = 4096;
/// C `mcaBuffer[RONTEC_MAXCHANS*4 + 4]` (`drvMcaRontec.c:81`).
const MCA_BUFFER_SIZE: usize = RONTEC_MAXCHANS * 4 + 4;
/// C `#define RONTEC_MESSAGE_SIZE 100` (`drvMcaRontec.c:29`) -- the reply
/// buffer size for plain ASCII exchanges (`sendMessage`).
const RONTEC_MESSAGE_SIZE: usize = 100;
/// C `#define RONTEC_TIMEOUT 2.0` (`drvMcaRontec.c:30`).
const RONTEC_TIMEOUT: Duration = Duration::from_secs(2);
/// C `int32ArrayRead`'s local `double timeout=10.0;` (`drvMcaRontec.c:442`)
/// -- distinct from [`RONTEC_TIMEOUT`], used only for the binary spectrum
/// transfer.
const SPECTRUM_TIMEOUT: Duration = Duration::from_secs(10);

/// The `pasynUser->reason` used for octet transactions against the
/// underlying serial port (mirrors `love::driver::OCTET_REASON`).
const OCTET_REASON: usize = 0;

fn protocol_error(message: impl Into<String>) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: message.into(),
    }
}

/// C `atoi`, applied to a byte slice: skip leading ASCII whitespace, an
/// optional sign, then decimal digits, stopping at the first non-digit (or
/// the end of the slice); `0` if no digits are found. C's `atoi` has
/// implementation-defined overflow behavior; this saturates instead, which
/// cannot diverge for any realistic Rontec time/count reply.
fn c_atoi(bytes: &[u8]) -> i32 {
    let mut i = 0;
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    let negative = match bytes.get(i) {
        Some(b'-') => {
            i += 1;
            true
        }
        Some(b'+') => {
            i += 1;
            false
        }
        _ => false,
    };
    let mut value: i32 = 0;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        value = value
            .saturating_mul(10)
            .saturating_add(i32::from(bytes[i] - b'0'));
        i += 1;
    }
    if negative { -value } else { value }
}

/// C `mcaRontecPvt` (`drvMcaRontec.c:62-88`), minus the asyn
/// interface/registration bookkeeping the Rust runtime already owns, and
/// minus the dead `ptschan`/`ptechan`/`ptotal`/`etotals` fields (see module
/// doc's "preserved quirks").
pub struct RontecDriver {
    base: PortDriverBase,
    serial: PortHandle,
    /// C `pPvt->serialPort`.
    serial_port: String,
    addr: i32,
    /// `driverReasons`-equivalent: resolved once by
    /// [`mca::interface::McaReason::create_params`] at connect time,
    /// indexed by `McaReason as usize` (see [`Self::reason_of`]).
    reasons: [usize; McaReason::COUNT],
    /// C `pPvt->nchans`.
    nchans: i32,
    /// C `pPvt->binning`.
    binning: i32,
    /// C `pPvt->plive` (milliseconds; see module doc).
    plive: i32,
    /// C `pPvt->preal` (milliseconds; see module doc).
    preal: i32,
    /// C `pPvt->elive` (milliseconds).
    elive: i32,
    /// C `pPvt->ereal` (milliseconds).
    ereal: i32,
    /// C `pPvt->acquiring`.
    acquiring: bool,
    /// C `pPvt->inputEos`: saved once at `RontecConfig` and restored after
    /// every spectrum read.
    input_eos: Vec<u8>,
    /// C `pPvt->mcaBuffer`: kept across transfers, so a short reply leaves
    /// the previous transfer's tail in place exactly as in C.
    mca_buffer: Vec<u8>,
}

impl RontecDriver {
    /// C `RontecConfig(portName,serialPort,serialPortAddress)`
    /// (`drvMcaRontec.c:165-257`). `serial_port_name` must already be
    /// registered (`drvAsynSerialPortConfigure`) -- this only looks it up,
    /// matching `pasynOctetSyncIO->connect`'s own precondition.
    pub fn connect(
        port_name: &str,
        serial_port_name: &str,
        serial_port_addr: i32,
    ) -> AsynResult<Self> {
        // C `pasynOctetSyncIO->connect` answers an unknown port with
        // asynError (`drvMcaRontec.c:172-178`).
        let entry = get_port(serial_port_name).ok_or_else(|| {
            protocol_error(format!(
                "RontecConfig: can't connect to serial port {serial_port_name}, status={}",
                AsynStatus::Error as i32
            ))
        })?;
        Self::on_serial(port_name, entry.handle, serial_port_name, serial_port_addr)
    }

    /// The tail of C `RontecConfig` (`drvMcaRontec.c:217-226`): publish the
    /// port as `owner`'s. A refused name prints C's line and nothing fails,
    /// because the iocsh wrapper drops `RontecConfig`'s status (`:618`).
    pub fn register(self, owner: &Arc<InterruptGate>) {
        let name = self.base.port_name.clone();
        let registered = claim_port(&name).is_ok_and(|claim| {
            create_port_runtime(&claim, self, RuntimeConfig::default())
                .is_ok_and(|(rt, _)| claim.publish(rt.port_handle().clone(), owner).is_ok())
        });
        if !registered {
            errlog_printf("RontecConfig ERROR: Can't register myself.\n");
        }
    }

    /// The part of C `RontecConfig` after the serial port is found.
    fn on_serial(
        port_name: &str,
        serial: PortHandle,
        serial_port_name: &str,
        serial_port_addr: i32,
    ) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let reasons = McaReason::create_params(&mut base)?;

        let mut driver = RontecDriver {
            base,
            serial,
            serial_port: serial_port_name.to_owned(),
            addr: serial_port_addr,
            reasons,
            nchans: RONTEC_MAXCHANS as i32,
            binning: 1,
            plive: 0,
            preal: 0,
            elive: 0,
            ereal: 0,
            acquiring: false,
            input_eos: Vec::new(),
            mca_buffer: vec![0; MCA_BUFFER_SIZE],
        };

        // C: "Get the input EOS to be able to reset it later"
        // (`drvMcaRontec.c:196-197`), status ignored.
        driver.input_eos = driver
            .serial
            .submit_blocking(RequestOp::GetInputEos, driver.eos_user(RONTEC_TIMEOUT))
            .ok()
            .and_then(|r| r.data)
            .unwrap_or_default();

        // C: `sendMessage(pPvt, "$SM 4,0", NULL);` -- send 4 bytes/channel,
        // don't clear after sending. Best-effort: C ignores this call's
        // status entirely and `RontecConfig` always returns success
        // regardless (`drvMcaRontec.c:199`).
        let _ = driver.send_message("$SM 4,0");

        Ok(driver)
    }

    fn octet_user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(OCTET_REASON)
            .with_addr(self.addr)
            .with_timeout(timeout)
    }

    /// C's `pasynOctetSyncIO` EOS calls take `lockPort`, which a down line
    /// does not refuse (`asynOctetSyncIO.c:306`); `writeRead` queues and does.
    fn eos_user(&self, timeout: Duration) -> AsynUser {
        self.octet_user(timeout).queue_even_if_not_connected()
    }

    /// Which [`McaReason`] (if any) `self.reasons` resolved `idx` to.
    /// `self.reasons[r as usize]` is what
    /// `DeviceSupport`'s drvUserCreate resolves a
    /// bound record's `pasynUser->reason` to for command `r` -- the reverse
    /// lookup here is what C's `pasynUser->reason == mcaCommand` direct
    /// comparison collapses to once drvInfo strings are resolved through a
    /// name-keyed table instead of C's raw enum values.
    fn reason_of(&self, idx: usize) -> Option<McaReason> {
        McaReason::ALL
            .into_iter()
            .find(|r| self.reasons[*r as usize] == idx)
    }

    /// C `sendMessage` (`drvMcaRontec.c:581-604`): the error line is printed
    /// through the serial port's trace, as C prints it on `pPvt->pasynUser`.
    fn send_message(&self, output: &str) -> AsynResult<Vec<u8>> {
        let result = self
            .serial
            .submit_blocking(
                RequestOp::OctetWriteRead {
                    data: output.as_bytes().to_vec(),
                    buf_size: RONTEC_MESSAGE_SIZE,
                    flush: true,
                },
                self.octet_user(RONTEC_TIMEOUT),
            )
            .map(|r| r.data.unwrap_or_default());
        // A failed read still wrote the whole command; a failed write read
        // nothing. A bare error carries neither count.
        let (input, status, nbytes_out): (&[u8], AsynStatus, usize) = match &result {
            Ok(data) => (data, AsynStatus::Success, output.len()),
            Err(e) => match (e.partial_read(), e.partial_write()) {
                (Some(p), _) => (&p.data, e.status(), output.len()),
                (None, Some(n)) => (&[], e.status(), n),
                (None, None) => (&[], e.status(), 0),
            },
        };
        if status != AsynStatus::Success || input.starts_with(b"!ERROR") {
            self.serial.user_trace().print(
                self.addr,
                OCTET_REASON as i32,
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "drvMcaRontec::sendMessage: error {}, status={}, nbytesOut={nbytes_out}, nbytesIn={}",
                    String::from_utf8_lossy(input),
                    status as i32,
                    input.len()
                ),
            );
        }
        result
    }

    /// C `int32ArrayRead` (`drvMcaRontec.c:436-478`), with the request
    /// clamped to `RONTEC_MAXCHANS` (module doc). Fills `data` and returns
    /// its length; a short reply is logged and the rest of `data` comes from
    /// the previous transfer's bytes, as C parses `pPvt->mcaBuffer` anyway.
    fn read_spectrum(&mut self, data: &mut [i32]) -> usize {
        let max_chans = data.len().min(RONTEC_MAXCHANS);

        // C: `sprintf(message, "$SS 0,%d,%d,%ld", binning, binning,
        // (long)maxChans*binning);`
        let message = format!(
            "$SS 0,{},{},{}",
            self.binning,
            self.binning,
            i64::from(self.binning) * max_chans as i64
        );
        // C: `nread = 4 + 4*maxChans;` -- 4-byte header + 4 bytes/channel.
        let nread = 4 + 4 * max_chans;

        // C brackets a single `pasynOctetSyncIO->writeRead` (flush → write →
        // read) with `setInputEos("",0)` / `setInputEos(saved,len)`
        // (`drvMcaRontec.c:458,461-462`) so the binary reply isn't cut short
        // by the ASCII input EOS, ignoring every status but the length. Each
        // is its own port lock in C as here, so another user of the serial
        // port can run between them in both.
        let _ = self.serial.submit_blocking(
            RequestOp::SetInputEos { eos: Vec::new() },
            self.eos_user(SPECTRUM_TIMEOUT),
        );
        let read_result = self.serial.submit_blocking(
            RequestOp::OctetWriteRead {
                data: message.into_bytes(),
                buf_size: nread,
                flush: true,
            },
            self.octet_user(SPECTRUM_TIMEOUT),
        );
        let _ = self.serial.submit_blocking(
            RequestOp::SetInputEos {
                eos: self.input_eos.clone(),
            },
            self.eos_user(SPECTRUM_TIMEOUT),
        );
        let raw = match &read_result {
            Ok(r) => r.data.as_deref().unwrap_or_default(),
            Err(e) => e.partial_read().map_or(&[][..], |p| &p.data),
        };
        let nbytes_in = raw.len().min(nread);
        self.mca_buffer[..nbytes_in].copy_from_slice(&raw[..nbytes_in]);
        if nbytes_in != nread {
            self.serial.user_trace().print(
                self.addr,
                OCTET_REASON as i32,
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "drvMcaRontec::int32ArrayRead: error reading MCA data, nbytesIn={nbytes_in}, should be {nread}"
                ),
            );
        }

        // C: `pin = &pPvt->mcaBuffer[4]; ... data[i] = (temp[0]<<24)+
        // (temp[1]<<16)+(temp[2]<<8)+temp[3];` -- big-endian; `from_be_bytes`
        // is the same bit pattern without C's shift into the sign bit.
        for (value, bytes) in data[..max_chans]
            .iter_mut()
            .zip(self.mca_buffer[4..nread].as_chunks::<4>().0)
        {
            *value = i32::from_be_bytes(*bytes);
        }
        max_chans
    }

    /// C `mcaReadStatus`'s handler inside `RontecWrite`
    /// (`drvMcaRontec.c:307-317`), with the uninitialized-read defect fixed
    /// at source (module doc): a field is only updated when its query
    /// actually succeeded.
    fn read_status(&mut self) {
        if let Ok(resp) = self.send_message("$FP")
            && let Some(&flag) = resp.get(4)
        {
            self.acquiring = flag == b'-';
        }

        // C: "Read 1 channel of the spectrum so we get the elapsed live and
        // real time" -- a side-effecting device interaction whose data is
        // discarded (`drvMcaRontec.c:311-312`); best-effort, matching C's
        // own disregard for this call's status.
        self.read_spectrum(&mut [0]);

        // Preserved quirk: `elive` mirrors `ereal` from the same `"$MS"`
        // reply (module doc's "unfixable-without-spec" entry).
        if let Ok(resp) = self.send_message("$MS")
            && resp.len() > 4
        {
            let value = c_atoi(&resp[4..]);
            self.ereal = value;
            self.elive = value;
        }
    }

    /// C `mcaNumChannels`'s handler (`drvMcaRontec.c:322-330`), with the
    /// division-by-zero defect fixed at source (module doc).
    fn set_num_channels(&mut self, user: &AsynUser, value: i32) {
        // C caches the value before the bounds check, and an out-of-range
        // write leaves that cached value in place -- preserved (matches
        // `mca::fastsweep`'s own documented `NumChannels` quirk).
        self.nchans = value;
        let nchans = if value > RONTEC_MAXCHANS as i32 {
            user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!("mcaRontecServer Illegal nuse field"),
            );
            self.nchans = RONTEC_MAXCHANS as i32;
            RONTEC_MAXCHANS as i32
        } else {
            value
        };
        if nchans > 0 {
            self.binning = RONTEC_MAXCHANS as i32 / nchans;
        }
    }

    /// C `RontecWrite` (`drvMcaRontec.c:272-379`), restructured: `ivalue`/
    /// `dvalue` mirror C's dual-argument convention (`int32Write` passes
    /// `dvalue=0.`, `float64Write` passes `ivalue=0`). Always returns
    /// `Ok(())` -- see module doc's "always returns success" quirk; unlike
    /// C, this is enforced by construction (no `?` anywhere in this
    /// function) rather than by a single trailing `return`.
    fn dispatch_write(&mut self, user: &AsynUser, ivalue: i32, dvalue: f64) {
        match self.reason_of(user.reason) {
            Some(McaReason::StartAcquire) => {
                let _ = if self.plive > 0 {
                    self.send_message(&format!("$LT {}", self.plive))
                } else if self.preal > 0 {
                    self.send_message(&format!("$MT {}", self.preal))
                } else {
                    self.send_message("$MT 0")
                };
            }
            Some(McaReason::StopAcquire) => {
                let _ = self.send_message("$MP ON");
            }
            Some(McaReason::Erase) => {
                let _ = self.send_message("$CC");
            }
            Some(McaReason::ReadStatus) => self.read_status(),
            Some(McaReason::NumChannels) => self.set_num_channels(user, ivalue),
            Some(McaReason::PresetRealTime) => self.preal = (1000.0 * dvalue) as i32,
            Some(McaReason::PresetLiveTime) => self.plive = (1000.0 * dvalue) as i32,
            // NOOPs for Rontec (module doc).
            Some(
                McaReason::ChannelAdvanceSource
                | McaReason::AcquireMode
                | McaReason::Sequence
                | McaReason::Prescale
                | McaReason::PresetSweeps
                | McaReason::PresetLowChannel
                | McaReason::PresetHighChannel
                | McaReason::DwellTime
                | McaReason::PresetCounts,
            ) => {}
            Some(
                McaReason::Data
                | McaReason::Acquiring
                | McaReason::ElapsedLiveTime
                | McaReason::ElapsedRealTime
                | McaReason::ElapsedCounts,
            )
            | None => user.print(
                TraceMask::ERROR,
                file!(),
                line!(),
                format_args!(
                    "drvMcaRontec::RontecWrite port {} got illegal command {}",
                    self.base.port_name, user.reason
                ),
            ),
        }
    }

    /// C `RontecRead`'s default arm (`drvMcaRontec.c:417-422`).
    fn illegal_read(&self, user: &AsynUser) -> AsynError {
        user.print(
            TraceMask::ERROR,
            file!(),
            line!(),
            format_args!(
                "drvMcaRontec::RontecRead got illegal command {}",
                user.reason
            ),
        );
        protocol_error(format!("RontecRead got illegal command {}", user.reason))
    }
}

impl PortDriver for RontecDriver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        self.dispatch_write(user, value, 0.0);
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        self.dispatch_write(user, 0, value);
        Ok(())
    }

    /// C `drvUserCreate` (`drvMcaRontec.c:503-524`): case-insensitive
    /// (`epicsStrCaseCmp`).
    fn drv_user_create(&mut self, req: &DrvUserRequest) -> AsynResult<DrvUserInfo> {
        match McaReason::ALL
            .into_iter()
            .find(|r| r.drv_info().eq_ignore_ascii_case(&req.drv_info))
        {
            Some(r) => Ok(DrvUserInfo::from_reason(self.reasons[r as usize])),
            None => {
                self.base.trace_print(
                    TraceMask::ERROR,
                    format_args!(
                        "drvMcaRontec::drvUserCreate, unknown command={}",
                        req.drv_info
                    ),
                );
                Err(AsynError::ParamNotFound(req.drv_info.clone()))
            }
        }
    }

    /// C `RontecReport` (`drvMcaRontec.c:548-559`).
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        let _ = writeln!(
            out,
            "Rontec {}: connected on serial port {}",
            self.base.port_name, self.serial_port
        );
        if level >= 1 {
            let _ = writeln!(out, "              maxChans: {RONTEC_MAXCHANS}");
            let _ = writeln!(out, "              nchans:   {}", self.nchans);
            let _ = writeln!(out, "              binning:  {}", self.binning);
        }
    }

    /// C `RontecRead`'s `mcaAcquiring` arm, reached only via `int32Read`
    /// (`drvMcaRontec.c:401-424`).
    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        match self.reason_of(user.reason) {
            Some(McaReason::Acquiring) => Ok(i32::from(self.acquiring)),
            _ => Err(self.illegal_read(user)),
        }
    }

    /// C `RontecRead`'s float arms, reached only via `float64Read`
    /// (`drvMcaRontec.c:401-424`).
    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        match self.reason_of(user.reason) {
            Some(McaReason::DwellTime) => Ok(0.0),
            Some(McaReason::ElapsedLiveTime) => Ok(f64::from(self.elive) / 1000.0),
            Some(McaReason::ElapsedRealTime) => Ok(f64::from(self.ereal) / 1000.0),
            Some(McaReason::ElapsedCounts) => Ok(0.0),
            _ => Err(self.illegal_read(user)),
        }
    }

    fn read_int32_array(&mut self, _user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        Ok(self.read_spectrum(buf))
    }

    /// C `int32ArrayWrite` (`drvMcaRontec.c:480-498`): the device memory
    /// cannot be written, so nothing is stored and no callback runs.
    fn write_int32_array(&mut self, user: &AsynUser, _data: &[i32]) -> AsynResult<()> {
        user.print(
            TraceMask::ERROR,
            file!(),
            line!(),
            format_args!("drvMcaRontec::int32ArrayWrite, write to Rontec not implemented"),
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::request::RequestResult;

    use super::*;
    use epics_rs::asyn::registry::PortRegistry;

    #[test]
    fn c_atoi_parses_leading_digits_and_stops_at_the_first_non_digit() {
        assert_eq!(c_atoi(b"1234\r"), 1234);
        assert_eq!(c_atoi(b" 42"), 42);
        assert_eq!(c_atoi(b"-7abc"), -7);
        assert_eq!(c_atoi(b"abc"), 0);
        assert_eq!(c_atoi(b""), 0);
    }

    fn dummy_reasons() -> [usize; McaReason::COUNT] {
        let mut r = [0usize; McaReason::COUNT];
        for (i, slot) in r.iter_mut().enumerate() {
            *slot = i;
        }
        r
    }

    /// A minimal, otherwise-inert `PortDriver` -- exists only so
    /// [`dummy_driver`] can obtain a genuine [`PortHandle`] (a port actor
    /// with a real channel) for the pure-logic tests below, none of which
    /// perform any I/O through it.
    struct NullTestDriver {
        base: PortDriverBase,
    }

    impl PortDriver for NullTestDriver {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
    }

    fn dummy_driver() -> RontecDriver {
        let base = PortDriverBase::new(
            "TESTRONTEC",
            1,
            PortFlags {
                multi_device: true,
                can_block: true,
                destructible: true,
            },
        );
        let serial_base = PortDriverBase::new(
            "TESTRONTECSERIAL",
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        let (serial_runtime, _serial_thread) = create_port_runtime(
            &PortRegistry::new().claim("TESTRONTECSERIAL").unwrap(),
            NullTestDriver { base: serial_base },
            RuntimeConfig::default(),
        )
        .expect("serial port runtime");
        RontecDriver {
            base,
            serial: serial_runtime.port_handle().clone(),
            serial_port: "TESTRONTECSERIAL".to_owned(),
            addr: 0,
            reasons: dummy_reasons(),
            nchans: RONTEC_MAXCHANS as i32,
            binning: 1,
            plive: 0,
            preal: 0,
            elive: 0,
            ereal: 0,
            acquiring: false,
            input_eos: Vec::new(),
            mca_buffer: vec![0; MCA_BUFFER_SIZE],
        }
    }

    /// The division-by-zero fix: `NumChannels(0)` must not touch `binning`
    /// (module doc).
    #[test]
    fn set_num_channels_guards_against_a_zero_value() {
        let mut d = dummy_driver();
        d.binning = 7;
        d.set_num_channels(&AsynUser::default(), 0);
        assert_eq!(d.nchans, 0);
        assert_eq!(
            d.binning, 7,
            "binning must be left untouched, not divide by zero"
        );
    }

    /// C caches the raw value before the too-large bounds check, and an
    /// out-of-range write leaves that cache in place (preserved quirk).
    #[test]
    fn set_num_channels_caches_an_out_of_range_value_before_clamping() {
        let mut d = dummy_driver();
        d.set_num_channels(&AsynUser::default(), RONTEC_MAXCHANS as i32 * 2);
        assert_eq!(d.nchans, RONTEC_MAXCHANS as i32);
        assert_eq!(d.binning, 1);
    }

    #[test]
    fn set_num_channels_computes_binning_for_a_normal_value() {
        let mut d = dummy_driver();
        d.set_num_channels(&AsynUser::default(), 1024);
        assert_eq!(d.nchans, 1024);
        assert_eq!(d.binning, 4);
    }

    #[test]
    fn preset_times_are_stored_in_milliseconds() {
        let mut d = dummy_driver();
        d.dispatch_write(&AsynUser::new(McaReason::PresetRealTime as usize), 0, 2.5);
        assert_eq!(d.preal, 2500);
        d.dispatch_write(&AsynUser::new(McaReason::PresetLiveTime as usize), 0, 1.0);
        assert_eq!(d.plive, 1000);
    }

    /// A serial port answering each Rontec command from `reply`, logging
    /// every write.
    struct FakeSerial {
        base: PortDriverBase,
        reply: fn(&[u8]) -> Vec<u8>,
        pending: Vec<u8>,
        writes: std::sync::Arc<std::sync::Mutex<Vec<Vec<u8>>>>,
        input_eos: Vec<u8>,
    }

    impl PortDriver for FakeSerial {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }

        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }

        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.writes.lock().unwrap().push(data.to_vec());
            self.pending = (self.reply)(data);
            Ok(data.len())
        }

        /// No reply is a timeout, carried the way the EOS interpose carries
        /// every failed read.
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
            if self.pending.is_empty() {
                return Err(AsynError::Status {
                    status: AsynStatus::Timeout,
                    message: "timeout".into(),
                }
                .with_partial_read(epics_rs::asyn::interpose::PartialOctetRead {
                    data: Vec::new(),
                    eom_reason: epics_rs::asyn::interpose::EomReason::empty(),
                }));
            }
            let n = self.pending.len().min(buf.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            Ok(n)
        }

        fn set_input_eos(&mut self, _user: &AsynUser, eos: &[u8]) -> AsynResult<()> {
            self.input_eos = eos.to_vec();
            Ok(())
        }

        fn get_input_eos(&self, _user: &AsynUser) -> Vec<u8> {
            self.input_eos.clone()
        }
    }

    /// A Rontec port on a [`FakeSerial`], both tracing into one file.
    struct Rig {
        rontec: epics_rs::asyn::runtime::port::PortRuntimeHandle,
        serial: epics_rs::asyn::runtime::port::PortRuntimeHandle,
        writes: std::sync::Arc<std::sync::Mutex<Vec<Vec<u8>>>>,
        trace_path: std::path::PathBuf,
    }

    impl Rig {
        fn new(tag: &str, reply: fn(&[u8]) -> Vec<u8>) -> Self {
            use epics_rs::asyn::services::PortServices;
            use epics_rs::asyn::trace::TraceFile;
            let services = PortServices::new();
            let config = || RuntimeConfig {
                services: services.clone(),
                ..RuntimeConfig::default()
            };
            let serial_name = format!("RIGSER_{tag}");
            let rontec_name = format!("RIGRON_{tag}");
            let writes = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
            let serial = FakeSerial {
                base: PortDriverBase::new(&serial_name, 1, PortFlags::default()),
                reply,
                pending: Vec::new(),
                writes: writes.clone(),
                input_eos: b"\r".to_vec(),
            };
            let (serial_rt, _) = create_port_runtime(
                &PortRegistry::new().claim(&serial_name).unwrap(),
                serial,
                config(),
            )
            .expect("serial runtime");
            let d = RontecDriver::on_serial(
                &rontec_name,
                serial_rt.port_handle().clone(),
                &serial_name,
                0,
            )
            .expect("rontec driver");
            writes.lock().unwrap().clear();
            let (rontec, _) = create_port_runtime(
                &PortRegistry::new().claim(&rontec_name).unwrap(),
                d,
                config(),
            )
            .expect("rontec runtime");
            let trace_path =
                std::env::temp_dir().join(format!("mca-rontec-{tag}-{}.trace", std::process::id()));
            let file = std::sync::Arc::new(std::sync::Mutex::new(
                std::fs::File::create(&trace_path).expect("trace file"),
            ));
            for name in [&serial_name, &rontec_name] {
                services
                    .trace()
                    .set_trace_file(Some(name), TraceFile::File(file.clone()));
            }
            Rig {
                rontec,
                serial: serial_rt,
                writes,
                trace_path,
            }
        }

        fn submit(&self, op: RequestOp, reason: McaReason) -> AsynResult<RequestResult> {
            self.rontec
                .port_handle()
                .submit_blocking(op, AsynUser::new(reason as usize))
        }

        fn trace(&self) -> String {
            std::fs::read_to_string(&self.trace_path).expect("trace file")
        }
    }

    impl Drop for Rig {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.trace_path);
        }
    }

    fn rontec_reply(cmd: &[u8]) -> Vec<u8> {
        match cmd {
            b"$CC" => b"!ERROR 3".to_vec(),
            b"$MP ON" => Vec::new(),
            _ => b"!OK".to_vec(),
        }
    }

    /// C `sendMessage` prints the `!ERROR` reply at `ASYN_TRACE_ERROR`
    /// (`drvMcaRontec.c:598-601`) while the write still succeeds.
    #[test]
    fn send_message_prints_an_error_reply() {
        let rig = Rig::new("err", rontec_reply);
        rig.submit(RequestOp::Int32Write { value: 1 }, McaReason::Erase)
            .expect("RontecWrite always succeeds");
        assert_eq!(rig.writes.lock().unwrap().as_slice(), [b"$CC".to_vec()]);
        assert!(
            rig.trace().contains(
                "drvMcaRontec::sendMessage: error !ERROR 3, status=0, nbytesOut=3, nbytesIn=8\n"
            ),
            "{}",
            rig.trace()
        );
        rig.submit(RequestOp::Int32Write { value: 1 }, McaReason::Sequence)
            .expect("noop");
        assert_eq!(rig.trace().matches("sendMessage").count(), 1);
    }

    #[test]
    fn send_message_prints_an_io_failure() {
        let rig = Rig::new("timeout", rontec_reply);
        rig.submit(RequestOp::Int32Write { value: 1 }, McaReason::StopAcquire)
            .expect("RontecWrite always succeeds");
        assert!(
            rig.trace()
                .contains("drvMcaRontec::sendMessage: error , status=1, nbytesOut=6, nbytesIn=0\n"),
            "{}",
            rig.trace()
        );
    }

    #[test]
    fn an_oversized_nuse_prints_c_illegal_nuse_line() {
        let rig = Rig::new("nuse", rontec_reply);
        rig.submit(
            RequestOp::Int32Write {
                value: RONTEC_MAXCHANS as i32 + 1,
            },
            McaReason::NumChannels,
        )
        .expect("write");
        assert!(rig.trace().contains("mcaRontecServer Illegal nuse field"));
    }

    /// C `RontecWrite`/`RontecRead` default arms (`drvMcaRontec.c:372-376,
    /// 417-422`).
    #[test]
    fn illegal_commands_print_c_lines() {
        let rig = Rig::new("illegal", rontec_reply);
        rig.submit(RequestOp::Int32Write { value: 1 }, McaReason::Acquiring)
            .expect("RontecWrite always succeeds");
        assert!(
            rig.trace()
                .contains("drvMcaRontec::RontecWrite port RIGRON_illegal got illegal command 17\n"),
            "{}",
            rig.trace()
        );
        assert!(rig.submit(RequestOp::Int32Read, McaReason::Erase).is_err());
        assert!(
            rig.trace()
                .contains("drvMcaRontec::RontecRead got illegal command 3\n")
        );
    }

    /// `$SS` answered in full with 100.. the first time, then with only two
    /// channels of 200.. .
    fn spectrum_reply(cmd: &[u8]) -> Vec<u8> {
        static CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        if !cmd.starts_with(b"$SS") {
            return b"!OK".to_vec();
        }
        let (base, nchans) = match CALLS.fetch_add(1, std::sync::atomic::Ordering::SeqCst) {
            0 => (100i32, 4),
            _ => (200, 2),
        };
        let mut reply = b"!SS\r".to_vec();
        for i in 0..nchans {
            reply.extend_from_slice(&(base + i).to_be_bytes());
        }
        reply
    }

    /// C `int32ArrayRead` logs a short reply but parses `mcaBuffer` anyway
    /// and returns `maxChans` with `asynSuccess` (`drvMcaRontec.c:463-477`).
    #[test]
    fn a_short_spectrum_is_logged_and_parsed_from_the_kept_buffer() {
        let rig = Rig::new("short", spectrum_reply);
        let read = || {
            rig.submit(
                RequestOp::Int32ArrayRead { max_elements: 4 },
                McaReason::Data,
            )
            .expect("short read is not an error")
            .int32_array
            .expect("array")
            .to_vec()
        };
        assert_eq!(read(), [100, 101, 102, 103]);
        assert!(!rig.trace().contains("int32ArrayRead"));
        assert_eq!(read(), [200, 201, 102, 103]);
        assert!(
            rig.trace().contains(
                "drvMcaRontec::int32ArrayRead: error reading MCA data, nbytesIn=12, should be 20\n"
            ),
            "{}",
            rig.trace()
        );
        assert_eq!(rig.writes.lock().unwrap()[1], b"$SS 0,1,1,4");
    }

    #[test]
    fn an_array_write_only_prints_not_implemented() {
        let rig = Rig::new("arraywrite", rontec_reply);
        rig.submit(
            RequestOp::Int32ArrayWrite {
                data: vec![1, 2, 3],
            },
            McaReason::Data,
        )
        .expect("C returns asynSuccess");
        assert!(
            rig.trace()
                .contains("drvMcaRontec::int32ArrayWrite, write to Rontec not implemented\n")
        );
        assert!(rig.writes.lock().unwrap().is_empty());
    }

    #[test]
    fn report_prints_c_rontec_report() {
        let mut d = dummy_driver();
        d.set_num_channels(&AsynUser::default(), 1024);
        let mut out = String::new();
        d.report(&mut out, 0);
        assert_eq!(
            out,
            "Rontec TESTRONTEC: connected on serial port TESTRONTECSERIAL\n"
        );
        out.clear();
        d.report(&mut out, 1);
        assert_eq!(
            out,
            "Rontec TESTRONTEC: connected on serial port TESTRONTECSERIAL\n\
             \x20             maxChans: 4096\n\
             \x20             nchans:   1024\n\
             \x20             binning:  4\n"
        );
    }

    #[test]
    fn connect_to_a_missing_serial_port_fails_with_c_text() {
        let err = RontecDriver::connect("RONTECNOPORT", "NOSUCHSERIAL", 0)
            .err()
            .expect("no such port");
        assert_eq!(
            err.message(),
            "RontecConfig: can't connect to serial port NOSUCHSERIAL, status=3"
        );
    }

    #[test]
    fn drv_user_create_ignores_case_and_prints_an_unknown_command() {
        let rig = Rig::new("drvuser", rontec_reply);
        let handle = rig.rontec.port_handle();
        let info = handle
            .drv_user_create_blocking(&DrvUserRequest::new("mca_erase", 0))
            .expect("case-insensitive match");
        assert_eq!(info.reason, McaReason::Erase as usize);
        assert!(
            handle
                .drv_user_create_blocking(&DrvUserRequest::new("NOT_A_COMMAND", 0))
                .is_err()
        );
        assert!(
            rig.trace()
                .contains("drvMcaRontec::drvUserCreate, unknown command=NOT_A_COMMAND\n"),
            "{}",
            rig.trace()
        );
    }

    /// C restores the input EOS `RontecConfig` saved, not the one in force
    /// when the spectrum is read (`drvMcaRontec.c:197,462`).
    #[test]
    fn a_spectrum_read_restores_the_input_eos_saved_at_config() {
        let rig = Rig::new("eos", rontec_reply);
        let serial = rig.serial.port_handle();
        serial
            .submit_blocking(
                RequestOp::SetInputEos {
                    eos: b"\n".to_vec(),
                },
                AsynUser::default(),
            )
            .expect("set eos");
        rig.submit(
            RequestOp::Int32ArrayRead { max_elements: 1 },
            McaReason::Data,
        )
        .expect("read");
        let eos = serial
            .submit_blocking(RequestOp::GetInputEos, AsynUser::default())
            .expect("get eos")
            .data;
        assert_eq!(eos.as_deref(), Some(&b"\r"[..]));
    }

    /// C `getInputEos` takes `lockPort`, so a serial line that is down at
    /// `RontecConfig` still yields its EOS (`drvMcaRontec.c:197`).
    #[test]
    fn config_on_a_down_serial_line_saves_its_input_eos() {
        let mut base = PortDriverBase::new("DOWNSER", 1, PortFlags::default());
        base.set_auto_connect(false);
        base.init_connected(false);
        let (serial, _) = create_port_runtime(
            &PortRegistry::new().claim("DOWNSER").unwrap(),
            FakeSerial {
                base,
                reply: rontec_reply,
                pending: Vec::new(),
                writes: Default::default(),
                input_eos: b"\r".to_vec(),
            },
            RuntimeConfig::default(),
        )
        .expect("serial runtime");
        let d = RontecDriver::on_serial("DOWNRONTEC", serial.port_handle().clone(), "DOWNSER", 0)
            .expect("driver");
        assert_eq!(d.input_eos, b"\r");
    }

    /// C `registerPort` refuses a taken name and `RontecConfig` prints its
    /// line (`drvMcaRontec.c:224-226`); the command does not fail.
    #[test]
    fn a_duplicate_port_prints_c_cant_register_line() {
        use epics_rs::base::runtime::log::{
            errlog_add_listener, errlog_flush, errlog_remove_listener,
        };
        let lines = Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
        let sink = lines.clone();
        let id = errlog_add_listener(move |m| sink.lock().unwrap().push(m.to_owned()));
        let (serial, _) = create_port_runtime(
            &PortRegistry::new().claim("DUPSER").unwrap(),
            FakeSerial {
                base: PortDriverBase::new("DUPSER", 1, PortFlags::default()),
                reply: rontec_reply,
                pending: Vec::new(),
                writes: Default::default(),
                input_eos: b"\r".to_vec(),
            },
            RuntimeConfig::default(),
        )
        .expect("serial runtime");
        let gate = InterruptGate::opened();
        for _ in 0..2 {
            RontecDriver::on_serial("DUPRONTEC", serial.port_handle().clone(), "DUPSER", 0)
                .expect("driver")
                .register(&gate);
        }
        errlog_flush();
        errlog_remove_listener(id);
        let cant = "RontecConfig ERROR: Can't register myself.\n";
        assert_eq!(
            lines.lock().unwrap().iter().filter(|l| *l == cant).count(),
            1,
            "{:?}",
            lines.lock().unwrap()
        );
        assert!(get_port("DUPRONTEC").is_some());
    }

    #[test]
    fn reason_of_resolves_every_registered_command() {
        let d = dummy_driver();
        for r in McaReason::ALL {
            assert_eq!(d.reason_of(r as usize), Some(r));
        }
        assert_eq!(d.reason_of(McaReason::COUNT), None);
    }
}
