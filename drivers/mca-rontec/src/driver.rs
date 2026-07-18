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
//! [`mca::interface::McaReason::create_params`] is called only so the
//! default `drv_user_create` (name lookup against `McaReason::ALL`) resolves
//! drvInfo strings for [`crate::driver::RontecDriver::reason_of`]; the
//! resulting param slots are otherwise unused.
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
//!   asyn caller, only logged. [`RontecDriver::dispatch_write`] reproduces
//!   this: every `send_message` result inside it is discarded.
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
//!   arms cannot fall through, so [`RontecDriver::dispatch_write`] closes
//!   this structurally rather than by convention.
//! - **Integer division by zero in `mcaNumChannels`** (`drvMcaRontec.c:322-330`):
//!   `pPvt->binning = pPvt->maxChans / pPvt->nchans;` is unguarded --
//!   `NumChannels(0)` (or negative) is a client-reachable write (the mca
//!   record's `NUSE` field has no lower bound of its own) that divides by
//!   zero, undefined behavior in C and a `SIGFPE` crash in practice on
//!   Linux/x86. [`RontecDriver::set_num_channels`] only recomputes
//!   `binning` when the (post-cache) channel count is positive.
//! - **Uninitialized-read on I/O failure in `mcaReadStatus`**
//!   (`drvMcaRontec.c:307-317`): `sendMessage`'s `asynStatus` return is
//!   discarded, and `response[4]`/`atoi(&response[4])` are read
//!   unconditionally from a *local, stack-allocated* buffer -- on a timeout
//!   or disconnected line, `response` was never written and its bytes are
//!   indeterminate, so `acquiring`/`elive`/`ereal` are corrupted with
//!   garbage rather than left at their last known value.
//!   [`RontecDriver::read_status`] applies each parsed field only when its
//!   `send_message` call actually succeeded.
//! - **`int32ArrayRead` buffer overflow and silent short-read corruption**
//!   (`drvMcaRontec.c:436-478`): (a) `nread = 4 + 4*maxChans` is written
//!   into the fixed `pPvt->mcaBuffer[RONTEC_MAXCHANS*4+4]` with no check
//!   that the caller's `maxChans` (ultimately the mca record's
//!   client-writable `NUSE`, never clamped on this path) is `<=
//!   RONTEC_MAXCHANS` -- an oversized `NUSE` overflows the fixed buffer.
//!   (b) `if (nbytesIn != nread)` only logs; the function still parses
//!   `mcaBuffer` as if the transfer fully succeeded, returning stale/garbage
//!   spectrum data on a short read. [`RontecDriver::read_spectrum`] clamps
//!   the request to [`RONTEC_MAXCHANS`] and returns an error instead of
//!   fabricating data when the reply is short.

use std::time::Duration;

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::request::RequestOp;
use epics_rs::asyn::user::AsynUser;

use mca::interface::McaReason;

/// C `#define RONTEC_MAXCHANS 4096` (`drvMcaRontec.c:28`).
const RONTEC_MAXCHANS: usize = 4096;
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
}

impl RontecDriver {
    /// C `RontecConfig(portName,serialPort,serialPortAddress)`
    /// (`drvMcaRontec.c:165-257`). `serial_port_name` must already be
    /// registered (`drvAsynSerialPortConfigure`) -- this only looks it up,
    /// matching `pasynOctetSyncIO->connect`'s own precondition. Unlike C,
    /// the input EOS is not saved here: [`Self::read_spectrum`] uses
    /// [`RequestOp::OctetReadBinary`], which brackets the save/suppress/
    /// restore atomically inside the port actor (C `getInputEos` +
    /// `setInputEos("",0)` + `setInputEos(saved)` around `int32ArrayRead`,
    /// `drvMcaRontec.c:196-197,458,462`), so there is no saved EOS to carry
    /// on this struct.
    pub fn connect(
        port_name: &str,
        serial_port_name: &str,
        serial_port_addr: i32,
    ) -> AsynResult<Self> {
        let entry = get_port(serial_port_name).ok_or_else(|| {
            protocol_error(format!(
                "RontecConfig: can't connect to serial port {serial_port_name}"
            ))
        })?;

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

        let driver = RontecDriver {
            base,
            serial: entry.handle,
            addr: serial_port_addr,
            reasons,
            nchans: RONTEC_MAXCHANS as i32,
            binning: 1,
            plive: 0,
            preal: 0,
            elive: 0,
            ereal: 0,
            acquiring: false,
        };

        // C: `sendMessage(pPvt, "$SM 4,0", NULL);` -- send 4 bytes/channel,
        // don't clear after sending. Best-effort: C ignores this call's
        // status entirely and `RontecConfig` always returns success
        // regardless (`drvMcaRontec.c:199`).
        let _ = driver.send_message("$SM 4,0", RONTEC_TIMEOUT);

        Ok(driver)
    }

    fn octet_user(&self, timeout: Duration) -> AsynUser {
        AsynUser::new(OCTET_REASON)
            .with_addr(self.addr)
            .with_timeout(timeout)
    }

    /// Which [`McaReason`] (if any) `self.reasons` resolved `idx` to.
    /// `self.reasons[r as usize]` is what
    /// [`epics_rs::asyn::port::DeviceSupport`]'s drvUserCreate resolves a
    /// bound record's `pasynUser->reason` to for command `r` -- the reverse
    /// lookup here is what C's `pasynUser->reason == mcaCommand` direct
    /// comparison collapses to once drvInfo strings are resolved through a
    /// name-keyed table instead of C's raw enum values.
    fn reason_of(&self, idx: usize) -> Option<McaReason> {
        McaReason::ALL
            .into_iter()
            .find(|r| self.reasons[*r as usize] == idx)
    }

    /// C `sendMessage` (`drvMcaRontec.c:581-604`), minus the `"!ERROR"`
    /// trace-log check: every caller of `sendMessage` in C already discards
    /// its `asynStatus` return regardless of that check's outcome (see
    /// module doc's "always returns success" quirk), so it has no
    /// observable effect through this driver's contract.
    fn send_message(&self, output: &str, timeout: Duration) -> AsynResult<Vec<u8>> {
        let user = self.octet_user(timeout);
        let result = self.serial.submit_blocking(
            RequestOp::OctetWriteRead {
                data: output.as_bytes().to_vec(),
                buf_size: RONTEC_MESSAGE_SIZE,
                flush: true,
            },
            user,
        )?;
        result
            .data
            .ok_or_else(|| protocol_error("sendMessage: writeRead returned no data"))
    }

    /// C `int32ArrayRead` (`drvMcaRontec.c:436-478`), with the buffer
    /// overflow / silent-short-read defects fixed at source (module doc).
    /// Returns the channels actually read (`<= max_chans_requested`).
    fn read_spectrum(&self, max_chans_requested: usize) -> AsynResult<Vec<i32>> {
        let max_chans = max_chans_requested.min(RONTEC_MAXCHANS);
        if max_chans == 0 {
            return Ok(Vec::new());
        }

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
        // by the ASCII input EOS. `RequestOp::OctetWriteRead{flush:true}` is
        // the writeRead half (see `Self::send_message`'s doc); the EOS
        // save/suppress/restore is issued explicitly around it the same way
        // C issues its own three separate `pasynOctetSyncIO`/`pasynOctet`
        // calls (there is no single actor op combining both).
        let saved_eos = self
            .serial
            .submit_blocking(RequestOp::GetInputEos, self.octet_user(SPECTRUM_TIMEOUT))?
            .data
            .unwrap_or_default();
        self.serial.submit_blocking(
            RequestOp::SetInputEos { eos: Vec::new() },
            self.octet_user(SPECTRUM_TIMEOUT),
        )?;
        let read_result = self.serial.submit_blocking(
            RequestOp::OctetWriteRead {
                data: message.into_bytes(),
                buf_size: nread,
                flush: true,
            },
            self.octet_user(SPECTRUM_TIMEOUT),
        );
        self.serial.submit_blocking(
            RequestOp::SetInputEos { eos: saved_eos },
            self.octet_user(SPECTRUM_TIMEOUT),
        )?;
        let raw = read_result?
            .data
            .ok_or_else(|| protocol_error("int32ArrayRead: read returned no data"))?;

        // Fixed defect: C proceeds to parse `mcaBuffer` even when
        // `nbytesIn != nread`, returning stale/garbage data (module doc).
        if raw.len() != nread {
            return Err(protocol_error(format!(
                "int32ArrayRead: short read, got {} bytes, expected {nread}",
                raw.len()
            )));
        }

        // C: `pin = &pPvt->mcaBuffer[4]; for (i=0;i<maxChans;i++) { ...
        // data[i] = (temp[0]<<24)+(temp[1]<<16)+(temp[2]<<8)+temp[3]; }` --
        // big-endian 32-bit reconstruction from 4 unsigned bytes.
        // `i32::from_be_bytes` gives the identical bit pattern as defined
        // behavior, in place of C's left-shift-into-the-sign-bit (technically
        // UB, though reliable on every real two's-complement target).
        let mut data = Vec::with_capacity(max_chans);
        for i in 0..max_chans {
            let off = 4 + i * 4;
            data.push(i32::from_be_bytes([
                raw[off],
                raw[off + 1],
                raw[off + 2],
                raw[off + 3],
            ]));
        }
        Ok(data)
    }

    /// C `mcaReadStatus`'s handler inside `RontecWrite`
    /// (`drvMcaRontec.c:307-317`), with the uninitialized-read defect fixed
    /// at source (module doc): a field is only updated when its query
    /// actually succeeded.
    fn read_status(&mut self) {
        if let Ok(resp) = self.send_message("$FP", RONTEC_TIMEOUT)
            && let Some(&flag) = resp.get(4)
        {
            self.acquiring = flag == b'-';
        }

        // C: "Read 1 channel of the spectrum so we get the elapsed live and
        // real time" -- a side-effecting device interaction whose data is
        // discarded (`drvMcaRontec.c:311-312`); best-effort, matching C's
        // own disregard for this call's status.
        let _ = self.read_spectrum(1);

        // Preserved quirk: `elive` mirrors `ereal` from the same `"$MS"`
        // reply (module doc's "unfixable-without-spec" entry).
        if let Ok(resp) = self.send_message("$MS", RONTEC_TIMEOUT)
            && resp.len() > 4
        {
            let value = c_atoi(&resp[4..]);
            self.ereal = value;
            self.elive = value;
        }
    }

    /// C `mcaNumChannels`'s handler (`drvMcaRontec.c:322-330`), with the
    /// division-by-zero defect fixed at source (module doc).
    fn set_num_channels(&mut self, value: i32) {
        // C caches the value before the bounds check, and an out-of-range
        // write leaves that cached value in place -- preserved (matches
        // `mca::fastsweep`'s own documented `NumChannels` quirk).
        self.nchans = value;
        let nchans = if value > RONTEC_MAXCHANS as i32 {
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
    fn dispatch_write(&mut self, reason: Option<McaReason>, ivalue: i32, dvalue: f64) {
        match reason {
            Some(McaReason::StartAcquire) => {
                let _ = if self.plive > 0 {
                    self.send_message(&format!("$LT {}", self.plive), RONTEC_TIMEOUT)
                } else if self.preal > 0 {
                    self.send_message(&format!("$MT {}", self.preal), RONTEC_TIMEOUT)
                } else {
                    self.send_message("$MT 0", RONTEC_TIMEOUT)
                };
            }
            Some(McaReason::StopAcquire) => {
                let _ = self.send_message("$MP ON", RONTEC_TIMEOUT);
            }
            Some(McaReason::Erase) => {
                let _ = self.send_message("$CC", RONTEC_TIMEOUT);
            }
            Some(McaReason::ReadStatus) => self.read_status(),
            Some(McaReason::NumChannels) => self.set_num_channels(ivalue),
            Some(McaReason::PresetRealTime) => self.preal = (1000.0 * dvalue) as i32,
            Some(McaReason::PresetLiveTime) => self.plive = (1000.0 * dvalue) as i32,
            // NOOPs for Rontec (module doc): ChannelAdvanceSource,
            // AcquireMode, Sequence, Prescale, PresetSweeps,
            // PresetLowChannel, PresetHighChannel, DwellTime, PresetCounts.
            // Also the illegal-command / unreachable (Data/Acquiring/
            // Elapsed*) default, matching C's default arm.
            _ => {}
        }
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
        let reason = self.reason_of(user.reason);
        self.dispatch_write(reason, value, 0.0);
        Ok(())
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let reason = self.reason_of(user.reason);
        self.dispatch_write(reason, 0, value);
        Ok(())
    }

    /// C `RontecRead`'s `mcaAcquiring` arm, reached only via `int32Read`
    /// (`drvMcaRontec.c:401-424`).
    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        match self.reason_of(user.reason) {
            Some(McaReason::Acquiring) => Ok(i32::from(self.acquiring)),
            _ => Err(protocol_error(format!(
                "RontecRead got illegal command {}",
                user.reason
            ))),
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
            _ => Err(protocol_error(format!(
                "RontecRead got illegal command {}",
                user.reason
            ))),
        }
    }

    fn read_int32_array(&mut self, _user: &AsynUser, buf: &mut [i32]) -> AsynResult<usize> {
        let data = self.read_spectrum(buf.len())?;
        let nactual = data.len();
        buf[..nactual].copy_from_slice(&data);
        Ok(nactual)
    }
}

#[cfg(test)]
mod tests {
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;

    use super::*;

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
            NullTestDriver { base: serial_base },
            RuntimeConfig::default(),
        );
        RontecDriver {
            base,
            serial: serial_runtime.port_handle().clone(),
            addr: 0,
            reasons: dummy_reasons(),
            nchans: RONTEC_MAXCHANS as i32,
            binning: 1,
            plive: 0,
            preal: 0,
            elive: 0,
            ereal: 0,
            acquiring: false,
        }
    }

    /// The division-by-zero fix: `NumChannels(0)` must not touch `binning`
    /// (module doc).
    #[test]
    fn set_num_channels_guards_against_a_zero_value() {
        let mut d = dummy_driver();
        d.binning = 7;
        d.set_num_channels(0);
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
        d.set_num_channels(RONTEC_MAXCHANS as i32 * 2);
        assert_eq!(d.nchans, RONTEC_MAXCHANS as i32);
        assert_eq!(d.binning, 1);
    }

    #[test]
    fn set_num_channels_computes_binning_for_a_normal_value() {
        let mut d = dummy_driver();
        d.set_num_channels(1024);
        assert_eq!(d.nchans, 1024);
        assert_eq!(d.binning, 4);
    }

    #[test]
    fn preset_times_are_stored_in_milliseconds() {
        let mut d = dummy_driver();
        d.dispatch_write(Some(McaReason::PresetRealTime), 0, 2.5);
        assert_eq!(d.preal, 2500);
        d.dispatch_write(Some(McaReason::PresetLiveTime), 0, 1.0);
        assert_eq!(d.plive, 1000);
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
