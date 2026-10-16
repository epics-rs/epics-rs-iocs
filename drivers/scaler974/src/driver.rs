//! Ortec 974 `ScalerDriver` implementation, ported from `Scaler974`
//! (`drvScaler974.cpp`).
//!
//! # Architecture
//! C's `Scaler974` is itself an `asynPortDriver` (`asynInt32`/
//! `asynInt32Array`) that `devScalerAsyn.c` drives via `pasynUser->reason`
//! commands, which in turn talks to a *separate* underlying serial/GPIB
//! octet port. `scaler-rs`'s [`ScalerDriver`] trait collapses both of those
//! C layers into one set of Rust methods called directly by
//! `ScalerAsynDeviceSupport` — no `asynInt32`/`PortDriver` registration is
//! needed here at all; [`Scaler974Driver`] only needs its own connection to
//! the underlying octet port (via `crate::connect::connect_octet`, same
//! pattern as `delaygen`/`love`).
//!
//! # Background poll thread
//! C spawns `eventThread` unconditionally in the constructor: it blocks on
//! an `epicsEventId`, and once signalled (by `scalerArm(1)`/`arm(true)`)
//! repeatedly sends `SHOW_COUNTS`, sleeping `polltime` (the `poll`
//! constructor argument, milliseconds) between each poll, until it observes
//! `scalerDone` become true — either because `counts[0] >= presetCount`, or
//! because `arm(false)` set `scalerDone` directly from the record-processing
//! thread while a poll was in flight. `read()` (called from the record's
//! process cycle) never does live I/O itself; they only read the
//! cache the poll thread maintains — matching C's `readInt32Array`, which
//! only reads cached `scalerReadSingle` params. This is reproduced with a
//! `std::thread` blocking on an `mpsc::Receiver<()>` "start" signal instead
//! of an `epicsEventId`, and an `Arc<Mutex<PollState>>` instead of
//! `asynPortDriver`'s param list. `scalerDone` is a [`DoneParam`]: set to 1
//! in the constructor (`drvScaler974.cpp:89`), so the port's boot replay
//! processes the scaler record at `iocRun`, and a change reaches the record
//! as `devScalerAsyn`'s done interrupt.
//!
//! # Preserved upstream quirks (not "fixed")
//! - **Only `arm(false)` (STOP) propagates a real I/O failure.** C
//!   `writeInt32` initializes `status = asynSuccess` and only reassigns it
//!   in the `scalerArm` + `value==0` branch (`drvScaler974.cpp:159`); the
//!   `scalerReset` branch (STOP+CLEAR_ALL), the `scalerArm`+`value!=0`
//!   (START) branch, and the `scalerPreset` branch (SET_COUNT_PRESET) all
//!   discard `sendCommand`'s return status and the function always returns
//!   `asynSuccess` for them. [`Scaler974Driver::reset`],
//!   [`Scaler974Driver::arm`] (`start=true` path), and
//!   [`Scaler974Driver::write_preset`] reproduce this — they attempt the
//!   send but always return `Ok`; only `arm(false)` propagates a real
//!   `Err`.
//! - **No per-channel preset.** C's `writeInt32` never reads
//!   `pasynUser->addr`/the `signal` argument devScalerAsyn passes for a
//!   preset write — `setIntegerParam(function, value)` (2-arg, list-0)
//!   always targets the *same* scalar `scalerPreset` param regardless of
//!   channel, and the wire command sent is identical regardless of which
//!   channel triggered it. [`Scaler974Driver::write_preset`] ignores its
//!   `channel` argument for the same reason — this scaler has exactly one
//!   hardware preset register, not one per channel.
//! - **The done-check compares against the raw requested preset, not the
//!   lossy wire-encoded one.** `eventThread` compares `counts[0]` against
//!   `presetCount` read back from the cached `scalerPreset` param — the
//!   *original* value the record wrote, not the `SET_COUNT_PRESET m,n`
//!   value actually programmed into the hardware (see
//!   `wire::encode_set_count_preset`'s doc on the lossy encoding). If the
//!   hardware's own internal stop-at-preset fires at the lossy-rounded
//!   value first, the software done-check compares against a preset the
//!   counts may never reach. This is a latent C driver limitation,
//!   reproduced as-is (`PollState::preset` holds the raw value from the
//!   last `write_preset` call).
//! - **`write_preset` always returns the unmodified `preset`.** Unlike a
//!   clock-owning driver (see the `ScalerDriver::write_preset` trait doc's
//!   VS64 example), C never reads back or reports the `SET_COUNT_PRESET`
//!   quantization to `devScalerAsyn`/the record — `psr->pr1` keeps
//!   whatever the user wrote.
//! - **`SHOW_COUNTS` elicits two reply lines; every other command elicits
//!   one.** C's six-arg `sendCommand` overload with a non-null `response`
//!   buffer (used only by `eventThread`'s `SHOW_COUNTS` call) does a
//!   `writeRead` into `response` (the counts) followed by a *second*,
//!   separate `read` into `statusString` (`drvScaler974.cpp:112-117`) —
//!   whose content is never inspected anywhere in the driver, only its
//!   `asynStatus` (which is itself discarded by the `eventThread` caller).
//!   The four-arg overload (STOP/START/CLEAR_ALL/SET_COUNT_PRESET) does a
//!   single `writeRead` only. Both go through [`send_command`], which
//!   reads the second reply without inspecting it.
//! - **The `timeOut` macro (`drvScaler974.cpp:24`, `0.1`) is dead code** —
//!   never referenced anywhere in the file. The only timeout actually used
//!   is the local `double timeout = 1.0` inside `sendCommand`, applied to
//!   *every* command uniformly. Not reproduced as a separate constant;
//!   folded into the single `Duration` the caller supplies to both
//!   `SyncIOHandle`s (see `Scaler974Driver::new`).
//! - **A negative/zero `poll` argument** cannot be represented by Rust's
//!   unsigned `Duration` the way C's `double polltime = poll/1000.` can
//!   (briefly) go non-positive. C only special-cases exactly `poll==0`
//!   (`drvScaler974.cpp:69`, defaulting to `100`); this treats any
//!   `poll <= 0` the same way, since there is no defined-behavior Rust
//!   `Duration` for a negative interval to fall back to instead.

use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use epics_rs::asyn::error::{AsynError, AsynResult};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::scaler::MAX_SCALER_CHANNELS;
use epics_rs::scaler::device_support::scaler_asyn::{DoneInterrupt, DoneParam, ScalerDriver};

use crate::wire::{encode_set_count_preset, scan_show_counts};

/// asyn reason used for octet transactions against the underlying serial
/// port (mirrors `delaygen::wire::OCTET_REASON`/`love`'s `OCTET_REASON`).
const OCTET_REASON: usize = 0;

/// C `char response[256]` — the reply buffer for the single-read command
/// path (STOP/START/CLEAR_ALL/SET_COUNT_PRESET).
const COMMAND_REPLY_BUF: usize = 256;

/// C `char response[100]` — the `SHOW_COUNTS` counts-data reply buffer.
const SHOW_COUNTS_DATA_BUF: usize = 100;

/// C `char statusString[20]` — `SHOW_COUNTS`'s second (discarded) reply.
const SHOW_COUNTS_STATUS_BUF: usize = 20;

/// C `#define MAX_CHANNELS 4`.
const MAX_CHANNELS: usize = 4;

/// C `if (poll==0) poll=100;` (`drvScaler974.cpp:69`) — milliseconds.
const DEFAULT_POLL_MS: u64 = 100;

fn asyn_to_ca(e: AsynError) -> CaError {
    CaError::Protocol(e.to_string())
}

struct PollState {
    /// Only indices `0..MAX_CHANNELS` are ever written; the rest of the
    /// fixed 64-element array required by [`ScalerDriver::read`] stays
    /// zero, matching a driver with `num_channels() == 4`.
    counts: [u32; MAX_SCALER_CHANNELS],
    /// C `scalerDone`.
    done: DoneParam,
    /// Raw (pre-`SET_COUNT_PRESET`-encoding) value from the last
    /// `write_preset` call, regardless of which channel it named — see
    /// the module doc's "no per-channel preset" quirk.
    preset: u32,
}

pub struct Scaler974Driver {
    handle: SyncIOHandle,
    state: Arc<Mutex<PollState>>,
    start_tx: mpsc::Sender<()>,
}

impl Scaler974Driver {
    /// C `Scaler974::Scaler974(portName, serialPort, serialAddr, poll)`.
    /// `handle` is used for reset/arm/write_preset; `poll_handle` is moved
    /// into the background poll thread for `SHOW_COUNTS` — both must be
    /// independently connected to the same underlying octet port (see
    /// `crate::connect::connect_octet`'s doc for why `SyncIOHandle` isn't
    /// just cloned). `poll` is the constructor's poll interval in
    /// milliseconds.
    pub fn new(handle: SyncIOHandle, poll_handle: SyncIOHandle, poll: i32) -> Self {
        let state = Arc::new(Mutex::new(PollState {
            counts: [0; MAX_SCALER_CHANNELS],
            done: DoneParam::new(1),
            preset: 0,
        }));
        let (start_tx, start_rx) = mpsc::channel();
        let poll_interval = Duration::from_millis(if poll <= 0 {
            DEFAULT_POLL_MS
        } else {
            poll as u64
        });

        let poll_state = state.clone();
        thread::spawn(move || Self::poll_loop(poll_handle, poll_state, start_rx, poll_interval));

        Self {
            handle,
            state,
            start_tx,
        }
    }

    /// C `sendCommand(command, statusString, maxStatusLen, statusLen)` —
    /// the single-read overload used by STOP/START/CLEAR_ALL/
    /// SET_COUNT_PRESET.
    fn send(&self, command: &str) -> CaResult<()> {
        let mut status = Vec::new();
        send_command(&self.handle, command, &mut status, COMMAND_REPLY_BUF, None)
            .map_err(asyn_to_ca)
    }

    /// C `eventThread` (`drvScaler974.cpp:217-254`). `response` and
    /// `counts` are the thread's locals there, declared outside the loop:
    /// a failed `SHOW_COUNTS` leaves both as the last poll left them, so
    /// the record keeps its counts while the port is down.
    fn poll_loop(
        handle: SyncIOHandle,
        state: Arc<Mutex<PollState>>,
        start_rx: mpsc::Receiver<()>,
        poll_interval: Duration,
    ) {
        let mut response = Vec::new();
        let mut counts = [0i32; MAX_CHANNELS];
        while start_rx.recv().is_ok() {
            loop {
                // `eventThread` never checks the status.
                let _ = send_command(
                    &handle,
                    "SHOW_COUNTS",
                    &mut response,
                    SHOW_COUNTS_DATA_BUF,
                    Some(SHOW_COUNTS_STATUS_BUF),
                );
                scan_show_counts(&response, &mut counts);
                let done = {
                    let mut st = state.lock().unwrap();
                    let done = apply_poll_result(&mut st, counts);
                    st.done.call_callbacks();
                    done
                };

                if done {
                    break;
                }
                thread::sleep(poll_interval);
            }
        }
    }
}

/// C `Scaler974::sendCommand` (`drvScaler974.cpp:103-127`): a `writeRead`
/// into `reply`, which a failure leaves untouched; with `status_read`, a
/// second `read` (the `SHOW_COUNTS` status line, never inspected) only
/// after the `writeRead` succeeded.
fn send_command(
    handle: &SyncIOHandle,
    command: &str,
    reply: &mut Vec<u8>,
    reply_buf: usize,
    status_read: Option<usize>,
) -> AsynResult<()> {
    *reply = handle.write_read(OCTET_REASON, command.as_bytes(), reply_buf)?;
    if let Some(status_buf) = status_read {
        handle.read_octet(OCTET_REASON, status_buf)?;
    }
    Ok(())
}

/// C `eventThread`'s per-iteration state update (`drvScaler974.cpp:239-249`):
/// "Get value of done in case scaler was stopped by scalerArm(0)" (i.e.
/// `arm(false)` may have set `done` directly, concurrently, while a poll
/// was in flight), then latch `done` if channel 0's count reached the
/// cached preset. Factored out of [`Scaler974Driver::poll_loop`] so it's
/// unit-testable without a live connection. Returns the (possibly
/// unchanged) `done` value.
fn apply_poll_result(st: &mut PollState, parsed: [i32; 4]) -> bool {
    let mut done = st.done.get() != 0;
    if !done && (parsed[0] as u32) >= st.preset {
        done = true;
    }
    st.done.set(done as i32);
    for (slot, value) in st.counts.iter_mut().zip(parsed).take(MAX_CHANNELS) {
        *slot = value as u32;
    }
    done
}

impl ScalerDriver for Scaler974Driver {
    /// C `scalerReset` branch: STOP then CLEAR_ALL, both status-discarded.
    fn reset(&mut self) -> CaResult<()> {
        let _ = self.send("STOP");
        let _ = self.send("CLEAR_ALL");
        self.state.lock().unwrap().done.call_callbacks();
        Ok(())
    }

    /// C `readInt32Array`: reads only the cached per-channel values — no
    /// live I/O (the poll thread owns that).
    fn read(&mut self, counts: &mut [u32; MAX_SCALER_CHANNELS]) -> CaResult<()> {
        *counts = self.state.lock().unwrap().counts;
        Ok(())
    }

    /// C `scalerPreset` branch. See the module doc: channel-agnostic,
    /// status-discarded, and the raw (unquantized) value is always
    /// returned.
    fn write_preset(&mut self, _channel: usize, preset: u32) -> CaResult<u32> {
        let cmd = encode_set_count_preset(preset);
        let _ = self.send(&cmd);
        let mut st = self.state.lock().unwrap();
        st.preset = preset;
        st.done.call_callbacks();
        Ok(preset)
    }

    /// C `scalerArm` branch. `start=true` (START) discards the send status
    /// and always returns `Ok`; `start=false` (STOP) propagates it.
    fn arm(&mut self, start: bool) -> CaResult<()> {
        if start {
            let _ = self.send("START");
            let st = self.state.lock().unwrap();
            st.done.set(0);
            // C `epicsEventSignal(this->eventId)` — wake the poll thread.
            // The receiver can only be dropped if the poll thread panicked;
            // ignore a send failure the same way C has no recovery path
            // for a dead eventThread either.
            let _ = self.start_tx.send(());
            st.done.call_callbacks();
            Ok(())
        } else {
            let result = self.send("STOP");
            let st = self.state.lock().unwrap();
            st.done.set(1);
            st.done.call_callbacks();
            result
        }
    }

    fn register_done_interrupt(&mut self, user: DoneInterrupt) {
        self.state.lock().unwrap().done.register(user);
    }

    /// C `#define MAX_CHANNELS 4`.
    fn num_channels(&self) -> usize {
        MAX_CHANNELS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh_state() -> PollState {
        PollState {
            counts: [0; MAX_SCALER_CHANNELS],
            done: DoneParam::new(0),
            preset: 0,
        }
    }

    #[test]
    fn apply_poll_result_updates_counts_and_leaves_undone_below_preset() {
        let mut st = fresh_state();
        st.preset = 100;
        let done = apply_poll_result(&mut st, [42, 1, 2, 3]);
        assert!(!done);
        assert_eq!(&st.counts[..4], &[42, 1, 2, 3]);
        assert_eq!(st.done.get(), 0);
    }

    #[test]
    fn apply_poll_result_latches_done_when_channel0_reaches_preset() {
        let mut st = fresh_state();
        st.preset = 100;
        let done = apply_poll_result(&mut st, [100, 0, 0, 0]);
        assert!(done);
        assert_eq!(st.done.get(), 1);

        // A later poll with a lower count (e.g. hardware reset mid-poll)
        // must not un-latch done -- `!done` is required before comparing.
        let done2 = apply_poll_result(&mut st, [5, 0, 0, 0]);
        assert!(done2);
    }

    #[test]
    fn apply_poll_result_respects_externally_set_done() {
        // Mirrors arm(false) setting `done=true` concurrently while a poll
        // was in flight -- the next poll iteration must observe it and not
        // require counts to reach preset.
        let mut st = fresh_state();
        st.preset = 1_000_000;
        st.done.set(1);
        let done = apply_poll_result(&mut st, [0, 0, 0, 0]);
        assert!(done);
    }

    #[test]
    fn apply_poll_result_only_populates_first_four_channels() {
        let mut st = fresh_state();
        st.counts[10] = 999; // pre-existing garbage beyond MAX_CHANNELS
        apply_poll_result(&mut st, [1, 2, 3, 4]);
        assert_eq!(&st.counts[..4], &[1, 2, 3, 4]);
        // Channels beyond MAX_CHANNELS are never touched by the driver.
        assert_eq!(st.counts[10], 999);
    }
}
