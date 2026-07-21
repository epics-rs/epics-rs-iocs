//! Detector-side operations shared by the asyn port driver and the acquisition
//! task (the `set*` / `get*` methods of C's `mythen` class).
//!
//! The transport lives behind a mutex here: a command/reply pair has to be
//! atomic against the acquisition task's readout, which C gets for free from
//! asyn's per-port request queue.

use std::num::NonZeroUsize;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::time::Duration;

use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use parking_lot::Mutex;

use crate::protocol::{self, READ_MODE_RAW};
use crate::transport::Transport;

/// How long C waits for a trigger before giving up
/// (C `MAX_TRIGGER_TIMEOUT_COUNT`).
const MAX_TRIGGER_TIMEOUT_COUNT: u32 = 50;

/// Everything the driver and the acquisition task both need to see.
#[derive(Debug, Default)]
pub struct DetState {
    /// C `acquiring_`.
    pub acquiring: AtomicBool,
    /// C `readmode_` (0 = raw, 1 = corrected).
    pub read_mode: AtomicI32,
    /// C `nbits_`, the detector's current bit depth.
    pub nbits: AtomicI32,
    /// C `nmodules`.
    pub nmodules: AtomicI32,
    /// C `frames_`, the frame count the user asked for.
    pub frames: AtomicI32,
    /// The major version of the firmware, which decides which commands exist.
    pub firmware_major: AtomicU32,
}

/// Everything `getSettings` reads back in one pass (C `getSettings`,
/// mythen.cpp:680). The optional fields exist only on firmware 3 and later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub use_flat_field: i32,
    pub use_bad_chan_intrpl: i32,
    pub use_count_rate: i32,
    pub nbits: i32,
    pub acquire_time: f64,
    pub frames: Option<i32>,
    pub tau: f64,
    pub threshold: f64,
    pub energy: Option<f64>,
    pub delay_time: Option<f64>,
    pub trigger: Option<i32>,
}

pub struct Detector {
    transport: Mutex<Transport>,
    /// Whether the detector is answering.
    ///
    /// Owned here and written in exactly one place, [`Detector::exchange`]: the
    /// outcome of a real command is the only thing that may change it.
    connected: AtomicBool,
    pub state: DetState,
}

impl Detector {
    pub fn new(transport: Transport) -> Self {
        Self {
            transport: Mutex::new(transport),
            // Optimistic: the first command is what finds out.
            connected: AtomicBool::new(true),
            state: DetState::default(),
        }
    }

    /// Whether the detector is currently believed to be answering.
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }

    /// The gate every detector command passes through.
    ///
    /// INVARIANT: while the detector is marked disconnected no command reaches
    /// the socket. Each one would otherwise sit out the full 5 s
    /// [`M1K_TIMEOUT`](crate::transport::M1K_TIMEOUT), and they are serialized
    /// through the port's single request queue — the 37 records that process at
    /// `iocInit` cost minutes between them instead of nothing at all.
    ///
    /// The caller gets [`AsynStatus::Disconnected`] instead, which is the
    /// record layer's own word for it.
    fn io<T>(
        &self,
        what: &str,
        exchange: impl FnOnce(&Transport) -> AsynResult<T>,
    ) -> AsynResult<T> {
        if !self.is_connected() {
            return Err(disconnected(what));
        }
        self.exchange(what, exchange)
    }

    /// Run one command and let its outcome own the connection state.
    ///
    /// The only writer of `connected`, and the only path to the socket. Every
    /// error inside here is the detector failing to answer in full — a timeout
    /// with nothing in hand, or a reply too short to decode — so every one of
    /// them means disconnected. Replies the detector *did* send and the driver
    /// rejects (`-get tau` out of range, a negative module count) are checked by
    /// the callers, above this line, and leave the state alone.
    fn exchange<T>(
        &self,
        what: &str,
        exchange: impl FnOnce(&Transport) -> AsynResult<T>,
    ) -> AsynResult<T> {
        let result = exchange(&self.transport.lock());
        match &result {
            Ok(_) => {
                if !self.connected.swap(true, Ordering::AcqRel) {
                    log::warn!("mythen: [{what}] answered; the detector is connected again");
                }
            }
            Err(e) => {
                if self.connected.swap(false, Ordering::AcqRel) {
                    log::error!(
                        "mythen: [{what}] got no answer ({e}); the detector is marked \
                         disconnected and no further command will be sent until an acquisition \
                         probes it"
                    );
                }
            }
        }
        result
    }

    /// The one command allowed past the gate: the probe that can bring the
    /// detector back.
    ///
    /// C has no such path — a detector that was off when the IOC booted stays
    /// unusable until the IOC is restarted, because C's constructor is the only
    /// place that reads the module count (mythen.cpp:1363).
    pub fn probe(&self) -> AsynResult<String> {
        let version = self.exchange("-get version", Transport::get_version)?;
        self.store_firmware(&version);
        Ok(version)
    }

    /// How many modules the detector reported, or `None` while that is not
    /// known — `-get nmodules` has not answered yet, or answered nonsense.
    ///
    /// Deliberately not a count of zero. Every readout length is derived from
    /// this, and a zero-length readout is a command sent whose reply nobody
    /// reads: the bytes stay in the socket and desynchronise every command
    /// after it. `None` is what stops that being expressible.
    pub fn nmodules(&self) -> Option<usize> {
        match self.state.nmodules.load(Ordering::Acquire) {
            n if n > 0 => Some(n as usize),
            _ => None,
        }
    }

    pub fn firmware_major(&self) -> u32 {
        self.state.firmware_major.load(Ordering::Acquire)
    }

    /// Send a command and check the status integer it replies with.
    pub fn send(&self, command: &str) -> AsynResult<()> {
        self.io(command, |t| t.send_command(command))?;
        Ok(())
    }

    fn get_int(&self, command: &str) -> AsynResult<i32> {
        self.io(command, |t| t.get_int(command))
    }

    fn get_float(&self, command: &str) -> AsynResult<f32> {
        self.io(command, |t| t.get_float(command))
    }

    /// C `getFirmware`, mythen.cpp:497.
    pub fn get_firmware(&self) -> AsynResult<String> {
        let version = self.io("-get version", Transport::get_version)?;
        self.store_firmware(&version);
        Ok(version)
    }

    fn store_firmware(&self, version: &str) {
        self.state
            .firmware_major
            .store(protocol::firmware_major(version), Ordering::Release);
    }

    /// C `-get nmodules` in the constructor, mythen.cpp:1363.
    pub fn read_nmodules(&self) -> AsynResult<i32> {
        let n = self.get_int("-get nmodules")?;
        if n > 0 {
            self.state.nmodules.store(n, Ordering::Release);
        }
        Ok(n)
    }

    /// C `getStatus`, mythen.cpp:509.
    ///
    /// While the detector reports "waiting for trigger" this backs off for an
    /// increasing amount of time — up to about a minute in total — and reports
    /// [`ADStatus::Error`] if the trigger never arrives.
    pub fn get_status(&self) -> AsynResult<ADStatus> {
        let mut bits = protocol::status_bits(self.get_int("-get status")?);

        if protocol::is_idle(bits) {
            return Ok(ADStatus::Idle);
        }

        let mut waited = 0;
        while bits.waiting_for_trigger && waited < MAX_TRIGGER_TIMEOUT_COUNT {
            std::thread::sleep(protocol::trigger_backoff(waited));
            bits = protocol::status_bits(self.get_int("-get status")?);
            waited += 1;
        }

        Ok(protocol::status_after_wait(
            bits,
            waited == MAX_TRIGGER_TIMEOUT_COUNT,
        ))
    }

    /// C `setAcquire(1)`, mythen.cpp:287.
    pub fn start(&self) -> AsynResult<bool> {
        if self.state.acquiring.load(Ordering::Acquire) {
            return Ok(false);
        }
        self.send("-start")?;
        self.state.acquiring.store(true, Ordering::Release);
        Ok(true)
    }

    /// C `setAcquire(0)`, mythen.cpp:281.
    ///
    /// UPSTREAM DEFECT (mythen.cpp:281): C sends `-stop` with
    /// `strlen(outString_)` — the length of whatever command happened to be in
    /// the buffer last — so the detector receives a truncated `-stop` (or
    /// `-stop` followed by stale bytes) depending on what ran before it. Here
    /// the command carries its own length.
    pub fn stop(&self) -> AsynResult<()> {
        self.state.acquiring.store(false, Ordering::Release);
        self.io("-stop", |t| t.send_command("-stop"))?;
        Ok(())
    }

    /// C `setFrames`, mythen.cpp:308. Single-image mode always asks the
    /// detector for exactly one frame.
    pub fn set_frames(&self, value: i32, image_mode: i32) -> AsynResult<()> {
        let frames = if image_mode == protocol::IMAGE_MODE_SINGLE {
            1
        } else {
            value
        };
        self.send(&format!("-frames {frames}"))?;
        self.state.frames.store(value, Ordering::Release);
        Ok(())
    }

    /// C `setTrigger`, mythen.cpp:331.
    pub fn set_trigger(&self, mode: i32) -> AsynResult<()> {
        match protocol::trigger_command(mode) {
            Some(command) => self.send(command),
            None => Ok(()),
        }
    }

    /// C `setTau`, mythen.cpp:354. Only -1 ("no correction") and positive
    /// constants are legal.
    pub fn set_tau(&self, value: f64) -> AsynResult<bool> {
        if value == -1.0 || value > 0.0 {
            self.send(&format!("-tau {value:.6}"))?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// C `setKthresh`, mythen.cpp:374 — set on every module in turn.
    pub fn set_kthresh(&self, value: f64) -> AsynResult<()> {
        self.for_each_module(&format!("-kthresh {value:.6}"))
    }

    /// C `setEnergy`, mythen.cpp:401 — firmware 3 and later only.
    pub fn set_energy(&self, value: f64) -> AsynResult<()> {
        if self.firmware_major() < 3 {
            return Ok(());
        }
        self.for_each_module(&format!("-energy {value:.6}"))
    }

    /// C `loadSettings`, mythen.cpp:599.
    pub fn load_settings(&self, index: i32) -> AsynResult<()> {
        let command = protocol::settings_command(index, self.firmware_major());
        self.for_each_module(command)
    }

    /// C `setReset`, mythen.cpp:660.
    pub fn reset(&self) -> AsynResult<()> {
        self.for_each_module("-reset")
    }

    /// `-module N` followed by `command`, for every module (C repeats this
    /// pattern in setKthresh / setEnergy / loadSettings / setReset).
    ///
    /// Refuses when the module count is unknown rather than looping zero times:
    /// a silent no-op here is a threshold the operator believes was applied.
    fn for_each_module(&self, command: &str) -> AsynResult<()> {
        let nmodules = self
            .nmodules()
            .ok_or_else(|| unknown_nmodules(&format!("send [{command}] to every module")))?;
        for module in 0..nmodules {
            self.send(&format!("-module {module}"))?;
            self.send(command)?;
        }
        Ok(())
    }

    pub fn set_exposure_time(&self, seconds: f64) -> AsynResult<()> {
        self.send(&format!("-time {}", protocol::to_hundred_ns(seconds)))
    }

    pub fn set_delay_after_trigger(&self, seconds: f64) -> AsynResult<()> {
        self.send(&format!("-delafter {}", protocol::to_hundred_ns(seconds)))
    }

    pub fn set_flat_field_correction(&self, value: i32) -> AsynResult<()> {
        self.send(&format!("-flatfieldcorrection {value}"))
    }

    pub fn set_bad_chan_intrpl(&self, value: i32) -> AsynResult<()> {
        self.send(&format!("-badchannelinterpolation {value}"))
    }

    pub fn set_rate_correction(&self, value: i32) -> AsynResult<()> {
        self.send(&format!("-ratecorrection {value}"))
    }

    pub fn set_use_gates(&self, value: i32) -> AsynResult<()> {
        self.send(&format!("-gateen {value}"))
    }

    pub fn set_num_gates(&self, value: i32) -> AsynResult<()> {
        self.send(&format!("-gates {value}"))
    }

    pub fn set_bit_depth(&self, index: i32) -> AsynResult<()> {
        self.send(&format!("-nbits {}", protocol::nbits_of_bit_depth(index)))
    }

    /// C `getSettings`, mythen.cpp:680.
    ///
    /// A reply outside the range C checks for (`goto error`) fails the whole
    /// read, exactly as in C.
    pub fn get_settings(&self) -> AsynResult<Settings> {
        let use_flat_field = self.get_bool("-get flatfieldcorrection")?;
        let use_bad_chan_intrpl = self.get_bool("-get badchannelinterpolation")?;
        let use_count_rate = self.get_bool("-get ratecorrection")?;

        let nbits = self.get_int("-get nbits")?;
        if nbits < 0 {
            return Err(settings_error("-get nbits", nbits.to_string()));
        }
        self.state.nbits.store(nbits, Ordering::Release);

        let time = self.get_int("-get time")?;
        if time < 0 {
            return Err(settings_error("-get time", time.to_string()));
        }
        let acquire_time = protocol::from_hundred_ns(time);

        let frames = self.get_int("-get frames")?;
        let frames = (frames >= 0).then_some(frames);

        let tau = self.get_float("-get tau")?;
        if !(tau == -1.0 || tau > 0.0) {
            return Err(settings_error("-get tau", tau.to_string()));
        }

        let threshold = self.get_float("-get kthresh")?;
        if threshold < 0.0 {
            return Err(settings_error("-get kthresh", threshold.to_string()));
        }

        let mut settings = Settings {
            use_flat_field,
            use_bad_chan_intrpl,
            use_count_rate,
            nbits,
            acquire_time,
            frames,
            tau: f64::from(tau),
            threshold: f64::from(threshold),
            energy: None,
            delay_time: None,
            trigger: None,
        };

        if self.firmware_major() >= 3 {
            let energy = self.get_float("-get energy")?;
            if energy < 0.0 {
                return Err(settings_error("-get energy", energy.to_string()));
            }
            settings.energy = Some(f64::from(energy));

            // UPSTREAM DEFECT (mythen.cpp:761): C parses this reply with
            // `stringToInt64`, reading eight bytes out of a buffer into which
            // `writeReadMeter` only ever read four (mythen.cpp:257) — the upper
            // half of the "delay" is whatever the previous reply left behind.
            // `-delafter` is a 100 ns count like `-time`, which C reads as a
            // 4-byte int, and `writeReadMeter` special-cases only `-get tau`
            // and `-get version`: the reply is four bytes by construction.
            let delay = self.get_int("-get delafter")?;
            if delay < 0 {
                return Err(settings_error("-get delafter", delay.to_string()));
            }
            settings.delay_time = Some(protocol::from_hundred_ns(delay));

            let cont = self.get_int("-get conttrig")?;
            if cont < 0 {
                return Err(settings_error("-get conttrig", cont.to_string()));
            }
            settings.trigger = Some(if cont == 1 {
                protocol::TRIGGER_CONTINUOUS
            } else {
                let single = self.get_int("-get trig")?;
                if single < 0 {
                    return Err(settings_error("-get trig", single.to_string()));
                }
                if single == 1 {
                    protocol::TRIGGER_SINGLE
                } else {
                    protocol::TRIGGER_NONE
                }
            });
        }

        Ok(settings)
    }

    /// A `-get` whose only legal replies are 0 and 1.
    fn get_bool(&self, command: &str) -> AsynResult<i32> {
        let value = self.get_int(command)?;
        if value != 0 && value != 1 {
            return Err(settings_error(command, value.to_string()));
        }
        Ok(value)
    }

    /// The size, in bytes, of the next readout reply (C `nread_expect`), or
    /// `None` while the module count is unknown.
    pub fn readout_len(&self) -> Option<NonZeroUsize> {
        NonZeroUsize::new(protocol::readout_len(
            self.state.read_mode.load(Ordering::Acquire),
            self.nmodules()?,
            self.state.nbits.load(Ordering::Acquire),
        ))
    }

    /// C `-readoutraw` / `-readout`, mythen.cpp:889.
    ///
    /// `expect` is a [`NonZeroUsize`] because it is the length the reply will
    /// be read back at: a zero would send the command and read nothing, and the
    /// reply the detector then sends would be picked up as the answer to
    /// whatever command came next.
    pub fn readout(&self, expect: NonZeroUsize, timeout: Duration) -> AsynResult<Vec<u8>> {
        let command = if self.state.read_mode.load(Ordering::Acquire) == READ_MODE_RAW {
            "-readoutraw"
        } else {
            "-readout"
        };
        self.io(command, |t| t.readout(command, expect, timeout))
    }
}

/// The error a caller gets while the detector is marked disconnected: the
/// command was never sent.
fn disconnected(what: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Disconnected,
        message: format!(
            "mythen: [{what}] not sent: the detector is disconnected — write ADAcquire to probe \
             it again"
        ),
    }
}

/// The error a caller gets when it asks for detector work whose size depends on
/// a module count the detector has never reported.
pub fn unknown_nmodules(what: &str) -> AsynError {
    AsynError::Status {
        status: AsynStatus::Error,
        message: format!(
            "mythen: cannot {what}: the module count is unknown — `-get nmodules` has not \
             answered since the IOC started"
        ),
    }
}

fn settings_error(command: &str, got: String) -> epics_rs::asyn::error::AsynError {
    epics_rs::asyn::error::AsynError::Status {
        status: epics_rs::asyn::error::AsynStatus::Error,
        message: format!("mythen: [{command}] unexpected reply: {got}"),
    }
}
