//! Detector-side operations shared by the asyn port driver and the acquisition
//! task (the `set*` / `get*` methods of C's `mythen` class).
//!
//! The transport lives behind a mutex here: a command/reply pair has to be
//! atomic against the acquisition task's readout, which C gets for free from
//! asyn's per-port request queue.

use std::num::NonZeroUsize;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::time::Duration;

use epics_libcom_rs::cfmt::CFloat;
use epics_rs::ad_core::driver::ADStatus;
use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::trace::{TraceManager, TraceMask};
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
    /// The mythen port C's `pasynUserSelf` prints on.
    port_name: String,
    /// C's `outString_`: the last command written to it, which C's
    /// refusal lines print whether or not it was sent.
    out_string: Mutex<String>,
    trace: Arc<TraceManager>,
}

impl Detector {
    pub fn new(transport: Transport, port_name: &str) -> Self {
        Self {
            trace: transport.trace(),
            transport: Mutex::new(transport),
            // Optimistic: the first command is what finds out.
            connected: AtomicBool::new(true),
            state: DetState::default(),
            port_name: port_name.to_string(),
            out_string: Mutex::default(),
        }
    }

    pub fn out_string(&self) -> String {
        self.out_string.lock().clone()
    }

    /// C's `"<check> (outString = %s)"` refusal on `pasynUserSelf` and the
    /// asynError that follows it (mythen.cpp:365, :392, :414).
    pub fn refused(&self, check: &str) -> AsynError {
        self.trace_error(&format!("{check} (outString = {})", self.out_string()));
        AsynError::Status {
            status: AsynStatus::Error,
            message: String::new(),
        }
    }

    /// An `ASYN_TRACE_ERROR` line on the mythen port.
    pub fn trace_error(&self, message: &str) {
        self.trace
            .output_device(&self.port_name, Some(0), 0, TraceMask::ERROR, message);
    }

    /// C `writeReadMeter`, mythen.cpp:239, which prints on every failed
    /// exchange — including one the asyn port refused while disconnected.
    fn meter<T>(
        &self,
        what: &str,
        exchange: impl FnOnce(&Transport) -> AsynResult<T>,
    ) -> AsynResult<T> {
        *self.out_string.lock() = what.to_string();
        let result = self.exchange(exchange);
        if result.is_err() {
            self.trace_error("mythen:writeReadMeter: error!\n");
        }
        result
    }

    /// Whether the detector is currently believed to be answering.
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Acquire)
    }

    /// Run one command and let its outcome own the connection state.
    ///
    /// The only writer of `connected`, and the only path to the socket. There
    /// is no gate in front of it: as in C, a detector that does not answer is
    /// refused by the asyn port itself, asynError for the send that fails and
    /// asynDisconnected while the port is down (asynManager.c:1549-1553). Every
    /// error inside here is the detector failing to answer in full — a timeout
    /// with nothing in hand, or a reply too short to decode — so every one of
    /// them means disconnected. Replies the detector *did* send and the driver
    /// rejects (`-get tau` out of range, a negative module count) are checked by
    /// the callers, above this line, and leave the state alone.
    fn exchange<T>(&self, exchange: impl FnOnce(&Transport) -> AsynResult<T>) -> AsynResult<T> {
        let result = exchange(&self.transport.lock());
        self.connected.store(result.is_ok(), Ordering::Release);
        result
    }

    /// The probe that brings a detector that stopped answering back.
    ///
    /// C has no such path — a detector that was off when the IOC booted stays
    /// unusable until the IOC is restarted, because C's constructor is the only
    /// place that reads the module count (mythen.cpp:1363).
    pub fn probe(&self) -> AsynResult<String> {
        *self.out_string.lock() = "-get version".to_string();
        let version = self
            .exchange(Transport::get_version)
            .inspect_err(|_| self.trace_error("mythen:writeReadMeter: error!\n"))?;
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

    /// Send a command and check the status integer it replies with
    /// (C `sendCommand`, mythen.cpp:221, which reports a negative one and
    /// carries on).
    pub fn send(&self, command: &str) -> AsynResult<()> {
        let value = self.meter(command, |t| t.send_command(command))?;
        if value < 0 {
            self.trace_error(&format!(
                "mythen:sendCommand: error, expected 0, received {value}\n"
            ));
        }
        Ok(())
    }

    fn get_int(&self, command: &str) -> AsynResult<i32> {
        self.meter(command, |t| t.get_int(command))
    }

    fn get_float(&self, command: &str) -> AsynResult<f32> {
        self.meter(command, |t| t.get_float(command))
    }

    /// C `getFirmware`, mythen.cpp:497.
    pub fn get_firmware(&self) -> AsynResult<String> {
        let version = self.meter("-get version", Transport::get_version)?;
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
        // UPSTREAM DEFECT (mythen.cpp:1364-1373): C tests `aux < 0` on the
        // leftover inString_ of a failed read and sets SDNModules from it.
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
        // UPSTREAM DEFECT (mythen.cpp:515-516): C decodes a failed read's
        // leftover inString_ as the status word; rs returns the error.
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
        // C sends `-stop` straight through pasynOctetSyncIO, not
        // writeReadMeter, and prints its own line.
        self.exchange(|t| t.send_command("-stop"))
            .inspect_err(|_| self.trace_error("mythen:setAcquire: error!\n"))?;
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
            self.send(&format!("-tau {value:.6}", value = CFloat(value)))?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// C `setKthresh`, mythen.cpp:374 — set on every module in turn.
    pub fn set_kthresh(&self, value: f64) -> AsynResult<()> {
        self.for_each_module(&format!("-kthresh {value:.6}", value = CFloat(value)))
    }

    /// C `setEnergy`, mythen.cpp:401 — firmware 3 and later only.
    pub fn set_energy(&self, value: f64) -> AsynResult<()> {
        if self.firmware_major() < 3 {
            return Ok(());
        }
        self.for_each_module(&format!("-energy {value:.6}", value = CFloat(value)))
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
    /// Like C it runs zero times while the module count is unknown, carries on
    /// past a failed command and returns the last command's status.
    fn for_each_module(&self, command: &str) -> AsynResult<()> {
        let nmodules = self.nmodules().unwrap_or(0);
        let mut status = Ok(());
        for module in 0..nmodules {
            let _ = self.send(&format!("-module {module}"));
            status = self.send(command);
        }
        status
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
        // UPSTREAM DEFECT (mythen.cpp:696-712): C ignores writeReadMeter's
        // status and decodes whatever a failed read left in inString_, so a
        // silent detector publishes these as 0. rs fails the read instead.
        let use_flat_field = self.get_bool("-get flatfieldcorrection")?;
        let use_bad_chan_intrpl = self.get_bool("-get badchannelinterpolation")?;
        let use_count_rate = self.get_bool("-get ratecorrection")?;

        let nbits = self.get_int("-get nbits")?;
        // UPSTREAM DEFECT (mythen.cpp:717-719): C accepts 0 here and then
        // divides by it (`chanperline_ = 32/aux`, SIGFPE). 0 is kept as C
        // keeps it; `protocol::channels_per_word` reads it as one channel.
        if nbits < 0 {
            return Err(self.settings_error("-get nbits", nbits.to_string()));
        }
        self.state.nbits.store(nbits, Ordering::Release);

        let time = self.get_int("-get time")?;
        if time < 0 {
            return Err(self.settings_error("-get time", time.to_string()));
        }
        let acquire_time = protocol::from_hundred_ns(time);

        let frames = self.get_int("-get frames")?;
        let frames = (frames >= 0).then_some(frames);

        let tau = self.get_float("-get tau")?;
        if !(tau == -1.0 || tau > 0.0) {
            return Err(self.settings_error("-get tau", CFloat(f64::from(tau)).g().to_string()));
        }

        let threshold = self.get_float("-get kthresh")?;
        if threshold < 0.0 {
            return Err(
                self.settings_error("-get kthresh", CFloat(f64::from(threshold)).g().to_string())
            );
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
                return Err(
                    self.settings_error("-get energy", CFloat(f64::from(energy)).g().to_string())
                );
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
                return Err(self.settings_error("-get delafter", delay.to_string()));
            }
            settings.delay_time = Some(protocol::from_hundred_ns(delay));

            let cont = self.get_int("-get conttrig")?;
            if cont < 0 {
                return Err(self.settings_error("-get conttrig", cont.to_string()));
            }
            settings.trigger = Some(if cont == 1 {
                protocol::TRIGGER_CONTINUOUS
            } else {
                let single = self.get_int("-get trig")?;
                if single < 0 {
                    return Err(self.settings_error("-get trig", single.to_string()));
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
            return Err(self.settings_error(command, value.to_string()));
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
        *self.out_string.lock() = command.to_string();
        self.exchange(|t| t.readout(command, expect, timeout))
    }
}

impl Detector {
    /// C `getSettings`' `goto error`, mythen.cpp:791.
    fn settings_error(&self, command: &str, got: String) -> AsynError {
        self.trace_error(&format!(
            "mythen:getSettings: error, outString={command}, inString={got}\n"
        ));
        AsynError::Status {
            status: AsynStatus::Error,
            message: format!("mythen: [{command}] unexpected reply: {got}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::interpose::{EomReason, OctetReadResult};
    use epics_rs::asyn::port::{InterfaceMask, PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::claim_port;
    use epics_rs::asyn::runtime::config::RuntimeConfig;
    use epics_rs::asyn::runtime::port::create_port_runtime;
    use epics_rs::asyn::user::AsynUser;

    /// A mythen that answers every command with status 0.
    struct Ok0(PortDriverBase);

    impl PortDriver for Ok0 {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            buf[..4].fill(0);
            OctetReadResult::ok(4, EomReason::CNT)
        }
    }

    /// A mythen that answers each `-get` float query with its scripted value
    /// and every other query with 0.
    struct Floats {
        base: PortDriverBase,
        last: Vec<u8>,
        floats: [(&'static str, f32); 3],
    }

    impl PortDriver for Floats {
        fn base(&self) -> &PortDriverBase {
            &self.base
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.base
        }
        fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
            self.last = data.to_vec();
            Ok(data.len())
        }
        fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> OctetReadResult {
            let reply = self
                .floats
                .iter()
                .find(|(command, _)| command.as_bytes() == self.last)
                .map_or([0; 4], |(_, v)| v.to_le_bytes());
            buf[..4].copy_from_slice(&reply);
            OctetReadResult::ok(4, EomReason::CNT)
        }
    }

    /// mythen.cpp:793 traces a refused reply with C's raw `inString`; rs
    /// names the float it decoded, as %g since that text has no C spelling.
    #[test]
    fn refused_float_reply_names_the_value_as_c_percent_g() {
        for (i, (refused, value, text)) in [
            ("-get tau", f32::NAN, "nan"),
            ("-get tau", -f32::NAN, "-nan"),
            ("-get kthresh", -1e-5, "-1e-05"),
            ("-get energy", -2.5e-7, "-2.5e-07"),
        ]
        .into_iter()
        .enumerate()
        {
            let name = format!("MYTHEN_DET_REFUSED_FLOAT_{i}");
            let base = PortDriverBase::new(
                &name,
                1,
                PortFlags::default(),
                InterfaceMask::OCTET_TRANSPORT,
                InterfaceMask::OCTET,
            );
            let mut floats = [
                ("-get tau", -1.0),
                ("-get kthresh", 0.0),
                ("-get energy", 0.0),
            ];
            floats.iter_mut().find(|(c, _)| *c == refused).unwrap().1 = value;
            let driver = Floats {
                base,
                last: Vec::new(),
                floats,
            };
            let (runtime, _) = create_port_runtime(
                &claim_port(&name).unwrap(),
                driver,
                RuntimeConfig::default(),
            )
            .unwrap();
            let det = Detector::new(Transport::new(runtime.port_handle().clone()), &name);
            det.state.firmware_major.store(3, Ordering::Release);
            match det.get_settings() {
                Err(AsynError::Status { message, .. }) => {
                    assert_eq!(
                        message,
                        format!("mythen: [{refused}] unexpected reply: {text}")
                    );
                }
                other => panic!("{refused}: {other:?}"),
            }
        }
    }

    /// mythen.cpp:381/:410 write `-kthresh %f` / `-energy %f`.
    #[test]
    fn module_float_commands_print_as_c_percent_f() {
        let name = "MYTHEN_DET_FLOAT_CMD";
        let base = PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::OCTET_TRANSPORT,
            InterfaceMask::OCTET,
        );
        let (runtime, _) = create_port_runtime(
            &claim_port(name).unwrap(),
            Ok0(base),
            RuntimeConfig::default(),
        )
        .unwrap();
        let det = Detector::new(Transport::new(runtime.port_handle().clone()), name);
        det.state.nmodules.store(1, Ordering::Release);
        det.state.firmware_major.store(3, Ordering::Release);
        for (v, text) in [(f64::NAN, "nan"), (-f64::NAN, "-nan")] {
            det.set_kthresh(v).unwrap();
            assert_eq!(det.out_string(), format!("-kthresh {text}"));
            det.set_energy(v).unwrap();
            assert_eq!(det.out_string(), format!("-energy {text}"));
        }
    }
}
