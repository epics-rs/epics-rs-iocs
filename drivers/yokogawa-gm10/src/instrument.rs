//! The GM10 instrument actor: owns the blocking `TcpStream` and `Cache`,
//! serializing every wire operation through one OS thread (mirrors C's
//! single-consumer `queue_func`/`qmesg` pair, `drvGM10.c:1311-1375` —
//! `qmesg` enqueues from any EPICS scan thread, `queue_func` is the sole
//! socket owner). Writes queue through [`Instrument::enqueue`] and complete
//! the record from the actor, as C does. Reads block on
//! [`Instrument::submit`]: the framework has no asynchronous read completion.

use crate::cache::*;
use crate::codec::*;
use crate::link::ChannelFamily;
use crate::wire::{self, RawResponse};
use epics_rs::base::server::driver_support::DriverSupport;
use parking_lot::Mutex;
use std::io;
use std::net::TcpStream;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::sync::mpsc;
use std::sync::{Arc, OnceLock};

const PORT: u16 = 34434;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptCategory {
    Channel,
    Misc,
    Info,
    Status,
    Error,
}

#[derive(Default)]
struct Interrupts {
    channel: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    misc: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    info: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    status: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    error: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
}

impl Interrupts {
    fn slot(&self, category: InterruptCategory) -> &Mutex<Vec<tokio::sync::mpsc::Sender<()>>> {
        match category {
            InterruptCategory::Channel => &self.channel,
            InterruptCategory::Misc => &self.misc,
            InterruptCategory::Info => &self.info,
            InterruptCategory::Status => &self.status,
            InterruptCategory::Error => &self.error,
        }
    }

    fn register(&self, category: InterruptCategory) -> tokio::sync::mpsc::Receiver<()> {
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        self.slot(category).lock().push(tx);
        rx
    }

    /// Coalescing wake, matching `scanIoRequest`: a `Full` error just means
    /// a scan is already pending, which is fine to drop; only a `Closed`
    /// receiver (record torn down) gets pruned.
    fn fire(&self, category: InterruptCategory) {
        let mut senders = self.slot(category).lock();
        senders.retain(|tx| {
            !matches!(
                tx.try_send(()),
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_))
            )
        });
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    ReadAllData,
    ReadSignal(u32),
    ReadMath(u32),
    ReadComm(u32),
    ReadAllMisc,
    ReadConst(u32),
    ReadVarConst(u32),
    ReadAllInfos,
    ReadStatus,
    SetSignalOutput(u32, f64),
    SetComm(u32, f64),
    SetConst(u32, f64),
    SetVarConst(u32, f64),
    SetBinaryOutput(u32, bool),
    SetOpMode(u16),
    SetCompute(u16),
    ClearError,
    AcknowledgeAlarms,
}

/// Run by the actor once its command was processed — C `queue_func`'s
/// `rset->process` call (`drvGM10.c:1327-1332`).
pub type Completion = Box<dyn FnOnce() + Send>;

struct Job {
    command: Command,
    done: Completion,
}

/// One `gm10Init` handle (C `struct devqueue`), looked up by name from the
/// [`Registry`].
pub struct Instrument {
    cache: Arc<Mutex<Cache>>,
    interrupts: Arc<Interrupts>,
    /// The `queue_func` thread's queue; unset when `init_gm10` failed after
    /// registering, which leaves C's `dq` with no thread to drain it.
    tx: OnceLock<mpsc::Sender<Job>>,
    /// `gm10_system_info(which=0, ...)` (`drvGM10.c:1620-1631`): the
    /// configured address, as `inet_ntop` prints it.
    pub peer_address: String,
}

impl Instrument {
    fn new(peer_address: String) -> Arc<Self> {
        Arc::new(Self {
            cache: Arc::new(Mutex::new(Cache::default())),
            interrupts: Arc::new(Interrupts::default()),
            tx: OnceLock::new(),
            peer_address,
        })
    }

    #[cfg(test)]
    pub(crate) fn connect_to(addr: std::net::SocketAddr) -> io::Result<Arc<Self>> {
        let instrument = Self::new(addr.ip().to_string());
        instrument.start(TcpStream::connect(addr)?)?;
        Ok(instrument)
    }

    /// The tail of `init_gm10` (`drvGM10.c:1446-1461`): read the unsolicited
    /// post-connect `E0` greeting, run the fixed load sequence (modules
    /// first, everything else indexes off it), then start the queue thread.
    fn start(&self, stream: TcpStream) -> io::Result<()> {
        let (tx, rx) = mpsc::channel::<Job>();
        let mut actor = Actor {
            stream,
            cache: self.cache.clone(),
            interrupts: self.interrupts.clone(),
            requeue: tx.clone(),
        };

        match actor.read_response()? {
            RawResponse::Ok => {}
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "no initial E0 greeting after connect",
                ));
            }
        }

        actor.load_modules()?;
        actor.load_status()?;
        actor.load_infos()?;
        actor.load_data_values(None)?;
        actor.load_misc_values(true, true, None)?;

        std::thread::Builder::new()
            .name("gm10-instrument".to_string())
            .spawn(move || actor.run(rx))?;
        let _ = self.tx.set(tx);
        Ok(())
    }

    pub fn register_interrupt(
        &self,
        category: InterruptCategory,
    ) -> tokio::sync::mpsc::Receiver<()> {
        self.interrupts.register(category)
    }

    /// C `qmesg` (`drvGM10.c:1342-1373`): queue `command` behind every
    /// earlier one; `done` runs after it. A gone actor drops `done` un-run.
    pub fn enqueue(&self, command: Command, done: Completion) {
        if let Some(tx) = self.tx.get() {
            let _ = tx.send(Job { command, done });
        }
    }

    /// Block until the actor thread has finished processing `command`. C's
    /// `command_process` discards every handler's status (`drvGM10.c:1263-1309`),
    /// so there is none to return.
    pub fn submit(&self, command: Command) {
        let (done_tx, done_rx) = mpsc::channel();
        self.enqueue(
            command,
            Box::new(move || {
                let _ = done_tx.send(());
            }),
        );
        let _ = done_rx.recv();
    }

    fn cache(&self) -> parking_lot::MutexGuard<'_, Cache> {
        self.cache.lock()
    }

    // -- test_* (`drvGM10.c:1473-1537`): pure cache reads, used at
    //    init_record time; valid only after the initial load above. --

    pub fn test_module(&self, module: usize) -> bool {
        !self.cache().modules[module].use_flag
    }

    pub fn test_signal(&self, channel: u32) -> bool {
        matches!(
            self.channel_type(channel),
            ChannelType::None | ChannelType::Unknown
        )
    }

    pub fn test_analog_signal(&self, channel: u32) -> bool {
        !matches!(
            self.channel_type(channel),
            ChannelType::InputAnalog | ChannelType::OutputAnalog
        )
    }

    pub fn test_binary_signal(&self, channel: u32) -> bool {
        !matches!(
            self.channel_type(channel),
            ChannelType::InputBinary | ChannelType::OutputBinary
        )
    }

    pub fn test_integer_signal(&self, channel: u32) -> bool {
        self.channel_type(channel) != ChannelType::InputInteger
    }

    pub fn test_output_analog_signal(&self, channel: u32) -> bool {
        self.channel_type(channel) != ChannelType::OutputAnalog
    }

    pub fn test_output_binary_signal(&self, channel: u32) -> bool {
        self.channel_type(channel) != ChannelType::OutputBinary
    }

    fn channel_type(&self, channel: u32) -> ChannelType {
        self.cache().meas_type[(channel - 1) as usize]
    }

    // -- get_* (`drvGM10.c:1762-2002`): pure cache reads. --

    /// `gm10_analog_get` (`drvGM10.c:1762-1800`).
    pub fn analog_get(&self, family: ChannelFamily, channel: u32) -> f64 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.meas_type[idx] {
                ChannelType::None | ChannelType::Unknown => 0.0,
                _ => scaled_value(cache.meas_data[idx].value, cache.meas_info[idx].scale),
            },
            ChannelFamily::Math => {
                scaled_value(cache.calc_data[idx].value, cache.calc_info[idx].scale)
            }
            ChannelFamily::Comm => {
                scaled_value(cache.comm_data[idx].value, cache.comm_info[idx].scale)
            }
            ChannelFamily::Const => f64::from(cache.constant[idx]),
            ChannelFamily::VarConst => f64::from(cache.varconstant[idx]),
        }
    }

    /// `gm10_integer_get` (`drvGM10.c:1802-1815`): intentionally unscaled —
    /// matches the source's own "hope it doesn't need to be scaled!!!".
    pub fn integer_get(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.meas_type[idx] {
            ChannelType::None | ChannelType::Unknown => 0,
            _ => cache.meas_data[idx].value,
        }
    }

    /// `gm10_binary_get` (`drvGM10.c:1817-1828`).
    pub fn binary_get(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.meas_type[idx] {
            ChannelType::None | ChannelType::Unknown => 0,
            _ => cache.meas_data[idx].value,
        }
    }

    /// `gm10_channel_get_egu` (`drvGM10.c:1831-1853`).
    pub fn channel_get_egu(&self, family: ChannelFamily, channel: u32) -> String {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.meas_type[idx] {
                ChannelType::None | ChannelType::Unknown => String::new(),
                _ => cache.meas_info[idx].unit.clone(),
            },
            ChannelFamily::Math => cache.calc_info[idx].unit.clone(),
            ChannelFamily::Comm => cache.comm_info[idx].unit.clone(),
            ChannelFamily::Const | ChannelFamily::VarConst => String::new(),
        }
    }

    /// `gm10_channel_get_expr` (`drvGM10.c:1855-1861`).
    pub fn channel_get_expr(&self, channel: u32) -> String {
        let cache = self.cache();
        let mut expr = cache.calc_expr[(channel - 1) as usize].expr.clone();
        expr.truncate(expr.floor_char_boundary(39));
        expr
    }

    /// `gm10_get_channel_status` (`drvGM10.c:1863-1885`).
    pub fn get_channel_status(&self, family: ChannelFamily, channel: u32) -> u8 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.meas_type[idx] {
                ChannelType::None | ChannelType::Unknown => 0,
                _ => cache.meas_info[idx].ch_status as u8,
            },
            ChannelFamily::Math => cache.calc_info[idx].ch_status as u8,
            ChannelFamily::Comm => cache.comm_info[idx].ch_status as u8,
            ChannelFamily::Const | ChannelFamily::VarConst => 0,
        }
    }

    /// `gm10_get_channel_mode` (`drvGM10.c:1887-1897`): signal channels
    /// only, and only for the two output types.
    pub fn get_channel_mode(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.meas_type[idx] {
            ChannelType::OutputAnalog | ChannelType::OutputBinary => cache.meas_info[idx].ch_mode,
            _ => 0,
        }
    }

    /// `gm10_get_data_status` (`drvGM10.c:1900-1922`).
    pub fn get_data_status(&self, family: ChannelFamily, channel: u32) -> DataStatus {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.meas_type[idx] {
                ChannelType::None | ChannelType::Unknown => DataStatus::Normal,
                _ => cache.meas_data[idx].data_status,
            },
            ChannelFamily::Math => cache.calc_data[idx].data_status,
            ChannelFamily::Comm => cache.comm_data[idx].data_status,
            ChannelFamily::Const | ChannelFamily::VarConst => DataStatus::Normal,
        }
    }

    /// `gm10_get_alarm_flag` (`drvGM10.c:1925-1930`).
    pub fn get_alarm_flag(&self) -> bool {
        self.cache().alarm_flag
    }

    /// `gm10_get_alarm` (`drvGM10.c:1933-1965`): `sub_channel` 0 means the
    /// aggregate `alarm_status`; 1-4 index `alarm[sub_channel-1]`.
    pub fn get_alarm(&self, family: ChannelFamily, channel: u32, sub_channel: u8) -> u8 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        let data = match family {
            ChannelFamily::Signal => match cache.meas_type[idx] {
                ChannelType::None | ChannelType::Unknown => return 0,
                _ => &cache.meas_data[idx],
            },
            ChannelFamily::Math => &cache.calc_data[idx],
            ChannelFamily::Comm => &cache.comm_data[idx],
            ChannelFamily::Const | ChannelFamily::VarConst => return 0,
        };
        if sub_channel == 0 {
            data.alarm_status
        } else {
            data.alarm[(sub_channel - 1) as usize]
        }
    }

    /// `gm10_get_error` (`drvGM10.c:1968-1977`): `channel` is 1-3.
    pub fn get_error(&self, channel: u32) -> String {
        let cache = self.cache();
        if cache.error_flag {
            cache.error.strings[(channel - 1) as usize].clone()
        } else {
            String::new()
        }
    }

    pub fn get_error_flag(&self) -> bool {
        self.cache().error_flag
    }

    /// `gm10_get_mode` (`drvGM10.c:1986-2002`).
    pub fn get_recording_mode(&self) -> bool {
        self.cache().recording_mode
    }
    pub fn get_compute_mode(&self) -> i32 {
        self.cache().compute_mode
    }
    pub fn get_settings_mode(&self) -> bool {
        self.cache().settings_mode
    }

    /// `gm10_module_info` (`drvGM10.c:1591-1616`).
    pub fn module_presence(&self, module: usize) -> bool {
        self.cache().modules[module].use_flag
    }
    pub fn module_string(&self, module: usize) -> String {
        let cache = self.cache();
        let m = &cache.modules[module];
        if m.use_flag {
            m.module_string.clone()
        } else if !m.module_string.is_empty() {
            format!("(unused) {}", m.module_string)
        } else {
            "(empty)".to_string()
        }
    }

    /// `gm10_channel_start` (`drvGM10.c:1725-1760`): the first-scan async
    /// read-refresh for a single channel, collapsed to a blocking round
    /// trip. Signal channels with no recognized type are a silent no-op.
    pub fn channel_start(&self, family: ChannelFamily, channel: u32) {
        if family == ChannelFamily::Signal
            && matches!(
                self.channel_type(channel),
                ChannelType::None | ChannelType::Unknown
            )
        {
            return;
        }
        let command = match family {
            ChannelFamily::Signal => Command::ReadSignal(channel),
            ChannelFamily::Math => Command::ReadMath(channel),
            ChannelFamily::Comm => Command::ReadComm(channel),
            ChannelFamily::Const => Command::ReadConst(channel),
            ChannelFamily::VarConst => Command::ReadVarConst(channel),
        };
        self.submit(command)
    }
}

pub struct Actor {
    stream: TcpStream,
    cache: Arc<Mutex<Cache>>,
    interrupts: Arc<Interrupts>,
    /// Its own queue, for the record-less `qmesg` of `load_status`.
    requeue: mpsc::Sender<Job>,
}

impl Actor {
    fn run(mut self, rx: mpsc::Receiver<Job>) {
        while let Ok(job) = rx.recv() {
            let _ = self.process(job.command);
            (job.done)();
        }
    }

    fn process(&mut self, command: Command) -> io::Result<()> {
        match command {
            Command::ReadAllData => self.load_data_values(None),
            Command::ReadSignal(ch) => self.load_data_values(Some((ChannelFamily::Signal, ch))),
            Command::ReadMath(ch) => self.load_data_values(Some((ChannelFamily::Math, ch))),
            Command::ReadComm(ch) => self.load_data_values(Some((ChannelFamily::Comm, ch))),
            Command::ReadAllMisc => self.load_misc_values(true, true, None),
            Command::ReadConst(ch) => self.load_misc_values(true, false, Some(ch)),
            Command::ReadVarConst(ch) => self.load_misc_values(false, true, Some(ch)),
            Command::ReadAllInfos => self.load_infos(),
            Command::ReadStatus => self.load_status(),
            Command::SetSignalOutput(ch, value) => self.set_signal_output(ch, value),
            Command::SetComm(ch, value) => self.set_output_value(&cmd_ocommch(ch, value)),
            Command::SetConst(ch, value) => self.set_output_value(&cmd_skconst_set(ch, value)),
            Command::SetVarConst(ch, value) => self.set_output_value(&cmd_swconst_set(ch, value)),
            Command::SetBinaryOutput(ch, on) => self.set_binary_value(ch, on),
            // `set_mode` sends nothing outside 0..1 / 0..3 (`drvGM10.c:1209-1224`).
            Command::SetOpMode(v @ 0..=1) => self.set_output_value(&cmd_orec_set(v == 1)),
            Command::SetCompute(v @ 0..=3) => self.set_output_value(&cmd_omath_set(v as u8)),
            Command::SetOpMode(_) | Command::SetCompute(_) => Ok(()),
            Command::ClearError => self.clear_error(),
            Command::AcknowledgeAlarms => self.acknowledge_alarms(),
        }
    }

    fn write_command(&mut self, command: &str) -> io::Result<()> {
        wire::write_command(&mut self.stream, command)
    }

    /// `response_reader` (`drvGM10.c:360-413`): reset the transient error
    /// flag on every round trip; on an error frame, run the `_ERR,<code>`
    /// follow-up and populate `Cache::error`.
    fn read_response(&mut self) -> io::Result<RawResponse> {
        let raw = wire::read_response(&mut self.stream)?;
        {
            self.cache.lock().error_flag = false;
        }
        if let RawResponse::Error(bytes) = &raw {
            self.cache.lock().error_flag = true;
            let text = String::from_utf8_lossy(bytes);
            let Some((code, parameter)) = parse_error_header(&text) else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "malformed E1 error header",
                ));
            };
            {
                let mut cache = self.cache.lock();
                cache.error.code = code;
                if let Some(parameter) = parameter {
                    cache.error.parameter = parameter;
                }
            }
            self.write_command(&cmd_err_query(code))?;
            let follow = wire::read_response(&mut self.stream)?;
            let RawResponse::Ascii(ascii_raw) = follow else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "_ERR query did not return ASCII",
                ));
            };
            let payload = String::from_utf8_lossy(wire::ascii_payload(&ascii_raw)).into_owned();
            if let Some(msg) = extract_quoted_message(&payload) {
                self.cache.lock().error.strings = split_error_message(msg);
            }
            self.interrupts.fire(InterruptCategory::Error);
        }
        Ok(raw)
    }

    fn expect_ok(&mut self, command: &str) -> io::Result<()> {
        self.write_command(command)?;
        match self.read_response()? {
            RawResponse::Ok => Ok(()),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "expected E0 OK response",
            )),
        }
    }

    fn expect_ascii(&mut self, command: &str) -> io::Result<Vec<u8>> {
        self.write_command(command)?;
        match self.read_response()? {
            RawResponse::Ascii(raw) => Ok(wire::ascii_payload(&raw).to_vec()),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "expected ASCII response",
            )),
        }
    }

    fn expect_binary(&mut self, command: &str) -> io::Result<Vec<u8>> {
        self.write_command(command)?;
        match self.read_response()? {
            RawResponse::Binary(raw) => Ok(raw),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "expected binary response",
            )),
        }
    }

    /// `load_modules` (`drvGM10.c:416-619`): must run before every other
    /// load function, which all index off `Cache::modules[..].use_flag`
    /// and `Cache::meas_type`.
    fn load_modules(&mut self) -> io::Result<()> {
        let payload = self.expect_ascii(&cmd_fsysconf())?;
        let lines = parse_fsysconf(&payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FSysConf body"))?;

        let mut guard = self.cache.lock();
        let cache = &mut *guard;
        for t in cache.meas_type.iter_mut() {
            *t = ChannelType::None;
        }
        for m in cache.modules.iter_mut() {
            m.use_flag = false;
            m.module_string.clear();
        }
        for line in &lines {
            // C indexes every wire-supplied slot unchecked (UB out of
            // range); out-of-range slots are skipped here.
            let Some(module) = cache.modules.get_mut(line.index) else {
                continue;
            };
            if !module_line_ok(line) {
                continue;
            }
            module.module_string = line.set_message.clone();
            let Some((mod_type, channel_number)) = classify_module(&line.set_message) else {
                continue;
            };
            if mod_type == ModuleType::Pid {
                continue;
            }
            module.mod_type = mod_type;
            module.channel_number = channel_number;
            module.use_flag = true;

            let base = line.index * 100;
            let [inputs, outputs] = channel_number.map(|n| n.max(0) as usize);
            let mut set = |start: usize, count: usize, t: ChannelType| {
                for slot in cache.meas_type.iter_mut().skip(start).take(count) {
                    *slot = t;
                }
            };
            match mod_type {
                ModuleType::InputAnalog => set(base, inputs, ChannelType::InputAnalog),
                ModuleType::InputDigital => set(base, inputs, ChannelType::InputBinary),
                ModuleType::InputPulse => set(base, inputs, ChannelType::InputInteger),
                ModuleType::OutputAnalog => set(base, outputs, ChannelType::OutputAnalog),
                ModuleType::OutputDigital => set(base, outputs, ChannelType::OutputBinary),
                ModuleType::InputOutputDigital => {
                    set(base, inputs, ChannelType::InputBinary);
                    set(base + inputs, outputs, ChannelType::OutputBinary);
                }
                ModuleType::Pid | ModuleType::Unknown => {}
            }
        }
        Ok(())
    }

    /// `load_status` (`drvGM10.c:625-656`).
    fn load_status(&mut self) -> io::Result<()> {
        // A failed sscanf leaves the previous value (`drvGM10.c:632,638`).
        let orec_payload = self.expect_ascii(&cmd_orec_query())?;
        if let Some(v) = parse_orec(&String::from_utf8_lossy(&orec_payload)) {
            self.cache.lock().recording_mode = v != 0;
        }
        let omath_payload = self.expect_ascii(&cmd_omath_query())?;
        if let Some(v) = parse_omath(&String::from_utf8_lossy(&omath_payload)) {
            self.cache.lock().compute_mode = v;
        }

        let was_settings_mode;
        {
            let mut cache = self.cache.lock();
            was_settings_mode = cache.settings_mode;
            cache.settings_mode = cache.recording_mode && (cache.compute_mode != 0);
        }
        self.interrupts.fire(InterruptCategory::Status);

        if was_settings_mode && !self.cache.lock().settings_mode {
            // `qmesg(dq, NULL, CMD_READ_ALL_INFOS, ..)` (`drvGM10.c:646`).
            let _ = self.requeue.send(Job {
                command: Command::ReadAllInfos,
                done: Box::new(|| {}),
            });
        }
        Ok(())
    }

    /// `load_infos` (`drvGM10.c:661-909`): FChInfo, then SRangeAO,
    /// SRangeDO, SRangeMath — all four share one `info_ioscanpvt` fire at
    /// the end.
    fn load_infos(&mut self) -> io::Result<()> {
        let fchinfo_payload = self.expect_ascii(&cmd_fchinfo())?;
        let info_lines = parse_fchinfo(&fchinfo_payload);
        {
            let mut guard = self.cache.lock();
            let cache = &mut *guard;
            for line in &info_lines {
                if line.family == InfoFamily::Signal
                    && !cache
                        .modules
                        .get(line.index / 100)
                        .is_some_and(|m| m.use_flag)
                {
                    continue;
                }
                let ch_status = match line.status {
                    CH_STATUS_NORMAL => ChStatus::Normal,
                    CH_STATUS_DIFF => ChStatus::Diff,
                    CH_STATUS_SKIP => ChStatus::Skip,
                    _ => ChStatus::Unknown,
                };
                // `drvGM10.c:733-742`: on SKIP, `cd->data_status = VL_SKIP`
                // is set for whichever family `ci`/`cd` point at — not
                // Signal-only ("since the input won't do this").
                let (infos, datas) = match line.family {
                    InfoFamily::Signal => (&mut cache.meas_info, &mut cache.meas_data),
                    InfoFamily::Math => (&mut cache.calc_info, &mut cache.calc_data),
                    InfoFamily::Comm => (&mut cache.comm_info, &mut cache.comm_data),
                };
                let (Some(ci), Some(cd)) = (infos.get_mut(line.index), datas.get_mut(line.index))
                else {
                    continue;
                };
                ci.ch_status = ch_status;
                ci.unit = line.unit.clone();
                ci.scale = line.scale;
                if line.status == CH_STATUS_SKIP {
                    cd.data_status = DataStatus::Skip;
                }
            }
        }

        for (query, parse) in [
            (cmd_srangeao_query(), parse_srangeao as fn(&[u8]) -> _),
            (cmd_srangedo_query(), parse_srangedo),
        ] {
            let payload = self.expect_ascii(&query)?;
            let parsed = parse(&payload);
            let mut cache = self.cache.lock();
            for line in parsed.as_ref().unwrap_or_else(|lines| lines) {
                if !cache
                    .modules
                    .get(line.index / 100)
                    .is_some_and(|m| m.use_flag)
                {
                    continue;
                }
                if let Some(ci) = cache.meas_info.get_mut(line.index) {
                    ci.ch_mode = line.mode;
                }
            }
            if parsed.is_err() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "SRange line is not a signal channel",
                ));
            }
        }

        let srangemath_payload = self.expect_ascii(&cmd_srangemath_query())?;
        let math_lines = parse_srangemath(&srangemath_payload);
        {
            let mut cache = self.cache.lock();
            for line in &math_lines {
                if let Some(slot) = cache.calc_expr.get_mut(line.index) {
                    *slot = ExprInfo {
                        on_flag: line.on_flag,
                        expr: line.expr.clone(),
                    };
                }
            }
        }

        self.interrupts.fire(InterruptCategory::Info);
        Ok(())
    }

    /// `load_data_values` (`drvGM10.c:938-1078`). `scope`: `None` = all
    /// channels (`FData,1`), `Some((family, channel))` = one channel.
    fn load_data_values(&mut self, scope: Option<(ChannelFamily, u32)>) -> io::Result<()> {
        let command = match scope {
            None => cmd_fdata_all(),
            Some((ChannelFamily::Signal, ch)) => cmd_fdata_signal(ch),
            Some((ChannelFamily::Math, ch)) => cmd_fdata_math(ch),
            Some((ChannelFamily::Comm, ch)) => cmd_fdata_comm(ch),
            Some((ChannelFamily::Const | ChannelFamily::VarConst, _)) => {
                // Unreachable: `channel_start` never routes Const/VarConst
                // here (they go through `load_misc_values`).
                return Ok(());
            }
        };
        let raw = self.expect_binary(&command)?;
        let records = parse_fdata_binary(&raw)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FData frame"))?;

        let mut alarm_flag = false;
        let alarm_flag_changed;
        {
            let mut cache = self.cache.lock();
            for record in &records {
                // `drvGM10.c:995`: applied uniformly regardless of
                // channel_type, exactly as the source does.
                if record.data_type != 1 {
                    continue;
                }
                let address = record.address as usize;
                if !cache.modules.get(address / 100).is_some_and(|m| m.use_flag) {
                    continue;
                }
                // C indexes `[address - 1]` unchecked; out of range is UB there.
                let Some(idx) = address.checked_sub(1) else {
                    continue;
                };
                let cd = match record.channel_type {
                    1 => cache.meas_data.get_mut(idx),
                    2 => cache.calc_data.get_mut(idx),
                    3 => cache.comm_data.get_mut(idx),
                    _ => None,
                };
                let Some(cd) = cd else {
                    continue;
                };
                cd.data_status = DataStatus::from_wire(record.status);
                cd.value = if cd.data_status == DataStatus::Normal {
                    record.value
                } else {
                    0
                };
                cd.alarm = [
                    record.alarms[0] & 0x3F,
                    record.alarms[1] & 0x3F,
                    record.alarms[2] & 0x3F,
                    record.alarms[3] & 0x3F,
                ];
                cd.alarm_status = ((cd.alarm[0] != 0) as u8)
                    | (((cd.alarm[1] != 0) as u8) << 1)
                    | (((cd.alarm[2] != 0) as u8) << 2)
                    | (((cd.alarm[3] != 0) as u8) << 3);
                if cd.alarm_status != 0 {
                    alarm_flag = true;
                }
            }
            alarm_flag_changed = cache.alarm_flag != alarm_flag;
            cache.alarm_flag = alarm_flag;
        }
        // `drvGM10.c:1071-1075`: a single-channel read wakes I/O Intr only
        // when the alarm summary flips.
        if scope.is_none() || alarm_flag_changed {
            self.interrupts.fire(InterruptCategory::Channel);
        }
        Ok(())
    }

    /// `load_misc_values` (`drvGM10.c:1086-1152`).
    fn load_misc_values(
        &mut self,
        want_const: bool,
        want_varconst: bool,
        single_channel: Option<u32>,
    ) -> io::Result<()> {
        if want_const {
            let command = match single_channel {
                Some(ch) => cmd_skconst_query(ch),
                None => cmd_skconst_query_all(),
            };
            let payload = self.expect_ascii(&command)?;
            let lines = parse_const_lines(&payload);
            let mut cache = self.cache.lock();
            for line in &lines {
                if let Some(slot) = cache.constant.get_mut(line.index) {
                    *slot = line.value as f32;
                }
            }
        }
        if want_varconst {
            let command = match single_channel {
                Some(ch) => cmd_swconst_query(ch),
                None => cmd_swconst_query_all(),
            };
            let payload = self.expect_ascii(&command)?;
            let lines = parse_const_lines(&payload);
            let mut cache = self.cache.lock();
            for line in &lines {
                if let Some(slot) = cache.varconstant.get_mut(line.index) {
                    *slot = line.value as f32;
                }
            }
        }
        // `drvGM10.c:1147-1148`: only the all-channel read wakes I/O Intr.
        if single_channel.is_none() {
            self.interrupts.fire(InterruptCategory::Misc);
        }
        Ok(())
    }

    /// `set_output_value` (`drvGM10.c:1158-1186`), reached only for
    /// `Comm`/`Const`/`VarConst`; `Signal` goes through
    /// [`Self::set_signal_output`] since it needs the type gate first.
    fn set_output_value(&mut self, command: &str) -> io::Result<()> {
        self.expect_ok(command)
    }

    /// `gm10_analog_set`'s `ADDR_SIGNAL` arm (`drvGM10.c:1639-1643`): a
    /// silent no-op unless the channel is genuinely `OutputAnalog`.
    fn set_signal_output(&mut self, channel: u32, value: f64) -> io::Result<()> {
        let is_output_analog =
            self.cache.lock().meas_type[(channel - 1) as usize] == ChannelType::OutputAnalog;
        if !is_output_analog {
            return Ok(());
        }
        self.expect_ok(&cmd_ocmdao(channel, value))
    }

    /// `set_binary_value` (`drvGM10.c:1189-1203`); the type gate is
    /// `gm10_binary_set`'s (`drvGM10.c:1658-1665`).
    fn set_binary_value(&mut self, channel: u32, on: bool) -> io::Result<()> {
        let is_output_binary =
            self.cache.lock().meas_type[(channel - 1) as usize] == ChannelType::OutputBinary;
        if !is_output_binary {
            return Ok(());
        }
        self.expect_ok(&cmd_ocmdrelay(channel, on))
    }

    /// `clear_error` (`drvGM10.c:1233-1248`).
    fn clear_error(&mut self) -> io::Result<()> {
        self.expect_ok(&cmd_oerrorclear())?;
        let mut cache = self.cache.lock();
        cache.error_flag = false;
        cache.error = ErrorState::default();
        cache.error.code = -1;
        drop(cache);
        self.interrupts.fire(InterruptCategory::Error);
        Ok(())
    }

    /// `acknowledge_alarms` (`drvGM10.c:1250-1260`): also forces an
    /// immediate all-channel data refresh "in case scan is slow".
    fn acknowledge_alarms(&mut self) -> io::Result<()> {
        self.expect_ok(&cmd_oalarmack())?;
        self.load_data_values(None)
    }
}

/// `init_gm10` (`drvGM10.c:1376-1465`), the `gm10Init` command. Every
/// failure is silent, as in C. The handle is registered before the connect,
/// so a unit that fails to connect or load still resolves for records.
pub fn gm10_init(registry: &Registry, device: Option<&[u8]>, address: Option<&[u8]>) {
    init_on_port(registry, device, address, PORT);
}

fn init_on_port(registry: &Registry, device: Option<&[u8]>, address: Option<&[u8]>, port: u16) {
    let Some(device) = device.filter(|d| valid_device_name(d)) else {
        return;
    };
    // Validated ASCII.
    let name = String::from_utf8_lossy(device).into_owned();
    if registry.get(&name).is_some() {
        return;
    }
    // C frees `dq` here but leaves it listed (a use-after-free); not
    // registering is the safe reading.
    let Some(ip) = address.and_then(inet_aton) else {
        return;
    };
    let instrument = Instrument::new(ip.to_string());
    let _ = registry.insert(name, instrument.clone());
    if let Ok(stream) = TcpStream::connect(SocketAddrV4::new(ip, port)) {
        let _ = instrument.start(stream);
    }
}

/// A letter, then letters, digits or underscores (`drvGM10.c:1389-1400`).
fn valid_device_name(name: &[u8]) -> bool {
    match name.split_first() {
        Some((first, rest)) => {
            first.is_ascii_alphabetic()
                && rest.iter().all(|&b| b.is_ascii_alphanumeric() || b == b'_')
        }
        None => false,
    }
}

/// glibc `inet_aton`: one to four `strtoul(.., 0)` parts, the last filling
/// the remaining bytes, optionally followed by whitespace and anything.
fn inet_aton(cp: &[u8]) -> Option<Ipv4Addr> {
    const MAX: [u64; 4] = [0xffff_ffff, 0xff_ffff, 0xffff, 0xff];
    let mut parts: Vec<u64> = Vec::with_capacity(3);
    let mut i = 0;
    loop {
        if !cp.get(i).is_some_and(u8::is_ascii_digit) {
            return None;
        }
        let (radix, start) = match (cp[i], cp.get(i + 1)) {
            (b'0', Some(b'x' | b'X')) if cp.get(i + 2).is_some_and(u8::is_ascii_hexdigit) => {
                (16, i + 2)
            }
            (b'0', _) => (8, i),
            _ => (10, i),
        };
        let end = start
            + cp[start..]
                .iter()
                .take_while(|b| (**b as char).is_digit(radix))
                .count();
        let mut val: u64 = 0;
        for &b in &cp[start..end] {
            val = val * radix as u64 + (b as char).to_digit(radix)? as u64;
            if val > 0xffff_ffff {
                return None;
            }
        }
        i = end;
        if cp.get(i) == Some(&b'.') {
            if parts.len() > 2 || val > 0xff {
                return None;
            }
            parts.push(val);
            i += 1;
        } else {
            if let Some(&c) = cp.get(i)
                && !matches!(c, 0 | b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
            {
                return None;
            }
            if val > MAX[parts.len()] {
                return None;
            }
            let mut word = val as u32;
            for (k, p) in parts.iter().enumerate() {
                word |= (*p as u32) << (24 - 8 * k);
            }
            return Some(Ipv4Addr::from(word));
        }
    }
}

/// `gm10_connect` (`drvGM10.c:1539-1552`): `gm10_queue_list`, the handles
/// `gm10Init` registered, newest first as C prepends them.
#[derive(Default)]
pub struct Registry {
    instruments: Mutex<Vec<(String, Arc<Instrument>)>>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert(&self, name: String, instrument: Arc<Instrument>) -> Result<(), &'static str> {
        let mut instruments = self.instruments.lock();
        if instruments.iter().any(|(n, _)| *n == name) {
            return Err("device already exists");
        }
        instruments.insert(0, (name, instrument));
        Ok(())
    }

    pub fn get(&self, name: &str) -> Option<Arc<Instrument>> {
        self.instruments
            .lock()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, i)| i.clone())
    }
}

/// `drvGM10.report`, `gm10_report` (`drvGM10.c:265-311`).
impl DriverSupport for Registry {
    fn report(&self, level: i32) -> Option<Vec<u8>> {
        let mut out = String::new();
        for (cnt, (name, instrument)) in self.instruments.lock().iter().enumerate() {
            out += &format!(
                "    Yokogawa GM10 #{}: {name} @ {}\n",
                cnt + 1,
                instrument.peer_address
            );
            if level != 0 {
                out += "        Modules:\n";
                let cache = instrument.cache();
                for (i, m) in cache.modules.iter().take(6).enumerate() {
                    if m.use_flag {
                        out += &format!("        #{}: {}\n", i + 1, m.module_string);
                    } else {
                        out += &format!("        #{}: empty\n", i + 1);
                    }
                }
            }
        }
        Some(out.into_bytes())
    }
}

/// Fake-device fixtures shared with `device_support`'s tests: building a
/// `GmDevice` for tests requires a live `Arc<Instrument>`, which requires
/// running the full `Instrument::start` init sequence against a fake device.
#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use std::io::{Read as _, Write as _};
    use std::net::TcpListener;

    #[test]
    fn out_of_range_wire_indexes_are_skipped() {
        let mut fsysconf = b"Unit:00  ".to_vec();
        for (slot, module) in [("12", "GX90XA-06"), ("09", "GX90XA-100")] {
            fsysconf.extend_from_slice(format!("{slot}:{module:<17}{module:<17}").as_bytes());
            fsysconf.extend_from_slice(b"----------------\r\n");
        }
        fsysconf.push(b'E');
        let mut responses = default_init_responses();
        responses[0] = ascii_frame(&fsysconf);
        responses[3] = ascii_frame(b"N 01000 X ,1\nN 09999 X ,1\nN A9999 X ,1\nE");
        responses[4] = ascii_frame(b"SRangeAO,09999,X\nE");
        responses[6] = ascii_frame(b"SRangeMath,9999,f\nE");
        responses[8] = ascii_frame(b"aaaaaaaa999,1.5\nE");
        responses[9] = ascii_frame(b"aaaaaaaa999,1.5\nE");
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        device.join().unwrap();
        let cache = instrument.cache();
        // C sizes meas_* at MAX_MEAS_CHAN = 1000, so address 1000 is stored.
        assert_eq!(cache.meas_type[999], ChannelType::InputAnalog);
        assert_eq!(cache.meas_info[999].scale, 1);
    }

    #[test]
    fn constants_are_stored_as_c_float() {
        let mut responses = default_init_responses();
        responses[8] = ascii_frame(b"aaaaaaaa001,0.1\nE");
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        device.join().unwrap();
        assert_eq!(
            instrument.analog_get(ChannelFamily::Const, 1),
            f64::from(0.1f32)
        );
    }

    #[test]
    fn channel_get_expr_cuts_on_char_boundary() {
        let instrument = Instrument::new("127.0.0.1".into());
        instrument.cache().calc_expr[0].expr = format!("{}\u{b0}C", "x".repeat(38));
        assert_eq!(instrument.channel_get_expr(1), "x".repeat(38));
    }

    pub(crate) fn ascii_frame(body: &[u8]) -> Vec<u8> {
        let mut out = b"EAxx".to_vec();
        out.extend_from_slice(body);
        out.extend_from_slice(b"EN\r\n");
        out
    }

    /// An `FData` frame of `(channel_type, address, status, alarms, value)`
    /// records, all `data_type` 1.
    pub(crate) fn fdata_binary(records: &[(u8, u16, u8, [u8; 4], i32)]) -> Vec<u8> {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&0u32.to_be_bytes());
        raw.extend_from_slice(&[0u8; 10]);
        raw.extend_from_slice(&(16 + 12 * records.len() as u16).to_be_bytes());
        raw.extend_from_slice(&[0u8; 16]);
        for &(channel_type, address, status, alarms, value) in records {
            raw.push((1 << 4) | channel_type);
            raw.push(status);
            raw.extend_from_slice(&address.to_be_bytes());
            raw.extend_from_slice(&alarms);
            raw.extend_from_slice(&value.to_be_bytes());
        }
        let total = (raw.len() - 8) as u32;
        raw[4..8].copy_from_slice(&total.to_be_bytes());
        raw
    }

    /// One `FData` record: meas channel 1 (address=1), normal status,
    /// value 1234, no alarms (byte layout matches
    /// `codec::tests::fdata_binary_decodes_one_record`).
    pub(crate) fn one_record_fdata_binary() -> Vec<u8> {
        fdata_binary(&[(1, 1, 0, [0; 4], 1234)])
    }

    /// One present module (index 0), `GX90XA-06` = `InputAnalog`, 6
    /// channels (byte layout matches `codec::tests::fsysconf_parses_one_present_module`).
    pub(crate) fn fsysconf_one_input_analog_module() -> Vec<u8> {
        let mut body = b"Unit:00".to_vec();
        body.extend_from_slice(b"  ");
        body.extend_from_slice(b"00:");
        body.extend_from_slice(format!("{:<17}", "GX90XA-06").as_bytes());
        body.extend_from_slice(format!("{:<17}", "GX90XA-06").as_bytes());
        body.extend_from_slice(b"----------------\r\n");
        body.push(b'E');
        ascii_frame(&body)
    }

    /// Overwrites signal channel `channel`'s cached type and value.
    pub(crate) fn set_signal(instrument: &Instrument, channel: u32, ty: ChannelType, value: i32) {
        let mut cache = instrument.cache();
        let idx = (channel - 1) as usize;
        cache.meas_type[idx] = ty;
        cache.meas_data[idx].value = value;
    }

    /// Plays the device side of one connection: an unsolicited `E0`
    /// greeting (matching `init_gm10`'s post-connect read), then one
    /// canned reply per entry in `responses`, in order. Returns every
    /// `\r\n`-terminated command line it received, for asserting exact
    /// wire traffic.
    pub(crate) fn spawn_fake_device(
        listener: TcpListener,
        responses: Vec<Vec<u8>>,
    ) -> std::thread::JoinHandle<Vec<String>> {
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream.write_all(b"E0\r\n").unwrap();
            let mut received = Vec::new();
            for response in responses {
                let mut line = Vec::new();
                let mut byte = [0u8; 1];
                loop {
                    if stream.read(&mut byte).unwrap() == 0 {
                        return received;
                    }
                    line.push(byte[0]);
                    if line.ends_with(b"\r\n") {
                        break;
                    }
                }
                received.push(String::from_utf8(line).unwrap());
                stream.write_all(&response).unwrap();
            }
            received
        })
    }

    /// The replies `connect_default_fixture`'s init sequence consumes.
    pub(crate) fn default_init_responses() -> Vec<Vec<u8>> {
        vec![
            fsysconf_one_input_analog_module(),  // FSysConf
            ascii_frame(b"ORec,0\r\n"),          // ORec?
            ascii_frame(b"OMath,0\r\n"),         // OMath?
            ascii_frame(b"N 0001 DEGC ,3\nE"),   // FChInfo
            ascii_frame(b"E"),                   // SRangeAO?
            ascii_frame(b"E"),                   // SRangeDO?
            ascii_frame(b"E"),                   // SRangeMath?
            one_record_fdata_binary(),           // FData,1
            ascii_frame(b"aaaaaaaa001,12.5\nE"), // SKConst?
            ascii_frame(b"aaaaaaaa001,-7.5\nE"), // SWConst?
        ]
    }

    /// One connected instrument: module 0 is a present `InputAnalog`
    /// module, channel 1 is `InputAnalog` (unit "DEGC", value 1.234),
    /// `K1`/`W1` (Const/VarConst) are set to 12.5/-7.5. Every other
    /// Signal channel (e.g. 2) is `ChannelType::None` — "channel does not
    /// exist" gate tests use it directly.
    pub(crate) fn connect_default_fixture() -> Arc<Instrument> {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let device = spawn_fake_device(listener, default_init_responses());
        let instrument = Instrument::connect_to(addr).unwrap();
        device.join().unwrap();
        instrument
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::*;
    use super::*;
    use std::net::TcpListener;

    #[test]
    fn connect_runs_full_init_sequence_then_error_round_trip() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let responses = vec![
            fsysconf_one_input_analog_module(),          // FSysConf
            ascii_frame(b"ORec,0\r\n"),                  // ORec?
            ascii_frame(b"OMath,0\r\n"),                 // OMath?
            ascii_frame(b"N 0001 DEGC ,3\nE"),           // FChInfo
            ascii_frame(b"E"),                           // SRangeAO?
            ascii_frame(b"E"),                           // SRangeDO?
            ascii_frame(b"E"),                           // SRangeMath?
            one_record_fdata_binary(),                   // FData,1
            ascii_frame(b"aaaaaaaa001,12.5\nE"),         // SKConst?
            ascii_frame(b"aaaaaaaa001,-7.5\nE"),         // SWConst?
            b"E1,205:1:12\r\n".to_vec(),                 // OErrorClear,0 -> device error
            ascii_frame(b"junk 'Something broke' junk"), // _ERR,205 follow-up
        ];
        let device = spawn_fake_device(listener, responses);

        let instrument = Instrument::connect_to(addr).unwrap();

        assert!(instrument.module_presence(0));
        assert_eq!(instrument.channel_get_egu(ChannelFamily::Signal, 1), "DEGC");
        assert_eq!(instrument.analog_get(ChannelFamily::Signal, 1), 1.234);
        assert_eq!(instrument.analog_get(ChannelFamily::Const, 1), 12.5);
        assert_eq!(instrument.analog_get(ChannelFamily::VarConst, 1), -7.5);
        assert!(!instrument.get_error_flag());

        let mut error_rx = instrument.register_interrupt(InterruptCategory::Error);
        instrument.submit(Command::ClearError);
        assert!(instrument.get_error_flag());
        assert_eq!(instrument.get_error(1), "Something broke");
        assert!(error_rx.try_recv().is_ok());

        let received = device.join().unwrap();
        assert_eq!(
            received,
            vec![
                "FSysConf\r\n",
                "ORec?\r\n",
                "OMath?\r\n",
                "FChInfo\r\n",
                "SRangeAO?\r\n",
                "SRangeDO?\r\n",
                "SRangeMath?\r\n",
                "FData,1\r\n",
                "SKConst?\r\n",
                "SWConst?\r\n",
                "OErrorClear,0\r\n",
                "_ERR,205\r\n",
            ]
        );
    }

    #[test]
    fn enqueue_runs_completion_after_the_command() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(b"E0\r\n".to_vec());
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        let (tx, rx) = mpsc::channel();
        let cache = instrument.cache.clone();
        instrument.enqueue(
            Command::ClearError,
            Box::new(move || {
                let _ = tx.send(cache.lock().error.code);
            }),
        );
        assert_eq!(rx.recv().unwrap(), -1);
        let received = device.join().unwrap();
        assert_eq!(received.last().unwrap(), "OErrorClear,0\r\n");
    }

    #[test]
    fn set_mode_out_of_range_sends_nothing() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(b"E0\r\n".to_vec());
        responses.push(b"E0\r\n".to_vec());
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        instrument.submit(Command::SetOpMode(2));
        instrument.submit(Command::SetCompute(4));
        instrument.submit(Command::SetCompute(3));
        instrument.submit(Command::SetOpMode(1));
        let received = device.join().unwrap();
        assert_eq!(
            received[received.len() - 2..],
            ["OMath,3\r\n", "ORec,1\r\n"]
        );
    }

    #[test]
    fn fdata_module_gate_is_address_over_100() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(fdata_binary(&[
            (1, 99, 0, [0; 4], 77),
            (1, 100, 0, [0; 4], 55),
            (1, 0, 0, [0; 4], 33),
        ]));
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        instrument.submit(Command::ReadAllData);
        device.join().unwrap();
        let cache = instrument.cache();
        assert_eq!(cache.meas_data[98].value, 77);
        assert_eq!(cache.meas_data[99].value, 0);
    }

    #[test]
    fn inet_aton_matches_glibc() {
        let ok = |s: &str| inet_aton(s.as_bytes()).map(|a| a.to_string());
        assert_eq!(ok("192.168.2.10").as_deref(), Some("192.168.2.10"));
        assert_eq!(ok("127.1").as_deref(), Some("127.0.0.1"));
        assert_eq!(ok("10.1.258").as_deref(), Some("10.1.1.2"));
        assert_eq!(ok("0x7f.0.0.01").as_deref(), Some("127.0.0.1"));
        assert_eq!(ok("3232236042").as_deref(), Some("192.168.2.10"));
        assert_eq!(ok("1.2.3.4 trailing").as_deref(), Some("1.2.3.4"));
        for bad in [
            "",
            " 1.2.3.4",
            "1.2.3.256",
            "1.2.3.4.5",
            "08.1.1.1",
            "1.2.3.4x",
            "localhost",
            "4294967296",
            "1..2",
            "1.2.3.",
        ] {
            assert_eq!(ok(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn gm10_init_rejects_bad_names_duplicates_and_addresses_silently() {
        let registry = Registry::new();
        for name in ["", "1dev", "dev-1", "dev 1"] {
            init_on_port(&registry, Some(name.as_bytes()), Some(b"127.0.0.1"), 1);
            assert!(registry.get(name).is_none(), "{name:?}");
        }
        init_on_port(&registry, Some(b"dev"), Some(b"localhost"), 1);
        assert!(registry.get("dev").is_none());
        init_on_port(&registry, Some(b"dev"), None, 1);
        assert!(registry.get("dev").is_none());
    }

    #[test]
    fn gm10_init_registers_before_a_failed_connect() {
        let port = {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            listener.local_addr().unwrap().port()
        };
        let registry = Registry::new();
        init_on_port(&registry, Some(b"gm_1"), Some(b"127.1"), port);
        let instrument = registry
            .get("gm_1")
            .expect("registered despite the refused connect");
        assert_eq!(instrument.peer_address, "127.0.0.1");
        assert!(instrument.tx.get().is_none());
        init_on_port(&registry, Some(b"gm_1"), Some(b"10.0.0.1"), port);
        assert_eq!(registry.get("gm_1").unwrap().peer_address, "127.0.0.1");
        // No queue thread: the command is dropped and submit returns.
        instrument.submit(Command::ReadAllData);
    }

    #[test]
    fn driver_report_lists_newest_first_with_modules_at_level_1() {
        let registry = Registry::new();
        registry
            .insert("first".into(), Instrument::new("10.0.0.1".into()))
            .unwrap();
        registry
            .insert("second".into(), connect_default_fixture())
            .unwrap();
        assert_eq!(
            registry.report(0).unwrap(),
            b"    Yokogawa GM10 #1: second @ 127.0.0.1\n    Yokogawa GM10 #2: first @ 10.0.0.1\n"
        );
        let level1 = String::from_utf8(registry.report(1).unwrap()).unwrap();
        assert!(level1.starts_with(
            "    Yokogawa GM10 #1: second @ 127.0.0.1\n        Modules:\n        #1: GX90XA-06\n        #2: empty\n"
        ));
        assert_eq!(level1.matches("        #6: empty\n").count(), 2);
        assert!(!level1.contains("#7:"));
    }

    #[test]
    fn single_channel_reads_fire_io_intr_only_on_alarm_flag_change() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(one_record_fdata_binary());
        responses.push(fdata_binary(&[(1, 1, 0, [1, 0, 0, 0], 5)]));
        responses.push(ascii_frame(b"aaaaaaaa001,1.5\nE"));
        responses.push(ascii_frame(b"aaaaaaaa001,1.5\nE"));
        responses.push(ascii_frame(b"aaaaaaaa001,1.5\nE"));
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        let mut channel = instrument.register_interrupt(InterruptCategory::Channel);
        let mut misc = instrument.register_interrupt(InterruptCategory::Misc);
        instrument.submit(Command::ReadSignal(1));
        assert!(channel.try_recv().is_err());
        instrument.submit(Command::ReadSignal(1));
        assert!(channel.try_recv().is_ok());
        instrument.submit(Command::ReadConst(1));
        assert!(misc.try_recv().is_err());
        instrument.submit(Command::ReadAllMisc);
        assert!(misc.try_recv().is_ok());
        device.join().unwrap();
    }

    #[test]
    fn malformed_error_header_still_sets_error_flag() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(b"E1,junk\r\n".to_vec());
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        instrument.submit(Command::ClearError);
        assert!(instrument.get_error_flag());
        device.join().unwrap();
    }

    #[test]
    fn srange_abort_keeps_earlier_lines_and_skips_the_info_scan() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(ascii_frame(b"E"));
        responses.push(ascii_frame(b"xxxxxxxxx00001T\nxxxxxxxxxA0002M\nE"));
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        let mut info = instrument.register_interrupt(InterruptCategory::Info);
        instrument.submit(Command::ReadAllInfos);
        assert_eq!(instrument.cache().meas_info[0].ch_mode, CH_MODE_DAC_RETRANS);
        assert!(info.try_recv().is_err());
        let received = device.join().unwrap();
        assert_eq!(received.last().unwrap(), "SRangeAO?\r\n");
    }

    #[test]
    fn load_status_keeps_the_previous_mode_on_a_malformed_reply() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(ascii_frame(b"ORec,1\r\n"));
        responses.push(ascii_frame(b"OMath,2\r\n"));
        responses.push(ascii_frame(b"junk\r\n"));
        responses.push(ascii_frame(b"OMath,x\r\n"));
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        instrument.submit(Command::ReadStatus);
        instrument.submit(Command::ReadStatus);
        assert!(instrument.get_recording_mode());
        assert_eq!(instrument.get_compute_mode(), 2);
        assert!(instrument.get_settings_mode());
        device.join().unwrap();
    }

    #[test]
    fn leaving_settings_mode_queues_the_info_reload_behind_pending_commands() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = default_init_responses();
        responses.push(ascii_frame(b"ORec,1\r\n"));
        responses.push(ascii_frame(b"OMath,1\r\n"));
        responses.push(ascii_frame(b"ORec,0\r\n"));
        responses.push(ascii_frame(b"OMath,0\r\n"));
        responses.push(b"E0\r\n".to_vec());
        for _ in 0..4 {
            responses.push(ascii_frame(b"E"));
        }
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        instrument.submit(Command::ReadStatus);
        // Both queued before the actor can finish the ReadStatus round trips.
        instrument.enqueue(Command::ReadStatus, Box::new(|| {}));
        instrument.enqueue(Command::ClearError, Box::new(|| {}));
        let received = device.join().unwrap();
        assert_eq!(
            received[received.len() - 7..],
            [
                "ORec?\r\n",
                "OMath?\r\n",
                "OErrorClear,0\r\n",
                "FChInfo\r\n",
                "SRangeAO?\r\n",
                "SRangeDO?\r\n",
                "SRangeMath?\r\n",
            ]
        );
    }
}
