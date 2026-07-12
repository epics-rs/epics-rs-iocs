//! The GM10 instrument actor: owns the blocking `TcpStream` and `Cache`,
//! serializing every wire operation through one OS thread (mirrors C's
//! single-consumer `queue_func`/`qmesg` pair, `drvGM10.c:1311-1375` —
//! `qmesg` enqueues from any EPICS scan thread, `queue_func` is the sole
//! socket owner). `DeviceSupport::read`/`write` collapse C's PACT-then-
//! completed-by-`queue_func` two-phase dance into one blocking round trip
//! on [`Instrument::submit`] — legitimate since nothing else here depends
//! on interleaving with other records' scans mid-flight.

use crate::cache::*;
use crate::codec::*;
use crate::link::ChannelFamily;
use crate::wire::{self, RawResponse};
use parking_lot::Mutex;
use std::collections::HashMap;
use std::io;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::mpsc;

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
    SetOpMode(bool),
    SetCompute(u8),
    ClearError,
    AcknowledgeAlarms,
}

struct Job {
    command: Command,
    done: mpsc::Sender<io::Result<()>>,
}

/// Handle to a connected GM10 unit. Cheap to clone (`Arc`-backed); one
/// instance per `gm10Init` handle, looked up by name from [`connect`]'s
/// registry.
pub struct Instrument {
    cache: Arc<Mutex<Cache>>,
    interrupts: Arc<Interrupts>,
    tx: mpsc::Sender<Job>,
    /// `gm10_system_info(which=0, ...)` (`drvGM10.c:1620-1631`): the
    /// address this handle connected to, formatted once at connect time.
    pub peer_address: String,
}

impl Instrument {
    /// `init_gm10` (`drvGM10.c:1378-1465`): connect, read the unsolicited
    /// post-connect `E0` greeting, then run the fixed load sequence —
    /// modules must load first, everything else indexes off it.
    pub fn connect(address: &str) -> io::Result<Arc<Self>> {
        let stream = TcpStream::connect((address, PORT))?;
        Self::connect_stream(stream)
    }

    #[cfg(test)]
    pub(crate) fn connect_to(addr: std::net::SocketAddr) -> io::Result<Arc<Self>> {
        Self::connect_stream(TcpStream::connect(addr)?)
    }

    fn connect_stream(stream: TcpStream) -> io::Result<Arc<Self>> {
        let peer_address = stream.peer_addr()?.ip().to_string();
        let cache = Arc::new(Mutex::new(Cache::default()));
        let interrupts = Arc::new(Interrupts::default());
        let mut actor = Actor {
            stream,
            cache: cache.clone(),
            interrupts: interrupts.clone(),
        };

        match wire::read_response(&mut actor.stream)? {
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

        let (tx, rx) = mpsc::channel::<Job>();
        std::thread::Builder::new()
            .name("gm10-instrument".to_string())
            .spawn(move || actor.run(rx))
            .map_err(|e| io::Error::other(e.to_string()))?;

        Ok(Arc::new(Self {
            cache,
            interrupts,
            tx,
            peer_address,
        }))
    }

    pub fn register_interrupt(
        &self,
        category: InterruptCategory,
    ) -> tokio::sync::mpsc::Receiver<()> {
        self.interrupts.register(category)
    }

    /// Block until the actor thread has finished processing `command`.
    pub fn submit(&self, command: Command) -> io::Result<()> {
        let (done_tx, done_rx) = mpsc::channel();
        self.tx
            .send(Job {
                command,
                done: done_tx,
            })
            .map_err(|_| {
                io::Error::new(io::ErrorKind::BrokenPipe, "instrument actor thread is gone")
            })?;
        done_rx.recv().map_err(|_| {
            io::Error::new(io::ErrorKind::BrokenPipe, "instrument actor thread is gone")
        })?
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
            ChannelFamily::Const => cache.constant[idx],
            ChannelFamily::VarConst => cache.varconstant[idx],
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
        expr.truncate(39);
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
    pub fn channel_start(&self, family: ChannelFamily, channel: u32) -> io::Result<()> {
        if family == ChannelFamily::Signal
            && matches!(
                self.channel_type(channel),
                ChannelType::None | ChannelType::Unknown
            )
        {
            return Ok(());
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
}

impl Actor {
    fn run(mut self, rx: mpsc::Receiver<Job>) {
        while let Ok(job) = rx.recv() {
            let result = self.process(job.command);
            let _ = job.done.send(result);
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
            Command::SetOpMode(on) => self.set_output_value(&cmd_orec_set(on)),
            Command::SetCompute(mode) => self.set_output_value(&cmd_omath_set(mode)),
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
            let text = String::from_utf8_lossy(bytes);
            let Some((code, parameter)) = parse_error_header(&text) else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "malformed E1 error header",
                ));
            };
            {
                let mut cache = self.cache.lock();
                cache.error_flag = true;
                cache.error.code = code;
                cache.error.parameter = parameter;
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

        let mut cache = self.cache.lock();
        for t in cache.meas_type.iter_mut() {
            *t = ChannelType::None;
        }
        for m in cache.modules.iter_mut() {
            m.use_flag = false;
            m.module_string.clear();
        }
        for line in &lines {
            if !module_line_ok(line) {
                continue;
            }
            cache.modules[line.index].module_string = line.set_message.clone();
            let Some((mod_type, channel_number)) = classify_module(&line.set_message) else {
                continue;
            };
            if mod_type == ModuleType::Pid {
                continue;
            }
            cache.modules[line.index].mod_type = mod_type;
            cache.modules[line.index].channel_number = channel_number;
            cache.modules[line.index].use_flag = true;

            let base = line.index * 100;
            let [inputs, outputs] = channel_number;
            match mod_type {
                ModuleType::InputAnalog => {
                    for j in 0..inputs as usize {
                        cache.meas_type[base + j] = ChannelType::InputAnalog;
                    }
                }
                ModuleType::InputDigital => {
                    for j in 0..inputs as usize {
                        cache.meas_type[base + j] = ChannelType::InputBinary;
                    }
                }
                ModuleType::InputPulse => {
                    for j in 0..inputs as usize {
                        cache.meas_type[base + j] = ChannelType::InputInteger;
                    }
                }
                ModuleType::OutputAnalog => {
                    for j in 0..outputs as usize {
                        cache.meas_type[base + j] = ChannelType::OutputAnalog;
                    }
                }
                ModuleType::OutputDigital => {
                    for j in 0..outputs as usize {
                        cache.meas_type[base + j] = ChannelType::OutputBinary;
                    }
                }
                ModuleType::InputOutputDigital => {
                    for j in 0..inputs as usize {
                        cache.meas_type[base + j] = ChannelType::InputBinary;
                    }
                    for j in 0..outputs as usize {
                        cache.meas_type[base + inputs as usize + j] = ChannelType::OutputBinary;
                    }
                }
                ModuleType::Pid | ModuleType::Unknown => {}
            }
        }
        Ok(())
    }

    /// `load_status` (`drvGM10.c:625-656`).
    fn load_status(&mut self) -> io::Result<()> {
        let orec_payload = self.expect_ascii(&cmd_orec_query())?;
        let recording_mode = parse_orec(&String::from_utf8_lossy(&orec_payload)).unwrap_or(0) != 0;
        let omath_payload = self.expect_ascii(&cmd_omath_query())?;
        let compute_mode = parse_omath(&String::from_utf8_lossy(&omath_payload)).unwrap_or(0);

        let was_settings_mode;
        {
            let mut cache = self.cache.lock();
            cache.recording_mode = recording_mode;
            cache.compute_mode = compute_mode;
            was_settings_mode = cache.settings_mode;
            cache.settings_mode = recording_mode && (compute_mode != 0);
        }
        self.interrupts.fire(InterruptCategory::Status);

        if was_settings_mode && !self.cache.lock().settings_mode {
            // C enqueues this fire-and-forget (`qmesg(..., NULL, ...)`,
            // `drvGM10.c:646`); running it inline keeps this single
            // worker thread the only actor either way.
            self.load_infos()?;
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
            let mut cache = self.cache.lock();
            for line in &info_lines {
                if line.family == InfoFamily::Signal && !cache.modules[line.index / 100].use_flag {
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
                match line.family {
                    InfoFamily::Signal => {
                        cache.meas_info[line.index].ch_status = ch_status;
                        cache.meas_info[line.index].unit = line.unit.clone();
                        cache.meas_info[line.index].scale = line.scale;
                        if line.status == CH_STATUS_SKIP {
                            cache.meas_data[line.index].data_status = DataStatus::Skip;
                        }
                    }
                    InfoFamily::Math => {
                        cache.calc_info[line.index].ch_status = ch_status;
                        cache.calc_info[line.index].unit = line.unit.clone();
                        cache.calc_info[line.index].scale = line.scale;
                        if line.status == CH_STATUS_SKIP {
                            cache.calc_data[line.index].data_status = DataStatus::Skip;
                        }
                    }
                    InfoFamily::Comm => {
                        cache.comm_info[line.index].ch_status = ch_status;
                        cache.comm_info[line.index].unit = line.unit.clone();
                        cache.comm_info[line.index].scale = line.scale;
                        if line.status == CH_STATUS_SKIP {
                            cache.comm_data[line.index].data_status = DataStatus::Skip;
                        }
                    }
                }
            }
        }

        let srangeao_payload = self.expect_ascii(&cmd_srangeao_query())?;
        let ao_lines = parse_srangeao(&srangeao_payload);
        let srangedo_payload = self.expect_ascii(&cmd_srangedo_query())?;
        let do_lines = parse_srangedo(&srangedo_payload);
        {
            let mut cache = self.cache.lock();
            for line in ao_lines.iter().chain(do_lines.iter()) {
                if !cache.modules[line.index / 100].use_flag {
                    continue;
                }
                cache.meas_info[line.index].ch_mode = line.mode;
            }
        }

        let srangemath_payload = self.expect_ascii(&cmd_srangemath_query())?;
        let math_lines = parse_srangemath(&srangemath_payload);
        {
            let mut cache = self.cache.lock();
            for line in &math_lines {
                cache.calc_expr[line.index] = ExprInfo {
                    on_flag: line.on_flag,
                    expr: line.expr.clone(),
                };
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
        {
            let mut cache = self.cache.lock();
            for record in &records {
                // `drvGM10.c:995`: applied uniformly regardless of
                // channel_type, exactly as the source does.
                if record.data_type != 1 {
                    continue;
                }
                let module_idx = (record.address as usize).saturating_sub(1) / 100;
                if !cache.modules[module_idx].use_flag {
                    continue;
                }
                let idx = (record.address - 1) as usize;
                let cd = match record.channel_type {
                    1 => &mut cache.meas_data[idx],
                    2 => &mut cache.calc_data[idx],
                    3 => &mut cache.comm_data[idx],
                    _ => continue,
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
            cache.alarm_flag = alarm_flag;
        }
        self.interrupts.fire(InterruptCategory::Channel);
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
                cache.constant[line.index] = line.value;
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
                cache.varconstant[line.index] = line.value;
            }
        }
        self.interrupts.fire(InterruptCategory::Misc);
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

/// `gm10_connect` (`drvGM10.c:1539-1552`): a name-keyed registry populated
/// by the `gm10Init` iocsh command.
#[derive(Default)]
pub struct Registry {
    instruments: Mutex<HashMap<String, Arc<Instrument>>>,
}

impl Registry {
    pub fn new() -> Self {
        Self {
            instruments: Mutex::new(HashMap::new()),
        }
    }

    pub fn insert(&self, name: String, instrument: Arc<Instrument>) -> Result<(), &'static str> {
        let mut instruments = self.instruments.lock();
        if instruments.contains_key(&name) {
            return Err("device already exists");
        }
        instruments.insert(name, instrument);
        Ok(())
    }

    pub fn get(&self, name: &str) -> Option<Arc<Instrument>> {
        self.instruments.lock().get(name).cloned()
    }
}

/// Fake-device fixtures shared with `device_support`'s tests: building a
/// `GmDevice` for tests requires a live `Arc<Instrument>`, which requires
/// running the full `connect_stream` init sequence against a fake device.
#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use std::io::{Read as _, Write as _};
    use std::net::TcpListener;

    pub(crate) fn ascii_frame(body: &[u8]) -> Vec<u8> {
        let mut out = b"EAxx".to_vec();
        out.extend_from_slice(body);
        out.extend_from_slice(b"EN\r\n");
        out
    }

    /// One `FData` record: meas channel 1 (address=1), normal status,
    /// value 1234, no alarms (byte layout matches
    /// `codec::tests::fdata_binary_decodes_one_record`).
    pub(crate) fn one_record_fdata_binary() -> Vec<u8> {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&0u32.to_be_bytes());
        raw.extend_from_slice(&[0u8; 10]);
        raw.extend_from_slice(&28u16.to_be_bytes());
        raw.extend_from_slice(&[0u8; 16]);
        raw.push((1 << 4) | 1);
        raw.push(0);
        raw.extend_from_slice(&1u16.to_be_bytes());
        raw.extend_from_slice(&[0, 0, 0, 0]);
        raw.extend_from_slice(&1234i32.to_be_bytes());
        let total = (raw.len() - 8) as u32;
        raw[4..8].copy_from_slice(&total.to_be_bytes());
        raw
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

    /// One connected instrument: module 0 is a present `InputAnalog`
    /// module, channel 1 is `InputAnalog` (unit "DEGC", value 1.234),
    /// `K1`/`W1` (Const/VarConst) are set to 12.5/-7.5. Every other
    /// Signal channel (e.g. 2) is `ChannelType::None` — "channel does not
    /// exist" gate tests use it directly.
    pub(crate) fn connect_default_fixture() -> Arc<Instrument> {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let responses = vec![
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
        ];
        let device = spawn_fake_device(listener, responses);
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
        assert!(instrument.submit(Command::ClearError).is_err());
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
}
