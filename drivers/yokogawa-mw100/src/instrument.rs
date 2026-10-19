//! The MW100 instrument actor: owns the blocking `TcpStream` and `Cache`,
//! serializing every wire operation through one OS thread (mirrors C's
//! single-consumer `queue_func`/`qmesg` pair, `drvMW100.c:1398-1462` —
//! `qmesg` enqueues from any EPICS scan thread, `queue_func` is the sole
//! socket owner). `DeviceSupport::read`/`write` collapse C's PACT-then-
//! completed-by-`queue_func` two-phase dance into one blocking round trip
//! on [`Instrument::submit`] — legitimate since nothing else here depends
//! on interleaving with other records' scans mid-flight.

use crate::cache::*;
use crate::codec::*;
use crate::link::ChannelFamily;
use crate::wire::{self, RawResponse};
use epics_rs::base::server::database::DeviceDone;
use epics_rs::base::server::driver_support::DriverSupport;
use parking_lot::Mutex;
use std::collections::VecDeque;
use std::fmt::Write as _;
use std::io;
use std::net::{SocketAddrV4, TcpStream};
use std::sync::mpsc;
use std::sync::{Arc, OnceLock};

const PORT: u16 = 34318;

/// `mw100_channel_io_handler`/`mw100_info_io_handler`/`mw100_status_io_handler`/
/// `mw100_error_io_handler` (`drvMW100.c:1643-1688`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptCategory {
    Input,
    Output,
    Info,
    Status,
    Error,
}

#[derive(Default)]
struct Interrupts {
    input: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    output: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    info: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    status: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
    error: Mutex<Vec<tokio::sync::mpsc::Sender<()>>>,
}

impl Interrupts {
    fn slot(&self, category: InterruptCategory) -> &Mutex<Vec<tokio::sync::mpsc::Sender<()>>> {
        match category {
            InterruptCategory::Input => &self.input,
            InterruptCategory::Output => &self.output,
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
    ReadAllInputs,
    ReadSignalInput(u32),
    ReadMath(u32),
    ReadAllOutputs,
    ReadSignalOutput(u32),
    ReadComm(u32),
    ReadConst(u32),
    ReadAllInfos,
    ReadStatus,
    SetSignalOutput(u32, f64),
    SetComm(u32, f64),
    SetConst(u32, f64),
    SetBinaryOutput(u32, bool),
    SetOpMode(i32),
    SetCompute(i32),
    ClearError,
    AcknowledgeAlarms,
}

/// Who `queue_func` hands the finished command back to: a blocking caller,
/// or the record whose `process()` it re-runs (drvMW100.c:1418-1424).
enum Done {
    Wait(mpsc::Sender<()>),
    Process(DeviceDone),
    NoRecord,
}

struct Job {
    command: Command,
    done: Done,
}

/// One `mw100Init` handle (C `struct devqueue`), looked up by name from the
/// [`Registry`].
pub struct Instrument {
    cache: Arc<Mutex<Cache>>,
    interrupts: Arc<Interrupts>,
    /// The `queue_func` thread's queue; unset when the connect or load
    /// failed, which leaves C's `dq` listed with no thread to drain it.
    tx: OnceLock<mpsc::Sender<Job>>,
    /// `mw100_system_info(which=0, ...)` (`drvMW100.c:1731-1742`): the
    /// configured address, as `inet_ntop` prints it.
    pub peer_address: String,
}

impl Instrument {
    fn new(peer_address: String) -> Arc<Self> {
        Arc::new(Self {
            cache: Arc::default(),
            interrupts: Arc::default(),
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

    /// The tail of `init_mw100` (`drvMW100.c:1531-1556`): read the
    /// unsolicited post-connect `E0` greeting, negotiate binary byte order,
    /// run the fixed load sequence (the cache keeps whatever loaded if a
    /// step fails), then start the queue thread.
    fn start(&self, stream: TcpStream) -> io::Result<()> {
        let (tx, rx) = mpsc::channel::<Job>();
        let mut actor = Actor {
            stream,
            cache: self.cache.clone(),
            interrupts: self.interrupts.clone(),
            rx,
            backlog: VecDeque::new(),
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

        // `#if EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE` (`drvMW100.c:1538-1543`):
        // this port always decodes little-endian, so it always negotiates BO1.
        actor.expect_ok(&cmd_bo1())?;

        actor.load_modules()?;
        actor.load_status()?;
        actor.load_infos()?;
        actor.load_input_values(None)?;
        actor.load_output_values(None)?;

        std::thread::Builder::new()
            .name("mw100-instrument".to_string())
            .spawn(move || actor.run())?;
        let _ = self.tx.set(tx);
        Ok(())
    }

    pub fn register_interrupt(
        &self,
        category: InterruptCategory,
    ) -> tokio::sync::mpsc::Receiver<()> {
        self.interrupts.register(category)
    }

    /// Block until the actor thread has finished processing `command`.
    /// Returns nothing: C's queue reports no status to the record
    /// (`queue_func`, drvMW100.c:1398-1424), so a device or wire error cannot alarm.
    pub fn submit(&self, command: Command) {
        let (done_tx, done_rx) = mpsc::channel();
        if self.send(command, Done::Wait(done_tx)) {
            let _ = done_rx.recv();
        }
    }

    /// C `qmesg(dq, precord, ...)`: queue `command` and return at once; the
    /// actor completes `done` when it has run. On a device with no queue
    /// thread `done` is dropped un-run and the record stays in PACT, as C's
    /// packet sits on a list nobody drains.
    pub fn submit_write(&self, command: Command, done: DeviceDone) {
        self.send(command, Done::Process(done));
    }

    fn send(&self, command: Command, done: Done) -> bool {
        self.tx
            .get()
            .is_some_and(|tx| tx.send(Job { command, done }).is_ok())
    }

    fn cache(&self) -> parking_lot::MutexGuard<'_, Cache> {
        self.cache.lock()
    }

    fn channel_type(&self, channel: u32) -> ChannelType {
        self.cache().ch_type[(channel - 1) as usize]
    }

    /// `mw100_channel_io_handler`'s `ADDR_SIGNAL` arm (`drvMW100.c:1647-1660`):
    /// which IOSCANPVT a Signal channel's I/O Intr scan wakes on depends on
    /// its concrete `ChannelType`, unlike Math (always `Input`) or Comm/Const
    /// (always `Output`).
    pub fn signal_interrupt_category(&self, channel: u32) -> InterruptCategory {
        match self.channel_type(channel) {
            ChannelType::OutputBinary | ChannelType::OutputAnalog => InterruptCategory::Output,
            _ => InterruptCategory::Input,
        }
    }

    // -- test_* (`drvMW100.c:1569-1626`): pure cache reads, used at
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

    // -- get_*/analog_get/etc (`drvMW100.c:1868-2047`): pure cache reads. --

    /// `mw100_analog_get` (`drvMW100.c:1868-1895`): only `Signal` is gated
    /// on channel existence — Math/Comm/Const resolve unconditionally.
    pub fn analog_get(&self, family: ChannelFamily, channel: u32) -> f64 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.ch_type[idx] {
                ChannelType::None | ChannelType::Unknown => 0.0,
                _ => scaled_value(cache.ch_data[idx].value, cache.ch_info[idx].scale),
            },
            ChannelFamily::Math => {
                scaled_value(cache.calc_data[idx].value, cache.calc_info[idx].scale)
            }
            ChannelFamily::Comm => f64::from(cache.comm_input[idx]),
            ChannelFamily::Const => f64::from(cache.constant[idx]),
        }
    }

    /// `mw100_integer_get` (`drvMW100.c:1897-1905`).
    pub fn integer_get(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.ch_type[idx] {
            ChannelType::None | ChannelType::Unknown => 0,
            _ => cache.ch_data[idx].value,
        }
    }

    /// `mw100_binary_get` (`drvMW100.c:1907-1915`): identical body to
    /// [`Self::integer_get`] in the C source too.
    pub fn binary_get(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.ch_type[idx] {
            ChannelType::None | ChannelType::Unknown => 0,
            _ => cache.ch_data[idx].value,
        }
    }

    /// `mw100_channel_get_egu` (`drvMW100.c:1918-1935`): only `Signal`/`Math`
    /// have a case at all — `UNIT`'s own link grammar never reaches Comm/Const.
    pub fn channel_get_egu(&self, family: ChannelFamily, channel: u32) -> String {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.ch_type[idx] {
                ChannelType::None | ChannelType::Unknown => String::new(),
                _ => cache.ch_info[idx].unit.clone(),
            },
            ChannelFamily::Math => cache.calc_info[idx].unit.clone(),
            ChannelFamily::Comm | ChannelFamily::Const => String::new(),
        }
    }

    /// `mw100_channel_get_expr` (`drvMW100.c:1937-1945`): the C source splits
    /// storage by `channel <= 60 ? calc_expr : short_calc_expr`, a split this
    /// port's single `Vec<ExprInfo>` (sized for the full 300-channel range)
    /// deliberately does not reproduce — see [`crate::codec`] docs on the
    /// buffer-overflow defect that split caused.
    pub fn channel_get_expr(&self, channel: u32) -> String {
        self.cache().calc_expr[(channel - 1) as usize].expr.clone()
    }

    /// `mw100_get_channel_status` (`drvMW100.c:1947-1964`).
    pub fn get_channel_status(&self, family: ChannelFamily, channel: u32) -> u8 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.ch_type[idx] {
                ChannelType::None | ChannelType::Unknown => 0,
                _ => cache.ch_info[idx].ch_status as u8,
            },
            ChannelFamily::Math => cache.calc_info[idx].ch_status as u8,
            ChannelFamily::Comm | ChannelFamily::Const => 0,
        }
    }

    /// `mw100_get_channel_mode` (`drvMW100.c:1966-1974`): Signal-only (no
    /// `type` parameter in the C signature at all), and only meaningful for
    /// the two output channel types.
    pub fn get_channel_mode(&self, channel: u32) -> i32 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match cache.ch_type[idx] {
            ChannelType::OutputBinary | ChannelType::OutputAnalog => cache.ch_info[idx].ch_mode,
            _ => 0,
        }
    }

    /// `mw100_get_data_status` (`drvMW100.c:1976-1993`).
    pub fn get_data_status(&self, family: ChannelFamily, channel: u32) -> DataStatus {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        match family {
            ChannelFamily::Signal => match cache.ch_type[idx] {
                ChannelType::None | ChannelType::Unknown => DataStatus::Normal,
                _ => cache.ch_data[idx].data_status,
            },
            ChannelFamily::Math => cache.calc_data[idx].data_status,
            ChannelFamily::Comm | ChannelFamily::Const => DataStatus::Normal,
        }
    }

    /// `mw100_get_alarm_flag` (`drvMW100.c:1995-2000`).
    pub fn get_alarm_flag(&self) -> bool {
        self.cache().alarm_flag
    }

    /// `mw100_get_alarm` (`drvMW100.c:2002-2026`): `sub_channel` 0 means the
    /// aggregate `alarm_status`; 1-4 index `alarm[sub_channel-1]`.
    pub fn get_alarm(&self, family: ChannelFamily, channel: u32, sub_channel: u8) -> u8 {
        let cache = self.cache();
        let idx = (channel - 1) as usize;
        let data = match family {
            ChannelFamily::Signal => match cache.ch_type[idx] {
                ChannelType::None | ChannelType::Unknown => return 0,
                _ => &cache.ch_data[idx],
            },
            ChannelFamily::Math => &cache.calc_data[idx],
            ChannelFamily::Comm | ChannelFamily::Const => return 0,
        };
        if sub_channel == 0 {
            data.alarm_status
        } else {
            data.alarm[(sub_channel - 1) as usize]
        }
    }

    /// `mw100_get_error` (`drvMW100.c:2028-2040`): `channel` is 1-3. The
    /// `entry == None` (`dq->error == NULL`) case returns an empty string;
    /// the C source's `channel == 0` "Unknown error." fallback is dead code
    /// (every real `ERROR` stringin call validates channel to 1-3 before
    /// calling — `devMW100_stringin.c:1729-1740`) and is not reproduced.
    pub fn get_error(&self, channel: u32) -> String {
        let cache = self.cache();
        if !cache.error.flag {
            return String::new();
        }
        match cache.error.entry {
            Some(entry) => entry.strings[(channel - 1) as usize].to_string(),
            None => String::new(),
        }
    }

    pub fn get_error_flag(&self) -> bool {
        self.cache().error.flag
    }

    /// `mw100_get_mode` (`drvMW100.c:2049-2065`).
    pub fn get_settings_mode(&self) -> bool {
        self.cache().settings_mode
    }
    pub fn get_measurement_mode(&self) -> bool {
        self.cache().measurement_mode
    }
    pub fn get_compute_mode(&self) -> bool {
        self.cache().compute_mode
    }

    /// `mw100_module_info` (`drvMW100.c:1692-1728`): `MODULE_PRESENCE`/
    /// `MODULE_STRING` bypass the `use_flag` gate (they report presence
    /// itself); `MODULE_MODEL`/`MODULE_CODE`/`MODULE_SPEED`/`MODULE_NUMBER`
    /// return a default when the module is unused.
    pub fn module_presence(&self, module: usize) -> bool {
        self.cache().modules[module].use_flag
    }

    /// Two-way "empty"-or-string, simpler than GM10's three-way scheme —
    /// `sprintf(str,"empty")` is the C source's literal fallback
    /// (`drvMW100.c:1702-1703`).
    pub fn module_string(&self, module: usize) -> String {
        let cache = self.cache();
        let m = &cache.modules[module];
        if m.use_flag {
            m.module_string.clone()
        } else {
            "empty".to_string()
        }
    }

    pub fn module_model(&self, module: usize) -> i32 {
        let cache = self.cache();
        if !cache.modules[module].use_flag {
            return 0;
        }
        cache.modules[module].model
    }

    pub fn module_code(&self, module: usize) -> String {
        let cache = self.cache();
        if !cache.modules[module].use_flag {
            return String::new();
        }
        cache.modules[module].code.clone()
    }

    pub fn module_speed(&self, module: usize) -> i32 {
        let cache = self.cache();
        if !cache.modules[module].use_flag {
            return 0;
        }
        cache.modules[module].speed
    }

    pub fn module_number(&self, module: usize) -> i32 {
        let cache = self.cache();
        if !cache.modules[module].use_flag {
            return 0;
        }
        cache.modules[module].number
    }

    /// `mw100_channel_start` (`drvMW100.c:1832-1866`): the first-scan async
    /// read-refresh for a single channel, collapsed to a blocking round
    /// trip. Signal channels with no recognized type are a silent no-op.
    pub fn channel_start(&self, family: ChannelFamily, channel: u32) {
        let command = match family {
            ChannelFamily::Signal => match self.channel_type(channel) {
                ChannelType::InputAnalog | ChannelType::InputInteger | ChannelType::InputBinary => {
                    Command::ReadSignalInput(channel)
                }
                ChannelType::OutputBinary | ChannelType::OutputAnalog => {
                    Command::ReadSignalOutput(channel)
                }
                ChannelType::None | ChannelType::Unknown => return,
            },
            ChannelFamily::Math => Command::ReadMath(channel),
            ChannelFamily::Comm => Command::ReadComm(channel),
            ChannelFamily::Const => Command::ReadConst(channel),
        };
        self.submit(command)
    }
}

pub struct Actor {
    stream: TcpStream,
    cache: Arc<Mutex<Cache>>,
    interrupts: Arc<Interrupts>,
    rx: mpsc::Receiver<Job>,
    /// Jobs taken off `rx` ahead of one the actor queued itself, in order.
    backlog: VecDeque<Job>,
}

impl Actor {
    fn run(mut self) {
        while let Some(job) = self.backlog.pop_front().or_else(|| self.rx.recv().ok()) {
            let _ = self.process(job.command);
            match job.done {
                Done::Wait(tx) => {
                    let _ = tx.send(());
                }
                Done::Process(done) => {
                    let _ = done.complete(done.priority());
                }
                Done::NoRecord => {}
            }
        }
    }

    /// C `qmesg(dq, NULL, ...)` from the queue thread: the job goes behind
    /// everything already queued.
    fn queue_self(&mut self, command: Command) {
        self.backlog.extend(self.rx.try_iter());
        self.backlog.push_back(Job {
            command,
            done: Done::NoRecord,
        });
    }

    fn process(&mut self, command: Command) -> io::Result<()> {
        match command {
            Command::ReadAllInputs => self.load_input_values(None),
            Command::ReadSignalInput(ch) => {
                self.load_input_values(Some((ChannelFamily::Signal, ch)))
            }
            Command::ReadMath(ch) => self.load_input_values(Some((ChannelFamily::Math, ch))),
            Command::ReadAllOutputs => self.load_output_values(None),
            Command::ReadSignalOutput(ch) => {
                self.load_output_values(Some((ChannelFamily::Signal, ch)))
            }
            Command::ReadComm(ch) => self.load_output_values(Some((ChannelFamily::Comm, ch))),
            Command::ReadConst(ch) => self.load_output_values(Some((ChannelFamily::Const, ch))),
            Command::ReadAllInfos => self.load_infos(),
            Command::ReadStatus => self.load_status(),
            Command::SetSignalOutput(ch, value) => self.set_signal_output(ch, value),
            Command::SetComm(ch, value) => self.expect_ok(&cmd_cmc_set(ch, value)),
            Command::SetConst(ch, value) => self.expect_ok(&cmd_skk_set(ch, value)),
            Command::SetBinaryOutput(ch, on) => self.set_binary_value(ch, on),
            Command::SetOpMode(value) => self.expect_ok_if(cmd_ds_set(value)),
            Command::SetCompute(value) => self.expect_ok_if(cmd_ex_set(value)),
            Command::ClearError => self.clear_error(),
            Command::AcknowledgeAlarms => self.acknowledge_alarms(),
        }
    }

    fn write_command(&mut self, command: &str) -> io::Result<()> {
        wire::write_command(&mut self.stream, command)
    }

    /// `response_reader` (`drvMW100.c:526-562`): resets BOTH the transient
    /// error flag and the matched-entry pointer on every round trip (C
    /// resets `error_flag`/`error` unconditionally at the top, before
    /// dispatching on response type). On a real `E1` error, decode the
    /// space-separated error code, look it up, and fire the error
    /// interrupt. On `E2` chained errors, only the flag is set — no lookup,
    /// no interrupt (`drvMW100.c:558-559` has no `scanIoRequest` call in
    /// that branch).
    fn read_response(&mut self) -> io::Result<RawResponse> {
        let raw = wire::read_response(&mut self.stream)?;
        {
            let mut cache = self.cache.lock();
            cache.error.flag = false;
            cache.error.entry = None;
        }
        match &raw {
            RawResponse::Error(bytes) => {
                self.cache.lock().error.flag = true;
                let text = String::from_utf8_lossy(bytes);
                let Some(code) = parse_error_code(&text) else {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "malformed E1 error header",
                    ));
                };
                self.cache.lock().error.entry = lookup_error(code);
                self.interrupts.fire(InterruptCategory::Error);
            }
            RawResponse::ChainErrors(_) => {
                self.cache.lock().error.flag = true;
            }
            _ => {}
        }
        Ok(raw)
    }

    /// `set_mode`'s `return 1` before writing: no command, nothing sent.
    fn expect_ok_if(&mut self, command: Option<String>) -> io::Result<()> {
        match command {
            Some(command) => self.expect_ok(&command),
            None => Ok(()),
        }
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

    /// Returns the full raw frame (leading `"EAxx"` header and trailing
    /// `"EN\r\n"` both still present) — every `codec` ASCII parser does its
    /// own `ptr += 4` header skip and treats the trailer's leading `'E'` as
    /// its scan terminator, matching `drvMW100.c`'s own per-consumer
    /// `ptr += 4` rather than a shared pre-strip.
    fn expect_ascii(&mut self, command: &str) -> io::Result<Vec<u8>> {
        self.write_command(command)?;
        match self.read_response()? {
            RawResponse::Ascii(raw) => Ok(raw),
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

    /// `load_modules` (`drvMW100.c:565-684`): must run before every other
    /// load function, which all index off `Cache::modules[..].use_flag`
    /// and `Cache::ch_type`. Addressing is base-10-per-module
    /// (`10*module_index + j`, `drvMW100.c:679-680`).
    fn load_modules(&mut self) -> io::Result<()> {
        let payload = self.expect_ascii(&cmd_cf0())?;
        let lines = parse_cf0(&payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed CF0 body"))?;

        let mut cache = self.cache.lock();
        for t in cache.ch_type.iter_mut() {
            *t = ChannelType::None;
        }
        for m in cache.modules.iter_mut() {
            *m = Module::default();
        }
        for line in &lines {
            if line.index >= MAX_MODULES {
                continue;
            }
            {
                let module = &mut cache.modules[line.index];
                module.set_message = line.set_message;
                module.status_message = line.status_message;
                module.error_message = line.error_message.clone();
            }
            if !module_line_ok(line) {
                continue;
            }
            let (model, code, speed, number) = classify_module_string(&line.set_message);
            let module = &mut cache.modules[line.index];
            module.use_flag = true;
            module.module_string = String::from_utf8_lossy(&line.set_message).into_owned();
            module.model = model;
            module.code = code;
            module.speed = speed;
            module.number = number;
        }
        for i in 0..MAX_MODULES {
            if !cache.modules[i].use_flag {
                continue;
            }
            let ty = channel_type_for_model(cache.modules[i].model);
            let number = cache.modules[i].number.max(0) as usize;
            let base = 10 * i;
            for j in 0..number {
                if let Some(slot) = cache.ch_type.get_mut(base + j) {
                    *slot = ty;
                }
            }
        }
        Ok(())
    }

    /// `load_status` (`drvMW100.c:686-728`): only `status[4]`'s bottom 3
    /// bits are used. A settings-mode 1-to-0 transition queues an info
    /// reload (`drvMW100.c:704-713`).
    fn load_status(&mut self) -> io::Result<()> {
        let payload = self.expect_ascii(&cmd_is0())?;
        let status = parse_is0(&payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed IS0 body"))?;
        let s4 = status[4];
        let settings_ended;
        {
            let mut cache = self.cache.lock();
            if s4 & 1 != 0 {
                cache.settings_mode = true;
                settings_ended = false;
            } else {
                settings_ended = cache.settings_mode;
                cache.settings_mode = false;
            }
            cache.measurement_mode = s4 & 2 != 0;
            cache.compute_mode = s4 & 4 != 0;
        }
        if settings_ended {
            self.queue_self(Command::ReadAllInfos);
        }
        self.interrupts.fire(InterruptCategory::Status);
        Ok(())
    }

    /// `load_infos` (`drvMW100.c:730-1004`): FE1, FO0, AO?, XD?, SO? — all
    /// five share one `info_ioscanpvt` fire at the end.
    fn load_infos(&mut self) -> io::Result<()> {
        let fe1_payload = self.expect_ascii(&cmd_fe1())?;
        let fe1_lines = parse_fe1(&fe1_payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FE1 body"))?;
        {
            let mut cache = self.cache.lock();
            for line in &fe1_lines {
                let ch_status = ch_status_from_wire(line.status);
                let info = match line.family {
                    InfoFamily::Signal => cache.ch_info.get_mut(line.index),
                    InfoFamily::Math => cache.calc_info.get_mut(line.index),
                };
                if let Some(info) = info {
                    info.ch_status = ch_status;
                    info.unit = line.unit.clone();
                    info.scale = line.scale;
                }
                // drvMW100.c:799: FD1 never reports a skipped channel.
                if ch_status == ChStatus::Skip {
                    let data = match line.family {
                        InfoFamily::Signal => cache.ch_data.get_mut(line.index),
                        InfoFamily::Math => cache.calc_data.get_mut(line.index),
                    };
                    if let Some(data) = data {
                        data.data_status = DataStatus::SkipOff;
                    }
                }
            }
        }

        let fo0_payload = self.expect_ascii(&cmd_fo0())?;
        let fo0_lines = parse_fo0(&fo0_payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FO0 body"))?;
        {
            let mut cache = self.cache.lock();
            for line in &fo0_lines {
                if let Some(info) = cache.ch_info.get_mut(line.index) {
                    info.ch_status = ch_status_from_wire(line.status);
                }
            }
        }

        let ao_payload = self.expect_ascii(&cmd_ao_query())?;
        let ao_lines = parse_ao(&ao_payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed AO? body"))?;
        {
            let mut cache = self.cache.lock();
            for line in &ao_lines {
                if let Some(info) = cache.ch_info.get_mut(line.index) {
                    info.ch_mode = line.mode;
                }
            }
        }

        let xd_payload = self.expect_ascii(&cmd_xd_query())?;
        let status_snapshot: Vec<u8> = {
            self.cache
                .lock()
                .ch_info
                .iter()
                .map(|info| ch_status_to_wire(info.ch_status))
                .collect()
        };
        let xd_lines = parse_xd(&xd_payload, |idx| {
            status_snapshot
                .get(idx)
                .copied()
                .unwrap_or(CH_STATUS_UNKNOWN)
        })
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed XD? body"))?;
        {
            let mut cache = self.cache.lock();
            for line in &xd_lines {
                if let Some(info) = cache.ch_info.get_mut(line.index) {
                    info.ch_mode = line.mode;
                }
            }
        }

        let so_payload = self.expect_ascii(&cmd_so_query())?;
        let so_lines = parse_so(&so_payload)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed SO? body"))?;
        {
            let mut cache = self.cache.lock();
            for line in &so_lines {
                if let Some(expr) = cache.calc_expr.get_mut(line.index) {
                    expr.on_flag = line.on_flag;
                    expr.expr = line.expr.clone();
                }
            }
        }

        self.interrupts.fire(InterruptCategory::Info);
        Ok(())
    }

    /// `load_input_values` (`drvMW100.c:1037-1134`). `scope`: `None` = all
    /// channels (`FD1,001,A300`), `Some((family, channel))` = one channel
    /// (Signal or Math only — Comm/Const never route here).
    fn load_input_values(&mut self, scope: Option<(ChannelFamily, u32)>) -> io::Result<()> {
        let command = match scope {
            None => cmd_fd1_all(),
            Some((ChannelFamily::Signal, ch)) => cmd_fd1_signal(ch),
            Some((ChannelFamily::Math, ch)) => cmd_fd1_math(ch),
            Some((ChannelFamily::Comm | ChannelFamily::Const, _)) => return Ok(()),
        };
        let raw = self.expect_binary(&command)?;
        let records = parse_fd1_binary(&raw)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FD1 frame"))?;

        let old_alarm_flag = self.cache.lock().alarm_flag;
        let mut new_alarm_flag = false;
        {
            let mut cache = self.cache.lock();
            for record in &records {
                let cd = if record.address > 100 {
                    cache.calc_data.get_mut((record.address - 101) as usize)
                } else {
                    cache.ch_data.get_mut((record.address - 1) as usize)
                };
                let Some(cd) = cd else { continue };

                cd.data_status = DataStatus::from_wire(record.value);
                cd.value = if cd.data_status == DataStatus::Normal {
                    record.value as i32
                } else {
                    0
                };
                cd.alarm = [
                    record.alarms1 & 0xF,
                    record.alarms1 & 0xF0,
                    record.alarms2 & 0xF,
                    record.alarms2 & 0xF0,
                ];
                cd.alarm_status = ((cd.alarm[0] != 0) as u8)
                    | (((cd.alarm[1] != 0) as u8) << 1)
                    | (((cd.alarm[2] != 0) as u8) << 2)
                    | (((cd.alarm[3] != 0) as u8) << 3);
                if cd.alarm_status != 0 {
                    new_alarm_flag = true;
                }
            }
            cache.alarm_flag = new_alarm_flag;
        }
        if scope.is_none() || new_alarm_flag != old_alarm_flag {
            self.interrupts.fire(InterruptCategory::Input);
        }
        Ok(())
    }

    /// `load_output_values` (`drvMW100.c:1137-1244`): three independently
    /// gated sub-blocks (Signal binary `FO1`, Comm ASCII `CM?`/`CMC?`, Const
    /// ASCII `SK?`/`SKK?`) — all three run for the all-channel variant, or
    /// exactly one for a single-family read.
    fn load_output_values(&mut self, scope: Option<(ChannelFamily, u32)>) -> io::Result<()> {
        let want_signal = matches!(scope, None | Some((ChannelFamily::Signal, _)));
        let want_comm = matches!(scope, None | Some((ChannelFamily::Comm, _)));
        let want_const = matches!(scope, None | Some((ChannelFamily::Const, _)));

        if want_signal {
            let command = match scope {
                None => cmd_fo1_all(),
                Some((_, ch)) => cmd_fo1_signal(ch),
            };
            let raw = self.expect_binary(&command)?;
            let records = parse_fo1_binary(&raw)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "malformed FO1 frame"))?;
            let mut cache = self.cache.lock();
            for record in &records {
                if let Some(cd) = cache.ch_data.get_mut((record.address - 1) as usize) {
                    cd.data_status = DataStatus::Normal;
                    cd.value = record.value as i32;
                }
            }
        }

        if want_comm {
            let command = match scope {
                None => cmd_cm_query_all(),
                Some((_, ch)) => cmd_cmc_query(ch),
            };
            let payload = self.expect_ascii(&command)?;
            let lines = parse_comm_lines(&payload);
            let mut cache = self.cache.lock();
            for line in &lines {
                if let Some(slot) = cache.comm_input.get_mut(line.index) {
                    *slot = line.value as f32;
                }
            }
        }

        if want_const {
            let command = match scope {
                None => cmd_sk_query_all(),
                Some((_, ch)) => cmd_skk_query(ch),
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

        if scope.is_none() {
            self.interrupts.fire(InterruptCategory::Output);
        }
        Ok(())
    }

    /// `mw100_analog_set`'s `ADDR_SIGNAL` arm (`drvMW100.c:1750-1754`): a
    /// silent no-op unless the channel is genuinely `OutputAnalog`. Value is
    /// clamped to `[-10.0, 10.0]` then unscaled before sending
    /// (`set_output_value`, `drvMW100.c:1255-1263`).
    fn set_signal_output(&mut self, channel: u32, value: f64) -> io::Result<()> {
        let idx = (channel - 1) as usize;
        let (is_output_analog, scale) = {
            let cache = self.cache.lock();
            (
                cache.ch_type[idx] == ChannelType::OutputAnalog,
                cache.ch_info[idx].scale,
            )
        };
        if !is_output_analog {
            return Ok(());
        }
        let clamped = value.clamp(-10.0, 10.0);
        let sval = unscaled_value(clamped, scale);
        self.expect_ok(&cmd_sp_set(channel, sval))
    }

    /// `mw100_binary_set` (`drvMW100.c:1766-1773`).
    fn set_binary_value(&mut self, channel: u32, on: bool) -> io::Result<()> {
        let is_output_binary =
            self.cache.lock().ch_type[(channel - 1) as usize] == ChannelType::OutputBinary;
        if !is_output_binary {
            return Ok(());
        }
        self.expect_ok(&cmd_vd_set(channel, on))
    }

    /// `clear_error` (`drvMW100.c:1324-1336`).
    fn clear_error(&mut self) -> io::Result<()> {
        self.expect_ok(&cmd_ce0())?;
        {
            let mut cache = self.cache.lock();
            cache.error.flag = false;
            cache.error.entry = None;
        }
        self.interrupts.fire(InterruptCategory::Error);
        Ok(())
    }

    /// `acknowledge_alarms` (`drvMW100.c:1338-1347`): also forces an
    /// immediate all-channel input refresh "in case scan is slow".
    fn acknowledge_alarms(&mut self) -> io::Result<()> {
        self.expect_ok(&cmd_ak0())?;
        self.load_input_values(None)
    }
}

fn ch_status_from_wire(status: u8) -> ChStatus {
    match status {
        CH_STATUS_NORMAL => ChStatus::Normal,
        CH_STATUS_DIFF => ChStatus::Diff,
        CH_STATUS_SKIP => ChStatus::Skip,
        _ => ChStatus::Unknown,
    }
}

fn ch_status_to_wire(status: ChStatus) -> u8 {
    match status {
        ChStatus::Normal => CH_STATUS_NORMAL,
        ChStatus::Diff => CH_STATUS_DIFF,
        ChStatus::Skip => CH_STATUS_SKIP,
        ChStatus::Unknown => CH_STATUS_UNKNOWN,
    }
}

/// One `mw100Init` entry (C `struct queue_link` plus its `devqueue`).
struct Device {
    name: String,
    instrument: Arc<Instrument>,
}

/// `mw100_queue_list` (`drvMW100.c:449`): every `mw100Init` device,
/// newest first.
#[derive(Default)]
pub struct Registry {
    devices: Mutex<Vec<Device>>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// `init_mw100` (`drvMW100.c:1465-1561`), which reports nothing; `false`
    /// is its `return 1`. A device whose connect or load fails stays listed
    /// and resolves records, which park on a queue no thread serves, as in
    /// C. A bad address registers nothing, where C leaves a freed
    /// `devqueue` in the list (`drvMW100.c:1525-1529`).
    pub fn init_mw100(&self, device: Option<&[u8]>, address: Option<&[u8]>) -> bool {
        self.init_on_port(device, address, PORT)
    }

    /// [`Self::init_mw100`] connecting to `port`, C's `PORT` there; a test's
    /// fake unit listens on a port of its own.
    fn init_on_port(&self, device: Option<&[u8]>, address: Option<&[u8]>, port: u16) -> bool {
        let Some(name) = device.filter(|d| valid_device_name(d)) else {
            return false;
        };
        let name = String::from_utf8_lossy(name).into_owned();
        if self.devices.lock().iter().any(|d| d.name == name) {
            return false;
        }
        let Some(address) = address.and_then(inet_aton) else {
            return false;
        };
        let instrument = Instrument::new(address.to_string());
        self.devices.lock().insert(
            0,
            Device {
                name,
                instrument: instrument.clone(),
            },
        );
        TcpStream::connect(SocketAddrV4::new(address, port))
            .and_then(|stream| instrument.start(stream))
            .is_ok()
    }

    /// `mw100_connect` (`drvMW100.c:1628-1641`).
    pub fn get(&self, name: &str) -> Option<Arc<Instrument>> {
        self.devices
            .lock()
            .iter()
            .find(|d| d.name == name)
            .map(|d| d.instrument.clone())
    }

    #[cfg(test)]
    pub(crate) fn insert_connected(&self, name: &str, instrument: Arc<Instrument>) {
        self.devices.lock().insert(
            0,
            Device {
                name: name.to_string(),
                instrument,
            },
        );
    }
}

/// `init_mw100`'s name check (`drvMW100.c:1477-1490`): a letter, then
/// letters, digits or `_`.
fn valid_device_name(name: &[u8]) -> bool {
    match name.split_first() {
        Some((first, rest)) => {
            first.is_ascii_alphabetic()
                && rest.iter().all(|&c| c.is_ascii_alphanumeric() || c == b'_')
        }
        None => false,
    }
}

/// `mw100_report` (`drvMW100.c:459-505`).
impl DriverSupport for Registry {
    fn report(&self, level: i32) -> Option<Vec<u8>> {
        let mut out = String::new();
        for (cnt, device) in self.devices.lock().iter().enumerate() {
            let _ = writeln!(
                out,
                "    Yokogawa MW100 #{}: {} @ {}",
                cnt + 1,
                device.name,
                device.instrument.peer_address
            );
            if level != 0 {
                out.push_str("        Modules:\n");
                for (i, module) in device.instrument.cache().modules.iter().take(6).enumerate() {
                    if module.use_flag {
                        let _ = writeln!(out, "        #{}: {}", i + 1, module.module_string);
                    } else {
                        let _ = writeln!(out, "        #{}: empty", i + 1);
                    }
                }
            }
        }
        Some(out.into_bytes())
    }
}

/// Fake-device fixtures shared with `device_support`'s tests: building an
/// `MwDevice` for tests requires a live `Arc<Instrument>`, which requires
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

    /// `IS0` response with all 8 fixed 3-digit fields zeroed (settings =
    /// measurement = compute = false). Matches `codec::parse_is0`'s exact
    /// fixed-width layout: 8 groups of 3 ASCII digits + 1 delimiter byte.
    pub(crate) fn is0_all_zero() -> Vec<u8> {
        ascii_frame(&b"000x".repeat(8))
    }

    /// `FE1` response: one Signal channel (1), Normal status, unit "DEGC",
    /// scale 3 — matches `codec::parse_fe1`'s exact byte layout (status
    /// char + 1 delim, 4-byte channel field, unit up to the next space, then
    /// `,` + 3-digit scale).
    pub(crate) fn fe1_one_signal_channel() -> Vec<u8> {
        ascii_frame(b"N 0001DEGC ,003\n")
    }

    /// One `CF0` present module: index 0, `MX110-UNV-M06` (model=110 =>
    /// `InputAnalog`, 6 channels), matching
    /// `codec::tests::cf0_parses_one_present_module`'s byte layout.
    pub(crate) fn cf0_one_input_analog_module() -> Vec<u8> {
        let mut body = Vec::new();
        body.push(b'0'); // module index 0
        body.extend_from_slice(b"xxx"); // 3 unexamined bytes
        body.extend_from_slice(b"MX110-UNV-M06"); // 13-byte set_message
        body.extend_from_slice(b"xxx"); // 3 unexamined bytes
        body.extend_from_slice(b"MX110-UNV-M06"); // 13-byte status_message
        body.push(b'x'); // 1 unexamined byte
        body.extend_from_slice(b"\r\n"); // empty error_message + CRLF
        body.push(b'E');
        ascii_frame(&body)
    }

    /// One `FD1` record: Signal channel 1 (address=1), normal status, value
    /// 1234, no alarms — record layout matches
    /// `codec::tests::fd1_binary_decodes_one_signal_record`'s byte layout,
    /// but (unlike that parser-only test) this fixture is read through
    /// [`wire::read_response`], which trusts the header's `length` field for
    /// the *total* wire byte count (`8 + length`) rather than deriving it
    /// from the record area — so the trailing 2 bytes this frame's own
    /// `length=30` implies (beyond what `parse_fd1_binary` actually
    /// consumes) must physically be present, or the reader blocks forever
    /// waiting for bytes the fake device already stopped sending.
    pub(crate) fn one_record_fd1_binary() -> Vec<u8> {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&30u32.to_le_bytes());
        raw.extend_from_slice(&[0u8; 20]);
        raw.extend_from_slice(&1u16.to_le_bytes());
        raw.push(0);
        raw.push(0);
        raw.extend_from_slice(&1234u32.to_le_bytes());
        raw.extend_from_slice(&[0u8; 2]);
        raw
    }

    /// Empty `FO1` binary frame (no output records) — `length == 22`
    /// (`number_values == 0`). Total wire bytes must be exactly `8 + 22`
    /// (see [`one_record_fd1_binary`] on why this has to match precisely
    /// rather than merely be "enough").
    pub(crate) fn empty_fo1_binary() -> Vec<u8> {
        let mut raw = vec![b'E', b'B', 0, 0];
        raw.extend_from_slice(&22u32.to_le_bytes());
        raw.extend_from_slice(&[0u8; 22]);
        raw
    }

    /// Plays the device side of one connection: an unsolicited `E0`
    /// greeting (matching `init_mw100`'s post-connect read), then one canned
    /// reply per entry in `responses`, in order. Returns every
    /// `\r\n`-terminated command line it received, for asserting exact wire
    /// traffic.
    pub(crate) fn spawn_fake_device(
        listener: TcpListener,
        responses: Vec<Vec<u8>>,
    ) -> std::thread::JoinHandle<Vec<String>> {
        spawn_gated_fake_device(listener, responses, None)
    }

    /// [`spawn_fake_device`], holding reply `n` back until `gate` fires.
    pub(crate) fn spawn_gated_fake_device(
        listener: TcpListener,
        responses: Vec<Vec<u8>>,
        gate: Option<(usize, mpsc::Receiver<()>)>,
    ) -> std::thread::JoinHandle<Vec<String>> {
        std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream.write_all(b"E0\r\n").unwrap();
            let mut received = Vec::new();
            for (n, response) in responses.into_iter().enumerate() {
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
                if let Some((at, gate)) = &gate
                    && *at == n
                {
                    gate.recv().unwrap();
                }
                stream.write_all(&response).unwrap();
            }
            received
        })
    }

    /// One connected instrument: module 0 is a present `InputAnalog` module
    /// (6 channels), channel 1 is `InputAnalog` (unit "DEGC", value 1.234).
    /// `K1`/`C1` (Const/Comm) are set to 12.5/-7.5. Every other Signal
    /// channel (e.g. 7) is `ChannelType::None` — "channel does not exist"
    /// gate tests use it directly.
    pub(crate) fn connect_default_fixture() -> Arc<Instrument> {
        let (instrument, device) = connect_fixture_then(Vec::new());
        device.join().unwrap();
        instrument
    }

    /// [`connect_default_fixture`], with the device then answering `extra`
    /// in order; join the handle for every command line it received.
    pub(crate) fn connect_fixture_then(
        extra: Vec<Vec<u8>>,
    ) -> (Arc<Instrument>, std::thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut responses = vec![
            b"E0\r\n".to_vec(),             // BO1
            cf0_one_input_analog_module(),  // CF0
            is0_all_zero(),                 // IS0
            fe1_one_signal_channel(),       // FE1
            ascii_frame(b"E"),              // FO0
            ascii_frame(b"E"),              // AO?
            ascii_frame(b"E"),              // XD?
            ascii_frame(b"E"),              // SO?
            one_record_fd1_binary(),        // FD1,001,A300
            empty_fo1_binary(),             // FO1,001,060
            ascii_frame(b"xxx001,-7.5\nE"), // CM?
            ascii_frame(b"xxx01,12.5\nE"),  // SK?
        ];
        responses.extend(extra);
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        (instrument, device)
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::*;
    use super::*;
    use std::net::TcpListener;

    #[test]
    fn settings_end_queues_the_info_reload_behind_queued_jobs() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let mut settings = b"000x".repeat(8);
        settings[16..19].copy_from_slice(b"001");
        let responses = vec![
            b"E0\r\n".to_vec(),
            cf0_one_input_analog_module(),
            ascii_frame(&settings), // IS0: settings mode
            fe1_one_signal_channel(),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            one_record_fd1_binary(),
            empty_fo1_binary(),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            is0_all_zero(),          // IS0: settings mode ended
            one_record_fd1_binary(), // the job queued behind it
            fe1_one_signal_channel(),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
        ];
        let (release, gate) = mpsc::channel();
        let device = spawn_gated_fake_device(listener, responses, Some((12, gate)));
        let instrument = Instrument::connect_to(addr).unwrap();
        assert!(instrument.get_settings_mode());
        assert!(instrument.send(Command::ReadStatus, Done::NoRecord));
        assert!(instrument.send(Command::ReadAllInputs, Done::NoRecord));
        release.send(()).unwrap();
        let received = device.join().unwrap();
        assert_eq!(
            received[12..],
            [
                "IS0\r\n",
                "FD1,001,A300\r\n",
                "FE1,001,A300\r\n",
                "FO0,001,060\r\n",
                "AO?\r\n",
                "XD?\r\n",
                "SO?\r\n",
            ]
        );
    }

    #[test]
    fn fe1_skipped_channel_reads_skip_off() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let responses = vec![
            b"E0\r\n".to_vec(),
            cf0_one_input_analog_module(),
            is0_all_zero(),
            ascii_frame(b"N 0001DEGC ,003\nS A001\n"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            one_record_fd1_binary(),
            empty_fo1_binary(),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
        ];
        let device = spawn_fake_device(listener, responses);
        let instrument = Instrument::connect_to(addr).unwrap();
        device.join().unwrap();
        assert_eq!(
            instrument.get_data_status(ChannelFamily::Math, 1),
            DataStatus::SkipOff
        );
        assert_eq!(
            instrument.get_data_status(ChannelFamily::Math, 2),
            DataStatus::Normal
        );
    }

    #[test]
    fn connect_runs_full_init_sequence_then_error_round_trip() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let responses = vec![
            b"E0\r\n".to_vec(),
            cf0_one_input_analog_module(),
            is0_all_zero(),
            fe1_one_signal_channel(),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            ascii_frame(b"E"),
            one_record_fd1_binary(),
            empty_fo1_binary(),
            ascii_frame(b"xxx001,-0.1\nE"),
            ascii_frame(b"xxx01,0.3\nE"),
            b"E1 205 x\r\n".to_vec(), // CE0 -> device error
        ];
        let device = spawn_fake_device(listener, responses);

        let instrument = Instrument::connect_to(addr).unwrap();

        assert!(instrument.module_presence(0));
        assert_eq!(instrument.channel_get_egu(ChannelFamily::Signal, 1), "DEGC");
        assert_eq!(instrument.analog_get(ChannelFamily::Signal, 1), 1.234);
        // float comm_input[]/constant[] (drvMW100.c:436-437) round atof.
        assert_eq!(
            instrument.analog_get(ChannelFamily::Const, 1),
            f64::from(0.3f32)
        );
        assert_eq!(
            instrument.analog_get(ChannelFamily::Comm, 1),
            f64::from(-0.1f32)
        );
        assert!(!instrument.get_error_flag());

        let mut error_rx = instrument.register_interrupt(InterruptCategory::Error);
        instrument.submit(Command::ClearError);
        assert!(instrument.get_error_flag());
        assert_eq!(
            instrument.get_error(1),
            "Cannot execute during MATH operation."
        );
        assert!(error_rx.try_recv().is_ok());

        let received = device.join().unwrap();
        assert_eq!(
            received,
            vec![
                "BO1\r\n",
                "CF0\r\n",
                "IS0\r\n",
                "FE1,001,A300\r\n",
                "FO0,001,060\r\n",
                "AO?\r\n",
                "XD?\r\n",
                "SO?\r\n",
                "FD1,001,A300\r\n",
                "FO1,001,060\r\n",
                "CM?\r\n",
                "SK?\r\n",
                "CE0\r\n",
            ]
        );
    }

    #[test]
    fn init_mw100_rejects_bad_names_and_addresses_without_registering() {
        let registry = Registry::new();
        for name in [
            None,
            Some(&b""[..]),
            Some(b"1dau"),
            Some(b"dau-1"),
            Some(b"_dau"),
        ] {
            assert!(!registry.init_mw100(name, Some(b"127.0.0.1")));
        }
        assert!(!registry.init_mw100(Some(b"dau1"), None));
        assert!(!registry.init_mw100(Some(b"dau1"), Some(b"localhost")));
        assert_eq!(registry.report(1).as_deref(), Some(&b""[..]));
    }

    #[test]
    fn init_mw100_duplicate_check_runs_before_connect() {
        let registry = Registry::new();
        registry.insert_connected("dau1", connect_default_fixture());
        // Nothing listens at 192.0.2.1 (TEST-NET-1); a connect would hang.
        assert!(!registry.init_mw100(Some(b"dau1"), Some(b"192.0.2.1")));
        assert_eq!(
            registry.report(0).as_deref(),
            Some(&b"    Yokogawa MW100 #1: dau1 @ 127.0.0.1\n"[..])
        );
    }

    #[test]
    fn failed_init_stays_listed_and_resolves_with_no_queue_thread() {
        let listener = TcpListener::bind(("127.0.0.2", 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        // E0 greeting, BO1, CF0, then EOF during the IS0 status load.
        let device = spawn_fake_device(
            listener,
            vec![b"E0\r\n".to_vec(), cf0_one_input_analog_module()],
        );
        let registry = Registry::new();
        registry.insert_connected("first", connect_default_fixture());
        assert!(!registry.init_on_port(Some(b"dau2"), Some(b"127.0.0.2"), port));
        device.join().unwrap();

        let dau2 = registry.get("dau2").unwrap();
        assert!(dau2.tx.get().is_none());
        assert_eq!(dau2.module_string(0), "MX110-UNV-M06");
        assert!(!registry.init_mw100(Some(b"dau2"), Some(b"192.0.2.1")));
        assert_eq!(
            registry.report(1).unwrap(),
            b"    Yokogawa MW100 #1: dau2 @ 127.0.0.2\n\
             \x20       Modules:\n\
             \x20       #1: MX110-UNV-M06\n\
             \x20       #2: empty\n\
             \x20       #3: empty\n\
             \x20       #4: empty\n\
             \x20       #5: empty\n\
             \x20       #6: empty\n\
             \x20   Yokogawa MW100 #2: first @ 127.0.0.1\n\
             \x20       Modules:\n\
             \x20       #1: MX110-UNV-M06\n\
             \x20       #2: empty\n\
             \x20       #3: empty\n\
             \x20       #4: empty\n\
             \x20       #5: empty\n\
             \x20       #6: empty\n"
        );
    }
}
