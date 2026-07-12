//! SRS DG645 digital delay generator asyn port driver.
//!
//! Ported from `drvAsynDG645.cpp`. Every parameter is exposed as a tag
//! (asyn drvInfo string, e.g. `@asyn(DG0) TRIG_LEVEL`) resolved to a
//! reason via the default [`PortDriver::drv_user_create`] (lookup by
//! parameter name), matching the C driver's own tag-based `create()`
//! (`asynDrvUser::create`, `epicsStrCaseCmp` against `commandTable[].tag`)
//! — no override needed here.
//!
//! # EOS ownership
//! Like the other two delaygen drivers, DG645 never appends a terminator
//! itself — every command in [`COMMANDS`] is bare command text. The
//! reference `dg645.cmd` startup fragment configures the underlying
//! octet port with input EOS `\r\n` and output EOS `\n`
//! (`asynOctetSetInputEos`/`asynOctetSetOutputEos`), which the IOC's
//! `st.cmd` must reproduce.
//!
//! # Fixed upstream defects (doc/upstream-c-defects.md)
//! - **#32 GH output tag/wire-command inversion** (`drvAsynDG645.cpp:475-479`):
//!   for every other output (T0/AB/CD/EF) `*_STEP_NEG` maps to wire
//!   suffix `,0` and `*_STEP_POS` to `,1`. The C table swapped this for GH
//!   only, while its own trailing "minus"/"plus" comments still read arg 0
//!   = minus, arg 1 = plus — confirming a copy-paste bug in the driver's
//!   own tag<->wire-value table, not a DG645-defined per-channel
//!   difference (the SPLA/SPLO argument has one fixed wire meaning across
//!   every channel). Corrected here to the T0/AB/CD/EF convention; see
//!   `commands_use_consistent_step_polarity_convention` below.
//! - **#33 `T0_OFSET_STEP` tag typo** (`drvAsynDG645.cpp:436`; missing the
//!   second F, every other output uses `*_OFFSET_STEP`): the wire commands
//!   themselves (`SSLO?0`/`SSLO 0,%-.2f`) never spell "offset" out at all,
//!   so this was a label-only typo in the driver's own tag, not a DG645
//!   wire command string — and no db template in this workspace referenced
//!   the old spelling. Corrected to `T0_OFFSET_STEP`.
//!
//! # Preserved upstream quirks (not "fixed")
//! - `cvtErrorCode`'s dead branch (`if(!check_errors) *outBuf=-1;` with no
//!   `return`, unconditionally overwritten by `*outBuf=pport->error` on
//!   the next line): STATUS_CODE always reads back `error`, regardless of
//!   check_errors state. See [`Dg645Driver::error`] usage in `read_int32`.
//! - `cvtErrorText`'s typo `"Status ckecking disabled"` (sic) — see
//!   [`status_text`].

use epics_rs::asyn::error::{AsynError, AsynResult, AsynStatus};
use epics_rs::asyn::param::ParamType;
use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::sync_io::SyncIOHandle;
use epics_rs::asyn::user::AsynUser;

use crate::wire::{atof, atoi, write_only, write_read};

/// C `commandTable[].readConv` — how a reply (or cached state) becomes the
/// value returned to device support.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Conv {
    /// `cvtSink`: no meaningful value; write-only tag.
    Sink,
    /// `cvtIdent`: `*IDN?` reply, chopped after the first comma and
    /// prefixed `"SRS "`.
    Ident,
    /// `cvtErrorCode`: cached `error`, regardless of `check_errors`.
    ErrorCode,
    /// `cvtErrorText`: cached `error` resolved through `STATUS_MSG`, or
    /// the disabled-checking text.
    ErrorText,
    /// `cvtStrInt`: reply parsed with `atoi`.
    StrInt,
    /// `cvtStrFloat`: reply parsed with `atof`.
    StrFloat,
    /// `cvtCopyText`: reply copied through unchanged.
    CopyText,
    /// `cvtChanRef`: `"chan,delay"` reply, channel half.
    ChanRef,
    /// `cvtChanDelay`: `"chan,delay"` reply, delay half (as text or f64
    /// depending on which asyn interface is reading).
    ChanDelay,
}

/// `%e`-style float formats used across [`COMMANDS`] (C printf specifiers
/// transcribed from `writeCommand`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum FloatFmt {
    /// `%-.2f`
    F2,
    /// `%-.6f`
    F6,
    /// `%e`
    E,
}

/// C `commandTable[].writeFunc` — how a written value becomes a wire
/// command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum WriteKind {
    /// `writeSink`: read-only tag, write is a no-op success.
    Sink,
    /// `writeIntParam`: `sprintf(writeCommand, value)`, one `%d`.
    Int,
    /// `writeStrParam`: `sprintf(writeCommand, value)`, one `%s`.
    Str,
    /// `writeFloatParam`: `sprintf(writeCommand, value)`, one float spec.
    Float(FloatFmt),
    /// `writeCommandOnly`: `writeCommand` sent verbatim, value ignored.
    CommandOnly,
    /// `writeChannelRef`: read `readCommand` first for `(chan, delay)`,
    /// then write `writeCommand` with the new channel and the old delay.
    ChannelRef,
    /// `writeChannelDelay`: read `readCommand` first for `(chan, delay)`,
    /// then write `writeCommand` with the old channel and the new delay.
    ChannelDelay,
    /// `statusChecking`: toggles `check_errors`; not a wire command.
    StatusChecking,
}

struct CommandSpec {
    tag: &'static str,
    read_cmd: &'static str,
    write_cmd: &'static str,
    conv: Conv,
    write_kind: WriteKind,
}

const fn cmd(
    tag: &'static str,
    read_cmd: &'static str,
    write_cmd: &'static str,
    conv: Conv,
    write_kind: WriteKind,
) -> CommandSpec {
    CommandSpec {
        tag,
        read_cmd,
        write_cmd,
        conv,
        write_kind,
    }
}

// Transcribed verbatim (tag/readCommand/writeCommand/order) from
// `commandTable[]` in `drvAsynDG645.cpp:300-481`. Section comments mirror
// the C source's own section comments for direct auditability.
#[rustfmt::skip]
static COMMANDS: &[CommandSpec] = &[
    // Failure-mode -- has to be first
    cmd("NONE", "", "", Conv::Sink, WriteKind::Sink),

    // Instrument management related commands
    cmd("IDENT", "*IDN?", "", Conv::Ident, WriteKind::Sink),
    cmd("STATUS_CODE", "", "", Conv::ErrorCode, WriteKind::Sink),
    cmd("STATUS", "", "", Conv::ErrorText, WriteKind::Sink),
    cmd("STATUS_CLEAR", "", "*CLS", Conv::Sink, WriteKind::CommandOnly),
    cmd("STATUS_CHECKING", "", "", Conv::Sink, WriteKind::StatusChecking),
    cmd("RESET", "", "*RST", Conv::Sink, WriteKind::CommandOnly),
    cmd("LOCAL", "", "LCAL", Conv::Sink, WriteKind::CommandOnly),
    cmd("REMOTE", "", "REMT", Conv::Sink, WriteKind::CommandOnly),
    cmd("RECALL", "", "*RCL%d", Conv::Sink, WriteKind::Int),
    cmd("SAVE", "", "*SAV%d", Conv::Sink, WriteKind::Int),

    // Instrument event status related commands
    cmd("EVENT_STATUS", "INSR?", "", Conv::StrInt, WriteKind::Sink),

    // Trigger related commands
    cmd("TRIG_LEVEL", "TLVL?", "TLVL%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("TRIG_RATE", "TRAT?", "TRAT%-.6f", Conv::StrFloat, WriteKind::Float(FloatFmt::F6)),
    cmd("TRIG_SOURCE", "TSRC?", "TSRC%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_INHIBIT", "INHB?", "INHB%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_DELAY", "", "*TRG", Conv::Sink, WriteKind::CommandOnly),

    // Advanced trigger related commands
    cmd("TRIG_ADV_MODE", "ADVT?", "ADVT%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_HOLDOFF", "HOLD?", "HOLD%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),
    cmd("TRIG_PRESCALE", "PRES?0", "PRES 0,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_AB_PRESCALE", "PRES?1", "PRES 1,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_CD_PRESCALE", "PRES?2", "PRES 2,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_EF_PRESCALE", "PRES?3", "PRES 3,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_GH_PRESCALE", "PRES?4", "PRES 4,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_AB_PHASE", "PHAS?1", "PHAS 1,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_CD_PHASE", "PHAS?2", "PHAS 2,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_EF_PHASE", "PHAS?3", "PHAS 3,%d", Conv::StrInt, WriteKind::Int),
    cmd("TRIG_GH_PHASE", "PHAS?4", "PHAS 4,%d", Conv::StrInt, WriteKind::Int),

    // Burst mode related commands
    cmd("BURST_MODE", "BURM?", "BURM%d", Conv::StrInt, WriteKind::Int),
    cmd("BURST_COUNT", "BURC?", "BURC%d", Conv::StrInt, WriteKind::Int),
    cmd("BURST_T0", "BURT?", "BURT%d", Conv::StrInt, WriteKind::Int),
    cmd("BURST_DELAY", "BURD?", "BURD%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),
    cmd("BURST_PERIOD", "BURP?", "BURP%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Interface configuration related commands
    cmd("RS232", "IFCF?0", "IFCF 0,%d", Conv::StrInt, WriteKind::Int),
    cmd("RS232_BAUD", "IFCF?1", "IFCF 1,%d", Conv::StrInt, WriteKind::Int),
    cmd("GPIB", "IFCF?2", "IFCF 2,%d", Conv::StrInt, WriteKind::Int),
    cmd("GPIB_ADDRESS", "IFCF?3", "IFCF 3,%d", Conv::StrInt, WriteKind::Int),
    cmd("TCPIP", "IFCF?4", "IFCF 4,%d", Conv::StrInt, WriteKind::Int),
    cmd("DHCP", "IFCF?5", "IFCF 5,%d", Conv::StrInt, WriteKind::Int),
    cmd("AUTO_IP", "IFCF?6", "IFCF 6,%d", Conv::StrInt, WriteKind::Int),
    cmd("STATIC_IP", "IFCF?7", "IFCF 7,%d", Conv::StrInt, WriteKind::Int),
    cmd("BARE_SOCKET", "IFCF?8", "IFCF 8,%d", Conv::StrInt, WriteKind::Int),
    cmd("TELNET", "IFCF?9", "IFCF 9,%d", Conv::StrInt, WriteKind::Int),
    cmd("VXI11", "IFCF?10", "IFCF 10,%d", Conv::StrInt, WriteKind::Int),
    cmd("IP_ADDRESS", "IFCF?11", "IFCF 11,%s", Conv::CopyText, WriteKind::Str),
    cmd("NET_MASK", "IFCF?12", "IFCF 12,%s", Conv::CopyText, WriteKind::Str),
    cmd("GATEWAY", "IFCF?13", "IFCF 13,%s", Conv::CopyText, WriteKind::Str),
    cmd("RS232_RESET", "", "IFRS 0", Conv::Sink, WriteKind::CommandOnly),
    cmd("GPIB_RESET", "", "IFRS 1", Conv::Sink, WriteKind::CommandOnly),
    cmd("TCPIP_RESET", "", "IFRS 2", Conv::Sink, WriteKind::CommandOnly),
    cmd("MAC_ADDRESS", "EMAC?", "", Conv::CopyText, WriteKind::Sink),

    // Delay channel A related commands
    cmd("A_REF", "DLAY?2", "DLAY 2,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("A_DELAY", "DLAY?2", "DLAY 2,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("A_DELAY_STEP_NEG", "", "SPDL 2,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("A_DELAY_STEP_POS", "", "SPDL 2,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("A_DELAY_STEP", "SSDL?2", "SSDL 2,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel B related commands
    cmd("B_REF", "DLAY?3", "DLAY 3,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("B_DELAY", "DLAY?3", "DLAY 3,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("B_DELAY_STEP_NEG", "", "SPDL 3,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("B_DELAY_STEP_POS", "", "SPDL 3,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("B_DELAY_STEP", "SSDL?3", "SSDL 3,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel C related commands
    cmd("C_REF", "DLAY?4", "DLAY 4,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("C_DELAY", "DLAY?4", "DLAY 4,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("C_DELAY_STEP_NEG", "", "SPDL 4,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("C_DELAY_STEP_POS", "", "SPDL 4,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("C_DELAY_STEP", "SSDL?4", "SSDL 4,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel D related commands
    cmd("D_REF", "DLAY?5", "DLAY 5,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("D_DELAY", "DLAY?5", "DLAY 5,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("D_DELAY_STEP_NEG", "", "SPDL 5,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("D_DELAY_STEP_POS", "", "SPDL 5,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("D_DELAY_STEP", "SSDL?5", "SSDL 5,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel E related commands
    cmd("E_REF", "DLAY?6", "DLAY 6,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("E_DELAY", "DLAY?6", "DLAY 6,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("E_DELAY_STEP_NEG", "", "SPDL 6,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("E_DELAY_STEP_POS", "", "SPDL 6,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("E_DELAY_STEP", "SSDL?6", "SSDL 6,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel F related commands
    cmd("F_REF", "DLAY?7", "DLAY 7,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("F_DELAY", "DLAY?7", "DLAY 7,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("F_DELAY_STEP_NEG", "", "SPDL 7,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("F_DELAY_STEP_POS", "", "SPDL 7,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("F_DELAY_STEP", "SSDL?7", "SSDL 7,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel G related commands
    cmd("G_REF", "DLAY?8", "DLAY 8,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("G_DELAY", "DLAY?8", "DLAY 8,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("G_DELAY_STEP_NEG", "", "SPDL 8,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("G_DELAY_STEP_POS", "", "SPDL 8,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("G_DELAY_STEP", "SSDL?8", "SSDL 8,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // Delay channel H related commands
    cmd("H_REF", "DLAY?9", "DLAY 9,%d,%e", Conv::ChanRef, WriteKind::ChannelRef),
    cmd("H_DELAY", "DLAY?9", "DLAY 9,%d,%e", Conv::ChanDelay, WriteKind::ChannelDelay),
    cmd("H_DELAY_STEP_NEG", "", "SPDL 9,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("H_DELAY_STEP_POS", "", "SPDL 9,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("H_DELAY_STEP", "SSDL?9", "SSDL 9,%e", Conv::StrFloat, WriteKind::Float(FloatFmt::E)),

    // T0 output commands
    cmd("T0_AMP", "LAMP?0", "LAMP 0,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("T0_OFFSET", "LOFF?0", "LOFF 0,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("T0_POLARITY", "LPOL?0", "LPOL 0,%d", Conv::StrInt, WriteKind::Int),
    cmd("T0_AMP_STEP_NEG", "", "SPLA 0,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("T0_AMP_STEP_POS", "", "SPLA 0,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("T0_AMP_STEP", "SSLA?0", "SSLA 0,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("T0_OFFSET_STEP_NEG", "", "SPLO 0,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("T0_OFFSET_STEP_POS", "", "SPLO 0,1", Conv::Sink, WriteKind::CommandOnly),
    // Fixed upstream defect (doc/upstream-c-defects.md #33): upstream's tag
    // was "T0_OFSET_STEP" (missing the second F, drvAsynDG645.cpp:436).
    // The wire commands themselves ("SSLO?0"/"SSLO 0,%-.2f") never spell
    // out "offset" at all -- this is a label-only typo in the driver's own
    // internal tag, not a DG645 wire command string, and no db template in
    // this workspace references the old spelling. Corrected to match every
    // other output's "*_OFFSET_STEP" naming.
    cmd("T0_OFFSET_STEP", "SSLO?0", "SSLO 0,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),

    // AB output commands
    cmd("AB_AMP", "LAMP?1", "LAMP 1,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("AB_OFFSET", "LOFF?1", "LOFF 1,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("AB_POLARITY", "LPOL?1", "LPOL 1,%d", Conv::StrInt, WriteKind::Int),
    cmd("AB_AMP_STEP_NEG", "", "SPLA 1,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("AB_AMP_STEP_POS", "", "SPLA 1,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("AB_AMP_STEP", "SSLA?1", "SSLA 1,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("AB_OFFSET_STEP_NEG", "", "SPLO 1,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("AB_OFFSET_STEP_POS", "", "SPLO 1,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("AB_OFFSET_STEP", "SSLO?1", "SSLO 1,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),

    // CD output commands
    cmd("CD_AMP", "LAMP?2", "LAMP 2,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("CD_OFFSET", "LOFF?2", "LOFF 2,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("CD_POLARITY", "LPOL?2", "LPOL 2,%d", Conv::StrInt, WriteKind::Int),
    cmd("CD_AMP_STEP_NEG", "", "SPLA 2,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("CD_AMP_STEP_POS", "", "SPLA 2,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("CD_AMP_STEP", "SSLA?2", "SSLA 2,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("CD_OFFSET_STEP_NEG", "", "SPLO 2,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("CD_OFFSET_STEP_POS", "", "SPLO 2,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("CD_OFFSET_STEP", "SSLO?2", "SSLO 2,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),

    // EF output commands
    cmd("EF_AMP", "LAMP?3", "LAMP 3,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("EF_OFFSET", "LOFF?3", "LOFF 3,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("EF_POLARITY", "LPOL?3", "LPOL 3,%d", Conv::StrInt, WriteKind::Int),
    cmd("EF_AMP_STEP_NEG", "", "SPLA 3,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("EF_AMP_STEP_POS", "", "SPLA 3,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("EF_AMP_STEP", "SSLA?3", "SSLA 3,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("EF_OFFSET_STEP_NEG", "", "SPLO 3,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("EF_OFFSET_STEP_POS", "", "SPLO 3,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("EF_OFFSET_STEP", "SSLO?3", "SSLO 3,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),

    // GH output commands
    cmd("GH_AMP", "LAMP?4", "LAMP 4,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("GH_OFFSET", "LOFF?4", "LOFF 4,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("GH_POLARITY", "LPOL?4", "LPOL 4,%d", Conv::StrInt, WriteKind::Int),
    // Fixed upstream defect (doc/upstream-c-defects.md #32,
    // drvAsynDG645.cpp:475-479): the C table's tag<->wire-value mapping was
    // swapped for GH only (POS bound to wire arg 0, NEG to arg 1), while its
    // own trailing "minus"/"plus" comments still read arg 0 = minus, arg 1 =
    // plus -- the same convention every other output (T0/AB/CD/EF) uses.
    // The SPLA/SPLO wire command's argument value has one fixed meaning
    // across every channel; this was a copy-paste bug in the driver's own
    // table, not a DG645-defined per-channel difference. Corrected to match
    // the T0/AB/CD/EF convention (NEG->0, POS->1); see
    // `commands_use_consistent_step_polarity_convention`.
    cmd("GH_AMP_STEP_NEG", "", "SPLA 4,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("GH_AMP_STEP_POS", "", "SPLA 4,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("GH_AMP_STEP", "SSLA?4", "SSLA 4,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
    cmd("GH_OFFSET_STEP_NEG", "", "SPLO 4,0", Conv::Sink, WriteKind::CommandOnly),
    cmd("GH_OFFSET_STEP_POS", "", "SPLO 4,1", Conv::Sink, WriteKind::CommandOnly),
    cmd("GH_OFFSET_STEP", "SSLO?4", "SSLO 4,%-.2f", Conv::StrFloat, WriteKind::Float(FloatFmt::F2)),
];

/// C `delayText[]` (`cvtChanDelay`): DG645 channel index -> short name.
const CHAN_NAMES: [&str; 10] = ["T0", "T1", "A", "B", "C", "D", "E", "F", "G", "H"];

/// C `statusMsg[]` (`drvAsynDG645.cpp:224-297`), transcribed verbatim.
#[rustfmt::skip]
static STATUS_MSG: &[(i32, &str)] = &[
    (0, "STATUS OK"),
    (10, "Illegal Value"), (11, "Illegal Mode"), (12, "Illegal Delay"),
    (13, "Illegal Link"), (14, "Recall Failed"), (15, "Not Allowed"),
    (16, "Failed Self Test"), (17, "Failed Auto Calibration"),
    (30, "Lost Data"), (32, "No Listener"),
    (40, "Failed ROM Check"), (41, "Failed Offset T0 Test"),
    (42, "Failed Offset AB Test"), (43, "Failed Offset CD Test"),
    (44, "Failed Offset EF Test"), (45, "Failed Offset GH Test"),
    (46, "Failed Amplitude T0 Test"), (47, "Failed Amplitude AB Test"),
    (48, "Failed Amplitude CD Test"), (49, "Failed Amplitude EF Test"),
    (50, "Failed Amplitude GH Test"), (51, "Failed FPGA Communications Test"),
    (52, "Failed GPIB Communications Test"), (53, "Failed DDS Communications Test"),
    (54, "Failed Serial EEPROM Communications Test"),
    (55, "Failed Temperature Sensor Communications Test"),
    (56, "Failed PLL Communications Test"), (57, "Failed DAC 0 Communications Test"),
    (58, "Failed DAC 1 Communications Test"), (59, "Failed DAC 2 Communications Test"),
    (60, "Failed Sample and Hold Operations Test"), (61, "Failed Vjitter Operations Test"),
    (62, "Failed Channel T0 Analog Delay Test"), (63, "Failed Channel T1 Analog Delay Test"),
    (64, "Failed Channel A Analog Delay Test"), (65, "Failed Channel B Analog Delay Test"),
    (66, "Failed Channel C Analog Delay Test"), (67, "Failed Channel D Analog Delay Test"),
    (68, "Failed Channel E Analog Delay Test"), (69, "Failed Channel F Analog Delay Test"),
    (70, "Failed Channel G Analog Delay Test"), (71, "Failed Channel H Analog Delay Test"),
    (80, "Failed Sample and Hold Calibration"), (81, "Failed T0 Calibration"),
    (82, "Failed T1 Calibration"), (83, "Failed A Calibration"),
    (84, "Failed B Calibration"), (85, "Failed C Calibration"),
    (86, "Failed D Calibration"), (87, "Failed E Calibration"),
    (88, "Failed F Calibration"), (89, "Failed G Calibration"),
    (90, "Failed H Calibration"), (91, "Failed Vjitter Calibration"),
    (110, "Illegal Command"), (111, "Undefined Comand"), (112, "Illegal Query"),
    (113, "Illegal Set"), (114, "Null Parameter"), (115, "Extra Parameters"),
    (116, "Missing Parameters"), (117, "Parameter Overflow"),
    (118, "Invalid Floating Point Number"), (120, "Invalid Integer"),
    (121, "Integer Overflow"), (122, "Invalid Hexidecimal"), (126, "Syntax Error"),
    (170, "Communication Error"), (171, "Over run"), (254, "Too Many Errors"),
];

/// Mimic C `printf("%e", value)`: one leading digit, 6-digit mantissa,
/// signed at-least-2-digit exponent (e.g. `1.234568e+03`, `0.000000e+00`).
/// Rust's `{:.6e}` gives the correctly-rounded mantissa (`1.234568e3`);
/// only the exponent needs reformatting to match C's convention.
fn fmt_e(value: f64) -> String {
    let s = format!("{value:.6e}");
    let (mantissa, exp) = s
        .split_once('e')
        .expect("format!(\"{:e}\") always emits 'e'");
    let exp_n: i32 = exp.parse().expect("exponent is always a valid integer");
    format!(
        "{mantissa}e{}{:02}",
        if exp_n < 0 { "-" } else { "+" },
        exp_n.abs()
    )
}

fn format_int_cmd(template: &str, value: i32) -> String {
    template.replacen("%d", &value.to_string(), 1)
}

fn format_float_cmd(template: &str, fmt: FloatFmt, value: f64) -> String {
    match fmt {
        FloatFmt::F2 => template.replacen("%-.2f", &format!("{value:.2}"), 1),
        FloatFmt::F6 => template.replacen("%-.6f", &format!("{value:.6}"), 1),
        FloatFmt::E => template.replacen("%e", &fmt_e(value), 1),
    }
}

fn format_str_cmd(template: &str, value: &str) -> String {
    template.replacen("%s", value, 1)
}

/// C `writeChannelRef`: `sprintf(writeCommand, new_chan, old_delay)`.
fn format_channel_ref_cmd(template: &str, new_chan: i32, old_delay: f64) -> String {
    let s = template.replacen("%d", &new_chan.to_string(), 1);
    s.replacen("%e", &fmt_e(old_delay), 1)
}

/// C `writeChannelDelay`: `sprintf(writeCommand, chan, new_delay)`.
fn format_channel_delay_cmd(template: &str, chan: i32, new_delay: f64) -> String {
    let s = template.replacen("%d", &chan.to_string(), 1);
    s.replacen("%e", &fmt_e(new_delay), 1)
}

/// C `sscanf(inpBuf, "%d,%lf", &chan, &delay)`. Falls back to `(atoi, 0.0)`
/// if there is no comma, matching the fact that C `sscanf` simply leaves
/// `delay` unfilled (there is no realistic reply from a live DG645 that
/// takes this path).
fn parse_chan_delay_pair(reply: &str) -> (i32, f64) {
    match reply.split_once(',') {
        Some((c, d)) => (atoi(c), atof(d)),
        None => (atoi(reply), 0.0),
    }
}

/// C `cvtIdent`.
fn cvt_ident(reply: &str) -> String {
    match reply.find(',') {
        Some(i) => format!("SRS {}", &reply[i + 1..]),
        None => reply.to_string(),
    }
}

/// C `cvtChanDelay`'s `Octet` branch: `sprintf(outBuf, "%s + %-.12f", ...)`.
fn chan_delay_text(reply: &str) -> String {
    let (chan, delay) = parse_chan_delay_pair(reply);
    let name = CHAN_NAMES.get(chan as usize).copied().unwrap_or("?");
    format!("{name} + {delay:.12}")
}

/// C `cvtErrorText`. Preserves the upstream typo `"ckecking"` verbatim.
fn status_text(error: i32, check_errors: bool) -> &'static str {
    if !check_errors {
        return "Status ckecking disabled";
    }
    STATUS_MSG
        .iter()
        .find(|(code, _)| *code == error)
        .map(|(_, msg)| *msg)
        .unwrap_or("Unknown Error")
}

/// SRS DG645 asyn port driver (single-device: `drvAsynDG645(myport,ioport,ioaddr)`
/// registers with `ASYN_CANBLOCK` only, no `ASYN_MULTIDEVICE`).
pub struct Dg645Driver {
    base: PortDriverBase,
    handle: SyncIOHandle,
    error: i32,
    check_errors: bool,
    status_code_reason: usize,
}

impl Dg645Driver {
    /// C `drvAsynDG645`: registers every tag, then runs the same
    /// three-step init sequence (`*IDN?`, `LERR?`, `*CLS`).
    pub fn new(port_name: &str, handle: SyncIOHandle) -> AsynResult<Self> {
        let mut base = PortDriverBase::new(
            port_name,
            1,
            PortFlags {
                multi_device: false,
                can_block: true,
                destructible: true,
            },
        );
        for spec in COMMANDS {
            // The stored ParamType is never read back through a typed
            // getter for any tag but STATUS_CODE — every other tag exists
            // purely so `find_param`/the default `drv_user_create` can
            // resolve it to a reason index, mirroring C's `create()`
            // tag-table scan.
            base.create_param(spec.tag, ParamType::Int32)?;
        }
        let status_code_reason = base
            .find_param("STATUS_CODE")
            .expect("STATUS_CODE is registered above");

        let mut driver = Self {
            base,
            handle,
            error: 0,
            check_errors: true,
            status_code_reason,
        };

        write_read(&driver.handle, "*IDN?")?;
        let lerr = write_read(&driver.handle, "LERR?")?;
        driver.error = atoi(&lerr);
        write_only(&driver.handle, "*CLS")?;

        Ok(driver)
    }

    fn spec(&self, reason: usize) -> AsynResult<&'static CommandSpec> {
        COMMANDS
            .get(reason)
            .ok_or(AsynError::ParamIndexOutOfRange(reason))
    }

    /// C `checkError`: re-queries `LERR?`; `call_param_callbacks` only
    /// fires the STATUS_CODE I/O Intr subscribers when the cached value
    /// actually changes, replicating C's explicit
    /// `if(old_error != pport->error) signalStatusUpdate(pport)` gate.
    fn check_error(&mut self) -> AsynResult<()> {
        let reply = write_read(&self.handle, "LERR?")?;
        self.error = atoi(&reply);
        self.base
            .set_int32_param(self.status_code_reason, 0, self.error)?;
        self.base.call_param_callbacks(0)?;
        Ok(())
    }

    /// C's shared write epilogue: `if (ASYN_ERROR(status) || !check_errors)
    /// return status; return checkError(pport);`
    fn after_write(&mut self, status: AsynResult<()>) -> AsynResult<()> {
        status?;
        if self.check_errors {
            self.check_error()?;
        }
        Ok(())
    }

    /// C `writeChannelRef`.
    fn write_channel_ref(&mut self, spec: &CommandSpec, new_chan: i32) -> AsynResult<()> {
        let reply = write_read(&self.handle, spec.read_cmd)?;
        let (_old_chan, old_delay) = parse_chan_delay_pair(&reply);
        let wire = format_channel_ref_cmd(spec.write_cmd, new_chan, old_delay);
        let status = write_only(&self.handle, &wire);
        self.after_write(status)
    }

    /// C `writeChannelDelay`.
    fn write_channel_delay(&mut self, spec: &CommandSpec, new_delay: f64) -> AsynResult<()> {
        let reply = write_read(&self.handle, spec.read_cmd)?;
        let (chan, _old_delay) = parse_chan_delay_pair(&reply);
        let wire = format_channel_delay_cmd(spec.write_cmd, chan, new_delay);
        let status = write_only(&self.handle, &wire);
        self.after_write(status)
    }

    /// C `statusChecking`.
    fn write_status_checking(&mut self, value: i32) -> AsynResult<()> {
        if !(0..=1).contains(&value) {
            return Err(AsynError::Status {
                status: AsynStatus::Error,
                message: "STATUS_CHECKING must be 0 or 1".into(),
            });
        }
        let new_enabled = value != 0;
        if new_enabled == self.check_errors {
            return Ok(());
        }
        if !new_enabled {
            self.check_errors = false;
            // "to force it to change later" (C comment, drvAsynDG645.cpp:927)
            self.error = -1;
            self.base
                .set_int32_param(self.status_code_reason, 0, self.error)?;
            self.base.call_param_callbacks(0)?;
            Ok(())
        } else {
            write_only(&self.handle, "*CLS")?;
            self.check_errors = true;
            self.check_error()
        }
    }
}

impl PortDriver for Dg645Driver {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }

    fn read_int32(&mut self, user: &AsynUser) -> AsynResult<i32> {
        let spec = self.spec(user.reason)?;
        match spec.conv {
            Conv::Sink => Ok(0),
            // Dead branch preserved: C's cvtErrorCode sets *outBuf=-1 when
            // !check_errors but then unconditionally overwrites it with
            // pport->error on the next line — STATUS_CODE always reads
            // back `error`, regardless of check_errors.
            Conv::ErrorCode => Ok(self.error),
            Conv::StrInt => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                Ok(atoi(&reply))
            }
            Conv::ChanRef => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                Ok(parse_chan_delay_pair(&reply).0)
            }
            _ => Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        }
    }

    fn write_int32(&mut self, user: &mut AsynUser, value: i32) -> AsynResult<()> {
        let spec = self.spec(user.reason)?;
        match spec.write_kind {
            WriteKind::Sink => Ok(()),
            WriteKind::Int => {
                let wire = format_int_cmd(spec.write_cmd, value);
                let status = write_only(&self.handle, &wire);
                self.after_write(status)
            }
            WriteKind::CommandOnly => {
                let status = write_only(&self.handle, spec.write_cmd);
                self.after_write(status)
            }
            WriteKind::ChannelRef => self.write_channel_ref(spec, value),
            WriteKind::StatusChecking => self.write_status_checking(value),
            _ => Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        }
    }

    fn read_float64(&mut self, user: &AsynUser) -> AsynResult<f64> {
        let spec = self.spec(user.reason)?;
        match spec.conv {
            Conv::Sink => Ok(0.0),
            Conv::StrFloat => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                Ok(atof(&reply))
            }
            Conv::ChanDelay => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                Ok(parse_chan_delay_pair(&reply).1)
            }
            _ => Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        }
    }

    fn write_float64(&mut self, user: &mut AsynUser, value: f64) -> AsynResult<()> {
        let spec = self.spec(user.reason)?;
        match spec.write_kind {
            WriteKind::Sink => Ok(()),
            WriteKind::Float(fmt) => {
                let wire = format_float_cmd(spec.write_cmd, fmt, value);
                let status = write_only(&self.handle, &wire);
                self.after_write(status)
            }
            WriteKind::ChannelDelay => self.write_channel_delay(spec, value),
            _ => Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        }
    }

    fn read_octet(&mut self, user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        let spec = self.spec(user.reason)?;
        let text = match spec.conv {
            Conv::Sink => String::new(),
            Conv::Ident => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                cvt_ident(&reply)
            }
            Conv::ErrorText => status_text(self.error, self.check_errors).to_string(),
            Conv::CopyText => write_read(&self.handle, spec.read_cmd)?,
            Conv::ChanDelay => {
                let reply = write_read(&self.handle, spec.read_cmd)?;
                chan_delay_text(&reply)
            }
            _ => return Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        };
        let bytes = text.as_bytes();
        let n = bytes.len().min(buf.len());
        buf[..n].copy_from_slice(&bytes[..n]);
        Ok(n)
    }

    fn write_octet(&mut self, user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        let spec = self.spec(user.reason)?;
        match spec.write_kind {
            WriteKind::Sink => Ok(0),
            WriteKind::Str => {
                // C treats `data` as a NUL-terminated C string (both for
                // building the outgoing command and for the `*nbytes`
                // returned to the caller via `strlen(data)`).
                let n = data.iter().position(|&b| b == 0).unwrap_or(data.len());
                let text = String::from_utf8_lossy(&data[..n]);
                let wire = format_str_cmd(spec.write_cmd, &text);
                let status = write_only(&self.handle, &wire);
                self.after_write(status)?;
                Ok(n)
            }
            _ => Err(AsynError::InterfaceNotSupported(spec.tag.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn command_table_tags_are_unique() {
        let mut seen = HashSet::new();
        for spec in COMMANDS {
            assert!(seen.insert(spec.tag), "duplicate tag: {}", spec.tag);
        }
        assert_eq!(
            COMMANDS.len(),
            136,
            "expected 136 rows (drvAsynDG645.cpp:300-481)"
        );
    }

    #[test]
    fn commands_use_consistent_step_polarity_convention() {
        // doc/upstream-c-defects.md #32: GH now follows the same
        // NEG->0/POS->1 wire-value convention as every other output.
        let get = |tag: &str| COMMANDS.iter().find(|c| c.tag == tag).unwrap();
        assert_eq!(get("GH_AMP_STEP_NEG").write_cmd, "SPLA 4,0");
        assert_eq!(get("GH_AMP_STEP_POS").write_cmd, "SPLA 4,1");
        assert_eq!(get("GH_OFFSET_STEP_NEG").write_cmd, "SPLO 4,0");
        assert_eq!(get("GH_OFFSET_STEP_POS").write_cmd, "SPLO 4,1");
        assert_eq!(get("T0_AMP_STEP_NEG").write_cmd, "SPLA 0,0");
        assert_eq!(get("T0_AMP_STEP_POS").write_cmd, "SPLA 0,1");
        assert_eq!(get("AB_OFFSET_STEP_NEG").write_cmd, "SPLO 1,0");
        assert_eq!(get("AB_OFFSET_STEP_POS").write_cmd, "SPLO 1,1");
    }

    #[test]
    fn command_table_fixes_ofset_typo() {
        // doc/upstream-c-defects.md #33: tag corrected to T0_OFFSET_STEP.
        assert!(!COMMANDS.iter().any(|c| c.tag == "T0_OFSET_STEP"));
        assert!(COMMANDS.iter().any(|c| c.tag == "T0_OFFSET_STEP"));
    }

    #[test]
    fn fmt_e_matches_c_printf_e() {
        assert_eq!(fmt_e(0.0), "0.000000e+00");
        assert_eq!(fmt_e(1234.5678), "1.234568e+03");
        assert_eq!(fmt_e(-0.0005), "-5.000000e-04");
        assert_eq!(fmt_e(1.0), "1.000000e+00");
        assert_eq!(fmt_e(9.9999999e2), "1.000000e+03"); // rounds up a digit
    }

    #[test]
    fn format_int_cmd_substitutes_single_placeholder() {
        assert_eq!(format_int_cmd("TSRC%d", 3), "TSRC3");
        assert_eq!(format_int_cmd("*RCL%d", 5), "*RCL5");
        assert_eq!(format_int_cmd("PRES 4,%d", -1), "PRES 4,-1");
    }

    #[test]
    fn format_float_cmd_matches_c_precision_per_row() {
        assert_eq!(format_float_cmd("TLVL%-.2f", FloatFmt::F2, 2.5), "TLVL2.50");
        assert_eq!(
            format_float_cmd("TRAT%-.6f", FloatFmt::F6, 10.0),
            "TRAT10.000000"
        );
        assert_eq!(
            format_float_cmd("HOLD%e", FloatFmt::E, 0.001),
            "HOLD1.000000e-03"
        );
    }

    #[test]
    fn format_str_cmd_substitutes_placeholder() {
        assert_eq!(
            format_str_cmd("IFCF 11,%s", "192.168.1.1"),
            "IFCF 11,192.168.1.1"
        );
    }

    #[test]
    fn format_channel_ref_and_delay_cmds_match_c_sprintf_arg_order() {
        // writeChannelRef: sprintf(writeCommand, new_chan, old_delay)
        assert_eq!(
            format_channel_ref_cmd("DLAY 2,%d,%e", 1, 1.5e-6),
            "DLAY 2,1,1.500000e-06"
        );
        // writeChannelDelay: sprintf(writeCommand, chan, new_delay)
        assert_eq!(
            format_channel_delay_cmd("DLAY 2,%d,%e", 2, 3.14159e-3),
            "DLAY 2,2,3.141590e-03"
        );
    }

    #[test]
    fn parse_chan_delay_pair_parses_sscanf_style_reply() {
        assert_eq!(parse_chan_delay_pair("2,1.2345e-06"), (2, 1.2345e-6));
        assert_eq!(parse_chan_delay_pair("9,-3.5"), (9, -3.5));
        assert_eq!(parse_chan_delay_pair("garbage"), (0, 0.0));
    }

    #[test]
    fn cvt_ident_chops_after_first_comma_and_prefixes_srs() {
        assert_eq!(
            cvt_ident("Stanford Research Systems,DG645,s/n12345,ver1.0"),
            "SRS DG645,s/n12345,ver1.0"
        );
        assert_eq!(cvt_ident("no comma here"), "no comma here");
    }

    #[test]
    fn chan_delay_text_matches_c_sprintf_format() {
        // chan 2 == "A" per delayText[]
        assert_eq!(chan_delay_text("2,0.000001"), "A + 0.000001000000");
        assert_eq!(chan_delay_text("0,0"), "T0 + 0.000000000000");
    }

    #[test]
    fn status_text_resolves_known_and_unknown_codes() {
        assert_eq!(status_text(0, true), "STATUS OK");
        assert_eq!(status_text(126, true), "Syntax Error");
        assert_eq!(status_text(254, true), "Too Many Errors");
        assert_eq!(status_text(9999, true), "Unknown Error");
    }

    #[test]
    fn status_text_preserves_disabled_checking_typo() {
        // sic: upstream cvtErrorText spells this "ckecking".
        assert_eq!(status_text(0, false), "Status ckecking disabled");
    }
}
