//! Constants and enumerations of the PhotonII driver (C `PhotonII.cpp`,
//! `PhotonII.h`).

use std::time::Duration;

/// Detector width, in pixels (C `PII_SIZEX`).
pub const PII_SIZE_X: usize = 768;
/// Detector height, in pixels (C `PII_SIZEY`).
pub const PII_SIZE_Y: usize = 1024;
/// Bytes per pixel in a p2util `.raw` frame — `sizeof(epicsInt32)`.
pub const PII_PIXEL_BYTES: usize = 4;

/// Timeout on every command sent to p2util (C `PII_COMMAND_TIMEOUT`).
pub const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);
/// Poll interval while waiting for a frame message or for a raw file to appear
/// (C `PII_FILE_READ_DELAY`).
pub const FILE_READ_DELAY: Duration = Duration::from_millis(10);
/// Grace period on top of the exposure time before a missing frame message or
/// a missing raw file is called a timeout (C `PII_FILE_READ_TIMEOUT`).
pub const FILE_READ_TIMEOUT: Duration = Duration::from_secs(3);
/// Longest line p2util is allowed to send (C `PII_MAX_MESSAGE_SIZE`).
pub const MAX_MESSAGE_SIZE: usize = 512;

/// The p2util message that announces a finished frame carries this substring
/// followed by the quoted path of the file it wrote (C `PhotonIITask`).
pub const FILE_WRITTEN_MARKER: &str = "bytes to ";

/// `ADFrameType` choices, as redefined by `PhotonII.template`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Normal = 0,
    Dark = 1,
    Adc0 = 2,
}

impl FrameType {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Normal),
            1 => Some(Self::Dark),
            2 => Some(Self::Adc0),
            _ => None,
        }
    }
}

/// `PII_TRIGGER_TYPE`: the p2util `--frame-trigger-mode` argument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerType {
    Step = 0,
    Continuous = 1,
}

impl TriggerType {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Step),
            1 => Some(Self::Continuous),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Step => "step",
            Self::Continuous => "continuous",
        }
    }
}

/// `PII_TRIGGER_EDGE`: the p2util `--frame-trigger-edge` argument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerEdge {
    Rising = 0,
    Falling = 1,
}

impl TriggerEdge {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Rising),
            1 => Some(Self::Falling),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Rising => "rising",
            Self::Falling => "falling",
        }
    }
}

/// `ADTriggerMode`: the p2util `--frame-trigger-source` argument.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerSource {
    Internal = 0,
    External = 1,
}

impl TriggerSource {
    pub fn from_i32(v: i32) -> Option<Self> {
        match v {
            0 => Some(Self::Internal),
            1 => Some(Self::External),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Internal => "internal",
            Self::External => "external",
        }
    }
}

/// Driver parameter names, as the records use them.
pub const PII_DRSUM_ENABLE: &str = "PII_DRSUM_ENABLE";
pub const PII_NUM_DARKS: &str = "PII_NUM_DARKS";
pub const PII_TRIGGER_TYPE: &str = "PII_TRIGGER_TYPE";
pub const PII_TRIGGER_EDGE: &str = "PII_TRIGGER_EDGE";
pub const PII_NUM_SUBFRAMES: &str = "PII_NUM_SUBFRAMES";
/// Internal, record-less: the `p2util` iocsh command writes the command line
/// here so it is sent by the port actor, which owns the socket.
pub const PII_UTIL: &str = "PII_UTIL";
/// Internal, record-less: the acquisition task drives the EPICS shutter
/// through here, because only the actor may touch the parameter library.
pub const PII_SHUTTER: &str = "PII_SHUTTER";
