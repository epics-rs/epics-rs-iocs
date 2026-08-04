/// Commands sent from the driver to the acquisition task.
pub enum AcqCommand {
    Start,
    Stop,
}

/// A stream mode (resolution + frame rate) available for both
/// Color (RGB8) and Depth (Z16), which is what the pipeline asks for.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct StreamMode {
    pub width: i32,
    pub height: i32,
    pub fps: i32,
}

impl StreamMode {
    /// The label the RSStreamMode enum shows for this mode.
    pub fn label(&self) -> String {
        format!("{}x{} @ {}fps", self.width, self.height, self.fps)
    }
}

/// The D435i's modes, used when the camera cannot be enumerated at startup so
/// the records still come up with a usable table.
pub const FALLBACK_STREAM_MODES: &[StreamMode] = &[
    StreamMode {
        width: 424,
        height: 240,
        fps: 15,
    }, //  0
    StreamMode {
        width: 424,
        height: 240,
        fps: 30,
    }, //  1
    StreamMode {
        width: 424,
        height: 240,
        fps: 60,
    }, //  2
    StreamMode {
        width: 640,
        height: 360,
        fps: 15,
    }, //  3
    StreamMode {
        width: 640,
        height: 360,
        fps: 30,
    }, //  4
    StreamMode {
        width: 640,
        height: 360,
        fps: 60,
    }, //  5
    StreamMode {
        width: 640,
        height: 480,
        fps: 15,
    }, //  6
    StreamMode {
        width: 640,
        height: 480,
        fps: 30,
    }, //  7  (default)
    StreamMode {
        width: 640,
        height: 480,
        fps: 60,
    }, //  8
    StreamMode {
        width: 848,
        height: 480,
        fps: 15,
    }, //  9
    StreamMode {
        width: 848,
        height: 480,
        fps: 30,
    }, // 10
    StreamMode {
        width: 848,
        height: 480,
        fps: 60,
    }, // 11
    StreamMode {
        width: 1280,
        height: 720,
        fps: 6,
    }, // 12
    StreamMode {
        width: 1280,
        height: 720,
        fps: 15,
    }, // 13
    StreamMode {
        width: 1280,
        height: 720,
        fps: 30,
    }, // 14
];

/// Preferred mode when the camera offers it: 640x480 @ 30fps.
pub const PREFERRED_MODE: StreamMode = StreamMode {
    width: 640,
    height: 480,
    fps: 30,
};

/// An mbbo/mbbi carries at most 16 states, so that is the most the enum can
/// offer. Cameras list far more combinations than that.
pub const MAX_STREAM_MODES: usize = 16;

/// Modes the camera at `serial` supports for Color(RGB8) and Depth(Z16) both.
///
/// RSStreamMode used to be a fixed table of what a D435i can do, which meant
/// offering a D405 modes it rejects (1280x720 @ 6fps) and hiding any it has
/// that the D435i does not. An empty `serial` means "first device found",
/// matching what the pipeline itself will open.
///
/// Falls back to [`FALLBACK_STREAM_MODES`] when the camera cannot be reached --
/// the IOC has to come up with a table either way, and the pipeline will report
/// the real problem when acquisition starts.
pub fn discover_stream_modes(serial: &str) -> Vec<StreamMode> {
    let Some(modes) = query_stream_modes(serial) else {
        log::warn!("D435i: camera not enumerable; offering the D435i's stream modes");
        return FALLBACK_STREAM_MODES.to_vec();
    };
    if modes.is_empty() {
        log::warn!("D435i: camera reported no colour+depth mode in common");
        return FALLBACK_STREAM_MODES.to_vec();
    }
    modes
}

fn query_stream_modes(serial: &str) -> Option<Vec<StreamMode>> {
    use realsense_rust::context::Context;
    use realsense_rust::kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind};
    use std::collections::HashSet;

    let ctx = Context::new().ok()?;
    let devices = ctx.query_devices(HashSet::new());
    let device = devices.iter().find(|d| {
        serial.is_empty()
            || d.info(Rs2CameraInfo::SerialNumber)
                .is_some_and(|s| s.to_string_lossy() == serial)
    })?;

    let mut colour: HashSet<(i32, i32, i32)> = HashSet::new();
    let mut depth: HashSet<(i32, i32, i32)> = HashSet::new();
    for sensor in device.sensors() {
        for profile in sensor.stream_profiles() {
            let target = match (profile.kind(), profile.format()) {
                (Rs2StreamKind::Color, Rs2Format::Rgb8) => &mut colour,
                (Rs2StreamKind::Depth, Rs2Format::Z16) => &mut depth,
                _ => continue,
            };
            if let Ok(intr) = profile.intrinsics() {
                target.insert((
                    intr.width() as i32,
                    intr.height() as i32,
                    profile.framerate(),
                ));
            }
        }
    }

    let mut modes: Vec<StreamMode> = colour
        .intersection(&depth)
        .map(|&(width, height, fps)| StreamMode { width, height, fps })
        .collect();
    // Ascending by pixel count then rate, so the enum reads in a sensible
    // order and the index is stable for a given camera.
    modes.sort_by_key(|m| (m.width * m.height, m.fps));
    Some(fit_to_enum(modes))
}

/// Trim `modes` to what an mbbo can hold, keeping every resolution.
///
/// Cameras offer more combinations than the record's 16 states, and simply
/// truncating the sorted list drops the largest frames -- the D435i loses
/// 1280x720 to four different rates at 424x240. Taking one rate from each
/// resolution in turn keeps every resolution reachable and spends what is left
/// on extra rates.
fn fit_to_enum(modes: Vec<StreamMode>) -> Vec<StreamMode> {
    if modes.len() <= MAX_STREAM_MODES {
        return modes;
    }
    let mut by_resolution: Vec<Vec<StreamMode>> = Vec::new();
    for mode in modes {
        match by_resolution
            .iter_mut()
            .find(|g| g[0].width == mode.width && g[0].height == mode.height)
        {
            Some(group) => group.push(mode),
            None => by_resolution.push(vec![mode]),
        }
    }

    let mut kept: Vec<StreamMode> = Vec::with_capacity(MAX_STREAM_MODES);
    let deepest = by_resolution.iter().map(Vec::len).max().unwrap_or(0);
    for rate in 0..deepest {
        for group in &by_resolution {
            if kept.len() == MAX_STREAM_MODES {
                break;
            }
            if let Some(mode) = group.get(rate) {
                kept.push(*mode);
            }
        }
    }
    kept.sort_by_key(|m| (m.width * m.height, m.fps));
    kept
}

/// Index of [`PREFERRED_MODE`] in `modes`, or the middle of the list.
pub fn default_mode_index(modes: &[StreamMode]) -> i32 {
    modes
        .iter()
        .position(|m| *m == PREFERRED_MODE)
        .unwrap_or(modes.len() / 2) as i32
}

/// Flags indicating which aspects of the pipeline need updating.
#[derive(Debug, Default)]
pub struct DirtyFlags {
    /// Resolution or FPS changed — pipeline must be restarted.
    pub reconfigure_pipeline: bool,
    /// Sensor option (exposure, gain, laser) changed — update in-place.
    pub update_sensor_options: bool,
}

impl DirtyFlags {
    pub fn any(&self) -> bool {
        self.reconfigure_pipeline || self.update_sensor_options
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn set_all(&mut self) {
        self.reconfigure_pipeline = true;
        self.update_sensor_options = true;
    }

    /// Take all flags (return current state and clear).
    pub fn take(&mut self) -> DirtyFlags {
        let taken = DirtyFlags {
            reconfigure_pipeline: self.reconfigure_pipeline,
            update_sensor_options: self.update_sensor_options,
        };
        self.clear();
        taken
    }
}
