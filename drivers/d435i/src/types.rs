/// Commands sent from the driver to the acquisition task.
pub enum AcqCommand {
    Start,
    Stop,
}

/// A valid stream mode (resolution + frame rate) supported by both
/// Color (RGB8) and Depth (Z16) sensors on the D435i.
pub struct StreamMode {
    pub width: i32,
    pub height: i32,
    pub fps: i32,
}

/// Valid stream modes: intersection of Color(RGB8) and Depth(Z16) capabilities.
/// Index 7 (640x480 @ 30fps) is the default.
pub const STREAM_MODES: &[StreamMode] = &[
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

pub const DEFAULT_STREAM_MODE: i32 = 7;

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
