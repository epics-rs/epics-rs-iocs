/// Commands sent from the driver to the acquisition task.
pub enum AcqCommand {
    Start,
    Stop,
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
