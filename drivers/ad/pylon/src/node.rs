//! Port of ADPylon `PylonFeature.cpp`: GenICam node access through pylon.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use ad_genicam::{GCFeatureType, GenICamNode, NodeResult};

use crate::camera::{Camera, NodeKind, NodeMap};
use crate::trace::Trace;

/// Transport-layer statistics, which live in the stream grabber's node map.
pub const TL_STATISTICS_FEATURE_NAMES: [&str; 7] = [
    "Statistic_Total_Buffer_Count",
    "Statistic_Failed_Buffer_Count",
    "Statistic_Buffer_Underrun_Count",
    "Statistic_Total_Packet_Count",
    "Statistic_Failed_Packet_Count",
    "Statistic_Resend_Request_Count",
    "Statistic_Resend_Packet_Count",
];

/// An mbbo/mbbi record holds at most this many choices.
const MAX_ENUM_CHOICES: usize = 16;

pub fn node_map_of(feature_name: &str) -> NodeMap {
    if TL_STATISTICS_FEATURE_NAMES.contains(&feature_name) {
        NodeMap::Stream
    } else {
        NodeMap::Camera
    }
}

fn feature_type_of(kind: NodeKind) -> GCFeatureType {
    match kind {
        NodeKind::Integer => GCFeatureType::Integer,
        NodeKind::Float => GCFeatureType::Double,
        NodeKind::Enum => GCFeatureType::Enum,
        NodeKind::String => GCFeatureType::String,
        NodeKind::Boolean => GCFeatureType::Boolean,
        NodeKind::Command => GCFeatureType::Cmd,
        NodeKind::Other => GCFeatureType::Unknown,
    }
}

/// The node is looked up by name on every access, so it follows the camera
/// through a close and reopen without being re-initialized.
pub struct PylonNode {
    camera: Arc<Camera>,
    map: NodeMap,
    name: String,
    declared: GCFeatureType,
    warned_choices: AtomicBool,
    trace: Trace,
}

impl PylonNode {
    pub(crate) fn new(
        camera: Arc<Camera>,
        feature_name: &str,
        declared: GCFeatureType,
        trace: Trace,
    ) -> Self {
        let node = Self {
            camera,
            map: node_map_of(feature_name),
            name: feature_name.to_string(),
            declared,
            warned_choices: AtomicBool::new(false),
            trace,
        };
        if let Some(actual) = node.actual_type() {
            if declared != GCFeatureType::Unknown && declared != actual {
                node.trace.error(&format!(
                    "ADPylon: input feature type={declared:?} != Pylon feature type={actual:?} \
                     for featurename={feature_name}"
                ));
            } else if actual == GCFeatureType::Unknown {
                node.trace.error(&format!(
                    "ADPylon: unknown feature type for featureName={feature_name}"
                ));
            }
        }
        node
    }

    fn actual_type(&self) -> Option<GCFeatureType> {
        self.camera.kind(self.map, &self.name).map(feature_type_of)
    }
}

impl GenICamNode for PylonNode {
    fn feature_type(&self) -> GCFeatureType {
        match self.declared {
            GCFeatureType::Unknown => self.actual_type().unwrap_or(GCFeatureType::Unknown),
            declared => declared,
        }
    }

    fn is_implemented(&self) -> bool {
        match self.actual_type() {
            None | Some(GCFeatureType::Unknown) => false,
            Some(actual) => self.declared == GCFeatureType::Unknown || self.declared == actual,
        }
    }

    fn is_available(&self) -> bool {
        self.is_implemented() && self.camera.access(self.map, &self.name).0
    }

    fn is_readable(&self) -> bool {
        self.is_implemented() && self.camera.access(self.map, &self.name).1
    }

    fn is_writable(&self) -> bool {
        self.is_implemented() && self.camera.access(self.map, &self.name).2
    }

    fn read_integer(&self) -> NodeResult<i64> {
        self.camera.get_int(self.map, &self.name)
    }

    fn read_integer_min(&self) -> NodeResult<i64> {
        Ok(self.camera.int_range(self.map, &self.name)?.0)
    }

    fn read_integer_max(&self) -> NodeResult<i64> {
        Ok(self.camera.int_range(self.map, &self.name)?.1)
    }

    fn read_increment(&self) -> NodeResult<i64> {
        Ok(self.camera.int_range(self.map, &self.name)?.2)
    }

    fn write_integer(&self, value: i64) -> NodeResult<()> {
        self.camera.set_int(self.map, &self.name, value)
    }

    fn read_boolean(&self) -> NodeResult<bool> {
        self.camera.get_bool(self.map, &self.name)
    }

    fn write_boolean(&self, value: bool) -> NodeResult<()> {
        self.camera.set_bool(self.map, &self.name, value)
    }

    fn read_double(&self) -> NodeResult<f64> {
        self.camera.get_float(self.map, &self.name)
    }

    fn read_double_min(&self) -> NodeResult<f64> {
        Ok(self.camera.float_range(self.map, &self.name)?.0)
    }

    fn read_double_max(&self) -> NodeResult<f64> {
        Ok(self.camera.float_range(self.map, &self.name)?.1)
    }

    fn write_double(&self, value: f64) -> NodeResult<()> {
        self.camera.set_float(self.map, &self.name, value)
    }

    fn read_enum_index(&self) -> NodeResult<i32> {
        Ok(self.camera.get_enum(self.map, &self.name)? as i32)
    }

    fn write_enum_index(&self, value: i32) -> NodeResult<()> {
        self.camera.set_enum(self.map, &self.name, value as i64)
    }

    fn read_enum_string(&self) -> NodeResult<String> {
        self.camera.get_enum_symbolic(self.map, &self.name)
    }

    fn read_enum_choices(&self) -> NodeResult<Vec<(String, i32)>> {
        let narrow = |entries: Vec<(String, i64)>| -> Vec<(String, i32)> {
            entries.into_iter().map(|(s, v)| (s, v as i32)).collect()
        };
        let all = self.camera.enum_entries(self.map, &self.name, false)?;
        if all.len() <= MAX_ENUM_CHOICES {
            return Ok(narrow(all));
        }
        // With more than 16 choices use the settable ones, which can be far
        // fewer: PixelFormat of a color camera lists every bayer pattern, but
        // only one set is valid for the current ReverseX and ReverseY.
        let settable = self.camera.enum_entries(self.map, &self.name, true)?;
        if !self.warned_choices.swap(true, Ordering::Relaxed) {
            self.trace.warning(&format!(
                "ADPylon: {} has more than {} choices. Only the settable choices will be used.",
                self.name,
                all.len()
            ));
            if settable.len() > MAX_ENUM_CHOICES {
                let lost: Vec<&str> = settable[MAX_ENUM_CHOICES..]
                    .iter()
                    .map(|(s, _)| s.as_str())
                    .collect();
                self.trace.warning(&format!(
                    "ADPylon: {} has {} settable choices, {} will be unreachable",
                    self.name,
                    settable.len(),
                    lost.join(" ")
                ));
            }
        }
        Ok(narrow(settable))
    }

    fn read_string(&self) -> NodeResult<String> {
        self.camera.get_string(self.map, &self.name)
    }

    fn write_string(&self, value: &str) -> NodeResult<()> {
        self.camera.set_string(self.map, &self.name, value)
    }

    fn write_command(&self) -> NodeResult<()> {
        self.camera.execute(self.map, &self.name)
    }
}
