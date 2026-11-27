//! XPS gathering readback: parse `GatheringDataMultipleLinesGet` buffers into
//! per-axis actual positions and following errors.
//!
//! Port of the data path in C `XPSController::readbackProfile()`
//! (XPSController.cpp): during a PVT execution the controller samples
//! `SetpointPosition;CurrentPosition` per axis on every trajectory pulse
//! (`GatheringOneData` event). Afterwards the samples are read back over the
//! command socket as lines of `;`-separated doubles — one line per sample,
//! [`NUM_GATHERING_ITEMS`] values per axis. The readback keeps the actual
//! position and derives the following error `actual - setpoint`; positions are
//! device/positioner units.

use std::cell::Cell;

use epics_libcom_rs::runtime::scanf::sscanf;

/// Gathering values sampled per axis (`SetpointPosition` + `CurrentPosition`,
/// C `NUM_GATHERING_ITEMS`).
pub const NUM_GATHERING_ITEMS: usize = 2;

/// Parsed gathering samples, indexed `[axis][sample]`
/// (C `profileReadbacks_` / `profileFollowingErrors_`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GatheringReadback {
    pub actuals: Vec<Vec<f64>>,
    pub following_errors: Vec<Vec<f64>>,
}

impl GatheringReadback {
    /// Empty readback for `num_axes` axes.
    pub fn new(num_axes: usize) -> Self {
        Self {
            actuals: vec![Vec::new(); num_axes],
            following_errors: vec![Vec::new(); num_axes],
        }
    }

    /// Number of samples parsed so far.
    pub fn num_samples(&self) -> usize {
        self.actuals.first().map_or(0, Vec::len)
    }
}

/// Append the lines of one `GatheringDataMultipleLinesGet` buffer to
/// `readback`, scanning each axis as C does with `sscanf("%lf;%lf%n")` and
/// stepping one byte past it; a scan short of [`NUM_GATHERING_ITEMS`] fails
/// the readback. Returns the number of lines consumed.
pub fn parse_gathering_buffer(
    buffer: &str,
    num_axes: usize,
    readback: &mut GatheringReadback,
) -> Result<usize, String> {
    let mut lines_read = 0;
    for line in buffer.lines() {
        let line = line.trim_end_matches('\r');
        if line.is_empty() {
            continue;
        }
        let mut rest = line.as_bytes();
        let mut sample = Vec::with_capacity(num_axes);
        for _ in 0..num_axes {
            let [setpoint, actual] = [0.0; 2].map(Cell::new);
            let end = Cell::new(0i32);
            let nitems = sscanf(rest, "%lf;%lf%n", &[&setpoint, &actual, &end]);
            if nitems != NUM_GATHERING_ITEMS as i32 {
                return Err(format!(
                    "Error reading Gathering.dat file, nitems={nitems}, should be {NUM_GATHERING_ITEMS}"
                ));
            }
            // Past the line end C reads on into the next line; here nothing.
            rest = rest.get(end.get() as usize + 1..).unwrap_or_default();
            sample.push((setpoint.get(), actual.get()));
        }
        for (axis, (setpoint, actual)) in sample.into_iter().enumerate() {
            readback.actuals[axis].push(actual);
            readback.following_errors[axis].push(actual - setpoint);
        }
        lines_read += 1;
    }
    Ok(lines_read)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_two_axis_lines() {
        let mut rb = GatheringReadback::new(2);
        // Two samples: axis0 (sp;cp) then axis1 (sp;cp), ';'-separated.
        let buf = "1.0;1.5;10.0;10.25\n2.0;2.25;20.0;19.5\n";
        let n = parse_gathering_buffer(buf, 2, &mut rb).expect("parse");
        assert_eq!(n, 2);
        assert_eq!(rb.num_samples(), 2);
        assert_eq!(rb.actuals[0], vec![1.5, 2.25]);
        assert_eq!(rb.following_errors[0], vec![0.5, 0.25]);
        assert_eq!(rb.actuals[1], vec![10.25, 19.5]);
        assert_eq!(rb.following_errors[1], vec![0.25, -0.5]);
    }

    #[test]
    fn tolerates_trailing_separator_and_blank_lines() {
        let mut rb = GatheringReadback::new(1);
        let buf = "0.5;0.75;\n\n1.5;1.25;\n";
        let n = parse_gathering_buffer(buf, 1, &mut rb).expect("parse");
        assert_eq!(n, 2);
        assert_eq!(rb.actuals[0], vec![0.75, 1.25]);
    }

    #[test]
    fn appends_across_buffers() {
        let mut rb = GatheringReadback::new(1);
        parse_gathering_buffer("0.0;0.1\n", 1, &mut rb).expect("first");
        parse_gathering_buffer("1.0;1.1\n", 1, &mut rb).expect("second");
        assert_eq!(rb.num_samples(), 2);
        assert_eq!(rb.actuals[0], vec![0.1, 1.1]);
    }

    #[test]
    fn short_line_is_an_error() {
        // C scans past the line end, where the last line holds nothing.
        let mut rb = GatheringReadback::new(2);
        let err = parse_gathering_buffer("1.0;1.5\n", 2, &mut rb).unwrap_err();
        assert_eq!(
            err,
            "Error reading Gathering.dat file, nitems=-1, should be 2"
        );
    }

    /// XPSController.cpp:1270 scans each axis with `sscanf("%lf;%lf%n")` and
    /// steps one byte past it.
    #[test]
    fn axes_are_scanned_as_c_sscanf() {
        let parse = |buf: &str, axes| {
            let mut rb = GatheringReadback::new(axes);
            parse_gathering_buffer(buf, axes, &mut rb).map(|_| rb.actuals)
        };
        let nitems = |n: i32| {
            Err(format!(
                "Error reading Gathering.dat file, nitems={n}, should be 2"
            ))
        };
        assert_eq!(parse("1.0;1.5abc", 1), Ok(vec![vec![1.5]]));
        assert_eq!(parse("0x1p1;2", 1), Ok(vec![vec![2.0]]));
        assert_eq!(parse("1;2 3;4", 2), Ok(vec![vec![2.0], vec![4.0]]));
        assert_eq!(parse("1.0 ;1.5", 1), nitems(1));
        assert_eq!(parse("1.0;;1.5", 1), nitems(1));
        assert_eq!(parse("1;2;;3;4", 2), nitems(0));
        // C reads the second axis from the next line.
        assert_eq!(parse("1.0;1.5\n2.0;2.5", 2), nitems(-1));
    }

    #[test]
    fn junk_value_is_an_error() {
        let mut rb = GatheringReadback::new(1);
        assert_eq!(
            parse_gathering_buffer("1.0;abc\n", 1, &mut rb).unwrap_err(),
            "Error reading Gathering.dat file, nitems=1, should be 2"
        );
    }
}
