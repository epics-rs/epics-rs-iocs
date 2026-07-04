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
//! device/positioner units, matching this driver's CSV profile convention.

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
/// `readback`, expecting [`NUM_GATHERING_ITEMS`] `;`-separated doubles per
/// axis on every line (C parses `"%lf;%lf"` per axis and fails the readback on
/// a short line). Returns the number of lines consumed.
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
        let values: Vec<f64> = line
            .split(';')
            .filter(|f| !f.trim().is_empty())
            .map(|f| {
                f.trim()
                    .parse::<f64>()
                    .map_err(|_| format!("bad gathering value '{f}'"))
            })
            .collect::<Result<_, _>>()?;
        if values.len() < num_axes * NUM_GATHERING_ITEMS {
            return Err(format!(
                "gathering line has {} values, expected {} ({} per axis x {num_axes} axes)",
                values.len(),
                num_axes * NUM_GATHERING_ITEMS,
                NUM_GATHERING_ITEMS,
            ));
        }
        for axis in 0..num_axes {
            let setpoint = values[axis * NUM_GATHERING_ITEMS];
            let actual = values[axis * NUM_GATHERING_ITEMS + 1];
            readback.actuals[axis].push(actual);
            readback.following_errors[axis].push(actual - setpoint);
        }
        lines_read += 1;
    }
    Ok(lines_read)
}

/// Render a readback as CSV: a `#` header naming the columns, then one row per
/// sample with `actual, following_error` per positioner — the file-based
/// counterpart of C posting `profileReadbacks_`/`profileFollowingErrors_` to
/// waveform records.
pub fn readback_csv(positioners: &[String], readback: &GatheringReadback) -> String {
    let mut out = String::from("# ");
    let header: Vec<String> = positioners
        .iter()
        .flat_map(|p| [format!("{p}.actual"), format!("{p}.following_error")])
        .collect();
    out.push_str(&header.join(", "));
    out.push('\n');
    for sample in 0..readback.num_samples() {
        let row: Vec<String> = (0..positioners.len())
            .flat_map(|axis| {
                [
                    format!("{:.6}", readback.actuals[axis][sample]),
                    format!("{:.6}", readback.following_errors[axis][sample]),
                ]
            })
            .collect();
        out.push_str(&row.join(", "));
        out.push('\n');
    }
    out
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
        // C: nitems != NUM_GATHERING_ITEMS fails the readback.
        let mut rb = GatheringReadback::new(2);
        let err = parse_gathering_buffer("1.0;1.5\n", 2, &mut rb).unwrap_err();
        assert!(err.contains("expected 4"), "unexpected error: {err}");
    }

    #[test]
    fn junk_value_is_an_error() {
        let mut rb = GatheringReadback::new(1);
        assert!(parse_gathering_buffer("1.0;abc\n", 1, &mut rb).is_err());
    }

    #[test]
    fn csv_round_trip_shape() {
        let mut rb = GatheringReadback::new(2);
        parse_gathering_buffer("1.0;1.5;10.0;10.25\n", 2, &mut rb).expect("parse");
        let csv = readback_csv(&["G1.P".to_string(), "G1.Q".to_string()], &rb);
        let mut lines = csv.lines();
        assert_eq!(
            lines.next(),
            Some("# G1.P.actual, G1.P.following_error, G1.Q.actual, G1.Q.following_error")
        );
        assert_eq!(
            lines.next(),
            Some("1.500000, 0.500000, 10.250000, 0.250000")
        );
        assert_eq!(lines.next(), None);
    }
}
