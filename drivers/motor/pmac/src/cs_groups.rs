//! Coordinate-system groups, ported from
//! `pmacApp/pmacAsynMotorPortSrc/pmacCsGroups.cpp`.
//!
//! A *group* is a named set of real axes together with the coordinate-system
//! axis definition each one takes (`A`, `B`, … `Z`, or an expression like
//! `U*2300+20`) and the CS number it belongs to. Switching to a group aborts
//! all motion, undefines every CS mapping, and re-issues the group's
//! `&<cs> #<axis>-><def>` definitions. A group whose axes map one-to-one onto
//! CS axis letters can additionally run *coordinated* deferred moves through
//! motion program 101 (see the C file's header for the PROG 101 listing and the
//! `i5n13`/`i5n20`/`i5n50` setup the controller needs).
//!
//! Every method here is pure: it returns the command strings to send, and
//! [`crate::controller::PmacController`] sends them. That keeps the group logic
//! testable without a controller and keeps the octet handle owned in one place.

use std::collections::BTreeMap;

/// CS axis letter -> the Q variable PROG 101 reads its demand from
/// (C `pmacCsGroups::axisNamesToQ`, Q71..Q79).
fn axis_letter_to_q(letter: char) -> Option<u32> {
    match letter {
        'A' => Some(71),
        'B' => Some(72),
        'C' => Some(73),
        'U' => Some(74),
        'V' => Some(75),
        'W' => Some(76),
        'X' => Some(77),
        'Y' => Some(78),
        'Z' => Some(79),
        _ => None,
    }
}

/// The Q variable PROG 101 takes the move time from (C `Q70`).
const Q_MOVE_TIME: u32 = 70;
/// The coordinated-move motion program (C `B101R`).
const COORD_MOVE_PROG: u32 = 101;

/// One axis's place in a coordinate system.
#[derive(Debug, Clone)]
pub struct CsAxisDef {
    /// The CS axis definition, e.g. `X` or `U*2300+20`.
    pub definition: String,
    /// The coordinate system this definition lives in.
    pub coord_sys: i32,
}

#[derive(Debug, Default)]
struct CsGroup {
    #[allow(dead_code)] // C keeps the name for reporting only.
    name: String,
    /// Real axis number -> its definition.
    axis_defs: BTreeMap<i32, CsAxisDef>,
}

/// A deferred move waiting to be executed (C `pmacAxis::deferredPosition_` /
/// `deferredMove_` / `deferredRelative_` / `deferredTime_`, held here instead of
/// on the axis so the one actor that executes them owns them).
#[derive(Debug, Clone, Copy)]
pub struct DeferredMove {
    /// Demand position, in controller counts.
    pub position: f64,
    /// True if the demand is relative to the current position.
    pub relative: bool,
    /// Estimated move time in milliseconds, used as PROG 101's `Q70`.
    pub time_ms: f64,
}

/// The last poll result the coordinated-move builder needs from each axis
/// (C reaches into `pmacAxis::previous_position_` / `moving_` for these).
#[derive(Debug, Clone, Copy, Default)]
pub struct AxisPoll {
    /// Last polled position, in controller counts.
    pub position: f64,
    /// Whether the axis was moving at that poll.
    pub moving: bool,
}

#[derive(Debug, Default)]
pub struct CsGroups {
    groups: BTreeMap<i32, CsGroup>,
    current: i32,
}

impl CsGroups {
    pub fn new() -> Self {
        // C: no axis is in a coordinate system until PINI writes the
        // COORDINATE_SYS_GROUP PV; group 0 is simply "nothing defined".
        Self::default()
    }

    /// C `pmacCsGroups::addGroup` (its `axisCount` argument is unused there too
    /// — the definitions arrive one at a time through [`Self::add_axis`]).
    pub fn add_group(&mut self, id: i32, name: &str) {
        self.groups.insert(
            id,
            CsGroup {
                name: name.to_string(),
                axis_defs: BTreeMap::new(),
            },
        );
    }

    /// C `pmacCsGroups::addAxisToGroup`. Errors if the group does not exist —
    /// C's `std::map::operator[]` would silently default-construct it, leaving a
    /// group nobody declared.
    pub fn add_axis(
        &mut self,
        id: i32,
        axis: i32,
        definition: &str,
        coord_sys: i32,
    ) -> Result<(), String> {
        let group = self
            .groups
            .get_mut(&id)
            .ok_or_else(|| format!("coordinate system group {id} has not been created"))?;
        group.axis_defs.insert(
            axis,
            CsAxisDef {
                definition: definition.to_string(),
                coord_sys,
            },
        );
        Ok(())
    }

    pub fn has_group(&self, id: i32) -> bool {
        self.groups.contains_key(&id)
    }

    /// The coordinate system `axis` is currently mapped into, or 0 if it is in
    /// none (C `pmacCsGroups::getAxisCoordSys`).
    pub fn axis_coord_sys(&self, axis: i32) -> i32 {
        self.groups
            .get(&self.current)
            .and_then(|g| g.axis_defs.get(&axis))
            .map(|d| d.coord_sys)
            .unwrap_or(0)
    }

    /// The commands that switch every axis into group `id`
    /// (C `pmacCsGroups::switchToGroup`): abort everything and undefine all CS
    /// mappings, then re-define this group's axes.
    ///
    /// **Upstream fix.** C indexes the axis-definition *map* with the loop
    /// counter — `(*pAxisDefs)[i]` for `i` in `0..size()` — but that map is
    /// keyed by **axis number**, not by position. With `std::map::operator[]`
    /// every missing key is default-inserted, so the loop emits a bogus
    /// `&0 #0->` for key 0 (axes are 1-based), grows the map as it walks it,
    /// and silently mis-maps any non-contiguous axis set. This port iterates
    /// the definitions themselves.
    pub fn switch_commands(&mut self, id: i32) -> Result<Vec<String>, String> {
        let group = self
            .groups
            .get(&id)
            .ok_or_else(|| format!("invalid coordinate system group number {id}"))?;
        // 0x01 is ctrl-A: abort all motion and all motion programs.
        let mut cmds = vec!["\x01\nundefine all".to_string()];
        for (axis, def) in &group.axis_defs {
            cmds.push(format!("&{} #{}->{}", def.coord_sys, axis, def.definition));
        }
        self.current = id;
        Ok(cmds)
    }

    /// The command that aborts the motion program in the CS `axis` is mapped
    /// into, or `None` if it is in no CS (C `pmacCsGroups::abortMotion`).
    pub fn abort_command(&self, axis: i32) -> Option<String> {
        match self.axis_coord_sys(axis) {
            0 => None,
            cs => Some(format!("&{cs}A")),
        }
    }

    /// The commands for a coordinated deferred move through PROG 101
    /// (C `pmacCsGroups::processDeferredCoordMoves`).
    ///
    /// Every axis with a pending move must be in the same coordinate system and
    /// map one-to-one onto a CS axis letter; every *other* axis in that CS is
    /// commanded to hold its last polled position, and must not already be
    /// moving. Returns the abort command followed by the PROG 101 move.
    pub fn coord_move_commands(
        &self,
        deferred: &BTreeMap<i32, DeferredMove>,
        poll: &BTreeMap<i32, AxisPoll>,
    ) -> Result<Vec<String>, String> {
        if deferred.is_empty() {
            return Ok(Vec::new());
        }

        let mut coord_sys = 0;
        let mut max_time_ms = 0.0f64;
        // CS axis letter -> (Q variable, demand), ordered by letter as C's
        // std::map<char,int> moveList is.
        let mut moves: BTreeMap<char, (u32, f64)> = BTreeMap::new();
        let mut cmds = Vec::new();

        for (&axis, move_) in deferred {
            if coord_sys == 0 {
                // C aborts the CS before validating the rest of the group.
                if let Some(abort) = self.abort_command(axis) {
                    cmds.push(abort);
                }
                coord_sys = self.axis_coord_sys(axis);
                if coord_sys == 0 {
                    return Err(format!(
                        "deferred coordinated move on real axis {axis} not in a coordinate system"
                    ));
                }
            } else {
                let cs = self.axis_coord_sys(axis);
                if cs != coord_sys {
                    return Err(format!(
                        "deferred coordinated move on multiple coordinate systems {coord_sys} and {cs}"
                    ));
                }
            }

            let def = self.definition(axis).unwrap_or_default();
            let q = single_letter(&def)
                .and_then(axis_letter_to_q)
                .ok_or_else(|| {
                    format!(
                        "illegal deferred coordinated move on real axis {axis} defined as \
                     {def} in CS {coord_sys}"
                    )
                })?;
            moves.insert(single_letter(&def).unwrap(), (q, move_.position));
            max_time_ms = max_time_ms.max(move_.time_ms);
        }

        // Axes in this CS with no deferred move hold their last polled position.
        // An axis that is already moving makes the coordinated move illegal.
        let mut holds: BTreeMap<char, (u32, f64)> = BTreeMap::new();
        for (&axis, def) in self.current_defs() {
            if def.coord_sys != coord_sys || deferred.contains_key(&axis) {
                continue;
            }
            let last = poll.get(&axis).copied().unwrap_or_default();
            if last.moving {
                return Err(format!(
                    "illegal deferred coordinated move - real axis {axis} in CS \
                     {coord_sys} is already moving"
                ));
            }
            // C takes `axisDef[0]` here without the one-character check it
            // applies to the moving axes; an expression definition ("U*2300+20")
            // is not one-to-one, so it has no Q demand to hold and is skipped
            // rather than being written to the Q of its leading letter.
            if let Some((letter, q)) =
                single_letter(&def.definition).and_then(|l| axis_letter_to_q(l).map(|q| (l, q)))
            {
                holds.insert(letter, (q, last.position));
            }
        }

        let mut move_str = String::new();
        for (q, value) in moves.values().chain(holds.values()) {
            move_str.push_str(&format!(" Q{q}={value:.6}"));
        }
        cmds.push(format!(
            "&{coord_sys} Q{Q_MOVE_TIME}={max_time_ms:.6}{move_str} B{COORD_MOVE_PROG}R"
        ));
        Ok(cmds)
    }

    fn current_defs(&self) -> impl Iterator<Item = (&i32, &CsAxisDef)> {
        self.groups
            .get(&self.current)
            .into_iter()
            .flat_map(|g| g.axis_defs.iter())
    }

    fn definition(&self, axis: i32) -> Option<String> {
        self.groups
            .get(&self.current)?
            .axis_defs
            .get(&axis)
            .map(|d| d.definition.clone())
    }
}

/// The CS axis letter of a definition that is exactly one character, else
/// `None` (C checks `axisDef.length() != 1`).
fn single_letter(def: &str) -> Option<char> {
    let mut chars = def.chars();
    let c = chars.next()?;
    chars.next().is_none().then_some(c)
}

/// The commands for a *fast* (uncoordinated) deferred move
/// (C `pmacController::processDeferredMoves`): one combined line that jogs every
/// deferred axis, absolute (`J=`) or relative (`J^`).
///
/// C builds this with `sprintf(command, "%s #%d%s%.2f", command, …)` — passing
/// the destination buffer as its own source argument, which is undefined
/// behaviour (overlapping copy). Building a `String` here has the behaviour the
/// C intends.
pub fn fast_deferred_command(deferred: &BTreeMap<i32, DeferredMove>) -> Option<String> {
    if deferred.is_empty() {
        return None;
    }
    let mut cmd = String::new();
    for (axis, move_) in deferred {
        let op = if move_.relative { "J^" } else { "J=" };
        cmd.push_str(&format!(" #{axis}{op}{:.2}", move_.position));
    }
    Some(cmd)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn group_with(axes: &[(i32, &str, i32)]) -> CsGroups {
        let mut g = CsGroups::new();
        g.add_group(1, "test");
        for (axis, def, cs) in axes {
            g.add_axis(1, *axis, def, *cs).unwrap();
        }
        g
    }

    #[test]
    fn switch_emits_abort_then_one_definition_per_axis() {
        let mut g = group_with(&[(1, "X", 2), (2, "Y", 2)]);
        let cmds = g.switch_commands(1).unwrap();
        assert_eq!(
            cmds,
            vec![
                "\x01\nundefine all".to_string(),
                "&2 #1->X".to_string(),
                "&2 #2->Y".to_string(),
            ]
        );
    }

    #[test]
    fn switch_maps_a_non_contiguous_axis_set_correctly() {
        // The C loop indexes the axis map by 0..size, so axes {1,3,5} come out
        // as keys 0..5 — a bogus `&0 #0->` plus default-inserted junk. Here the
        // definitions themselves are iterated.
        let mut g = group_with(&[(1, "X", 2), (3, "Y", 2), (5, "Z", 2)]);
        let cmds = g.switch_commands(1).unwrap();
        assert_eq!(cmds.len(), 4);
        assert!(!cmds.iter().any(|c| c.contains("#0->")));
        assert_eq!(cmds[1], "&2 #1->X");
        assert_eq!(cmds[2], "&2 #3->Y");
        assert_eq!(cmds[3], "&2 #5->Z");
    }

    #[test]
    fn switch_rejects_an_unknown_group() {
        let mut g = group_with(&[(1, "X", 2)]);
        assert!(g.switch_commands(7).is_err());
    }

    #[test]
    fn add_axis_rejects_an_undeclared_group() {
        let mut g = CsGroups::new();
        assert!(g.add_axis(3, 1, "X", 2).is_err());
    }

    #[test]
    fn axis_coord_sys_is_zero_until_a_group_is_selected() {
        let mut g = group_with(&[(1, "X", 2)]);
        assert_eq!(g.axis_coord_sys(1), 0);
        g.switch_commands(1).unwrap();
        assert_eq!(g.axis_coord_sys(1), 2);
        assert_eq!(g.axis_coord_sys(9), 0);
        assert_eq!(g.abort_command(1).as_deref(), Some("&2A"));
        assert_eq!(g.abort_command(9), None);
    }

    #[test]
    fn fast_deferred_combines_absolute_and_relative_jogs() {
        let mut d = BTreeMap::new();
        d.insert(
            1,
            DeferredMove {
                position: 100.0,
                relative: false,
                time_ms: 10.0,
            },
        );
        d.insert(
            2,
            DeferredMove {
                position: -5.5,
                relative: true,
                time_ms: 0.0,
            },
        );
        assert_eq!(fast_deferred_command(&d).unwrap(), " #1J=100.00 #2J^-5.50");
        assert_eq!(fast_deferred_command(&BTreeMap::new()), None);
    }

    #[test]
    fn coord_move_holds_the_other_axes_in_the_cs_at_their_last_position() {
        let mut g = group_with(&[(1, "X", 2), (2, "Y", 2), (3, "Z", 2)]);
        g.switch_commands(1).unwrap();

        let mut deferred = BTreeMap::new();
        deferred.insert(
            1,
            DeferredMove {
                position: 10.0,
                relative: false,
                time_ms: 250.0,
            },
        );
        let mut poll = BTreeMap::new();
        poll.insert(
            2,
            AxisPoll {
                position: 3.0,
                moving: false,
            },
        );
        poll.insert(
            3,
            AxisPoll {
                position: -4.0,
                moving: false,
            },
        );

        let cmds = g.coord_move_commands(&deferred, &poll).unwrap();
        assert_eq!(cmds[0], "&2A");
        // X (Q77) moves; Y (Q78) and Z (Q79) hold. Q70 is the longest move time.
        assert_eq!(
            cmds[1],
            "&2 Q70=250.000000 Q77=10.000000 Q78=3.000000 Q79=-4.000000 B101R"
        );
    }

    #[test]
    fn coord_move_rejects_axes_in_different_coordinate_systems() {
        let mut g = group_with(&[(1, "X", 2), (2, "X", 3)]);
        g.switch_commands(1).unwrap();
        let mut deferred = BTreeMap::new();
        for axis in [1, 2] {
            deferred.insert(
                axis,
                DeferredMove {
                    position: 1.0,
                    relative: false,
                    time_ms: 0.0,
                },
            );
        }
        let err = g
            .coord_move_commands(&deferred, &BTreeMap::new())
            .unwrap_err();
        assert!(err.contains("multiple coordinate systems"));
    }

    #[test]
    fn coord_move_rejects_an_axis_that_is_in_no_coordinate_system() {
        let g = CsGroups::new(); // no group selected -> every axis has CS 0
        let mut deferred = BTreeMap::new();
        deferred.insert(
            1,
            DeferredMove {
                position: 1.0,
                relative: false,
                time_ms: 0.0,
            },
        );
        let err = g
            .coord_move_commands(&deferred, &BTreeMap::new())
            .unwrap_err();
        assert!(err.contains("not in a coordinate system"));
    }

    #[test]
    fn coord_move_rejects_a_non_one_to_one_axis_definition() {
        let mut g = group_with(&[(1, "U*2300+20", 2)]);
        g.switch_commands(1).unwrap();
        let mut deferred = BTreeMap::new();
        deferred.insert(
            1,
            DeferredMove {
                position: 1.0,
                relative: false,
                time_ms: 0.0,
            },
        );
        let err = g
            .coord_move_commands(&deferred, &BTreeMap::new())
            .unwrap_err();
        assert!(err.contains("illegal deferred coordinated move"));
    }

    #[test]
    fn coord_move_rejects_a_held_axis_that_is_already_moving() {
        let mut g = group_with(&[(1, "X", 2), (2, "Y", 2)]);
        g.switch_commands(1).unwrap();
        let mut deferred = BTreeMap::new();
        deferred.insert(
            1,
            DeferredMove {
                position: 10.0,
                relative: false,
                time_ms: 0.0,
            },
        );
        let mut poll = BTreeMap::new();
        poll.insert(
            2,
            AxisPoll {
                position: 3.0,
                moving: true,
            },
        );
        let err = g.coord_move_commands(&deferred, &poll).unwrap_err();
        assert!(err.contains("already moving"));
    }
}
