//! Typed wrappers for the hexapod-specific HXP RPC functions.
//!
//! Same marshalling rules as [`crate::xps::commands`] (`FuncName (args)` with a
//! space before `(`, doubles as `%.13g`), matching the vendor
//! `hxp_drivers.cpp`. The group-level functions the HXP driver also uses
//! (`GroupStatusGet`, `GroupKill`, `GroupInitialize`, `GroupHomeSearch`,
//! `GroupMoveAbort`, `GroupMotionEnable`/`Disable`, `FirmwareVersionGet`) are
//! wire-identical to their XPS counterparts and reused from there.

use crate::xps::rpc::{XpsResult, XpsSocket, format_g};

/// Double precision used by the vendor library (`%.13g`).
const G13: usize = 13;

/// Format a double for the wire exactly as `hxp_drivers.cpp` (`%.13g`).
fn g(value: f64) -> String {
    format_g(value, G13)
}

/// Join six coordinates as `%.13g` fields.
fn g6(p: &[f64; 6]) -> String {
    p.iter().map(|v| g(*v)).collect::<Vec<_>>().join(",")
}

impl XpsSocket {
    /// `HexapodMoveAbsolute(group, coordSystem, X..W)` — move all six hexapod
    /// axes to absolute coordinates in `coord_system` (`Work`/`Tool`/`Base`).
    pub fn hexapod_move_absolute(
        &self,
        group: &str,
        coord_system: &str,
        p: &[f64; 6],
    ) -> XpsResult<()> {
        let cmd = format!("HexapodMoveAbsolute ({group},{coord_system},{})", g6(p));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `HexapodMoveIncremental(group, coordSystem, dX..dW)` — incremental move
    /// in `coord_system`.
    pub fn hexapod_move_incremental(
        &self,
        group: &str,
        coord_system: &str,
        p: &[f64; 6],
    ) -> XpsResult<()> {
        let cmd = format!("HexapodMoveIncremental ({group},{coord_system},{})", g6(p));
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// `HexapodCoordinateSystemGet(group, coordSystem)` → the coordinate
    /// system's origin `(X, Y, Z, U, V, W)`.
    pub fn hexapod_coordinate_system_get(
        &self,
        group: &str,
        coord_system: &str,
    ) -> XpsResult<[f64; 6]> {
        let cmd = format!(
            "HexapodCoordinateSystemGet ({group},{coord_system},double *,double *,double *,double *,double *,double *)"
        );
        let r = self.exec(&cmd)?.require_ok()?;
        Ok([
            r.double(1),
            r.double(2),
            r.double(3),
            r.double(4),
            r.double(5),
            r.double(6),
        ])
    }

    /// `HexapodCoordinateSystemSet(group, coordSystem, X..W)` — redefine the
    /// coordinate system's origin.
    pub fn hexapod_coordinate_system_set(
        &self,
        group: &str,
        coord_system: &str,
        p: &[f64; 6],
    ) -> XpsResult<()> {
        let cmd = format!(
            "HexapodCoordinateSystemSet ({group},{coord_system},{})",
            g6(p)
        );
        self.exec(&cmd)?.require_ok()?;
        Ok(())
    }

    /// Read all six current group positions: `GroupPositionCurrentGet` on
    /// classic HXP firmware, `HexapodPositionCurrentGet` on HXP-D (C
    /// `HXPSetHexapodForFirmwareXPS_D` swaps the function name globally).
    pub fn hexapod_positions_get(&self, group: &str, hxpd: bool) -> XpsResult<[f64; 6]> {
        let name = if hxpd {
            "HexapodPositionCurrentGet"
        } else {
            "GroupPositionCurrentGet"
        };
        let cmd = format!("{name} ({group},double *,double *,double *,double *,double *,double *)");
        let r = self.exec(&cmd)?.require_ok()?;
        Ok([
            r.double(1),
            r.double(2),
            r.double(3),
            r.double(4),
            r.double(5),
            r.double(6),
        ])
    }
}
