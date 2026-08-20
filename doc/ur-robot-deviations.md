# ur-robot deliberate deviations

Where the Rust port intentionally diverges from upstream urRobot C
(survey base 093785d, resynced to fe89715). Unlike
`port-parity-defects.md` (port bugs) and `upstream-c-defects.md`
(C bugs), every row here is a deliberate behavioral choice, added as
the improvement plan (`ur-driver-improvement-plan.md`) lands. Planned
rows not yet shipped (A3 mbbi conversion) are added when their item
lands.

| Plan | C behavior | Our behavior | Rationale |
|------|------------|--------------|-----------|
| A1 | A lost RTDE receive stream stays dead until an operator writes RECONNECT; records keep serving the last decoded packet. | The receive poll thread detects a broken or stale stream and reconnects on its own. | This IOC's deployed role is monitoring; telemetry must not stay down waiting for an operator. |
| A2 | Records keep their last values through an outage at NO_ALARM; nothing marks them stale. | Device-state readbacks carry COMM/INVALID from the first poll cycle that sees the outage, cleared in the same batch that carries fresh values. Per-driver `alarm_targets()` with completeness tests. | A stale readback indistinguishable from a live one is a wrong readback for a monitoring IOC. |
| A4 | One `dashboard.db` mixes readbacks with 13 control records (Shutdown, PowerOff, BrakeRelease, …). | `dashboard.db` is read-only; the control surface lives in `dashboard_ctrl.db`, loaded only where control is wanted. | An unloaded record cannot be written via CA/PVA — removal of the surface, not a permission check. |
| B2 | A refused control-script command returns `false`; every driver call site drops the bool, the record completes normally, and a refused moveJ enters the wait state. | Refusal is `Err(CommandRefused { reason })` → asynError → record SEVR; dropping a refusal no longer compiles; a refused move finishes the motion task instead of entering `WaitingMotion`; a failed safety-limit query releases the busy record before the error propagates. | C conflates "refused" with a query answering "no"; the dropped refusals report success on a robot that did nothing. |
| B5 | One `recv()` per gripper request is treated as the whole reply (ur_rtde `robotiq_gripper.cpp:98-104` has the same single-read `receive()`). | `transact` frames replies on `'\n'` with a carry-over buffer, cleared on connect and disconnect. | Wire correctness, not preference: TCP preserves no message boundaries, so the C behavior misparses split and coalesced replies (`upstream-c-defects.md` #220). |
| C1 | No grasp-state PV: operators combine `IsActive`, `MoveStatus`, `IsOpen`/`IsClosed` themselves. | `GraspState` mbbi (0 Unknown / 1 Inactive / 2 Moving / 3 Open / 4 Closed empty / 5 Holding inner / 6 Holding outer) derived per poll from device facts alone; at-target mid-stroke maps to Unknown because no named state is provable. | MISGRIP/DROPPED need command context and stay with the sequencer (vision_inspection_plan.md §0.2); this row is the IOC-side half. |
| B3 | A setting written while the device is unreachable errors and the value is lost. PINI processes every setting record at iocsh time, before any connect can have succeeded, so every boot-time setting (gripper range/unit/speed/force, AUTO_ACTIVATE, TCP offset) is structurally lost. | Gripper speed/force/range/unit configure the persistent `RobotiqGripper` object at write time; ACTIVATE and the six `TCPOffset_*` writes stage in the driver and `try_connect` delivers them once the link is up. The io port is exempt: its writes are momentary pin actions, and delivering one minutes later on a reconnect would be unsafe. | The PINI race cannot be won by ordering; the connection owner applying staged state is the only closure. |
