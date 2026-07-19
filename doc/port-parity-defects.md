# Port-parity defects

Regressions where a **Rust port diverges from its (correct) upstream C** —
the port is wrong, the C is right. This is the inverse of
`upstream-c-defects.md` (which records defects *in* the upstream C that the
port preserved or fixed). Entries here are fixed in the Rust driver crates.

Status legend: OPEN = confirmed, fix pending; FIXED = corrected at source
(commit noted); WONTFIX = documented divergence deliberately not changed.

Found by the 2026-07-19 parity re-audit round (opus panels) + orchestrator
family expansion. Each finding's *structural anchor* was searched
workspace-wide to find every port exhibiting the same defect (per the
"a citation is a sample of a family" rule); the **Family** line records that
cross-port result.

Severity: HIGH = wrong device state / wrong readback on the primary target;
MED = wrong behavior on a reachable but secondary path; LOW = latent /
unreachable-in-practice / cosmetic churn.

---

## PP-1 [HIGH] Amptek `preset_lt_done` decoded without the device-type gate — OPEN

- **Rust:** `drivers/mca-amptek/src/status.rs:149` (`preset_lt_done = raw[35] & 64 != 0`), consumed by `driver.rs:1131-1139` (`read_int32` `mcaAcquiring_` auto-stop).
- **C:** `AmptekSrc/DP5Status.cpp:70-79`. Bit 6 of `RAW[35]` means `PresetLtDone` **only** when `bDMCA_LiveTime = (DEVICE_ID == dppMCA8000D && Firmware >= 0x67)`; for every other device (the primary DP5, PX5, DP5G, TB5, DP5-X, older-firmware MCA8000D) C hardcodes `PresetLtDone = false` and that bit instead means `AFAST_LOCKED` (fast discriminator locked — a normal steady-state condition during acquisition).
- **Failure:** on a plain DP5 with the fast threshold locked (expected seconds into any real acquisition), the port reads `preset_lt_done == true`, sends `DisableMcaMcs`, and reports `acquiring=0` — acquisition spuriously auto-stops with no time preset configured.
- **Family:** Amptek-only (DP5 status decode); no other port decodes this register.
- **Fix:** gate `preset_lt_done` on `device_id == MCA8000D && firmware >= 0x67`, mirroring `bDMCA_LiveTime`.

## PP-2 [HIGH] Amptek spectrum read performs two UDP round trips instead of one — OPEN

- **Rust:** `drivers/mca-amptek/src/driver.rs:1180-1182` — `send_command(SendSpectrumStatus)` (full round trip, response discarded) immediately followed by `send_and_receive(target, SendSpectrumStatus)` (second full round trip, response used).
- **C:** `drvAmptek.cpp:1006-1011` + `ConsoleHelper.cpp` — one round trip: `sendCommand(XMTPT_SEND_SPECTRUM_STATUS)` sends and receives into `PacketIn`; the following `ReceiveData()` is pure parsing of that buffer, zero extra socket I/O.
- **Failure:** every spectrum read (driven each devMcaAsyn cycle) sends the request to the device twice, discards the first response, doubling wire traffic and per-read latency.
- **Family:** Amptek-only (this send/parse split is Amptek's ConsoleHelper shape).
- **Fix:** drop the redundant leading `send_command`; keep only `send_and_receive`.

## PP-3 [MED] Amptek spectrum read never refreshes the trailing DP4 status block — OPEN

- **Rust:** `drivers/mca-amptek/src/driver.rs:1178-1202` (`read_int32_array`) discards the trailing 64-byte status block that every `XMTPT_SEND_SPECTRUM_STATUS` response carries; `protocol.rs:291-299` documents that the caller "slices out any trailing 64-byte DP4 status block ... decodes it separately" but `read_int32_array` never does.
- **C:** `ConsoleHelper.cpp:736-754` (`ProcessSpectrumEx`) copies those bytes into `m_DP5_Status` and calls `Process_Status()` on every spectrum read.
- **Failure:** `self.last_status` (preset flags, HV, temperature) is stale after a spectrum read. **Currently masked** in `mca-amptek-ioc` because devMcaAsyn's per-cycle contract always issues `ReadStatus` before `Data`; surfaces for any consumer calling `read_int32_array` (the `asynInt32Array` interface) without an accompanying status poll.
- **Family:** Amptek-only.
- **Fix:** in `read_int32_array`, slice the trailing status block and refresh `last_status` via `status::process_status`.

## PP-4 [HIGH/MED] MW100 skipped-channel `data_status` never set to `SKIP_OFF` — OPEN

- **Rust:** `drivers/yokogawa-mw100/src/codec.rs` `parse_fe1` skip branch (~1155-1166) + `instrument.rs` `load_infos` FE1 apply loop (698-711) — writes only `ch_info`, never `ch_data.data_status`.
- **C:** `drvMW100.c:793-802` — the FE1 skip path sets `cd->data_status = VL_SKIP_OFF` (the *only* place a skipped channel's data status is set; the FD1 input poll never returns skipped channels).
- **Failure:** every skipped analog/math channel reports `VAL_STATUS = Normal` instead of `SkipOff`, permanently.
- **Family:** **MW100-only.** GM10 handles this correctly — `instrument.rs:671-688` sets `data_status = DataStatus::Skip` on SKIP (`drvGM10.c:733-742`). Cross-check confirmed the sibling is clean.
- **Fix:** carry a skip marker from `parse_fe1` and set the corresponding `ch_data.data_status` to `SkipOff` in `load_infos`.

## PP-5 [MED] Momentary `bo` VAL not reset to 0 after TRIG / ERROR_CLEAR — OPEN (FAMILY: GM10 + MW100)

- **MW100 — Rust:** `device_support.rs` `write()` arms `InputTrig`/`OutputTrig`/`InfoTrig`/`StatTrig`/`ErrorClearSet` (773-784). **C:** `devMW100_bo.c:192-202` resets `val = 0` for `REC_TRIG`/`REC_ERROR` in the PACT pass.
- **GM10 — Rust:** `device_support.rs:664-675` (`ChanTrig`/`MiscTrig`/`InfoTrig`/`StatTrig`/`ErrorClearSet` submit and return, no VAL reset). **C:** `devGM10_bo.c:192-196` resets `val = 0` for `REC_TRIG`/`REC_ERROR`.
- **Failure:** `caput 1` to a trigger/error-clear `bo` fires the command but VAL stays 1; the momentary output never returns to 0, breaking operator displays / client logic keyed on VAL→0. (C keeps `ALARM_ACK`/`OPMODE`/`VAL` un-reset — the asymmetry must be preserved.)
- **Family:** **GM10 + MW100** (both siblings). The MW100 auditor found it in MW100; the GM10 auditor missed it — the family expansion (same anchor: trigger `bo` write that submits without resetting VAL) caught GM10. One finding, two sites → one commit.
- **Fix:** after a successful submit of a trigger/error-clear op, set the record's VAL to 0 (only those op kinds — not opmode/alarm-ack/analog/binary VAL).

## PP-6 [MED] Compute-mode out-of-range masked-and-sent instead of rejected — OPEN (FAMILY: GM10 + MW100)

- **GM10 — Rust:** `codec.rs:84` `format!("OMath,{}\r\n", (b'0' + (mode & 0x3)) as char)`, from `instrument.rs` `SetCompute`; no range check. **C:** `drvGM10.c:1218-1226` `set_mode(CMD_SET_COMPUTE)` guards `if((value<0)||(value>3)) return 1;` before building the command.
- **MW100 — Rust:** `codec.rs:83` `format!("EX{}\r\n", (b'0' + (mode & 0x3)) as char)`; `SetCompute` unconditional. **C:** `drvMW100.c:1308-1315` same `value>3` guard.
- **Failure:** `COMPUTE_CMD` is an `mbbo` whose VAL can be driven 0-15. `caput …ComputeCmd.VAL 4` → C sends nothing (write errors, device unchanged); Rust masks `4 & 3 = 0` and silently sends `OMath,0` / `EX0`, switching the math engine to mode 0. Values 5→mode1, 7→mode3, etc.
- **Family:** **GM10 + MW100** — the `& 0x3` command-builder anchor matches exactly these two sites workspace-wide (all other `& 0x…` hits are device-response bit *decodes*, a different pattern). One finding, two sites → one commit.
- **Fix:** reject `mode > 3` before building the command (return an error, send nothing), matching C.

## PP-7 [LOW] GM10 channel + misc I/O-Intr fired unconditionally — OPEN

- **Rust:** `instrument.rs:785` (`load_data_values` fires `InterruptCategory::Channel` unconditionally) and `instrument.rs:820` (`load_misc_values` fires `InterruptCategory::Misc` unconditionally).
- **C:** `drvGM10.c:1071-1078` fires `scanIoRequest(channel_ioscanpvt)` only for `CMD_READ_ALL_DATA` or when the aggregate `alarm_flag` toggled; `:1148-1149` fires `misc_ioscanpvt` only for `CMD_READ_ALL_MISC`.
- **Failure:** a periodic single-channel poll re-scans every I/O-Intr channel/misc record and re-emits monitors on every poll — spurious scan/monitor churn C never produces (not wrong data).
- **Family:** **GM10-only.** MW100 was verified correct by its auditor (`scanIoRequest` firing conditions, incl. the single-channel alarm-flag reset, match C).
- **Fix:** gate the two fires on read-all (and, for channel, the alarm-flag toggle) as C does.

## PP-8 [LOW] GM10 `scaled_value` panics on `scale == 7` — OPEN

- **Rust:** `cache.rs:112-115` `SCALER[scale as usize & 0x7]` — `& 0x7` yields 0..=7 against a 7-element array; `scale ≡ 7 (mod 8)` (parsed as full `u8` in `codec.rs:454`) indexes `SCALER[7]` → panic on the instrument actor thread.
- **C:** `drvGM10.c:83-89` indexes `scaler[(int)scale]` (OOB read of adjacent memory, garbage, no crash).
- **Failure:** a nonconforming/corrupt `FChInfo` frame reporting scale `7` crashes the Rust instrument thread (vs C's silent garbage). Latent — the protocol scale is 0-6 on a conforming device.
- **Family:** GM10-only (this SCALER table/mask is GM10-specific).
- **Fix:** reject/clamp `scale > 6` instead of masking into an OOB index (no panic).

---

# Second wave — full 61-driver exhaustive re-audit (2026-07-19/20)

Found by the second parity sweep across **all 61 ported drivers** (34 AD/misc +
3 deferred + 27 motor; N/A: d435i has no C upstream, measComp trio absent
locally). Same rules: each anchor searched workspace-wide for the family.

## PP-12 [HIGH] smartmotor: VCONFAC velocity + ACONFAC accel unit conversions dropped

- **Rust:** `drivers/motor-smartmotor/src/smartmotor.rs:160,209` (velocity written raw, no `VCONFAC` scale) and `:64-66,163` (accel written raw, no `ACONFAC` scale + min-clamp applied to the wrong magnitude).
- **C:** `devSmartMotor.cc:58,190,250,290` multiply commanded velocity by `VCONFAC = 16.1063`; `:66,191,253-265` scale accel by `ACONFAC = 3.958322e-3` before the `AT=` command and clamp the *scaled* value.
- **Failure:** on real hardware every commanded velocity is ~16× too slow and every commanded acceleration is ~250× too large; the accel min-clamp guards the unscaled number so it never engages. Device-breaking on the primary target.
- **Family:** smartmotor-only (these constants are SmartMotor-specific); split into two commits (velocity, accel) — distinct constants/sites.
- **Fix:** apply `VCONFAC`/`ACONFAC` at the same sites C does; clamp the scaled accel.

## PP-13 [MED] acstech80 position/encoder readback rounds where C truncates

- **Rust:** `drivers/motor-acstech80/src/spiiplus.rs:444,449` — `nint()` (round-to-nearest) on the `FPOS`/`APOS` feedback strings.
- **C:** `devSPiiPlus.cc` uses `(long)atof(...)` (truncation toward zero).
- **Failure:** fractional feedback (e.g. 100.6 cts) reports 101 vs C's 100 — off-by-one readback near integer boundaries.
- **Family:** acstech80-only (nint-vs-cast on a readback string; other ports cast).
- **Fix:** truncate (`as i32`/`trunc`) to match `(long)atof`.

## PP-14 [MED] kohzu speed clamp applied only to jog, not positional/home moves

- **Rust:** `drivers/motor-kohzu/src/kohzu.rs:223,238,275` — positional and home move builders omit the `[1, 4095500]` speed clamp that the jog path applies.
- **C:** `drvKohzuHDR.cc` clamps the speed for every move command.
- **Failure:** a record with `VBAS=0` / very high `VELO` emits an out-of-range speed on absolute/home moves; the controller rejects the command (jog works, positioning does not).
- **Family:** kohzu-only (this clamp band is device-specific).
- **Fix:** hoist the clamp to a shared helper used by jog, positional, and home builders.

## PP-15 [MED] parker/oem motorStatusHome_ (encoder_home) never set

- **Rust:** `drivers/motor-parker/src/oem.rs:294-309` — the home-limit / `motorStatusHome_` MSTA bit is never assigned.
- **C:** `drvOms58.cc`/OEM status decode sets the home bit from the controller's home-switch state.
- **Failure:** under `UEIP=Yes` the record's `ATHM` field never asserts; homing-complete logic and displays keyed on `ATHM` never fire.
- **Family:** see PP-22 (MSTA status-bit divergences) — but this one is a straight omission, fix at source.
- **Fix:** decode and set `motorStatusHome_` from the OEM status word.

## PP-16 [MED] pi-gcs2: stale direction on velocity move + SPA precision loss

- **Rust:** `drivers/motor-pi-gcs2/src/gcs2.rs:628-646` — `last_direction` not updated on a velocity (jog) move; `:369` builds the `SPA` accel/decel value with `{:.6}`.
- **C:** `PIGCSController.cpp` updates the cached direction on every commanded move and formats `SPA` with `%.12g`.
- **Failure:** MSTA `RA_DIRECTION` reports the previous move's direction after a jog; `SPA` loses precision for accel/decel values needing >6 significant digits.
- **Family:** direction → PP-22; float-format → PP-18.
- **Fix:** set `last_direction` from the velocity sign; format `SPA` with a `%.12g`-equivalent.

## PP-17 [MED] smaract MCS2 drops FOLLOWING_LIMIT_REACHED (slip/stall)

- **Rust:** `drivers/motor-smaract/src/mcs2.rs:322-338` — the `0x0400` FOLLOWING_LIMIT_REACHED status bit is not mapped to `motorStatusSlipStall_`.
- **C:** `smarActMCS2.cpp` maps it to the slip/stall MSTA bit.
- **Failure:** a following-error stall is never reported to the record; operators lose the `SLIP_STALL` indication.
- **Family:** smaract-only (MCS2 status word); MCS-classic path uses a different word.
- **Fix:** map `0x0400` to `motorStatusSlipStall_`.

## PP-18 [MED] npoint/c300: POS wire format `{}` vs C `%f` (FAMILY: float wire format)

- **Rust:** `drivers/motor-npoint-c300/src/c300.rs:186,200` — position setpoint formatted with `{}` (Rust `f64::Display`, shortest round-trip).
- **C:** `C300Driver.cpp` uses `%f` (fixed 6 decimals).
- **Failure:** high-precision setpoints serialize to a different digit string than C; on a controller that parses fixed-width or rounds the field this changes the commanded position.
- **Family:** float-Display-vs-C-printf — **c300 `{}`/`%f`** and **pi-gcs2 `{:.6}`/`%.12g`** (PP-16). Both are wire-format divergences; fix each to match its C format specifier.
- **Fix:** format with `{:.6}` (c300) / `%.12g`-equivalent (pi-gcs2).

## PP-19 [MED] motorsim move-accept doesn't set done=0 immediately

- **Rust:** `drivers/motor-motorsim/src/motorsim.rs:232-303` — on accepting a move the driver does not clear `motorStatusDone_` before the first status poll.
- **C:** `motorSimDriver.cpp` sets `done=0` at move-accept time.
- **Failure:** a zero/near-zero-distance move can be observed with `DMOV=1` for one cycle (record may latch move-complete on a move that never appeared to start). Contingent on framework poll ordering.
- **Family:** motorsim-only.
- **Fix:** set `motorStatusDone_ = 0` synchronously when a move is accepted.

## PP-20 [MED] model-1 poll RETRY-once comms debounce dropped (FAMILY)

- **Rust:** `faulhaber.rs:342-359`, `kohzu.rs:322-331`, `pijeds.rs:310-319`, `smartmotor.rs` poll path, `thorlabs` poll path — a single failed/short poll reply immediately raises a comms error / PROBLEM.
- **C:** the model-1 `devXxx.cc` drivers run a `NORMAL → RETRY → COMM_ERR` state machine (one tolerated failure, often with a flush + re-read) before declaring the axis in error; thorlabs additionally recovers from a flipped command echo.
- **Failure:** one dropped byte / transient short reply spuriously alarms the axis (and, for thorlabs, a persistent `comms_error` if the echo state flips) where C rides through it.
- **Family:** the five model-1 poll ports above. One structural pattern (missing debounce state); fix as a shared retry-once helper per driver's poll.
- **Fix:** implement the NORMAL→RETRY→COMM_ERR debounce (flush + single re-read) before signalling error.

## PP-21 [MED] init-probe failure fatal to axis creation (FAMILY)

- **Rust:** `ims/mdriveplus.rs:142-146` (version reply <2 chars aborts axis creation), plus the same abort-on-probe shape in `kohzu` (IDN), `mclennan`, `micronix`, `oriel`.
- **C:** these controllers log/retry a failed identity probe but still create the axis (MForce-1 IMS drives legitimately error on `PR VR`).
- **Failure:** a drive that doesn't answer the identity/version query is dropped entirely — the axis never exists in the IOC (vs C creating a usable axis).
- **Family:** the init-probe-fatal ports above; verify each controller's C tolerates the probe failure before relaxing (do not relax a probe C treats as fatal).
- **Fix:** demote the probe failure from fatal to logged-and-continue where C does.

## PP-22 [MED] MSTA encoder/gain/home status bits diverge from C (FAMILY)

- **Rust:** two opposite divergences on `motorStatus*_` bits —
  - **wrongly ASSERTED** where C leaves clear: `mclennan/pm304.rs:438-439` (`EA_PRESENT`+`GAIN_SUPPORT`), `micos` taurus/hydra (has_encoder/direction hardcoded true).
  - **wrongly OMITTED** where C sets it: `pijeds.rs:343-359` (has_encoder never set → `EA_PRESENT=0` forces `UEIP=No`), parker/oem home bit (PP-15).
- **C:** each sets these bits from the controller's actual capability/status word.
- **Failure:** `MSTA` reports wrong encoder-present / gain-support / direction / home state; `UEIP` and homing/closed-loop logic key off these.
- **Family:** the sites above (plus the LOW-severity hardcodes listed in "Documented, not fixed"). Structural cause: MSTA bits set from a constant instead of the status word — fix per driver from the real status.
- **Fix:** derive each bit from the controller status, not a literal.

## PP-23 [LOW/AMBIGUOUS] jog velocity sign dropped via `.abs()` (FAMILY — needs manual verify)

- **Rust:** `faulhaber.rs:279` (SP), `mclennan/pm304.rs:325-327` (SV), `oriel/emc18011.rs`, `pijeds.rs` jog builders — jog speed emitted as magnitude; direction carried only by a separate sign/command field or dropped.
- **C:** the corresponding `devXxx.cc` sends a signed velocity on the same command.
- **Status:** **OPEN pending manual verification** — for some of these controllers the direction is legitimately a separate field and `.abs()` is correct; for others the sign is lost. Requires the device command reference to classify each; not fixed without it (do not guess protocol semantics).

## PP-24 [MED] marccd exposure countdown anchored before shutter opens

- **Rust:** `drivers/ad-marccd/src/...` — the exposure-time countdown/deadline is started before the shutter-open handshake completes.
- **C:** `marccdApp` starts timing at shutter-open.
- **Failure:** the frame is read out early by the shutter-open latency → systematic under-exposure.
- **Family:** marccd-only.
- **Fix:** anchor the exposure deadline at shutter-open.

## PP-25 [MED] marccd read-mode enum choices not narrowed (series-mode hang)

- **Rust:** the port drops the `read_enum`/menu-narrowing that restricts valid read modes per server capability.
- **C:** `marccdApp` narrows the menu so series/burst modes unavailable on the server can't be selected.
- **Failure:** selecting a server-unsupported read mode issues a command the server never answers → acquisition hangs.
- **Family:** marccd-only.
- **Fix:** restore the capability-narrowed enum.

## PP-26 [MED] eiger setShutter dropped from the internal-trigger loop

- **Rust:** `drivers/ad-eiger/src/...` internal-trigger acquire loop never calls the shutter open/close that C issues per frame.
- **C:** `eigerDetector.cpp` toggles the shutter inside the internal-trigger loop.
- **Failure:** with an external shutter wired, frames are exposed with the shutter in the wrong state.
- **Family:** eiger-only (marccd has a shutter but on a different path — PP-24).
- **Fix:** issue the shutter open/close inside the internal-trigger loop.

## PP-27 [MED] simdetector image `time_stamp` (double) never set

- **Rust:** `drivers/ad-simdetector/src/...` sets the NDArray epics/`epicsTS` fields but leaves the `double time_stamp` at 0.
- **C:** `ADDriver`/`simDetector` sets both `timeStamp` (double) and `epicsTS`.
- **Failure:** downstream NTNDArray `dataTimeStamp` is 0; pipeline stages / clients keyed on the double timestamp see an invalid time.
- **Family:** update-timestamps invariant — every AD port that builds an NDArray must set both fields. simdetector confirmed; the other AD ports should be swept against this invariant.
- **Fix:** set `time_stamp` alongside `epicsTS` (single update-timestamps helper).

## PP-28 [MED] specs-analyser readback params not mirrored from internal state

- **Rust:** `drivers/ad-specs-analyser/src/...` — Connected / ServerName / ProtocolVersion / message-counter readback params are not written back from the driver's internal connection state.
- **C:** `specsAnalyser.cpp` publishes each on state change.
- **Failure:** operator screens show stale/blank connection status and a frozen message counter even while the driver is live.
- **Family:** specs-analyser-only.
- **Fix:** mirror the internal state into the readback params on change.

## PP-29 [MED] quadem/pcr4 reset() aborts the reboot-wait loop on non-ACK

- **Rust:** `drivers/quadem-*/src/...` `reset()` returns `?` (propagates the error) on a non-ACK reply during the post-reset reboot.
- **C:** the C driver polls/waits through the reboot window, tolerating non-ACK until the device answers.
- **Failure:** a `Reset` bails out mid-reboot and leaves the driver in error instead of waiting for the device to come back.
- **Family:** quadem family (verify pcr4 + other quadem models share the reset path).
- **Fix:** retry/wait through the reboot window instead of propagating the first non-ACK.

## PP-30 [MED] opcua mbbo values-undefined branch skips mask/shift

- **Rust:** `drivers/opcua/src/...` — the mask/shift applied on the output path (upstream-c-defects #208) is missing from the "values undefined" mbbo branch.
- **C:** applies the mask/shift on both branches.
- **Failure:** an mbbo with no explicit state values writes an unmasked/unshifted raw value to the OPC UA node.
- **Family:** opcua-only.
- **Fix:** apply the same mask/shift in the values-undefined branch.

## PP-31 [MED] StreamDevice `ExtraInput=Ignore` not honored (FAMILY)

- **Rust:** `drivers/microepsilon-*/src/...` and `drivers/syringepump/src/...` (Teledyne H) — the reply parser rejects trailing/padding bytes that StreamDevice's `ExtraInput=Ignore` mode is configured to discard.
- **C/StreamDevice:** with `ExtraInput=Ignore` a match consumes the fields and ignores the remainder of the line.
- **Failure:** devices that pad replies (fixed-width, trailing status) fail every read with a parse error.
- **Family:** microepsilon + syringepump; any port re-implementing a StreamDevice protocol with `ExtraInput=Ignore` set.
- **Fix:** stop treating trailing bytes as a parse error when the protocol declares `ExtraInput=Ignore`.

## PP-32 [MED] ip shared worker log-and-skip drops record INVALID alarm (FAMILY)

- **Rust:** the shared IP-vacuum worker (`mks`, `televac`, `tpg261`, `mpc`) logs and skips a comms/parse failure without setting the record's alarm status.
- **C:** the corresponding device support returns an error so the record goes `READ/INVALID`.
- **Failure:** a failed read silently keeps the last value with `NO_ALARM`; operators can't distinguish live data from a dead link. Framework `set_param_status` (param.rs:1089) exists to signal this.
- **Family:** the four IP-vacuum ports above (shared worker).
- **Fix:** route comms/parse failure through `set_param_status` so the record alarms INVALID.

## PP-33 [MED] ip/tpg261 gauge-status byte never alarms the pressure record

- **Rust:** `drivers/ip-tpg261/src/...` — the per-gauge status (off / underrange / overrange / sensor error) is parsed but never mapped to the pressure record's alarm.
- **C:** `devTPG261` sets the record INVALID/alarm when the gauge status is not "measurement OK".
- **Failure:** a switched-off or errored gauge reports its stale/garbage pressure with `NO_ALARM`.
- **Family:** tpg261-specific status decode (distinct from PP-32's transport failure).
- **Fix:** map non-OK gauge status to the record alarm.

## PP-34 [MED] twincat-ads PLC BOOL/BIT write: truncate-then-!=0 vs C value>0

- **Rust:** `drivers/twincat-ads/src/...` — a BOOL/BIT write truncates the value to an integer then tests `!= 0`.
- **C:** the ADS device support tests `value > 0`.
- **Failure:** a fractional value in `(0,1)` writes FALSE where C writes... (0.5 → Rust truncates to 0 → FALSE; C `0.5 > 0` → TRUE); a negative value writes TRUE in Rust (`-2 != 0`) but FALSE in C (`-2 > 0` false). Opposite PLC state on both edges.
- **Family:** twincat-ads-only.
- **Fix:** test `value > 0` before writing the BOOL.

## PP-35 [MED] twincat-ads-ioc: time-source out-of-range → EPICS; adsTimeoutMS=0 not clamped

- **Rust:** `iocs/twincat-ads-ioc/src/main.rs` — an out-of-range time-source config selects EPICS time instead of PLC; `adsTimeoutMS=0` is accepted and yields an instant-timeout client.
- **C:** the iocsh config validates/clamps both.
- **Failure:** a misconfigured time source silently timestamps from the IOC not the PLC; a `0` timeout makes every ADS request time out immediately.
- **Family:** twincat-ads-ioc crate (config surface) — distinct from the driver PP-34.
- **Fix:** validate the time-source enum and clamp/reject `timeout == 0`.

## PP-36 [LOW→corruption] mythen partial/timed-out readout yields a silent corrupt NDArray

- **Rust:** `drivers/ad-mythen/src/...` — a short/timed-out frame readout still publishes an NDArray built from the partial buffer and leaves the detector running on a hard error.
- **C:** `mythen` treats a short readout as an acquisition error (no frame published, detector stopped/reset).
- **Failure:** on a comms hiccup the client receives a corrupt image indistinguishable from a good one, and the detector is left in an inconsistent running state.
- **Family:** mythen-only (this readout-length check is mythen-specific).
- **Fix:** treat a short readout as an error — do not publish, stop the detector.

---

# Third wave — measComp (usb-ctr / usb-2408 / meascomp), 2026-07-19

Audited after the user supplied the measComp C upstream. **Key scope finding:**
`usb-ctr` and `usb-2408` are **partial ports** — only a scalar control/waveform
surface is wired. The IOC db (`db/meascomp_*.template`, loaded by
`iocs/usb-{ctr,2408}-ioc/st.cmd`) defines only ai/ao/bi/bo/longin/longout/mbbo
records; there are **zero** waveform/mca/aai/aao records anywhere. So the
MCA-spectrum / scaler-count-array / time-waveform **array data path**, the whole
**scaler subsystem**, and the MCS **trigger-mode / point0-action / prescale**
controls are UNPORTED (no records *and* no driver methods) — recorded below as
scope reductions, not defects. Findings from the raw audit that target those
subsystems (originally rated HIGH against full C) are therefore **not live
defects** in this IOC configuration.

`meascomp` (the `uldaq-sys` safe wrapper) audited **clean**: FFI argument
order/type verified against every C `ul*` call site; only USB-only addressing
scope, the TC `-9999` caller-policy boundary, and `MAX_DEVICES=64` vs C's `100`
noted (all benign). The systemic "raw uldaq const vs CBW_* menu" risk the
usb-2408 auditor flagged is **resolved**: the db mbbo menus carry uldaq ordinals
(`Range +/-10V → 5 = BIP10VOLTS`), matching the driver's pass-through — correct.

## PP-40 [HIGH] usb-2408 internal-waveform amplitude is 2× too large

- **Rust:** `drivers/meascomp/usb-2408/src/wave_gen.rs:66,72-74,80,99` — uses full `amplitude` as peak (`offset + amplitude*sin`, square `offset±amplitude`, saw/random full-span).
- **C:** `drvMultiFunction.cpp:1542,1546,1549-1550,1554,1568-1570` — `AMPLITUDE` is peak-to-peak: `amplitude/2` about the offset.
- **Failure:** `WAVEGEN_AMPLITUDE=1V` produces ±1V (2Vpp) where C produces ±0.5V (1Vpp) — every internal waveform on the DAC is double the intended voltage (over-drive risk). `WAVEGEN_AMPLITUDE` record exists → live.
- **Family:** usb-2408 wave_gen (all four internal wave types).
- **Fix:** halve the amplitude about the offset, matching C's peak-to-peak semantics.

## PP-41 [MED] usb-2408 cluster (live, within ported wave/AO/AI scope)

Each sub-finding has an existing record and is a real divergence:
- **WAVEGEN_ENABLE ignored** — `driver.rs:329-330` hardcodes first/last chan = 0..MAX; C iterates enabled channels and errors if none (`drvMultiFunction.cpp:1603-1636`).
- **Immediate AO write has no generator-running guard** — `driver.rs:115-131`; C refuses `ANALOG_OUT_VALUE` with `asynError` while `waveGenRunning_` (`:2131-2135`).
- **Analog-in read not gated on channel type** — `poller.rs:119-138` reads voltage on every channel incl. thermocouple; C `continue`s on `type != AI_CHAN_TYPE_VOLTAGE` (`:2764`), overwriting TC records with garbage.
- **Volts→TC switch doesn't reprogram TC type + open-detect** — `driver.rs:157-166`; C re-applies `AI_CFG_CHAN_TC_TYPE` + `setOpenThermocoupleDetect()` (`:1969-1982`).
- **Wavegen pulse width treated as 0..1 fraction, delay dropped** — `wave_gen.rs:83-92`; C uses time-based `pulseWidth/dwell` sample counts with a delay region (`:1556-1566`). (`WAVEGEN_PULSE_DELAY` itself is unported — no record.)
- LOW sub: digital_output direction gate (`driver.rs:393-404`↔`:2404`); TC-type/open-detect `isThermocouple` guard (`:167-183`↔`:2004,2019`); wave-dig `-9999` bad-rate sentinel (`wave_dig.rs:152-154`↔`:1842-1846`); sin/saw period `numPoints-1` off-by-one (`wave_gen.rs:66,80`↔`:1545,1553`).

## PP-42 [MED] usb-ctr cluster (live, within ported MCS/pulse/counter scope)

- **MCS `SINGLEIO` threshold dropped** — `mcs.rs:164` always `SO_SINGLEIO`; C uses `SO_DEFAULTIO` and adds `SO_SINGLEIO` only when `dwell >= 0.01` (`drvUSBCTR.cpp:674,678-679`) — short-dwell high-rate scans lose data.
- **Pulse-generator input clamps dropped** — `pulse_gen.rs:17-26` passes frequency/duty/delay raw; C clamps to `[0.023,48e6]`/`[.0001,.9999]`/`[0,67.11]` (`:466-472`). `PULSE_*` records exist.
- **Model/`numCounters_` hardcoded to 8** — `scaler.rs:37` etc.; C derives 8 (CTR08) vs 4 (CTR04) and bounds loops (`:364-372,544,687`) — on a CTR04 the port drives nonexistent counters 4-7.
- **MCS start missing re-entry / already-complete guards** — `driver.rs:180-221`; C skips if `MCSRunning_` and short-circuits `currentPoint >= numTimePoints` (`:1189-1198`). (The `scalerRunning_` guard is moot — scaler unported.)
- **Elapsed real/live time never published** — `mcs.rs:241` computes `elapsed` only for the done check; C writes `mcaElapsedRealTime_`/`LiveTime_` each read (`:797-800`) — the `MCS:ElapsedReal` record (exists, I/O Intr) stays 0.
- LOW sub: `mca_num_channels` clamp to `maxTimePoints_` + actual-dwell writeback (`:1235-1240,711`); digital_output direction gate (`driver.rs:304-315`↔`:1348`); `counterReset_` `ulCClear` vs C `ulCLoad(CRT_LOAD,0)` (`:1124`, equivalence unconfirmed); MCS channel `range=0` vs `BIP10VOLTS` (benign for CTR).

---

# measComp unported subsystems — BEING COMPLETED (user decision 2026-07-19)

**Correction to the initial framing:** these subsystems are NOT beyond-C scope —
they ARE part of the standard C USB-CTR / USB-2408 IOC. The authoritative C boot
`measComp/iocBoot/iocUSBCTR/st.cmd` wires the scaler via the **standard EPICS
`scalerRecord`** (`scaler.db`, `DTYP="Asyn Scaler"`), the MCS control scalars via
`measCompMCS.template`, and the MCA spectrum via the **standard `mca` record**
(`simple_mca.db`, commented out by default → optional). The Rust ports omitted
them. Per the user's "complete it" decision they are being ported (worktree
panels `usbctr-completion`, `usb2408-completion`), reusing the sibling
`scaler974` (ScalerDriver trait) and `mca` module. Live defects that live INSIDE
these subsystems (scaler DONE-on-arm, ext-trigger, point0 Skip, prescale) are
fixed as part of the completion, grounded in `drvUSBCTR.cpp`.

- **usb-ctr scaler** → standard `scalerRecord` via ScalerDriver trait (like `scaler974`); wire `scaler.db` into st.cmd.
- **usb-ctr MCA spectrum + time-waveforms** → `read_int32_array`/`read_float32_array`/`read_float64_array` for `mcaData_`/`scalerRead_`/`MCSTimeWF_`/`MCSAbsTimeWF_`; MCA spectrum to the standard `mca` record (commented example matching C's `simple_mca.db`).
- **usb-ctr MCS trigger-mode / point0-action / prescale** → `ulDaqInSetTrigger` + unconditional `SO_EXTTRIGGER`; Clear/NoClear/Skip (not a bool); prescale counter config. Fix the wrong `mcs.rs:83-85` comment.
- **usb-2408 waveform-gen user/internal arrays** → `UserTimeWF`/`IntTimeWF` waveform records + `write_float32_array` user buffer plumbing (fixes User-type → 0V).
- **usb-2408 AO sync-write** → `ANALOG_OUT_SYNC_MASTER` + `AOUTARRAY_FF_SIMULTANEOUS`.
- **usb-2408 per-counter value poller read** is a Rust ADDITION (not a C divergence); kept — revisit only if it errors on non-counting-configured counters.

---

## Documented, not fixed (unreachable / port-is-stricter / degenerate)

- **PP-9 [LOW] GM10 FData module-presence gate `(address-1)/100` vs C `address/100`** (`instrument.rs:752` ↔ `drvGM10.c:995`). Differs only at addresses that are exact multiples of 100, which are never real channels (channels are `module*100 + 1..=n`). C is itself internally inconsistent (`:712` uses the 0-based form). Unreachable — not fixed.
- **PP-10 [LOW] GM10 strict whole-string link parse vs C lenient prefix parse** (`link.rs:82` ↔ `devGM10_*.c` `atoi`/`strtol`). Rust rejects trailing garbage in db link text that C would accept; only reachable via malformed db. The port being stricter is defensible — not fixed.
- **PP-11 [LOW] Rontec zero-length spectrum read early-return** (`drivers/mca-rontec/src/driver.rs:284`). Rust returns empty for `max_chans == 0`; C still sends `$SS …,0` and drains a 4-byte reply. Degenerate — a real mca record never requests 0 channels. Not fixed.
- **PP-37 [LOW] Motor MSTA direction/encoder bits hardcoded to a constant** — `micos` taurus/hydra/corvus (direction=true, has_encoder=true), `pi` c662/c630 (`EA_POSITION`/powered hardcoded), `attocube` (direction hardcoded), `mvp2001` (encoder_position set where C leaves 0), various C-series. These set an MSTA bit from a literal instead of the status word. Low impact where the constant happens to match the common configuration; the *reachable* wrong-state cases (pijeds omit, mclennan/micos assert) are promoted to **PP-22** and fixed. The remaining constant-hardcodes are recorded here — fix opportunistically when touching each driver, not a separate round.
- **PP-38 [LOW] Motor init/version probe made fatal on controllers where C also treats it as fatal** — a subset of the PP-21 candidates turned out to match C (the probe *is* required). Recorded so a later reviewer doesn't re-flag them: only the ports listed under **PP-21** are confirmed-divergent; the rest abort exactly as C does.
- **PP-39 [LOW] oriel/emc18011 missing second-message drain after `L`** (`drivers/motor-oriel/src/emc18011.rs:167-184`). C drains a second reply line after the `L` (limits) query; Rust reads one. Only matters if the controller emits the trailing line on this firmware — unverified against hardware. Recorded, not fixed without a device to confirm.

### No new defects (audited clean)

acs (MCB4B), acsmotion, amci, aerotech (both variants), oms-asyn, parker/acr, phytron — motor. These were audited value-for-value against their C upstream with no divergence found.

### Audit deferred — upstream absent locally

measComp / usb-2408 / usb-ctr — the measComp C module is **not present** on this machine, so these three ports could not be audited. Provide the measComp source path to complete them.

### Scope-limited audits (siblings not yet covered)

- **newport** — only `smc100` audited; `agap`/`agilis`/`conex`/`esp300`/`hxp`/`mm3000`/`mm4000`/`pm500`/`pmnc`/`xps` not yet swept.
- **pi** — C-series (c862/c848/c844/c663/c662/c630) + E-series (prior round) audited; any other PI model not covered.
- **npoint** — only `c300` audited, not `lc400`.
