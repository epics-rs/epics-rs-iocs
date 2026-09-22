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

## Wave-2 fix status — FIXED on unmerged worktree branches (user merges)

All verified against upstream C first; one commit per finding; docs untouched by
the fixers. Per-crate `fmt`/`clippy -D warnings`/`nextest` green; full-workspace
pass still owed before push.

- **branch `caucus/MXSR5DMPDZ/motor-fix-a-250d2112-1`:** PP-12 (`8a37c55` velocity, `a145992` accel), PP-13 `702a85e`, PP-14 `3fd0d3e`, PP-19 `9c6a91a`, PP-20 `cf72a37`.
- **branch `caucus/MXSR5DMPDZ/motor-fix-b-240d1f7f-1`:** PP-15 `0f479d8`, PP-16 `95bf86b`, PP-17 `a135c08`, PP-18 `b24f896`, PP-21 `5be3a47` (ims/mclennan/micronix fixed; kohzu/oriel verified not-a-defect), PP-22 `6d87be0`.
- **branch `caucus/MXSR5DMPDZ/ad-fix-cc1be77a-1`:** PP-24 `1f0f748`, PP-25 `fd946c0`, PP-26 `615fe51`, PP-27 `03705dc` (sibling sweep: simdetector was the only omission), PP-28 `f541208`, PP-29 `a14bb78`, PP-36 `0070e62`.
- **branch `caucus/MXSR5DMPDZ/proto-fix-c74a3f6f-1`:** PP-30 `d526968`, PP-31 `e885106`, PP-32 `80d6e18`, PP-33 `50331d8`, PP-34 `650dd8f`, PP-35 `31b3a14`.

Not fixed: PP-23 (deferred — needs device manual); PP-37/38/39 (observations).
PP-40/41/42 (measComp live) were recorded **FIXED** inside the completion port, but
those branches were never merged — see the wave-4 status correction (2026-09-22).

Partial gap noted: PP-20's *input flush before re-read* is not reproduced —
`SyncIOHandle` exposes no flush primitive; the tolerate-one-failure debounce (the
record-observable behavior) is implemented and the flush gap is documented in
`motor_common::comm`.

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

## PP-40 [HIGH] usb-2408 internal-waveform amplitude is 2× too large — FIX NEVER MERGED (re-opened as PP-86)

- **Rust:** `drivers/meascomp/usb-2408/src/wave_gen.rs:66,72-74,80,99` — uses full `amplitude` as peak (`offset + amplitude*sin`, square `offset±amplitude`, saw/random full-span).
- **C:** `drvMultiFunction.cpp:1542,1546,1549-1550,1554,1568-1570` — `AMPLITUDE` is peak-to-peak: `amplitude/2` about the offset.
- **Failure:** `WAVEGEN_AMPLITUDE=1V` produces ±1V (2Vpp) where C produces ±0.5V (1Vpp) — every internal waveform on the DAC is double the intended voltage (over-drive risk). `WAVEGEN_AMPLITUDE` record exists → live.
- **Family:** usb-2408 wave_gen (all four internal wave types).
- **Fix:** halve the amplitude about the offset, matching C's peak-to-peak semantics.

## PP-41 [MED] usb-2408 cluster (live, within ported wave/AO/AI scope) — FIX NEVER MERGED (partly re-landed on main; open subs re-opened as PP-60/81/88/89/90)

Each sub-finding has an existing record and is a real divergence:
- **WAVEGEN_ENABLE ignored** — `driver.rs:329-330` hardcodes first/last chan = 0..MAX; C iterates enabled channels and errors if none (`drvMultiFunction.cpp:1603-1636`).
- **Immediate AO write has no generator-running guard** — `driver.rs:115-131`; C refuses `ANALOG_OUT_VALUE` with `asynError` while `waveGenRunning_` (`:2131-2135`).
- **Analog-in read not gated on channel type** — `poller.rs:119-138` reads voltage on every channel incl. thermocouple; C `continue`s on `type != AI_CHAN_TYPE_VOLTAGE` (`:2764`), overwriting TC records with garbage.
- **Volts→TC switch doesn't reprogram TC type + open-detect** — `driver.rs:157-166`; C re-applies `AI_CFG_CHAN_TC_TYPE` + `setOpenThermocoupleDetect()` (`:1969-1982`).
- **Wavegen pulse width treated as 0..1 fraction, delay dropped** — `wave_gen.rs:83-92`; C uses time-based `pulseWidth/dwell` sample counts with a delay region (`:1556-1566`). (`WAVEGEN_PULSE_DELAY` itself is unported — no record.)
- LOW sub: digital_output direction gate (`driver.rs:393-404`↔`:2404`) FIXED `346be12`; TC-type/open-detect `isThermocouple` guard (`:167-183`↔`:2004,2019`) FIXED `13b3600`; wave-dig `-9999` bad-rate sentinel (`wave_dig.rs:152-154`↔`:1842-1846`) **DEFERRED — see below**; sin/saw period `numPoints-1` off-by-one (`wave_gen.rs:66,80`↔`:1545,1553`) FIXED `a48e827`.

**DEFERRED (wave-dig `ERR_BAD_RATE` → `-9999` dwell-actual sentinel):** C returns `-9999` for the actual-dwell when `ulDaqOutScan` reports `ERR_BAD_RATE`. Implementing the guard needs the uldaq `UlError::ERR_BAD_RATE` numeric value. `uldaq.h` is **not installed** anywhere on this machine (searched machine-wide 2026-07-19); the constant is absent from `drivers/meascomp/uldaq-sys/src/lib.rs` (a curated subset). Per the no-guessing rule the value was not hardcoded. **Blocked on:** the path to `uldaq.h`/libuldaq SDK header, or that header being installed — then add `ERR_BAD_RATE` to `uldaq-sys` and implement. Authorization alone is insufficient; the value's source is required.

## PP-42 [MED] usb-ctr cluster (live, within ported MCS/pulse/counter scope) — FIX NEVER MERGED (partly re-landed on main; open subs re-opened as PP-45/51/57/58/60/63)

- **MCS `SINGLEIO` threshold dropped** — `mcs.rs:164` always `SO_SINGLEIO`; C uses `SO_DEFAULTIO` and adds `SO_SINGLEIO` only when `dwell >= 0.01` (`drvUSBCTR.cpp:674,678-679`) — short-dwell high-rate scans lose data.
- **Pulse-generator input clamps dropped** — `pulse_gen.rs:17-26` passes frequency/duty/delay raw; C clamps to `[0.023,48e6]`/`[.0001,.9999]`/`[0,67.11]` (`:466-472`). `PULSE_*` records exist.
- **Model/`numCounters_` hardcoded to 8** — `scaler.rs:37` etc.; C derives 8 (CTR08) vs 4 (CTR04) and bounds loops (`:364-372,544,687`) — on a CTR04 the port drives nonexistent counters 4-7.
- **MCS start missing re-entry / already-complete guards** — `driver.rs:180-221`; C skips if `MCSRunning_` and short-circuits `currentPoint >= numTimePoints` (`:1189-1198`). (The `scalerRunning_` guard is moot — scaler unported.)
- **Elapsed real/live time never published** — `mcs.rs:241` computes `elapsed` only for the done check; C writes `mcaElapsedRealTime_`/`LiveTime_` each read (`:797-800`) — the `MCS:ElapsedReal` record (exists, I/O Intr) stays 0.
- LOW sub: `mca_num_channels` clamp to `maxTimePoints_` + actual-dwell writeback (`:1235-1240,711`); digital_output direction gate (`driver.rs:304-315`↔`:1348`); `counterReset_` `ulCClear` vs C `ulCLoad(CRT_LOAD,0)` (`:1124`, equivalence unconfirmed); MCS channel `range=0` vs `BIP10VOLTS` (benign for CTR).

---

# measComp coverage map — DRIVER-level complete (2/2), BOARD-level partial (2/18), 2026-07-20

Exhaustive check of whether any measComp subsystem remains unported.

**measComp has exactly TWO device drivers** (`measCompApp/src`): `drvMultiFunction.cpp`
(generic; runtime-detects ~17 board families by ProductID via a `boardEnums[]`
range/input-type table) and `drvUSBCTR.cpp` (USB-CTR counter/scaler/MCS). Everything
else in `src/` is `test_*` / `measCompAppMain` / `measCompDiscover`.

**Driver level — 2/2 ported and complete:**
- `drvUSBCTR.cpp` → `usb-ctr` — complete (scaler/MCS/MCA/pulse/counter all done; PP-42/43 fixed).
- `drvMultiFunction.cpp` → `usb-2408` — complete **for the USB-2408-2AO board config**.
  Verified its param set covers every subsystem `USB2408.substitutions` loads (AnalogInMode,
  AnalogIn, AnalogOut, BinaryDir/In/Out, Counter, Device, LongIn/Out, TemperatureIn,
  WaveformDig(N), WaveformGen(N)); also carries VoltageIn + TriggerMode.

**Board level — only 2 of 18 board configs exist as Rust IOCs.** `usb-2408` hardcodes the
USB-2408-2AO board (`params.rs`: `MAX_ANALOG_IN=8`, `MAX_ANALOG_OUT=2`, `NUM_IO_BITS=8`,
AO range `BIP10VOLTS`); it has NO board-family table, NO ProductID switch. The generic C
driver serves 16 more boards that have **zero Rust port** (no crate config, no
`.substitutions`, no iocBoot):
- Analog multifunction: USB-231, USB-1208, USB-1608G, USB-1608G-2AO, USB-1608HS-2AO,
  USB-1808, USB-3104, USB-3105, E1608.
- Temperature: TC32, ETC, USBTEMP, USBTEMP-AI.
- Digital/relay: EDIO24, USBERB24, USBSSR08.

**Subsystems the `usb-2408` port does NOT yet implement** (needed only by some other boards):
`measCompEncoder` (USB-1808 only) and `measCompPulseGen` (USB-1608G/1808/1608HS-2AO/1608G-2AO;
note `usb-ctr` DOES have `PULSE_*`, so the logic exists to borrow). VoltageIn/Trigger are
already present in `usb-2408`.

**Open scope question (NOT a defect):** whether these 16 board variants are in scope. If the
campaign's intent is one representative IOC per C driver, measComp is DONE at driver level.
If board variants are wanted, each needs board-specific constants + range/input-type tables +
`.substitutions`/iocBoot, plus Encoder/PulseGen subsystems for the boards that use them.
Awaiting user decision — no work started.

---

# measComp unported subsystems — COMPLETED (user decision 2026-07-19) — STATUS CORRECTED 2026-09-22: branches never merged, see wave 4

**Status: DONE.** Both completion ports finished and verified on their branches
(per-crate `fmt`/`clippy -D warnings`/`nextest` green; full-workspace pass still
owed before push).

- **branch `caucus/MXSR5DMPDZ/usb2408-completion-367adb0c-1`** (7 commits, nextest 12/12):
  waveform-gen user/internal arrays `fe0091b`; AO simultaneous sync-write + sync-master
  `782434d`; PP-40 amplitude peak-to-peak `8e2794a`; PP-41 cluster `2d7bbae`; LOW subs
  `13b3600`/`a48e827`/`346be12`. One LOW sub (wave-dig `ERR_BAD_RATE` sentinel) DEFERRED —
  see PP-41 block above.
- **branch `caucus/MXSR5DMPDZ/usbctr-completion-22f0c1fd-1`** (nextest 28/28):
  scaler via standard `scalerRecord`/`ScalerDriver` trait `5e11e4b`; MCS control scalars +
  ext-trigger + point0-action `0057eab`/`cde36a1`/`be5845c`; MCS prescale/channel-advance
  counter `50dce47`; MCA_DATA `read_int32_array` + 9 missing mca params + mca record/`asynMCA`
  device support + vendored `simple_mca.db` `a934a56`; MCSTimeWF/MCSAbsTimeWF array reads
  `5967d4b`; `measCompMCSWaveform.template` + 9 per-counter waveforms wired `f603139`. All
  PP-42 live fixes included.

The original scope list follows (all items now delivered):


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

## PP-43 [LOW] usb-ctr MCS time-waveform not recomputed on `MCA_DWELL_TIME` write — FIX NEVER MERGED (re-opened in PP-71)

- **Rust:** `drivers/meascomp/usb-ctr` computes the MCS time waveform only at `start_mcs`.
- **C:** `drvUSBCTR.cpp:1302-1304` — `writeFloat64` recomputes the time waveform and does
  array callbacks whenever `MCA_DWELL_TIME` is written, so `MCSTimeWF`/`MCSAbsTimeWF` update
  immediately on a dwell change even before the scan starts.
- **Failure:** a client that writes `MCA_DWELL_TIME` then reads the time-waveform records
  *before* starting the MCS sees stale/last-scan values under Rust; C shows the new dwell.
  These waveform records are now wired (usbctr-completion `f603139`), so this is a live,
  record-observable divergence — it was out of the completion's *listed* item scope and left
  as-is by the porter. Surfaced here as a follow-up candidate, not silently dropped.
- **Family:** single site (`writeFloat64` dwell branch). `writeInt32`'s `mcaNumChannels_`
  branch (`:1229-1236`) only clamps and does NOT call `computeMCSTimes` — so the num-channels
  write must NOT recompute either (adding it would diverge from C). Verified single-site.
- **Fixed:** commit `42d27b3` on branch `caucus/MXSR5DMPDZ/usbctr-completion-22f0c1fd-1`.
  Factored `mcs::compute_mcs_times(state, num_points, dwell)` (`time_buffer[i] = i*dwell` over
  `MCA_NUM_CHANNELS`, buffer-bounded; absolute-time buffer untouched); the `write_float64`
  dwell branch calls it then publishes `MCS_TIME_WF` via `set_float32_array` +
  `call_param_callbacks`. nextest `-p usb-ctr` 30/30 (2 new). Full-workspace pass still owed.

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

measComp / usb-2408 / usb-ctr — superseded: audited in the third wave (2026-07-19) and the fourth wave (2026-09-22) against `~/codes/measComp`.

### Scope-limited audits (siblings not yet covered)

- **newport** — only `smc100` audited; `agap`/`agilis`/`conex`/`esp300`/`hxp`/`mm3000`/`mm4000`/`pm500`/`pmnc`/`xps` not yet swept.
- **pi** — C-series (c862/c848/c844/c663/c662/c630) + E-series (prior round) audited; any other PI model not covered.
- **npoint** — only `c300` audited, not `lc400`.

---

# Fourth wave — measComp C-parity audit + live hardware, 2026-09-22

Codex-style C→Rust audit of `usb-ctr`, `usb-2408`, `meascomp`, `uldaq-sys`
and `iocs/meascomp/*` against measComp R4-4 HEAD
(`drvUSBCTR.cpp`, `drvMultiFunction.cpp`, `measCompApp/Db`, `iocBoot`),
libuldaq 1.2.1 source and the now-installed `/usr/local/include/uldaq.h`.
Five read-only panels (A usb-ctr MCS/MCA/scaler, B usb-ctr
counter/pulse/DIO/init, C usb-2408 AI/TC/AO/DIO, D usb-2408 waveform
gen/dig, E wrapper/FFI/db/st.cmd) produced 68 raw findings; 16 cross-panel
duplicates were merged, leaving PP-44..PP-95 (52). Rust side is `main`
`cc61da4`.

Both boards were exercised live on this host: USB-CTR08 `01DAB0FB`
(09db:0127) and USB-2408-2AO `01DA523D` (09db:00fe), each IOC on its own
CA port. No loopback wiring exists between outputs and inputs (timer
outputs → counter inputs: 0 counts at 1 kHz; AO → AI: no change), so
findings whose only observable is an output voltage or an external count
are marked "static".

**Status correction for PP-40..PP-43 and the "COMPLETED" block.** None of
the 16 commits cited there (`fe0091b 782434d 8e2794a 2d7bbae 13b3600
a48e827 346be12 5e11e4b 0057eab cde36a1 be5845c 50dce47 a934a56 5967d4b
f603139 42d27b3`) exists in this repository; the `caucus/MXSR5DMPDZ/*`
branches were never merged. `main`'s MCS/scaler/waveform features came
from the later independent PR #12. Every sub-item was re-checked on
`main`: those still open are re-opened below and tagged
**regression of PP-4x (fix never merged)**. Sub-items that do hold on
`main`: PP-41 WAVEGEN_ENABLE iteration, AI read gated on channel type
(`d7be812`), Volts→TC reprogram and isThermocouple guard (`546c66d`);
PP-42 MCS re-entry guard and elapsed-time publish. PP-42's
`ulCClear` vs `ulCLoad(CRT_LOAD,0)` question is settled as equivalent
(`CtrUsbCtrx.cpp:104-107`, `CtrUsb24xx.cpp:58-60`).

Class legend (per `port-translation-lessons.md`): **ref-indep** =
defect regardless of C; **ref-faithful** = adopt C's posture;
**contract** = C db/wire contract; **unimpl** = C feature absent.
**Live** = result on the attached hardware, or "static".

## usb-ctr — MCS / MCA / scaler

## PP-44 [HIGH] Scaler arm never loads counter-0 output-compare registers — FIXED
- **Rust:** `usb-ctr/src/scaler.rs:69-76` loads only `CRT_MAX_LIMIT` for presets > 0; no `CRT_OUTPUT_VAL0/VAL1` in the scaler path (only MCS, `mcs.rs:186-187`).
- **C:** `drvUSBCTR.cpp:1061-1081` `setScalerPresets` (called from `scalerArm_`, `:1174`) loads `CRT_OUTPUT_VAL0=0`, `CRT_OUTPUT_VAL1=PR1` on counter 0 every arm.
- **Impact:** C0O (documented gate for counters 1-7) never switches at PR1; after any MCS start VAL1 stays 0xFFFFFFFF. The preset is enforced only in software one poll late, so S2..S8 over-count.
- **Class:** unimpl. **Live:** static (needs C0O→C1GT wiring).

## PP-45 [HIGH] MCS/scaler mutual exclusion, already-complete start and NumChannels clamp absent — FIXED (partial regression of PP-42)
- **Rust:** `scaler_dev.rs:38-44,61-72` reset/arm have no MCS-running check; `driver.rs:183-185` starts on `value != 0 && !already_running` only; no `MCA_NUM_CHANNELS` branch; `poller.rs:74-86` services only the scaler when both run.
- **C:** `drvUSBCTR.cpp:1160,1171` scaler reset/arm skipped while `MCSRunning_`; `:1185-1188` MCS start refused (asynError) while `scalerRunning_`; `:1191-1198` already-complete start toggles `mcaAcquiring_` 1→0 without starting; `:1235-1240` clamp to `maxTimePoints_`; `:1184` start on any value.
- **Impact:** a scaler count during an MCS zeroes all counters mid-bin then hits `ERR_ALREADY_ACTIVE`; StartAll after a completed, un-erased run re-acquires over the data; `NuseAll > MaxChannels` is not clamped.
- **Class:** unimpl. **Live:** confirmed — after a completed 300-point run, `StartAll` restarted from 0 (CurrentChannel 50 at 0.5 s).

## PP-46 [HIGH] `start_mcs` failure leaves MCA_ACQUIRING=1 forever — FIXED
- **Rust:** `mcs.rs:125-138,189-198` return early via `?`, `running` stays false; `driver.rs:206-223` then sets `mca_acquiring=1` unconditionally; the poller calls `read_mcs` only when `running` (`poller.rs:79`).
- **C:** `drvUSBCTR.cpp:556-560,704-709` log and continue, `:713-714` set `MCSRunning_`; the next `readMCS` sees SS_IDLE (`:751-753`) and clears `mcaAcquiring_` (`:802-806`).
- **Impact:** any rejected scan wedges HardwareAcquiring and the Acquiring busy until a manual StopAll.
- **Class:** ref-indep. **Live:** confirmed — `Dwell=1e-6` → "uldaq error 22", HardwareAcquiring/Acquiring stuck at 1 for 2 s until StopAll.

## PP-47 [MED] Scaler counter scan at 10 kHz instead of 100 Hz; SS_IDLE ends the count — FIXED
- **Rust:** `scaler.rs:79` `rate = 10000.0` into `ulCInScan(…,20,…, SO_CONTINUOUS|SO_SINGLEIO, CINSCAN_FF_CTR64_BIT)`; `scaler.rs:144` stops and reports done on `status == SS_IDLE`.
- **C:** `drvUSBCTR.cpp:892-893,933-935` `rate = 100`; `readScaler` (`:976-986`) completes only on a preset.
- **Impact:** 100× the SINGLEIO USB transfers; a scan overrun that goes idle is reported to the scalerRecord as a completed count with partial data.
- **Class:** ref-faithful. **Live:** log shows "Scaler started, rate=10000 Hz"; completion not observable (counter 0 unwired).
- **Also fixed with it:** C readScaler takes the counts of the first complete ring set that reaches a preset (`:970-987`); the port always took the last set.

## PP-48 [MED] `read_mcs` early returns skip SS_IDLE / PresetReal detection; PresetReal latched at start — FIXED
- **Rust:** `mcs.rs:226-232` returns on status error, `:234-236` returns while `current_total_count == 0`, both before the done test `:258-261`; `driver.rs:199-201` reads `MCA_PRESET_REAL_TIME` only at start.
- **C:** `drvUSBCTR.cpp:751-753,788-794` evaluate SS_IDLE and `elapsed >= presetReal` every poll, re-reading `presetReal`.
- **Impact:** with an external trigger/clock not yet seen, PresetReal never stops the run; mid-run PresetReal changes are ignored.
- **Class:** ref-faithful. **Live:** static (no trigger source).

## PP-49 [MED] MCA_STOP_ACQUIRE does not drain transferred points — FIXED
- **Rust:** `driver.rs:225-232` → `mcs::stop_mcs` (`mcs.rs:270-277`) calls `ulDaqInScanStop` only.
- **C:** `drvUSBCTR.cpp:853-858` forced stop runs a final `readMCS()` (copies points, updates CurrentPoint/elapsed) before `ulDaqInScanStop`.
- **Impact:** up to one poll period of points is lost and CurrentChannel is stale after StopAll.
- **Class:** unimpl. **Live:** static.

## PP-50 [MED] `eraseMCS` does not publish resets and wipes time bases C keeps — FIXED
- **Rust:** `mcs.rs:67-74`, `driver.rs:233-236` zero buffers incl. `time_buffer`/`abs_time_buffer`, set `current_point=0` without setting `MCS_CURRENT_POINT`, elapsed params, or `start_time`.
- **C:** `drvUSBCTR.cpp:823-843` sets CurrentPoint 0, elapsed live/real/counts 0 on all addrs with callbacks, resets `startTime_`, clears only `MCSBuffer_`.
- **Impact:** after EraseAll, CurrentChannel/ElapsedReal keep old values; an erase mid-run does not restart the PresetReal clock.
- **Class:** ref-faithful. **Live:** confirmed — after EraseAll, CurrentChannel=100 and ElapsedReal=1.037 unchanged.

## PP-51 [MED] SO_SINGLEIO forced at every dwell — FIXED (regression of PP-42, fix never merged)
- **Rust:** `mcs.rs:171` `let mut options = SO_SINGLEIO;`.
- **C:** `drvUSBCTR.cpp:674-679` `SO_DEFAULTIO`, `SO_SINGLEIO` only when `dwell >= 0.01` (libuldaq picks BLOCKIO above 1 kHz, `DaqIUsbBase.cpp:160`).
- **Impact:** short-dwell MCS does one USB transfer per scan, risking overrun.
- **Class:** ref-faithful. **Live:** static.

## PP-52 [MED] TRIGGER_MODE is a bool gating SO_EXTTRIGGER; `ulDaqInSetTrigger` never called — FIXED (regression, fix never merged)
- **Rust:** `meascomp_mcs.template:173-179` bo Internal/External; `driver.rs:195` `trigger_mode != 0`; `mcs.rs:175-177` adds `SO_EXTTRIGGER` only then; no TRIGGER_MODE branch in `write_int32`; `meascomp/src/counter.rs:139` `daq_in_set_trigger` has no caller.
- **C:** `measCompMCS.template:293-305` mbbo raw 0/1/6/7; `drvUSBCTR.cpp:1129-1148` → `TRIG_POS_EDGE/NEG_EDGE/HIGH/LOW` via `ulDaqInSetTrigger`; `:680-681` always `SO_EXTTRIGGER`.
- **Impact:** mode 0 free-runs in Rust but waits for a rising edge in C; mode 1 is rising in Rust, falling in C; level modes unreachable.
- **Class:** ref-faithful. **Live:** consistent — `TrigMode=Internal` acquired immediately with the trigger input unconnected (C would wait).

## PP-53 [MED] Point0Action Skip and external-advance prescale unimplemented — FIXED (regression, fix never merged)
- **Rust:** `driver.rs:202-205` `point0_no_clear = action != 0` (Skip→NoClear, no `numPoints+1`, no drop); `mcs.rs:90-92,104` discards `prescale` under a comment wrongly claiming C ignores it; `MCS_PRESCALE_COUNTER` never read; no `Point0Action`/`PrescaleCounter` records.
- **C:** `drvUSBCTR.cpp:607,761-763` Skip; `:579-603` prescale counter programming; `measCompMCS.template:242-274`.
- **Impact:** `MCS:Prescale` has no device effect; Skip mode unreachable.
- **Class:** unimpl. **Live:** confirmed PVs absent.

## PP-54 [MED] drvMca param contract broken; mca record not registered — FIXED (regression, fix never merged)
- **Rust:** `params.rs:114-118` renamed params (`MCA_CH_ADVANCE_SOURCE`, `MCA_PRESET_REAL_TIME`, `MCA_ELAPSED_REAL_TIME`, …) and 11 drvMca params absent; `usb-ctr-ioc/src/main.rs:57-69` does not register `mca`.
- **C:** `drvUSBCTR.cpp:331-351` creates all 21 drvMca.h params; C st.cmd offers `simple_mca.db` (`DTYP=asynMCA`); `devMcaAsyn` resolves all 21 (`dev_mca_asyn.rs:221-234`).
- **Impact:** the documented optional mca-record configuration cannot load.
- **Class:** contract. **Live:** static.

## PP-55 [MED] MCS readout/busy record protocol diverges from C — FIXED
- **Rust:** `meascomp_mcs.template:75-101` ReadAll/ReadAllOnce FLNK → ReadFanout → ClearAcquiring; ReadAll no SCAN/SDIS; `:19-24` EraseAll no FLNK; `:54-58` SetAcquiring no `VAL 1`, no FLNK to SetClientWait (`:131-137` unreachable).
- **C:** `measCompMCS.template:83-90` ReadAll SCAN "1 second", SDIS Acquiring; `:12-19` EraseAll FLNK ReadAllOnce; `:30-45,150-158` StartAll→SetAcquiring(VAL 1)→SetClientWait; `USBCTR_SNL.st` clears Acquiring only on HardwareAcquiring 1→0.
- **Impact:** a ReadAll mid-run releases the Acquiring busy early (`caput -c` returns before data); no 1 Hz spectrum refresh; spectra stale after erase; ClientWait never raised; on a fresh IOC StartAll writes 0 into Acquiring.
- **Class:** contract. **Live:** confirmed — `ReadAll` mid-run set Acquiring=Done while HardwareAcquiring=Acquiring.
- **Found while fixing (live):** a StartAll on a run that already has all its points wedged Acquiring at 1. The driver pulses MCA_ACQUIRING 1 → 0 inside the write (C `:1191-1198`), which upstream's SNL sees through CA monitors; epics-rs asyn-rs delivers plain I/O Intr through a coalescing mailbox (`asyn-rs-0.30.0/src/interrupt.rs:214-215`), so the record only ever sees the final 0 and AcquireDone never fires. The db now raises Acquiring before the start and reads MCA_ACQUIRING back after it (StartSeq/StartCheck). The framework deviation from C asyn's per-record ring stays open in epics-rs.

## PP-56 [LOW] MCS readout element counts swapped vs C — FIXED
- **Rust:** `driver.rs:267` MCA_DATA `n = min(buf, src, num_channels)`; `:298-303` AbsTimeWF `n = min(…, current_point)` (comment "as C readMCS reports them" is wrong).
- **C:** `drvUSBCTR.cpp:1403-1406` MCA_DATA `min(numRead, currentPoint)`, min 1; `:1473,1481-1482` AbsTimeWF `min(nElements, mcaNumChannels)`.
- **Impact:** spectrum NORD = NuseAll mid-run (C: acquired points); AbsTimeWF NORD the reverse.
- **Class:** ref-faithful. **Live:** not distinguishable after a completed run (both 100).

## PP-57 [LOW] MCS actual dwell never written back; `Dwell_RBV` missing — FIXED (regression of PP-42 LOW, fix never merged)
- **Rust:** the in/out `rate` from `ulDaqInScan` (`mcs.rs:189-198`) is only logged; no `MCS:Dwell_RBV` record.
- **C:** `drvUSBCTR.cpp:711` `setDoubleParam(mcaDwellTime_, 1./rate)`; `measCompMCS.template:122-127` Dwell_RBV.
- **Impact:** the clock-quantized dwell is never shown.
- **Class:** unimpl. **Live:** confirmed PV absent.
- **Also fixed with it:** `start_mcs` replaced a dwell ≤ 0 with a 1 kHz rate; C passes `1/dwell` and lets libuldaq reject it, so the reported dwell is always the rate actually used.

## usb-ctr — pulse generators, counters, DIO, init

## PP-58 [MED] Pulse-generator input clamps missing — FIXED (regression of PP-42, fix never merged)
- **Rust:** `pulse_gen.rs:17-21,28` passes frequency (1000 Hz fallback for period ≤ 0), duty and delay raw to `ulTmrPulseOutStart`.
- **C:** `drvUSBCTR.cpp:465-472` clamps frequency to [0.023, 48e6], duty to [.0001, .9999], delay to [0, 67.11].
- **Impact:** values C clamps are rejected by libuldaq (`TmrDevice.cpp:61-75`), the generator is left stopped while Run reads Run.
- **Class:** ref-faithful. **Live:** confirmed — Frequency 1e9 / DutyCycle 1.5 / Delay 100 and Frequency 0.001 all passed through (RBVs 1e-9, 1.5, 100) and returned "uldaq error 58: Invalid frequency specified".

## PP-59 [MED] Timers not stopped at init; "running" inferred from the PULSE_RUN setpoint — FIXED
- **Rust:** constructor `driver.rs:38-104` never calls `ulTmrPulseOutStop`; running state is the Run param set before the hardware call (`:121,128,318-322`).
- **C:** `drvUSBCTR.cpp:427-430` stops all 4 timers at construction; `pulseGenRunning_[]` set only after a successful start (`:490`), gating stop (`:1106`) and restarts (`:1112,1295`).
- **Impact:** after an IOC restart a generator left running keeps pulsing while Run shows Stop; after a rejected start, a later Period/Duty write starts the output without Run being re-written.
- **Class:** ref-faithful. **Live:** static (output not observable).

## PP-60 [MED] DIGITAL_OUTPUT writes ignore the direction mask (both drivers) — FIXED (regression of PP-41/PP-42 LOW, fix never merged)
- **Rust:** `usb-ctr/src/driver.rs:382-393` and `usb-2408/src/driver.rs:677-688` call `ulDBitOut` for every bit in `mask`.
- **C:** `drvUSBCTR.cpp:1344-1348` and `drvMultiFunction.cpp:2383-2413` write only `mask & outMask & direction` (2408: one `ulDOut` when the whole port is output).
- **Impact:** CTR: writes to input bits fail with error 51 and pollute LastErrorMessage; 2408: bits C treats as inputs are driven (see PP-62).
- **Family:** both drivers' `write_uint32_digital` DIGITAL_OUTPUT branch; no other site.
- **Class:** ref-faithful. **Live:** confirmed on CTR — `Lo=165` with all bits In → "digital_bit_out error: uldaq error 51".

## PP-61 [MED] USB-CTR boot DIO directions differ from C — FIXED
- **Rust:** `usb-ctr-ioc/st.cmd:49-52` `VAL=0` for Bd5..Bd8; `driver.rs:76-80` forces `ulDConfigPort(AUXPORT, DD_INPUT)` in the constructor.
- **C:** `USBCTR.substitutions:56-59` Bd5..Bd8 `VAL=1`; constructor (`:255-439`) never configures direction.
- **Impact:** outputs unusable by default; every restart tri-states bits wired as outputs until the Bd PINI runs.
- **Class:** ref-faithful. **Live:** confirmed Bd1..Bd8 all In on a fresh IOC.

## PP-62 [MED] USB-2408 digital-direction model missing — FIXED
- **Rust:** `usb-2408/src/driver.rs:689-708` a direction write on the non-configurable port only sets LAST_ERROR_MESSAGE; `usb-2408-ioc/st.cmd:60-61` loads no Bd records.
- **C:** AUXPORT is `DPIOT_NONCONFIG` (`DioUsb24xx.cpp:15`); a direction write does `ulDBitOut(port,i,0)` per masked bit ("set open collector output to 0", `drvMultiFunction.cpp:2369-2376`) and stores the mask used by the output gate; `USB2408.substitutions:48-60` Bd1-4=In, Bd5-8=Out, PINI.
- **Impact:** Bo1-4/Lo bits 0-3 drive open-drain outputs C never drives; C releases bits 0-3 at iocInit, Rust leaves the latched state.
- **Class:** ref-faithful. **Live:** confirmed `Lo=15` pulls DIO0-3 low (Li 255→240).

## PP-63 [MED] No model detection: MODEL param/record missing, counters hardcoded to 8 — FIXED (regression of PP-42, fix never merged)
- **Rust:** `params.rs:7` `MAX_COUNTERS=8` used by `scaler_dev.rs:83-85`, `scaler.rs:37,85,115`, `poller.rs:87`, `mcs.rs:116-124`; no MODEL param.
- **C:** `drvUSBCTR.cpp:362-372` MODEL + `numCounters_` 8 (CTR08) / 4 (CTR04), bounds `:544,687,895,921`, `scalerChannels_` (`:418`); `measCompMCS.template:282-291` Model mbbi.
- **Impact:** on a CTR04 the poller calls `ulCIn(4..7)` every cycle (ERR_BAD_CTR), the scaler never starts, the MCS wedges (PP-46).
- **Class:** unimpl. **Live:** `USBCTR:MCS:Model` not found; CTR04 effects static.

## PP-64 [LOW] Derived calc records miss CP/FLNK links (Width, generator dwell) — FIXED
- **Rust:** `meascomp_pulse_gen.template:94-99,113-118` CalcWidth/Width_RBV CP on Period_RBV only (DutyCycle_RBV NPP); `meascomp_wave_gen.template:88-93,116-121` CalcUserDwell/CalcIntDwell INPB NumPoints NPP.
- **C:** `measCompPulseGen.template:87-95,127-133` DutyCycle FLNK CalcWidth, Width_RBV CP on both; `measCompWaveformGen.template:158,206` NumPoints CP.
- **Impact:** Width/Width_RBV stay 0 from boot and stale after DutyCycle-only changes; NumPoints changes don't rescale dwell.
- **Class:** ref-indep. **Live:** confirmed — at startup Period 0.001, DutyCycle 0.5, Width/Width_RBV 0.
- **Found while fixing (live):** with NumPoints CP, CalcIntDwell/CalcUserDwell fired at iocInit while their frequency input was still 0 and wrote a zero dwell to the driver, so the generator failed every start with ERR_BAD_RATE. C's calcs are saved by their init order; the four dwell/frequency calcouts now write only a non-zero result.

## Cross-driver (both IOCs / shared db)

## PP-65 [HIGH] Autosave non-functional since `9b794e6`; both IOCs share one save path — FIXED
- **Rust:** `usb-ctr-ioc/src/main.rs:31-34`, `usb-2408-ioc/src/main.rs:30-33` set `MEASCOMP` to `CARGO_MANIFEST_DIR/..` (`iocs/meascomp`); both st.cmd `set_requestfile_path("$(MEASCOMP)")` / `set_savefile_path("$(MEASCOMP)/autosave")` / `auto_settings.sav`; `auto_settings.req` lives in each IOC dir. Before `9b794e6` `MEASCOMP` was the IOC dir.
- **C:** `iocBoot/save_restore.cmd:23,28-34` per-IOC `autosave/` relative to each iocBoot dir.
- **Impact:** nothing is ever saved or restored (the "restore across a restart" feature `0b243c7` is dead); when launched from an IOC dir both IOCs would write the same `auto_settings.sav` and restore each other's PVs.
- **Class:** ref-indep. **Live:** confirmed — after ~9 min of changes on both IOCs with a 30 s monitor set, `iocs/meascomp/autosave/` is empty and `iocs/meascomp/*.req` does not exist.
- **Second cause (found while fixing):** epics-rs 0.30 snapshots the autosave configuration when the script's `iocInit()` runs `perform_build` (`epics-base-rs` `ioc_app.rs:1088,1222`), so a `create_monitor_set` after `iocInit()` — C's usual order — is never scheduled. Same ordering in `iocs/d435i-ioc/st.d435i.cmd`, `st.d405.cmd`; all four moved before `iocInit()`. The framework-side deviation from C autosave stays open in epics-rs.

## PP-66 [MED] Output records lack the PINI C relies on; restored values never reach the driver — FIXED (FAMILY)
- **Rust:** no PINI on `meascomp_pulse_gen.template:1` Run (also no OSV MINOR, not in `auto_settings.req`) and `:60` IdleState; `meascomp_counter.template:7` Reset (no `VAL 1`); `meascomp_binary_out.template:1` Bo (no PHAS 2); `meascomp_analog_out.template:1` Ao (no PHAS 2/VAL 0); `meascomp_temperature.template:34,41` Filter/OpenTCDetect; `meascomp_wave_dig.template:69-102` ExtTrigger/ExtClock/Continuous/AutoRestart/BurstMode; `meascomp_wave_gen.template:143-162` ExtTrigger/ExtClock/Continuous. epics-rs pass-1 restore writes VAL without processing (`save_set.rs:342-447`).
- **C:** all `PINI YES`: `measCompPulseGen.template:8,181`, `measCompCounter.template:8` (VAL 1), `measCompBinaryOut.template:1` (PHAS 2), `measCompAnalogOut.template:2-4` (PHAS 2, VAL 0), `measCompTemperatureIn.template:49,60`, `measCompWaveformDig.template:135-213`, `measCompWaveformGen.template:216-247`; `measCompPulseGen_settings.req:6` saves Run.
- **Impact (once PP-65 is fixed):** restored Continuous/ExtTrigger/OpenTCDetect/IdleState display but the driver keeps 0; counters not zeroed at boot; DACs and DIO outputs not driven at boot; running pulse generators not resumed.
- **Family:** every output record listed in either `auto_settings.req`; a test asserting PINI on each would close it.
- **Class:** contract. **Live:** static (autosave itself broken, PP-65).

## PP-67 [MED] Write failures never return asynError, so records never alarm (both drivers) — FIXED (FAMILY)
- **Rust:** `usb-ctr/src/driver.rs:238-247,360-369,418-427` and `usb-2408/src/driver.rs:160-181` (`finish_write`), `:619,722` log to LAST_ERROR_MESSAGE then return `Ok(())`; restart stop errors dropped (`let _ = pulse_gen::stop`, `:132,325`).
- **C:** `drvUSBCTR.cpp:1254,1316,1368`, `drvMultiFunction.cpp:2211` et al. `return (status==0) ? asynSuccess : asynError`.
- **Impact:** failed pulse start, counter reset, DIO write or scan start leaves the record NO_ALARM; the failure is visible only in the `LastErrorMessage` waveform (a Rust addition — C never writes `lastErrorMessage_`), which is also never cleared on success.
- **Class:** contract. **Live:** confirmed — every failure in this session (errors 15, 16, 22, 51, 56, 58) left the written record NO_ALARM; LastErrorMessage kept the last error after later successful writes.

## PP-68 [LOW] Zero-value writes ignored where C acts on any write — FIXED
- **Rust:** `usb-ctr/src/driver.rs:171-177` and `usb-2408/src/driver.rs:258` COUNTER_RESET only `if value != 0`; `usb-2408/src/driver.rs:282` ANALOG_OUT_SYNC_WRITE; `:410` WAVEDIG_READ_WF.
- **C:** `drvUSBCTR.cpp:1119-1126`, `drvMultiFunction.cpp:2063-2071,2159-2161` act on every write.
- **Impact:** `caput …Reset 0` / `SyncWrite 0` do nothing in Rust.
- **Class:** ref-faithful. **Live:** static.

## PP-69 [LOW] POLL_TIME_MS reports work time, not cycle time; sub-ms POLL_SLEEP_MS truncated (both drivers) — FIXED
- **Rust:** `usb-ctr/src/poller.rs:62,175-177,182`, `usb-2408/src/poller.rs:69,256-258,263`.
- **C:** `drvUSBCTR.cpp:1494,1501-1503,1539`, `drvMultiFunction.cpp:2599-2601,2851` (cycle time incl. sleep; exact float sleep).
- **Impact:** PollTimeMS reads far below PollSleepMS; `PollSleepMS=0.5` busy-loops.
- **Class:** contract. **Live:** confirmed — PollSleepMS 50, PollTimeMS 1.65 (CTR) / 28.6 (2408).

## PP-70 [MED] Absolute-time waveforms use the Unix epoch instead of the EPICS epoch (both drivers) — FIXED
- **Rust:** `usb-2408/src/wave_dig.rs:215,275-280`, `usb-ctr/src/mcs.rs:279-284` `duration_since(UNIX_EPOCH)`.
- **C:** `drvMultiFunction.cpp:2716-2726`, `drvUSBCTR.cpp:782` `now.secPastEpoch + nsec/1e9` (1990 epoch).
- **Impact:** every AbsTimeWF element is +631152000 s off C.
- **Class:** contract. **Live:** confirmed — `WaveDigAbsTimeWF[0]` and `MCS:AbsTimeWF[0]` = 1.79004e9 (Unix now 1790036774; EPICS epoch now 1158884774).

## PP-71 [LOW] Time-base waveforms not recomputed on dwell/point-count writes (both drivers) — FIXED (includes PP-43, fix never merged)
- **Rust:** `usb-ctr/src/driver.rs:307-370` no `MCA_DWELL_TIME` branch (time WF built only in `start_mcs`, `mcs.rs:205`); `usb-2408` WAVEDIG_TIME_WF built only on a successful Run (`wave_dig.rs:159-162`); WaveGenUser/IntTimeWF Passive, no PINI, never computed (`meascomp_wave_gen.template:123-135`, `driver.rs:624-650`).
- **C:** `drvUSBCTR.cpp:1302-1304` → `computeMCSTimes` (`:873-885`); `drvMultiFunction.cpp:2097-2105,2308-2310` `computeWaveDigTimes`, `:2196-2199,2313-2316` `computeWaveGenTimes`, all with I/O Intr callbacks.
- **Impact:** time axes are zero/stale until a run and don't follow Dwell/NumPoints changes; generator time axes never populate.
- **Class:** ref-faithful. **Live:** confirmed — after `WaveGenUserDwell=0.002`, `WaveGenUserTimeWF`/`IntTimeWF` read all zeros.

## PP-72 [LOW] Empty uniqueID opens the first enumerated device — FIXED
- **Rust:** `meascomp/src/device.rs:37-38` `if unique_id.is_empty() { descriptors[0].clone() }`.
- **C:** `measCompDiscover.cpp:169-182,208` exact match or -1; constructors abort (`drvUSBCTR.cpp:274-278`, `drvMultiFunction.cpp:838-842`).
- **Impact:** with both boards attached, an empty UNIQUE_ID binds either IOC to whichever board enumerates first.
- **Class:** ref-indep. **Live:** static (would contend for the attached boards).

## PP-73 [LOW] No `report()` override in either driver — FIXED
- **Rust:** neither `usb-ctr` nor `usb-2408` implements `PortDriver::report`.
- **C:** `drvUSBCTR.cpp:1545-1570`, `drvMultiFunction.cpp:2856-2915`.
- **Impact:** `asynReport` shows no pulse-gen/scaler/MCS/waveform runtime state.
- **Class:** unimpl. **Live:** static.

## PP-74 [LOW] Wrapper C-string buffers are `[0i8; N]` — FIXED
- **Rust:** `meascomp/src/error.rs:15`, `device.rs:89,106`.
- **C:** libuldaq supports Raspberry Pi OS (`README.md:15`), where `c_char` is `u8`.
- **Impact:** the crate does not compile on aarch64/armv7 Linux.
- **Class:** ref-indep. **Live:** static.

## PP-75 [LOW] C records missing from the Rust IOCs — FIXED
- **Rust:** no `Bo<n>_RBV`, `WaveGen<n>InternalWF`, `MCS:Dwell_RBV`, `MCS:Model`, `MCS:SNL_Connected`, `MCS:Asyn`, `Ao<n>Return`/`Ao<n>Pulse`, `Ai<n>Rate`.
- **C:** `measCompBinaryOut.template:15-25`, `measCompWaveformGenN.template:15-22`, `measCompMCS.template:5-10,122-127,282-291,320-322`, `measCompAnalogOut.template:27-47`, `measCompAnalogIn.template:31-37`.
- **Impact:** C OPIs/clients lose these readbacks. Driver-side gaps behind some of them are PP-57, PP-63, PP-79, PP-93.
- **Class:** unimpl. **Live:** confirmed not found: `USBCTR:MCS:Model`, `USBCTR:Bo1_RBV`, `USB2408:Bo1_RBV`, `USB2408:WaveGen1InternalWF`, `USB2408:Ai1Rate`.

## PP-76 [LOW] Record names and state strings diverge from the C templates — OPEN
- **Rust:** `meascomp_wave_gen.template:16` `WaveGenFreq`; `meascomp_wave_gen_n.template:4` `$(R)WaveType` ("User/Sin/…"); bo states "Off/On" on WaveDig/WaveGen ExtTrigger/ExtClock/AutoRestart/BurstMode, WaveGen Enable, Ti OpenTCDetect; TCType "J".."N".
- **C:** `measCompWaveformGen.template:78` `Frequency`; `measCompWaveformGenN.template:39-56` `$(R)Type` ("Sin wave", …); "Internal/External", "Disable/Enable"; "Type J".."Type N".
- **Impact:** C OPIs, scripts and autosave files fail against the Rust IOC.
- **Class:** contract. **Live:** confirmed `USB2408:WaveGen1Type` and `USB2408:WaveGenFrequency` not found.

## PP-77 [LOW] Record types/menus/defaults changed (digitizer channels, TC filter, MCS dwell) — OPEN
- **Rust:** `meascomp_wave_dig.template:23-39` FirstChan/NumChans longout, NumChans default 8 (`driver.rs:67`); `meascomp_temperature.template:34-39` Filter "Off"(0)/"On"(1); `meascomp_mcs.template:143-149` Dwell VAL 0.001.
- **C:** `measCompWaveformDig.template:20-67` mbbo, NumChans raw = index+1, default 1 channel; `measCompTemperatureIn.template:49-58` "Filter"(0)/"No filter"(0x400); `measCompMCS.template:114-120` Dwell 0.1.
- **Impact:** numeric NumChans writes differ by one; the default 8-channel × 1 ms digitizer run always fails (PP-90) where C's 1-channel default runs.
- **Class:** contract. **Live:** confirmed default NumChans=8; `NumChans 8, Dwell 0.001` → "uldaq error 22".

## usb-2408 — AI / TC / AO / DIO

## PP-78 [HIGH] Analog-output record takes raw DAC counts; C's takes volts — FIXED
- **Rust:** `meascomp_analog_out.template:1-5` `ao` asynInt32 with no LINR/EGUL/EGUF/DRVL/DRVH; no `get_bounds` override (asyn-rs default (0,0)); `driver.rs:263-279` `ulAOut(…, AOUT_FF_NOSCALEDATA, value)`.
- **C:** `drvMultiFunction.cpp:1920-1928` `getBounds` 0..65535; `measCompAnalogOut.template:1-17` LINR LINEAR; `USB2408.substitutions:135-137` EGU/DRV ±10.
- **Impact:** `caput Ao1 5` gives +5 V in C and DAC code 5 (≈ −9.998 V) in Rust; `caput Ao1 0` drives −10 V; any negative volts fails with ERR_BAD_DA_VAL; no ±10 V clamp; TweakVal steps in counts. Fix needs both the record fields and a `get_bounds` for ANALOG_OUT_VALUE.
- **Class:** contract. **Live:** confirmed — every negative put (−0.5, −9.9, −10) returned "uldaq error 56: Invalid D/A output value specified"; positive volts were accepted as counts. Both DACs were left at code 32768 (≈0 V) after testing.

## PP-79 [HIGH] AI data rate never programmed: every conversion at 3750 S/s instead of 60 S/s — FIXED
- **Rust:** ANALOG_IN_RATE created (`params.rs:132`) but `write_int32` has no branch; `meascomp/src/analog_in.rs:43` `ai_set_config_dbl` has no caller; no `Ai<n>Rate` record; `auto_settings.req:4` notes it dropped.
- **C:** `drvMultiFunction.cpp:1994-2001` `ulAISetConfigDbl(AI_CFG_CHAN_DATA_RATE, ch, value)` on the 2408 (`:1140-1146`); `measCompAnalogIn.template:31-37` Rate PINI VAL 60.
- **Impact:** libuldaq default `CHR_3750` (`AiUsb24xx.cpp:986-987`) applies to `ulAIn`, `ulTIn` and the scan queue: no 50/60 Hz rejection, higher noise, and a much shorter digitizer minimum period than C allows.
- **Class:** unimpl. **Live:** static (no call site); noise not compared against spec.

## PP-80 [MED] AI/TC records lose C's averaging and forced callbacks; two `ulAIn` per channel — FIXED
- **Rust:** `meascomp_analog_in.template:1-6`, `meascomp_temperature.template:1-6` asynFloat64 I/O Intr, no averaging; `poller.rs:238-249` post only changed values; `poller.rs:134-151` two `ulAIn` per voltage channel (NOSCALEDATA + scaled; ANALOG_IN_VALUE has no record).
- **C:** `measCompAnalogIn.template:1-12` asynInt32Average, `measCompTemperatureIn.template:1-7` asynFloat64Average, SCAN 1 second; forced callback every poll (`:2784-2785,2832-2833`); one `ulAIn(NOSCALEDATA)` per channel (`:2779`).
- **Impact:** single samples at poll rate instead of a 1 s mean (≈√20 more noise, ≈20× monitor traffic); unchanging values (−9999, railed inputs) freeze their timestamp; double ADC time per sweep.
- **Class:** contract. **Live:** confirmed — `camonitor USB2408:Ai1` delivered 38 updates in 3 s (C: 3).

## PP-81 [MED] Immediate AO write not refused while the generator runs — FIXED (regression of PP-41, fix never merged)
- **Rust:** `driver.rs:263-279` calls `ulAOut` without checking `wave_gen.running`.
- **C:** `drvMultiFunction.cpp:2130-2135` refuses with `asynError` and never calls `ulAOut`.
- **Impact:** libuldaq rejects the write (ERR_ALREADY_ACTIVE, `AoDevice.cpp:282-283`), so the device is unaffected, but the record shows no alarm and its VAL no longer matches the DAC.
- **Class:** ref-faithful. **Live:** confirmed — `Ao1=40000` during a continuous run → "uldaq error 16: A background process is already in progress", record NO_ALARM.

## PP-82 [MED] USB-2408 port declared non-blocking; C declares ASYN_CANBLOCK — DEFERRED (blocked on epics-rs)
- **Rust:** `driver.rs:48-52` `can_block: false`; writes do USB I/O and take the device mutex the poller holds for its whole sweep (`poller.rs:93-173`).
- **C:** `drvMultiFunction.cpp:821` `ASYN_MULTIDEVICE | ASYN_CANBLOCK` (USBCTR deliberately omits it, `drvUSBCTR.cpp:259-260`).
- **Impact:** a CA put or scan thread blocks for a poll sweep (tens of ms; ≈300 ms once PP-79 sets 60 S/s), and blocking USB I/O runs on a tokio worker.
- **Class:** contract. **Live:** static.
- **Why deferred (tried, reverted):** with `can_block: true` the port runs on asyn-rs's async write completion, which in 0.30 (a) never turns a failed write into WRITE_ALARM (`AsynAsyncWriteCompletion::wait`, `asyn-rs-0.30.0/src/adapter.rs:1017-1027`) and (b) discards a driver readback that arrives while the record is still PACT (`adapter.rs:406-418`). Live: a refused `Ao1` write during generation raised no alarm, and a refused `WaveGenRun`/`WaveDigRun` stayed at Run with `caput -c` timing out. That breaks PP-67 and every busy record's failure path, which is worse than a put blocking for one poll sweep. Re-apply once epics-rs completes async writes with the error and the pending readback, as C asyn's second process pass does.

## PP-83 [LOW] Default thermocouple type K; C defaults to J — FIXED
- **Rust:** `driver.rs:74` `TC_K`; `meascomp_temperature.template:19` `VAL 1`.
- **C:** `drvMultiFunction.cpp:1281-1283` `TC_TYPE_J`; template index 0 = Type J; libuldaq default `TC_J`.
- **Impact:** a J sensor on a fresh IOC reads tens of °C off.
- **Class:** ref-faithful. **Live:** confirmed `Ti1TCType` = K on a fresh IOC.

## PP-84 [LOW] AiMode offers "Pseudo-diff", which the USB-2408 rejects — FIXED
- **Rust:** `meascomp_analog_in_mode.template:8` `TWVL 3` passed straight to `ulAIn` (`poller.rs:72-74,134-147`).
- **C:** `measCompAnalogInMode.template:7-10` only 0/1; `drvMultiFunction.cpp:1989` maps to DIFF/SE; device supports only those (`AiUsb24xx.cpp:66-67`).
- **Impact:** every voltage read fails with ERR_BAD_INPUT_MODE and the Ai records freeze on their last value.
- **Class:** ref-indep. **Live:** confirmed — `AiMode=2` → "uldaq error 15: Invalid input mode specified" on every channel, every poll.

## PP-85 [LOW] USB-2408 poller logs every error every cycle — FIXED
- **Rust:** `usb-2408/src/poller.rs:94-103,178-190` warn + rewrite LAST_ERROR_MESSAGE each cycle, no recovery message.
- **C:** `drvMultiFunction.cpp:2619-2625,2645-2650` report only on `!prevStatus`; `:2843-2846` "Device returned to normal status".
- **Impact:** a persistent error floods stderr (hundreds of lines/s).
- **Family:** `usb-ctr/src/poller.rs:100-102` has the same per-cycle logging, but C `USBCTR::pollerThread` (`drvUSBCTR.cpp:1512-1515`) also prints every cycle — distinct, not a divergence.
- **Class:** ref-faithful. **Live:** confirmed — 336 WARN lines within ~1 s in Pseudo-diff mode.

## usb-2408 — waveform generator / digitizer

## PP-86 [HIGH] Internal waveform amplitude used as peak instead of peak-to-peak — FIXED (regression of PP-40, fix never merged)
- **Rust:** `wave_gen.rs:77-114` sin/square/sawtooth/random swing `offset ± amplitude`; unit tests `wave_gen.rs:284-293` assert the regressed values.
- **C:** `drvMultiFunction.cpp:1542,1546,1549-1550,1554,1568-1570` span `offset ± amplitude/2` (pulse is full amplitude in both).
- **Impact:** every internal waveform is 2× the C voltage — over-drive risk.
- **Class:** ref-faithful. **Live:** static (no AO→AI loopback).

## PP-87 [HIGH] Generator stop rewrites every AO channel, including channels not in the scan — FIXED
- **Rust:** `wave_gen.rs:161-167` saves only `first..=last`; `stop_wave_gen` (`:242-249`) writes `saved_outputs[ch]` for all `ch in 0..MAX_ANALOG_OUT`; `saved_outputs` starts `[0.0; 2]` (`:34`).
- **C:** `drvMultiFunction.cpp:1721-1733` restores only enabled channels.
- **Impact:** with only WaveGen1 enabled, every stop drives AO2 to DAC code 0 (−10 V) or a stale value, silently overriding Ao2 while its record shows the old value.
- **Class:** ref-indep. **Live:** static (output not observable); this session's one-shot and continuous runs with WaveGen2 disabled would have driven AO2 to code 0 on stop — AO2 was re-written to 32768 afterwards.

## PP-88 [MED] Pulse width treated as a fraction; PULSE_DELAY ignored — FIXED (regression of PP-41, fix never merged)
- **Rust:** `wave_gen.rs:96-105` `pulse_samples = pulse_width * n`, high from sample 0; `driver.rs:536-551` never reads `WAVEGEN_PULSE_DELAY`.
- **C:** `drvMultiFunction.cpp:1557-1565` `nPulse = pulseWidth/dwell + 0.5`, `nDelay = pulseDelay/dwell + 0.5`, clamped to leave ≥ 1 low sample.
- **Impact:** wrong pulse length, no delay, all-high when width ≥ period.
- **Class:** ref-faithful. **Live:** static.

## PP-89 [LOW] Sin/sawtooth period divisor `n` instead of `numPoints-1`; different random generator — FIXED (regression of PP-41 LOW, fix never merged)
- **Rust:** `wave_gen.rs:79,93,108-112`.
- **C:** `drvMultiFunction.cpp:1545,1553,1569-1570` (`srand(1); rand()`), float32 staging (`:1506,1660`).
- **Impact:** DAC codes differ from C at every point.
- **Class:** ref-faithful. **Live:** static.

## PP-90 [MED] Wave-dig ERR_BAD_RATE never sets the −9999 DwellActual sentinel — FIXED (PP-41 DEFERRED item, now unblocked)
- **Rust:** `wave_dig.rs:140-154` discards the `UlError` code; `WAVEDIG_DWELL_ACTUAL` not written on failure; `uldaq-sys` has no `ERR_BAD_RATE`.
- **C:** `drvMultiFunction.cpp:1836,1842-1846` `-9999` on `ERR_BAD_RATE`; `ERR_BAD_RATE = 22` (`/usr/local/include/uldaq.h:164`).
- **Impact:** a rejected rate leaves DwellActual stale.
- **Class:** ref-faithful. **Live:** confirmed — `NumChans 8, Dwell 0.001` → "uldaq error 22", DwellActual stayed 0.002 from the previous run (C: −9999).

## PP-91 [MED] Digitizer queue uses the first channel's range for every channel — FIXED
- **Rust:** `driver.rs:346-348` `range = ANALOG_IN_RANGE[first_chan]`; `wave_dig.rs:103-111` same range in every `AiQueueElement`.
- **C:** `drvMultiFunction.cpp:1787-1802` per-channel `gainArray[i] = analogInRange_[firstChan+i]`.
- **Impact:** channels behind a narrower first-channel range clip; a wider one loses resolution.
- **Class:** ref-faithful. **Live:** static (needs a known input).

## PP-92 [MED] Three disagreeing end-of-scan paths instead of C's single `stopWaveDig` — FIXED
- **Rust:** auto-restart (`wave_dig.rs:219-262`) fires no VoltWF/AbsTimeWF callbacks and reuses stale settings; manual Stop (`driver.rs:406-408`) delivers no partial data; a scan-status error (`wave_dig.rs:186-192`) returns early with `running` stuck true, freezing Run, CurrentPoint and the scalar AI/TC polling (`poller.rs:126`).
- **C:** `drvMultiFunction.cpp:1861-1882` clears Run, `readWaveDig` callbacks, `ulAInScanStop`, then `startWaveDig()` (re-reads all params) if AutoRestart; poller continues after a status error (`:2695-2732`).
- **Impact:** AutoRestart never delivers completed scans; Stop discards data; one scan error freezes the digitizer and all AI records.
- **Family:** the scan-status-error early return also exists at `usb-ctr/src/mcs.rs:226` (PP-48) and `scaler.rs:103`.
- **Class:** ref-faithful. **Live:** static.

## PP-93 [MED] Waveform-parameter writes never run `defineWaveform`: no INT_WF, readbacks stale, no live restart — FIXED (partial regression of the "COMPLETED" user/internal-array item)
- **Rust:** `driver.rs:233-620` has no branch for WAVE_TYPE, USER/INT_NUM_POINTS, ENABLE, EXT_*, CONTINUOUS, USER/INT_DWELL, PULSE_*, AMPLITUDE, OFFSET; `WAVEGEN_INT_WF` (`params.rs:201`) never written, `read_float32_array` returns 0 for it (`:635-636`).
- **C:** `drvMultiFunction.cpp:2171-2183,2294-2305` `defineWaveform(addr)` and, while running, stop+start; `defineWaveform` updates NUM_POINTS/DWELL/FREQ and fires INT_WF (`:1520-1573`).
- **Impact:** amplitude/type/enable changes during a continuous run take effect only after a manual Stop/Run; WaveGenNumPoints/Dwell/Freq stale until Run; no internal-waveform preview.
- **Class:** unimpl. **Live:** confirmed — WaveGenFreq/NumPoints/Dwell stayed 0/2048/0 after `IntNumPoints=200, IntDwell=0.001` until Run published 5/200/0.001.

## PP-94 [MED] User waveform ignores Amplitude/Offset, repeats a short buffer, accepts oversize writes — FIXED
- **Rust:** `driver.rs:539-547` `user[i % user.len()]` unscaled; `write_float32_array` (`:653-665`) truncates and returns Ok.
- **C:** `drvMultiFunction.cpp:1652-1660` `user*amplitude + offset`; short write leaves the rest of the buffer; `:2513-2518` oversize → asynError. (Upstream `i`/`k` index bug at `:1653`, see `upstream-c-defects.md`.)
- **Impact:** DAC output differs from C whenever Amplitude ≠ 1 or Offset ≠ 0; a short user WF plays repeatedly.
- **Class:** ref-faithful. **Live:** static.

## PP-95 [LOW] Poller publishes Run=0 through `write_int32`, re-running stop logic — FIXED
- **Rust:** `poller.rs:222,232` `write_int32_blocking(wave_{gen,dig}_run, 0, 0)` after releasing the state lock → `driver.rs:414/325` → `stop_wave_*`.
- **C:** `drvMultiFunction.cpp:2678-2680,2730-2732` stop under the driver lock inside the poll; Run=0 via `setIntegerParam`.
- **Impact:** a client that restarts immediately on completion can have its new scan stopped (and the AO restore of PP-87 re-run). Timing-dependent.
- **Class:** ref-indep. **Live:** static.

## Live hardware verification (2026-09-22)

IOCs: `usb-ctr-ioc` (CA 5064) and `usb-2408-ioc` (CA 5074), release build of
`cc61da4`, stock st.cmd. Results not already cited above:

- `list-devices`: both boards found (USB-CTR08 `01DAB0FB`, USB-2408-2AO `01DA523D`) — pass
- Both IOCs iocInit with no warnings (190 / 175 records) — pass
- Device info (ModelName, ModelNumber 295/254, FirmwareVersion 0.10/1.01, UniqueID, ULVersion 1.2.0) — pass
- CTR pulse generator 1234.5 Hz / 25 % → Frequency_RBV 1234.5, Period_RBV 0.000810042 — pass
- CTR counter Reset (Counter5 1 → 0) — pass
- CTR DIO with bits Out: Lo 165 → Li 165; Bo2 → 167 — pass
- CTR MCS internal advance, 100 pts × 10 ms → ElapsedReal 1.037, CurrentChannel 100, TimeWF 0, 0.01, … — pass
- CTR scaler count TP=1 s: never completes — expected without counter-0 clock wiring (C identical); arm/stop path works
- 2408 AI 8 ch Differential, ranges ±10 V … ±0.078 V, Single-ended — pass
- 2408 TC on Ai1: open-TC detect On → −9999, Off → finite garbage (open input) — pass
- 2408 DIO open-drain: Lo 15 → Li 240 — pass
- 2408 wave digitizer 2 ch × 500 pts × 2 ms → CurrentPoint 500, DwellActual 0.002 — pass
- 2408 wave generator internal sine one-shot 200 pts → Run returns to Stop, CurrentPoint 200; continuous run and stop — pass (output voltage not measurable)

## Review Log — 2026-09-22 (wave 4)

52 findings (8 HIGH / 27 MED / 17 LOW) from 68 raw across 5 panels; 16
merged as cross-panel duplicates. 27 are live-confirmed on the attached
hardware, the rest are static (no loopback wiring).
Independence split: 8 ref-indep, 23 ref-faithful, 11 contract, 10 unimpl.
16 are regressions of PP-40..43 / the COMPLETED block whose fixes were
never merged (PP-45 part, 51, 52, 53, 54, 57, 58, 60, 63, 71, 81, 86, 88,
89, 90, 93 part).

Themes:
- **Unmerged fix branches.** The wave-3 inventory recorded FIXED against
  branches that never reached `main`; nothing verified the claims on
  `main`. This is the single largest source of open items.
- **db contract drift from the C templates** (PP-55, 64, 66, 75, 76, 77,
  78): missing PINI/PHAS, raw-count AO, renamed PVs, dropped CP links.
  The Rust templates were re-authored instead of derived from the C ones.
- **Error propagation** (PP-46, 67, 90, 92): uldaq failures are logged to a
  Rust-only waveform while records stay NO_ALARM and state flags wedge.
- **Missing device configuration calls** (PP-44, 52, 53, 79): C issues
  `ulCLoad(OUTPUT_VAL*)`, `ulDaqInSetTrigger`, `ulAISetConfigDbl(DATA_RATE)`
  that the port never makes.
- **Autosave dead since `9b794e6`** (PP-65) masks PP-66 today.

FFI layer (`uldaq-sys`) is clean: 165 constants, 4 struct layouts and 45
prototypes verified against `uldaq.h` with a compiled dump.
