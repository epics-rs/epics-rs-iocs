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

## Documented, not fixed (unreachable / port-is-stricter / degenerate)

- **PP-9 [LOW] GM10 FData module-presence gate `(address-1)/100` vs C `address/100`** (`instrument.rs:752` ↔ `drvGM10.c:995`). Differs only at addresses that are exact multiples of 100, which are never real channels (channels are `module*100 + 1..=n`). C is itself internally inconsistent (`:712` uses the 0-based form). Unreachable — not fixed.
- **PP-10 [LOW] GM10 strict whole-string link parse vs C lenient prefix parse** (`link.rs:82` ↔ `devGM10_*.c` `atoi`/`strtol`). Rust rejects trailing garbage in db link text that C would accept; only reachable via malformed db. The port being stricter is defensible — not fixed.
- **PP-11 [LOW] Rontec zero-length spectrum read early-return** (`drivers/mca-rontec/src/driver.rs:284`). Rust returns empty for `max_chans == 0`; C still sends `$SS …,0` and drains a 4-byte reply. Degenerate — a real mca record never requests 0 channels. Not fixed.
