# Upstream C/C++ defect register

Defects found in the upstream EPICS C/C++ modules while porting them to
this workspace. One entry per defect, grouped by upstream module.

Handling policy:

- **Wave 1 (2026-07-10 ~ 07-12)** ported upstream defects *verbatim*
  (wire-parity-first); each entry below is marked `preserved`,
  `fixed-in-port`, or `not-reproduced` per how that port handled it.
- **Wave 2 onward (user decision 2026-07-12)**: upstream defects are NOT
  reproduced. Ports fix them at source, and every instance is appended
  to this register instead. The Wave-1 dividing rule for template links
  still applies (unambiguous in-file target = typo, fix with citation;
  no possible target = record here).

Framework-mapping deviations (epics-rs API shape, not upstream bugs) are
NOT listed here — they live in each port's commit message / report.

## areaDetector/ADSimDetector (`simDetector.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 1 | Stop path computes `ADStatusIdle`/`ADStatusAborted` then unconditionally overwrites with `ADStatusAcquire` (simDetector.cpp:918) | preserved (pinned by test) |
| 2 | `computeImage` failure path `if (status) continue;` retries immediately with `acquire` still set — hot loop on persistent allocation failure | preserved |
| 3 | Bayer/YUV color modes leave `ndims=0, colorDim=-1` then index `dims[]` with them (UB) | not-reproduced (treated as Mono) |
| 4 | `db/simDetector.template` sets `ZRST` twice on `XSineOperation_RBV`/`YSineOperation_RBV` (dead first line) | not-reproduced (dead line dropped) |

## areaDetector/ADCSimDetector

| # | Defect | Port handling |
|---|--------|---------------|
| 5 | Example st.cmd passes `dataType=7` meaning NDFloat64 from before the Int64/UInt64 enum insertion; 7 is now NDUInt64, contradicting the db's `TYPE=Float64,FTVL=DOUBLE` | fixed-in-port (st.cmd uses 9 = NDFloat64, commented) |
| 6 | Example st.cmd `NDFFTConfigure("FFT3", …` missing closing paren | fixed-in-port |

## areaDetector/ADURL (`URLDriver.cpp` + `url.template`)

| # | Defect | Port handling |
|---|--------|---------------|
| 7 | `URLSelect` mbbo: `EIST`("URL9") has no `EIVL`; `NIST`("URL10") `NIVL="8"` duplicates `SVST`("URL8") `SVVL` — URL10 drives URL8's seq link, URL9 is indistinguishable from unset | preserved (template comment) |

## areaDetector/ADPilatus (`pilatusDetector.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 8 | `readTiff` returns success with an unwritten buffer when its retry loop expires (C: uninitialised memory published) | preserved (buffer zero-filled, not uninitialised) |
| 9 | `readBadPixelFile` replacement index `ygood*ny+xgood` — should be `*nx` (wrong pixel replaced on non-square arrays) | preserved (test pins it) |
| 10 | `thread` reply parsing: channel-3 values overwrite channel-0's `ThTemp0`/`ThHumid0` | preserved |
| 11 | `averageFlatField` divides by zero → NaN when no pixel reaches `MinFlatField` | preserved |
| 12 | `pilatusStatus` reuses one temp/humid pair across all channels | preserved |

## areaDetector/ADmarCCD (`marCCD.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 13 | `readTiff` returns success with an unfilled buffer on decode failure; C also repeats strip 0 for multi-strip TIFFs | preserved success-semantics (zero-filled); multi-strip decode corrected |
| 14 | `getImageData` publishes the buffer even when the read errored | preserved |
| 15 | `MarState_RBV` record duplicated in the template | preserved |
| 16 | `collectSeries` returns early on a file-template error, leaving the acquisition task spinning | not-reproduced (port cleans up and stops) |

## epics-modules/quadEM (`drvTetrAMM.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 17 | `readStatus` failure path: the restore of `Acquire=1` sits after `goto error`, so C never restarts acquisition after a failed status read — but the surrounding logic reads as if it should; the Rust port initially restored it and diverged from wire behavior | fixed-in-port (`70a006e` matches the C `goto` behavior: no restart) |

## epics-modules/vac (`devVacSen.c`, `vsRecord.c`, `devDigitelPump.c`)

All preserved verbatim in Wave 1 (wire behavior a working install depends on):

| # | Defect | Port handling |
|---|--------|---------------|
| 18 | devVacSen `monitor()` tests `chgc & IGn_FIELD` after `readWrite` zeroed `chgc` — dead branches | preserved |
| 19 | vsRecord `checkAlarms` alarm-checks PRES (`val=pvs->val` then `val=pvs->pres`) | preserved |
| 20 | devDigitelPump `S32` reads `sp3s` where S22/S12 read `s2hs`/`s1hs` | preserved |
| 21 | devDigitelPump setpoint guard `v<1e-4 \|\| v>1e-11` (`\|\|` where `&&` meant) — true for every non-negative v | preserved |
| 22 | devDigitelPump Digitel `case 3:` writes `s2mr`/`s2vr` (means `s3mr`/`s3vr`); `case 2:` fall-through overwrites | preserved |
| 23 | devDigitelPump MPC slot 8 leaves `pvalue` stale → SP4R mirrors SP3R; QPC send guard skips slots 6–8 → replies duplicate slot 5 | preserved |
| 24 | devDigitelPump `strncpy(&recBuf[139],…,2)` no terminator → S3TR never holds the bakeout time | preserved |
| 25 | devVacSen `char sign; int exp;` uninitialised | fixed-in-port (seeded `('+', 0)`) |
| 26 | devVacSen MX200 init ignores `sscanf` return | fixed-in-port (short param set rejected) |
| 27 | devVacSen MX200 relay recode `sprintf` into a string literal (UB; C works only because it lands in the reply buffer) | fixed-in-port (writes the buffer explicitly) |
| 28 | devVacSen `goto finish` on control failure before `responseBuffer` zeroed | fixed-in-port (zeroed) |
| 29 | devDigitelPump `t1/val1/val2` uninitialised when no `spfg` bit matches | fixed-in-port (zeroed, no command sent) |
| 30 | devDigitelPump uninitialised `nwrite`→`*nread` for QPC command < 10 chars | fixed-in-port (reply-too-small path) |
| 31 | devDigitelPump indexes `readBuffer[4]`/`[5]` on short replies | fixed-in-port (reads NUL) |

(18–24 are wire/record-visible behavior → preserved; 25–31 are C UB with
no defined wire behavior to preserve → the port picks the defined
equivalent. This split predates the Wave-2 policy.)

## epics-modules/delaygen (`drvAsynDG645.cpp`, `colbyPDL100A.db`)

| # | Defect | Port handling |
|---|--------|---------------|
| 32 | DG645 GH-output inversion bug (output polarity table) | preserved |
| 33 | DG645 "ofset" typo in command/label table | preserved |
| 34 | DG645 "disabled" status-text typo | preserved |
| 35 | Colby db "step" `ao` record has no `OUT`/`DTYP` (dead wiring) | preserved |
| 36 | Colby db `connect`/`disconnect` sseq reference `$(P)$(A).CNCT` on an asynRecord the upstream st.cmd never loads under that `R=` macro | preserved |

## epics-modules/SyringePump (`teledynePumpD.template`, `teledynePumpH.template`)

| # | Defect | Port handling |
|---|--------|---------------|
| 37 | `PistonUp` OUT references `setPistonUp`, not defined in `teled_h.proto` — record can never function | preserved (OUT unwired + comment) |
| 38 | `AlarmI` calc references undefined `$(s):$(ta):$(ss):BDetStatus` PV (both D and H) | preserved |
| 39 | D-series `PressSeq`/`MaxFlowSeq` `LNK2` → `$(s):$(ta):$(ss):Run.PROC` but Run is defined as `$(P)$(PUMP)Run` — core run trigger dangling (naming-scheme typo) | fixed-in-port (repointed, cited) |
| 40 | D-series `PressSeq` `DOL3` → `$(s):$(ta):$(ss):PressSet` but the record is `$(P)$(PUMP)PressureSP` — setpoint source dangling (naming-scheme typo) | fixed-in-port (repointed, cited) |
| 41 | D-series `FlowRateTweakDown/Up` reference never-defined `FlowRateSP` (vestigial block) | preserved |
| 42 | D-series `RefillRateTweakDown/Up` reference `RefillRateSP` which exists only in the ISCO family templates; `teled_d.proto` has no refill command (copy-paste residue) | preserved |

## epics-modules/microEpsilon (`capaNCDT6200Sup.c`)

| # | Defect | Port handling |
|---|--------|---------------|
| 43 | `capaNCDT6200Configure(portName, IPaddress, IPport)` third arg accepted but silently ignored — always connects to hardcoded port 10001 | preserved |
| 44 | Channel availability masks use non-power-of-2 literals (`&1`/`&5`/`&21`/`&85` for chan 1–4) and the displacement value mask differs between channels (chan1 `&0xFFFFFFFF` no-op; chan2–4 `&0xFFFFFF` drops the top byte) | preserved (no spec to check against) |

## epics-modules/motor (motorPI legacy, earlier campaign)

| # | Defect | Port handling |
|---|--------|---------------|
| 45 | E-710 (`drvPIE710.c`): status shift uses `2^8` (XOR = 10) where `1<<8` (256) is meant — status bits mis-shifted | preserved (flagged in `drivers/motor/pi`) |
