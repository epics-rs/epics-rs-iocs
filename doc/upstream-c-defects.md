# Upstream C/C++ defect register

Defects found in the upstream EPICS C/C++ modules while porting them to
this workspace. One entry per defect, grouped by upstream module.

Handling policy:

- **Wave 1 (2026-07-10 ~ 07-12)** ported upstream defects *verbatim*
  (wire-parity-first); entries were marked `preserved`,
  `fixed-in-port`, or `not-reproduced` per how that port handled it.
- **Retro-fix round (2026-07-12, user decision)**: every Wave-1
  `preserved` entry was retroactively resolved at source on its port's
  branch, one commit per entry citing this register. Resulting states:
  `retro-fixed (sha)` — behavior corrected;
  `removed (sha)` — dead record/link with no derivable in-file target
  deleted (PV-surface change noted in the commit);
  `not-applicable-in-framework (sha)` — the C defect's observable
  cannot occur in epics-rs by construction (comment-only commit);
  `unfixable-without-spec` — intent underivable from any available
  source; left as ported, no guess fabricated.
- **Wave 2 onward**: upstream defects are NOT reproduced. Ports fix
  them at source, and every instance is appended to this register.
  The dividing rule for template links still applies (unambiguous
  in-file target = typo, fix with citation; no possible target =
  remove + record here).

Framework-mapping deviations (epics-rs API shape, not upstream bugs) are
NOT listed here — they live in each port's commit message / report.

## areaDetector/ADSimDetector (`simDetector.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 1 | Stop path computes `ADStatusIdle`/`ADStatusAborted` then unconditionally overwrites with `ADStatusAcquire` (simDetector.cpp:918) | retro-fixed (`9a0f5bb`) |
| 2 | `computeImage` failure path `if (status) continue;` retries immediately with `acquire` still set — hot loop on persistent allocation failure | retro-fixed (`b2c3c0d`; no unit test — path lives in the async task loop with no failure-injection point) |
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
| 7 | `URLSelect` mbbo: `EIST`("URL9") has no `EIVL`; `NIST`("URL10") `NIVL="8"` duplicates `SVST`("URL8") `SVVL` — URL10 drives URL8's seq link, URL9 is indistinguishable from unset | retro-fixed (`81f11f2`: distinct `EIVL="9"`/`NIVL="10"`, each URLn drives its own seq link) |

## areaDetector/ADPilatus (`pilatusDetector.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 8 | `readTiff` returns success with an unwritten buffer when its retry loop expires (C: uninitialised memory published) | retro-fixed (`8202ce3`: error on timeout, no publish) |
| 9 | `readBadPixelFile` replacement index `ygood*ny+xgood` — should be `*nx` (wrong pixel replaced on non-square arrays) | retro-fixed (`5e67565`: width stride; dead `ny` param dropped) |
| 10 | `thread` reply parsing: channel-3 values overwrite channel-0's `ThTemp0`/`ThHumid0` | retro-fixed (`e4886c0`: block removed — no ThTemp3/ThHumid3 params exist, the reply defines channels 0–2 only, remap would fabricate hardware) |
| 11 | `averageFlatField` divides by zero → NaN when no pixel reaches `MinFlatField` | retro-fixed (`6e9be7c`: skip normalization with error, no publish) |
| 12 | `pilatusStatus` reuses one temp/humid pair across all channels | retro-fixed (`b78be7c`: per-channel independent resolver) |

## areaDetector/ADmarCCD (`marCCD.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 13 | `readTiff` returns success with an unfilled buffer on decode failure; C also repeats strip 0 for multi-strip TIFFs | retro-fixed (`c2c6fc9`: error on timeout, no publish; multi-strip decode was already corrected in the port) |
| 14 | `getImageData` publishes the buffer even when the read errored | retro-fixed (`0f3c179`: propagate error, no publish) |
| 15 | `MarState_RBV` record duplicated in the template | retro-fixed (`a25071c`: bare duplicate removed) |
| 16 | `collectSeries` returns early on a file-template error, leaving the acquisition task spinning | not-reproduced (port cleans up and stops) |

## epics-modules/quadEM (`drvTetrAMM.cpp`)

| # | Defect | Port handling |
|---|--------|---------------|
| 17 | `readStatus` failure path: the restore of `Acquire=1` sits after `goto error`, so C never restarts acquisition after a failed status read — but the surrounding logic reads as if it should; the Rust port initially restored it and diverged from wire behavior | fixed-in-port (`70a006e` matches the C `goto` behavior: no restart) |

## epics-modules/vac (`devVacSen.c`, `vsRecord.c`, `devDigitelPump.c`)

| # | Defect | Port handling |
|---|--------|---------------|
| 18 | devVacSen `monitor()` tests `chgc & IGn_FIELD` after `readWrite` zeroed `chgc` — dead branches | not-applicable-in-framework (`1360d4a`: the posts those branches meant to emit already happen by construction — IG1S/IG2S/DGSS are pp(TRUE) and post at caput time) |
| 19 | vsRecord `checkAlarms` alarm-checks PRES (`val=pvs->val` then `val=pvs->pres`) | retro-fixed (`5367909`: alarms check VAL) |
| 20 | devDigitelPump `S32` reads `sp3s` where S22/S12 read `s2hs`/`s1hs` | retro-fixed (`870621d`: reads `s3hs`, symmetric) |
| 21 | devDigitelPump setpoint guard `v<1e-4 \|\| v>1e-11` (`\|\|` where `&&` meant) — true for every non-negative v | retro-fixed (`0fbcdae`: rejects outside `[1e-11, 1e-4]` Torr) |
| 22 | devDigitelPump Digitel `case 3:` writes `s2mr`/`s2vr` (means `s3mr`/`s3vr`); `case 2:` fall-through overwrites | retro-fixed (`405731b`: setpoint-3 decodes into index 2) |
| 23 | devDigitelPump MPC slot 8 leaves `pvalue` stale → SP4R mirrors SP3R; QPC send guard skips slots 6–8 → replies duplicate slot 5 | retro-fixed (`60253f6` MPC: slot 8 reads its own setpoint; `56e1d7e` QPC: slots 6–8 skipped, single-setpoint decode — the C duplication was stale-buffer re-parsing, verified against devDigitelPump.c:988-1048/828-853) |
| 24 | devDigitelPump `strncpy(&recBuf[139],…,2)` no terminator → S3TR never holds the bakeout time | retro-fixed (`5e8ce38`: parses only its two digits; `Scratch::strncpy` zeroes the whole tail — closes the short-copy-tail-leak family) |
| 25 | devVacSen `char sign; int exp;` uninitialised | fixed-in-port (seeded `('+', 0)`) |
| 26 | devVacSen MX200 init ignores `sscanf` return | fixed-in-port (short param set rejected) |
| 27 | devVacSen MX200 relay recode `sprintf` into a string literal (UB; C works only because it lands in the reply buffer) | fixed-in-port (writes the buffer explicitly) |
| 28 | devVacSen `goto finish` on control failure before `responseBuffer` zeroed | fixed-in-port (zeroed) |
| 29 | devDigitelPump `t1/val1/val2` uninitialised when no `spfg` bit matches | fixed-in-port (zeroed, no command sent) |
| 30 | devDigitelPump uninitialised `nwrite`→`*nread` for QPC command < 10 chars | fixed-in-port (reply-too-small path) |
| 31 | devDigitelPump indexes `readBuffer[4]`/`[5]` on short replies | fixed-in-port (reads NUL) |

(25–31 are C UB with no defined wire behavior to preserve — the port
picked the defined equivalent from the start. 18–24 were preserved in
Wave 1 as wire/record-visible behavior, then resolved in the retro-fix
round.)

## epics-modules/delaygen (`drvAsynDG645.cpp`, `colbyPDL100A.db`)

| # | Defect | Port handling |
|---|--------|---------------|
| 32 | DG645 GH-output inversion bug (output polarity table) | retro-fixed (`28b47ba`) |
| 33 | DG645 "ofset" typo in command/label table | retro-fixed (`1dd6d49`) |
| 34 | DG645 "disabled" status-text typo | retro-fixed (`2b14f4c`) |
| 35 | Colby db "step" `ao` record has no `OUT`/`DTYP` (dead wiring) | retro-fixed (`79fb988`: wired to the driver's own write-step command — derivable in-file target) |
| 36 | Colby db `connect`/`disconnect` sseq reference `$(P)$(A).CNCT` on an asynRecord the upstream st.cmd never loads under that `R=` macro | removed (`d982f7f`; PV-surface change noted) |

## epics-modules/SyringePump (`teledynePumpD.template`, `teledynePumpH.template`)

| # | Defect | Port handling |
|---|--------|---------------|
| 37 | `PistonUp` OUT references `setPistonUp`, not defined in `teled_h.proto` — record can never function | removed (`25c5f5e`; PV-surface change noted) |
| 38 | `AlarmI` calc references undefined `$(s):$(ta):$(ss):BDetStatus` PV (both D and H) | removed (`5a1bfaf`: dead link removed; PV-surface change noted) |
| 39 | D-series `PressSeq`/`MaxFlowSeq` `LNK2` → `$(s):$(ta):$(ss):Run.PROC` but Run is defined as `$(P)$(PUMP)Run` — core run trigger dangling (naming-scheme typo) | fixed-in-port (repointed, cited) |
| 40 | D-series `PressSeq` `DOL3` → `$(s):$(ta):$(ss):PressSet` but the record is `$(P)$(PUMP)PressureSP` — setpoint source dangling (naming-scheme typo) | fixed-in-port (repointed, cited) |
| 41 | D-series `FlowRateTweakDown/Up` reference never-defined `FlowRateSP` (vestigial block) | removed (`dcc7776`; PV-surface change noted) |
| 42 | D-series `RefillRateTweakDown/Up` reference `RefillRateSP` which exists only in the ISCO family templates; `teled_d.proto` has no refill command (copy-paste residue) | removed (`60aa0f0`; PV-surface change noted) |

## epics-modules/microEpsilon (`capaNCDT6200Sup.c`)

| # | Defect | Port handling |
|---|--------|---------------|
| 43 | `capaNCDT6200Configure(portName, IPaddress, IPport)` third arg accepted but silently ignored — always connects to hardcoded port 10001 | retro-fixed (`c25cadd`: IPport honored) |
| 44 | Channel availability masks use non-power-of-2 literals (`&1`/`&5`/`&21`/`&85` for chan 1–4) and the displacement value mask differs between channels (chan1 `&0xFFFFFFFF` no-op; chan2–4 `&0xFFFFFF` drops the top byte) | unfixable-without-spec — no bit-layout documentation for `channelBitField` exists in the module (source, headers, README all checked); the value-mask asymmetry has a plausible internal-consistency reading (24-bit range divisor) but fixing on it would be a guess |

## epics-modules/motor (motorPI legacy, earlier campaign)

| # | Defect | Port handling |
|---|--------|---------------|
| 45 | E-710 (`drvPIE710.c`): status shift uses `2^8` (XOR = 10) where `1<<8` (256) is meant — status bits mis-shifted | retro-fixed (`2b9c3c0` on `feat/newport-motor-drivers`: `wrapping_mul(256)`, shift test pins low-byte→high-byte) |

---

# Wave 2 (2026-07-12 ~ 07-13, epics-rs 0.23.0 baseline)

Per the Wave-2 policy, none of these were reproduced: every entry marked
`fixed-in-port` was fixed at source in the port, cited in that port's
commit message. Entries marked `unfixable-without-spec` or `preserved`
remain open upstream defects the port did not guess at.

## areaDetector/ADEiger (commit `0aa16de`)

| # | Defect | Port handling |
|---|--------|---------------|
| 46 | `eigerParam.cpp:226` — `EigerParam::put(bool)` indexes the enum with `!value`, the inverse of `fetch` (:548); the constructor's `mMonitorEnable->put(false)` (eigerDetector.cpp:1662) therefore *enables* the monitor at startup | fixed-in-port (`encode_bool` indexes with `value`; test pins both directions) |
| 47 | `eigerDetector.cpp:1829` — threshold-3/4 branch `else if (Pilatus4)` sits after `else if (Eiger2 \|\| Pilatus4)`, unreachable; thresholds 3/4 never reach the NDArray attributes | fixed-in-port (one model-driven threshold list, no shadowable branch) |
| 48 | `streamApi.cpp:389` + `eigerDetector.cpp:1503-1507` — `uncompress()` return discarded (failed decompression publishes garbage pixels); `getFrame`'s `err` unchecked before `pArray` deref (uninitialised pointer on error) | fixed-in-port (decode errors propagate as `Err`, end the series) |
| 49 | `eigerDetector.cpp:1937-1996` — `parseTiffFile`: no bounds checks on IFD offset/entries/tags; NDArray allocated before validation (leaked on early returns); `memcpy` of device-controlled `StripByteCounts` into a `width*height*depth/8` buffer — heap overflow on a malformed monitor image | fixed-in-port (bounds-checked decode requiring `StripByteCounts == w*h*elem`; 5 rejection tests) |
| 50 | `eigerDetector.cpp:258` — `mSequenceId->put(...)` in the constructor when state is `"na"`, but `mSequenceId` is created at :272 — uninitialised member deref | fixed-in-port (structurally impossible: params exist before any write) |

## areaDetector/ADMythen (commit `079f15d`)

| # | Defect | Port handling |
|---|--------|---------------|
| 51 | `mythen.cpp:281` — `-stop` written with `strlen(outString_)` of the *previous* command: truncated or padded `-stop` on the wire | fixed-in-port (command carries its own length) |
| 52 | `mythen.cpp:965,1044` — NDArray declared `dims[0]=1280` while `decodeRawReadout` writes `1280*nmodules`; two-module detectors publish only module 0 | fixed-in-port (array is `1280*nmodules` wide) |
| 53 | `mythen.cpp:761` — `-get delafter` parsed as int64 from a 4-byte read: upper half is stale buffer | fixed-in-port (int32, in-file derivable) |
| 54 | `mythen.cpp:928-936` — `ImageMode=Continuous` never clears `acquire`/`acquiring_`: task spins in `while(1)` holding the driver lock | fixed-in-port (acquisition ends in every mode; PV surface unchanged — template offers Single/Multiple only) |
| 55 | `mythen.cpp:1347` — `NDDataType` published `NDInt32`, arrays emitted `NDUInt32` | fixed-in-port (`NDUInt32`) |

## areaDetector/ADTimePix3 (commit `c4cb82a`)

| # | Defect | Port handling |
|---|--------|---------------|
| 56 | `serval_http.cpp:49` — base64 `strchr` matches the terminating NUL: byte 0 decodes as index 64 | fixed-in-port (strict decoder, `None` on non-alphabet byte) |
| 57 | `serval_http.cpp:91` — `strip_quotes` drops first/last char unconditionally: `null` → `"ul"` | fixed-in-port (strings unquoted, everything else dumped) |
| 58 | `serval_http.cpp:1296` — mangled name fed to `std::map::operator[]`, which inserts it and reports orientation UP | fixed-in-port (`orientation_index` returns `Option`) |
| 59 | `serval_http.cpp:37` — every GET carries junk `?anon=true&key=value` | fixed-in-port (not sent) |
| 60 | `serval_http.cpp:77-87` — Basic auth applied on GET but not PUT/getJson | fixed-in-port (every request) |
| 61 | `serval_http.cpp:185,569,1490,2199,2254` — HTTP calls with no timeout | fixed-in-port (5 s poll / 10 s config) |
| 62 | `serval_http.cpp:1518,1823,1930,2320,2367,2372` — booleans PUT as JSON strings, read back as bools at :1269 | fixed-in-port (JSON booleans) |
| 63 | `serval_http.cpp:2286-2363` — enum index into a `json` array via `operator[]`: out-of-range grows the array with nulls, PUT carries `null` | fixed-in-port (`enum_name` errors on out-of-range) |
| 64 | `serval_http.cpp:1198` — `ADMaxSizeX = PixCount / NumberOfRows` unguarded → SIGFPE | fixed-in-port (guarded) |
| 65 | `serval_http.cpp:1190` — `ADSerialNumber` set from the software version | fixed-in-port (`Info.Boards[0].ChipboardId`) |
| 66 | `serval_http.cpp:1603/1881/image-channel` — three disagreeing "is this a stream" predicates; a `tcp://` image channel still gets a `FilePattern` | fixed-in-port (one `is_stream()` rule) |
| 67 | `serval_http.cpp:1005` + `mask_io.cpp:529` — `bpc2ImgIndex()` is not the inverse of `pelIndex()` for all quad orientations (C's own comment concedes it) | fixed-in-port (inverse derived from the one forward map; bijection test) |
| 68 | `mask_io.cpp:210-226` — `rowsCols` divides by `rowLength`, 0 before the first `/detector` reply | fixed-in-port (`Geometry::new` returns `None`) |
| 69 | `mask_io.cpp:237,250,275` — `buf[j*ROWS + i]` stride in maskReset/Rectangle/Circle; every other site uses `j*COLS+i` | fixed-in-port (uniform `cols` stride) |
| 70 | `mask_io.cpp:238` — `maskReset` *assigns* OnOff, wiping BPC bits 1 and 8 | fixed-in-port (touches only bit 0) |
| 71 | `mask_io.cpp:159` — `bufBPC[pelIndex(i,j)] \|= 1` unchecked | fixed-in-port (bounds-checked, dropped count returned) |
| 72 | `serval_stream.cpp:524,1389`; `histogram_io.cpp:311` — leftover byte count computed against the wrong buffer base: heap over-read and permanent stream desync | fixed-in-port (decoder owns its buffer; offsets are indices) |
| 73 | `serval_stream.cpp:487,1353` — `int` overflow before the range check | fixed-in-port (checked `usize` math + dimension caps) |
| 74 | `serval_stream.cpp:568-1443`; `histogram_io.cpp:352` — unconditional `__builtin_bswap` (breaks on big-endian hosts) | fixed-in-port (`from_be_bytes`) |
| 75 | `network_client.cpp:157` — blocking `recv` with no read timeout, "cancelled" by another thread closing the socket | fixed-in-port (2 s timeout; worker owns its socket) |
| 76 | `acquire.cpp:514-518` — HTTP-error path busy-spins at 100% CPU | fixed-in-port (sleeps between retries) |
| 77 | `acquire.cpp:625-628` — acquisition thread never joined: one leaked joinable thread per stop | fixed-in-port (persistent worker threads) |
| 78 | `ADTimePix.cpp:1579` — `lastServalConnected_`/`lastDetConnected_` never initialised | fixed-in-port (edge starts from "down") |
| 79 | `ADTimePix.cpp:791` — `writeFloat64` uses 2-arg `setDoubleParam` (addr 0) on a maxAddr=8 port | fixed-in-port (honours `user.addr`) |
| 80 | `ADTimePix.cpp:371,409-411,448-450,505-506` — early returns skip `callParamCallbacks`, latching ADAcquire | fixed-in-port (callbacks always run) |
| 81 | `serval_stream.cpp:582-700` — worker threads mutate the asyn param library with no port lock | fixed-in-port (all updates via the port handle) |
| 82 | `ADTimePix3.template:89-93` — `Health` bo with no ZNAM/ONAM | fixed-in-port (`Idle`/`Refresh`) |
| 83 | `Server.template:1752` — `PrvHstTotalCounts_RBV` is an `ai` with `DTYP asynInt64`; its Img twin at :657 is `int64in` | fixed-in-port (`int64in`) |

## areaDetector/ADMerlin (commit `9d293bd`)

| # | Defect | Port handling |
|---|--------|---------------|
| 84 | `mpxConnection.cpp:56` — `strtok` over a fixed 2304-byte window runs past the text header into binary pixel data | fixed-in-port (text region bounded by the frame's declared `offset`) |
| 85 | `merlinDetector.cpp:225-227` — PR1 header parse commented out, `profileMask` stays 0: every PR1 frame discarded | fixed-in-port (profiles parsed and published) |
| 86 | `merlinDetector.cpp:406` — Y-profile "invert" loop copies in order, inverts nothing | fixed-in-port (actually reversed) |
| 87 | `merlinDetector.cpp:1121` — `profileMaskParm & (MPXPROFILES_IMAGE == MPXPROFILES_IMAGE)` masks against constant 1 | fixed-in-port (proper bit test) |
| 88 | `mpxConnection.cpp:803` — lock around `mpxWriteRead` deliberately removed: status thread and `writeInt32` interleave on one command socket | fixed-in-port (socket owned by the port actor — serial by construction) |
| 89 | `merlinDetector.h:189` — `imagesRemaining` plain `int` written/decremented by two threads unsynchronised | fixed-in-port (`AtomicI32`) |
| 90 | `merlinApp/Db/merlin.template:54-70` — `FileFormat`/`FileFormat_RBV` redefined with no file plugin and no writer behind them | fixed-in-port (removed; PV-surface change noted) |
| — | `ThresholdScan` frame count `(stop−start)/step`, off by one iff the Labview server treats the window as inclusive | unfixable-without-spec (kept C's formula) |

## areaDetector/ADPhotonII (commit `0281954`)

| # | Defect | Port handling |
|---|--------|---------------|
| 91 | `PhotonII.cpp:398-401` — quote parsing: no-quote → NULL deref; one quote → `numChars` wraps to `SIZE_MAX` in `strncpy` | fixed-in-port (`Result`-returning parse, aborts with status message) |
| 92 | `PhotonII.cpp:332-342` — `switch (frameType)` misses `ADC0` (offered by the template): stale `set --runnumber` re-sent, no `grab`, task times out | fixed-in-port (`grab --adc0frame`, per p2util_help.txt:27) |
| 93 | `PhotonII.cpp:338-355` — dark sends `--count numDarks` but waits `numImages` messages | fixed-in-port (one count drives both) |
| 94 | `PhotonII.cpp:161,414-416` — NDArray sized from `ADSizeX/Y` params while `fread` expects `detSizeX_*detSizeY_*4`: heap overrun when SizeX < 768 | fixed-in-port (frames always full 768×1024; other sizes rejected) |
| 95 | `PhotonII.cpp:124 vs 416` — `NDDataType=NDUInt32` published, frames allocated `NDInt32` | fixed-in-port (`Int32` everywhere) |
| 96 | `PhotonII.cpp:591` — `strncpy` of a 512-byte command does not NUL-terminate; `strlen` immediately follows | fixed-in-port (command is a `String` through the actor) |
| 97 | `PhotonII.cpp:605-606` — `((PhotonII*)findAsynPortDriver(...))->p2util(...)` no NULL check: mistyped port name segfaults iocsh | fixed-in-port (iocsh error naming the missing port) |
| — | `NDAutoIncrement` never honoured (C threw `createFileName()` away, PhotonII.cpp:325); whether p2util advances the run number itself is underivable | unfixable-without-spec (no auto-increment added — guessing risks silent overwrites) |

## areaDetector/ADPSL (commit `e88c2c8`)

| # | Defect | Port handling |
|---|--------|---------------|
| 98 | `PSL.cpp:960` — `doCallbacksEnum(..., i, functions[i], 0)` indexes `functions[]` with the choice counter, not the loop variable: wrong param gets the table, OOB read when i>2 | fixed-in-port (enum reads answered by reason) |
| 99 | `PSL.cpp:275-282` — `getChoiceFromIndex` dereferences `set.end()` on out-of-range (UB) | fixed-in-port (`Err(NoSuchChoice)`) |
| 100 | `PSL.cpp:556-598` — `getImage`: server-announced `dataLen` copied into a header-geometry NDArray unchecked (heap overrun); unknown mode parses mode as geometry; zero-length read spins forever; `alloc` not NULL-checked | fixed-in-port (`parse_image_header` + `read_frame` validate everything) |
| 101 | `PSL.cpp:368-371 vs 549-553` — ColorMode=Mono for server mode `RGB` while `getImage` publishes RGB1 3-D for the same data | fixed-in-port (`parse_mode` returns RGB1) |
| 102 | `PSL.cpp:695-705` — `PSLTask` skips the frame wait when `arrayCallbacks==0`, busy-spinning | fixed-in-port (wait always, read out conditionally) |
| 103 | `PSL.cpp:356-449` — GetSize/GetMode/GetFliplr/GetFlipud parsed without the multi-camera `[...]` peel every other reply gets: never parse on a multi-camera server | fixed-in-port (one uniform peel) |
| 104 | `PSL.cpp:254` — `while ((pBracket = strchr(++pBracket,'[')) != NULL);` empty-bodied, result unused | fixed-in-port (dead code dropped) |

## areaDetector/ADPixirad (commit `8c5637f`)

| # | Defect | Port handling |
|---|--------|---------------|
| 105 | `pixirad.cpp:1182-1200` — UDP reassembly stores a packet at the expected index before advancing by the identifier gap: on loss the packet lands in the missing slot and the frame tail shifts; a wrapped identifier misaligns every later group | fixed-in-port (packet placed at `group_start + id`; wrap opens the next group at its boundary) |
| 106 | `pixirad.cpp:342-359` — `set_closest_Eth_DAC` starts at i=1: step 0 closest → uninitialised `*DAC`/`*EthSet` sent to the box | fixed-in-port (best seeded with step 0) |
| 107 | `pixirad.cpp:384-400` — `calculateThresholds` decrements `VThMax` after the match, programs the decremented value, reports the energy of the un-decremented one | fixed-in-port (`best_vth_max` returns the matching VTHMAX) |
| 108 | `pixirad.cpp:1023-1044` — `dataTask` allocates `pImage` only when `ColorsCollected==0`, `memcpy`s unconditionally: stale count → write through NULL | fixed-in-port (data task owns the buffer; NULL path gone) |
| 109 | `pixirad.cpp:565` — `strstr(...) + strlen(...)` without NULL check in the constructor | fixed-in-port (`parse_additional_info` returns `Option`) |
| 110 | `pixirad.cpp:606-640` — unrecognised `maxSizeX/Y` prints and continues with an uninitialised `SENSOR` | fixed-in-port (`pixiradConfig` fails) |
| 111 | `pixirad.cpp:1367-1372` — dew point computed from the param library even when the broadcast lacked a reading; can switch cooling off on a stale value | fixed-in-port (dew point/cooling ladder only on a complete broadcast) |
| 112 | `pixirad.template` Threshold2/3/4 — `VAL=10.000` + `PINI YES` overrides the driver's 15/20/25 keV: all four colours start at one energy | fixed-in-port (template VALs = driver defaults) |
| 113 | `pixirad.template` TriggerMode_RBV — `field(ZRVL,"0")` twice | fixed-in-port (duplicate removed) |

## areaDetector/ADBruker (commit `f9ae3a8`)

| # | Defect | Port handling |
|---|--------|---------------|
| 114 | `BISDetector.cpp:170` — SFRM header line 42 read twice (`wordOrder`, then `longOrder`); LONGORD's real line never read | fixed-in-port (duplicate read dropped); long order left unvalidated — unfixable-without-spec (no SFRM spec on disk; nothing fabricated) |
| 115 | `BISDetector.cpp:213` — overflow/underflow tables indexed with header counts never checked against declared table lengths: OOB read on a corrupt header | fixed-in-port (bounds-checked) |
| 116 | `BISDetector.cpp:186` — `bytesPerPixel` keeps the previous frame's value when NPIXELB is absent | fixed-in-port (required) |
| 117 | `BISDetector.cpp:152` — reads `HDRBLKS*512` bytes without checking file length | fixed-in-port (short files rejected) |
| 118 | `BISDetector.cpp:279` — frame written into an NDArray sized from SizeX/SizeY params, not the parsed frame | fixed-in-port (array sized from the frame) |
| 119 | `BISDetector.cpp:412` — `sscanf(strstr(...))` no NULL check: null deref on a status message lacking the key | fixed-in-port (`Option` parse) |
| 120 | `BISDetector.cpp:520` — one event signals both the exposure timer and user Stop: a Stop mid-exposure still reads and publishes the frame | fixed-in-port (distinct events; regression test) |
| 121 | `BIS.template` — FileFormat ZRST `"SRFM"` | fixed-in-port (`"SFRM"`) |
| 122 | `BISDetector.cpp:757` — `"$(ADBRUKER"` unclosed macro in an `asynPrint` | fixed-in-port (closed) |
| 123 | upstream `st.cmd` — creates a third IP port (49154) the driver never connects to | fixed-in-port (not created; noted in st.cmd) |

## epics-modules/specsAnalyser (commit `c86841f`)

| # | Defect | Port handling |
|---|--------|---------------|
| 124 | `specsAnalyser.cpp:91,1868` — `SPECS_PROTOCOL_VERSION` created `asynParamInt32`, only ever `setStringParam`'d | fixed-in-port (`ParamType::Octet`) |
| 125 | `specsAnalyser.cpp:1570-1590` — `getAnalyserParameter(bool&)` inverts the wire true/false mapping vs its int/string siblings | fixed-in-port |
| 126 | `specsAnalyser.cpp:1506-1513` — `getAnalyserParameterType` leaves an uninitialised output on an unknown `ValueType` string | fixed-in-port (`Option::None`) |
| 127 | `specsAnalyser.cpp:2126-2141,1925-2030` — ad-hoc string family: `cleanString` substr underflow; `commandResponse` backslash-escape lookback re-indexed per continuation chunk; ERROR digit loop past an empty string | fixed-in-port (bounds-checked parse, escape state carried across chunks) |
| 128 | `specsAnalyser.cpp:1740-1754` — `readRunModes()` lacks the `.clear()` its sibling has: RunMode enum choices duplicate on every reconnect | fixed-in-port (stateless read + replace-semantics choices) |
| 129 | `specsAnalyser.template:58-61` — "disable redundant fields" block disables `MaxSizeX_RBV` twice instead of `MaxSizeY_RBV` | fixed-in-port (unambiguous in-file pairing) |

## epics-modules/quadEM — Wave-2 sub-drivers (commits `629379c`, `20abe52`, `fbf53c5`, `e197930`)

| # | Defect | Port handling |
|---|--------|---------------|
| 130 | `drvNSLS_EM.cpp:376-384` — `readMeter` leaves `phase` uninitialised when the data line has no phase tag; ping-pong filter tests garbage | fixed-in-port (`Option<i32>`) |
| 131 | `drvNSLS_EM.cpp:176-187` — `findModule` writes into fixed 16-element `moduleInfo_` unbounded | fixed-in-port (unbounded Vec) |
| 132 | `drvPCR4.cpp:228,271` — line parsed via `(epicsFloat64*)ASCIIData`: every stored double overwrites 8 chars of the buffer still being parsed; short fields read channels 2-4 out of channel 1's binary image | fixed-in-port (parses into its own array) |
| 133 | `drvPCR4.cpp:300-303` — `strcpy` unbounded + `atoi(strstr(...)+5)` no NULL check: any reply not naming the model segfaults the IOC | fixed-in-port (`parse_version` returns `None`) |
| 134 | `drvT4UDirect_EM.cpp:1107-1176` — one UDP frame reassembled with five `read()`s; each recvfrom discards the datagram remainder, so reads 2-5 land on later packets | fixed-in-port (datagram read whole, parsed from buffer) |
| 135 | `drvT4U_EM.cpp:1139-1148` — `readBroadcastPayload` checks status but not byte count: short read publishes uninitialised heap as currents | fixed-in-port (short payload drops the frame) |
| 136 | `drvT4U_EM.cpp:1063,764` (+direct twin) — `new char[]` freed with `delete` | fixed-in-port (not reproducible in Rust) |
| 137 | `drvT4U_EM.cpp:375,393` (+direct twin) — `enable_cmd[value]` indexes a 2-element array with an unchecked epicsInt32 | fixed-in-port (`value != 0`) |
| 138 | `Db/T4U_EM.template:6` — Model readback initial VAL is 14 (FX4); the T4U reports 13 | fixed-in-port (13) |
| 139 | `Db/T4U_EM.template:394-443` — shared template binds QE_WSMODE/QE_RPP, created only by the direct driver: middle-layer IOC loads 4 records with no parameters | fixed-in-port (split into T4UDirect_EM.template) |
| 140 | `Db/FX4.template:88` — `record(longout, "(P)$(R)SetRange")` missing `$`: record created under the literal name, Range mbbo loses its only output link | fixed-in-port (`$(P)`) |
| 141 | `drvFX4.cpp` `onMessageEvent` — gate events of unmergeable messages discarded: `gateLevel_` stale, gate filter and trigger arming act on the wrong level | fixed-in-port (gate events always applied; ADC merge stays conditional) |
| 142 | `drvFX4.cpp` `onMessageEvent` — ADC value read in try/catch, gate value not: malformed gate throws past `sendGetEvent`, stalling acquisition | fixed-in-port (both malformed values skipped) |
| — | T4U `scale_reg_to_param` promotes a negative register through `u32` (C-identical); register signedness underivable | unfixable-without-spec |
| — | `FX4.template` `GetHVVReadback` reads `.../monitor_voltage_internal` where every sibling ends in `/value`; the meter's PV surface is underivable | preserved (candidate, not guessed) |

## epics-modules/ether_ip (commit `7934cbd`)

| # | Defect | Port handling |
|---|--------|---------------|
| 143 | `ether_ip.c:1361-1365,1277-1281` — CIP SINT decoded through unsigned `CN_USINT` though the header declares `signed char`: −2 reaches an `ai` as 254 | fixed-in-port (signed accessors sign-extend) |
| 144 | `ether_ip.c:1286-1290` — `get_CIP_double` unpacks DINT/INT unsigned: −2 → 4.29e9 (latent, public API) | fixed-in-port |
| 145 | `ether_ip.c:1440-1447` — `get_CIP_STRING` writes `size+1` bytes into a `size`-byte buffer (one-byte OOB) and ignores actual data length | fixed-in-port (copy bounded by data and `max−1`) |
| 146 | `drvEtherIP.c:909-930` — the `delay > 60` clock-jump branch dereferences `list` after the loop walked it to NULL: guaranteed NULL deref when the wall clock steps backwards | fixed-in-port (monotonic `Instant` — failure mode unrepresentable) |
| 147 | `drvEtherIP.c:1325-1326` — `malloc` failure returns holding `plc->lock` → deadlock | fixed-in-port (RAII locking) |
| 148 | `devEtherIP.c:1094` — `mask = 255` for an un-indexed binary link regardless of NOBT: mbbi bit 1 selected by `0x1FE`, aliasing eight bits (papered over by a `bits==1` special case in `get_bits`) | fixed-in-port (`mask=1` when NOBT>1; `get_bits`/`put_bits` one uniform rule) |
| 149 | `devEtherIP.c:1838-1925` — `wf_read` reads `0..NELM`, ignoring the link's element index that `analyze_link` registered | fixed-in-port (reads `element..element+NELM`; PV-visible only for indexed waveform links) |

## epics-modules/urRobot + ur_rtde 68ac4e18 (commit `2ab50a4`)

| # | Defect | Port handling |
|---|--------|---------------|
| 150 | ur_rtde `script_client.cpp:143` — PolyScopeX direct-torque guard tests `minor == 22`, every gated line is marked `$5.23`: never fires | fixed-in-port (threshold 23) |
| 151 | ur_rtde `rtde_io_interface.cpp:255-281` — analog-out sends both channel doubles, assigns one; the other is uninitialised | fixed-in-port (`Payload::AnalogOut` sum type) |
| 152 | ur_rtde `rtde_io_interface.cpp:186` — `1u << output_id` truncated to uint8_t unbounded: id ≥ 8 → mask 0 | fixed-in-port (`digital_mask()` errors) |
| 153 | ur_rtde `rtde_control_interface.cpp:2511` — ready-for-command loop with no sleep busy-spins a core up to 3 s | fixed-in-port (1 ms poll) |
| 154 | ur_rtde `rtde_receive_interface.cpp:203`, `rtde_control_interface.cpp:626` — `major>=3 && minor>=4` false on PolyScope 5.0-5.3: output-register block silently dropped | fixed-in-port (real version comparison) |
| 155 | ur_rtde `robotiq_gripper.cpp` — every `while (getVar(..) != x)` unbounded: hangs the port thread forever | fixed-in-port (all bounded, `UrError::Timeout`) |
| 156 | ur_rtde `robotiq_gripper.cpp:181-185` — `autoCalibrate` min branch dead store | fixed-in-port (adjustment applied after the read) |
| 157 | urRobot `rtde_receive_driver.cpp:30-36`, `rtde_control_driver.cpp:66-77` — `try_connect()` false when already connected: RECONNECT on a healthy link answers asynError | fixed-in-port (live connection answers success) |
| 158 | urRobot `rtde_control_driver.cpp:526-540` — unopenable script / disconnected ScriptClient falls through to `asynSuccess` with the error flag clear | fixed-in-port (flag raised, error returned) |
| 159 | ur_rtde `dashboard_client.cpp:342-360` — `setUserRole` switch with no `break`s: every role falls to "restricted" | observed only (urRobot never calls it; not ported) |
| 160 | urRobot `rtde_control_driver.cpp:377-386` — comment says m→mm, code does mm→m | fixed-in-port (comment corrected) |
| — | ur_rtde `GripperConfig::MIN_POSITION_STOP_ADJUST = -5` applied to both calibration ends (widens open, narrows closed); intended sign underivable without the Robotiq spec | unfixable-without-spec (literal kept) |

## epics-modules/ip (commits `496b5c1`…`b577ef9`)

| # | Defect | Port handling |
|---|--------|---------------|
| 161 | `devAiMKS.c:317-347` — `pai->val = 0.` before decoding: any non-pressure reply publishes 0 Torr, and on the 937's spurious `SYNTAX!`/`NotCMD!` replies the alarm is also suppressed — false perfect vacuum as good data | fixed-in-port (non-pressure reply never writes the pressure; status carries reason + severity) |
| 162 | `devXxEurotherm.c:272-273` — `strncat` bounded by `sizeof(buffer)-strlen(buffer)` instead of `−1`: 95-char payload writes `buffer[100]` (unreachable today only because stringout VAL is 40 bytes) | fixed-in-port (frame is a length-built `Vec<u8>`) |
| 163 | `devAiHeidND261.c:229-231` — `completeIO` chops two more bytes after asyn's EOS already stripped `\n\n`: always drops the last two data characters | unfixable-without-spec (whether they are digits or unit chars needs the ND261 manual, absent; port does not chop — correct or identical under both readings) |

## epics-modules/Yokogawa_DAS (commits `807d2e5` GM10, `90c33a2` MW100)

| # | Defect | Port handling |
|---|--------|---------------|
| 164 | `GM10_pulse_input_channel.db:77,111,145,179` — THSV `"MAJR"` in all four Alarm records (not a legal severity string; every sibling family spells `"MAJOR"`) | fixed-in-port |
| 165 | `devGM10_bo.c:120-139` vs `GM10_system.db` — bo dset dispatches INFO_TRIG, db never wires a record to it | fixed-in-port (`InfoPoll` added, mirroring the other three pollers) |
| 166 | `devMW100_bo.c` INFO_TRIG vs `MW100_system.db` — same gap as GM10 | fixed-in-port (`InfoPoll`) |
| 167 | `devMW100_bi.c` MEASURE_MODE vs `MW100_system.db` — command exists, no record wired (its two sibling status bits have records) | fixed-in-port (`MeasureMode` bi) |
| 168 | `MW100_MX114_channel.db` + `MW100_MX115_channel.db` — every Alarm1-4 `THSV` reads `"MAJR"` (8 sites) against seven correctly-spelled `"MAJOR"` in the same records | fixed-in-port |

## epics-modules/tpmac (commit `5b71200`)

| # | Defect | Port handling |
|---|--------|---------------|
| 169 | `pmacAsynIPPort.c` documents the PMAC error reply `<BELL>ERRxxx<CR>` in its own header, yet `lowLevelWriteRead`/`motorAxisWriteRead` report success for it — every move/home/stop/set-position can fail silently | fixed-in-port (`controller::octet_write_read` errors on a BELL reply) |
| 170 | `pmacCsGroups.cpp` `switchToGroup` indexes an axis-keyed `std::map` with the loop counter: any group whose axes are not exactly `0..n-1` maps wrong, and `operator[]` grows the map mid-iteration | fixed-in-port (regression test `switch_maps_a_non_contiguous_axis_set_correctly`) |
| 171 | `pmacAsynCoord.c` `motorAxisMove` ignores its `relative` argument: a REL move on a CS axis drives to the absolute position | fixed-in-port |
| 172 | `pmacAsynCoord.c` `drvPmacGetAxesStatus` sets `motorAxisProblem` from `CS_STATUS2_AMP_FAULT`, then immediately overwrites it with `CS_STATUS2_RUNTIME_ERR`: an amp fault never reaches the record | fixed-in-port (both bits raise PROBLEM) |
| 173 | `pmacAsynIPPort.c` `writeIt` sends the 16-bit `wValue` in host order while `htons`-ing `wLength`: wrong packet header on a big-endian IOC | fixed-in-port (written little-endian explicitly) |
| 174 | `pmacController.cpp` `processDeferredMoves` + `pmacCsGroups` build commands with `sprintf(buf, "%s…", buf, …)` — overlapping src/dst, UB | fixed-in-port (structurally absent) |
| — | `pmacAxis::getAxisStatus` writes `motorStatusPowerOn_` twice per poll (`!(status1 & OPEN_LOOP)` then `amp_enabled`); which was intended is underivable | observed only (port keeps the observable behaviour: powered = amp_enabled) |
| — | `PMAC_FEEDRATE_LIM_ = 100` defined and never used | observed only (not reproduced) |

## epics-modules/twincat-ads (commits `ec4464c`, `6c31da6`, `3dee54e`)

| # | Defect | Port handling |
|---|--------|---------------|
| 175 | `adsAsynPortDriverUtils.cpp:818,850` — `octetBinary2ascii` INT64/UINT64 formats are `"% PRId64"`/`"% PRIu64"` with the macro *inside* the literal, so it never expands: reading an LINT over the octet interface returns the literal text ` PRId64` | fixed-in-port |
| 176 | `adsAsynPortDriverUtils.cpp:837,845` — UINT16/UINT32 printed with `%d`: a UDINT above 2³¹ renders negative (`4000000000` → `-294967296`) | fixed-in-port |
| 177 | `adsAsynPortDriverUtils.cpp:560` — `windowsToEpicsTimeStamp` computes the sub-second remainder as `plcTime - secPastEpoch * WINDOWS_TICK_PER_SEC` in 32-bit arithmetic (`uint32_t` × `int`): wraps for every real timestamp, `nsec` is garbage on every sample | fixed-in-port |
| 178 | `adsAsynPortDriver.cpp:1557` — `parsePlcInfofromDrvInfo` finds options with `strstr` over the whole drvInfo: a PLC symbol containing an option keyword (`Main.TS_MS_setpoint`, `Main.bADSPORT_OK`) mis-parses and the record fails to bind | fixed-in-port (structural parse) |
| 179 | `adsAsynPortDriver.cpp:1622-1625` — the `.ADR.` parse-failure path assigns `-1` to unsigned fields, leaving `SIZE_MAX` behind before returning the error | fixed-in-port |
| 180 | `adsAsynPortDriver.cpp:2229` — the octet symbolic-*write* path `strncpy`s the variable name without NUL termination into a buffer reused across stacked commands (the read path at `:2244` terminates) | fixed-in-port |
| 181 | `adsAsynPortDriver.cpp:4600` — the `ADST_BIT` arm of `adsUpdateParameter` omits `asynParamInt64`, which every other integer PLC type accepts | fixed-in-port |
| 182 | `adsAsynPortDriver.cpp:4574` et al — float→integer casts through `(int)`: UB past the integer range | fixed-in-port (Rust `as` saturates) |
| 183 | `adsAsynPortDriver.cpp:674` vs `:1394` — on a failed sub-request `bulkReadThread` advances the *status* pointer but not the *data* pointer, while `adsAddToBulkRead` sizes the read as if every sub-request occupies its bytes; the two cannot both hold, so one failing variable shifts every later variable's bytes — silent data corruption. The Beckhoff reference (`AdsDef.h:71`) documents the 0xF080 response only as "{list of results} and {list of data}" | fixed-in-port without guessing: the decoder disambiguates the layout off the wire length (the two candidates predict different totals whenever a sub-request fails), and rejects a response matching neither |
| 184 | `adsAsynPortDriver.cpp:4716-4790` — `fireCallbacks` passes `lastCallbackSize` (**bytes**) as the **element count** to `doCallbacksInt16Array`/`Int32Array`/`Float32Array`/`Float64Array`: a 100-element LREAL array is published as 800 elements read out of a 100-element buffer — OOB read on every array record | fixed-in-port (regression test `an_array_sample_is_served_element_wise_not_byte_wise`) |
| 185 | `adsAsynPortDriver.cpp:3223` — `writeFloat64Array` copies `nElements * nElements * sizeof(epicsFloat64)` bytes | fixed-in-port |
| 186 | `adsAsynPortDriver.cpp:496` — destructor `delete`s a `new[]` array | fixed-in-port (structurally absent) |
| 187 | `adsAsynPortDriver.cpp:306` — constructor `memset(pAdsParamArray_, 0, sizeof(*pAdsParamArray_))` zeroes 8 bytes instead of the parameter table | fixed-in-port (structurally absent) |
| 188 | `adsAsynPortDriver.cpp:346-358` — the `if (nvals != 6)` check is duplicated | fixed-in-port |
| 189 | `adsAsynPortDriver.cpp:552`, `:3565` — `exit(-1)`/`exit(1)` on connection loss kills the IOC with the PLC; the constructor's `for(;;) connect()` also blocks `iocInit` forever when the PLC is off | fixed-in-port (reconnect supervisor re-resolves every handle; first connect non-blocking) |
| 190 | `adsAsynPortDriver.cpp:3800-3806` — the symbol handle is zeroed before the `printf` that logs it: the failure message always reads `0xffffffff` | fixed-in-port |
| 191 | `adsAsynPortDriver.cpp:1260` — `unlock()` with no matching `lock()` on the alloc-failure path | fixed-in-port (structurally absent) |
| 192 | `adsAsynPortDriver.cpp:1307` — `poll_info` labels bulk slot 0 `timeLoDW`; it is `timeHiDW` | fixed-in-port |
| 193 | `adsAsynPortDriverUtils.cpp:409-419` — `adsTypeSize` returns `-1` as a `size_t` (`SIZE_MAX`) for an unknown type | fixed-in-port |
| 194 | `adsAsynPortDriver.cpp:2651`, `:2802` — `printf` passes `plcDataType` where the message text says "bytes" | fixed-in-port |
| 195 | iocsh arg label `"(EPCIS=0,PLC=1)"` contradicts `ADS_TIME_BASE_PLC = 0` (`adsAsynPortDriverUtils.h:59-60`), typo included | fixed-in-port |
| 196 | `adsExApp/Db/adsTestAsyn.db` — `SetFAmplitudeRB` and `SetBEnableUpdateSineRB` each defined twice; `$(ADSCLIENT)`/`$(ADSSERVER_PORT)` referenced but defined by no startup script; `"$(P):Int32Array"` stray colon | fixed-in-port |

## epics-modules/opcua (commits `805f587`, `c40df9c`, `a701836`, `e6a2801`)

| # | Defect | Port handling |
|---|--------|---------------|
| 197 | `linkParser.cpp:225-300` + `devOpcua.h:95-97` — a link naming a session or subscription but carrying neither `i=` nor `s=` is accepted: `linkInfo::identifierNumber` is never initialized and `identifierIsNumeric` defaults false, so the item silently addresses `ns=0;s=""` and the configuration error surfaces only as a one-shot BadNodeIdUnknown log at the server | fixed-in-port (link parse error) |
| 198 | `linkParser.cpp:234-239` — escaped-separator handling erases the backslash in place and re-finds the separator, but keeps the `=` position computed *before* the erase: an escaped separator left of the `=` shifts the option name/value split by one character | fixed-in-port (unescapes while scanning; no position survives a string mutation) |
| 199 | `DataElementOpen62541Leaf.cpp:1066-1073` — the DOUBLE arm of `writeScalar(const char*, len)` sets `outgoingData` but never calls `markAsDirty()` and never clears `ret` (compare FLOAT at `:1056-1064`): a stringout/lso record writing to a Double node always raises WRITE_ALARM/INVALID "value out of range" and never sends anything | fixed-in-port |
| 200 | `DataElementOpen62541Leaf.h:48-51` — `isWithinRange` is `!(v < lowest || v > max)`, which passes NaN and ±inf (both comparisons false) into an undefined-behaviour `static_cast`; for an `epicsInt64` target it also admits exactly 2⁶³ because `INT64_MAX`→`double` rounds up | fixed-in-port (rejected as out-of-range) |
| 201 | `DataElementOpen62541Leaf.h:150-157` — `string_to(const std::string&, epicsInt32&)` assigns a 64-bit `std::stol` result with no range check: a STRING node holding "99999999999" silently truncates into a longin (only the `epicsUInt32` overload at `:168-176` checks) | fixed-in-port |
| 202 | `UpdateQueue.h:64-68` — on overrun with `discardOldest` the C pops the front then dereferences `updq.front()` again for the loss count; at capacity 1 (reachable via `cqsize=1` or `opcua_MinimumClientQueueSize=1`) the queue is empty there — `std::queue::front()` on empty is UB | fixed-in-port (loss count goes to the update being pushed when nothing else remains; no capacity is special) |
| 203 | `SessionOpen62541.cpp:463-467`, `:660`, `Session.h:359` — the session hands the server's *node* limit (`MaxNodesPerRead`) to the batcher as its *items*-per-batch limit while every item contributes two nodes: a server stating `MaxNodesPerRead = 100` is sent Reads of 200 nodes | fixed-in-port (item limit = node limit / attributes-per-item; one chunking function for every read and write) |
| 204 | `SessionOpen62541.cpp:2555-2578` — `readComplete` guards the DataType result with `i >= response->resultsSize`, then after `i++` reads the Value result unchecked: a server answering with fewer results than asked walks off the array | fixed-in-port (both results checked; a missing one is a read failure for that item) |
| 205 | `SessionOpen62541.cpp:1322-1330` — `markConnectionLoss` clears the reader and writer queues but not `outstandingOps`: transactions of a lost connection survive into the next one as "unknown transaction id" log noise | fixed-in-port (a connection owns its calls; pending queues cleared when it goes down) |
| 206 | `SessionOpen62541.cpp:817` — `registerNodes` prints every registered item to stdout unconditionally (`it->show(0)` outside any debug guard) | fixed-in-port (one debug log line) |
| 207 | `SessionOpen62541.cpp:862` — `addNamespaceMapping` opens with a leftover `errlogPrintf` debug line, its message missing the space before "index" | fixed-in-port (dropped) |
| 208 | `devOpcua.cpp:816`, `:908-910` — the mbbo and mbboDirect read branches shift `prec->rval` in place, so RVAL holds the shifted value (RVAL == VAL for mbboDirect) until the record's next forward conversion recomputes it | fixed-in-port (RVAL keeps the raw the server sent, masked to the node's bit window) |
| 209 | `devOpcua.cpp:783`, `:891` — a readRequest on an *output* record falls through to the read branch, which pops an update that a read request never queued | fixed-in-port (requests the read it names) |
| — | `RecordConnector.cpp:56-77` vs `Update.h:45` — the reason a record processes for is kept in two places (`pconnector->reason` and inside the queued update), free to fall out of step | observed only (port keeps the queue as the single source) |
| — | `mbbiDirectRecord.c:157-162` (epics-base) only shifts where epics-rs masks with MASK then shifts (asyn convention): with NOBT set, out-of-window bits the server sends are dropped in the port and kept in C | observed only (framework-carried deviation, not an opcua defect) |
