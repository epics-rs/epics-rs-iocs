//! Rust port of `epics-modules/vac`.
//!
//! Two device families, each a custom record type plus an asyn-octet device
//! support:
//!
//! * `vs` + `devVacSen` — vacuum gauge controllers (Granville-Phillips GP307
//!   and GP350, Televac MM200, CC10 and MX200).
//! * `digitel` + `devDigitelPump` — ion pump controllers (Gamma Vacuum MPC and
//!   QPC, Perkin-Elmer Digitel 500 and 1500).
//!
//! # EOS ownership
//!
//! Neither C device support calls `setInputEos` or `setOutputEos`. Every
//! terminator is configured by the startup script, and this port keeps that
//! split: the protocol modules emit bare commands and the shipped `st.cmd`
//! files carry the `asynOctetSetInputEos` / `asynOctetSetOutputEos` calls.
//!
//! # Deviations from the C source
//!
//! Faithfulness to the wire protocol is the priority, so upstream's parsing
//! quirks — sticky `sscanf` targets, fall-through `switch` blocks, stale
//! command buffers — are reproduced rather than fixed. Where the C code has
//! undefined behaviour there is nothing to reproduce, and this port picks a
//! defined answer:
//!
//! * `devVacSen.c` leaves `char sign; int exp;` uninitialised at the top of
//!   `readWrite_vs`. This port seeds them `('+', 0)`.
//! * `devVacSen.c`'s MX200 init ignores `sscanf`'s return value, so a short
//!   `userParam` leaves the station variables indeterminate. This port requires
//!   all four station parameters and rejects the link otherwise.
//! * `devVacSen.c`'s MX200 relay recode is `*readBuffer = sprintf("%2x", value)`
//!   — `sprintf` into a string literal. This port writes into the reply buffer,
//!   the only form the decoder can consume.
//! * `devVacSen.c` jumps to `finish:` on a control-command failure, before
//!   `responseBuffer` is zeroed, and copies uninitialised stack into `recBuf`.
//!   This port zeroes the buffer.
//! * `devDigitelPump.c` leaves `t1`, `val1` and `val2` uninitialised when no
//!   `spfg` bit matches. This port starts them at zero, so no command is sent.
//! * `devDigitelPump.c` reads an uninitialised `nwrite` into `*nread` when a
//!   QPC command is shorter than ten characters. This port reports zero bytes
//!   read, which is the "Cmd reply too small" path.
//! * `devDigitelPump.c` indexes `readBuffer[4]` and `readBuffer[5]` on replies
//!   shorter than that, reading the previous reply's bytes. This port reads NUL.
//!
//! These upstream bugs are *not* fixed, because fixing them would change the
//! wire behaviour a working installation depends on:
//!
//! * `devVacSen.c`'s `monitor()` tests `chgc & IGn_FIELD` after `readWrite_vs`
//!   has already zeroed `chgc`, so those branches never run.
//! * `vsRecord.c`'s `checkAlarms` assigns `val = pvs->val` and then immediately
//!   `val = pvs->pres`, so `PRES` is what is alarm-checked.
//! * `devDigitelPump.c`'s `sprintf(pvalue, "S32%.0e", pr->sp3s)` reads `sp3s`
//!   where the S22/S12 analogues read `s2hs`/`s1hs`.
//! * `devDigitelPump.c`'s setpoint range guards `v < 1e-4 || v > 1e-11` are
//!   true for every non-negative `v`; `&&` was meant.
//! * `devDigitelPump.c`'s Digitel `case 3:` writes `s2mr`/`s2vr` where it means
//!   `s3mr`/`s3vr`, and the `case 2:` fall-through then overwrites them.
//! * `devDigitelPump.c` leaves `pvalue` holding the previous slot's command at
//!   MPC slot 8, so `SP4R` mirrors `SP3R`; and the QPC send guard skips slots
//!   6-8 entirely, so their replies duplicate slot 5's.
//! * `devDigitelPump.c`'s `strncpy(pvalue, &recBuf[139], 2)` writes two bytes
//!   and no terminator, so the following `sscanf(pvalue, "%lf", &pr->s3tr)`
//!   reads on into the previous copy's tail. `S3TR` never holds the bakeout
//!   time the reply carries.

pub mod device_support;
pub mod ioc;
pub mod protocol;
pub mod records;
