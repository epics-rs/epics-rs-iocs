//! Workspace conventions enforced by test, not by review.
//!
//! `tests/db_paths.rs` holds the single rule for EPICS database loading:
//! every `dbLoadRecords`/`dbLoadTemplate` path in `iocs/**/*.cmd` is
//! `$(MACRO)/db/<file>`, the macro has a `set_default` or `set_default_path`
//! in the owning IOC's Rust sources, and the file exists where the macro
//! points.
//!
//! `tests/cfloat.rs` runs epics-rs's `cfloat-gate` over every member: a
//! precision or exponent float placeholder formats a `CFloat`, C's spelling.
