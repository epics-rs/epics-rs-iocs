//! The `vs` vacuum-gauge record (`vsRecord.c` / `vsRecord.dbd`).

use std::any::Any;

use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::{
    AlarmSeverity, CommonFields, DeclaredFields, FieldDesc, FieldIx, FieldMetadataOverride,
    ProcessOutcome, Record, declared_set,
};
use epics_rs::base::types::EpicsValue;

use super::dbd_generated::VS_FIELDS;
use super::{rec_gbl_alarm, rec_gbl_range, record_fields, set_sevr, severity_of};

/// `chgc` bits — `vsRecord.c:60-66`. `special()` sets one per `SPC_MOD` field
/// touched since the last process; `readWrite_vs` consumes them.
pub const IG1_FIELD: u16 = 0x0001;
pub const IG2_FIELD: u16 = 0x0002;
pub const DGS_FIELD: u16 = 0x0004;
pub const SP1_FIELD: u16 = 0x0010;
pub const SP2_FIELD: u16 = 0x0020;
pub const SP3_FIELD: u16 = 0x0040;
pub const SP4_FIELD: u16 = 0x0080;

/// `vsOFFON_On`.
const ON: u16 = 1;

#[derive(Debug, Clone)]
pub struct VsRecord {
    /// `field(INP,DBF_INLINK)` — the record owns the field (`vsRecord.dbd`);
    /// the loader also mirrors the text into the common INP, which is where
    /// device support reads it.
    pub inp: String,
    pub tipe: u16,
    pub err: i16,
    pub prec: i16,

    pub ig1s: u16,
    pub ig2s: u16,
    pub dgss: u16,
    pub ig1r: u16,
    pub ig2r: u16,
    pub dgsr: u16,
    pub fltr: u16,
    pub sp: [u16; 6],

    pub sps: [f64; 4],
    pub spr: [f64; 4],

    pub val: f64,
    pub pres: f64,
    pub cgap: f64,
    pub cgbp: f64,
    pub lprs: f64,
    pub lcap: f64,
    pub lcbp: f64,
    pub chgc: u16,

    pub hopr: f32,
    pub lopr: f32,
    pub hlpr: f32,
    pub llpr: f32,
    pub hapr: f32,
    pub lapr: f32,
    pub halr: f32,
    pub lalr: f32,
    pub hbpr: f32,
    pub lbpr: f32,
    pub hblr: f32,
    pub lblr: f32,

    // `monitor()`'s previous-value shadows.
    pub pi1s: u16,
    pub pi2s: u16,
    pub pdss: u16,
    pub pig1: u16,
    pub pig2: u16,
    pub pdgs: u16,
    pub pflt: u16,
    pub psp: [u16; 6],
    pub pss: [f64; 4],
    pub psr: [f64; 4],
    pub pval: f64,
    pub ppre: f64,
    pub pcga: f64,
    pub pcgb: f64,
    pub plpe: f64,
    pub plca: f64,
    pub plcb: f64,

    pub hihi: f32,
    pub lolo: f32,
    pub high: f32,
    pub low: f32,
    pub hhsv: u16,
    pub llsv: u16,
    pub hsv: u16,
    pub lsv: u16,
    pub hyst: f64,
    pub lalm: f64,

    /// Set once device support has completed a read cycle. C's `readWrite_vs`
    /// clears `pr->udf` on every `pact == 1` return, including both error
    /// returns, so UDF is cleared as soon as the device support has run at all.
    pub dev_ran: bool,
    /// The `recGblSetSevr(pr, <stat>, INVALID_ALARM)` device support raised:
    /// READ_ALARM from a reply, COMM_ALARM from a refused `queueRequest`.
    pub dev_alarm: Option<u16>,
}

impl Default for VsRecord {
    fn default() -> Self {
        Self {
            inp: String::new(),
            tipe: 0,
            err: 5,
            prec: 1,
            ig1s: 0,
            ig2s: 0,
            dgss: 0,
            ig1r: 0,
            ig2r: 0,
            dgsr: 0,
            fltr: 0,
            sp: [0; 6],
            sps: [0.0; 4],
            spr: [0.0; 4],
            val: 0.0,
            pres: 0.0,
            cgap: 0.0,
            cgbp: 0.0,
            lprs: 0.0,
            lcap: 0.0,
            lcbp: 0.0,
            chgc: 0,
            hopr: 1e-4,
            lopr: 1e-12,
            hlpr: -4.0,
            llpr: -12.0,
            hapr: 1000.0,
            lapr: 1e-4,
            halr: 3.0,
            lalr: -4.0,
            hbpr: 1000.0,
            lbpr: 1e-4,
            hblr: 3.0,
            lblr: -4.0,
            pi1s: 0,
            pi2s: 0,
            pdss: 0,
            pig1: 0,
            pig2: 0,
            pdgs: 0,
            pflt: 0,
            psp: [0; 6],
            pss: [0.0; 4],
            psr: [0.0; 4],
            pval: 0.0,
            ppre: 0.0,
            pcga: 0.0,
            pcgb: 0.0,
            plpe: 0.0,
            plca: 0.0,
            plcb: 0.0,
            hihi: 1e-6,
            lolo: 1e-12,
            high: 1e-7,
            low: 2e-12,
            hhsv: 0,
            llsv: 0,
            hsv: 0,
            lsv: 0,
            hyst: 0.0,
            lalm: 0.0,
            dev_ran: false,
            dev_alarm: None,
        }
    }
}

// Array-backed fields are handled outside the macro.
record_fields! {
    get_scalar, put_scalar, VsRecord;
    "TYPE": Enum = tipe;
    "ERR": Short = err;
    "PREC": Short = prec;
    "IG1S": Enum = ig1s;
    "IG2S": Enum = ig2s;
    "DGSS": Enum = dgss;
    "IG1R": Enum = ig1r;
    "IG2R": Enum = ig2r;
    "DGSR": Enum = dgsr;
    "FLTR": Enum = fltr;
    "VAL": Double = val;
    "PRES": Double = pres;
    "CGAP": Double = cgap;
    "CGBP": Double = cgbp;
    "LPRS": Double = lprs;
    "LCAP": Double = lcap;
    "LCBP": Double = lcbp;
    "CHGC": UShort = chgc;
    "HOPR": Float = hopr;
    "LOPR": Float = lopr;
    "HLPR": Float = hlpr;
    "LLPR": Float = llpr;
    "HAPR": Float = hapr;
    "LAPR": Float = lapr;
    "HALR": Float = halr;
    "LALR": Float = lalr;
    "HBPR": Float = hbpr;
    "LBPR": Float = lbpr;
    "HBLR": Float = hblr;
    "LBLR": Float = lblr;
    "PI1S": Enum = pi1s;
    "PI2S": Enum = pi2s;
    "PDSS": Enum = pdss;
    "PIG1": Enum = pig1;
    "PIG2": Enum = pig2;
    "PDGS": Enum = pdgs;
    "PFLT": Enum = pflt;
    "PVAL": Double = pval;
    "PPRE": Double = ppre;
    "PCGA": Double = pcga;
    "PCGB": Double = pcgb;
    "PLPE": Double = plpe;
    "PLCA": Double = plca;
    "PLCB": Double = plcb;
    "HIHI": Float = hihi;
    "LOLO": Float = lolo;
    "HIGH": Float = high;
    "LOW": Float = low;
    "HHSV": Enum = hhsv;
    "LLSV": Enum = llsv;
    "HSV": Enum = hsv;
    "LSV": Enum = lsv;
    "HYST": Double = hyst;
    "LALM": Double = lalm;
}

/// `SP<n>`/`PSP<n>` (1..=6) and `SP<n>S`/`SP<n>R`/`PS<n>S`/`PS<n>R` (1..=4).
fn indexed(name: &str) -> Option<(&'static str, usize)> {
    let (tag, digit, limit) = match name.as_bytes() {
        [b'S', b'P', d, b'S'] => ("SPnS", d, 4),
        [b'S', b'P', d, b'R'] => ("SPnR", d, 4),
        [b'P', b'S', d, b'S'] => ("PSnS", d, 4),
        [b'P', b'S', d, b'R'] => ("PSnR", d, 4),
        [b'S', b'P', d] => ("SPn", d, 6),
        [b'P', b'S', b'P', d] => ("PSPn", d, 6),
        _ => return None,
    };
    let n = (*digit as char).to_digit(10)? as usize;
    (1..=limit).contains(&n).then_some((tag, n - 1))
}

impl VsRecord {
    fn get_indexed(&self, name: &str) -> Option<EpicsValue> {
        let (tag, i) = indexed(name)?;
        Some(match tag {
            "SPn" => EpicsValue::Enum(self.sp[i]),
            "PSPn" => EpicsValue::Enum(self.psp[i]),
            "SPnS" => EpicsValue::Double(self.sps[i]),
            "SPnR" => EpicsValue::Double(self.spr[i]),
            "PSnS" => EpicsValue::Double(self.pss[i]),
            _ => EpicsValue::Double(self.psr[i]),
        })
    }

    fn put_indexed(&mut self, name: &str, value: EpicsValue) -> Option<CaResult<()>> {
        let (tag, i) = indexed(name)?;
        Some(match (tag, value) {
            ("SPn", EpicsValue::Enum(v)) => {
                self.sp[i] = v;
                Ok(())
            }
            ("PSPn", EpicsValue::Enum(v)) => {
                self.psp[i] = v;
                Ok(())
            }
            ("SPnS", EpicsValue::Double(v)) => {
                self.sps[i] = v;
                Ok(())
            }
            ("SPnR", EpicsValue::Double(v)) => {
                self.spr[i] = v;
                Ok(())
            }
            ("PSnS", EpicsValue::Double(v)) => {
                self.pss[i] = v;
                Ok(())
            }
            ("PSnR", EpicsValue::Double(v)) => {
                self.psr[i] = v;
                Ok(())
            }
            _ => Err(epics_rs::base::error::CaError::TypeMismatch(name.into())),
        })
    }

    /// C `vsRecord.c::checkAlarms`, minus the leading `udf` branch, which the
    /// framework's `rec_gbl_check_udf` owns.
    ///
    /// Fixes doc/upstream-c-defects.md #19: `checkAlarms` loaded `val = pvs->val`
    /// then immediately overwrote it with `val = pvs->pres` (marked
    /// `/* need to be removed someday */` in `vsRecord.c:347`), so `PRES` — not
    /// `VAL` — was alarm-checked. `VAL` is the field the `.dbd` documents as the
    /// gauge pressure the alarm limits guard, so the limits are now applied to
    /// `VAL`.
    fn limit_alarms(&mut self, common: &mut CommonFields) {
        let (hihi, lolo, high, low) = (self.hihi, self.lolo, self.high, self.low);
        let val = self.val;
        let hyst = self.hyst;
        let lalm = self.lalm;

        // C's locals are `float hihi, ...; double hyst, lalm;`, so `hihi - hyst`
        // widens hihi to double. The `.dbd` types already match.
        for (sevr_index, stat, limit, over) in [
            (self.hhsv, alarm_status::HIHI_ALARM, hihi as f64, true),
            (self.llsv, alarm_status::LOLO_ALARM, lolo as f64, false),
            (self.hsv, alarm_status::HIGH_ALARM, high as f64, true),
            (self.lsv, alarm_status::LOW_ALARM, low as f64, false),
        ] {
            let sevr = severity_of(sevr_index);
            if sevr_index == 0 {
                continue;
            }
            let tripped = if over {
                val >= limit || (lalm == limit && val >= limit - hyst)
            } else {
                val <= limit || (lalm == limit && val <= limit + hyst)
            };
            if tripped {
                if set_sevr(common, stat, sevr) {
                    self.lalm = limit;
                }
                return;
            }
        }
        // Out of alarm by at least HYST.
        self.lalm = val;
    }
}

impl Record for VsRecord {
    fn record_type(&self) -> &'static str {
        "vs"
    }

    fn get_field(&self, name: &str) -> Option<EpicsValue> {
        if name == "INP" {
            return Some(EpicsValue::String(self.inp.clone().into()));
        }
        get_scalar(self, name).or_else(|| self.get_indexed(name))
    }

    fn put_field(&mut self, name: &str, value: EpicsValue) -> CaResult<()> {
        if name == "INP" {
            return match value {
                EpicsValue::String(v) => {
                    self.inp = v.as_str_lossy().into_owned();
                    Ok(())
                }
                _ => Err(CaError::TypeMismatch(name.into())),
            };
        }
        match self.put_indexed(name, value.clone()) {
            Some(r) => r,
            None => put_scalar(self, name, value),
        }
    }

    fn declared_fields(&self) -> &'static [FieldDesc] {
        VS_FIELDS
    }

    /// `pp(TRUE)` in `vsRecord.dbd`.
    fn process_passive_fields(&self) -> &'static [&'static str] {
        &[
            "IG1S", "IG2S", "DGSS", "SP1S", "SP2S", "SP3S", "SP4S", "HIHI", "LOLO", "HIGH", "LOW",
            "HHSV", "LLSV", "HSV", "LSV",
        ]
    }

    /// C `vsRecord.c::special` — record which `SPC_MOD` field was written.
    fn special(&mut self, field: &str, after: bool, _common: &CommonFields) -> CaResult<()> {
        if !after {
            return Ok(());
        }
        self.chgc |= match field {
            "IG1S" => IG1_FIELD,
            "IG2S" => IG2_FIELD,
            "DGSS" => DGS_FIELD,
            "SP1S" => SP1_FIELD,
            "SP2S" => SP2_FIELD,
            "SP3S" => SP3_FIELD,
            "SP4S" => SP4_FIELD,
            _ => 0,
        };
        Ok(())
    }

    /// C `vsRecord.c::process`'s tail: `monitor()` latches the previous-value
    /// shadows, then `chgc` is cleared.
    ///
    /// doc/upstream-c-defects.md #18 — not applicable in this framework. C's
    /// `monitor()` posted the `IG1S`/`IG2S`/`DGSS` setting fields via
    /// `pvs->chgc & IGn_FIELD`, because EPICS base does not auto-post a field on
    /// `dbPutField`. Those branches were already dead in C: `readWrite_vs` zeroed
    /// `chgc` during the `pact == 0` pass, before `monitor()` ever runs. And
    /// here there is no observable to restore even if they fired: `IG1S`/`IG2S`/
    /// `DGSS` are `pp(TRUE)` non-primary fields, so the framework posts them at
    /// caput time (and again on value change via the shadow), by construction.
    /// The port therefore keeps the unconditional shadow latch below and adds no
    /// change-flag-gated posts.
    fn process(&mut self) -> CaResult<ProcessOutcome> {
        self.pval = self.val;
        self.ppre = self.pres;
        self.pi1s = self.ig1s;
        self.pi2s = self.ig2s;
        self.pdss = self.dgss;
        self.pss = self.sps;
        self.pig1 = self.ig1r;
        self.pig2 = self.ig2r;
        self.pdgs = self.dgsr;
        self.pflt = self.fltr;
        self.psp = self.sp;
        self.psr = self.spr;
        self.plpe = self.lprs;
        self.pcga = self.cgap;
        self.plca = self.lcap;
        self.pcgb = self.cgbp;
        self.plcb = self.lcbp;
        self.chgc = 0;
        Ok(ProcessOutcome::complete())
    }

    /// C posts every readback field with the cycle's alarm mask when the alarm
    /// transitioned, even if its value did not change (`monitor()`'s
    /// `|| (alrm_chg_flg)` clauses). The `*S` setting fields are not in that
    /// list — their posts are gated on `chgc` / their own shadow.
    fn alarm_cycle_monitored_fields(&self) -> DeclaredFields {
        static MONITORED: [FieldIx; 21] = declared_set(
            VS_FIELDS,
            [
                "VAL", "PRES", "IG1R", "IG2R", "DGSR", "FLTR", "SP1", "SP2", "SP3", "SP4", "SP5",
                "SP6", "SP1R", "SP2R", "SP3R", "SP4R", "LPRS", "CGAP", "LCAP", "CGBP", "LCBP",
            ],
        );
        &MONITORED
    }

    fn check_alarms(&mut self, common: &mut CommonFields) {
        if let Some(stat) = self.dev_alarm {
            set_sevr(common, stat, AlarmSeverity::Invalid);
        }
        // C returns immediately on UDF; the framework's `rec_gbl_check_udf`
        // raises `UDF_ALARM` at `UDFS`.
        if common.udf != 0 {
            return;
        }
        self.limit_alarms(common);
    }

    fn value_is_undefined(&self) -> bool {
        !self.dev_ran
    }

    /// C `get_precision` / `get_graphic_double` / `get_control_double` /
    /// `get_alarm_double`.
    fn field_metadata_override(&self, field: &str) -> Option<FieldMetadataOverride> {
        let precision = Some(match field {
            "VAL" | "PRES" | "CGAP" | "CGBP" | "SP1R" | "SP2R" | "SP3R" | "SP4R" => self.prec,
            "LPRS" | "LCAP" | "LCBP" => self.prec + 1,
            _ => 0,
        });

        let disp_limits = match field {
            "VAL" | "PRES" => Some((self.hopr as f64, self.lopr as f64)),
            "CGAP" => Some((self.hapr as f64, self.lapr as f64)),
            "CGBP" => Some((self.hbpr as f64, self.lbpr as f64)),
            "LPRS" => Some((self.hlpr as f64, self.llpr as f64)),
            "LCAP" => Some((self.halr as f64, self.lalr as f64)),
            "LCBP" => Some((self.hblr as f64, self.lblr as f64)),
            _ => rec_gbl_range(VS_FIELDS, field),
        };

        let ctrl_limits = match field {
            "VAL" | "PRES" | "CGAP" | "CGBP" => Some((self.hopr as f64, self.lopr as f64)),
            _ => rec_gbl_range(VS_FIELDS, field),
        };

        // Alarm limits are published only while an ion gauge is switched on;
        // with both off the pressure is the 9.9e9 sentinel and the limits would
        // be meaningless.
        let alarm_limits = match field {
            "VAL" | "PRES" if self.ig1s == ON || self.ig2s == ON => Some((
                self.hihi as f64,
                self.high as f64,
                self.low as f64,
                self.lolo as f64,
            )),
            "VAL" | "PRES" => Some((0.0, 0.0, 0.0, 0.0)),
            _ => rec_gbl_alarm(VS_FIELDS, field),
        };

        Some(FieldMetadataOverride {
            precision,
            disp_limits,
            ctrl_limits,
            alarm_limits,
            ..FieldMetadataOverride::default()
        })
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn Any> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The framework clears `udf` (via `clears_udf` / `value_is_undefined`)
    /// before it calls `check_alarms`, so a defined record reaches the limit
    /// check with `udf == false`. `CommonFields::default()` starts at the
    /// dbCommon `UDF = 1`.
    fn common() -> CommonFields {
        let mut common = CommonFields::default();
        common.udf = 0;
        common
    }

    #[test]
    fn indexed_fields_round_trip() {
        let mut r = VsRecord::default();
        r.put_field("SP3S", EpicsValue::Double(1e-6)).unwrap();
        assert_eq!(r.sps[2], 1e-6);
        assert_eq!(r.get_field("SP3S"), Some(EpicsValue::Double(1e-6)));
        r.sp[5] = 1;
        assert_eq!(r.get_field("SP6"), Some(EpicsValue::Enum(1)));
        assert_eq!(r.get_field("SP7"), None);
        assert_eq!(r.get_field("SP5S"), None);
    }

    #[test]
    fn every_declared_field_is_readable() {
        let r = VsRecord::default();
        for f in r.declared_fields() {
            assert!(r.get_field(f.name).is_some(), "{} unreadable", f.name);
        }
    }

    #[test]
    fn special_records_the_changed_control_field() {
        let mut r = VsRecord::default();
        r.special("IG1S", true, &CommonFields::default()).unwrap();
        r.special("SP4S", true, &CommonFields::default()).unwrap();
        assert_eq!(r.chgc, IG1_FIELD | SP4_FIELD);
        // The "before" callback must not latch anything.
        let mut r = VsRecord::default();
        r.special("IG1S", false, &CommonFields::default()).unwrap();
        assert_eq!(r.chgc, 0);
    }

    #[test]
    fn process_latches_the_shadow_fields_and_clears_the_change_mask() {
        let mut r = VsRecord {
            val: 1e-7,
            pres: 1e-7,
            sp: [1, 0, 1, 0, 1, 0],
            chgc: IG1_FIELD,
            ..Default::default()
        };
        r.process().unwrap();
        assert_eq!(r.pval, 1e-7);
        assert_eq!(r.ppre, 1e-7);
        assert_eq!(r.psp, [1, 0, 1, 0, 1, 0]);
        assert_eq!(r.chgc, 0);
    }

    #[test]
    fn undefined_until_device_support_has_run() {
        let mut r = VsRecord::default();
        assert!(r.value_is_undefined());
        r.dev_ran = true;
        assert!(!r.value_is_undefined());
    }

    #[test]
    fn alarms_are_checked_against_val_not_pres() {
        // Regression for doc/upstream-c-defects.md #19: VAL above HIHI trips the
        // alarm even though PRES sits below every limit. Pre-fix (PRES-checked)
        // this stayed NO_ALARM.
        let mut r = VsRecord {
            dev_ran: true,
            hhsv: 2, // MAJOR
            hihi: 1e-6,
            val: 1e-5,   // above HIHI
            pres: 1e-12, // below every limit
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsta, alarm_status::HIHI_ALARM);
        assert_eq!(c.nsev, AlarmSeverity::Major);
        assert_eq!(r.lalm, 1e-6f32 as f64);
    }

    #[test]
    fn pres_no_longer_drives_the_alarm() {
        // The old defect alarmed on PRES; verify a high PRES with a safe VAL now
        // stays clear.
        let mut r = VsRecord {
            dev_ran: true,
            hhsv: 2, // MAJOR
            hihi: 1e-6,
            val: 1e-12, // below every limit
            pres: 1e-5, // above HIHI, but PRES is not checked anymore
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
    }

    #[test]
    fn a_zero_severity_disables_its_limit() {
        let mut r = VsRecord {
            dev_ran: true,
            hhsv: 0,
            hihi: 1e-6,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
        // Out of alarm: LALM latches the value.
        assert_eq!(r.lalm, 1e-5);
    }

    #[test]
    fn hysteresis_holds_the_alarm_until_the_value_backs_off() {
        let mut r = VsRecord {
            dev_ran: true,
            hhsv: 1, // MINOR
            hihi: 1e-6,
            hyst: 5e-7,
            val: 2e-6,
            ..Default::default()
        };
        r.check_alarms(&mut common());
        assert_eq!(r.lalm, 1e-6f32 as f64);

        // 7e-7 is below HIHI but within HYST of it, and LALM == HIHI.
        r.val = 7e-7;
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsta, alarm_status::HIHI_ALARM);

        // 4e-7 clears HIHI - HYST.
        r.val = 4e-7;
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
        assert_eq!(r.lalm, 4e-7);
    }

    #[test]
    fn the_first_tripped_limit_wins_and_latches_only_when_it_raises() {
        let mut r = VsRecord {
            dev_ran: true,
            hhsv: 1,
            hsv: 2,
            hihi: 1e-6,
            high: 1e-7,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        // HIHI is tested first and returns, even though HIGH is more severe.
        assert_eq!(c.nsta, alarm_status::HIHI_ALARM);
        assert_eq!(c.nsev, AlarmSeverity::Minor);
    }

    #[test]
    fn a_device_read_alarm_outranks_the_limit_alarms() {
        let mut r = VsRecord {
            dev_ran: true,
            dev_alarm: Some(alarm_status::READ_ALARM),
            hhsv: 1,
            hihi: 1e-6,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsta, alarm_status::READ_ALARM);
        assert_eq!(c.nsev, AlarmSeverity::Invalid);
    }

    #[test]
    fn an_undefined_record_skips_the_limit_check() {
        let mut r = VsRecord {
            hhsv: 1,
            hihi: 1e-6,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        c.udf = 1;
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
        assert_eq!(r.lalm, 0.0);
    }

    #[test]
    fn precision_and_limits_follow_the_field() {
        let r = VsRecord::default();
        assert_eq!(r.field_metadata_override("VAL").unwrap().precision, Some(1));
        assert_eq!(
            r.field_metadata_override("LPRS").unwrap().precision,
            Some(2)
        );
        assert_eq!(r.field_metadata_override("ERR").unwrap().precision, Some(0));
        assert_eq!(
            r.field_metadata_override("CGAP").unwrap().disp_limits,
            Some((1000.0, 1e-4f32 as f64))
        );
        assert_eq!(
            r.field_metadata_override("CGAP").unwrap().ctrl_limits,
            Some((1e-4f32 as f64, 1e-12f32 as f64))
        );
    }

    #[test]
    fn alarm_limits_are_published_only_while_an_ion_gauge_is_on() {
        let mut r = VsRecord::default();
        assert_eq!(
            r.field_metadata_override("VAL").unwrap().alarm_limits,
            Some((0.0, 0.0, 0.0, 0.0))
        );
        r.ig2s = ON;
        assert_eq!(
            r.field_metadata_override("VAL").unwrap().alarm_limits,
            Some((
                1e-6f32 as f64,
                1e-7f32 as f64,
                2e-12f32 as f64,
                1e-12f32 as f64
            ))
        );
        let (a, b, c, d) = r
            .field_metadata_override("CGAP")
            .unwrap()
            .alarm_limits
            .unwrap();
        assert!([a, b, c, d].iter().all(|v| v.is_nan()));
    }
}
