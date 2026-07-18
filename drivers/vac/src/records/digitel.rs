//! The `digitel` ion-pump record (`digitelRecord.c` / `digitelRecord.dbd`).

use std::any::Any;

use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::recgbl::alarm_status;
use epics_rs::base::server::record::{
    AlarmSeverity, CommonFields, FieldDesc, FieldMetadataOverride, ProcessOutcome, Record,
};
use epics_rs::base::types::{DbFieldType, EpicsValue, PvString};

use super::{record_fields, set_sevr, severity_of};

/// `SPC_MOD` clamp limits (`digitelRecord.c:96-99`).
const DIGITEL_MAXSP: f64 = 1e-3;
const DIGITEL_MINSP: f64 = 0.0;
const DIGITEL_MAXHY: f64 = 1e-3;
const DIGITEL_MINHY: f64 = 0.0;

/// `flgs` bits — `choiceDigitel.h`.
const MOD_DSPL: u32 = 0x0001;
const MOD_KLCK: u32 = 0x0002;
const MOD_MODS: u32 = 0x0004;
const MOD_BAKE: u32 = 0x0008;
const MOD_SETP: u32 = 0x0010;

/// `spfg` bits, one per setpoint group (`SP1S`..`SP4S`, etc.).
const MOD_SPNS: [u32; 4] = [0x0001, 0x0010, 0x0100, 0x4000];
const MOD_SNHS: [u32; 4] = [0x0002, 0x0020, 0x0200, 0x8000];
const MOD_SNMS: [u32; 4] = [0x0004, 0x0040, 0x0400, 0x1_0000];
const MOD_SNVS: [u32; 4] = [0x0008, 0x0080, 0x0800, 0x2_0000];
const MOD_S3BS: u32 = 0x1000;
const MOD_S3TS: u32 = 0x2000;

/// `menuYesNoYES`.
const YES: u16 = 1;

// --- menus (digitelRecord.dbd) -------------------------------------------

const DSPL_CHOICES: &[&str] = &["VOLTS", "CURR", "PRES"];
const KLCK_CHOICES: &[&str] = &["Unlocked", "Locked"];
const MODS_CHOICES: &[&str] = &["STBY", "OPER"];
const MODR_CHOICES: &[&str] = &["STBY", "OPER", "CONN", "COOL", "PERR", "LOCK"];
const SET1_CHOICES: &[&str] = &["Off", "On"];
const S1MS_CHOICES: &[&str] = &["Pressure", "Current"];
const S1VS_CHOICES: &[&str] = &["Off", "On"];
const BAKS_CHOICES: &[&str] = &["Disabled", "Enabled"];
const S3BS_CHOICES: &[&str] = &["Real Time", "Heat On Time"];
const PTYP_CHOICES: &[&str] = &[
    "30  Liter/sec",
    "60  Liter/sec",
    "120 Liter/sec",
    "220 Liter/sec",
    "400 Liter/sec",
    "700 Liter/sec",
    "1200 Liter/sec",
];
const CMOR_CHOICES: &[&str] = &["Off", "On"];
const BKIN_CHOICES: &[&str] = &["Absent", "Installed"];
const TYPE_CHOICES: &[&str] = &["MPC", "D500", "D1500", "QPC"];
const ALARM_SEVR: &[&str] = &["NO_ALARM", "MINOR", "MAJOR", "INVALID"];
const YESNO_CHOICES: &[&str] = &["NO", "YES"];

#[derive(Debug, Clone)]
pub struct DigitelRecord {
    /// Controller model / firmware version (MPC/QPC only report them).
    pub modl: PvString,
    pub vers: PvString,
    pub tipe: u16,

    pub val: f64,
    pub lval: f64,

    pub hihi: f64,
    pub lolo: f64,
    pub high: f64,
    pub low: f64,
    pub hhsv: u16,
    pub llsv: u16,
    pub hsv: u16,
    pub lsv: u16,
    pub hyst: f64,
    pub lalm: f64,

    pub dspl: u16,
    pub klck: u16,
    pub mods: u16,
    pub modr: u16,
    pub baks: u16,
    pub bakr: u16,
    pub cool: f64,
    pub cmor: u16,
    pub set: [u16; 4],
    pub accw: f64,
    pub acci: f64,
    pub ptyp: u16,

    // Setpoint groups 1-4.
    pub sps: [f64; 4],
    pub spr: [f64; 4],
    pub shs: [f64; 4],
    pub shr: [f64; 4],
    pub sms: [u16; 4],
    pub smr: [u16; 4],
    pub svs: [u16; 4],
    pub svr: [u16; 4],
    // Group-3 bakeout extras.
    pub s3bs: u16,
    pub s3br: u16,
    pub s3ts: f64,
    pub s3tr: f64,

    pub hopr: f32,
    pub lopr: f32,
    pub hctr: f32,
    pub lctr: f32,
    pub hvtr: f32,
    pub lvtr: f32,
    pub hlpr: f32,
    pub llpr: f32,

    // Simulation links and their fetched values.
    pub siml: PvString,
    pub simm: u16,
    pub slmo: PvString,
    pub svmo: u16,
    pub sls1: PvString,
    pub svs1: u16,
    pub sls2: PvString,
    pub svs2: u16,
    pub slcr: PvString,
    pub svcr: f64,

    pub tonl: u32,
    pub crnt: f64,
    pub volt: f64,
    pub flgs: u32,
    pub spfg: u32,
    pub bkin: u16,

    // `monitor()`'s previous-value shadows.
    pub ival: f64,
    pub ilva: f64,
    pub imod: u16,
    pub ibak: u16,
    pub icol: f64,
    pub isp: [u16; 4],
    pub iacw: f64,
    pub iaci: f64,
    pub ipty: u16,
    pub ibkn: u16,
    pub is_: [f64; 4],
    pub ih: [f64; 4],
    pub im: [u16; 4],
    pub ii: [u16; 4],
    pub ib3: u16,
    pub it3: f64,
    pub iton: u32,
    pub icrn: f64,
    pub ivol: f64,
    pub cycl: i32,
    pub err: i16,
    pub ierr: i16,

    /// Set once device support (or the simulation branch) has produced a value;
    /// C clears `pr->udf` on every such path.
    pub dev_ran: bool,
    /// `recGblSetSevr(pr, READ_ALARM, INVALID_ALARM)` raised by device support.
    pub read_alarm: bool,
}

impl Default for DigitelRecord {
    fn default() -> Self {
        Self {
            modl: PvString::new(),
            vers: PvString::new(),
            tipe: 0,
            val: 0.0,
            lval: 0.0,
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
            dspl: 2,
            klck: 0,
            mods: 0,
            modr: 0,
            baks: 0,
            bakr: 0,
            cool: 0.0,
            cmor: 0,
            set: [0; 4],
            accw: 0.0,
            acci: 0.0,
            ptyp: 0,
            sps: [0.0; 4],
            spr: [0.0; 4],
            shs: [0.0; 4],
            shr: [0.0; 4],
            sms: [0; 4],
            smr: [0; 4],
            svs: [0; 4],
            svr: [0; 4],
            s3bs: 0,
            s3br: 0,
            s3ts: 0.0,
            s3tr: 0.0,
            hopr: 1e-4,
            lopr: 1e-11,
            hctr: 0.5,
            lctr: 1e-9,
            hvtr: 7000.0,
            lvtr: 0.0,
            hlpr: -4.0,
            llpr: -11.0,
            siml: PvString::new(),
            simm: 0,
            slmo: PvString::new(),
            svmo: 0,
            sls1: PvString::new(),
            svs1: 0,
            sls2: PvString::new(),
            svs2: 0,
            slcr: PvString::new(),
            svcr: 0.0,
            tonl: 0,
            crnt: 0.0,
            volt: 0.0,
            flgs: 0,
            spfg: 0,
            bkin: 0,
            ival: 0.0,
            ilva: 0.0,
            imod: 0,
            ibak: 0,
            icol: 0.0,
            isp: [0; 4],
            iacw: 0.0,
            iaci: 0.0,
            ipty: 0,
            ibkn: 0,
            is_: [0.0; 4],
            ih: [0.0; 4],
            im: [0; 4],
            ii: [0; 4],
            ib3: 0,
            it3: 0.0,
            iton: 0,
            icrn: 0.0,
            ivol: 0.0,
            cycl: 0,
            err: 0,
            ierr: 0,
            dev_ran: false,
            read_alarm: false,
        }
    }
}

record_fields! {
    SCALAR_FIELDS, get_scalar, put_scalar, DigitelRecord;
    "TYPE": Enum = tipe, false;
    "VAL": Double = val, true;
    "LVAL": Double = lval, true;
    "HIHI": Double = hihi, false;
    "LOLO": Double = lolo, false;
    "HIGH": Double = high, false;
    "LOW": Double = low, false;
    "HHSV": Enum = hhsv, false;
    "LLSV": Enum = llsv, false;
    "HSV": Enum = hsv, false;
    "LSV": Enum = lsv, false;
    "HYST": Double = hyst, false;
    "LALM": Double = lalm, true;
    "DSPL": Enum = dspl, false;
    "KLCK": Enum = klck, false;
    "MODS": Enum = mods, false;
    "MODR": Enum = modr, true;
    "BAKS": Enum = baks, false;
    "BAKR": Enum = bakr, true;
    "COOL": Double = cool, true;
    "CMOR": Enum = cmor, true;
    "ACCW": Double = accw, true;
    "ACCI": Double = acci, true;
    "PTYP": Enum = ptyp, true;
    "S3BS": Enum = s3bs, false;
    "S3BR": Enum = s3br, true;
    "S3TS": Double = s3ts, false;
    "S3TR": Double = s3tr, true;
    "HOPR": Float = hopr, false;
    "LOPR": Float = lopr, false;
    "HCTR": Float = hctr, false;
    "LCTR": Float = lctr, false;
    "HVTR": Float = hvtr, false;
    "LVTR": Float = lvtr, false;
    "HLPR": Float = hlpr, false;
    "LLPR": Float = llpr, false;
    "SIMM": Enum = simm, false;
    "SVMO": Enum = svmo, false;
    "SVS1": Enum = svs1, false;
    "SVS2": Enum = svs2, false;
    "SVCR": Double = svcr, false;
    "TONL": ULong = tonl, true;
    "CRNT": Double = crnt, true;
    "VOLT": Double = volt, true;
    "FLGS": ULong = flgs, true;
    "SPFG": ULong = spfg, true;
    "BKIN": Enum = bkin, true;
    "IVAL": Double = ival, true;
    "ILVA": Double = ilva, true;
    "IMOD": Enum = imod, true;
    "IBAK": Enum = ibak, true;
    "ICOL": Double = icol, true;
    "IACW": Double = iacw, true;
    "IACI": Double = iaci, true;
    "IPTY": Enum = ipty, true;
    "IBKN": Enum = ibkn, true;
    "IB3": Enum = ib3, true;
    "IT3": Double = it3, true;
    "ITON": ULong = iton, true;
    "ICRN": Double = icrn, true;
    "IVOL": Double = ivol, true;
    "CYCL": Long = cycl, false;
    "ERR": Short = err, true;
    "IERR": Short = ierr, true;
}

/// Indexed setpoint-group fields. Tag identifies the array; the returned index
/// is zero-based.
fn indexed(name: &str) -> Option<(&'static str, usize)> {
    let (tag, digit) = match name.as_bytes() {
        [b'S', b'P', d, b'S'] => ("SPnS", d),
        [b'S', b'P', d, b'R'] => ("SPnR", d),
        [b'S', d, b'H', b'S'] => ("SnHS", d),
        [b'S', d, b'H', b'R'] => ("SnHR", d),
        [b'S', d, b'M', b'S'] => ("SnMS", d),
        [b'S', d, b'M', b'R'] => ("SnMR", d),
        [b'S', d, b'V', b'S'] => ("SnVS", d),
        [b'S', d, b'V', b'R'] => ("SnVR", d),
        [b'S', b'E', b'T', d] => ("SETn", d),
        [b'I', b'S', b'P', d] => ("ISPn", d),
        [b'I', b'S', d] => ("ISn", d),
        [b'I', b'H', d] => ("IHn", d),
        [b'I', b'M', d] => ("IMn", d),
        [b'I', b'I', d] => ("IIn", d),
        _ => return None,
    };
    let n = (*digit as char).to_digit(10)? as usize;
    (1..=4).contains(&n).then_some((tag, n - 1))
}

/// `(name, dbf_type, read_only)` for the indexed fields, in a fixed order so
/// `field_list` is stable.
static INDEXED_FIELDS: &[FieldDesc] = &{
    const fn f(name: &'static str, dbf_type: DbFieldType, ro: bool) -> FieldDesc {
        FieldDesc::new(name, dbf_type, ro)
    }
    use DbFieldType::{Double, Enum};
    [
        f("SP1S", Double, false),
        f("SP2S", Double, false),
        f("SP3S", Double, false),
        f("SP4S", Double, false),
        f("SP1R", Double, true),
        f("SP2R", Double, true),
        f("SP3R", Double, true),
        f("SP4R", Double, true),
        f("S1HS", Double, false),
        f("S2HS", Double, false),
        f("S3HS", Double, false),
        f("S4HS", Double, false),
        f("S1HR", Double, true),
        f("S2HR", Double, true),
        f("S3HR", Double, true),
        f("S4HR", Double, true),
        f("S1MS", Enum, false),
        f("S2MS", Enum, false),
        f("S3MS", Enum, false),
        f("S4MS", Enum, false),
        f("S1MR", Enum, true),
        f("S2MR", Enum, true),
        f("S3MR", Enum, true),
        f("S4MR", Enum, true),
        f("S1VS", Enum, false),
        f("S2VS", Enum, false),
        f("S3VS", Enum, false),
        f("S4VS", Enum, false),
        f("S1VR", Enum, true),
        f("S2VR", Enum, true),
        f("S3VR", Enum, true),
        f("S4VR", Enum, true),
        f("SET1", Enum, true),
        f("SET2", Enum, true),
        f("SET3", Enum, true),
        f("SET4", Enum, true),
        f("ISP1", Enum, true),
        f("ISP2", Enum, true),
        f("ISP3", Enum, true),
        f("ISP4", Enum, true),
        f("IS1", Double, true),
        f("IS2", Double, true),
        f("IS3", Double, true),
        f("IS4", Double, true),
        f("IH1", Double, true),
        f("IH2", Double, true),
        f("IH3", Double, true),
        f("IH4", Double, true),
        f("IM1", Enum, true),
        f("IM2", Enum, true),
        f("IM3", Enum, true),
        f("IM4", Enum, true),
        f("II1", Enum, true),
        f("II2", Enum, true),
        f("II3", Enum, true),
        f("II4", Enum, true),
    ]
};

/// DBF_STRING readbacks and the simulation link fields.
static STRING_FIELDS: &[FieldDesc] = &{
    const fn f(name: &'static str, ro: bool) -> FieldDesc {
        FieldDesc::new(name, DbFieldType::String, ro)
    }
    [
        f("MODL", true),
        f("VERS", true),
        f("SIML", true),
        f("SLMO", true),
        f("SLS1", true),
        f("SLS2", true),
        f("SLCR", true),
    ]
};

static ALL_FIELDS: std::sync::LazyLock<Vec<FieldDesc>> = std::sync::LazyLock::new(|| {
    SCALAR_FIELDS
        .iter()
        .chain(INDEXED_FIELDS.iter())
        .chain(STRING_FIELDS.iter())
        .cloned()
        .collect()
});

impl DigitelRecord {
    fn get_indexed(&self, name: &str) -> Option<EpicsValue> {
        let (tag, i) = indexed(name)?;
        Some(match tag {
            "SPnS" => EpicsValue::Double(self.sps[i]),
            "SPnR" => EpicsValue::Double(self.spr[i]),
            "SnHS" => EpicsValue::Double(self.shs[i]),
            "SnHR" => EpicsValue::Double(self.shr[i]),
            "SnMS" => EpicsValue::Enum(self.sms[i]),
            "SnMR" => EpicsValue::Enum(self.smr[i]),
            "SnVS" => EpicsValue::Enum(self.svs[i]),
            "SnVR" => EpicsValue::Enum(self.svr[i]),
            "SETn" => EpicsValue::Enum(self.set[i]),
            "ISPn" => EpicsValue::Enum(self.isp[i]),
            "ISn" => EpicsValue::Double(self.is_[i]),
            "IHn" => EpicsValue::Double(self.ih[i]),
            "IMn" => EpicsValue::Enum(self.im[i]),
            _ => EpicsValue::Enum(self.ii[i]),
        })
    }

    fn put_indexed(&mut self, name: &str, value: EpicsValue) -> Option<CaResult<()>> {
        let (tag, i) = indexed(name)?;
        Some(match (tag, value) {
            ("SPnS", EpicsValue::Double(v)) => {
                self.sps[i] = v;
                Ok(())
            }
            ("SPnR", EpicsValue::Double(v)) => {
                self.spr[i] = v;
                Ok(())
            }
            ("SnHS", EpicsValue::Double(v)) => {
                self.shs[i] = v;
                Ok(())
            }
            ("SnHR", EpicsValue::Double(v)) => {
                self.shr[i] = v;
                Ok(())
            }
            ("SnMS", EpicsValue::Enum(v)) => {
                self.sms[i] = v;
                Ok(())
            }
            ("SnMR", EpicsValue::Enum(v)) => {
                self.smr[i] = v;
                Ok(())
            }
            ("SnVS", EpicsValue::Enum(v)) => {
                self.svs[i] = v;
                Ok(())
            }
            ("SnVR", EpicsValue::Enum(v)) => {
                self.svr[i] = v;
                Ok(())
            }
            ("SETn", EpicsValue::Enum(v)) => {
                self.set[i] = v;
                Ok(())
            }
            ("ISPn", EpicsValue::Enum(v)) => {
                self.isp[i] = v;
                Ok(())
            }
            ("ISn", EpicsValue::Double(v)) => {
                self.is_[i] = v;
                Ok(())
            }
            ("IHn", EpicsValue::Double(v)) => {
                self.ih[i] = v;
                Ok(())
            }
            ("IMn", EpicsValue::Enum(v)) => {
                self.im[i] = v;
                Ok(())
            }
            ("IIn", EpicsValue::Enum(v)) => {
                self.ii[i] = v;
                Ok(())
            }
            _ => Err(CaError::TypeMismatch(name.into())),
        })
    }

    fn get_string(&self, name: &str) -> Option<EpicsValue> {
        let s = match name {
            "MODL" => &self.modl,
            "VERS" => &self.vers,
            "SIML" => &self.siml,
            "SLMO" => &self.slmo,
            "SLS1" => &self.sls1,
            "SLS2" => &self.sls2,
            "SLCR" => &self.slcr,
            _ => return None,
        };
        Some(EpicsValue::String(s.clone()))
    }

    fn put_string(&mut self, name: &str, value: EpicsValue) -> Option<CaResult<()>> {
        let slot = match name {
            "MODL" => &mut self.modl,
            "VERS" => &mut self.vers,
            "SIML" => &mut self.siml,
            "SLMO" => &mut self.slmo,
            "SLS1" => &mut self.sls1,
            "SLS2" => &mut self.sls2,
            "SLCR" => &mut self.slcr,
            _ => return None,
        };
        Some(match value {
            EpicsValue::String(s) => {
                *slot = s;
                Ok(())
            }
            _ => Err(CaError::TypeMismatch(name.into())),
        })
    }

    /// C `digitelRecord.c::checkAlarms`, minus the leading `udf` branch, which
    /// the framework's `rec_gbl_check_udf` owns.
    ///
    /// C narrows `hihi/lolo/high/low/hyst/lalm` to `float` but keeps `val`
    /// `double`; a `float`-narrowed limit is compared against the full-precision
    /// pressure. The narrowing is reproduced so a limit that is not exactly
    /// representable in `float` trips at the same boundary as upstream.
    fn limit_alarms(&mut self, common: &mut CommonFields) {
        let val = self.val;
        let hyst = self.hyst as f32 as f64;
        let lalm = self.lalm as f32 as f64;

        for (sevr_index, stat, limit, over) in [
            (
                self.hhsv,
                alarm_status::HIHI_ALARM,
                self.hihi as f32 as f64,
                true,
            ),
            (
                self.llsv,
                alarm_status::LOLO_ALARM,
                self.lolo as f32 as f64,
                false,
            ),
            (
                self.hsv,
                alarm_status::HIGH_ALARM,
                self.high as f32 as f64,
                true,
            ),
            (
                self.lsv,
                alarm_status::LOW_ALARM,
                self.low as f32 as f64,
                false,
            ),
        ] {
            if sevr_index == 0 {
                continue;
            }
            let sevr = severity_of(sevr_index);
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
        // Out of alarm by at least HYST: the full-precision pressure latches.
        self.lalm = val;
    }
}

impl Record for DigitelRecord {
    fn record_type(&self) -> &'static str {
        "digitel"
    }

    fn get_field(&self, name: &str) -> Option<EpicsValue> {
        get_scalar(self, name)
            .or_else(|| self.get_indexed(name))
            .or_else(|| self.get_string(name))
    }

    fn put_field(&mut self, name: &str, value: EpicsValue) -> CaResult<()> {
        if let Some(r) = self.put_indexed(name, value.clone()) {
            return r;
        }
        if let Some(r) = self.put_string(name, value.clone()) {
            return r;
        }
        put_scalar(self, name, value)
    }

    fn declared_fields(&self) -> &'static [FieldDesc] {
        let fields: &Vec<FieldDesc> = &ALL_FIELDS;
        // `ALL_FIELDS` is a `LazyLock` that lives for the whole program.
        unsafe { std::slice::from_raw_parts(fields.as_ptr(), fields.len()) }
    }

    fn menu_field_choices(&self, field: &str) -> Option<&'static [&'static str]> {
        match field {
            "TYPE" => Some(TYPE_CHOICES),
            "HHSV" | "LLSV" | "HSV" | "LSV" => Some(ALARM_SEVR),
            "DSPL" => Some(DSPL_CHOICES),
            "KLCK" => Some(KLCK_CHOICES),
            "MODS" | "SVMO" | "IMOD" => Some(MODS_CHOICES),
            "MODR" => Some(MODR_CHOICES),
            "BAKS" | "BAKR" | "IBAK" => Some(BAKS_CHOICES),
            "CMOR" => Some(CMOR_CHOICES),
            "PTYP" | "IPTY" => Some(PTYP_CHOICES),
            "BKIN" | "IBKN" => Some(BKIN_CHOICES),
            "SIMM" => Some(YESNO_CHOICES),
            "SET1" | "SET2" | "SET3" | "SET4" | "ISP1" | "ISP2" | "ISP3" | "ISP4" | "SVS1"
            | "SVS2" => Some(SET1_CHOICES),
            "S3BS" | "S3BR" | "IB3" => Some(S3BS_CHOICES),
            _ => match indexed(field) {
                Some(("SnMS" | "SnMR" | "IMn", _)) => Some(S1MS_CHOICES),
                Some(("SnVS" | "SnVR" | "IIn", _)) => Some(S1VS_CHOICES),
                Some(("SETn" | "ISPn", _)) => Some(SET1_CHOICES),
                _ => None,
            },
        }
    }

    /// `pp(TRUE)` in `digitelRecord.dbd`.
    fn process_passive_fields(&self) -> &'static [&'static str] {
        &[
            "HIHI", "LOLO", "HIGH", "LOW", "HHSV", "LLSV", "HSV", "LSV", "DSPL", "KLCK", "MODS",
            "BAKS", "TYPE", "SP1S", "S1HS", "S1MS", "S1VS", "SP2S", "S2HS", "S2MS", "S2VS", "SP3S",
            "S3HS", "S3MS", "S3VS", "S3BS", "S3TS", "SP4S", "S4HS", "S4MS", "S4VS",
        ]
    }

    /// C `digitelRecord.c::special` — record which `SPC_MOD` field changed, and
    /// clamp the setpoint / hysteresis fields. `TYPE` carries `SPC_MOD` but the
    /// C `special()` has no arm for it, so it is a no-op here too.
    fn special(&mut self, field: &str, after: bool) -> CaResult<()> {
        if !after {
            return Ok(());
        }
        match field {
            "DSPL" => self.flgs |= MOD_DSPL,
            "KLCK" => self.flgs |= MOD_KLCK,
            "MODS" => self.flgs |= MOD_MODS,
            "BAKS" => self.flgs |= MOD_BAKE,
            "S3BS" => {
                self.spfg |= MOD_S3BS;
                self.flgs |= MOD_SETP;
            }
            "S3TS" => {
                self.spfg |= MOD_S3TS;
                self.flgs |= MOD_SETP;
            }
            _ => match indexed(field) {
                Some(("SPnS", i)) => {
                    self.sps[i] = self.sps[i].clamp(DIGITEL_MINSP, DIGITEL_MAXSP);
                    self.spfg |= MOD_SPNS[i];
                    self.flgs |= MOD_SETP;
                }
                Some(("SnHS", i)) => {
                    self.shs[i] = self.shs[i].clamp(DIGITEL_MINHY, DIGITEL_MAXHY);
                    self.spfg |= MOD_SNHS[i];
                    self.flgs |= MOD_SETP;
                }
                Some(("SnMS", i)) => {
                    self.spfg |= MOD_SNMS[i];
                    self.flgs |= MOD_SETP;
                }
                Some(("SnVS", i)) => {
                    self.spfg |= MOD_SNVS[i];
                    self.flgs |= MOD_SETP;
                }
                _ => {}
            },
        }
        Ok(())
    }

    /// C `digitelRecord.c::process`'s simulation branch. The device-support
    /// `read()` runs the real hardware exchange (and latches the `I*` shadows);
    /// this override supplies the values a simulated pump would report.
    ///
    /// C reads `SLMO`/`SLS1`/`SLS2`/`SLCR` into `SVMO`/`SVS1`/`SVS2`/`SVCR`
    /// through `dbGetLink` inside this branch; the framework fetches those
    /// links (see [`Self::multi_input_links`]) before `process` runs, so the
    /// values are already in place.
    fn process(&mut self) -> CaResult<ProcessOutcome> {
        if self.simm == YES {
            self.modr = self.svmo;
            self.volt = if self.modr == 1 { 6000.0 } else { 0.0 };
            self.set[0] = self.svs1;
            self.set[1] = self.svs2;
            self.crnt = self.svcr;
            if self.modr == 0 {
                self.crnt = 0.0;
            }
            self.val = 0.005 * (self.crnt / 8.0);
            self.lval = if self.val <= 0.0 {
                -10.0
            } else {
                self.val.log10()
            };
            // C `pdg->udf = 0`.
            self.dev_ran = true;
        }
        Ok(ProcessOutcome::complete())
    }

    /// C reads `SIML`→`SIMM` every cycle, then in simulation mode reads
    /// `SLMO`/`SLS1`/`SLS2`/`SLCR`. The framework fetches all five every cycle;
    /// the extra four fetches are inert outside the simulation branch, which is
    /// the only consumer of `SVMO`/`SVS1`/`SVS2`/`SVCR`.
    fn multi_input_links(&self) -> &[(&'static str, &'static str)] {
        &[
            ("SIML", "SIMM"),
            ("SLMO", "SVMO"),
            ("SLS1", "SVS1"),
            ("SLS2", "SVS2"),
            ("SLCR", "SVCR"),
        ]
    }

    /// C posts every readback with the cycle's alarm mask when the alarm
    /// transitioned, even if the value did not change (`monitor()`'s
    /// `|| alrm_chg_flg` clauses).
    fn alarm_cycle_monitored_fields(&self) -> &'static [&'static str] {
        &[
            "VAL", "LVAL", "MODR", "BAKR", "COOL", "BKIN", "SET1", "SET2", "SET3", "SET4", "ACCW",
            "ACCI", "PTYP", "SP1R", "S1HR", "S1MR", "S1VR", "SP2R", "S2HR", "S2MR", "S2VR", "SP3R",
            "S3HR", "S3MR", "S3VR", "S3BR", "S3TR", "SP4R", "S4HR", "S4MR", "S4VR", "TONL", "CRNT",
            "VOLT", "ERR",
        ]
    }

    fn check_alarms(&mut self, common: &mut CommonFields) {
        if self.read_alarm {
            set_sevr(common, alarm_status::READ_ALARM, AlarmSeverity::Invalid);
        }
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
            "VAL" | "CRNT" => 1,
            "LVAL" => 2,
            _ => 0,
        });

        let is_hys = matches!(indexed(field), Some(("SnHS" | "SnHR", _)));
        let is_sp = matches!(indexed(field), Some(("SPnS" | "SPnR", _)));

        let disp_limits = if is_hys {
            Some((DIGITEL_MAXHY, DIGITEL_MINHY))
        } else if is_sp {
            Some((DIGITEL_MAXSP, DIGITEL_MINSP))
        } else {
            match field {
                "VAL" => Some((self.hopr as f64, self.lopr as f64)),
                "CRNT" => Some((self.hctr as f64, self.lctr as f64)),
                "LVAL" => Some((self.hlpr as f64, self.llpr as f64)),
                "VOLT" => Some((self.hvtr as f64, self.lvtr as f64)),
                _ => None,
            }
        };

        let ctrl_limits = if matches!(indexed(field), Some(("SnHS", _))) {
            Some((DIGITEL_MAXHY, DIGITEL_MINHY))
        } else if matches!(indexed(field), Some(("SPnS", _))) {
            Some((DIGITEL_MAXSP, DIGITEL_MINSP))
        } else {
            None
        };

        let alarm_limits = match field {
            "VAL" => Some((self.hihi, self.high, self.low, self.lolo)),
            _ => None,
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

    fn common() -> CommonFields {
        CommonFields {
            udf: 0,
            ..CommonFields::default()
        }
    }

    #[test]
    fn every_declared_field_is_readable() {
        let r = DigitelRecord::default();
        for f in r.declared_fields() {
            assert!(r.get_field(f.name).is_some(), "{} unreadable", f.name);
        }
    }

    #[test]
    fn indexed_names_map_to_the_right_slot() {
        let mut r = DigitelRecord::default();
        r.put_field("SP2S", EpicsValue::Double(3e-4)).unwrap();
        assert_eq!(r.sps[1], 3e-4);
        r.put_field("S4VS", EpicsValue::Enum(1)).unwrap();
        assert_eq!(r.svs[3], 1);
        r.set[2] = 1;
        assert_eq!(r.get_field("SET3"), Some(EpicsValue::Enum(1)));
        r.is_[0] = 1.5;
        assert_eq!(r.get_field("IS1"), Some(EpicsValue::Double(1.5)));
        // `ISP1` must not be read as `IS` with a `P1` index.
        r.isp[0] = 1;
        assert_eq!(r.get_field("ISP1"), Some(EpicsValue::Enum(1)));
        assert_eq!(r.get_field("SP5S"), None);
    }

    #[test]
    fn group3_bakeout_fields_are_scalars_not_indexed() {
        assert_eq!(indexed("S3BS"), None);
        assert_eq!(indexed("S3TS"), None);
        assert_eq!(indexed("S3HS"), Some(("SnHS", 2)));
    }

    #[test]
    fn special_sets_the_flag_bits() {
        let mut r = DigitelRecord::default();
        r.special("DSPL", true).unwrap();
        assert_eq!(r.flgs, MOD_DSPL);

        let mut r = DigitelRecord::default();
        r.special("S2MS", true).unwrap();
        assert_eq!(r.spfg, MOD_SNMS[1]);
        assert_eq!(r.flgs, MOD_SETP);

        // The "before" callback latches nothing.
        let mut r = DigitelRecord::default();
        r.special("DSPL", false).unwrap();
        assert_eq!(r.flgs, 0);
    }

    #[test]
    fn special_clamps_setpoints_to_the_1e_minus_3_ceiling() {
        let mut r = DigitelRecord::default();
        r.sps[0] = 1.0; // way above the ceiling
        r.special("SP1S", true).unwrap();
        assert_eq!(r.sps[0], DIGITEL_MAXSP);
        assert_eq!(r.spfg, MOD_SPNS[0]);
        assert_eq!(r.flgs, MOD_SETP);

        r.shs[2] = -1.0; // below the floor
        r.special("S3HS", true).unwrap();
        assert_eq!(r.shs[2], DIGITEL_MINHY);
        assert!(r.spfg & MOD_SNHS[2] != 0);
    }

    #[test]
    fn type_special_is_a_no_op() {
        let mut r = DigitelRecord::default();
        r.special("TYPE", true).unwrap();
        assert_eq!(r.flgs, 0);
        assert_eq!(r.spfg, 0);
    }

    #[test]
    fn simulation_math_drives_val_from_the_current_link() {
        let mut r = DigitelRecord {
            simm: YES,
            svmo: 1, // OPER
            svcr: 8.0,
            ..Default::default()
        };
        r.process().unwrap();
        assert_eq!(r.modr, 1);
        assert_eq!(r.volt, 6000.0);
        assert_eq!(r.crnt, 8.0);
        assert_eq!(r.val, 0.005 * (8.0 / 8.0));
        assert_eq!(r.lval, r.val.log10());
        assert!(r.dev_ran);
    }

    #[test]
    fn simulation_standby_zeroes_current_and_floors_lval() {
        let mut r = DigitelRecord {
            simm: YES,
            svmo: 0, // STBY
            svcr: 8.0,
            ..Default::default()
        };
        r.process().unwrap();
        assert_eq!(r.modr, 0);
        assert_eq!(r.volt, 0.0);
        assert_eq!(r.crnt, 0.0);
        assert_eq!(r.val, 0.0);
        assert_eq!(r.lval, -10.0);
    }

    #[test]
    fn process_is_inert_outside_simulation_mode() {
        let mut r = DigitelRecord {
            simm: 0,
            val: 1e-9,
            ..Default::default()
        };
        r.process().unwrap();
        assert_eq!(r.val, 1e-9);
        assert!(!r.dev_ran);
    }

    #[test]
    fn alarms_check_val_against_narrowed_limits() {
        let mut r = DigitelRecord {
            dev_ran: true,
            hhsv: 2, // MAJOR
            hihi: 1e-6,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsta, alarm_status::HIHI_ALARM);
        assert_eq!(c.nsev, AlarmSeverity::Major);
        assert_eq!(r.lalm, 1e-6f32 as f64);
    }

    #[test]
    fn no_alarm_latches_the_full_precision_value() {
        let mut r = DigitelRecord {
            dev_ran: true,
            hhsv: 0,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
        assert_eq!(r.lalm, 1e-5);
    }

    #[test]
    fn a_read_alarm_is_invalid_and_survives_the_limit_chain() {
        let mut r = DigitelRecord {
            dev_ran: true,
            read_alarm: true,
            hhsv: 1, // MINOR limit cannot outrank INVALID
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
        let mut r = DigitelRecord {
            hhsv: 1,
            hihi: 1e-6,
            val: 1e-5,
            ..Default::default()
        };
        let mut c = common();
        c.udf = 1;
        r.check_alarms(&mut c);
        assert_eq!(c.nsev, AlarmSeverity::NoAlarm);
    }

    #[test]
    fn metadata_precision_and_limits_follow_the_field() {
        let r = DigitelRecord::default();
        assert_eq!(r.field_metadata_override("VAL").unwrap().precision, Some(1));
        assert_eq!(
            r.field_metadata_override("LVAL").unwrap().precision,
            Some(2)
        );
        assert_eq!(
            r.field_metadata_override("CRNT").unwrap().precision,
            Some(1)
        );
        assert_eq!(
            r.field_metadata_override("MODR").unwrap().precision,
            Some(0)
        );
        assert_eq!(
            r.field_metadata_override("SP1S").unwrap().ctrl_limits,
            Some((DIGITEL_MAXSP, DIGITEL_MINSP))
        );
        assert_eq!(
            r.field_metadata_override("S2HS").unwrap().disp_limits,
            Some((DIGITEL_MAXHY, DIGITEL_MINHY))
        );
        assert_eq!(
            r.field_metadata_override("VAL").unwrap().alarm_limits,
            Some((1e-6, 1e-7, 2e-12, 1e-12))
        );
        assert_eq!(
            r.field_metadata_override("CRNT").unwrap().alarm_limits,
            None
        );
    }

    #[test]
    fn string_fields_round_trip() {
        let mut r = DigitelRecord::default();
        r.put_field(
            "MODL",
            EpicsValue::String(PvString::from_bytes(&b"MPC2"[..])),
        )
        .unwrap();
        assert_eq!(
            r.get_field("MODL"),
            Some(EpicsValue::String(PvString::from_bytes(&b"MPC2"[..])))
        );
        r.put_field(
            "SIML",
            EpicsValue::String(PvString::from_bytes(&b"sim.VAL"[..])),
        )
        .unwrap();
        assert_eq!(r.siml.as_bytes(), b"sim.VAL");
    }
}
