//! `USBCTR_SNL.st`: reads the MCS spectra and the absolute time base when a
//! client asks (`{P}DoReadAll`) and when the hardware stops acquiring, then
//! releases the `{P}Acquiring` busy record `measCompMCS.template` sets.
//!
//! [`Flags::monitor_changes`] is the state `monitor_changes` as a function
//! of its two event flags; [`run`] binds it to the template's records.

use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::database::db_access::DbMultiMonitor;
use epics_rs::base::server::snl::{PutQueue, SeqMacros, SeqPv};
use epics_rs::base::types::EpicsValue;
use epics_rs::base::types::c_parse::atoi;

/// `MAX_COUNTERS` (`USBCTR_SNL.st:22`).
const MAX_COUNTERS: i32 = 9;

/// `program USBCTR_SNL(...)`'s macros, which those given to `seq` override.
const PROGRAM_MACROS: &str = "P=USBCTR:MCS:, R=mca, NUM_COUNTERS=8,  FIELD=READ";

/// The macro table `seq` builds for this program from `macro_def`
/// (seq_main.c:93-96).
pub fn seq_macros(macro_def: &str) -> SeqMacros {
    SeqMacros::new(PROGRAM_MACROS, macro_def)
}

/// The macros `init` reads (`USBCTR_SNL.st:52-55`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbCtrSnlConfig {
    pub prefix: String,
    pub record: String,
    /// `atoi(NUM_COUNTERS)`, before `init` checks its range.
    pub num_counters: i32,
    pub field: String,
}

impl UsbCtrSnlConfig {
    /// What `init` reads from `seq`'s macro table ([`seq_macros`]).
    pub fn from_macros(macros: &SeqMacros) -> Self {
        let get = |name: &str| macros.value(name).unwrap_or_default().to_owned();
        Self {
            prefix: get("P"),
            record: get("R"),
            num_counters: atoi(get("NUM_COUNTERS")),
            field: get("FIELD"),
        }
    }
}

/// The `when` clause of `monitor_changes` that fired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Clause {
    /// `efTestAndClear(ReadArraysMon) && (ReadArrays == 1)`.
    ReadArrays,
    /// `efTestAndClear(HardwareAcquiringMon)`.
    HardwareAcquiring,
}

/// `ReadArraysMon` and `HardwareAcquiringMon`, each raised by every monitor
/// of the variable synced to it, the connect-time one included
/// (seq_ca.c:328-329).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    pub read_arrays: bool,
    pub hardware_acquiring: bool,
}

impl Flags {
    /// Both channels' connect-time monitors have arrived.
    pub fn connected() -> Self {
        Self {
            read_arrays: true,
            hardware_acquiring: true,
        }
    }

    /// One evaluation of `monitor_changes` (`USBCTR_SNL.st:85-117`); the
    /// connection clause cannot fire on the IOC's own records. The first
    /// clause clears its flag even when `ReadArrays` is not 1.
    pub fn monitor_changes(&mut self, read_arrays: i32) -> Option<Clause> {
        if std::mem::take(&mut self.read_arrays) && read_arrays == 1 {
            return Some(Clause::ReadArrays);
        }
        if std::mem::take(&mut self.hardware_acquiring) {
            return Some(Clause::HardwareAcquiring);
        }
        None
    }
}

fn long(v: i32) -> EpicsValue {
    EpicsValue::Long(v)
}

/// Run `USBCTR_SNL.st` against the records `measCompMCS.template` and the
/// spectrum records loaded.
///
/// One deviation, loud rather than silent: a PV the database does not hold
/// leaves C in `waitConnected` forever; here it is an error return naming
/// the missing PVs.
pub async fn run(
    config: UsbCtrSnlConfig,
    db: PvDatabase,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // `init`.
    let mut num_counters = config.num_counters;
    if num_counters <= 0 || num_counters > MAX_COUNTERS {
        println!("NUM_COUNTERS is illegal.");
        num_counters = 0;
    }
    // `waitConnected`.
    if num_counters <= 0 {
        println!(">>>>>>>>>>>>>> USBCTR is dead. <<<<<<<<<<<<<<");
        return Ok(());
    }

    let p = &config.prefix;
    let pv_read_arrays = format!("{p}DoReadAll");
    let pv_hardware_acquiring = format!("{p}HardwareAcquiring");
    let pv_asyn_debug = format!("{p}Asyn.TB1");
    let queue = PutQueue::new(&db, "mca_control");
    let read_array: Vec<SeqPv> = (1..=num_counters)
        .map(|n| {
            let name = format!("{p}{}{n}.{}", config.record, config.field);
            queue.pv(&format!("ReadArray[{}]", n - 1), &name)
        })
        .collect();
    let read_arrays = queue.pv("ReadArrays", &pv_read_arrays);
    let abs_time_wf = queue.pv("MCSAbsTimeWF", &format!("{p}AbsTimeWF.PROC"));
    let snl_connected = queue.pv("SNL_Connected", &format!("{p}SNL_Connected"));
    let acquiring = queue.pv("Acquiring", &format!("{p}Acquiring"));

    let mut monitor = DbMultiMonitor::new(
        &db,
        &[pv_read_arrays.clone(), pv_hardware_acquiring.clone()],
    )
    .await;
    monitor
        .add_vars(&db, std::slice::from_ref(&pv_asyn_debug))
        .await;
    let assigned = read_array
        .iter()
        .chain([&read_arrays, &abs_time_wf, &snl_connected, &acquiring])
        .map(SeqPv::name)
        .chain([pv_hardware_acquiring.as_str(), pv_asyn_debug.as_str()]);
    let missing: Vec<&str> = assigned.filter(|pv| db.get_pv(pv).is_err()).collect();
    if !missing.is_empty() {
        return Err(format!("USBCTR_SNL: not in the database: {}", missing.join(", ")).into());
    }
    println!("USBCTR: All channels connected.");
    let _ = snl_connected.put(long(1));

    // `monitor_changes`, with each variable as its connect-time monitor left it.
    let as_i32 = |v: Option<EpicsValue>| v.and_then(|v| v.to_f64()).unwrap_or(0.0) as i32;
    let mut vars = Synced {
        read_arrays_pv: pv_read_arrays.clone(),
        read_arrays: as_i32(db.get_pv(&pv_read_arrays).ok()),
        hardware_acquiring: as_i32(db.get_pv(&pv_hardware_acquiring).ok()),
        flags: Flags::connected(),
    };
    loop {
        // `pvSysFlush` at state entry (seq_task.c:303-304), then the monitors
        // that arrived before this evaluation.
        queue.flush().await;
        for event in monitor.try_changes() {
            vars.take(event);
        }
        let debug = monitor.var(&pv_asyn_debug) != 0.0;
        match vars.flags.monitor_changes(vars.read_arrays) {
            Some(Clause::ReadArrays) => {
                if debug {
                    println!("USBCTR.st: Read array data");
                }
                for ch in &read_array {
                    let _ = ch.put(long(1));
                }
                let _ = abs_time_wf.put(long(1));
                vars.read_arrays = 0;
                let _ = read_arrays.put(long(0));
            }
            Some(Clause::HardwareAcquiring) => {
                if debug {
                    println!(
                        "USBCTR.st: HardwareAcquiringMon, HardwareAcquiring={}",
                        vars.hardware_acquiring
                    );
                }
                if vars.hardware_acquiring == 0 {
                    for ch in &read_array {
                        let _ = ch.put_sync(long(1)).await;
                    }
                    let _ = abs_time_wf.put(long(1));
                    let _ = acquiring.put(long(0));
                }
            }
            None => {
                for event in monitor.wait_changes().await {
                    vars.take(event);
                }
            }
        }
    }
}

/// `ReadArrays` and `HardwareAcquiring`: each monitor writes the variable
/// and raises its flag.
struct Synced {
    read_arrays_pv: String,
    read_arrays: i32,
    hardware_acquiring: i32,
    flags: Flags,
}

impl Synced {
    fn take(&mut self, (pv, v): (String, f64)) {
        if pv == self.read_arrays_pv {
            self.read_arrays = v as i32;
            self.flags.read_arrays = true;
        } else {
            self.hardware_acquiring = v as i32;
            self.flags.hardware_acquiring = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seq_macros_override_the_programs() {
        let config = UsbCtrSnlConfig::from_macros(&seq_macros(
            "P=USBCTR:MCS:, R=mca, NUM_COUNTERS=9, FIELD=PROC",
        ));
        assert_eq!(
            config,
            UsbCtrSnlConfig {
                prefix: "USBCTR:MCS:".into(),
                record: "mca".into(),
                num_counters: 9,
                field: "PROC".into(),
            }
        );
        let defaults = UsbCtrSnlConfig::from_macros(&seq_macros("R"));
        assert_eq!(defaults.prefix, "USBCTR:MCS:");
        assert_eq!(defaults.record, "mca");
        assert_eq!(defaults.num_counters, 8);
        assert_eq!(defaults.field, "READ");
    }

    #[test]
    fn num_counters_is_glibc_atoi() {
        assert_eq!(atoi(" +3x"), 3);
        assert_eq!(atoi("-2"), -2);
        use std::ffi::c_long;
        // strtol saturates at LONG_MIN/LONG_MAX, then truncates to int.
        assert_eq!(atoi("-99999999999999999999"), c_long::MIN as i32);
        assert_eq!(atoi("99999999999999999999"), c_long::MAX as i32);
        // 2^32 + 3 truncates to 3 where it fits a long.
        let wrapped = if c_long::BITS == 64 { 3 } else { i32::MAX };
        assert_eq!(atoi("4294967299"), wrapped);
    }

    #[test]
    fn the_connect_time_monitors_read_the_arrays_once() {
        let mut flags = Flags::connected();
        assert_eq!(flags.monitor_changes(0), Some(Clause::HardwareAcquiring));
        assert_eq!(flags.monitor_changes(0), None);
    }

    #[test]
    fn a_read_request_of_zero_clears_its_flag_and_falls_through() {
        let mut flags = Flags {
            read_arrays: true,
            hardware_acquiring: false,
        };
        assert_eq!(flags.monitor_changes(0), None);
        assert!(!flags.read_arrays);
        flags.read_arrays = true;
        assert_eq!(flags.monitor_changes(1), Some(Clause::ReadArrays));
        assert_eq!(flags.monitor_changes(1), None);
    }

    #[test]
    fn a_read_request_goes_before_an_acquisition_change() {
        let mut flags = Flags::connected();
        assert_eq!(flags.monitor_changes(1), Some(Clause::ReadArrays));
        assert_eq!(flags.monitor_changes(1), Some(Clause::HardwareAcquiring));
    }
}
