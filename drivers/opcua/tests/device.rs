//! Record device support: how a value from the server lands in each record type,
//! and what each record type sends back (`devOpcua.cpp`'s per-record `dset`).
//!
//! The records here are the framework's real ones, so the conversions the record
//! owns (LINR, MASK/SHFT, the state table, the bit fields) are exercised as the
//! IOC would run them, not re-implemented in a double.

use std::sync::Arc;
use std::time::{Duration, SystemTime};

use async_opcua::types::{StatusCode, Variant};
use async_trait::async_trait;
use parking_lot::Mutex;

use epics_rs::base::server::device_support::DeviceSupport;
use epics_rs::base::server::record::{AlarmSeverity, ProcessContext, Record, ScanType};
use epics_rs::base::server::records::ai::AiRecord;
use epics_rs::base::server::records::ao::AoRecord;
use epics_rs::base::server::records::bi::BiRecord;
use epics_rs::base::server::records::bo::BoRecord;
use epics_rs::base::server::records::int64in::Int64inRecord;
use epics_rs::base::server::records::longin::LonginRecord;
use epics_rs::base::server::records::longout::LongoutRecord;
use epics_rs::base::server::records::lsi::LsiRecord;
use epics_rs::base::server::records::mbbi::MbbiRecord;
use epics_rs::base::server::records::mbbo::MbboRecord;
use epics_rs::base::server::records::mbbo_direct::MbboDirectRecord;
use epics_rs::base::server::records::stringin::StringinRecord;
use epics_rs::base::server::records::waveform::{ArrayKind, WaveformRecord};
use epics_rs::base::types::{DbFieldType, EpicsValue};

use opcua::client::{UaConnection, UaConnector};
use opcua::device_support::OpcuaDevice;
use opcua::item::Leaf;
use opcua::queue::{ConnectionStatus, ProcessReason, Update};
use opcua::registry::Registry;
use opcua::session::SessionConfig;
use opcua::value::EnumChoices;

// --------------------------------------------------------------------- fixtures

/// The device tests never connect: the worker is not started, so the requests the
/// device queues just sit in the session's command channel.
struct NoConnector;

#[async_trait]
impl UaConnector for NoConnector {
    async fn connect(&self, _config: &SessionConfig) -> Result<Arc<dyn UaConnection>, String> {
        Err("no server".to_string())
    }
}

fn registry() -> Arc<Registry> {
    let registry = Registry::new(Arc::new(NoConnector));
    registry
        .add_session(SessionConfig::new("S", "opc.tcp://server:4840"))
        .unwrap();
    registry
}

/// Bind one record to `ns=2;s=Node` on session `S`.
///
/// `init_record(0)` is what `iocInit` runs once the database has set the record's
/// fields, and it is where mbbi/mbbo derive `sdef` from their state table — a
/// record configured here field by field would otherwise reach the device with a
/// state table it does not know it has.
fn bind(registry: &Arc<Registry>, record: &mut dyn Record, link: &str) -> OpcuaDevice {
    record.init_record(0).expect("the record initialises");
    let mut device = OpcuaDevice::new(registry.clone(), link.to_string());
    device.set_record_info("REC", ScanType::IoIntr);
    device.init(record).expect("the link binds");
    device
}

fn leaf(registry: &Arc<Registry>) -> Arc<Mutex<Leaf>> {
    let session = registry.session("S").unwrap();
    let items = session.items.lock();
    let item = items.last().expect("the record added an item").clone();
    let leaf = item.lock().leaves[0].clone();
    drop(items);
    leaf
}

/// A value arrives from the server, and the record processes for it.
fn incoming(
    device: &mut OpcuaDevice,
    leaf: &Arc<Mutex<Leaf>>,
    record: &mut dyn Record,
    v: Variant,
) {
    {
        let mut leaf = leaf.lock();
        leaf.state = ConnectionStatus::Up;
        leaf.incoming = Some(v.clone());
        leaf.queue.push(Update::new(
            ProcessReason::IncomingData,
            Some(v),
            StatusCode::Good,
            SystemTime::UNIX_EPOCH + Duration::from_secs(1),
        ));
    }
    let outcome = device.read(record).expect("the update is delivered");
    // The framework runs the record's own conversion next when the device did
    // not compute the value itself.
    record.set_device_did_compute(outcome.did_compute);
    record.process().expect("the record processes");
}

/// The record processes on its own (a put, a scan): its value goes out.
fn outgoing(device: &mut OpcuaDevice, record: &mut dyn Record) {
    record.process().expect("the record processes");
    device.write(record).expect("the value is sent");
}

/// Set the leaf up as a live element that has already seen a value of this type.
fn typed(leaf: &Arc<Mutex<Leaf>>, template: Variant) {
    let mut leaf = leaf.lock();
    leaf.state = ConnectionStatus::Up;
    leaf.incoming = Some(template);
}

fn field(record: &dyn Record, name: &str) -> EpicsValue {
    record.get_field(name).expect("the field exists")
}

/// The per-cycle state the framework pushes into device support, for a record
/// whose value is no longer undefined.
fn defined() -> ProcessContext {
    ProcessContext {
        udf: false,
        udfs: AlarmSeverity::Invalid,
        nsev: AlarmSeverity::NoAlarm,
        phas: 0,
        tse: 0,
        time: SystemTime::UNIX_EPOCH,
        tsel: String::new(),
        dtyp: opcua::device_support::DTYP.to_string(),
    }
}

// ------------------------------------------------------------------ scalar input

#[test]
fn longin_takes_the_value_into_val() {
    let registry = registry();
    let mut record = LonginRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Int32(-7));

    assert_eq!(field(&record, "VAL"), EpicsValue::Long(-7));
    assert_eq!(
        device.last_timestamp(),
        Some(SystemTime::UNIX_EPOCH + Duration::from_secs(1))
    );
    assert_eq!(device.last_alarm(), None);
}

#[test]
fn int64in_takes_a_value_wider_than_an_int32() {
    let registry = registry();
    let mut record = Int64inRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Int64(1 << 40));

    assert_eq!(field(&record, "VAL"), EpicsValue::Int64(1 << 40));
}

#[test]
fn a_value_out_of_the_records_range_raises_read_invalid() {
    let registry = registry();
    let mut record = LonginRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::Int64(i64::from(i32::MAX) + 1),
    );

    // READ_ALARM, INVALID — and the record keeps the value it had.
    assert_eq!(device.last_alarm(), Some((1, 3)));
    assert_eq!(field(&record, "VAL"), EpicsValue::Long(1));
}

#[test]
fn bi_hands_the_raw_value_to_the_records_own_conversion() {
    let registry = registry();
    let mut record = BiRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Boolean(true));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(1));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));
}

#[test]
fn stringin_truncates_to_the_epics_string_size() {
    let registry = registry();
    let mut record = StringinRecord::new("");
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    let long = "x".repeat(60);
    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::String(long.as_str().into()),
    );

    assert_eq!(
        field(&record, "VAL"),
        EpicsValue::String("x".repeat(39).into())
    );
}

#[test]
fn lsi_takes_a_long_string_and_sets_len() {
    let registry = registry();
    let mut record = LsiRecord::new("");
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::String("a long string".into()),
    );

    assert_eq!(
        field(&record, "VAL"),
        EpicsValue::CharArray(b"a long string".to_vec())
    );
    assert_eq!(field(&record, "LEN"), EpicsValue::ULong(14));
}

// ------------------------------------------------------------------------ analog

#[test]
fn ai_without_conversion_applies_aslo_aoff_and_keeps_the_fraction() {
    let registry = registry();
    let mut record = AiRecord::new(0.0);
    record.put_field("ASLO", EpicsValue::Double(2.0)).unwrap();
    record.put_field("AOFF", EpicsValue::Double(1.0)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Double(1.5));

    // 1.5 * 2 + 1 — the record's own no-conversion path would have gone through
    // RVAL, an epicsInt32, and lost the 0.5.
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(4.0));
}

#[test]
fn ai_smooths_against_the_previous_value_once_it_is_defined() {
    let registry = registry();
    let mut record = AiRecord::new(0.0);
    record.put_field("SMOO", EpicsValue::Double(0.5)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    // The first value lands whole: the record is still UDF.
    incoming(&mut device, &leaf, &mut record, Variant::Double(10.0));
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(10.0));

    // The record is defined now, so the next value is smoothed into it.
    device.set_process_context(&defined());
    incoming(&mut device, &leaf, &mut record, Variant::Double(20.0));
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(15.0));
}

#[test]
fn ai_with_a_linear_conversion_lets_the_record_convert_rval() {
    let registry = registry();
    let mut record = AiRecord::new(0.0);
    record.put_field("LINR", EpicsValue::Short(1)).unwrap(); // LINEAR
    record.put_field("ESLO", EpicsValue::Double(0.5)).unwrap();
    record.put_field("EOFF", EpicsValue::Double(3.0)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Int32(10));

    assert_eq!(field(&record, "RVAL"), EpicsValue::Long(10));
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(8.0));
}

#[test]
fn ao_reads_back_through_the_records_inverse_conversion() {
    let registry = registry();
    let mut record = AoRecord::new(0.0);
    record.put_field("LINR", EpicsValue::Short(1)).unwrap(); // LINEAR
    record.put_field("ESLO", EpicsValue::Double(0.5)).unwrap();
    record.put_field("EOFF", EpicsValue::Double(3.0)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Int32(10));

    // RVAL keeps the raw; VAL is the engineering value the record derived. The
    // forward convert must not run and recompute RVAL from the stale VAL.
    assert_eq!(field(&record, "RVAL"), EpicsValue::Long(10));
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(8.0));
}

#[test]
fn ao_without_conversion_sends_val_and_reads_back_into_val() {
    let registry = registry();
    let mut record = AoRecord::new(0.0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::Double(2.5));
    assert_eq!(field(&record, "VAL"), EpicsValue::Double(2.5));

    record.put_field("VAL", EpicsValue::Double(7.5)).unwrap();
    outgoing(&mut device, &mut record);
    assert_eq!(leaf.lock().outgoing, Some(Variant::Double(7.5)));
    assert!(leaf.lock().dirty);
}

// ----------------------------------------------------------------------- binary

#[test]
fn bo_reads_back_val_from_the_raw_value() {
    let registry = registry();
    let mut record = BoRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(1));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(1));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));
}

#[test]
fn bo_sends_rval() {
    let registry = registry();
    let mut record = BoRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    typed(&leaf, Variant::Boolean(false));

    outgoing(&mut device, &mut record);

    assert_eq!(leaf.lock().outgoing, Some(Variant::Boolean(true)));
}

#[test]
fn mbbo_direct_reads_back_val_and_its_bits_from_the_shifted_raw() {
    let registry = registry();
    let mut record = MbboDirectRecord::default();
    record.put_field("SHFT", EpicsValue::UShort(4)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(0x50));

    // RVAL keeps the raw the server sent. The C shifts `prec->rval` in place
    // (devOpcua.cpp:908-910), leaving RVAL equal to VAL until the next put
    // recomputes it; the record's own readback keeps the two fields apart.
    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(0x50));
    assert_eq!(field(&record, "VAL"), EpicsValue::Long(5));
    assert_eq!(field(&record, "B0"), EpicsValue::UChar(1));
    assert_eq!(field(&record, "B1"), EpicsValue::UChar(0));
    assert_eq!(field(&record, "B2"), EpicsValue::UChar(1));
}

// ------------------------------------------------------------------ enumerations

#[test]
fn mbbi_with_state_values_lets_the_record_look_the_state_up() {
    let registry = registry();
    let mut record = MbbiRecord::new(0);
    record.put_field("ZRVL", EpicsValue::ULong(10)).unwrap();
    record.put_field("ONVL", EpicsValue::ULong(20)).unwrap();
    record
        .put_field("ONST", EpicsValue::String("Running".into()))
        .unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(20));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(20));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));
}

#[test]
fn mbbi_without_state_values_takes_the_raw_value_as_the_state_index() {
    let registry = registry();
    let mut record = MbbiRecord::new(0);
    record
        .put_field("TWST", EpicsValue::String("Third".into()))
        .unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(2));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(2));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(2));
}

#[test]
fn the_servers_enumeration_fills_a_state_table_the_database_left_empty() {
    let registry = registry();
    let mut record = MbbiRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    let mut choices = EnumChoices::new();
    choices.insert(3, "Idle".to_string());
    choices.insert(7, "Busy".to_string());
    leaf.lock().choices = Some(Arc::new(choices));

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(7));

    assert_eq!(field(&record, "ZRVL"), EpicsValue::ULong(3));
    assert_eq!(field(&record, "ZRST"), EpicsValue::String("Idle".into()));
    assert_eq!(field(&record, "ONVL"), EpicsValue::ULong(7));
    assert_eq!(field(&record, "ONST"), EpicsValue::String("Busy".into()));
    // An unused slot must not hold 0 — an incoming 0 would match it instead of
    // the state that really carries it (devOpcua.cpp:725-727).
    assert_eq!(field(&record, "TWVL"), EpicsValue::ULong(u32::MAX));
    assert_eq!(field(&record, "TWST"), EpicsValue::String("".into()));
    // The state the value names, through the table the server just defined.
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));

    // And the new table is posted DBE_PROPERTY, once.
    let mut rx = device
        .property_post_receiver()
        .expect("mbbi posts properties");
    let posted = rx.try_recv().expect("the table was posted");
    assert_eq!(posted.len(), 32);
    assert!(rx.try_recv().is_err());
}

#[test]
fn a_state_table_the_database_defined_is_not_overwritten_by_the_server() {
    let registry = registry();
    let mut record = MbbiRecord::new(0);
    record.put_field("ZRVL", EpicsValue::ULong(10)).unwrap();
    record
        .put_field("ZRST", EpicsValue::String("Mine".into()))
        .unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    let mut choices = EnumChoices::new();
    choices.insert(3, "Theirs".to_string());
    leaf.lock().choices = Some(Arc::new(choices));

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(10));

    assert_eq!(field(&record, "ZRVL"), EpicsValue::ULong(10));
    assert_eq!(field(&record, "ZRST"), EpicsValue::String("Mine".into()));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(0));
}

#[test]
fn mbbo_sends_rval_when_the_database_defined_state_values() {
    let registry = registry();
    let mut record = MbboRecord::new(0);
    record.put_field("ZRVL", EpicsValue::ULong(10)).unwrap();
    record.put_field("ONVL", EpicsValue::ULong(20)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    typed(&leaf, Variant::UInt32(0));

    record.put_field("VAL", EpicsValue::Enum(1)).unwrap();
    outgoing(&mut device, &mut record);

    assert_eq!(leaf.lock().outgoing, Some(Variant::UInt32(20)));
}

#[test]
fn mbbo_sends_the_state_index_when_no_state_values_are_defined() {
    let registry = registry();
    let mut record = MbboRecord::new(0);
    record
        .put_field("ONST", EpicsValue::String("On".into()))
        .unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    typed(&leaf, Variant::UInt32(0));

    record.put_field("VAL", EpicsValue::Enum(1)).unwrap();
    outgoing(&mut device, &mut record);

    assert_eq!(leaf.lock().outgoing, Some(Variant::UInt32(1)));
}

#[test]
fn without_nobt_the_device_widens_the_mask_to_every_bit() {
    // The record's own init leaves MASK at 0 when the database gives no NOBT
    // (mbboRecord.c:136-137) — upstream's own example shape
    // (S7-1500-DB1.db:45-54). The device widens it (devOpcua.cpp:143-157), which
    // is what keeps the readback's mask from erasing the value.
    let registry = registry();
    let mut record = MbboRecord::new(0);
    for (field, value) in [("ZRVL", 1u32), ("ONVL", 2), ("TWVL", 4), ("THVL", 8)] {
        record.put_field(field, EpicsValue::ULong(value)).unwrap();
    }
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    assert_eq!(field(&record, "MASK"), EpicsValue::ULong(u32::MAX));

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(4));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(4));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(2));
}

#[test]
fn nobt_and_shft_put_the_mask_over_the_nodes_bit_window() {
    // NOBT = 3 bits at SHFT = 4: the device shifts the record's NOBT-derived
    // mask into place (devOpcua.cpp:143-157), and the readback then takes only
    // the window's bits — 0x120 masks to 0x20, shifts to 2, which is ONVL.
    let registry = registry();
    let mut record = MbboRecord::new(0);
    record.put_field("NOBT", EpicsValue::UShort(3)).unwrap();
    record.put_field("SHFT", EpicsValue::UShort(4)).unwrap();
    record.put_field("ZRVL", EpicsValue::ULong(1)).unwrap();
    record.put_field("ONVL", EpicsValue::ULong(2)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    assert_eq!(field(&record, "MASK"), EpicsValue::ULong(0x70));

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(0x120));

    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));
}

#[test]
fn an_mbbo_value_no_state_carries_is_the_unknown_state() {
    let registry = registry();
    let mut record = MbboRecord::new(0);
    record.put_field("ZRVL", EpicsValue::ULong(10)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(11));

    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(65535));
}

#[test]
fn mbbo_reads_back_the_state_the_raw_value_names() {
    let registry = registry();
    let mut record = MbboRecord::new(0);
    record.put_field("ZRVL", EpicsValue::ULong(10)).unwrap();
    record.put_field("ONVL", EpicsValue::ULong(20)).unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(&mut device, &leaf, &mut record, Variant::UInt32(20));

    assert_eq!(field(&record, "RVAL"), EpicsValue::ULong(20));
    assert_eq!(field(&record, "VAL"), EpicsValue::Enum(1));
}

// ------------------------------------------------------------------------ arrays

#[test]
fn a_waveform_takes_the_array_in_its_own_element_type() {
    let registry = registry();
    let mut record = WaveformRecord::new(4, DbFieldType::Double);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::from(vec![1.0f64, 2.0, 3.0]),
    );

    assert_eq!(
        field(&record, "VAL"),
        EpicsValue::DoubleArray(vec![1.0, 2.0, 3.0])
    );
    assert_eq!(field(&record, "NORD"), EpicsValue::ULong(3));
}

#[test]
fn an_array_longer_than_nelm_is_clamped() {
    let registry = registry();
    let mut record = WaveformRecord::new(2, DbFieldType::Long);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::from(vec![1i32, 2, 3, 4]),
    );

    assert_eq!(field(&record, "VAL"), EpicsValue::LongArray(vec![1, 2]));
}

#[test]
fn a_char_waveform_holds_a_string_node_as_its_bytes() {
    let registry = registry();
    let mut record = WaveformRecord::new(40, DbFieldType::Char);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);

    incoming(
        &mut device,
        &leaf,
        &mut record,
        Variant::String("text".into()),
    );

    assert_eq!(
        field(&record, "VAL"),
        EpicsValue::CharArray(b"text".to_vec())
    );
}

#[test]
fn an_aao_sends_its_first_nord_elements() {
    let registry = registry();
    let mut record = WaveformRecord::with_kind(ArrayKind::Aao);
    record.put_field("NELM", EpicsValue::Long(4)).unwrap();
    record
        .put_field("FTVL", EpicsValue::Short(DbFieldType::Long as i16))
        .unwrap();
    record
        .put_field("VAL", EpicsValue::LongArray(vec![7, 8]))
        .unwrap();
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    typed(&leaf, Variant::from(vec![0i32, 0, 0, 0]));

    outgoing(&mut device, &mut record);

    assert_eq!(leaf.lock().outgoing, Some(Variant::from(vec![7i32, 8])));
}

// ------------------------------------------------------------- the write pathway

#[test]
fn an_output_record_takes_the_type_of_the_value_the_node_last_sent() {
    let registry = registry();
    let mut record = LongoutRecord::new(300);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    typed(&leaf, Variant::Byte(0));

    outgoing(&mut device, &mut record);

    // 300 does not fit a Byte: the write is refused, WRITE_ALARM / INVALID.
    assert_eq!(device.last_alarm(), Some((2, 3)));
    assert_eq!(leaf.lock().outgoing, None);

    record.put_field("VAL", EpicsValue::Long(200)).unwrap();
    outgoing(&mut device, &mut record);
    assert_eq!(leaf.lock().outgoing, Some(Variant::Byte(200)));
    assert_eq!(device.last_alarm(), None);
}

#[test]
fn a_record_on_a_session_that_is_down_raises_a_comm_alarm() {
    let registry = registry();
    let mut record = LongoutRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    assert_eq!(leaf.lock().state, ConnectionStatus::Down);

    outgoing(&mut device, &mut record);

    // COMM_ALARM, INVALID — and nothing is queued for the server.
    assert_eq!(device.last_alarm(), Some((9, 3)));
    assert!(!leaf.lock().dirty);
}

#[test]
fn an_input_record_on_a_session_that_is_down_raises_a_comm_alarm() {
    let registry = registry();
    let mut record = LonginRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");

    device.read(&mut record).expect("the read stage runs");

    assert_eq!(device.last_alarm(), Some((9, 3)));
}

#[test]
fn the_records_value_does_not_overtake_the_initial_read() {
    let registry = registry();
    let mut record = LongoutRecord::new(5);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    leaf.lock().state = ConnectionStatus::InitialRead;

    outgoing(&mut device, &mut record);

    assert_eq!(device.last_alarm(), None);
    assert!(!leaf.lock().dirty);
}

#[test]
fn bini_write_asks_an_output_record_to_send_its_value() {
    let registry = registry();
    let mut record = LongoutRecord::new(5);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n bini=write");
    let leaf = leaf(&registry);
    {
        let mut leaf = leaf.lock();
        leaf.state = ConnectionStatus::InitialWrite;
        leaf.incoming = Some(Variant::Int32(0));
        leaf.queue.push(Update::new(
            ProcessReason::WriteRequest,
            None,
            StatusCode::Good,
            SystemTime::UNIX_EPOCH,
        ));
    }

    device.read(&mut record).expect("the request is delivered");

    assert_eq!(leaf.lock().outgoing, Some(Variant::Int32(5)));
    assert!(leaf.lock().dirty);
}

// ---------------------------------------------------------------- failure reasons

#[test]
fn each_failure_reason_raises_its_own_alarm() {
    let registry = registry();
    let mut record = LonginRecord::new(0);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    leaf.lock().state = ConnectionStatus::Up;

    for (reason, alarm) in [
        (ProcessReason::ReadFailure, (1, 3)),
        (ProcessReason::WriteFailure, (2, 3)),
        (ProcessReason::ConnectionLoss, (9, 3)),
    ] {
        leaf.lock().queue.push(Update::new(
            reason,
            None,
            StatusCode::Good,
            SystemTime::UNIX_EPOCH,
        ));
        device.read(&mut record).expect("the update is delivered");
        assert_eq!(device.last_alarm(), Some(alarm), "{reason:?}");
    }

    // A completed write leaves the record alone.
    leaf.lock().queue.push(Update::new(
        ProcessReason::WriteComplete,
        None,
        StatusCode::Good,
        SystemTime::UNIX_EPOCH,
    ));
    device.read(&mut record).expect("the update is delivered");
    assert_eq!(device.last_alarm(), None);
    assert_eq!(field(&record, "VAL"), EpicsValue::Long(0));
}

#[test]
fn a_bad_status_code_carries_no_value() {
    let registry = registry();
    let mut record = LonginRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    {
        let mut leaf = leaf.lock();
        leaf.state = ConnectionStatus::Up;
        leaf.queue.push(Update::new(
            ProcessReason::IncomingData,
            Some(Variant::Int32(9)),
            StatusCode::BadNodeIdUnknown,
            SystemTime::UNIX_EPOCH,
        ));
    }

    device.read(&mut record).expect("the update is delivered");

    assert_eq!(device.last_alarm(), Some((1, 3)));
    assert_eq!(field(&record, "VAL"), EpicsValue::Long(1));
}

#[test]
fn an_uncertain_status_code_carries_its_value_with_a_minor_alarm() {
    let registry = registry();
    let mut record = LonginRecord::new(1);
    let mut device = bind(&registry, &mut record, "S ns=2;s=Node monitor=n");
    let leaf = leaf(&registry);
    {
        let mut leaf = leaf.lock();
        leaf.state = ConnectionStatus::Up;
        leaf.queue.push(Update::new(
            ProcessReason::IncomingData,
            Some(Variant::Int32(9)),
            StatusCode::UncertainLastUsableValue,
            SystemTime::UNIX_EPOCH,
        ));
    }

    device.read(&mut record).expect("the update is delivered");

    assert_eq!(device.last_alarm(), Some((1, 1)));
    assert_eq!(field(&record, "VAL"), EpicsValue::Long(9));
}

#[test]
fn a_record_type_with_no_opc_ua_device_support_is_refused() {
    let registry = registry();
    let mut record = epics_rs::base::server::records::calc::CalcRecord::new("");
    let mut device = OpcuaDevice::new(registry.clone(), "S ns=2;s=Node monitor=n".to_string());
    device.set_record_info("REC", ScanType::Passive);

    let err = device
        .init(&mut record)
        .expect_err("calc has no OPC UA dset");
    assert!(err.to_string().contains("calc"), "{err}");
}
