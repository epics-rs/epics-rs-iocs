//! Which timestamp an item takes when its `@element` source is unusable.

use std::time::{Duration, UNIX_EPOCH};

use async_opcua::types::{DataValue, DateTime, Variant};

use opcua::item::Item;
use opcua::link::TimestampSource;
use opcua::queue::ProcessReason;

/// `DataElementOpen62541Node::setIncomingData` keeps `tsData = tsSource` when
/// the timestamp element is missing, so the server time must not be taken.
#[test]
fn an_unusable_timestamp_element_takes_the_source_time() {
    let mut item = Item::pending();
    item.link.timestamp = TimestampSource::Data;
    item.link.timestamp_element = "sourceTime".to_string();

    let source = DateTime::from((2030, 1, 1, 0, 0, 0));
    let server = DateTime::from((2001, 1, 1, 0, 0, 0));
    let value = DataValue {
        value: Some(Variant::Int32(7)),
        source_timestamp: Some(source),
        server_timestamp: Some(server),
        ..Default::default()
    };
    item.set_incoming_data(value, ProcessReason::IncomingData);

    assert_eq!(
        item.last_timestamp,
        Some(UNIX_EPOCH + Duration::from_secs(1_893_456_000))
    );
}
