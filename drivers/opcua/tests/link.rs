//! Link grammar tests (`linkParser.cpp`, and the examples in the upstream
//! `documentation/how-to/record_configuration_*.md`).

use std::collections::HashMap;

use opcua::link::{
    Bini, InfoDefaults, LinkError, LinkInfo, LinkTarget, NameResolver, NodeIdentifier, RecordKind,
    TimestampSource, parse_link, split_escaped,
};

/// One session `sess`, one subscription `subs` on it. Every other name is taken
/// for an `opcuaItem` record.
struct Names;

impl NameResolver for Names {
    fn subscription_session(&self, name: &str) -> Option<String> {
        (name == "subs").then(|| "sess".to_string())
    }
    fn is_session(&self, name: &str) -> bool {
        name == "sess"
    }
}

const INPUT: RecordKind = RecordKind {
    is_output: false,
    is_item_record: false,
};
const OUTPUT: RecordKind = RecordKind {
    is_output: true,
    is_item_record: false,
};
const ITEM: RecordKind = RecordKind {
    is_output: false,
    is_item_record: true,
};

fn parse(link: &str, kind: RecordKind) -> Result<LinkInfo, LinkError> {
    parse_link(link, kind, &InfoDefaults::default(), &Names)
}

fn ok(link: &str, kind: RecordKind) -> LinkInfo {
    parse(link, kind).unwrap_or_else(|e| panic!("parsing '{link}' failed: {e}"))
}

fn message(link: &str, kind: RecordKind) -> String {
    match parse(link, kind) {
        Ok(_) => panic!("parsing '{link}' should have failed"),
        Err(e) => e.0,
    }
}

// ---------------------------------------------------------------- first token

#[test]
fn subscription_link_carries_the_subscriptions_session() {
    let l = ok("subs ns=2;s=Sensor", INPUT);
    assert_eq!(
        l.target,
        LinkTarget::Subscription {
            subscription: "subs".into(),
            session: "sess".into()
        }
    );
    assert_eq!(l.session(), Some("sess"));
    assert_eq!(l.subscription(), Some("subs"));
    assert!(l.linked_to_item);
}

#[test]
fn session_link_has_no_subscription() {
    let l = ok("sess ns=2;s=Sensor monitor=n", INPUT);
    assert_eq!(l.target, LinkTarget::Session("sess".into()));
    assert_eq!(l.subscription(), None);
    assert!(l.linked_to_item);
}

#[test]
fn unknown_name_is_an_item_record_and_the_record_is_a_structure_element() {
    let l = ok("itemRec element=Temperature", INPUT);
    assert_eq!(l.target, LinkTarget::ItemRecord("itemRec".into()));
    assert_eq!(l.item_record(), Some("itemRec"));
    assert!(!l.linked_to_item);
    assert_eq!(l.element, "Temperature");
    assert_eq!(l.element_path, ["Temperature"]);
}

#[test]
fn empty_link_is_rejected() {
    assert_eq!(
        message("", INPUT),
        "link is missing subscription/session/opcuaItemRecord name"
    );
}

// ------------------------------------------------------------ node addressing

#[test]
fn string_identifier() {
    let l = ok("subs ns=2;s=Some.Node.Name", INPUT);
    assert_eq!(l.namespace_index, 2);
    assert_eq!(
        l.identifier,
        Some(NodeIdentifier::String("Some.Node.Name".into()))
    );
}

#[test]
fn numeric_identifier_accepts_decimal_and_hex() {
    assert_eq!(
        ok("subs ns=1;i=2045", INPUT).identifier,
        Some(NodeIdentifier::Numeric(2045))
    );
    assert_eq!(
        ok("subs ns=1;i=0x10", INPUT).identifier,
        Some(NodeIdentifier::Numeric(16))
    );
}

#[test]
fn namespace_index_defaults_to_zero() {
    assert_eq!(ok("subs i=2258", INPUT).namespace_index, 0);
}

#[test]
fn a_link_without_an_identifier_is_rejected() {
    // The C leaves `linkInfo::identifierNumber` uninitialized and defaults
    // `identifierIsNumeric` to false, so this silently addressed `ns=0;s=""`.
    assert_eq!(
        message("subs ns=2", INPUT),
        "link is missing the node identifier ('i=' or 's=')"
    );
}

#[test]
fn out_of_range_numbers_are_rejected() {
    assert_eq!(
        message("subs ns=70000;i=1", INPUT),
        "error converting '70000' to UInt16"
    );
    assert_eq!(
        message("subs ns=1;i=-3", INPUT),
        "error converting '-3' to UInt32"
    );
}

#[test]
fn node_options_are_rejected_on_a_structure_element() {
    // C reaches its `invalid option` arm here, because every node-option arm is
    // guarded by `linkedToItem`; the message did not say why.
    let msg = message("itemRec ns=2;s=X", INPUT);
    assert!(
        msg.starts_with("option 'ns' addresses the OPC UA node"),
        "{msg}"
    );
}

// ------------------------------------------------------------------- monitoring

#[test]
fn monitor_defaults_on_and_needs_a_subscription() {
    assert!(ok("subs i=1", INPUT).monitor);
    assert_eq!(
        message("sess i=1", INPUT),
        "monitor=y requires link to a subscription"
    );
    assert!(!ok("sess i=1 monitor=n", INPUT).monitor);
}

#[test]
fn readback_is_an_alias_of_monitor() {
    assert!(!ok("sess i=1 readback=no", OUTPUT).monitor);
}

#[test]
fn yes_no_forms() {
    for yes in ["y", "yes", "Y", "T", "true", "1"] {
        assert!(
            ok(&format!("subs i=1 monitor={yes}"), INPUT).monitor,
            "{yes}"
        );
    }
    for no in ["n", "no", "N", "F", "false", "0"] {
        assert!(
            !ok(&format!("sess i=1 monitor={no}"), INPUT).monitor,
            "{no}"
        );
    }
    assert_eq!(message("subs i=1 monitor=x", INPUT), "illegal value 'x'");
    assert_eq!(
        message("subs i=1 monitor=", INPUT),
        "no value for option 'monitor'"
    );
}

#[test]
fn sampling_deadband_and_queue_sizes() {
    let l = ok(
        "subs i=1 sampling=100 deadband=0.5 qsize=10 discard=new",
        INPUT,
    );
    assert_eq!(l.sampling_interval, 100.0);
    assert_eq!(l.deadband, 0.5);
    assert_eq!(l.queue_size, 10);
    assert!(!l.discard_oldest);
    assert_eq!(
        message("subs i=1 discard=oldest", INPUT),
        "illegal value 'oldest'"
    );
}

#[test]
fn client_queue_size_is_derived_from_the_server_queue_size() {
    // ceil(1.5 * qsize), floored at 3 (the ClientQueueSizeFactor /
    // MinimumClientQueueSize defaults).
    assert_eq!(ok("subs i=1", INPUT).client_queue_size, 3);
    assert_eq!(ok("subs i=1 qsize=1", INPUT).client_queue_size, 3);
    assert_eq!(ok("subs i=1 qsize=2", INPUT).client_queue_size, 3);
    assert_eq!(ok("subs i=1 qsize=3", INPUT).client_queue_size, 5);
    assert_eq!(ok("subs i=1 qsize=10", INPUT).client_queue_size, 15);
    // An explicit cqsize wins outright — no factor, no minimum.
    assert_eq!(ok("subs i=1 qsize=10 cqsize=4", INPUT).client_queue_size, 4);
}

#[test]
fn register_node() {
    assert!(!ok("subs i=1", INPUT).register_node);
    assert!(ok("subs i=1 register=y", INPUT).register_node);
}

// -------------------------------------------------------------------- timestamp

#[test]
fn timestamp_selection() {
    assert_eq!(ok("subs i=1", INPUT).timestamp, TimestampSource::Server);
    assert_eq!(
        ok("subs i=1 timestamp=source", INPUT).timestamp,
        TimestampSource::Source
    );
    assert_eq!(
        ok("subs i=1 timestamp=data", INPUT).timestamp,
        TimestampSource::Data
    );
    assert_eq!(
        message("subs i=1 timestamp=wall", INPUT),
        "illegal value 'wall'"
    );
}

#[test]
fn an_item_record_takes_its_data_timestamp_from_a_named_element() {
    let l = ok("subs i=1 timestamp=@sourceTime", ITEM);
    assert_eq!(l.timestamp, TimestampSource::Data);
    assert_eq!(l.timestamp_element, "sourceTime");
    // Bare `data` has no element to take the time from on an item record.
    assert_eq!(
        message("subs i=1 timestamp=data", ITEM),
        "illegal value 'data'"
    );
    // ... and the `@element` form is meaningless anywhere else.
    assert_eq!(
        message("subs i=1 timestamp=@sourceTime", INPUT),
        "illegal value '@sourceTime'"
    );
}

// ------------------------------------------------------------------------ bini

#[test]
fn bini_read_and_ignore_are_open_to_every_record() {
    assert_eq!(ok("subs i=1", INPUT).bini, Bini::Read);
    assert_eq!(ok("subs i=1 bini=ignore", INPUT).bini, Bini::Ignore);
    assert_eq!(ok("subs i=1 bini=read", OUTPUT).bini, Bini::Read);
}

#[test]
fn bini_write_needs_something_to_write() {
    assert_eq!(ok("subs i=1 bini=write", OUTPUT).bini, Bini::Write);
    assert_eq!(ok("subs i=1 bini=write", ITEM).bini, Bini::Write);
    assert_eq!(
        message("subs i=1 bini=write", INPUT),
        "illegal value 'write' for option 'bini'"
    );
}

// --------------------------------------------------------------------- escapes

#[test]
fn element_paths_split_on_dots() {
    let l = ok("itemRec element=level.value", INPUT);
    assert_eq!(l.element_path, ["level", "value"]);
}

#[test]
fn an_escaped_dot_stays_inside_an_element_name() {
    let l = ok(r"itemRec element=outer.inner\.name", INPUT);
    assert_eq!(l.element_path, ["outer", "inner.name"]);
}

#[test]
fn escaped_separators_stay_inside_an_option_value() {
    // A Siemens S7 node identifier contains a space, and an element name may too.
    let l = ok(r#"subs ns=4;s="Data\ block"."My\ tag""#, INPUT);
    assert_eq!(
        l.identifier,
        Some(NodeIdentifier::String(r#""Data block"."My tag""#.into()))
    );
    let l = ok(r"itemRec element=my\;elem.x", INPUT);
    assert_eq!(l.element_path, ["my;elem", "x"]);
}

#[test]
fn an_escaped_separator_left_of_the_equals_sign_does_not_shift_the_split() {
    // The C erases the backslash from the link string but keeps the `=`
    // position it computed beforehand (`linkParser.cpp:234-239`), so an escaped
    // separator in front of the `=` moved the name/value boundary by one
    // character. Here the option is simply unknown, and says so.
    assert_eq!(
        message(r"subs i=1 mon\ itor=y", INPUT),
        "invalid option 'mon itor'"
    );
}

#[test]
fn options_separate_on_spaces_tabs_and_semicolons_alike() {
    let l = ok("subs\tns=2;s=X\tmonitor=y;bini=ignore  register=y", INPUT);
    assert_eq!(l.namespace_index, 2);
    assert_eq!(l.identifier, Some(NodeIdentifier::String("X".into())));
    assert!(l.monitor);
    assert_eq!(l.bini, Bini::Ignore);
    assert!(l.register_node);
}

#[test]
fn an_option_without_a_value_is_rejected() {
    assert_eq!(
        message("subs i=1 monitor", INPUT),
        "expected '=' in 'monitor'"
    );
    assert_eq!(
        message("subs i=1 nosuch=1", INPUT),
        "invalid option 'nosuch'"
    );
}

#[test]
fn split_escaped_keeps_empty_tokens() {
    assert_eq!(split_escaped("a..b", '.'), ["a", "", "b"]);
    assert_eq!(split_escaped(".a", '.'), ["", "a"]);
    assert_eq!(split_escaped("a.", '.'), ["a", ""]);
    assert_eq!(split_escaped("", '.'), [""]);
    assert_eq!(split_escaped(r"a\.b.c", '.'), ["a.b", "c"]);
    // A backslash that does not precede the delimiter is an ordinary character
    // (Windows paths in a node identifier survive).
    assert_eq!(split_escaped(r"C:\dir.x", '.'), [r"C:\dir", "x"]);
}

// ------------------------------------------------------ deprecated info() tags

#[test]
fn info_tags_set_defaults_that_link_options_then_override() {
    let mut info = HashMap::new();
    info.insert("opcua:SAMPLING".to_string(), "50".to_string());
    info.insert("opcua:QSIZE".to_string(), "8".to_string());
    info.insert("opcua:DISCARD".to_string(), "new".to_string());
    info.insert("opcua:TIMESTAMP".to_string(), "source".to_string());
    info.insert("opcua:READBACK".to_string(), "n".to_string());
    info.insert("opcua:ELEMENT".to_string(), "a.b".to_string());
    let info = InfoDefaults::from_info(&info);

    let l = parse_link("sess i=1", INPUT, &info, &Names).expect("parse");
    assert_eq!(l.sampling_interval, 50.0);
    assert_eq!(l.queue_size, 8);
    assert!(!l.discard_oldest);
    assert_eq!(l.timestamp, TimestampSource::Source);
    assert!(!l.monitor);
    assert_eq!(l.element_path, ["a", "b"]);

    let l = parse_link(
        "subs i=1 sampling=10 qsize=2 monitor=y",
        INPUT,
        &info,
        &Names,
    )
    .expect("parse");
    assert_eq!(l.sampling_interval, 10.0);
    assert_eq!(l.queue_size, 2);
    assert!(l.monitor);
}

#[test]
fn an_empty_info_tag_is_no_tag() {
    let mut info = HashMap::new();
    info.insert("opcua:ELEMENT".to_string(), String::new());
    let info = InfoDefaults::from_info(&info);
    assert!(info.element.is_none());
}
