//! INP/OUT link parsing (`linkParser.cpp`, `devOpcua.h:81-113`).
//!
//! Grammar (`documentation/how-to/record_configuration_scalar.md`):
//!
//! ```text
//! @<session|subscription|itemRecord> [<key>=<value> ...]
//! ```
//!
//! The first token names a subscription, a session, or an `opcuaItem` record.
//! Naming a subscription implies its session. Naming an `opcuaItem` record makes
//! this record a *structure element* of that item: node addressing options
//! (`ns`, `s`, `i`, `sampling`, ...) belong to the item and are rejected here,
//! and only `element`/`timestamp`/`monitor`/`bini` remain meaningful.
//!
//! Options are separated by spaces, tabs or semicolons. A separator or an
//! element delimiter can be escaped with a backslash.

use std::collections::HashMap;
use std::sync::atomic::Ordering;

use crate::defaults;

/// Default element-path delimiter (`linkParser.h:24`).
pub const ELEMENT_DELIMITER: char = '.';

/// `enum LinkOptionBini` (`devOpcua.h:35`) — behaviour of a record at IOC init.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Bini {
    /// Take the value from the initial read.
    #[default]
    Read,
    /// Discard the initial read; leave the record's value alone.
    Ignore,
    /// After the initial read, write the record's value back to the server.
    Write,
}

impl Bini {
    pub fn as_str(self) -> &'static str {
        match self {
            Bini::Read => "read",
            Bini::Ignore => "ignore",
            Bini::Write => "write",
        }
    }
}

/// `enum LinkOptionTimestamp` (`devOpcua.h:50`) — which timestamp the record takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TimestampSource {
    /// The server's `serverTimestamp`.
    #[default]
    Server,
    /// The value's `sourceTimestamp`.
    Source,
    /// A DateTime element inside the structure.
    Data,
}

impl TimestampSource {
    pub fn as_str(self) -> &'static str {
        match self {
            TimestampSource::Server => "server",
            TimestampSource::Source => "source",
            TimestampSource::Data => "data",
        }
    }
}

/// The node identifier part of a NodeId — `i=<number>` or `s=<string>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeIdentifier {
    Numeric(u32),
    String(String),
}

/// What the first token of a link resolved to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkTarget {
    /// `<session>` — an unmonitored item on that session.
    Session(String),
    /// `<subscription>` — a monitored item; the session is the subscription's.
    Subscription {
        subscription: String,
        session: String,
    },
    /// `<opcuaItem record>` — this record is a data element of that item.
    ItemRecord(String),
}

impl Default for LinkTarget {
    /// No session — an item that has not been adopted yet
    /// ([`crate::item::Item::pending`]).
    fn default() -> Self {
        Self::Session(String::new())
    }
}

/// Tells the parser what an existing name refers to. The session and
/// subscription registries are populated by iocsh before `dbLoadRecords`, so a
/// name that is in neither can only be an `opcuaItem` record — whose existence
/// is checked when the item tree is wired, once every record has been added.
pub trait NameResolver {
    /// The session a subscription runs on, or `None` if `name` is not a subscription.
    fn subscription_session(&self, name: &str) -> Option<String>;
    /// Whether `name` is a configured session.
    fn is_session(&self, name: &str) -> bool;
}

/// Parsed configuration of one record's link (`struct linkInfo`, `devOpcua.h:81`).
///
/// `Default` is what an item that has not been adopted by the record addressing
/// its node holds ([`crate::item::Item::pending`]) — not a link any record has;
/// [`parse_link`] builds every real one from the module defaults.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LinkInfo {
    /// The link names a session or subscription, so this record addresses an OPC
    /// UA node directly. `false` when it names an `opcuaItem` record instead
    /// (`linkInfo::linkedToItem`).
    pub linked_to_item: bool,
    /// This record *is* an `opcuaItem` record (`linkInfo::isItemRecord`).
    pub is_item_record: bool,
    pub target: LinkTarget,

    pub namespace_index: u16,
    /// `None` only for a structure element, which inherits the item's node.
    pub identifier: Option<NodeIdentifier>,
    pub register_node: bool,

    pub sampling_interval: f64,
    pub queue_size: u32,
    pub client_queue_size: u32,
    pub discard_oldest: bool,
    pub deadband: f64,

    pub element: String,
    pub element_path: Vec<String>,
    pub timestamp: TimestampSource,
    pub timestamp_element: String,
    pub bini: Bini,

    pub is_output: bool,
    pub monitor: bool,
}

impl LinkInfo {
    /// Session this record's item lives on.
    pub fn session(&self) -> Option<&str> {
        match &self.target {
            LinkTarget::Session(s) => Some(s),
            LinkTarget::Subscription { session, .. } => Some(session),
            LinkTarget::ItemRecord(_) => None,
        }
    }

    /// Subscription this record's item is monitored on, if any.
    pub fn subscription(&self) -> Option<&str> {
        match &self.target {
            LinkTarget::Subscription { subscription, .. } => Some(subscription),
            _ => None,
        }
    }

    /// Name of the `opcuaItem` record this record is an element of, if any.
    pub fn item_record(&self) -> Option<&str> {
        match &self.target {
            LinkTarget::ItemRecord(name) => Some(name),
            _ => None,
        }
    }
}

/// A link that could not be parsed. The message is what reaches the operator, so
/// it repeats the C wording where the C had a message worth keeping.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkError(pub String);

impl std::fmt::Display for LinkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for LinkError {}

fn err<T>(msg: impl Into<String>) -> Result<T, LinkError> {
    Err(LinkError(msg.into()))
}

/// `getYesNo` (`linkParser.cpp:41`).
fn yes_no(c: char) -> Result<bool, LinkError> {
    match c {
        'Y' | 'y' | 'T' | 't' | '1' => Ok(true),
        'N' | 'n' | 'F' | 'f' | '0' => Ok(false),
        _ => err(format!("illegal value '{c}'")),
    }
}

/// `splitString` (`linkParser.cpp:54`) — split on `delim`, honouring `\<delim>`
/// escapes, keeping empty tokens for leading/trailing/repeated delimiters.
pub fn split_escaped(s: &str, delim: char) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut current = String::new();
    let mut chars = s.chars().peekable();

    while let Some(c) = chars.next() {
        if c == '\\' && chars.peek() == Some(&delim) {
            current.push(chars.next().expect("peeked"));
        } else if c == delim {
            tokens.push(std::mem::take(&mut current));
        } else {
            current.push(c);
        }
    }
    tokens.push(current);
    tokens
}

/// One `key=value` option, with separator escapes already resolved.
struct Opt {
    name: String,
    value: String,
}

/// Split the option list into `key=value` pairs.
///
/// Separators are space, tab and semicolon; `\<sep>` embeds a separator in a
/// value (the identifier of a Siemens node is `\"db\".\"item\"`, and an element
/// name may contain a space). The C does this by erasing the backslash from the
/// link string in place and re-running `find_first_of` from the same index
/// (`linkParser.cpp:236-239`), which recomputes the separator position but *not*
/// the already-computed `=` position — so an escaped separator to the left of
/// the `=` shifts the name/value split by one character. Unescaping while
/// scanning, as here, has no such ordering hazard.
fn split_options(s: &str) -> Result<Vec<Opt>, LinkError> {
    const SEPARATORS: [char; 3] = [' ', '\t', ';'];
    let is_sep = |c: char| SEPARATORS.contains(&c);

    let mut opts = Vec::new();
    let mut token = String::new();
    let mut chars = s.chars().peekable();

    let flush = |token: &mut String, opts: &mut Vec<Opt>| -> Result<(), LinkError> {
        if token.is_empty() {
            return Ok(());
        }
        let raw = std::mem::take(token);
        match raw.split_once('=') {
            Some((name, value)) => {
                opts.push(Opt {
                    name: name.to_string(),
                    value: value.to_string(),
                });
                Ok(())
            }
            None => err(format!("expected '=' in '{raw}'")),
        }
    };

    while let Some(c) = chars.next() {
        match c {
            '\\' if chars.peek().is_some_and(|&n| is_sep(n)) => {
                token.push(chars.next().expect("peeked"));
            }
            c if is_sep(c) => flush(&mut token, &mut opts)?,
            c => token.push(c),
        }
    }
    flush(&mut token, &mut opts)?;
    Ok(opts)
}

/// Values a record can take from `info()` tags instead of the link
/// (`linkParser.cpp:96-190`). Deprecated upstream; still honoured, still warned about.
#[derive(Debug, Default, Clone)]
pub struct InfoDefaults {
    pub sampling: Option<String>,
    pub qsize: Option<String>,
    pub discard: Option<String>,
    pub timestamp: Option<String>,
    pub readback: Option<String>,
    pub element: Option<String>,
}

impl InfoDefaults {
    /// Collect the `opcua:*` tags out of a record's `info()` map.
    pub fn from_info(info: &HashMap<String, String>) -> Self {
        let get = |k: &str| info.get(k).filter(|v| !v.is_empty()).cloned();
        Self {
            sampling: get("opcua:SAMPLING"),
            qsize: get("opcua:QSIZE"),
            discard: get("opcua:DISCARD"),
            timestamp: get("opcua:TIMESTAMP"),
            readback: get("opcua:READBACK"),
            element: get("opcua:ELEMENT"),
        }
    }

    fn any(&self) -> bool {
        self.sampling.is_some()
            || self.qsize.is_some()
            || self.discard.is_some()
            || self.timestamp.is_some()
            || self.readback.is_some()
            || self.element.is_some()
    }
}

/// What the parser needs to know about the record the link belongs to.
#[derive(Debug, Clone, Copy)]
pub struct RecordKind {
    /// The record has an OUT link rather than an INP link.
    pub is_output: bool,
    /// The record's type is `opcuaItem`.
    pub is_item_record: bool,
}

/// `parseLink` (`linkParser.cpp:78`).
///
/// `link` is the INST_IO text *without* the leading `@` (the framework hands it
/// over with the `@` still attached; the caller strips it).
pub fn parse_link(
    link: &str,
    kind: RecordKind,
    info: &InfoDefaults,
    resolver: &dyn NameResolver,
) -> Result<LinkInfo, LinkError> {
    // Start from the global defaults, then let info() tags and finally link
    // options override them — the C precedence (`linkParser.cpp:96-190` before
    // the option loop at :225).
    let mut pinfo = LinkInfo {
        linked_to_item: true,
        is_item_record: kind.is_item_record,
        // Replaced below once the first token is resolved.
        target: LinkTarget::Session(String::new()),
        namespace_index: 0,
        identifier: None,
        register_node: false,
        sampling_interval: defaults::DEFAULT_SAMPLING_INTERVAL.get(),
        queue_size: defaults::DEFAULT_SERVER_QUEUE_SIZE.load(Ordering::Relaxed) as u32,
        client_queue_size: 0,
        discard_oldest: defaults::DEFAULT_DISCARD_OLDEST.load(Ordering::Relaxed) != 0,
        deadband: 0.0,
        element: String::new(),
        element_path: Vec::new(),
        timestamp: if defaults::DEFAULT_USE_SERVER_TIME.load(Ordering::Relaxed) != 0 {
            TimestampSource::Server
        } else {
            TimestampSource::Source
        },
        timestamp_element: String::new(),
        bini: Bini::Read,
        is_output: kind.is_output,
        monitor: defaults::DEFAULT_OUTPUT_READBACK.load(Ordering::Relaxed) != 0,
    };

    if info.any() {
        log::warn!(
            "DEPRECATION WARNING: setting parameters through info items is deprecated; \
             use link parameters instead."
        );
    }
    apply_info_defaults(&mut pinfo, info)?;

    let link = link.trim_start();
    let (name, rest) = match link.find([' ', '\t', ';']) {
        Some(i) => (&link[..i], &link[i..]),
        None => (link, ""),
    };

    if name.is_empty() {
        return err("link is missing subscription/session/opcuaItemRecord name");
    }

    pinfo.target = if let Some(session) = resolver.subscription_session(name) {
        LinkTarget::Subscription {
            subscription: name.to_string(),
            session,
        }
    } else if resolver.is_session(name) {
        LinkTarget::Session(name.to_string())
    } else {
        // Not a session and not a subscription: it can only be an opcuaItem
        // record. Whether such a record exists is settled when the item tree is
        // wired — at parse time the record may not have been loaded yet.
        pinfo.linked_to_item = false;
        LinkTarget::ItemRecord(name.to_string())
    };

    for opt in split_options(rest)? {
        apply_option(&mut pinfo, &opt)?;
    }

    // `linkParser.cpp:317` — derive the client queue size from the server one.
    if pinfo.client_queue_size == 0 {
        let factor = defaults::CLIENT_QUEUE_SIZE_FACTOR.get().abs();
        let minimum = defaults::MINIMUM_CLIENT_QUEUE_SIZE
            .load(Ordering::Relaxed)
            .unsigned_abs() as u32;
        let sized = (factor * f64::from(pinfo.queue_size)).ceil() as u32;
        pinfo.client_queue_size = sized.max(minimum);
    }

    check_consistency(&pinfo)?;
    Ok(pinfo)
}

fn apply_info_defaults(pinfo: &mut LinkInfo, info: &InfoDefaults) -> Result<(), LinkError> {
    if let Some(s) = &info.sampling {
        pinfo.sampling_interval = parse_f64(s)?;
    }
    if let Some(s) = &info.qsize {
        pinfo.queue_size = parse_u32(s)?;
    }
    if let Some(s) = &info.discard {
        pinfo.discard_oldest = parse_discard(s)?;
    }
    if let Some(s) = &info.timestamp {
        parse_timestamp(pinfo, s)?;
    }
    if let Some(s) = &info.readback {
        pinfo.monitor = yes_no(first_char(s, "readback")?)?;
    }
    if let Some(s) = &info.element {
        set_element(pinfo, s);
    }
    Ok(())
}

fn apply_option(pinfo: &mut LinkInfo, opt: &Opt) -> Result<(), LinkError> {
    let (name, value) = (opt.name.as_str(), opt.value.as_str());

    // Node-related options are meaningless on a structure element: the node is
    // the item record's. The C reaches the `invalid option` arm for these
    // because every node-option arm is guarded by `linkedToItem`
    // (`linkParser.cpp:250-283`), which yields "invalid option 'ns'" — true, but
    // it does not say why. Name the actual reason.
    let node_option = matches!(
        name,
        "ns" | "s" | "i" | "sampling" | "deadband" | "qsize" | "cqsize" | "discard" | "register"
    );
    if node_option && !pinfo.linked_to_item {
        return err(format!(
            "option '{name}' addresses the OPC UA node, which belongs to the \
             opcuaItem record this record is an element of"
        ));
    }

    match name {
        "ns" => pinfo.namespace_index = parse_u16(value)?,
        "s" => pinfo.identifier = Some(NodeIdentifier::String(value.to_string())),
        "i" => pinfo.identifier = Some(NodeIdentifier::Numeric(parse_u32(value)?)),
        "sampling" => pinfo.sampling_interval = parse_f64(value)?,
        "deadband" => pinfo.deadband = parse_f64(value)?,
        "qsize" => pinfo.queue_size = parse_u32(value)?,
        "cqsize" => pinfo.client_queue_size = parse_u32(value)?,
        "discard" => pinfo.discard_oldest = parse_discard(value)?,
        "register" => pinfo.register_node = yes_no(first_char(value, name)?)?,
        "timestamp" => parse_timestamp(pinfo, value)?,
        "monitor" | "readback" => pinfo.monitor = yes_no(first_char(value, name)?)?,
        "element" => set_element(pinfo, value),
        "bini" => {
            pinfo.bini = match value {
                "read" => Bini::Read,
                "ignore" => Bini::Ignore,
                // A plain input record has nothing to write back at init.
                "write" if pinfo.is_item_record || pinfo.is_output => Bini::Write,
                _ => {
                    return err(format!("illegal value '{value}' for option '{name}'"));
                }
            }
        }
        _ => return err(format!("invalid option '{name}'")),
    }
    Ok(())
}

/// `linkParser.cpp:327` — the two rules that make a monitored link coherent.
fn check_consistency(pinfo: &LinkInfo) -> Result<(), LinkError> {
    if pinfo.monitor && pinfo.linked_to_item && pinfo.subscription().is_none() {
        return err("monitor=y requires link to a subscription");
    }

    // A node addressed by this record needs an identifier. The C leaves
    // `linkInfo::identifierNumber` uninitialized and defaults
    // `identifierIsNumeric` to false (`devOpcua.h:95-97`), so a link with
    // neither `i=` nor `s=` silently builds the node `ns=0;s=""` and only fails
    // at the server, once, as BadNodeIdUnknown. Reject it where it is a
    // configuration error.
    if pinfo.linked_to_item && pinfo.identifier.is_none() {
        return err("link is missing the node identifier ('i=' or 's=')");
    }

    // The C's other consistency check — `monitor=y` on a structure element whose
    // item record is not monitored — needs the item, so it runs when the tree is
    // wired (`item::check_element_monitor`).
    Ok(())
}

fn set_element(pinfo: &mut LinkInfo, value: &str) {
    pinfo.element = value.to_string();
    pinfo.element_path = split_escaped(value, ELEMENT_DELIMITER);
}

fn parse_timestamp(pinfo: &mut LinkInfo, value: &str) -> Result<(), LinkError> {
    match value {
        "server" => pinfo.timestamp = TimestampSource::Server,
        "source" => pinfo.timestamp = TimestampSource::Source,
        "data" if !pinfo.is_item_record => pinfo.timestamp = TimestampSource::Data,
        _ if pinfo.is_item_record && value.starts_with('@') => {
            pinfo.timestamp = TimestampSource::Data;
            pinfo.timestamp_element = value[1..].to_string();
        }
        _ => return err(format!("illegal value '{value}'")),
    }
    Ok(())
}

fn parse_discard(value: &str) -> Result<bool, LinkError> {
    match value {
        "new" => Ok(false),
        "old" => Ok(true),
        _ => err(format!("illegal value '{value}'")),
    }
}

fn first_char(value: &str, name: &str) -> Result<char, LinkError> {
    value
        .chars()
        .next()
        .ok_or_else(|| LinkError(format!("no value for option '{name}'")))
}

/// `epicsParseUInt32`/`epicsParseUInt16` accept a `0x`/`0` base prefix (base 0).
fn parse_int_base0(value: &str) -> Option<u64> {
    let (negative, digits) = match value.strip_prefix('-') {
        Some(rest) => (true, rest.trim()),
        None => (false, value.strip_prefix('+').unwrap_or(value).trim()),
    };
    if negative {
        return None;
    }
    let parsed = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        u64::from_str_radix(hex, 16).ok()?
    } else if digits.len() > 1 && digits.starts_with('0') {
        u64::from_str_radix(&digits[1..], 8).ok()?
    } else {
        digits.parse().ok()?
    };
    Some(parsed)
}

fn parse_u32(value: &str) -> Result<u32, LinkError> {
    parse_int_base0(value)
        .and_then(|v| u32::try_from(v).ok())
        .ok_or_else(|| LinkError(format!("error converting '{value}' to UInt32")))
}

fn parse_u16(value: &str) -> Result<u16, LinkError> {
    parse_int_base0(value)
        .and_then(|v| u16::try_from(v).ok())
        .ok_or_else(|| LinkError(format!("error converting '{value}' to UInt16")))
}

fn parse_f64(value: &str) -> Result<f64, LinkError> {
    value
        .trim()
        .parse()
        .map_err(|_| LinkError(format!("error converting '{value}' to Double")))
}
