//! open62541's `UA_print` (`ua_types_print.c` in the 1.2 amalgamation the C
//! module builds with, `open62541.c:11247-11900`): the text the C shows for a
//! value it has no conversion of its own for.

use std::fmt::Write;

use async_opcua::types::{
    ByteString, ByteStringBody, DataValue, DateTime, DiagnosticInfo, ExpandedNodeId,
    ExtensionObject, UAString, UaNullable, Variant, VariantScalarTypeId, XmlBody,
};
use chrono::{Datelike, Timelike};
use epics_libcom_rs::cfmt::CFloat;

use crate::session::status_name;
use crate::value::type_name;

/// `UA_print` of a variant's value. `None` for a structure the session
/// decoded: `printStructure` walks the type's members, which the generated
/// `async-opcua` types do not expose.
pub fn ua_print(v: &Variant) -> Option<String> {
    let mut printer = Printer {
        out: String::new(),
        depth: 0,
        offset: chrono::Local::now().offset().local_minus_utc(),
    };
    printer.value(v)?;
    Some(printer.out)
}

struct Printer {
    out: String,
    depth: usize,
    /// `UA_DateTime_localTimeUtcOffset`, in seconds.
    offset: i32,
}

impl Printer {
    fn push(&mut self, s: &str) {
        self.out.push_str(s);
    }

    /// `UA_PrintContext_addNewlineTabs`.
    fn newline(&mut self) {
        self.out.push('\n');
        for _ in 0..self.depth {
            self.out.push('\t');
        }
    }

    /// The next member of a `{ ... }` block: `,` after the one before it, a
    /// new line, then `name: `.
    fn member(&mut self, first: &mut bool, name: &str) {
        if !std::mem::take(first) {
            self.push(",");
        }
        self.newline();
        self.push(name);
        self.push(": ");
    }

    fn open(&mut self, head: &str) {
        self.push(head);
        self.depth += 1;
    }

    fn close(&mut self) {
        self.depth -= 1;
        self.newline();
        self.push("}");
    }

    /// `printJumpTable[type->typeKind]`.
    fn value(&mut self, v: &Variant) -> Option<()> {
        match v {
            Variant::Empty => self.push("NullVariant"),
            Variant::Boolean(b) => self.push(if *b { "true" } else { "false" }),
            Variant::SByte(x) => self.push(&x.to_string()),
            Variant::Byte(x) => self.push(&x.to_string()),
            Variant::Int16(x) => self.push(&x.to_string()),
            Variant::UInt16(x) => self.push(&x.to_string()),
            Variant::Int32(x) => self.push(&x.to_string()),
            Variant::UInt32(x) => self.push(&x.to_string()),
            Variant::Int64(x) => self.push(&x.to_string()),
            Variant::UInt64(x) => self.push(&x.to_string()),
            Variant::Float(x) => self.push(&CFloat(f64::from(*x)).to_string()),
            Variant::Double(x) => self.push(&CFloat(*x).to_string()),
            Variant::String(s) => self.string(s),
            Variant::XmlElement(x) if x.is_ua_null() => self.push("NullString"),
            Variant::XmlElement(x) => self.quoted(&x.to_string()),
            Variant::DateTime(dt) => self.push(&date_time(dt, self.offset)),
            Variant::Guid(g) => self.push(&g.to_string()),
            Variant::StatusCode(s) => self.push(status_name(*s)),
            Variant::ByteString(b) => self.byte_string(b),
            // `UA_NodeId_print` and `NodeId`'s `Display` are the same text.
            Variant::NodeId(id) => self.push(&id.to_string()),
            Variant::ExpandedNodeId(id) => self.push(&expanded_node_id(id)),
            Variant::QualifiedName(qn) => {
                self.open("{");
                let first = &mut true;
                self.member(first, "NamespaceIndex");
                self.push(&qn.namespace_index.to_string());
                self.member(first, "Name");
                self.string(&qn.name);
                self.close();
            }
            Variant::LocalizedText(lt) => {
                self.open("{");
                let first = &mut true;
                self.member(first, "Locale");
                self.string(&lt.locale);
                self.member(first, "Text");
                self.string(&lt.text);
                self.close();
            }
            Variant::ExtensionObject(eo) => self.extension_object(eo)?,
            Variant::Variant(inner) => self.variant(inner)?,
            Variant::DataValue(dv) => self.data_value(dv)?,
            Variant::DiagnosticInfo(di) => self.diagnostic_info(di),
            Variant::Array(a) => self.array(&a.values, a.value_type)?,
        }
        Some(())
    }

    fn quoted(&mut self, s: &str) {
        self.push("\"");
        self.push(s);
        self.push("\"");
    }

    fn string(&mut self, s: &UAString) {
        match s.value() {
            Some(s) => self.quoted(s),
            None => self.push("NullString"),
        }
    }

    fn byte_string(&mut self, b: &ByteString) {
        if b.is_null() {
            self.push("NullByteString");
        } else {
            self.quoted(&b.as_base64());
        }
    }

    /// `printVariant`.
    fn variant(&mut self, v: &Variant) -> Option<()> {
        if matches!(v, Variant::Empty) {
            self.push("NullVariant");
            return Some(());
        }
        self.open("{");
        let first = &mut true;
        self.member(first, "DataType");
        self.push(&type_name(v));
        self.member(first, "Value");
        self.value(v)?;
        if let Variant::Array(a) = v
            && let Some(dims) = a.dimensions.as_ref().filter(|d| !d.is_empty())
        {
            self.member(first, "ArrayDimensions");
            let dims: Vec<Variant> = dims.iter().map(|d| Variant::UInt32(*d)).collect();
            self.array(&dims, VariantScalarTypeId::UInt32)?;
        }
        self.close();
        Some(())
    }

    /// `printArray`.
    fn array(&mut self, values: &[Variant], value_type: VariantScalarTypeId) -> Option<()> {
        self.open(&format!("Array({}, {value_type:?}) {{", values.len()));
        for (i, v) in values.iter().enumerate() {
            self.newline();
            self.push(&format!("{i}: "));
            self.value(v)?;
            if i + 1 < values.len() {
                self.push(",");
            }
        }
        self.close();
        Some(())
    }

    /// `printExtensionObject`. C closes an encoded body's block with
    /// `addName(ctx, "}")`, which prints "}: "; the stray ": " is not copied.
    fn extension_object(&mut self, eo: &ExtensionObject) -> Option<()> {
        if eo.is_null() {
            self.push("ExtensionObject(No Body)");
        } else if let Some(body) = eo.inner_as::<ByteStringBody>() {
            self.open("ExtensionObject(Binary Encoded) {");
            let first = &mut true;
            self.member(first, "DataType");
            self.push(&body.encoding_id().to_string());
            self.member(first, "Body");
            self.byte_string(body.raw_body());
            self.close();
        } else {
            // Any other body is a structure decoded into a generated type.
            let body = eo.inner_as::<XmlBody>()?;
            self.open("ExtensionObject(XML Encoded) {");
            let first = &mut true;
            self.member(first, "DataType");
            self.push(&body.encoding_id().to_string());
            self.member(first, "Body");
            self.quoted(&String::from_utf8_lossy(body.raw_body()));
            self.close();
        }
        Some(())
    }

    /// `printDataValue`: only the fields present, `{ }` for none.
    fn data_value(&mut self, dv: &DataValue) -> Option<()> {
        self.open("{");
        let first = &mut true;
        if let Some(v) = &dv.value {
            self.member(first, "Value");
            self.variant(v)?;
        }
        if let Some(s) = dv.status {
            self.member(first, "Status");
            self.push(status_name(s));
        }
        if let Some(t) = &dv.source_timestamp {
            self.member(first, "SourceTimestamp");
            self.push(&date_time(t, self.offset));
        }
        if let Some(ps) = dv.source_picoseconds {
            self.member(first, "SourcePicoseconds");
            self.push(&ps.to_string());
        }
        if let Some(t) = &dv.server_timestamp {
            self.member(first, "ServerTimestamp");
            self.push(&date_time(t, self.offset));
        }
        if let Some(ps) = dv.server_picoseconds {
            self.member(first, "ServerPicoseconds");
            self.push(&ps.to_string());
        }
        self.close_members(*first);
        Some(())
    }

    /// `printDiagnosticInfo`: only the fields present, `{ }` for none.
    fn diagnostic_info(&mut self, di: &DiagnosticInfo) {
        self.open("{");
        let first = &mut true;
        for (name, field) in [
            ("SymbolicId", di.symbolic_id),
            ("NamespaceUri", di.namespace_uri),
            ("LocalizedText", di.localized_text),
            ("Locale", di.locale),
        ] {
            if let Some(x) = field {
                self.member(first, name);
                self.push(&x.to_string());
            }
        }
        if let Some(s) = &di.additional_info {
            self.member(first, "AdditionalInfo");
            self.string(s);
        }
        if let Some(s) = di.inner_status_code {
            self.member(first, "InnerStatusCode");
            self.push(status_name(s));
        }
        if let Some(inner) = &di.inner_diagnostic_info {
            self.member(first, "InnerDiagnosticInfo");
            self.diagnostic_info(inner);
        }
        self.close_members(*first);
    }

    fn close_members(&mut self, empty: bool) {
        if empty {
            self.depth -= 1;
            self.push(" }");
        } else {
            self.close();
        }
    }
}

/// `printDateTime` of a time first moved into the local zone, as the C's
/// `readScalar` does (`DataElementOpen62541Leaf.cpp:393-400`). Two open62541
/// defects are not copied: it labels the zone `offset / 36`, which is HHMM
/// for whole hours only (+5:30 prints as +0550), and a time it prints nested
/// in a structure stays UTC under that local label.
fn date_time(dt: &DateTime, offset: i32) -> String {
    let local = dt.as_chrono() + chrono::Duration::seconds(offset.into());
    let mut s = String::new();
    let _ = write!(
        s,
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03} (UTC{}{:02}{:02})",
        local.year(),
        local.month(),
        local.day(),
        local.hour(),
        local.minute(),
        local.second(),
        local.nanosecond() / 1_000_000,
        if offset < 0 { '-' } else { '+' },
        offset.abs() / 3600,
        offset.abs() / 60 % 60,
    );
    s
}

/// `UA_ExpandedNodeId_print`: the server index unless 0, the namespace URI
/// when set (and then no namespace index), then the node id.
fn expanded_node_id(id: &ExpandedNodeId) -> String {
    let mut s = String::new();
    if id.server_index != 0 {
        let _ = write!(s, "svr={};", id.server_index);
    }
    let mut node_id = id.node_id.clone();
    if let Some(uri) = id.namespace_uri.value() {
        let _ = write!(s, "nsu={uri};");
        node_id.namespace = 0;
    }
    let _ = write!(s, "{node_id}");
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// open62541 1.2's `printDateTime` for the same time, read under
    /// TZ=Asia/Seoul and TZ=UTC; the half-hour zones are the defect C prints
    /// as +0550 and -0250.
    #[test]
    fn a_date_time_prints_in_the_zone_it_is_given() {
        let dt = DateTime::ymd_hms_nano(2026, 10, 20, 1, 2, 3, 456_789_100);
        assert_eq!(
            date_time(&dt, 9 * 3600),
            "2026-10-20 10:02:03.456 (UTC+0900)"
        );
        assert_eq!(date_time(&dt, 0), "2026-10-20 01:02:03.456 (UTC+0000)");
        assert_eq!(
            date_time(&dt, 5 * 3600 + 1800),
            "2026-10-20 06:32:03.456 (UTC+0530)"
        );
        assert_eq!(
            date_time(&dt, -(2 * 3600 + 1800)),
            "2026-10-19 22:32:03.456 (UTC-0230)"
        );
        let epoch = DateTime::ymd_hms_nano(1601, 1, 1, 0, 0, 0, 0);
        assert_eq!(
            date_time(&epoch, 9 * 3600),
            "1601-01-01 09:00:00.000 (UTC+0900)"
        );
    }
}
