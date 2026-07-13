//! `AdsSymbolEntry` decode — the reply to `ADSIGRP_SYM_INFOBYNAMEEX`.
//!
//! Layout (`AdsLib/standalone/AdsDef.h`), packed little-endian, followed by the
//! three NUL-terminated strings whose lengths the header announces:
//!
//! ```text
//! entryLength   u32   (total bytes of this entry, header + strings + padding)
//! iGroup        u32
//! iOffs         u32
//! size          u32   (bytes the symbol occupies in the PLC)
//! dataType      u32   (ADSDATATYPEID)
//! flags         u32
//! nameLength    u16   (excluding the NUL)
//! typeLength    u16
//! commentLength u16
//! name    [nameLength]    '\0'
//! type    [typeLength]    '\0'
//! comment [commentLength] '\0'
//! ```

use super::defs::AdsType;
use super::error::AdsError;
use super::frame::Reader;

/// Fixed part of an `AdsSymbolEntry`.
pub const SYMBOL_ENTRY_HEADER_LEN: usize = 30;

/// A PLC symbol as reported by `SYM_INFOBYNAMEEX`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SymbolEntry {
    pub index_group: u32,
    pub index_offset: u32,
    /// Total size of the symbol in bytes (element size × element count).
    pub size: u32,
    pub data_type: AdsType,
    pub flags: u32,
    pub name: String,
    pub type_name: String,
    pub comment: String,
}

impl SymbolEntry {
    /// Number of elements, derived as `size / element_size`.
    ///
    /// `None` when the type has no fixed element size (`ADST_BIGTYPE`,
    /// `ADST_WSTRING`, unknown ids) — the C driver's `adsTypeSize` returns
    /// `(size_t)-1` there and callers must fall back to the raw byte count.
    pub fn element_count(&self) -> Option<usize> {
        match self.data_type.element_size() {
            Some(0) | None => None,
            Some(n) => Some(self.size as usize / n),
        }
    }
}

/// Decode one `AdsSymbolEntry` from the start of `buf`.
pub fn decode_symbol_entry(buf: &[u8]) -> Result<SymbolEntry, AdsError> {
    let mut r = Reader::new(buf);
    let _entry_length = r.u32()?;
    let index_group = r.u32()?;
    let index_offset = r.u32()?;
    let size = r.u32()?;
    let data_type = AdsType::from_u32(r.u32()?);
    let flags = r.u32()?;
    let name_len = r.u16()? as usize;
    let type_len = r.u16()? as usize;
    let comment_len = r.u16()? as usize;

    // The announced lengths exclude the NUL; read the bytes then consume it,
    // rather than scanning for a NUL that may legally appear inside a comment.
    let name = read_string(&mut r, name_len)?;
    let type_name = read_string(&mut r, type_len)?;
    let comment = read_string(&mut r, comment_len)?;

    Ok(SymbolEntry {
        index_group,
        index_offset,
        size,
        data_type,
        flags,
        name,
        type_name,
        comment,
    })
}

fn read_string(r: &mut Reader<'_>, len: usize) -> Result<String, AdsError> {
    let s = String::from_utf8_lossy(r.bytes(len)?).into_owned();
    r.skip(1)?; // trailing NUL
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a `SymbolEntry` the way a PLC would, so the tests exercise the
    /// decoder against the same layout they assert on.
    fn build(e: &SymbolEntry) -> Vec<u8> {
        let (name, ty, comment) = (e.name.as_str(), e.type_name.as_str(), e.comment.as_str());
        let entry_len =
            (SYMBOL_ENTRY_HEADER_LEN + name.len() + ty.len() + comment.len() + 3) as u32;
        let mut b = Vec::new();
        b.extend_from_slice(&entry_len.to_le_bytes());
        b.extend_from_slice(&e.index_group.to_le_bytes());
        b.extend_from_slice(&e.index_offset.to_le_bytes());
        b.extend_from_slice(&e.size.to_le_bytes());
        b.extend_from_slice(&e.data_type.to_u32().to_le_bytes());
        b.extend_from_slice(&e.flags.to_le_bytes());
        b.extend_from_slice(&(name.len() as u16).to_le_bytes());
        b.extend_from_slice(&(ty.len() as u16).to_le_bytes());
        b.extend_from_slice(&(comment.len() as u16).to_le_bytes());
        b.extend_from_slice(name.as_bytes());
        b.push(0);
        b.extend_from_slice(ty.as_bytes());
        b.push(0);
        b.extend_from_slice(comment.as_bytes());
        b.push(0);
        b
    }

    /// A symbol with the given type/size and placeholder text fields.
    fn sym(
        index_group: u32,
        index_offset: u32,
        size: u32,
        data_type: AdsType,
        name: &str,
        type_name: &str,
    ) -> SymbolEntry {
        SymbolEntry {
            index_group,
            index_offset,
            size,
            data_type,
            flags: 0,
            name: name.into(),
            type_name: type_name.into(),
            comment: String::new(),
        }
    }

    #[test]
    fn header_is_30_bytes() {
        let buf = build(&sym(0x4020, 0, 4, AdsType::Int32, "", ""));
        assert_eq!(buf.len(), SYMBOL_ENTRY_HEADER_LEN + 3);
    }

    #[test]
    fn decodes_scalar_symbol() {
        let mut e = sym(0x4020, 0x1234, 4, AdsType::Real32, "MAIN.fTest", "REAL");
        e.flags = 0x08;
        e.comment = "test var".into();
        let decoded = decode_symbol_entry(&build(&e)).unwrap();
        assert_eq!(decoded, e);
        assert_eq!(decoded.element_count(), Some(1));
    }

    #[test]
    fn element_count_divides_size_by_element_size() {
        // ARRAY[0..9] OF INT — 10 elements × 2 bytes.
        let buf = build(&sym(
            0x4020,
            0,
            20,
            AdsType::Int16,
            "MAIN.arr",
            "ARRAY [0..9] OF INT",
        ));
        assert_eq!(decode_symbol_entry(&buf).unwrap().element_count(), Some(10));
    }

    #[test]
    fn element_count_is_none_for_types_without_fixed_element_size() {
        // ADST_BIGTYPE (a STRUCT): C `adsTypeSize` returns (size_t)-1.
        let buf = build(&sym(
            0x4020,
            0,
            64,
            AdsType::BigType,
            "MAIN.stStatus",
            "DUT_AxisStatus",
        ));
        assert_eq!(decode_symbol_entry(&buf).unwrap().element_count(), None);
    }

    #[test]
    fn string_symbol_counts_bytes_as_elements() {
        // STRING[80] occupies 81 bytes; ADST_STRING has element size 1.
        let buf = build(&sym(
            0x4020,
            0,
            81,
            AdsType::String,
            "MAIN.sName",
            "STRING(80)",
        ));
        assert_eq!(decode_symbol_entry(&buf).unwrap().element_count(), Some(81));
    }

    #[test]
    fn empty_comment_still_consumes_its_nul() {
        let buf = build(&sym(1, 2, 4, AdsType::Int32, "a", "DINT"));
        let s = decode_symbol_entry(&buf).unwrap();
        assert_eq!(s.comment, "");
        assert_eq!(s.name, "a");
        assert_eq!(s.type_name, "DINT");
    }

    #[test]
    fn truncated_entry_errors() {
        let mut buf = build(&sym(1, 2, 4, AdsType::Int32, "MAIN.x", "DINT"));
        buf.truncate(SYMBOL_ENTRY_HEADER_LEN + 2);
        assert!(matches!(
            decode_symbol_entry(&buf),
            Err(AdsError::ShortFrame { .. })
        ));
    }
}
