//! Sum-up read (`ADSIGRP_SUMUP_READ`, 0xF080) — many variables in one ADS
//! round trip.
//!
//! The driver's poll loop batches every `POLL_RATE=`-tagged parameter of one AMS
//! port into a single READ_WRITE. The request writes a list of
//! `{indexGroup, indexOffset, size}` triples and sets `indexOffset` to the
//! number of sub-requests; the response is, per Beckhoff `AdsDef.h:71`,
//! *"{list of results} and {list of data}"*:
//!
//! ```text
//! result[0] u32 … result[cnt-1] u32     one ADS error code per sub-request
//! data[0] … data[cnt-1]                 the values
//! ```
//!
//! # Where a failed sub-request leaves the data area
//!
//! The reference header does not say whether a sub-request that *failed* still
//! occupies its requested bytes in the data area. Both layouts are self-
//! consistent:
//!
//! * **Fixed** — every sub-request occupies `size` bytes regardless of its
//!   result; failures leave a hole.
//! * **Compacted** — only successful sub-requests contribute bytes.
//!
//! The C driver assumes *compacted*: `bulkReadThread` (adsAsynPortDriver.cpp:674)
//! does `if (*stat++) continue;` — skipping a failed entry's status without
//! advancing the data pointer — while `adsAddToBulkRead` (:1394) sizes the read
//! as `cnt*4 + Σ(all sizes)`, i.e. as if the layout were *fixed*. Those two
//! cannot both be right, and nothing in the reference resolves it.
//!
//! So this decoder does not assume either: it reads the answer off the wire. The
//! two layouts predict different total data lengths whenever a sub-request
//! fails, and the PLC tells us the length it actually sent. When no sub-request
//! failed the two predictions coincide and the distinction is moot. If the
//! response matches neither, that is a protocol violation and we say so instead
//! of handing out misaligned slices.
//!
//! This is the one place the C driver silently corrupts data: under the *fixed*
//! layout, one failed variable shifts every later variable in the same batch by
//! its width, so an unrelated healthy record silently receives another
//! variable's bytes. Whichever layout a given PLC uses, this decoder is right.

use super::defs::ADSIGRP_SUMUP_READ;
use super::error::AdsError;
use super::frame::Reader;

/// Bytes of status that precede the data area, per sub-request.
const RESULT_LEN: usize = 4;

/// One sub-request of a sum-up read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SumEntry {
    pub index_group: u32,
    pub index_offset: u32,
    /// Bytes to read for this variable.
    pub size: u32,
}

/// A sum-up READ_WRITE request, ready for [`AdsClient::read_write`].
///
/// [`AdsClient::read_write`]: super::client::AdsClient::read_write
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SumUpRequest {
    pub index_group: u32,
    /// The sub-request count travels in `indexOffset` (`AdsDef.h:71`).
    pub index_offset: u32,
    /// Largest response we can receive: every result word plus every value.
    pub read_length: u32,
    /// The `{group, offset, size}` triples.
    pub payload: Vec<u8>,
}

/// Build the sum-up request for `entries`.
pub fn build_request(entries: &[SumEntry]) -> SumUpRequest {
    let mut payload = Vec::with_capacity(entries.len() * 12);
    for e in entries {
        payload.extend_from_slice(&e.index_group.to_le_bytes());
        payload.extend_from_slice(&e.index_offset.to_le_bytes());
        payload.extend_from_slice(&e.size.to_le_bytes());
    }
    SumUpRequest {
        index_group: ADSIGRP_SUMUP_READ,
        index_offset: entries.len() as u32,
        read_length: fixed_len(entries) as u32,
        payload,
    }
}

/// Total response length if every sub-request occupies its requested bytes.
fn fixed_len(entries: &[SumEntry]) -> usize {
    entries.len() * RESULT_LEN + entries.iter().map(|e| e.size as usize).sum::<usize>()
}

/// Which data-area layout the PLC actually used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataLayout {
    /// Failed sub-requests still occupy their requested bytes.
    Fixed,
    /// Only successful sub-requests contribute bytes.
    Compacted,
}

/// The decoded result of one sub-request: its bytes, or the ADS error the PLC
/// reported for it.
pub type SumResult<'a> = Result<&'a [u8], AdsError>;

/// Split a sum-up response into one result per sub-request.
///
/// `entries` must be the same slice passed to [`build_request`]. Returns the
/// per-entry results and the layout the PLC used (useful for logging; the
/// caller does not need to act on it).
pub fn decode_response<'a>(
    resp: &'a [u8],
    entries: &[SumEntry],
) -> Result<(Vec<SumResult<'a>>, DataLayout), AdsError> {
    let cnt = entries.len();
    let mut r = Reader::new(resp);

    let mut status = Vec::with_capacity(cnt);
    for _ in 0..cnt {
        status.push(r.u32()?);
    }
    let data = &resp[cnt * RESULT_LEN..];

    // Both layouts predict a data length; the bytes we were actually sent pick
    // one. They agree whenever nothing failed, so the common path is decided
    // either way.
    let fixed_data = fixed_len(entries) - cnt * RESULT_LEN;
    let compact_data: usize = entries
        .iter()
        .zip(&status)
        .filter(|(_, s)| **s == 0)
        .map(|(e, _)| e.size as usize)
        .sum();

    let layout = if data.len() == fixed_data {
        DataLayout::Fixed
    } else if data.len() == compact_data {
        DataLayout::Compacted
    } else {
        // Neither prediction holds: the PLC sent a data area we cannot index
        // safely. Refusing beats handing every later entry a misaligned slice.
        return Err(AdsError::ShortRead {
            need: fixed_data,
            got: data.len(),
        });
    };

    let mut out = Vec::with_capacity(cnt);
    let mut pos = 0usize;
    for (e, &s) in entries.iter().zip(&status) {
        let size = e.size as usize;
        if s != 0 {
            out.push(Err(AdsError::Ads(s)));
            // The hole is only there under the fixed layout.
            if layout == DataLayout::Fixed {
                pos += size;
            }
            continue;
        }
        let end = pos + size;
        if end > data.len() {
            return Err(AdsError::ShortRead {
                need: end,
                got: data.len(),
            });
        }
        out.push(Ok(&data[pos..end]));
        pos = end;
    }
    Ok((out, layout))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entries() -> Vec<SumEntry> {
        vec![
            SumEntry {
                index_group: 0xF005,
                index_offset: 1,
                size: 4,
            },
            SumEntry {
                index_group: 0xF005,
                index_offset: 2,
                size: 8,
            },
            SumEntry {
                index_group: 0xF005,
                index_offset: 3,
                size: 2,
            },
        ]
    }

    /// Build a response with the given per-entry statuses under `layout`.
    fn response(statuses: &[u32], layout: DataLayout) -> Vec<u8> {
        let es = entries();
        let mut out = Vec::new();
        for s in statuses {
            out.extend_from_slice(&s.to_le_bytes());
        }
        for (i, e) in es.iter().enumerate() {
            let ok = statuses[i] == 0;
            if ok || layout == DataLayout::Fixed {
                // Fill each entry's bytes with its 1-based index so a
                // misaligned decode is unmistakable.
                out.extend(std::iter::repeat_n((i + 1) as u8, e.size as usize));
            }
        }
        out
    }

    #[test]
    fn request_carries_the_count_in_index_offset() {
        let req = build_request(&entries());
        assert_eq!(req.index_group, ADSIGRP_SUMUP_READ);
        assert_eq!(req.index_offset, 3, "sub-request count");
        // 3 results + 4 + 8 + 2 bytes of data.
        assert_eq!(req.read_length, 3 * 4 + 14);
        assert_eq!(req.payload.len(), 3 * 12);

        let mut r = Reader::new(&req.payload);
        assert_eq!(r.u32().unwrap(), 0xF005);
        assert_eq!(r.u32().unwrap(), 1);
        assert_eq!(r.u32().unwrap(), 4);
        assert_eq!(r.u32().unwrap(), 0xF005);
        assert_eq!(r.u32().unwrap(), 2);
        assert_eq!(r.u32().unwrap(), 8);
    }

    #[test]
    fn empty_request_is_well_formed() {
        let req = build_request(&[]);
        assert_eq!(req.index_offset, 0);
        assert_eq!(req.read_length, 0);
        assert!(req.payload.is_empty());
    }

    #[test]
    fn all_successful_decodes_every_entry() {
        // With no failures the two layouts are byte-identical, so this is the
        // path that must work regardless of which one the PLC uses.
        for layout in [DataLayout::Fixed, DataLayout::Compacted] {
            let resp = response(&[0, 0, 0], layout);
            let (out, _) = decode_response(&resp, &entries()).unwrap();
            assert_eq!(out[0].as_ref().unwrap(), &[1u8, 1, 1, 1]);
            assert_eq!(out[1].as_ref().unwrap(), &[2u8; 8]);
            assert_eq!(out[2].as_ref().unwrap(), &[3u8, 3]);
        }
    }

    /// The C bug: under the fixed layout a failed entry leaves a hole, and C
    /// does not skip it, so entry 3 would be handed entry 2's bytes.
    #[test]
    fn a_failure_under_the_fixed_layout_does_not_shift_later_entries() {
        let resp = response(&[0, 0x0710, 0], DataLayout::Fixed);
        let (out, layout) = decode_response(&resp, &entries()).unwrap();
        assert_eq!(layout, DataLayout::Fixed);
        assert_eq!(out[0].as_ref().unwrap(), &[1u8; 4]);
        assert_eq!(out[1].as_ref().unwrap_err().code(), Some(0x0710));
        assert_eq!(
            out[2].as_ref().unwrap(),
            &[3u8, 3],
            "entry 3 must get its own bytes, not the tail of entry 2's hole"
        );
    }

    #[test]
    fn a_failure_under_the_compacted_layout_also_decodes_correctly() {
        let resp = response(&[0, 0x0710, 0], DataLayout::Compacted);
        let (out, layout) = decode_response(&resp, &entries()).unwrap();
        assert_eq!(layout, DataLayout::Compacted);
        assert_eq!(out[0].as_ref().unwrap(), &[1u8; 4]);
        assert_eq!(out[1].as_ref().unwrap_err().code(), Some(0x0710));
        assert_eq!(out[2].as_ref().unwrap(), &[3u8, 3]);
    }

    #[test]
    fn the_layout_is_read_off_the_wire_not_assumed() {
        // Same statuses, two different byte counts — each must be recognized.
        let es = entries();
        let fixed = response(&[0x0710, 0, 0], DataLayout::Fixed);
        let compact = response(&[0x0710, 0, 0], DataLayout::Compacted);
        assert_ne!(fixed.len(), compact.len());

        assert_eq!(decode_response(&fixed, &es).unwrap().1, DataLayout::Fixed);
        assert_eq!(
            decode_response(&compact, &es).unwrap().1,
            DataLayout::Compacted
        );
        // And both agree on the surviving entries' bytes.
        for resp in [fixed, compact] {
            let (out, _) = decode_response(&resp, &es).unwrap();
            assert_eq!(out[1].as_ref().unwrap(), &[2u8; 8]);
            assert_eq!(out[2].as_ref().unwrap(), &[3u8, 3]);
        }
    }

    #[test]
    fn every_entry_failing_yields_every_error() {
        let resp = response(&[1, 2, 3], DataLayout::Compacted);
        let (out, _) = decode_response(&resp, &entries()).unwrap();
        let codes: Vec<_> = out.iter().map(|r| r.as_ref().unwrap_err().code()).collect();
        assert_eq!(codes, vec![Some(1), Some(2), Some(3)]);
    }

    #[test]
    fn a_response_matching_neither_layout_is_rejected() {
        // One byte short of both predictions: indexing it would misalign.
        let mut resp = response(&[0, 0x0710, 0], DataLayout::Fixed);
        resp.pop();
        assert!(matches!(
            decode_response(&resp, &entries()),
            Err(AdsError::ShortRead { .. })
        ));
    }

    #[test]
    fn a_response_too_short_for_the_status_words_is_rejected() {
        assert!(matches!(
            decode_response(&[0, 0, 0, 0], &entries()),
            Err(AdsError::ShortFrame { .. })
        ));
    }
}
