//! Device-notification frame decode.
//!
//! A `DEVICE_NOTIFICATION` payload (`AdsLib/standalone/NotificationDispatcher.cpp`)
//! is a stamp list, each stamp carrying a sample list:
//!
//! ```text
//! length     u32           (bytes that follow, excluding this field)
//! numStamps  u32
//!   repeat numStamps:
//!     timestamp  u64       (Windows FILETIME, 100 ns ticks since 1601-01-01)
//!     numSamples u32
//!       repeat numSamples:
//!         hNotify u32
//!         size    u32
//!         data    [size]
//! ```

use super::error::AdsError;
use super::frame::Reader;

/// One notified value: a handle, its raw little-endian bytes, and the PLC-side
/// timestamp of the stamp it arrived in.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NotificationSample {
    pub handle: u32,
    /// Windows FILETIME as delivered by the PLC (100 ns ticks since 1601).
    pub timestamp: u64,
    pub data: Vec<u8>,
}

/// Decode a `DEVICE_NOTIFICATION` payload into a flat sample list.
///
/// Stamps only group samples by timestamp; the C dispatcher likewise flattens
/// them, dispatching each sample to its handle's callback with the stamp's time.
pub fn decode_notification(payload: &[u8]) -> Result<Vec<NotificationSample>, AdsError> {
    let mut r = Reader::new(payload);
    let _length = r.u32()?;
    let num_stamps = r.u32()?;

    let mut out = Vec::new();
    for _ in 0..num_stamps {
        let timestamp = r.u64()?;
        let num_samples = r.u32()?;
        for _ in 0..num_samples {
            let handle = r.u32()?;
            let size = r.u32()? as usize;
            let data = r.bytes(size)?.to_vec();
            out.push(NotificationSample {
                handle,
                timestamp,
                data,
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `(handle, data)` — one notified sample as the fake PLC encodes it.
    type Sample<'a> = (u32, &'a [u8]);
    /// `(timestamp, samples)` — one stamp.
    type Stamp<'a> = (u64, &'a [Sample<'a>]);

    fn build(stamps: &[Stamp<'_>]) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&(stamps.len() as u32).to_le_bytes());
        for (ts, samples) in stamps {
            body.extend_from_slice(&ts.to_le_bytes());
            body.extend_from_slice(&(samples.len() as u32).to_le_bytes());
            for (h, d) in *samples {
                body.extend_from_slice(&h.to_le_bytes());
                body.extend_from_slice(&(d.len() as u32).to_le_bytes());
                body.extend_from_slice(d);
            }
        }
        let mut out = (body.len() as u32).to_le_bytes().to_vec();
        out.extend_from_slice(&body);
        out
    }

    #[test]
    fn decodes_single_stamp_single_sample() {
        let buf = build(&[(0x01D9_ABCD_0000_0000, &[(7, &42i32.to_le_bytes())])]);
        let got = decode_notification(&buf).unwrap();
        assert_eq!(
            got,
            vec![NotificationSample {
                handle: 7,
                timestamp: 0x01D9_ABCD_0000_0000,
                data: 42i32.to_le_bytes().to_vec(),
            }]
        );
    }

    #[test]
    fn flattens_multiple_stamps_and_samples_keeping_per_stamp_time() {
        let buf = build(&[
            (100, &[(1, &[0xAA]), (2, &[0xBB, 0xCC])]),
            (200, &[(3, &[])]),
        ]);
        let got = decode_notification(&buf).unwrap();
        assert_eq!(got.len(), 3);
        assert_eq!((got[0].handle, got[0].timestamp), (1, 100));
        assert_eq!(got[0].data, vec![0xAA]);
        assert_eq!((got[1].handle, got[1].timestamp), (2, 100));
        assert_eq!(got[1].data, vec![0xBB, 0xCC]);
        // Zero-length sample: legal, and must not be mistaken for end-of-frame.
        assert_eq!((got[2].handle, got[2].timestamp), (3, 200));
        assert!(got[2].data.is_empty());
    }

    #[test]
    fn zero_stamps_yields_no_samples() {
        let buf = build(&[]);
        assert!(decode_notification(&buf).unwrap().is_empty());
    }

    #[test]
    fn truncated_payload_errors() {
        let mut buf = build(&[(1, &[(9, &[1, 2, 3, 4])])]);
        buf.truncate(buf.len() - 2);
        assert!(matches!(
            decode_notification(&buf),
            Err(AdsError::ShortFrame { .. })
        ));
    }

    #[test]
    fn oversized_sample_length_errors_instead_of_allocating() {
        // A hostile/corrupt frame claiming a 4 GiB sample must be rejected by the
        // bounds check, not turned into a giant allocation.
        let mut buf = Vec::new();
        let mut body = Vec::new();
        body.extend_from_slice(&1u32.to_le_bytes()); // numStamps
        body.extend_from_slice(&0u64.to_le_bytes()); // timestamp
        body.extend_from_slice(&1u32.to_le_bytes()); // numSamples
        body.extend_from_slice(&1u32.to_le_bytes()); // hNotify
        body.extend_from_slice(&u32::MAX.to_le_bytes()); // size
        buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
        buf.extend_from_slice(&body);
        assert!(matches!(
            decode_notification(&buf),
            Err(AdsError::ShortFrame { .. })
        ));
    }
}
