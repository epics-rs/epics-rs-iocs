//! Bitshuffle + LZ4 (`bslz4`) decoding.
//!
//! The detector emits bslz4 in two places and the C driver reaches for a
//! different library in each: `bshuf_decompress_lz4` for the ZeroMQ stream
//! (streamApi.cpp:139) and the HDF5 `bslz4` filter for the FileWriter files.
//! Both carry the same payload — a 12-byte header followed by LZ4-compressed
//! bit-transposed blocks — so this port decodes them with one function.
//!
//! Header (big-endian, as written by the bitshuffle library):
//!   * `u64` total uncompressed size in bytes
//!   * `u32` block size in bytes (0 = "the encoder used the default")
//!
//! Each block is framed `[u32 compressed_len BE][lz4 block]`; the trailing
//! `n_elems % 8` elements are stored raw.
//!
//! The C stream path throws the header away (`pInput += 12`) and passes
//! `block_size = 0`, so it silently assumes the encoder used the default block
//! size. This port reads the size the encoder recorded instead, which decodes
//! the same stream and additionally decodes one written with a non-default
//! block size.

/// Length of the bslz4 header, in bytes (C `pInput += 12`, streamApi.cpp:127).
pub const HEADER_LEN: usize = 12;

/// Bitshuffle target block size in bytes (library `BSHUF_TARGET_BLOCK_SIZE_B`).
const TARGET_BLOCK_SIZE_B: usize = 8192;
/// A block's element count is always a multiple of this (`BSHUF_BLOCKED_MULT`).
const BLOCKED_MULT: usize = 8;
/// Recommended minimum block size in elements (`BSHUF_MIN_RECOMMEND_BLOCK`).
const MIN_RECOMMEND_BLOCK: usize = 128;

#[derive(Debug, PartialEq)]
pub enum Bslz4Error {
    Truncated(&'static str),
    /// The header's uncompressed size disagrees with the frame geometry the
    /// image header declared.
    SizeMismatch {
        header: usize,
        expected: usize,
    },
    Lz4(String),
}

impl std::fmt::Display for Bslz4Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Truncated(what) => write!(f, "bslz4 payload truncated: {what}"),
            Self::SizeMismatch { header, expected } => write!(
                f,
                "bslz4 header declares {header} bytes, frame geometry needs {expected}"
            ),
            Self::Lz4(m) => write!(f, "bslz4 lz4 block: {m}"),
        }
    }
}

impl std::error::Error for Bslz4Error {}

/// Default block size in elements (library `bshuf_default_block_size`).
pub fn default_block_size(elem_size: usize) -> usize {
    let bs = TARGET_BLOCK_SIZE_B / elem_size.max(1);
    let bs = (bs / BLOCKED_MULT) * BLOCKED_MULT;
    bs.max(MIN_RECOMMEND_BLOCK)
}

/// 8x8 bit-matrix transpose of a quadword (library macro `TRANS_BIT_8X8`).
#[inline]
fn trans_bit_8x8(mut x: u64) -> u64 {
    let t = (x ^ (x >> 7)) & 0x00AA_00AA_00AA_00AA;
    x = x ^ t ^ (t << 7);
    let t = (x ^ (x >> 14)) & 0x0000_CCCC_0000_CCCC;
    x = x ^ t ^ (t << 14);
    let t = (x ^ (x >> 28)) & 0x0000_0000_F0F0_F0F0;
    x = x ^ t ^ (t << 28);
    x
}

#[inline]
fn read_u64_le(b: &[u8], off: usize) -> u64 {
    u64::from_le_bytes(b[off..off + 8].try_into().unwrap())
}

/// Transpose bytes for data organized as one row per bit
/// (library `bshuf_trans_byte_bitrow_scal`, bitshuffle_core.c:306).
fn trans_byte_bitrow(input: &[u8], out: &mut [u8], size: usize, elem_size: usize) {
    let nbyte_row = size / 8;
    for jj in 0..elem_size {
        for ii in 0..nbyte_row {
            for kk in 0..8 {
                out[ii * 8 * elem_size + jj * 8 + kk] = input[(jj * 8 + kk) * nbyte_row + ii];
            }
        }
    }
}

/// Shuffle bits within the bytes of eight-element groups
/// (library `bshuf_shuffle_bit_eightelem_scal`, bitshuffle_core.c:331, LE path).
fn shuffle_bit_eightelem(input: &[u8], out: &mut [u8], size: usize, elem_size: usize) {
    let nbyte = elem_size * size;
    let mut jj = 0;
    while jj < 8 * elem_size {
        let mut ii = 0;
        while ii + 8 * elem_size - 1 < nbyte {
            let mut x = trans_bit_8x8(read_u64_le(input, ii + jj));
            for kk in 0..8 {
                out[ii + jj / 8 + kk * elem_size] = x as u8;
                x >>= 8;
            }
            ii += 8 * elem_size;
        }
        jj += 8;
    }
}

/// Undo the bitshuffle transform of one block (`bshuf_untrans_bit_elem_scal`).
///
/// `size` (element count) is a multiple of 8.
fn untrans_bit_elem(input: &[u8], size: usize, elem_size: usize) -> Vec<u8> {
    let n = size * elem_size;
    let mut tmp = vec![0u8; n];
    let mut out = vec![0u8; n];
    trans_byte_bitrow(input, &mut tmp, size, elem_size);
    shuffle_bit_eightelem(&tmp, &mut out, size, elem_size);
    out
}

/// LZ4-decode and un-bitshuffle one `[u32 len BE][lz4]` block.
///
/// Returns the block's bytes and the offset just past the frame.
fn decode_block(
    buf: &[u8],
    pos: usize,
    size: usize,
    elem_size: usize,
) -> Result<(Vec<u8>, usize), Bslz4Error> {
    if pos + 4 > buf.len() {
        return Err(Bslz4Error::Truncated("block length"));
    }
    let clen = u32::from_be_bytes(buf[pos..pos + 4].try_into().unwrap()) as usize;
    let start = pos + 4;
    let end = start
        .checked_add(clen)
        .ok_or(Bslz4Error::Truncated("block length overflows"))?;
    if end > buf.len() {
        return Err(Bslz4Error::Truncated("block payload"));
    }
    let shuffled = lz4_flex::block::decompress(&buf[start..end], size * elem_size)
        .map_err(|e| Bslz4Error::Lz4(e.to_string()))?;
    if shuffled.len() != size * elem_size {
        return Err(Bslz4Error::Truncated("block decoded short"));
    }
    Ok((untrans_bit_elem(&shuffled, size, elem_size), end))
}

/// Decode a header-less bslz4 block stream (library `bshuf_decompress_lz4`).
///
/// `block_size` is in elements; pass 0 for the encoder default.
pub fn decode_blocks(
    buf: &[u8],
    n_elems: usize,
    elem_size: usize,
    block_size: usize,
) -> Result<Vec<u8>, Bslz4Error> {
    let block_size = if block_size == 0 {
        default_block_size(elem_size)
    } else {
        block_size
    };

    let mut out = Vec::with_capacity(n_elems * elem_size);
    let mut pos = 0usize;

    for _ in 0..(n_elems / block_size) {
        let (block, next) = decode_block(buf, pos, block_size, elem_size)?;
        out.extend_from_slice(&block);
        pos = next;
    }
    // One trailing partial block, rounded down to a multiple of 8.
    let last = n_elems % block_size;
    let last = last - last % BLOCKED_MULT;
    if last > 0 {
        let (block, next) = decode_block(buf, pos, last, elem_size)?;
        out.extend_from_slice(&block);
        pos = next;
    }
    // The final `n_elems % 8` elements were stored raw.
    let leftover = (n_elems % BLOCKED_MULT) * elem_size;
    if leftover > 0 {
        if pos + leftover > buf.len() {
            return Err(Bslz4Error::Truncated("trailing raw elements"));
        }
        out.extend_from_slice(&buf[pos..pos + leftover]);
    }
    Ok(out)
}

/// Decode a bslz4 payload that carries the 12-byte header.
///
/// `expected_bytes`, when given, is checked against the header's declared size:
/// the image header and the compressed payload must agree on the frame size
/// before we allocate from it.
pub fn decode_with_header(
    buf: &[u8],
    elem_size: usize,
    expected_bytes: Option<usize>,
) -> Result<Vec<u8>, Bslz4Error> {
    if buf.len() < HEADER_LEN {
        return Err(Bslz4Error::Truncated("header"));
    }
    let total_bytes = u64::from_be_bytes(buf[0..8].try_into().unwrap()) as usize;
    let block_bytes = u32::from_be_bytes(buf[8..12].try_into().unwrap()) as usize;

    if let Some(expected) = expected_bytes
        && total_bytes != expected
    {
        return Err(Bslz4Error::SizeMismatch {
            header: total_bytes,
            expected,
        });
    }

    let elem_size = elem_size.max(1);
    let n_elems = total_bytes / elem_size;
    // The header records the block size in bytes; the codec works in elements.
    let block_size = block_bytes / elem_size;
    decode_blocks(&buf[HEADER_LEN..], n_elems, elem_size, block_size)
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::codec::{Codec, CodecName};
    use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDataType, NDDimension};

    /// Encode with ad-plugins' independent bitshuffle implementation (written
    /// from the reference library) and decode with ours: the two must agree.
    /// `compress_bslz4` emits the header-less default-block stream, which is
    /// exactly what the C stream path feeds `bshuf_decompress_lz4`.
    fn roundtrip_via_ad_plugins(values: Vec<u16>) {
        let n = values.len();
        let src = NDArray {
            unique_id: 0,
            timestamp: Default::default(),
            time_stamp: 0.0,
            dims: vec![NDDimension::new(n)],
            data_size: n * 2,
            pool_id: 0,
            data: NDDataBuffer::U16(values.clone()),
            attributes: Default::default(),
            codec: None,
        };
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);
        let payload = compressed.data.as_u8_slice();

        let decoded = decode_blocks(payload, n, 2, 0).expect("decode");
        let decoded: Vec<u16> = decoded
            .chunks_exact(2)
            .map(|c| u16::from_ne_bytes([c[0], c[1]]))
            .collect();
        assert_eq!(decoded, values);
    }

    #[test]
    fn decodes_a_single_partial_block() {
        // Fewer elements than one block: exercises the trailing-partial path.
        roundtrip_via_ad_plugins((0..1000u16).collect());
    }

    #[test]
    fn decodes_multiple_full_blocks_plus_remainder() {
        // u16 default block = 4096 elements; 10000 = 2 full + partial + raw tail.
        roundtrip_via_ad_plugins((0..10_000u16).map(|i| i.wrapping_mul(7)).collect());
    }

    #[test]
    fn decodes_an_exact_multiple_of_the_block_size() {
        let block = default_block_size(2);
        roundtrip_via_ad_plugins((0..(block * 2) as u16).collect());
    }

    #[test]
    fn decodes_a_size_with_a_raw_tail() {
        // n % 8 != 0 leaves elements stored verbatim after the last block.
        roundtrip_via_ad_plugins((0..133u16).collect());
    }

    #[test]
    fn agrees_with_ad_plugins_decoder_on_the_ndarray_path() {
        // The pass-through path hands ad-plugins a header-stripped BSLZ4 array;
        // check our decoder and theirs produce the same pixels from it.
        let values: Vec<u32> = (0..5000u32).map(|i| i * 3).collect();
        let n = values.len();
        let src = NDArray {
            unique_id: 0,
            timestamp: Default::default(),
            time_stamp: 0.0,
            dims: vec![NDDimension::new(n)],
            data_size: n * 4,
            pool_id: 0,
            data: NDDataBuffer::U32(values.clone()),
            attributes: Default::default(),
            codec: None,
        };
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);
        let ours = decode_blocks(compressed.data.as_u8_slice(), n, 4, 0).unwrap();

        let theirs = ad_plugins_rs::codec::decompress_bslz4(&compressed).unwrap();
        let NDDataBuffer::U32(theirs) = theirs.data else {
            panic!("expected U32");
        };
        let ours: Vec<u32> = ours
            .chunks_exact(4)
            .map(|c| u32::from_ne_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        assert_eq!(ours, values);
        assert_eq!(theirs, values);
    }

    #[test]
    fn header_is_parsed_and_cross_checked() {
        let values: Vec<u16> = (0..2000).collect();
        let n = values.len();
        let src = NDArray {
            unique_id: 0,
            timestamp: Default::default(),
            time_stamp: 0.0,
            dims: vec![NDDimension::new(n)],
            data_size: n * 2,
            pool_id: 0,
            data: NDDataBuffer::U16(values.clone()),
            attributes: Default::default(),
            codec: Some(Codec {
                name: CodecName::BSLZ4,
                compressed_size: 0,
                level: 0,
                shuffle: 0,
                compressor: 0,
                original_data_type: NDDataType::UInt16,
            }),
        };
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);

        // Re-frame with the 12-byte header the detector prepends.
        let mut framed = Vec::new();
        framed.extend_from_slice(&((n * 2) as u64).to_be_bytes());
        framed.extend_from_slice(&((default_block_size(2) * 2) as u32).to_be_bytes());
        framed.extend_from_slice(compressed.data.as_u8_slice());

        let decoded = decode_with_header(&framed, 2, Some(n * 2)).unwrap();
        let decoded: Vec<u16> = decoded
            .chunks_exact(2)
            .map(|c| u16::from_ne_bytes([c[0], c[1]]))
            .collect();
        assert_eq!(decoded, values);

        // A header that disagrees with the declared frame geometry is rejected
        // rather than used to size an allocation.
        assert_eq!(
            decode_with_header(&framed, 2, Some(n * 2 + 8)),
            Err(Bslz4Error::SizeMismatch {
                header: n * 2,
                expected: n * 2 + 8,
            })
        );
    }

    #[test]
    fn zero_block_size_in_header_means_default() {
        let values: Vec<u16> = (0..3000).collect();
        let n = values.len();
        let src = NDArray {
            unique_id: 0,
            timestamp: Default::default(),
            time_stamp: 0.0,
            dims: vec![NDDimension::new(n)],
            data_size: n * 2,
            pool_id: 0,
            data: NDDataBuffer::U16(values.clone()),
            attributes: Default::default(),
            codec: None,
        };
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);

        let mut framed = Vec::new();
        framed.extend_from_slice(&((n * 2) as u64).to_be_bytes());
        framed.extend_from_slice(&0u32.to_be_bytes()); // 0 = default
        framed.extend_from_slice(compressed.data.as_u8_slice());

        let decoded = decode_with_header(&framed, 2, None).unwrap();
        assert_eq!(decoded.len(), n * 2);
    }

    #[test]
    fn truncated_payloads_are_rejected() {
        assert_eq!(
            decode_with_header(&[0u8; 4], 2, None),
            Err(Bslz4Error::Truncated("header"))
        );
        // Header promises 1024 bytes but no blocks follow.
        let mut framed = Vec::new();
        framed.extend_from_slice(&1024u64.to_be_bytes());
        framed.extend_from_slice(&0u32.to_be_bytes());
        assert!(decode_with_header(&framed, 2, None).is_err());
    }

    #[test]
    fn default_block_size_matches_the_library() {
        // 8192 / elem_size, floored to a multiple of 8, min 128.
        assert_eq!(default_block_size(1), 8192);
        assert_eq!(default_block_size(2), 4096);
        assert_eq!(default_block_size(4), 2048);
        assert_eq!(default_block_size(8), 1024);
    }
}
