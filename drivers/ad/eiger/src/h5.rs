//! FileWriter HDF5 parse (port of `eigerDetector::parseH5File`,
//! eigerDetector.cpp:1680).
//!
//! The C driver opens the downloaded `_data_*.h5` blob straight out of memory
//! with `H5LTopen_file_image` and reads `/entry/data/data` one hyperslab at a
//! time. This port does the same with `hdf5-reader` (pure Rust, read-only),
//! plus a locally-registered bitshuffle/LZ4 filter — HDF5 filter id 32008 —
//! which is what libhdf5 loads out of `HDF5_PLUGIN_PATH` in the C build. No
//! plugin path, no `libhdf5`, no `libbshuf` is needed here.

use epics_rs::ad_core::ndarray::{NDDataBuffer, NDDataType};
use hdf5_reader::error::{Error as H5Error, Result as H5Result};
use hdf5_reader::messages::filter_pipeline::FilterDescription;
use hdf5_reader::{Datatype, FilterRegistry, Hdf5File, OpenOptions, SliceInfo, SliceInfoElem};

use crate::bslz4;

/// The bitshuffle+LZ4 HDF5 filter id, as registered by Dectris' `bslz4` plugin.
pub const FILTER_BSLZ4: u16 = 32008;

/// The two dataset paths the C driver tries, in order.
const DATA_PATHS: [&str; 2] = ["/entry/data/data", "/entry/data"];

/// How one `/entry/data/data` dataset is laid out.
///
/// C accepts exactly 3-D `(nImages, height, width)` and 4-D
/// `(nImages, nThresholds, height, width)`.
#[derive(Debug, PartialEq)]
pub struct Layout {
    pub n_images: usize,
    pub n_thresh: usize,
    pub height: usize,
    pub width: usize,
}

/// Classify a dataset's shape (C, eigerDetector.cpp:1733-1765).
pub fn layout_of(dims: &[u64]) -> Result<Layout, String> {
    let d: Vec<usize> = dims.iter().map(|&v| v as usize).collect();
    match d.as_slice() {
        [n, h, w] => Ok(Layout {
            n_images: *n,
            n_thresh: 1,
            height: *h,
            width: *w,
        }),
        [n, t, h, w] => Ok(Layout {
            n_images: *n,
            n_thresh: *t,
            height: *h,
            width: *w,
        }),
        _ => Err(format!(
            "number of dimensions must be 3 or 4, got {}",
            d.len()
        )),
    }
}

/// Map the dataset's element type to an NDArray type.
///
/// The detector writes unsigned data. Bad pixels and gaps are very large
/// positive numbers, which makes autoscaling awkward, so `signed_data`
/// reinterprets the same bytes as signed (C, eigerDetector.cpp:1782-1794).
pub fn data_type_of(dtype: &Datatype, signed_data: bool) -> Result<NDDataType, String> {
    let Datatype::FixedPoint { size, .. } = dtype else {
        return Err(format!("invalid data type {dtype:?}"));
    };
    Ok(match (size, signed_data) {
        (1, false) => NDDataType::UInt8,
        (1, true) => NDDataType::Int8,
        (2, false) => NDDataType::UInt16,
        (2, true) => NDDataType::Int16,
        (4, false) => NDDataType::UInt32,
        (4, true) => NDDataType::Int32,
        _ => return Err(format!("invalid data type {dtype:?}")),
    })
}

/// The bitshuffle+LZ4 HDF5 filter (id 32008).
///
/// Each chunk is `[u64 BE uncompressed bytes][u32 BE block bytes][block stream]`.
/// The declared uncompressed size is authoritative: `hdf5-reader` passes the
/// expected output length in `max_output_len`, and the two must agree.
pub fn bslz4_filter(
    _desc: &FilterDescription,
    data: &[u8],
    element_size: usize,
) -> H5Result<Vec<u8>> {
    bslz4::decode_with_header(data, element_size, None)
        .map_err(|e| H5Error::FilterError(format!("bslz4: {e}")))
}

/// A filter registry that can decode the FileWriter's chunks.
///
/// LZ4 (32004) is built into `hdf5-reader`; bslz4 (32008) is registered here.
pub fn filter_registry() -> FilterRegistry {
    let mut registry = FilterRegistry::new();
    registry.register(FILTER_BSLZ4, Box::new(bslz4_filter));
    registry
}

/// One image read out of a FileWriter data file.
pub struct H5Frame {
    /// `[width, height]`, matching C's `ndDims`.
    pub dims: [usize; 2],
    pub data: NDDataBuffer,
    /// 0-based threshold index within the frame (C's `j`).
    pub threshold: usize,
}

/// Read every image out of an in-memory `_data_*.h5` blob.
///
/// C reads one hyperslab at a time straight into a pooled NDArray; this reads
/// the same hyperslabs and hands each back as an owned buffer for the caller to
/// publish.
pub fn parse(buf: &[u8], signed_data: bool) -> Result<Vec<H5Frame>, String> {
    let options = OpenOptions {
        filter_registry: Some(filter_registry()),
        ..Default::default()
    };
    let file = Hdf5File::from_bytes_with_options(buf, options)
        .map_err(|e| format!("unable to open memory as file: {e}"))?;

    let mut last_err = String::new();
    let dataset = DATA_PATHS
        .iter()
        .find_map(|path| match file.dataset(path) {
            Ok(d) => Some(d),
            Err(e) => {
                last_err = format!("unable to open '{path}': {e}");
                None
            }
        })
        .ok_or_else(|| format!("no data dataset ({last_err})"))?;

    let layout = layout_of(dataset.shape())?;
    let data_type = data_type_of(dataset.dtype(), signed_data)?;
    let n_dims = dataset.ndim();

    let mut frames = Vec::with_capacity(layout.n_images * layout.n_thresh);
    for i in 0..layout.n_images {
        for j in 0..layout.n_thresh {
            let mut selections = vec![SliceInfoElem::Index(i as u64)];
            if n_dims == 4 {
                selections.push(SliceInfoElem::Index(j as u64));
            }
            selections.push(SliceInfoElem::Slice {
                start: 0,
                end: layout.height as u64,
                step: 1,
            });
            selections.push(SliceInfoElem::Slice {
                start: 0,
                end: layout.width as u64,
                step: 1,
            });
            let selection = SliceInfo { selections };

            let data = read_slice(&dataset, &selection, data_type)
                .map_err(|e| format!("couldn't read image {i}/{j}: {e}"))?;
            frames.push(H5Frame {
                dims: [layout.width, layout.height],
                data,
                threshold: j,
            });
        }
    }
    Ok(frames)
}

/// Read one hyperslab as the NDArray element type.
///
/// The file always stores unsigned data; `signed_data` only changes how the
/// same bits are labelled, so the read is always unsigned and the buffer is
/// re-tagged (C reads with the *file* datatype into the NDArray's buffer, which
/// is the same reinterpretation).
fn read_slice(
    dataset: &hdf5_reader::Dataset,
    selection: &SliceInfo,
    data_type: NDDataType,
) -> H5Result<NDDataBuffer> {
    Ok(match data_type {
        NDDataType::UInt8 => NDDataBuffer::U8(
            dataset
                .read_slice::<u8>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        NDDataType::Int8 => NDDataBuffer::I8(
            dataset
                .read_slice::<u8>(selection)?
                .into_raw_vec_and_offset()
                .0
                .into_iter()
                .map(|v| v as i8)
                .collect(),
        ),
        NDDataType::UInt16 => NDDataBuffer::U16(
            dataset
                .read_slice::<u16>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        NDDataType::Int16 => NDDataBuffer::I16(
            dataset
                .read_slice::<u16>(selection)?
                .into_raw_vec_and_offset()
                .0
                .into_iter()
                .map(|v| v as i16)
                .collect(),
        ),
        NDDataType::UInt32 => NDDataBuffer::U32(
            dataset
                .read_slice::<u32>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        NDDataType::Int32 => NDDataBuffer::I32(
            dataset
                .read_slice::<u32>(selection)?
                .into_raw_vec_and_offset()
                .0
                .into_iter()
                .map(|v| v as i32)
                .collect(),
        ),
        other => {
            return Err(H5Error::InvalidData(format!(
                "unsupported data type {other:?}"
            )));
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDArray, NDDimension};

    #[test]
    fn a_3d_dataset_is_one_threshold() {
        assert_eq!(
            layout_of(&[7, 512, 1030]).unwrap(),
            Layout {
                n_images: 7,
                n_thresh: 1,
                height: 512,
                width: 1030,
            }
        );
    }

    #[test]
    fn a_4d_dataset_carries_the_threshold_axis() {
        assert_eq!(
            layout_of(&[7, 2, 512, 1030]).unwrap(),
            Layout {
                n_images: 7,
                n_thresh: 2,
                height: 512,
                width: 1030,
            }
        );
    }

    #[test]
    fn other_ranks_are_rejected() {
        assert!(layout_of(&[512, 1030]).is_err());
        assert!(layout_of(&[2, 2, 2, 512, 1030]).is_err());
    }

    #[test]
    fn unsigned_and_signed_element_types() {
        let u32t = Datatype::FixedPoint {
            size: 4,
            signed: false,
            byte_order: hdf5_reader::ByteOrder::LittleEndian,
        };
        assert_eq!(data_type_of(&u32t, false).unwrap(), NDDataType::UInt32);
        assert_eq!(data_type_of(&u32t, true).unwrap(), NDDataType::Int32);

        let u16t = Datatype::FixedPoint {
            size: 2,
            signed: false,
            byte_order: hdf5_reader::ByteOrder::LittleEndian,
        };
        assert_eq!(data_type_of(&u16t, false).unwrap(), NDDataType::UInt16);
        assert_eq!(data_type_of(&u16t, true).unwrap(), NDDataType::Int16);

        let f64t = Datatype::FloatingPoint {
            size: 8,
            byte_order: hdf5_reader::ByteOrder::LittleEndian,
        };
        assert!(data_type_of(&f64t, false).is_err());
    }

    /// The filter is what libhdf5 would have loaded from `HDF5_PLUGIN_PATH`;
    /// feed it a chunk built exactly the way the FileWriter frames one.
    #[test]
    fn the_bslz4_filter_decodes_a_filewriter_chunk() {
        let pixels: Vec<u32> = (0..4096u32).map(|i| (i * 7) % 1013).collect();
        let src = NDArray::with_data(
            vec![NDDimension::new(pixels.len())],
            NDDataBuffer::U32(pixels.clone()),
        );
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src);

        let mut chunk = Vec::new();
        chunk.extend_from_slice(&((pixels.len() * 4) as u64).to_be_bytes());
        chunk.extend_from_slice(&((bslz4::default_block_size(4) * 4) as u32).to_be_bytes());
        chunk.extend_from_slice(compressed.data.as_u8_slice());

        let desc = FilterDescription {
            id: FILTER_BSLZ4,
            name: None,
            client_data: vec![0, 0],
        };
        let out = bslz4_filter(&desc, &chunk, 4).unwrap();
        let got: Vec<u32> = out
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        assert_eq!(got, pixels);
    }

    #[test]
    fn the_bslz4_filter_rejects_a_truncated_chunk() {
        let desc = FilterDescription {
            id: FILTER_BSLZ4,
            name: None,
            client_data: vec![],
        };
        assert!(bslz4_filter(&desc, &[0u8; 8], 4).is_err());
    }
}
