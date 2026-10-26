//! FileWriter HDF5 parse (port of `eigerDetector::parseH5File`,
//! eigerDetector.cpp:1680).
//!
//! The C driver opens the downloaded `_data_*.h5` blob straight out of memory
//! with `H5LTopen_file_image` and reads `/entry/data/data` one hyperslab at a
//! time. This port does the same with `hdf5-reader` (pure Rust, read-only),
//! plus a locally-registered bitshuffle/LZ4 filter — HDF5 filter id 32008 —
//! which is what libhdf5 loads out of `HDF5_PLUGIN_PATH` in the C build. No
//! plugin path, no `libhdf5`, no `libbshuf` is needed here.

use epics_rs::ad_core::ndarray::{NDDataType, NDDataViewMut};
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
        _ => Err("number of dimensions must be 3 or 4".into()),
    }
}

/// Map the dataset's element type to an NDArray type.
///
/// The detector writes unsigned data. Bad pixels and gaps are very large
/// positive numbers, which makes autoscaling awkward, so `signed_data`
/// reinterprets the same bytes as signed (C, eigerDetector.cpp:1782-1794).
pub fn data_type_of(dtype: &Datatype, signed_data: bool) -> Result<NDDataType, String> {
    let Datatype::FixedPoint { size, .. } = dtype else {
        return Err("invalid data type".into());
    };
    Ok(match (size, signed_data) {
        (1, false) => NDDataType::UInt8,
        (1, true) => NDDataType::Int8,
        (2, false) => NDDataType::UInt16,
        (2, true) => NDDataType::Int16,
        (4, false) => NDDataType::UInt32,
        (4, true) => NDDataType::Int32,
        _ => return Err("invalid data type".into()),
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
    bslz4::decode_with_header(data, element_size)
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
    /// The NDArray type the pixels go out as (C's `ndType`).
    pub data_type: NDDataType,
    pub pixels: H5Pixels,
    /// 0-based threshold index within the frame (C's `j`).
    pub threshold: usize,
}

/// One hyperslab as `hdf5-reader` returns it, in the file's unsigned type.
pub enum H5Pixels {
    U8(Vec<u8>),
    U16(Vec<u16>),
    U32(Vec<u32>),
}

impl H5Pixels {
    /// Copy into `out`, an array of the frame's `data_type`. SignedData only
    /// relabels the same bits.
    ///
    /// C's `H5Dread` reads straight into `pData` (eigerDetector.cpp:1869);
    /// `hdf5-reader` reads a hyperslab only into an array of its own, so the
    /// pixels pass once more here.
    pub fn write_into(&self, out: NDDataViewMut<'_>) {
        fn copy<S: Copy, D>(src: &[S], dst: &mut [D], cast: impl Fn(S) -> D) {
            for (d, s) in dst.iter_mut().zip(src) {
                *d = cast(*s);
            }
        }
        match (self, out) {
            (Self::U8(v), NDDataViewMut::U8(d)) => d.copy_from_slice(v),
            (Self::U8(v), NDDataViewMut::I8(d)) => copy(v, d, |x| x as i8),
            (Self::U16(v), NDDataViewMut::U16(d)) => d.copy_from_slice(v),
            (Self::U16(v), NDDataViewMut::I16(d)) => copy(v, d, |x| x as i16),
            (Self::U32(v), NDDataViewMut::U32(d)) => d.copy_from_slice(v),
            (Self::U32(v), NDDataViewMut::I32(d)) => copy(v, d, |x| x as i32),
            (_, out) => unreachable!("an H5 frame is never {:?}", out.data_type()),
        }
    }
}

/// A failure whose line `parse` has already printed.
#[derive(Debug)]
pub struct Reported;

fn report(err: &mut dyn FnMut(&str), msg: &str) -> Reported {
    err(msg);
    Reported
}

/// Read every image out of an in-memory `_data_*.h5` blob.
///
/// C reads one hyperslab at a time straight into a pooled NDArray; this reads
/// the same hyperslabs and hands each back for the caller to copy into one.
///
/// `err` receives the lines C's `parseH5File` prints, in its words; `Err(Reported)`
/// means one of them ended the parse.
pub fn parse(
    buf: &[u8],
    signed_data: bool,
    err: &mut dyn FnMut(&str),
) -> Result<Vec<H5Frame>, Reported> {
    let options = OpenOptions {
        filter_registry: Some(filter_registry()),
        ..Default::default()
    };
    let file = Hdf5File::from_bytes_with_options(buf, options)
        .map_err(|_| report(err, "unable to open memory as file"))?;

    let [data_data, data] = DATA_PATHS;
    let dataset = match file.dataset(data_data) {
        Ok(d) => d,
        Err(_) => {
            err("unable to open '/entry/data/data'. Will try '/entry/data'");
            file.dataset(data)
                .map_err(|_| report(err, "unable to open '/entry/data' dataset"))?
        }
    };

    let layout = layout_of(dataset.shape()).map_err(|e| report(err, &e))?;
    let data_type = data_type_of(dataset.dtype(), signed_data).map_err(|e| report(err, &e))?;
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

            let pixels = read_slice(&dataset, &selection, data_type)
                .map_err(|_| report(err, "couldn't read image"))?;
            frames.push(H5Frame {
                dims: [layout.width, layout.height],
                data_type,
                pixels,
                threshold: j,
            });
        }
    }
    Ok(frames)
}

/// Read one hyperslab in the file's unsigned type.
///
/// The file always stores unsigned data; `signed_data` only changes how the
/// same bits are labelled (C reads with the *file* datatype into the
/// NDArray's buffer, which is the same reinterpretation).
fn read_slice(
    dataset: &hdf5_reader::Dataset,
    selection: &SliceInfo,
    data_type: NDDataType,
) -> H5Result<H5Pixels> {
    Ok(match data_type.element_size() {
        1 => H5Pixels::U8(
            dataset
                .read_slice::<u8>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        2 => H5Pixels::U16(
            dataset
                .read_slice::<u16>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        4 => H5Pixels::U32(
            dataset
                .read_slice::<u32>(selection)?
                .into_raw_vec_and_offset()
                .0,
        ),
        _ => {
            return Err(H5Error::InvalidData(format!(
                "unsupported data type {data_type:?}"
            )));
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDimension};

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
            NDDataBuffer::from_vec(pixels.clone()),
        );
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src).unwrap();

        let mut chunk = Vec::new();
        chunk.extend_from_slice(&((pixels.len() * 4) as u64).to_be_bytes());
        chunk.extend_from_slice(&((bslz4::default_block_size(4) * 4) as u32).to_be_bytes());
        chunk.extend_from_slice(compressed.data().as_u8_slice());

        let desc = FilterDescription {
            id: FILTER_BSLZ4,
            name: None,
            client_data: vec![0, 0],
        };
        let out = bslz4_filter(&desc, &chunk, 4).unwrap();
        let got: Vec<u32> = out
            .as_chunks::<4>()
            .0
            .iter()
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
