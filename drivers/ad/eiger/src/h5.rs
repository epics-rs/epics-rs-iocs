//! FileWriter HDF5 parse (port of `eigerDetector::parseH5File`,
//! eigerDetector.cpp:1680).
//!
//! The C driver opens the downloaded `_data_*.h5` blob straight out of memory
//! with `H5LTopen_file_image` and reads `/entry/data/data` one hyperslab at a
//! time. This port reads the same images with `hdf5-reader` (pure Rust, read-only),
//! plus a locally-registered bitshuffle/LZ4 filter — HDF5 filter id 32008 —
//! which is what libhdf5 loads out of `HDF5_PLUGIN_PATH` in the C build. No
//! plugin path, no `libhdf5`, no `libbshuf` is needed here.

use std::ops::Range;
use std::sync::Arc;

use epics_rs::ad_core::ndarray::NDDataType;
use hdf5_reader::error::{Error as H5Error, Result as H5Result};
use hdf5_reader::messages::filter_pipeline::FilterDescription;
use hdf5_reader::{
    ByteOrder, Dataset, DatasetChunk, Datatype, FilterRegistry, Hdf5File, OpenOptions,
};

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
/// C takes only the native unsigned types (`H5Tequal` against
/// `H5T_NATIVE_UINT*`), so an image's bytes go into the NDArray unconverted.
pub fn data_type_of(dtype: &Datatype, signed_data: bool) -> Result<NDDataType, String> {
    let native = if cfg!(target_endian = "little") {
        ByteOrder::LittleEndian
    } else {
        ByteOrder::BigEndian
    };
    let Datatype::FixedPoint {
        size,
        signed: false,
        byte_order,
    } = dtype
    else {
        return Err("invalid data type".into());
    };
    if *byte_order != native {
        return Err("invalid data type".into());
    }
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
    /// 0-based threshold index within the frame (C's `j`).
    pub threshold: usize,
    pixels: Pixels,
}

/// Where one image's decoded bytes are held.
enum Pixels {
    /// Its own chunk: the FileWriter writes one image per chunk.
    Chunk(DatasetChunk),
    /// Its span of the whole dataset, read at once for any other layout.
    Span(Arc<Vec<u8>>, Range<usize>),
}

impl H5Frame {
    /// The image's bytes as the NDArray holds them: SignedData only relabels
    /// them. C's `H5Dread` decodes them into `pData` (eigerDetector.cpp:1869),
    /// so copy them straight into the pool array.
    pub fn bytes(&self) -> &[u8] {
        match &self.pixels {
            Pixels::Chunk(chunk) => chunk.bytes(),
            Pixels::Span(all, at) => &all[at.clone()],
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
/// C reads one hyperslab at a time straight into a pooled NDArray; this
/// decodes the same images, in C's order, for the caller to copy into one.
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

    let images =
        images(&dataset, &layout, data_type).map_err(|_| report(err, "couldn't read image"))?;
    Ok(images
        .into_iter()
        .enumerate()
        .map(|(k, pixels)| H5Frame {
            dims: [layout.width, layout.height],
            data_type,
            threshold: k % layout.n_thresh,
            pixels,
        })
        .collect())
}

/// Every image's bytes, in C's order (`i`, then `j`).
fn images(dataset: &Dataset, layout: &Layout, data_type: NDDataType) -> H5Result<Vec<Pixels>> {
    if let Some(chunks) = image_chunks(dataset, layout)? {
        return Ok(chunks.into_iter().map(Pixels::Chunk).collect());
    }
    let all = Arc::new(dataset.read_raw_bytes()?);
    let size = layout.height * layout.width * data_type.element_size();
    Ok((0..layout.n_images * layout.n_thresh)
        .map(|k| Pixels::Span(Arc::clone(&all), k * size..(k + 1) * size))
        .collect())
}

/// The decoded chunks in C's image order, when every image is a chunk of its
/// own; `None` for any other layout.
fn image_chunks(dataset: &Dataset, layout: &Layout) -> H5Result<Option<Vec<DatasetChunk>>> {
    let four_d = dataset.ndim() == 4;
    let mut one_image = vec![1, layout.height as u32, layout.width as u32];
    if four_d {
        one_image.insert(0, 1);
    }
    if dataset.chunks() != Some(one_image) {
        return Ok(None);
    }
    let mut slots: Vec<Option<DatasetChunk>> = std::iter::repeat_with(|| None)
        .take(layout.n_images * layout.n_thresh)
        .collect();
    for chunk in dataset.iter_chunks()? {
        let chunk = chunk?;
        let at = chunk.offsets();
        let k = at[0] as usize * layout.n_thresh + if four_d { at[1] as usize } else { 0 };
        let Some(slot) = slots.get_mut(k) else {
            return Ok(None);
        };
        *slot = Some(chunk);
    }
    // An unwritten chunk reads as the fill value, which only the whole read
    // materializes.
    Ok(slots.into_iter().collect())
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

    /// C's `H5Tequal` against `H5T_NATIVE_UINT*` (eigerDetector.cpp:1782-1794)
    /// refuses a signed or a foreign-endian file type.
    #[test]
    fn only_native_unsigned_file_types_are_read() {
        let fixed = |signed, byte_order| Datatype::FixedPoint {
            size: 2,
            signed,
            byte_order,
        };
        let foreign = if cfg!(target_endian = "little") {
            ByteOrder::BigEndian
        } else {
            ByteOrder::LittleEndian
        };
        assert!(data_type_of(&fixed(true, ByteOrder::LittleEndian), false).is_err());
        assert!(data_type_of(&fixed(false, foreign), false).is_err());
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
