//! Pixels written straight into the pool buffer a frame goes out in, as C's
//! `getFrame`s write `pData`.

use epics_rs::ad_core::ndarray::{NDDataType, NDDataViewMut};

/// The type a detector frame is allocated as: its unsigned type, or with
/// SignedData set the signed type over the same bits. C relabels
/// `pArray->dataType` after the fill (eigerDetector.cpp:1519-1533); a pool
/// array's type is fixed once allocated, so the label is chosen up front.
pub(crate) fn frame_type(data_type: NDDataType, signed: bool) -> NDDataType {
    match (data_type, signed) {
        (NDDataType::UInt8, true) => NDDataType::Int8,
        (NDDataType::UInt16, true) => NDDataType::Int16,
        (NDDataType::UInt32, true) => NDDataType::Int32,
        (other, _) => other,
    }
}

/// Put little-endian pixels just written into `out` as bytes into host
/// order. The detector is little-endian, so on a little-endian host this is
/// nothing.
pub(crate) fn le_to_host(out: NDDataViewMut<'_>) {
    if cfg!(target_endian = "little") {
        return;
    }
    match out {
        NDDataViewMut::U16(d) => d.iter_mut().for_each(|x| *x = u16::from_le(*x)),
        NDDataViewMut::I16(d) => d.iter_mut().for_each(|x| *x = i16::from_le(*x)),
        NDDataViewMut::U32(d) => d.iter_mut().for_each(|x| *x = u32::from_le(*x)),
        NDDataViewMut::I32(d) => d.iter_mut().for_each(|x| *x = i32::from_le(*x)),
        _ => {}
    }
}
