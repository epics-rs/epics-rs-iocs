//! The pure buffer arithmetic of `NDDriverStdArrays` — `fillBuffer`,
//! `copyBuffer`, the `dimProd_` cumulative product, and the `currentIndex`
//! multi-dimensional index (NDDriverStdArrays.cpp:94-118, 177-181, 256-265).
//!
//! None of this touches the asyn parameter library or the NDArray pool, so it
//! is unit-tested directly against the C expressions without an IOC.
//!
//! ## C integer-conversion semantics
//!
//! `copyBuffer<epicsType, NDArrayType>` and `fillBuffer<NDArrayType>` cast with
//! a plain C cast `(NDArrayType)value`, which for integer targets truncates
//! toward zero and then wraps modulo 2^N. Rust's `as` saturates instead.
//! [`CCast::from_f64_c`] restores the C behaviour by going through `i128`
//! (truncate) before the narrowing bit-truncation. Every waveform input type
//! (`epicsInt8/16/32`, `epicsFloat32/64`) is exactly representable in `f64`, so
//! converting each input element to `f64` first and then casting to the output
//! type reproduces `(NDArrayType)(epicsType)value` for all input/output pairs.

use epics_rs::ad_core::ndarray::NDDataBuffer;

/// `ND_ARRAY_MAX_DIMS` (NDArray.h) — the fixed dimension bound the C driver
/// uses for `arrayDimensions_`, `dimProd_`, and `currentIndex`.
pub const ND_ARRAY_MAX_DIMS: usize = 10;

/// A C cast target: `(Self)doubleValue`.
trait CCast: Copy {
    fn from_f64_c(v: f64) -> Self;
}

macro_rules! impl_ccast_int {
    ($($t:ty),*) => {$(
        impl CCast for $t {
            fn from_f64_c(v: f64) -> Self {
                // `v as i128` truncates toward zero and saturates only at the
                // i128 bounds (~1.7e38); `as $t` then keeps the low bits,
                // reproducing the C modular wrap.
                (v as i128) as Self
            }
        }
    )*};
}

macro_rules! impl_ccast_float {
    ($($t:ty),*) => {$(
        impl CCast for $t {
            fn from_f64_c(v: f64) -> Self { v as Self }
        }
    )*};
}

impl_ccast_int!(i8, u8, i16, u16, i32, u32);
impl_ccast_float!(f32, f64);

fn fill_typed<T: CCast>(buf: &mut [T], fill_value: f64) {
    let fill = T::from_f64_c(fill_value);
    for x in buf.iter_mut() {
        *x = fill;
    }
}

/// `fillBuffer<NDArrayType>(fillValueDouble)` (NDDriverStdArrays.cpp:94-103):
/// every element of the output buffer is set to `(NDArrayType)fillValueDouble`.
///
/// The C `switch (dataType)` in `writeXXXArray` has no `NDInt64`/`NDUInt64`
/// case (NDDriverStdArrays.cpp:188-213), so `fillBuffer` is never instantiated
/// for those two output types; an `Int64`/`UInt64` array is left at its
/// pool-allocated zero. This port reproduces that by leaving those buffers
/// untouched.
pub fn fill_buffer(buf: &mut NDDataBuffer, fill_value: f64) {
    match buf {
        NDDataBuffer::I8(v) => fill_typed(v, fill_value),
        NDDataBuffer::U8(v) => fill_typed(v, fill_value),
        NDDataBuffer::I16(v) => fill_typed(v, fill_value),
        NDDataBuffer::U16(v) => fill_typed(v, fill_value),
        NDDataBuffer::I32(v) => fill_typed(v, fill_value),
        NDDataBuffer::U32(v) => fill_typed(v, fill_value),
        NDDataBuffer::F32(v) => fill_typed(v, fill_value),
        NDDataBuffer::F64(v) => fill_typed(v, fill_value),
        // C parity: no NDInt64/NDUInt64 case in the fillBuffer switch.
        NDDataBuffer::I64(_) | NDDataBuffer::U64(_) => {}
    }
}

fn copy_typed<T: CCast>(
    out: &mut [T],
    next_element: usize,
    stride: usize,
    input: &[f64],
    total: usize,
) {
    if total == 0 {
        return;
    }
    let mut j = 0usize;
    for &val in input {
        // C writes `pOut[j]` where `pOut = pData + nextElement`, i.e. absolute
        // index `nextElement + j`, and only wraps `j` (not the sum) modulo
        // `total`. With the caller's clamp `nextElement + nElements <= total`
        // and the default `stride == 1`, `nextElement + j < total` always, so
        // this matches C exactly. For `stride > 1` combined with
        // `nextElement > 0` the C code can index past the buffer end (a latent
        // out-of-bounds write / UB); this port folds the absolute index back
        // into the buffer with `% total` to stay memory-safe. See the
        // deviation note in the crate docs.
        let idx = (next_element + j) % total;
        out[idx] = T::from_f64_c(val);
        j += stride;
        if j >= total {
            j %= total;
        }
    }
}

/// `copyBuffer<epicsType, NDArrayType>(nextElement, stride, pValue, nElements)`
/// (NDDriverStdArrays.cpp:105-118). `input` holds the `nElements` (already
/// clamped) waveform values converted to `f64`; each is cast to the output
/// element type with C semantics and stored strided into `out`.
///
/// As with [`fill_buffer`], the C `switch (dataType)` has no `NDInt64`/
/// `NDUInt64` case (NDDriverStdArrays.cpp:226-251), so `copyBuffer` is never
/// instantiated for those output types and this port leaves them untouched.
pub fn copy_buffer(out: &mut NDDataBuffer, next_element: usize, stride: usize, input: &[f64]) {
    let total = out.len();
    match out {
        NDDataBuffer::I8(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::U8(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::I16(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::U16(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::I32(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::U32(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::F32(v) => copy_typed(v, next_element, stride, input, total),
        NDDataBuffer::F64(v) => copy_typed(v, next_element, stride, input, total),
        // C parity: no NDInt64/NDUInt64 case in the copyBuffer switch.
        NDDataBuffer::I64(_) | NDDataBuffer::U64(_) => {}
    }
}

/// `dimProd_` (NDDriverStdArrays.cpp:177-181): `dimProd_[0] = dim[0]`, and
/// `dimProd_[i] = dim[i] * dimProd_[i-1]` for `1 <= i < numDimensions`.
/// Entries at or beyond `num_dimensions` stay zero (the C `memset`).
pub fn dim_prod(
    array_dimensions: &[usize; ND_ARRAY_MAX_DIMS],
    num_dimensions: usize,
) -> [usize; ND_ARRAY_MAX_DIMS] {
    let mut prod = [0usize; ND_ARRAY_MAX_DIMS];
    if num_dimensions == 0 {
        return prod;
    }
    let n = num_dimensions.min(ND_ARRAY_MAX_DIMS);
    prod[0] = array_dimensions[0];
    for i in 1..n {
        prod[i] = array_dimensions[i] * prod[i - 1];
    }
    prod
}

/// `currentIndex` (NDDriverStdArrays.cpp:256-265): converts the linear
/// `nextElement` write cursor (already incremented by the copy) into a
/// 1-based multi-dimensional index of the last written element.
///
/// The arithmetic is done in `i32` to match the C `int itemp` — including C's
/// truncated division/remainder for the `nextElement == 0` corner
/// (`itemp == -1`). Divisions guard against a zero dimension/product (which in
/// C would be undefined) by leaving that component at its `memset` zero.
pub fn current_index(
    next_element: i32,
    num_dimensions: usize,
    dim_prod: &[usize; ND_ARRAY_MAX_DIMS],
    array_dimensions: &[usize; ND_ARRAY_MAX_DIMS],
) -> [i32; ND_ARRAY_MAX_DIMS] {
    let mut index = [0i32; ND_ARRAY_MAX_DIMS];
    let n = num_dimensions.min(ND_ARRAY_MAX_DIMS) as i32;
    let mut itemp = next_element - 1;

    let mut i = n - 1;
    while i > 0 {
        if i < n - 1 {
            let m = dim_prod[i as usize] as i32;
            if m != 0 {
                itemp %= m;
            }
        }
        let d = dim_prod[(i - 1) as usize] as i32;
        index[i as usize] = if d != 0 { 1 + itemp / d } else { 0 };
        i -= 1;
    }
    if n >= 1 {
        let d0 = array_dimensions[0] as i32;
        index[0] = if d0 != 0 { 1 + itemp % d0 } else { 0 };
    }
    index
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::ad_core::ndarray::NDDataType;

    fn buf(dt: NDDataType, n: usize) -> NDDataBuffer {
        NDDataBuffer::zeros(dt, n)
    }

    #[test]
    fn fill_buffer_casts_toward_zero_and_wraps_like_c() {
        // (epicsInt8)300 == 44; (epicsUInt8)-1 == 255; truncation drops the
        // fraction before the wrap.
        let mut b = buf(NDDataType::Int8, 3);
        fill_buffer(&mut b, 300.7);
        assert!(matches!(&b, NDDataBuffer::I8(v) if v == &[44, 44, 44]));

        let mut b = buf(NDDataType::UInt8, 2);
        fill_buffer(&mut b, -1.9);
        assert!(matches!(&b, NDDataBuffer::U8(v) if v == &[255, 255]));

        let mut b = buf(NDDataType::Float32, 2);
        fill_buffer(&mut b, 2.5);
        assert!(matches!(&b, NDDataBuffer::F32(v) if v == &[2.5, 2.5]));
    }

    #[test]
    fn fill_buffer_is_a_noop_for_int64_and_uint64() {
        // C parity: the fillBuffer switch has no NDInt64/NDUInt64 case.
        let mut b = buf(NDDataType::Int64, 3);
        fill_buffer(&mut b, 7.0);
        assert!(matches!(&b, NDDataBuffer::I64(v) if v == &[0, 0, 0]));
        let mut b = buf(NDDataType::UInt64, 3);
        fill_buffer(&mut b, 7.0);
        assert!(matches!(&b, NDDataBuffer::U64(v) if v == &[0, 0, 0]));
    }

    #[test]
    fn copy_buffer_stride_one_writes_contiguously_from_next_element() {
        let mut b = buf(NDDataType::Float64, 6);
        copy_buffer(&mut b, 2, 1, &[10.0, 11.0, 12.0]);
        assert!(matches!(&b, NDDataBuffer::F64(v)
            if v == &[0.0, 0.0, 10.0, 11.0, 12.0, 0.0]));
    }

    #[test]
    fn copy_buffer_applies_the_stride() {
        // stride 2, starting at element 0: fills indices 0, 2, 4.
        let mut b = buf(NDDataType::Int32, 6);
        copy_buffer(&mut b, 0, 2, &[1.0, 2.0, 3.0]);
        assert!(matches!(&b, NDDataBuffer::I32(v) if v == &[1, 0, 2, 0, 3, 0]));
    }

    #[test]
    fn copy_buffer_wraps_the_stride_cursor_modulo_total() {
        // total 4, stride 3, start 0: j = 0, 3, (6 -> 2), (5 -> 1).
        let mut b = buf(NDDataType::Int32, 4);
        copy_buffer(&mut b, 0, 3, &[1.0, 2.0, 3.0, 4.0]);
        // index 0 <- 1, index 3 <- 2, index 2 <- 3, index 1 <- 4.
        assert!(matches!(&b, NDDataBuffer::I32(v) if v == &[1, 4, 3, 2]));
    }

    #[test]
    fn copy_buffer_converts_input_to_the_output_type_with_c_casts() {
        // f64 input 257.9 -> (epicsInt8) = 1 (truncate to 257, wrap mod 256).
        let mut b = buf(NDDataType::Int8, 2);
        copy_buffer(&mut b, 0, 1, &[257.9, -1.0]);
        assert!(matches!(&b, NDDataBuffer::I8(v) if v == &[1, -1]));
    }

    #[test]
    fn dim_prod_is_the_cumulative_product() {
        let mut dims = [0usize; ND_ARRAY_MAX_DIMS];
        dims[0] = 4;
        dims[1] = 3;
        dims[2] = 2;
        let p = dim_prod(&dims, 3);
        assert_eq!(p[0], 4);
        assert_eq!(p[1], 12);
        assert_eq!(p[2], 24);
        // Entries beyond num_dimensions stay zero.
        assert_eq!(p[3], 0);
    }

    #[test]
    fn current_index_of_a_2d_array_is_one_based_row_and_column() {
        // A [4, 3] array (dim0 = 4 fastest). dimProd_ = [4, 12].
        let mut dims = [0usize; ND_ARRAY_MAX_DIMS];
        dims[0] = 4;
        dims[1] = 3;
        let p = dim_prod(&dims, 2);
        // After writing 5 elements, nextElement = 5, last element linear = 4,
        // which is (col 1, row 2) 0-based -> C reports 1-based (1, 2).
        let ci = current_index(5, 2, &p, &dims);
        assert_eq!(ci[0], 1); // 1 + (4 % 4)
        assert_eq!(ci[1], 2); // 1 + (4 / 4)
        assert_eq!(ci[2], 0);
    }

    #[test]
    fn current_index_of_a_1d_array_counts_within_the_first_dimension() {
        let mut dims = [0usize; ND_ARRAY_MAX_DIMS];
        dims[0] = 10;
        let p = dim_prod(&dims, 1);
        let ci = current_index(7, 1, &p, &dims);
        assert_eq!(ci[0], 7); // 1 + ((7 - 1) % 10)
    }
}
