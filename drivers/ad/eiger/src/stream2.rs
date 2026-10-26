//! SIMPLON ZeroMQ stream interface, version 2: the CBOR parser of `stream2.c`
//! and the `Stream2API` class of `stream2Api.cpp`.
//!
//! The detector PUSHes one self-described CBOR map per message on
//! `tcp://HOST:31001`, typed by its first key: `start`, `image` or `end`.
//! The parser keeps `stream2.c`'s result codes because the caller compares
//! them against streamApi.h's codes (see [`Stream2Error::code`]).

use std::collections::VecDeque;
use std::ops::Range;

use bytes::Bytes;
use epics_rs::ad_core::codec::{Codec, CodecName};
use epics_rs::ad_core::ndarray::{NDArray, NDDataBuffer, NDDataType, NDDimension};
use epics_rs::ad_core::ndarray_pool::{DriverArray, DriverPool};
use epics_rs::ad_core::timestamp::EpicsTimestamp;

use crate::bslz4;
use crate::pixels::{frame_type, le_to_host};

const ZMQ_PORT: u16 = 31001;

/// stream2.c `MAX_KEY_LEN`.
const MAX_KEY_LEN: usize = 64;
const MULTI_DIMENSIONAL_ARRAY_ROW_MAJOR: u64 = 40;
const DECTRIS_COMPRESSION: u64 = 56500;

const TYPED_ARRAY_UINT8: u64 = 64;
const TYPED_ARRAY_UINT16_LITTLE_ENDIAN: u64 = 69;
const TYPED_ARRAY_UINT32_LITTLE_ENDIAN: u64 = 70;

/// stream2.h `enum stream2_result`, by value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    OutOfMemory = 1,
    Signature = 2,
    Decode = 3,
    Parse = 4,
    NotImplemented = 5,
}

type PResult<T> = Result<T, ParseError>;

// ---------------------------------------------------------------------------
// CBOR reader (the subset of tinycbor stream2.c uses)
// ---------------------------------------------------------------------------

const MAJOR_UINT: u8 = 0;
const MAJOR_BYTES: u8 = 2;
const MAJOR_TEXT: u8 = 3;
const MAJOR_ARRAY: u8 = 4;
const MAJOR_MAP: u8 = 5;
const MAJOR_TAG: u8 = 6;
const MAJOR_SIMPLE: u8 = 7;
const BREAK: u8 = 0xff;

/// A cursor over one message. Malformed or truncated encoding is
/// `ParseError::Decode` (tinycbor's errors through `CBOR_RESULT`); a well-formed
/// item of the wrong type is `ParseError::Parse`, as stream2.c returns it.
struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

/// Where a container's items end: after `n` items, or at a break byte.
#[derive(Clone, Copy)]
enum Len {
    Definite(u64),
    Indefinite,
}

impl<'a> Reader<'a> {
    fn peek(&self) -> PResult<u8> {
        self.buf.get(self.pos).copied().ok_or(ParseError::Decode)
    }

    fn major(&self) -> PResult<u8> {
        Ok(self.peek()? >> 5)
    }

    fn take(&mut self, n: usize) -> PResult<&'a [u8]> {
        let end = self.pos.checked_add(n).ok_or(ParseError::Decode)?;
        let s = self.buf.get(self.pos..end).ok_or(ParseError::Decode)?;
        self.pos = end;
        Ok(s)
    }

    /// Read an item's initial byte and argument.
    fn head(&mut self) -> PResult<(u8, Len)> {
        let b = self.peek()?;
        self.pos += 1;
        let (major, info) = (b >> 5, b & 0x1f);
        let len = match info {
            0..=23 => Len::Definite(u64::from(info)),
            24..=27 => {
                let n = 1usize << (info - 24);
                let mut v = 0u64;
                for &x in self.take(n)? {
                    v = (v << 8) | u64::from(x);
                }
                Len::Definite(v)
            }
            31 if (MAJOR_BYTES..=MAJOR_MAP).contains(&major) => Len::Indefinite,
            _ => return Err(ParseError::Decode),
        };
        Ok((major, len))
    }

    fn definite(&mut self) -> PResult<(u8, u64)> {
        match self.head()? {
            (major, Len::Definite(n)) => Ok((major, n)),
            _ => Err(ParseError::Decode),
        }
    }

    /// Whether a container still has items (tinycbor `cbor_value_is_valid`
    /// on the inner iterator); consumes the break of an indefinite one.
    fn more(&mut self, remaining: &mut Len) -> PResult<bool> {
        match remaining {
            Len::Definite(0) => Ok(false),
            Len::Definite(n) => {
                *n -= 1;
                Ok(true)
            }
            Len::Indefinite => {
                if self.peek()? == BREAK {
                    self.pos += 1;
                    Ok(false)
                } else {
                    Ok(true)
                }
            }
        }
    }

    /// Enter a map or array of the given major type; `Parse` if it is not one.
    fn enter(&mut self, major: u8) -> PResult<Len> {
        if self.major()? != major {
            return Err(ParseError::Parse);
        }
        let (_, len) = self.head()?;
        Ok(match (major, len) {
            (MAJOR_MAP, Len::Definite(n)) => {
                Len::Definite(n.checked_mul(2).ok_or(ParseError::Decode)?)
            }
            (_, len) => len,
        })
    }

    /// Enter a container whose length C reads up front
    /// (`cbor_value_get_{array,map}_length`): indefinite is
    /// `CborErrorUnknownLength`, which `CBOR_RESULT` makes `Parse`.
    fn enter_counted(&mut self, major: u8) -> PResult<u64> {
        let save = self.pos;
        match self.enter(major)? {
            Len::Definite(n) => Ok(if major == MAJOR_MAP { n / 2 } else { n }),
            Len::Indefinite => {
                self.pos = save;
                Err(ParseError::Parse)
            }
        }
    }

    /// The bytes of a definite or chunked string of `major` type.
    fn string(&mut self, major: u8) -> PResult<Vec<u8>> {
        let (_, len) = self.head()?;
        match len {
            Len::Definite(n) => Ok(self
                .take(usize::try_from(n).map_err(|_| ParseError::Decode)?)?
                .to_vec()),
            Len::Indefinite => {
                let mut out = Vec::new();
                while self.peek()? != BREAK {
                    let (m, n) = self.definite()?;
                    if m != major {
                        return Err(ParseError::Decode);
                    }
                    out.extend_from_slice(
                        self.take(usize::try_from(n).map_err(|_| ParseError::Decode)?)?,
                    );
                }
                self.pos += 1;
                Ok(out)
            }
        }
    }

    /// Skip one whole item, tags included (`cbor_value_advance`).
    fn skip(&mut self) -> PResult<()> {
        let (major, len) = self.head()?;
        match (major, len) {
            (MAJOR_BYTES | MAJOR_TEXT, Len::Definite(n)) => {
                self.take(usize::try_from(n).map_err(|_| ParseError::Decode)?)?;
            }
            (MAJOR_BYTES | MAJOR_TEXT, Len::Indefinite) => {
                while self.peek()? != BREAK {
                    let (m, n) = self.definite()?;
                    if m != major {
                        return Err(ParseError::Decode);
                    }
                    self.take(usize::try_from(n).map_err(|_| ParseError::Decode)?)?;
                }
                self.pos += 1;
            }
            (MAJOR_ARRAY | MAJOR_MAP, mut len) => {
                if let (MAJOR_MAP, Len::Definite(n)) = (major, len) {
                    len = Len::Definite(n.checked_mul(2).ok_or(ParseError::Decode)?);
                }
                while self.more(&mut len)? {
                    self.skip()?;
                }
            }
            (MAJOR_TAG, _) => self.skip()?,
            _ => {}
        }
        Ok(())
    }

    /// `cbor_value_skip_tag`.
    fn skip_tags(&mut self) -> PResult<()> {
        while self.major()? == MAJOR_TAG {
            self.head()?;
        }
        Ok(())
    }

    /// stream2.c `parse_key`. `None` is a key of `MAX_KEY_LEN` bytes or more:
    /// C hands back an empty key with the cursor still on it, because its
    /// failed copy does not advance.
    fn key(&mut self) -> PResult<Option<Vec<u8>>> {
        if self.major()? != MAJOR_TEXT {
            return Err(ParseError::Parse);
        }
        let save = self.pos;
        let key = self.string(MAJOR_TEXT)?;
        if key.len() >= MAX_KEY_LEN {
            self.pos = save;
            return Ok(None);
        }
        Ok(Some(key))
    }

    /// stream2.c `parse_tag`: exactly one tag.
    fn tag(&mut self) -> PResult<u64> {
        if self.major()? != MAJOR_TAG {
            return Err(ParseError::Parse);
        }
        Ok(self.definite()?.1)
    }

    fn bool(&mut self) -> PResult<bool> {
        match self.peek()? {
            0xf4 => {
                self.pos += 1;
                Ok(false)
            }
            0xf5 => {
                self.pos += 1;
                Ok(true)
            }
            _ => Err(ParseError::Parse),
        }
    }

    fn double(&mut self) -> PResult<f64> {
        let b = self.peek()?;
        if b >> 5 != MAJOR_SIMPLE {
            return Err(ParseError::Parse);
        }
        let v = match b {
            0xf9 => {
                self.pos += 1;
                let s = self.take(2)?;
                f64::from(half_to_float(u16::from_be_bytes([s[0], s[1]])))
            }
            0xfa => {
                self.pos += 1;
                let s = self.take(4)?;
                f64::from(f32::from_be_bytes(s.try_into().unwrap()))
            }
            0xfb => {
                self.pos += 1;
                f64::from_be_bytes(self.take(8)?.try_into().unwrap())
            }
            _ => return Err(ParseError::Parse),
        };
        Ok(v)
    }

    fn uint64(&mut self) -> PResult<u64> {
        if self.major()? != MAJOR_UINT {
            return Err(ParseError::Parse);
        }
        Ok(self.definite()?.1)
    }

    fn text(&mut self) -> PResult<String> {
        if self.major()? != MAJOR_TEXT {
            return Err(ParseError::Parse);
        }
        Ok(String::from_utf8_lossy(&self.string(MAJOR_TEXT)?).into_owned())
    }

    /// A definite byte string, as an offset range into the message
    /// (`consume_byte_string_nocopy`).
    fn byte_range(&mut self) -> PResult<Range<usize>> {
        if self.major()? != MAJOR_BYTES {
            return Err(ParseError::Parse);
        }
        let (_, len) = self.head()?;
        let Len::Definite(n) = len else {
            return Err(ParseError::Parse);
        };
        let start = self.pos;
        self.take(usize::try_from(n).map_err(|_| ParseError::Decode)?)?;
        Ok(start..self.pos)
    }

    fn array_2_uint64(&mut self) -> PResult<[u64; 2]> {
        if self.enter_counted(MAJOR_ARRAY)? != 2 {
            return Err(ParseError::Parse);
        }
        Ok([self.uint64()?, self.uint64()?])
    }
}

/// IEEE half to single, as `_Float16`/`_mm_cvtph_ps` convert it.
fn half_to_float(h: u16) -> f32 {
    let sign = u32::from(h >> 15) << 31;
    let exp = u32::from((h >> 10) & 0x1f);
    let mant = u32::from(h & 0x3ff);
    let bits = match (exp, mant) {
        (0, 0) => sign,
        (0, m) => {
            // Subnormal: normalise.
            let shift = m.leading_zeros() - 21;
            sign | ((113 - shift) << 23) | (((m << shift) & 0x3ff) << 13)
        }
        (0x1f, m) => sign | 0x7f80_0000 | (m << 13),
        (e, m) => sign | ((e + 112) << 23) | (m << 13),
    };
    f32::from_bits(bits)
}

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

/// stream2.h `struct stream2_compression`, for the two algorithms parsed.
#[derive(Debug, Clone, PartialEq)]
pub struct Compression {
    pub algorithm: String,
    pub elem_size: u64,
    pub orig_size: u64,
}

/// stream2.h `struct stream2_multidim_array`.
#[derive(Debug, Clone, PartialEq)]
pub struct MultidimArray {
    /// `[rows, columns]`, row-major.
    pub dim: [u64; 2],
    /// The RFC 8746 typed-array tag.
    pub tag: u64,
    /// The array's bytes within the message.
    pub bytes: Range<usize>,
    pub compression: Option<Compression>,
}

/// One channel of an image message's `data` map.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageData {
    pub channel: String,
    pub data: MultidimArray,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StartMsg {
    /// `(channel, energy)` of `threshold_energy`, in map order.
    pub threshold_energy: Vec<(String, f64)>,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct ImageMsg {
    pub series_date: Option<String>,
    pub start_time: [u64; 2],
    pub data: Vec<ImageData>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Msg {
    Start(StartMsg),
    Image(ImageMsg),
    End,
}

impl Msg {
    /// stream2.h `enum stream2_msg_type`.
    fn type_code(&self) -> i32 {
        match self {
            Self::Start(_) => 0,
            Self::Image(_) => 1,
            Self::End => 2,
        }
    }
}

/// stream2.c `stream2_typed_array_elem_size`.
fn typed_array_elem_size(tag: u64) -> PResult<u64> {
    if (64..=87).contains(&tag) {
        let f = (tag >> 4) & 1;
        let ll = tag & 3;
        Ok(1 << (f + ll))
    } else {
        Err(ParseError::NotImplemented)
    }
}

/// `parse_dectris_compression`, entered at its tag.
fn dectris_compression(r: &mut Reader) -> PResult<(Compression, Range<usize>)> {
    if r.tag()? != DECTRIS_COMPRESSION {
        return Err(ParseError::Parse);
    }
    if r.enter_counted(MAJOR_ARRAY)? != 3 {
        return Err(ParseError::Parse);
    }
    let algorithm = r.text()?;
    let elem_size = r.uint64()?;
    let bytes = r.byte_range()?;
    let orig_size = match algorithm.as_str() {
        "bslz4" | "lz4" => {
            if bytes.len() < 12 {
                return Err(ParseError::Decode);
            }
            u64::from_be_bytes(r.buf[bytes.start..bytes.start + 8].try_into().unwrap())
        }
        _ => return Err(ParseError::NotImplemented),
    };
    Ok((
        Compression {
            algorithm,
            elem_size,
            orig_size,
        },
        bytes,
    ))
}

/// `parse_bytes`.
fn bytes_of(r: &mut Reader) -> PResult<(Option<Compression>, Range<usize>)> {
    if r.major()? == MAJOR_TAG {
        let save = r.pos;
        let tag = r.tag()?;
        r.pos = save;
        if tag != DECTRIS_COMPRESSION {
            return Err(ParseError::Parse);
        }
        let (c, b) = dectris_compression(r)?;
        return Ok((Some(c), b));
    }
    Ok((None, r.byte_range()?))
}

/// `parse_typed_array`: `(tag, compression, bytes, element count)`.
fn typed_array(r: &mut Reader) -> PResult<(u64, Option<Compression>, Range<usize>, u64)> {
    let tag = r.tag()?;
    let (compression, bytes) = bytes_of(r)?;
    let elem_size = typed_array_elem_size(tag)?;
    let size = match &compression {
        None => bytes.len() as u64,
        Some(c) => c.orig_size,
    };
    if size % elem_size != 0 {
        return Err(ParseError::Parse);
    }
    Ok((tag, compression, bytes, size / elem_size))
}

/// `parse_multidim_array`.
fn multidim_array(r: &mut Reader) -> PResult<MultidimArray> {
    if r.tag()? != MULTI_DIMENSIONAL_ARRAY_ROW_MAJOR {
        return Err(ParseError::Parse);
    }
    if r.enter_counted(MAJOR_ARRAY)? != 2 {
        return Err(ParseError::Parse);
    }
    let dim = r.array_2_uint64()?;
    let (tag, compression, bytes, len) = typed_array(r)?;
    if dim[0].wrapping_mul(dim[1]) != len {
        return Err(ParseError::Parse);
    }
    Ok(MultidimArray {
        dim,
        tag,
        bytes,
        compression,
    })
}

/// Iterate a map C walks with `cbor_value_is_valid` (definite or indefinite),
/// skipping each value's tags first unless `keep_tag` says not to.
fn fields(
    r: &mut Reader,
    keep_tag: impl Fn(&[u8]) -> bool,
    f: impl FnMut(&mut Reader, &[u8]) -> PResult<bool>,
) -> PResult<()> {
    let mut len = r.enter(MAJOR_MAP)?;
    msg_fields(r, &mut len, keep_tag, f)
}

fn goniometer_axis(r: &mut Reader) -> PResult<()> {
    fields(
        r,
        |_| false,
        |r, key| {
            Ok(match key {
                b"increment" | b"start" => {
                    r.double()?;
                    true
                }
                _ => false,
            })
        },
    )
}

/// `channel -> multidim array` maps (`flatfield`, `pixel_mask`, `data`).
fn channel_arrays(r: &mut Reader, mut f: impl FnMut(String, MultidimArray)) -> PResult<()> {
    let n = r.enter_counted(MAJOR_MAP)?;
    for _ in 0..n {
        let channel = r.text()?;
        f(channel, multidim_array(r)?);
    }
    Ok(())
}

fn start_msg(r: &mut Reader, top: &mut Len) -> PResult<Msg> {
    let mut msg = StartMsg {
        threshold_energy: Vec::new(),
    };
    msg_fields(
        r,
        top,
        |key| key == b"countrate_correction_lookup_table",
        |r, key| {
            match key {
                b"series_id" | b"image_size_x" | b"image_size_y" | b"number_of_images"
                | b"saturation_value" => {
                    r.uint64()?;
                }
                b"series_unique_id"
                | b"arm_date"
                | b"detector_description"
                | b"detector_serial_number"
                | b"image_dtype"
                | b"sensor_material" => {
                    r.text()?;
                }
                b"beam_center_x"
                | b"beam_center_y"
                | b"count_time"
                | b"frame_time"
                | b"incident_energy"
                | b"incident_wavelength"
                | b"pixel_size_x"
                | b"pixel_size_y"
                | b"sensor_thickness" => {
                    r.double()?;
                }
                b"countrate_correction_enabled"
                | b"flatfield_enabled"
                | b"pixel_mask_enabled"
                | b"virtual_pixel_interpolation_enabled" => {
                    r.bool()?;
                }
                b"channels" => {
                    for _ in 0..r.enter_counted(MAJOR_ARRAY)? {
                        r.text()?;
                    }
                }
                b"countrate_correction_lookup_table" => {
                    typed_array(r)?;
                }
                b"detector_translation" => {
                    if r.enter_counted(MAJOR_ARRAY)? != 3 {
                        return Err(ParseError::Parse);
                    }
                    for _ in 0..3 {
                        r.double()?;
                    }
                }
                b"flatfield" | b"pixel_mask" => channel_arrays(r, |_, _| {})?,
                b"goniometer" => fields(
                    r,
                    |_| false,
                    |r, key| {
                        Ok(match key {
                            b"chi" | b"kappa" | b"omega" | b"phi" | b"two_theta" => {
                                goniometer_axis(r)?;
                                true
                            }
                            _ => false,
                        })
                    },
                )?,
                b"threshold_energy" => {
                    for _ in 0..r.enter_counted(MAJOR_MAP)? {
                        let channel = r.text()?;
                        let energy = r.double()?;
                        msg.threshold_energy.push((channel, energy));
                    }
                }
                b"user_data" => r.skip()?,
                _ => return Ok(false),
            }
            Ok(true)
        },
    )?;
    Ok(Msg::Start(msg))
}

fn image_msg(r: &mut Reader, top: &mut Len) -> PResult<Msg> {
    let mut msg = ImageMsg::default();
    msg_fields(
        r,
        top,
        |_| false,
        |r, key| {
            match key {
                b"series_id" | b"image_id" => {
                    r.uint64()?;
                }
                b"series_unique_id" => {
                    r.text()?;
                }
                b"real_time" | b"stop_time" => {
                    r.array_2_uint64()?;
                }
                b"series_date" => msg.series_date = Some(r.text()?),
                b"start_time" => msg.start_time = r.array_2_uint64()?,
                b"user_data" => r.skip()?,
                b"data" => channel_arrays(r, |channel, data| {
                    msg.data.push(ImageData { channel, data })
                })?,
                _ => return Ok(false),
            }
            Ok(true)
        },
    )?;
    Ok(Msg::Image(msg))
}

fn end_msg(r: &mut Reader, top: &mut Len) -> PResult<Msg> {
    msg_fields(
        r,
        top,
        |_| false,
        |r, key| {
            match key {
                b"series_id" => {
                    r.uint64()?;
                }
                b"series_unique_id" => {
                    r.text()?;
                }
                _ => return Ok(false),
            }
            Ok(true)
        },
    )?;
    Ok(Msg::End)
}

/// The entries of a map already entered as `remaining`; `f` returns whether
/// it consumed the value, else the value is skipped.
fn msg_fields(
    r: &mut Reader,
    remaining: &mut Len,
    keep_tag: impl Fn(&[u8]) -> bool,
    mut f: impl FnMut(&mut Reader, &[u8]) -> PResult<bool>,
) -> PResult<()> {
    // A map entry is two items; `more` counts them one at a time.
    while r.more(remaining)? {
        let key = match r.key()? {
            Some(key) => key,
            None => {
                // The key is still unread, so it is the item the value slot
                // skips, and the real value is read as the next key.
                if let Len::Definite(n) = remaining {
                    *n += 1;
                }
                Vec::new()
            }
        };
        if !keep_tag(&key) {
            r.skip_tags()?;
        }
        if !r.more(remaining)? {
            return Err(ParseError::Decode);
        }
        if !f(r, &key)? {
            r.skip()?;
        }
    }
    Ok(())
}

/// stream2.c `stream2_parse_msg`.
pub fn parse_msg(buf: &[u8]) -> PResult<Msg> {
    // https://www.rfc-editor.org/rfc/rfc8949.html#name-self-described-cbor
    const MAGIC: [u8; 3] = [0xd9, 0xd9, 0xf7];
    if !buf.starts_with(&MAGIC) {
        return Err(ParseError::Signature);
    }
    let mut r = Reader {
        buf,
        pos: MAGIC.len(),
    };
    let mut top = r.enter(MAJOR_MAP)?;

    // parse_msg_type: the first entry must be `type`.
    if !r.more(&mut top)? {
        return Err(ParseError::Parse);
    }
    if r.key()?.as_deref() != Some(b"type".as_slice()) {
        return Err(ParseError::Parse);
    }
    r.skip_tags()?;
    if !r.more(&mut top)? {
        return Err(ParseError::Decode);
    }
    match r.key()?.unwrap_or_default().as_slice() {
        b"start" => start_msg(&mut r, &mut top),
        b"image" => image_msg(&mut r, &mut top),
        b"end" => end_msg(&mut r, &mut top),
        _ => Err(ParseError::Parse),
    }
}

// ---------------------------------------------------------------------------
// RFC 3339 (eigerApp/rfc3339Src/rfc3339.h)
// ---------------------------------------------------------------------------

/// POSIX seconds of 1990-01-01, the EPICS epoch.
const POSIX_TIME_AT_EPICS_EPOCH: i64 = 631_152_000;

/// `timegm`, with its normalisation of out-of-range fields.
fn timegm(year: i64, mon0: i64, mday: i64, hour: i64, min: i64, sec: i64) -> i64 {
    let y = year + mon0.div_euclid(12);
    let m = mon0.rem_euclid(12) + 1;
    // days_from_civil (H. Hinnant).
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468 + (mday - 1);
    days * 86_400 + hour * 3600 + min * 60 + sec
}

/// `rfc3339::parseRfc3339Timestamp`, as POSIX seconds plus nanoseconds, or
/// `None` where C returns `rfc3339::ZERO`.
fn parse_rfc3339(s: &str) -> Option<(i64, u32)> {
    let b = s.as_bytes();
    let num = |r: Range<usize>| -> Option<i64> {
        let d = b.get(r)?;
        d.iter()
            .all(u8::is_ascii_digit)
            .then(|| d.iter().fold(0i64, |a, &c| a * 10 + i64::from(c - b'0')))
    };
    let at = |i: usize, c: &[u8]| b.get(i).is_some_and(|x| c.contains(x));

    let year = num(0..4)?;
    let month = num(5..7)?;
    let day = num(8..10)?;
    let hour = num(11..13)?;
    let minute = num(14..16)?;
    let second = num(17..19)?;
    if !(at(4, b"-") && at(7, b"-") && at(10, b"Tt ") && at(13, b":") && at(16, b":")) {
        return None;
    }

    let mut pos = 19;
    let mut nsec = 0u32;
    if at(pos, b".") {
        let digits = b[pos + 1..]
            .iter()
            .take_while(|c| c.is_ascii_digit())
            .count();
        if !(1..=9).contains(&digits) {
            return None;
        }
        let frac = num(pos + 1..pos + 1 + digits)? as u32;
        nsec = frac * 10u32.pow(9 - digits as u32);
        pos += 1 + digits;
    }

    let mut offset = 0i64;
    if at(pos, b"Z") {
        pos += 1;
    } else if at(pos, b"+-") {
        let sign = if b[pos] == b'-' { -1 } else { 1 };
        let h = num(pos + 1..pos + 3)?;
        let m = num(pos + 4..pos + 6)?;
        if !at(pos + 3, b":") {
            return None;
        }
        offset = sign * (h * 3600 + m * 60);
        pos += 6;
    }
    if pos != b.len() {
        return None;
    }

    Some((
        timegm(year, month - 1, day, hour, minute, second) - offset,
        nsec,
    ))
}

/// `epicsTime(gm_tm_nano_sec)`: POSIX seconds to the EPICS epoch, truncated to
/// `epicsUInt32` as `epicsTimeFromTime_t` stores it.
fn epics_from_posix(posix: i64, nsec: u32) -> EpicsTimestamp {
    EpicsTimestamp {
        sec: (posix - POSIX_TIME_AT_EPICS_EPOCH) as u32,
        nsec,
    }
}

/// C's `rfc3339::ZERO` (1900-01-00 00:00:00), converted as C converts it.
fn zero_timestamp() -> EpicsTimestamp {
    epics_from_posix(timegm(1900, 0, 0, 0, 0, 0), 0)
}

// ---------------------------------------------------------------------------
// Stream2API
// ---------------------------------------------------------------------------

/// What `Stream2API::getHeader`/`waitFrame` return besides success.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stream2Error {
    /// `STREAM_TIMEOUT`.
    Timeout,
    /// `STREAM_ERROR`.
    Error,
    /// A `stream2_result` from the parser.
    Parse(ParseError),
}

impl Stream2Error {
    /// The int C's `streamTask` compares against streamApi.h's codes
    /// (`STREAM_SUCCESS` 0, `STREAM_TIMEOUT` 1, `STREAM_WRONG_HTYPE` 2,
    /// `STREAM_ERROR` 3). A parse failure returns its `stream2_result` through
    /// the same int, so `OUT_OF_MEMORY` reads as a timeout, `SIGNATURE` as a
    /// stray packet and `DECODE` as an error.
    pub fn code(self) -> i32 {
        match self {
            Self::Timeout => 1,
            Self::Error => 3,
            Self::Parse(e) => e as i32,
        }
    }
}

/// What `waitFrame` saw.
pub enum Frame {
    /// An image message with this many thresholds (`data` channels).
    Image(usize),
    End,
}

/// One threshold's frame, ready to publish.
pub struct Stream2Frame {
    /// `[x, y]` (C's `dims[0] = dim[1]`, `dims[1] = dim[0]`), filled, with a
    /// codec when it is passed through still compressed.
    pub array: DriverArray,
    pub timestamp: Option<EpicsTimestamp>,
    /// `ThresholdNumber` and `ThresholdEnergy`, from the start message's entry
    /// at this threshold's index.
    pub threshold: Option<(i32, f64)>,
}

/// The failure was printed in C's words; there is nothing more to say.
#[derive(Debug)]
pub struct Reported;

fn err(function: &str, msg: &str) -> Reported {
    epics_rs::base::stderr_println!("Stream2Api::{function}: {msg}");
    Reported
}

/// ZeroMQ PULL socket on the detector's stream2 port (C `Stream2API`).
pub struct Stream2Api {
    socket: zeromq::PullSocket,
    /// Frames of a multipart message not yet read. C's `zmq_msg_recv` reads
    /// one frame per call.
    pending: VecDeque<Bytes>,
    threshold_energy: Vec<(String, f64)>,
    image: Option<(Bytes, ImageMsg)>,
    /// `mCachedTs`: the last `series_date` and what it converted to. C starts
    /// it at an empty string with `ts` uninitialised, so a first empty
    /// `series_date` is a cache hit there; here that hit reads 0.
    cached_ts: (String, EpicsTimestamp),
}

impl Stream2Api {
    pub async fn connect(hostname: &str) -> Result<Self, String> {
        Self::connect_to(&format!("tcp://{hostname}:{ZMQ_PORT}")).await
    }

    pub(crate) async fn connect_to(addr: &str) -> Result<Self, String> {
        use zeromq::Socket;
        let mut socket = zeromq::PullSocket::new();
        socket
            .connect(addr)
            .await
            .map_err(|e| format!("connect {addr}: {e}"))?;
        Ok(Self {
            socket,
            pending: VecDeque::new(),
            threshold_energy: Vec::new(),
            image: None,
            cached_ts: (String::new(), EpicsTimestamp { sec: 0, nsec: 0 }),
        })
    }

    /// A Stream2Api with `messages` already received and no peer.
    #[cfg(test)]
    pub(crate) fn with_received(messages: Vec<Vec<u8>>) -> Self {
        Self {
            socket: <zeromq::PullSocket as zeromq::Socket>::new(),
            pending: messages.into_iter().map(Bytes::from).collect(),
            threshold_energy: Vec::new(),
            image: None,
            cached_ts: (String::new(), EpicsTimestamp { sec: 0, nsec: 0 }),
        }
    }

    /// `poll` plus `zmq_msg_recv` of one frame.
    async fn recv(&mut self, timeout: std::time::Duration) -> Result<Bytes, Stream2Error> {
        use zeromq::SocketRecv;
        if let Some(b) = self.pending.pop_front() {
            return Ok(b);
        }
        let msg = match tokio::time::timeout(timeout, self.socket.recv()).await {
            Err(_) => return Err(Stream2Error::Timeout),
            Ok(Ok(m)) => m,
            Ok(Err(_)) => {
                err("poll", "failed to poll socket");
                return Err(Stream2Error::Error);
            }
        };
        self.pending = msg.into_vecdeque();
        self.pending.pop_front().ok_or(Stream2Error::Error)
    }

    fn parse(buf: &[u8]) -> Result<Msg, Stream2Error> {
        parse_msg(buf).map_err(|e| {
            epics_rs::base::stderr_println!("error: error {} parsing message", e as i32);
            Stream2Error::Parse(e)
        })
    }

    /// C `Stream2API::getHeader`.
    pub async fn get_header(&mut self, timeout: std::time::Duration) -> Result<(), Stream2Error> {
        let buf = self.recv(timeout).await?;
        match Self::parse(&buf)? {
            Msg::Start(sm) => {
                self.threshold_energy = sm.threshold_energy;
                Ok(())
            }
            other => {
                err(
                    "getHeader",
                    &format!(
                        "unexpected message type, should be STREAM2_MSG_START (0), actual={}",
                        other.type_code()
                    ),
                );
                Err(Stream2Error::Error)
            }
        }
    }

    /// C `Stream2API::waitFrame`.
    pub async fn wait_frame(
        &mut self,
        timeout: std::time::Duration,
    ) -> Result<Frame, Stream2Error> {
        let buf = self.recv(timeout).await?;
        match Self::parse(&buf)? {
            Msg::Image(im) => {
                let n = im.data.len();
                self.image = Some((buf, im));
                Ok(Frame::Image(n))
            }
            Msg::End => Ok(Frame::End),
            Msg::Start(_) => Err(Stream2Error::Error),
        }
    }

    /// C `Stream2API::getFrame`: the frame in an array from `pool`, its
    /// pixels written straight into it as C writes `pData`. `Err` after
    /// printing C's line; the image message is released after its last
    /// threshold either way.
    pub fn get_frame(
        &mut self,
        thresh: usize,
        decompress: bool,
        signed: bool,
        extract_timestamp: bool,
        pool: &DriverPool,
    ) -> Result<Stream2Frame, Reported> {
        let result = self.frame(thresh, decompress, signed, extract_timestamp, pool);
        if self
            .image
            .as_ref()
            .is_some_and(|(_, im)| thresh + 1 >= im.data.len())
        {
            self.image = None;
        }
        result
    }

    fn frame(
        &mut self,
        thresh: usize,
        decompress: bool,
        signed: bool,
        extract_timestamp: bool,
        pool: &DriverPool,
    ) -> Result<Stream2Frame, Reported> {
        let Some((buf, im)) = &self.image else {
            return Err(Reported);
        };
        let Some(sid) = im.data.get(thresh).map(|d| &d.data) else {
            return Err(Reported);
        };
        let (series_date, start_time) = (im.series_date.clone().unwrap_or_default(), im.start_time);
        let fname = "getFrame";
        let dims = [sid.dim[1] as usize, sid.dim[0] as usize];
        let data_type = match sid.tag {
            TYPED_ARRAY_UINT8 => NDDataType::UInt8,
            TYPED_ARRAY_UINT16_LITTLE_ENDIAN => NDDataType::UInt16,
            TYPED_ARRAY_UINT32_LITTLE_ENDIAN => NDDataType::UInt32,
            other => {
                return Err(err(fname, &format!("unknown dataType {other}")));
            }
        };
        let blob = &buf[sid.bytes.clone()];
        let nd_dims = vec![NDDimension::new(dims[0]), NDDimension::new(dims[1])];
        let refused = |_| err(fname, "failed to allocate NDArray for frame");

        let array = match &sid.compression {
            Some(c) if !decompress => {
                let (name, payload) = if c.algorithm == "lz4" {
                    (CodecName::LZ4HDF5, blob)
                } else {
                    (CodecName::BSLZ4, &blob[bslz4::HEADER_LEN..])
                };
                let codec = Codec {
                    name,
                    compressed_size: payload.len(),
                    level: 0,
                    shuffle: 0,
                    compressor: 0,
                    // SignedData relabels the type the codec plugin
                    // decompresses to.
                    original_data_type: frame_type(data_type, signed),
                };
                // A compressed frame holds its payload bytes, which its dims
                // do not size.
                let mut array = pool
                    .alloc_sized(nd_dims, NDDataType::UInt8, payload.len())
                    .map_err(refused)?;
                let mut fill = array.fill();
                fill.data_mut().as_u8_slice_mut().copy_from_slice(payload);
                fill.set_codec(Some(codec));
                array
            }
            compression => {
                // parse_msg holds the bytes, or the compressed orig_size, to
                // the dims, so this is the dims' size without their product.
                let bytes = compression
                    .as_ref()
                    .map_or(blob.len(), |c| c.orig_size as usize);
                let mut array = pool
                    .alloc_sized(nd_dims, frame_type(data_type, signed), bytes)
                    .map_err(refused)?;
                let mut fill = array.fill();
                let out = fill.data_mut().as_u8_slice_mut();
                match compression {
                    None => out.copy_from_slice(blob),
                    Some(c) => uncompress(blob, c, dims, data_type, out)?,
                }
                le_to_host(fill.data_mut().view_mut());
                array
            }
        };

        let timestamp = extract_timestamp.then(|| self.timestamp_from(&series_date, start_time));

        let threshold = self
            .threshold_energy
            .get(thresh)
            .map(|(channel, energy)| (threshold_number(channel), *energy));

        Ok(Stream2Frame {
            array,
            timestamp,
            threshold,
        })
    }

    /// C `extractTimeStampFromMessage`.
    fn timestamp_from(&mut self, series_date: &str, start_time: [u64; 2]) -> EpicsTimestamp {
        const NSEC_PER_SEC: u64 = 1_000_000_000;
        let series_ts = match &self.cached_ts {
            (s, ts) if s == series_date => *ts,
            _ => {
                let ts = match parse_rfc3339(series_date) {
                    Some((posix, nsec)) => epics_from_posix(posix, nsec),
                    None => {
                        err(
                            "extractTimeStampFromMessage",
                            &format!(
                                "Failed to parse timestamp '{series_date}' as an RFC3339-compliant timestamp"
                            ),
                        );
                        zero_timestamp()
                    }
                };
                self.cached_ts = (series_date.to_string(), ts);
                ts
            }
        };
        // C divides by start_time[1] unchecked; a zero clock frequency is
        // SIGFPE there. Here it counts as no elapsed time.
        let elapsed_ns = NSEC_PER_SEC
            .wrapping_mul(start_time[0])
            .checked_div(start_time[1])
            .unwrap_or(0);
        let mut frame_sec = u64::from(series_ts.sec);
        let mut frame_nsec = u64::from(series_ts.nsec).wrapping_add(elapsed_ns);
        frame_sec = frame_sec.wrapping_add(frame_nsec / NSEC_PER_SEC);
        frame_nsec %= NSEC_PER_SEC;
        EpicsTimestamp {
            sec: frame_sec as u32,
            nsec: frame_nsec as u32,
        }
    }
}

/// `sscanf(channel, "threshold_%d", &n)`; C leaves `n` unset when the scan
/// fails, here it is 0.
fn threshold_number(channel: &str) -> i32 {
    let Some(rest) = channel.strip_prefix("threshold_") else {
        return 0;
    };
    let rest = rest.trim_start();
    let (neg, digits) = match rest.as_bytes().first() {
        Some(b'-') => (true, &rest[1..]),
        Some(b'+') => (false, &rest[1..]),
        _ => (false, rest),
    };
    let n = digits
        .bytes()
        .take_while(u8::is_ascii_digit)
        .fold(0i32, |a, c| {
            a.wrapping_mul(10).wrapping_add(i32::from(c - b'0'))
        });
    if neg { n.wrapping_neg() } else { n }
}

/// stream2Api.cpp `uncompress`, into `out`.
///
/// UPSTREAM DEFECT (stream2Api.cpp:303): `getFrame` ignores what `uncompress`
/// returns, so a failed decode publishes the uninitialised NDArray. Here the
/// frame is dropped after C's line.
fn uncompress(
    blob: &[u8],
    c: &Compression,
    dims: [usize; 2],
    data_type: NDDataType,
    out: &mut [u8],
) -> Result<(), Reported> {
    let fname = "uncompress";
    if c.algorithm == "lz4" {
        let mut src = NDArray::with_data(
            vec![NDDimension::new(dims[0]), NDDimension::new(dims[1])],
            NDDataBuffer::from_slice(blob),
        );
        src.codec = Some(Codec {
            name: CodecName::LZ4HDF5,
            compressed_size: blob.len(),
            level: 0,
            shuffle: 0,
            compressor: 0,
            original_data_type: data_type,
        });
        match ad_plugins_rs::codec::decompress_lz4hdf5(&src) {
            Ok(decoded) => {
                out.copy_from_slice(decoded.data().as_u8_slice());
                Ok(())
            }
            Err(e) => Err(err(
                fname,
                &format!("decompress_lz4hdf5 failed, result={e:?}\n"),
            )),
        }
    } else {
        bslz4::decode_blocks_into(&blob[bslz4::HEADER_LEN..], out, data_type.element_size(), 0)
            .map_err(|e| err(fname, &format!("bshuf_decompress_lz4 failed, result={e}")))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A minimal CBOR writer for the fixtures.
    #[derive(Default)]
    struct W(Vec<u8>);

    impl W {
        fn head(&mut self, major: u8, n: u64) -> &mut Self {
            let m = major << 5;
            match n {
                0..=23 => self.0.push(m | n as u8),
                24..=0xff => self.0.extend([m | 24, n as u8]),
                0x100..=0xffff => {
                    self.0.push(m | 25);
                    self.0.extend((n as u16).to_be_bytes());
                }
                0x1_0000..=0xffff_ffff => {
                    self.0.push(m | 26);
                    self.0.extend((n as u32).to_be_bytes());
                }
                _ => {
                    self.0.push(m | 27);
                    self.0.extend(n.to_be_bytes());
                }
            }
            self
        }
        fn magic(&mut self) -> &mut Self {
            self.0.extend([0xd9, 0xd9, 0xf7]);
            self
        }
        fn map(&mut self, n: u64) -> &mut Self {
            self.head(MAJOR_MAP, n)
        }
        fn array(&mut self, n: u64) -> &mut Self {
            self.head(MAJOR_ARRAY, n)
        }
        fn uint(&mut self, n: u64) -> &mut Self {
            self.head(MAJOR_UINT, n)
        }
        fn tag(&mut self, n: u64) -> &mut Self {
            self.head(MAJOR_TAG, n)
        }
        fn text(&mut self, s: &str) -> &mut Self {
            self.head(MAJOR_TEXT, s.len() as u64);
            self.0.extend(s.as_bytes());
            self
        }
        fn bytes(&mut self, b: &[u8]) -> &mut Self {
            self.head(MAJOR_BYTES, b.len() as u64);
            self.0.extend(b);
            self
        }
        fn double(&mut self, v: f64) -> &mut Self {
            self.0.push(0xfb);
            self.0.extend(v.to_be_bytes());
            self
        }
        /// `tag 40 [[rows, cols], tag <typed> <payload>]`.
        fn multidim(&mut self, rows: u64, cols: u64, typed: u64) -> &mut Self {
            self.tag(40)
                .array(2)
                .array(2)
                .uint(rows)
                .uint(cols)
                .tag(typed)
        }
    }

    pub(crate) fn start(thresholds: &[(&str, f64)]) -> Vec<u8> {
        let mut w = W::default();
        w.magic().map(5).text("type").text("start");
        w.text("series_id").uint(7);
        w.text("image_dtype").text("uint16");
        w.text("goniometer")
            .map(1)
            .text("omega")
            .map(2)
            .text("start")
            .double(1.5)
            .text("increment")
            .double(0.1);
        w.text("threshold_energy").map(thresholds.len() as u64);
        for (c, e) in thresholds {
            w.text(c).double(*e);
        }
        w.0
    }

    pub(crate) fn image(channels: &[(&str, &[u16])], rows: u64, cols: u64) -> Vec<u8> {
        let mut w = W::default();
        w.magic().map(4).text("type").text("image");
        w.text("series_date").text("2025-12-05T12:34:56.5Z");
        w.text("start_time")
            .array(2)
            .uint(25_000_000)
            .uint(50_000_000);
        w.text("data").map(channels.len() as u64);
        for (c, px) in channels {
            let raw: Vec<u8> = px.iter().flat_map(|p| p.to_le_bytes()).collect();
            w.text(c).multidim(rows, cols, 69).bytes(&raw);
        }
        w.0
    }

    #[test]
    fn a_start_message_carries_its_thresholds() {
        let msg = parse_msg(&start(&[("threshold_1", 6000.0), ("threshold_2", 9000.5)])).unwrap();
        assert_eq!(
            msg,
            Msg::Start(StartMsg {
                threshold_energy: vec![
                    ("threshold_1".into(), 6000.0),
                    ("threshold_2".into(), 9000.5)
                ]
            })
        );
    }

    #[test]
    fn an_image_message_points_at_each_channel() {
        let buf = image(&[("threshold_1", &[1, 2, 3, 4, 5, 6])], 2, 3);
        let Msg::Image(im) = parse_msg(&buf).unwrap() else {
            panic!("expected an image");
        };
        assert_eq!(im.start_time, [25_000_000, 50_000_000]);
        assert_eq!(im.data.len(), 1);
        let d = &im.data[0].data;
        assert_eq!(d.dim, [2, 3]);
        assert_eq!(d.tag, 69);
        assert_eq!(buf[d.bytes.clone()].len(), 12);
    }

    #[test]
    fn an_end_message_ends() {
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("end")
            .text("series_id")
            .uint(1);
        assert_eq!(parse_msg(&w.0).unwrap(), Msg::End);
    }

    #[test]
    fn result_codes_follow_stream2_c() {
        // No self-described CBOR magic.
        assert_eq!(parse_msg(b"{}").unwrap_err(), ParseError::Signature);
        // The first key is not `type`.
        let mut w = W::default();
        w.magic().map(1).text("series_id").uint(1);
        assert_eq!(parse_msg(&w.0).unwrap_err(), ParseError::Parse);
        // An unknown message type.
        let mut w = W::default();
        w.magic().map(1).text("type").text("dheader");
        assert_eq!(parse_msg(&w.0).unwrap_err(), ParseError::Parse);
        // A known key with the wrong type.
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("end")
            .text("series_id")
            .text("x");
        assert_eq!(parse_msg(&w.0).unwrap_err(), ParseError::Parse);
        // Truncated.
        let buf = image(&[("threshold_1", &[1, 2, 3, 4])], 2, 2);
        assert_eq!(
            parse_msg(&buf[..buf.len() - 3]).unwrap_err(),
            ParseError::Decode
        );
        // dim0*dim1 disagrees with the array length.
        let buf = image(&[("threshold_1", &[1, 2, 3, 4])], 3, 2);
        assert_eq!(parse_msg(&buf).unwrap_err(), ParseError::Parse);
        // A compression C does not know.
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("image")
            .text("data")
            .map(1);
        w.text("threshold_1").multidim(1, 1, 64).tag(56500).array(3);
        w.text("zstd").uint(1).bytes(&[0; 12]);
        assert_eq!(parse_msg(&w.0).unwrap_err(), ParseError::NotImplemented);
    }

    #[test]
    fn unknown_keys_and_value_tags_are_skipped() {
        let mut w = W::default();
        w.magic().map(3).text("type").text("end");
        w.text("extra")
            .array(2)
            .map(1)
            .text("a")
            .bytes(&[1, 2])
            .tag(1)
            .uint(3);
        w.text("series_id").tag(99).uint(4);
        assert_eq!(parse_msg(&w.0).unwrap(), Msg::End);
    }

    #[test]
    fn a_long_key_desynchronises_the_map_as_in_c() {
        // C's failed key copy leaves the iterator on the key, skips it as the
        // value, and then reads the value as the next key.
        let long = "k".repeat(MAX_KEY_LEN);
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("end")
            .text(&long)
            .uint(1);
        assert_eq!(parse_msg(&w.0).unwrap_err(), ParseError::Parse);
    }

    #[test]
    fn stream_codes_alias_as_in_c() {
        assert_eq!(Stream2Error::Parse(ParseError::OutOfMemory).code(), 1);
        assert_eq!(Stream2Error::Parse(ParseError::Signature).code(), 2);
        assert_eq!(Stream2Error::Parse(ParseError::Decode).code(), 3);
        assert_eq!(Stream2Error::Parse(ParseError::Parse).code(), 4);
    }

    #[test]
    fn rfc3339_parses_as_rfc3339_h() {
        // 2025-12-05T12:34:56Z
        assert_eq!(
            parse_rfc3339("2025-12-05T12:34:56"),
            Some((1_764_938_096, 0))
        );
        assert_eq!(
            parse_rfc3339("2025-12-05T12:34:56.789Z"),
            Some((1_764_938_096, 789_000_000))
        );
        assert_eq!(
            parse_rfc3339("2025-12-05t12:34:56.789-04:00"),
            Some((1_764_938_096 + 4 * 3600, 789_000_000))
        );
        assert_eq!(
            parse_rfc3339("2025-12-05 12:34:56.123456789+01:30")
                .unwrap()
                .1,
            123_456_789
        );
        assert_eq!(parse_rfc3339("2025-12-05T12:34:56.1234567890"), None);
        assert_eq!(parse_rfc3339("2025-12-05T12:34"), None);
        assert_eq!(parse_rfc3339("2025-12-05T12:34:56+0100"), None);
        assert_eq!(parse_rfc3339(""), None);
    }

    #[test]
    fn an_unparsable_series_date_converts_as_c_converts_zero() {
        // timegm(1900-01-00) - POSIX_TIME_AT_EPICS_EPOCH, truncated to u32.
        assert_eq!(zero_timestamp().sec, 1_454_740_096);
        assert_eq!(zero_timestamp().nsec, 0);
    }

    #[test]
    fn half_floats_widen_exactly() {
        assert_eq!(half_to_float(0x3c00), 1.0);
        assert_eq!(half_to_float(0xc000), -2.0);
        assert_eq!(half_to_float(0x0001), 2f32.powi(-24));
        assert!(half_to_float(0x7c00).is_infinite());
    }

    #[test]
    fn threshold_number_scans_as_sscanf() {
        assert_eq!(threshold_number("threshold_2"), 2);
        assert_eq!(threshold_number("threshold_12x"), 12);
        assert_eq!(threshold_number("threshold_-1"), -1);
        assert_eq!(threshold_number("t1"), 0);
    }

    pub(crate) fn end() -> Vec<u8> {
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("end")
            .text("series_id")
            .uint(7);
        w.0
    }

    fn sizes(array: &DriverArray) -> Vec<usize> {
        array.dims.iter().map(|d| d.size).collect()
    }

    fn assert_same(got: &NDDataBuffer, want: &NDDataBuffer) {
        assert_eq!(got.data_type(), want.data_type());
        assert_eq!(got.as_u8_slice(), want.as_u8_slice());
    }

    /// A Stream2Api whose socket is never used: the frame and timestamp paths
    /// work on the stored messages only.
    async fn api() -> Stream2Api {
        Stream2Api::with_received(Vec::new())
    }

    fn load(api: &mut Stream2Api, buf: Vec<u8>) {
        let Msg::Image(im) = parse_msg(&buf).unwrap() else {
            panic!("expected an image");
        };
        api.image = Some((Bytes::from(buf), im));
    }

    #[tokio::test]
    async fn get_frame_swaps_dims_and_attaches_thresholds_and_time() {
        let mut a = api().await;
        a.threshold_energy = vec![
            ("threshold_1".into(), 6000.0),
            ("threshold_2".into(), 9000.0),
        ];
        let px1: &[u16] = &[1, 2, 3, 4, 5, 6];
        let px2: &[u16] = &[7, 8, 9, 10, 11, 12];
        load(
            &mut a,
            image(&[("threshold_1", px1), ("threshold_2", px2)], 2, 3),
        );

        let pool = DriverPool::standalone(0);
        let f = a.get_frame(0, true, false, true, &pool).unwrap();
        assert_eq!(sizes(&f.array), [3, 2]);
        assert_same(f.array.data(), &NDDataBuffer::from_slice(px1));
        assert_eq!(f.threshold, Some((1, 6000.0)));
        // Series start + 25e6 ticks of a 50 MHz clock.
        let ts = f.timestamp.unwrap();
        assert_eq!(ts.sec as i64, 1_764_938_096 - POSIX_TIME_AT_EPICS_EPOCH + 1);
        assert_eq!(ts.nsec, 0);
        assert!(a.image.is_some(), "kept until its last threshold");

        let f = a.get_frame(1, true, false, false, &pool).unwrap();
        assert_same(f.array.data(), &NDDataBuffer::from_slice(px2));
        assert_eq!(f.threshold, Some((2, 9000.0)));
        assert!(f.timestamp.is_none());
        assert!(a.image.is_none(), "released after its last threshold");
    }

    #[tokio::test]
    async fn get_frame_passes_compressed_data_through_with_its_codec() {
        let pixels: Vec<u32> = (0..64).collect();
        let src = NDArray::with_data(
            vec![NDDimension::new(64)],
            NDDataBuffer::from_vec(pixels.clone()),
        );
        let lz4 = ad_plugins_rs::codec::compress_lz4hdf5(&src, 0).unwrap();
        let blob = lz4.data().as_u8_slice().to_vec();

        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("image")
            .text("data")
            .map(1);
        w.text("threshold_1").multidim(8, 8, 70).tag(56500).array(3);
        w.text("lz4").uint(4).bytes(&blob);

        let mut a = api().await;
        let pool = DriverPool::standalone(0);
        load(&mut a, w.0.clone());
        let f = a.get_frame(0, false, false, false, &pool).unwrap();
        let codec = f.array.codec.clone().unwrap();
        assert_eq!(codec.name, CodecName::LZ4HDF5);
        assert_eq!(codec.original_data_type, NDDataType::UInt32);
        assert_same(f.array.data(), &NDDataBuffer::from_vec(blob.clone()));

        load(&mut a, w.0.clone());
        let f = a.get_frame(0, false, true, false, &pool).unwrap();
        let codec = f.array.codec.clone().unwrap();
        assert_eq!(codec.original_data_type, NDDataType::Int32, "SignedData");

        load(&mut a, w.0);
        let f = a.get_frame(0, true, false, false, &pool).unwrap();
        assert!(f.array.codec.is_none());
        assert_same(f.array.data(), &NDDataBuffer::from_vec(pixels));
    }

    #[tokio::test]
    async fn get_frame_decodes_bslz4_without_its_header() {
        let pixels: Vec<u16> = (0..256u16).map(|i| i % 37).collect();
        let src = NDArray::with_data(
            vec![NDDimension::new(256)],
            NDDataBuffer::from_vec(pixels.clone()),
        );
        let compressed = ad_plugins_rs::codec::compress_bslz4(&src).unwrap();
        let mut blob = Vec::new();
        blob.extend((256u64 * 2).to_be_bytes());
        blob.extend(((bslz4::default_block_size(2) * 2) as u32).to_be_bytes());
        blob.extend(compressed.data().as_u8_slice());

        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("image")
            .text("data")
            .map(1);
        w.text("threshold_1")
            .multidim(16, 16, 69)
            .tag(56500)
            .array(3);
        w.text("bslz4").uint(2).bytes(&blob);

        let mut a = api().await;
        let pool = DriverPool::standalone(0);
        load(&mut a, w.0.clone());
        let f = a.get_frame(0, true, false, false, &pool).unwrap();
        assert_same(f.array.data(), &NDDataBuffer::from_vec(pixels.clone()));

        // SignedData labels the same bits signed.
        load(&mut a, w.0.clone());
        let f = a.get_frame(0, true, true, false, &pool).unwrap();
        let signed: Vec<i16> = pixels.iter().map(|&p| p as i16).collect();
        assert_same(f.array.data(), &NDDataBuffer::from_vec(signed));

        load(&mut a, w.0);
        let f = a.get_frame(0, false, false, false, &pool).unwrap();
        assert_eq!(f.array.codec.as_ref().unwrap().name, CodecName::BSLZ4);
        assert_same(
            f.array.data(),
            &NDDataBuffer::from_vec(blob[bslz4::HEADER_LEN..].to_vec()),
        );
    }

    #[tokio::test]
    async fn an_unknown_typed_array_is_refused_by_get_frame() {
        // Tag 65 (uint16 big-endian) parses but is no NDArray type.
        let mut w = W::default();
        w.magic()
            .map(2)
            .text("type")
            .text("image")
            .text("data")
            .map(1);
        w.text("threshold_1")
            .multidim(1, 2, 65)
            .bytes(&[0, 1, 0, 2]);
        let mut a = api().await;
        load(&mut a, w.0);
        assert!(
            a.get_frame(0, true, false, false, &DriverPool::standalone(0))
                .is_err()
        );
        assert!(a.image.is_none());
    }
}
