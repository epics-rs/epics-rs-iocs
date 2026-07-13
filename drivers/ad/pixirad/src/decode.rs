//! Turning a frame of UDP payload into an image
//! (C `PIXIE_data_utilities.cpp`: `get_pixie_raw_data` and everything it
//! calls).
//!
//! The chip does not send pixels. It sends, bit plane by bit plane, the state
//! of its 16 data outputs; each counter's value is spread over `code_depth`
//! consecutive words, one bit per word. On top of that the counters are
//! pseudo-random: a hardware LFSR counts, and a lookup table maps its state
//! back to a number of pulses. The four steps below are C's, in C's order:
//! gather (with a byte swap, the wire is big-endian), un-bit-stream, un-scramble
//! the counter values, then put the pixels where they belong on the sensor.

use crate::types::{
    Asic, CONVERSION_TABLE_DEPTH, PIII_PRANDOM_07BITS_B0, PIII_PRANDOM_07BITS_B1,
    PIII_PRANDOM_15BITS_B0, PIII_PRANDOM_15BITS_B1, PS_COUNTER_WIDTH, Sensor,
};

/// The frame did not carry as many bytes as this sensor's geometry needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameTooShort {
    pub have: usize,
    pub need: usize,
}

impl std::fmt::Display for FrameTooShort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "the frame carries {} bytes, the sensor needs {}",
            self.have, self.need
        )
    }
}

/// The Pixie-II counter table (C `genera_tabella_clock`, "developed by Sandro").
///
/// The LFSR state after `i` pulses is `tempo`, so `table[tempo] = i` inverts it.
fn pii_conversion_table() -> Vec<u16> {
    let mut table = vec![0u16; CONVERSION_TABLE_DEPTH];
    let mut state: u32 = 0;
    let modulus: u32 = 1 << PS_COUNTER_WIDTH;
    for i in 1..CONVERSION_TABLE_DEPTH {
        let bit1 = (state >> 14) & 1;
        let bit2 = (state >> 6) & 1;
        let feedback = 1 - (bit1 ^ bit2);
        state = (state * 2 + feedback) % modulus;
        table[state as usize] = i as u16;
    }
    table[0] = 0;
    table
}

/// The Pixie-III counter table: generate the LFSR sequence, then invert it
/// (C `GeneratePIIIConversionTable` + `InvertPIIIConversionTable`).
fn piii_conversion_table(code_depth: usize) -> Vec<u16> {
    let (bit0, bit1) = if code_depth == 15 {
        (PIII_PRANDOM_15BITS_B0, PIII_PRANDOM_15BITS_B1)
    } else {
        (PIII_PRANDOM_07BITS_B0, PIII_PRANDOM_07BITS_B1)
    };
    let feedback = [1u32, 0, 0, 1];
    let mask: u32 = (1u32 << code_depth) - 1;

    let mut table = vec![0u16; CONVERSION_TABLE_DEPTH];
    let mut state: u32 = 0;
    for i in 0..(mask as usize).min(CONVERSION_TABLE_DEPTH - 1) {
        let mut b = 0usize;
        if state & (1 << bit0) != 0 {
            b = 1;
        }
        if state & (1 << bit1) != 0 {
            b += 2;
        }
        state = ((state * 2) + feedback[b]) & mask;
        table[i + 1] = state as u16;
    }

    // Invert it: the generated table maps pulses to LFSR state, the decoder
    // needs state to pulses.
    let forward = table.clone();
    for (i, state) in forward.iter().enumerate() {
        table[*state as usize % CONVERSION_TABLE_DEPTH] = i as u16;
    }
    // The generator gives state 0 to both 0 and 32767 pulses; the inversion has
    // to pick, and 0 pulses is the right answer.
    table[0] = 0;
    table
}

/// One counter's worth of bit planes → one counter value per data output
/// (C `convert_bit_stream_to_counts`). Bit plane `i` carries bit
/// `code_depth-1-i` of every output's counter, output `j` in bit `j`.
fn convert_bit_stream_to_counts(code_depth: usize, planes: &[u16], counters: &mut [u16]) {
    for (j, counter) in counters.iter_mut().enumerate() {
        let mut value = 0u16;
        for (i, plane) in planes.iter().enumerate().take(code_depth) {
            if plane & (1 << j) != 0 {
                value |= 1 << (code_depth - 1 - i);
            }
        }
        *counter = value;
    }
}

/// Counter state → pulse count (C `decode_pixie_data_buffer`).
fn decode_counts(table: &[u16], buffer: &mut [u16]) {
    for v in buffer.iter_mut() {
        *v = table[*v as usize % CONVERSION_TABLE_DEPTH];
    }
}

/// The chip reads each sector out backwards and interleaves the sectors, one
/// pixel per data output per clock (C `databuffer_sorting`).
fn databuffer_sorting(buffer: &mut [u16], sensor: &Sensor, scratch: &mut [u16]) {
    let pixels_in_sector = sensor.cols_per_dout * sensor.rows;
    for sector in 0..sensor.dout {
        for pixel in 0..pixels_in_sector {
            scratch[(sector + 1) * pixels_in_sector - pixel - 1] =
                buffer[sector + pixel * sensor.dout];
        }
    }
    buffer.copy_from_slice(&scratch[..buffer.len()]);
}

/// The Pixie-II reads its columns as a snake: odd columns come out reversed
/// (C `map_data_buffer_on_pixie`).
fn map_on_pixie_ii(buffer: &mut [u16], sensor: &Sensor) {
    for col in (1..sensor.cols).step_by(2) {
        buffer[col * sensor.rows..(col + 1) * sensor.rows].reverse();
    }
}

/// The Pixie-III delivers each sector row-major and column-reversed; put it
/// back column-major (C `map_data_buffer_on_pixieIII`).
fn map_on_pixie_iii(buffer: &mut [u16], sensor: &Sensor, scratch: &mut [u16]) {
    let sector_pixels = sensor.rows * sensor.cols_per_dout;
    for sector in 0..sensor.dout {
        let base = sector * sector_pixels;
        for row in 0..sensor.rows {
            for col in 0..sensor.cols_per_dout {
                scratch[row + (sensor.cols_per_dout - 1 - col) * sensor.rows] =
                    buffer[base + row * sensor.cols_per_dout + col];
            }
        }
        buffer[base..base + sector_pixels].copy_from_slice(&scratch[..sector_pixels]);
    }
}

/// Decodes frames of one sensor. The scratch buffers are kept between frames so
/// that a running acquisition allocates nothing.
pub struct Decoder {
    sensor: Sensor,
    table: Vec<u16>,
    planes: Vec<u16>,
    scratch: Vec<u16>,
    image: Vec<u16>,
}

impl Decoder {
    pub fn new(sensor: Sensor) -> Self {
        let table = match sensor.asic {
            Asic::PII => pii_conversion_table(),
            Asic::PIII => piii_conversion_table(sensor.bit_per_cnt_std),
        };
        let counters_per_module = sensor.cols_per_dout * sensor.rows;
        Self {
            table,
            planes: vec![0u16; sensor.modules * counters_per_module * sensor.bit_per_cnt_std],
            scratch: vec![0u16; sensor.matrix_size_pxls],
            image: vec![0u16; sensor.image_pixels()],
            sensor,
        }
    }

    /// How many payload bytes a frame of this kind must carry.
    pub fn frame_bytes(&self, is_autocal: bool) -> usize {
        let s = &self.sensor;
        s.modules * s.cols_per_dout * s.rows * s.code_depth(is_autocal) * 2
    }

    /// Decode one frame; the result is one `u16` per pixel, module after
    /// module, each module column-major (x = the fast axis, `sensor.rows`
    /// long).
    pub fn decode(&mut self, is_autocal: bool, payload: &[u8]) -> Result<&[u16], FrameTooShort> {
        let s = self.sensor;
        let code_depth = s.code_depth(is_autocal);
        let need = self.frame_bytes(is_autocal);
        if payload.len() < need {
            return Err(FrameTooShort {
                have: payload.len(),
                need,
            });
        }

        let counters_per_module = s.cols_per_dout * s.rows;

        // Gather: the modules are interleaved word by word on the wire, and
        // every word arrives big-endian (C's `my_bytes_swap`).
        for module in 0..s.modules {
            for counter in 0..counters_per_module {
                for plane in 0..code_depth {
                    let src = module + counter * s.modules * code_depth + plane * s.modules;
                    let bytes = &payload[src * 2..src * 2 + 2];
                    self.planes[module * counters_per_module * code_depth
                        + counter * code_depth
                        + plane] = u16::from_be_bytes([bytes[0], bytes[1]]);
                }
            }
        }

        for module in 0..s.modules {
            for counter in 0..counters_per_module {
                let planes = &self.planes
                    [module * counters_per_module * code_depth + counter * code_depth..]
                    [..code_depth];
                let counters =
                    &mut self.image[module * s.matrix_size_pxls + counter * s.dout..][..s.dout];
                convert_bit_stream_to_counts(code_depth, planes, counters);
            }
        }

        for module in 0..s.modules {
            let buffer =
                &mut self.image[module * s.matrix_size_pxls..(module + 1) * s.matrix_size_pxls];
            // An autocalibration frame carries the DAC settings the pixels
            // ended up with, not a pulse count, so it is not run through the
            // counter table.
            if !is_autocal {
                decode_counts(&self.table, buffer);
            }
            databuffer_sorting(buffer, &s, &mut self.scratch);
            match s.asic {
                Asic::PII => map_on_pixie_ii(buffer, &s),
                Asic::PIII => map_on_pixie_iii(buffer, &s, &mut self.scratch),
            }
        }

        Ok(&self.image)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Build;

    /// A two-output, two-column-per-output, two-row sensor: small enough to
    /// work out by hand, the same shape as the real one.
    fn tiny_sensor(asic: Asic) -> Sensor {
        Sensor {
            asic,
            build: Build::PX1,
            modules: 1,
            rows: 2,
            cols: 4,
            dout: 2,
            cols_per_dout: 2,
            matrix_size_pxls: 8,
            bit_per_cnt_std: 15,
            autocal_bit_cnt: 3,
            num_udp_packets: 1,
            num_autocal_udp_packets: 1,
        }
    }

    #[test]
    fn bit_planes_become_counters() {
        // Two outputs, three bit planes, most significant plane first. Output 0
        // reads 1, 0, 1 = 0b101 = 5; output 1 reads 0, 1, 1 = 0b011 = 3.
        let planes = [0b01u16, 0b10, 0b11];
        let mut counters = [0u16; 2];
        convert_bit_stream_to_counts(3, &planes, &mut counters);
        assert_eq!(counters, [0b101, 0b011]);
    }

    /// Both counters are 15-bit LFSRs whose sequence visits 32767 of the 32768
    /// states, so the table has to give a distinct pulse count to every state
    /// it can reach. The state it never reaches keeps the table's 0 — the
    /// ambiguity C's own comment describes ("Francois' table assigns 0 to both
    /// 0 and 32767").
    fn assert_table_inverts_the_counter(table: &[u16]) {
        assert_eq!(table.len(), CONVERSION_TABLE_DEPTH);
        assert_eq!(table[0], 0);
        let mut times_reported = vec![0u32; CONVERSION_TABLE_DEPTH];
        for v in table {
            times_reported[*v as usize] += 1;
        }
        assert_eq!(
            times_reported[0], 2,
            "0 pulses: the reset state and the unreachable one"
        );
        for (pulses, count) in times_reported.iter().enumerate().skip(1) {
            let expected = if pulses == CONVERSION_TABLE_DEPTH - 1 {
                0
            } else {
                1
            };
            assert_eq!(
                *count, expected,
                "{pulses} pulses is reported {count} times"
            );
        }
    }

    #[test]
    fn sorting_undoes_the_interleaved_backwards_readout() {
        let sensor = tiny_sensor(Asic::PII);
        // Interleaved by output, and each sector arrives last pixel first.
        let mut buffer: Vec<u16> = vec![0, 10, 1, 11, 2, 12, 3, 13];
        let mut scratch = vec![0u16; 8];
        databuffer_sorting(&mut buffer, &sensor, &mut scratch);
        assert_eq!(buffer, vec![3, 2, 1, 0, 13, 12, 11, 10]);
    }

    #[test]
    fn pixie_ii_mapping_reverses_the_odd_columns() {
        let sensor = tiny_sensor(Asic::PII);
        let mut buffer: Vec<u16> = vec![0, 1, 2, 3, 4, 5, 6, 7];
        map_on_pixie_ii(&mut buffer, &sensor);
        // Columns 1 and 3 (rows 2 each) come out reversed, 0 and 2 do not.
        assert_eq!(buffer, vec![0, 1, 3, 2, 4, 5, 7, 6]);
    }

    #[test]
    fn pixie_iii_mapping_transposes_each_sector() {
        let sensor = tiny_sensor(Asic::PIII);
        let mut buffer: Vec<u16> = vec![0, 1, 2, 3, 4, 5, 6, 7];
        let mut scratch = vec![0u16; 8];
        map_on_pixie_iii(&mut buffer, &sensor, &mut scratch);
        // Sector 0 arrives row-major (rows 2, cols 2) as [0,1 / 2,3]; it must
        // come out column-major with the columns reversed: col 1 first.
        assert_eq!(buffer, vec![1, 3, 0, 2, 5, 7, 4, 6]);
    }

    #[test]
    fn pii_conversion_table_inverts_the_counter() {
        assert_table_inverts_the_counter(&pii_conversion_table());
    }

    #[test]
    fn piii_conversion_table_inverts_the_counter() {
        assert_table_inverts_the_counter(&piii_conversion_table(15));
    }

    /// Build the bytes the chip would send for a given image, then decode them.
    /// Autocalibration frames skip the counter table, so this checks the wire
    /// layout — gather, bit planes, sorting, mapping — end to end.
    #[test]
    fn an_autocal_frame_round_trips_through_the_decoder() {
        let sensor = tiny_sensor(Asic::PIII);
        let code_depth = sensor.autocal_bit_cnt; // 3
        let counters_per_module = sensor.cols_per_dout * sensor.rows; // 4

        // The counter values the chip holds, in readout order.
        let raw: Vec<u16> = vec![1, 2, 3, 4, 5, 6, 7, 0];

        // Encode: counter c of output j lives in raw[c*dout + j].
        let mut payload = vec![0u8; counters_per_module * code_depth * 2];
        for counter in 0..counters_per_module {
            for plane in 0..code_depth {
                let mut word = 0u16;
                for output in 0..sensor.dout {
                    let value = raw[counter * sensor.dout + output];
                    if value & (1 << (code_depth - 1 - plane)) != 0 {
                        word |= 1 << output;
                    }
                }
                let index = counter * code_depth + plane;
                payload[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
            }
        }

        let mut decoder = Decoder::new(sensor);
        let image = decoder.decode(true, &payload).unwrap().to_vec();

        // The same raw values, put through the sort and the map.
        let mut expected = raw.clone();
        let mut scratch = vec![0u16; 8];
        databuffer_sorting(&mut expected, &sensor, &mut scratch);
        map_on_pixie_iii(&mut expected, &sensor, &mut scratch);
        assert_eq!(image, expected);
    }

    #[test]
    fn a_short_frame_is_refused() {
        let mut decoder = Decoder::new(Sensor::from_size(402, 512).unwrap());
        let err = decoder.decode(false, &[0u8; 16]).unwrap_err();
        assert_eq!(err.have, 16);
        assert_eq!(err.need, 402 * 32 * 15 * 2);
    }
}
