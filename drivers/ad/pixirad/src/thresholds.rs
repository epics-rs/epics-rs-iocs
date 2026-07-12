//! Turning the four colour energies plus the hit energy into the register
//! values the box wants (C `calculateThresholds` and `set_closest_Eth_DAC`).

use crate::types::{
    Asic, EXTDAC_LSB, INT_DAC_STEPS, NUM_THRESHOLDS, PIII_P0, PIII_P1, THRESH_A_COEFF,
    THRESH_B_COEFF, THRESHOLD_FRACTIONS, VAGND, VTH1_ACCURACY, VTHMAX_DECR_STEP,
    VTHMAX_LOWER_LIMIT, VTHMAX_UPPER_LIMIT,
};

/// What the box has to be told, and what it will really threshold at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    /// Only meaningful on the Pixie-II.
    pub vth_max: i32,
    pub registers: [i32; NUM_THRESHOLDS],
    pub actual_energy: [f64; NUM_THRESHOLDS],
}

/// The threshold voltage that gives this energy (C `get_vth_from_fit`).
fn vth_from_fit(energy_kev: f64) -> f64 {
    (-THRESH_B_COEFF + (THRESH_B_COEFF.powi(2) + 4.0 * THRESH_A_COEFF * energy_kev).sqrt())
        / (2.0 * THRESH_A_COEFF)
}

/// The energy a threshold voltage corresponds to (C `get_energy_from_fit`).
fn energy_from_fit(vth: f64) -> f64 {
    THRESH_A_COEFF * vth.powi(2) + THRESH_B_COEFF * vth
}

/// The threshold voltage a (VTHMAX, internal-DAC fraction) pair produces
/// (C `get_Vth_from_settings`).
fn vth_from_settings(vth_max: i32, fraction: f64) -> f64 {
    (vth_max as f64 * EXTDAC_LSB - VAGND) * fraction
}

/// The internal-DAC step whose energy is closest to `energy`
/// (C `set_closest_Eth_DAC`).
///
/// C started its scan at index 1 and only ever wrote the caller's `DAC` and
/// `EthSet` inside the loop body, so when step 0 was the closest the caller
/// used an uninitialised register value.
fn closest_dac_step(allowed_energy: &[f64; INT_DAC_STEPS], energy: f64) -> (i32, f64) {
    let mut best = 0usize;
    let mut best_delta = (allowed_energy[0] - energy).abs();
    for (i, e) in allowed_energy.iter().enumerate().skip(1) {
        let delta = (e - energy).abs();
        if delta <= best_delta {
            best_delta = delta;
            best = i;
        }
    }
    (best as i32, allowed_energy[best])
}

/// The (VTHMAX, DAC step) pair whose threshold voltage is closest to `vth1`.
///
/// C's search decremented VTHMAX *after* the inner scan and then handed the
/// decremented value both to the box and to the allowed-energy table, while
/// reporting the energy of the *un*-decremented match: the box was programmed
/// one VTHMAX step away from the setting the search had actually accepted. The
/// match here keeps the VTHMAX that produced it. C also left VTHMAX at
/// `VTHMAX_LOWER_LIMIT - 1` and the DAC step at 31 when nothing matched to
/// within `VTH1_ACCURACY`; the best pair seen is used instead, which is the
/// same answer whenever a match exists.
fn best_vth_max(vth1: f64) -> (i32, usize) {
    let mut best = (VTHMAX_UPPER_LIMIT, 0usize);
    let mut best_delta = f64::INFINITY;

    let mut vth_max = VTHMAX_UPPER_LIMIT;
    while vth_max >= VTHMAX_LOWER_LIMIT {
        for (step, fraction) in THRESHOLD_FRACTIONS.iter().enumerate() {
            let delta = (vth1 - vth_from_settings(vth_max, *fraction)).abs();
            if delta < best_delta {
                best_delta = delta;
                best = (vth_max, step);
            }
            if delta <= VTH1_ACCURACY {
                return best;
            }
        }
        vth_max -= VTHMAX_DECR_STEP;
    }
    best
}

/// The registers for the requested energies (C `calculateThresholds`).
pub fn calculate(asic: Asic, requested_energy: &[f64; NUM_THRESHOLDS]) -> Thresholds {
    let mut registers = [0i32; NUM_THRESHOLDS];
    let mut actual_energy = [0f64; NUM_THRESHOLDS];

    if asic == Asic::PIII {
        // The Pixie-III threshold DAC is linear in energy, so every colour is
        // set independently.
        for i in 0..NUM_THRESHOLDS {
            registers[i] = (PIII_P1 * requested_energy[i] + PIII_P0 + 0.5) as i32;
            actual_energy[i] = (registers[i] as f64 - PIII_P0) / PIII_P1;
        }
        return Thresholds {
            vth_max: 0,
            registers,
            actual_energy,
        };
    }

    // The Pixie-II has one VTHMAX for the whole chip and a 32-step internal DAC
    // per colour, so the first colour picks VTHMAX and the rest have to take
    // the steps it leaves them.
    let vth1 = vth_from_fit(requested_energy[0]);
    let (vth_max, step) = best_vth_max(vth1);
    registers[0] = step as i32;
    actual_energy[0] = energy_from_fit(vth_from_settings(vth_max, THRESHOLD_FRACTIONS[step]));

    let mut allowed_energy = [0f64; INT_DAC_STEPS];
    for (i, e) in allowed_energy.iter_mut().enumerate() {
        *e = energy_from_fit(vth_from_settings(vth_max, THRESHOLD_FRACTIONS[i]));
    }
    for i in 1..NUM_THRESHOLDS {
        let (dac, energy) = closest_dac_step(&allowed_energy, requested_energy[i]);
        registers[i] = dac;
        actual_energy[i] = energy;
    }

    Thresholds {
        vth_max,
        registers,
        actual_energy,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn piii_registers_are_linear_in_energy() {
        let t = calculate(Asic::PIII, &[10.0, 15.0, 20.0, 25.0, 5.0]);
        assert_eq!(t.registers[0], (PIII_P1 * 10.0 + PIII_P0 + 0.5) as i32);
        assert_eq!(t.registers[4], (PIII_P1 * 5.0 + PIII_P0 + 0.5) as i32);
        for i in 0..NUM_THRESHOLDS {
            let round_trip = (t.registers[i] as f64 - PIII_P0) / PIII_P1;
            assert!((t.actual_energy[i] - round_trip).abs() < 1e-9);
        }
    }

    #[test]
    fn pii_vth_max_is_the_one_that_produced_the_match() {
        let t = calculate(Asic::PII, &[10.0, 15.0, 20.0, 25.0, 5.0]);
        assert!(
            (VTHMAX_LOWER_LIMIT..=VTHMAX_UPPER_LIMIT).contains(&t.vth_max),
            "vth_max {} out of range",
            t.vth_max
        );
        // The energy reported for the first colour is the energy of the setting
        // that is actually sent: (vth_max, registers[0]).
        let vth = vth_from_settings(t.vth_max, THRESHOLD_FRACTIONS[t.registers[0] as usize]);
        assert!((t.actual_energy[0] - energy_from_fit(vth)).abs() < 1e-9);
        // ... and it is the energy that was asked for, to the accuracy of the
        // search.
        assert!(
            (t.actual_energy[0] - 10.0).abs() < 0.1,
            "actual {}",
            t.actual_energy[0]
        );
    }

    #[test]
    fn pii_other_colours_snap_to_a_step_of_the_chosen_vth_max() {
        let t = calculate(Asic::PII, &[10.0, 15.0, 20.0, 25.0, 5.0]);
        for i in 1..NUM_THRESHOLDS {
            let step = t.registers[i] as usize;
            assert!(step < INT_DAC_STEPS);
            let vth = vth_from_settings(t.vth_max, THRESHOLD_FRACTIONS[step]);
            assert!((t.actual_energy[i] - energy_from_fit(vth)).abs() < 1e-9);
        }
    }

    #[test]
    fn closest_dac_step_can_pick_step_zero() {
        // The regression C could not report: the table's step 0 is the closest.
        let mut table = [0f64; INT_DAC_STEPS];
        for (i, e) in table.iter_mut().enumerate() {
            *e = i as f64 * 10.0;
        }
        assert_eq!(closest_dac_step(&table, -5.0), (0, 0.0));
        assert_eq!(closest_dac_step(&table, 31.0), (3, 30.0));
    }

    #[test]
    fn an_energy_no_setting_can_reach_still_yields_a_legal_setting() {
        // C walked VTHMAX below its lower limit here and reported it.
        let t = calculate(Asic::PII, &[10_000.0, 0.0, 0.0, 0.0, 0.0]);
        assert!((VTHMAX_LOWER_LIMIT..=VTHMAX_UPPER_LIMIT).contains(&t.vth_max));
        assert!((t.registers[0] as usize) < INT_DAC_STEPS);
    }
}
