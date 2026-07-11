//! Uniform source replacing C `rand() / (double)RAND_MAX`.
//!
//! DEVIATION: the C driver draws from libc `rand()`, whose sequence is
//! platform-specific and whose normalised range is the *closed* interval
//! `[0, 1]`. Nothing in the simulated data depends on the exact sequence — the
//! value only feeds the per-time-point noise term — so this port uses a
//! self-contained xorshift64* generator over the *half-open* `[0, 1)`, giving
//! `rndm` in `[-1, 1)` rather than C's `[-1, 1]`.

/// xorshift64* PRNG. Seeded per detector instance.
pub struct Rng {
    state: u64,
}

impl Rng {
    /// Seed the generator. A zero seed is replaced (xorshift has a fixed point at 0).
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                seed
            },
        }
    }

    /// Seed from the system clock.
    pub fn from_entropy() -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x2545_F491_4F6C_DD1D);
        Self::new(nanos)
    }

    /// Next uniform double in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        let v = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
        // 53 significand bits: exact and uniform on [0, 1).
        (v >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// C `rndm = 2.*(rand()/(double)RAND_MAX - 0.5)` (ADCSimDetector.cpp:166).
    pub fn next_rndm(&mut self) -> f64 {
        2.0 * (self.next_f64() - 0.5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rndm_stays_in_the_half_open_symmetric_interval() {
        let mut rng = Rng::new(12345);
        for _ in 0..10_000 {
            let v = rng.next_rndm();
            assert!((-1.0..1.0).contains(&v), "out of range: {v}");
        }
    }

    #[test]
    fn same_seed_gives_same_sequence() {
        let a: Vec<f64> = (0..8)
            .scan(Rng::new(7), |r, _| Some(r.next_rndm()))
            .collect();
        let b: Vec<f64> = (0..8)
            .scan(Rng::new(7), |r, _| Some(r.next_rndm()))
            .collect();
        assert_eq!(a, b);
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        assert_ne!(a.next_rndm(), b.next_rndm());
    }

    #[test]
    fn zero_seed_is_not_a_fixed_point() {
        let mut rng = Rng::new(0);
        assert_ne!(rng.next_f64(), rng.next_f64());
    }
}
