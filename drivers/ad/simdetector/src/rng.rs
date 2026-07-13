//! Uniform `[0, 1)` source replacing C `rand() / (double)RAND_MAX`.
//!
//! DEVIATION: the C driver draws from libc `rand()`, whose sequence is
//! platform-specific and whose range is the *closed* interval `[0, 1]`. Nothing
//! in the simulated wire format depends on the exact sequence — the values only
//! feed the noise background and the per-peak height jitter — so this port uses
//! a self-contained xorshift64* generator over the *half-open* `[0, 1)`. The
//! half-open range additionally guarantees the background rotation offset
//! `(int)(nElements * rand01)` stays a valid index, which the C code relies on
//! implicitly.

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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_are_in_half_open_unit_interval() {
        let mut rng = Rng::new(12345);
        for _ in 0..10_000 {
            let v = rng.next_f64();
            assert!((0.0..1.0).contains(&v), "out of range: {v}");
        }
    }

    #[test]
    fn same_seed_gives_same_sequence() {
        let a: Vec<f64> = (0..8)
            .scan(Rng::new(7), |r, _| Some(r.next_f64()))
            .collect();
        let b: Vec<f64> = (0..8)
            .scan(Rng::new(7), |r, _| Some(r.next_f64()))
            .collect();
        assert_eq!(a, b);
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        assert_ne!(a.next_f64(), b.next_f64());
    }

    #[test]
    fn zero_seed_is_not_a_fixed_point() {
        let mut rng = Rng::new(0);
        assert_ne!(rng.next_f64(), rng.next_f64());
    }
}
