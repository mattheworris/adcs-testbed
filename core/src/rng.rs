//! Tiny deterministic noise source for the sensor models (no_std, no alloc).

pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Self {
        Rng(if seed == 0 { 0x1234_5678 } else { seed })
    }

    /// xorshift32
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Approximately standard normal: sum of 12 uniforms minus 6 (Irwin-Hall).
    /// Cheap on an MCU and good enough for sensor noise.
    pub fn gauss(&mut self) -> f32 {
        let mut s = 0.0;
        for _ in 0..12 {
            s += self.uniform();
        }
        s - 6.0
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn gauss_has_unit_variance() {
        let mut r = super::Rng::new(42);
        let n = 20_000;
        let (mut m, mut v) = (0.0f64, 0.0f64);
        for _ in 0..n {
            let g = r.gauss() as f64;
            m += g;
            v += g * g;
        }
        m /= n as f64;
        v = v / n as f64 - m * m;
        assert!(m.abs() < 0.03, "mean {m}");
        assert!((v - 1.0).abs() < 0.05, "var {v}");
    }
}
