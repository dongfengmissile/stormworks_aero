//! Small vector type and FAR's Hermite curves.

use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: V3) -> V3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
    /// Unit vector, or zero for a (near) zero vector (Unity's `normalized`).
    pub fn norm(self) -> V3 {
        let l = self.len();
        if l > 1e-12 {
            self * (1.0 / l)
        } else {
            V3::ZERO
        }
    }
    /// Component of `self` perpendicular to unit `n` (Unity `ProjectOnPlane`).
    pub fn reject(self, n: V3) -> V3 {
        self - n * self.dot(n)
    }
    pub fn get(self, i: usize) -> f64 {
        [self.x, self.y, self.z][i]
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, k: f64) -> V3 {
        v3(self.x * k, self.y * k, self.z * k)
    }
}
impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for V3 {
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}
impl SubAssign for V3 {
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}

/// Cubic Hermite curve through `(x, y, slope)` keys, clamped outside its range
/// (FAR's `FARFloatCurve` and Unity's `FloatCurve` with equal in/out tangents).
#[derive(Clone, Debug, Default)]
pub struct Curve(pub Vec<[f64; 3]>);

impl Curve {
    pub fn eval(&self, x: f64) -> f64 {
        let k = &self.0;
        if k.is_empty() {
            return 0.0;
        }
        if x <= k[0][0] {
            return k[0][1];
        }
        let last = k[k.len() - 1];
        if x >= last[0] {
            return last[1];
        }
        let i = k.windows(2).position(|w| x < w[1][0]).unwrap_or(k.len() - 2);
        let (a, b) = (k[i], k[i + 1]);
        let h = b[0] - a[0];
        if h <= 0.0 {
            return b[1];
        }
        let t = (x - a[0]) / h;
        let (t2, t3) = (t * t, t * t * t);
        (2.0 * t3 - 3.0 * t2 + 1.0) * a[1]
            + (t3 - 2.0 * t2 + t) * h * a[2]
            + (-2.0 * t3 + 3.0 * t2) * b[1]
            + (t3 - t2) * h * b[2]
    }

    /// FAR `AddCurve`: both curves share their key x positions.
    pub fn add(&mut self, o: &Curve) {
        for (a, b) in self.0.iter_mut().zip(&o.0) {
            a[1] += b[1];
            a[2] += b[2];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hermite_hits_keys_and_clamps() {
        let c = Curve(vec![[0.0, 1.0, 0.0], [1.0, 3.0, 0.0], [2.0, 2.0, -1.0]]);
        assert_eq!(c.eval(-5.0), 1.0);
        assert_eq!(c.eval(1.0), 3.0);
        assert_eq!(c.eval(9.0), 2.0);
        assert!((c.eval(0.5) - 2.0).abs() < 1e-12);
        assert_eq!(v3(1.0, 0.0, 0.0).cross(v3(0.0, 1.0, 0.0)), v3(0.0, 0.0, 1.0));
    }
}
