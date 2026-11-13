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

/// Row-vector 4x4 product `r · m` with `r` a rotation about unit `axis` by `angle`.
pub fn rotated4(m: &[f64; 16], axis: [f64; 3], angle: f64) -> [f64; 16] {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let r = [
        [c + x * x * (1.0 - c), x * y * (1.0 - c) + z * s, x * z * (1.0 - c) - y * s],
        [y * x * (1.0 - c) - z * s, c + y * y * (1.0 - c), y * z * (1.0 - c) + x * s],
        [z * x * (1.0 - c) + y * s, z * y * (1.0 - c) - x * s, c + z * z * (1.0 - c)],
    ];
    let mut out = *m;
    for i in 0..3 {
        for j in 0..4 {
            out[4 * i + j] = (0..3).map(|k| r[i][k] * m[4 * k + j]).sum();
        }
    }
    out
}

#[cfg(test)]
mod rotated4_tests {
    #[test]
    fn rotation_keeps_translation_and_turns_about_axis() {
        let mut m = [0.0; 16];
        for i in 0..4 {
            m[5 * i] = 1.0;
        }
        m[12] = 5.0;
        let r = super::rotated4(&m, [0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2);
        assert_eq!(r[12], 5.0);
        // Row x maps into ±z, y stays.
        assert!((r[2].abs() - 1.0).abs() < 1e-12 && r[0].abs() < 1e-12);
        assert!((r[5] - 1.0).abs() < 1e-12);
    }
}

/// The eye of a perspective viewproj (row vectors p·M, clip.w = −z_eye): the one point with
/// clip x = y = w = 0. None for a degenerate matrix.
pub fn camera_eye(m: &[f64; 16]) -> Option<V3> {
    // Σ_i e_i·M[i][j] = −M[3][j] for the columns j = 0, 1, 3 (Cramer's rule).
    let col = |j: usize| [m[j], m[4 + j], m[8 + j]];
    let (a, b, c) = (col(0), col(1), col(3));
    let r = [-m[12], -m[13], -m[15]];
    let det3 = |x: [f64; 3], y: [f64; 3], z: [f64; 3]| {
        x[0] * (y[1] * z[2] - y[2] * z[1]) - x[1] * (y[0] * z[2] - y[2] * z[0]) + x[2] * (y[0] * z[1] - y[1] * z[0])
    };
    let d = det3(a, b, c);
    if d.abs() < 1e-12 {
        return None;
    }
    // Rows are the equations (columns of M); unknowns e_x, e_y, e_z.
    let rows = [[a[0], a[1], a[2]], [b[0], b[1], b[2]], [c[0], c[1], c[2]]];
    let solve = |k: usize| {
        let mut r2 = rows;
        for (row, v) in r2.iter_mut().zip(r) {
            row[k] = v;
        }
        det3(r2[0], r2[1], r2[2]) / d
    };
    Some(v3(solve(0), solve(1), solve(2)))
}

#[cfg(test)]
mod eye_tests {
    #[test]
    fn eye_of_a_look_at_viewproj() {
        // View: camera at (3, -2, 7) looking down −z (row vectors: p·V = p − eye), GL projection.
        let e = [3.0, -2.0, 7.0];
        let mut v = [0.0; 16];
        for k in 0..3 {
            v[k * 5] = 1.0;
            v[12 + k] = -e[k];
        }
        v[15] = 1.0;
        let (n, f, t) = (0.1, 1000.0, 1.0f64);
        let mut p = [0.0; 16];
        p[0] = 1.0 / t;
        p[5] = 1.0 / t;
        p[10] = -(f + n) / (f - n);
        p[11] = -1.0;
        p[14] = -2.0 * f * n / (f - n);
        let mut m = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                m[i * 4 + j] = (0..4).map(|k| v[i * 4 + k] * p[k * 4 + j]).sum();
            }
        }
        let got = super::camera_eye(&m).unwrap();
        assert!((got.x - 3.0).abs() < 1e-9 && (got.y + 2.0).abs() < 1e-9 && (got.z - 7.0).abs() < 1e-9, "{got:?}");
    }
}
