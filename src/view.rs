//! Voxel view: a tiny z-buffered software rasteriser for the exposed voxel faces.
//! It renders into an RGBA image the overlay uploads as a texture, plus a per-pixel
//! cell id for exact picking. (ImGui's 16-bit indices cannot hold a big vehicle.)

use crate::math::{v3, V3};
use crate::voxel::{Grid, EMPTY, VOXEL_M};

pub struct Face {
    /// Convex polygon, body frame (metres).
    pub poly: Vec<V3>,
    pub normal: V3,
    /// Component the face belongs to, and the grid cell just inside it.
    pub comp: u32,
    pub cell: usize,
}

/// Grid-unit point to the body frame (same mapping as `aero_pos`).
fn to_body(p: V3) -> V3 {
    v3(p.z * VOXEL_M, p.x * VOXEL_M, -p.y * VOXEL_M)
}

/// Faces of every part solid, minus those pressed flat against a full neighbour cell.
pub fn faces(g: &Grid) -> Vec<Face> {
    let mut out = Vec::new();
    for (comp, solid) in &g.solids {
        let centre = solid.iter().flatten().fold(V3::ZERO, |a, b| a + *b) * (1.0 / solid.iter().map(|f| f.len()).sum::<usize>().max(1) as f64);
        for f in solid {
            if f.len() < 3 {
                continue;
            }
            let mut n = (f[1] - f[0]).cross(f[2] - f[0]).norm();
            let fc = f.iter().fold(V3::ZERO, |a, b| a + *b) * (1.0 / f.len() as f64);
            if n.dot(fc - centre) < 0.0 {
                n = -n;
            }
            let inside = fc - n * 0.01;
            let cell = [inside.x, inside.y, inside.z].map(|v| (v + 0.5).floor() as i32);
            let Some(ci) = g.index(cell) else { continue };
            // Axis-aligned face on a cell boundary against a full cell: internal, skip.
            let axis = (0..3).find(|&a| n.get(a).abs() > 0.999);
            if let Some(a) = axis {
                let on_boundary = f.iter().all(|p| ((p.get(a) + 0.5) - (p.get(a) + 0.5).round()).abs() < 1e-6);
                let mut nb = cell;
                nb[a] += n.get(a).signum() as i32;
                if on_boundary && g.index(nb).is_some_and(|j| g.owner[j] != EMPTY && g.fill[j] > 0.999) {
                    continue;
                }
            }
            out.push(Face { poly: f.iter().map(|p| to_body(*p)).collect(), normal: to_body(n) * (1.0 / VOXEL_M), comp: *comp, cell: ci });
        }
    }
    out
}

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    /// Orbit angles (radians) around `target`, distance in metres.
    pub yaw: f64,
    pub pitch: f64,
    pub dist: f64,
    pub target: V3,
}

pub struct Projection {
    eye: V3,
    fwd: V3,
    right: V3,
    up: V3,
    focal: f64,
    w: f64,
    h: f64,
    /// Row-vector body frame -> OpenGL clip matrix (the game camera); replaces the orbit camera.
    clip: Option<[f64; 16]>,
}

impl Camera {
    pub fn projection(&self, w: usize, h: usize) -> Projection {
        // Body frame: x fwd, y right, z down. yaw 0 looks at the vehicle's left side.
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let to_eye = v3(-sy * cp, -cy * cp, -sp);
        let eye = self.target + to_eye * self.dist;
        let fwd = -to_eye;
        let world_up = v3(0.0, 0.0, -1.0);
        let right = fwd.cross(world_up).norm();
        let up = right.cross(fwd).norm();
        Projection { eye, fwd, right, up, focal: h as f64 / (2.0 * (22.5f64).to_radians().tan()), w: w as f64, h: h as f64, clip: None }
    }
}

impl Projection {
    /// Project with an external camera: `clip = [p, 1]·m`, NDC -1..1, clip.w = view depth.
    pub fn from_clip(m: [f64; 16], w: usize, h: usize) -> Projection {
        let z = V3::ZERO;
        Projection { eye: z, fwd: z, right: z, up: z, focal: 0.0, w: w as f64, h: h as f64, clip: Some(m) }
    }
    /// Screen position (pixels) and view depth; depth <= 0 is behind the camera.
    pub fn project(&self, p: V3) -> (f64, f64, f64) {
        if let Some(m) = &self.clip {
            let c = |k: usize| p.x * m[k] + p.y * m[4 + k] + p.z * m[8 + k] + m[12 + k];
            let cw = c(3);
            if cw <= 1e-6 {
                return (0.0, 0.0, -1.0);
            }
            return (self.w * (0.5 + 0.5 * c(0) / cw), self.h * (0.5 - 0.5 * c(1) / cw), cw);
        }
        let d = p - self.eye;
        let z = d.dot(self.fwd);
        let k = self.focal / z.max(1e-3);
        (self.w * 0.5 + d.dot(self.right) * k, self.h * 0.5 - d.dot(self.up) * k, z)
    }
    /// Screen right and up directions in the body frame.
    pub fn axes(&self) -> (V3, V3) {
        (self.right, self.up)
    }
}

pub struct Image {
    pub w: usize,
    pub h: usize,
    pub rgba: Vec<u8>,
    /// Face index + 1 under each pixel, 0 = background.
    pub id: Vec<u32>,
}

/// `color(face index)`; the id buffer holds face index + 1.
pub fn render(faces: &[Face], color: &dyn Fn(usize) -> [u8; 3], cam: &Camera, w: usize, h: usize) -> Image {
    let (w, h) = (w.max(8), h.max(8));
    render_with(faces, color, &cam.projection(w, h), w, h, [24, 26, 30, 255])
}

/// Render through any projection onto a `background` colour (alpha 0 = see-through).
pub fn render_with(faces: &[Face], color: &dyn Fn(usize) -> [u8; 3], pr: &Projection, w: usize, h: usize, background: [u8; 4]) -> Image {
    let mut img = Image { w, h, rgba: background.repeat(w * h), id: vec![0; w * h] };
    let mut depth = vec![f64::MAX; w * h];
    // External cameras: light from above-front in the body frame, no back-face test (no eye).
    let light = if pr.clip.is_some() { v3(0.3, -0.3, -1.0).norm() } else { (-pr.fwd + pr.up * 0.6 + pr.right * 0.3).norm() };
    for (fi, f) in faces.iter().enumerate() {
        if pr.clip.is_none() && f.normal.dot(f.poly[0] - pr.eye) >= 0.0 {
            continue; // back face
        }
        let p: Vec<(f64, f64, f64)> = f.poly.iter().map(|c| pr.project(*c)).collect();
        if p.iter().any(|q| q.2 <= 0.05) {
            continue;
        }
        let base = color(fi);
        let shade = 0.45 + 0.55 * f.normal.dot(light).abs();
        let col = base.map(|c| (c as f64 * shade).min(255.0) as u8);
        let edge = base.map(|c| (c as f64 * shade * 0.55) as u8);
        // Fan triangles; mark which triangle edges are polygon edges for the outline.
        let n = p.len();
        for i in 1..n - 1 {
            let outline = [i == n - 2, true, i == 1]; // edges opposite vertex 0, 1, 2 of (0, i, i+1)
            raster(&mut img, &mut depth, [p[0], p[i], p[i + 1]], outline, col, edge, fi as u32 + 1);
        }
    }
    img
}

fn raster(img: &mut Image, depth: &mut [f64], p: [(f64, f64, f64); 3], outline: [bool; 3], col: [u8; 3], edge: [u8; 3], id: u32) {
    let area = (p[1].0 - p[0].0) * (p[2].1 - p[0].1) - (p[2].0 - p[0].0) * (p[1].1 - p[0].1);
    if area.abs() < 1e-9 {
        return;
    }
    let minx = p.iter().map(|q| q.0).fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let maxx = p.iter().map(|q| q.0).fold(f64::MIN, f64::max).ceil().min(img.w as f64 - 1.0);
    let miny = p.iter().map(|q| q.1).fold(f64::MAX, f64::min).floor().max(0.0) as usize;
    let maxy = p.iter().map(|q| q.1).fold(f64::MIN, f64::max).ceil().min(img.h as f64 - 1.0);
    if maxx < 0.0 || maxy < 0.0 {
        return;
    }
    // Pixel distance to edge k = w_k · 2·area / |edge k|.
    let len = |a: usize, b: usize| (p[a].0 - p[b].0).hypot(p[a].1 - p[b].1).max(1e-9);
    let k = [area.abs() / len(1, 2), area.abs() / len(2, 0), area.abs() / len(0, 1)];
    for y in miny..=maxy as usize {
        for x in minx..=maxx as usize {
            let (fx, fy) = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = ((p[1].0 - fx) * (p[2].1 - fy) - (p[2].0 - fx) * (p[1].1 - fy)) / area;
            let w1 = ((p[2].0 - fx) * (p[0].1 - fy) - (p[0].0 - fx) * (p[2].1 - fy)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9 {
                continue;
            }
            let z = w0 * p[0].2 + w1 * p[1].2 + w2 * p[2].2;
            let i = y * img.w + x;
            if z >= depth[i] {
                continue;
            }
            depth[i] = z;
            let w = [w0, w1, w2];
            let on_edge = (0..3).any(|e| outline[e] && w[e] * k[e] < 1.0);
            img.rgba[i * 4..i * 4 + 3].copy_from_slice(if on_edge { &edge } else { &col });
            img.rgba[i * 4 + 3] = 255;
            img.id[i] = id;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::aero_pos;

    #[test]
    fn renders_and_picks_a_block() {
        let g = Grid::from_cells(&[([0, 0, 0], 1.0, 0), ([1, 0, 0], 1.0, 1)]);
        let f = faces(&g);
        assert_eq!(f.len(), 10, "two cubes share one hidden face pair");
        let cam = Camera { yaw: 0.3, pitch: 0.4, dist: 3.0, target: aero_pos([0, 0, 0]) };
        let img = render(&f, &|_| [200, 200, 200], &cam, 64, 48);
        let centre = img.id[24 * 64 + 32];
        assert!(centre != 0, "something drawn at the centre");
        assert!(img.id[0] == 0, "corner is background");
        // The same view through an equivalent clip matrix hits the same block.
        let pr = cam.projection(64, 48);
        let (r, u, fw) = (pr.right, pr.up, pr.fwd);
        let e = pr.eye;
        let (fx, fy) = (pr.focal / 32.0, pr.focal / 24.0);
        // clip = (x·fx, y·fy, -, depth) with x, y, depth = d·right, d·up, d·fwd and d = p - eye.
        let col = |a: V3, k: f64| [a.x * k, a.y * k, a.z * k, -(e.dot(a)) * k];
        let (cx, cy, cw) = (col(r, fx), col(u, fy), col(fw, 1.0));
        let m: [f64; 16] = std::array::from_fn(|i| { let (row, c) = (i / 4, i % 4); let v = [cx, cy, [0.0; 4], cw][c]; v[row] });
        let img2 = render_with(&f, &|_| [200, 200, 200], &Projection::from_clip(m, 64, 48), 64, 48, [0; 4]);
        assert_eq!(img2.id[24 * 64 + 32], centre);
        assert_eq!(img2.rgba[3], 0, "transparent background");
    }
}

/// World point -> screen pixels through the main camera (row vectors p·M, OpenGL NDC).
pub(crate) fn project(vp: &[f64; 16], p: crate::math::V3, size: [f32; 2]) -> Option<[f32; 2]> {
    let w = p.x * vp[3] + p.y * vp[7] + p.z * vp[11] + vp[15];
    if w <= 1e-3 {
        return None; // behind the camera
    }
    let x = (p.x * vp[0] + p.y * vp[4] + p.z * vp[8] + vp[12]) / w;
    let y = (p.x * vp[1] + p.y * vp[5] + p.z * vp[9] + vp[13]) / w;
    Some([((x * 0.5 + 0.5) * size[0] as f64) as f32, ((0.5 - y * 0.5) * size[1] as f64) as f32])
}


#[cfg(test)]
mod arrow_tests {
    use super::project;
    use crate::math::v3;

    #[test]
    fn projects_row_vector_viewproj() {
        // x' = x, y' = y, w = z (column 3), so NDC = (x/z, y/z).
        let mut vp = [0.0; 16];
        (vp[0], vp[5], vp[11]) = (1.0, 1.0, 1.0);
        let size = [200.0, 100.0];
        assert_eq!(project(&vp, v3(0.0, 0.0, 2.0), size), Some([100.0, 50.0]));
        assert_eq!(project(&vp, v3(2.0, 2.0, 2.0), size), Some([200.0, 0.0])); // NDC y up = screen top
        assert_eq!(project(&vp, v3(0.0, 0.0, -1.0), size), None);
    }
}
