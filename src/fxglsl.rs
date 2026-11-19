//! The effects' shaders (`fxdraw` draws with them).
#![cfg_attr(not(windows), allow(dead_code))] // drawn by `fxdraw` (Windows only); tested everywhere

macro_rules! noise_glsl {
    () => {
"float hash(vec2 p) {
    vec3 q = fract(vec3(p.xyx) * 0.1031);
    q += dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), f.x), mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), f.x), f.y);
}
"
    };
}

macro_rules! light_glsl {
    () => {
"// The game's lights as its light pass (light.glslf) had them this frame: sun and moon colour ×
// intensity summed, sky light from below and from above. No shadows: the game's shadow map
// (stairs and stripes through the mist at any filtering) looked wrong on it.
uniform vec3 sun_col;
uniform vec3 amb_low;
uniform vec3 amb_high;

// Light on droplets: they scatter it every way, so the sun counts at any angle (as the game lights
// its particles). sky: how open they are to the sky above (0.5: as much as below); shade: how much
// sunlight gets through the mist over them.
vec3 droplet_light(float sky, float shade) {
    return mix(amb_low, amb_high, sky) + sun_col * shade;
}
"
    };
}

/// The game's depth is log2(w * 0.01 + 1) / log2(202) (depth_utils.glslh): the scene's view depth here.
macro_rules! scene_w_glsl {
    () => {
"float scene_w() {
    float d = texture(gdepth, gl_FragCoord.xy / target_size).r;
    return (exp2(d * log2(202.0)) - 1.0) / 0.01;
}
"
    };
}

/// Floats per vertex attribute, in location order: the trails' (`VERTEX`) and the mist's.
pub const RIBBON_ATTRS: [usize; 3] = [3, 4, 3];
pub const CLOUD_ATTRS: [usize; 13] = [4, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4];

pub const CLOUD_VERTEX: &str = "#version 330
layout(location = 0) in vec4 pos; // camera-relative, and 1 when the camera is in the box
layout(location = 1) in vec3 root;
layout(location = 2) in vec3 ax_u;
layout(location = 3) in vec3 ax_s;
layout(location = 4) in vec3 ax_n;
layout(location = 5) in vec4 geom;
layout(location = 6) in vec4 col0;
layout(location = 7) in vec4 col1;
layout(location = 8) in vec4 mean0;
layout(location = 9) in vec4 mean1;
layout(location = 10) in vec4 par;
layout(location = 11) in vec4 par2;
layout(location = 12) in vec4 par3;
uniform mat4 view_proj;
out vec3 v_rel;
out float v_w;
flat out vec3 f_root, f_u, f_s, f_n;
flat out vec4 f_geom, f_c0, f_c1, f_m0, f_m1, f_par, f_par2, f_par3;
flat out float f_inside;
void main() {
    gl_Position = view_proj * vec4(pos.xyz, 1.0);
    v_w = gl_Position.w;
    v_rel = pos.xyz;
    f_inside = pos.w;
    f_root = root; f_u = ax_u; f_s = ax_s; f_n = ax_n;
    f_geom = geom; f_c0 = col0; f_c1 = col1; f_m0 = mean0; f_m1 = mean1; f_par = par; f_par2 = par2; f_par3 = par3;
}
";

// The mist over a wing panel (`fxwing`): the camera ray marched through its box, from the face
// up to whatever the scene has behind (the wing itself), summing `fxwing` density.
pub const CLOUD_FRAGMENT: &str = concat!("#version 330
in vec3 v_rel;
in float v_w;
flat in vec3 f_root, f_u, f_s, f_n;
flat in float f_inside; // the camera is in the box: these are its far faces
flat in vec4 f_geom, f_c0, f_c1, f_m0, f_m1, f_par, f_par2, f_par3; // (b, c_root, taper, tan sweep); condensate column per station (x2); its mean height, m (x2); (k, thick m, height, shock); (free tip, flow in streak units, span0 m, fade m); (slope, wake m, root spill m, tip spill m)
uniform sampler2D gdepth;
uniform vec2 target_size;
uniform float fog_density;
out vec4 color_out;
", noise_glsl!(), scene_w_glsl!(), light_glsl!(), "
// The stations' (column, mean height) at chord position x (`fxwing::condense`), not smeared
// across the shock: the mist ends (or thins) within a few percent of the chord of it.
vec2 station(float x, float shock) {
    float s = clamp(x * 8.0 - 0.5, 0.0, 7.0);
    float i = min(floor(s), 6.0);
    float f = s - i;
    float xs = shock * 8.0 - 0.5;
    if (xs > i && xs < i + 1.0) f = smoothstep(xs - 0.35, xs + 0.35, s);
    // Stations i and i + 1 weighted, without indexing an array (slow on a GPU: it spills).
    const vec4 k0 = vec4(0.0, 1.0, 2.0, 3.0), k1 = vec4(4.0, 5.0, 6.0, 7.0);
    vec4 w0 = vec4(equal(k0, vec4(i))) * (1.0 - f) + vec4(equal(k0, vec4(i + 1.0))) * f;
    vec4 w1 = vec4(equal(k1, vec4(i))) * (1.0 - f) + vec4(equal(k1, vec4(i + 1.0))) * f;
    return vec2(dot(w0, f_c0) + dot(w1, f_c1), dot(w0, f_m0) + dot(w1, f_m1));
}

// Extinction at q (camera-relative), 1/m, and the optical depth of the mist over it.
vec2 density(vec3 q) {
    vec3 d = q - f_root;
    float b = f_geom.x;
    float eta = dot(d, f_s) / b;
    float c = f_geom.y * (1.0 + (f_geom.z - 1.0) * eta);
    float x = 0.5 - (dot(d, f_u) + f_geom.w * eta * b) / c;
    // Behind the trailing edge the mist rides the air leaving along the flight path.
    float aft = max(x - 1.0, 0.0) * c;
    float y = dot(d, f_n) - aft * f_par3.x;
    float er = eta * b, et = (1.0 - eta) * b;
    if (er < -f_par3.z || et < -f_par3.w || x < 0.0 || aft > f_par3.y || y < 0.0 || y > f_par.z) return vec2(0.0);
    // Towards a free tip the lift fades out, and with it the layer's density and thickness.
    float tip = f_par2.x > 0.5 ? sqrt(clamp(et / (0.15 * b), 0.0, 1.0)) : 1.0;
    float span = tip;
    // At a junction both panels draw half, each share fading out across the other's side: the
    // two add up to one sheet.
    if (f_par3.z > 0.0) span *= er < 0.0 ? 0.5 * (1.0 - smoothstep(0.0, f_par3.z, -er)) : mix(0.5, 1.0, smoothstep(0.0, f_par3.z, er));
    if (f_par3.w > 0.0) span *= et < 0.0 ? 0.5 * (1.0 - smoothstep(0.0, f_par3.w, -et)) : mix(0.5, 1.0, smoothstep(0.0, f_par3.w, et));
    // From the stagnation point; past the last station it evaporates as it goes.
    // The shock's line wavers a little along the span, and is a few percent of the chord deep.
    float sp0 = f_par2.z + er;
    vec2 st = station(x, f_par.w + 0.05 * (noise(vec2(sp0 * 0.7, 3.1)) - 0.5));
    float col = st.x * min(x * 16.0, 1.0) * exp(-max(x - 0.9375, 0.0) * c / max(f_par2.w, 0.01));
    // Above the highest lumps there (most of the box, where it rises over the wake): nothing,
    // before the noise, which is most of the cost.
    float l0 = 4.0 * max(st.y, 0.005) * mix(0.3, 1.0, tip);
    if (col <= 0.0 || span <= 0.0 || y >= 1.6 * l0) return vec2(0.0);
    // Streaks along the flow, riding it at the airspeed (metres long, so they read as motion,
    // not a strobe), and billows: the layer's thickness heaves in lumps on the flow. Both as big
    // as the layer is thick (from 10 cm), or a thick one stands in thin fins.
    float sc = 0.1 / max(f_par.y, 0.1);
    float sp = sp0 * sc, along = x * c * sc - f_par2.y;
    float n = 0.6 * noise(vec2(sp * 3.0, along * 0.08)) + 0.4 * noise(vec2(sp * 7.0 + 31.0, along * 0.2));
    float lump = 0.6 * noise(vec2(sp * 3.0 + 57.0, along * 0.3)) + 0.4 * noise(vec2(sp * 8.0 + 91.0, along * 0.8));
    // LUMP = 0.6 (fxwing). The column spread over a layer of that mean height, densest at the
    // surface and thinning to nothing at 4 mean heights (the box holds it all: no flat top).
    float l = l0 * (1.0 + 0.6 * (2.0 * lump - 1.0));
    if (y >= l) return vec2(0.0);
    // The noise only thins it.
    float amp = 0.25 + 0.35 * clamp(aft / max(f_par2.w, 0.1), 0.0, 1.0);
    float u = 1.0 - y / l, e0 = col * span * mix(1.0 - 2.0 * amp, 1.0, n);
    float e = e0 * 3.0 / l * u * u;
    // Few droplets show just past the threshold: it fades in rather than switching on.
    // Over it: the profile's integral up to the top, e0·u³ (taking the thin fade-in as dense).
    return f_par.x * vec2(e * e / (e + 0.25), e0 * u * u * u);
}

void main() {
    float t0 = length(v_rel);
    vec3 dir = v_rel / t0;
    float w_per_m = v_w / t0; // view depth per metre along the ray
    float scene_t = scene_w() / w_per_m;
    float b = f_geom.x;
    float len = length(vec3(b + f_par3.z + f_par3.w, f_geom.y * max(1.0, f_geom.z) + abs(f_geom.w) * b + f_par3.y, f_par.z));
    float t1 = min(t0 + len, scene_t);
    if (f_inside > 0.5) {
        // From the camera to the far face.
        t1 = min(t0, scene_t);
        t0 = 0.0;
    } else if (scene_t < t0 - 0.02) discard;
    // Only march where the mist can be: between the surface plane and the box top, and within the
    // span with its spills. A thin layer seen from above then gets every sample.
    float dn = dot(dir, f_n), ds = dot(dir, f_s);
    dn = (dn < 0.0 ? -1.0 : 1.0) * max(abs(dn), 1e-6);
    ds = (ds < 0.0 ? -1.0 : 1.0) * max(abs(ds), 1e-6);
    float on = -dot(f_root, f_n), os = -dot(f_root, f_s);
    float ya = -on / dn, yb = (f_par.z - on) / dn, sa = (-f_par3.z - os) / ds, sb = (b + f_par3.w - os) / ds;
    t0 = max(t0, max(min(ya, yb), min(sa, sb)));
    t1 = min(t1, min(max(ya, yb), max(sa, sb)));
    if (t1 <= t0) discard;
    // Samples: a step through the layer's height of under 0.4 of its mean height (it is densest
    // low down), and along it of under 4: from 8 (a short look down into it) to 32 (along it).
    float th = max(f_par.y, 0.05);
    float N = clamp(ceil((t1 - t0) * max(abs(dn) / (0.4 * th), 0.25 / th)), 8.0, 32.0);
    float dt = (t1 - t0) / N;
    float j = hash(gl_FragCoord.xy);
    // Front to back. Lit from above the wing: mist with more mist over it is in its shade (its
    // lumps' own: what makes it read as billows, not a film).
    vec3 col = vec3(0.0);
    float trans = 1.0;
    for (int i = 0; i < 32; i++) {
        if (float(i) >= N) break;
        float t = t0 + (float(i) + j) * dt;
        vec3 q = dir * t;
        vec2 s = density(q);
        if (s.x <= 0.0) continue;
        float a = 1.0 - exp(-s.x * dt);
        float lit = exp(-0.08 * s.y);
        col += trans * a * 0.9 * droplet_light(mix(0.5, 1.0, lit), mix(0.4, 1.0, lit));
        trans *= 1.0 - a;
        if (trans < 0.01) break;
    }
    if (trans > 0.999) discard;
    float fog = 1.0 / exp(v_w * fog_density);
    color_out = vec4(col, 1.0 - trans) * fog;
}
");

pub const VERTEX: &str = "#version 330
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 thread;
layout(location = 2) in vec3 coil;
uniform mat4 view_proj;
out vec4 v_thread;
out vec3 v_coil;
out float v_w;
void main() {
    gl_Position = view_proj * vec4(pos, 1.0);
    v_w = gl_Position.w;
    v_thread = thread;
    v_coil = coil;
}
";

// A condensed thread (`fx::chord_depth` across it) wound in a coil round the vortex axis, thick
// and thin along its length, streaked with wisps that turn with it. The game's depth is
// log2(w * 0.01 + 1) / log2(202) (depth_utils.glslh); fog as in apply_fog.
pub const FRAGMENT: &str = concat!("#version 330
in vec4 v_thread; // across from the ribbon's middle (m), thread radius (m), optical depth scale, core / radius
in vec3 v_coil;   // coil phase (rad), along the trail in radii, coil radius (m)
in float v_w;
uniform sampler2D gdepth;
uniform vec2 target_size;
uniform float fog_density;
out vec4 color_out;
", light_glsl!(), "

float chord_depth(float x, float c) {
    x = min(abs(x), 1.0);
    float l = sqrt(max(1.0 - x * x, 0.0));
    if (c >= 1.0) return 4.0 / 3.0 * l * l * l;
    float lc = min(sqrt(max(c * c - x * x, 0.0)), l);
    float o = x > 1e-4 ? atan(x * (l - lc), x * x + l * lc) / x : 1.0 / max(lc, 1e-4) - 1.0 / max(l, 1e-4);
    float c2 = c * c;
    float c4 = c2 * c2;
    return 2.0 * (o - (l - lc)) + 2.0 * lc * (2.0 / c2 - x * x / c4 - 1.0) - 2.0 / 3.0 * lc * lc * lc / c4;
}

",
noise_glsl!(),
"
// Streaks wound round the thread (six round it, a turn per six radii along), long along the helix.
float wisps(float around) {
    vec2 q = vec2((around + v_coil.x) * (6.0 / 6.2832) + v_coil.y, v_coil.y * 0.15);
    return 0.65 * noise(q) + 0.35 * noise(q * 2.7 + 5.3);
}

void main() {
    float d = texture(gdepth, gl_FragCoord.xy / target_size).r;
    float scene_w = (exp2(d * log2(202.0)) - 1.0) / 0.01;
    // Behind geometry: hidden; close in front of it: fades, so no hard line where it cuts in.
    float soft = clamp((scene_w - v_w) / 0.5, 0.0, 1.0);
    // Thicker and thinner along its length; seen from the side the coil puts it off the middle.
    float lump = 0.6 + 0.8 * noise(vec2(v_coil.y * 0.4, 1.7));
    float r = max(v_thread.y * lump, 1e-3);
    float x = (v_thread.x - v_coil.z * sin(v_coil.x)) / r;
    if (abs(x) >= 1.0) discard;
    float front = asin(x);
    float streaks = mix(0.3, 1.7, 0.5 * (wisps(front) + wisps(3.14159 - front)));
    float tau = v_thread.z * lump * chord_depth(x, v_thread.w / lump) * streaks;
    float a = (1.0 - exp(-tau)) * soft / exp(v_w * fog_density);
    // Thick parts shade themselves a little, thin wisps stay bright.
    float thin = exp(-0.4 * tau);
    vec3 col = 0.9 * droplet_light(mix(0.6, 0.85, thin), mix(0.5, 1.0, thin));
    color_out = vec4(col * a, a);
}
");

#[cfg(test)]
mod tests {
    /// The shaders compile (skipped without glslangValidator).
    #[test]
    fn shaders_compile() {
        for (name, src) in [("r.vert", super::VERTEX), ("r.frag", super::FRAGMENT), ("c.vert", super::CLOUD_VERTEX), ("c.frag", super::CLOUD_FRAGMENT)] {
            let path = std::env::temp_dir().join(format!("aero_fx_{name}"));
            std::fs::write(&path, src).unwrap();
            let Ok(out) = std::process::Command::new("glslangValidator").arg(&path).output() else { return };
            assert!(out.status.success(), "{name}: {}", String::from_utf8_lossy(&out.stdout));
        }
    }
}
