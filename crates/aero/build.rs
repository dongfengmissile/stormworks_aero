fn main() {
    // Unique stamp per compiled binary, logged at bootstrap. On-disk checksums
    // cannot tell you what a RUNNING game has loaded; this can.
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    println!("cargo:rustc-env=AERO_BUILD_STAMP={stamp}");
}
