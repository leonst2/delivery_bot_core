//! rclrs's build script finds the interface libraries through AMENT_PREFIX_PATH but doesn't rerun when it changes.
//! In a cached workspace, a new interface package would then fail to link, so every Rust node needs this script.

use std::env;

fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    let prefixes = env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    for prefix in prefixes.split(':').filter(|prefix| !prefix.is_empty()) {
        println!("cargo:rustc-link-search=native={prefix}/lib");
    }
}
