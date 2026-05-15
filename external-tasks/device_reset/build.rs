use std::env;

fn main() {
    // Set the log index directory for cu29 logging
    let out_dir = env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-env=LOG_INDEX_DIR={}", out_dir);
    
    // Add library search path for RealSense
    println!("cargo:rustc-link-search=native=/usr/local/lib");
    println!("cargo:rustc-link-lib=realsense2");
} 