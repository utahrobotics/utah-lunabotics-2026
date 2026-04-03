use std::env::{self, VarError};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    if let Err(e) = std::env::var("ACTUATOR_SERIAL") {
        if e == VarError::NotPresent {
            println!("cargo:warning=ACTUATOR_SERIAL environment variable not set");
        } else {
            println!("cargo:warning=ACTUATOR_SERIAL environment variable not set to a valid value");
        }
    }

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("../memory-layouts/pi-pico.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");
}
