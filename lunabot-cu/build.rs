fn main() {
    println!(
        "cargo:rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").unwrap()
    );
    if std::env::var("MUJOCO_STATIC_LINK_DIR").is_err() {
        panic!(
            "Please set the MUJOCO_STATIC_LINK_DIR env variable. Instructions for mujoco installation at https://mujoco-rs.readthedocs.io/en/stable/installation.html#static-linking"
        )
    }
}
