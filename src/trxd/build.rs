//! The DATV audio transcoder (src/dvbs2/aacx.c) against a static libavcodec
//! built by package/ffmpeg-aac: only when FFMPEG_AAC_DIR names its prefix
//! (Buildroot's staging /usr, or a test image's). Without it trxd builds as
//! before and DATV goes out without sound (src/dvbs2/aac.rs says why).
fn main() {
    println!("cargo::rustc-check-cfg=cfg(has_aac)");
    println!("cargo:rerun-if-env-changed=FFMPEG_AAC_DIR");
    println!("cargo:rerun-if-changed=src/dvbs2/aacx.c");
    let Ok(dir) = std::env::var("FFMPEG_AAC_DIR") else { return };
    if dir.is_empty() {
        return;
    }
    cc::Build::new().file("src/dvbs2/aacx.c").include(format!("{dir}/include")).compile("aacx");
    println!("cargo:rustc-link-search=native={dir}/lib");
    println!("cargo:rustc-link-lib=static=avcodec");
    println!("cargo:rustc-link-lib=static=swresample");
    println!("cargo:rustc-link-lib=static=avutil");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-cfg=has_aac");
}
