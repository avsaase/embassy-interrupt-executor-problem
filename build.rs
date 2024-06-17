use std::{
    env,
    f64::consts::TAU,
    fs::{self, File},
    io::Write,
    path::{Path, PathBuf},
};

const LUT_SIZE: usize = 256;

fn main() {
    // This build script copies the `memory.x` file from the crate root into
    // a directory where the linker can always find it at build time.
    // For many projects this is optional, as the linker always searches the
    // project root directory -- wherever `Cargo.toml` is. However, if you
    // are using a workspace or have a more complicated build setup, this
    // build script becomes required. Additionally, by requesting that
    // Cargo re-run the build script whenever `memory.x` is changed,
    // updating `memory.x` ensures a rebuild of the application with the
    // new memory settings.

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    // File::create(out.join("memory.x"))
    //     .unwrap()
    //     .write_all(include_bytes!("memory.x"))
    //     .unwrap();
    // println!("cargo:rustc-link-search={}", out.display());

    fs::write(
        out.join("link_ram.x"),
        include_bytes!("./link_ram_cortex_m.x"),
    )
    .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=link_ram.x");

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    // println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Generate LUTs

    let table = generate_sine_lut::<LUT_SIZE>();
    let path = Path::new(&std::env::var("OUT_DIR").unwrap()).join("sine_lut.rs");
    let mut file = File::create(path).unwrap();
    writeln!(&mut file, "#[rustfmt::skip]").unwrap();
    writeln!(&mut file, "pub const SINE_LUT: [i16; {}] = [", LUT_SIZE).unwrap();
    for entry in table {
        writeln!(&mut file, "    {},", entry).unwrap();
    }
    writeln!(&mut file, "];").unwrap();

    let table = generate_triangle_lut::<LUT_SIZE>();
    let path = Path::new(&std::env::var("OUT_DIR").unwrap()).join("triangle_lut.rs");
    let mut file = File::create(path).unwrap();
    writeln!(&mut file, "#[rustfmt::skip]").unwrap();
    writeln!(&mut file, "pub const TRIANGLE_LUT: [i16; {}] = [", LUT_SIZE).unwrap();
    for entry in table {
        writeln!(&mut file, "    {},", entry).unwrap();
    }
    writeln!(&mut file, "];").unwrap();
}

fn generate_sine_lut<const N: usize>() -> [i16; N] {
    let mut table = [0i16; N];

    table.iter_mut().enumerate().for_each(|(index, value)| {
        let angle = (index as f64 / N as f64) * TAU;
        *value = (angle.sin() * i16::MAX as f64).round() as i16;
    });
    table
}

fn generate_triangle_lut<const N: usize>() -> [i16; N] {
    let amplitude = i16::MAX as f64;
    let period = N as f64;
    let mut table = [0; N];
    for (index, value) in table.iter_mut().enumerate() {
        let x = index as f64;
        *value = (4. * amplitude / period
            * ((x - period / 4.).rem_euclid(period) - period / 2.).abs()
            - amplitude) as i16;
    }
    table
}
