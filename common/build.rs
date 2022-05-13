// Copyright 2021-2022 Jacob Alexander
// Copyright 2021 Zion Koyl
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use kll_compiler::{Filestore, KllGroups, Layouts};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    // Read variables from project.env
    println!("cargo:rerun-if-changed=project.env");
    dotenv::from_filename("project.env").ok();

    // Setup memory map for linker
    let linker_file = &PathBuf::from(env::var_os("TOP_LEVEL").unwrap())
        .join(env::var_os("LINKER_SCRIPT").unwrap());
    let memory_x = std::fs::read_to_string(linker_file)
        .unwrap_or_else(|_| panic!("Unable to read file: {:?}", linker_file));
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x.as_bytes())
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={}", linker_file.display());

    // Gather useful data from project.env
    let bvid_rslt = env::var("BOOT_VID").unwrap();
    let bpid_rslt = env::var("BOOT_PID").unwrap();
    let vid_rslt = env::var("DEVICE_VID").unwrap();
    let pid_rslt = env::var("DEVICE_PID").unwrap();
    let usb_manufacturer = env::var("USB_MANUFACTURER").unwrap();
    let usb_product = env::var("USB_PRODUCT").unwrap();
    let usb_serial_chip = env::var("USB_SERIAL_CHIP").unwrap();
    let hidio_device_name = env::var("HIDIO_DEVICE_NAME").unwrap();
    let hidio_device_vendor = env::var("HIDIO_DEVICE_VENDOR").unwrap();
    let hidio_firmware_name = env::var("HIDIO_FIRMWARE_NAME").unwrap();

    println!("cargo:rustc-env=BOOT_VID={}", bvid_rslt);
    println!("cargo:rustc-env=BOOT_PID={}", bpid_rslt);
    println!("cargo:rustc-env=VID={}", vid_rslt);
    println!("cargo:rustc-env=PID={}", pid_rslt);
    println!("cargo:rustc-env=USB_MANUFACTURER={}", usb_manufacturer);
    println!("cargo:rustc-env=USB_PRODUCT={}", usb_product);
    println!("cargo:rustc-env=USB_SERIAL_CHIP={}", usb_serial_chip);
    println!("cargo:rustc-env=HIDIO_DEVICE_NAME={}", hidio_device_name);
    println!("cargo:rustc-env=HIDIO_DEVICE_VENDOR={}", hidio_device_vendor);
    println!("cargo:rustc-env=HIDIO_FIRMWARE_NAME={}", hidio_firmware_name);

    // Generate Rust code from KLL files
    let mut filestore = Filestore::new();

    // Add basemap
    let basemap_file = PathBuf::from(env::var("KLL_BASEMAP").unwrap());
    filestore.load_file(&basemap_file);
    println!(
        "cargo:rerun-if-changed={}",
        basemap_file.as_path().display()
    );

    // Gather kll layers
    // First split on ; (layers) then on , (combined files in a layer)
    let mut defaultmap_files = Vec::new();
    let layers = env::var("KLL_LAYERS").unwrap_or_else(|_| "".to_string());
    if !layers.is_empty() {
        for (i, layer) in layers.split(';').enumerate() {
            for layer_file in layer.split(',') {
                let file = PathBuf::from(layer_file);
                // Make sure the file exists
                assert!(file.is_file(), "{:?} does not exist", file);
                filestore.load_file(&file);
                println!("cargo:rerun-if-changed={}", file.as_path().display());
                if i == 0 {
                    defaultmap_files.push(file);
                } else {
                    // TODO
                }
            }
        }
    }

    // Retrieve layouts
    let layouts_path = PathBuf::from(env::var_os("TOP_LEVEL").unwrap()).join("common/layouts");
    let layouts = Layouts::from_dir(layouts_path);

    // TODO Handle layers
    //      Figure out how to merge files
    //      Figure out how to pass multiple layers
    let groups = KllGroups::new(&filestore, &[], &[basemap_file], &defaultmap_files, &[]);

    // Verify and generate rust
    kll_compiler::emitters::kllcore::verify(&groups).unwrap();
    let outfile = out.join("generated_kll.rs");
    kll_compiler::emitters::kllcore::write(&outfile, &groups, layouts);
}
