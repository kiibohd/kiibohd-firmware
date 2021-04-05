// Copyright 2021 Jacob Alexander
// Copyright 2021 Zion Koyl
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::io::Write;
use std::path::PathBuf;
fn main() {
    // Setup memory map for linker
    if env::var_os("CARGO_FEATURE_RT").is_some() {
        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("../../common/memory/atsam4s8b.x"))
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
        // TODO - Re-add once cmake is removed
        //println!("cargo:rerun-if-changed=memory.x");
    }
    //println!("cargo:rerun-if-changed=build.rs");

    // Read PID and HID from project.env
    let projectfile = File::open("project.env");
    let mut projectread = String::new();
    Some(projectfile.unwrap().read_to_string(&mut projectread));

    let mut bvid_rslt = "";
    let mut bpid_rslt = "";
    let mut vid_rslt = "";
    let mut pid_rslt = "";
    for line in projectread.lines() {
        let curline = &line;
        if curline.contains("BOOT_VID=\"") {
            let strt = curline
                .find("BOOT_VID=\"")
                .unwrap()
                .checked_add(10)
                .unwrap();
            let end = curline.rfind("\"").unwrap();
            bvid_rslt = &curline[strt..end];
        } else if curline.contains("BOOT_PID=\"") {
            let strt = curline
                .find("BOOT_PID=\"")
                .unwrap()
                .checked_add(10)
                .unwrap();
            let end = curline.rfind("\"").unwrap();
            bpid_rslt = &curline[strt..end];
        } else if curline.contains("DEVICE_VID=\"") {
            let strt = curline
                .find("DEVICE_VID=\"")
                .unwrap()
                .checked_add(12)
                .unwrap();
            let end = curline.rfind("\"").unwrap();
            vid_rslt = &curline[strt..end];
        } else if curline.contains("DEVICE_PID=\"") {
            let strt = curline
                .find("DEVICE_PID=\"")
                .unwrap()
                .checked_add(12)
                .unwrap();
            let end = curline.rfind("\"").unwrap();
            pid_rslt = &curline[strt..end];
        }
    }

    // Load custom DefaultMap from Cargo.toml if one exists
    let defmap_rslt: String;
    let partmap_rslt: String;
    match env::var_os("DefaultMapOverride") {
        Some(x) => defmap_rslt = x.into_string().unwrap(),
        None => defmap_rslt = String::from("geminiduskdawn/release.1 stdFuncMap"),
    }
    let defmap: &str = &defmap_rslt;

    // Load custom PartialMap's from Cargo.toml if one exists
    match env::var_os("PartialMapsExpandedOverride") {
        Some(x) => partmap_rslt = x.into_string().unwrap(),
        None => partmap_rslt = String::from("geminiduskdawn/release.1.layer.1 stdFuncMap;geminiduskdawn/release.1.layer.2 stdFuncMap"),
    }
    let partmap: &str = &partmap_rslt;

    // Build controller static library
    // During the transition to rust, we'll need to link in the original kiibohd
    // controller codebase.
    let dst = cmake::Config::new("../../common/controller")
        .cflag("-D_rustlib_")
        .define("CHIP", "sam4s8b")
        .define("COMPILER", "gcc")
        .define("ScanModule", "Gemini_Dusk_Dawn")
        .define("MacroModule", "PixelMap")
        .define("OutputModule", "USB")
        .define("DebugModule", "full")
        .define("LayoutName", "")
        .define("BaseMap", "scancode_map")
        .define("DefaultMap", defmap)
        .define("PartialMaps", partmap)
        // TODO - Store VIDs in a config file
        .define("VENDOR_ID", vid_rslt)
        .define("PRODUCT_ID", pid_rslt)
        .define("BOOT_VENDOR_ID", bvid_rslt)
        .define("BOOT_PRODUCT_ID", bpid_rslt)
        .always_configure(false)
        .generator("Ninja")
        .build();
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=kiibohd_static");
}
