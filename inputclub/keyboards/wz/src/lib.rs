// Copyright 2021-2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]

pub use atsam4_hal as hal;
use defmt_rtt as _;
use panic_probe as _;
use paste::paste;

use atsam4_hal::{define_pin_map, gpio::*, pac::MATRIX};

/// [AUTO GENERATED]
pub mod kll {
    //#![allow(clippy::all)]
    include!(concat!(env!("OUT_DIR"), "/generated_kll.rs"));
}

define_pin_map! {
    struct Pins,

    // Debug LED
    pin debug_led = b0<Output<PushPull>, into_push_pull_output>,

    // Sense
    pin sense1 = a26<Input<PullDown>, into_pull_down_input>,
    pin sense2 = a25<Input<PullDown>, into_pull_down_input>,
    pin sense3 = a24<Input<PullDown>, into_pull_down_input>,
    pin sense4 = a13<Input<PullDown>, into_pull_down_input>,
    pin sense5 = a14<Input<PullDown>, into_pull_down_input>,
    pin sense6 = a31<Input<PullDown>, into_pull_down_input>,

    // Strobe
    pin strobe1 = b1<Output<PushPull>, into_push_pull_output>,
    pin strobe2 = b2<Output<PushPull>, into_push_pull_output>,
    pin strobe3 = b3<Output<PushPull>, into_push_pull_output>,
    pin strobe4 = a18<Output<PushPull>, into_push_pull_output>,
    pin strobe5 = a19<Output<PushPull>, into_push_pull_output>,
    pin strobe6 = a23<Output<PushPull>, into_push_pull_output>,
    pin strobe7 = a20<Output<PushPull>, into_push_pull_output>,
    pin strobe8 = a11<Output<PushPull>, into_push_pull_output>,
    pin strobe9 = a8<Output<PushPull>, into_push_pull_output>,
    pin strobe10 = a7<Output<PushPull>, into_push_pull_output>,
    pin strobe11 = a6<Output<PushPull>, into_push_pull_output>,
    pin strobe12 = a5<Output<PushPull>, into_push_pull_output>,
    pin strobe13 = a27<Output<PushPull>, into_push_pull_output>,
    pin strobe14 = a28<Output<PushPull>, into_push_pull_output>,
    pin strobe15 = a29<Output<PushPull>, into_push_pull_output>,
    pin strobe16 = a30<Output<PushPull>, into_push_pull_output>,
    pin strobe17 = a2<Output<PushPull>, into_push_pull_output>,

    // ISSI
    pin issi_sdb = a15<Output<PushPull>, into_push_pull_output>,
    pin issi_intb = a16<Input<PullDown>, into_pull_down_input>,
    pin issi0_sda = a3<PfA, into_peripheral_function_a>,
    pin issi0_scl = a4<PfA, into_peripheral_function_a>,
    pin issi1_sda = b4<PfA, into_peripheral_function_a>,
    pin issi1_scl = b5<PfA, into_peripheral_function_a>,

    // Serial Console (UART1)
    pin uart0_rx = a9<PfA, into_peripheral_function_a>,
    pin uart0_tx = a10<PfA, into_peripheral_function_a>,

    // USB (UDP)
    pin udp_ddm = b10<SysFn, into_system_function>,
    pin udp_ddp = b11<SysFn, into_system_function>,
}
