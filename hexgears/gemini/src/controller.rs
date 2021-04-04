// Copyright 2021 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

extern "C" {
    /// Setup Latency Measurements
    pub fn Latency_init();

    /// Initialize CLI
    pub fn CLI_init();

    /// CLI Process
    pub fn CLI_process();

    /// NVM Storage Init
    pub fn Storage_init();

    /// NVM Storage Load Settings
    pub fn storage_load_settings();

    /// Output Module Setup
    pub fn Output_setup();

    /// Output Module Poll/Process
    pub fn Output_poll();

    /// Output Module Periodic
    pub fn Output_periodic();

    /// Macro Module Setup
    pub fn Macro_setup();

    /// Macro Module Poll/Process
    pub fn Macro_poll();

    /// Macro Module Periodic
    pub fn Macro_periodic();

    /// Scan Module Setup
    pub fn Scan_setup();

    /// Scan Module Poll/Process
    pub fn Scan_poll();

    /// Scan Module Periodic
    pub fn Scan_periodic();
}
