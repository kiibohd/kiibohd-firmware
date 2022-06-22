// Copyright 2021-2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

use atsam4_hal::timer::ClockSource;
use const_env::from_env;

// ----- Flash Config -----

pub const FLASH_CONFIG_SIZE: usize = 524288 / core::mem::size_of::<u32>();
extern "C" {
    #[link_name = "_flash"]
    pub static mut FLASH_CONFIG: [u32; FLASH_CONFIG_SIZE];
}

// ----- Constants -----

// General clock frequencies
pub const MCU_FREQ: u32 = 120_000_000;

// RTT frequency calculations
pub const RTT_PRESCALER: usize = 4; // Most accurate than evenly counts seconds

// Timer frequency calculations
pub const TCC0_DIV: ClockSource = ClockSource::MckDiv128;
pub const TCC0_FREQ: u32 = MCU_FREQ / TCC0_DIV.div();
pub const TCC1_DIV: ClockSource = ClockSource::MckDiv128;
pub const TCC1_FREQ: u32 = MCU_FREQ / TCC1_DIV.div();
pub const TCC2_FREQ: u32 = TCC1_FREQ;

pub const BUF_CHUNK: usize = 64;
pub const ID_LEN: usize = 10;
pub const RX_BUF: usize = 8;
pub const SERIALIZATION_LEN: usize = 277;
pub const TX_BUF: usize = 8;

pub const CSIZE: usize = 17; // Number of columns
pub const RSIZE: usize = 6; // Number of rows
pub const MSIZE: usize = RSIZE * CSIZE; // Total matrix size

// Remap lookup
// 0 mapped keys are ignored
pub const SWITCH_REMAP: &[u8] = &[
    1,   // C1;R1:0
    20,  // C1;R2:1
    39,  // C1;R3:2
    58,  // C1;R4:3
    77,  // C1;R5:4
    96,  // C1;R6:5
    2,   // C2;R1:6
    21,  // C2;R2:7
    40,  // C2;R3:8
    59,  // C2;R4:9
    0,   // C2;R5:10
    97,  // C2;R6:11
    3,   // C3;R1:12
    22,  // C3;R2:13
    41,  // C3;R3:14
    60,  // C3;R4:15
    78,  // C3;R5:16
    98,  // C3;R6:17
    4,   // C4;R1:18
    23,  // C4;R2:19
    42,  // C4;R3:20
    61,  // C4;R4:21
    79,  // C4;R5:22
    0,   // C4;R6:23
    5,   // C5;R1:24
    24,  // C5;R2:25
    43,  // C5;R3:26
    62,  // C5;R4:27
    80,  // C5;R5:28
    0,   // C5;R6:29
    6,   // C6;R1:30
    25,  // C6;R2:31
    44,  // C6;R3:32
    63,  // C6;R4:33
    81,  // C6;R5:34
    99,  // C6;R6:35
    7,   // C7;R1:36
    26,  // C7;R2:37
    45,  // C7;R3:38
    64,  // C7;R4:39
    82,  // C7;R5:40
    0,   // C7;R6:41
    8,   // C8;R1:42
    27,  // C8;R2:43
    46,  // C8;R3:44
    65,  // C8;R4:45
    83,  // C8;R5:46
    0,   // C8;R6:47
    9,   // C9;R1:48
    28,  // C9;R2:49
    47,  // C9;R3:50
    66,  // C9;R4:51
    84,  // C9;R5:52
    0,   // C9;R6:53
    10,  // C10;R1:54
    29,  // C10;R2:55
    48,  // C10;R3:56
    67,  // C10;R4:57
    85,  // C10;R5:58
    100, // C10;R6:59
    11,  // C11;R1:60
    30,  // C11;R2:61
    49,  // C11;R3:62
    68,  // C11;R4:63
    86,  // C11;R5:64
    101, // C11;R6:65
    12,  // C12;R1:66
    31,  // C12;R2:67
    50,  // C12;R3:68
    69,  // C12;R4:69
    87,  // C12;R5:70
    0,   // C12;R6:71
    13,  // C13;R1:72
    32,  // C13;R2:73
    51,  // C13;R3:74
    0,   // C13;R4:75
    0,   // C13;R5:76
    102, // C13;R6:77
    0,   // C14;R1:78
    33,  // C14;R2:79
    52,  // C14;R3:80
    70,  // C14;R4:81
    88,  // C14;R5:82
    103, // C14;R6:83
    14,  // C15;R1:84
    34,  // C15;R2:85
    53,  // C15;R3:86
    0,   // C15;R4:87
    0,   // C15;R5:88
    104, // C15;R6:89
    15,  // C16;R1:90
    35,  // C16;R2:91
    54,  // C16;R3:92
    0,   // C16;R4:93
    89,  // C16;R5:94
    105, // C16;R6:95
    16,  // C17;R1:96
    36,  // C17;R2:97
    55,  // C17;R3:98
    0,   // C17;R4:99
    0,   // C17;R5:100
    106, // C17;R6:101
];

pub const CTRL_QUEUE_SIZE: usize = 5;
pub const KBD_QUEUE_SIZE: usize = 25;
pub const KBD_LED_QUEUE_SIZE: usize = 3;
pub const MOUSE_QUEUE_SIZE: usize = 10;

pub const SCAN_PERIOD_US: u32 = 1000 / CSIZE as u32; // Scan all strobes within 1 ms (1000 Hz) for USB
pub const DEBOUNCE_US: u32 = 5000; // 5 ms TODO Tuning
pub const IDLE_MS: u32 = 600_000; // 600 seconds TODO Tuning

// KLL Constants
pub const LAYOUT_SIZE: usize = 256;
pub const MAX_ACTIVE_LAYERS: usize = 8;
pub const MAX_ACTIVE_TRIGGERS: usize = 64;
pub const MAX_LAYERS: usize = 16;
pub const MAX_LAYER_STACK_CACHE: usize = 64;
pub const MAX_LAYER_LOOKUP_SIZE: usize = 64;
pub const MAX_OFF_STATE_LOOKUP: usize = 16;
pub const STATE_SIZE: usize = 32;

#[from_env]
pub const VID: u16 = 0x1c11;
#[from_env]
pub const PID: u16 = 0xb04d;
#[from_env]
pub const USB_MANUFACTURER: &str = "Unknown";
#[from_env]
pub const USB_PRODUCT: &str = "Kiibohd";
#[from_env]
pub const HIDIO_DEVICE_NAME: &str = "Kiibohd";
#[from_env]
pub const HIDIO_DEVICE_VENDOR: &str = "Unknown";
#[from_env]
pub const HIDIO_FIRMWARE_NAME: &str = "kiibohd-firmware";
#[from_env]
pub const VERGEN_GIT_SEMVER: &str = "N/A";
#[from_env]
pub const VERGEN_GIT_COMMIT_COUNT: &str = "0";
