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

pub const CSIZE: usize = 18; // Number of columns
pub const RSIZE: usize = 6; // Number of rows
pub const MSIZE: usize = RSIZE * CSIZE; // Total matrix size
pub const ADC_SAMPLES: usize = 2; // Number of samples per key per strobe
pub const ADC_BUF_SIZE: usize = ADC_SAMPLES * RSIZE; // Size of ADC buffer per strobe
pub const INVERT_STROBE: bool = true; // P-Mosfets need to be inverted
pub const ISSI_DRIVER_CHIPS: usize = 2;
pub const ISSI_DRIVER_QUEUE_SIZE: usize = 5;
pub const ISSI_DRIVER_CS_LAYOUT: [u8; ISSI_DRIVER_CHIPS] = [0, 1];
// Must be 256 or less, or a power of 2; e.g. 512 due limitations with embedded-dma
// Actual value should be -> ISSI_DRIVER_CHIPS * 198 (e.g. 396);
// Size is determined by the largest SPI tx transaction
pub const SPI_TX_BUF_SIZE: usize = 512;
// Size is determined by the largest SPI rx transaction
pub const SPI_RX_BUF_SIZE: usize = (32 + 2) * ISSI_DRIVER_CHIPS;

// Remap lookup
// 0 mapped keys are ignored
pub const SWITCH_REMAP: &[u8] = &[
    1,  // C1;R1:0
    17, // C1;R2:1
    35, // C1;R3:2
    52, // C1;R4:3
    66, // C1;R5:4
    81, // C1;R6:5
    0,  // C2;R1:6
    18, // C2;R2:7
    36, // C2;R3:8
    0,  // C2;R4:9
    67, // C2;R5:10
    82, // C2;R6:11
    2,  // C3;R1:12
    19, // C3;R2:13
    37, // C3;R3:14
    53, // C3;R4:15
    68, // C3;R5:16
    83, // C3;R6:17
    3,  // C4;R1:18
    20, // C4;R2:19
    38, // C4;R3:20
    54, // C4;R4:21
    69, // C4;R5:22
    84, // C4;R6:23
    4,  // C5;R1:24
    21, // C5;R2:25
    39, // C5;R3:26
    55, // C5;R4:27
    70, // C5;R5:28
    0,  // C5;R6:29
    5,  // C6;R1:30
    22, // C6;R2:31
    40, // C6;R3:32
    56, // C6;R4:33
    71, // C6;R5:34
    0,  // C6;R6:35
    0,  // C7;R1:36
    23, // C7;R2:37
    41, // C7;R3:38
    57, // C7;R4:39
    72, // C7;R5:40
    85, // C7;R6:41
    6,  // C8;R1:42
    24, // C8;R2:43
    42, // C8;R3:44
    58, // C8;R4:45
    73, // C8;R5:46
    86, // C8;R6:47
    7,  // C9;R1:48
    25, // C9;R2:49
    43, // C9;R3:50
    59, // C9;R4:51
    74, // C9;R5:52
    0,  // C9;R6:53
    8,  // C10;R1:54
    26, // C10;R2:55
    44, // C10;R3:56
    60, // C10;R4:57
    75, // C10;R5:58
    87, // C10;R6:59
    9,  // C11;R1:60
    27, // C11;R2:61
    45, // C11;R3:62
    61, // C11;R4:63
    76, // C11;R5:64
    88, // C11;R6:65
    10, // C12;R1:66
    28, // C12;R2:67
    46, // C12;R3:68
    62, // C12;R4:69
    77, // C12;R5:70
    89, // C12;R6:71
    11, // C13;R1:72
    29, // C13;R2:73
    47, // C13;R3:74
    63, // C13;R4:75
    0,  // C13;R5:76
    0,  // C13;R6:77
    12, // C14;R1:78
    30, // C14;R2:79
    0,  // C14;R3:80
    64, // C14;R4:81
    78, // C14;R5:82
    90, // C14;R6:83
    13, // C15;R1:84
    31, // C15;R2:85
    48, // C15;R3:86
    65, // C15;R4:87
    79, // C15;R5:88
    91, // C15;R6:89
    14, // C16;R1:90
    32, // C16;R2:91
    49, // C16;R3:92
    0,  // C16;R4:93
    0,  // C16;R5:94
    92, // C16;R6:95
    15, // C17;R1:96
    33, // C17;R2:97
    50, // C17;R3:98
    0,  // C17;R4:99
    80, // C17;R5:100
    93, // C17;R6:101
    16, // C18;R1:102
    34, // C18;R2:103
    51, // C18;R3:104
    0,  // C18;R4:105
    0,  // C18;R5:106
    94, // C18;R6:107
];

pub const CTRL_QUEUE_SIZE: usize = 5;
pub const KBD_QUEUE_SIZE: usize = 25;
pub const KBD_LED_QUEUE_SIZE: usize = 3;
pub const MOUSE_QUEUE_SIZE: usize = 10;

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
