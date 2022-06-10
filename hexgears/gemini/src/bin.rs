// Copyright 2021-2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]
#![no_main]

use cortex_m_rt::exception;

// ----- Flash Config -----

const FLASH_CONFIG_SIZE: usize = 524288 / core::mem::size_of::<u32>();
extern "C" {
    #[link_name = "_flash"]
    static mut FLASH_CONFIG: [u32; FLASH_CONFIG_SIZE];
}

// ----- RTIC -----

// RTIC requires that unused interrupts are declared in an extern block when
// using software tasks; these free interrupts will be used to dispatch the
// software tasks.
#[rtic::app(device = gemini::hal::pac, peripherals = true, dispatchers = [UART1, USART0, USART1, SSC, PWM, ACC, ADC, SPI])]
mod app {
    use crate::FLASH_CONFIG;
    use const_env::from_env;
    use core::convert::Infallible;
    use core::fmt::Write;
    use heapless::spsc::{Producer, Queue};
    use heapless::String;
    use kiibohd_hid_io::*;
    use kiibohd_usb::HidCountryCode;

    use gemini::{
        hal::{
            chipid::ChipId,
            clock::{ClockController, MainClock, SlowClock},
            efc::Efc,
            gpio::*,
            pac::TC0,
            prelude::*,
            rtt::RealTimeTimer,
            time::duration::Extensions,
            timer::{ClockSource, TimerCounter, TimerCounterChannel},
            udp::{
                usb_device,
                usb_device::{
                    bus::UsbBusAllocator,
                    device::{UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
                },
                UdpBus,
            },
            watchdog::Watchdog,
            ToggleableOutputPin,
        },
        kll, Pins,
    };

    // ----- Sizes -----

    const BUF_CHUNK: usize = 64;
    const ID_LEN: usize = 10;
    const RX_BUF: usize = 8;
    const SERIALIZATION_LEN: usize = 277;
    const TX_BUF: usize = 8;

    const CSIZE: usize = 17; // Number of columns
    const RSIZE: usize = 6; // Number of rows
    const MSIZE: usize = RSIZE * CSIZE; // Total matrix size

    // Remap lookup
    // 0 mapped keys are ignored
    const SWITCH_REMAP: &[u8] = &[
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

    const CTRL_QUEUE_SIZE: usize = 5;
    const KBD_QUEUE_SIZE: usize = 25;
    const MOUSE_QUEUE_SIZE: usize = 10;

    const SCAN_PERIOD_US: u32 = 1000 / CSIZE as u32; // Scan all strobes within 1 ms (1000 Hz) for USB
    const DEBOUNCE_US: u32 = 5000; // 5 ms TODO Tuning
    const IDLE_MS: u32 = 600_000; // 600 seconds TODO Tuning

    // KLL Constants
    const LAYOUT_SIZE: usize = 256;
    const MAX_ACTIVE_LAYERS: usize = 8;
    const MAX_ACTIVE_TRIGGERS: usize = 64;
    const MAX_LAYERS: usize = 16;
    const MAX_LAYER_STACK_CACHE: usize = 64;
    const MAX_LAYER_LOOKUP_SIZE: usize = 64;
    const MAX_OFF_STATE_LOOKUP: usize = 16;
    const STATE_SIZE: usize = 32;

    #[from_env]
    const VID: u16 = 0x1c11;
    #[from_env]
    const PID: u16 = 0xb04d;
    #[from_env]
    const USB_MANUFACTURER: &str = "Unknown";
    #[from_env]
    const USB_PRODUCT: &str = "Kiibohd";
    #[from_env]
    const HIDIO_DEVICE_NAME: &str = "Kiibohd";
    #[from_env]
    const HIDIO_DEVICE_VENDOR: &str = "Unknown";
    #[from_env]
    const HIDIO_FIRMWARE_NAME: &str = "kiibohd-firmware";
    #[from_env]
    const VERGEN_GIT_SEMVER: &str = "N/A";
    #[from_env]
    const VERGEN_GIT_COMMIT_COUNT: &str = "0";

    // ----- Types -----

    type HidInterface = kiibohd_usb::HidInterface<
        'static,
        UdpBus,
        KBD_QUEUE_SIZE,
        MOUSE_QUEUE_SIZE,
        CTRL_QUEUE_SIZE,
    >;
    type HidioCommandInterface = CommandInterface<
        HidioInterface<MESSAGE_LEN>,
        TX_BUF,
        RX_BUF,
        BUF_CHUNK,
        MESSAGE_LEN,
        SERIALIZATION_LEN,
        ID_LEN,
    >;
    type Matrix = kiibohd_keyscanning::Matrix<
        PioX<Output<PushPull>>,
        PioX<Input<PullDown>>,
        CSIZE,
        RSIZE,
        MSIZE,
        SCAN_PERIOD_US,
        DEBOUNCE_US,
        IDLE_MS,
    >;
    type LayerLookup = kll_core::layout::LayerLookup<'static, LAYOUT_SIZE>;
    type LayerState = kll_core::layout::LayerState<
        'static,
        LAYOUT_SIZE,
        STATE_SIZE,
        MAX_LAYERS,
        MAX_ACTIVE_LAYERS,
        MAX_ACTIVE_TRIGGERS,
        MAX_LAYER_STACK_CACHE,
        MAX_OFF_STATE_LOOKUP,
    >;

    type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;

    // ----- Structs -----

    pub struct HidioInterface<const H: usize> {
        mcu: Option<String<12>>,
        serial: Option<String<126>>,
    }

    impl<const H: usize> HidioInterface<H> {
        fn new(chip: &ChipId, serial: Option<String<126>>) -> Self {
            let mcu = if let Some(model) = chip.model() {
                let mut mcu: String<12> = String::new();
                if write!(mcu, "{:?}", model).is_ok() {
                    Some(mcu)
                } else {
                    None
                }
            } else {
                None
            };

            Self { mcu, serial }
        }
    }

    impl<const H: usize> KiibohdCommandInterface<H> for HidioInterface<H> {
        fn h0001_device_name(&self) -> Option<&str> {
            Some(HIDIO_DEVICE_NAME)
        }

        fn h0001_device_mcu(&self) -> Option<&str> {
            if let Some(mcu) = &self.mcu {
                Some(mcu)
            } else {
                None
            }
        }

        fn h0001_device_serial_number(&self) -> Option<&str> {
            if let Some(serial) = &self.serial {
                Some(serial)
            } else {
                None
            }
        }

        fn h0001_device_vendor(&self) -> Option<&str> {
            Some(HIDIO_DEVICE_VENDOR)
        }

        fn h0001_firmware_name(&self) -> Option<&str> {
            Some(HIDIO_FIRMWARE_NAME)
        }

        fn h0001_firmware_version(&self) -> Option<&str> {
            Some(VERGEN_GIT_SEMVER)
        }
    }

    //
    // Shared resources used by tasks/interrupts
    //
    #[shared]
    struct Shared {
        ctrl_producer: Producer<'static, kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE>,
        debug_led: Pb0<Output<PushPull>>,
        hidio_intf: HidioCommandInterface,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        layer_state: LayerState,
        matrix: Matrix,
        mouse_producer: Producer<'static, kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE>,
        rtt: RealTimeTimer,
        tcc0: TimerCounterChannel<TC0, 0>,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
        wdt: Watchdog,
    }

    //
    // Local resources, static mut variables
    //
    #[local]
    struct Local {}

    //
    // Initialization
    //
    #[init(
        local = [
            ctrl_queue: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new(),
            kbd_queue: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new(),
            mouse_queue: Queue<kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE> = Queue::new(),
            serial_number: String<126> = String::new(),
            usb_bus: Option<UsbBusAllocator<UdpBus>> = None,
    ])]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Fix vector table (Bootloader bug?)
        //unsafe { cx.core.SCB.vtor.write(0x406000) };
        //TODO - cortex-m-rt v0.8 set-vtor feature
        // Fix stack pointer (Bootloader bug?)
        //TODO - This is not safe, should be done another way (maybe not necessary?)
        //TODO - cortex-m-rt v0.8 set-sp feature
        //let sp = 0x20020000;
        //unsafe { asm!("msr MSP, {}", in(reg) sp) };

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        defmt::info!(">>>> Initializing <<<<");

        // Show processor registers
        defmt::trace!("MSP: {:#010x}", cortex_m::register::msp::read());
        defmt::trace!("PSP: {:#010x}", cortex_m::register::psp::read());

        // Determine which chip is running
        let chip = ChipId::new(cx.device.CHIPID);
        defmt::info!("MCU: {:?}", chip.model());

        // Setup main and slow clocks
        defmt::trace!("Clock initialization");
        let clocks = ClockController::new(
            cx.device.PMC,
            &cx.device.SUPC,
            &cx.device.EFC0,
            MainClock::Crystal12Mhz,
            SlowClock::RcOscillator32Khz,
        );

        // Setup gpios
        defmt::trace!("GPIO initialization");
        let gpio_ports = Ports::new(
            (
                cx.device.PIOA,
                clocks.peripheral_clocks.pio_a.into_enabled_clock(),
            ),
            (
                cx.device.PIOB,
                clocks.peripheral_clocks.pio_b.into_enabled_clock(),
            ),
        );
        let pins = Pins::new(gpio_ports, &cx.device.MATRIX);

        // Prepare watchdog to be fed
        let mut wdt = Watchdog::new(cx.device.WDT);
        wdt.feed();
        defmt::trace!("Watchdog first feed");

        // Setup flash controller (needed for unique id)
        let efc = Efc::new(cx.device.EFC0, unsafe { &mut FLASH_CONFIG });
        // Retrieve unique id and format it for the USB descriptor
        let uid = efc.read_unique_id().unwrap();
        write!(
            cx.local.serial_number,
            "{:x}{:x}{:x}{:x}",
            uid[0], uid[1], uid[2], uid[3]
        )
        .unwrap();
        defmt::info!("UID: {}", cx.local.serial_number);

        // Setup Keyscanning Matrix
        defmt::trace!("Keyscanning Matrix initialization");
        let cols = [
            pins.strobe1.downgrade(),
            pins.strobe2.downgrade(),
            pins.strobe3.downgrade(),
            pins.strobe4.downgrade(),
            pins.strobe5.downgrade(),
            pins.strobe6.downgrade(),
            pins.strobe7.downgrade(),
            pins.strobe8.downgrade(),
            pins.strobe9.downgrade(),
            pins.strobe10.downgrade(),
            pins.strobe11.downgrade(),
            pins.strobe12.downgrade(),
            pins.strobe13.downgrade(),
            pins.strobe14.downgrade(),
            pins.strobe15.downgrade(),
            pins.strobe16.downgrade(),
            pins.strobe17.downgrade(),
        ];
        let rows = [
            pins.sense1.downgrade(),
            pins.sense2.downgrade(),
            pins.sense3.downgrade(),
            pins.sense4.downgrade(),
            pins.sense5.downgrade(),
            pins.sense6.downgrade(),
        ];
        let mut matrix = Matrix::new(cols, rows).unwrap();
        matrix.next_strobe().unwrap(); // Initial strobe

        // Setup kll-core
        let loop_condition_lookup: &[u32] = &[0]; // TODO: Use KLL Compiler

        // Load datastructures into kll-core
        let layer_lookup = LayerLookup::new(
            kll::LAYER_LOOKUP,
            kll::TRIGGER_GUIDES,
            kll::RESULT_GUIDES,
            kll::TRIGGER_RESULT_MAPPING,
            loop_condition_lookup,
        );

        // Initialize LayerState for kll-core
        let layer_state = LayerState::new(layer_lookup, 0);

        // Setup HID-IO interface
        defmt::trace!("HID-IO Interface initialization");
        let hidio_intf = HidioCommandInterface::new(
            &[
                HidIoCommandId::SupportedIds,
                HidIoCommandId::GetInfo,
                HidIoCommandId::TestPacket,
            ],
            HidioInterface::<MESSAGE_LEN>::new(&chip, Some(cx.local.serial_number.clone())),
        )
        .unwrap();

        // Setup USB
        defmt::trace!("UDP initialization");
        let (kbd_producer, kbd_consumer) = cx.local.kbd_queue.split();
        let (mouse_producer, mouse_consumer) = cx.local.mouse_queue.split();
        let (ctrl_producer, ctrl_consumer) = cx.local.ctrl_queue.split();
        let mut udp_bus = UdpBus::new(
            cx.device.UDP,
            clocks.peripheral_clocks.udp,
            pins.udp_ddm,
            pins.udp_ddp,
        );
        udp_bus.remote_wakeup_enabled(true); // Enable hardware support for remote wakeup
        *cx.local.usb_bus = Some(UsbBusAllocator::<UdpBus>::new(udp_bus));
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();
        let usb_hid = HidInterface::new(
            usb_bus,
            HidCountryCode::NotSupported,
            kbd_consumer,
            mouse_consumer,
            ctrl_consumer,
        );
        let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer(USB_MANUFACTURER)
            .max_packet_size_0(64)
            .max_power(500)
            .product(USB_PRODUCT)
            .supports_remote_wakeup(true)
            .serial_number(cx.local.serial_number)
            .device_release(VERGEN_GIT_COMMIT_COUNT.parse().unwrap())
            .build();

        // TODO This should only really be run when running with a debugger for development
        usb_dev.force_reset().unwrap();

        // Setup main timer
        let tc0 = TimerCounter::new(
            cx.device.TC0,
            clocks.peripheral_clocks.tc_0.into_enabled_clock(),
        );
        let tc0_chs = tc0.split();
        let mut tcc0 = tc0_chs.ch0;
        tcc0.clock_input(ClockSource::MckDiv128);
        tcc0.start((SCAN_PERIOD_US * 1000).nanoseconds());
        defmt::trace!("TCC0 started");
        tcc0.enable_interrupt();

        // Setup secondary timer (used for watchdog, activity led and sleep related functionality)
        let mut rtt = RealTimeTimer::new(cx.device.RTT, 3, false);
        rtt.start(500_000u32.microseconds());
        rtt.enable_alarm_interrupt();
        defmt::trace!("RTT Timer started");

        (
            Shared {
                ctrl_producer,
                debug_led: pins.debug_led,
                hidio_intf,
                kbd_producer,
                layer_state,
                matrix,
                mouse_producer,
                rtt,
                tcc0,
                usb_dev,
                usb_hid,
                wdt,
            },
            Local {},
            init::Monotonics {},
        )
    }

    /// Keyscanning Task (Uses TC0)
    /// High-priority scheduled tasks as consistency is more important than speed for scanning
    /// key states
    /// Scans one strobe at a time
    #[task(priority = 13, binds = TC0, shared = [hidio_intf, layer_state, matrix, tcc0])]
    fn tc0(mut cx: tc0::Context) {
        cx.shared.tcc0.lock(|w| w.clear_interrupt_flags());

        let process_macros = cx.shared.matrix.lock(|matrix| {
            cx.shared.layer_state.lock(|layer_state| {
                // Scan one strobe (strobes have already been enabled and allowed to settle)
                if let Ok((reading, strobe)) = matrix.sense::<Infallible>() {
                    for (i, entry) in reading.iter().enumerate() {
                        for event in
                            entry.trigger_event(SWITCH_REMAP[strobe * RSIZE + i] as usize, true)
                        {
                            let hidio_event = HidIoEvent::TriggerEvent(event);

                            // Enqueue KLL trigger event
                            let ret = layer_state.process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                            debug_assert!(
                                ret.is_ok(),
                                "Failed to enqueue: {:?} - {:?}",
                                event,
                                ret
                            );

                            // Enqueue HID-IO trigger event
                            cx.shared.hidio_intf.lock(|hidio_intf| {
                                if let Err(err) = hidio_intf.process_event(hidio_event) {
                                    defmt::error!("Hidio TriggerEvent Error: {:?}", err);
                                }
                            });
                        }
                    }
                }

                // Strobe next column
                matrix.next_strobe::<Infallible>().unwrap() == 0
            })
        });

        // If a full matrix scanning cycle has finished, process macros
        if process_macros && macro_process::spawn().is_err() {
            defmt::warn!("Could not schedule macro_process");
        }
    }

    /// Activity tick
    /// Used visually determine MCU status
    #[task(priority = 1, binds = RTT, shared = [debug_led, rtt, wdt])]
    fn rtt(mut cx: rtt::Context) {
        cx.shared.rtt.lock(|w| w.clear_interrupt_flags());

        // Feed watchdog
        cx.shared.wdt.lock(|w| w.feed());

        // Blink debug led
        // TODO: Remove (or use feature flag)
        cx.shared.debug_led.lock(|w| w.toggle().ok());
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(priority = 10, shared = [ctrl_producer, hidio_intf, kbd_producer, layer_state, matrix, mouse_producer])]
    fn macro_process(mut cx: macro_process::Context) {
        cx.shared.layer_state.lock(|layer_state| {
            // Confirm off-state lookups
            cx.shared.matrix.lock(|matrix| {
                layer_state.process_off_state_lookups::<MAX_LAYER_LOOKUP_SIZE>(&|index| {
                    matrix
                        .generate_event(index)
                        .unwrap()
                        .trigger_event(index, false)
                });
            });

            // Finalize triggers to generate CapabilityRun events
            for cap_run in layer_state.finalize_triggers::<MAX_LAYER_LOOKUP_SIZE>() {
                match cap_run {
                    kll_core::CapabilityRun::NoOp { .. } => {}
                    kll_core::CapabilityRun::HidKeyboard { .. }
                    | kll_core::CapabilityRun::HidKeyboardState { .. } => {
                        cx.shared.kbd_producer.lock(|kbd_producer| {
                            debug_assert!(
                                kiibohd_usb::enqueue_keyboard_event(cap_run, kbd_producer).is_ok(),
                                "KBD_QUEUE_SIZE too small"
                            );
                        })
                    }
                    kll_core::CapabilityRun::HidProtocol { .. } => {}
                    kll_core::CapabilityRun::HidConsumerControl { .. }
                    | kll_core::CapabilityRun::HidSystemControl { .. } => {
                        cx.shared.ctrl_producer.lock(|ctrl_producer| {
                            debug_assert!(
                                kiibohd_usb::enqueue_ctrl_event(cap_run, ctrl_producer).is_ok(),
                                "CTRL_QUEUE_SIZE too small"
                            );
                        })
                    }
                    /*
                    kll_core::CapabilityRun::McuFlashMode { .. } => {}
                    kll_core::CapabilityRun::HidioOpenUrl { .. }
                    | kll_core::CapabilityRun::HidioUnicodeString { .. }
                    | kll_core::CapabilityRun::HidioUnicodeState { .. } => {}
                    kll_core::CapabilityRun::LayerClear { .. }
                    | kll_core::CapabilityRun::LayerRotate { .. }
                    | kll_core::CapabilityRun::LayerState { .. } => {}
                    */
                    _ => {
                        panic!("{:?} is unsupported by this keyboard", cap_run);
                    }
                }
            }

            // Next time iteration
            layer_state.increment_time();
        });

        // Schedule USB processing
        if usb_process::spawn().is_err() {
            defmt::warn!("Could not schedule usb_process");
        }
    }

    /// USB Outgoing Events Task
    /// Sends outgoing USB HID events generated by the macro_process task
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(priority = 11, shared = [usb_hid, usb_dev])]
    fn usb_process(cx: usb_process::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut usb_hid = cx.shared.usb_hid;
        usb_hid.lock(|usb_hid| {
            // Update USB events
            if usb_hid.update() {
                usb_dev.lock(|usb_dev| {
                    match usb_dev.state() {
                        UsbDeviceState::Suspend => {
                            // Issue USB Resume if enabled
                            if usb_dev.remote_wakeup_enabled() {
                                usb_dev.bus().remote_wakeup();
                            }
                        }

                        UsbDeviceState::Configured => {
                            // Commit USB events
                            while usb_hid.push().is_err() {}
                        }

                        _ => {}
                    }
                });
            }
        });
    }

    /// ISSI I2C0 Interrupt
    #[task(priority = 12, binds = TWI0)]
    fn twi0(_: twi0::Context) {
        //unsafe { TWI0_Handler() };
    }

    /// ISSI I2C1 Interrupt
    #[task(priority = 12, binds = TWI1)]
    fn twi1(_: twi1::Context) {}

    /// USB Device Interupt
    #[task(priority = 14, binds = UDP, shared = [hidio_intf, usb_dev, usb_hid])]
    fn udp(cx: udp::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut usb_hid = cx.shared.usb_hid;
        let mut hidio_intf = cx.shared.hidio_intf;

        // Poll USB endpoints
        usb_dev.lock(|usb_dev| {
            usb_hid.lock(|usb_hid| {
                hidio_intf.lock(|hidio_intf| {
                    if usb_dev.poll(&mut usb_hid.interfaces()) {
                        usb_hid.poll(hidio_intf);
                    }
                });
            });
        });
    }
}

#[exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault!");
}

// Timestamps currently use the cycle counter
// TODO: Use something better and easier to read? (but still fast)
defmt::timestamp!("{=u32} us", {
    // TODO (HaaTa): Determine a way to calculate the divider automatically
    //               Or transition to a hardware timer?
    cortex_m::peripheral::DWT::cycle_count() / 120
});

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
