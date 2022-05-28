// Copyright 2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(alloc_error_handler)]
#![feature(type_alias_impl_trait)]
#![macro_use]

use cortex_m_rt::exception;
use defmt_rtt as _;
use embassy_nrf as _;
use nrf52833_hal as hal;
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::{raw, Softdevice};
use nrf_softdevice_defmt_rtt as _; // global logger
use panic_probe as _;
use rtic::app;

/// [AUTO GENERATED]
/// Generated KLL
mod kll {
    include!(concat!(env!("OUT_DIR"), "/generated_kll.rs"));
}

// ----- RTIC -----

// RTIC requires that unused interrupts are declared in an extern block when
// using software tasks; these free interrupts will be used to dispatch the
// software tasks.
// See:
// <https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsds_s113%2FSDS%2Fs1xx%2Fsd_mgr%2Fsd_enable_disable.html>
// for reserved peripherals
// See: <https://github.com/nrf-rs/nrf-pacs/blob/master/pacs/nrf52833-pac/src/lib.rs> for a list of
// interrupts
#[app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI2_EGU2, SWI3_EGU3, SWI4_EGU4, QDEC, NFCT, I2S])]
mod app {
    use alloc_cortex_m::CortexMHeap;
    use core::alloc::Layout;
    use core::mem;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use const_env::from_env;
    use core::fmt::Write;
    use embassy::executor::Executor;
    use embassy::util::Forever;
    use embedded_hal::{
        timer::CountDown,
    };
    use heapless::spsc::{Producer, Queue};
    use heapless::String;
    use kiibohd_hid_io::*;
    use kiibohd_usb::HidCountryCode;
    use nrf_softdevice::{raw, Softdevice};
    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDeviceBuilder, UsbVidPid},
    };
    use void::Void;

    use crate::hal::{
        clocks::{self, Clocks},
        gpio::{self, Input, Level, Output, Pin, PullDown, PushPull},
        pac::{
            TIMER1,
        },
        timer::{Timer, Periodic},
        usbd::{UsbPeripheral, Usbd},
    };

    // ----- Sizes -----

    const BUF_CHUNK: usize = 64;
    const ID_LEN: usize = 10;
    const RX_BUF: usize = 8;
    const SERIALIZATION_LEN: usize = 277;
    const TX_BUF: usize = 8;

    const CSIZE: usize = 16; // Number of columns
    const RSIZE: usize = 5; // Number of rows
    const MSIZE: usize = RSIZE * CSIZE; // Total matrix size

    const CTRL_QUEUE_SIZE: usize = 2;
    const KBD_QUEUE_SIZE: usize = 2;
    const MOUSE_QUEUE_SIZE: usize = 2;

    const SCAN_PERIOD_US: u32 = 1000 / CSIZE as u32; // Scan all strobes within 1 ms (1000 Hz) for USB
    const DEBOUNCE_US: u32 = 5000; // 5 ms TODO Tuning
    const IDLE_MS: u32 = 600_000; // 600 seconds TODO Tuning

    // ----- KLL Constants -----

    // TODO - Tune
    const LAYOUT_SIZE: usize = 256;
    const MAX_ACTIVE_LAYERS: usize = 2;
    const MAX_ACTIVE_TRIGGERS: usize = 32;
    const MAX_LAYERS: usize = 2;
    const MAX_LAYER_STACK_CACHE: usize = 2;
    const MAX_LAYER_LOOKUP_SIZE: usize = 2;
    const MAX_OFF_STATE_LOOKUP: usize = 2;
    const STATE_SIZE: usize = 2;

    // ----- Env Constants -----

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
        Usbd<UsbPeripheral<'static>>,
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
        Pin<Output<PushPull>>,
        Pin<Input<PullDown>>,
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

    type UsbDevice = usb_device::device::UsbDevice<'static, Usbd<UsbPeripheral<'static>>>;

    // ----- Embassy -----

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    // Global embassy executor
    static EXECUTOR: Forever<Executor> = Forever::new();

    // Out Of Memory (OOM) condition
    #[alloc_error_handler]
    fn alloc_error(_layout: Layout) -> ! {
        panic!("Alloc error");
    }

    #[embassy::task]
    async fn embassy_task() {
        defmt::info!("hello from embassy_task");
    }

    #[embassy::task]
    async fn embassy_task2() {
        defmt::info!("hello from embassy_task2");
    }

    #[embassy::task]
    async fn softdevice_task(sd: &'static Softdevice) {
        sd.run().await;
    }

    // ----- Structs -----

    pub struct HidioInterface<const H: usize> {
        mcu: Option<String<8>>,
        serial: Option<String<126>>,
    }

    impl<const H: usize> HidioInterface<H> {
        fn new(mcu: Option<String<8>>, serial: Option<String<126>>) -> Self {
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
        hidio_intf: HidioCommandInterface,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        layer_state: LayerState,
        matrix: Matrix,
        timer1: Timer<TIMER1, Periodic>,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
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
            clocks: Option<Clocks<clocks::ExternalOscillator, clocks::Internal, clocks::LfOscStopped>> = None,
            ctrl_queue: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new(),
            kbd_queue: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new(),
            mouse_queue: Queue<kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE> = Queue::new(),
            serial_number: String<126> = String::new(),
            usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
    ])]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        defmt::info!(">>>> Initializing <<<<");

        // Show processor registers
        defmt::trace!("MSP: {:#010x}", cortex_m::register::msp::read());
        defmt::trace!("PSP: {:#010x}", cortex_m::register::psp::read());

        // Determine which chip is running
        let mut chip: String<8> = String::new();
        write!(
            chip,
            "nRF{:05x}",
            cx.device.FICR.info.part.read().bits()
        ).unwrap();

        // Setup main and slow clocks
        defmt::trace!("Clock initialization");
        let clocks = Clocks::new(cx.device.CLOCK);
        *cx.local.clocks = Some(clocks.enable_ext_hfosc());
        let clocks = cx.local.clocks.as_ref().unwrap();

        // Setup gpios
        defmt::trace!("GPIO initialization");
        let p0 = gpio::p0::Parts::new(cx.device.P0);

        let sense1 = p0.p0_02.into_pulldown_input().degrade();
        let sense2 = p0.p0_03.into_pulldown_input().degrade();
        let sense3 = p0.p0_04.into_pulldown_input().degrade();
        let sense4 = p0.p0_05.into_pulldown_input().degrade();
        let sense5 = p0.p0_28.into_pulldown_input().degrade();

        let strobe1 = p0.p0_06.into_push_pull_output(Level::Low).degrade();
        let strobe2 = p0.p0_07.into_push_pull_output(Level::Low).degrade();
        let strobe3 = p0.p0_08.into_push_pull_output(Level::Low).degrade();
        let strobe4 = p0.p0_09.into_push_pull_output(Level::Low).degrade();
        let strobe5 = p0.p0_10.into_push_pull_output(Level::Low).degrade();
        let strobe6 = p0.p0_11.into_push_pull_output(Level::Low).degrade();
        let strobe7 = p0.p0_12.into_push_pull_output(Level::Low).degrade();
        let strobe8 = p0.p0_13.into_push_pull_output(Level::Low).degrade();
        let strobe9 = p0.p0_14.into_push_pull_output(Level::Low).degrade();
        let strobe10 = p0.p0_15.into_push_pull_output(Level::Low).degrade();
        let strobe11 = p0.p0_16.into_push_pull_output(Level::Low).degrade();
        let strobe12 = p0.p0_17.into_push_pull_output(Level::Low).degrade();
        let strobe13 = p0.p0_19.into_push_pull_output(Level::Low).degrade();
        let strobe14 = p0.p0_20.into_push_pull_output(Level::Low).degrade();
        let strobe15 = p0.p0_21.into_push_pull_output(Level::Low).degrade();
        let strobe16 = p0.p0_22.into_push_pull_output(Level::Low).degrade();

        // Retrieve unique id
        write!(
            cx.local.serial_number,
            "{:08x}{:08x}",
            cx.device.FICR.deviceid[0].read().bits(), cx.device.FICR.deviceid[1].read().bits()
        )
        .unwrap();
        defmt::info!("UID: {}", cx.local.serial_number);

        // Setup Keyscanning Matrix
        defmt::trace!("Keyscanning Matrix initialization");
        let cols = [
            strobe1,
            strobe2,
            strobe3,
            strobe4,
            strobe5,
            strobe6,
            strobe7,
            strobe8,
            strobe9,
            strobe10,
            strobe11,
            strobe12,
            strobe13,
            strobe14,
            strobe15,
            strobe16,
        ];
        let rows = [
            sense1,
            sense2,
            sense3,
            sense4,
            sense5,
        ];
        let mut matrix = Matrix::new(cols, rows).unwrap();
        matrix.next_strobe().unwrap(); // Initial strobe

        // Setup kll-core
        let loop_condition_lookup: &[u32] = &[0]; // TODO: Use KLL Compiler

        // Load datastructures into kll-core
        let layer_lookup = LayerLookup::new(
            crate::kll::LAYER_LOOKUP,
            crate::kll::TRIGGER_GUIDES,
            crate::kll::RESULT_GUIDES,
            crate::kll::TRIGGER_RESULT_MAPPING,
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
            HidioInterface::<MESSAGE_LEN>::new(Some(chip), Some(cx.local.serial_number.clone())),
        )
        .unwrap();

        // Setup USB
        defmt::trace!("USB initialization");
        let (kbd_producer, kbd_consumer) = cx.local.kbd_queue.split();
        let (_mouse_producer, _mouse_consumer) = cx.local.mouse_queue.split();
        let (ctrl_producer, ctrl_consumer) = cx.local.ctrl_queue.split();
        *cx.local.usb_bus = Some(Usbd::new(UsbPeripheral::new(cx.device.USBD, clocks)));
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();
        let usb_hid = HidInterface::new(
            usb_bus,
            HidCountryCode::NotSupported,
            kbd_consumer,
            //mouse_consumer,
            ctrl_consumer,
        );
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer(USB_MANUFACTURER)
            .max_packet_size_0(64)
            .max_power(500)
            .product(USB_PRODUCT)
            .supports_remote_wakeup(true) // TODO Add support
            .serial_number(cx.local.serial_number)
            .device_release(VERGEN_GIT_COMMIT_COUNT.parse().unwrap())
            .build();

        // Setup main timer
        let mut timer1 = Timer::periodic(cx.device.TIMER1);
        timer1.start(SCAN_PERIOD_US);
        defmt::trace!("TIMER1 started");
        timer1.enable_interrupt();

        // TODO - GPIO pin sleep/wakeup configuration
        // TODO - RGB PWM indicator configuration
        // TODO - BLE Setup
        // TODO - Softdevice setup?
        // TODO - Charging/Standby detection
        // TODO - BLE Profiles

        (
            Shared {
                ctrl_producer,
                hidio_intf,
                kbd_producer,
                layer_state,
                matrix,
                timer1,
                usb_dev,
                usb_hid,
            },
            Local {},
            init::Monotonics {},
        )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        //let sd = Softdevice::enable(&config);

        let executor = EXECUTOR.put(Executor::new());
        executor.run(|spawner| {
            defmt::unwrap!(spawner.spawn(embassy_task()));
            defmt::unwrap!(spawner.spawn(embassy_task2()));
            //unwrap!(spawner.spawn(softdevice_task(sd)));
            //unwrap!(spawner.spawn(bluetooth_task(sd)));
        });

        loop {
            // Now Wait For Interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    /// Keyscanning Task (Uses TC0)
    /// High-priority scheduled tasks as consistency is more important than speed for scanning
    /// key states
    /// Scans one strobe at a time
    #[task(priority = 5, binds = TIMER1, shared = [hidio_intf, layer_state, matrix, timer1])]
    fn timer1(mut cx: timer1::Context) {
        let process_macros = cx.shared.matrix.lock(|matrix| {
            cx.shared.layer_state.lock(|layer_state| {
                // Scan one strobe (strobes have already been enabled and allowed to settle)
                if let Ok((reading, strobe)) = matrix.sense::<Void>() {
                    for (i, entry) in reading.iter().enumerate() {
                        for event in entry.trigger_event(strobe * RSIZE + i) {
                            let hidio_event = HidIoEvent::TriggerEvent(event);

                            // Enqueue KLL trigger event
                            let ret = layer_state.process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                            assert!(ret.is_ok(), "Failed to enqueue: {:?} - {:?}", event, ret);

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
                matrix.next_strobe::<Void>().unwrap() == 0
            })
        });

        // If a full matrix scanning cycle has finished, process macros
        if process_macros && macro_process::spawn().is_err() {
            defmt::warn!("Could not schedule macro_process");
        }
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(priority = 6, shared = [ctrl_producer, hidio_intf, kbd_producer, layer_state, matrix])]
    fn macro_process(mut cx: macro_process::Context) {
        cx.shared.layer_state.lock(|layer_state| {
            // Confirm off-state lookups
            cx.shared.matrix.lock(|matrix| {
                layer_state.process_off_state_lookups::<MAX_LAYER_LOOKUP_SIZE>(&|index| {
                    matrix.generate_event(index).unwrap().trigger_event(index)
                });
            });

            // Finalize triggers to generate CapabilityRun events
            for cap_run in layer_state.finalize_triggers::<MAX_LAYER_LOOKUP_SIZE>() {
                match cap_run {
                    kll_core::CapabilityRun::NoOp { .. } => {}
                    kll_core::CapabilityRun::HidKeyboard { .. }
                    | kll_core::CapabilityRun::HidKeyboardState { .. } => {
                        cx.shared.kbd_producer.lock(|kbd_producer| {
                            kiibohd_usb::enqueue_keyboard_event(cap_run, kbd_producer).unwrap();
                        })
                    }
                    kll_core::CapabilityRun::HidProtocol { .. } => {}
                    kll_core::CapabilityRun::HidConsumerControl { .. }
                    | kll_core::CapabilityRun::HidSystemControl { .. } => {
                        cx.shared.ctrl_producer.lock(|ctrl_producer| {
                            kiibohd_usb::enqueue_ctrl_event(cap_run, ctrl_producer).unwrap();
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
    #[task(priority = 6, shared = [usb_hid])]
    fn usb_process(cx: usb_process::Context) {
        let mut usb_hid = cx.shared.usb_hid;
        usb_hid.lock(|usb_hid| {
            // Commit USB events
            usb_hid.push();
        });
    }

    /// USB Device Interupt
    #[task(priority = 2, binds = USBD, shared = [hidio_intf, usb_dev, usb_hid])]
    fn usbd(cx: usbd::Context) {
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

// ----- Other -----

#[exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault!");
}

// Timestamps currently use the cycle counter
// TODO: Use something better and easier to read? (but still fast)
defmt::timestamp!("{=u32} us", {
    // TODO (HaaTa): Determine a way to calculate the divider automatically
    //               Or transition to a hardware timer?
    cortex_m::peripheral::DWT::cycle_count() / 64
});

/// Same panicking *behavior* as `panic-probe` but doesn't print a panic message
/// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
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
