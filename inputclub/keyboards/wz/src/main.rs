// Copyright 2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]
#![no_main]

use cortex_m_rt::exception;
use defmt_rtt as _;
use nrf52833_hal as hal;
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
#[app(device = crate::hal::pac)]
mod app {
    use const_env::from_env;
    use core::convert::Infallible;
    use core::fmt::Write;
    use heapless::spsc::{Producer, Queue};
    use heapless::String;
    use kiibohd_hid_io::*;
    use kiibohd_usb::HidCountryCode;
    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDeviceBuilder, UsbVidPid},
    };

    use crate::hal::{
        clocks::{self, Clocks},
        pac::USBD,
        usbd::{UsbPeripheral, Usbd},
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
    const MAX_ACTIVE_TRIGGERS: usize = 2;
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
    // TODO - GPIO
    /*
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
    */
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

    // ----- Structs -----

    pub struct HidioInterface<const H: usize> {
        mcu: Option<String<12>>,
        serial: Option<String<126>>,
    }

    impl<const H: usize> HidioInterface<H> {
        fn new(serial: Option<String<126>>) -> Self {
            /*
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
            */
            // TODO
            let mcu: String<12> = String::from("TODO");
            let mcu = Some(mcu);

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
        //debug_led: Pb0<Output<PushPull>>,
        hidio_intf: HidioCommandInterface,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        layer_state: LayerState,
        //matrix: Matrix,
        //rtt: RealTimeTimer,
        //tcc0: TimerCounterChannel<TC0, 0>,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
        //wdt: Watchdog,
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

        // TODO Chip
        // Determine which chip is running
        /*
        let chip = ChipId::new(cx.device.CHIPID);
        defmt::info!("MCU: {:?}", chip.model());
        */

        // Setup main and slow clocks
        defmt::trace!("Clock initialization");
        let clocks = Clocks::new(cx.device.CLOCK);
        *cx.local.clocks = Some(clocks.enable_ext_hfosc());
        let clocks = cx.local.clocks.as_ref().unwrap();

        // TODO gpios
        // Setup gpios
        /*
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
        */

        // TODO - WDG (do we need a watchdog with the softdevice?)
        // Prepare watchdog to be fed
        /*
        let mut wdt = Watchdog::new(cx.device.WDT);
        wdt.feed();
        defmt::trace!("Watchdog first feed");
        */

        // TODO Get unique id (DEVICEID)
        /*
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
        */

        // TODO - GPIOS
        // Setup Keyscanning Matrix
        /*
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
        */

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
            HidioInterface::<MESSAGE_LEN>::new(Some(cx.local.serial_number.clone())),
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

        // TODO - Main timer
        /*
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
        */

        // TODO - Activity timer?
        /*
        // Setup secondary timer (used for watchdog, activity led and sleep related functionality)
        let mut rtt = RealTimeTimer::new(cx.device.RTT, 3, false);
        rtt.start(500_000u32.microseconds());
        rtt.enable_alarm_interrupt();
        defmt::trace!("RTT Timer started");
        */

        // TODO - GPIO pin sleep/wakeup configuration
        // TODO - RGB PWM indicator configuration
        // TODO - BLE Setup
        // TODO - Softdevice setup?
        // TODO - Charging/Standby detection
        // TODO - BLE Profiles

        (
            Shared {
                ctrl_producer,
                //debug_led: pins.debug_led,
                hidio_intf,
                kbd_producer,
                layer_state,
                //matrix,
                //rtt,
                //tcc0,
                usb_dev,
                usb_hid,
                //wdt,
            },
            Local {},
            init::Monotonics {},
        )
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
