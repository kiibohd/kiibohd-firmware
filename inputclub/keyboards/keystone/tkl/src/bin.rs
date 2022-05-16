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
#[rtic::app(device = keystonetkl::hal::pac, peripherals = true, dispatchers = [UART1, USART0, USART1, SSC, PWM, ACC, TWI0, TWI1])]
mod app {
    use crate::FLASH_CONFIG;
    use const_env::from_env;
    use core::fmt::Write;
    use heapless::spsc::{Producer, Queue};
    use heapless::String;
    use is31fl3743b::Is31fl3743bAtsam4Dma;
    use kiibohd_hid_io::*;
    use kiibohd_usb::HidCountryCode;

    use keystonetkl::{
        hal::{
            adc::{Adc, AdcPayload, Continuous, SingleEndedGain},
            chipid::ChipId,
            clock::{ClockController, MainClock, SlowClock},
            efc::Efc,
            gpio::*,
            pac::TC0,
            pdc::{ReadDma, ReadDmaPaused, ReadWriteDmaLen, RxDma, RxTxDma, Transfer, W},
            prelude::*,
            rtt::RealTimeTimer,
            spi::{SpiMaster, SpiPayload, Variable},
            time::duration::Extensions,
            timer::{ClockSource, TimerCounter, TimerCounterChannel},
            udp::{
                usb_device,
                usb_device::{
                    bus::UsbBusAllocator,
                    device::{UsbDeviceBuilder, UsbVidPid},
                },
                UdpBus,
            },
            watchdog::Watchdog,
        },
        kll, Pins,
    };

    // ----- Sizes -----

    const BUF_CHUNK: usize = 64;
    const ID_LEN: usize = 10;
    const RX_BUF: usize = 8;
    const SERIALIZATION_LEN: usize = 277;
    const TX_BUF: usize = 8;

    const CSIZE: usize = 18; // Number of columns
    const RSIZE: usize = 6; // Number of rows
    const MSIZE: usize = RSIZE * CSIZE; // Total matrix size
    const ADC_SAMPLES: usize = 2; // Number of samples per key per strobe
    const ADC_BUF_SIZE: usize = ADC_SAMPLES * RSIZE; // Size of ADC buffer per strobe
    const INVERT_STROBE: bool = true; // P-Mosfets need to be inverted
    const ISSI_DRIVER_CHIPS: usize = 2;
    const ISSI_DRIVER_QUEUE_SIZE: usize = 5;
    const ISSI_DRIVER_CS_LAYOUT: [u8; ISSI_DRIVER_CHIPS] = [0, 1];
    // Must be 256 or less, or a power of 2; e.g. 512 due limitations with embedded-dma
    // Actual value should be -> ISSI_DRIVER_CHIPS * 198 (e.g. 396);
    // Size is determined by the largest SPI tx transaction
    const SPI_TX_BUF_SIZE: usize = 512;
    // Size is determined by the largest SPI rx transaction
    const SPI_RX_BUF_SIZE: usize = (32 + 2) * ISSI_DRIVER_CHIPS;

    const CTRL_QUEUE_SIZE: usize = 2;
    const KBD_QUEUE_SIZE: usize = 2;
    const MOUSE_QUEUE_SIZE: usize = 2;

    // KLL Constants
    // TODO - Tune
    const LAYOUT_SIZE: usize = 256;
    const MAX_ACTIVE_LAYERS: usize = 2;
    const MAX_ACTIVE_TRIGGERS: usize = 2;
    const MAX_LAYERS: usize = 2;
    const MAX_LAYER_STACK_CACHE: usize = 2;
    const MAX_LAYER_LOOKUP_SIZE: usize = 2;
    const MAX_OFF_STATE_LOOKUP: usize = 2;
    const STATE_SIZE: usize = 2;

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

    type AdcTransfer = Transfer<W, &'static mut [u16; ADC_BUF_SIZE], RxDma<AdcPayload<Continuous>>>;
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
    type Matrix = kiibohd_hall_effect_keyscanning::Matrix<
        PioX<Output<PushPull>>,
        CSIZE,
        MSIZE,
        INVERT_STROBE,
    >;
    type SpiTransferRxTx = Transfer<
        W,
        (
            &'static mut [u32; SPI_RX_BUF_SIZE],
            &'static mut [u32; SPI_TX_BUF_SIZE],
        ),
        RxTxDma<SpiPayload<Variable, u32>>,
    >;
    type SpiParkedDma = (
        SpiMaster<u32>,
        &'static mut [u32; SPI_RX_BUF_SIZE],
        &'static mut [u32; SPI_TX_BUF_SIZE],
    );
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
        adc: Option<AdcTransfer>,
        ctrl_producer: Producer<'static, kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE>,
        debug_led: Pa15<Output<PushPull>>,
        hidio_intf: HidioCommandInterface,
        issi: Is31fl3743bAtsam4Dma<ISSI_DRIVER_CHIPS, ISSI_DRIVER_QUEUE_SIZE>,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        layer_state: LayerState,
        matrix: Matrix,
        rtt: RealTimeTimer,
        spi: Option<SpiParkedDma>,
        spi_rxtx: Option<SpiTransferRxTx>,
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
            adc_buf: [u16; ADC_BUF_SIZE] = [0; ADC_BUF_SIZE],
            ctrl_queue: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new(),
            kbd_queue: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new(),
            mouse_queue: Queue<kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE> = Queue::new(),
            serial_number: String<126> = String::new(),
            spi_tx_buf: [u32; SPI_TX_BUF_SIZE] = [0; SPI_TX_BUF_SIZE],
            spi_rx_buf: [u32; SPI_RX_BUF_SIZE] = [0; SPI_RX_BUF_SIZE],
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
        let mut pins = Pins::new(gpio_ports, &cx.device.MATRIX);

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

        // Setup hall effect matrix
        defmt::trace!("HE Matrix initialization");
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
            pins.strobe18.downgrade(),
        ];
        let mut matrix = Matrix::new(cols).unwrap();
        matrix.next_strobe().unwrap(); // Strobe first column

        // Setup ADC for hall effect matrix
        defmt::trace!("ADC initialization");
        let gain = SingleEndedGain::Gain4x;
        let offset = true;
        let mut adc = Adc::new(
            cx.device.ADC,
            clocks.peripheral_clocks.adc.into_enabled_clock(),
        );

        adc.enable_channel(&mut pins.sense1);
        adc.enable_channel(&mut pins.sense2);
        adc.enable_channel(&mut pins.sense3);
        adc.enable_channel(&mut pins.sense4);
        adc.enable_channel(&mut pins.sense5);
        adc.enable_channel(&mut pins.sense6);

        adc.gain(&mut pins.sense1, gain);
        adc.gain(&mut pins.sense2, gain);
        adc.gain(&mut pins.sense3, gain);
        adc.gain(&mut pins.sense4, gain);
        adc.gain(&mut pins.sense5, gain);
        adc.gain(&mut pins.sense6, gain);

        adc.offset(&mut pins.sense1, offset);
        adc.offset(&mut pins.sense2, offset);
        adc.offset(&mut pins.sense3, offset);
        adc.offset(&mut pins.sense4, offset);
        adc.offset(&mut pins.sense5, offset);
        adc.offset(&mut pins.sense6, offset);

        adc.autocalibration(true);
        adc.enable_rxbuff_interrupt(); // TODO Re-enable
        let adc = adc.with_continuous_pdc();

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

        // Setup SPI for LED Drivers
        defmt::trace!("SPI ISSI Driver initialization");
        let wdrbt = false; // Wait data read before transfer enabled
        let llb = false; // Local loopback
                         // Cycles to delay between consecutive transfers
        let dlybct = 0; // No delay
        let mut spi = SpiMaster::<u32>::new(
            cx.device.SPI,
            clocks.peripheral_clocks.spi.into_enabled_clock(),
            pins.spi_miso,
            pins.spi_mosi,
            pins.spi_sck,
            atsam4_hal::spi::PeripheralSelectMode::Variable,
            wdrbt,
            llb,
            dlybct,
        );

        // Setup each CS channel
        let mode = atsam4_hal::spi::spi::MODE_3;
        let csa = atsam4_hal::spi::ChipSelectActive::ActiveAfterTransfer;
        let bits = atsam4_hal::spi::BitWidth::Width8Bit;
        let baud = atsam4_hal::spi::Hertz(12_000_000_u32);
        // Cycles to delay from CS to first valid SPCK
        let dlybs = 0; // Half an SPCK clock period
        let cs_settings =
            atsam4_hal::spi::ChipSelectSettings::new(mode, csa, bits, baud, dlybs, dlybct);
        for i in 0..ISSI_DRIVER_CHIPS {
            spi.cs_setup(i as u8, cs_settings.clone()).unwrap();
        }
        spi.enable_txbufe_interrupt();

        // Setup SPI with pdc
        let spi = spi.with_pdc_rxtx();

        // Setup ISSI LED Driver
        let issi_default_brightness = 255; // TODO compile-time configuration + flash default
        let issi_default_enable = true; // TODO compile-time configuration + flash default
        let mut issi = Is31fl3743bAtsam4Dma::<ISSI_DRIVER_CHIPS, ISSI_DRIVER_QUEUE_SIZE>::new(
            ISSI_DRIVER_CS_LAYOUT,
            issi_default_brightness,
            issi_default_enable,
        );

        // TODO Move scaling and pwm initialization to kll pixelmap setup
        for chip in issi.pwm_page_buf() {
            chip.iter_mut().for_each(|e| *e = 255);
        }
        for chip in issi.scaling_page_buf() {
            chip.iter_mut().for_each(|e| *e = 100);
        }
        defmt::info!("pwm: {:?}", issi.pwm_page_buf());
        defmt::info!("scaling: {:?}", issi.scaling_page_buf());

        // Start ISSI LED Driver initialization
        issi.reset().unwrap(); // Queue reset DMA transaction
        issi.scaling().unwrap(); // Queue scaling default
        issi.pwm().unwrap(); // Queue pwm default
        let (rx_len, tx_len) = issi.tx_function(cx.local.spi_tx_buf).unwrap();
        let spi_rxtx = spi.read_write_len(cx.local.spi_rx_buf, rx_len, cx.local.spi_tx_buf, tx_len);

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
        let (_mouse_producer, _mouse_consumer) = cx.local.mouse_queue.split();
        let (ctrl_producer, ctrl_consumer) = cx.local.ctrl_queue.split();
        let udp_bus = UdpBus::new(
            cx.device.UDP,
            clocks.peripheral_clocks.udp,
            pins.udp_ddm,
            pins.udp_ddp,
        );
        *cx.local.usb_bus = Some(UsbBusAllocator::<UdpBus>::new(udp_bus));
        let usb_bus = cx.local.usb_bus.as_ref().unwrap();
        let usb_hid = HidInterface::new(
            usb_bus,
            HidCountryCode::NotSupported,
            kbd_consumer,
            //mouse_consumer,
            ctrl_consumer,
        );
        let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer(USB_MANUFACTURER)
            .max_packet_size_0(64)
            .max_power(500)
            .product(USB_PRODUCT)
            .supports_remote_wakeup(true) // TODO Add support
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
        tcc0.start(200_000u32.nanoseconds());
        //tcc0.clock_input(ClockSource::Slck32768Hz);
        //tcc0.start(500_000_000u32.nanoseconds());
        defmt::trace!("TCC0 started");
        tcc0.enable_interrupt();

        // Setup secondary timer (used for watchdog, activity led and sleep related functionality)
        let mut rtt = RealTimeTimer::new(cx.device.RTT, 3, false);
        rtt.start(500_000u32.microseconds());
        rtt.enable_alarm_interrupt();
        defmt::trace!("RTT Timer started");

        (
            Shared {
                adc: Some(adc.read(cx.local.adc_buf)),
                ctrl_producer,
                debug_led: pins.debug_led,
                hidio_intf,
                issi,
                kbd_producer,
                layer_state,
                matrix,
                rtt,
                spi: None,
                spi_rxtx: Some(spi_rxtx),
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
    #[task(priority = 13, binds = TC0, shared = [adc, issi, layer_state, tcc0, spi, spi_rxtx])]
    fn tc0(mut cx: tc0::Context) {
        cx.shared.tcc0.lock(|w| w.clear_interrupt_flags());

        let process_macros = false;

        /*
        // TODO Determine best place to reinitialize SPI PDC
        cx.shared.spi.lock(|spi| {
            spi.enable_txbufe_interrupt();
            let spi = spi.with_pdc_rxtx();
            if let Some((spi, rx_buf, tx_buf)) = spi.take() {
                cx.shared.issi.lock(|issi| {
                    if let Ok((rx_len, tx_len)) = issi.tx_function(tx_buf) {
                        cx.shared.spi_rxtx.lock(|spi_rxtx| {
                            spi_rxtx.replace(spi.read_write_len(rx_buf, rx_len, tx_buf, tx_len));
                        });
                    }
                });
            }
        });
        */

        // Start next ADC DMA buffer read
        cx.shared.adc.lock(|adc| {
            if let Some(adc) = adc {
                adc.resume();
            }
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
        //cx.shared.debug_led.lock(|w| w.toggle().ok());
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(priority = 10, shared = [ctrl_producer, kbd_producer, layer_state, matrix])]
    fn macro_process(mut cx: macro_process::Context) {
        cx.shared.layer_state.lock(|layer_state| {
            // Confirm off-state lookups
            cx.shared.matrix.lock(|matrix| {
                layer_state.process_off_state_lookups::<MAX_LAYER_LOOKUP_SIZE>(&|index| {
                    matrix.generate_event(index)
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
    #[task(priority = 11, shared = [usb_hid])]
    fn usb_process(cx: usb_process::Context) {
        let mut usb_hid = cx.shared.usb_hid;
        usb_hid.lock(|usb_hid| {
            // Commit USB events
            usb_hid.push();
        });
    }

    /// ADC Interrupt
    #[task(priority = 14, binds = ADC, shared = [adc, hidio_intf, layer_state, matrix])]
    fn adc(mut cx: adc::Context) {
        let mut adc = cx.shared.adc;
        let mut matrix = cx.shared.matrix;
        let mut layer_state = cx.shared.layer_state;

        adc.lock(|adc_pdc| {
            // Retrieve DMA buffer
            let (buf, adc) = adc_pdc.take().unwrap().wait();
            defmt::trace!("DMA BUF: {}", buf);

            matrix.lock(|matrix| {
                layer_state.lock(|layer_state| {
                    // Current strobe
                    let strobe = matrix.strobe();

                    // Process retrieved ADC buffer
                    // Loop through buffer. The buffer may have multiple buffers for each key.
                    // For example, 12 entries + 6 rows, column 1:
                    //  Col Row Sample: Entry
                    //    1   0      0  6 * 1 + 0 = 6
                    //    1   1      1  6 * 1 + 1 = 7
                    //    1   2      2  6 * 1 + 2 = 8
                    //    1   3      3  6 * 1 + 3 = 9
                    //    1   4      4  6 * 1 + 4 = 10
                    //    1   5      5  6 * 1 + 5 = 11
                    //    1   0      6  6 * 1 + 0 = 6
                    //    1   1      7  6 * 1 + 1 = 7
                    //    1   2      8  6 * 1 + 2 = 8
                    //    0   3      9  6 * 1 + 3 = 9
                    //    0   4     10  6 * 1 + 4 = 10
                    //    0   5     11  6 * 1 + 5 = 11
                    for (i, sample) in buf.iter().enumerate() {
                        let index = RSIZE * strobe + i - (i / RSIZE) * RSIZE;
                        match matrix.record::<ADC_SAMPLES>(index, *sample) {
                            Ok(val) => {
                                // If data bucket has accumulated enough samples, pass to the next stage
                                if let Some(sense) = val {
                                    for event in sense.trigger_event(index, false) {
                                        let hidio_event = HidIoEvent::TriggerEvent(event);

                                        // Enqueue KLL trigger event
                                        let ret = layer_state
                                            .process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                                        assert!(
                                            ret.is_ok(),
                                            "Failed to enqueue: {:?} - {:?}",
                                            event,
                                            ret
                                        );

                                        // Enqueue HID-IO trigger event
                                        cx.shared.hidio_intf.lock(|hidio_intf| {
                                            if let Err(err) = hidio_intf.process_event(hidio_event)
                                            {
                                                defmt::error!(
                                                    "Hidio TriggerEvent Error: {:?}",
                                                    err
                                                );
                                            }
                                        });
                                    }
                                }
                            }
                            Err(e) => {
                                defmt::error!(
                                    "Sample record failed ({}, {}, {}):{} -> {}",
                                    i,
                                    strobe,
                                    index,
                                    sample,
                                    e
                                );
                            }
                        }
                    }

                    // Strobe next column
                    if let Ok(strobe) = matrix.next_strobe() {
                        // On strobe wrap-around, schedule event processing
                        if strobe == 0 {
                            defmt::warn!("STROBE WRAP-AROUND");
                            if macro_process::spawn().is_err() {
                                defmt::warn!("Could not schedule macro_process");
                            }
                        }
                    }
                });
            });

            // Prepare next DMA read, but don't start it yet
            adc_pdc.replace(adc.read_paused(buf));
        });
    }

    /// SPI Interrupt
    #[task(priority = 12, binds = SPI, shared = [issi, spi, spi_rxtx])]
    fn spi(mut cx: spi::Context) {
        let mut issi = cx.shared.issi;
        let mut spi_rxtx = cx.shared.spi_rxtx;

        spi_rxtx.lock(|spi_rxtx| {
            // Retrieve DMA buffer
            if let Some(spi_buf) = spi_rxtx.take() {
                let ((rx_buf, tx_buf), spi) = spi_buf.wait();

                issi.lock(|issi| {
                    // Process Rx buffer if applicable
                    issi.rx_function(rx_buf).unwrap();

                    // Prepare the next DMA transaction
                    if let Ok((rx_len, tx_len)) = issi.tx_function(tx_buf) {
                        spi_rxtx.replace(spi.read_write_len(rx_buf, rx_len, tx_buf, tx_len));
                    } else {
                        // Disable PDC
                        let mut spi = spi.revert();
                        spi.disable_txbufe_interrupt();

                        // No more transactions ready, park spi peripheral and buffers
                        cx.shared.spi.lock(|spi_periph| {
                            spi_periph.replace((spi, rx_buf, tx_buf));
                        });
                    }
                });
            }
        });
    }

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
