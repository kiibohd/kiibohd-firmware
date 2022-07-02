// Copyright 2021-2022 Jacob Alexander
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

#![no_std]
#![no_main]

mod constants;
mod hidio;

use cortex_m_rt::exception;

// ----- RTIC -----

// RTIC requires that unused interrupts are declared in an extern block when
// using software tasks; these free interrupts will be used to dispatch the
// software tasks.
#[rtic::app(device = keystonetkl::hal::pac, peripherals = true, dispatchers = [UART1, USART0, USART1, SSC, PWM, ACC, TWI0, TWI1])]
mod app {
    use crate::constants::*;
    use crate::hidio::*;
    use core::fmt::Write;
    use dwt_systick_monotonic::*;
    use fugit::{HertzU32 as Hertz, RateExtU32};
    use heapless::spsc::{Consumer, Producer, Queue};
    use heapless::String;
    use is31fl3743b::Is31fl3743bAtsam4Dma;
    use kiibohd_hid_io::*;
    use kiibohd_usb::HidCountryCode;

    use keystonetkl::{
        hal::{
            adc::{Adc, AdcPayload, Continuous, SingleEndedGain},
            chipid::ChipId,
            clock::{ClockController, Enabled, MainClock, SlowClock, Tc0Clock, Tc1Clock, Tc2Clock},
            efc::Efc,
            gpio::*,
            pac::TC0,
            pdc::{ReadDma, ReadDmaPaused, ReadWriteDmaLen, RxDma, RxTxDma, Transfer, W},
            prelude::*,
            spi::{SpiMaster, SpiPayload, Variable},
            timer::{TimerCounter, TimerCounterChannel, TimerCounterChannels},
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

    // ----- Types -----

    #[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
    pub enum LedTest {
        /// No active test
        Disabled,
        /// Reset led controller (after reading test data, before setting disabled).
        Reset,
        /// Active test, need to query results from controller (next state is OpenReady)
        OpenQuery,
        /// Test finished, can read results directly (next state is Reset)
        OpenReady,
        /// Active test, need to query results from controller (next state is ShortReady)
        ShortQuery,
        /// Test finished, can read results directly (next state is Reset)
        ShortReady,
    }

    type AdcTransfer = Transfer<W, &'static mut [u16; ADC_BUF_SIZE], RxDma<AdcPayload<Continuous>>>;
    type HidInterface = kiibohd_usb::HidInterface<
        'static,
        UdpBus,
        KBD_QUEUE_SIZE,
        KBD_LED_QUEUE_SIZE,
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
    type Matrix = kiibohd_hall_effect_keyscanning::Matrix<
        PioX<Output<PushPull>>,
        CSIZE,
        MSIZE,
        INVERT_STROBE,
    >;
    type RealTimeTimer = keystonetkl::hal::rtt::RealTimeTimer<RTT_PRESCALER, false>;
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
    type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = DwtSystick<MCU_FREQ>;

    // ----- Structs -----

    //
    // Shared resources used by tasks/interrupts
    //
    #[shared]
    struct Shared {
        adc: Option<AdcTransfer>,
        hidio_intf: HidioCommandInterface,
        issi: Is31fl3743bAtsam4Dma<ISSI_DRIVER_CHIPS, ISSI_DRIVER_QUEUE_SIZE>,
        layer_state: LayerState,
        led_test: LedTest,
        matrix: Matrix,
        spi: Option<SpiParkedDma>,
        spi_rxtx: Option<SpiTransferRxTx>,
        usb_dev: UsbDevice,
        usb_hid: HidInterface,
    }

    //
    // Local resources, static mut variables, no locking necessary
    // (e.g. can be initialized in init and used in 1 other task function)
    //
    #[local]
    struct Local {
        ctrl_producer: Producer<'static, kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE>,
        debug_led: Pa15<Output<PushPull>>,
        kbd_led_consumer: Consumer<'static, kiibohd_usb::LedState, KBD_LED_QUEUE_SIZE>,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        mouse_producer: Producer<'static, kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE>,
        rtt: RealTimeTimer,
        tcc0: TimerCounterChannel<TC0, Tc0Clock<Enabled>, 0, TCC0_FREQ>,
        tcc1: TimerCounterChannel<TC0, Tc1Clock<Enabled>, 1, TCC1_FREQ>,
        wdt: Watchdog,
    }

    //
    // Initialization
    //
    #[init(
        local = [
            adc_buf: [u16; ADC_BUF_SIZE] = [0; ADC_BUF_SIZE],
            ctrl_queue: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new(),
            kbd_queue: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new(),
            kbd_led_queue: Queue<kiibohd_usb::LedState, KBD_LED_QUEUE_SIZE> = Queue::new(),
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
        let baud: Hertz = 12_u32.MHz();
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
                HidIoCommandId::GetInfo,
                HidIoCommandId::ManufacturingTest,
                HidIoCommandId::SupportedIds,
                HidIoCommandId::TestPacket,
            ],
            HidioInterface::<MESSAGE_LEN>::new(&chip, Some(cx.local.serial_number.clone())),
        )
        .unwrap();

        // Setup USB
        defmt::trace!("UDP initialization");
        let (kbd_producer, kbd_consumer) = cx.local.kbd_queue.split();
        let (kbd_led_producer, kbd_led_consumer) = cx.local.kbd_led_queue.split();
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
            kbd_led_producer,
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
        let tc0 = TimerCounter::new(cx.device.TC0);
        let tc0_chs: TimerCounterChannels<
            TC0,
            Tc0Clock<Enabled>,
            Tc1Clock<Enabled>,
            Tc2Clock<Enabled>,
            TCC0_FREQ,
            TCC1_FREQ,
            TCC2_FREQ,
        > = tc0.split(
            clocks.peripheral_clocks.tc_0.into_enabled_clock(),
            clocks.peripheral_clocks.tc_1.into_enabled_clock(),
            clocks.peripheral_clocks.tc_2.into_enabled_clock(),
        );

        // Keyscanning Timer
        let mut tcc0 = tc0_chs.ch0;
        tcc0.clock_input(TCC0_DIV);
        tcc0.start(200_u32.micros());
        defmt::trace!("TCC0 started - Keyscanning");
        tcc0.enable_interrupt();

        // LED Frame Timer
        let mut tcc1 = tc0_chs.ch1;
        tcc1.clock_input(TCC1_DIV);
        tcc1.start(17_u32.millis()); // 17 ms -> ~60 fps (16.6667 ms)
        defmt::trace!("TCC1 started - LED Frame Scheduling");
        tcc1.enable_interrupt();

        // Setup secondary timer (used for watchdog, activity led and sleep related functionality)
        let mut rtt = RealTimeTimer::new(cx.device.RTT);
        rtt.start(500_000u32.micros());
        rtt.enable_alarm_interrupt();
        defmt::trace!("RTT Timer started");

        // Initialize tickless monotonic timer
        let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, MCU_FREQ);
        defmt::trace!("DwtSystick (Monotonic) started");

        (
            Shared {
                adc: Some(adc.read(cx.local.adc_buf)),
                hidio_intf,
                issi,
                layer_state,
                led_test: LedTest::Disabled,
                matrix,
                spi: None,
                spi_rxtx: Some(spi_rxtx),
                usb_dev,
                usb_hid,
            },
            Local {
                ctrl_producer,
                debug_led: pins.debug_led,
                kbd_led_consumer,
                kbd_producer,
                mouse_producer,
                rtt,
                tcc0,
                tcc1,
                wdt,
            },
            init::Monotonics(mono),
        )
    }

    /// Timer task (TC0)
    /// - Keyscanning Task (Uses tcc0)
    ///   High-priority scheduled tasks as consistency is more important than speed for scanning
    ///   key states
    ///   Scans one strobe at a time
    #[task(priority = 13, binds = TC0, local = [
        tcc0,
    ], shared = [
        adc,
    ])]
    fn tc0(mut cx: tc0::Context) {
        // Check for keyscanning interrupt (tcc0)
        cx.shared.adc.lock(|adc| {
            if cx.local.tcc0.clear_interrupt_flags() {
                // Start next ADC DMA buffer read
                if let Some(adc) = adc {
                    adc.resume();
                }
            }
        });
    }

    /// Timer task (TC1)
    /// - LED frame scheduling (Uses tcc1)
    ///   Schedules a lower priority task which is skipped if the previous frame is still
    ///   processing
    #[task(priority = 13, binds = TC1, local = [
        tcc1,
    ], shared = [])]
    fn tc1(cx: tc1::Context) {
        // Check for LED frame scheduling interrupt
        if cx.local.tcc1.clear_interrupt_flags() {
            // Attempt to schedule LED frame
            if led_frame_process::spawn().is_err() {
                defmt::warn!("Unable to schedule frame...FPS unstable");
            }
        }
    }

    /// Activity tick
    /// Used visually determine MCU status
    #[task(priority = 1, binds = RTT, local = [
        debug_led,
        rtt,
        wdt,
    ], shared = [])]
    fn rtt(cx: rtt::Context) {
        cx.local.rtt.clear_interrupt_flags();

        // Feed watchdog
        cx.local.wdt.feed();

        // Blink debug led
        // TODO: Remove (or use feature flag)
        //cx.local.debug_led.toggle().ok();
    }

    /// LED Frame Processing Task
    /// Handles each LED frame, triggered at a constant rate.
    /// Frames are skipped if the previous frame is still processing.
    #[task(priority = 8, shared = [
        hidio_intf,
        issi,
        led_test,
        spi,
        spi_rxtx,
    ])]
    fn led_frame_process(mut cx: led_frame_process::Context) {
        cx.shared.issi.lock(|issi| {
            // Look for manufacturing test commands
            (cx.shared.hidio_intf, cx.shared.led_test).lock(|hidio_intf, led_test| {
                // Only check for new tests if one is not currently running
                match *led_test {
                    LedTest::Disabled => {
                        if hidio_intf.interface().manufacturing_config.led_short_test {
                            // Enqueue short test
                            issi.short_circuit_detect_setup().unwrap();
                            *led_test = LedTest::ShortQuery;
                            led_test::spawn_after(800_u32.micros()).unwrap();

                            hidio_intf
                                .mut_interface()
                                .manufacturing_config
                                .led_short_test = false;
                        } else if hidio_intf.interface().manufacturing_config.led_open_test {
                            // Enqueue open test
                            issi.open_circuit_detect_setup().unwrap();
                            *led_test = LedTest::OpenQuery;
                            led_test::spawn_after(800_u32.micros()).unwrap();

                            hidio_intf
                                .mut_interface()
                                .manufacturing_config
                                .led_open_test = false;
                        }
                    }
                    LedTest::Reset => {
                        // Reset LED state
                        // The PWM and Scaling registers are reset, but we have a full copy
                        // in memory on the MCU so it will be the previous state.
                        issi.reset().unwrap();
                        *led_test = LedTest::Disabled;
                    }
                    _ => {}
                }
            });

            // Enable SPI DMA to update frame
            (cx.shared.spi, cx.shared.spi_rxtx).lock(|spi_periph, spi_rxtx| {
                // We only need to re-enable DMA if the queue was previously empty and "parked"
                if let Some((mut spi, rx_buf, tx_buf)) = spi_periph.take() {
                    spi.enable_txbufe_interrupt();
                    let spi = spi.with_pdc_rxtx();
                    // Look for issi event queue
                    if let Ok((rx_len, tx_len)) = issi.tx_function(tx_buf) {
                        spi_rxtx.replace(spi.read_write_len(rx_buf, rx_len, tx_buf, tx_len));
                    } else {
                        // Nothing to do (repark dma)
                        let mut spi = spi.revert();
                        spi.disable_txbufe_interrupt();
                        spi_periph.replace((spi, rx_buf, tx_buf));
                    }
                }
            });
        });

        // Handle processing of "next" frame
        // If this takes too long, the next frame update won't be scheduled (i.e. it'll be
        // skipped).
        // TODO - KLL Pixelmap
        // TODO - HIDIO frame updates
    }

    /// LED Test Results
    /// Asynchronous task to handle LED test results (both short and open).
    /// This task is schedule at least 750 us after the test is started.
    #[task(priority = 7, shared = [
        hidio_intf,
        issi,
        led_test,
        usb_hid,
    ])]
    fn led_test(cx: led_test::Context) {
        // Check for test results
        (cx.shared.hidio_intf, cx.shared.issi, cx.shared.led_test, cx.shared.usb_hid).lock(
            |hidio_intf, issi, led_test, usb_hid| {
                match *led_test {
                    LedTest::ShortQuery => {
                        // Schedule read of the short test results
                        issi.short_circuit_detect_read().unwrap();
                        *led_test = LedTest::ShortReady;
                        // NOTE: This should be quick, but we don't want to poll
                        led_test::spawn_after(2_u32.millis()).unwrap();
                        led_frame_process::spawn().ok(); // Attempt to schedule frame earlier
                    }
                    LedTest::ShortReady => {
                        // Read short results
                        let short_results = issi.short_circuit_raw().unwrap();

                        // 1 byte id, 1 byte length, 32 bytes of data, 1 byte id, ...
                        // Buffer size defined by kiibohd_hidio
                        let mut data: heapless::Vec<u8, { kiibohd_hid_io::MESSAGE_LEN - 4 }> =
                            heapless::Vec::new();
                        data.push(0).unwrap(); // Id
                        data.push(32).unwrap(); // Length
                        data.extend_from_slice(&short_results[0]).unwrap(); // Data
                        data.push(1).unwrap(); // Id
                        data.push(32).unwrap(); // Length
                        data.extend_from_slice(&short_results[1]).unwrap(); // Data
                        hidio_intf
                            .h0051_manufacturingres(h0051::Cmd {
                                command: 0x0001,
                                argument: 0x0002,
                                data,
                            })
                            .unwrap();

                        *led_test = LedTest::Reset;
                        usb_hid.poll(hidio_intf); // Flush hidio packets
                    }
                    LedTest::OpenQuery => {
                        // Schedule read of the short test results
                        issi.open_circuit_detect_read().unwrap();
                        *led_test = LedTest::OpenReady;
                        // NOTE: This should be quick, but we don't want to poll
                        led_test::spawn_after(2_u32.millis()).unwrap();
                        led_frame_process::spawn().ok(); // Attempt to schedule frame earlier
                    }
                    LedTest::OpenReady => {
                        // Read short results
                        let open_results = issi.open_circuit_raw().unwrap();

                        // 1 byte id, 1 byte length, 32 bytes of data, 1 byte id, ...
                        // Buffer size defined by kiibohd_hidio
                        let mut data: heapless::Vec<u8, { kiibohd_hid_io::MESSAGE_LEN - 4 }> =
                            heapless::Vec::new();
                        data.push(0).unwrap(); // Id
                        data.push(32).unwrap(); // Length
                        data.extend_from_slice(&open_results[0]).unwrap(); // Data
                        data.push(1).unwrap(); // Id
                        data.push(32).unwrap(); // Length
                        data.extend_from_slice(&open_results[1]).unwrap(); // Data
                        hidio_intf
                            .h0051_manufacturingres(h0051::Cmd {
                                command: 0x0001,
                                argument: 0x0003,
                                data,
                            })
                            .unwrap();

                        *led_test = LedTest::Reset;
                        usb_hid.poll(hidio_intf); // Flush hidio packets
                    }
                    _ => {}
                }
            },
        );
    }

    /// Macro Processing Task
    /// Handles incoming key scan triggers and turns them into results (actions and hid events)
    /// Has a lower priority than keyscanning to schedule around it.
    #[task(priority = 10, local = [
        ctrl_producer,
        kbd_led_consumer,
        kbd_producer,
        mouse_producer,
    ], shared = [
        hidio_intf,
        layer_state,
        matrix,
    ])]
    fn macro_process(mut cx: macro_process::Context) {
        (cx.shared.layer_state, cx.shared.matrix).lock(|layer_state, matrix| {
            // Query HID LED Events
            cx.shared.hidio_intf.lock(|hidio_intf| {
                while let Some(state) = cx.local.kbd_led_consumer.dequeue() {
                    // Convert to a TriggerEvent
                    let event = state.trigger_event();
                    let hidio_event = HidIoEvent::TriggerEvent(event);

                    // Enqueue KLL trigger event
                    let ret = layer_state.process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                    debug_assert!(ret.is_ok(), "Failed to enqueue: {:?} - {:?}", event, ret);

                    // Enqueue HID-IO trigger event
                    if let Err(err) = hidio_intf.process_event(hidio_event) {
                        defmt::error!("Hidio TriggerEvent Error: {:?}", err);
                    }
                }
            });

            // Confirm off-state lookups
            layer_state.process_off_state_lookups::<MAX_LAYER_LOOKUP_SIZE>(&|index| {
                matrix.generate_event(index)
            });

            // Finalize triggers to generate CapabilityRun events
            for cap_run in layer_state.finalize_triggers::<MAX_LAYER_LOOKUP_SIZE>() {
                match cap_run {
                    kll_core::CapabilityRun::NoOp { .. } => {}
                    kll_core::CapabilityRun::HidKeyboard { .. }
                    | kll_core::CapabilityRun::HidKeyboardState { .. } => {
                        debug_assert!(
                            kiibohd_usb::enqueue_keyboard_event(cap_run, cx.local.kbd_producer)
                                .is_ok(),
                            "KBD_QUEUE_SIZE too small"
                        );
                    }
                    kll_core::CapabilityRun::HidProtocol { .. } => {}
                    kll_core::CapabilityRun::HidConsumerControl { .. }
                    | kll_core::CapabilityRun::HidSystemControl { .. } => {
                        debug_assert!(
                            kiibohd_usb::enqueue_ctrl_event(cap_run, cx.local.ctrl_producer)
                                .is_ok(),
                            "CTRL_QUEUE_SIZE too small"
                        );
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
    #[task(priority = 11, shared = [
        usb_dev,
        usb_hid,
    ])]
    fn usb_process(cx: usb_process::Context) {
        let usb_dev = cx.shared.usb_dev;
        let usb_hid = cx.shared.usb_hid;
        (usb_hid, usb_dev).lock(|usb_hid, usb_dev| {
            // Update USB events
            if usb_hid.update() {
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
            }
        });
    }

    /// ADC Interrupt
    #[task(priority = 14, binds = ADC, shared = [
        adc,
        hidio_intf,
        layer_state,
        matrix,
    ])]
    fn adc(mut cx: adc::Context) {
        let adc = cx.shared.adc;
        let matrix = cx.shared.matrix;
        let layer_state = cx.shared.layer_state;

        (adc, layer_state, matrix).lock(|adc_pdc, layer_state, matrix| {
            // Retrieve DMA buffer
            let (buf, adc) = adc_pdc.take().unwrap().wait();
            //defmt::trace!("DMA BUF: {}", buf);

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
            //    1   3      9  6 * 1 + 3 = 9
            //    1   4     10  6 * 1 + 4 = 10
            //    1   5     11  6 * 1 + 5 = 11
            for (i, sample) in buf.iter().enumerate() {
                let index = RSIZE * strobe + i - (i / RSIZE) * RSIZE;
                match matrix.record::<ADC_SAMPLES>(index, *sample) {
                    Ok(val) => {
                        // If data bucket has accumulated enough samples, pass to the next stage
                        if let Some(sense) = val {
                            for event in
                                sense.trigger_event(SWITCH_REMAP[index as usize] as usize, false)
                            {
                                let hidio_event = HidIoEvent::TriggerEvent(event);

                                // Enqueue KLL trigger event
                                let ret =
                                    layer_state.process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                                assert!(ret.is_ok(), "Failed to enqueue: {:?} - {:?}", event, ret);

                                /* TODO - Logs too noisy currently
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
                                */
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
                if strobe == 0 && macro_process::spawn().is_err() {
                    defmt::warn!("Could not schedule macro_process");
                }
            }

            // Prepare next DMA read, but don't start it yet
            adc_pdc.replace(adc.read_paused(buf));
        });
    }

    /// SPI Interrupt
    #[task(priority = 12, binds = SPI, shared = [
        issi,
        spi,
        spi_rxtx,
    ])]
    fn spi(cx: spi::Context) {
        let issi = cx.shared.issi;
        let spi_periph = cx.shared.spi;
        let spi_rxtx = cx.shared.spi_rxtx;

        (spi_periph, spi_rxtx, issi).lock(|spi_periph, spi_rxtx, issi| {
            // Retrieve DMA buffer
            if let Some(spi_buf) = spi_rxtx.take() {
                let ((rx_buf, tx_buf), spi) = spi_buf.wait();

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
                    spi_periph.replace((spi, rx_buf, tx_buf));
                }
            }
        });
    }

    /// USB Device Interupt
    #[task(priority = 14, binds = UDP, shared = [
        hidio_intf,
        usb_dev,
        usb_hid,
    ])]
    fn udp(cx: udp::Context) {
        let usb_dev = cx.shared.usb_dev;
        let usb_hid = cx.shared.usb_hid;
        let hidio_intf = cx.shared.hidio_intf;

        // Poll USB endpoints
        (usb_dev, usb_hid, hidio_intf).lock(|usb_dev, usb_hid, hidio_intf| {
            if usb_dev.poll(&mut usb_hid.interfaces()) {
                // Retrive HID Lock LED events
                usb_hid.pull();

                // Process HID-IO
                usb_hid.poll(hidio_intf);
            }
        });
    }
}

#[exception]
unsafe fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault!");
}

defmt::timestamp!("{=u64} us", {
    atsam4_hal::timer::DwtTimer::<{ constants::MCU_FREQ }>::now()
        / ((constants::MCU_FREQ / 1_000_000) as u64)
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
