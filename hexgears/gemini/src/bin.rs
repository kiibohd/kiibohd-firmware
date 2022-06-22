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
#[rtic::app(device = gemini::hal::pac, peripherals = true, dispatchers = [UART1, USART0, USART1, SSC, PWM, ACC, ADC, SPI])]
mod app {
    use crate::constants::*;
    use crate::hidio::*;
    use core::convert::Infallible;
    use core::fmt::Write;
    use dwt_systick_monotonic::*;
    use heapless::spsc::{Consumer, Producer, Queue};
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

    type RealTimeTimer = gemini::hal::rtt::RealTimeTimer<RTT_PRESCALER, false>;
    type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = DwtSystick<MCU_FREQ>;

    // ----- Structs -----

    //
    // Shared resources used by tasks/interrupts
    //
    #[shared]
    struct Shared {
        hidio_intf: HidioCommandInterface,
        layer_state: LayerState,
        matrix: Matrix,
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
        debug_led: Pb0<Output<PushPull>>,
        kbd_led_consumer: Consumer<'static, kiibohd_usb::LedState, KBD_LED_QUEUE_SIZE>,
        kbd_producer: Producer<'static, kiibohd_usb::KeyState, KBD_QUEUE_SIZE>,
        mouse_producer: Producer<'static, kiibohd_usb::MouseState, MOUSE_QUEUE_SIZE>,
        rtt: RealTimeTimer,
        tcc0: TimerCounterChannel<TC0, 0, TCC0_FREQ>,
        tcc1: TimerCounterChannel<TC0, 1, TCC1_FREQ>,
        wdt: Watchdog,
    }

    //
    // Initialization
    //
    #[init(
        local = [
            ctrl_queue: Queue<kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE> = Queue::new(),
            kbd_queue: Queue<kiibohd_usb::KeyState, KBD_QUEUE_SIZE> = Queue::new(),
            kbd_led_queue: Queue<kiibohd_usb::LedState, KBD_LED_QUEUE_SIZE> = Queue::new(),
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
        let tc0 = TimerCounter::new(
            cx.device.TC0,
            clocks.peripheral_clocks.tc_0.into_enabled_clock(),
        );
        let tc0_chs: TimerCounterChannels<TC0, TCC0_FREQ, TCC1_FREQ, TCC2_FREQ> = tc0.split();

        // Keyscanning Timer
        let mut tcc0 = tc0_chs.ch0;
        tcc0.clock_input(TCC0_DIV);
        tcc0.start((SCAN_PERIOD_US * 1000).nanos());
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
        rtt.start(500_u32.millis());
        rtt.enable_alarm_interrupt();
        defmt::trace!("RTT Timer started");

        // Initialize tickless monotonic timer
        let mono = DwtSystick::new(&mut cx.core.DCB, cx.core.DWT, cx.core.SYST, MCU_FREQ);
        defmt::trace!("DwtSystick (Monotonic) started");

        (
            Shared {
                hidio_intf,
                layer_state,
                matrix,
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
    /// - LED frame scheduling (Uses tcc1)
    ///   Schedules a lower priority task which is skipped if the previous frame is still
    ///   processing
    #[task(priority = 13, binds = TC0, local = [
        tcc0,
        tcc1,
    ], shared = [
        hidio_intf,
        layer_state,
        matrix,
    ])]
    fn tc0(mut cx: tc0::Context) {
        // Check for keyscanning interrupt (tcc0)
        (cx.shared.layer_state, cx.shared.matrix).lock(|layer_state, matrix| {
            if cx.local.tcc0.clear_interrupt_flags() {
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
                let process_macros = matrix.next_strobe::<Infallible>().unwrap() == 0;

                // If a full matrix scanning cycle has finished, process macros
                if process_macros && macro_process::spawn().is_err() {
                    defmt::warn!("Could not schedule macro_process");
                }
            }
        });

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
        cx.local.debug_led.toggle().ok();
    }

    /// LED Frame Processing Task
    /// Handles each LED frame, triggered at a constant rate.
    /// Frames are skipped if the previous frame is still processing.
    #[task(priority = 8, shared = [
        hidio_intf,
    ])]
    fn led_frame_process(mut cx: led_frame_process::Context) {
        cx.shared.hidio_intf.lock(|_hidio_intf| {
            // TODO
        });

        // Handle processing of "next" frame
        // If this takes too long, the next frame update won't be scheduled (i.e. it'll be
        // skipped).
        // TODO - KLL Pixelmap
        // TODO - HIDIO frame updates
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
                matrix
                    .generate_event(index)
                    .unwrap()
                    .trigger_event(index, false)
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

    /// ISSI I2C0 Interrupt
    #[task(priority = 12, binds = TWI0)]
    fn twi0(_: twi0::Context) {
        //unsafe { TWI0_Handler() };
    }

    /// ISSI I2C1 Interrupt
    #[task(priority = 12, binds = TWI1)]
    fn twi1(_: twi1::Context) {}

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
