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
    use core::convert::Infallible;
    use core::fmt::Write;
    use crate::constants::*;
    use crate::hidio::*;
    use heapless::String;
    use heapless::spsc::{Consumer, Producer, Queue};
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

    type UsbDevice = usb_device::device::UsbDevice<'static, UdpBus>;

    // ----- Structs -----

    //
    // Shared resources used by tasks/interrupts
    //
    #[shared]
    struct Shared {
        ctrl_producer: Producer<'static, kiibohd_usb::CtrlState, CTRL_QUEUE_SIZE>,
        debug_led: Pb0<Output<PushPull>>,
        hidio_intf: HidioCommandInterface,
        kbd_led_consumer: Consumer<'static, kiibohd_usb::LedState, KBD_LED_QUEUE_SIZE>,
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
                kbd_led_consumer,
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
    #[task(priority = 10, shared = [
        ctrl_producer,
        hidio_intf,
        kbd_led_consumer,
        kbd_producer,
        layer_state,
        matrix,
        mouse_producer,
    ])]
    fn macro_process(mut cx: macro_process::Context) {
        cx.shared.layer_state.lock(|layer_state| {
            // Query HID LED Events
            cx.shared.kbd_led_consumer.lock(|kbd_led_consumer| {
                while let Some(state) = kbd_led_consumer.dequeue() {
                    // Convert to a TriggerEvent
                    let event = state.trigger_event();
                    let hidio_event = HidIoEvent::TriggerEvent(event);

                    // Enqueue KLL trigger event
                    let ret = layer_state.process_trigger::<MAX_LAYER_LOOKUP_SIZE>(event);
                    debug_assert!(ret.is_ok(), "Failed to enqueue: {:?} - {:?}", event, ret);

                    // Enqueue HID-IO trigger event
                    cx.shared.hidio_intf.lock(|hidio_intf| {
                        if let Err(err) = hidio_intf.process_event(hidio_event) {
                            defmt::error!("Hidio TriggerEvent Error: {:?}", err);
                        }
                    });
                }
            });

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
                    defmt::trace!("Device State: {:?}", usb_dev.state());
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
                        // Retrive HID Lock LED events
                        usb_hid.pull();

                        // Process HID-IO
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
