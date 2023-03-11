//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
mod messaging;

use crate::messaging::packets::*;
use messaging::wrapper::{self, MAX_BUFFER_SIZE, MAX_MESSAGE_SIZE};

use hid::*;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::{entry, XOSC_CRYSTAL_FREQ};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;

use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, prelude,
    sio::Sio,
    usb,
    watchdog::Watchdog,
    Timer,
};
use pac::interrupt;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<usb::UsbBus>> = None;

const USB_POLL_RATE_MILLIS: u32 = 100;
const LED_BLINK_RATE_MILLIS: u32 = 500;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = XOSC_CRYSTAL_FREQ;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver
    let usb_hid = HIDClass::new(bus_ref, CustomBidirectionalReport::desc(), 255);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a SRGBMods VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16d0, 0x1124))
        .manufacturer("Fake company")
        .product("RPRGB")
        .serial_number("TEST")
        .device_class(0)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };
    // let core = pac::CorePeripherals::take().unwrap();
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut led_pin = pins.led.into_push_pull_output();

    info!("up");

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut poll_count_down = timer.count_down();
    let mut led_count_down = timer.count_down();
    let mut led_on = true;
    // Create a count_down timer for 500 milliseconds
    led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
    poll_count_down.start(USB_POLL_RATE_MILLIS.millis());
    loop {
        if poll_count_down.wait().is_ok() {
            poll_usb();
            poll_count_down.start(USB_POLL_RATE_MILLIS.millis());
        };

        if led_count_down.wait().is_ok() {
            if led_on {
                led_pin.set_low().unwrap();
                led_on = false;
            } else {
                led_pin.set_high().unwrap();
                led_on = true;
            }
            led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
        }
    }
}

fn poll_usb() {
    let rep = CustomBidirectionalReport {
        input_buffer: [0u8; 32],
        output_buffer: [0u8; 32],
    };
    // let mut buf = [1u8; 33];
    // match serialize(&mut buf, &rep) {
    //     Ok(r) => {
    //         info!("{:?}", r);
    //         info!("{:X}", buf);
    //     }
    //     Err(_) => {
    //         error!("could not serialize")
    //     }
    // }

    // critical_section::with(|_| unsafe {
    //     USB_HID
    //         .as_mut()
    //         .map(|usb_hid| match usb_hid.push_input(&rep) {
    //             Ok(i) => {
    //                 // info!("sent {} bytes", i)
    //             }

    //             Err(e) => handle_usberror(e),
    //         });
    // })
}

static mut LAST_STATE: UsbDeviceState = UsbDeviceState::Default;

// buffer guaranteed to hold max message size
static mut message_buffer: [u8; MAX_MESSAGE_SIZE] = [0u8; MAX_MESSAGE_SIZE];
static mut wrapper_buffer: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
static mut CONTROLLER: Option<Controller> = None;

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    if usb_dev.poll(&mut [usb_hid]) {
        let r = wrapper::read_all(usb_hid);
        match r {
            Ok(mut data) => {
                let controller = CONTROLLER.as_mut().unwrap();
                match controller.handle(&mut data) {
                    Ok(response) => {
                        // wrapper::write_all(usb_hid, response);
                    }
                    Err(e) => error!("{}", e),
                };
            }
            Err(e) => handle_usberror(e),
        }
        // let rep = SetLEDsReport {
        //     zone_id: 0,
        //     led_count: 0,
        //     start_led: 0,
        //     led_rgb: [0u8; 32],
        // };
        // match usb_hid.pull_raw_report(&mut data) {
        //     Ok(info) => {
        //         info!("report ID {:X}", info.report_id);
        //         info!("report type {:?}", info.report_type as u8);
        //         info!("{:X}", data);
        //         serde_bytes::from_str();
        //     }

        //     Err(err) => handle_usberror(err),
        // };
        // info!("poll returned true")
    } else {
        // info!("poll returned false")
    }

    let newState = usb_dev.state();
    if newState != LAST_STATE {
        LAST_STATE = newState;
        match newState {
            UsbDeviceState::Default => info!("usb state: default"),
            UsbDeviceState::Addressed => info!("usb state: addressed"),
            UsbDeviceState::Configured => info!("usb state: configured"),
            UsbDeviceState::Suspend => info!("usb state: suspended"),
        }
    }
}

fn handle_usberror(err: usb_device::UsbError) {
    match err {
        usb_device::UsbError::WouldBlock => {}

        usb_device::UsbError::ParseError => {
            error!(" // Parsing failed due to invalid input.  ")
        }

        usb_device::UsbError::BufferOverflow => {
            error!(" // A buffer too short for the data to read was passed, or provided data cannot fit within
            // length constraints.  ")
        }

        usb_device::UsbError::EndpointOverflow => {
            error!(
                " // Classes attempted to allocate more endpoints than the peripheral supports.  "
            )
        }

        usb_device::UsbError::EndpointMemoryOverflow => {
            error!(" // Classes attempted to allocate more packet buffer memory than the peripheral supports. This
            // can be caused by either a single class trying to allocate a packet buffer larger than the
            // peripheral supports per endpoint, or multiple allocated endpoints together using more memory
            // than the peripheral has available for the buffers.  ")
        }

        usb_device::UsbError::InvalidEndpoint => {
            error!(" // The endpoint address is invalid or already used.  ")
        }

        usb_device::UsbError::Unsupported => {
            error!(" // Operation is not supported by device or configuration.  ")
        }

        usb_device::UsbError::InvalidState => {
            error!(" // Operation is not valid in the current state of the object.  ")
        }
    }
}

// End of file
