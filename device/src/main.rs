//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
mod messaging;

extern crate alloc;

use alloc::vec::{self, Vec};
use bsp::hal::adc;
use bsp::hal::dma::{DMAExt, SingleChannel, CH0, CH1};
use cortex_m::delay::Delay;
use cortex_m::singleton;
use embedded_alloc::Heap;
use smart_leds_trait::RGB8;
use ws2812_pio::{dma_buf, leds, LEDs, Ws2812Direct};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use crate::hsv::HSV64;
use crate::messaging::packets::*;
use messaging::wrapper::{self, MAX_MESSAGE_SIZE};

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
    pac,
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

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024 * 32;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    info!("allocator initialized");
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

    // split the PIO state machines into individual objects, so that Ws2812 can use them.
    // each state machine will handle two channels of LEDs

    // board is driving 16 LED chains, i guess?
    // we have 8 state machines, so two chains per machine
    const LEDS_PER_CHANNEL: usize = 9;
    const BYTES_PER_CHANNEL: usize = ((LEDS_PER_CHANNEL * 24) + 31) / 32;
    let mut leds0 = leds!(HSV64, 9);
    let mut leds1 = leds!(HSV64, 9);
    let mut leds2 = leds!(HSV64, 9);
    let mut leds3 = leds!(HSV64, 9);
    let mut leds4 = leds!(HSV64, 9);
    let mut leds5 = leds!(HSV64, 9);
    let mut leds6 = leds!(HSV64, 9);
    let mut leds7 = leds!(HSV64, 9);

    let leds: [LEDs<9, HSV64>; 8] = [leds0, leds1, leds2, leds3, leds4, leds5, leds6, leds7];

    // dma.ch0
    //     .ch()
    //     .ch_al1_ctrl
    //     .write(|w| unsafe { w.ring_size().bits(4) });

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut lights = Ws2812Direct::new(
        pac.PIO0,
        pac.PIO1,
        pac.DMA.split(&mut pac.RESETS),
        &mut pac.RESETS,
        (
            pins.gpio0.into_mode(),
            pins.gpio1.into_mode(),
            pins.gpio2.into_mode(),
            pins.gpio3.into_mode(),
            pins.gpio4.into_mode(),
            pins.gpio5.into_mode(),
            pins.gpio6.into_mode(),
            pins.gpio7.into_mode(),
            pins.gpio8.into_mode(),
            pins.gpio9.into_mode(),
            pins.gpio10.into_mode(),
            pins.gpio11.into_mode(),
            pins.gpio12.into_mode(),
            pins.gpio13.into_mode(),
            pins.gpio14.into_mode(),
            pins.gpio15.into_mode(),
        ),
        (
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
            dma_buf!(),
        ),
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let (mut spec_reset, mut spec_strobe, mut spec_out) = (
        pins.gpio16.into_push_pull_output(),
        pins.gpio17.into_push_pull_output(),
        pins.gpio28.into_floating_input(),
    );
    let mut adc = adc::Adc::new(pac.ADC, &mut pac.RESETS);

    let mut spectrum = audio::Spectrum::default();

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

    let mut step = 10110;
    // let mut step = 19080;
    // let mut step = 17200;
    // let mut step = 5900;
    // let mut step = 8000;
    // let mut step = 12345;
    // let mut step = 27500;
    let mut controller = new_controller(leds, step);
    unsafe {
        CONTROLLER = Some(controller);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };
    let FREQ = clocks.system_clock.freq();
    let core = pac::CorePeripherals::take().unwrap();
    // todo foo lol i'm tired
    let mut delay = Foo(cortex_m::delay::Delay::new(core.SYST, FREQ.to_Hz()));

    let mut led_pin = pins.led.into_push_pull_output();

    info!("up");

    let mut poll_count_down = timer.count_down();
    let mut led_count_down = timer.count_down();
    let mut led_on = true;
    // Create a count_down timer for 500 milliseconds
    led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
    poll_count_down.start(USB_POLL_RATE_MILLIS.millis());
    let mut times_ns: u64 = 0;
    let mut times_count = 0;
    loop {
        let start = timer.get_counter();
        if poll_count_down.wait().is_ok() {
            poll_usb();
            poll_count_down.start(USB_POLL_RATE_MILLIS.millis());
        };

        if led_count_down.wait().is_ok() {
            // info!("buf0 {:X}", buf0);
            // info!("buf1 {:X}", buf1);
            if led_on {
                led_pin.set_low().unwrap();
                led_on = false;
            } else {
                led_pin.set_high().unwrap();
                led_on = true;
            }
            // for v in leds.channel0.iter() {
            //     info!("v {:X} {:X} {:X}", v.r, v.g, v.b)
            // }

            // step += 100;
            // info!("{:?}", step);
            // critical_section::with(|_| unsafe {
            //     CONTROLLER.as_mut().map(|c| {
            //         c.inc_step(100);
            //     })
            // });

            // info!("{:?}", spectrum.decay);
            // spectrum.attenuate();

            led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
        }

        // let r = (spectrum.decay[0..3]
        //     .iter()
        //     .fold(0, |a: u32, v| a + *v as u32)
        //     / (3 * u8::MAX as u32)) as u8;
        // let g = (spectrum.decay[3..5]
        //     .iter()
        //     .fold(0, |a: u32, v| a + *v as u32)
        //     / (2 * u8::MAX as u32)) as u8;
        // let b = (spectrum.decay[5..7]
        //     .iter()
        //     .fold(0, |a: u32, v| a + *v as u32)
        //     / (2 * u8::MAX as u32)) as u8;

        // for i in 0..leds0.channel0.len() {
        //     leds0.channel0[i].r = r;
        //     leds0.channel0[i].g = g;
        //     leds0.channel0[i].b = b;
        // }

        // for v in leds.channel0.iter_mut() {
        //     hsv::step_hue(v, step);
        // }
        // controller.step_hue();

        let leds = critical_section::with(|_| unsafe {
            CONTROLLER
                .as_mut()
                .map(|c| {
                    c.step_hue();
                    c.bufs()
                })
                .unwrap()
        });
        lights = lights.write(leds);

        // spectrum.update(
        //     &mut adc,
        //     &mut delay.0,
        //     &mut spec_reset,
        //     &mut spec_strobe,
        //     &mut spec_out,
        // );

        // light = light.pump();

        // match timer.get_counter().checked_duration_since(start) {
        //     Some(d) => {
        //         times_count += 1;
        //         times_ns += d.to_nanos();
        //         if times_count >= 100 {
        //             let ns = times_ns / 100;
        //             info!("loop time: {}ns", ns);
        //             times_count = 0;
        //             times_ns = 0;
        //         }
        //     }
        //     None => {
        //         error!("couldn't do since?")
        //     }
        // }
    }
}

mod audio {
    use cortex_m::delay::Delay;
    use defmt::info;
    use embedded_hal::adc::{Channel, OneShot};
    use embedded_hal::digital::v2::OutputPin;
    use rp_pico::hal::gpio::PushPull;
    use rp_pico::hal::{adc, gpio};

    static SPECTRUM_FACTORS: [u16; 7] = [6, 8, 8, 8, 7, 7, 10];
    const WINDOW_SIZE: usize = 100;
    const DECAY_WEIGHT: u32 = 10;
    const PEAKDECAY: u16 = 1;
    const NOISEFLOOR: u16 = 65;

    pub struct Spectrum {
        pub decay: [u16; 7],
        pub peaks: [u16; 7],
        mins: [u16; 7],
        maxs: [u16; 7],
    }

    impl Default for Spectrum {
        fn default() -> Self {
            Self {
                decay: [0u16; 7],
                peaks: [0u16; 7],
                mins: [u16::MAX; 7],
                maxs: [0u16; 7],
            }
        }
    }

    impl Spectrum {
        pub fn update<I: gpio::PinId, J: gpio::PinId, K: Channel<adc::Adc, ID = u8>>(
            &mut self,
            adc: &mut adc::Adc,
            delay: &mut Delay,
            reset: &mut gpio::Pin<I, gpio::Output<PushPull>>,
            strobe: &mut gpio::Pin<J, gpio::Output<PushPull>>,
            data: &mut K,
        ) {
            reset.set_high().unwrap();
            delay.delay_us(10);
            reset.set_low().unwrap();
            delay.delay_us(72);

            let mut raws = [0u16; 7];

            // cycle through each MSGEQ7 bin and read the analog values
            for i in 0..7 {
                // set up the MSGEQ7
                strobe.set_low().unwrap();
                delay.delay_us(25);

                // let temp1: u16 = adc.read(data).unwrap();
                // let temp2: u16 = adc.read(data).unwrap();
                // let temp3: u16 = adc.read(data).unwrap();

                // raws[i] = (temp1 + temp2 + temp3) / 3;
                raws[i] = adc.read(data).unwrap();

                strobe.set_high().unwrap();
                delay.delay_us(25);
            }

            for i in 0..7 {
                let mut raw = raws[i];
                if raw < NOISEFLOOR {
                    raw = 0;
                } else {
                    raw -= NOISEFLOOR;
                }

                // immediately update mins/maxes if we broke the range
                self.mins[i] = self.mins[i].min(raw);
                self.maxs[i] = self.maxs[i].max(raw);

                //uint8_t I = A + (uint8_t)(((int16_t)(B-A) * F) >> 8)
                // let lerped = self.mins[i]
                //     + (((self.maxs[i] - self.mins[i]) as u32 * raw as u32) ) as u16;
                let mut lerped = raw;
                if self.mins[i] != self.maxs[i] {
                    lerped = (u16::MAX / (self.maxs[i] - self.mins[i])) * (raw - self.mins[i]);
                }
                // if i == 5 {
                //     info!("min {:?} max {:?}", self.mins[i], self.maxs[i]);
                //     info!("raw {:?} lerped {:?}", raw, lerped);
                // }

                // apply correction factor per frequency bin
                // let corrected = lerped * SPECTRUM_FACTORS[i];

                // process time-averaged values
                self.decay[i] = (((self.decay[i] as u32 * (DECAY_WEIGHT - 1)) + lerped as u32)
                    / DECAY_WEIGHT) as u16;

                // process peak values
                if self.peaks[i] < self.decay[i] {
                    self.peaks[i] = self.decay[i]
                };
                if self.peaks[i] > PEAKDECAY {
                    self.peaks[i] -= PEAKDECAY;
                } else {
                    self.peaks[i] = 0;
                }
            }

            // if self.i_history0 == 0 {
            //     info!("raws {:?}", raws);
            // }
        }

        pub fn attenuate(&mut self) {
            for i in 0..7 {
                if self.mins[i] < u16::MAX - PEAKDECAY {
                    self.mins[i] += PEAKDECAY;
                }
                if self.maxs[i] > PEAKDECAY {
                    self.maxs[i] -= PEAKDECAY;
                }
            }
        }
    }
}

struct Foo(Delay);

impl ws2812_pio::DelayUs for Foo {
    fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us)
    }
}

mod hsv {
    use defmt::info;
    use smart_leds_trait::RGB8;

    #[derive(Clone, Copy)]
    pub struct HSV64 {
        inc: bool,
        pub h: u32,
        pub s: u16,
        pub v: u8,
    }

    impl HSV64 {
        pub fn step_hue(&mut self, step: u32) {
            if self.inc {
                self.h += step;
                if self.h > MAX_HUE {
                    self.h = MAX_HUE;
                    self.inc = false
                }
                // if self.h > HUE_EDGE_LEN / 2 {
                //     self.h = HUE_EDGE_LEN / 2;
                //     self.inc = false
                // }
            } else {
                if self.h < step {
                    self.h = 0;
                    self.inc = true;
                } else {
                    self.h -= step
                }
            }
            // if self.h > MAX_HUE {
            //     self.h = 0
            // }
        }

        pub const fn new() -> Self {
            Self {
                inc: true,
                h: 0,
                s: u16::MAX,
                v: 128,
            }
        }
    }

    impl Default for HSV64 {
        fn default() -> Self {
            Self {
                inc: true,
                h: 0,
                s: u16::MAX,
                v: 128,
            }
        }
    }

    impl Into<RGB8> for HSV64 {
        fn into(self) -> RGB8 {
            let (r, g, b) = hsv2rgb(self.h, self.s, self.v);
            RGB8::new(r, g, b)
        }
    }

    // neato: https://www.sciencedirect.com/science/article/abs/pii/S0045790615002827
    // found implemented here: https://bitbucket.org/chernov/colormath_hsv/src/master/colormath/hsv.cpp
    const MAX_VALUE: u8 = 0xFF;

    const MAX_SATURATION: u16 = 0xFFFF;

    pub const HUE_EDGE_LEN: u32 = 65537;
    pub const MAX_HUE: u32 = HUE_EDGE_LEN * 6;

    pub fn rgb2hsv(r: u8, g: u8, b: u8) -> (u32, u16, u8) {
        let (max, mid, min): (u8, u8, u8);
        let edge: u32;
        let inverse: bool;

        if r > g {
            if g >= b {
                max = r;
                mid = g;
                min = b;
                edge = 0;
                inverse = false;
            } else if r > b {
                max = r;
                mid = b;
                min = g;
                edge = 5;
                inverse = true;
            } else {
                max = b;
                mid = r;
                min = g;
                edge = 4;
                inverse = false;
            }
        } else if r > b {
            max = g;
            mid = r;
            min = b;
            edge = 1;
            inverse = true;
        } else {
            if g > b {
                max = g;
                mid = b;
                min = r;
                edge = 2;
                inverse = false;
            } else {
                max = b;
                mid = g;
                min = r;
                edge = 3;
                inverse = true;
            }
        }

        let v = max;

        let delta: u32 = (max - min) as u32;
        if delta == 0 {
            return (0, 0, v);
        }

        let s = (((delta << 16) - 1) / (v as u32)) as u16;

        let mut h = ((((mid - min) as u32) << 16) / (delta)) + 1;
        if inverse {
            h = HUE_EDGE_LEN - h;
        }
        h += edge * HUE_EDGE_LEN;

        (h, s, v)
    }

    pub fn hsv2rgb(h: u32, s: u16, v: u8) -> (u8, u8, u8) {
        if s == 0 || v == 0 {
            return (v, v, v);
        }

        let delta: u8 = (((s as u32 * v as u32) >> 16) + 1) as u8;

        // i guess it's fine if this is just the low bits of delta...?
        let min: u8 = v - (delta as u8);

        let i = (h / HUE_EDGE_LEN).min(5);
        let mut f = h - HUE_EDGE_LEN * i;
        if i % 2 == 1 {
            f = HUE_EDGE_LEN - f;
        }
        let c = (((f * delta as u32) >> 16) as u8) + min;

        let (c, m) = (c as u8, min as u8);

        let foos = [
            (v, c, m),
            (c, v, m),
            (m, v, c),
            (m, c, v),
            (c, m, v),
            (v, m, c),
        ];
        foos[i as usize]

        // if h >= HUE_EDGE_LEN * 4 {
        //     h -= HUE_EDGE_LEN * 4;
        //     if h < HUE_EDGE_LEN {
        //         b = v;
        //         g = min;
        //         mid = &mut r;
        //     } else {
        //         h -= HUE_EDGE_LEN;
        //         h = HUE_EDGE_LEN - h;
        //         r = v;
        //         g = min;
        //         mid = &mut b;
        //     }
        // } else if h >= HUE_EDGE_LEN * 2 {
        //     h -= HUE_EDGE_LEN * 2;
        //     if h < HUE_EDGE_LEN {
        //         g = v;
        //         r = min;
        //         mid = &mut b;
        //     } else {
        //         h -= HUE_EDGE_LEN;
        //         h = HUE_EDGE_LEN - h;
        //         b = v;
        //         r = min;
        //         mid = &mut g;
        //     }
        // } else {
        //     if h < HUE_EDGE_LEN {
        //         r = v;
        //         b = min;
        //         mid = &mut g;
        //     } else {
        //         h -= HUE_EDGE_LEN;
        //         h = HUE_EDGE_LEN - h;
        //         g = v;
        //         b = min;
        //         mid = &mut r;
        //     }
        // }

        // *mid = (((h * delta) >> 16) as u8) + min;

        // (r, g, b)
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

static mut CONTROLLER: Option<DeviceController<9>> = None;

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
            Ok(data) => {
                let controller = CONTROLLER.as_mut().unwrap();
                let mut resp: Vec<u8> = Vec::new();
                match controller.handle(&data.into(), &mut resp) {
                    Ok(_) => {
                        wrapper::write_all(usb_hid, &mut resp).map_err(handle_usberror);
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
