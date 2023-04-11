//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
mod messaging;

extern crate alloc;

use alloc::vec::{self, Vec};
use bsp::hal::adc;
use bsp::hal::dma::{DMAExt, CH0, CH1};
use cortex_m::delay::Delay;
use cortex_m::singleton;
use embedded_alloc::Heap;
use smart_leds_trait::RGB8;
use ws2812_pio::{buf, LEDs, Ws2812Direct};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use crate::messaging::packets::*;
use messaging::wrapper::{self, MAX_BUFFER_SIZE, MAX_MESSAGE_SIZE};

use hid::*;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

// PIOExt for the split() method that is needed to bring
// PIO0 into useable form for Ws2812:
use bsp::hal::pio::{InstalledProgram, PIOExt, UninitStateMachine, SM0};

use bsp::{entry, XOSC_CRYSTAL_FREQ};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;

use panic_probe as _;

use bsp::hal::{
    self,
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
    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio1, sm4, sm5, sm6, sm7) = pac.PIO1.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    let program0 = ws2812_pio::init(&mut pio0);
    let program0_0: InstalledProgram<pac::PIO0>;
    let program0_1: InstalledProgram<pac::PIO0>;
    let program0_2: InstalledProgram<pac::PIO0>;
    unsafe {
        program0_0 = program0.share();
        program0_1 = program0.share();
        program0_2 = program0.share();
    }

    // board is driving 16 LED chains, i guess?
    // we have 8 state machines, so two chains per machine
    const LEDS_PER_DEVICE: usize = 9;
    let mut leds0: LEDs<LEDS_PER_DEVICE, RGB8> = LEDs {
        channel0: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
        channel1: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
    };
    let buf0_0 = buf!(LEDS_PER_DEVICE);
    let buf0_1 = buf!(LEDS_PER_DEVICE);

    let mut leds1: LEDs<LEDS_PER_DEVICE, RGB8> = LEDs {
        channel0: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
        channel1: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
    };
    let buf1_0 = buf!(LEDS_PER_DEVICE);
    let buf1_1 = buf!(LEDS_PER_DEVICE);

    let mut leds2: LEDs<LEDS_PER_DEVICE, RGB8> = LEDs {
        channel0: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
        channel1: [RGB8::new(0xFF, 0x00, 0x00); LEDS_PER_DEVICE],
    };
    let buf2_0 = buf!(LEDS_PER_DEVICE);
    let buf2_1 = buf!(LEDS_PER_DEVICE);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut light0 = Ws2812Direct::new(
        program0_0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        (dma.ch0, dma.ch1),
        buf0_0,
        buf0_1,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut light1 = Ws2812Direct::new(
        program0_1,
        pins.gpio2.into_mode(),
        pins.gpio3.into_mode(),
        (dma.ch2, dma.ch3),
        buf1_0,
        buf1_1,
        sm1,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut light2 = Ws2812Direct::new(
        program0_2,
        pins.gpio4.into_mode(),
        pins.gpio5.into_mode(),
        (dma.ch4, dma.ch5),
        buf2_0,
        buf2_1,
        sm2,
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

    let controller = Controller {};
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

    info!("led_us: {}", light0.led_us.to_micros());

    let mut poll_count_down = timer.count_down();
    let mut led_count_down = timer.count_down();
    let mut led_on = true;
    // let mut step = 0;
    let mut step = 5900;
    // let mut step = 8000;
    // let mut step = 12345;
    // let mut step = 27500;
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

            // // gotta fit these back into u32s, so split em up:
            // // high bits: 0xAABB -> shift, 0x00AA -> truncate, 0xAA
            // info!("{:X}", (smushed >> 32) as u32);
            // // low bits: 0xAABB -> truncate, 0xBB
            // info!("{:X}", smushed as u32);

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
        for v in leds0.channel0.iter_mut() {
            hsv::step_hue(v, step);
        }
        for v in leds0.channel1.iter_mut() {
            hsv::step_hue(v, step);
        }
        light0 = light0.write(&leds0);

        for v in leds1.channel0.iter_mut() {
            hsv::step_hue(v, step);
        }
        for v in leds1.channel1.iter_mut() {
            hsv::step_hue(v, step);
        }
        light1 = light1.write(&leds1);

        for v in leds2.channel0.iter_mut() {
            hsv::step_hue(v, step);
        }
        for v in leds2.channel1.iter_mut() {
            hsv::step_hue(v, step);
        }
        light2 = light2.write(&leds2);

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

    pub fn step_hue(c: &mut RGB8, step: u32) {
        let (h, s, v) = rgb2hsv(c.r, c.g, c.b);

        let mut new = h + step;
        if new > MAX_HUE {
            new = new - MAX_HUE;
        }

        (c.r, c.g, c.b) = hsv2rgb(new, s, 128);
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

        let delta: u32 = (((s as u32 * v as u32) >> 16) + 1) as u32;

        // i guess it's fine if this is just the low bits of delta...?
        let min: u8 = v - (delta as u8);

        let i = (h / HUE_EDGE_LEN).min(5);
        let f = h - HUE_EDGE_LEN * i;
        let c = (((f * delta) >> 16) as u8) + min;

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
            Ok(data) => {
                let controller = CONTROLLER.as_mut().unwrap();
                let mut resp: Vec<u8> = Vec::with_capacity(MAX_MESSAGE_SIZE);
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
