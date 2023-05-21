//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
mod messaging;

#[macro_use]
extern crate alloc;

use alloc::vec::{self, Vec};
use bsp::hal::adc;
use bsp::hal::dma::{DMAExt, SingleChannel, CH0, CH1};
use bsp::hal::multicore::{Multicore, Stack};
use bsp::hal::sio::Spinlock0;
use cobs::decode_in_place;
use core::cell::{RefCell, RefMut};
use core::ops::{Deref, DerefMut};
use core::sync::atomic::AtomicPtr;
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use cortex_m::singleton;
use embedded_alloc::Heap;
use smart_leds_trait::RGB8;
use ws2812_pio::{dma_buf, leds, LEDs, Ws2812Direct};

#[global_allocator]
static HEAP: Heap = Heap::empty();

use crate::hsv::HSV64;
use crate::messaging::packets::*;
use messaging::wrapper::{self};

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

static mut CONTROLLER: Option<DeviceController<9>> = None;

const USB_POLL_RATE_MILLIS: u32 = 100;
const LED_BLINK_RATE_MILLIS: u32 = 500;

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let external_xtal_freq_hz = XOSC_CRYSTAL_FREQ;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
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

    // board is driving 16 LED chains, i guess?
    // we have 8 state machines, so two chains per machine
    const LEDS_PER_CHANNEL: usize = 9;
    let leds0 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds1 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds2 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds3 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds4 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds5 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds6 = leds!(HSV64, LEDS_PER_CHANNEL);
    let leds7 = leds!(HSV64, LEDS_PER_CHANNEL);

    let leds: [LEDs<9, HSV64>; 8] = [leds0, leds1, leds2, leds3, leds4, leds5, leds6, leds7];

    let bufs = (
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
        dma_buf!(),
    );

    leds[0].fill(bufs.0);
    leds[1].fill(bufs.1);
    leds[2].fill(bufs.2);
    leds[3].fill(bufs.3);
    leds[4].fill(bufs.4);
    leds[5].fill(bufs.5);
    leds[6].fill(bufs.6);
    leds[7].fill(bufs.7);

    let mut step = hsv::HUE_EDGE_LEN;
    // let mut step = 42000;
    cortex_m::interrupt::free(|cs| {
        let _lock = Spinlock0::claim();
        unsafe {
            CONTROLLER = Some(new_controller(leds, step));
        }
    });

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
        bufs,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    let mut led_count_down = timer.count_down();
    // Create a count_down timer for 500 milliseconds
    led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
    loop {
        // if led_count_down.wait().is_ok() {
        //     let inc = 100;
        //     step += inc;
        //     info!("{:?}", step);
        //     cortex_m::interrupt::free(|cs| {
        //         let _lock = Spinlock0::claim();
        //         let controller = unsafe { CONTROLLER.as_mut().unwrap() };
        //         controller.inc_step(inc);
        //     });
        //     led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
        // }

        // let input = sio.fifo.read();
        // if let Some(word) = input {
        //     delay.delay_ms(word);
        //     led_pin.toggle().unwrap();
        //     sio.fifo.write_blocking(CORE1_TASK_COMPLETE);
        // };
        // delay.delay_ms(5);

        cortex_m::interrupt::free(|cs| {
            let _lock = Spinlock0::claim();
            let controller = unsafe { CONTROLLER.as_mut().unwrap() };
            controller.step_hue();
            lights.write(controller.bufs());
        });

        // leds.iter_mut().for_each(|l| {
        //     l.channel0.iter_mut().for_each(|l| l.step_hue(step));
        //     l.channel1.iter_mut().for_each(|l| l.step_hue(step));
        // });
    }
}

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
    let mut sio = Sio::new(pac.SIO);

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
    let sys_freq = clocks.system_clock.freq().to_Hz();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // split the PIO state machines into individual objects, so that Ws2812 can use them.
    // each state machine will handle two channels of LEDs

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
    let usb_hid = HIDClass::new(bus_ref, CustomBidirectionalReport::desc(), 5);
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
    let FREQ = clocks.system_clock.freq();
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = Foo(cortex_m::delay::Delay::new(core.SYST, FREQ.to_Hz()));

    let mut led_pin = pins.led.into_push_pull_output();

    info!("up");

    let mut led_count_down = timer.count_down();
    let mut led_on = true;
    // Create a count_down timer for 500 milliseconds
    led_count_down.start(LED_BLINK_RATE_MILLIS.millis());
    let mut times_ns: u64 = 0;
    let mut times_count = 0; // Start up the second core to blink the second LED

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            core1_task(sys_freq)
        })
        .unwrap();

    loop {
        let start = timer.get_counter();

        if led_count_down.wait().is_ok() {
            if led_on {
                led_pin.set_low().unwrap();
                led_on = false;
            } else {
                led_pin.set_high().unwrap();
                led_on = true;
            }

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
        pub fn from_rgb(r: u8, g: u8, b: u8) -> Self {
            let (h, s, v) = rgb2hsv(r, g, b);
            Self {
                inc: false,
                h,
                s,
                v,
            }
        }
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

static mut LAST_STATE: UsbDeviceState = UsbDeviceState::Default;
static mut MESSAGE_BUFFER: Vec<u8> = Vec::new();
static mut MESSAGE_PENDING: bool = false;

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();

    let usb_hid = USB_HID.as_mut().unwrap();
    if usb_dev.poll(&mut [usb_hid]) {
        let mut buf = [0u8; 64];
        match usb_hid.pull_raw_output(&mut buf) {
            Ok(i) => {
                info!("raw output size {}", i);
                info!("{:X}", buf);
                if i == 0 {
                    MESSAGE_PENDING = false;
                    debug!("done because 0 len message");
                } else if buf.iter().any(|b| *b == 0) {
                    MESSAGE_PENDING = false;
                    debug!("done because found 0");
                } else {
                    MESSAGE_PENDING = true;
                }

                if i > 0 {
                    MESSAGE_BUFFER.extend(buf[0..i].iter());
                }
            }
            Err(e) => {
                handle_usberror(e);
                MESSAGE_PENDING = false;
            }
        }
    }

    if !MESSAGE_PENDING && !MESSAGE_BUFFER.is_empty() {
        info!("pre decode {:X}", MESSAGE_BUFFER.as_slice());
        if let Ok(i) =
            decode_in_place(MESSAGE_BUFFER.as_mut_slice()).map_err(|_| UsbError::ParseError)
        {
            MESSAGE_BUFFER.truncate(i);

            info!("post decode {} {:X}", i, MESSAGE_BUFFER.as_slice());

            let usb_hid = USB_HID.as_mut().unwrap();
            cortex_m::interrupt::free(|cs| {
                let mut resp = {
                    let _lock = Spinlock0::claim();
                    let controller = CONTROLLER.as_mut().unwrap();
                    controller.handle(&mut MESSAGE_BUFFER).map_or_else(
                        |e| {
                            info!("{:?}", e);
                            let mut errvec = Vec::new();
                            errvec.push(0);
                            errvec
                        },
                        |v| v,
                    )
                };
                match wrapper::write_all(usb_hid, &mut resp) {
                    Err(e) => handle_usberror(e),
                    _ => {}
                };
            });
        } else {
            debug!("unable to cobs decode message");
        }

        MESSAGE_PENDING = false;
        MESSAGE_BUFFER.clear();
    }
    // if usb_dev.poll(&mut [usb_hid]) {
    //     let r = cortex_m::interrupt::free(|cs| wrapper::read_all(usb_hid));
    //     match r {
    //         Ok(data) => cortex_m::interrupt::free(|cs| {
    //             let mut resp = {
    //                 let _lock = Spinlock0::claim();
    //                 let controller = CONTROLLER.as_mut().unwrap();
    //                 controller.handle(&mut data.into()).map_or_else(
    //                     |e| {
    //                         info!("{:?}", e);
    //                         let mut errvec = Vec::new();
    //                         errvec.push(0);
    //                         errvec
    //                     },
    //                     |v| v,
    //                 )
    //             };
    //             match wrapper::write_all(usb_hid, &mut resp) {
    //                 Err(e) => handle_usberror(e),
    //                 _ => {}
    //             };
    //         }),
    //         Err(e) => handle_usberror(e),
    //     }
    // }

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
