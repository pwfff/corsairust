use super::packet_ids::{Handler, PacketIdHandler};
use alloc::{boxed::Box, collections::BTreeMap};
use alloc::{format, string::*, vec::Vec};
use defmt::{debug, info};
use genio::Read;
use openrgb_data::{
    Controller, DeviceType, Header, Mode, ModeFlag, OpenRGBReadableSync, OpenRGBWritable,
    OpenRGBWritableSync, PacketId, WriteVec, Zone, ZoneType, LED,
};
use smart_leds_trait::RGB8;
use ws2812_pio::LEDs;

use crate::hsv::{self, HSV64, rgb2hsv};

use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct DeviceController<const SIZE: usize> {
    data: ControllerData<SIZE>,
    profiles: BTreeMap<String, Controller>,
}

pub struct ControllerData<const SIZE: usize> {
    leds: [LEDs<SIZE, hsv::HSV64>; 8],
    active_mode: usize,
    modes: Vec<Mode>,
    zones: Vec<Zone>,
}

pub fn new_controller<const SIZE: usize>(
    leds: [LEDs<SIZE, hsv::HSV64>; 8],
    hue_step: u32,
) -> DeviceController<SIZE> {
    let mut modes = Vec::new();
    modes.push(Mode {
        name: format!("rainbow"),
        value: 0,
        flags: ModeFlag::HasSpeed.into(),
        speed_min: Some(0),
        speed_max: hsv::HUE_EDGE_LEN.into(),
        speed: Some(12345),
        brightness_min: None,
        brightness_max: None,
        brightness: None,
        color_mode: None,
        colors: Vec::new(),
        colors_min: None,
        colors_max: None,
        direction: None,
    });
    modes.push(Mode {
        name: format!("default"),
        value: 1,
        flags: ModeFlag::HasPerLEDColor.into(),
        speed_min: None,
        speed_max: None,
        speed: None,
        brightness_min: None,
        brightness_max: None,
        brightness: None,
        color_mode: None,
        colors: Vec::new(),
        colors_min: None,
        colors_max: None,
        direction: None,
    });

    let mut zones = Vec::new();
    for i in 0..leds.len() {
        zones.push(Zone {
            name: format!("Zone {i}"),
            r#type: ZoneType::Linear,
            leds_min: 9,
            leds_max: 9,
            leds_count: 9,
            matrix: None,
        });
    }

    // TODO: profile persistence...
    let profiles = BTreeMap::new();

    DeviceController {
        data: ControllerData {
            leds,
            active_mode: 0,
            modes,
            zones,
        },
        profiles,
    }
}

impl<'a, const SIZE: usize> DeviceController<SIZE> {
    pub fn handle(&mut self, data_in: &mut Vec<u8>) -> Result<Vec<u8>> {
        let mut data_out = Vec::new();
        let mut w = WriteVec::new(&mut data_out);
        let h = data_in.as_slice().read_any(DEFAULT_PROTOCOL)?;
        let rest = data_in.split_off(h.size(DEFAULT_PROTOCOL));
        PacketIdHandler::from(h.packet_id).handle(self, h, &rest, &mut w)?;

        Ok(data_out)
    }

    pub fn step_hue(&mut self) {
        if self.data.active_mode == 0 {
            for leds in self.data.leds.iter_mut() {
                for i in 0..SIZE {
                    leds.channel0[i].step_hue(self.data.modes[0].speed.unwrap_or(0));
                    leds.channel1[i].step_hue(self.data.modes[0].speed.unwrap_or(0));
                }
            }
        }
    }

    pub fn set_step(&mut self, inc: u32) {
        self.data.modes[0].speed = Some(inc);
    }

    pub fn inc_step(&mut self, inc: u32) {
        self.data.modes[0].speed = Some(self.data.modes[0].speed.unwrap_or(0) + inc);
    }

    pub fn bufs(&self) -> &[LEDs<SIZE, hsv::HSV64>; 8] {
        &self.data.leds
    }

    fn controller_count(&self) -> u32 {
        1
    }

    fn set_mode(&mut self, i: usize, mode: Mode) {
        self.data.modes[i] = mode;
    }
}

impl<const SIZE: usize> ControllerData<SIZE> {
    fn openrgb_leds(&self) -> Vec<LED> {
        let mut leds = Vec::new();
        for (i, device) in self.leds.iter().enumerate() {
            for (j, led) in device.channel0.iter().enumerate() {
                let rgb: RGB8 = (*led).into();
                leds.push(LED {
                    name: String::new(),
                    value: (rgb.r as u32) << 16 | (rgb.g as u32) << 8 | (rgb.b as u32),
                });
            }
            for (j, led) in device.channel1.iter().enumerate() {
                let rgb: RGB8 = (*led).into();
                leds.push(LED {
                    name: String::new(),
                    value: (rgb.r as u32) << 16 | (rgb.g as u32) << 8 | (rgb.b as u32),
                });
            }
        }

        leds
    }

    fn as_controller(&self) -> Controller {
        Controller {
            r#type: DeviceType::LEDStrip,
            name: format!("corairust"),
            vendor: format!("me"),
            description: format!("its an rp2040, baby"),
            version: format!("1"),
            serial: format!("123"),
            location: format!("a lil computer"),
            active_mode: 0,
            modes: self.modes.to_vec(),
            zones: self.zones.to_vec(),
            leds: self.openrgb_leds(),
            colors: Vec::new(),
        }
    }

    fn from_controller(&mut self, c: &Controller) {
        self.modes = c.modes.clone();
        self.zones = c.zones.clone();

        let mut leds = [
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
            LEDs::<SIZE, HSV64>::default(),
        ];

        c.leds.iter().enumerate().for_each(|(i, l)| {
            let b = l.value as u8;
            let g = (l.value >> 8) as u8;
            let r = (l.value >> 16) as u8;

            let mut led = &mut leds[i/16];
            let channel_i = i % 16;
            if channel_i < 8 {
                led.channel0[channel_i] = HSV64::from_rgb(r, g, b);
            } else {
                led.channel1[channel_i-8] = HSV64::from_rgb(r, g, b);
            }
        });

        self.leds = leds;
    }
}

pub trait ResponseType {}

#[derive(Default)]
pub struct RequestProtocolVersion {}

impl Handler for RequestProtocolVersion {
    fn handle<'a, const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        debug!("{}", data_out.len());
        debug!("{}", data_out.capacity());
        data_out
            .write_packet(
                DEFAULT_PROTOCOL,
                // TODO: device ids i guess?
                0,
                PacketId::RequestProtocolVersion,
                DEFAULT_PROTOCOL,
            )
            .map_err(|e| e.into())
    }
}

#[derive(Default)]
pub struct RequestControllerCount {}

impl Handler for RequestControllerCount {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        data_out
            .write_packet(
                DEFAULT_PROTOCOL,
                // TODO: device ids i guess?
                0,
                PacketId::RequestControllerCount,
                controller.controller_count(),
            )
            .map_err(|e| e.into())
    }
}

#[derive(Default)]
pub struct RequestControllerData {}

impl Handler for RequestControllerData {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        data_out
            .write_packet(
                DEFAULT_PROTOCOL,
                // TODO: device ids i guess?
                header.device_id,
                PacketId::RequestControllerData,
                controller.data.as_controller(),
            )
            .map_err(|e| e.into())
    }
}

#[derive(Default)]
pub struct SetClientName {}

impl Handler for SetClientName {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("{:X}", data_in.as_slice());

        // let mut stream = data_in.as_slice();
        // let len = stream.read_value::<u16>(DEFAULT_PROTOCOL)?;
        // info!("{:X}", stream);
        // let mut buf = alloc::vec![0; len as usize];
        // stream
        //     .read_exact(&mut buf)
        //     .map_err(|e| Error::Oops(format!("readerr")))?;
        // info!("{:X}", buf.as_slice());
        // buf.pop();
        // info!("{:X}", buf.as_slice());

        // let client = String::from_utf8(buf).map_err(|e| Error::Oops(format!("{}", e)))?;
        let client: String = data_in.as_slice().read_value(DEFAULT_PROTOCOL)?;
        info!("client connected: {:X}", client.as_str());
        Ok(())
    }
}

#[derive(Default)]
pub struct RGBControllerUpdateMode {}

impl Handler for RGBControllerUpdateMode {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("got set mode {:X}", data_in.as_slice());

        let mut stream = data_in.as_slice();
        // length from header...? i guess?
        let _ = stream.read_value::<u16>(DEFAULT_PROTOCOL)?;
        // same length, but in the packet...
        let _ = stream.read_value::<u32>(DEFAULT_PROTOCOL)?;
        // mode index
        let i = stream.read_value::<i32>(DEFAULT_PROTOCOL)?;
        let mode = stream.read_value::<Mode>(DEFAULT_PROTOCOL)?;
        controller.set_mode(i as usize, mode);
        Ok(())
    }
}

#[derive(Default)]
pub struct RequestProfileList {}

impl Handler for RequestProfileList {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("got request profile list");

        data_out.write_packet(
            DEFAULT_PROTOCOL,
            0,
            PacketId::RequestProfileList,
            (
                controller.profiles.len(),
                controller
                    .profiles
                    .keys()
                    .into_iter()
                    .collect::<Vec<&String>>(),
            ),
        )?;

        Ok(())
    }
}

#[derive(Default)]
pub struct RequestSaveProfile {}

impl Handler for RequestSaveProfile {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("got save profile {:X}", data_in.as_slice());

        let mut stream = data_in.as_slice();

        let name = stream.read_value::<String>(DEFAULT_PROTOCOL)?;

        controller.profiles.insert(name, controller.data.as_controller());

        Ok(())
    }
}

#[derive(Default)]
pub struct RequestLoadProfile {}

impl Handler for RequestLoadProfile {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("got save profile {:X}", data_in.as_slice());

        let mut stream = data_in.as_slice();

        let name = stream.read_value::<String>(DEFAULT_PROTOCOL)?;

        let entry = controller
            .profiles
            .get(&name)
            .ok_or(Error::Oops(format!("profile not found")))?;

        controller.data.from_controller(entry);

        Ok(())
    }
}


#[derive(Default)]
pub struct RequestDeleteProfile {}

impl Handler for RequestDeleteProfile {
    fn handle<const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        info!("got save profile {:X}", data_in.as_slice());

        let mut stream = data_in.as_slice();

        let name = stream.read_value::<String>(DEFAULT_PROTOCOL)?;

        controller.profiles.remove(&name);

        Ok(())
    }
}
