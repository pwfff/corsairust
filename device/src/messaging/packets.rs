extern crate alloc;

use alloc::{format, string::ToString, vec::Vec};
use corsairust_macros::all_fields_with;
use defmt::debug;
use openrgb_data::{
    Controller, DeviceType, Header, Mode, ModeFlag, OpenRGBReadableSync, OpenRGBWritableSync,
    PacketId, WriteVec, Zone, ZoneType, LED,
};
use serde::{Deserialize, Serialize};
use smart_leds_trait::RGB8;
use ws2812_pio::LEDs;

use crate::hsv;

use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct DeviceController<const SIZE: usize> {
    leds: [LEDs<SIZE, hsv::HSV64>; 8],
    hue_step: u32,
    modes: Vec<Mode>,
    zones: Vec<Zone>,
}

pub fn new_controller<const SIZE: usize>(
    leds: [LEDs<SIZE, hsv::HSV64>; 8],
    hue_step: u32,
) -> DeviceController<SIZE> {
    let mut modes = Vec::new();
    modes.push(Mode {
        name: format!("default"),
        value: 0,
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

    DeviceController {
        leds,
        hue_step,
        modes,
        zones,
    }
}

impl<'a, const SIZE: usize> DeviceController<SIZE> {
    pub fn handle(&self, data_in: &Vec<u8>) -> Result<Vec<u8>> {
        let mut data_out = Vec::new();
        let mut w = WriteVec::new(&mut data_out);
        let h = data_in.as_slice().read_any(DEFAULT_PROTOCOL)?;
        match h.packet_id {
            PacketId::RequestControllerCount => {
                RequestControllerCount::handle(self, h, data_in, &mut w)
            }
            PacketId::RequestProtocolVersion => {
                RequestProtocolVersion::handle(self, h, data_in, &mut w)
            }
            PacketId::RequestControllerData => {
                RequestControllerData::handle(self, h, data_in, &mut w)
            }
            _ => Err(Error::UnknownPacket(9999)),
        }?;

        Ok(data_out)
    }

    pub fn step_hue(&mut self) {
        for leds in self.leds.iter_mut() {
            for i in 0..SIZE {
                leds.channel0[i].step_hue(self.hue_step);
                leds.channel1[i].step_hue(self.hue_step);
            }
        }
    }

    pub fn inc_step(&mut self, inc: u32) {
        self.hue_step += inc;
    }

    pub fn bufs(&self) -> &[LEDs<SIZE, hsv::HSV64>; 8] {
        &self.leds
    }

    fn controller_count(&self) -> u32 {
        1
    }

    fn openrgb_leds(&self) -> Vec<LED> {
        let mut leds = Vec::new();
        for (i, device) in self.leds.iter().enumerate() {
            for (j, led) in device.channel0.iter().enumerate() {
                let rgb: RGB8 = (*led).into();
                leds.push(LED {
                    name: format!("device {i} led {j}"),
                    value: (rgb.r as u32) << 16 | (rgb.g as u32) << 8 | (rgb.b as u32),
                });
            }
            for (j, led) in device.channel1.iter().enumerate() {
                let rgb: RGB8 = (*led).into();
                leds.push(LED {
                    name: format!("device {i} led {j}"),
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
}

pub trait ResponseType {}

pub trait Handler: Send + Sync + Sized {
    fn handle<'a, const SIZE: usize>(
        controller: &DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()>;
}

pub struct RequestProtocolVersion {}

impl Handler for RequestProtocolVersion {
    fn handle<'a, const SIZE: usize>(
        controller: &DeviceController<SIZE>,
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

pub struct RequestControllerCount {}

impl Handler for RequestControllerCount {
    fn handle<const SIZE: usize>(
        controller: &DeviceController<SIZE>,
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

pub struct RequestControllerData {}

impl Handler for RequestControllerData {
    fn handle<const SIZE: usize>(
        controller: &DeviceController<SIZE>,
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
                controller.as_controller(),
            )
            .map_err(|e| e.into())
    }
}
