extern crate alloc;

use alloc::{string::ToString, vec::Vec};
use corsairust_macros::all_fields_with;
use defmt::debug;
use openrgb_data::{
    Controller, DeviceType, Header, OpenRGBReadableSync, OpenRGBWritableSync, PacketId, WriteVec,
};
use serde::{Deserialize, Serialize};
use ws2812_pio::LEDs;

use crate::hsv::HSV64;

use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct DeviceController<const SIZE: usize> {
    leds: [LEDs<SIZE, HSV64>; 8],
    hue_step: u32,
}

pub fn new_controller<const SIZE: usize>(
    leds: [LEDs<SIZE, HSV64>; 8],
    hue_step: u32,
) -> DeviceController<SIZE> {
    DeviceController { leds, hue_step }
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

    pub fn bufs(&self) -> &[LEDs<SIZE, HSV64>; 8] {
        &self.leds
    }

    fn controller_count(&self) -> u32 {
        69
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
        let c = Controller {
            r#type: DeviceType::LEDStrip,
            name: "foo".to_string(),
            vendor: "vendor".to_string(),
            description: "vendor".to_string(),
            version: "vendor".to_string(),
            serial: "vendor".to_string(),
            location: "vendor".to_string(),
            active_mode: 0,
            modes: Vec::new(),
            zones: Vec::new(),
            leds: Vec::new(),
            colors: Vec::new(),
        };
        data_out
            .write_packet(
                DEFAULT_PROTOCOL,
                // TODO: device ids i guess?
                header.device_id,
                PacketId::RequestControllerData,
                c,
            )
            .map_err(|e| e.into())
    }
}
