extern crate alloc;

use alloc::vec::Vec;
use corsairust_macros::all_fields_with;
use defmt::debug;
use openrgb_data::{OpenRGBReadableSync, OpenRGBWritableSync, PacketId, WriteVec};
use serde::{Deserialize, Serialize};
use ws2812_pio::LEDs;

use crate::hsv::HSV64;

use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct Controller<const SIZE: usize> {
    leds: [LEDs<SIZE, HSV64>; 8],
    hue_step: u32,
}

pub fn new_controller<const SIZE: usize>(
    leds: [LEDs<SIZE, HSV64>; 8],
    hue_step: u32,
) -> Controller<SIZE> {
    Controller { leds, hue_step }
}

impl<'a, const SIZE: usize> Controller<SIZE> {
    pub fn handle(&self, data_in: &Vec<u8>, data_out: &mut Vec<u8>) -> Result<()> {
        let mut w = WriteVec::new(data_out);
        let h = data_in.as_slice().read_any(DEFAULT_PROTOCOL)?;
        match h.packet_id {
            PacketId::RequestControllerCount => {
                RequestControllerCount::handle(self, data_in, &mut w)
            }
            PacketId::RequestProtocolVersion => {
                RequestProtocolVersion::handle(self, data_in, &mut w)
            }
            i => Err(Error::UnknownPacket(9999)),
        }
    }

    pub fn step_hue(&mut self) {
        for leds in self.leds.iter_mut() {
            for i in 0..leds.channel0.len() {
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
        controller: &Controller<SIZE>,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()>;
}

pub struct RequestProtocolVersion {}

impl Handler for RequestProtocolVersion {
    fn handle<'a, const SIZE: usize>(
        controller: &Controller<SIZE>,
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

#[all_fields_with("postcard::fixint::le")]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct RequestControllerCountResponse {
    packet_id: u32,
    controller_count: u32,
}

impl Handler for RequestControllerCount {
    fn handle<const SIZE: usize>(
        controller: &Controller<SIZE>,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        data_out
            .write_packet(
                DEFAULT_PROTOCOL,
                // TODO: device ids i guess?
                0,
                PacketId::RequestProtocolVersion,
                controller.controller_count(),
            )
            .map_err(|e| e.into())
    }
}
