extern crate alloc;

use alloc::vec::Vec;
use corsairust_macros::all_fields_with;
use defmt::debug;
use openrgb_data::{OpenRGBReadableSync, OpenRGBWritableSync, PacketId, WriteVec};
use serde::{Deserialize, Serialize};

use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct Controller {}
impl Controller {
    pub fn handle<'a>(&self, data_in: &Vec<u8>, data_out: &mut Vec<u8>) -> Result<()> {
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

    fn controller_count(&self) -> u32 {
        69
    }
}

pub trait ResponseType {}

pub trait Handler: Send + Sync + Sized {
    fn handle<'a>(
        controller: &Controller,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()>;
}

pub struct RequestProtocolVersion {}

impl Handler for RequestProtocolVersion {
    fn handle<'a>(
        controller: &Controller,
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
    fn handle(controller: &Controller, data_in: &Vec<u8>, data_out: &mut WriteVec) -> Result<()> {
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
