use corsairust_macros::all_fields_with;
use openrgb_data::{OpenRGBReadableSync, PacketId};
use serde::{Deserialize, Serialize};

use super::wrapper::MAX_MESSAGE_SIZE;
use super::{Error, Result};

pub static DEFAULT_PROTOCOL: u32 = 3;

pub struct Controller {}
impl Controller {
    pub fn handle<'a>(&self, data: &'a mut [u8; MAX_MESSAGE_SIZE]) -> Result<&'a mut [u8]> {
        let h = OpenRGBReadableSync::read_any(&mut data.as_slice(), DEFAULT_PROTOCOL)
            .map_err(|e| Error::Oops("()"))?;
        let p = match h.packet_id {
            PacketId::RequestControllerCount => Ok(RequestControllerCount::new(data)),
            i => Err(Error::UnknownPacket(9999)),
        }?;

        let r = p.handle(self);

        postcard::to_slice(&r, data).map_err(|e| Error::PostcardError(e))
    }

    fn controller_count(&self) -> u32 {
        69
    }
}

pub trait ResponseType {}

pub trait Handler<R: Serialize> {
    fn new(data: &[u8; MAX_MESSAGE_SIZE]) -> Self;
    fn handle(&self, controller: &Controller) -> R;
}

pub struct RequestControllerCount {}

#[all_fields_with("postcard::fixint::le")]
#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct RequestControllerCountResponse {
    packet_id: u32,
    controller_count: u32,
}

impl Handler<RequestControllerCountResponse> for RequestControllerCount {
    fn new(_: &[u8; MAX_MESSAGE_SIZE]) -> Self {
        // we don't need any info from the message
        Self {}
    }

    fn handle(&self, controller: &Controller) -> RequestControllerCountResponse {
        RequestControllerCountResponse {
            packet_id: 0,
            controller_count: controller.controller_count(),
        }
    }
}
