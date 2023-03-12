use arrayref::array_ref;
use postcard::fixint::*;
use serde::{Deserialize, Serialize};

use super::wrapper::MAX_MESSAGE_SIZE;
use super::{Error, Result};

pub struct Controller {}
impl Controller {
    pub fn handle<'a>(&self, data: &'a mut [u8; MAX_MESSAGE_SIZE]) -> Result<&'a mut [u8]> {
        let packet_id = u32::from_le_bytes(*array_ref!(data, 0, 4));
        let p = match packet_id {
            0 => Ok(RequestControllerCount::new(data)),
            i => Err(Error::UnknownPacket(i)),
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

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
pub struct RequestControllerCountResponse {
    packet_id: LE<u32>,
    controller_count: LE<u32>,
}

impl Handler<RequestControllerCountResponse> for RequestControllerCount {
    fn new(_: &[u8; MAX_MESSAGE_SIZE]) -> Self {
        // we don't need any info from the message
        Self {}
    }

    fn handle(&self, controller: &Controller) -> RequestControllerCountResponse {
        RequestControllerCountResponse {
            packet_id: 0.into(),
            controller_count: controller.controller_count().into(),
        }
    }
}
