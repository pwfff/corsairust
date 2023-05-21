use alloc::vec::Vec;
use enum_dispatch::enum_dispatch;
use openrgb_data::{Header, PacketId, WriteVec};

use super::{packets::*, Error, Result};
// use enum_primitive_derive::Primitive;
// use num_traits::FromPrimitive;

#[enum_dispatch]
pub trait Handler {
    fn handle<'a, const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()>;
}

impl Handler for () {
    fn handle<'a, const SIZE: usize>(
        &self,
        controller: &mut DeviceController<SIZE>,
        header: Header,
        data_in: &Vec<u8>,
        data_out: &mut WriteVec,
    ) -> Result<()> {
        Err(Error::UnknownPacket(header.packet_id as u32))
    }
}

#[macro_export]
macro_rules! packet {
    ($v:ident, $($p:literal),* $(,)?) => {};
}

// #[derive(Primitive, PartialEq, Debug, Copy, Clone, Default)]
#[repr(u32)]
#[enum_dispatch(Handler)]
pub enum PacketIdHandler {
    /// Request RGBController device count from server.
    RequestControllerCount,

    /// Request RGBController data block.
    RequestControllerData,

    /// Request OpenRGB SDK protocol version from server.
    RequestProtocolVersion,

    /// Send client name string to server.
    SetClientName,

    // /// Indicate to clients that device list has updated.
    // DeviceListUpdated = 100,
    /// Request profile list.
    RequestProfileList,

    /// Save current configuration in a new profile.
    RequestSaveProfile,

    /// Load a given profile.
    RequestLoadProfile,

    /// Delete a given profile.
    RequestDeleteProfile,

    // /// RGBController::ResizeZone().
    // RGBControllerResizeZone = 1000,

    // /// RGBController::UpdateLEDs().
    // RGBControllerUpdateLeds = 1050,

    // /// RGBController::UpdateZoneLEDs().
    // RGBControllerUpdateZoneLeds = 1051,

    // /// RGBController::UpdateSingleLED().
    // RGBControllerUpdateSingleLed = 1052,

    // /// RGBController::SetCustomMode().
    // RGBControllerSetCustomMode = 1100,
    /// RGBController::UpdateMode().
    RGBControllerUpdateMode,

    // /// RGBController::SaveMode().
    // RGBControllerSaveMode = 1102,
    UnknownPacket(()),
}

impl From<PacketId> for PacketIdHandler {
    fn from(value: PacketId) -> Self {
        match value {
            PacketId::RequestControllerCount => {
                PacketIdHandler::RequestControllerCount(RequestControllerCount::default())
            }
            PacketId::RequestControllerData => {
                PacketIdHandler::RequestControllerData(RequestControllerData::default())
            }
            PacketId::RequestProtocolVersion => {
                PacketIdHandler::RequestProtocolVersion(RequestProtocolVersion::default())
            }
            PacketId::SetClientName => PacketIdHandler::SetClientName(SetClientName::default()),
            PacketId::RequestProfileList => {
                PacketIdHandler::RequestProfileList(RequestProfileList::default())
            }
            PacketId::RequestSaveProfile => {
                PacketIdHandler::RequestSaveProfile(RequestSaveProfile::default())
            }
            PacketId::RequestLoadProfile => {
                PacketIdHandler::RequestLoadProfile(RequestLoadProfile::default())
            }
            PacketId::RequestDeleteProfile => {
                PacketIdHandler::RequestDeleteProfile(RequestDeleteProfile::default())
            }
            PacketId::RGBControllerUpdateMode => {
                PacketIdHandler::RGBControllerUpdateMode(RGBControllerUpdateMode::default())
            }
            _ => PacketIdHandler::UnknownPacket(()),
        }
    }
}
