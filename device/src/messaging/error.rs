extern crate alloc;

use alloc::{format, string::String};
use defmt::Format;

#[derive(Debug)]
pub enum Error {
    // TODO: Oops
    Oops(String),
    UnknownPacket(u32),
}

pub type Result<T> = ::core::result::Result<T, Error>;

impl From<openrgb_data::OpenRGBError> for Error {
    fn from(value: openrgb_data::OpenRGBError) -> Self {
        Self::Oops(format!("{value:?}"))
    }
}

impl Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        Format::format(format!("{self:?}").as_str(), fmt)
    }
}
