#[derive(defmt::Format)]
pub enum Error {
    // TODO: Oops
    Oops(&'static str),
    UnknownPacket(u32),
    PostcardError(postcard::Error),
}

pub type Result<T> = ::core::result::Result<T, Error>;
