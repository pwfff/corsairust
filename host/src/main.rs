#[macro_use]
extern crate error_chain;
extern crate hidapi;

use error_chain::ChainedError;
use futures::executor::block_on;
use hidapi::HidDevice;
use openrgb::DEFAULT_PROTOCOL;
use openrgb_data::{
    Controller, Header, OpenRGBError, OpenRGBReadable, OpenRGBReadableSync, OpenRGBSync,
    OpenRGBWritable, OpenRGBWritableSync, PacketId,
};
use std::pin::Pin;
use tokio::{
    io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    select,
    time::{sleep, Duration},
};

mod errors {
    use genio::error::BufferOverflow;
    use openrgb_data::OpenRGBError;

    // Create the Error, ErrorKind, ResultExt, and Result types
    error_chain! {
            foreign_links {
                Fmt(::std::fmt::Error);
                Io(::std::io::Error);
                Hid(::hidapi::HidError);
            }
    }

    impl From<OpenRGBError> for Error {
        fn from(s: OpenRGBError) -> Self {
            format!("{:?}", s).into()
        }
    }
}

use errors::*;

#[tokio::main]
async fn main() -> io::Result<()> {
    // Connect to device using its VID and PID
    let (VID, PID) = (0x16d0, 0x1124);

    let api = hidapi::HidApi::new().unwrap();

    let w = DevWriter {
        api,
        address: (VID, PID),
    };

    // spawn(s.serve());

    let s = TcpListener::bind("127.0.0.1:6742").await?;

    println!("listening on 6742");

    loop {
        select! {
            Ok((socket, _)) = s.accept() => {
                let mut reader = Blocker {
                    s: Box::pin(socket),
                };
                match handle(&w, &mut reader).await {
                    Ok(_) => println!("handled a packet bruv"),
                    Err(e) => println!("error handling packet: {}", e.display_chain().to_string()),
                };
            }

            _ = sleep(Duration::from_millis(100)) => {
                match w.read() {
                    Ok((s, b)) => {
                        if s > 0 {
                            println!("unexpected input from device: {b:X?}");
                        }
                    }
                    Err(e) => {
                        println!("{e:?}")
                    }
                }
            }
        }
    }
}

async fn handle(dev: &DevWriter, s: &mut Blocker) -> Result<()> {
    loop {
        let header = s
            .read_any(DEFAULT_PROTOCOL)
            .map_err(|e| format!("{:?}", e))?;
        println!("got header {:?}", header);
        let mut buf = vec![0; header.len as usize];
        s.read_exact(&mut buf).await?;
        println!("got rest of packet {:?}", buf);

        let total_len = header.len as usize + header.size(DEFAULT_PROTOCOL);
        let mut packet: Vec<u8> = Vec::<u8>::with_capacity(total_len);
        header.write(&mut packet, DEFAULT_PROTOCOL)?;
        buf.write(&mut packet, DEFAULT_PROTOCOL)?;

        println!("forwarding {:X?}", packet);

        let encoded = cobs::encode_vec(&packet);

        dev.write(&encoded)?;

        let (got, mut gotted) = dev.read()?;
        //TODO: don't unwrap or something idk
        cobs::decode_in_place(gotted.as_mut()).unwrap();
        println!("{}: {:X?}", got, gotted);
        // orf orf orf orf
        // let f = header.size(DEFAULT_PROTOCOL);

        let header = gotted.as_ref().read_any(DEFAULT_PROTOCOL)?;
        let len = header.len as usize;
        println!(
            "forwarding response {}: {:X?} {:X?}",
            len,
            header,
            &gotted[16..16 + len]
        );
        s.write_all(&gotted[..16 + len]).await?;
    }
}

struct DevWriter {
    api: hidapi::HidApi,
    address: (u16, u16),
}

impl DevWriter {
    fn open(&self) -> Result<HidDevice> {
        self.api
            .open(self.address.0, self.address.1)
            .chain_err(|| "could not open device")
    }

    fn write(&self, buf: &[u8]) -> Result<usize> {
        self.open()?
            .write(&buf)
            .chain_err(|| "error writing to device")
    }

    fn read(&self) -> Result<(usize, Box<[u8]>)> {
        let mut buf = [0u8; 64];
        let read = self.open()?.read_timeout(&mut buf, 150)?;
        Ok((read, Box::new(buf)))
    }
}

struct TinyController {}

// impl RGBControllerInterface for TinyController {}

pub struct Blocker {
    s: Pin<Box<TcpStream>>,
}

impl AsyncRead for Blocker {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut io::ReadBuf<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        self.s.as_mut().poll_read(cx, buf)
    }
}

impl std::io::Read for Blocker {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        block_on(AsyncReadExt::read(self, buf))
    }
}

impl genio::Read for Blocker {
    type ReadError = crate::ErrorKind;

    fn read(&mut self, buf: &mut [u8]) -> std::result::Result<usize, Self::ReadError> {
        std::io::Read::read(self, buf).map_err(|e| ErrorKind::Msg(format!("{}", e)))
    }
}

impl AsyncWrite for Blocker {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &[u8],
    ) -> std::task::Poll<std::result::Result<usize, std::io::Error>> {
        self.s.as_mut().poll_write(cx, buf)
    }

    fn poll_flush(
        mut self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<std::result::Result<(), std::io::Error>> {
        self.s.as_mut().poll_flush(cx)
    }

    fn poll_shutdown(
        mut self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<std::result::Result<(), std::io::Error>> {
        self.s.as_mut().poll_shutdown(cx)
    }
}

impl std::io::Write for Blocker {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        block_on(AsyncWriteExt::write(self, buf))
    }

    fn flush(&mut self) -> io::Result<()> {
        block_on(AsyncWriteExt::flush(self))
    }
}

// impl From<Blocker> for GenioWrite<Blocker> {
//     fn from(value: Blocker) -> Self {
//         GenioWrite::new(value)
//     }
// }

impl genio::Write for Blocker {
    type WriteError = ErrorKind;

    type FlushError = ErrorKind;

    fn write(&mut self, buf: &[u8]) -> std::result::Result<usize, Self::WriteError> {
        std::io::Write::write(self, buf).map_err(|e| ErrorKind::Msg(format!("{}", e)))
    }

    fn flush(&mut self) -> std::result::Result<(), Self::FlushError> {
        std::io::Write::flush(self).map_err(|e| ErrorKind::Msg(format!("{}", e)))
    }

    fn size_hint(&mut self, bytes: usize) {}
}

impl OpenRGBSync for Blocker {}
