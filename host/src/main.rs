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
    net::{
        tcp::{ReadHalf, WriteHalf},
        TcpListener, TcpStream,
    },
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
    println!("{:?}", s.ttl());

    loop {
        select! {
            Ok((mut socket, _)) = s.accept() => {
                socket.set_nodelay(true)?;
                let (reader, mut writer) = socket.split();
                let mut reader = Blocker {
                    s: Box::pin(reader),
                };
                match handle(&w, &mut reader, &mut writer).await {
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

async fn handle<'a>(dev: &DevWriter, s: &mut Blocker<'a>, w: &mut WriteHalf<'a>) -> Result<()> {
    loop {
        let header = s
            .read_any(DEFAULT_PROTOCOL)
            .map_err(|e| format!("{:X?}", e))?;
        println!("got header {:X?}", header);
        let mut buf = vec![0; header.len as usize];
        s.read_exact(&mut buf).await?;
        println!("got rest of packet {:X?}", buf);

        let total_len = header.len as usize + header.size(DEFAULT_PROTOCOL);
        let mut packet: Vec<u8> = Vec::<u8>::with_capacity(total_len);
        header.write(&mut packet, DEFAULT_PROTOCOL)?;
        buf.write(&mut packet, DEFAULT_PROTOCOL)?;

        println!("forwarding {:X?}", packet);

        let encoded = cobs::encode_vec(&packet);

        dev.write(&encoded)?;

        let mut from_device = Vec::<u8>::new();
        loop {
            let (got, mut gotted) = dev.read()?;
            println!("cobbed {}: {:X?}", got, gotted);
            from_device.append(&mut gotted.to_vec());
            if gotted.iter().any(|b| *b == 0) {
                break;
            }
        }
        //TODO: don't unwrap or something idk
        println!("all cobbed {}: {:X?}", from_device.len(), from_device);
        cobs::decode_in_place(from_device.as_mut()).unwrap();
        println!("uncobbed {}: {:X?}", from_device.len(), from_device);
        // orf orf orf orf
        // let f = header.size(DEFAULT_PROTOCOL);

        let header = from_device.as_slice().read_any(DEFAULT_PROTOCOL)?;
        println!("header from device: {:X?}", header);
        let len = header.len as usize;
        println!(
            "forwarding response {}: {:X?} {:X?}",
            len,
            header,
            &from_device[..16 + len]
        );
        w.write_all(&from_device[..16 + len]).await?;
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
        let mut buf = [0u8; 32];
        let read = self.open()?.read_timeout(&mut buf, 150)?;
        Ok((read, Box::new(buf)))
    }
}

struct TinyController {}

// impl RGBControllerInterface for TinyController {}

pub struct Blocker<'a> {
    s: Pin<Box<ReadHalf<'a>>>,
}

impl<'a> AsyncRead for Blocker<'a> {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut io::ReadBuf<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        self.s.as_mut().poll_read(cx, buf)
    }
}

impl<'a> std::io::Read for Blocker<'a> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        block_on(AsyncReadExt::read(self, buf))
    }
}

impl<'a> genio::Read for Blocker<'a> {
    type ReadError = crate::ErrorKind;

    fn read(&mut self, buf: &mut [u8]) -> std::result::Result<usize, Self::ReadError> {
        std::io::Read::read(self, buf).map_err(|e| ErrorKind::Msg(format!("{}", e)))
    }
}

// impl From<Blocker> for GenioWrite<Blocker> {
//     fn from(value: Blocker) -> Self {
//         GenioWrite::new(value)
//     }
// }
