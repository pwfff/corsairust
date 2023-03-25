#[macro_use]
extern crate error_chain;
extern crate hidapi;

use enum_dispatch::enum_dispatch;
use futures::executor::block_on;
use genio::Write;
use hidapi::{DeviceInfo, HidDevice};
use openrgb::DEFAULT_PROTOCOL;
use openrgb_data::{
    Controller, Header, OpenRGBError, OpenRGBReadable, OpenRGBReadableSync, OpenRGBSync,
    OpenRGBWritable, OpenRGBWritableSync, PacketId,
};
use std::{
    pin::Pin,
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::{
    io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    spawn,
    sync::Mutex,
};

mod errors {
    use genio::error::ReadExactError;
    use openrgb_data::OpenRGBError;

    // Create the Error, ErrorKind, ResultExt, and Result types
    error_chain! {}

    impl From<OpenRGBError> for Error {
        fn from(s: OpenRGBError) -> Self {
            ErrorKind::Msg(format!("{:?}", s)).into()
        }
    }

    impl<T> From<ReadExactError<T>> for Error {
        fn from(s: ReadExactError<T>) -> Self {
            s.into()
        }
    }

    impl From<std::io::Error> for Error {
        fn from(s: std::io::Error) -> Self {
            s.into()
        }
    }

    impl From<hidapi::HidError> for Error {
        fn from(s: hidapi::HidError) -> Self {
            s.into()
        }
    }
}

use errors::*;

#[tokio::main]
async fn main() -> io::Result<()> {
    // Connect to device using its VID and PID
    let (VID, PID) = (0x16d0, 0x1124);

    let api = hidapi::HidApi::new().unwrap();

    let dev = connect(&api, VID, PID);
    let d = Arc::new(Mutex::new(dev));

    let s = Foo { device: d.clone() };

    spawn(s.serve());

    loop {
        match api.open(VID, PID) {
            Ok(device) => match poll(&device) {
                Ok(_) => {
                    let mut f = d.lock().await;
                    *f = device;
                }
                Err(e) => println!("err polling: {}", e),
            },
            Err(e) => println!("err connecting: {}", e),
        }
        std::thread::sleep(Duration::from_millis(100))
    }
}

fn connect(api: &hidapi::HidApi, VID: u16, PID: u16) -> hidapi::HidDevice {
    loop {
        match api.open(VID, PID) {
            Ok(device) => return device,
            Err(e) => println!("err connecting: {}", e),
        }
        std::thread::sleep(Duration::from_millis(1000))
    }
}

struct Foo {
    device: Arc<Mutex<HidDevice>>,
}

impl Foo {
    async fn handle(&mut self, s: &mut Blocker) -> Result<()> {
        let header = s
            .read_any(DEFAULT_PROTOCOL)
            .map_err(|e| format!("{:?}", e))?;

        let mut buf = Vec::<u8>::with_capacity(header.len as usize);
        s.read_exact(&mut buf).await?;

        let mut packet: Vec<u8> = Vec::new();
        packet.write_value(header, DEFAULT_PROTOCOL)?;
        packet.write(&mut buf, DEFAULT_PROTOCOL)?;
        // genio::Write::write_all(&mut packet, &buf)?;

        self.device.lock().await.write(&mut buf)?;

        Ok(())
    }

    // async fn new_device(&mut self, d: Box<hidapi::HidDevice>) {
    //     self.device.lock().await;
    //     self.device = Arc::new(Mutex::new(*d));
    // }

    async fn serve(mut self) -> Result<()> {
        let s = TcpListener::bind("localhost:8123").await?;
        loop {
            match s.accept().await {
                Ok((socket, _)) => {
                    let mut reader = Blocker {
                        s: Box::pin(socket),
                    };
                    self.handle(&mut reader).await?;
                }
                Err(e) => {
                    println!("{}", e);
                    // TODO: restart
                }
            };
        }
    }
}

fn poll(device: &hidapi::HidDevice) -> hidapi::HidResult<usize> {
    loop {
        // Read data from device
        let mut buf = [0u8; 64];
        let res = device.read_timeout(&mut buf[..], 150)?;
        println!("Read: {:?}", &buf[..res]);

        // Write data to device
        let mut buf = [0u8; 64];
        buf[17..33].copy_from_slice(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis()
                .to_be_bytes()
                .as_slice(),
        );
        // let res = device.send_feature_report(&buf)?;
        let res = device.write(&buf)?;
        println!("Wrote: {:?} byte(s)", res);
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
