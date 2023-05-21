#[macro_use]
extern crate error_chain;
extern crate hidapi;

use error_chain::ChainedError;
use futures::executor::block_on;
use genio::Read;
use hidapi::HidDevice;
use openrgb::{
    data::OpenRGBReadable, data::OpenRGBWritable, data::PacketId, OpenRGBReadableStream,
    OpenRGBStream, OpenRGBWritableStream, DEFAULT_PROTOCOL,
};
use openrgb_data::{
    Controller, Header, OpenRGBError, OpenRGBReadableSync, OpenRGBSync, OpenRGBWritableSync,
};
use std::{pin::Pin, sync::Arc};
use tokio::{
    io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt},
    net::{
        tcp::{ReadHalf, WriteHalf},
        TcpListener, TcpStream,
    },
    select,
    sync::Mutex,
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

    impl From<openrgb::OpenRGBError> for Error {
        fn from(s: openrgb::OpenRGBError) -> Self {
            format!("{:?}", s).into()
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
    tracing_subscriber::fmt::init();

    // Connect to device using its VID and PID
    let (VID, PID) = (0x16d0, 0x1124);

    let api = hidapi::HidApi::new().unwrap();

    let w = Arc::new(Mutex::new(Device {
        api,
        address: (VID, PID),
    }));

    // spawn(s.serve());

    let s = TcpListener::bind("127.0.0.1:6742").await?;

    println!("listening on 6742");
    println!("{:?}", s.ttl());

    loop {
        match s.accept().await {
            Ok((mut socket, _)) => {
                socket.set_nodelay(true)?;
                // let mut reader = Blocker {
                //     s: Box::pin(reader),
                // };
                let w = w.clone();
                tokio::spawn(async move {
                    match handle(w, socket).await {
                        Ok(_) => println!("handled a packet bruv"),
                        Err(e) => {
                            println!("error handling packet: {}", e.display_chain().to_string())
                        }
                    }
                });
            }
            Err(_) => {}
        }
    }
}

async fn handle(dev: Arc<Mutex<Device>>, mut sock: TcpStream) -> Result<()> {
    loop {
        println!("looking for packet");
        let header = sock
            .read_any(DEFAULT_PROTOCOL)
            .await
            .map_err(|e| format!("{:X?}", e))?;
        let id = header.packet_id.clone();
        println!("got header {:X?}", header);
        let mut buf = vec![0; header.len as usize];
        sock.read_exact(&mut buf).await?;
        println!("got rest of packet {:X?}", buf);

        let total_len = header.len as usize + header.size(DEFAULT_PROTOCOL);
        let mut packet: Vec<u8> = Vec::<u8>::with_capacity(total_len);
        header.write(&mut packet, DEFAULT_PROTOCOL).await?;
        buf.write(&mut packet, DEFAULT_PROTOCOL).await?;

        println!("forwarding to device {:X?}", packet);

        let encoded = cobs::encode_vec(&packet);

        let mut from_device = {
            let lock = dev.lock().await.open()?;
            lock.write(&encoded)?;

            if id == PacketId::SetClientName || id == PacketId::RGBControllerUpdateMode {
                println!("bailing because no expected response");
                // TODO: all the other packets without responses...
                continue;
            }

            let mut from_device = Vec::<u8>::new();
            println!("trying to read from device");
            loop {
                let mut gotted = [0; 32];
                let got = lock.read(&mut gotted)?;
                if got > 0 {
                    from_device.append(&mut gotted.to_vec());
                    if gotted.iter().any(|b| *b == 0) {
                        break;
                    }
                }
            }
            from_device
        };

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
            "forwarding response to client {}: {:X?} {:X?}",
            len,
            header,
            &from_device[..16 + len]
        );
        sock.write_all(&from_device[..16 + len]).await?;
        println!("wrote response {}: {:X?}", len, &from_device[..16 + len]);
    }
}

struct Device {
    api: hidapi::HidApi,
    address: (u16, u16),
}

impl Device {
    fn open(&self) -> Result<HidDevice> {
        self.api
            .open(self.address.0, self.address.1)
            .chain_err(|| "could not open device")
    }
}
