extern crate hidapi;

use hidapi::DeviceInfo;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[tokio::main]
async fn main() {
    let server = openrgb::OpenRGBServer::new("localhost:8123").await.unwrap();
    tokio::spawn(async move { server.listen().await });

    let api = hidapi::HidApi::new().unwrap();
    // Print out information about all connected devices
    loop {
        // Connect to device using its VID and PID
        let (VID, PID) = (0x16d0, 0x1124);
        // match api.open(VID, PID) {
        //     Ok(device) => match poll(&device) {
        //         Ok(_) => {}
        //         Err(e) => println!("err polling: {}", e),
        //     },
        //     Err(e) => println!("err connecting: {}", e),
        // }
        std::thread::sleep(Duration::from_millis(100))
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
