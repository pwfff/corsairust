extern crate hidapi;

use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let api = hidapi::HidApi::new().unwrap();
    // Print out information about all connected devices
    for device in api.device_list() {
        println!("{:#?}", device);
    }

    loop {
        let (VID, PID) = (0x16d0, 0x1124);
        let device = api.open(VID, PID).unwrap();

        // Read data from device
        let mut buf = [0u8; 33];
        let res = device.read(&mut buf[..]).unwrap();
        println!("Read: {:?}", &buf[..res]);

        // Write data to device
        let mut buf = [0u8; 33];
        buf[17..33].copy_from_slice(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis()
                .to_be_bytes()
                .as_slice(),
        );
        let res = device.write(&buf).unwrap();
        println!("Wrote: {:?} byte(s)", res);
    }
    // Connect to device using its VID and PID
}
