use alloc::vec::Vec;
use cobs::{decode_in_place, encode, max_encoding_length};
use defmt::*;

// todo: more generic than rp_pico?
use rp_pico::hal::usb;
use usb_device::UsbError;
use usbd_hid::hid_class::HIDClass;

// max bytes we can send per 'report'
pub const MAX_BUFFER_SIZE: usize = 32;

pub fn read_all(usb_hid: &'static HIDClass<'static, usb::UsbBus>) -> Result<Vec<u8>, UsbError> {
    info!("initializing reading stuff");
    // assuming this is first message, set ourselves up accordingly
    let mut message_buffer = Vec::new();

    let mut buf = [0u8; MAX_BUFFER_SIZE * 2];
    let mut done = false;
    let mut i = 0;

    while !done {
        buf.fill(0);

        let mut attempts = 0;
        let mut maybe: Option<usize> = None;
        while maybe == None {
            match usb_hid.pull_raw_output(&mut buf) {
                Err(e) => match e {
                    usb_device::UsbError::WouldBlock => {
                        attempts += 1;
                        if attempts > 10 {
                            debug!("bailing after 10 WouldBlocks");
                            return Err(e);
                        }
                    }
                    _ => return Err(e),
                },
                Ok(j) => {
                    maybe = Some(j);
                }
            }
        }

        let j = maybe.unwrap();
        info!("raw output size {}", j);
        info!("{:X}", buf);

        if j == 0 {
            done = true;
            debug!("done because 0 len message");
        } else if buf.iter().any(|b| *b == 0) {
            done = true;
            debug!("done because found 0");
        }
        if j > 0 {
            message_buffer.extend(buf[0..j].iter());
            i += j;
        }

        debug!(
            "message so far ({} bytes pre decode) {:X}",
            i,
            message_buffer.as_slice()
        );
    }

    i = decode_in_place(message_buffer.as_mut_slice()).map_err(|_| UsbError::ParseError)?;

    message_buffer.truncate(i);

    debug!("read message ({} bytes) {:X}", i, message_buffer.as_slice());

    Ok(message_buffer)
}

pub fn write_all<'a>(
    usb_hid: &'static HIDClass<'static, usb::UsbBus>,
    data: &'a mut [u8],
) -> Result<(), UsbError> {
    debug!("responding with {:X}", data);
    let mut message_buffer = vec![0; max_encoding_length(data.len())];
    let mut len = encode(data, message_buffer.as_mut_slice());
    debug!("responding with encoded {:X}", message_buffer[0..len]);

    let mut i = 0;
    while len > 0 {
        // debug!("{} left to write", len);
        let mut j = MAX_BUFFER_SIZE;
        if len < j {
            j = len;
        }
        // rep.input_buffer[0..j].copy_from_slice(&message_buffer[i..i + j]);
        loop {
            match usb_hid.push_raw_input(&message_buffer[i..i + j]) {
                Ok(_) => break,
                Err(e) => match e {
                    UsbError::WouldBlock => {}
                    e => return Err(e),
                },
            }
        }

        // debug!("wrote {} encoded bytes: {:X}", j, message_buffer[i..i + j]);

        if len >= MAX_BUFFER_SIZE {
            len -= MAX_BUFFER_SIZE;
        } else {
            len = 0;
        }
        i += MAX_BUFFER_SIZE;
        // debug!("i {} j {} len {}", i, j, len);
    }

    debug!("responded");

    Ok(())
}
