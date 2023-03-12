use cobs::{decode_in_place, encode};
use defmt::*;

use hid::CustomBidirectionalReport;
// todo: more generic than rp_pico?
use postcard::from_bytes;
use rp_pico::hal::usb;
use serde::{Deserialize, Serialize};
use usb_device::UsbError;
use usbd_hid::hid_class::HIDClass;

use crate::messaging::Error;

// max bytes we can send per 'report'
pub const MAX_BUFFER_SIZE: usize = 32;

// max space we could possibly use with 'message_count' being a byte
pub const MAX_MESSAGE_SIZE: usize = MAX_BUFFER_SIZE * 255;

pub fn read_all(
    usb_hid: &'static HIDClass<'static, usb::UsbBus>,
) -> Result<[u8; MAX_MESSAGE_SIZE], UsbError> {
    // assuming this is first message, set ourselves up accordingly
    let mut message_buffer = &mut [0u8; MAX_MESSAGE_SIZE];

    let mut buf = [0u8; MAX_BUFFER_SIZE];
    let mut done = false;
    let mut i = 0;
    while !done {
        let j = usb_hid.pull_raw_output(&mut buf)?;
        info!("raw output size {}", j);
        info!("{:X}", buf);
        done = (j == 0) || buf.iter().any(|b| *b == 0);
        if j > 0 {
            message_buffer[i..i + j].copy_from_slice(&buf[0..j]);
            i += j;
        }
        if i > MAX_MESSAGE_SIZE {
            return Err(UsbError::BufferOverflow);
        }
        // Some(id) => match id {
        // 0 => {
        //     // let foo = RequestControllerCount::new(&message_buffer);
        //     // this is hardcoded 'request controller count', respond with 69
        //     let mut rep = CustomBidirectionalReport {
        //         input_buffer: [0u8; 32],
        //         output_buffer: [0u8; 32],
        //     };
        //     rep.input_buffer[6] = 69;
        //     match usb_hid.push_input(&rep) {
        //         Ok(i) => {
        //             info!("sent {} bytes", i)
        //         }

        //         Err(e) => e,
        //     };
        // }
        // _ => warn!("unknown packet ID {}", id),
        // },

        // None => {}
        // },
    }
    i = decode_in_place(message_buffer).map_err(|_| UsbError::ParseError)?;
    message_buffer[i..MAX_MESSAGE_SIZE].fill(0);

    debug!("read message ({} bytes) {:X}", i, message_buffer[0..i]);

    Ok(*message_buffer)
}

pub fn write_all<'a>(
    usb_hid: &'static HIDClass<'static, usb::UsbBus>,
    data: &'a mut [u8],
) -> Result<[u8; MAX_MESSAGE_SIZE], UsbError> {
    debug!("responding with {:X}", data);
    let message_buffer = &mut [0u8; MAX_MESSAGE_SIZE];
    let mut len = encode(data, message_buffer);
    debug!("responding with encoded {:X}", message_buffer[0..len]);

    let mut rep = CustomBidirectionalReport {
        input_buffer: [0u8; 32],
        output_buffer: [0u8; 32],
    };

    let mut i = 0;
    while len > 0 {
        let mut j = MAX_BUFFER_SIZE;
        if len < j {
            j = len;
        }
        rep.input_buffer[0..j].copy_from_slice(&message_buffer[i..i + j]);
        let n = usb_hid.push_input(&rep)?;

        debug!("wrote {} encoded bytes", j);

        if len >= MAX_BUFFER_SIZE {
            len -= MAX_BUFFER_SIZE;
        } else {
            len = 0;
        }
        i += MAX_BUFFER_SIZE;
    }

    Ok(*message_buffer)
}
