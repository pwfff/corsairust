use defmt::*;

// todo: more generic than rp_pico?
use postcard::from_bytes;
use rp_pico::hal::usb;
use serde::{Deserialize, Serialize};
use usb_device::UsbError;
use usbd_hid::hid_class::HIDClass;

// max bytes we can send per 'report'
pub const MAX_BUFFER_SIZE: usize = 32;

// max size of the actual message content per 'report' (6 byte header)
const BYTES_PER_MESSAGE: usize = MAX_BUFFER_SIZE - 6;

// max space we could possibly use with 'message_count' being a byte
pub const MAX_MESSAGE_SIZE: usize = BYTES_PER_MESSAGE * 255;

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
struct Wrapper {
    packet_id: u32,
    message_num: u8,
    message_count: u8,
    message: [u8; BYTES_PER_MESSAGE],
}

pub fn read_all(
    usb_hid: &'static HIDClass<'static, usb::UsbBus>,
) -> Result<[u8; MAX_MESSAGE_SIZE], UsbError> {
    // assuming this is first message, set ourselves up accordingly
    let mut message_buffer = &mut [0u8; MAX_MESSAGE_SIZE];
    let mut wrapper_buffer = &mut [0u8; MAX_BUFFER_SIZE];

    let mut buf = [0u8; MAX_BUFFER_SIZE];
    let mut more = None;
    while let None = more {
        let i = usb_hid.pull_raw_output(&mut buf)?;
        info!("raw output size {}", i);
        info!("{:X}", buf[0..i]);
        more = process_message(&mut message_buffer, &mut wrapper_buffer, &buf).map_err(|e| {
            error!("{}", e);
            UsbError::InvalidState
        })?;
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

    Ok(*message_buffer)
}

pub fn process_message(
    message_buffer: &mut [u8; MAX_MESSAGE_SIZE],
    wrapper_buffer: &mut [u8; MAX_BUFFER_SIZE],
    data: &[u8; 32],
) -> Result<Option<u32>, &'static str> {
    let message = from_bytes::<Wrapper>(wrapper_buffer).map_err(|e| {
        error!("{}", e);
        "error deserializing new message"
    })?;
    // first message, clear message_buffer and start from the top
    if message.message_num == 0 {
        debug!("processing first message in group");
        wrapper_buffer.copy_from_slice(data);
        message_buffer.fill(0);
        message_buffer[0..26].copy_from_slice(&message.message[..]);
        if message.message_count == 1 {
            return Ok(Some(message.packet_id));
        }
        return Ok(None);
    }

    debug!("processing later message in group");

    // not the first message, try to keep appending to message_buffer
    let old_message = from_bytes::<Wrapper>(wrapper_buffer).map_err(|e| {
        error!("{}", e);
        "error deserializing old message?"
    })?;
    let id = message.packet_id;
    let old_id = old_message.packet_id;
    if id != old_id {
        return Err("unexpected message ID");
    }

    if message.message_count != old_message.message_count {
        return Err("message count changed unexpectedly");
    }

    if message.message_num != old_message.message_num + 1 {
        return Err("message out of order");
    }

    let i: usize = message.message_num.into();
    wrapper_buffer.copy_from_slice(data);
    message_buffer[i * 26..(i + 1) * 26].copy_from_slice(&message.message[..]);
    if message.message_num + 1 == message.message_count {
        return Ok(Some(message.packet_id));
    }
    return Ok(None);
}
