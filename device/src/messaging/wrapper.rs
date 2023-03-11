use defmt::*;

// todo: more generic than rp_pico?
use postcard::from_bytes;
use rp_pico::hal::usb;
use serde::{Deserialize, Serialize};
use usb_device::UsbError;
use usbd_hid::hid_class::HIDClass;

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
        info!("{:X}", buf[0..j]);
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

    Ok(*message_buffer)
}
