#![no_std]
pub use usbd_hid::descriptor::generator_prelude::*;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = 0xFF69, usage = 0x0200) = {
        (usage = 0x01,) = {
            input_buffer=input;
        };
         (usage = 0x02,) = {
            output_buffer=output;
        };
    }
 )]
pub struct CustomBidirectionalReport {
    pub input_buffer: [u8; 32],
    pub output_buffer: [u8; 32],
}
