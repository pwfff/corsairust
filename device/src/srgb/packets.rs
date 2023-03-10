use binary_layout::prelude::*;

define_layout!(wrapper, LittleEndian, {
    packet_id: u32,
    message_num: u8,
    message_count: u8,
    message: [u8; 26],
});
