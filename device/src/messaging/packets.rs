use binary_layout::prelude::*;

use super::wrapper::MAX_MESSAGE_SIZE;

pub trait Handler {
    fn new(data: &[u8; MAX_MESSAGE_SIZE]) -> Self;
    fn handle(&self, buf: &mut [u8; MAX_MESSAGE_SIZE]);
}

define_layout!(request_controller_count_response, LittleEndian, {
    controller_count: u32,
});

pub struct RequestControllerCount {}

impl Handler for RequestControllerCount {
    fn new(_: &[u8; MAX_MESSAGE_SIZE]) -> Self {
        // we don't need any info from the message
        Self {}
    }

    fn handle(&self, buf: &mut [u8; MAX_MESSAGE_SIZE]) {
        let mut view = request_controller_count_response::View::new(buf);
        view.controller_count_mut().write(6);
    }
}
