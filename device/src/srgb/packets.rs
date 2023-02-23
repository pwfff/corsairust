use binary_layout::prelude::*;

struct SRGBModsColor {
    r: u8,
    g: u8,
    b: u8,
}

fn ToColors(v: &[u8]) -> &[SRGBModsColor] {
    let mut ret: &[SRGBModsColor];
    for ele in v.chunks_exact(3) {
        ret[0] = SRGBModsColor {
            r: ele[0],
            g: ele[1],
            b: ele[2],
        }
    }

    ret
}

define_layout!(srgbmods_packet, BigEndian, {
    pad1: u8,
    packet_num: u8,
    pad2: u8,
    total_packets: u8,
    channel: u8,
    rgbs: [u8; 60],
});

// INFO  [6, 0, FF, 9, 1, A1, 1, 15, 0, 26, FF, 0, 75, 8, 95, 20, 81, 2, 91, 2, C0]
pub struct SRGBModsHostReport {}

pub fn ParseHostReport(v: &u8) -> &SRGBModsHostReport {}
