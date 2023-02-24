use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rusb::{
    Context, Device, DeviceDescriptor, DeviceHandle, Direction, Result, TransferType, UsbContext,
};

extern crate rusb;

const VID: u16 = 0x16d0;
const PID: u16 = 0x1124;

#[derive(Debug)]
struct Endpoint {
    config: u8,
    iface: u8,
    setting: u8,
    address: u8,
}

fn main() {
    loop {
        match rusb::Context::new() {
            Ok(mut context) => match open_device(&mut context, VID, PID) {
                Some((mut device, device_desc, mut handle)) => {
                    let mut buf = [0u8; 63];
                    buf[16..32].copy_from_slice(
                        SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap()
                            .as_millis()
                            .to_be_bytes()
                            .as_slice(),
                    );
                    buf[1] = buf[buf.len() - 1];
                    match find_writable_endpoint(&mut device, &device_desc, TransferType::Interrupt)
                    {
                        Some(endpoint) => {
                            write_endpoint(&mut handle, endpoint, TransferType::Interrupt, &mut buf)
                        }
                        None => println!("No writable interrupt endpoint"),
                    }

                    match find_writable_endpoint(&mut device, &device_desc, TransferType::Bulk) {
                        Some(endpoint) => {
                            write_endpoint(&mut handle, endpoint, TransferType::Bulk, &mut buf)
                        }
                        None => println!("No writable bulk endpoint"),
                    }

                    match find_readable_endpoint(&mut device, &device_desc, TransferType::Interrupt)
                    {
                        Some(endpoint) => {
                            read_endpoint(&mut handle, endpoint, TransferType::Interrupt)
                        }
                        None => println!("No readable interrupt endpoint"),
                    }

                    match find_readable_endpoint(&mut device, &device_desc, TransferType::Bulk) {
                        Some(endpoint) => read_endpoint(&mut handle, endpoint, TransferType::Bulk),
                        None => println!("No readable bulk endpoint"),
                    }
                }
                None => {}
            },
            Err(e) => {
                eprintln!("error opening device: {:?}", e)
            }
        }
        std::thread::sleep(Duration::from_millis(500))
    }
}

fn open_device<T: UsbContext>(
    context: &mut T,
    vid: u16,
    pid: u16,
) -> Option<(Device<T>, DeviceDescriptor, DeviceHandle<T>)> {
    let devices = match context.devices() {
        Ok(d) => d,
        Err(_) => return None,
    };

    for device in devices.iter() {
        let device_desc = match device.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };

        if device_desc.vendor_id() == vid && device_desc.product_id() == pid {
            match device.open() {
                Ok(handle) => return Some((device, device_desc, handle)),
                Err(e) => panic!("Device found but failed to open: {}", e),
            }
        }
    }

    None
}

fn find_writable_endpoint<T: UsbContext>(
    device: &mut Device<T>,
    device_desc: &DeviceDescriptor,
    transfer_type: TransferType,
) -> Option<Endpoint> {
    for n in 0..device_desc.num_configurations() {
        let config_desc = match device.config_descriptor(n) {
            Ok(c) => c,
            Err(_) => continue,
        };

        for interface in config_desc.interfaces() {
            for interface_desc in interface.descriptors() {
                for endpoint_desc in interface_desc.endpoint_descriptors() {
                    if endpoint_desc.direction() == Direction::Out
                        && endpoint_desc.transfer_type() == transfer_type
                    {
                        return Some(Endpoint {
                            config: config_desc.number(),
                            iface: interface_desc.interface_number(),
                            setting: interface_desc.setting_number(),
                            address: endpoint_desc.address(),
                        });
                    }
                }
            }
        }
    }

    None
}

fn write_endpoint(
    handle: &mut DeviceHandle<Context>,
    endpoint: Endpoint,
    transfer_type: TransferType,
    buf: &mut [u8],
) {
    println!("Writing to endpoint: {:?}", endpoint);

    let has_kernel_driver = match handle.kernel_driver_active(endpoint.iface) {
        Ok(true) => {
            handle.detach_kernel_driver(endpoint.iface).ok();
            true
        }
        _ => false,
    };

    println!(" - kernel driver? {}", has_kernel_driver);

    match configure_endpoint(handle, &endpoint) {
        Ok(_) => {
            let timeout = Duration::from_secs(1);

            match transfer_type {
                TransferType::Interrupt => {
                    match handle.write_interrupt(endpoint.address, &buf, timeout) {
                        Ok(len) => {
                            println!(" - write: {:?}", len);
                        }
                        Err(err) => println!("could not read from endpoint: {}", err),
                    }
                }
                TransferType::Bulk => match handle.write_bulk(endpoint.address, &buf, timeout) {
                    Ok(len) => {
                        println!(" - write: {:?}", len);
                    }
                    Err(err) => println!("could not read from endpoint: {}", err),
                },
                _ => (),
            }
        }
        Err(err) => println!("could not configure endpoint: {}", err),
    }

    if has_kernel_driver {
        handle.attach_kernel_driver(endpoint.iface).ok();
    }
}

fn find_readable_endpoint<T: UsbContext>(
    device: &mut Device<T>,
    device_desc: &DeviceDescriptor,
    transfer_type: TransferType,
) -> Option<Endpoint> {
    for n in 0..device_desc.num_configurations() {
        let config_desc = match device.config_descriptor(n) {
            Ok(c) => c,
            Err(_) => continue,
        };

        for interface in config_desc.interfaces() {
            for interface_desc in interface.descriptors() {
                for endpoint_desc in interface_desc.endpoint_descriptors() {
                    if endpoint_desc.direction() == Direction::In
                        && endpoint_desc.transfer_type() == transfer_type
                    {
                        return Some(Endpoint {
                            config: config_desc.number(),
                            iface: interface_desc.interface_number(),
                            setting: interface_desc.setting_number(),
                            address: endpoint_desc.address(),
                        });
                    }
                }
            }
        }
    }

    None
}

fn read_endpoint<T: UsbContext>(
    handle: &mut DeviceHandle<T>,
    endpoint: Endpoint,
    transfer_type: TransferType,
) {
    println!("Reading from endpoint: {:?}", endpoint);

    let has_kernel_driver = match handle.kernel_driver_active(endpoint.iface) {
        Ok(true) => {
            handle.detach_kernel_driver(endpoint.iface).ok();
            true
        }
        _ => false,
    };

    println!(" - kernel driver? {}", has_kernel_driver);

    match configure_endpoint(handle, &endpoint) {
        Ok(_) => {
            let mut buf = [0; 256];
            let timeout = Duration::from_secs(1);

            match transfer_type {
                TransferType::Interrupt => {
                    match handle.read_interrupt(endpoint.address, &mut buf, timeout) {
                        Ok(len) => {
                            println!(" - read: {:?}", &buf[..len]);
                        }
                        Err(err) => println!("could not read from endpoint: {}", err),
                    }
                }
                TransferType::Bulk => match handle.read_bulk(endpoint.address, &mut buf, timeout) {
                    Ok(len) => {
                        println!(" - read: {:?}", &buf[..len]);
                    }
                    Err(err) => println!("could not read from endpoint: {}", err),
                },
                _ => (),
            }
        }
        Err(err) => println!("could not configure endpoint: {}", err),
    }

    if has_kernel_driver {
        handle.attach_kernel_driver(endpoint.iface).ok();
    }
}

fn configure_endpoint<T: UsbContext>(
    handle: &mut DeviceHandle<T>,
    endpoint: &Endpoint,
) -> Result<()> {
    // handle.set_active_configuration(endpoint.config)?;
    handle.claim_interface(endpoint.iface)?;
    handle.set_alternate_setting(endpoint.iface, endpoint.setting)?;
    Ok(())
}
