//! Implementation of BaseDeviceOps trait for VirtPL011

use axaddrspace::device::{AccessWidth, DeviceAddrRange};
use axaddrspace::GuestPhysAddrRange;
use axdevice_base::{BaseDeviceOps, EmuDeviceType};
use axerrno::AxResult;

use crate::pl011::VirtPL011;

impl BaseDeviceOps<GuestPhysAddrRange> for VirtPL011 {
    fn emu_type(&self) -> EmuDeviceType {
        EmuDeviceType::Console
    }

    fn address_range(&self) -> GuestPhysAddrRange {
        GuestPhysAddrRange::from_start_size(self.base_gpa().into(), self.length())
    }

    fn handle_read(
        &self,
        addr: <GuestPhysAddrRange as DeviceAddrRange>::Addr,
        width: AccessWidth,
    ) -> AxResult<usize> {
        let offset = addr.as_usize() - self.base_gpa();

        if offset >= self.length() {
            return Ok(0);
        }

        let reg_offset = offset & !0x3;
        let value = self.read_reg(reg_offset);

        let result = match width {
            AccessWidth::Byte => {
                let byte_offset = offset & 0x3;
                ((value >> (byte_offset * 8)) & 0xFF) as usize
            }
            AccessWidth::Word => {
                let half_offset = (offset >> 1) & 0x1;
                ((value >> (half_offset * 16)) & 0xFFFF) as usize
            }
            AccessWidth::Dword => value as usize,
            AccessWidth::Qword => value as usize,
        };

        Ok(result)
    }

    fn handle_write(
        &self,
        addr: <GuestPhysAddrRange as DeviceAddrRange>::Addr,
        width: AccessWidth,
        val: usize,
    ) -> AxResult {
        let offset = addr.as_usize() - self.base_gpa();

        if offset >= self.length() {
            return Ok(());
        }

        let reg_offset = offset & !0x3;

        // For UART_DR writes
        if reg_offset == crate::regs::UART_DR {
            let tx_byte = val as u8;
            self.write_reg(reg_offset, tx_byte as u32);
            // Direct output to console
            output_byte(tx_byte);
        } else {
            // For other registers
            let write_val = match width {
                AccessWidth::Byte => val as u32,
                AccessWidth::Word => val as u32,
                AccessWidth::Dword => val as u32,
                AccessWidth::Qword => val as u32, // Lower 32 bits
            };
            self.write_reg(reg_offset, write_val);
        }

        Ok(())
    }
}

/// Line buffer for console output
static LINE_BUFFER: spin::Mutex<alloc::string::String> =
    spin::Mutex::new(alloc::string::String::new());

/// Output a byte to console
fn output_byte(byte: u8) {
    let mut buffer = LINE_BUFFER.lock();

    match byte {
        b'\n' => {
            info!("[Guest] {}", buffer);
            buffer.clear();
        }
        b'\r' => {
            // Ignore carriage returns
        }
        _ if byte.is_ascii() && !byte.is_ascii_control() => {
            buffer.push(byte as char);
            if buffer.len() >= 256 {
                info!("[Guest] {}", buffer);
                buffer.clear();
            }
        }
        _ => { }
    }
}
