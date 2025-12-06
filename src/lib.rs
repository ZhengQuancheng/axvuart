#![no_std]
#![feature(trait_alias)]

extern crate alloc;

#[macro_use]
extern crate log;

mod regs;
mod pl011;
mod ops_impl;

pub use pl011::{IrqCallback, VirtPL011, VirtPL011Config};
pub use regs::*;
pub use axdevice_base::{BaseDeviceOps, BaseMmioDeviceOps, EmuDeviceType};
