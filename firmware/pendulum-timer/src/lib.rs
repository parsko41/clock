#![no_std]
//! Pendulum timer firmware library: the hardware-free parts live here so they
//! can be unit-tested on a PC (see ../host-tests).

extern crate alloc;

pub mod creds;
pub mod mdns;
pub mod timing;
