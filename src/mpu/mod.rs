use crate::mpu::config::DigitalLowPassFilter;
use crate::mpu::sensor::Mpu6050;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::time::delay_ms_assembly;
use crate::twi::{TwiWrapper, TWI};
use error::Error;
use nb::Error::WouldBlock;
use structs::{Accel, Gyro, Quaternion};

use self::config::Fifo;
use self::structs::RawSensorOffset;

#[allow(unused)]
mod config;
mod dmp_firmware;
mod firmware_loader;
#[allow(unused)]
mod registers;
#[allow(unused)]
mod sensor;
/// structs to deal with mpu output, like quaternions
pub mod structs;

mod error;

/// MPU Sample Rate Divider under DMP mode
pub const SAMPLE_RATE_DIVIDER_MPU: u8 = 0;
/// MPU Sample Rate Divider under RAW mode
pub const SAMPLE_RATE_DIVIDER_RAW: u8 = 0;

type I2c = TwiWrapper;

struct Mpu {
    mpu: Mpu6050,
    dmp_enabled: bool,
}

static MPU: Mutex<OnceCell<Mpu>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    // Safety: The TWI mutex is not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    let mut mpu = Mpu6050::new(twi);

    mpu.initialize_dmp(twi);

    mpu.set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU);
    mpu.set_digital_lowpass_filter(twi, DigitalLowPassFilter::Filter5);
    MPU.modify(|m| {
        m.initialize(Mpu {
            mpu,
            dmp_enabled: true,
        })
    });
}

/// Is the DMP (digital motion processor) of the MPU enabled?
/// It is enabled by default.
pub fn is_dmp_enabled() -> bool {
    MPU.modify(|mpu| mpu.dmp_enabled)
}

/// Disable the DMP (digital motion processor) of the MPU
///
/// # Panics
/// when the global constant `SAMPLE_RATE_DIVIDER_RAW` is wrong (i.e. won't panic under normal conditions)
pub fn disable_dmp() {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_RAW);
    mpu.mpu.disable_dmp(twi);
    mpu.mpu.disable_fifo(twi);
    mpu.dmp_enabled = false;
}

/// Enable the DMP (digital motion processor) of the MPU
///
/// # Errors
/// when the global constant `SAMPLE_RATE_DIVIDER_MPU` is wrong (i.e. will not panic under normal conditions)
pub fn enable_dmp() {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU);
    mpu.mpu.enable_dmp(twi);
    mpu.mpu.enable_fifo(twi);
    mpu.dmp_enabled = true;
}

/// This reads the most recent angle from the DMP, if there are any new ones available.
/// If there is no new angle available, it returns `WouldBlock`.
/// Do not call this function if the DMP is disabled.
///
/// # Panics
/// When the dmp is disabled.
///
/// # Errors
/// when a TWI(I2C) operation failed
pub fn read_dmp_bytes() -> nb::Result<Quaternion, ()> {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    assert!(mpu.dmp_enabled);

    // If there isn't a full packet ready, return none
    let mut len = mpu.mpu.get_fifo_count(twi);
    if len < 28 {
        return Err(WouldBlock);
    }

    // If we got mis-aligned, we skip a packet
    if len % 28 != 0 {
        let skip = len % 28;
        let mut buf = [0; 28];

        let _ = mpu.mpu.read_fifo(twi, &mut buf[..skip]);
        return Err(WouldBlock);
    }

    // Keep reading while there are more full packets
    let mut buf = [0; 28];
    while len >= 28 {
        let _ = mpu.mpu.read_fifo(twi, &mut buf);
        len -= 28;
    }

    // Convert the last full packet we received to a Quaternion
    Ok(Quaternion::from_bytes(&buf[..16]))
}

/// This reads the most recent acceleration and gyroscope information from the MPU.
/// This function can be called both if the DMP is enabled or disabled.
///
/// # Errors
/// when a TWI operation failed
pub fn read_raw() -> Result<(Accel, Gyro), Error> {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    let accel = mpu.mpu.accel(twi);
    let gyro = mpu.mpu.gyro(twi);

    Ok((accel, gyro))
}

/// Calibrate the MPU and write offsets to Hardware Offset Registers
pub fn calibrate(duration_ms: u32) {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    // reset DMP
    let dmp_enabled = is_dmp_enabled();
    if dmp_enabled {
        disable_dmp();
    }

    for _ in 0..(duration_ms / 50) {
        // configure FIFO
        mpu.mpu.enable_fifo(twi);
        mpu.mpu.set_fifo_enabled(twi, {
            let mut fifo = Fifo::all_disabled();
            fifo.xg = true;
            fifo.yg = true;
            fifo.zg = true;
            fifo.accel = true;
            fifo
        });

        // fill FIFO
        mpu.mpu.reset_fifo(twi);
        delay_ms_assembly(50);
        mpu.mpu.set_fifo_enabled(twi, Fifo::all_disabled());

        let mut buf = [0; 12];
        let fifo_count = mpu.mpu.get_fifo_count(twi);
        let packet_count = fifo_count / buf.len();

        let mut offset_accel = RawSensorOffset::default();
        let mut offset_gyro = RawSensorOffset::default();

        for _ in 0..packet_count {
            let bytes = mpu.mpu.read_fifo(twi, &mut buf);

            let mut accel_bytes = [0; 6];
            accel_bytes.copy_from_slice(&bytes[0..6]);
            offset_accel -= Accel::from_bytes(accel_bytes).into();

            let mut gyro_bytes = [0; 6];
            gyro_bytes.copy_from_slice(&bytes[6..12]);
            offset_gyro -= Gyro::from_bytes(gyro_bytes).into();
        }

        // compute averages
        offset_accel /= packet_count as i64;
        offset_gyro /= packet_count as i64;

        // scale from +-2G to +-16G and from +-2000dps to +-1000dps sensitivity ranges
        offset_accel /= 8;
        offset_gyro *= 2;

        // subtract gravity 1G (in +-16G range) from z axis
        if offset_accel.z > 0 {
            offset_accel.z -= 2048;
        } else {
            offset_accel.z += 2048;
        }

        // preserve bit 0 of factory value (for temperature compensation)
        offset_accel.x &= !1;
        offset_accel.y &= !1;
        offset_accel.z &= !1;

        // add existing / factory offsets
        offset_accel += mpu.mpu.get_offset_accel(twi).into();
        offset_gyro += mpu.mpu.get_offset_gyro(twi).into();

        // write offsets to registers
        mpu.mpu.set_offset_accel(twi, offset_accel.into());
        mpu.mpu.set_offset_gyro(twi, offset_gyro.into());
    }

    mpu.mpu.disable_fifo(twi);
    if dmp_enabled {
        enable_dmp();
    }
}
