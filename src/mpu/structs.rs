use core::ops::{AddAssign, DivAssign, MulAssign, SubAssign};

use fixed::{types, FixedI32};

/// A quaternion is a mathematical way of representing angles.
/// These are not very intuitive, but this is what the hardware returns.
/// You should convert these to `YawPitchRoll` before doing further logic on them.
///
/// Warning: This struct uses a `FixedI32` with 30 fractional bits. You may want to convert these to a more useful format.
#[derive(Copy, Clone, Debug)]
#[allow(missing_docs)]
pub struct Quaternion {
    pub w: FixedI32<types::extra::U30>,
    pub x: FixedI32<types::extra::U30>,
    pub y: FixedI32<types::extra::U30>,
    pub z: FixedI32<types::extra::U30>,
}

impl Quaternion {
    pub(crate) fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 16);

        let w =
            FixedI32::<types::extra::U30>::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let x =
            FixedI32::<types::extra::U30>::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let y = FixedI32::<types::extra::U30>::from_be_bytes([
            bytes[8], bytes[9], bytes[10], bytes[11],
        ]);
        let z = FixedI32::<types::extra::U30>::from_be_bytes([
            bytes[12], bytes[13], bytes[14], bytes[15],
        ]);
        Quaternion { w, x, y, z }
    }
}

/// The accelerometer values.
/// They are in the range of [-2G, 2G].
pub type Accel = RawSensor;

/// The gyroscope values.
/// They are in the range of [-2000 deg/second, 2000 deg/second].
pub type Gyro = RawSensor;

#[derive(Copy, Clone, Debug, Default)]
#[allow(missing_docs)]
pub struct RawSensor {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl RawSensor {
    pub(crate) fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
    }

    pub(crate) fn to_bytes(self) -> [u8; 6] {
        let [b0, b1] = i16::to_be_bytes(self.x);
        let [b2, b3] = i16::to_be_bytes(self.y);
        let [b4, b5] = i16::to_be_bytes(self.z);
        [b0, b1, b2, b3, b4, b5]
    }
}

impl Into<[i16; 3]> for RawSensor {
    fn into(self) -> [i16; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Default)]
pub(crate) struct RawSensorOffset {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

impl From<RawSensor> for RawSensorOffset {
    fn from(value: RawSensor) -> Self {
        Self {
            x: value.x.into(),
            y: value.y.into(),
            z: value.z.into(),
        }
    }
}

fn wrapping_cast(value: i64) -> i16 {
    let shift = (if value > 0 { i16::MIN } else { i16::MAX }) as i64;
    (((value - shift) % 2_i64.pow(16)) + shift) as i16
}

impl Into<RawSensor> for RawSensorOffset {
    fn into(self) -> RawSensor {
        RawSensor {
            x: wrapping_cast(self.x),
            y: wrapping_cast(self.y),
            z: wrapping_cast(self.z),
        }
    }
}

impl AddAssign for RawSensorOffset {
    fn add_assign(&mut self, rhs: Self) {
        self.x = self.x.saturating_add(rhs.x);
        self.y = self.y.saturating_add(rhs.y);
        self.z = self.z.saturating_add(rhs.z);
    }
}

impl SubAssign for RawSensorOffset {
    fn sub_assign(&mut self, rhs: Self) {
        self.x = self.x.saturating_sub(rhs.x);
        self.y = self.y.saturating_sub(rhs.y);
        self.z = self.z.saturating_sub(rhs.z);
    }
}

impl MulAssign<i64> for RawSensorOffset {
    fn mul_assign(&mut self, rhs: i64) {
        self.x = self.x.saturating_mul(rhs);
        self.y = self.y.saturating_mul(rhs);
        self.z = self.z.saturating_mul(rhs);
    }
}

impl DivAssign<i64> for RawSensorOffset {
    fn div_assign(&mut self, rhs: i64) {
        self.x = self.x.saturating_div(rhs);
        self.y = self.y.saturating_div(rhs);
        self.z = self.z.saturating_div(rhs);
    }
}
