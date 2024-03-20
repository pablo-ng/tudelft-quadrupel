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
