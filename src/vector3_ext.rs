#[cfg(feature = "nalgebra")]
use nalgebra::Vector3;
#[cfg(feature = "vqm")]
use vqm::Vector3;

pub trait Vector3Ext {
    fn from_le_bytes_6(buf: [u8; 6]) -> Self;
    fn from_be_bytes_6(buf: [u8; 6]) -> Self;
}

impl Vector3Ext for Vector3<f32> {
    /// Creates a `Vector3<f32>` from a 6-byte little-endian byte array.
    /// ```
    /// #[cfg(feature = "nalgebra")]
    /// # use nalgebra::Vector3;
    /// #[cfg(feature = "vqm")]
    /// # use vqm::Vector3;
    /// # use imu_sensors::Vector3Ext;
    /// let bytes = [0x01, 0x00, 0x00, 0x01, 0x2A, 0x00];
    /// let v = Vector3::<f32>::from_le_bytes_6(bytes);
    /// assert_eq!(Vector3::<f32>::new(1.0, 256.0, 42.0), v);
    /// ```
    fn from_le_bytes_6(buf: [u8; 6]) -> Self {
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Self::new(f32::from(x), f32::from(y), f32::from(z))
    }

    /// Creates a `Vector3<f32>` from a 6-byte big-endian byte array.
    /// ```
    /// #[cfg(feature = "nalgebra")]
    /// # use nalgebra::Vector3;
    /// #[cfg(feature = "vqm")]
    /// # use vqm::Vector3;
    /// # use imu_sensors::Vector3Ext;
    /// let bytes = [0x00, 0x01, 0x01, 0x00, 0x00, 0x2A];
    /// let v = Vector3::<f32>::from_be_bytes_6(bytes);
    /// assert_eq!(Vector3::<f32>::new(1.0, 256.0, 42.0), v);
    /// ```
    fn from_be_bytes_6(buf: [u8; 6]) -> Self {
        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Self::new(f32::from(x), f32::from(y), f32::from(z))
    }
}
