use super::{ImuAxisOrder, ImuBus, imu_device_config::ImuDeviceConfig};

use vqm::Vector3f32;

/// Accelerometer scale factor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AccFullScale {
    #[default]
    Max = 0,
    Scale1G = 1,
    Scale2G = 2,
    Scale4G = 3,
    Scale8G = 4,
    Scale16G = 5,
    Scale32G = 6,
}

/// Units for acceleration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AccUnits {
    #[default]
    /// Acceleration in gravity units.
    G,
    /// Acceleration in meters per second squared.
    Mps2,
}

/// Gyro scale factor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GyroFullScale {
    #[default]
    Max = 0,
    Scale125Dps = 1,
    Scale250Dps = 2,
    Scale500Dps = 3,
    Scale1000Dps = 4,
    Scale2000Dps = 5,
    Scale4000Dps = 6,
}

/// Units for gyro rotation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GyroUnits {
    #[default]
    /// Degrees per second.
    Dps,
    /// Radians per second.
    Rps,
}

// Shared data members
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuCommon {
    pub acc_offset: Vector3f32,
    pub acc_scale: f32,

    pub gyro_offset: Vector3f32,
    pub gyro_scale: f32,

    pub acc_sample_rate_hz: u32,
    pub gyro_sample_rate_hz: u32,

    pub axis_order: ImuAxisOrder,
}

#[allow(missing_docs)]
impl ImuCommon {
    pub const G0: f32 = 9.806_65;

    #[must_use]
    pub const fn new(axis_order: ImuAxisOrder) -> Self {
        const GYRO_2000DPS_RES: f32 = 2000.0 / 32768.0;
        const ACC_8G_RES: f32 = 8.0 / 32768.0;
        Self {
            acc_offset: Vector3f32::new(0.0, 0.0, 0.0),
            acc_scale: ACC_8G_RES,
            gyro_offset: Vector3f32::new(0.0, 0.0, 0.0),
            gyro_scale: GYRO_2000DPS_RES,
            gyro_sample_rate_hz: 1000,
            acc_sample_rate_hz: 1000,
            axis_order,
        }
    }
}

impl Default for ImuCommon {
    fn default() -> Self {
        Self::new(ImuAxisOrder::XPOS_YPOS_ZPOS)
    }
}

// Imu trait uses Bus as an associated type.
#[allow(async_fn_in_trait)]
pub trait Imu {
    type Bus: ImuBus;
    // This forces the IMU error to be the same as the Bus error
    /// The global error type for this IMU, required to be printable and capable of wrapping raw bus transaction failures.
    type Error: core::fmt::Debug + From<<Self::Bus as ImuBus>::Error>;

    const TARGET_OUTPUT_DATA_RATE_MAX: u8 = 0;

    fn bus(&mut self) -> &mut Self::Bus;
    fn common(&self) -> &ImuCommon;
    fn common_mut(&mut self) -> &mut ImuCommon;
    fn config(&self) -> &ImuDeviceConfig;

    //async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
    /// Passes raw payloads straight to the bus. Provided as a default wrapper.
    #[inline]
    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let address = self.config().address;
        self.bus().bus_write_read(address, write, read).await.map_err(Self::Error::from)
    }

    async fn read_acc(&mut self) -> Result<Vector3f32, Self::Error>;

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error>;

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error>;

    #[inline]
    #[must_use]
    fn acc_scale(&self) -> f32 {
        self.common().acc_scale
    }

    #[inline]
    #[must_use]
    fn acc_offset(&self) -> Vector3f32 {
        self.common().acc_offset
    }

    #[inline]
    fn set_acc_offset(&mut self, acc_offset: Vector3f32) {
        self.common_mut().acc_offset = acc_offset;
    }

    #[inline]
    #[must_use]
    fn acc_offset_mapped(&self) -> Vector3f32 {
        self.common().axis_order.map_vector(self.common().acc_offset)
    }

    #[inline]
    fn set_acc_offset_mapped(&mut self, acc_offset: Vector3f32) {
        let acc_offset_mapped = self.common().axis_order.axis_order_inverse().map_vector(acc_offset);
        self.set_acc_offset(acc_offset_mapped);
    }

    #[inline]
    #[must_use]
    fn gyro_scale(&self) -> f32 {
        self.common().gyro_scale
    }

    #[inline]
    #[must_use]
    fn gyro_offset(&self) -> Vector3f32 {
        self.common().gyro_offset
    }

    #[inline]
    fn set_gyro_offset(&mut self, gyro_offset: Vector3f32) {
        self.common_mut().gyro_offset = gyro_offset;
    }

    #[inline]
    #[must_use]
    fn gyro_offset_mapped(&self) -> Vector3f32 {
        self.common().axis_order.map_vector(self.common().gyro_offset)
    }

    #[inline]
    fn set_gyro_offset_mapped(&mut self, gyro_offset: Vector3f32) {
        let gyro_offset_mapped = self.common().axis_order.axis_order_inverse().map_vector(gyro_offset);
        self.set_gyro_offset(gyro_offset_mapped);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<ImuCommon>();
    }
}
