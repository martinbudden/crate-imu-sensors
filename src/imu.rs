use super::{AxisOrder, ImuBus, ImuDeviceConfig, ImuError};

use embassy_time::{Duration, Instant, Timer};
use vqm::Vector3f32;

use cfg_if::cfg_if;

/// Accelerometer scale factor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AccSensitivity {
    #[default]
    Max = 0,
    Scale1G = 1,
    Scale2G = 2,
    Scale4G = 3,
    Scale8G = 4,
    Scale16G = 5,
    Scale32G = 6,
    Scale64G = 7,
    Scale128G = 8,
    Scale200G = 9,
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
pub enum GyroSensitivity {
    #[default]
    Max = 0,
    Scale15p625Dps = 1,
    Scale31p25Dps = 2,
    Scale62p5Dps = 3,
    Scale125Dps = 4,
    Scale250Dps = 5,
    Scale500Dps = 6,
    Scale1000Dps = 7,
    Scale2000Dps = 8,
    Scale4000Dps = 9,
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

    axis_order: AxisOrder,
}

#[allow(missing_docs)]
impl ImuCommon {
    pub const G0: f32 = 9.806_65;

    #[must_use]
    pub const fn new(axis_order: AxisOrder) -> Self {
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

    #[inline]
    pub async fn delay_ms(delay: u32) {
        Timer::after(Duration::from_millis(delay.into())).await;
    }

    #[inline]
    #[must_use]
    pub fn axis_order(self) -> AxisOrder {
        self.axis_order
    }

    #[inline]
    #[must_use]
    pub fn map_vector(self, v: Vector3f32) -> Vector3f32 {
        cfg_if! {
        if #[cfg(feature = "axis_xpos_ypos_zpos")] {
            v
        } else if #[cfg(feature = "axis_yneg_xpos_zpos")] {
            Vector3f32::new(-v.y,   v.x,   v.z)
        } else if #[cfg(feature = "axis_ypos_xneg_zpos")] {
            Vector3f32::new( v.y,  -v.x,   v.z)
        } else if #[cfg(feature = "axis_xneg_yneg_zpos")] {
            Vector3f32::new(-v.x,  -v.y,   v.z)
        } else if #[cfg(feature = "axis_xpos_zpos_yneg")] {
            Vector3f32::new( v.x,   v.z,  -v.y )
        } else {
        self.axis_order.map_vector(v)
        }}
    }

    #[inline]
    #[must_use]
    pub fn map_acc_gyro(self, acc: Vector3f32, gyro: Vector3f32) -> (Vector3f32, Vector3f32) {
        // use a feature flag to hardcode the mapping, so that the match statement can be bypassed for optimal performance.
        cfg_if! {
        if #[cfg(feature = "axis_xpos_ypos_zpos")] {
            (acc, gyro)
        } else if #[cfg(feature = "axis_yneg_xpos_zpos")] {
            (Vector3f32::new(-acc.y,  acc.x,  acc.z),
             Vector3f32::new(-gyro.y, gyro.x, gyro.z))
        } else if #[cfg(feature = "axis_ypos_xneg_zpos")] {
            (Vector3f32::new(acc.y,  -acc.x,  acc.z),
             Vector3f32::new(gyro.y, -gyro.x, gyro.z))
        } else if #[cfg(feature = "axis_xneg_yneg_zpos")] {
            (Vector3f32::new(-acc.x,  -acc.y,  acc.z),
             Vector3f32::new(-gyro.x, -gyro.y, gyro.z))
        } else if #[cfg(feature = "axis_xpos_zpos_yneg")] {
            (Vector3f32::new(acc.x,  acc.z,  -acc.y ),
             Vector3f32::new(gyro.x, gyro.z, -gyro.y ))
        } else {
            self.axis_order.map_acc_gyro(acc, gyro)
        }}
    }
}

impl Default for ImuCommon {
    fn default() -> Self {
        Self::new(AxisOrder::XPOS_YPOS_ZPOS)
    }
}

// Imu trait uses Bus as an associated type.
#[allow(async_fn_in_trait)]
pub trait Imu {
    type Bus: ImuBus;

    const TARGET_OUTPUT_DATA_RATE_MAX: u8 = 0;

    fn bus(&mut self) -> &mut Self::Bus;
    fn common(&self) -> &ImuCommon;
    fn common_mut(&mut self) -> &mut ImuCommon;
    fn config(&self) -> &ImuDeviceConfig;

    //async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), ImuError>;
    /// Passes raw payloads straight to the bus. Provided as a default wrapper.
    #[inline]
    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        let address = self.config().address;
        self.bus().bus_write_read(address, write, read).await
    }

    async fn read_acc(&mut self) -> Result<(Vector3f32, Instant), ImuError>;

    async fn read_gyro(&mut self) -> Result<(Vector3f32, Instant), ImuError>;

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32, Instant), ImuError>;

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
