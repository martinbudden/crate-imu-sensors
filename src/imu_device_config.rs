use super::ImuAxisOrder;

#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

// Imu configuration, set on construction and read-only thereafter
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct ImuDeviceConfig {
    pub gyro_id_msp: u16,
    pub acc_id_msp: u16,
    /// 8-bit id assigned by IMU manufacturer.
    pub device_id: u8,
    pub address: u8,
    pub axis_order: ImuAxisOrder,
    /// Flags for describing IMU characteristics.
    pub flags: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for ImuDeviceConfig {}

impl Default for ImuDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(missing_docs)]
impl ImuDeviceConfig {
    // Betaflight compatible acc and gyro ids
    // Used for reporting gyro and acc type back to MSP (MultiWii Serial Protocol)
    pub const MSP_ACC_ID_DEFAULT: u16 = 0;
    pub const MSP_ACC_ID_NONE: u16 = 1;
    pub const MSP_ACC_ID_MPU6050: u16 = 2;
    pub const MSP_ACC_ID_MPU6000: u16 = 3;
    pub const MSP_ACC_ID_MPU6500: u16 = 4;
    pub const MSP_ACC_ID_MPU9250: u16 = 5;
    pub const MSP_ACC_ID_ICM20601: u16 = 6;
    pub const MSP_ACC_ID_ICM20602: u16 = 7;
    pub const MSP_ACC_ID_ICM20608G: u16 = 8;
    pub const MSP_ACC_ID_ICM20649: u16 = 9;
    pub const MSP_ACC_ID_ICM20689: u16 = 10;
    pub const MSP_ACC_ID_ICM42605: u16 = 11;
    pub const MSP_ACC_ID_ICM42688P: u16 = 12;
    pub const MSP_ACC_ID_BMI160: u16 = 13;
    pub const MSP_ACC_ID_BMI270: u16 = 14;
    pub const MSP_ACC_ID_LSM6DSO: u16 = 15;
    pub const MSP_ACC_ID_LSM6DSV16X: u16 = 16;
    pub const MSP_ACC_ID_IIM42653: u16 = 17;
    pub const MSP_ACC_ID_ICM45605: u16 = 18;
    pub const MSP_ACC_ID_ICM45686: u16 = 19;
    pub const MSP_ACC_ID_ICM40609D: u16 = 20;
    pub const MSP_ACC_ID_IIM42652: u16 = 21;
    pub const MSP_ACC_ID_LSM6DSK320X: u16 = 22;
    pub const MSP_ACC_ID_ICM42622P: u16 = 23;
    pub const MSP_ACC_ID_ICM42686P: u16 = 24;
    pub const MSP_ACC_ID_VIRTUAL: u16 = 25;

    pub const MSP_GYRO_ID_NONE: u16 = 0;
    pub const MSP_GYRO_ID_DEFAULT: u16 = 1;

    pub const MSP_GYRO_ID_MPU6050: u16 = 2;
    pub const MSP_GYRO_ID_L3GD20: u16 = 3;
    pub const MSP_GYRO_ID_MPU6000: u16 = 4;
    pub const MSP_GYRO_ID_MPU6500: u16 = 5;
    pub const MSP_GYRO_ID_MPU9250: u16 = 6;
    pub const MSP_GYRO_ID_ICM20601: u16 = 7;
    pub const MSP_GYRO_ID_ICM20602: u16 = 8;
    pub const MSP_GYRO_ID_ICM20608G: u16 = 9;
    pub const MSP_GYRO_ID_ICM20649: u16 = 10;
    pub const MSP_GYRO_ID_ICM20689: u16 = 11;
    pub const MSP_GYRO_ID_ICM42605: u16 = 12;
    pub const MSP_GYRO_ID_ICM42688P: u16 = 13;
    pub const MSP_GYRO_ID_BMI160: u16 = 14;
    pub const MSP_GYRO_ID_BMI270: u16 = 15;
    pub const MSP_GYRO_ID_LSM6DSO: u16 = 16;
    pub const MSP_GYRO_ID_LSM6DSV16X: u16 = 17;
    pub const MSP_GYRO_ID_IIM42653: u16 = 18;
    pub const MSP_GYRO_ID_ICM45605: u16 = 19;
    pub const MSP_GYRO_ID_ICM45686: u16 = 20;
    pub const MSP_GYRO_ID_ICM40609D: u16 = 21;
    pub const MSP_GYRO_ID_IIM42652: u16 = 22;
    pub const MSP_GYRO_ID_LSM6DSK320X: u16 = 23;
    pub const MSP_GYRO_ID_ICM42622P: u16 = 24;
    pub const MSP_GYRO_ID_ICM42686P: u16 = 25;
    pub const MSP_GYRO_ID_VIRTUAL: u16 = 26;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            gyro_id_msp: 0,
            acc_id_msp: 0,
            device_id: 0,
            address: 0,
            axis_order: ImuAxisOrder::XPOS_YPOS_ZPOS,
            flags: 0,
        }
    }
    #[must_use]
    pub const fn from_axis_order(axis_order: ImuAxisOrder) -> Self {
        Self { gyro_id_msp: 0, acc_id_msp: 0, device_id: 0, address: 0, axis_order, flags: 0 }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<ImuDeviceConfig>();
        #[cfg(feature = "serde")]
        is_serde::<ImuDeviceConfig>();
        #[cfg(feature = "storage")]
        is_storage::<ImuDeviceConfig>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        let config = ImuDeviceConfig::new();
        assert_eq!(0, config.address);
    }
}
