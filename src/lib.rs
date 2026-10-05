#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
//#![deny(missing_docs)]
#![deny(
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unused_must_use,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]

#[cfg(all(feature = "i2c", feature = "spi"))]
compile_error!("Features 'i2c' and 'spi' are mutually exclusive and cannot be enabled together.");

mod axis;
mod bmi270;
mod i2c;
mod icm20602;
mod imu;
mod imu426xx;
mod imu_bus;
mod imu_bus_i2c;
mod imu_bus_spi;
mod imu_device;
mod imu_device_config;
mod imu_mock;
mod lsm6ds;
mod mpu6050;
mod mpu6886;
mod qmi8658a;
mod spi;

pub use axis::ImuAxisOrder;
pub use bmi270::Bmi270;
pub use icm20602::Icm20602;
pub use imu::{AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuCommon};
pub use imu_bus::{ImuBus, MockImuBus, SetupError};
pub use imu_bus_i2c::ImuI2cBus;
pub use imu_bus_spi::ImuSpiBus;
pub use imu_device::{ImuDevice, ImuError};
pub use imu_device_config::ImuDeviceConfig;
pub use imu_mock::ImuMock;
pub use imu426xx::Imu426xx;
pub use lsm6ds::Lsm6ds;
pub use mpu6050::Mpu6050;
pub use mpu6886::Mpu6886;
pub use qmi8658a::Qmi8658a;
