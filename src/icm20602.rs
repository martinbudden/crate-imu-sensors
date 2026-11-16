use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{
    AccSensitivity, AccUnits, GyroSensitivity, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice,
    ImuDeviceConfig, ImuError,
};

const I2C_ADDRESS: u8 = 0x68;
const I2C_ADDRESS_ALTERNATIVE: u8 = 0x69;

/// IMU Registers.
struct Reg;

impl Reg {
    const _XG_OFFS_TC_H: u8 = 0x04;
    const _XG_OFFS_TC_L: u8 = 0x05;
    const _YG_OFFS_TC_H: u8 = 0x07;
    const _YG_OFFS_TC_L: u8 = 0x08;
    const _ZG_OFFS_TC_H: u8 = 0x0A;
    const _ZG_OFFS_TC_L: u8 = 0x0B;

    const _SELF_TEST_X_ACCEL: u8 = 0x0D;
    const _SELF_TEST_Y_ACCEL: u8 = 0x0E;
    const _SELF_TEST_Z_ACCEL: u8 = 0x0F;

    //  GYRO OFFSET ADJUSTMENT REGISTERS
    const _XG_OFFS_USRH: u8 = 0x13;
    const _XG_OFFS_USRL: u8 = 0x14;
    const _YG_OFFS_USRH: u8 = 0x15;
    const _YG_OFFS_USRL: u8 = 0x16;
    const _ZG_OFFS_USRH: u8 = 0x17;
    const _ZG_OFFS_USRL: u8 = 0x18;

    const SMPLRT_DIV: u8 = 0x19;
    const CONFIG: u8 = 0x1A;
    const GYRO_CONFIG: u8 = 0x1B;
    const ACCEL_CONFIG: u8 = 0x1C;
    const ACCEL_CONFIG2: u8 = 0x1D;

    const FIFO_ENABLE: u8 = 0x23;
    const _GYRO_FIFO_EN: u8 = 0b_0000_1000;
    const _ACC_FIFO_EN: u8 = 0b_0000_0100;

    const INT_PIN_CFG: u8 = 0x37;
    const INT_ENABLE: u8 = 0x38;
    const _FIFO_WM_INT_STATUS: u8 = 0x39; // FIFO watermark interrupt status

    pub const ACCEL_XOUT_H: u8 = 0x3B;
    const _ACCEL_XOUT_L: u8 = 0x3C;
    const _ACCEL_YOUT_H: u8 = 0x3D;
    const _ACCEL_YOUT_L: u8 = 0x3E;
    const _ACCEL_ZOUT_H: u8 = 0x3F;
    const _ACCEL_ZOUT_L: u8 = 0x40;

    const _TEMP_OUT_H: u8 = 0x41;
    const _TEMP_OUT_L: u8 = 0x42;

    const GYRO_XOUT_H: u8 = 0x43;
    const _GYRO_XOUT_L: u8 = 0x44;
    const _GYRO_YOUT_H: u8 = 0x45;
    const _GYRO_YOUT_L: u8 = 0x46;
    const _GYRO_ZOUT_H: u8 = 0x47;
    const _GYRO_ZOUT_L: u8 = 0x48;

    const _FIFO_WM_TH1: u8 = 0x60; // FIFO watermark threshold in number of bytes
    const _FIFO_WM_TH2: u8 = 0x61;

    const _SIGNAL_PATH_RESET: u8 = 0x68;
    const _ACCEL_INTEL_CTRL: u8 = 0x69;
    const USER_CTRL: u8 = 0x6A;
    const PWR_MGMT_1: u8 = 0x6B;
    const _PWR_MGMT_2: u8 = 0x6C;

    const _FIFO_COUNT_H: u8 = 0x72;
    const _FIFO_COUNT_L: u8 = 0x73;
    const _FIFO_R_W: u8 = 0x74;

    const WHO_AM_I: u8 = 0x75;

    // ACCELEROMETER OFFSET REGISTERS
    const _XA_OFFSET_H: u8 = 0x77;
    const _XA_OFFSET_L: u8 = 0x78;
    const _YA_OFFSET_H: u8 = 0x7A;
    const _YA_OFFSET_L: u8 = 0x7B;
    const _ZA_OFFSET_H: u8 = 0x7D;
    const _ZA_OFFSET_L: u8 = 0x7E;
}

const CONFIG_DLPF_0: u8 = 0;
const CONFIG_DLPF_176HZ: u8 = 0x01;

#[derive(Debug, PartialEq)]
pub struct Icm20602<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for Icm20602<B> {
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), ImuError> {
        Icm20602::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), ImuError> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for Icm20602<B> {
    type Bus = B;

    #[inline]
    fn bus(&mut self) -> &mut Self::Bus {
        &mut self.bus
    }

    #[inline]
    fn common(&self) -> &ImuCommon {
        &self.common
    }

    #[inline]
    fn common_mut(&mut self) -> &mut ImuCommon {
        &mut self.common
    }

    #[inline]
    fn config(&self) -> &ImuDeviceConfig {
        &self.config
    }

    async fn read_acc(&mut self) -> Result<Vector3f32, ImuError> {
        let mut buf = [0u8; 6];
        self.write_read(&[Reg::ACCEL_XOUT_H], &mut buf).await?;
        let acc = Vector3f32::from_be_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, ImuError> {
        let mut buf = [0u8; 6];
        self.write_read(&[Reg::GYRO_XOUT_H], &mut buf).await?;
        let gyro = Vector3f32::from_be_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), ImuError> {
        let mut buf = [0u8; 14];
        self.write_read(&[Reg::ACCEL_XOUT_H], &mut buf).await?;

        let [a0, a1, a2, a3, a4, a5, _t0, _t1, g0, g1, g2, g3, g4, g5] = buf;

        let acc_buf = [a0, a1, a2, a3, a4, a5];
        let gyro_buf = [g0, g1, g2, g3, g4, g5];
        let acc = Vector3f32::from_be_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_be_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_acc_gyro(self.common.axis_order, acc, gyro))
    }
}

async fn delay_ms(delay: u32) {
    Timer::after(Duration::from_millis(delay.into())).await;
}

impl<B: ImuBus> Icm20602<B> {
    const DEVICE_ID: u8 = 0x12;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_GYRO_ID_ICM20602,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_ICM20602,
                axis_order,
                device_id: Self::DEVICE_ID,
                address: I2C_ADDRESS,
                flags: 0,
            },
        }
    }

    /// Set a newly constructed `Icm20602` to use it's alternative I2C address.
    #[must_use]
    pub fn with_alternative_address(mut self) -> Self {
        self.config.address = I2C_ADDRESS_ALTERNATIVE;
        self
    }

    /// # Errors
    pub async fn write_register(&mut self, reg: u8, data: u8) -> Result<(), ImuError> {
        self.bus.write_register(self.config.address, reg, data).await
    }

    /// # Errors
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, ImuError> {
        self.bus.read_register(self.config.address, reg).await
    }

    /// # Errors
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), ImuError> {
        const DEVICE_RESET: u8 = 0x01 << 7;
        const CLKSEL_1: u8 = 0x01;
        const DATA_RDY_INT_EN: u8 = 0x01;

        // reset the device
        self.write_register(Reg::PWR_MGMT_1, DEVICE_RESET).await?;
        delay_ms(10).await;

        // CLKSEL must be set to 001 to achieve full gyroscope performance.
        // This sets PLL clock and wakes device.
        self.write_register(Reg::PWR_MGMT_1, CLKSEL_1).await?;
        delay_ms(100).await;

        // Check WhoAmI
        let chip_id = self.read_register(Reg::WHO_AM_I).await?;
        if chip_id != Self::DEVICE_ID {
            return Err(ImuError::IncorrectWhoAmI);
        }
        delay_ms(1).await;

        // Disable FIFO
        self.write_register(Reg::FIFO_ENABLE, 0x00).await?;
        delay_ms(1).await;
        self.write_register(Reg::USER_CTRL, 0x00).await?;
        delay_ms(1).await;

        // Use 50us interrupt pulse (default)
        self.write_register(Reg::INT_PIN_CFG, 0x00).await?;
        delay_ms(1).await;

        // Gyro scale is fixed at 2000DPS, the maximum supported.
        //enum gyro_scale_e { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
        //const GYRO_FCHOICE_B:u8 = 0x00; // enables gyro update rate and filter configuration using Reg::CONFIG

        let (config, gyro_config, sample_rate_divider) =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);
        self.write_register(Reg::CONFIG, config).await?;
        delay_ms(1).await;
        self.write_register(Reg::GYRO_CONFIG, gyro_config).await?;
        delay_ms(1).await;
        self.write_register(Reg::SMPLRT_DIV, sample_rate_divider).await?;
        delay_ms(1).await;

        let (acc_config1, acc_config2) =
            self.calculate_acc_scale(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(Reg::ACCEL_CONFIG, acc_config1).await?;
        delay_ms(1).await;
        self.write_register(Reg::ACCEL_CONFIG2, acc_config2).await?;
        delay_ms(1).await;

        // enable data ready interrupt, now that everything has been configured
        self.write_register(Reg::INT_ENABLE, DATA_RDY_INT_EN).await?;
        delay_ms(10).await;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    /// Calculate and set `gyro_scale` and `gyro_sample_rate` given the requested gyro sensitivity, units, and ODR.
    /// Targets are rounded upwards.
    /// Values greater than the maximum supported result in setting maximum supported value.
    ///
    /// Returns `(config, gyro_config, sample_rate_divider)`,
    /// the values needed to set the `CONFIG`, `GYRO_CONFIG` and `SMPLRT_DIV` registers.
    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> (u8, u8, u8) {
        const GYRO_RANGE_250_DPS: u8 = 0b_0000_0000;
        const GYRO_RANGE_500_DPS: u8 = 0b_0000_1000;
        const GYRO_RANGE_1000_DPS: u8 = 0b_0001_0000;
        const GYRO_RANGE_2000_DPS: u8 = 0b_0001_1000;

        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            GyroSensitivity::Scale250Dps => (1.0 / 131.0, GYRO_RANGE_250_DPS),
            GyroSensitivity::Scale500Dps => (1.0 / 65.5, GYRO_RANGE_500_DPS),
            GyroSensitivity::Scale1000Dps => (1.0 / 32.8, GYRO_RANGE_1000_DPS),
            _ => (1.0 / 16.4, GYRO_RANGE_2000_DPS),
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        // SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        let (gyro_sample_rate_hz, sample_rate_divider) = match target_output_data_rate_hz {
            501..=1000 => (1000, 0),
            251..=500 => (500, 1),
            201..=250 => (250, 3),
            126..=200 => (200, 4),
            101..=125 => (125, 7),
            51..=100 => (100, 9),
            26..=50 => (50, 19),
            0..=25 => (25, 39),
            _ => (8_000, 0),
        };
        let config_register_value = if gyro_sample_rate_hz == 8000 { CONFIG_DLPF_0 } else { CONFIG_DLPF_176HZ };
        self.common.gyro_sample_rate_hz = gyro_sample_rate_hz;
        self.common.acc_sample_rate_hz = gyro_sample_rate_hz;

        (config_register_value, gyro_register_value, sample_rate_divider)
    }

    /// Calculate and set `acc_scale` and `acc_sample_rate` given the requested acc sensitivity, units, and ODR.
    /// Targets are rounded upwards.
    /// Values greater than the maximum supported result in setting maximum supported value.
    /// Returns `(acc_config1, acc_config2)`,
    /// the values needed to set the `ACC_CONFIG1` and `ACC_CONFIG2` registers.
    pub fn calculate_acc_scale(
        &mut self,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
        target_output_data_rate_hz: u32,
    ) -> (u8, u8) {
        const ACC_RANGE_16G: u8 = 0b_0001_1000;
        const ACC_RANGE_8G: u8 = 0b_0001_0000;
        const ACC_RANGE_4G: u8 = 0b_0000_1000;
        const ACC_RANGE_2G: u8 = 0b_0000_0000;

        let (scale, acc_config1) = match acc_sensitivity {
            AccSensitivity::Scale2G => (2.0 / 32768.0, ACC_RANGE_2G),
            AccSensitivity::Scale4G => (4.0 / 32768.0, ACC_RANGE_4G),
            AccSensitivity::Scale8G => (8.0 / 32768.0, ACC_RANGE_8G),
            _ => {
                // default includes  AccFullScale::Scale16G
                (16.0 / 32768.0, ACC_RANGE_16G)
            }
        };
        self.common.acc_scale = if acc_units == AccUnits::G { scale } else { scale * ImuCommon::G0 };

        let acc_config2 = if target_output_data_rate_hz > 1000 || target_output_data_rate_hz == 0 {
            0b_1000 // set ACCEL_FCHOICE_B bit
        } else {
            0
        };
        (acc_config1, acc_config2)
    }
}

#[cfg(test)]
mod tests {
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    #![allow(clippy::float_cmp)]

    use super::*;
    use crate::{ImuAxisOrder, MockImuBus};

    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new()
            .with_register(Reg::WHO_AM_I, Icm20602::<MockImuBus>::DEVICE_ID)
            .with_register(Reg::ACCEL_XOUT_H, 4);
        assert_eq!(4, imu_bus.registers[Reg::ACCEL_XOUT_H as usize]);

        let mut imu: Icm20602<MockImuBus> = Icm20602::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroSensitivity::Max, GyroUnits::Dps, AccSensitivity::Max, AccUnits::G));
        assert!(result.is_ok());
        if let Ok((gyro_sample_rate_hz, acc_sample_rate_hz)) = result {
            assert_eq!(8000, gyro_sample_rate_hz);
            assert_eq!(8000, acc_sample_rate_hz);

            assert_eq!(0.06097561, imu.common.gyro_scale);
            assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
            assert_eq!(8000, imu.common.gyro_sample_rate_hz);
            assert_eq!(8000, imu.common.acc_sample_rate_hz);
        }
        let reg = pollster::block_on(imu.read_register(Reg::INT_PIN_CFG));
        assert!(reg.is_ok());
        assert_eq!(Ok(0x00), reg);
    }
}
