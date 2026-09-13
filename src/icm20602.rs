use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDeviceConfig};

const I2C_ADDRESS: u8 = 0x68;
const _I2C_ADDRESS_ALTERNATIVE: u8 = 0x69;

// **** IMU Registers and associated bitflags ****
const _REG_XG_OFFS_TC_H: u8 = 0x04;
const _REG_XG_OFFS_TC_L: u8 = 0x05;
const _REG_YG_OFFS_TC_H: u8 = 0x07;
const _REG_YG_OFFS_TC_L: u8 = 0x08;
const _REG_ZG_OFFS_TC_H: u8 = 0x0A;
const _REG_ZG_OFFS_TC_L: u8 = 0x0B;

const _REG_SELF_TEST_X_ACCEL: u8 = 0x0D;
const _REG_SELF_TEST_Y_ACCEL: u8 = 0x0E;
const _REG_SELF_TEST_Z_ACCEL: u8 = 0x0F;

//  GYRO OFFSET ADJUSTMENT REGISTERS
const _REG_XG_OFFS_USRH: u8 = 0x13;
const _REG_XG_OFFS_USRL: u8 = 0x14;
const _REG_YG_OFFS_USRH: u8 = 0x15;
const _REG_YG_OFFS_USRL: u8 = 0x16;
const _REG_ZG_OFFS_USRH: u8 = 0x17;
const _REG_ZG_OFFS_USRL: u8 = 0x18;

const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_CONFIG2: u8 = 0x1D;

const REG_FIFO_ENABLE: u8 = 0x23;
const _GYRO_FIFO_EN: u8 = 0b_0000_1000;
const _ACC_FIFO_EN: u8 = 0b_0000_0100;

const REG_INT_PIN_CFG: u8 = 0x37;
const REG_INT_ENABLE: u8 = 0x38;
const _FIFO_WM_INT_STATUS: u8 = 0x39; // FIFO watermark interrupt status

pub const REG_ACCEL_XOUT_H: u8 = 0x3B;
const _REG_ACCEL_XOUT_L: u8 = 0x3C;
const _REG_ACCEL_YOUT_H: u8 = 0x3D;
const _REG_ACCEL_YOUT_L: u8 = 0x3E;
const _REG_ACCEL_ZOUT_H: u8 = 0x3F;
const _REG_ACCEL_ZOUT_L: u8 = 0x40;

const _REG_TEMP_OUT_H: u8 = 0x41;
const _REG_TEMP_OUT_L: u8 = 0x42;

const REG_GYRO_XOUT_H: u8 = 0x43;
const _REG_GYRO_XOUT_L: u8 = 0x44;
const _REG_GYRO_YOUT_H: u8 = 0x45;
const _REG_GYRO_YOUT_L: u8 = 0x46;
const _REG_GYRO_ZOUT_H: u8 = 0x47;
const _REG_GYRO_ZOUT_L: u8 = 0x48;

const _REG_FIFO_WM_TH1: u8 = 0x60; // FIFO watermark threshold in number of bytes
const _REG_FIFO_WM_TH2: u8 = 0x61;

const _REG_SIGNAL_PATH_RESET: u8 = 0x68;
const _REG_ACCEL_INTEL_CTRL: u8 = 0x69;
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const _REG_PWR_MGMT_2: u8 = 0x6C;

const _REG_FIFO_COUNT_H: u8 = 0x72;
const _REG_FIFO_COUNT_L: u8 = 0x73;
const _REG_FIFO_R_W: u8 = 0x74;

const _REG_WHO_AM_I: u8 = 0x75;

// ACCELEROMETER OFFSET REGISTERS
const _REG_XA_OFFSET_H: u8 = 0x77;
const _REG_XA_OFFSET_L: u8 = 0x78;
const _REG_YA_OFFSET_H: u8 = 0x7A;
const _REG_YA_OFFSET_L: u8 = 0x7B;
const _REG_ZA_OFFSET_H: u8 = 0x7D;
const _REG_ZA_OFFSET_L: u8 = 0x7E;
// **** IMU Registers and associated bitflags ****

#[derive(Debug, PartialEq)]
pub struct Icm20602<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> Imu for Icm20602<B> {
    type Bus = B;
    type Error = <B as ImuBus>::Error;

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

    async fn read_acc(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_ACCEL_XOUT_H], &mut buf).await?;
        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_GYRO_XOUT_H], &mut buf).await?;
        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 14];
        self.write_read(&[REG_ACCEL_XOUT_H], &mut buf).await?;

        let [a0, a1, a2, a3, a4, a5, _t0, _t1, g0, g1, g2, g3, g4, g5] = buf;

        let acc_buf = [a0, a1, a2, a3, a4, a5];
        let gyro_buf = [g0, g1, g2, g3, g4, g5];
        let acc = Vector3f32::from_le_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_le_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_acc_gyro(self.common.axis_order, acc, gyro))
    }
}

async fn delay_ms(delay: u32) {
    Timer::after(Duration::from_millis(delay.into())).await;
}

impl<B: ImuBus> Icm20602<B> {
    const DEVICE_ID: u8 = 0x68;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_GYRO_ID_LSM6DSO,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_LSM6DSO,
                axis_order,
                device_id: Self::DEVICE_ID,
                address: I2C_ADDRESS,
                flags: 0,
            },
        }
    }

    /// # Errors
    pub async fn write_register(&mut self, reg: u8, data: u8) -> Result<(), B::Error> {
        self.bus.write_register(self.config.address, reg, data).await
    }

    /// # Errors
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), B::Error> {
        const DEVICE_RESET: u8 = 0x01 << 7;
        const CLKSEL_1: u8 = 0x01;
        const DATA_RDY_INT_EN: u8 = 0x01;

        //let _chip_id = self.bus.read_register(self.config.address, REG_WHO_AM_I);
        //delay_ms(1);

        // clear the power management register
        self.write_register(REG_PWR_MGMT_1, 0).await?;
        delay_ms(10).await;

        // reset the device
        self.write_register(REG_PWR_MGMT_1, DEVICE_RESET).await?;
        delay_ms(10).await;

        // CLKSEL must be set to 001 to achieve full gyroscope performance.
        self.write_register(REG_PWR_MGMT_1, CLKSEL_1).await?;
        delay_ms(10).await;

        self.write_register(REG_FIFO_ENABLE, 0x00).await?;
        delay_ms(1).await;

        // Interrupt settings
        self.write_register(REG_INT_PIN_CFG, 0x22).await?;
        delay_ms(1).await;

        // data ready interrupt enabled
        self.write_register(REG_INT_ENABLE, DATA_RDY_INT_EN).await?;
        delay_ms(10).await;

        self.write_register(REG_USER_CTRL, 0x00).await?;
        delay_ms(1).await;
        // Gyro scale is fixed at 2000DPS, the maximum supported.
        //enum gyro_scale_e { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
        //const GYRO_FCHOICE_B:u8 = 0x00; // enables gyro update rate and filter configuration using REG_CONFIG

        // FIFO disabled, LPF disabled, base output data rate 8kHz (before divider applied)
        self.write_register(REG_CONFIG, 0).await?;
        delay_ms(1).await;
        let (gyro_register_value, sample_rate_divider) =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);
        self.write_register(REG_GYRO_CONFIG, gyro_register_value).await?;
        delay_ms(1).await;

        self.write_register(REG_SMPLRT_DIV, sample_rate_divider).await?;
        delay_ms(1).await;

        let acc_register_value = self.calculate_acc_scale(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(REG_ACCEL_CONFIG, acc_register_value).await?;
        //_acc_resolution = ACC_8G_RES;
        delay_ms(1).await;

        self.write_register(REG_ACCEL_CONFIG2, 0).await?;
        delay_ms(1).await;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> (u8, u8) {
        const GYRO_RANGE_250_DPS: u8 = 0b_0000_0000;
        const GYRO_RANGE_500_DPS: u8 = 0b_0000_1000;
        const GYRO_RANGE_1000_DPS: u8 = 0b_0001_0000;
        const GYRO_RANGE_2000_DPS: u8 = 0b_0001_1000;

        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            GyroFullScale::Scale250Dps => (250.0 / 32768.0, GYRO_RANGE_250_DPS),
            GyroFullScale::Scale500Dps => (500.0 / 32768.0, GYRO_RANGE_500_DPS),
            GyroFullScale::Scale1000Dps => (1000.0 / 32768.0, GYRO_RANGE_1000_DPS),
            _ => (2000.0 / 32768.0, GYRO_RANGE_2000_DPS),
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        // SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        let (gyro_sample_rate_hz, sample_rate_divider) = match target_output_data_rate_hz {
            2001..=4000 => (4000, 1),
            1001..=2000 => (2000, 3),
            501..=1000 => (1000, 7),
            201..=500 => (500, 15),
            101..=200 => (200, 31),
            51..=100 => (100, 63),
            26..=50 => (50, 127),
            _ => (8_000, 0),
        };
        self.common.gyro_sample_rate_hz = gyro_sample_rate_hz;
        self.common.acc_sample_rate_hz = gyro_sample_rate_hz;

        (gyro_register_value, sample_rate_divider)
    }

    pub fn calculate_acc_scale(
        &mut self,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
        _target_output_data_rate_hz: u32,
    ) -> u8 {
        const ACC_RANGE_16G: u8 = 0b_0001_1000;
        const ACC_RANGE_8G: u8 = 0b_0001_0000;
        const ACC_RANGE_4G: u8 = 0b_0000_1000;
        const ACC_RANGE_2G: u8 = 0b_0000_0000;

        let (scale, acc_register_value) = match acc_sensitivity {
            AccFullScale::Scale2G => (2.0 / 32768.0, ACC_RANGE_2G),
            AccFullScale::Scale4G => (4.0 / 32768.0, ACC_RANGE_4G),
            AccFullScale::Scale8G => (8.0 / 32768.0, ACC_RANGE_8G),
            _ => {
                // default includes  AccFullScale::Scale16G
                (16.0 / 32768.0, ACC_RANGE_16G)
            }
        };
        self.common.acc_scale = if acc_units == AccUnits::G { scale } else { scale * ImuCommon::G0 };

        acc_register_value
    }
}

#[cfg(test)]
mod tests {
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    #![allow(clippy::float_cmp)]

    use super::*;
    use crate::{ImuAxisOrder, MockImuBus};

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    impl<B: ImuBus> Icm20602<B> {
        async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
            self.bus.read_register(self.config.address, reg).await
        }
    }

    #[test]
    fn normal_types() {}
    #[test]
    fn imu_init() {
        let mut imu_bus = MockImuBus::new();
        assert_eq!(0, imu_bus.registers[REG_ACCEL_XOUT_H as usize]);
        imu_bus.registers[REG_ACCEL_XOUT_H as usize] = 4;
        let mut imu: Icm20602<MockImuBus> = Icm20602::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_odr, acc_odr) = result.unwrap();

        let reg = pollster::block_on(imu.read_register(REG_INT_PIN_CFG));
        assert_eq!(0x22, reg.unwrap());

        assert_eq!(8000, gyro_odr);
        assert_eq!(8000, acc_odr);

        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(8000, imu.common.gyro_sample_rate_hz);
        assert_eq!(8000, imu.common.acc_sample_rate_hz);
    }
}
