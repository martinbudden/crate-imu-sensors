use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{
    AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice, ImuDeviceConfig,
};

const I2C_ADDRESS: u8 = 0x68;

// **** IMU Registers and associated bitflags ****
#[allow(unused)]
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

    const _XG_OFFS_USRH: u8 = 0x13;
    const _XG_OFFS_USRL: u8 = 0x14;
    const _YG_OFFS_USRH: u8 = 0x15;
    const _YG_OFFS_USRL: u8 = 0x16;
    const _ZG_OFFS_USRH: u8 = 0x17;
    const _ZG_OFFS_USRL: u8 = 0x18;

    const SAMPLE_RATE_DIVIDER: u8 = 0x19;
    const _DIVIDE_BY_1: u8 = 0x00;
    const DIVIDE_BY_2: u8 = 0x01;

    const CONFIG: u8 = 0x1A;
    const DLPF_CFG_1: u8 = 0x01;
    const _DLPF_CFG_7: u8 = 0x07;

    const GYRO_CONFIG: u8 = 0x1B;
    const ACCEL_CONFIG: u8 = 0x1C;
    const ACCEL_CONFIG2: u8 = 0x1D;

    const FIFO_ENABLE: u8 = 0x23;
    const _GYRO_FIFO_EN: u8 = 0b_0000_1000;
    const _ACC_FIFO_EN: u8 = 0b_0000_0100;

    const INT_PIN_CFG: u8 = 0x37;
    const INT_ENABLE: u8 = 0x38;
    const _FIFO_WM_INT_STATUS: u8 = 0x39;

    const ACCEL_XOUT_H: u8 = 0x3B;
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

    const _FIFO_WM_TH1: u8 = 0x60;
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

    const _XA_OFFSET_H: u8 = 0x77;
    const _XA_OFFSET_L: u8 = 0x78;
    const _YA_OFFSET_H: u8 = 0x7A;
    const _YA_OFFSET_L: u8 = 0x7B;
    const _ZA_OFFSET_H: u8 = 0x7D;
    const _ZA_OFFSET_L: u8 = 0x7E;
}

#[derive(Debug, PartialEq)]
pub struct Mpu6886<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for Mpu6886<B> {
    type Error = B::Error;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error> {
        Mpu6886::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for Mpu6886<B> {
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
        self.write_read(&[Reg::ACCEL_XOUT_H], &mut buf).await?;
        let acc = Vector3f32::from_be_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[Reg::GYRO_XOUT_H], &mut buf).await?;
        let gyro = Vector3f32::from_be_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 14];
        self.write_read(&[Reg::GYRO_XOUT_H], &mut buf).await?;

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

impl<B: ImuBus> Mpu6886<B> {
    const DEVICE_ID: u8 = 0;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_ACC_ID_DEFAULT,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_DEFAULT,
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
    #[allow(clippy::items_after_statements)]
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), B::Error> {
        let _chip_id = self.bus.read_register(self.config.address, Reg::WHO_AM_I).await;
        delay_ms(1).await;

        // Clear the power management register.
        self.write_register(Reg::PWR_MGMT_1, 0).await?;
        delay_ms(10).await;

        // Reset the device.
        const DEVICE_RESET: u8 = 0x01u8 << 7;
        self.write_register(Reg::PWR_MGMT_1, DEVICE_RESET).await?;
        delay_ms(10).await;

        // CLKSEL must be set to 001 to achieve full gyroscope performance.
        const CLKSEL_1: u8 = 0x01;
        self.write_register(Reg::PWR_MGMT_1, CLKSEL_1).await?;
        delay_ms(10).await;

        let (config, divider) =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);

        self.write_register(Reg::GYRO_CONFIG, config).await?;
        delay_ms(1).await;

        self.write_register(Reg::SAMPLE_RATE_DIVIDER, divider).await?;
        delay_ms(1).await;

        let acc_register_value =
            self.calculate_acc_scale_and_odr(acc_sensitivity, acc_units, target_output_data_rate_hz);

        self.write_register(Reg::ACCEL_CONFIG, acc_register_value).await?;
        delay_ms(1).await;

        // Configure filtering.
        const ACC_FCHOICE_B: u8 = 0x00; // Filter:218.1 3-DB BW (Hz), least filtered 1kHz update variant
        self.write_register(Reg::ACCEL_CONFIG2, ACC_FCHOICE_B).await?;
        delay_ms(1).await;

        // Configure FIFO.
        self.write_register(Reg::FIFO_ENABLE, 0x00).await?; // FIFO disabled
        delay_ms(1).await;
        const FIFO_MODE_OVERWRITE: u8 = 0b_0100_0000;
        self.write_register(Reg::CONFIG, Reg::DLPF_CFG_1 | FIFO_MODE_OVERWRITE).await?;
        delay_ms(1).await;

        // Configure interrupts.
        // M5 Unified settings
        //self.write_register(INT_PIN_CFG, 0b_1100_0000).await; // Active low, open drain 50us pulse width, clear on read
        self.write_register(Reg::INT_PIN_CFG, 0x22).await?;
        delay_ms(1).await;

        const DATA_RDY_INT_EN: u8 = 0x01;
        self.write_register(Reg::INT_ENABLE, DATA_RDY_INT_EN).await?; // data ready interrupt enabled
        delay_ms(10).await;

        self.write_register(Reg::USER_CTRL, 0x00).await?;
        delay_ms(1).await;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        _gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        _target_output_data_rate_hz: u32,
    ) -> (u8, u8) {
        const GFS_2000DPS: u8 = 3;
        const GYRO_FCHOICE_B: u8 = 0x00; // enables gyro update rate and filter configuration using CONFIG
        self.common.gyro_scale = 2000.0 / 32768.0;
        if gyro_units == GyroUnits::Rps {
            self.common.gyro_scale = self.common.gyro_scale.to_radians();
        }
        // M5Stack default divider is two, giving 500Hz output rate
        self.common.gyro_sample_rate_hz = 500;

        ((GFS_2000DPS << 3) | GYRO_FCHOICE_B, Reg::DIVIDE_BY_2)
    }

    pub fn calculate_acc_scale_and_odr(
        &mut self,
        _acc_sensitivity: AccFullScale,
        acc_scale: AccUnits,
        _target_output_data_rate_hz: u32,
    ) -> u8 {
        // Accelerometer scale is fixed at 8G, the maximum supported.
        //enum acc_scale_e { AFS_2G = 0, AFS_4G = 1, AFS_8G = 2, AFS_16G = 3 };
        const AFS_8G: u8 = 2;
        self.common.acc_scale = 8.0 / 32768.0;
        if acc_scale == AccUnits::Mps2 {
            self.common.acc_scale *= ImuCommon::G0;
        }
        self.common.acc_sample_rate_hz = self.common.gyro_sample_rate_hz;

        AFS_8G << 3
    }
}

#[cfg(test)]
mod tests {
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    #![allow(clippy::float_cmp)]
    use super::*;
    use crate::{AccFullScale, AccUnits, GyroFullScale, GyroUnits, ImuAxisOrder, MockImuBus};

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    impl<B: ImuBus> Mpu6886<B> {
        /// # Errors
        pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
            self.bus.read_register(self.config.address, reg).await
        }
    }

    #[test]
    fn normal_types() {}
    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new();
        let mut imu: Mpu6886<MockImuBus> = Mpu6886::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_sample_rate_hz, acc_sample_rate_hz) = result.unwrap();

        assert_eq!(500, gyro_sample_rate_hz);
        assert_eq!(500, acc_sample_rate_hz);
        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(8.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(500, imu.common.gyro_sample_rate_hz);
        assert_eq!(500, imu.common.acc_sample_rate_hz);
    }
}
