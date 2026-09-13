use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{
    AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice, ImuDeviceConfig,
};

const I2C_ADDRESS: u8 = 0x6A;
const _I2C_ADDRESS_ALTERNATIVE: u8 = 0x6B;

use cfg_if::cfg_if;

// **** IMU Registers and associated bitflags ****
const _REG_RESERVED_00: u8 = 0x00;
const _REG_FUNC_CFG_ACCESS: u8 = 0x01;
const _REG_RESERVED_03: u8 = 0x03;

cfg_if! {
if #[cfg(feature = "lsm6ds3tr_c")] {
const REG_RESERVED_02: u8 = 0x02;
const REG_SENSOR_SYNC_TIME_FRAME: u8 = 0x04;
const REG_SENSOR_SYNC_RES_RATIO: u8 = 0x05;
const REG_FIFO_CTRL1: u8 = 0x06;
const REG_FIFO_CTRL2: u8 = 0x07;
const REG_FIFO_CTRL3: u8 = 0x08;
const REG_FIFO_CTRL4: u8 = 0x09;
const REG_FIFO_CTRL5: u8 = 0x0A;
const REG_DRDY_PULSE_CFG_G: u8 = 0x0B;
const REG_RESERVED_0C: u8 = 0x0C;
const REG_MASTER_CONFIG: u8 = 0x1A;

} else if #[cfg(feature = "ism330dhcx")] {

const REG_PIN_CTRL: u8 = 0x02;
const REG_RESERVED_04: u8 = 0x04;
const REG_RESERVED_05: u8 = 0x05;
const REG_RESERVED_06: u8 = 0x06;
const REG_FIFO_CTRL1: u8 = 0x07;
const REG_FIFO_CTRL2: u8 = 0x08;
const REG_FIFO_CTRL3: u8 = 0x09;
const REG_FIFO_CTRL4: u8 = 0x0A;
const REG_COUNTER_BDR_REG1: u8 = 0x0B;
const REG_COUNTER_BDR_REG2: u8 = 0x0C;
const REG_ALL_INT_SRC: u8 = 0x1A;

} else if #[cfg(feature = "lsm6dsox")] {
const REG_PIN_CTRL: u8 = 0x02;
const REG_S4S_TPH_L: u8 = 0x04;
const REG_S4S_TPH_H: u8 = 0x05;
const REG_S4S_RR: u8 = 0x06;
const REG_FIFO_CTRL1: u8 = 0x07;
const REG_FIFO_CTRL2: u8 = 0x08;
const REG_FIFO_CTRL3: u8 = 0x09;
const REG_FIFO_CTRL4: u8 = 0x0A;
const REG_COUNTER_BDR_REG1: u8 = 0x0B;
const REG_COUNTER_BDR_REG2: u8 = 0x0C;
const REG_ALL_INT_SRC: u8 = 0x1A;
}
}

const REG_DATA_READY_PULSE_CONFIG: u8 = 0x0B;
const DATA_READY_PULSED: u8 = 0b_1000_0000;
const REG_INT1_CTRL: u8 = 0x0D;
const INT1_DRDY_G: u8 = 0b_0000_0010;
const REG_INT2_CTRL: u8 = 0x0E;
const INT2_DRDY_G: u8 = 0b_0000_0010;
const _REG_WHO_AM_I: u8 = 0x0F;
const _REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C: u8 = 0x6A;
const _REG_WHO_AM_I_RESPONSE_ISM330DHCX: u8 = 0x6B;
const _REG_WHO_AM_I_RESPONSE_LSM6DSOX: u8 = 0x6C;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const BDU: u8 = 0b_0100_0000;
const IF_INC: u8 = 0b_0000_0100;
const SW_RESET: u8 = 0b_0000_0001;
const _REG_CTRL4_C: u8 = 0x13;
const _I2C_DISABLE: u8 = 0b_0000_0100;
const _LPF1_SEL_G: u8 = 0b_0000_0010;
const _REG_CTRL5_C: u8 = 0x14;
const _REG_CTRL6_C: u8 = 0x15;
const _XL_HM_MODE_DISABLE: u8 = 0b_0001_0000;
const _LPF1_MEDIUM_HI: u8 = 0x00;
const _LPF1_MEDIUM_LO: u8 = 0x01;
const _LPF1_LO: u8 = 0x02;
const _LPF1_HI: u8 = 0x03;
const _REG_CTRL7_G: u8 = 0x16;
const _REG_CTRL8_XL: u8 = 0x17;
const _REG_CTRL9_XL: u8 = 0x18;
const _REG_CTRL10_C: u8 = 0x19;
const _REG_WAKE_UP_SRC: u8 = 0x1B;
const _REG_TAP_SRC: u8 = 0x1C;
const _REG_D6D_SRC: u8 = 0x1D;
const _REG_STATUS_REG: u8 = 0x1E;
const _REG_RESERVED_1F: u8 = 0x1F;

const _REG_OUT_TEMP_L: u8 = 0x20;
const _REG_OUT_TEMP_H: u8 = 0x22;

const REG_OUTX_L_G: u8 = 0x22;
const _REG_OUTX_H_G: u8 = 0x23;
const _REG_OUTY_L_G: u8 = 0x24;
const _REG_OUTY_H_G: u8 = 0x25;
const _REG_OUTZ_L_G: u8 = 0x26;
const _REG_OUTZ_H_G: u8 = 0x27;

const REG_OUTX_L_ACC: u8 = 0x28;
const _REG_OUTX_H_ACC: u8 = 0x29;
const _REG_OUTY_L_ACC: u8 = 0x2A;
const _REG_OUTY_H_ACC: u8 = 0x2B;
const _REG_OUTZ_L_ACC: u8 = 0x2C;
const _REG_OUTZ_H_ACC: u8 = 0x2D;
// **** IMU Registers and associated bitflags ****

#[derive(Debug, PartialEq)]
pub struct Lsm6ds<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for Lsm6ds<B> {
    type Error = B::Error;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error> {
        Lsm6ds::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for Lsm6ds<B> {
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
        self.write_read(&[REG_OUTX_L_ACC], &mut buf).await?;
        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_OUTX_L_G], &mut buf).await?;
        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 12];
        self.write_read(&[REG_OUTX_L_G], &mut buf).await?;

        let [g0, g1, g2, g3, g4, g5, a0, a1, a2, a3, a4, a5] = buf;

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

impl<B: ImuBus> Lsm6ds<B> {
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
        //if (chip_id != REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C && chip_id != REG_WHO_AM_I_RESPONSE_ISM330DHCX && chip_id != REG_WHO_AM_I_RESPONSE_LSM6DSOX) {

        // Software reset
        self.write_register(REG_CTRL3_C, SW_RESET).await?;

        // Set data ready pulsed
        self.write_register(REG_DATA_READY_PULSE_CONFIG, DATA_READY_PULSED).await?;
        delay_ms(1).await;

        // Interrupt pins are by default forced to ground, so active high
        // Enable gyro data ready on INT1 pin
        self.write_register(REG_INT1_CTRL, INT1_DRDY_G).await?;
        delay_ms(1).await;

        // Enable gyro data ready on INT2 pin
        self.write_register(REG_INT2_CTRL, INT2_DRDY_G).await?;
        delay_ms(1).await;

        // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
        self.write_register(REG_CTRL3_C, BDU | IF_INC).await?;
        delay_ms(1).await;

        let gyro_register_value =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);
        self.write_register(REG_CTRL2_G, gyro_register_value).await?;
        delay_ms(1).await;

        let acc_register_value =
            self.calculate_acc_scale_and_odr(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(REG_CTRL1_XL, acc_register_value).await?;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const GYRO_RANGE_125_DPS: u8 = 0b_0010;
        const _GYRO_RANGE_245_DPS: u8 = 0b_0000; // LSM6DS3TR_C
        const _GYRO_RANGE_250_DPS: u8 = 0b_0000; // ISM330DHCX, LSM6DSOX
        const GYRO_RANGE_500_DPS: u8 = 0b_0100;
        const GYRO_RANGE_1000_DPS: u8 = 0b_1000;
        const GYRO_RANGE_2000_DPS: u8 = 0b_1100;

        const GYRO_ODR_12P5_HZ: u8 = 0b_0001_0000;
        const GYRO_ODR_26_HZ: u8 = 0b_0010_0000;
        const GYRO_ODR_52_HZ: u8 = 0b_0011_0000;
        const GYRO_ODR_104_HZ: u8 = 0b_0100_0000;
        const GYRO_ODR_208_HZ: u8 = 0b_0101_0000; // corrected: was 0b_010100000 (9 bits)
        const GYRO_ODR_416_HZ: u8 = 0b_0110_0000;
        const GYRO_ODR_833_HZ: u8 = 0b_0111_0000;
        const GYRO_ODR_1666_HZ: u8 = 0b_1000_0000;
        const GYRO_ODR_3332_HZ: u8 = 0b_1001_0000;
        const GYRO_ODR_6664_HZ: u8 = 0b_1010_0000;

        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            GyroFullScale::Scale125Dps | GyroFullScale::Scale250Dps => (245.0 / 32768.0, GYRO_RANGE_125_DPS),
            GyroFullScale::Scale500Dps => (500.0 / 32768.0, GYRO_RANGE_500_DPS),
            GyroFullScale::Scale1000Dps => (1000.0 / 32768.0, GYRO_RANGE_1000_DPS),
            _ => (2000.0 / 32768.0, GYRO_RANGE_2000_DPS),
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        let (gyro_sample_rate_hz, gyro_odr) = match target_output_data_rate_hz {
            1667..=3332 => (3332, GYRO_ODR_3332_HZ),
            834..=1666 => (1666, GYRO_ODR_1666_HZ),
            417..=833 => (833, GYRO_ODR_833_HZ),
            209..=416 => (416, GYRO_ODR_416_HZ),
            105..=208 => (208, GYRO_ODR_208_HZ),
            53..=104 => (104, GYRO_ODR_104_HZ),
            27..=52 => (52, GYRO_ODR_52_HZ),
            13..=26 => (26, GYRO_ODR_26_HZ),
            1..=12 => (12, GYRO_ODR_12P5_HZ),
            _ => (6664, GYRO_ODR_6664_HZ),
        };
        self.common.gyro_sample_rate_hz = gyro_sample_rate_hz;

        gyro_register_value | gyro_odr
    }

    pub fn calculate_acc_scale_and_odr(
        &mut self,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const ACC_RANGE_2G: u8 = 0b_0000;
        const ACC_RANGE_4G: u8 = 0b_1000;
        const ACC_RANGE_8G: u8 = 0b_1100;
        const ACC_RANGE_16G: u8 = 0b_0100;
        const ACC_ODR_12P5_HZ: u8 = 0b_0001_0000;
        const ACC_ODR_26_HZ: u8 = 0b_0010_0000;
        const ACC_ODR_52_HZ: u8 = 0b_0011_0000;
        const ACC_ODR_104_HZ: u8 = 0b_0100_0000;
        const ACC_ODR_208_HZ: u8 = 0b_0101_0000; // corrected: was 0b_010100000 (9 bits)
        const ACC_ODR_416_HZ: u8 = 0b_0110_0000;
        const ACC_ODR_833_HZ: u8 = 0b_0111_0000;
        const ACC_ODR_1666_HZ: u8 = 0b_1000_0000;
        const ACC_ODR_3332_HZ: u8 = 0b_1001_0000;
        const ACC_ODR_6664_HZ: u8 = 0b_1010_0000;

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

        let (acc_sample_rate_hz, acc_odr) = match target_output_data_rate_hz {
            1667..=3332 => (3332, ACC_ODR_3332_HZ),
            834..=1666 => (1666, ACC_ODR_1666_HZ),
            417..=833 => (833, ACC_ODR_833_HZ),
            209..=416 => (416, ACC_ODR_416_HZ),
            105..=208 => (208, ACC_ODR_208_HZ),
            53..=104 => (104, ACC_ODR_104_HZ),
            27..=52 => (52, ACC_ODR_52_HZ),
            13..=26 => (26, ACC_ODR_26_HZ),
            1..=12 => (12, ACC_ODR_12P5_HZ),
            _ => (6664, ACC_ODR_6664_HZ),
        };
        self.common.acc_sample_rate_hz = acc_sample_rate_hz;

        acc_register_value | acc_odr
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

    impl<B: ImuBus> Lsm6ds<B> {
        async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
            self.bus.read_register(self.config.address, reg).await
        }
    }

    #[test]
    fn normal_types() {}
    #[test]
    fn imu_init() {
        let mut imu_bus = MockImuBus::new();
        assert_eq!(0, imu_bus.registers[REG_CTRL3_C as usize]);
        imu_bus.registers[REG_CTRL3_C as usize] = 4;
        let mut imu: Lsm6ds<MockImuBus> = Lsm6ds::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_odr, acc_odr) = result.unwrap();

        assert_eq!(6664, gyro_odr);
        assert_eq!(6664, acc_odr);

        let reg = pollster::block_on(imu.read_register(REG_CTRL3_C));
        assert_eq!(BDU | IF_INC, reg.unwrap());

        let reg = pollster::block_on(imu.read_register(REG_DATA_READY_PULSE_CONFIG));
        assert_eq!(DATA_READY_PULSED, reg.unwrap());

        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(6664, imu.common.gyro_sample_rate_hz);
        assert_eq!(6664, imu.common.acc_sample_rate_hz);
    }
}
