use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{
    AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice, ImuDeviceConfig,
};

const I2C_ADDRESS: u8 = 0x6A;
const _I2C_ADDRESS_ALTERNATIVE: u8 = 0x6B;

// **** IMU Registers and associated bitflags ****

const _REG_WHO_AM_I: u8 = 0x00;
const _REG_REVISION_ID: u8 = 0x01;
const REG_RESET: u8 = 0x60;

const REG_CTRL1: u8 = 0x02;
const REG_CTRL2: u8 = 0x03;
const REG_CTRL3: u8 = 0x04;
const _REG_CTRL5: u8 = 0x06;
const REG_CTRL7: u8 = 0x08;
const _REG_CTRL8: u8 = 0x09;
const _REG_CTRL9: u8 = 0x0a;

const _REG_CAL1_L: u8 = 0x0b;
const _REG_CAL1_H: u8 = 0x0c;
const _REG_CAL2_L: u8 = 0x0d;
const _REG_CAL2_H: u8 = 0x0e;
const _REG_CAL3_L: u8 = 0x0f;
const _REG_CAL3_H: u8 = 0x10;
const _REG_CAL4_L: u8 = 0x11;
const _REG_CAL4_H: u8 = 0x12;

const _REG_TEMP_L: u8 = 0x033;
const _REG_TEMP_H: u8 = 0x034;
const REG_AX_L: u8 = 0x035;
const _REG_AX_H: u8 = 0x036;
const _REG_AY_L: u8 = 0x037;
const _REG_AY_H: u8 = 0x038;
const _REG_AZ_L: u8 = 0x039;
const _REG_AZ_H: u8 = 0x031;

const REG_GX_L: u8 = 0x03B;
const _REG_GX_H: u8 = 0x03c;
const _REG_GY_L: u8 = 0x03d;
const _REG_GY_H: u8 = 0x03e;
const _REG_GZ_L: u8 = 0x03f;
const _REG_GZ_H: u8 = 0x040;

#[allow(missing_docs)]
#[derive(Debug, PartialEq)]
pub struct Qmi8658a<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for Qmi8658a<B> {
    type Error = B::Error;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error> {
        Qmi8658a::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for Qmi8658a<B> {
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
        self.write_read(&[REG_AX_L], &mut buf).await?;
        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_GX_L], &mut buf).await?;
        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 12];
        self.write_read(&[REG_AX_L], &mut buf).await?;

        let [a0, a1, a2, a3, a4, a5, g0, g1, g2, g3, g4, g5] = buf;

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

impl<B: ImuBus> Qmi8658a<B> {
    pub const MAX_SPI_FREQUENCY_HZ: u32 = 15_000_000;
    const DEVICE_ID: u8 = 0;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_GYRO_ID_DEFAULT,
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
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), B::Error> {
        // CTRL1
        // auto increment required for burst mode reading, ie for ACC an GYRO registers
        const ADDRESS_AUTO_INCREMENT: u8 = 0b_0100_0000;
        const _BIG_ENDIAN: u8 = 0b_0010_0000;
        const _INT1_ENABLE: u8 = 0b_0001_0000;
        // Data ready (DRDY) is signalled on the INT2 pin.
        const INT2_ENABLE: u8 = 0b_0000_1000;

        // CTRL7
        const GYRO_ENABLE: u8 = 0b_0000_0010;
        const ACC_ENABLE: u8 = 0b_0000_0001;

        // soft RESET
        self.write_register(REG_RESET, 0x0b).await?;
        // soft reset takes a maximum of 15ms
        delay_ms(15).await;

        // REG_CTRL1
        self.write_register(REG_CTRL1, ADDRESS_AUTO_INCREMENT | INT2_ENABLE).await?;
        delay_ms(1).await;

        // REG_CTRL2
        let acc_register_value =
            self.calculate_acc_scale_and_odr(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(REG_CTRL2, acc_register_value).await?;
        delay_ms(1).await;

        // REG_CTRL3
        let gyro_register_value =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);
        self.write_register(REG_CTRL3, gyro_register_value).await?;
        delay_ms(1).await;

        // No REG_CTRL4
        // REG_CTRL5 is LPF filters - leave all off
        // No REG_CTRL6

        // REG_CTRL7, DRDY is enabled by default, sets INT2 pin high
        self.write_register(REG_CTRL7, GYRO_ENABLE | ACC_ENABLE).await?;
        delay_ms(1).await;
        // REG_CTRL8 is motion detection - leave all off

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const GYRO_ODR_7172_P_4_HZ: u8 = 0b_0000_0000;
        const GYRO_ODR_3587_P_2_HZ: u8 = 0b_0000_0001;
        const GYRO_ODR_1793_P_6_HZ: u8 = 0b_0000_0010;
        const GYRO_ODR_896_P_8_HZ: u8 = 0b_0000_0011;
        const GYRO_ODR_448_P_4_HZ: u8 = 0b_0000_0100;
        const GYRO_ODR_224_P_2_HZ: u8 = 0b_0000_0101;
        const GYRO_ODR_112_P_1_HZ: u8 = 0b_0000_0110;
        const GYRO_ODR_56_P_05_HZ: u8 = 0b_0000_0111;
        const GYRO_ODR_28_P_025_HZ: u8 = 0b_0000_1000;

        const GYRO_RANGE_2048_DPS: u8 = 0b_0110_0000;
        const GYRO_RANGE_1024_DPS: u8 = 0b_0110_0000;
        const GYRO_RANGE_512_DPS: u8 = 0b_0101_0000;
        const GYRO_RANGE_256_DPS: u8 = 0b_0100_0000;
        const GYRO_RANGE_128_DPS: u8 = 0b_0011_0000;
        const _GYRO_RANGE_64_DPS: u8 = 0b_0010_0000;
        const _GYRO_RANGE_32_DPS: u8 = 0b_0001_0000;
        const _GYRO_RANGE_16_DPS: u8 = 0b_0000_0000;

        // calculate the GYRO_ODR bit values to write to the REG_GYRO_CONFIG0 register
        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            GyroFullScale::Scale125Dps => (128.0 / 32768.0, GYRO_RANGE_128_DPS),
            GyroFullScale::Scale250Dps => (256.0 / 32768.0, GYRO_RANGE_256_DPS),
            GyroFullScale::Scale500Dps => (512.0 / 32768.0, GYRO_RANGE_512_DPS),
            GyroFullScale::Scale1000Dps => (1024.0 / 32768.0, GYRO_RANGE_1024_DPS),
            _ => (2048.0 / 32768.0, GYRO_RANGE_2048_DPS),
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        let (gyro_sample_rate_hz, gyro_odr) = match target_output_data_rate_hz {
            1794..=3587 => (3587, GYRO_ODR_3587_P_2_HZ),
            897..=1793 => (1793, GYRO_ODR_1793_P_6_HZ),
            449..=896 => (896, GYRO_ODR_896_P_8_HZ),
            225..=448 => (448, GYRO_ODR_448_P_4_HZ),
            113..=224 => (224, GYRO_ODR_224_P_2_HZ),
            57..=112 => (112, GYRO_ODR_112_P_1_HZ),
            29..=56 => (56, GYRO_ODR_56_P_05_HZ),
            1..=28 => (28, GYRO_ODR_28_P_025_HZ),
            _ => (7172, GYRO_ODR_7172_P_4_HZ),
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
        const ACC_ODR_7172_P_4_HZ: u8 = 0b_0000_0000;
        const ACC_ODR_3587_P_2_HZ: u8 = 0b_0000_0001;
        const ACC_ODR_1793_P_6_HZ: u8 = 0b_0000_0010;
        const ACC_ODR_896_P_8_HZ: u8 = 0b_0000_0011;
        const ACC_ODR_448_P_4_HZ: u8 = 0b_0000_0100;
        const ACC_ODR_224_P_2_HZ: u8 = 0b_0000_0101;
        const ACC_ODR_112_P_1_HZ: u8 = 0b_0000_0110;
        const ACC_ODR_56_P_05_HZ: u8 = 0b_0000_0111;
        const ACC_ODR_28_P_025_HZ: u8 = 0b_0000_1000;

        const ACCEL_RANGE_16G: u8 = 0b_0011_0000;
        const ACCEL_RANGE_8G: u8 = 0b_0010_0000;
        const ACCEL_RANGE_4G: u8 = 0b_0001_0000;
        const ACCEL_RANGE_2G: u8 = 0b_0000_0000;
        // calculate the ACCEL_ODR bit values to write to the REG_ACCEL_CONFIG0 register
        let (scale, acc_register_value) = match acc_sensitivity {
            AccFullScale::Scale2G => (2.0 / 32768.0, ACCEL_RANGE_2G),
            AccFullScale::Scale4G => (4.0 / 32768.0, ACCEL_RANGE_4G),
            AccFullScale::Scale8G => (8.0 / 32768.0, ACCEL_RANGE_8G),
            _ => {
                // default includes  AccFullScale::Scale16G
                (16.0 / 32768.0, ACCEL_RANGE_16G)
            }
        };
        self.common.acc_scale = if acc_units == AccUnits::G { scale } else { scale * ImuCommon::G0 };

        let (acc_sample_rate_hz, acc_odr) = match target_output_data_rate_hz {
            1794..=3587 => (3587, ACC_ODR_3587_P_2_HZ),
            897..=1793 => (1793, ACC_ODR_1793_P_6_HZ),
            449..=896 => (896, ACC_ODR_896_P_8_HZ),
            225..=448 => (448, ACC_ODR_448_P_4_HZ),
            113..=224 => (224, ACC_ODR_224_P_2_HZ),
            57..=112 => (112, ACC_ODR_112_P_1_HZ),
            29..=56 => (56, ACC_ODR_56_P_05_HZ),
            1..=28 => (28, ACC_ODR_28_P_025_HZ),
            _ => (7172, ACC_ODR_7172_P_4_HZ),
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
    use core::future::Future;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    impl<B: ImuBus> Qmi8658a<B> {
        /// # Errors
        pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
            self.bus.read_register(self.config.address, reg).await
        }
    }

    // A lightweight VTable for a host spin-loop waker that requires zero allocations
    // Kept here as a reference in case I ever want to get rid of pollster.
    const NOOP_VTABLE: RawWakerVTable =
        RawWakerVTable::new(|_| RawWaker::new(core::ptr::null(), &NOOP_VTABLE), |_| {}, |_| {}, |_| {});

    // Zero-dependency block_on function
    fn block_on<F: Future>(mut future: F) -> F::Output {
        let mut future = unsafe { core::pin::Pin::new_unchecked(&mut future) };
        let raw_waker = RawWaker::new(core::ptr::null(), &NOOP_VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut context = Context::from_waker(&waker);

        loop {
            match future.as_mut().poll(&mut context) {
                Poll::Ready(output) => return output,
                Poll::Pending => {
                    //std::thread::yield_now();
                    core::hint::spin_loop();
                }
            }
        }
    }

    #[test]
    fn normal_types() {}
    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new();
        let mut imu: Qmi8658a<MockImuBus> = Qmi8658a::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result = block_on(imu.init(7172, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_odr, acc_odr) = result.unwrap();

        assert_eq!(7172, gyro_odr);
        assert_eq!(7172, acc_odr);
        assert_eq!(2048.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(7172, imu.common.gyro_sample_rate_hz);
        assert_eq!(7172, imu.common.acc_sample_rate_hz);
    }
}
