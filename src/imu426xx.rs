use embassy_time::{Duration, Timer};
use vqm::Vector3f32;

use super::{
    AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice, ImuDeviceConfig,
};

const I2C_ADDRESS: u8 = 0x6A;
const _I2C_ADDRESS_ALTERNATIVE: u8 = 0x6B;

// **** IMU Registers and associated bitflags ****
const _REG_SENSOR_CONFIG0: u8 = 0x03;

const REG_DEVICE_CONFIG: u8 = 0x11;
const DEVICE_CONFIG_DEFAULT: u8 = 0b_0000_0000;
const REG_INT_CONFIG: u8 = 0x14;
const _INT1_MODE_LATCHED: u8 = 0b_0000_0100;
const INT1_MODE_PULSED: u8 = 0b_0000_0000;
const INT1_DRIVE_CIRCUIT_PUSH_PULL: u8 = 0b_0000_0010;
const INT1_POLARITY_ACTIVE_HIGH: u8 = 0b_0000_0001;
const _REG_FIFO_CONFIG: u8 = 0x16;

const _REG_TEMP_DATA1: u8 = 0x1D;
const _REG_TEMP_DATA0: u8 = 0x1E;
const REG_ACCEL_DATA_X1: u8 = 0x1F;
const _REG_ACCEL_DATA_X0: u8 = 0x20;
const _REG_ACCEL_DATA_Y1: u8 = 0x21;
const _REG_ACCEL_DATA_Y0: u8 = 0x22;
const _REG_ACCEL_DATA_Z1: u8 = 0x23;
const _REG_ACCEL_DATA_Z0: u8 = 0x24;
const REG_GYRO_DATA_X1: u8 = 0x25;
const _REG_GYRO_DATA_X0: u8 = 0x26;
const _REG_GYRO_DATA_Y1: u8 = 0x27;
const _REG_GYRO_DATA_Y0: u8 = 0x28;
const _REG_GYRO_DATA_Z1: u8 = 0x29;
const _REG_GYRO_DATA_Z0: u8 = 0x2A;

const _REG_TMST_FSYNCH: u8 = 0x2B;
const _REG_TMST_FSYNCHL: u8 = 0x2C;
const _REG_INT_STATUS: u8 = 0x2D;

const _REG_FIFO_COUNTH: u8 = 0x2E;
const _REG_FIFO_COUNTL: u8 = 0x2F;
const _REG_FIFO_DATA: u8 = 0x30;

const _REG_APEX_DATA0: u8 = 0x31;
const _REG_APEX_DATA1: u8 = 0x33;
const _REG_APEX_DATA2: u8 = 0x34;
const _REG_APEX_DATA3: u8 = 0x34;
const _REG_APEX_DATA4: u8 = 0x35;
const _REG_APEX_DATA5: u8 = 0x36;

const _REG_INT_STATUS2: u8 = 0x37;
const _REG_INT_STATUS3: u8 = 0x38;

const _REG_SIGNAL_PATH_RESET: u8 = 0x4B;
const REG_INTF_CONFIG0: u8 = 0x4C;
const _SENSOR_DATA_BIG_ENDIAN: u8 = 0b_0001_0000; // default
const SENSOR_DATA_LITTLE_ENDIAN: u8 = 0b_0000_0000; // datasheet gives very little information on how this works
const UI_SIFS_CFG_DISABLE_I2C: u8 = 0b_0000_0011;
const REG_INTF_CONFIG1: u8 = 0x4D;
const _AFSR_DISABLE: u8 = 0x40;

const REG_PWR_MGMT0: u8 = 0x4E;
const PWR_OFF: u8 = 0x00;
const PWR_TEMP_ENABLED: u8 = 0b_0000_0000;
const PWR_GYRO_LOW_NOISE: u8 = 0b_0000_1100;
const PWR_ACCEL_LOW_NOISE: u8 = 0b_0000_0011;

const REG_GYRO_CONFIG0: u8 = 0x4F;

const REG_ACCEL_CONFIG0: u8 = 0x50;

const _REG_GYRO_CONFIG1: u8 = 0x51;
const REG_GYRO_ACCEL_CONFIG0: u8 = 0x52;
const ACCEL_FILTER_LOW_LATENCY: u8 = 0b_1111_0000;
const GYRO_FILTER_LOW_LATENCY: u8 = 0b_0000_1111;
const _REG_ACCEL_CONFIG1: u8 = 0x53;

const _REG_TMST_CONFIG: u8 = 0x54;
const _REG_APEX_CONFIG: u8 = 0x56;
const _REG_SMD_CONFIG: u8 = 0x57;
const _REG_FIFO_CONFIG1: u8 = 0x5F;
const _REG_FIFO_CONFIG2: u8 = 0x60;
const _REG_FIFO_CONFIG3: u8 = 0x61;
const _REG_FSYNC_CONFIG: u8 = 0x62;
const REG_INT_CONFIG0: u8 = 0x63;
const INT_CLEAR_ON_STATUS_BIT_READ: u8 = 0b_0000_0000;
const REG_INT_CONFIG1: u8 = 0x64;
const _INT_ASYNC_RESET: u8 = 0b_0000_1000; // this bit should be set to 0 for proper INT1 and INT2 pin operation
const INT_TPULSE_DURATION_8US: u8 = 0b_0100_0000; // interrupt puls duration 8us, required for ODR >=4kHz
const INT_TDEASSERT_DISABLE: u8 = 0b_0010_0000; // required for ODR >= 4kHz

const REG_INT_SOURCE0: u8 = 0x65;
const INT1_UI_DATA_READY_ENABLED: u8 = 0b_0000_1000;
const _REG_INT_SOURCE1: u8 = 0x66;
const _REG_INT_SOURCE3: u8 = 0x68;
const _REG_INT_SOURCE4: u8 = 0x69;

const _REG_FIFO_LOST_PKT0: u8 = 0x6C;
const _REG_FIFO_LOST_PKT1: u8 = 0x6D;
const _REG_SELF_TEST_CONFIG3: u8 = 0x70;

const _REG_WHO_AM_I: u8 = 0x75;
const _WHO_AM_I_RESPONSE_ICM42605: u8 = 0x43;
const _WHO_AM_I_RESPONSE_ICM42688P: u8 = 0x47;

const REG_BANK_SEL: u8 = 0x76;
const _REG_INTF_CONFIG4: u8 = 0x7A;
const _REG_INTF_CONFIG5: u8 = 0x7B;
const _REG_INTF_CONFIG6: u8 = 0x7C;

// User Bank 1 Register Map
const _REG_BANK1_GYRO_CONFIG_STATIC2: u8 = 0x0B;
const REG_BANK1_GYRO_CONFIG_STATIC3: u8 = 0x0C;
const REG_BANK1_GYRO_CONFIG_STATIC4: u8 = 0x0D;
const REG_BANK1_GYRO_CONFIG_STATIC5: u8 = 0x0E;
const _REG_BANK1_GYRO_CONFIG_STATIC6: u8 = 0x0F;
const _REG_BANK1_GYRO_CONFIG_STATIC7: u8 = 0x10;
const _REG_BANK1_GYRO_CONFIG_STATIC8: u8 = 0x11;
const _REG_BANK1_GYRO_CONFIG_STATIC9: u8 = 0x12;
const _REG_BANK1_GYRO_CONFIG_STATIC10: u8 = 0x13;

// User Bank 2 Register Map
const _REG_BANK2_ACCEL_CONFIG_STATIC2: u8 = 0x03;
const _REG_BANK2_ACCEL_CONFIG_STATIC3: u8 = 0x04;
const _REG_BANK2_ACCEL_CONFIG_STATIC4: u8 = 0x05;

#[allow(missing_docs)]
#[derive(Debug, PartialEq)]
pub struct Imu426xx<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for Imu426xx<B> {
    type Error = B::Error;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error> {
        Imu426xx::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for Imu426xx<B> {
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
        self.write_read(&[REG_ACCEL_DATA_X1], &mut buf).await?;
        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_GYRO_DATA_X1], &mut buf).await?;
        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 12];
        self.write_read(&[REG_GYRO_DATA_X1], &mut buf).await?;

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

impl<B: ImuBus> Imu426xx<B> {
    const DEVICE_ID: u8 = 0;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_GYRO_ID_ICM42605,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_ICM42605,
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

    /// Return the gyro and acc sample rates actually set.
    /// # Errors
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), B::Error> {
        self.write_register(REG_BANK_SEL, 0).await?;
        self.write_register(REG_PWR_MGMT0, PWR_OFF).await?;

        // default reset configuration
        self.write_register(REG_DEVICE_CONFIG, DEVICE_CONFIG_DEFAULT).await?;
        delay_ms(1).await;

        /*let chip_id = self.bus.read_register_with_timeout(REG_WHO_AM_I, 100);
        if chip_id != WHO_AM_I_RESPONSE_ICM42605 && chip_id != WHO_AM_I_RESPONSE_ICM42688P {
            return NOT_DETECTED;
        }*/
        delay_ms(1).await;

        // set AntiAlias filter, see pages 28ff of TDK ICM-42688-P Datasheet
        {
            const GYRO_AAF_DELT: u8 = 38;
            const GYRO_AAF_DELTSQR: u16 = 1440;
            const GYRO_AAF_BITSH: u8 = 4; // gives 3dB Bandwidth of 2029 Hz
            self.write_register(REG_BANK_SEL, 1).await?;
            self.write_register(REG_BANK1_GYRO_CONFIG_STATIC3, GYRO_AAF_DELT).await?;
            self.write_register(REG_BANK1_GYRO_CONFIG_STATIC4, (GYRO_AAF_DELTSQR & 0xFF) as u8).await?;
            self.write_register(REG_BANK1_GYRO_CONFIG_STATIC5, (GYRO_AAF_BITSH << 4) | (GYRO_AAF_DELTSQR >> 8) as u8)
                .await?;
        }
        self.write_register(REG_BANK_SEL, 0).await?;

        // REG_GYRO_CONFIG1 defaults to first order gyro ui filter, 3rd order GYRO_DEC2_M2_ORD filter, so is left unchanged
        self.write_register(REG_GYRO_ACCEL_CONFIG0, ACCEL_FILTER_LOW_LATENCY | GYRO_FILTER_LOW_LATENCY).await?;

        // Configure interrupts
        self.write_register(
            REG_INT_CONFIG,
            INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PUSH_PULL | INT1_POLARITY_ACTIVE_HIGH,
        )
        .await?;
        self.write_register(REG_INT_CONFIG0, INT_CLEAR_ON_STATUS_BIT_READ).await?;

        // set interrupt pulse duration to 8us and disable de-assert duration, both required for ODR >= 4kHz
        self.write_register(REG_INT_CONFIG1, INT_TPULSE_DURATION_8US | INT_TDEASSERT_DISABLE).await?;
        self.write_register(REG_INT_SOURCE0, INT1_UI_DATA_READY_ENABLED).await?;

        // Configure INTF
        self.write_register(REG_INTF_CONFIG0, SENSOR_DATA_LITTLE_ENDIAN | UI_SIFS_CFG_DISABLE_I2C).await?;
        {
            // Disable AFSR to prevent stalls in gyro output (undocumented in datasheet)
            const CONFIG1_DEFAULT_VALUE: u8 = 0b_1001_0001;
            const CONFIG1_AFSR_MASK: u8 = 0b_0011_1111;
            const CONFIG1_AFSR_DISABLE: u8 = 0b_0100_0000;
            //let  intFConfig1 = self.bus.read_register(REG_INTF_CONFIG1);
            //intFConfig1 &= CONFIG1_AFSR_MASK;
            //intFConfig1 |= CONFIG1_AFSR_DISABLE;
            //self.write_register(REG_INTF_CONFIG1, intFConfig1);
            self.write_register(REG_INTF_CONFIG1, (CONFIG1_DEFAULT_VALUE & CONFIG1_AFSR_MASK) | CONFIG1_AFSR_DISABLE)
                .await?;
        }

        // Turn on gyro and acc on before configuring Output Data Rate(ODR) and Full Scale Rate (FSRP)
        self.write_register(REG_PWR_MGMT0, PWR_TEMP_ENABLED | PWR_GYRO_LOW_NOISE | PWR_ACCEL_LOW_NOISE).await?;
        delay_ms(1).await;

        let gyro_register_value =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);

        self.write_register(REG_GYRO_CONFIG0, gyro_register_value).await?;
        delay_ms(1).await;

        let acc_register_value =
            self.calculate_acc_scale_and_odr(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(REG_ACCEL_CONFIG0, acc_register_value).await?;
        delay_ms(1).await;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const GYRO_RANGE_2000_DPS: u8 = 0b_0000_0000;
        const GYRO_RANGE_1000_DPS: u8 = 0b_0010_0000;
        const GYRO_RANGE_500_DPS: u8 = 0b_0100_0000;
        const GYRO_RANGE_250_DPS: u8 = 0b_0110_0000;
        const GYRO_RANGE_125_DPS: u8 = 0b_1000_0000;
        const _GYRO_RANGE_62_P_5_DPS: u8 = 0b_1010_0000;
        const _GYRO_RANGE_31_P_25_DPS: u8 = 0b_1100_0000;
        const _GYRO_RANGE_15_P_625_DPS: u8 = 0b_1110_0000;

        const GYRO_ODR_32000_HZ: u8 = 0b_0000_0001;
        const GYRO_ODR_16000_HZ: u8 = 0b_0000_0010;
        const GYRO_ODR_8000_HZ: u8 = 0b_0000_0011;
        const GYRO_ODR_4000_HZ: u8 = 0b_0000_0100;
        const GYRO_ODR_2000_HZ: u8 = 0b_0000_0101;
        const GYRO_ODR_1000_HZ: u8 = 0b_0000_0110;
        const GYRO_ODR_200_HZ: u8 = 0b_0000_0111;
        const GYRO_ODR_100_HZ: u8 = 0b_0000_1000;
        const GYRO_ODR_50_HZ: u8 = 0b_0000_1001;
        const GYRO_ODR_25_HZ: u8 = 0b_0000_1010;
        const GYRO_ODR_12_P_5_HZ: u8 = 0b_0000_1011;
        const GYRO_ODR_500_HZ: u8 = 0b_0000_1111;

        // calculate the GYRO_RANGE bit values to write to the REG_GYRO_CONFIG0 register
        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            GyroFullScale::Scale125Dps => (125.0 / 32768.0, GYRO_RANGE_125_DPS),
            GyroFullScale::Scale250Dps => (250.0 / 32768.0, GYRO_RANGE_250_DPS),
            GyroFullScale::Scale500Dps => (500.0 / 32768.0, GYRO_RANGE_500_DPS),
            GyroFullScale::Scale1000Dps => (1000.0 / 32768.0, GYRO_RANGE_1000_DPS),
            _ => (2000.0 / 32768.0, GYRO_RANGE_2000_DPS),
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        // calculate the GYRO_ODR bit values to write to the REG_GYRO_CONFIG0 register
        let (gyro_sample_rate_hz, gyro_odr) = match target_output_data_rate_hz {
            8001..=16_000 => (16_000, GYRO_ODR_16000_HZ),
            0 | 4001..=8000 => (8000, GYRO_ODR_8000_HZ),
            2001..=4000 => (4000, GYRO_ODR_4000_HZ),
            1001..=2000 => (2000, GYRO_ODR_2000_HZ),
            501..=1000 => (1000, GYRO_ODR_1000_HZ),
            201..=500 => (500, GYRO_ODR_500_HZ),
            101..=200 => (200, GYRO_ODR_200_HZ),
            51..=100 => (100, GYRO_ODR_100_HZ),
            26..=50 => (50, GYRO_ODR_50_HZ),
            14..=25 => (25, GYRO_ODR_25_HZ),
            1..=13 => (12, GYRO_ODR_12_P_5_HZ),
            _ => (32_000, GYRO_ODR_32000_HZ),
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
        const ACCEL_RANGE_16G: u8 = 0b_0000_0000;
        const ACCEL_RANGE_8G: u8 = 0b_0010_0000;
        const ACCEL_RANGE_4G: u8 = 0b_0100_0000;
        const ACCEL_RANGE_2G: u8 = 0b_0110_0000;

        const ACCEL_ODR_32000_HZ: u8 = 0b_0000_0001;
        const ACCEL_ODR_16000_HZ: u8 = 0b_0000_0010;
        const ACCEL_ODR_8000_HZ: u8 = 0b_0000_0011;
        const ACCEL_ODR_4000_HZ: u8 = 0b_0000_0100;
        const ACCEL_ODR_2000_HZ: u8 = 0b_0000_0101;
        const ACCEL_ODR_1000_HZ: u8 = 0b_0000_0110;
        const ACCEL_ODR_200_HZ: u8 = 0b_0000_0111;
        const ACCEL_ODR_100_HZ: u8 = 0b_0000_1000;
        const ACCEL_ODR_50_HZ: u8 = 0b_0000_1001;
        const ACCEL_ODR_25_HZ: u8 = 0b_0000_1010;
        const ACCEL_ODR_12_P_5_HZ: u8 = 0b_0000_1011;
        const ACCEL_ODR_500_HZ: u8 = 0b_0000_1111;

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
            8001..=16_000 => (16_000, ACCEL_ODR_16000_HZ),
            0 | 4001..=8000 => (8000, ACCEL_ODR_8000_HZ),
            2001..=4000 => (4000, ACCEL_ODR_4000_HZ),
            1001..=2000 => (2000, ACCEL_ODR_2000_HZ),
            501..=1000 => (1000, ACCEL_ODR_1000_HZ),
            201..=500 => (500, ACCEL_ODR_500_HZ),
            101..=200 => (200, ACCEL_ODR_200_HZ),
            51..=100 => (100, ACCEL_ODR_100_HZ),
            26..=50 => (50, ACCEL_ODR_50_HZ),
            14..=25 => (25, ACCEL_ODR_25_HZ),
            1..=13 => (12, ACCEL_ODR_12_P_5_HZ),
            _ => (32_000, ACCEL_ODR_32000_HZ),
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

    impl<B: ImuBus> Imu426xx<B> {
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
        let mut imu: Imu426xx<MockImuBus> = Imu426xx::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_odr, acc_odr) = result.unwrap();

        assert_eq!(8000, gyro_odr);
        assert_eq!(8000, acc_odr);
        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(8000, imu.common.gyro_sample_rate_hz);
        assert_eq!(8000, imu.common.acc_sample_rate_hz);
    }
}
