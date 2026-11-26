use embassy_time::Instant;
use vqm::Vector3f32;

use super::{
    AccSensitivity, AccUnits, AxisOrder, GyroSensitivity, GyroUnits, Imu, ImuBus, ImuCommon, ImuDevice,
    ImuDeviceConfig, ImuError, Vector3Ext,
};

/// Common driver for LSM6DS3TR-C, ISM330DHCX, and LSM6DSOX IMUs.
/// Maximum output data rate (ODR) is 6664Hz.
/// Maximum gyro range is +/- 2000 DPS.
/// Maximum acc range is +/- 16G.
#[derive(Debug, PartialEq)]
pub struct Lsm6ds<B: ImuBus> {
    bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

const I2C_ADDRESS: u8 = 0x6A;
const I2C_ADDRESS_ALTERNATIVE: u8 = 0x6B;

/// IMU Registers.
struct Reg;

impl Reg {
    const _RESERVED_00: u8 = 0x00;
    const _FUNC_CFG_ACCESS: u8 = 0x01;
    const _RESERVED_03: u8 = 0x03;

    const INT1_CTRL: u8 = 0x0D;
    const INT2_CTRL: u8 = 0x0E;
    const WHO_AM_I: u8 = 0x0F;
    const CTRL1_XL: u8 = 0x10;
    const CTRL2_G: u8 = 0x11;
    const CTRL3_C: u8 = 0x12;
    const _CTRL4_C: u8 = 0x13;
    const _CTRL5_C: u8 = 0x14;
    const _CTRL6_C: u8 = 0x15;
    const _LPF1_LO: u8 = 0x02;
    const _LPF1_HI: u8 = 0x03;
    const _CTRL7_G: u8 = 0x16;
    const _CTRL8_XL: u8 = 0x17;
    const _CTRL9_XL: u8 = 0x18;
    const _CTRL10_C: u8 = 0x19;
    const _WAKE_UP_SRC: u8 = 0x1B;
    const _TAP_SRC: u8 = 0x1C;
    const _D6D_SRC: u8 = 0x1D;
    const _STATUS_REG: u8 = 0x1E;
    const _RESERVED_1F: u8 = 0x1F;

    const _OUT_TEMP_L: u8 = 0x20;
    const _OUT_TEMP_H: u8 = 0x21;

    const OUTX_L_G: u8 = 0x22;
    const _OUTX_H_G: u8 = 0x23;
    const _OUTY_L_G: u8 = 0x24;
    const _OUTY_H_G: u8 = 0x25;
    const _OUTZ_L_G: u8 = 0x26;
    const _OUTZ_H_G: u8 = 0x27;

    const OUTX_L_ACC: u8 = 0x28;
    const _OUTX_H_ACC: u8 = 0x29;
    const _OUTY_L_ACC: u8 = 0x2A;
    const _OUTY_H_ACC: u8 = 0x2B;
    const _OUTZ_L_ACC: u8 = 0x2C;
    const _OUTZ_H_ACC: u8 = 0x2D;
}

struct RegLsm6ds3trC;
impl RegLsm6ds3trC {
    const _RESERVED_02: u8 = 0x02;
    const _SENSOR_SYNC_TIME_FRAME: u8 = 0x04;
    const _SENSOR_SYNC_RES_RATIO: u8 = 0x05;
    const _FIFO_CTRL1: u8 = 0x06;
    const _FIFO_CTRL2: u8 = 0x07;
    const _FIFO_CTRL3: u8 = 0x08;
    const _FIFO_CTRL4: u8 = 0x09;
    const _FIFO_CTRL5: u8 = 0x0A;
    const DRDY_PULSE_CFG_G: u8 = 0x0B;
    const _RESERVED_0C: u8 = 0x0C;
    const _MASTER_CONFIG: u8 = 0x1A;
}

struct RegIsm330dhcx;
impl RegIsm330dhcx {
    const _PIN_CTRL: u8 = 0x02;
    const _RESERVED_04: u8 = 0x04;
    const _RESERVED_05: u8 = 0x05;
    const _RESERVED_06: u8 = 0x06;
    const _FIFO_CTRL1: u8 = 0x07;
    const _FIFO_CTRL2: u8 = 0x08;
    const _FIFO_CTRL3: u8 = 0x09;
    const _FIFO_CTRL4: u8 = 0x0A;
    const COUNTER_BDR_REG1: u8 = 0x0B;
    const _COUNTER_BDR_REG2: u8 = 0x0C;
    const _ALL_INT_SRC: u8 = 0x1A;
}

struct RegLsm6dsox;
impl RegLsm6dsox {
    const _PIN_CTRL: u8 = 0x02;
    const _S4S_TPH_L: u8 = 0x04;
    const _S4S_TPH_H: u8 = 0x05;
    const _S4S_RR: u8 = 0x06;
    const _FIFO_CTRL1: u8 = 0x07;
    const _FIFO_CTRL2: u8 = 0x08;
    const _FIFO_CTRL3: u8 = 0x09;
    const _FIFO_CTRL4: u8 = 0x0A;
    const COUNTER_BDR_REG1: u8 = 0x0B;
    const _COUNTER_BDR_REG2: u8 = 0x0C;
    const _ALL_INT_SRC: u8 = 0x1A;
}

// IMU register bitflags
const INT1_DRDY_G: u8 = 0b_0000_0010;
const INT2_DRDY_G: u8 = 0b_0000_0010;
const BDU: u8 = 0b_0100_0000;
const IF_INC: u8 = 0b_0000_0100;
const SW_RESET: u8 = 0b_0000_0001;
const _I2C_DISABLE: u8 = 0b_0000_0100;
const _LPF1_SEL_G: u8 = 0b_0000_0010;
const _XL_HM_MODE_DISABLE: u8 = 0b_0001_0000;
const _LPF1_MEDIUM_HI: u8 = 0x00;
const _LPF1_MEDIUM_LO: u8 = 0x01;

const _WHO_AM_I_RESPONSE_LSM6DS3: u8 = 0x69;
const _WHO_AM_I_RESPONSE_LSM6DS33: u8 = 0x69;

const WHO_AM_I_RESPONSE_LSM6DS3TR_C: u8 = 0x6A;
const _WHO_AM_I_RESPONSE_LSM6DSM: u8 = 0x6A;

const WHO_AM_I_RESPONSE_ISM330DHCX: u8 = 0x6B;

const WHO_AM_I_RESPONSE_LSM6DSOX: u8 = 0x6C;
const _WHO_AM_I_RESPONSE_LSM6DSO: u8 = 0x6C;
const _WHO_AM_I_RESPONSE_LSM6DSO32: u8 = 0x6C;
const _WHO_AM_I_RESPONSE_LSM6DSV: u8 = 0x70;
const _WHO_AM_I_RESPONSE_LSM6DSV320X: u8 = 0x73;

impl<B: ImuBus> ImuDevice for Lsm6ds<B> {
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), ImuError> {
        Lsm6ds::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }
}

impl<B: ImuBus> Imu for Lsm6ds<B> {
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

    async fn read_acc(&mut self) -> Result<(Vector3f32, Instant), ImuError> {
        let mut buf = [0u8; 6];
        self.write_read(&[Reg::OUTX_L_ACC], &mut buf).await?;
        let timestamp = Instant::now();

        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok((self.common.map_vector(acc), timestamp))
    }

    async fn read_gyro(&mut self) -> Result<(Vector3f32, Instant), ImuError> {
        let mut buf = [0u8; 6];
        self.write_read(&[Reg::OUTX_L_G], &mut buf).await?;
        let timestamp = Instant::now();

        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok((self.common.map_vector(gyro), timestamp))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32, Instant), ImuError> {
        let mut buf = [0u8; 12];
        self.write_read(&[Reg::OUTX_L_G], &mut buf).await?;
        let timestamp = Instant::now();

        let [g0, g1, g2, g3, g4, g5, a0, a1, a2, a3, a4, a5] = buf;

        let acc_buf = [a0, a1, a2, a3, a4, a5];
        let gyro_buf = [g0, g1, g2, g3, g4, g5];
        let acc = Vector3f32::from_le_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_le_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;
        let (acc, gyro) = self.common.map_acc_gyro(acc, gyro);
        Ok((acc, gyro, timestamp))
    }
}

impl<B: ImuBus> Lsm6ds<B> {
    const DEVICE_ID_UNSET: u8 = 0;

    /// Constructor.
    pub fn new(bus: B, axis_order: AxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_GYRO_ID_NONE,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_NONE,
                axis_order,
                device_id: Self::DEVICE_ID_UNSET,
                address: I2C_ADDRESS,
                flags: 0,
            },
        }
    }

    /// Set a newly constructed `Lsm6ds` to use it's alternative I2C address.
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

    /// Initialize IMU by resetting and setting registers.
    /// Returns `(gyro_sample_rate_hz, acc_sample_rate_hz)`.
    /// # Errors
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), ImuError> {
        const RESET_TIMEOUT_MS: u32 = 10;
        const RESET_POLL_INTERVAL_MS: u32 = 1;

        // Check WhoAmI
        let chip_id = self.read_register(Reg::WHO_AM_I).await?;
        match chip_id {
            WHO_AM_I_RESPONSE_LSM6DS3TR_C => {
                self.config.device_id = chip_id;
                self.config.gyro_id_msp = ImuDeviceConfig::MSP_GYRO_ID_LSM6DS3TR;
                self.config.acc_id_msp = ImuDeviceConfig::MSP_ACC_ID_LSM6DS3TR;
            }
            WHO_AM_I_RESPONSE_ISM330DHCX => {
                self.config.device_id = chip_id;
                self.config.gyro_id_msp = ImuDeviceConfig::MSP_GYRO_ID_ISM330DHCX;
                self.config.acc_id_msp = ImuDeviceConfig::MSP_ACC_ID_ISM330DHCX;
            }
            WHO_AM_I_RESPONSE_LSM6DSOX => {
                self.config.device_id = chip_id;
                self.config.gyro_id_msp = ImuDeviceConfig::MSP_GYRO_ID_LSM6DSO;
                self.config.acc_id_msp = ImuDeviceConfig::MSP_ACC_ID_LSM6DSO;
            }
            _ => return Err(ImuError::IncorrectWhoAmI),
        }
        // Software reset
        self.write_register(Reg::CTRL3_C, SW_RESET).await?;
        let mut reset_complete = false;
        for _ in 0..RESET_TIMEOUT_MS {
            ImuCommon::delay_ms(RESET_POLL_INTERVAL_MS).await;
            let ctrl3_c = self.read_register(Reg::CTRL3_C).await?;
            if ctrl3_c & SW_RESET == 0 {
                reset_complete = true;
                break;
            }
        }
        _ = reset_complete;
        #[cfg(not(test))]
        if !reset_complete {
            return Err(ImuError::InitializationError);
        }

        // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
        self.write_register(Reg::CTRL3_C, BDU | IF_INC).await?;
        ImuCommon::delay_ms(1).await;

        let reg_ctrl2_g = self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);
        self.write_register(Reg::CTRL2_G, reg_ctrl2_g).await?;
        ImuCommon::delay_ms(1).await;

        let reg_ctrl1_xl = self.calculate_acc_scale_and_odr(acc_sensitivity, acc_units, target_output_data_rate_hz);
        self.write_register(Reg::CTRL1_XL, reg_ctrl1_xl).await?;

        if chip_id == WHO_AM_I_RESPONSE_LSM6DS3TR_C {
            // Set data ready pulsed
            const DATA_READY_PULSED: u8 = 0b_1000_0000;
            self.write_register(RegLsm6ds3trC::DRDY_PULSE_CFG_G, DATA_READY_PULSED).await?;
            ImuCommon::delay_ms(1).await;
        } else if chip_id == WHO_AM_I_RESPONSE_ISM330DHCX {
            const DATA_READY_PULSED: u8 = 0b_1000_0000;

            self.write_register(RegIsm330dhcx::COUNTER_BDR_REG1, DATA_READY_PULSED).await?;
        } else if chip_id == WHO_AM_I_RESPONSE_LSM6DSOX {
            const DATA_READY_PULSED: u8 = 0b_1000_0000;

            self.write_register(RegLsm6dsox::COUNTER_BDR_REG1, DATA_READY_PULSED).await?;
        }

        // Enable interrupts now IMU is configured.
        // We deliberately only generate an interrupt when the gyro has data ready:
        //     acc and gyro are read simultaneously when there is gyro data ready,
        //     the latest available acc value is read.
        // Interrupt pins are by default forced to ground, so active high
        // Enable data ready on INT1 pin
        self.write_register(Reg::INT1_CTRL, INT1_DRDY_G).await?;
        ImuCommon::delay_ms(1).await;

        // Enable data ready on INT2 pin
        self.write_register(Reg::INT2_CTRL, INT2_DRDY_G).await?;
        ImuCommon::delay_ms(1).await;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    /// Calculate and set `gyro_scale` and `gyro_sample_rate` given the requested gyro sensitivity, units, and ODR.
    /// Targets are rounded upwards.
    /// Values greater than the maximum supported result in setting maximum supported value.
    ///
    /// Returns the value needed to set the `CTRL2_G` register.
    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const GYRO_RANGE_125_DPS: u8 = 0b_0010;
        const _GYRO_RANGE_245_DPS: u8 = 0b_0000; // LSM6DS3TR_C
        const GYRO_RANGE_250_DPS: u8 = 0b_0000; // ISM330DHCX, LSM6DSOX
        const GYRO_RANGE_500_DPS: u8 = 0b_0100;
        const GYRO_RANGE_1000_DPS: u8 = 0b_1000;
        const GYRO_RANGE_2000_DPS: u8 = 0b_1100;

        // const GYRO_ODR_12P5_HZ: u8 = 0b_0001_0000; not supported
        const GYRO_ODR_26_HZ: u8 = 0b_0010_0000;
        const GYRO_ODR_52_HZ: u8 = 0b_0011_0000;
        const GYRO_ODR_104_HZ: u8 = 0b_0100_0000;
        const GYRO_ODR_208_HZ: u8 = 0b_0101_0000; // corrected: was 0b_010100000 (9 bits)
        const GYRO_ODR_416_HZ: u8 = 0b_0110_0000;
        const GYRO_ODR_833_HZ: u8 = 0b_0111_0000;
        const GYRO_ODR_1666_HZ: u8 = 0b_1000_0000;
        const GYRO_ODR_3332_HZ: u8 = 0b_1001_0000;
        const GYRO_ODR_6664_HZ: u8 = 0b_1010_0000;

        let (scale_dps, gyro_range) = match gyro_sensitivity {
            GyroSensitivity::Scale125Dps => (0.004_375, GYRO_RANGE_125_DPS),
            GyroSensitivity::Scale250Dps => (0.008_75, GYRO_RANGE_250_DPS),
            GyroSensitivity::Scale500Dps => (0.017_5, GYRO_RANGE_500_DPS),
            GyroSensitivity::Scale1000Dps => (0.035, GYRO_RANGE_1000_DPS),
            _ => (0.070, GYRO_RANGE_2000_DPS),
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
            0..=26 => (26, GYRO_ODR_26_HZ),
            _ => (6664, GYRO_ODR_6664_HZ),
        };
        self.common.gyro_sample_rate_hz = gyro_sample_rate_hz;

        gyro_range | gyro_odr
    }

    /// Calculate and set `acc_scale` and `acc_sample_rate` given the requested acc sensitivity, units, and ODR.
    /// Targets are rounded upwards.
    /// Values greater than the maximum supported result in setting maximum supported value.
    ///
    /// Returns the value needed to set the `CTRL1_XL` register.
    pub fn calculate_acc_scale_and_odr(
        &mut self,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        const ACC_RANGE_2G: u8 = 0b_0000;
        const ACC_RANGE_4G: u8 = 0b_1000;
        const ACC_RANGE_8G: u8 = 0b_1100;
        const ACC_RANGE_16G: u8 = 0b_0100;
        // const ACC_ODR_12P5_HZ: u8 = 0b_0001_0000; not supported
        const ACC_ODR_26_HZ: u8 = 0b_0010_0000;
        const ACC_ODR_52_HZ: u8 = 0b_0011_0000;
        const ACC_ODR_104_HZ: u8 = 0b_0100_0000;
        const ACC_ODR_208_HZ: u8 = 0b_0101_0000; // corrected: was 0b_010100000 (9 bits)
        const ACC_ODR_416_HZ: u8 = 0b_0110_0000;
        const ACC_ODR_833_HZ: u8 = 0b_0111_0000;
        const ACC_ODR_1666_HZ: u8 = 0b_1000_0000;
        const ACC_ODR_3332_HZ: u8 = 0b_1001_0000;
        const ACC_ODR_6664_HZ: u8 = 0b_1010_0000;

        let (scale, acc_scale) = match acc_sensitivity {
            AccSensitivity::Scale2G => (0.000_061, ACC_RANGE_2G),
            AccSensitivity::Scale4G => (0.000_122, ACC_RANGE_4G),
            AccSensitivity::Scale8G => (0.000_244, ACC_RANGE_8G),
            // Default to maximum range.
            _ => {
                // default includes  AccFullScale::Scale16G
                (0.000_488, ACC_RANGE_16G)
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
            0..=26 => (26, ACC_ODR_26_HZ),
            _ => (6664, ACC_ODR_6664_HZ),
        };
        self.common.acc_sample_rate_hz = acc_sample_rate_hz;

        acc_scale | acc_odr
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AxisOrder, MockImuBus};

    #[test]
    fn imu_init() {
        const WHO_AM_I: u8 = WHO_AM_I_RESPONSE_LSM6DS3TR_C;
        const DATA_READY_PULSED: u8 = 0b_1000_0000;

        let imu_bus = MockImuBus::new().with_register(Reg::WHO_AM_I, WHO_AM_I).with_register(Reg::CTRL3_C, 4);
        assert_eq!(4, imu_bus.registers[Reg::CTRL3_C as usize]);
        assert_eq!(WHO_AM_I, imu_bus.registers[Reg::WHO_AM_I as usize]);

        let mut imu: Lsm6ds<MockImuBus> = Lsm6ds::new(imu_bus, AxisOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.read_register(Reg::WHO_AM_I));
        assert_eq!(Ok(WHO_AM_I), result);

        let reg = imu.calculate_acc_scale_and_odr(AccSensitivity::Max, AccUnits::Mps2, 8000);
        assert_eq!(reg, 0xA4);
        assert!((imu.common.acc_scale - 0.004_785_645_3).abs() < 1e-6);
        assert_eq!(imu.common.acc_sample_rate_hz, 6664);

        let reg = imu.calculate_gyro_scale_and_odr(GyroSensitivity::Max, GyroUnits::Rps, 8000);
        assert_eq!(reg, 0xAC);
        assert!((imu.common.gyro_scale - 0.001_221_730_5).abs() < 1e-6);
        assert_eq!(imu.common.gyro_sample_rate_hz, 6664);

        let result =
            pollster::block_on(imu.init(8000, GyroSensitivity::Max, GyroUnits::Dps, AccSensitivity::Max, AccUnits::G));
        assert_eq!(Ok((6664, 6664)), result);

        let reg = pollster::block_on(imu.read_register(Reg::CTRL3_C));
        assert!(reg.is_ok());
        assert_eq!(Ok(BDU | IF_INC), reg);

        let reg = pollster::block_on(imu.read_register(RegLsm6ds3trC::DRDY_PULSE_CFG_G));
        assert!(reg.is_ok());
        assert_eq!(Ok(DATA_READY_PULSED), reg);

        assert!((0.07 - imu.common.gyro_scale) < 1e-6);
        assert!((0.000_244 - imu.common.acc_scale) < 1e-6);
        assert_eq!(6664, imu.common.gyro_sample_rate_hz);
        assert_eq!(6664, imu.common.acc_sample_rate_hz);
    }

    #[test]
    fn test_acc_sensitivity() {
        let cases = [
            (AccSensitivity::Scale2G, 0.000_061, 0b0000),
            (AccSensitivity::Scale4G, 0.000_122, 0b1000),
            (AccSensitivity::Scale8G, 0.000_244, 0b1100),
            (AccSensitivity::Scale16G, 0.000_488, 0b0100),
        ];

        for (sensitivity, expected_scale, expected_range) in cases {
            let bus = MockImuBus::new();
            let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

            let reg = imu.calculate_acc_scale_and_odr(sensitivity, AccUnits::G, 8000);
            assert_eq!(reg & 0x0C, expected_range);
            assert!(
                (imu.common.acc_scale - expected_scale).abs() < 1e-9,
                "Incorrect scale for {:?}: expected {}, got {}",
                sensitivity,
                expected_scale,
                imu.common.acc_scale,
            );
        }
    }

    #[test]
    fn test_acc_sensitivity_mps2() {
        let bus = MockImuBus::new();
        let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

        _ = imu.calculate_acc_scale_and_odr(AccSensitivity::Scale16G, AccUnits::Mps2, 104);
        let expected = 0.000_488 * ImuCommon::G0;
        assert!((imu.common.acc_scale - expected).abs() < 1e-7, "Expected {}, got {}", expected, imu.common.acc_scale);
    }

    #[test]
    fn test_gyro_sensitivity() {
        let cases = [
            (GyroSensitivity::Scale125Dps, 0.004_375, 0b0010),
            (GyroSensitivity::Scale250Dps, 0.008_750, 0b0000),
            (GyroSensitivity::Scale500Dps, 0.017_500, 0b0100),
            (GyroSensitivity::Scale1000Dps, 0.035_000, 0b1000),
            (GyroSensitivity::Scale2000Dps, 0.070_000, 0b1100),
        ];

        for (sensitivity, expected_scale, expected_range) in cases {
            let bus = MockImuBus::new();
            let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

            let reg = imu.calculate_gyro_scale_and_odr(sensitivity, GyroUnits::Dps, 8000);
            assert_eq!(reg & 0x0F, expected_range);
            assert!(
                (imu.common.gyro_scale - expected_scale).abs() < 1e-9,
                "Incorrect scale for {:?}: expected {}, got {}",
                sensitivity,
                expected_scale,
                imu.common.gyro_scale,
            );
        }
    }

    #[test]
    fn test_gyro_sensitivity_radians() {
        let bus = MockImuBus::new();
        let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

        _ = imu.calculate_gyro_scale_and_odr(GyroSensitivity::Scale2000Dps, GyroUnits::Rps, 104);
        let expected = 0.070_f32.to_radians();
        assert!(
            (imu.common.gyro_scale - expected).abs() < 1e-7,
            "Expected {}, got {}",
            expected,
            imu.common.gyro_scale,
        );
    }
    #[test]
    fn test_acc_odr_boundaries() {
        let cases = [
            (0, 26, 0x20),
            (12, 26, 0x20),
            (13, 26, 0x20),
            (26, 26, 0x20),
            (27, 52, 0x30),
            (52, 52, 0x30),
            (53, 104, 0x40),
            (104, 104, 0x40),
            (105, 208, 0x50),
            (208, 208, 0x50),
            (209, 416, 0x60),
            (416, 416, 0x60),
            (417, 833, 0x70),
            (833, 833, 0x70),
            (834, 1666, 0x80),
            (1666, 1666, 0x80),
            (1667, 3332, 0x90),
            (3332, 3332, 0x90),
            (3333, 6664, 0xA0),
            (8000, 6664, 0xA0),
        ];

        for (requested, expected_rate, expected_register) in cases {
            let bus = MockImuBus::new();
            let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

            let reg = imu.calculate_acc_scale_and_odr(AccSensitivity::Scale2G, AccUnits::G, requested);
            assert_eq!(reg & 0xF0, expected_register, "Wrong register for requested ODR {requested}");
            assert_eq!(
                imu.common.acc_sample_rate_hz, expected_rate,
                "Wrong reported ODR for requested ODR {requested}",
            );
        }
    }

    #[test]
    fn test_gyro_odr_boundaries() {
        let cases = [
            (0, 26, 0x20),
            (1, 26, 0x20),
            (12, 26, 0x20),
            (13, 26, 0x20),
            (26, 26, 0x20),
            (27, 52, 0x30),
            (52, 52, 0x30),
            (53, 104, 0x40),
            (104, 104, 0x40),
            (105, 208, 0x50),
            (208, 208, 0x50),
            (209, 416, 0x60),
            (416, 416, 0x60),
            (417, 833, 0x70),
            (833, 833, 0x70),
            (834, 1666, 0x80),
            (1666, 1666, 0x80),
            (1667, 3332, 0x90),
            (3332, 3332, 0x90),
            (3333, 6664, 0xA0),
            (8000, 6664, 0xA0),
        ];

        for (requested, expected_rate, expected_register) in cases {
            let bus = MockImuBus::new();
            let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

            let reg = imu.calculate_gyro_scale_and_odr(GyroSensitivity::Scale125Dps, GyroUnits::Dps, requested);
            assert_eq!(reg & 0xF0, expected_register);
            assert_eq!(
                imu.common.gyro_sample_rate_hz, expected_rate,
                "Wrong reported ODR for requested ODR {requested}"
            );
        }
    }
    #[test]
    fn test_device_identification() {
        let cases = [
            (
                WHO_AM_I_RESPONSE_LSM6DS3TR_C,
                ImuDeviceConfig::MSP_GYRO_ID_LSM6DS3TR,
                ImuDeviceConfig::MSP_ACC_ID_LSM6DS3TR,
            ),
            (
                WHO_AM_I_RESPONSE_ISM330DHCX,
                ImuDeviceConfig::MSP_GYRO_ID_ISM330DHCX,
                ImuDeviceConfig::MSP_ACC_ID_ISM330DHCX,
            ),
            (WHO_AM_I_RESPONSE_LSM6DSOX, ImuDeviceConfig::MSP_GYRO_ID_LSM6DSO, ImuDeviceConfig::MSP_ACC_ID_LSM6DSO),
        ];

        for (device_id, expected_gyro_id, expected_acc_id) in cases {
            let bus = MockImuBus::new().with_register(Reg::WHO_AM_I, device_id);

            let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

            // This test assumes the mock clears SW_RESET when CTRL3_C
            // is written with SW_RESET. See the mock enhancement below.
            let result = pollster::block_on(imu.init(
                104,
                GyroSensitivity::Scale2000Dps,
                GyroUnits::Dps,
                AccSensitivity::Scale16G,
                AccUnits::G,
            ));
            assert!(result.is_ok(), "Failed for device ID {device_id:#04x}");

            assert_eq!(imu.config.device_id, device_id);
            assert_eq!(imu.config.gyro_id_msp, expected_gyro_id);
            assert_eq!(imu.config.acc_id_msp, expected_acc_id);
        }
    }

    #[test]
    fn test_incorrect_who_am_i() {
        let bus = MockImuBus::new().with_register(Reg::WHO_AM_I, 0xFF);

        let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(
            104,
            GyroSensitivity::Scale2000Dps,
            GyroUnits::Dps,
            AccSensitivity::Scale16G,
            AccUnits::G,
        ));
        assert_eq!(result, Err(ImuError::IncorrectWhoAmI));
    }

    #[test]
    fn test_init_register_configuration() {
        let bus = MockImuBus::new().with_register(Reg::WHO_AM_I, WHO_AM_I_RESPONSE_LSM6DS3TR_C);

        let mut imu = Lsm6ds::new(bus, AxisOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(
            104,
            GyroSensitivity::Scale500Dps,
            GyroUnits::Dps,
            AccSensitivity::Scale4G,
            AccUnits::G,
        ));
        assert_eq!(result, Ok((104, 104)));

        let ctrl3 = pollster::block_on(imu.read_register(Reg::CTRL3_C));
        assert_eq!(ctrl3, Ok(BDU | IF_INC));

        let ctrl2 = pollster::block_on(imu.read_register(Reg::CTRL2_G));
        assert_eq!(ctrl2, Ok(0x44));

        let ctrl1 = pollster::block_on(imu.read_register(Reg::CTRL1_XL));
        assert_eq!(ctrl1, Ok(0x48));

        let int1 = pollster::block_on(imu.read_register(Reg::INT1_CTRL));
        assert_eq!(int1, Ok(INT1_DRDY_G));

        let int2 = pollster::block_on(imu.read_register(Reg::INT2_CTRL));
        assert_eq!(int2, Ok(INT2_DRDY_G));
    }
}
