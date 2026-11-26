use embassy_time::Instant;
use vqm::Vector3f32;

use super::{
    AccDevice, AccSensitivity, AccUnits, AxisOrder, Imu, ImuBus, ImuCommon, ImuDeviceConfig, ImuError, Vector3Ext,
};

const I2C_ADDRESS: u8 = 0x3A;
const I2C_ADDRESS_ALTERNATIVE: u8 = 0xA6;

/// IMU Registers.
struct Reg;

impl Reg {
    // 0x01 to 0x1C 1 to 28 Reserved N/A N/A Reserved; do not access
    const DEVID: u8 = 0x00; // R 11100101 Device ID
    const _THRESH_SHOCK: u8 = 0x1D; // // R/W 00000000 Shock threshold
    const _OFSX: u8 = 0x1E; // R/W 00000000 X-axis offset
    const _OFSY: u8 = 0x1F; // R/W 00000000 Y-axis offset
    const _OFSZ: u8 = 0x20; // R/W 00000000 Z-axis offset
    const _DUR: u8 = 0x21; // R/W 00000000 Shock duration
    const _LATENT: u8 = 0x22; //  R/W 00000000 Shock latency
    const _WINDOW: u8 = 0x23; //  R/W 00000000 Shock window
    const _THRESH_ACT: u8 = 0x24; // R/W 00000000 Activity threshold
    const _THRESH_INACT: u8 = 0x25; // R/W 00000000 Inactivity threshold
    const _TIME_INACT: u8 = 0x26; // R/W 00000000 Inactivity time
    const _ACT_INACT_CTL: u8 = 0x27; // R/W 00000000 Axis enable control for activity and inactivity detection
    const _SHOCK_AXES: u8 = 0x2A; // R/W 00000000 Axis control for single shock/double shock
    const _ACT_SHOCK_STATUS: u8 = 0x2B; // R 00000000 Source of single shock/double shock
    const BW_RATE: u8 = 0x2C; // R/W 00001010 Data rate and power mode control
    const POWER_CTL: u8 = 0x2D; // R/W 00000000 Power saving features control
    const INT_ENABLE: u8 = 0x2E; // R/W 00000000 Interrupt enable control
    const INT_MAP: u8 = 0x2F; // R/W 00000000 Interrupt mapping control
    const _INT_SOURCE: u8 = 0x30; // R 00000010 Interrupt source
    // Interrupts are by default active high bit D5, inverts this.
    const _DATA_FORMAT: u8 = 0x31; // R/W 00000000 Data format control
    const DATAX0: u8 = 0x32; // R 00000000 X-Axis Data 0
    const _DATAX1: u8 = 0x33; // R 00000000 X-Axis Data 1
    const _DATAY0: u8 = 0x34; // R 00000000 Y-Axis Data 0
    const _DATAY1: u8 = 0x35; // R 00000000 Y-Axis Data 1
    const _DATAZ0: u8 = 0x36; // R 00000000 Z-Axis Data 0
    const _DATAZ1: u8 = 0x37; // R 00000000 Z-Axis Data 1
    const _FIFO_CTL: u8 = 0x38; // R/W 00000000 FIFO control, default is FIFO is bypassed
    const _FIFO_STATUS: u8 = 0x39; // R 000
}

const WHO_AM_I_RESPONSE_ADXL375: u8 = 0b_1110_0101; // 0xE5

#[derive(Debug, PartialEq)]
pub struct Adxl375<B: ImuBus> {
    bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> AccDevice for Adxl375<B> {
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        _acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<u32, ImuError> {
        Adxl375::init(self, target_output_data_rate_hz, acc_units).await
    }

    async fn read_acc(&mut self) -> Result<(Vector3f32, Instant), ImuError> {
        <Self as Imu>::read_acc(self).await
    }
}

impl<B: ImuBus> Imu for Adxl375<B> {
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
        self.write_read(&[Reg::DATAX0], &mut buf).await?;
        let timestamp = Instant::now();

        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok((self.common.map_vector(acc), timestamp))
    }

    async fn read_gyro(&mut self) -> Result<(Vector3f32, Instant), ImuError> {
        core::future::ready(()).await;
        Err(ImuError::GyroNotAvailable)
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32, Instant), ImuError> {
        core::future::ready(()).await;
        Err(ImuError::GyroNotAvailable)
    }
}

impl<B: ImuBus> Adxl375<B> {
    pub const MAX_SPI_FREQUENCY_HZ: u32 = 5_000_000;
    const DEVICE_ID: u8 = WHO_AM_I_RESPONSE_ADXL375;

    /// Constructor.
    pub fn new(bus: B, axis_order: AxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_ACC_ID_NONE,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_DEFAULT,
                axis_order,
                device_id: Self::DEVICE_ID,
                address: I2C_ADDRESS,
                flags: 0,
            },
        }
    }

    /// Set a newly constructed `Adxl375` to use its alternative I2C address.
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
    pub async fn init(&mut self, target_output_data_rate_hz: u32, acc_units: AccUnits) -> Result<u32, ImuError> {
        // Check WhoAmI
        let chip_id = self.read_register(Reg::DEVID).await?;
        if chip_id != WHO_AM_I_RESPONSE_ADXL375 {
            return Err(ImuError::IncorrectWhoAmI);
        }
        ImuCommon::delay_ms(1).await;

        // Place in standby mode, need to do this before setting to measurement mode.
        self.write_register(Reg::POWER_CTL, 0).await?;
        ImuCommon::delay_ms(1).await;
        // Set to measurement mode.
        self.write_register(Reg::POWER_CTL, 0b_0000_1000).await?;
        ImuCommon::delay_ms(1).await;

        // Set to use INT1 pin.
        self.write_register(Reg::INT_MAP, 0b_0000_0000).await?;

        // Enable data ready interrupt.
        self.write_register(Reg::INT_ENABLE, 0b_1000_0000).await?;
        ImuCommon::delay_ms(1).await;

        // const INT_SOURCE: u8 = 0x30; // R 00000010 Interrupt source
        // const DATA_FORMAT: u8 = 0x31; // R/W 00000000 Data format control

        let acc_register_value = self.calculate_acc_scale_and_odr(acc_units, target_output_data_rate_hz);
        self.write_register(Reg::BW_RATE, acc_register_value).await?;
        ImuCommon::delay_ms(1).await;

        // return the acc sample rate actually set
        Ok(self.common.acc_sample_rate_hz)
    }

    pub fn calculate_acc_scale_and_odr(&mut self, acc_units: AccUnits, target_output_data_rate_hz: u32) -> u8 {
        //const ACC_ODR_12P5_HZ: u8 = 0b_0000_0111;
        const ACC_ODR_25_HZ: u8 = 0b_0000_1000;
        const ACC_ODR_50_HZ: u8 = 0b_0000_1001;
        const ACC_ODR_100_HZ: u8 = 0b_0000_1010;
        const ACC_ODR_200_HZ: u8 = 0b_0000_1011;
        const ACC_ODR_400_HZ: u8 = 0b_0000_1100;
        const ACC_ODR_800_HZ: u8 = 0b_0000_1101;
        // Above 800Hz ODR the LSB of the output data is always set to zero (ie we only have 8-bit resolution in total).
        const ACC_ODR_1600_HZ: u8 = 0b_0000_1110;
        const ACC_ODR_3200_HZ: u8 = 0b_0000_1111;

        // Scale is fixed at max value 200G.
        let scale = 200.0 / 32768.0;
        self.common.acc_scale = if acc_units == AccUnits::G { scale } else { scale * ImuCommon::G0 };

        let (acc_sample_rate_hz, acc_odr) = match target_output_data_rate_hz {
            801..=1600 => (1600, ACC_ODR_1600_HZ),
            401..=800 => (800, ACC_ODR_800_HZ),
            201..=400 => (400, ACC_ODR_400_HZ),
            101..=200 => (200, ACC_ODR_200_HZ),
            51..=100 => (100, ACC_ODR_100_HZ),
            26..=50 => (50, ACC_ODR_50_HZ),
            0..=25 => (25, ACC_ODR_25_HZ),
            _ => (3200, ACC_ODR_3200_HZ),
        };
        self.common.acc_sample_rate_hz = acc_sample_rate_hz;

        acc_odr
    }
}

#[cfg(test)]
mod tests {
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    use super::*;
    use crate::MockImuBus;

    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new().with_register(Reg::DEVID, Adxl375::<MockImuBus>::DEVICE_ID);
        assert_eq!(Adxl375::<MockImuBus>::DEVICE_ID, imu_bus.registers[Reg::DEVID as usize]);
    }
}
