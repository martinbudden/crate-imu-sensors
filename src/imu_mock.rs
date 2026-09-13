use vqm::Vector3f32;

use super::{
    AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuBus, ImuCommon, ImuDevice, ImuDeviceConfig,
};

struct Reg;

impl Reg {
    const ACC_XL: u8 = 0x20;
    const _ACC_XH: u8 = 0x21;
    const _ACC_YL: u8 = 0x22;
    const _ACC_YH: u8 = 0x23;
    const _ACC_ZL: u8 = 0x24;
    const _ACC_ZH: u8 = 0x25;
    const GYRO_XL: u8 = 0x26;
    const _GYRO_XH: u8 = 0x27;
    const _GYRO_YL: u8 = 0x28;
    const _GYRO_YH: u8 = 0x29;
    const _GYRO_ZL: u8 = 0x2A;
    const _GYRO_ZH: u8 = 0x2B;
}

#[allow(missing_docs)]
#[derive(Debug, PartialEq)]
pub struct ImuMock<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuDeviceConfig,
}

impl<B: ImuBus> ImuDevice for ImuMock<B> {
    type Error = B::Error;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error> {
        ImuMock::init(self, target_output_data_rate_hz, gyro_sensitivity, gyro_units, acc_sensitivity, acc_units).await
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        <Self as Imu>::read_acc_gyro(self).await
    }
}

impl<B: ImuBus> Imu for ImuMock<B> {
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
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, Reg::ACC_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");
        let acc = Vector3f32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, acc))
    }

    async fn read_gyro(&mut self) -> Result<Vector3f32, Self::Error> {
        let mut buf = [0u8; 6];
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, Reg::GYRO_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");
        let gyro = Vector3f32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_vector(self.common.axis_order, gyro))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error> {
        let mut buf = [0u8; 12];
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, Reg::ACC_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");

        let [a0, a1, a2, a3, a4, a5, g0, g1, g2, g3, g4, g5] = buf;

        let acc_buf = [a0, a1, a2, a3, a4, a5];
        let gyro_buf = [g0, g1, g2, g3, g4, g5];
        let acc = Vector3f32::from_le_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3f32::from_le_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;
        Ok(ImuAxisOrder::map_acc_gyro(self.common.axis_order, acc, gyro))
    }
}

impl<B: ImuBus> ImuMock<B> {
    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxisOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuDeviceConfig {
                gyro_id_msp: ImuDeviceConfig::MSP_ACC_ID_DEFAULT,
                acc_id_msp: ImuDeviceConfig::MSP_ACC_ID_DEFAULT,
                axis_order,
                device_id: 0,
                address: 0,
                flags: 0,
            },
        }
    }

    /// # Panics
    pub async fn set_acc(&mut self, acc: Vector3f32) {
        let acc_unscaled =
            ((acc + self.common.acc_offset) / self.common.acc_scale).clamp(f32::from(i16::MIN), f32::from(i16::MAX));
        #[allow(clippy::cast_possible_truncation)]
        let x_i16 = acc_unscaled.x as i16;
        #[allow(clippy::cast_possible_truncation)]
        let y_i16 = acc_unscaled.y as i16;
        #[allow(clippy::cast_possible_truncation)]
        let z_i16 = acc_unscaled.z as i16;

        let x = x_i16.to_le_bytes();
        let y = y_i16.to_le_bytes();
        let z = z_i16.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        #[allow(clippy::expect_used)]
        self.bus().write_registers(0, Reg::ACC_XL, &data).await.expect("write_resisters cannot fail for ImuMock");
    }

    /// # Panics
    pub async fn set_gyro(&mut self, gyro: Vector3f32) {
        let gyro_unscaled =
            ((gyro + self.common.gyro_offset) / self.common.gyro_scale).clamp(f32::from(i16::MIN), f32::from(i16::MAX));

        #[allow(clippy::cast_possible_truncation)]
        let x_i16 = gyro_unscaled.x as i16;
        #[allow(clippy::cast_possible_truncation)]
        let y_i16 = gyro_unscaled.y as i16;
        #[allow(clippy::cast_possible_truncation)]
        let z_i16 = gyro_unscaled.z as i16;

        let x = x_i16.to_le_bytes();
        let y = y_i16.to_le_bytes();
        let z = z_i16.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        #[allow(clippy::expect_used)]
        self.bus().write_registers(0, Reg::GYRO_XL, &data).await.expect("write_resisters cannot fail for ImuMock");
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
        self.bus.write_register(0, 0, 0).await?;

        self.calculate_acc_scale(acc_sensitivity, acc_units);

        self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_units, target_output_data_rate_hz);

        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        target_output_data_rate_hz: u32,
    ) {
        let scale_dps = match gyro_sensitivity {
            GyroFullScale::Scale125Dps => 125.0,
            GyroFullScale::Scale250Dps => 250.0,
            GyroFullScale::Scale500Dps => 500.0,
            GyroFullScale::Scale1000Dps => 1000.0,
            _ => 2000.0,
        };
        self.common.gyro_scale = if gyro_units == GyroUnits::Dps { scale_dps } else { scale_dps.to_radians() };

        self.common.gyro_sample_rate_hz = target_output_data_rate_hz;
    }

    pub fn calculate_acc_scale(&mut self, acc_sensitivity: AccFullScale, acc_units: AccUnits) {
        let scale = match acc_sensitivity {
            AccFullScale::Scale2G => 2.0 / 32768.0,
            AccFullScale::Scale4G => 4.0 / 32768.0,
            AccFullScale::Scale8G => 8.0 / 32768.0,
            _ => 16.0 / 32768.0,
        };
        self.common.acc_scale = if acc_units == AccUnits::G { scale } else { scale * ImuCommon::G0 };
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

    impl<B: ImuBus> ImuMock<B> {
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
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let result =
            pollster::block_on(imu.init(8000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G));
        let (gyro_register_value, acc_register_value) = result.unwrap();

        assert_eq!(8000, gyro_register_value);
        assert_eq!(1000, acc_register_value);
        //assert_eq!(2000.0 / 32768.0, state.gyro_scale);
        //assert_eq!(16.0 / 32768.0, state.acc_scale);
        //assert_eq!(6664, state.gyro_sample_rate_hz);
        //assert_eq!(6664, state.acc_sample_rate_hz);
    }
    #[test]
    fn acc_buf() {
        let imu_bus = MockImuBus::new();
        let imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let acc = Vector3f32::from_le_bytes_6(data) * imu.common.acc_scale - imu.common.acc_offset;
        assert_eq!(Vector3f32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
    #[test]
    fn gyro_buf() {
        let imu_bus = MockImuBus::new();
        let imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x10, 0x00, 0x00, 0x01, 0x00, 0x7f];
        let gyro = Vector3f32::from_le_bytes_6(data) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: 0.976_562_5, y: 15.625, z: 1984.375 }, gyro);

        let data: [u8; 6] = [0x01, 0x00, 0x80, 0x00, 0xff, 0x7f];
        let gyro = Vector3f32::from_le_bytes_6(data) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: 0.061_035_156, y: 7.8125, z: 1999.939 }, gyro);
    }
    #[test]
    fn scale_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let acc_scale = imu.acc_scale() * 32768.0;
        assert_eq!(8.0, acc_scale);

        let acc = Vector3f32::new(0.5, 2.0, 1.0);
        pollster::block_on(imu.set_acc(acc));

        let mut buf = [0u8; 6];
        let _result = pollster::block_on(imu.bus().read_registers(0, Reg::ACC_XL, &mut buf));
        assert_eq!([0x00, 0x08, 0x00, 0x20, 0x00, 0x10], buf);

        let a = Vector3f32::from_le_bytes_6(buf) * imu.common.acc_scale - imu.common.acc_offset;
        assert_eq!(Vector3f32::new(0.5, 2.0, 1.0), a);
    }
    #[test]
    fn read_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);

        let acc = Vector3f32::new(0.5, 2.0, 1.0);
        pollster::block_on(imu.set_acc(acc));
        let result = pollster::block_on(imu.read_acc());
        let a = result.unwrap();
        assert_eq!(Vector3f32::new(0.5, 2.0, 1.0), a);
    }
    #[test]
    fn scale_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);
        let gyro_scale = imu.gyro_scale() * 32768.0;
        assert_eq!(2000.0, gyro_scale);

        let mut buf = [0u8; 6];

        let gyro = Vector3f32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro(gyro));
        let _result = pollster::block_on(imu.bus().read_registers(0, Reg::GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x08, 0x00, 0x40, 0x00, 0x70], buf);
        let g = Vector3f32::from_le_bytes_6(buf) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: 125.0, y: 1000.0, z: 1750.0 }, g);

        let gyro = Vector3f32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let _result = pollster::block_on(imu.bus().read_registers(0, Reg::GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x20, 0x00, 0x40, 0xFF, 0x7F], buf);
        let g = Vector3f32::from_le_bytes_6(buf) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro = Vector3f32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let _result = pollster::block_on(imu.bus().read_registers(0, Reg::GYRO_XL, &mut buf));
        assert_eq!([0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F], buf);
        let g = Vector3f32::from_le_bytes_6(buf) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro = Vector3f32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let _result = pollster::block_on(imu.bus().read_registers(0, Reg::GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x80, 0x00, 0x80, 0x00, 0x80], buf);
        let g = Vector3f32::from_le_bytes_6(buf) * imu.common.gyro_scale - imu.common.gyro_offset;
        assert_eq!(Vector3f32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
    #[test]
    fn read_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);
        let gyro_scale = imu.gyro_scale() * 32768.0;
        assert_eq!(2000.0, gyro_scale);

        let gyro = Vector3f32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro(gyro));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3f32 { x: 125.0, y: 1000.0, z: 1750.0 }, g);

        let gyro = Vector3f32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3f32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro = Vector3f32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3f32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro = Vector3f32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro(gyro));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3f32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
}
