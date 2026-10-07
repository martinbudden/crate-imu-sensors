use super::{ImuBus, ImuError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MockImuBus {
    // Standard IMU register maps are usually 128 or 256 bytes
    pub registers: [u8; 256],
}

impl Default for MockImuBus {
    fn default() -> Self {
        Self::new()
    }
}

impl MockImuBus {
    #[allow(unused)]
    #[must_use]
    pub const fn new() -> Self {
        Self { registers: [0u8; 256] }
    }
}

impl ImuBus for MockImuBus {
    async fn bus_write_read(&mut self, _address: u8, _write: &[u8], _read: &mut [u8]) -> Result<(), ImuError> {
        embassy_time::Timer::after_ticks(0).await;
        Ok(())
    }

    async fn read_register(&mut self, _address: u8, reg: u8) -> Result<u8, ImuError> {
        embassy_time::Timer::after_ticks(0).await;
        Ok(self.registers[reg as usize])
    }

    async fn read_registers(&mut self, _address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        let start = reg as usize;
        let end = start + data.len();

        // Copy slice from internal memory to the output buffer
        data.copy_from_slice(&self.registers[start..end]);
        embassy_time::Timer::after_ticks(0).await;
        Ok(())
    }

    async fn write_register(&mut self, _address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        self.registers[reg as usize] = data;
        embassy_time::Timer::after_ticks(0).await;
        Ok(())
    }

    async fn write_registers(&mut self, _address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        let start = reg as usize;
        let end = start + data.len();

        self.registers[start..end].copy_from_slice(data);
        embassy_time::Timer::after_ticks(0).await;
        Ok(())
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MockImuBus>();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    #[test]
    fn test_imu_mock() {
        let mut bus = MockImuBus::new();
        let write_data = [0xAA, 0xBB];

        // In a test environment, you'd "await" these
        pollster::block_on(bus.write_registers(0, 0x10, &write_data)).unwrap();

        let mut read_data = [0u8; 2];
        pollster::block_on(bus.read_registers(0, 0x10, &mut read_data)).unwrap();

        assert_eq!(read_data, [0xAA, 0xBB]);
    }
}
