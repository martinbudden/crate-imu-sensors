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
    /// Set a register of a newly constructed `MockImuBus`.
    #[must_use]
    pub fn with_register(mut self, reg: u8, data: u8) -> Self {
        self.registers[reg as usize] = data;
        self
    }
}

impl ImuBus for MockImuBus {
    fn is_spi(&self) -> bool {
        false
    }

    async fn bus_write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        embassy_time::Timer::after_ticks(0).await;
        // Ensure we have a register address to read from
        if write.is_empty() {
            return Ok(());
        }

        let reg = write[0] as usize;

        if reg >= self.registers.len() {
            return Ok(());
        }

        // Calculate how many bytes we can safely copy
        let registers = &self.registers[reg..];
        let copy_len = registers.len().min(read.len());

        read[..copy_len].copy_from_slice(&registers[..copy_len]);

        Ok(())
    }

    async fn read_register(&mut self, _address: u8, reg: u8) -> Result<u8, ImuError> {
        embassy_time::Timer::after_ticks(0).await;
        Ok(self.registers[reg as usize])
    }

    async fn read_registers(&mut self, _address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        embassy_time::Timer::after_ticks(0).await;

        let start = reg as usize;
        let end = start + data.len();
        // Copy slice from internal memory to the output buffer
        data.copy_from_slice(&self.registers[start..end]);
        Ok(())
    }

    async fn write_register(&mut self, _address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        embassy_time::Timer::after_ticks(0).await;

        self.registers[reg as usize] = data;
        Ok(())
    }

    async fn write_registers(&mut self, _address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        embassy_time::Timer::after_ticks(0).await;

        let start = reg as usize;
        let end = start + data.len();

        self.registers[start..end].copy_from_slice(data);
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
    use super::*;
    #[test]
    fn test_imu_mock() {
        let mut bus = MockImuBus::new();
        let write_data = [0xAA, 0xBB];

        // In a test environment, you'd "await" these
        _ = pollster::block_on(bus.write_registers(0, 0x10, &write_data));

        let mut read_data = [0u8; 2];
        _ = pollster::block_on(bus.read_registers(0, 0x10, &mut read_data));

        assert_eq!(read_data, [0xAA, 0xBB]);
    }
}
