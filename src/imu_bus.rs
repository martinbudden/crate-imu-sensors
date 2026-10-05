//use embassy_rp::i2c::{I2c, Async};
#[cfg(feature = "rp2040")]
use embassy_rp::gpio::Output;
#[cfg(all(feature = "rp2040", feature = "spi"))]
use embassy_rp::gpio::Pin;
#[cfg(all(feature = "rp2040", feature = "i2c"))]
use embassy_rp::i2c::{Async, I2c, Instance};
#[cfg(all(feature = "rp2040", feature = "spi"))]
use embassy_rp::spi::{Async, Instance, Spi};

use super::ImuError;

#[allow(async_fn_in_trait)]
pub trait ImuBus {
    /// The core transaction primitive. Writes bytes, then reads bytes back.
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError>;

    /// Reads a single 8-bit register value.
    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        self.bus_write_read(address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    /// Reads multiple sequential registers into a destination buffer.
    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        self.bus_write_read(address, &[reg], data).await
    }

    /// Writes a single byte to a register.
    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        self.bus_write_read(address, &[reg, data], &mut []).await
    }

    /// Writes a contiguous slice of data starting at a target register.
    ///
    /// # Panics
    /// Panics if the `data` slice exceeds 63 bytes.
    // TODO: optimize ImuBus write_registers
    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        // Enforce a hard maximum upper bound for your safe stack array allocation
        const MAX_WRITE_LEN: usize = 64;
        assert!(data.len() < MAX_WRITE_LEN - 1, "Data transmission block exceeds maximum size (63) bytes");

        let mut write_buf = [0u8; MAX_WRITE_LEN];
        write_buf[0] = reg;
        // Copy the payload into the scratchpad directly following the register byte
        write_buf[1..=data.len()].copy_from_slice(data);

        self.bus_write_read(address, &write_buf[0..=data.len()], &mut []).await
    }
}

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

#[cfg(all(feature = "rp2040", feature = "i2c"))]
impl<T: Instance> ImuBus for I2c<'_, T, Async> {
    type Error = embassy_rp::i2c::Error;
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        // On the Pico, I2C write_read is natively async.
        // We just delegate the call and .await the result.
        self.write_read_async(address, write.iter().copied(), read).await

        // Explicitly call the method on the I2c struct itself
        // instead of let the compiler guess (and pick the trait method)
        //I2c::<'d, T, Async>::write_read(self, address, write, read).await
    }

    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        // Write the register address, read back 1 byte
        self.bus_write_read(address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        // Write the starting register address, read back 'data.len()' bytes
        // The MPU6050 automatically increments the register pointer internally
        self.bus_write_read(address, &[reg], data).await
    }

    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        // To write, we send [register, value] and expect 0 bytes back
        self.bus_write_read(address, &[reg, data], &mut []).await
    }

    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        // This is trickier: I2C writes usually need the register and data in one contiguous stream.
        // For no_std, we can use a small local buffer or a loop if the bus supports it.
        // For a simple MPU6050 config, usually we only write 1-2 bytes at a time.
        const MAX_WRITE_LEN: usize = 64;
        assert!(data.len() < MAX_WRITE_LEN - 1, "Data transmission block exceeds maximum size (63) bytes");
        let mut write_buf = [0u8; MAX_WRITE_LEN];
        write_buf[0] = reg;
        // Copy the payload into the scratchpad directly following the register byte
        write_buf[1..=data.len()].copy_from_slice(data);

        self.bus_write_read(address, &write_buf[0..=data.len()], &mut []).await
    }
}

/*#[cfg(all(feature = "rp2040", feature = "spi"))]
pub struct SpiBusWrapper<'d, T: Instance, CS: Pin> {
    pub spi: Spi<'d, T, Async>,
    // Remove CS from the angle brackets here
    pub cs: Output<'d>,
    // This _marker ensures the compiler knows CS is "used"
    _pcs: core::marker::PhantomData<CS>,
}

#[cfg(all(feature = "rp2040", feature = "spi"))]
impl<'d, T: Instance, CS: embassy_rp::gpio::Pin> SpiBusWrapper<'d, T, CS> {
    async fn write_burst(&mut self, reg: u8, data: &[u8]) -> Result<(), embassy_rp::spi::Error> {
        self.cs.set_low();

        // 1. Write register (ensure Write bit is 0)
        let mut dummy = [0u8; 1];
        self.spi.transfer(&mut dummy, &[reg & 0x7F]).await?;

        // 2. Write all data bytes in the same session
        // We need a dummy buffer for the SPI read-back
        // (For small writes, a stack array is fine; for large, use a loop)
        for chunk in data.chunks(16) {
            let mut d_buf = [0u8; 16];
            self.spi.transfer(&mut d_buf[..chunk.len()], chunk).await?;
        }

        self.cs.set_high();
        Ok(())
    }
}
*/
#[cfg(all(feature = "rp2040", feature = "spi"))]
pub struct SpiBusWrapper<'d, T: Instance> {
    pub spi: Spi<'d, T, Async>,
    pub cs: Output<'d>,
}

#[cfg(all(feature = "rp2040", feature = "spi"))]
impl<'d, T: Instance> ImuBus for SpiBusWrapper<'d, T> {
    //impl<'d, T: Instance, CS: embassy_rp::gpio::Pin> ImuBus for SpiBusWrapper<'d, T, CS> {
    type Error = embassy_rp::spi::Error;

    async fn bus_write_read(&mut self, _addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        self.cs.set_low(); // Pull CS low to start transaction

        // Use transfer for full-duplex SPI communication
        let res = self.spi.transfer(read, write).await;

        self.cs.set_high(); // Pull CS high to end transaction
        res
    }

    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];

        // If SPI is enabled, we need to set the Read Bit (0x80)
        #[cfg(feature = "spi")]
        let reg = reg | 0x80;

        self.bus_write_read(address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        // Write the starting register address, read back 'data.len()' bytes
        // The MPU6050 automatically increments the register pointer internally
        self.bus_write_read(address, &[reg], data).await
    }

    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        // To write, we send [register, value] and expect 0 bytes back
        self.bus_write_read(address, &[reg, data], &mut []).await
    }
    /// # Panics
    /// Panics if the `data` slice exceeds 63 bytes to protect the stack.
    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        const MAX_WRITE_LEN: usize = 64;
        assert!(data.len() < MAX_WRITE_LEN - 1, "Data transmission block exceeds maximum size (63) bytes");
        let mut write_buf = [0u8; MAX_WRITE_LEN];
        write_buf[0] = reg & 0x7F;
        write_buf[1..=data.len()].copy_from_slice(data);
        self.bus_write_read(address, &write_buf[..=data.len()], &mut []).await
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SetupError<E> {
    /// An error occurred with the I2C/SPI bus during setup.
    Bus(E),
    /// An incorrect 'Who Am I' value was returned from the IMU.
    ImuWhoAmI(u8),
}

impl<E> From<E> for SetupError<E> {
    fn from(error: E) -> Self {
        SetupError::Bus(error)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {}

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
