#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuError {
    AccNotAvailable,
    GyroNotAvailable,
    InvalidBusTransaction,
    SpiBus,
    SpiBusRead,
    SpiBusWrite,
    I2cBus,
    I2cBusRead,
    I2cBusWrite,
    MissingRegister,
    DataSizeTooBig,
    IncorrectWhoAmI,
    InitializationError,
    InitializationLoadConfigError,
}

#[allow(async_fn_in_trait)]
pub trait ImuBus {
    /// Returns true if the bus is an SPI bus.
    fn is_spi(&self) -> bool;

    /// Performs a bus transaction.
    ///
    /// `address` is the 7-bit I2C device address and is ignored by SPI.
    ///
    /// `write` must contain at least the register address.
    ///
    /// If `read` is non-empty, the operation consists of writing the
    /// register address followed by reading the requested number of bytes.
    ///
    /// If `read` is empty, the operation is a write-only transaction.
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError>;

    /// Reads a single 8-bit register value.
    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, ImuError> {
        let mut value = [0u8; 1];

        self.bus_write_read(address, &[reg], &mut value).await?;

        Ok(value[0])
    }

    /// Reads multiple sequential registers.
    ///
    /// Requires hardware register auto-increment to be enabled where the device requires it.
    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), ImuError> {
        self.bus_write_read(address, &[reg], data).await
    }

    /// Writes a single byte to a register.
    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), ImuError> {
        self.bus_write_read(address, &[reg, data], &mut []).await
    }

    /// Writes multiple sequential registers.
    ///
    /// Maximum payload: 63 bytes, excluding the register address.
    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        const MAX_WRITE_PAYLOAD: usize = 63;

        if data.len() > MAX_WRITE_PAYLOAD {
            return Err(ImuError::DataSizeTooBig);
        }

        let mut write_buf = [0u8; MAX_WRITE_PAYLOAD + 1];

        write_buf[0] = reg;
        write_buf[1..=data.len()].copy_from_slice(data);

        self.bus_write_read(address, &write_buf[..=data.len()], &mut []).await
    }
}
