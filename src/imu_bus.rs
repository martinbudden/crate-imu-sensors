#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuError {
    SpiBus,
    SpiBusRead,
    SpiBusWrite,
    I2cBus,
    I2cBusRead,
    I2cBusWrite,
    MissingRegister,
    DataSizeTooBig,
}

/*#[derive(Debug, Clone, PartialEq)]
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
}*/

#[allow(async_fn_in_trait)]
pub trait ImuBus {
    /// The core transaction primitive. Writes bytes while reading bytes back.
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
    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), ImuError> {
        // Enforce a hard maximum upper bound for your safe stack array allocation
        const MAX_WRITE_LEN: usize = 64;
        debug_assert!(data.len() < MAX_WRITE_LEN, "Data transmission block exceeds maximum size (63) bytes");

        if data.len() < MAX_WRITE_LEN {
            let mut write_buf = [0u8; MAX_WRITE_LEN];
            write_buf[0] = reg;
            // Copy the payload into the scratchpad directly following the register byte
            write_buf[1..=data.len()].copy_from_slice(data);

            self.bus_write_read(address, &write_buf[0..=data.len()], &mut []).await
        } else {
            Err(ImuError::DataSizeTooBig)
        }
    }
}
