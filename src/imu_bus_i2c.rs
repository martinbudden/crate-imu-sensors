use embedded_hal_async::i2c::I2c;

use super::{ImuBus, ImuError};

#[derive(Debug)]
pub struct ImuI2cBus<I2C> {
    pub bus: I2C,
}

impl<I2C> ImuI2cBus<I2C> {
    pub fn new(bus: I2C) -> Self {
        Self { bus }
    }
}

impl<I2C> ImuBus for ImuI2cBus<I2C>
where
    I2C: I2c,
{
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        self.bus.write_read(address, write, read).await.map_err(|_| ImuError::I2cBus)
    }
}
