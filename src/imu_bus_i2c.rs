use super::ImuBus;

#[allow(unused)]
#[derive(Debug)]
pub enum ImuI2cError<E> {
    Bus(E),
    MissingRegister,
}

impl<E> From<E> for ImuI2cError<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

#[allow(unused)]
#[derive(Debug)]
pub struct ImuI2cBus<I2C> {
    pub bus: I2C,
}

impl<I2C> ImuI2cBus<I2C> {
    #[allow(unused)]
    pub fn new(bus: I2C) -> Self {
        Self { bus }
    }
}

impl<I2C> ImuBus for ImuI2cBus<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
    I2C::Error: core::fmt::Debug,
{
    type Error = ImuI2cError<I2C::Error>;

    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(address, write, read).await.map_err(ImuI2cError::Bus)
    }
}
