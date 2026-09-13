use embedded_hal_async::spi::{Operation, SpiDevice};

use super::ImuBus;

#[allow(unused)]
#[derive(Debug)]
pub enum ImuSpiError<E> {
    Bus(E),
    MissingRegister,
}

impl<E> From<E> for ImuSpiError<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

#[allow(unused)]
#[derive(Debug)]
pub struct ImuSpiBus<SPI> {
    pub bus: SPI,
}

impl<SPI> ImuSpiBus<SPI> {
    #[allow(unused)]
    pub fn new(bus: SPI) -> Self {
        Self { bus }
    }
}

impl<SPI> ImuBus for ImuSpiBus<SPI>
where
    SPI: SpiDevice,
    SPI::Error: core::fmt::Debug,
{
    type Error = ImuSpiError<SPI::Error>;

    async fn bus_write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        if read.is_empty() {
            self.bus.write(write).await.map_err(ImuSpiError::Bus)
        } else {
            // Register read.
            // SPI reads require bit 7 of the register address to be set.
            let register = write.first().copied().ok_or(ImuSpiError::MissingRegister)?;

            let register = register | 0x80;

            self.bus
                .transaction(&mut [Operation::Write(&[register]), Operation::Read(read)])
                .await
                .map_err(ImuSpiError::Bus)
        }
    }
}
