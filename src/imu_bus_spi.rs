use embedded_hal_async::spi::{Operation, SpiDevice};

use super::{ImuBus, ImuError};

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
{
    async fn bus_write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        if read.is_empty() {
            self.bus.write(write).await.map_err(|_| ImuError::SpiBus)
        } else {
            // Register read.
            // SPI reads require bit 7 of the register address to be set.
            let register = write.first().copied().ok_or(ImuError::MissingRegister)?;

            let register = register | 0x80;

            self.bus
                .transaction(&mut [Operation::Write(&[register]), Operation::Read(read)])
                .await
                .map_err(|_| ImuError::SpiBus)
        }
    }
}
