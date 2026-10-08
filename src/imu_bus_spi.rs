use embedded_hal_async::spi::{Operation, SpiDevice};

use super::{ImuBus, ImuError};

#[derive(Debug)]
pub struct ImuSpiBus<SPI> {
    bus: SPI,
}

impl<SPI> ImuSpiBus<SPI> {
    pub fn new(bus: SPI) -> Self {
        Self { bus }
    }
}

impl<SPI> ImuBus for ImuSpiBus<SPI>
where
    SPI: SpiDevice<u8>,
{
    fn is_spi(&self) -> bool {
        true
    }

    async fn bus_write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), ImuError> {
        if write.is_empty() {
            return Err(ImuError::SpiBus);
        }

        if read.is_empty() {
            // SPI write.
            // write = [register, data...]
            self.bus.write(write).await.map_err(|_| ImuError::SpiBusWrite)
        } else {
            // SPI read.
            // SPI reads require bit 7 of the register address to be set.
            // ExclusiveDevice keeps CS asserted across the complete transaction,
            // so the register command and data transfer form one SPI transaction.
            let command = [write[0] | 0x80];

            self.bus
                .transaction(&mut [Operation::Write(&command), Operation::Read(read)])
                .await
                .map_err(|_| ImuError::SpiBusRead)
        }
    }
}
