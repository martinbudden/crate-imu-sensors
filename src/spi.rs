#![allow(unused)]
use super::MockImuBus;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi;

// Wrapper for SPI
pub struct SpiInterface<B, CS> {
    pub bus: B,
    pub cs: CS,
}

pub struct MySpiDriver<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> MySpiDriver<SPI, CS>
where
    SPI: embedded_hal_async::spi::SpiBus,
    CS: OutputPin,
{
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, SPI::Error> {
        // 1. Select the device (Pull CS LOW)
        _ = self.cs.set_low().ok();

        // 2. Perform SPI Transfer (e.g., Read command 0x80 | reg)
        let mut data = [reg | 0x80, 0];
        self.spi.transfer_in_place(&mut data).await?;

        // 3. Deselect the device (Pull CS HIGH)
        _ = self.cs.set_high().ok();

        Ok(data[1])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        //is_normal::<SpiInterface<MockImuBus>>();
    }
}
