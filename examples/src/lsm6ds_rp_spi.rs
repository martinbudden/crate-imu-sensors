//! Minimal test for motor mixer.
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PINs 11-14
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::clk_sys_freq,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, DMA_CH1},
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use {defmt_rtt as _, panic_probe as _};

use imu_sensors::{AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuSpiBus, Lsm6ds};

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>,
                 embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sck = p.PIN_18;
    let mosi = p.PIN_19;
    let miso = p.PIN_16;
    let cs = Output::new(p.PIN_17, Level::High);

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000;
    let spi = Spi::new(p.SPI0, sck, mosi, miso, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config);

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    let imu_bus = ImuSpiBus::new(spi_device);

    let mut imu = Lsm6ds::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);
    let (acc_odr, gyro_odr) =
        imu.init(1000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G).await.unwrap();
    info!("IMU init acc: acc_odr, gyro_odr {} {}", acc_odr, gyro_odr);

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting imu-sensors rp spi test");

    info!("Reading IMU indefinitely");
    let mut count: u32 = 0;
    loop {
        let acc = imu.read_acc().await.unwrap();
        info!("acc x:{} y:{} z:{}", acc.x, acc.y, acc.z);
        Timer::after(Duration::from_millis(1)).await;
        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
