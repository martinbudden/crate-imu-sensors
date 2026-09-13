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
    i2c::{Config as I2cConfig, I2c, InterruptHandler},
    peripherals::I2C0,
};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use imu_sensors::{AccFullScale, AccUnits, GyroFullScale, GyroUnits, Imu, ImuAxisOrder, ImuI2cBus, Lsm6ds};

// Bind the I2C interrupt to Embassy's I2C interrupt handler.
bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_4;
    let scl = p.PIN_5;

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;

    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c_config);
    let imu_bus = ImuI2cBus::new(i2c);

    let mut imu = Lsm6ds::new(imu_bus, ImuAxisOrder::XPOS_YPOS_ZPOS);
    let (acc_odr, gyro_odr) =
        imu.init(1000, GyroFullScale::Max, GyroUnits::Dps, AccFullScale::Max, AccUnits::G).await.unwrap();
    info!("IMU init acc: acc_odr, gyro_odr {} {}", acc_odr, gyro_odr);

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting imu-sensors rp i2c test");

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
