use vqm::Vector3f32;

use super::{AccFullScale, AccUnits, GyroFullScale, GyroUnits};

#[allow(unused)]
#[allow(async_fn_in_trait)]
pub trait ImuDevice {
    type Error: core::fmt::Debug;

    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroFullScale,
        gyro_units: GyroUnits,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), Self::Error>;

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), Self::Error>;
}
