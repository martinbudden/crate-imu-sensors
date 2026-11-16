use vqm::Vector3f32;

use super::{AccSensitivity, AccUnits, GyroSensitivity, GyroUnits, ImuError};

#[allow(async_fn_in_trait)]
pub trait ImuDevice {
    /// Return the gyro and acc sample rates set.
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: GyroSensitivity,
        gyro_units: GyroUnits,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<(u32, u32), ImuError>;

    async fn read_acc_gyro(&mut self) -> Result<(Vector3f32, Vector3f32), ImuError>;
}
