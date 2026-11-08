use vqm::Vector3f32;

use super::{AccFullScale, AccUnits, ImuError};

#[allow(async_fn_in_trait)]
pub trait AccDevice {
    // Return the acc sample rate set.
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        acc_sensitivity: AccFullScale,
        acc_units: AccUnits,
    ) -> Result<u32, ImuError>;

    async fn read_acc(&mut self) -> Result<Vector3f32, ImuError>;
}
