use embassy_time::Instant;

#[cfg(feature = "nalgebra")]
use nalgebra::Vector3;
#[cfg(feature = "vqm")]
use vqm::Vector3;

use super::{AccSensitivity, AccUnits, ImuError};

#[allow(async_fn_in_trait)]
pub trait AccDevice {
    // Return the acc sample rate set.
    async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        acc_sensitivity: AccSensitivity,
        acc_units: AccUnits,
    ) -> Result<u32, ImuError>;

    async fn read_acc(&mut self) -> Result<(Vector3<f32>, Instant), ImuError>;
}
