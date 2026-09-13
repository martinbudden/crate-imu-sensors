#![allow(unused)]
use super::MockImuBus;
pub struct I2cInterface<B> {
    pub bus: B,
    pub address: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<I2cInterface<MockImuBus>>();
    }
}
