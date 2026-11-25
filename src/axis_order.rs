//#![allow(clippy::return_self_not_must_use)]
use vqm::Vector3f32;

#[cfg(feature = "serde")]
use ::serde::{Deserialize, Serialize};
#[cfg(feature = "storage")]
use {postcard::experimental::max_size::MaxSize, sequential_storage::map::PostcardValue};

#[allow(missing_docs)]
#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub enum AxisOrder {
    #[default]
    XPOS_YPOS_ZPOS = 0,
    YPOS_XNEG_ZPOS = 1, // rotate  90 degrees anticlockwise
    XNEG_YNEG_ZPOS = 2, // rotate 180 degrees
    YNEG_XPOS_ZPOS = 3, // rotate 270 degrees anticlockwise
    XPOS_YNEG_ZNEG = 4,
    YPOS_XPOS_ZNEG = 5,
    XNEG_YPOS_ZNEG = 6,
    YNEG_XNEG_ZNEG = 7,

    ZPOS_YNEG_XPOS = 8,
    YPOS_ZPOS_XPOS = 9,
    ZNEG_YPOS_XPOS = 10,
    YNEG_ZNEG_XPOS = 11,

    ZPOS_YPOS_XNEG = 12,
    YPOS_ZNEG_XNEG = 13,
    ZNEG_YNEG_XNEG = 14,
    YNEG_ZPOS_XNEG = 15,

    ZPOS_XPOS_YPOS = 16,
    XNEG_ZPOS_YPOS = 17,
    ZNEG_XNEG_YPOS = 18,
    XPOS_ZNEG_YPOS = 19,

    ZPOS_XNEG_YNEG = 20,
    XNEG_ZNEG_YNEG = 21,
    ZNEG_XPOS_YNEG = 22,
    XPOS_ZPOS_YNEG = 23,

    XPOS_YPOS_ZPOS_45 = 24, // rotate  45 degrees anticlockwise
    YPOS_XNEG_ZPOS_45 = 25, // rotate 135 degrees anticlockwise
    XNEG_YNEG_ZPOS_45 = 26, // rotate 225 degrees anticlockwise
    YNEG_XPOS_ZPOS_45 = 27, // rotate 315 degrees anticlockwise

                            //XPOS_YPOS_ZPOS_135 = YPOS_XNEG_ZPOS_45,
                            //XPOS_YPOS_ZPOS_225 = XNEG_YNEG_ZPOS_45,
                            //XPOS_YPOS_ZPOS_315 = YNEG_XPOS_ZPOS_45,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for AxisOrder {}

impl AxisOrder {
    pub const COUNT: u8 = 28;

    /// Forgiving conversion from u8 to `ImuAxisOrder`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::XPOS_YPOS_ZPOS,
            1 => Self::YPOS_XNEG_ZPOS,
            2 => Self::XNEG_YNEG_ZPOS,
            3 => Self::YNEG_XPOS_ZPOS,
            4 => Self::XPOS_YNEG_ZNEG,
            5 => Self::YPOS_XPOS_ZNEG,
            6 => Self::XNEG_YPOS_ZNEG,
            7 => Self::YNEG_XNEG_ZNEG,

            8 => Self::ZPOS_YNEG_XPOS,
            9 => Self::YPOS_ZPOS_XPOS,
            10 => Self::ZNEG_YPOS_XPOS,
            11 => Self::YNEG_ZNEG_XPOS,

            12 => Self::ZPOS_YPOS_XNEG,
            13 => Self::YPOS_ZNEG_XNEG,
            14 => Self::ZNEG_YNEG_XNEG,
            15 => Self::YNEG_ZPOS_XNEG,

            16 => Self::ZPOS_XPOS_YPOS,
            17 => Self::XNEG_ZPOS_YPOS,
            18 => Self::ZNEG_XNEG_YPOS,
            19 => Self::XPOS_ZNEG_YPOS,

            20 => Self::ZPOS_XNEG_YNEG,
            21 => Self::XNEG_ZNEG_YNEG,
            22 => Self::ZNEG_XPOS_YNEG,
            23 => Self::XPOS_ZPOS_YNEG,

            24 => Self::XPOS_YPOS_ZPOS_45,
            25 => Self::YPOS_XNEG_ZPOS_45,
            26 => Self::XNEG_YNEG_ZPOS_45,
            27 => Self::YNEG_XPOS_ZPOS_45,
            _ => Self::default(),
        }
    }
}

impl AxisOrder {
    // #[rustfmt::skip]
    #[inline]
    #[must_use]
    pub fn map_vector(self, v: Vector3f32) -> Vector3f32 {
        const SIN45: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const COS45: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const SIN135: f32 = SIN45;
        const COS135: f32 = -COS45;
        const SIN225: f32 = -SIN45;
        const COS225: f32 = -COS45;
        const SIN315: f32 = -SIN45;
        const COS315: f32 = COS45;

        match self {
            Self::XPOS_YPOS_ZPOS => v,
            Self::YPOS_XNEG_ZPOS => Vector3f32::new(v.y, -v.x, v.z),
            Self::XNEG_YNEG_ZPOS => Vector3f32::new(-v.x, -v.y, v.z),
            Self::YNEG_XPOS_ZPOS => Vector3f32::new(-v.y, v.x, v.z),
            Self::XPOS_YNEG_ZNEG => Vector3f32::new(v.x, -v.y, -v.z),
            Self::YPOS_XPOS_ZNEG => Vector3f32::new(v.y, v.x, -v.z),
            Self::XNEG_YPOS_ZNEG => Vector3f32::new(-v.x, v.y, -v.z),
            Self::YNEG_XNEG_ZNEG => Vector3f32::new(-v.y, -v.x, -v.z),
            Self::ZPOS_YNEG_XPOS => Vector3f32::new(v.z, -v.y, v.x),
            Self::YPOS_ZPOS_XPOS => Vector3f32::new(v.y, v.z, v.x),
            Self::ZNEG_YPOS_XPOS => Vector3f32::new(-v.z, v.y, v.x),
            Self::YNEG_ZNEG_XPOS => Vector3f32::new(-v.y, -v.z, v.x),
            Self::ZPOS_YPOS_XNEG => Vector3f32::new(v.z, v.y, -v.x),
            Self::YPOS_ZNEG_XNEG => Vector3f32::new(v.y, -v.z, -v.x),
            Self::ZNEG_YNEG_XNEG => Vector3f32::new(-v.z, -v.y, -v.x),
            Self::YNEG_ZPOS_XNEG => Vector3f32::new(-v.y, v.z, -v.x),
            Self::ZPOS_XPOS_YPOS => Vector3f32::new(v.z, v.x, v.y),
            Self::XNEG_ZPOS_YPOS => Vector3f32::new(-v.x, v.z, v.y),
            Self::ZNEG_XNEG_YPOS => Vector3f32::new(-v.z, -v.x, v.y),
            Self::XPOS_ZNEG_YPOS => Vector3f32::new(v.x, -v.z, v.y),
            Self::ZPOS_XNEG_YNEG => Vector3f32::new(v.z, -v.x, -v.y),
            Self::XNEG_ZNEG_YNEG => Vector3f32::new(-v.x, -v.z, -v.y),
            Self::ZNEG_XPOS_YNEG => Vector3f32::new(-v.z, v.x, -v.y),
            Self::XPOS_ZPOS_YNEG => Vector3f32::new(v.x, v.z, -v.y),
            Self::XPOS_YPOS_ZPOS_45 => Vector3f32::new(v.x * COS45 + v.y * SIN45, -v.x * SIN45 + v.y * COS45, v.z),
            Self::YPOS_XNEG_ZPOS_45 => Vector3f32::new(v.x * COS135 + v.y * SIN135, -v.x * SIN135 + v.y * COS135, v.z),
            Self::XNEG_YNEG_ZPOS_45 => Vector3f32::new(v.x * COS225 + v.y * SIN225, -v.x * SIN225 + v.y * COS225, v.z),
            Self::YNEG_XPOS_ZPOS_45 => Vector3f32::new(v.x * COS315 + v.y * SIN315, -v.x * SIN315 + v.y * COS315, v.z),
        }
    }

    #[inline]
    #[must_use]
    pub fn map_acc_gyro(self, acc: Vector3f32, gyro: Vector3f32) -> (Vector3f32, Vector3f32) {
        const SIN45: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const COS45: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const SIN135: f32 = SIN45;
        const COS135: f32 = -COS45;
        const SIN225: f32 = -SIN45;
        const COS225: f32 = -COS45;
        const SIN315: f32 = -SIN45;
        const COS315: f32 = COS45;

        match self {
            Self::XPOS_YPOS_ZPOS => (acc, gyro),
            Self::YPOS_XNEG_ZPOS => (Vector3f32::new(acc.y, -acc.x, acc.z), Vector3f32::new(gyro.y, -gyro.x, gyro.z)),
            Self::XNEG_YNEG_ZPOS => (Vector3f32::new(-acc.x, -acc.y, acc.z), Vector3f32::new(-gyro.x, -gyro.y, gyro.z)),
            Self::YNEG_XPOS_ZPOS => (Vector3f32::new(-acc.y, acc.x, acc.z), Vector3f32::new(-gyro.y, gyro.x, gyro.z)),
            Self::XPOS_YNEG_ZNEG => (Vector3f32::new(acc.x, -acc.y, -acc.z), Vector3f32::new(gyro.x, -gyro.y, -gyro.z)),
            Self::YPOS_XPOS_ZNEG => (Vector3f32::new(acc.y, acc.x, -acc.z), Vector3f32::new(gyro.y, gyro.x, -gyro.z)),
            Self::XNEG_YPOS_ZNEG => (Vector3f32::new(-acc.x, acc.y, -acc.z), Vector3f32::new(-gyro.x, gyro.y, -gyro.z)),
            Self::YNEG_XNEG_ZNEG => {
                (Vector3f32::new(-acc.y, -acc.x, -acc.z), Vector3f32::new(-gyro.y, -gyro.x, -gyro.z))
            }
            Self::ZPOS_YNEG_XPOS => (Vector3f32::new(acc.z, -acc.y, acc.x), Vector3f32::new(gyro.z, -gyro.y, gyro.x)),
            Self::YPOS_ZPOS_XPOS => (Vector3f32::new(acc.y, acc.z, acc.x), Vector3f32::new(gyro.y, gyro.z, gyro.x)),
            Self::ZNEG_YPOS_XPOS => (Vector3f32::new(-acc.z, acc.y, acc.x), Vector3f32::new(-gyro.z, gyro.y, gyro.x)),
            Self::YNEG_ZNEG_XPOS => (Vector3f32::new(-acc.y, -acc.z, acc.x), Vector3f32::new(-gyro.y, -gyro.z, gyro.x)),
            Self::ZPOS_YPOS_XNEG => (Vector3f32::new(acc.z, acc.y, -acc.x), Vector3f32::new(gyro.z, gyro.y, -gyro.x)),
            Self::YPOS_ZNEG_XNEG => (Vector3f32::new(acc.y, -acc.z, -acc.x), Vector3f32::new(gyro.y, -gyro.z, -gyro.x)),
            Self::ZNEG_YNEG_XNEG => {
                (Vector3f32::new(-acc.z, -acc.y, -acc.x), Vector3f32::new(-gyro.z, -gyro.y, -gyro.x))
            }
            Self::YNEG_ZPOS_XNEG => (Vector3f32::new(-acc.y, acc.z, -acc.x), Vector3f32::new(-gyro.y, gyro.z, -gyro.x)),
            Self::ZPOS_XPOS_YPOS => (Vector3f32::new(acc.z, acc.x, acc.y), Vector3f32::new(gyro.z, gyro.x, gyro.y)),
            Self::XNEG_ZPOS_YPOS => (Vector3f32::new(-acc.x, acc.z, acc.y), Vector3f32::new(-gyro.x, gyro.z, gyro.y)),
            Self::ZNEG_XNEG_YPOS => (Vector3f32::new(-acc.z, -acc.x, acc.y), Vector3f32::new(-gyro.z, -gyro.x, gyro.y)),
            Self::XPOS_ZNEG_YPOS => (Vector3f32::new(acc.x, -acc.z, acc.y), Vector3f32::new(gyro.x, -gyro.z, gyro.y)),
            Self::ZPOS_XNEG_YNEG => (Vector3f32::new(acc.z, -acc.x, -acc.y), Vector3f32::new(gyro.z, -gyro.x, -gyro.y)),
            Self::XNEG_ZNEG_YNEG => {
                (Vector3f32::new(-acc.x, -acc.z, -acc.y), Vector3f32::new(-gyro.x, -gyro.z, -gyro.y))
            }
            Self::ZNEG_XPOS_YNEG => (Vector3f32::new(-acc.z, acc.x, -acc.y), Vector3f32::new(-gyro.z, gyro.x, -gyro.y)),
            Self::XPOS_ZPOS_YNEG => (Vector3f32::new(acc.x, acc.z, -acc.y), Vector3f32::new(gyro.x, gyro.z, -gyro.y)),
            Self::XPOS_YPOS_ZPOS_45 => (
                Vector3f32::new(acc.x * COS45 + acc.y * SIN45, -acc.x * SIN45 + acc.y * COS45, acc.z),
                Vector3f32::new(gyro.x * COS45 + gyro.y * SIN45, -gyro.x * SIN45 + gyro.y * COS45, gyro.z),
            ),
            Self::YPOS_XNEG_ZPOS_45 => (
                Vector3f32::new(acc.x * COS135 + acc.y * SIN135, -acc.x * SIN135 + acc.y * COS135, acc.z),
                Vector3f32::new(gyro.x * COS135 + gyro.y * SIN135, -gyro.x * SIN135 + gyro.y * COS135, gyro.z),
            ),
            Self::XNEG_YNEG_ZPOS_45 => (
                Vector3f32::new(acc.x * COS225 + acc.y * SIN225, -acc.x * SIN225 + acc.y * COS225, acc.z),
                Vector3f32::new(gyro.x * COS225 + gyro.y * SIN225, -gyro.x * SIN225 + gyro.y * COS225, gyro.z),
            ),
            Self::YNEG_XPOS_ZPOS_45 => (
                Vector3f32::new(acc.x * COS315 + acc.y * SIN315, -acc.x * SIN315 + acc.y * COS315, acc.z),
                Vector3f32::new(gyro.x * COS315 + gyro.y * SIN315, -gyro.x * SIN315 + gyro.y * COS315, gyro.z),
            ),
        }
    }

    #[must_use]
    #[inline]
    pub fn axis_order_inverse(self) -> Self {
        match self {
            Self::YPOS_XNEG_ZPOS => Self::YNEG_XPOS_ZPOS,
            Self::YNEG_XPOS_ZPOS => Self::YPOS_XNEG_ZPOS,
            Self::YPOS_ZPOS_XPOS => Self::ZPOS_XPOS_YPOS,
            Self::ZNEG_YPOS_XPOS => Self::ZPOS_YPOS_XNEG,
            Self::YNEG_ZNEG_XPOS => Self::ZPOS_XNEG_YNEG,
            Self::ZPOS_YPOS_XNEG => Self::ZNEG_YPOS_XPOS,
            Self::YPOS_ZNEG_XNEG => Self::ZNEG_XPOS_YNEG,
            Self::YNEG_ZPOS_XNEG => Self::ZNEG_XNEG_YPOS,
            Self::ZPOS_XPOS_YPOS => Self::YPOS_ZPOS_XPOS,
            Self::ZNEG_XNEG_YPOS => Self::YNEG_ZPOS_XNEG,
            Self::XPOS_ZNEG_YPOS => Self::XPOS_ZPOS_YNEG,
            Self::ZPOS_XNEG_YNEG => Self::YNEG_ZNEG_XPOS,
            Self::ZNEG_XPOS_YNEG => Self::YPOS_ZNEG_XNEG,
            Self::XPOS_ZPOS_YNEG => Self::XPOS_ZNEG_YPOS,
            Self::XPOS_YPOS_ZPOS_45 => Self::YNEG_XPOS_ZPOS_45, // 45 => 315
            Self::YPOS_XNEG_ZPOS_45 => Self::XNEG_YNEG_ZPOS_45, // 135 => 225
            Self::XNEG_YNEG_ZPOS_45 => Self::YPOS_XNEG_ZPOS_45, // 225 => 1355
            Self::YNEG_XPOS_ZPOS_45 => Self::XPOS_YPOS_ZPOS_45, // 315 => 45
            _ => self,                                          // other axis orders are self-inverting
        }
    }
}

impl TryFrom<u8> for AxisOrder {
    type Error = ();

    /// Validating conversion, invalid values return error.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let default = Self::default();

        if value == default as u8 {
            // The input was actually the default value.
            Ok(default)
        } else {
            let ret = Self::from_u8(value);
            if ret == default {
                // The input was invalid and got converted to the default.
                Err(())
            } else {
                Ok(ret)
            }
        }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: MaxSize + for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<AxisOrder>();
        #[cfg(feature = "serde")]
        is_serde::<AxisOrder>();
        #[cfg(feature = "storage")]
        is_storage::<AxisOrder>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ImuCommon;

    #[test]
    fn imu_state_default() {
        let state = ImuCommon::default();
        let z = Vector3f32::default();
        assert_eq!(state.acc_offset, z);
    }
    #[test]
    fn map_vector() {
        const INPUT: Vector3f32 = Vector3f32::new(2.0, 3.0, 5.0);
        let output = AxisOrder::map_vector(AxisOrder::XPOS_YPOS_ZPOS, INPUT);
        assert_eq!(Vector3f32::new(2.0, 3.0, 5.0), output);
        let output = AxisOrder::map_vector(AxisOrder::YPOS_XNEG_ZPOS, INPUT);
        assert_eq!(Vector3f32::new(3.0, -2.0, 5.0), output);
    }
    #[test]
    fn axis_order_inverse() {
        let input = AxisOrder::XPOS_YPOS_ZPOS;
        let output = AxisOrder::axis_order_inverse(input);
        let output_inverse = AxisOrder::axis_order_inverse(output);
        assert_eq!(input, output_inverse);
        for ii in 0..AxisOrder::COUNT {
            let axis_order = AxisOrder::from_u8(ii);
            let output = AxisOrder::axis_order_inverse(axis_order);
            let output_inverse = AxisOrder::axis_order_inverse(output);
            assert_eq!(axis_order, output_inverse);
        }
    }
}
