# `imu-sensors` Rust Crate<br>[![Crates.io](https://img.shields.io/crates/v/imu-sensors.svg)](https://crates.io/crates/imu-sensors) [![Documentation](https://docs.rs/imu-sensors/badge.svg)](https://docs.rs/imu-sensors) [![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

`imu-sensors` is a Rust crate that implements drivers for a variety of Inertial Measurement Units (IMUs) on both I2C and SPI.

`imu-sensors` uses the [Embassy framework](https://github.com/embassy-rs/embassy) to support Raspberry Pi Pico, STM32, and ESP32 microcontrollers.

This crate is `no_std`, `no alloc`, and the Minimum Supported Rust Version (MSRV) is `Rust 1.89`.

## IMUs

The following IMUs are supported:

| IMU                                                                                                 | ID            |
| ----------------------------------------------------------------------------------------------------| ------------- |
| Bosch [BMI270](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)                | `BMI270`      |
| TDK [ICM-20602](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20602/)              | `ICM420602`   |
| TDK [ICM-42605](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42605/)              | `ICM426xx`    |
| TDK [ICM-42688-P](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)          | `ICM426xx`    |
| TDK [MPU-6000](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6000) | `MPU6000`     |
| ST [ISM330DHCX](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)                             | `ISM330DHCX`  |
| ST [LSM6DS3TR-C](https://www.st.com/en/mems-and-sensors/lsm6ds3tr-c.html)                           | `LSM6DS3TR_C` |
| ST [LSM6DSOX](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)                                 | `LSM6DSOX`    |
| QST [QMI-8658](https://www.qstcorp.com/en_imu_prod/QMI8658/)                                        | `QMI8658`     |
| Invensense MPU-6886                                                                                 | `MPU6886`     |
| ImuMock - mock IMU for testing                                                                      | `ImuMock`     |

The ICM-42605 and ICM-42688 are broadly compatible and share the same driver.

The LSM6DS3TR-C, ISM330DHCX, and LSM6DSOX are broadly compatible and share the same driver.

The MPU-6886 is an IMU that is used by M5 Stack devices, it does not seem to be used anywhere else.

The MPU-6000, although discontinued, is still important. It was widely used as the preferred IMU on many flight controllers:
these flight controllers can be repurposed for other projects.

## Crate status

> **⚠️ Note:** This crate is currently under active development.
>
> The implementations above are provisional and require further testing.

## Dependencies

This library uses the [vqm(vector quaternion matrix) crate](https://crates.io/crates/vqm) for its `Vector3f32` classes.

## Original implementation

I originally implemented this crate as a C++ library:
[Library-Sensors](https://github.com/martinbudden/Library-Sensors).

## License

Licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT)>

at your option.
