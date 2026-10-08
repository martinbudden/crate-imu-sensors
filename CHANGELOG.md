# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

Releases of the form `0.1.n` do not adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0.html),
that is each release may contain incompatible API changes.

Once the API has stabilized this project will adopt semantic versioning, the first release to do so will be `0.2.0`.

## [0.1.6] - 2026-11-xx

### Added

- `ImuBus` `is_spi()` function.
- IMU sensors now check `WHO_AM_I` on initialization.
- `with_alternative_address` builder method.
- started work on dummy byte required for Bmi270 SPI reads.

### Removed

- All use of `unwrap()`.

## [0.1.5] - 2026-10-07

### Added

- Support for continuous integration.
- Examples.

### Changed

- Updated to `vqm` `0.1.17`.
- Split `serde` feature into `serde` and `storage`.
- Updated to Rust version 1.89.
- Simplified error handling by using `ImuError` enum.
- Change to use `Reg` `struct` for IMU register constants.

### Removed

- Removed unneeded inclusions of `embassy-rp`, `embassy-stm32`, `esp-hal`, and `xtensa-lx` from `Cargo.toml`.

## [0.1.4] - 2026-09-05

### Changed

- updated to `vqm` `0.1.16`.

## [0.1.3] - 2026-08-04

### Added

- BMI279 driver.
- ICM200602 driver.
- feature flag for static mapping of IMU axes.
- clamping to `ImuMock` `set_acc` and `set_gyro`.
- default implementation of `ImuBus` trait.
- `map_acc_gyro_slice` function to `Imu` trait.

### Changed

- updated to `vqm` `0.1.14`.
- set gyro and acc scales in `init`.
- improved documentation.
- moved `map_acc`, `map_gyro`, and `map_acc_gyro` functions into `Imu` trait.
- improved IMU testing.
- tidied IMU implementations, especially use of registers.
- made use of `enum`s for acc and gyro scaling consistent.

### Removed

- `axis_order` parameter from `map_acc`, `map_gyro` etc functions.

## [0.1.2] - 2026-05-24

### Added

- QMI8658A driver.

### Changed

- updated to `vqm` `0.1.9`.
- made `serde` an optional feature.
- Changed Apache license to standard unabridged text.

## [0.1.1] - 2026-05-10

### Added

- `ImuMock`
- `.markdownlint.json`

### Changed

- all features are now lower case.
- updated to `vqm` `0.1.3`.
- improved gyro scaling for DPS and RPS (degrees and radians per second).

### Removed

- `ImuReading`

## [0.1.0] - 2026-04-25

Initial release.
