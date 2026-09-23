# ST LSM6DSK320X

This fork adds generic SPI support for the LSM6DSK320X accelerometer and gyroscope.
No board pin mapping, alignment, custom defaults or existing board configuration is changed.

## Source

- Rotorflight base: `7fea6d707a0c22207ba904af63ecae3a57f626de` (RF2 master).
- Betaflight source: [`accgyro_spi_lsm6dsv16x.c` at
  `805313c231e479f226689bfa27e2609580543418`](https://github.com/betaflight/betaflight/blob/805313c231e479f226689bfa27e2609580543418/src/main/drivers/accgyro/accgyro_spi_lsm6dsv16x.c).
- Original support: [Betaflight PR #14891](https://github.com/betaflight/betaflight/pull/14891),
  merged January 28, 2026.

The port extracts the LSM6DSK320X initialization and shared SPI read functions into
an independent driver. It preserves the upstream GPL license and does not add
LSM6DSV16X support. Unused register definitions are omitted.

## Configuration

`USE_ACCGYRO_LSM6DSK320X` enables the driver and SPI gyro infrastructure.
The unified `STM32F405`, `STM32F7X2`, `STM32F745`, `STM32G47X` and `STM32H743`
targets include it. F411 excludes it, consistent with the existing restrictions
on newer IMUs. Legacy board-specific targets can opt in with the same define.

Detection checks `WHO_AM_I` register `0x0f` for `0x75`. The gyro is detected
automatically. The accelerometer supports `acc_hardware = AUTO` (the default)
or `acc_hardware = LSM6DSK320X`. The CLI `status` command reports the new name.
Hardware enum values are appended, preserving all existing IDs including FAKE:
gyro ID 22, accelerometer ID 23.

| Setting | Value |
| --- | --- |
| Maximum SPI clock | 10 MHz (rounded down by the bus divider) |
| Gyro range / sensitivity | +/-2000 dps / 0.070 dps per LSB |
| Accelerometer range / 1 g | +/-16 g / 2048 LSB |
| Gyro / accelerometer ODR | 8000 Hz / 1000 Hz, high-accuracy ODR mode 1 |
| Gyro hardware LPF | Betaflight NORMAL / OPTION_1 / OPTION_2 register settings |
| Accelerometer LPF2 | Enabled, ODR/4 |
| Data ready | Pulsed gyro DRDY on INT1, rising edge |

The driver uses Rotorflight's per-device `hardware_lpf`, `mpuGyroInit()` and
`mpuIntcallback()` interfaces. DMA reads one command byte plus six little-endian
16-bit words (gyro XYZ, then accelerometer XYZ). If interrupts or DMA are unavailable,
it falls back to synchronous SPI reads. Register addresses are recorded on the gyro
device after generic MPU initialization. Scheduler sample rates match the programmed
ODRs on every enabled target; no MPU-style sample divider is applied.

The new gyro retains Rotorflight's conservative software overflow checking until
hardware testing establishes whether it can be marked as overflow protected.

## Validation

Run the focused host tests with:

```sh
make -C src/test test_accgyro_lsm6dsk320x_unittest
```

The tests cover chip identification, initialization registers, scaling, LPF options,
sample-rate declarations, signed little-endian reads, DMA frame offsets and fallback
when interrupts or DMA are absent. They use mocked SPI and cannot validate electrical
signals, actual interrupt timing, sensor noise or flight behavior.

Use the repository's ARM GCC 9.3.1 toolchain for full firmware builds:

```sh
make arm_sdk_install
make TARGET=STM32F405 -j4
make TARGET=STM32F7X2 -j4
make TARGET=STM32H743 -j4
```

Before hardware use, verify SPI/CS/INT1 resources, chip orientation, detection after
cold boot and reboot, accelerometer calibration, axis signs, measured sample rate,
CPU load and interrupt/DMA operation. In particular, the driver requests 8 kHz gyro
sampling on F405 as well as F7/H7. This differs from RF2's divided 4 kHz setup for
some other IMUs and requires a CPU-load check with the intended board configuration.

This is firmware support in this fork. It is not an upstream Rotorflight release or
a claim of bench/flight validation. An unmodified Configurator may not recognize the
new hardware IDs by name; use its CLI `status` and leave accelerometer selection on
`AUTO` until its sensor-name table is updated separately.
