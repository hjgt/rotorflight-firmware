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

Detection checks `WHO_AM_I` register `0x0f` for `0x75`. The gyro is detected automatically. The
accelerometer supports `acc_hardware = AUTO` (the default) or `acc_hardware = LSM6DSK320X`.
Hardware enum values are appended, preserving all existing IDs including FAKE: gyro ID 22,
accelerometer ID 23. The reset sequence verifies the identity again by re-reading `WHO_AM_I`.

Only the LSM6DSK320X is supported. The pin compatible LSM6DSV16X (`0x0f` returns `0x70`) and
LSM6DSV320X (`0x0f` returns `0x73`) are different parts and are not detected. Any other ID
leaves the gyro undetected, which `status` reports as `NOGYRO` together with
`Devices detected: SPI:0`.

`gyroregisters` dumps the LSM control registers (`0x0f`, `0x10`-`0x17`, `0x0d`) while an
LSM6DSK320X is the active gyro. Its MPU register set (`0x75`, `0x1a`, `0x1b`) is reserved on
this part and only reports `0x00`/`0xff`; note also that after a failed detection the CS pin is
handed back by `spiPreinitByTag()`, so every register then reads `0xff` regardless of the
hardware.

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

## Upstream alignment

The port started from the pinned revision above. Betaflight has since reworked the same source file
(`accgyro_spi_lsm6dsv16x.c` on `master`, where `lsm6dsvReset()`, `lsm6dsvConfigure()` and
`lsm6dsvGyroInit()` serve the LSM6DSV16X, LSM6DSV32X and LSM6DSK320X). The corrections and
robustness fixes below were ported from that work, so initialisation now follows the upstream flow.

- **`CTRL6` bit 3 must be 1.** The port wrote `CTRL6` as `LPF1_G_BW | FS_G_2000DPS`, that is `0x04`,
  `0x14`, `0x24` or `0x34`, clearing bit 3. On this part `FS_G` is only bits `[2:0]` and bit 3 must
  be 1 (datasheet DS15060 Table 64, reset value `0x08`), so the full-scale selection was invalid.
  `CTRL6` is now written as `0x0c`/`0x1c`/`0x2c`/`0x3c`.
- **`OP_MODE` and `ODR` are written separately.** The operating mode is programmed while both sensors
  are powered down, and the ODRs follow afterwards with those mode bits retained, gyro before
  accelerometer and 500 us apart. Setting mode and ODR in one write leaves the rates at
  7.68 kHz / 960 Hz instead of the 8 kHz / 1 kHz declared in `gyro_sync.c`.
- **Both sensors are powered down before `SW_RESET`** by clearing their ODRs, because an MCU-only
  reset may leave the IMU running (AN5763 sections 3.1 and 5.7). `SW_RESET` also sets `BDU` back to
  1, so `BDU` is cleared first: a dropped `SW_RESET` write can then no longer pass as a completed
  reset and leave the previous firmware's `CTRL8` in place.
- **The reset is polled, not assumed.** After the fixed 10 ms settling delay, `CTRL3` is read until
  `SW_RESET` clears with `BDU` set, bounded by `LSM6DSK320X_RESET_TIMEOUT_MS` (20 ms).
- **Every initialisation write is read back and verified**, and the whole sequence is retried
  `LSM6DSK320X_INIT_ATTEMPTS` (3) times.
- **A sensor that cannot be configured is reported** with `failureMode(FAILURE_GYRO_INIT_FAILED)`
  instead of being left silently unconfigured.
- **The read function returns `false` during `GYRO_EXTI_INIT`**, so the call that only configures
  acquisition cannot be mistaken for a sample.

## Remaining divergence from upstream

- **DMA callback.** Upstream uses a driver-specific `lsm6dsv16xDmaCallback` with
  `gyro->lsm6dsvDmaSequence` / `lsm6dsvDmaReady` bookkeeping, which requires those fields in
  `gyroDev_t`. This port keeps Rotorflight's shared `mpuIntcallback`, as the surrounding Rotorflight
  drivers do.
- **Single part.** Upstream distinguishes LSM6DSV16X, LSM6DSV32X and LSM6DSK320X with separate sensor
  enums and tells the 32X from the 16X by reading `CTRL8` bit 2. This driver detects and configures
  the LSM6DSK320X only.

## Validation

Run the focused host tests with:

```sh
make -C src/test test_accgyro_lsm6dsk320x_unittest
```

The tests cover chip identification, initialization registers, reset verification, scaling, LPF
options, sample-rate declarations, signed little-endian reads, DMA frame offsets, init failure
reporting and fallback when interrupts or DMA are absent. They use mocked SPI and cannot validate
electrical signals, actual interrupt timing, sensor noise or flight behavior.

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
