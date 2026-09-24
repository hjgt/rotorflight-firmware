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

Detection checks `WHO_AM_I` register `0x0f` for `0x75` (LSM6DSK320X) or `0x70` (LSM6DSV16X).
The register compatible LSM6DSV16X is driven by the same code and is reported as
`LSM6DSK320X_SPI`, so `status` names it `LSM6DSK320X` too. The gyro is detected automatically.
The accelerometer supports `acc_hardware = AUTO` (the default) or
`acc_hardware = LSM6DSK320X`. Hardware enum values are appended, preserving all existing IDs
including FAKE: gyro ID 22, accelerometer ID 23.

Any other ID leaves the gyro undetected, which `status` reports as `NOGYRO` together with
`Devices detected: SPI:0`. That includes the LSM6DSV320X (`0x0f` returns `0x73`). Note that
`0x70` is shared with the LSM6DSV32X, which upstream distinguishes by reading `CTRL8` bit 2;
this port does not implement that check, so an LSM6DSV32X would be configured as an
LSM6DSV16X.

`gyroregisters` dumps the LSM control registers (`0x0f`, `0x10`-`0x17`, `0x0d`) when one of
these sensors is the active gyro. Its MPU register set (`0x75`, `0x1a`, `0x1b`) is reserved on
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

## Known issues

The port is a copy of the pinned upstream revision above. Betaflight has since reworked the same
source file (`accgyro_spi_lsm6dsv16x.c` on `master`, where the LSM6DSV16X and LSM6DSK320X support
share one implementation). The defects below are still present here.

### `CTRL6` bit 3 is cleared, which is not allowed on this part

`lsm6dsk320xGyroInit()` writes `CTRL6` as `LPF1_G_BW | FS_G_2000DPS`, that is `0x04`, `0x14`,
`0x24` or `0x34` depending on the hardware LPF. On the 320X `FS_G` is only bits `[2:0]` and **bit 3
must be 1** (datasheet DS15060 Table 64, reset value `0x08`), so the value for ±2000 dps has to be
`0x0c`. Upstream now writes `(whoAmI == LSM6DSK320X_WHO_AM_I_CONST ? 0x08 : 0) | ...`.

This affects only the LSM6DSK320X: on the LSM6DSV16X `FS_G` occupies bits `[3:0]`, where `0x04` is
the correct value, so the fix has to be conditioned on the detected `WHO_AM_I` and must not be
applied unconditionally. Until then the LSM6DSK320X gyro full-scale selection is invalid.

### `OP_MODE` and `ODR` are programmed in a single register write

The port writes `OP_MODE` together with `ODR` (`CTRL1 = 0x19`, `CTRL2 = 0x1c`). Upstream splits
them: both sensors are put into the required `OP_MODE` while still powered down, then the ODR is
written after a short delay with the `OP_MODE` bits retained. Its comment records the consequence of
the combined write: "otherwise the rates become 7.68 kHz / 960 Hz". The rates declared in
`gyro_sync.c` (8000 Hz / 1000 Hz) therefore do not match the sensor's actual rates, which skews
rate-dependent filtering and the gyro integration.

### Initialisation is not verified

Upstream performs the software reset with retries, verifies every initialisation write by reading
the register back (`lsm6dsvWriteRegVerified()`, `LSM6DSV_INIT_ATTEMPTS` = 3) and reports
`FAILURE_GYRO_INIT_FAILED` when the sensor cannot be configured. The port does none of this, so a
sensor that ignores its configuration writes leaves the firmware running with a silently
unconfigured gyro and no warning in `status`.

Two related differences to review when re-syncing: upstream returns `false` from the read function
during `GYRO_EXTI_INIT` so that no stale sample is consumed, and it uses a driver-specific DMA
callback rather than the shared `mpuIntcallback`.

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
