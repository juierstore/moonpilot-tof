# tof_sense_ms — Nooploop TOFSense / TOFSense-M / TOFSense-MS range finder

PX4 serial driver for the Nooploop (空循环) TOFSense family. It decodes the
**NLink** frames, filters invalid zones and publishes a standard
`distance_sensor` uORB message so EKF2 can fuse it as a range finder.

The driver is intentionally board independent: the serial port, mounting
orientation and aggregation mode come from standard PX4 parameters
(`SENS_TOFMS_CFG`, `SENS_TOFMS_ROT`, ...), from the command line, or both.

| Item | Value |
|---|---|
| Module | `tof_sense_ms` |
| Kconfig | `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS` |
| Device type | `DRV_DIST_DEVTYPE_TOFMS` (`0x9D`) |
| Output | `distance_sensor` (`type = LASER`, orientation configurable, default 25 = down) |
| Dependency | `drivers_rangefinder` (`PX4Rangefinder`) |
| Instances | 2 (`SENS_TOFMS_CFG` / `SENS_TOFMS_CFG2`) |
| Platforms | NuttX flight controllers and POSIX / SITL |

## Supported sensors

| Model | Frame | Length |
|---|---|---|
| TOFSense, TOFSense-F, TOFSense-S | single point, function `0x00` | 16 bytes |
| TOFSense-M / TOFSense-MS | zone map, function `0x01` | 400 bytes (8x8), 106/112 bytes (4x4) |

The frame type is detected automatically; no model parameter is required.

## Parameters

All parameters are in the QGC **Sensors** group. They are read at boot, a
reboot is required after changing them.

| Parameter | Default | Description |
|---|---|---|
| `SENS_TOFMS_CFG` | `0` (disabled) | Standard serial port selection for instance 0 (`101` TELEM 1, `102` TELEM 2, `103` TELEM 3, `104` SERIAL 4, `201` GPS 1, ...). Generating this parameter also creates the `rc.serial` autostart line. |
| `SENS_TOFMS_CFG2` | `0` | Optional second instance, e.g. a forward looking sensor. |
| `SENS_TOFMS_ROT` | `25` | Mounting orientation of instance 0 (`25` down, `0` forward, `24` up, ...). |
| `SENS_TOFMS_ROT2` | `25` | Mounting orientation of instance 1. |
| `SENS_TOFMS_AGG` | `0` | Aggregation of instance 0: `0` center zones, `1` median, `2` mean. |
| `SENS_TOFMS_AGG2` | `0` | Aggregation of instance 1. |
| `SENS_TOFMS_MIN` | `0.15` m | Minimum distance, also published as `distance_sensor.min_distance`. |
| `SENS_TOFMS_MAX` | `4.0` m | Maximum distance, also published as `distance_sensor.max_distance`. |

The baudrate of the selected port is taken from the matching
`SER_<port>_BAUD` parameter (`rc.serial` passes `-b p:${BAUD_PARAM}`), exactly
like the GPS driver. The sensor default is **921600 8N1**, so set
`SER_TELx_BAUD = 921600` for the selected port. The corvon_743v2 board
integration presets this.

## Command line

```
tof_sense_ms start [-i <0..1>] [-d <dev>] [-b <baud>] [-R <orientation>]
                   [-a <0|1|2>] [-m <min>] [-M <max>]
tof_sense_ms status [-i <instance>]
tof_sense_ms stop   [-i <instance>]
```

`-b`, `-R`, `-a`, `-m`, `-M` also accept `p:<param_name>` so the same command
line can be used from `rc.serial` with module parameters.

Examples:

```sh
# instance 0: TELEM 3, downward facing, center zone mean
tof_sense_ms start -i 0 -d /dev/ttyS4 -b 921600 -R 25 -a 0

# instance 1: SERIAL 4, forward facing, median of all zones
tof_sense_ms start -i 1 -d /dev/ttyS7 -b 921600 -R 0 -a 1

# statistics of both instances
tof_sense_ms status
```

## Status output

```
INFO  [tof_sense_ms] instance 0: /dev/ttyS4 @ 921600 Bd, orientation 25, aggregation center, range 0.15..4.00 m
INFO  [tof_sense_ms]   frames: 547 total, 10 Hz, published 547, no-valid-zone 0, out-of-range 0
INFO  [tof_sense_ms]   parser: header/length errors 0, checksum errors 0
INFO  [tof_sense_ms]   link: timeouts 0, last frame 32 ms ago, last distance 2.501 m (type 1, 64 zones)
tof_sense_ms:0 read: 11741 events, 0us elapsed
tof_sense_ms:0 com_err: 0 events
```

The counters are per instance and classify the failure modes separately:

| Counter | Meaning |
|---|---|
| `frames` | valid NLink frames decoded (checksum passed) |
| `Hz` | decoded frame rate over the last second |
| `published` | frames turned into a `distance_sensor` sample |
| `no-valid-zone` | frame decoded but the sensor marked every zone invalid |
| `out-of-range` | zones valid but every distance outside `[min, max]` |
| `header/length errors` | `0x57` followed by a bad function mark or a bad zone map, buffer overrun |
| `checksum errors` | frame checksum mismatch (line noise / wrong baudrate) |
| `timeouts` | more than one second without a valid frame; parser is resynchronised |

## Aggregation

The 8x8 (or 4x4) zone map cannot be fed to EKF2 directly, so the driver
reduces it to a single distance:

| Mode | Algorithm | Typical use |
|---|---|---|
| `0` center | mean of the center 4x4 zones (2x2 for 4x4 frames); if the center is invalid, fall back to the median of all valid zones | altitude hold (default) |
| `1` median | median of all valid zones | uneven ground, sparse obstacles |
| `2` mean | mean of all valid zones | flat ground, smoothest output |

Only zones with `status == 0` and `min <= d <= max` are used. A frame with no
usable zone is **not published** (EKF2 then times out naturally), which is
safer than publishing a fake distance. `signal_quality` is the mean signal of
the contributing zones clamped to 0..100.

## EKF2

`distance_sensor` is just a measurement; EKF2 has to be told to use it:

| Parameter | Recommended | Notes |
|---|---|---|
| `EKF2_RNG_CTRL` | `1` (conditional) or `2` (always) | required, otherwise the range finder is ignored |
| `EKF2_HGT_REF` | `0` (baro) or `2` (range) | `2` for indoor altitude hold |
| `EKF2_RNG_NOISE` | `0.1` m | tune to the observed jitter |
| `EKF2_RNG_A_HMAX` | e.g. `3.5` | slightly below the reliable range of the sensor |

Keep the barometer enabled: a range finder alone has no absolute reference and
cannot be the only height source for takeoff.

`distance_sensor.variance` is left at the PX4 default (0), like the other
native range finder drivers (tfmini, mb12xx, ...). EKF2 uses
`EKF2_RNG_NOISE` for the range measurement noise, not this field.

## Wiring

- Sensor TX -> flight controller RX, common ground, 5 V supply
- The sensor only transmits; RX on the flight controller is sufficient
- Use a port with enough RX buffer at high frame rates (400 bytes/frame,
  e.g. 10 Hz -> 4 kB/s, 100 Hz -> 40 kB/s)

## Tests

The protocol layer (`tof_sense_ms_parser.{h,cpp}`) has no PX4 dependency and is
covered by a host side test suite (vendor reference frame, point frames, 8x8 /
4x4 frames, checksum corruption, truncation, resynchronisation, invalid zone
maps, buffer overrun):

```sh
cd test
make test
# 77 checks, 0 failures
```

## Limitations

- Receive only; the query mode (`57 10 FF ..`) and the NAssistant
  configuration are not implemented.
- CAN variants of TOFSense are not supported (they use a different protocol).
- 4x4 mode is supported by the parser but has not been tested against real
  4x4 hardware.
- Aggregation is a simple robust statistic; no plane fitting or tilt
  compensation.
- The `signal_quality` mapping of the single point series is approximate.

## License

BSD-3-Clause, same as PX4.
