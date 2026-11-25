# boards/corvon/743v2

Board integration snippets for the CORVON 743V2 (STM32H743VIH6).

| File | Purpose |
|---|---|
| `default.px4board.append` | `CONFIG_DRIVERS_DISTANCE_SENSOR_TOF_SENSE_MS=y` (+ `UXRCE_DDS_CLIENT`) |
| `init/rc.board_defaults.append` | `SENS_TOFMS_*` defaults and the TELEM 2 / CM5 uXRCE-DDS configuration |

Apply them automatically (idempotent):

```bash
python3 tools/apply_to_px4.py --px4-root ~/px4/PX4-Autopilot --board corvon/743v2
```

Or apply the prebuilt patch:

```bash
cd ~/px4/PX4-Autopilot
git apply /path/to/moonpilot-tof/patches/corvon_743v2_tof.patch
```

Full explanation (port map, CM5 boundary, build, QGC): **[docs/corvon_743v2.md](../../../docs/corvon_743v2.md)**.
