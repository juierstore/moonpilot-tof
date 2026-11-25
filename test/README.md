# test

| Path | Purpose |
|---|---|
| `test_parser.cpp` | Host unit test for the pure C++ NLink parser |
| `tof_sample_frame.h` | 400 byte reference frame from the Nooploop documentation |
| `Makefile` | `make test` / `make clean` |
| `sitl/` | PX4 SITL two-instance and error-path smoke tests |

```bash
make test
# 77 checks, 0 failures
```

SITL:

```bash
cp ../boards/px4/sitl/tof.px4board ~/px4/PX4-Autopilot/boards/px4/sitl/
cd ~/px4/PX4-Autopilot && make px4_sitl_tof -j"$(nproc)"
cd - && ./sitl/run_sitl_tof.sh
```

See [../docs/SITL_TESTS.md](../docs/SITL_TESTS.md).
