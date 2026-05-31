# AD5933 Evaluation Board Software

A C++17 host-side driver and CLI for the Analog Devices **EVAL-AD5933 / EVAL-AD5934**
impedance analyzer evaluation boards. The library wraps the Cypress FX2 USB
bridge (via `libcyusb` / `libusb-1.0`), downloads the board firmware, and
exposes a small object-oriented API to:

- program the AD5933 sweep registers,
- read on-die temperature,
- run a frequency sweep and collect raw real/imag samples,
- perform a resistor-based gain-factor calibration (mid-point or multi-point),
- save the calibrated magnitude/phase data as CSV.

A thin command-line front-end (`eval_ad5933_program`) is provided on top of the
library so common sweeps can be run without recompiling.

---

## Repository layout

```
evalAd5933/
├── CMakeLists.txt
├── cypressfw/
│   └── AD5933_34FW.hex          # FX2 firmware loaded at connect()
├── include/
│   ├── ad5933types.h            # register map, enums, POD types
│   ├── ad5933math.h             # pure, hardware-free math helpers
│   └── libad5933.h              # Ad5933 class
├── lib/
│   └── libad5933.cpp            # Ad5933 implementation
├── src/
│   └── main.cpp                 # CLI front-end
└── tests/
    └── test_math.cpp            # hardware-free unit tests
```

---

## Dependencies

| Dependency | Notes |
|------------|-------|
| C++17 compiler | tested with GCC 13 |
| CMake >= 3.10 | |
| `libusb-1.0` (dev) | `sudo apt install libusb-1.0-0-dev` |
| **cyusb_linux** | Build `libcyusb.so` first. Expected at `../cyusb_linux/` relative to this repo (override with `-DLIB_CYUSB_DIR=...`). |

You will also need USB permissions and a `cyusb` config entry for the EVAL
board (VID `0x0456`, PID `0xb203`) — see the next section.

---

## Permissions and cyusb configuration

`libcyusb` only opens devices whose VID/PID appear in its `<VPD>` allow-list,
and it needs read/write access to the USB device node. Without both, every
call to `connect()` fails with `cyusb_open failed: -19` ("device not found")
even though `lsusb` sees the board.

### 1. Whitelist the AD5933 in `cyusb.conf`

The library looks for `~/.config/cyusb/cyusb.conf` first, then
`/etc/cyusb.conf`. Start from the template shipped with cyusb_linux and add
the AD5933 entry **inside** the `<VPD>` block:

```sh
mkdir -p ~/.config/cyusb
cp ../cyusb_linux/configs/cyusb.conf ~/.config/cyusb/cyusb.conf
sed -i 's|</VPD>|0456\tb203\t\tEVAL-AD5933\n</VPD>|' ~/.config/cyusb/cyusb.conf
```

### 2. Allow non-root USB access via a udev rule

```sh
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0456", ATTRS{idProduct}=="b203", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-ad5933.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Replug the board afterwards. Verify with `ls -l /dev/bus/usb/<bus>/<dev>` —
the node should now be world-RW.

Without the udev rule the program still works under `sudo -E` (preserve
`HOME` so it finds the user `cyusb.conf`).

---

## Building

```sh
# 1. Build libcyusb (see the cyusb_linux README) and leave it at ../cyusb_linux
# 2. Configure + build this project
cmake -S . -B build
cmake --build build -j
```

Useful CMake options:

| Option | Default | Description |
|--------|---------|-------------|
| `LIB_CYUSB_DIR` | `../cyusb_linux` | Path to the cyusb_linux source tree |
| `EVAL_AD5933_BUILD_TESTS` | `ON` | Build the hardware-free math tests |
| `AD5933_FW_INSTALL_PATH` | `${CMAKE_INSTALL_PREFIX}/share/ad5933/AD5933_34FW.hex` | Firmware path baked into the shared library as the default |

### Run the unit tests

```sh
cd build
ctest --output-on-failure
```

These cover frequency-code conversion, temperature decoding, complex-sample
decoding (including negative quadrants), and gain-factor math. They do **not**
require any hardware to be attached.

### Install

```sh
sudo cmake --install build
```

Installs the shared library, headers, the CLI, and the firmware hex file into
`${CMAKE_INSTALL_PREFIX}` (default `/usr/local`).

---

## Using the CLI

With the board plugged in:

```sh
./build/eval_ad5933_program --fw cypressfw/AD5933_34FW.hex \
    --start 30000 --delta 100 --steps 200 --settle 50 \
    --vrange 1 --pga 1 --r1 10000 \
    --output sweep.csv
```

All flags:

```
--start <Hz>            start frequency                          (default 3500)
--delta <Hz>            frequency increment                      (default 50)
--steps <n>             number of increments, max 511            (default 100)
--settle <n>            settling cycles, max 511                 (default 50)
--refclk <Hz>           reference clock frequency                (default 13e6)
--r1 <Ohm>              calibration resistor (RES_ONLY)          (default 10000)
--vrange <2|1|0.4|0.2>  output excitation Vpp                    (default 1)
--pga <1|5>             PGA gain                                 (default 1)
--clk <int|ext>         clock source                             (default ext)
--fw <path>             FX2 firmware .hex path                   (default: installed)
--output <file>         CSV output path                          (default captured_data.csv)
--quiet                 suppress per-sample progress
-h, --help              show usage
```

The program runs **two sweeps**: a first one over the calibration resistor to
compute the gain factor, then a second one whose results — calibrated
magnitude and phase — are written to the CSV. CSV columns:

```
index, frequency_hz, magnitude, phase_deg
```

`magnitude` is `1 / |Z|` after calibration (the AD5933 datasheet convention);
`phase_deg` is in `[-180, 180]`.

---

## Using the library

```cpp
#include "libad5933.h"

UserParameters_st p{
    13'000'000,        // mRefClockFrequency
    30'000, 100,       // mStartFrequency, mDeltaFrequency
    200, 50,           // mNumberOfIncrements, mNumberSettlingTimeCycles
    DEFAULT_X1,
    EXTERNAL_CLOCK,
    RANGE_1VPP,
    GAIN_X1,
    RES_ONLY, MID_POINT,
    10'000.0,          // mR1 (calibration resistor)
    0.0, 0.0,
};

Ad5933 dev;
dev.connect(0x0456, 0xb203, "cypressfw/AD5933_34FW.hex");
dev.setDeviceParameters(&p);

dev.programDeviceRegisters();
dev.startSweep();          // calibration sweep
dev.doCalibration();

dev.programDeviceRegisters();
dev.startSweep();          // measurement sweep, now calibrated
dev.saveData("sweep.csv");
dev.deinit();
```

All error conditions throw `std::invalid_argument`, `std::logic_error`,
`std::runtime_error` or `CyusbError` — wrap the calls in a `try` block.

A progress callback can be registered to observe each sample as it is captured:

```cpp
dev.setProgressCallback([](unsigned int i, const ImpedData_ct& s) {
    std::printf("[%u] f=%.1f |Z_raw|=%.3f phase=%.2f\n",
                i, s.frequency, s.m, s.phase);
});
```

### Required call order

1. `connect(vid, pid, fwPath)` — opens USB, downloads FX2 firmware.
2. `setDeviceParameters(&params)` — validates and stores the parameter struct
   (the pointer must remain valid for the lifetime of the `Ad5933` instance).
3. `programDeviceRegisters()` — pushes sweep configuration to the chip.
4. `startSweep()` — runs the sweep, blocks until done.
5. `doCalibration()` (optional but recommended) — computes the gain factor.
6. *(Repeat 3–4 for measurement sweeps; calibration persists.)*
7. `saveData(path)` — write CSV.
8. `deinit()` — close USB.

`readTemperature()` can be called at any point between `connect()` and
`deinit()`.

---

## Current limitations

- **Calibration circuits.** Only `RES_ONLY` (pure resistive DUT) is
  implemented. `CAP_ONLY`, `RES_CAP_SERIES`, `RES_CAP_PARALLEL` and
  `COMPLEX_CIRCUIT` are present in the enum but `setDeviceParameters()` will
  throw if selected. Adding these requires the corresponding analog model and
  hardware to validate against.
- **Single-range sweep.** The AD5933 frequency sweep is limited to 100 kHz
  per the datasheet, and this library enforces that as the upper bound on
  start/delta frequencies. Multi-range stitching is not implemented.
- **Synchronous, blocking API.** `startSweep()` blocks until the sweep
  completes or polling times out. There is no cancellation API.
- **No PGA-gain auto-ranging.** Output range and PGA gain are user choices;
  the library does not detect saturation or out-of-range samples.
- **Output is `1/|Z|`, not impedance in ohms.** The datasheet convention is
  kept; invert in your post-processing if you want |Z| in ohms.
- **Phase is wrapped to `[-180°, 180°]`.** No unwrapping across sweep points.
- **Single device per process.** `cyusb_open` / `cyusb_close` are used as
  globals; concurrent `Ad5933` instances are not supported.
- **Firmware path** must be reachable at runtime. The compiled-in default
  points at `AD5933_FW_INSTALL_PATH` (see CMake options); pass `--fw` or
  the third argument to `connect()` to override.
- **Linux only**, tested with libusb-1.0 + cyusb_linux. Windows / macOS are
  out of scope.

---

## License

BSD 3-Clause License. See [LICENSE](LICENSE).
