# bno055_cyphal_reader

STM32G474 firmware for reading a BNO055 IMU and publishing the data over Cyphal.

This project follows the same VSCode + CMake workflow as `bldc_joint`.
It now uses:

- the newer HAL/CMSIS layout from the current STM32CubeG4 package
- `libcxxcanard` as a git submodule in `Drivers/libcxxcanard/`
- an `App/` layer for the firmware logic, instead of putting everything into `Core/Src`

## Requirements

- ARM GCC toolchain
- CMake 3.22+
- Ninja
- OpenOCD
- `gdb-multiarch`

## Build

Configure and build from the terminal:

```bash
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
```

Other presets are available too:

- `Debug`
- `RelWithDebInfo`
- `Release`
- `MinSizeRel`

## Flash

Flash over SWD with OpenOCD:

```bash
./scripts/flash_openocd.sh
```

## VSCode

Open the repository in VSCode and use the provided tasks:

- `Configure firmware`
- `Build firmware`
- `Build + Flash`
- `Build firmware + Debug (OpenOCD)`
- `Build + Flash + Debug (OpenOCD)`

## Project layout

- `App/` - application entrypoint and Cyphal communication logic
- `Core/` - CubeMX-generated board and IMU support code
- `Drivers/STM32G4xx_HAL_Driver/` - STM32 HAL driver sources
- `Drivers/CMSIS/` - CMSIS headers, DSP headers, and math library
- `Drivers/libcxxcanard/` - Cyphal stack as a git submodule
- `cmake/` - shared build tooling and toolchain files
- `.vscode/` - editor tasks, settings, and debug configs
- `scripts/flash_openocd.sh` - SWD flashing script

## Notes

- `Core/Src/main.c` only does hardware init and then calls `app()`.
- IMU sampling and Cyphal publishing live in `App/main.cpp` and `App/communication.cpp`.
- The project is set up for STM32G474 and FDCAN.
