# TIcar Light

MSPM0G3507-based firmware for a laser line-tracking car with a two-axis laser gimbal.

This repository contains the TI Code Composer Studio / TI Arm Clang version of the project. It targets a Dimensity-style MSPM0G3507 minimum system board, four closed-loop stepper drivers, digital gray sensors, an OLED menu, a JY61P yaw sensor, and a UART vision module.

> Current firmware line: `ccs1.2`<br>
> Active development branch: `1.2ccsadc`

## Features

- Differential-drive chassis controlled by STEP/DIR closed-loop stepper drivers.
- Timer-driven STEP pulse scheduler using TIMG0 at 50 kHz.
- Digital gray-sensor line tracking with a NO-YAW right-angle turn mode.
- Optional yaw-assisted motor tracking with JY61P heading feedback.
- Two-axis laser gimbal for vision-guided target tracking.
- UART vision link for laser/target error input.
- OLED menu for motor tests, gimbal tests, and mission entry.
- XDS110 build and safe flashing scripts.
- Recovery-oriented boot diagnostics and PA14 status LED.

## Hardware Overview

Main hardware used by the current firmware:

| Module | Interface | Notes |
| --- | --- | --- |
| MCU | TI MSPM0G3507 | CCS / TI Arm Clang project |
| Chassis motors | 2 x STEP/DIR drivers | Left and right differential drive |
| Gimbal motors | 2 x STEP/DIR drivers | Yaw and pitch axes |
| Gray sensors | Digital GPIO inputs | S1-S7, bit6-bit0 |
| OLED | I2C0 | PA0 SDA, PA1 SCL |
| JY61P | UART1 | Yaw feedback |
| Vision / Link | UART3 | Line-based target/laser error input |
| Log UART | UART0 | Type-C CH340, 115200 baud |

## Pin Map

### Stepper Drivers

| Axis | STEP | DIR | EN |
| --- | --- | --- | --- |
| Chassis left | PA13 | PB24 | PA28 |
| Chassis right | PA12 | PA22 | PA2 |
| Gimbal yaw | PA7 | PB18 | PA31 |
| Gimbal pitch | PA8 | PA9 | PB19 |

The chassis left/right logical mapping follows `config/pin_map.h`, where the real car wiring is already corrected in software.

### UART

| UART | Pins | Purpose |
| --- | --- | --- |
| UART0 | PA10 TX / PA11 RX | Type-C CH340 log UART |
| UART1 | PB6 TX / PB7 RX | JY61P |
| UART3 | PB2 TX / PB3 RX | Vision / Link / Exchange |

### Gray Sensors

Digital gray sensor inputs are mapped as `S1` to `S7`:

| Sensor | Pin | Mask bit |
| --- | --- | --- |
| S1 | PA15 | `0x40` |
| S2 | PA16 | `0x20` |
| S3 | PA17 | `0x10` |
| S4 | PA24 | `0x08` |
| S5 | PA25 | `0x04` |
| S6 | PA26 | `0x02` |
| S7 | PA27 | `0x01` |

### Reserved Pins

- PA0 / PA1: OLED I2C0.
- PA5 / PA6: external crystal hardware reservation.
- PA10 / PA11: UART0 log and BSL data lines.
- PA18: BSL invoke.
- PA14: debug/status LED.
- PA19 / PA20: SWD.
- PA21 / PA23: VREF-related pins.
- PB14 / PB15 / PB16 / PB17: onboard SPI flash reservation.

## Repository Layout

```text
app/            Application logic: menu, state machine, line tracking, gimbal, route, pose solver
config/         Board parameters and pin map
doc/            Wiring notes, recovery notes, style notes, project logs
generated/      SysConfig-style generated configuration
hardware/       Peripheral drivers: motor, gray, OLED, UART, key, stepper pulse
system/         Board init, interrupt entry, delay, system-level diagnostics
targetConfigs/  CCS target configurations for XDS110, J-Link, and recovery flows
tools/          Build, flash, recovery, and diagnostic scripts
```

## Quick Start

### Prerequisites

- Windows with PowerShell.
- TI Code Composer Studio or CCS Theia.
- TI Arm Clang toolchain compatible with the project settings.
- MSPM0 SDK 2.10.00.04 or matching SDK path configured in CCS.
- XDS110 debugger for the safe flashing script.

### Clone

```powershell
git clone https://github.com/Sagitta-Aria/TIcar-light.git
cd TIcar-light
git checkout 1.2ccsadc
```

### Build

```powershell
.\tools\build_ccs.ps1 -Clean
```

Successful builds produce:

```text
Debug\codex-build\light-car-ccs1.2.out
```

### Flash With XDS110

The safe flashing script downloads the MAIN program only. It does not perform Factory Reset, mass erase, or NONMAIN writes.

```powershell
.\tools\flash_xds110_safe.ps1 -SkipBuild
```

If the debugger is busy, close CCS debug sessions before flashing.

## Firmware Modes

### Motor NO YAW

`app/motor_no_yaw.c` implements digital-gray line tracking without yaw feedback.

Current behavior:

- Gray sensors are read as digital GPIO signals.
- S1/S2 within the configured time window triggers a right-angle turn.
- After trigger, the car continues forward briefly, then performs a strong right turn.
- Strong turn command: left wheel moves, right wheel stops.
- S2 reacquisition returns the car to normal line tracking.
- Normal line tracking always keeps both wheels above the configured minimum speed.
- UART logs are avoided during normal motion and only emitted after error stop.

Important parameters live in `config/board_config.h`:

```c
CAR_MOTOR_NO_YAW_BASE_SPEED_SPS
CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_SPS
CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS
CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US
CAR_MOTOR_NO_YAW_TURN_APPROACH_MS
CAR_MOTOR_NO_YAW_TURN_SPEED_SPS
```

### Motor Track

`app/motor_track.c` is the yaw-assisted tracking mode. It uses gray sensors plus JY61P yaw feedback for right-angle confirmation.

### Gimbal Tracking

`app/gimbal.c` controls the two-axis laser gimbal:

- X error controls `MOTOR_GIMBAL_1` / yaw.
- Y error controls `MOTOR_GIMBAL_2` / pitch.
- Deadband, PID-like gains, speed limits, offsets, and target selection come from `app/staticconfig.c`.
- Pitch travel is limited using relative STEP count from the moment gimbal tracking is enabled.

Vision input is parsed by the Link path. The expected line format is:

```text
centerDx,centerDy;circleDx,circleDy
```

Each value represents an error from the vision side. The active static configuration decides whether to use center error or circle error.

## Menu

The OLED menu is intentionally simple:

```text
Gimbal
Motor
Mission
```

Typical pages:

- `Gimbal`: choose one of the predefined static gimbal configurations.
- `Motor / Step Test`: fixed-distance chassis motor test.
- `Motor / Track Run`: line-tracking run.
- `Motor / NO YAW`: digital gray tracking without yaw feedback.
- `Mission`: placeholder for integrated competition tasks.

Long-press K2 exits running test pages.

## Status LED

PA14 is used as the board status LED:

| LED behavior | Meaning |
| --- | --- |
| Slow blink | Main loop is alive |
| Fast blink | Non-fatal board error, such as OLED/I2C issue |
| Solid on | Fatal board error |

## Development Notes

- STEP pulse output is handled in the TIMG0 interrupt.
- Interrupt handlers should only copy bytes, count events, clear flags, or set state flags.
- Do not print UART logs, refresh OLED, or run heavy control logic inside interrupts.
- Normal builds use the internal 32 MHz SYSOSC by default.
- XDS110 safe download should be preferred during normal development.
- Do not write NONMAIN, change BSL configuration, or run Factory Reset unless you are intentionally recovering the chip.

## Recovery

Recovery-related scripts are in `tools/`.

The normal safe flash path is:

```powershell
.\tools\flash_xds110_safe.ps1 -SkipBuild
```

Factory Reset scripts exist for recovery, but they require explicit confirmation flags and should not be part of the normal workflow.

For detailed recovery notes, see:

- `doc/XDS110_RECOVERY_DEBUG_LOG_2026-05-23.md`
- `tools/read_boot_diag.js`

## Roadmap

- Stabilize NO-YAW tracking on right-angle turns.
- Tune gimbal yaw/pitch direction, deadband, and speed limits on real hardware.
- Finalize the UART vision protocol.
- Add mission-level task orchestration.
- Calibrate chassis STEP-to-distance conversion.
- Add a formal open-source license.

## Contributing

This is an embedded firmware project tied to a specific robot build. Contributions are easiest to review when they are small and hardware-aware.

Recommended workflow:

1. Keep pin-map changes isolated and document the real wiring.
2. Build before submitting changes.
3. Avoid mixing generated SysConfig changes with unrelated application logic.
4. Describe whether a change was tested on hardware, simulated, or only compiled.

## License

No open-source license has been selected yet. Until a `LICENSE` file is added, reuse is not formally granted. Add a license such as MIT, Apache-2.0, or BSD-3-Clause before presenting this as a fully reusable open-source project.
