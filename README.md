# light-car1.0

MSPM0G3507 Keil/uVision firmware project for the laser tracking car.

Version: 1.0

## Directory layout

- `app/`: application logic. `main.c` only starts the board and runs `App_Task()`.
- `hardware/`: board devices and reusable hardware drivers, for example `motor.c`, `gray.c`, `encoder.c`, `key.c`, `link.c`, `jy61p.c`, `jq8400.c`, and `oled.c`.
- `system/`: system-level startup glue, delay, board bring-up, and interrupt dispatch.
- `config/`: project constants and pin aliases. Change thresholds, tracking speed, and pin mapping here first.
- `generated/`: low-level TI DriverLib/SysConfig-style peripheral initialization.
- `MDK-ARM/`: Keil project file and startup assembly.
- `doc/`: pin map and wiring notes.

## Build

Open `MDK-ARM\light-car1.0.uvprojx` with Keil uVision, or build from command line:

```powershell
& 'D:\keil\UV4\UV4.exe' -b 'D:\激光循迹\light-car1.0\MDK-ARM\light-car1.0.uvprojx' -j0 -o 'D:\激光循迹\light-car1.0\Listings\build.log'
```

The project uses the TI MSPM0 SDK at:

```text
D:\Ti\mspm0_sdk_2_10_00_04
```

## Where to add code

- Motor control: `hardware\motor.c`, higher-level speed helpers in `app\motion.c`.
- Motor direction confirmation: `app\motor_test.c`.
- Route planner: `app\route.c`.
- Encoder speed loop: `app\speed_control.c`.
- Car state transitions: `app\state_machine.c`.
- Line tracking: `app\tracking.c`.
- Gray sensor threshold or active level: `config\board_config.h`.
- Inter-board serial link: `hardware\link.c`.
- JY61P IMU protocol: `hardware\jy61p.c`.
- JQ8400 audio module protocol: `hardware\jq8400.c`.

## Motor direction confirmation

`CAR_ENABLE_MOTOR_TEST_MODE` is normally `0`, so the test module is kept as a reusable debug tool without changing normal tracking startup.

To enable it, temporarily set:

```c
#define CAR_ENABLE_MOTOR_TEST_MODE      (1U)
```

Steps:

1. Lift the car so both wheels are off the ground.
2. Press Key 1 to run the next low-speed test step.
3. Press Key 2 at any time to stop.
4. Observe whether each step matches its name on the UART log:

```text
left forward
left reverse
right forward
right reverse
both forward
both reverse
```

Each step runs for `CAR_MOTOR_TEST_RUN_MS` and then stops automatically. After the motor direction is confirmed, set `CAR_ENABLE_MOTOR_TEST_MODE` to `0` to restore Key 1 as the tracking start key.

## Route planner

`route.c` is the outer planning layer for the square track. It uses encoder relative ticks to decide when the car is close to a corner, lowers the base duty before the right angle, and uses JY61P yaw when available to decide when the 90 degree turn is complete.

The tracking controller still follows the line with gray sensors. The route layer only changes speed and turn-limit targets so the car enters corners more gently.

Tune the route constants in `config\board_config.h` after testing the real car.

## Encoder speed loop

`speed_control.c` uses the current GPIO AB-phase encoder counts as a temporary backend and runs a conservative PI speed loop every `CAR_SPEED_CONTROL_PERIOD_MS`. The upper layers still send signed left/right speed commands through `Motion_SetSpeed()`, so tracking code does not need to care whether the car is open-loop or closed-loop.

Start real-car tuning from these constants in `config\board_config.h`:

- `CAR_ENABLE_SPEED_CONTROL`
- `CAR_SPEED_MAX_TARGET_TICKS`
- `CAR_SPEED_KP`
- `CAR_SPEED_KI`
- `CAR_SPEED_MIN_ACTIVE_DUTY`
- `CAR_SPEED_LEFT_ENCODER_SIGN`
- `CAR_SPEED_RIGHT_ENCODER_SIGN`

If a wheel spins forward but its measured speed is negative, change that wheel's encoder sign from `1` to `-1`.
