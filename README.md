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
- Car state transitions: `app\state_machine.c`.
- Line tracking: `app\tracking.c`.
- Gray sensor threshold or active level: `config\board_config.h`.
- Inter-board serial link: `hardware\link.c`.
- JY61P IMU protocol: `hardware\jy61p.c`.
- JQ8400 audio module protocol: `hardware\jq8400.c`.
