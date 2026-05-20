# light-car1.0ccs

TI CCS / TI Arm Clang version of the MSPM0G3507 laser tracking car firmware.

This project is a clean CCS project created from the TI MSPM0G3507 empty template, then populated with the existing modular light-car source folders:

- `app/`
- `hardware/`
- `system/`
- `config/`
- `generated/`

Use this project for the CCS line of development. The version name for GitHub tracking is `1.0ccs`.

Build from PowerShell:

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

This build only compiles and links. It does not download to the board.

Open in CCS:

1. `File -> Import Project(s)`
2. Select `D:\Ti\light-car1.0ccs`
3. Use `SEGGER J-Link Emulator` with target `MSPM0G3507`

Do not use this folder as a Keil project. The Keil line remains separate at `D:\激光循迹\light-car1.0`.
