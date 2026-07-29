# 产品 Profile 与扩展边界

工程只有一个代码库，但有两个编译期产品 Profile。不要通过复制一套 `xxx_gmr.c`
来增加新功能；先判断它属于公共能力、产品组合还是硬件资源。

## 选择方式

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Clean
.\tools\build_ccs.ps1 -Profile Full -Clean
.\tools\build_ccs.ps1 -Profile Gmr -Board Dimeng -BluetoothRole Slave -Clean
```

未传 `-Profile` 时默认构建 `Gmr`，未传 `-Board` 时默认选择 `Tianmeng`。构建脚本写入
`CAR_ACTIVE_PROFILE=CAR_PROFILE_GMR/FULL`和板型宏，并只编译该 Profile 拥有的产品源文件。
旧的 `CAR_LIBRARY_GMR_CONFIG_ENABLED=0/1` 仅保留为构建兼容输入，新代码禁止使用。

## 配置归属

| 需求 | 修改位置 |
| --- | --- |
| 选择 Gmr 或 Full | `tools/build_ccs.ps1 -Profile ...` |
| 改某个产品的默认库组合 | `config/profiles/profile_gmr.h` 或 `profile_full.h` |
| 新增一种算法/驱动方法及依赖检查 | `config/library_config.h` |
| 查看 UART、引脚和外设所有权 | `config/resource_config.h`、`config/pin_map.h` |
| 改实车速度、时间和任务参数 | `config/board_config.h`、`config/control_config.h` |
| 增加菜单任务名和启动事件 | `app/task_registry.c` |
| 增加两产品共用的运行时能力 | 公共 `app/`、`hardware/` 或 `system/` 模块 |
| 增加产品独有的大流程 | Profile 专属源文件，并登记到构建脚本源文件清单 |

`library_config.h`只定义“有哪些方法”、派生开关和非法组合检查；它不再保存某个
产品的当前默认值。业务驱动只读取 `CAR_LIBRARY_*_ENABLED` 和
`CAR_PROFILE_IS_GMR/FULL`，不得自行硬编码 UART 实例。

## 当前资源矩阵

| 资源 | Gmr | Full |
| --- | --- | --- |
| PC 调试串口 | UART0，PA10 TX / PA11 RX | UART0 TX；H7 IMU启用时PA11由H7独占 |
| 板载 JY61 | UART1保留，Task5不使用 | UART1，供车身姿态前馈 |
| UART3 PB2/PB3 | 默认是外部M0姿态；地猛星蓝牙构建改由HC-05独占 | K230视觉链路 |
| 两车蓝牙 | 天猛星UART2 PB15/PB16；地猛星UART3 PB2/PB3 | 仅天猛星UART2 PB15/PB16 |
| H7 IMU | `CAR_LIBRARY_H7_IMU_NONE` | `CAR_LIBRARY_H7_IMU_UART_JY61` |
| 天猛星SPI六轴 | 默认关闭，可选IMU660RA/RB/RC | 默认关闭，可选IMU660RA/RB/RC |
| H7 LCD | 关闭 | 关闭 |
| 本地 SSD1306 | 开启 | 开启 |

Gmr 默认由外部M0姿态独占UART3，Task5只读取该链路；地猛星启用蓝牙后UART3
切给HC-05，外部M0姿态和依赖它的Task5航向反馈不可用。板载JY61及UART1仍保留。
Full的UART3固定给K230，因此`Full + Dimeng + Bluetooth`会在编译期拒绝。

## 增加任务

1. 在 `task_registry.c` 对应 Profile 表中增加显示名、missionId和启动事件。
2. 在状态机中实现该 missionId 的进入、周期和退出行为。
3. 只有任务确实需要专用子页时才修改对应菜单；普通直接启动任务不再增加菜单
   `switch`。
4. 两个产品行为相同的代码放公共模块；行为不同但接口相同的实现才允许保留
   Profile 专属文件。
5. 至少构建 `Gmr` 和 `Full`，涉及功能开关时再增加关闭组合构建。

## 当前公共边界

- `app/app_runtime.c`：两产品相同的按键活跃检测和状态机调度胶水。
- `app/task_registry.c`：菜单索引、名称、missionId和启动事件的唯一映射。
- `app/tuning_common.c`：两套UART调参台共用的文本解析和PWM百分比换算。
- `config/resource_config.h`：调试、蓝牙、外部M0姿态和H7 UART的唯一资源别名。

现有 `gmr_app/menu/state_machine/tuning_console` 仍包含真实的产品行为差异，后续应按
上述边界逐块抽取，不应一次合并成充满条件编译的单个大文件。
