# 电赛复用库使用手册

## 配置分层

工程把可复用能力分成四层，比赛前不要把算法选择重新写进任务代码：

| 文件 | 只负责什么 |
| --- | --- |
| `config/profile_select.h` | 定义并判断当前Gmr/Full产品模式 |
| `config/profiles/profile_*.h` | 选择该产品默认启用的库和具体方法 |
| `config/library_config.h` | 定义可选方法、派生开关和依赖检查 |
| `config/resource_config.h` | 统一声明调试/H7 UART资源所有权 |
| `config/board_config.h` | 赛道速度、转向时间、任务行为和板级数值 |
| `config/control_config.h` | 已标定的电机PI、视觉增益和双IMU闭环参数 |
| `app/staticconfig.c` | point/circle二维视觉参数表 |
| `config/pin_map.h` | 板卡接线和外设引脚 |

CCS会根据当前Profile的选择高亮对应`#if`参数分支。不要修改派生开关，
也不要在`motor_no_yaw.c`、`gimbal.c`或`state_machine.c`里临时改方法。

## 已封装方法

| 类别 | 可选方法 | 当前默认 | 状态 |
| --- | --- | --- | --- |
| 主控板引脚 | 地猛星48P、天猛星64P | 地猛星48P | 两套均通过TI Arm Clang编译链接 |
| 灰度输入 | 7路数字GPIO、7路模拟ADC | 7路数字GPIO | 数字方案已用于当前车；模拟采样保留 |
| 循迹算法 | 关闭、数字量灰度加权 | 数字量灰度加权 | 已用于Task1/Task4 |
| 循迹输出 | 直接PWM、PWM+编码器交叉同步、编码器速度闭环 | 编码器速度闭环 | 三种组合均通过编译；参数独立 |
| 直角转向 | 关闭、单轮锁死、双轮反转 | 双轮反转 | 两种转向代码和参数完整保留 |
| 强转速度 | 固定速度、JY61转角线性曲线 | JY61转角曲线 | 已接入；当前起始/接近速度相同 |
| 二维云台 | 关闭、视觉P/D+趋势前馈 | 二维视觉 | point/circle两套已调参数 |
| 姿态矫正 | 关闭、H7反馈+JY61前馈双IMU | 双IMU | Task4辅助和Task8独占模式 |
| 起步搜点 | 关闭、固定yaw搜索 | 固定yaw搜索 | 当前400 SPS |
| 丢目标恢复 | 关闭、yaw往复扫描 | 关闭 | 框架已接入，速度/摆幅尚未实车标定 |
| 强转固定随动 | 关闭、近中远分段STEP | 关闭 | 框架已接入，STEP数尚未实车标定 |
| H7 LCD显示 | 关闭、UART文本行协议 | 两Profile均关闭 | Full使用UART0、GMR使用UART3；支持0～9行增量刷新 |
| H7 IMU输入 | 关闭、UART JY61兼容帧 | GMR关闭、Full开启 | 与板载UART1 JY61独立 |
| 天猛星SPI六轴 | 关闭、自动识别、IMU660RA/RB/RC | 两Profile均关闭 | SPI1统一驱动已接入；尚未实物验证 |
| 本地OLED显示 | 关闭、I2C SSD1306 | 两Profile均开启 | 支持前5行ASCII菜单；可与H7 LCD同时开启 |

“框架已接入”不等于可直接比赛。两个未标定方法的参数仍为0，启用前必须架空、
限速并完成实车验证。

## 当前任务依赖

| 任务 | 必需库 |
| --- | --- |
| Task1 | 循迹算法 + 直角转向 |
| Task2/Task3/Task7 | 二维视觉云台 |
| Task4 | 循迹算法 + 直角转向 + 二维视觉云台；双IMU可单独关闭 |
| Task5 | 始终保留；关闭视觉/双IMU后对应`mode`命令会拒绝启动 |
| Task6 | 始终保留，用于底盘开环/闭环测试 |
| Task8 | 双IMU姿态矫正 |
| Task9 | 始终保留，用于编码器手推检查 |

缺少依赖的任务会自动从显示菜单轮换中跳过；状态机也会拒绝对应启动事件。
RTOS任务仍保持静态创建，关闭库不会改变优先级、任务栈或10 ms截止时间。
两个显示库都关闭时，任务和按键状态机仍正常运行，只是不再输出菜单画面。

## 常用组合

当前完整Task4方案只需保留默认选择。

只有循迹、没有云台：

```c
#define CAR_LIBRARY_GIMBAL_TRACKING_METHOD \
    CAR_LIBRARY_GIMBAL_TRACKING_NONE
#define CAR_LIBRARY_GIMBAL_ATTITUDE_METHOD \
    CAR_LIBRARY_GIMBAL_ATTITUDE_NONE
```

简单赛道使用直接PWM和单轮锁死：

```c
#define CAR_LIBRARY_LINE_DRIVE_METHOD \
    CAR_LIBRARY_LINE_DRIVE_DIRECT_PWM
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD \
    CAR_LIBRARY_RIGHT_ANGLE_TURN_LOCKED_INNER
```

只要普通加权循迹、不识别直角：

```c
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD \
    CAR_LIBRARY_RIGHT_ANGLE_TURN_NONE
```

该组合仍保留`MotorNoYaw_ApplyTask1LineCommand()`供新任务复用，但现有Task1/Task4
按圈数依赖直角计数，因此会从菜单隐藏。

只使用本地I2C OLED：

```c
#define CAR_LIBRARY_H7_LCD_METHOD \
    CAR_LIBRARY_H7_LCD_NONE
#define CAR_LIBRARY_LOCAL_OLED_METHOD \
    CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306
```

两个屏幕同时显示同一份菜单：

```c
#define CAR_LIBRARY_H7_LCD_METHOD \
    CAR_LIBRARY_H7_LCD_UART_TEXT
#define CAR_LIBRARY_LOCAL_OLED_METHOD \
    CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306
```

完全关闭显示：

```c
#define CAR_LIBRARY_H7_LCD_METHOD \
    CAR_LIBRARY_H7_LCD_NONE
#define CAR_LIBRARY_LOCAL_OLED_METHOD \
    CAR_LIBRARY_LOCAL_OLED_NONE
```

OLED字体大小在`config/board_config.h`的当前高亮OLED参数段设置。显示选择只影响
初始化和UI输出，不会关闭当前板型的状态灯，也不会改变任务状态机和电机控制周期。

## Flash容量约束

MSPM0G3507可用Flash为128 KiB。2026-07-23用TI Arm Clang 4.0.4 LTS、`-O2`构建当前完整方案：

| 板型 | Flash text | 占用 | 剩余 |
| --- | ---: | ---: | ---: |
| 地猛星48P | 101160 B | 77.2% | 29912 B |
| 天猛星64P | 101112 B | 77.2% | 29960 B |

引脚宏和未被使用的常量是编译期选择，不会把两套板型数据同时放入Flash。
真正会增长固件的是新驱动、字库、协议解析、浮点数学和日志字符串。后续可选模块必须用
`#if CAR_LIBRARY_...`包住实现和大型常量，并在每次加库后重新运行`tiarmsize.exe`。
建议比赛版长期至少保留10～15 KiB余量，避免最后联调时才触发链接溢出。

IMU660RB/RC明确型号构建不包含BMI270配置流；IMU660RA和AUTO构建会额外包含
8192字节官方配置数据。接线、构建命令和统一采样接口见`doc/IMU660RX.md`。

## 组合构建

分别构建两个产品：

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Clean
.\tools\build_ccs.ps1 -Profile Full -Clean
```

不改配置文件，临时检查单轮锁死方案：

```powershell
.\tools\build_ccs.ps1 `
  -BuildDir Debug\library-check-legacy `
  -Defines 'CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD=CAR_LIBRARY_RIGHT_ANGLE_TURN_LOCKED_INNER' `
  -Clean
```

多个临时选择可传数组：

```powershell
.\tools\build_ccs.ps1 `
  -BuildDir Debug\library-check-no-gimbal `
  -Defines @(
    'CAR_LIBRARY_GIMBAL_TRACKING_METHOD=CAR_LIBRARY_GIMBAL_TRACKING_NONE',
    'CAR_LIBRARY_GIMBAL_ATTITUDE_METHOD=CAR_LIBRARY_GIMBAL_ATTITUDE_NONE'
  ) `
  -Clean
```

构建只验证编译和链接，不会自动烧录。更换方法后仍要按架空轮、低限幅、单任务、
完整赛道的顺序做硬件验证。
