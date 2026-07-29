# IMU660RA/RB/RC SPI六轴模块

## 支持范围

`hardware/imu660rx.c`为天猛星64P开发板提供统一的IMU660RX轮询驱动：

| 模块 | 内部芯片 | 芯片ID | 支持内容 |
| --- | --- | ---: | --- |
| IMU660RA | BMI270 | `0x24` | 初始化配置、三轴加速度、三轴角速度 |
| IMU660RB | LSM6DSR | `0x6B` | 初始化配置、三轴加速度、三轴角速度 |
| IMU660RC | LSM6DSV16X | `0x70` | 初始化配置、三轴加速度、三轴角速度 |

三个型号对外使用同一个`IMU660RX_Init()`和`IMU660RX_Read()`接口。当前驱动使用
SPI Mode 0、8 MHz、加速度量程正负8 g、角速度量程正负2000 dps。RC的芯片内
四元数输出和INT中断模式尚未启用，当前只读取六轴原始寄存器。

资料来源：[逐飞科技IMU660RX Product](https://gitee.com/seekfree/IMU660RX_Product)，
核对版本为提交`d73c66c9748db4daea36c2234f9b8b792088c58c`。RA初始化所需的
8192字节BMI270配置流保存在`hardware/imu660ra_config_file.inc`；上游仓库使用
GPL-3.0，发布含该数据的固件或源码前需检查项目整体许可证兼容性。

## 天猛星接线

| IMU660RX信号 | MSPM0G3507 | 天猛星排针 | 说明 |
| --- | --- | --- | --- |
| `SCK/SPC` | PB9 | U21-28 | SPI1时钟 |
| `MOSI/SDI` | PB8 | U21-27 | MCU输出、传感器输入，TI名称PICO |
| `MISO/SDO` | PB7 | U21-26 | 传感器输出、MCU输入，TI名称POCI |
| `CS` | PB14 | U22-34 | 低电平片选 |
| `INT1` | PB17 | U22-15 | 当前轮询驱动不使用，可不接 |
| `INT2` | PB12 | U21-16 | 当前轮询驱动不使用，可不接 |
| `VCC` | EXT_3V3 | U21-22或U21-38 | 推荐使用3.3 V |
| `GND` | GND | 任意GND针 | 必须与主控共地 |

PB7、PB8、PB9与板载U24 SPI Flash共用SPI1。驱动会把Flash CS PB6保持为高电平，
再访问IMU。PB14同时是H8接口的LCD CS，因此外接IMU660RX时不能再使用H8 SPI
LCD；本工程默认的PA0/PA1 I2C OLED不受影响。

## 选择型号

两个Profile默认都选择`CAR_LIBRARY_IMU660RX_NONE`，未接模块时不会初始化SPI1，
也不会增加RA配置流的Flash占用。临时构建自动识别版本：

```powershell
.\tools\build_ccs.ps1 `
  -Profile Gmr `
  -BuildDir Debug\profile-gmr-imu-auto `
  -Defines 'CAR_LIBRARY_IMU660RX_METHOD=CAR_LIBRARY_IMU660RX_AUTO' `
  -Clean
```

正式固件建议明确选择实际型号，以便RB/RC构建不携带RA的8192字节配置：

```powershell
-Defines 'CAR_LIBRARY_IMU660RX_METHOD=CAR_LIBRARY_IMU660RX_RA'
-Defines 'CAR_LIBRARY_IMU660RX_METHOD=CAR_LIBRARY_IMU660RX_RB'
-Defines 'CAR_LIBRARY_IMU660RX_METHOD=CAR_LIBRARY_IMU660RX_RC'
```

当前TI Arm Clang 4.0.4 `-O2`实测，Full+AUTO的Flash text为127664字节，
128 KiB中只剩3408字节；Full正式固件不要使用AUTO，必须明确选择实际型号。
GMR+AUTO的Flash text为97624字节，仍有33448字节余量。

也可以在当前产品的`config/profiles/profile_gmr.h`或`profile_full.h`中修改
`CAR_LIBRARY_IMU660RX_METHOD`默认值。该功能只允许与
`CAR_LIBRARY_BOARD_TIANMENG_64P`组合，地猛星板型会在编译期报错。

## 读取数据

板级启动会自动调用`IMU660RX_Init()`。初始化失败会设置非致命错误位
`BOARD_ERROR_IMU660RX`，并在UART日志打印状态码；其它车辆功能仍继续启动。

任务中读取一帧：

```c
#include "imu660rx.h"

IMU660RXSample sample;

if (IMU660RX_Read(&sample) == IMU660RX_STATUS_OK) {
    /* sample.accelMgX/Y/Z: mg */
    /* sample.gyroMdpsX/Y/Z: mdps */
}
```

`IMU660RX_Read()`会在一次片选内连续读取六轴数据，并返回原始`int16_t`和换算后的
整数工程单位。函数只能在任务上下文调用；并发调用返回`IMU660RX_STATUS_BUSY`，
不能从中断服务函数调用。

## 首次上板检查

1. 先断开电机动力和H8 SPI LCD，只连接IMU与3.3 V。
2. 明确选择RA/RB/RC型号构建并下载，观察UART日志中`imu660rx status=0`。
3. 静止平放时确认某一加速度轴约为正负1000 mg，三轴角速度接近0 mdps。
4. 分别绕X、Y、Z轴转动，确认对应角速度方向和安装方向，再做零偏标定。
5. 只有轮询读取验证通过后，才把数据接入姿态融合或车辆闭环。
