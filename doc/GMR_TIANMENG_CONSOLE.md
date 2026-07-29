# 天猛星 Gmr 赛题控制台

当前Gmr是单一发布目标，不再承载旧地猛星、双车蓝牙、串口PID或循迹测试菜单。
旧实现保留在Git历史提交`e6e0d09`和对应源文件中，不进入当前Gmr链接结果。

## 当前外设

| 外设 | 资源 | 当前用途 |
| --- | --- | --- |
| 左右编码电机 | TIMA0 PWM + 4路编码GPIO | 驱动已初始化；姿态任务中强制停车 |
| 七路数字灰度 | PA15/16/17/24/25/26/27 | GPIO驱动已初始化；尚无正式赛题算法 |
| H7通信 | UART2，PB15 TX / PB16 RX | LCD文本输出、任务启动和停止 |
| 外部M0姿态 | UART3，PB2 TX / PB3 RX | 100 Hz连续yaw和yaw角速度 |
| 本地OLED | I2C0，PA0 SDA / PA1 SCL | 显示姿态数据和链路统计 |

UART0、UART1、蓝牙、云台STEP和IMU660RX在当前Gmr默认构建中不初始化。

## 唯一任务

`Attitude`任务是被动监视页，显示：

- 连续yaw，单位0.01度转换为两位小数；
- yaw角速度，单位0.01度每秒转换为两位小数；
- M0发送序号、有效帧数和坏帧数。

任务进入和退出都会调用`EncoderMotor_Stop()`，不会根据姿态数据驱动电机。

## H7协议

链路固定为115200、8-N-1，MCU TX接H7 RX，MCU RX接H7 TX，双方共地。

M0到H7沿用文本LCD协议：

```text
@CLEAR\n
@L0=GMR Tianmeng\n
@L1=> Attitude\n
```

H7到M0当前支持：

```text
@START=1\n
@STOP\n
```

解析在UART2 ISR中只生成命令，状态事件由5 ms Comm任务投递给Mission任务。

## 构建

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Clean
```

默认等价于天猛星、蓝牙关闭、JY61关闭、UART0日志关闭。构建不会自动烧录。
