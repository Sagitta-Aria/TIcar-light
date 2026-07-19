# H7 BMI088 姿态链路

`D:\stm32H7\projects\CtrBoard-H7_Gimbal` 把达妙 H7 板作为独立的滤波陀螺仪模块。
H7 以 BMI088 数据就绪中断驱动 2 kHz 采样，完成静止零偏、三轴 Kalman 角速度
滤波和 Mahony 姿态更新，再通过 UART7 的 PE8 TTL TX 以 100 Hz 输出。

## 接线

达妙板 P13 引出了 UART7 的普通 3.3 V TTL 信号，可直接连接地猛星，不需要
RS485 接收器。按信号名接线，不依赖插座方向：

| 达妙 H7 P13 信号 | 地猛星 MSPM0G3507 | 说明 |
| --- | --- | --- |
| `UART7_TX / PE8` | `PA11 / UART0_RX` | H7 姿态数据 |
| GND | GND | 必须共地 |
| `UART7_RX / PE7` | 不接 | 当前为单向链路 |
| 5 V | 不接 | 两板各自供电时禁止相连 |

PA10/UART0 TX 也不接 H7。达妙板的芯片外设 `USART2/USART3` 对外已经经过 RS485
收发器，没有 485 接收器时不能直接接 M0；这里使用的是板上第二路普通串口接口，
其 MCU 外设名为 `UART7`。

## 串口与帧格式

- 115200 bit/s，8-N-1，无流控。
- 每 10 ms 连续发送一个 `0x52` 角速度帧和一个 `0x53` 欧拉角帧。
- 每帧11字节，线格式兼容JY61P；M0由独立`h7_gyro_link`在UART0/PA11解析。

| 字节 | 内容 |
| ---: | --- |
| 0 | 帧头 `0x55` |
| 1 | `0x52` 角速度，或 `0x53` 欧拉角 |
| 2..3 | X轴，有符号小端 `int16_t` |
| 4..5 | Y轴，有符号小端 `int16_t` |
| 6..7 | Z/yaw轴，有符号小端 `int16_t` |
| 8..9 | 保留，当前为0 |
| 10 | 字节0到9的8位累加和 |

角速度换算为 `dps = raw * 2000 / 32768`，角度换算为
`deg = raw * 180 / 32768`。M0 解析后统一使用 `0.01 deg` 和
`0.01 deg/s` 定点单位。H7 的轴向符号在
`App/inc/gimbal_project_config.h` 的 `GIMBAL_GYRO_LINK_*_SIGN` 设置。

## 启动与失效保护

1. H7 上电后必须静止约1秒；任一轴超过5 dps会重新开始2000点零偏采样。
2. H7 校准完成前不发送姿态帧。
3. M0不再对H7姿态做二次校零、低通或积分，收到首对有效帧后直接锁存当前yaw目标。
4. H7任一角度或角速度帧超过100 ms未更新，M0立即停止yaw输出；恢复后重新锁存目标。
5. 板载JY61仍在UART1/PB7独立运行，只提供底座角速度前馈；JY61掉线时关闭前馈，H7反馈环继续工作。

H7是反馈源，板载JY61是前馈源，两者不能共用解析缓存或串口中断。
`BODY_MOTION_GYRO_ALPHA_Q1024`、`BODY_MOTION_YAW_BETA_Q1024`只作用于JY61前馈估计，
不作用于H7反馈。

H7板按云台照片竖直安装，BMI088的 `+Y` 轴朝上。H7在滤波和Mahony计算前将
传感器坐标转换为车体坐标 `[X, Y, Z] = [sensor X, -sensor Z, sensor Y]`，因此
M0解析到的Z轴角速度和yaw角对应云台绕竖直轴旋转。

## Task8控制关系

H7反馈角度外环生成目标yaw角速度，H7反馈角速度生成阻尼修正，板载JY61角速度
只作为底座运动前馈：

```text
rateRef = clamp(KpAngle * (h7YawTarget - h7Yaw))
rateCorrection = clamp(KpRate * (rateRef - h7YawRate))
motorRate = rateRef + rateCorrection - Kff * jy61BaseYawRate
motorSps = clamp(motorRate * stepsPerRev / 360 * motorSign)
```

H7角度误差先经过连续死区：`±0.30 deg`内按0处理，超过死区后减去死区宽度再
进入角度Kp。这样零点附近不会反复换向，跨过死区边界时速度命令也不会突跳。

H7掉线会停机；JY61掉线只令最后一项为0。

## 地猛星UART0注意事项

PA10/PA11的单独引脚与板载CH340仍属于同一网络。接入H7前必须断开CH340的TX
到PA11，使用期间不要再通过Type-C发送Task5命令。固件保留PA10日志TX，但关闭
UART0文本RX，UART0中断只服务H7姿态解析。需要恢复Task5在线命令时，必须先断开
H7并把`CAR_ENABLE_LOG_UART_RX`和UART0中断归属恢复为日志控制台。
