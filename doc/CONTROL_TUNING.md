# Task5 在线标定与 Task8 云台姿态环

Task5 用于左右编码电机的开环测量和速度 PI 在线测试。测试前必须架空车轮，并准备能立即断开电机动力电源的物理手段。串口命令会一直保持到下一条命令，USB 断开不会自动停车。

## 串口和任务入口

- UART0：115200，8-N-1，无流控。
- MCU TX/RX：PA10/PA11。
- OLED 菜单选择 `Task 5 PID`，按 K2 进入。
- 进入 Task5 后会停止所有电机并关闭视觉。100 Hz 姿态任务保持运行，默认由底盘调参模式占用执行机构。
- 长按 K2 退出时，左右 PWM 强制清零、积分清零，并恢复正常任务模式。

Task5 默认处于 `mode chassis`。只有显式发送 `mode gimbal` 后才停止底盘调参、
启动 JY61 静止校准和云台 yaw 姿态保持；切回 `mode chassis` 会先停止云台。
`stop` 同时停止底盘和云台输出。Task8 是独立的云台姿态实验入口，不启动视觉、
不启动底盘，也没有接入 Task4。

## 定时器与调度

| 资源 | 当前用途 | 中断负载 |
| --- | --- | --- |
| `TIMA0` | 左右底盘 20 kHz 硬件 PWM，CCP1/CCP3 | 不开周期中断 |
| `TIMG6` | 两路云台 STEP，20 kHz 固定节拍 | 仅任一云台轴命令非零时开启 |
| `TIMG0` | 数字灰度 100 us 快采样 | 仅 Task1/Task4 正式循迹时开启 |
| `TIMG7/TIMA1/TIMG8/TIMG12` | 空闲 | 无 |

云台/姿态 FreeRTOS 任务使用 `xTaskDelayUntil` 固定每 10 ms 运行，任务优先级为 6；
底盘控制任务同为 6，输入/任务/通信/UI 依次为 4/3/2/1。`TIMG6` STEP 中断优先级
为 0，`TIMG0` 灰度为 1，JY61 UART 为 2。STEP 与灰度不再放在同一个 ISR，
而且两轴 0 SPS 时 `TIMG6` 会停表，不产生 20 kHz 空中断。

## Task8 姿态估计与控制

JY61 驱动同时解析 `0x52` 角速度帧和 `0x53` 欧拉角帧。`body_motion` 是唯一的
姿态计算模块，底盘和云台以后都读取同一份 `BodyMotionSnapshot`，不能各算一份。
角度单位为 `0.01 deg`，角速度单位为 `0.01 deg/s`，算法为：

```text
omegaFiltered += alpha * (omegaRaw - bias - omegaFiltered)
yawPredict = yawEstimate + omegaFiltered * frameDt
yawEstimate = yawPredict + beta * (yawUnwrapped - yawPredict)
yawControl = yawEstimate + omegaFiltered * (frameAge + predictionTime)
```

`frameDt` 来自 JY61 帧的 RTOS 时间戳，不假定串口严格等间隔；yaw 在正负 180 度
处会解包角。启动后先累计 100 个静止角速度样本求零偏，100 Hz 输出时约 1 秒。
角度或角速度任一超过 100 ms 未更新，立即进入 `STALE`、停止 yaw，并在数据恢复后
重新抓取保持基准，避免补偿一段不可见运动。

云台控制使用角速度前馈和 STEP 位置误差比例项：

```text
stepReference = step0 + sign * (yawControl - yaw0) * stepsPerRev / 36000
speedFF = turnGate * sign * yawRateX100 * stepsPerRev / 36000
          * KffQ1024 / 1024
speedCommand = clamp(speedFF + KpQ1024 * stepError / 1024)
```

`turnGate=0` 时前馈被硬置零，角度反馈仍然工作；只有上层确认检测到转弯后才设为
1。这里的“检测到”应当使用 S5/S6/S7 直角窗口确认后形成的转弯状态，不能使用
`grayMask != 0`，因为直线循迹时灰度同样非零。Task8 和每次姿态环启动默认
`turnGate=0`；本版本仍未把姿态环接入 Task4。

当前实际编译值以 `config/control_config.h` 和 Task5 `gshow` 为准；在线修改只保留
在 RAM 中。无论 `Kff` 设为多少，`turnGate=0` 时都不会产生角速度前馈。
`Motor_GetStepCount()` 统计的是已经安排输出的 STEP 脉冲，不是编码器；电机失步无法
被这个位置项发现。因此它能修正指令斜坡造成的相位误差，但不等于机械角度全闭环。

3200 step/rev 时，90 度是 800 step；若底盘匀速 1 秒转完 90 度，理想云台速度是
800 SPS。斜坡值为 `A SPS/ms` 时，从 0 到 800 SPS 需要 `800/A ms`。终点速度
不能直接当位移；若一秒内从静止加速到 `Vmax` 再降到静止，且能到达限速，理论
位移为 `Vmax * (1 - Vmax/(1000*A)) step`。实际还要看底盘角速度曲线、负载和
失步，并通过 Task5 观察 `err/cmd`。

Task5 云台调参命令如下；参数只保存在 RAM，复位后恢复
`config/control_config.h` 默认值：

| 命令 | 作用 | 示例 |
| --- | --- | --- |
| `mode gimbal` | 停底盘并启动静止校准和 HOLD | `mode gimbal` |
| `mode chassis` | 停云台并恢复底盘调参 | `mode chassis` |
| `gcal` | 重新采集 100 个静止零偏样本 | `gcal` |
| `ghold on\|off` | 开启保持或只观察姿态 | `ghold off` |
| `gff on\|off` | 手动开关角速度前馈门，用于 Task5 独立标定 | `gff off` |
| `gsteps N` | 电机每机械圈脉冲数 | `gsteps 3200` |
| `gsign -1\|1` | 补偿方向 | `gsign -1` |
| `gkff Q1024` | 角速度前馈增益 | `gkff 1024` |
| `gkp Q1024` | STEP 位置误差比例增益 | `gkp 256` |
| `glpf Q1024` | 角速度低通 alpha，范围 1..1024 | `glpf 256` |
| `gbeta Q1024` | yaw 角度校正 beta，范围 1..1024 | `gbeta 10` |
| `gpred MS` | 附加预测时间，范围 0..100 ms | `gpred 0` |
| `gmax SPS` | yaw 最大命令，不能超过全局 1000 SPS | `gmax 1000` |
| `gaccel SPS_PER_MS` | STEP 每毫秒斜坡增量 | `gaccel 5` |
| `glimit STEP` | 相对启动位置限制，0 表示关闭 | `glimit 1200` |
| `gshow` | 输出姿态、零偏、误差、命令和全部参数 | `gshow` |

建议先 `mode gimbal` 并保持整车静止，等 OLED 从 `CAL` 进入 `HOLD FF0`；先在
`gff off` 下确认角度反馈方向和 `gkp`，云台若同向运动先改 `gsign`。再用
`gff on` 单独测试匀速转动并调整 `gkff`，测试结束恢复 `gff off`。以后接入 Task4
时由确认后的转弯状态自动开门。最后才增加 `gpred` 或加快 `gaccel`；全过程必须
保留机械行程余量及物理断电手段。

底盘控制任务始终每 20 ms 读取并清零一次左右编码器窗口计数。串口默认每 500 ms 输出一行状态；执行命令或修改参数时立即回显并刷新。每次修改 `pwm` 或 `target` 后会丢弃第一个混合窗口，再从新的完整 20 ms 窗口累计平均值。

Task1 的 `CAR_MOTOR_NO_YAW_*_COUNTS_PER_PERIOD` 参数与串口
`target/move` 完全同单位且保留正负号：配置值 `-5` 就是目标 `-5 count/20ms`，不会再先当
CPS 后发生二次换算。Task4 为兼容现有配置仍写 CPS，但构建时要求能够无损
换算成整数 count/20ms。

## 命令

串口 PWM 参数使用百分比，范围 `-100..100%` 映射到内部 `-1200..1200 PWM count`：

```text
PWM_raw = PWM_percent * 1200 / 100
```

TIMA0 周期为 1600，软件限幅暂为 1200，因此当前 `30%=360 count`。日志同时输出百分比和 `raw`；FF 计算始终使用换算后的实际 PWM count。

| 命令 | 作用 | 示例 |
| --- | --- | --- |
| `mode chassis\|gimbal` | 在底盘调参和云台调参之间安全切换 | `mode chassis` |
| `set L R` | 输出 PWM 百分比，稳定 4 秒后采集 50 个窗口；第二个点自动计算并应用 FF 和 runstart | `set 30 0` |
| `set clear` | 清除左右已保存的第一个采样点并停车 | `set clear` |
| `pwm L R` | 左右轮开环 PWM 百分比，范围 -100..100 | `pwm 15 0` |
| `target L R` | 闭环目标，范围 -100..100 count/20 ms | `target 12 12` |
| `move LS RS LD RD` | 以左右速度 LS/RS 行驶有符号编码距离 LD/RD；速度范围 1..100 count/20 ms | `move 50 50 1000 1000` |
| `gray on\|off` | 开关 Task1 灰度联调；OLED 显示 S1～S7、左右目标和反馈 | `gray on` |
| `start L R` | 实际速度绝对值小于15 count/20ms时使用的 PWM 百分比，范围 0..100 | `start 25 22` |
| `runstart L R` | 左右轮转动后的运行摩擦 PWM 百分比，范围 0..100 | `runstart 12 11` |
| `ff L R` | 左右 FF_Q1024 | `ff 28000 29500` |
| `kp L R` | 左右 KP_Q1024 | `kp 256 256` |
| `ki L R` | 左右 KI_Q1024 | `ki 8 8` |
| `ilim L R` | 左右积分输出限幅百分比，范围 0..100 | `ilim 20 20` |
| `ffcalc P1 C1 P2 C2` | 用两个 PWM 百分比/count 点计算 FF_Q1024 和 runstart | `ffcalc 30 8 50 15` |
| `oled ff\|start\|speed\|pid\|gray\|gimbal` | 运行时切换 Task5 OLED 测量页面 | `oled start` |
| `avg` | 清空平均值，下一完整窗口重新累计 | `avg` |
| `clear` | 清空左右积分 | `clear` |
| `stop` | 立即停止底盘和云台输出 | `stop` |
| `show` | 显示模式、参数、反馈、平均值和串口错误 | `show` |
| `help` | 显示命令摘要 | `help` |

`move` 是非阻塞距离测试命令。LS/RS 是左右轮速度幅值，LD/RD 是有符号 encoder
count，正数前进、负数后退。执行后 OLED 自动切到 PID 页面，底盘控制任务仍按
20 ms 周期运行，Task5 通信任务每 5 ms 检查左右累计编码器 count。某一轮先达到
目标距离时会先停该轮，另一轮继续运行；`stop` 会立即取消距离命令并停车。

周期状态行示例：

```text
D mode=open set=idle gray=0 gray_mask=0 startup=0,0 pwm_pct=30,0 pwm_raw=360,0 target=0,0 count=8,0 avg=8,0 sum=198,0 pwm_a_cc=1240 irq_pa13=0 irq_pb24=0 level_pa13=1 level_pb24=1 n=24
```

`count` 是最近一个 20 ms 窗口；`avg` 是本次换点后有效窗口的整数平均值；`sum/n` 是未取整的精确平均值；`n` 是已计入平均的窗口数。`pwm_a_cc` 是 TIMA0 CCP1 当前硬件比较寄存器值，向下计数 PWM 下应满足 `pwm_a_cc = 1600 - abs(pwm_raw_left)`；它不是 PA22 电平的串口采样波形。

Task5 的 `D` 状态行还会输出 `irq_pa13`、`irq_pb24`、`level_pa13` 和
`level_pb24`。`startup=1` 表示对应轮有非零目标且实际速度绝对值小于15，正在使用
静摩擦 `start`；`0` 表示使用 `runstart` 或当前没有闭环目标。前两项中断计数是从上电初始化起累计服务的右编码器 A/B GPIO 中断
次数，后两项是输出该行时直接读取的 MCU 管脚电平。若两项中断都增长但
`A_R` 仍接近 0，应检查两相是否具有正确的 90 度正交关系。诊断只追加到
非 `P` 状态行；所有 `P` 行仍保持固定九通道。

同一刷新周期还会输出一条供 SerialPlot 使用的纯数字帧：

```text
P left_target,left_feedback,left_pwm_pct,right_target,right_feedback,right_pwm_pct,pwm_a_cc,left_ff_q1024,right_ff_q1024
```

例如：

```text
P 0,8,30,0,0,0,1240,35109,0
```

第 3、6 通道是 `-100..100%` 的直观 PWM 百分比；内部 raw count 仍在
`D` 状态行的 `pwm_raw` 字段中，FF 拟合也始终使用 raw count。第 8、9
通道是当前 RAM 中的左右 `FF_Q1024`，计算成功后会每 500 ms 持续输出。

SerialPlot 选择 `ASCII`，通道数设为 `9`，列分隔符选择 `comma`，`Filter by Prefix` 选择 `Include` 并填写 `P`，`Hex data` 不勾选。软件会先过滤其他日志，再去掉 `P` 前缀并解析九个通道。

## 自动采样 set

`set` 参数是 PWM 百分比。固件先换算成实际 PWM count，再开始异步采样，不会阻塞控制任务。执行顺序为：

1. 把百分比换算成 raw count，立即输出指定左右 PWM。
2. 等待 4000 ms，让机械速度充分稳定。
3. 丢弃切换产生的混合窗口。
4. 采集 50 个完整的 20 ms 编码器窗口，共 1000 ms，并用这 50 个窗口求平均值。
5. 打印该点的 `pwm_pct/pwm_raw/sum/n/avg_x1000`，然后自动停车。

Task5 OLED 页面可以在运行时切换，不需要重新烧录：

- `oled ff`：显示最终左右 FF 和已保存的 runstart 百分比。FF 使用
  `FF_Q1024 / 1024` 的三位小数，例如内部值 `35109` 显示为 `34.286`。
- `oled start`：显示最近 20 ms 左右编码器 count，以及保存的 start/runstart 百分比。
- `oled speed`：显示当前 PWM、平均count/20ms及换算后的count/s。
- `oled pid`：显示左右目标count/20ms、反馈count/20ms和输出PWM百分比。
- `gray on`：自动切到 `oled gray`，每20ms直接读取 S1～S7，并使用
  Task1 的基础速度、差速增益和上下限更新左右闭环目标。`T` 是左右目标
  count/20ms，`F` 是左右实际反馈；没有任何灰度输入时目标立即置零。
  当灰度给出的左右原始目标完全相同时，正常闭环会用本周期左右反馈的平均值
  做直线同步修正；OLED/SerialPlot 的 `T` 显示修正后的有效目标。有灰度差速时
  不做该修正，Task5 的 `target/move` 独立标定模式也不受影响。
  使用 `gray off` 或 `stop` 停车并退出灰度联调。

测 start 时先发送 `oled start`，每次都先用 `pwm 0 0` 让车轮完全停止，
再用 `pwm 5 0`、`pwm 7 0` 等逐级测试左轮；右轮使用 `pwm 0 5`、
`pwm 0 7`。取重复测试都能可靠启动的最小百分比，最后用一次
`start LEFT RIGHT` 保存，OLED 的 `Start` 行会显示最终值。

runstart 和 FF 必须使用两个已经稳定转动的点，不能拿刚好能起步的临界点拟合。
每个车轮独立保存自己的第一个点，第二次有效采样后按精确的 `sum/n` 计算
`FF_Q1024` 和稳态直线截距 `runstart`，并立即写入该轮当前 RAM 参数。
某侧 PWM 写 0 表示本次跳过该轮。

计算完成时会额外输出一次固定格式的拟合记录，不会随 500 ms 状态重复打印：

```text
FIT,left,30,360,200,25,8000,50,600,375,25,15000,35109,86
```

字段顺序为：

```text
FIT,motor,
p1_pct,p1_raw,c1_sum,c1_n,c1_avg_x1000,
p2_pct,p2_raw,c2_sum,c2_n,c2_avg_x1000,
ff_q1024,runstart_raw
```

为兼容 SerialPlot 的 `Include P` 和固定九通道设置，自动拟合完成后还会
输出一条带 `P` 前缀的九数值结果行：

```text
P FIT:-32768,motor:1,pwm1_raw:360,count1_x1000:8000,pwm2_raw:600,count2_x1000:15000,ff_q1024:35109,left_ff:35109,right_ff:0
```

其中 `FIT=-32768` 是结果行标记，`motor=1/2` 表示左/右轮；结果行第 7
个数是本次计算值，第 8、9 个数是当前左右 FF。所有 `P` 行都保持九个
通道，因此不会触发 `invalid number of channels`。

手动执行 `ffcalc` 时也会输出一条：

```text
FITCALC,pct1,raw1,count1,pct2,raw2,count2,ff_q1024,runstart_raw
```

随后也会输出同格式的 `P FIT:` 九通道结果，其中 `motor=0` 表示手动
`ffcalc`，字段名为 `count1/count2`，数值就是命令中输入的原始计数。

推荐左右轮分开标定：

```text
set clear
set 30 0
# 等待 #SET left point1 stored 和 #SET done
set 50 0
# 返回 #SET left ff_q1024=... runstart_raw=... applied

set 0 30
# 等待 #SET right point1 stored 和 #SET done
set 0 50
# 返回 #SET right ff_q1024=... runstart_raw=... applied
```

也可以同时采样，例如 `set 30 30`、`set 50 50`，左右 FF/runstart 仍分别计算。若返回 `count sign mismatch`，表示 PWM 太小、电机未稳定转动，或者编码器方向配置错误。`stop` 会取消正在进行的采样并停车，但保留已经完成的第一个点；`set clear` 才会清除点位。

## 1. 确认方向

左右轮必须分开测。先从很小的正 PWM 开始：

```text
pwm 10 0
pwm 0 10
```

正 PWM 应让对应车轮向前，正向转动时 `count` 也应为正。电机方向由 `CAR_CHASSIS_LEFT_REVERSE/CAR_CHASSIS_RIGHT_REVERSE` 修正，编码器方向由 `CHASSIS_LEFT_ENCODER_SIGN/CHASSIS_RIGHT_ENCODER_SIGN` 修正，不能通过使用负的 FF/KP/KI 修方向。

## 2. 分开测 start、runstart 和 FF

`start` 是静止起步阈值。对单个车轮从 PWM=0% 缓慢增加，每次测试前先用
`pwm 0 0` 等车轮完全停止，记录重复测试都能可靠启动的最小百分比。`show`
会同时给出 `start_pct/start_raw`；最终写回配置的是 raw count。

`runstart` 是轮子已经转动后的稳态摩擦截距。选至少两个远离启动临界点的
稳定 PWM，可以用 `set` 自动采样和计算，也可以用 `pwm` 手动输出，每个点
等待 `n` 累计若干个窗口后记录 `pwm_raw` 和 `sum/n`：

```text
FF_Q1024 = (PWM_count2 - PWM_count1)
           / (count2 - count1) * 1024
RUN_START_raw = abs(PWM_count1 - FF_Q1024 * count1 / 1024)
```

也可直接发送：

```text
ffcalc PWM_percent1 count1 PWM_percent2 count2
```

左右轮分别测量。自动 `set` 会把 FF 和 runstart 应用到 RAM；`ffcalc` 只打印
结果，不自动指定给某个轮子。二者都会先把百分比换成实际 count，不能直接把
百分比数字代入公式。Trace-f407 的 `百分比 × 42` 只适用于 PWM_MAX=4200；
本工程当前为 `百分比 × 12`。

## 3. 闭环测试

先写入左右实测截距和斜率，保持 Kp/Ki 为 0：

```text
start LEFT_START RIGHT_START
runstart LEFT_RUN_START RIGHT_RUN_START
ff LEFT_FF RIGHT_FF
kp 0 0
ki 0 0
ilim 0 0
target LEFT_COUNT RIGHT_COUNT
```

控制器定义为：

```text
error = targetCount - feedbackCount
integralScaled += KI_Q1024 * error

startupActive = targetCount != 0 && abs(feedbackCount) < 15
selectedStart = startupActive ? startPwm : runStartPwm
PWM = sign(targetCount) * selectedStart
    + FF_Q1024 * targetCount / 1024
    + KP_Q1024 * error / 1024
    + integralScaled / 1024
```

每个20 ms控制周期都按实际编码器速度绝对值重新选择：小于15时使用
`startPwm`，等于或大于15时使用 `runStartPwm`。正反转采用同一阈值；目标为0时
PWM仍直接清零，不会因为低速而输出 `startPwm`。

这里各项使用加号。因为 `error=target-feedback`，正的 FF/KP/KI 必须对正目标产生正向输出；若写成减号，正误差会把 PWM 往反方向推。正反方向统一由电机和编码器方向配置处理。

前馈稳定后，保持 Ki=0 逐步增加 Kp；响应足够快且不持续振荡后，再给 Ki 和 `ilim` 一个较小值以消除稳态误差。每次修改在线参数都会自动清空已有积分。

## 4. 写回编译参数

在线参数只保存在 RAM，复位后恢复为 `config/control_config.h`：

```text
CHASSIS_LEFT_START_PWM_COUNTS
CHASSIS_LEFT_RUN_START_PWM_COUNTS
CHASSIS_LEFT_FF_Q1024 / CHASSIS_LEFT_KP_Q1024 / CHASSIS_LEFT_KI_Q1024
CHASSIS_LEFT_INTEGRAL_LIMIT_PWM_COUNTS
CHASSIS_RIGHT_START_PWM_COUNTS
CHASSIS_RIGHT_RUN_START_PWM_COUNTS
CHASSIS_RIGHT_FF_Q1024 / CHASSIS_RIGHT_KP_Q1024 / CHASSIS_RIGHT_KI_Q1024
CHASSIS_RIGHT_INTEGRAL_LIMIT_PWM_COUNTS
```

写回并重新构建后，先架空复测 Task5，再测试 Task1/Task4。
