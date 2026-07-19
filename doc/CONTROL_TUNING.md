# Task5 在线标定与 Task8 云台姿态环

Task5 用于左右编码电机的开环测量和速度 PI 在线测试。测试前必须架空车轮，并准备能立即断开电机动力电源的物理手段。串口命令会一直保持到下一条命令，USB 断开不会自动停车。
视觉模式会实际驱动 yaw/pitch 两轴，测试前还必须确认机械行程内没有线缆或限位干涉。

## 串口和任务入口

- UART0：115200，8-N-1；当前PA11由H7姿态链路独占。
- MCU PA10只保留输出，UART0文本RX关闭，因此Type-C在线命令暂不可用。
- OLED 菜单选择 `Task 5 PID`，按 K2 进入。
- 进入 Task5 后会停止所有电机并关闭视觉。100 Hz 姿态任务保持运行，默认由底盘调参模式占用执行机构。
- 长按 K2 退出时，左右 PWM 强制清零、积分清零，并恢复正常任务模式。

Task5默认处于`mode chassis`。只有显式发送`mode gimbal`后才停止底盘调参、
启动H7反馈的云台yaw保持；板载JY61前馈默认关闭，可用`gff on`单独启用。
发送 `mode vision` 会停止底盘和姿态环，启动 UART3 视觉解析与二维云台闭环，
并默认输出 `B` 前缀视觉响应波形。`stop` 同时停止底盘、姿态和视觉云台输出。
Task8 是独立的云台姿态实验入口，不启动视觉或底盘。

## 定时器与调度

| 资源 | 当前用途 | 中断负载 |
| --- | --- | --- |
| `TIMA0` | 左右底盘 20 kHz 硬件 PWM，CCP1/CCP3 | 不开周期中断 |
| `TIMG6` | 两路云台 STEP，20 kHz 固定节拍 | 仅任一云台轴命令非零时开启 |
| `TIMG0` | 数字灰度 100 us 快采样 | 仅 Task1/Task4 正式循迹时开启 |
| `TIMG7/TIMA1/TIMG8/TIMG12` | 空闲 | 无 |

云台/姿态 FreeRTOS 任务使用 `xTaskDelayUntil` 固定每 10 ms 运行，任务优先级为 6；
底盘控制任务同为 6，输入/任务/通信/UI 依次为 4/3/2/1。`TIMG6` STEP 中断优先级
为0，`TIMG0`灰度为1，H7 UART0和JY61 UART1均为2。STEP与灰度不再放在同一个ISR，
而且两轴 0 SPS 时 `TIMG6` 会停表，不产生 20 kHz 空中断。

## Task4/Task8双传感器控制

两路传感器必须分开：H7通过UART0/PA11提供云台yaw角度和yaw角速度反馈；板载
JY61通过UART1/PB7提供底座yaw角速度前馈。两路都解析`0x52`和`0x53`帧，但使用
独立缓存、帧计数、时间戳和中断。H7已经完成零偏、滤波和姿态解算，M0只做单位
换算、yaw跨±180度展开和100 ms超时判断，不再次融合。

Task8在首个有效H7反馈到达时锁存当前yaw为目标，10 ms控制公式为：

```text
rateRef = clamp(gkp / 1024 * (h7YawTarget - h7Yaw))
rateCorrection = clamp(grkp / 1024 * (rateRef - h7YawRate))
baseFeedForward = gkff / 1024 * jy61BaseYawRate
motorRate = rateRef + rateCorrection - baseFeedForward
speedCommand = clamp(motorRate * gsteps / 360 * gsign)
```

H7的角度或角速度任一超过100 ms未更新，立即停止yaw并清除目标；数据恢复后重新
锁存，不追赶掉线期间的不可见运动。JY61未校准或掉线时只令`baseFeedForward=0`，
H7反馈环仍继续工作。Task5进入`mode gimbal`时默认`gff off`；Task8入口默认开启
JY61前馈。

Task4调用`GimbalAttitude_StartAssist()`，姿态环不直接写电机，而是把矫正SPS交给
视觉云台统一合成。收到首帧进入循迹后，H7反馈在整个LINE阶段保持开启；视觉误差
超出yaw死区，或Task4正在执行主动yaw基础/强转命令时，参考角跟随当前H7角度，
避免姿态环抵消主动转动。主动yaw停止后重新锁定当前H7角度，并恢复反馈矫正。
JY61前馈只在灰度确认后的`TURN_APPROACH/TURN_LEFT/TURN_RIGHT/TURN_EXIT`阶段开门。
Task4中H7超时会清除姿态矫正，视觉闭环仍可继续；Task8中H7超时则直接停止yaw。

Task4已经锁定过目标后，视觉连续60 ms没有新帧会进入丢失重搜：以丢失瞬间的yaw
STEP位置为中心，用400 SPS在正负400 STEP之间往返。搜索期间yaw只执行摆动命令，
暂停固定随动速度、H7矫正和JY61前馈；收到下一帧后在同一个10 ms控制周期退出搜索
并恢复视觉追踪。速度和幅度分别由`CAR_MISSION4_GIMBAL_LOST_SEARCH_SPEED_SPS`、
`CAR_MISSION4_GIMBAL_LOST_SEARCH_AMPLITUDE_STEPS`配置。

`Motor_GetStepCount()`只用于相对启动位置的行程保护，不参与H7姿态反馈计算；
步进电机失步会被H7角度环观察到，但如果已经顶到机械限位，必须依靠断电保护。

Task5 云台调参命令如下；参数只保存在 RAM，复位后恢复
`config/control_config.h` 默认值：

| 命令 | 作用 | 示例 |
| --- | --- | --- |
| `mode gimbal` | 停底盘并启动H7反馈HOLD，JY61前馈默认关闭 | `mode gimbal` |
| `mode chassis` | 停云台并恢复底盘调参 | `mode chassis` |
| `mode vision` | 停底盘和姿态环，启动二维视觉云台及 B 波形 | `mode vision` |
| `gcal` | 重新采集板载JY61的100个静止零偏样本 | `gcal` |
| `ghold on\|off` | 开启保持或只观察姿态 | `ghold off` |
| `gff on\|off` | 手动开关板载JY61底座角速度前馈 | `gff off` |
| `gplot on\|off` | 开关 20 ms 云台九通道输出，进入云台模式默认开启 | `gplot on` |
| `gsteps N` | 电机每机械圈脉冲数 | `gsteps 3200` |
| `gsign -1\|1` | yaw电机方向 | `gsign -1` |
| `gh7sign -1\|1` | H7反馈坐标方向 | `gh7sign -1` |
| `gjysign -1\|1` | JY61前馈坐标方向 | `gjysign 1` |
| `gkff Q1024` | JY61底座角速度前馈增益 | `gkff 1024` |
| `gkp Q1024` | H7 yaw角度外环增益，8192表示8.0 | `gkp 8192` |
| `grkp Q1024` | H7 yaw角速度反馈增益；0表示关闭，307约为0.3 | `grkp 0` |
| `glpf Q1024` | 仅JY61前馈角速度低通alpha | `glpf 512` |
| `gbeta Q1024` | 仅JY61姿态角校正beta | `gbeta 256` |
| `gpred MS` | 仅JY61前馈估计附加预测时间 | `gpred 10` |
| `gmax SPS` | yaw 最大命令，不能超过全局 1000 SPS | `gmax 1000` |
| `gaccel SPS_PER_MS` | STEP 每毫秒斜坡增量 | `gaccel 100` |
| `glimit STEP` | 相对启动位置限制，0 表示关闭 | `glimit 6400` |
| `gshow` | 输出姿态、零偏、误差、命令和全部参数 | `gshow` |
| `vconfig DIST SOURCE` | 切换 Near/Mid/Far 与 Center/Circle 参数并重启追踪 | `vconfig mid center` |
| `vplot on\|off` | 开关 20 ms 视觉十通道 B 波形 | `vplot on` |
| `vshow` | 输出视觉帧、控制误差、两轴命令及当前参数 | `vshow` |

建议先`mode gimbal`并保持`gff off`，确认OLED显示`HOLD FF0`。轻推云台，H7
角度误差应驱动电机回到原方向；若发散立即断电，优先核对`gsign`和`gh7sign`，再从
较小`gkp/grkp`逐步增加。反馈稳定后保持底座静止执行`gcal`，再`gff on`并只转动
底座，确认JY61前馈方向；方向相反时改`gjysign`。最后才调`gkff`和`gaccel`。

Task5 云台模式每 20 ms 输出一行 `A` 前缀纯数字帧。SerialPlot 使用 ASCII、
9 通道、逗号分隔，`Filter by Prefix` 选择 `Include` 并填写 `A`：

```text
A h7_yaw_x100,target_yaw_x100,angle_error_x100,h7_rate_x100_s,
  jy61_rate_x100_s,rate_ref_x100_s,ff_sps,cmd_sps,step_sps
```

各通道含义：

| 通道 | 含义 |
| ---: | --- |
| 1 | H7连续yaw反馈，单位0.01 deg |
| 2 | 锁存的H7 yaw目标，单位0.01 deg |
| 3 | `target-h7Yaw`角度误差，单位0.01 deg |
| 4 | H7 yaw角速度反馈，单位0.01 deg/s |
| 5 | JY61滤波后的底座yaw角速度，单位0.01 deg/s |
| 6 | H7角度外环生成的目标角速度，单位0.01 deg/s |
| 7 | JY61前馈对电机命令的有符号SPS分量 |
| 8 | 姿态控制器给STEP模块的目标速度`cmd_sps` |
| 9 | STEP斜坡后的当前输出频率`step_sps` |

Task5 视觉模式每 20 ms 输出一行 `B` 前缀纯数字帧。SerialPlot 使用 ASCII、
10 通道、逗号分隔，`Filter by Prefix` 选择 `Include` 并填写 `B`：

```text
B raw_x,error_x,yaw_cmd_sps,yaw_step_sps,yaw_step_count,
  raw_y,error_y,pitch_cmd_sps,pitch_step_sps,pitch_step_count
```

| 通道 | 含义 |
| ---: | --- |
| 1 | 视觉选择源的 X 原始误差，单位 0.1 像素 |
| 2 | 加入 `offsetX` 后的 yaw 控制误差，单位 0.1 像素 |
| 3 | yaw 闭环目标速度，单位 SPS |
| 4 | yaw 斜坡后的 STEP 输出频率，单位 SPS |
| 5 | yaw 累计有符号 STEP 数 |
| 6 | 视觉选择源的 Y 原始误差，单位 0.1 像素 |
| 7 | 加入 `offsetY` 后的 pitch 控制误差，单位 0.1 像素 |
| 8 | pitch 闭环目标速度，单位 SPS |
| 9 | pitch 斜坡后的 STEP 输出频率，单位 SPS |
| 10 | pitch 累计有符号 STEP 数 |

当前 K230 协议发送的是 `target-current` 误差，不包含目标和当前点的绝对坐标，
所以 B 通道 1/6 表示目标误差变化。需要绝对坐标时必须先扩展视觉串口协议。
推荐测试顺序为 `mode vision`、`vconfig near center`、`vshow`，确认参数后再打开
SerialPlot；结束时发送 `stop`，不能用拔掉串口代替停车。

`step_sps` 和 `step_count` 都来自 MCU 的 STEP 发生器，不是机械轴编码器反馈。
闭环步进驱动器虽然用电机编码器在驱动器内部纠正位置，但当前接线只有 STEP/DIR，
因此从 MCU 视角仍然只能把 STEP 当作伪反馈：正常无报警时可以近似认为机械轴跟上，
失步、堵转或驱动器内部跟随误差则无法从这些通道直接判断。

要获得真实云台反馈，按优先级可采用：驱动器若支持 UART/RS485/CAN，则读取其实际
位置和速度；若只提供 `ALM/IN_POSITION`，至少把报警/到位脚接回 MCU，但它不能给出
连续角度；最直接的是在 yaw 输出轴增加 AS5600、MT6701 等绝对磁编码器，由 MCU
计算实际角度和角速度。只有驱动器通信返回实际位置，或外部绝对编码器连续回传
位置并接入控制器后，`step_error` 才能升级为真正的机械角度误差。

底盘控制任务始终每 20 ms 读取并清零一次左右编码器窗口计数。串口默认每 500 ms 输出一行状态；执行命令或修改参数时立即回显并刷新。每次修改 `pwm` 或 `target` 后会丢弃第一个混合窗口，再从新的完整 20 ms 窗口累计平均值。

NO YAW强转当前把内轮目标设为`-1 count/20ms`，通过正常闭环直接产生轻微反向
作用；左转命令为`(-1, outer)`，右转命令为`(outer, -1)`。强转期间不启用零目标
阻尼分支，普通停车、出弯和标定也不启用。零目标阻尼接口仍保留供后续单独试验。

Task1 的 `CAR_MOTOR_NO_YAW_*_COUNTS_PER_PERIOD` 与串口 `target/move` 同单位且保留
正负号。正式 Task4 直接调用 Task1 的 `MotorNoYaw_Start()`；保留的 Task4 profile 宏
仅作为兼容入口，不参与当前三条 Task4 子菜单路线。

## 命令

串口 PWM 参数使用百分比，范围 `-100..100%` 映射到内部 `-1200..1200 PWM count`：

```text
PWM_raw = PWM_percent * 1200 / 100
```

TIMA0 周期为 1600，软件限幅暂为 1200，因此当前 `30%=360 count`。日志同时输出百分比和 `raw`；FF 计算始终使用换算后的实际 PWM count。

| 命令 | 作用 | 示例 |
| --- | --- | --- |
| `mode chassis\|gimbal\|vision` | 在底盘、姿态和视觉云台调试之间安全切换 | `mode chassis` |
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
