# 03. 应用层逻辑：菜单、状态机、循迹和闭环

应用层解决的问题是：硬件模块都能用了以后，小车现在应该做什么？

## app.c：总调度

`App_Init()` 初始化应用模块：

```c
Tracking_Init();
Route_Init();
SpeedControl_Init();
MotorTest_Init();
Menu_Init();
StateMachine_Init();
```

`App_Task()` 是主循环每轮跑的任务：

```c
App_HandleKeyEvent(Key_PopEvent());
StateMachine_Task();
SpeedControl_Task();
Menu_Task(StateMachine_GetState());
Link_Task();
delay_ms(CAR_APP_LOOP_DELAY_MS);
```

人话版：

1. 取一次按键事件。
2. 根据当前状态执行当前任务。
3. 用编码器刷新速度闭环输出。
4. 刷新 OLED 菜单或监视页面。
5. 处理串口后台任务。
6. 延时 10ms 左右，控制主循环节奏。

## 按键怎么控制 OLED 菜单

按键先由 `key.c` 变成 `KEY_EVENT_1` 或 `KEY_EVENT_2`，然后 `App_HandleKeyEvent()` 根据当前状态解释它。

在菜单状态：

- KEY1：确认当前菜单项。
- KEY2：移动到下一个菜单项。

代码逻辑：

```c
if (state == CAR_STATE_MENU) {
    if (event == KEY_EVENT_1) {
        StateMachine_Dispatch(Menu_Confirm());
    } else if (event == KEY_EVENT_2) {
        Menu_Next();
    }
}
```

在校准、测试、错误等页面：

- KEY1 常用于确认、下一步、应用。
- KEY2 常用于返回菜单。

所以你看 OLED 菜单时，要同时看三层：

```text
key.c              把 PB9/PB8 中断变成事件
app.c              把按键事件翻译成菜单操作或状态机事件
menu.c/state_machine.c  真正切页面或切状态
```

## state_machine.c：整车状态机

状态机就是“当前小车处在哪个大模式”。

`CarState` 包括：

- `CAR_STATE_MENU`：OLED 菜单。
- `CAR_STATE_GRAY_CALIBRATION`：灰度校准。
- `CAR_STATE_TRACKING`：正式循迹，带路线外环。
- `CAR_STATE_TRACKING_TEST`：单独测试循迹。
- `CAR_STATE_MOTOR_TEST`：电机方向测试。
- `CAR_STATE_PID_MONITOR`：速度闭环数据监视。
- `CAR_STATE_GRAY_MONITOR`：灰度数据监视。
- `CAR_STATE_ENCODER_MONITOR`：编码器数据监视。
- `CAR_STATE_MISSION`：赛题任务占位。
- `CAR_STATE_STOP`、`CAR_STATE_ERROR`：停止或错误。

外部不直接随便改状态，而是发事件：

```c
StateMachine_Dispatch(CAR_EVENT_GRAY_CALIBRATION_START);
```

然后状态机决定要不要切换状态。

这是一种很重要的嵌入式写法：把“发生了什么”和“状态怎么变”分开。按键、菜单、异常都只发事件，状态机统一决定整车行为。

## menu.c：OLED 菜单和监视页

`menu.c` 主要做两类事：

1. 菜单选择：主菜单和测试菜单。
2. 监视页面：显示灰度、PID、编码器等数据。

主菜单有：

```text
1.Calib
2.Test
3.Mission
```

测试菜单有：

```text
Motor Dir
Track Only
PID Data
Gray Data
Exchange
Encoder
Back
```

`Menu_Confirm()` 会把当前菜单项转换成状态机事件。例如选中 `Gray Data` 时返回：

```c
CAR_EVENT_GRAY_MONITOR_START
```

`Menu_Task()` 不会每轮都刷屏，而是用 `CAR_MENU_REFRESH_TICKS` 控制刷新周期，避免 OLED 刷太频繁。

## tracking.c：灰度误差到左右轮命令

`Tracking_Task()` 是循迹主入口。

流程：

```text
如果循迹没启用 -> 直接返回
读取路线层给的基础速度 baseDuty 和转向限制 turnLimit
Gray_Update() 采 7 路灰度
如果看到线 -> Gray_GetWeightedLineError() 算误差
TrackingException_Update() 决定正常跑、保持、搜线或停车
Motion_SetSpeed(leftDuty, rightDuty)
```

灰度误差的方向：

- 负数：线偏左。
- 0 附近：线在中间。
- 正数：线偏右。

转向修正的核心在 `tracking_exception.c` 里：

```c
correction = lineError * CAR_TRACK_TURN_GAIN / GRAY_LINE_ERROR_SCALE;
leftDuty = baseDuty + correction;
rightDuty = baseDuty - correction;
```

如果线偏右，`lineError` 为正，左轮加速、右轮减速，小车向右修。

## tracking_exception.c：丢线和异常处理

这个模块让循迹不要“一丢线就立刻乱停”。

它会处理几种情况：

- ADC 采样失败：连续失败达到阈值后停车并进入错误。
- 灰度全 0：说明看不到线，先短暂保持上一拍输出。
- 保持后还没找回：根据上一次线偏左还是偏右，温和搜线。
- 搜线超时：按配置停车。
- 多路同时压线：认为可能进入宽线/路口，状态标成 `wide_line`。

它的输出不是直接控制电机，而是告诉 `tracking.c`：

```text
继续跑，并给出 leftDuty/rightDuty
或者立即停车
```

## motion.c：统一运动接口

`motion.c` 是上层和电机之间的中间层。

如果开启速度闭环：

```c
Motion_SetSpeed() -> SpeedControl_SetTarget()
```

如果关闭速度闭环：

```c
Motion_SetSpeed() -> Motor_SetSpeed()
```

好处是：循迹代码不用管当前到底是开环 PWM，还是编码器闭环。它只要调用 `Motion_SetSpeed()`。

## speed_control.c：编码器 PI 速度闭环

速度闭环的目标是：左右电机即使有差异，也尽量按目标速度走。

每个轮子有一个结构体：

```c
typedef struct {
    int16_t targetCommand;
    int16_t currentCommand;
    int32_t lastCount;
    int32_t integral;
    int16_t actualTicks;
    int16_t outputPwm;
    int8_t encoderSign;
} SpeedControlWheel;
```

人话版：

- `targetCommand`：上层想要的速度命令。
- `currentCommand`：当前正在慢慢靠近的命令，避免突然加速。
- `lastCount`：上一次编码器计数。
- `actualTicks`：一个控制周期里实际走了多少编码器 tick。
- `integral`：PI 控制里的积分项。
- `outputPwm`：最后给电机的 PWM 命令。
- `encoderSign`：编码器方向修正。

核心流程：

```text
读取当前编码器计数
计算本周期实际 tick
目标命令换算成目标 tick
误差 = 目标 tick - 实际 tick
输出 = 当前命令 + KP * 误差 + KI * 积分
限幅
Motor_SetSpeed(输出)
```

你现在可以先理解成：如果轮子比目标慢，就加 PWM；如果比目标快，就减 PWM。

## route.c：路线外环

`route.c` 比普通循迹高一层。循迹只关心“跟着线走”，路线外环关心“现在是直道、接近拐角、正在转弯、出弯”。

路线阶段：

```text
IDLE
STRAIGHT
APPROACH_CORNER
TURNING
EXIT_CORNER
```

它用两个信息判断阶段：

- 编码器路程：判断直道走了多远、是否接近拐角。
- JY61P yaw：判断直角是否转够 90 度。

它不直接控制电机，而是给 `tracking.c` 提供两个参数：

- `Route_GetBaseDuty()`：当前阶段推荐的基础速度。
- `Route_GetTurnLimit()`：当前阶段允许最大转向修正。

所以 `tracking.c` 仍然负责根据灰度误差算左右轮，只是基础速度和转向幅度会随路线阶段变化。

## motor_test.c：电机方向确认

电机方向测试用于确认接线和方向有没有反。

测试步骤：

```text
left_forward
left_reverse
right_forward
right_reverse
both_forward
both_reverse
done
```

菜单里选 `Motor Dir` 后，KEY1 会切到下一步，KEY2 返回。每一步只低速短时间输出，并会自动停车，防止车一直跑。

