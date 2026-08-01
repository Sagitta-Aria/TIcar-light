#ifndef LIBRARY_CONFIG_H
#define LIBRARY_CONFIG_H

#include "profile_select.h"

/*
 * 电赛复用库选择入口。
 *
 * 【初次使用先看这里】
 * 1. 本文件上半部分的 0U、1U、2U、3U 只是“方法编号”：
 *      1U = 无符号整数 1，2U = 无符号整数 2。
 *    字母 U 是 C 语言的 unsigned（无符号）后缀，不代表单位，也不代表开启。
 * 2. 这些编号不是速度、PWM、电压、传感器数量或调参值。同一个编号可以在
 *    不同方法类别中重复，因为程序只在各自类别内部比较编号。
 * 3. 不要把当前方案写成裸数字，例如不要写成：
 *      #define CAR_LIBRARY_GRAY_INPUT_METHOD  (2U)
 *    应始终填写带含义的方法名，例如：
 *      #define CAR_LIBRARY_GRAY_INPUT_METHOD \
 *          CAR_LIBRARY_GRAY_INPUT_ANALOG_ADC_7
 *    这样以后看代码时，能够直接知道选择的是“7路ADC模拟灰度”。
 * 4. 当前产品组合不在本文件中修改：GMR看profiles/profile_gmr.h，完整版本看
 *    profiles/profile_full.h。上方方法编号和下方派生开关不要手动修改。
 * 5. 每一类只能选择一种方法。具体速度、增益、时间和限幅仍放在
 *    board_config.h、control_config.h 和 staticconfig.c。
 *
 * 比赛前通过这里组合已经调试好的方法，不要在业务代码里临时替换算法。
 */

/* ---------- 主控板引脚配置库 ---------- */
#define CAR_LIBRARY_BOARD_DIMENG_48P                (1U) /* 地猛星 48P 最小系统板。 */
#define CAR_LIBRARY_BOARD_TIANMENG_64P              (2U) /* 天猛星 64P 开发板。 */

/*
 * 未由构建参数指定板型时，使用各产品 Profile 的发布板型。
 * 切换板型会自动改变电机方向脚、按键、UART1 和板载状态灯的引脚定义。
 */
#ifndef CAR_LIBRARY_BOARD_PROFILE
#if CAR_PROFILE_IS_GMR
#define CAR_LIBRARY_BOARD_PROFILE \
    CAR_LIBRARY_BOARD_TIANMENG_64P
#else
#define CAR_LIBRARY_BOARD_PROFILE \
    CAR_LIBRARY_BOARD_DIMENG_48P
#endif
#endif

#define CAR_LIBRARY_BOARD_IS_DIMENG \
    (CAR_LIBRARY_BOARD_PROFILE == CAR_LIBRARY_BOARD_DIMENG_48P)
#define CAR_LIBRARY_BOARD_IS_TIANMENG \
    (CAR_LIBRARY_BOARD_PROFILE == CAR_LIBRARY_BOARD_TIANMENG_64P)

#if ((CAR_LIBRARY_BOARD_PROFILE != CAR_LIBRARY_BOARD_DIMENG_48P) && \
    (CAR_LIBRARY_BOARD_PROFILE != CAR_LIBRARY_BOARD_TIANMENG_64P))
#error "Unknown main-board pin profile"
#endif

/* 板载JY61 UART1可按固件变体完全停止初始化；GMR发布目标默认关闭。 */
#ifndef CAR_JY61P_ENABLED
#if CAR_PROFILE_IS_GMR
#define CAR_JY61P_ENABLED                           (0U)
#else
#define CAR_JY61P_ENABLED                           (1U)
#endif
#endif

#if ((CAR_JY61P_ENABLED != 0U) && (CAR_JY61P_ENABLED != 1U))
#error "CAR_JY61P_ENABLED must be 0 or 1"
#endif

/* ---------- 灰度输入库 ---------- */
#define CAR_LIBRARY_GRAY_INPUT_NONE                 (0U) /* 方法编号0：完全关闭灰度输入和采样外设。 */
#define CAR_LIBRARY_GRAY_INPUT_DIGITAL_GPIO_7       (1U) /* 方法编号1：读取7路GPIO数字灰度，只得到黑/白状态。 */
#define CAR_LIBRARY_GRAY_INPUT_ANALOG_ADC_7         (2U) /* 方法编号2：读取7路ADC模拟灰度，可得到连续强度值。 */
#define CAR_LIBRARY_GRAY_INPUT_INFRARED_GPIO_8      (3U) /* 方法编号3：八路红外GPIO，黑线输出高电平。 */

/* ---------- 循迹算法库 ---------- */
#define CAR_LIBRARY_LINE_FOLLOW_NONE                 (0U) /* 方法编号0：关闭循迹算法，不根据灰度误差控制底盘。 */
#define CAR_LIBRARY_LINE_FOLLOW_DIGITAL_WEIGHTED     (1U) /* 方法编号1：对7路数字灰度加权，计算循迹方向和偏差。 */

/* ---------- 循迹底盘输出库 ---------- */
#define CAR_LIBRARY_LINE_DRIVE_DIRECT_PWM            (1U) /* 方法编号1：循迹结果直接改变左右电机PWM，不使用编码器闭环。 */
#define CAR_LIBRARY_LINE_DRIVE_PWM_ENCODER_SYNC      (2U) /* 方法编号2：PWM驱动，并用编码器差值补偿左右轮不同步。 */
#define CAR_LIBRARY_LINE_DRIVE_ENCODER_SPEED_LOOP    (3U) /* 方法编号3：把目标轮速交给左右轮编码器速度闭环。 */

/* ---------- 直角转向库 ---------- */
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_NONE             (0U) /* 方法编号0：关闭独立的直角转向流程。 */
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_LOCKED_INNER     (1U) /* 方法编号1：内轮停止、外轮前转，完成单轮锁死转向。 */
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_COUNTER_ROTATE   (2U) /* 方法编号2：内外轮反向旋转；出弯后内轮略快地向前恢复。 */

/* ---------- 强转速度曲线库 ---------- */
#define CAR_LIBRARY_TURN_SPEED_FIXED                  (1U) /* 方法编号1：强转期间始终使用固定的左右轮转向速度。 */
#define CAR_LIBRARY_TURN_SPEED_JY61_ANGLE_PROFILE     (2U) /* 方法编号2：根据JY61转过的角度，分阶段调整转向速度。 */

/* ---------- 二维激光打靶云台库 ---------- */
#define CAR_LIBRARY_GIMBAL_TRACKING_NONE              (0U) /* 方法编号0：关闭视觉目标跟踪和激光打靶控制。 */
#define CAR_LIBRARY_GIMBAL_TRACKING_2D_LASER_PD_FF    (1U) /* 方法编号1：用二维目标误差进行PD加前馈云台控制。 */

/* ---------- 云台姿态矫正库 ---------- */
#define CAR_LIBRARY_GIMBAL_ATTITUDE_NONE              (0U) /* 方法编号0：关闭车身姿态引起的云台补偿。 */
#define CAR_LIBRARY_GIMBAL_ATTITUDE_H7_JY61_DUAL_IMU  (1U) /* 方法编号1：融合H7云台姿态和JY61车身姿态进行矫正。 */

/* ---------- 云台起步搜点库 ---------- */
#define CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_NONE        (0U) /* 方法编号0：启动后不执行专门的搜点动作。 */
#define CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_FIXED_YAW   (1U) /* 方法编号1：启动时将yaw转到预设方向寻找目标。 */

/* ---------- 云台丢目标恢复库 ---------- */
#define CAR_LIBRARY_GIMBAL_LOST_TARGET_NONE           (0U) /* 方法编号0：目标丢失后不进行自动扫描。 */
#define CAR_LIBRARY_GIMBAL_LOST_TARGET_YAW_SWEEP      (1U) /* 方法编号1：目标丢失后让yaw往返扫描，尝试重新捕获。 */

/* ---------- Task4 强转固定 yaw 随动库 ---------- */
#define CAR_LIBRARY_GIMBAL_TURN_FOLLOW_NONE           (0U) /* 方法编号0：底盘强转时不执行Task4专用yaw随动。 */
#define CAR_LIBRARY_GIMBAL_TURN_FOLLOW_PHASED_STEP    (1U) /* 方法编号1：底盘强转时按阶段改变Task4的yaw目标。 */

/* ---------- H7 UART LCD显示库 ---------- */
#define CAR_LIBRARY_H7_LCD_NONE                        (0U) /* 方法编号0：关闭H7 LCD，不发送@L0至@L9显示命令。 */
#define CAR_LIBRARY_H7_LCD_UART_TEXT                   (1U) /* 方法编号1：通过所选H7 UART向H7发送文本行。 */

/* ---------- H7 UART IMU输入库 ---------- */
#define CAR_LIBRARY_H7_IMU_NONE                        (0U) /* 方法编号0：不初始化H7姿态接收。 */
#define CAR_LIBRARY_H7_IMU_UART_JY61                   (1U) /* 方法编号1：接收H7输出的JY61兼容姿态帧。 */

/* ---------- 天猛星SPI六轴IMU库 ---------- */
#define CAR_LIBRARY_IMU660RX_NONE                       (0U) /* 方法编号0：不初始化外接IMU660RX。 */
#define CAR_LIBRARY_IMU660RX_AUTO                       (1U) /* 方法编号1：按芯片ID自动识别RA/RB/RC。 */
#define CAR_LIBRARY_IMU660RX_RA                         (2U) /* 方法编号2：BMI270，对应IMU660RA。 */
#define CAR_LIBRARY_IMU660RX_RB                         (3U) /* 方法编号3：LSM6DSR，对应IMU660RB。 */
#define CAR_LIBRARY_IMU660RX_RC                         (4U) /* 方法编号4：LSM6DSV16X，对应IMU660RC。 */

/* ---------- 本地I2C OLED显示库 ---------- */
#define CAR_LIBRARY_LOCAL_OLED_NONE                    (0U) /* 方法编号0：不初始化板载I2C OLED。 */
#define CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306             (1U) /* 方法编号1：使用I2C SSD1306显示菜单前5行。 */

/* 产品Profile集中给出默认功能组合，命令行-D仍可覆盖单个METHOD。 */
#if CAR_PROFILE_IS_GMR
#include "profiles/profile_gmr.h"
#else
#include "profiles/profile_full.h"
#endif

/*
 * ---------- 供实现层使用的派生开关；不要手动修改 ----------
 *
 * 下面这些宏会把“当前方案”的选择自动换算成真/假条件，供 .c 文件中的
 * #if 使用。例如选了数字灰度后，CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL 自动为真。
 * 它们是选择结果，不是新的方法选项；手动修改会造成配置显示与实际代码不一致。
 */

/* 当前灰度输入是否为GPIO数字量。 */
#define CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL \
    ((CAR_LIBRARY_GRAY_INPUT_METHOD == \
         CAR_LIBRARY_GRAY_INPUT_DIGITAL_GPIO_7) || \
        (CAR_LIBRARY_GRAY_INPUT_METHOD == \
         CAR_LIBRARY_GRAY_INPUT_INFRARED_GPIO_8))

/* 当前是否选择独立封装的八路红外循迹模块。 */
#define CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8 \
    (CAR_LIBRARY_GRAY_INPUT_METHOD == \
        CAR_LIBRARY_GRAY_INPUT_INFRARED_GPIO_8)

/* 当前是否需要初始化任意灰度输入。 */
#define CAR_LIBRARY_GRAY_INPUT_ENABLED \
    (CAR_LIBRARY_GRAY_INPUT_METHOD != CAR_LIBRARY_GRAY_INPUT_NONE)

/* 是否启用了任意一种循迹算法。 */
#define CAR_LIBRARY_LINE_FOLLOW_ENABLED \
    (CAR_LIBRARY_LINE_FOLLOW_METHOD != CAR_LIBRARY_LINE_FOLLOW_NONE)

/* 当前循迹底盘是否使用编码器速度闭环。 */
#define CAR_LIBRARY_LINE_DRIVE_USES_SPEED_LOOP \
    (CAR_LIBRARY_LINE_DRIVE_METHOD == \
        CAR_LIBRARY_LINE_DRIVE_ENCODER_SPEED_LOOP)

/* 当前循迹底盘是否由PWM方法直接控制。 */
#define CAR_LIBRARY_LINE_DRIVE_USES_PWM \
    ((CAR_LIBRARY_LINE_DRIVE_METHOD == \
        CAR_LIBRARY_LINE_DRIVE_DIRECT_PWM) || \
     (CAR_LIBRARY_LINE_DRIVE_METHOD == \
        CAR_LIBRARY_LINE_DRIVE_PWM_ENCODER_SYNC))

/* 当前循迹底盘是否额外使用编码器做左右轮同步补偿。 */
#define CAR_LIBRARY_LINE_DRIVE_USES_PWM_ENCODER_SYNC \
    (CAR_LIBRARY_LINE_DRIVE_METHOD == \
        CAR_LIBRARY_LINE_DRIVE_PWM_ENCODER_SYNC)

/* 是否启用了任意一种直角转向方法。 */
#define CAR_LIBRARY_RIGHT_ANGLE_TURN_ENABLED \
    (CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD != \
        CAR_LIBRARY_RIGHT_ANGLE_TURN_NONE)

/* 直角转向是否需要读取JY61角度来切换速度阶段。 */
#define CAR_LIBRARY_TURN_SPEED_USES_JY61 \
    (CAR_LIBRARY_RIGHT_ANGLE_TURN_ENABLED && \
     (CAR_LIBRARY_TURN_SPEED_METHOD == \
        CAR_LIBRARY_TURN_SPEED_JY61_ANGLE_PROFILE))

/* 是否启用了任意一种二维激光打靶云台跟踪方法。 */
#define CAR_LIBRARY_GIMBAL_TRACKING_ENABLED \
    (CAR_LIBRARY_GIMBAL_TRACKING_METHOD != \
        CAR_LIBRARY_GIMBAL_TRACKING_NONE)

/* 是否启用了车身姿态对云台的矫正。 */
#define CAR_LIBRARY_GIMBAL_ATTITUDE_ENABLED \
    (CAR_LIBRARY_GIMBAL_ATTITUDE_METHOD != \
        CAR_LIBRARY_GIMBAL_ATTITUDE_NONE)

/* 云台跟踪已开启，并且选择了启动固定yaw搜点时，此结果才为真。 */
#define CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_ENABLED \
    (CAR_LIBRARY_GIMBAL_TRACKING_ENABLED && \
     (CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_METHOD == \
        CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_FIXED_YAW))

/* 云台跟踪已开启，并且选择了丢目标yaw扫描时，此结果才为真。 */
#define CAR_LIBRARY_GIMBAL_LOST_TARGET_ENABLED \
    (CAR_LIBRARY_GIMBAL_TRACKING_ENABLED && \
     (CAR_LIBRARY_GIMBAL_LOST_TARGET_METHOD == \
        CAR_LIBRARY_GIMBAL_LOST_TARGET_YAW_SWEEP))

/* 云台跟踪已开启，并且选择了Task4分阶段随动时，此结果才为真。 */
#define CAR_LIBRARY_GIMBAL_TURN_FOLLOW_ENABLED \
    (CAR_LIBRARY_GIMBAL_TRACKING_ENABLED && \
     (CAR_LIBRARY_GIMBAL_TURN_FOLLOW_METHOD == \
        CAR_LIBRARY_GIMBAL_TURN_FOLLOW_PHASED_STEP))

/* 是否启用了H7 UART文本LCD；关闭后不会发送任何@L/@CLEAR命令。 */
#define CAR_LIBRARY_H7_LCD_ENABLED \
    (CAR_LIBRARY_H7_LCD_METHOD == CAR_LIBRARY_H7_LCD_UART_TEXT)

/* 是否启用了H7 UART姿态输入；与板载UART1 JY61是两路独立传感器。 */
#define CAR_LIBRARY_H7_IMU_ENABLED \
    (CAR_LIBRARY_H7_IMU_METHOD == CAR_LIBRARY_H7_IMU_UART_JY61)

/* 是否启用了天猛星SPI1上的外接IMU660RA/RB/RC。 */
#define CAR_LIBRARY_IMU660RX_ENABLED \
    (CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_NONE)

/* RA和AUTO构建需要编入BMI270的8192字节官方配置数据。 */
#define CAR_LIBRARY_IMU660RX_USES_RA \
    ((CAR_LIBRARY_IMU660RX_METHOD == CAR_LIBRARY_IMU660RX_AUTO) || \
     (CAR_LIBRARY_IMU660RX_METHOD == CAR_LIBRARY_IMU660RX_RA))

/* 是否启用了本地I2C SSD1306 OLED；关闭后不初始化I2C OLED外设。 */
#define CAR_LIBRARY_LOCAL_OLED_ENABLED \
    (CAR_LIBRARY_LOCAL_OLED_METHOD == \
        CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306)

/* 任意显示后端启用时为真；两个后端可以同时启用。 */
#define CAR_LIBRARY_DISPLAY_ENABLED \
    (CAR_LIBRARY_H7_LCD_ENABLED || CAR_LIBRARY_LOCAL_OLED_ENABLED)

/* 转向角度曲线或双IMU矫正启用时，系统必须更新车身姿态数据。 */
#define CAR_LIBRARY_BODY_MOTION_REQUIRED \
    (CAR_LIBRARY_TURN_SPEED_USES_JY61 || \
     CAR_LIBRARY_GIMBAL_ATTITUDE_ENABLED)

/*
 * ---------- 选择值和依赖关系校验；不要手动修改 ----------
 *
 * 以下检查在编译阶段阻止无效配置：方法名写错、选择了不存在的编号，或者组合
 * 之间缺少必要依赖时，编译器会直接报出对应的 #error，而不是让错误带到赛场。
 */

/* 灰度输入可关闭，或选择旧7路数字、7路模拟、八路红外数字输入。 */
#if ((CAR_LIBRARY_GRAY_INPUT_METHOD != CAR_LIBRARY_GRAY_INPUT_NONE) && \
    (CAR_LIBRARY_GRAY_INPUT_METHOD != \
        CAR_LIBRARY_GRAY_INPUT_DIGITAL_GPIO_7) && \
    (CAR_LIBRARY_GRAY_INPUT_METHOD != \
        CAR_LIBRARY_GRAY_INPUT_ANALOG_ADC_7) && \
    (CAR_LIBRARY_GRAY_INPUT_METHOD != \
        CAR_LIBRARY_GRAY_INPUT_INFRARED_GPIO_8))
#error "Unknown gray input library method"
#endif

/* 循迹算法只能选择关闭或数字量加权循迹。 */
#if ((CAR_LIBRARY_LINE_FOLLOW_METHOD != CAR_LIBRARY_LINE_FOLLOW_NONE) && \
    (CAR_LIBRARY_LINE_FOLLOW_METHOD != \
        CAR_LIBRARY_LINE_FOLLOW_DIGITAL_WEIGHTED))
#error "Unknown line-follow library method"
#endif

/* 底盘输出只能选择三种已经实现并验证过的驱动方法之一。 */
#if ((CAR_LIBRARY_LINE_DRIVE_METHOD != \
        CAR_LIBRARY_LINE_DRIVE_DIRECT_PWM) && \
    (CAR_LIBRARY_LINE_DRIVE_METHOD != \
        CAR_LIBRARY_LINE_DRIVE_PWM_ENCODER_SYNC) && \
    (CAR_LIBRARY_LINE_DRIVE_METHOD != \
        CAR_LIBRARY_LINE_DRIVE_ENCODER_SPEED_LOOP))
#error "Unknown line-drive library method"
#endif

/* 直角转向只能选择关闭、内轮锁死或双轮反转。 */
#if ((CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD != \
        CAR_LIBRARY_RIGHT_ANGLE_TURN_NONE) && \
    (CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD != \
        CAR_LIBRARY_RIGHT_ANGLE_TURN_LOCKED_INNER) && \
    (CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD != \
        CAR_LIBRARY_RIGHT_ANGLE_TURN_COUNTER_ROTATE))
#error "Unknown right-angle turn library method"
#endif

/* 强转轮速只能选择固定速度或JY61角度分阶段速度。 */
#if ((CAR_LIBRARY_TURN_SPEED_METHOD != CAR_LIBRARY_TURN_SPEED_FIXED) && \
    (CAR_LIBRARY_TURN_SPEED_METHOD != \
        CAR_LIBRARY_TURN_SPEED_JY61_ANGLE_PROFILE))
#error "Unknown turn-speed library method"
#endif

/* 激光打靶云台只能选择关闭或二维PD加前馈跟踪。 */
#if ((CAR_LIBRARY_GIMBAL_TRACKING_METHOD != \
        CAR_LIBRARY_GIMBAL_TRACKING_NONE) && \
    (CAR_LIBRARY_GIMBAL_TRACKING_METHOD != \
        CAR_LIBRARY_GIMBAL_TRACKING_2D_LASER_PD_FF))
#error "Unknown gimbal-tracking library method"
#endif

/* 云台姿态矫正只能选择关闭或H7/JY61双IMU矫正。 */
#if ((CAR_LIBRARY_GIMBAL_ATTITUDE_METHOD != \
        CAR_LIBRARY_GIMBAL_ATTITUDE_NONE) && \
    (CAR_LIBRARY_GIMBAL_ATTITUDE_METHOD != \
        CAR_LIBRARY_GIMBAL_ATTITUDE_H7_JY61_DUAL_IMU))
#error "Unknown gimbal-attitude library method"
#endif

/* 启动搜点只能选择关闭或固定yaw搜点。 */
#if ((CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_METHOD != \
        CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_NONE) && \
    (CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_METHOD != \
        CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_FIXED_YAW))
#error "Unknown gimbal startup-search library method"
#endif

/* 丢目标恢复只能选择关闭或yaw往返扫描。 */
#if ((CAR_LIBRARY_GIMBAL_LOST_TARGET_METHOD != \
        CAR_LIBRARY_GIMBAL_LOST_TARGET_NONE) && \
    (CAR_LIBRARY_GIMBAL_LOST_TARGET_METHOD != \
        CAR_LIBRARY_GIMBAL_LOST_TARGET_YAW_SWEEP))
#error "Unknown gimbal lost-target library method"
#endif

/* Task4强转yaw随动只能选择关闭或分阶段目标值。 */
#if ((CAR_LIBRARY_GIMBAL_TURN_FOLLOW_METHOD != \
        CAR_LIBRARY_GIMBAL_TURN_FOLLOW_NONE) && \
    (CAR_LIBRARY_GIMBAL_TURN_FOLLOW_METHOD != \
        CAR_LIBRARY_GIMBAL_TURN_FOLLOW_PHASED_STEP))
#error "Unknown gimbal turn-follow library method"
#endif

/* H7 LCD只能选择关闭或UART文本协议。 */
#if ((CAR_LIBRARY_H7_LCD_METHOD != CAR_LIBRARY_H7_LCD_NONE) && \
    (CAR_LIBRARY_H7_LCD_METHOD != CAR_LIBRARY_H7_LCD_UART_TEXT))
#error "Unknown H7 LCD library method"
#endif

/* H7 IMU只能选择关闭或JY61兼容UART输入。 */
#if ((CAR_LIBRARY_H7_IMU_METHOD != CAR_LIBRARY_H7_IMU_NONE) && \
    (CAR_LIBRARY_H7_IMU_METHOD != CAR_LIBRARY_H7_IMU_UART_JY61))
#error "Unknown H7 IMU library method"
#endif

/* 外接SPI六轴只能关闭、自动识别或明确选择RA/RB/RC。 */
#if ((CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_NONE) && \
    (CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_AUTO) && \
    (CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_RA) && \
    (CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_RB) && \
    (CAR_LIBRARY_IMU660RX_METHOD != CAR_LIBRARY_IMU660RX_RC))
#error "Unknown IMU660RX library method"
#endif

/* 当前SPI1/CS接线只在天猛星64P配置中定义。 */
#if (CAR_LIBRARY_IMU660RX_ENABLED && !CAR_LIBRARY_BOARD_IS_TIANMENG)
#error "IMU660RX SPI library requires the Tianmeng 64P board profile"
#endif

/* 八路红外的IR8与IMU660RX SCK共用PB9，二者不能同时初始化。 */
#if (CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8 && \
    CAR_LIBRARY_IMU660RX_ENABLED)
#error "Eight-channel infrared IR8 on PB9 conflicts with IMU660RX SPI SCK"
#endif

/* 双IMU云台姿态方案必须有H7反馈，板载JY61不能单独替代它。 */
#if (CAR_LIBRARY_GIMBAL_ATTITUDE_ENABLED && \
    !CAR_LIBRARY_H7_IMU_ENABLED)
#error "Dual-IMU gimbal attitude requires the H7 IMU UART library"
#endif

/* 本地OLED只能选择关闭或I2C SSD1306驱动。 */
#if ((CAR_LIBRARY_LOCAL_OLED_METHOD != CAR_LIBRARY_LOCAL_OLED_NONE) && \
    (CAR_LIBRARY_LOCAL_OLED_METHOD != \
        CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306))
#error "Unknown local OLED library method"
#endif

/* 数字量加权算法只能处理数字灰度，若选模拟ADC输入则拒绝编译。 */
#if ((CAR_LIBRARY_LINE_FOLLOW_METHOD == \
        CAR_LIBRARY_LINE_FOLLOW_DIGITAL_WEIGHTED) && \
    !CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL)
#error "Digital weighted line follow requires a digital line-sensor input"
#endif

/* 直角转向依赖循迹状态，关闭循迹后不能单独启用直角转向。 */
#if (CAR_LIBRARY_RIGHT_ANGLE_TURN_ENABLED && \
    !CAR_LIBRARY_LINE_FOLLOW_ENABLED)
#error "Right-angle turn requires a line-follow library"
#endif

#endif
