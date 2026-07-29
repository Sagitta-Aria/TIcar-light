#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "library_config.h"
#include "ti_msp_dl_config.h"

/*
 * pin_map.h：把业务名称映射到SysConfig生成的端口、引脚和复用定义。
 *
 * 这里描述“接到哪个脚”，不放速度、增益或算法选择。改线时必须同时核对
 * doc/PINOUT.md和SysConfig；只改本文件不能凭空创建定时器/ADC复用功能。
 * generated/ti_msp_dl_config.*由SysConfig生成，不要直接手工修改。
 *
 * 当前板型由library_config.h的CAR_LIBRARY_BOARD_PROFILE选择。
 * 详细排针与硬件冲突见doc/PINOUT.md。
 */

/*
 * 底盘改为两路编码直流电机，云台仍为两路 STEP/DIR 步进电机。
 * PA31/PB19 已被底盘方向和编码器占用，云台 EN 必须在硬件上固定有效。
 */

/* 左轮：实车接 TB6612 B 通道，TIMA0 CCP3 输出 PWM。 */
#define PIN_CHASSIS_LEFT_PWM_PORT            GPIOA
#define PIN_CHASSIS_LEFT_PWM                 DL_GPIO_PIN_12
#define PIN_CHASSIS_LEFT_PWM_IOMUX           IOMUX_PINCM34
#define PIN_CHASSIS_LEFT_PWM_FUNC            IOMUX_PINCM34_PF_TIMA0_CCP3
#define PIN_CHASSIS_LEFT_PWM_CC_INDEX        DL_TIMER_CC_3_INDEX

#if CAR_LIBRARY_BOARD_IS_TIANMENG
/* 天猛星的PA21/PA23是VREF-/VREF+网络，方向脚迁到PA29/PA30。 */
#define PIN_CHASSIS_LEFT_IN1_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN1                 DL_GPIO_PIN_29
#define PIN_CHASSIS_LEFT_IN1_IOMUX           IOMUX_PINCM4
#define PIN_CHASSIS_LEFT_IN2_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN2                 DL_GPIO_PIN_30
#define PIN_CHASSIS_LEFT_IN2_IOMUX           IOMUX_PINCM5
#else
#define PIN_CHASSIS_LEFT_IN1_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN1                 DL_GPIO_PIN_21
#define PIN_CHASSIS_LEFT_IN1_IOMUX           IOMUX_PINCM46
#define PIN_CHASSIS_LEFT_IN2_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN2                 DL_GPIO_PIN_23
#define PIN_CHASSIS_LEFT_IN2_IOMUX           IOMUX_PINCM53
#endif

/* 右轮：实车接 TB6612 A 通道，TIMA0 CCP1 输出 PWM。 */
#define PIN_CHASSIS_RIGHT_PWM_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_PWM                DL_GPIO_PIN_22
#define PIN_CHASSIS_RIGHT_PWM_IOMUX          IOMUX_PINCM47
#define PIN_CHASSIS_RIGHT_PWM_FUNC           IOMUX_PINCM47_PF_TIMA0_CCP1
#define PIN_CHASSIS_RIGHT_PWM_CC_INDEX       DL_TIMER_CC_1_INDEX
#define PIN_CHASSIS_RIGHT_IN1_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_IN1                DL_GPIO_PIN_31
#define PIN_CHASSIS_RIGHT_IN1_IOMUX          IOMUX_PINCM6
#define PIN_CHASSIS_RIGHT_IN2_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_IN2                DL_GPIO_PIN_28
#define PIN_CHASSIS_RIGHT_IN2_IOMUX          IOMUX_PINCM3

/* 左右编码器 A/B 相输入，四路均使用 GPIO 双边沿中断。 */
#define PIN_CHASSIS_LEFT_ENCODER_A_PORT      GPIOA
#define PIN_CHASSIS_LEFT_ENCODER_A           DL_GPIO_PIN_13
#define PIN_CHASSIS_LEFT_ENCODER_A_IOMUX     IOMUX_PINCM35
#define PIN_CHASSIS_LEFT_ENCODER_B_PORT      GPIOB
#define PIN_CHASSIS_LEFT_ENCODER_B           DL_GPIO_PIN_24
#define PIN_CHASSIS_LEFT_ENCODER_B_IOMUX     IOMUX_PINCM52
#define PIN_CHASSIS_RIGHT_ENCODER_A_PORT     GPIOB
#define PIN_CHASSIS_RIGHT_ENCODER_A          DL_GPIO_PIN_19
#define PIN_CHASSIS_RIGHT_ENCODER_A_IOMUX    IOMUX_PINCM45
#define PIN_CHASSIS_RIGHT_ENCODER_B_PORT     GPIOB
#define PIN_CHASSIS_RIGHT_ENCODER_B          DL_GPIO_PIN_20
#define PIN_CHASSIS_RIGHT_ENCODER_B_IOMUX    IOMUX_PINCM48

/* 云台轴语义别名；右侧 STEPPER_GIMBAL_1/2 是 SysConfig 生成的硬件名。 */
#define PIN_STEPPER_GIMBAL_YAW_STEP_PORT     STEPPER_GIMBAL_2_STEP_PORT
#define PIN_STEPPER_GIMBAL_YAW_STEP          STEPPER_GIMBAL_2_STEP_PIN
#define PIN_STEPPER_GIMBAL_YAW_DIR_PORT      STEPPER_GIMBAL_2_DIR_PORT
#define PIN_STEPPER_GIMBAL_YAW_DIR           STEPPER_GIMBAL_2_DIR_PIN
#define PIN_STEPPER_GIMBAL_PITCH_STEP_PORT   STEPPER_GIMBAL_1_STEP_PORT
#define PIN_STEPPER_GIMBAL_PITCH_STEP        STEPPER_GIMBAL_1_STEP_PIN
#define PIN_STEPPER_GIMBAL_PITCH_DIR_PORT    STEPPER_GIMBAL_1_DIR_PORT
#define PIN_STEPPER_GIMBAL_PITCH_DIR         STEPPER_GIMBAL_1_DIR_PIN

/* K1/K2沿用SysConfig的KEY组；有效电平在board_config.h配置。 */
#define PIN_KEY_PORT                    KEY_PORT /* 两个按键共用的GPIO端口。 */
#define PIN_KEY_1                       KEY_1_PIN /* K1：切换菜单项。 */
#define PIN_KEY_2                       KEY_2_PIN /* K2：确认/长按停止。 */
#define PIN_KEY_1_IOMUX                 KEY_1_IOMUX
#define PIN_KEY_2_IOMUX                 KEY_2_IOMUX
#define PIN_KEY_1_EDGE_RISE_FALL        KEY_1_EDGE_RISE_FALL
#define PIN_KEY_2_EDGE_RISE_FALL        KEY_2_EDGE_RISE_FALL

/* UART1的引脚会跟随板型；当前业务层由JY61P驱动使用。 */
#define PIN_UART1_TX_PORT               GPIO_JY61P_TX_PORT
#define PIN_UART1_TX                    GPIO_JY61P_TX_PIN
#define PIN_UART1_TX_IOMUX              GPIO_JY61P_IOMUX_TX
#define PIN_UART1_RX_PORT               GPIO_JY61P_RX_PORT
#define PIN_UART1_RX                    GPIO_JY61P_RX_PIN
#define PIN_UART1_RX_IOMUX              GPIO_JY61P_IOMUX_RX

/* HC-05 UART follows the selected board header and must be wired crossed. */
#if CAR_LIBRARY_BOARD_IS_TIANMENG
#define PIN_BLUETOOTH_UART_TX_PORT       GPIOB
#define PIN_BLUETOOTH_UART_TX            DL_GPIO_PIN_15
#define PIN_BLUETOOTH_UART_TX_IOMUX      IOMUX_PINCM32
#define PIN_BLUETOOTH_UART_TX_FUNC       IOMUX_PINCM32_PF_UART2_TX
#define PIN_BLUETOOTH_UART_RX_PORT       GPIOB
#define PIN_BLUETOOTH_UART_RX            DL_GPIO_PIN_16
#define PIN_BLUETOOTH_UART_RX_IOMUX      IOMUX_PINCM33
#define PIN_BLUETOOTH_UART_RX_FUNC       IOMUX_PINCM33_PF_UART2_RX
#else
#define PIN_BLUETOOTH_UART_TX_PORT       GPIOB
#define PIN_BLUETOOTH_UART_TX            DL_GPIO_PIN_2
#define PIN_BLUETOOTH_UART_TX_IOMUX      IOMUX_PINCM15
#define PIN_BLUETOOTH_UART_TX_FUNC       IOMUX_PINCM15_PF_UART3_TX
#define PIN_BLUETOOTH_UART_RX_PORT       GPIOB
#define PIN_BLUETOOTH_UART_RX            DL_GPIO_PIN_3
#define PIN_BLUETOOTH_UART_RX_IOMUX      IOMUX_PINCM16
#define PIN_BLUETOOTH_UART_RX_FUNC       IOMUX_PINCM16_PF_UART3_RX
#endif

/* 模拟灰度方法使用SysConfig建立的两个ADC实例；数字方法不会启动转换。 */
#define PIN_GRAY_ADC0                   GRAY_ADC0_INST /* S4至S7所在ADC实例。 */
#define PIN_GRAY_ADC1                   GRAY_ADC1_INST /* S1至S3所在ADC实例。 */

/* 数字灰度从车头朝前按左到右排列：S1,S2,S3,S4,S5,S6,S7。 */
#define PIN_GRAY_DIGITAL_PORT           GPIOA          /* 七路数字灰度当前全部位于GPIOA。 */
#define PIN_GRAY_1                      DL_GPIO_PIN_15  /* S1：最左侧，左转出弯检测。 */
#define PIN_GRAY_2                      DL_GPIO_PIN_16  /* S2：左侧直角组合检测。 */
#define PIN_GRAY_3                      DL_GPIO_PIN_17  /* S3：左侧普通循迹修正。 */
#define PIN_GRAY_4                      DL_GPIO_PIN_24  /* S4：中间循迹基准。 */
#define PIN_GRAY_5                      DL_GPIO_PIN_25  /* S5：右侧普通循迹修正。 */
#define PIN_GRAY_6                      DL_GPIO_PIN_26  /* S6：右侧直角组合检测。 */
#define PIN_GRAY_7                      DL_GPIO_PIN_27  /* S7：最右侧，右转出弯检测。 */

/* 每路GPIO输入复用值沿用GRAY_Sx的SysConfig名称。 */
#define PIN_GRAY_1_IOMUX                GRAY_S1_IOMUX
#define PIN_GRAY_2_IOMUX                GRAY_S2_IOMUX
#define PIN_GRAY_3_IOMUX                GRAY_S3_IOMUX
#define PIN_GRAY_4_IOMUX                GRAY_S4_IOMUX
#define PIN_GRAY_5_IOMUX                GRAY_S5_IOMUX
#define PIN_GRAY_6_IOMUX                GRAY_S6_IOMUX
#define PIN_GRAY_7_IOMUX                GRAY_S7_IOMUX

/* 板载调试灯：地猛星用PA14，天猛星用板载PB22 USER_LED。 */
#if CAR_LIBRARY_BOARD_IS_TIANMENG
#define PIN_DEBUG_LED_PORT              GPIOB
#define PIN_DEBUG_LED                   DL_GPIO_PIN_22
#define PIN_DEBUG_LED_IOMUX             IOMUX_PINCM50
#else
#define PIN_DEBUG_LED_PORT              GPIOA
#define PIN_DEBUG_LED                   DL_GPIO_PIN_14
#define PIN_DEBUG_LED_IOMUX             IOMUX_PINCM36
#endif

/*
 * 天猛星预留扩展引脚。这些宏只记录已确认的接线，不会初始化外设，
 * 也不会增加Flash体积；对应驱动启用时仍需单独的库开关。
 */
#if CAR_LIBRARY_BOARD_IS_TIANMENG
#define PIN_TIANMENG_EXTENSIONS_AVAILABLE  (1U)

#define PIN_KEY_3_PORT                  GPIOB
#define PIN_KEY_3                       DL_GPIO_PIN_21
#define PIN_KEY_3_IOMUX                 IOMUX_PINCM49
#define PIN_KEY_4_PORT                  GPIOB
#define PIN_KEY_4                       DL_GPIO_PIN_10
#define PIN_KEY_4_IOMUX                 IOMUX_PINCM27
#define PIN_KEY_5_PORT                  GPIOB
#define PIN_KEY_5                       DL_GPIO_PIN_11
#define PIN_KEY_5_IOMUX                 IOMUX_PINCM28

#define PIN_RELAY_CTRL_PORT             GPIOB
#define PIN_RELAY_CTRL                  DL_GPIO_PIN_27
#define PIN_RELAY_CTRL_IOMUX            IOMUX_PINCM58

#define PIN_AUX_PWM_PORT                GPIOA
#define PIN_AUX_PWM                     DL_GPIO_PIN_14
#define PIN_AUX_PWM_IOMUX               IOMUX_PINCM36
#define PIN_AUX_PWM_FUNC                IOMUX_PINCM36_PF_TIMG12_CCP0
#define PIN_AUX_PWM_CC_INDEX            DL_TIMER_CC_0_INDEX

#define PIN_EXTERNAL_LED_1_PORT         GPIOB
#define PIN_EXTERNAL_LED_1              DL_GPIO_PIN_23
#define PIN_EXTERNAL_LED_1_IOMUX        IOMUX_PINCM51
#define PIN_EXTERNAL_LED_2_PORT         GPIOB
#define PIN_EXTERNAL_LED_2              DL_GPIO_PIN_25
#define PIN_EXTERNAL_LED_2_IOMUX        IOMUX_PINCM56

#define PIN_FLASH_CS_PORT               GPIOB
#define PIN_FLASH_CS                    DL_GPIO_PIN_6
#define PIN_FLASH_CS_IOMUX              IOMUX_PINCM23
#define PIN_IMU_SPI_POCI_PORT           GPIOB
#define PIN_IMU_SPI_POCI                DL_GPIO_PIN_7
#define PIN_IMU_SPI_POCI_IOMUX          IOMUX_PINCM24
#define PIN_IMU_SPI_POCI_FUNC           IOMUX_PINCM24_PF_SPI1_POCI
#define PIN_IMU_SPI_PICO_PORT           GPIOB
#define PIN_IMU_SPI_PICO                DL_GPIO_PIN_8
#define PIN_IMU_SPI_PICO_IOMUX          IOMUX_PINCM25
#define PIN_IMU_SPI_PICO_FUNC           IOMUX_PINCM25_PF_SPI1_PICO
#define PIN_IMU_SPI_SCK_PORT            GPIOB
#define PIN_IMU_SPI_SCK                 DL_GPIO_PIN_9
#define PIN_IMU_SPI_SCK_IOMUX           IOMUX_PINCM26
#define PIN_IMU_SPI_SCK_FUNC            IOMUX_PINCM26_PF_SPI1_SCLK
#define PIN_IMU_CS_PORT                 GPIOB
#define PIN_IMU_CS                      DL_GPIO_PIN_14
#define PIN_IMU_CS_IOMUX                IOMUX_PINCM31
#define PIN_IMU_INT1_PORT               GPIOB
#define PIN_IMU_INT1                    DL_GPIO_PIN_17
#define PIN_IMU_INT1_IOMUX              IOMUX_PINCM43
#define PIN_IMU_INT2_PORT               GPIOB
#define PIN_IMU_INT2                    DL_GPIO_PIN_12
#define PIN_IMU_INT2_IOMUX              IOMUX_PINCM29
#else
#define PIN_TIANMENG_EXTENSIONS_AVAILABLE  (0U)
#endif

#endif
