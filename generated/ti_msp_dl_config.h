#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifndef CONFIG_MSPM0G350X
#define CONFIG_MSPM0G350X
#endif

#ifndef CONFIG_MSPM0G3507
#define CONFIG_MSPM0G3507
#endif

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__CC_ARM)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__) || defined(__clang__)
#define SYSCONFIG_WEAK __attribute__((weak))
#else
#define SYSCONFIG_WEAK
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STARTUP_DELAY                                                (16U)
#define CPUCLK_FREQ                                                     32000000
#define SYSCFG_DL_ENABLE_HFXT_PLL                                           (0U)

#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)

/*
 * Stepper DIR/STEP outputs.
 * 左底盘: PA7 STEP, PB18 DIR
 * 右底盘: PA8 STEP, PA9 DIR
 * 云台1:  PA12 STEP, PA22 DIR
 * 云台2:  PA13 STEP, PB24 DIR
 */
#define STEPPER_CHASSIS_LEFT_STEP_PORT                                     GPIOA
#define STEPPER_CHASSIS_LEFT_STEP_PIN                              DL_GPIO_PIN_7
#define STEPPER_CHASSIS_LEFT_STEP_IOMUX                           (IOMUX_PINCM14)
#define STEPPER_CHASSIS_LEFT_DIR_PORT                                      GPIOB
#define STEPPER_CHASSIS_LEFT_DIR_PIN                              DL_GPIO_PIN_18
#define STEPPER_CHASSIS_LEFT_DIR_IOMUX                            (IOMUX_PINCM44)

#define STEPPER_CHASSIS_RIGHT_STEP_PORT                                    GPIOA
#define STEPPER_CHASSIS_RIGHT_STEP_PIN                             DL_GPIO_PIN_8
#define STEPPER_CHASSIS_RIGHT_STEP_IOMUX                          (IOMUX_PINCM19)
#define STEPPER_CHASSIS_RIGHT_DIR_PORT                                     GPIOA
#define STEPPER_CHASSIS_RIGHT_DIR_PIN                              DL_GPIO_PIN_9
#define STEPPER_CHASSIS_RIGHT_DIR_IOMUX                           (IOMUX_PINCM20)

#define STEPPER_GIMBAL_1_STEP_PORT                                         GPIOA
#define STEPPER_GIMBAL_1_STEP_PIN                                 DL_GPIO_PIN_12
#define STEPPER_GIMBAL_1_STEP_IOMUX                               (IOMUX_PINCM34)
#define STEPPER_GIMBAL_1_DIR_PORT                                          GPIOA
#define STEPPER_GIMBAL_1_DIR_PIN                                  DL_GPIO_PIN_22
#define STEPPER_GIMBAL_1_DIR_IOMUX                                (IOMUX_PINCM47)

#define STEPPER_GIMBAL_2_STEP_PORT                                         GPIOA
#define STEPPER_GIMBAL_2_STEP_PIN                                 DL_GPIO_PIN_13
#define STEPPER_GIMBAL_2_STEP_IOMUX                               (IOMUX_PINCM35)
#define STEPPER_GIMBAL_2_DIR_PORT                                          GPIOB
#define STEPPER_GIMBAL_2_DIR_PIN                                  DL_GPIO_PIN_24
#define STEPPER_GIMBAL_2_DIR_IOMUX                                (IOMUX_PINCM52)

/* OLED I2C0: PA0 SDA, PA1 SCL，开漏释放并带上拉。 */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_0
#define GPIO_OLED_IOMUX_SDA                                       (IOMUX_PINCM1)
#define GPIO_OLED_IOMUX_SDA_FUNC                        IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_1
#define GPIO_OLED_IOMUX_SCL                                       (IOMUX_PINCM2)
#define GPIO_OLED_IOMUX_SCL_FUNC                        IOMUX_PINCM2_PF_I2C0_SCL

/* Type-C CH340 日志 UART0: PA10 TX, PA11 RX。PA18 仍只作为 BSL invoke。 */
#define LogUart_INST                                                       UART0
#define LogUart_INST_FREQUENCY                                          32000000
#define LogUart_INST_IRQHandler                                 UART0_IRQHandler
#define LogUart_INST_INT_IRQN                                     UART0_INT_IRQn
#define GPIO_LogUart_RX_PORT                                             GPIOA
#define GPIO_LogUart_TX_PORT                                             GPIOA
#define GPIO_LogUart_RX_PIN                                     DL_GPIO_PIN_11
#define GPIO_LogUart_TX_PIN                                     DL_GPIO_PIN_10
#define GPIO_LogUart_IOMUX_RX                                   (IOMUX_PINCM22)
#define GPIO_LogUart_IOMUX_TX                                   (IOMUX_PINCM21)
#define GPIO_LogUart_IOMUX_RX_FUNC                    IOMUX_PINCM22_PF_UART0_RX
#define GPIO_LogUart_IOMUX_TX_FUNC                    IOMUX_PINCM21_PF_UART0_TX
#define LogUart_BAUD_RATE                                            (115200)
#define LogUart_IBRD_32_MHZ_115200_BAUD                                  (17)
#define LogUart_FBRD_32_MHZ_115200_BAUD                                  (23)

/* JY61P UART1: PB6 TX, PB7 RX。JQ8400 暂停接入后释放给姿态模块。 */
#define JY61P_INST                                                         UART1
#define JY61P_INST_FREQUENCY                                            32000000
#define JY61P_INST_IRQHandler                                   UART1_IRQHandler
#define JY61P_INST_INT_IRQN                                       UART1_INT_IRQn
#define GPIO_JY61P_RX_PORT                                                 GPIOB
#define GPIO_JY61P_TX_PORT                                                 GPIOB
#define GPIO_JY61P_RX_PIN                                          DL_GPIO_PIN_7
#define GPIO_JY61P_TX_PIN                                          DL_GPIO_PIN_6
#define GPIO_JY61P_IOMUX_RX                                      (IOMUX_PINCM24)
#define GPIO_JY61P_IOMUX_TX                                      (IOMUX_PINCM23)
#define GPIO_JY61P_IOMUX_RX_FUNC                       IOMUX_PINCM24_PF_UART1_RX
#define GPIO_JY61P_IOMUX_TX_FUNC                       IOMUX_PINCM23_PF_UART1_TX
#define JY61P_BAUD_RATE                                                 (115200)
#define JY61P_IBRD_32_MHZ_115200_BAUD                                       (17)
#define JY61P_FBRD_32_MHZ_115200_BAUD                                       (23)

/* Exchange UART3: PB2 TX, PB3 RX. */
#define Exchange_INST                                                      UART3
#define Exchange_INST_FREQUENCY                                         32000000
#define Exchange_INST_IRQHandler                                UART3_IRQHandler
#define Exchange_INST_INT_IRQN                                    UART3_INT_IRQn
#define GPIO_Exchange_RX_PORT                                              GPIOB
#define GPIO_Exchange_TX_PORT                                              GPIOB
#define GPIO_Exchange_RX_PIN                                       DL_GPIO_PIN_3
#define GPIO_Exchange_TX_PIN                                       DL_GPIO_PIN_2
#define GPIO_Exchange_IOMUX_RX                                   (IOMUX_PINCM16)
#define GPIO_Exchange_IOMUX_TX                                   (IOMUX_PINCM15)
#define GPIO_Exchange_IOMUX_RX_FUNC                    IOMUX_PINCM16_PF_UART3_RX
#define GPIO_Exchange_IOMUX_TX_FUNC                    IOMUX_PINCM15_PF_UART3_TX
#define Exchange_BAUD_RATE                                              (115200)
#define Exchange_IBRD_32_MHZ_115200_BAUD                                    (17)
#define Exchange_FBRD_32_MHZ_115200_BAUD                                    (23)

/* Gray sensors: ADC0/ADC1 sequence sampling. */
#define GRAY_ADC0_INST                                                       ADC0
#define GRAY_ADC1_INST                                                       ADC1
#define GRAY_ADC0_MEM_GRAY4                                      DL_ADC12_MEM_IDX_0
#define GRAY_ADC0_MEM_GRAY5                                      DL_ADC12_MEM_IDX_1
#define GRAY_ADC0_MEM_GRAY6                                      DL_ADC12_MEM_IDX_2
#define GRAY_ADC0_MEM_GRAY7                                      DL_ADC12_MEM_IDX_3
#define GRAY_ADC1_MEM_GRAY1                                      DL_ADC12_MEM_IDX_0
#define GRAY_ADC1_MEM_GRAY2                                      DL_ADC12_MEM_IDX_1
#define GRAY_ADC1_MEM_GRAY3                                      DL_ADC12_MEM_IDX_2
#define GRAY_S1_IOMUX                                             (IOMUX_PINCM37)
#define GRAY_S2_IOMUX                                             (IOMUX_PINCM38)
#define GRAY_S3_IOMUX                                             (IOMUX_PINCM39)
#define GRAY_S4_IOMUX                                             (IOMUX_PINCM54)
#define GRAY_S5_IOMUX                                             (IOMUX_PINCM55)
#define GRAY_S6_IOMUX                                             (IOMUX_PINCM59)
#define GRAY_S7_IOMUX                                             (IOMUX_PINCM60)

/* Keys: PB9/PB8. */
#define KEY_PORT                                                           GPIOB
#define KEY_1_PIN                                                   DL_GPIO_PIN_9
#define KEY_1_IIDX                                                DL_GPIO_IIDX_DIO9
#define KEY_1_IOMUX                                                (IOMUX_PINCM26)
#define KEY_2_PIN                                                   DL_GPIO_PIN_8
#define KEY_2_IIDX                                                DL_GPIO_IIDX_DIO8
#define KEY_2_IOMUX                                                (IOMUX_PINCM25)

/*
 * Encoder input fallback definitions.
 * 1.1ccs 中 PA12/PA13/PA22 已分配给步进电机，编码器输入默认关闭。
 * 这些宏只用于编码器输入关闭时保持模块可编译，不要按这里接编码器。
 */
#define ENCODER_PORT                                                       GPIOA
#define ENCODER_LEFT_A_PIN                                        DL_GPIO_PIN_12
#define ENCODER_LEFT_A_IIDX                                      DL_GPIO_IIDX_DIO12
#define ENCODER_LEFT_A_IOMUX                                     (IOMUX_PINCM34)
#define ENCODER_LEFT_B_PIN                                        DL_GPIO_PIN_13
#define ENCODER_LEFT_B_IIDX                                      DL_GPIO_IIDX_DIO13
#define ENCODER_LEFT_B_IOMUX                                     (IOMUX_PINCM35)
#define ENCODER_RIGHT_A_PIN                                       DL_GPIO_PIN_22
#define ENCODER_RIGHT_A_IIDX                                     DL_GPIO_IIDX_DIO22
#define ENCODER_RIGHT_A_IOMUX                                    (IOMUX_PINCM47)
#define ENCODER_RIGHT_B_PIN                                       DL_GPIO_PIN_23
#define ENCODER_RIGHT_B_IIDX                                     DL_GPIO_IIDX_DIO23
#define ENCODER_RIGHT_B_IOMUX                                    (IOMUX_PINCM53)

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
bool SYSCFG_DL_SYSCTL_isClockOk(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_LogUart_init(void);
void SYSCFG_DL_JY61P_init(void);
void SYSCFG_DL_Exchange_init(void);
void SYSCFG_DL_GRAY_ADC0_init(void);
void SYSCFG_DL_GRAY_ADC1_init(void);
bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif
