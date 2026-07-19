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

/* 云台两轴 STEP 调度：TIMG6 每 50us 调度一次，不再承载灰度采样。 */
#define STEPPER_TIMER_INST                                                (TIMG6)
#define STEPPER_TIMER_INST_IRQHandler                            TIMG6_IRQHandler
#define STEPPER_TIMER_INST_INT_IRQN                              (TIMG6_INT_IRQn)
#define STEPPER_TIMER_CLOCK_HZ                                       (32000000U)
#define STEPPER_TIMER_TICK_HZ                                           (20000U)
#define STEPPER_TIMER_LOAD_VALUE \
    ((STEPPER_TIMER_CLOCK_HZ / STEPPER_TIMER_TICK_HZ) - 1U)

/* 数字灰度快采样：TIMG0 每 100us 采样一次，仅在 Task1/Task4 循迹时启动。 */
#define GRAY_SAMPLE_TIMER_INST                                            (TIMG0)
#define GRAY_SAMPLE_TIMER_INST_IRQHandler                        TIMG0_IRQHandler
#define GRAY_SAMPLE_TIMER_INST_INT_IRQN                          (TIMG0_INT_IRQn)
#define GRAY_SAMPLE_TIMER_CLOCK_HZ                                   (32000000U)
#define GRAY_SAMPLE_TIMER_TICK_HZ                                       (10000U)
#define GRAY_SAMPLE_TIMER_LOAD_VALUE \
    ((GRAY_SAMPLE_TIMER_CLOCK_HZ / GRAY_SAMPLE_TIMER_TICK_HZ) - 1U)

#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)

/* 云台两轴继续使用 STEP/DIR，底盘已经改为 PWM 编码电机。 */
#define STEPPER_GIMBAL_1_STEP_PORT                                         GPIOA
#define STEPPER_GIMBAL_1_STEP_PIN                                  DL_GPIO_PIN_7
#define STEPPER_GIMBAL_1_STEP_IOMUX                               (IOMUX_PINCM14)
#define STEPPER_GIMBAL_1_DIR_PORT                                          GPIOB
#define STEPPER_GIMBAL_1_DIR_PIN                                  DL_GPIO_PIN_18
#define STEPPER_GIMBAL_1_DIR_IOMUX                                (IOMUX_PINCM44)

#define STEPPER_GIMBAL_2_STEP_PORT                                         GPIOA
#define STEPPER_GIMBAL_2_STEP_PIN                                  DL_GPIO_PIN_8
#define STEPPER_GIMBAL_2_STEP_IOMUX                               (IOMUX_PINCM19)
#define STEPPER_GIMBAL_2_DIR_PORT                                          GPIOA
#define STEPPER_GIMBAL_2_DIR_PIN                                   DL_GPIO_PIN_9
#define STEPPER_GIMBAL_2_DIR_IOMUX                                (IOMUX_PINCM20)

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

/* UART0: PA10保留日志TX，PA11由H7 BMI088姿态链路独占接收。 */
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

/* H7姿态反馈与日志TX共用UART0；H7只连接PA11 RX。 */
#define H7GyroLink_INST                                                    UART0
#define H7GyroLink_INST_FREQUENCY                                       32000000
#define H7GyroLink_INST_IRQHandler                              UART0_IRQHandler
#define H7GyroLink_INST_INT_IRQN                                  UART0_INT_IRQn
#define H7GyroLink_BAUD_RATE                                           (115200)

/* 板载JY61P作为底座前馈：UART1 PB6 TX、PB7 RX。 */
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

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
bool SYSCFG_DL_SYSCTL_isClockOk(void);
void SYSCFG_DL_STEPPER_TIMER_init(void);
void SYSCFG_DL_GRAY_SAMPLE_TIMER_init(void);
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
