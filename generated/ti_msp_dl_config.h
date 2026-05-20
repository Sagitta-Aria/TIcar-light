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
#define CPUCLK_FREQ                                                     80000000

#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)

/* PWM: PA8 = TIMA0_C0, PA7 = TIMA0_C1. */
#define PWM_INST                                                           TIMA0
#define PWM_INST_IRQHandler                                     TIMA0_IRQHandler
#define PWM_INST_INT_IRQN                                       (TIMA0_INT_IRQn)
#define PWM_INST_CLK_FREQ                                               80000000
#define PWM_PERIOD_COUNTS                                                 (4000U)
#define GPIO_PWM_C0_PORT                                                   GPIOA
#define GPIO_PWM_C0_PIN                                            DL_GPIO_PIN_8
#define GPIO_PWM_C0_IOMUX                                        (IOMUX_PINCM19)
#define GPIO_PWM_C0_IOMUX_FUNC                       IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_C0_IDX                                      DL_TIMER_CC_0_INDEX
#define GPIO_PWM_C1_PORT                                                   GPIOA
#define GPIO_PWM_C1_PIN                                            DL_GPIO_PIN_7
#define GPIO_PWM_C1_IOMUX                                        (IOMUX_PINCM14)
#define GPIO_PWM_C1_IOMUX_FUNC                       IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_PWM_C1_IDX                                      DL_TIMER_CC_1_INDEX

/* OLED I2C0: PA0 SDA, PA1 SCL. */
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

/* JY61P UART0: PA28 TX, PA31 RX. */
#define JY61P_INST                                                         UART0
#define JY61P_INST_FREQUENCY                                            40000000
#define JY61P_INST_IRQHandler                                   UART0_IRQHandler
#define JY61P_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_JY61P_RX_PORT                                                 GPIOA
#define GPIO_JY61P_TX_PORT                                                 GPIOA
#define GPIO_JY61P_RX_PIN                                         DL_GPIO_PIN_31
#define GPIO_JY61P_TX_PIN                                         DL_GPIO_PIN_28
#define GPIO_JY61P_IOMUX_RX                                       (IOMUX_PINCM6)
#define GPIO_JY61P_IOMUX_TX                                       (IOMUX_PINCM3)
#define GPIO_JY61P_IOMUX_RX_FUNC                        IOMUX_PINCM6_PF_UART0_RX
#define GPIO_JY61P_IOMUX_TX_FUNC                        IOMUX_PINCM3_PF_UART0_TX
#define JY61P_BAUD_RATE                                                 (115200)
#define JY61P_IBRD_40_MHZ_115200_BAUD                                       (21)
#define JY61P_FBRD_40_MHZ_115200_BAUD                                       (45)

/* JQ8400 UART1: PB6 TX, PB7 RX. */
#define JQ8400_INST                                                        UART1
#define JQ8400_INST_FREQUENCY                                           40000000
#define JQ8400_INST_IRQHandler                                  UART1_IRQHandler
#define JQ8400_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_JQ8400_RX_PORT                                                GPIOB
#define GPIO_JQ8400_TX_PORT                                                GPIOB
#define GPIO_JQ8400_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_JQ8400_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_JQ8400_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_JQ8400_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_JQ8400_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_JQ8400_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define JQ8400_BAUD_RATE                                                (115200)
#define JQ8400_IBRD_40_MHZ_115200_BAUD                                      (21)
#define JQ8400_FBRD_40_MHZ_115200_BAUD                                      (45)

/* Exchange UART3: PB2 TX, PB3 RX. */
#define Exchange_INST                                                      UART3
#define Exchange_INST_FREQUENCY                                         80000000
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
#define Exchange_IBRD_80_MHZ_115200_BAUD                                    (43)
#define Exchange_FBRD_80_MHZ_115200_BAUD                                    (26)

/* Gray sensors: ADC0/ADC1 sequence sampling. */
#define GRAY_ADC0_INST                                                       ADC0
#define GRAY_ADC1_INST                                                       ADC1
#define GRAY_ADC0_MEM_GRAY1                                      DL_ADC12_MEM_IDX_0
#define GRAY_ADC0_MEM_GRAY6                                      DL_ADC12_MEM_IDX_1
#define GRAY_ADC0_MEM_GRAY7                                      DL_ADC12_MEM_IDX_2
#define GRAY_ADC1_MEM_GRAY2                                      DL_ADC12_MEM_IDX_0
#define GRAY_ADC1_MEM_GRAY3                                      DL_ADC12_MEM_IDX_1
#define GRAY_ADC1_MEM_GRAY4                                      DL_ADC12_MEM_IDX_2
#define GRAY_ADC1_MEM_GRAY5                                      DL_ADC12_MEM_IDX_3
#define GRAY_S1_IOMUX                                             (IOMUX_PINCM36)
#define GRAY_S2_IOMUX                                             (IOMUX_PINCM37)
#define GRAY_S3_IOMUX                                             (IOMUX_PINCM38)
#define GRAY_S4_IOMUX                                             (IOMUX_PINCM39)
#define GRAY_S5_IOMUX                                             (IOMUX_PINCM40)
#define GRAY_S6_IOMUX                                             (IOMUX_PINCM54)
#define GRAY_S7_IOMUX                                             (IOMUX_PINCM55)

/* TB6612 direction pins: PB18/PB19 for A, PB20/PB24 for B. */
#define MOTOR_DIR_PORT                                                     GPIOB
#define MOTOR_A_IN1_PIN                                            DL_GPIO_PIN_19
#define MOTOR_A_IN1_IOMUX                                         (IOMUX_PINCM45)
#define MOTOR_A_IN2_PIN                                            DL_GPIO_PIN_18
#define MOTOR_A_IN2_IOMUX                                         (IOMUX_PINCM44)
#define MOTOR_B_IN1_PIN                                            DL_GPIO_PIN_20
#define MOTOR_B_IN1_IOMUX                                         (IOMUX_PINCM48)
#define MOTOR_B_IN2_PIN                                            DL_GPIO_PIN_24
#define MOTOR_B_IN2_IOMUX                                         (IOMUX_PINCM52)

/* Keys: PB9/PB8. */
#define KEY_PORT                                                           GPIOB
#define KEY_1_PIN                                                   DL_GPIO_PIN_9
#define KEY_1_IIDX                                                DL_GPIO_IIDX_DIO9
#define KEY_1_IOMUX                                                (IOMUX_PINCM26)
#define KEY_2_PIN                                                   DL_GPIO_PIN_8
#define KEY_2_IIDX                                                DL_GPIO_IIDX_DIO8
#define KEY_2_IOMUX                                                (IOMUX_PINCM25)

/* Encoders: left PA12/PA13, right PA22/PA23. */
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
void SYSCFG_DL_PWM_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_JY61P_init(void);
void SYSCFG_DL_JQ8400_init(void);
void SYSCFG_DL_Exchange_init(void);
void SYSCFG_DL_GRAY_ADC0_init(void);
void SYSCFG_DL_GRAY_ADC1_init(void);
bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif
