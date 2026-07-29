#ifndef RESOURCE_CONFIG_H
#define RESOURCE_CONFIG_H

#include "bluetooth_config.h"
#include "board_config.h"
#include "pin_map.h"
#include "ti_msp_dl_config.h"

/* 电脑调试口在两个Profile中都固定使用UART0 PA10/PA11。 */
#define CAR_DEBUG_UART_INST             LogUart_INST
#define CAR_DEBUG_UART_INST_INT_IRQN    LogUart_INST_INT_IRQN
#define CAR_DEBUG_UART_BAUD_RATE        LogUart_BAUD_RATE
#define CAR_DEBUG_UART_TX_PORT          GPIO_LogUart_TX_PORT
#define CAR_DEBUG_UART_TX_PIN           GPIO_LogUart_TX_PIN
#define CAR_DEBUG_UART_RX_PORT          GPIO_LogUart_RX_PORT
#define CAR_DEBUG_UART_RX_PIN           GPIO_LogUart_RX_PIN

/* HC-05 transport follows the selected main-board pin profile. */
#if CAR_LIBRARY_BOARD_IS_TIANMENG
#define CAR_BLUETOOTH_UART_INST          UART2
#define CAR_BLUETOOTH_UART_INST_INT_IRQN UART2_INT_IRQn
#define CAR_BLUETOOTH_UART_IS_UART2       (1U)
#define CAR_BLUETOOTH_UART_IS_UART3       (0U)
#else
#define CAR_BLUETOOTH_UART_INST          UART3
#define CAR_BLUETOOTH_UART_INST_INT_IRQN UART3_INT_IRQn
#define CAR_BLUETOOTH_UART_IS_UART2       (0U)
#define CAR_BLUETOOTH_UART_IS_UART3       (1U)
#endif
#define CAR_BLUETOOTH_UART_FREQUENCY     (32000000U)
#define CAR_BLUETOOTH_UART_BAUD_RATE     CAR_BLUETOOTH_DATA_BAUD_RATE

#define CAR_BLUETOOTH_USES_UART3 \
    (CAR_BLUETOOTH_ENABLED && CAR_BLUETOOTH_UART_IS_UART3)

#if (CAR_BLUETOOTH_USES_UART3 && CAR_PROFILE_IS_FULL)
#error "Full profile reserves UART3 PB2/PB3 for K230; Dimeng Bluetooth is unavailable"
#endif

#if (CAR_BLUETOOTH_ENABLED && !CAR_BLUETOOTH_TRANSPORT_CONFIGURED)
#error "Bluetooth role selected without a configured UART transport"
#endif

/* 地猛星蓝牙启用时，GMR把UART3从外部M0姿态切换给HC-05。 */
#define CAR_M0_ATTITUDE_UART_REQUIRED \
    (CAR_PROFILE_IS_GMR && !CAR_BLUETOOTH_USES_UART3)
#define CAR_M0_ATTITUDE_UART_INST       Exchange_INST
#define CAR_M0_ATTITUDE_UART_INST_INT_IRQN Exchange_INST_INT_IRQN
#define CAR_M0_ATTITUDE_UART_BAUD_RATE  Exchange_BAUD_RATE
#define CAR_M0_ATTITUDE_UART_TX_PORT    GPIO_Exchange_TX_PORT
#define CAR_M0_ATTITUDE_UART_TX_PIN     GPIO_Exchange_TX_PIN
#define CAR_M0_ATTITUDE_UART_RX_PORT    GPIO_Exchange_RX_PORT
#define CAR_M0_ATTITUDE_UART_RX_PIN     GPIO_Exchange_RX_PIN
#define CAR_M0_ATTITUDE_UART_RX_IOMUX   GPIO_Exchange_IOMUX_RX
#define CAR_M0_ATTITUDE_UART_RX_IOMUX_FUNC GPIO_Exchange_IOMUX_RX_FUNC

/* H7 uses independent UART2 on Tianmeng GMR and UART0 on Full. */
#if CAR_PROFILE_IS_GMR
#if !CAR_LIBRARY_BOARD_IS_TIANMENG
#error "Competition GMR profile requires the Tianmeng board"
#endif
#define CAR_H7_UART_REQUIRED \
    (CAR_LIBRARY_H7_IMU_ENABLED || CAR_LIBRARY_H7_LCD_ENABLED)
#define CAR_H7_UART_INST                UART2
#define CAR_H7_UART_INST_INT_IRQN       UART2_INT_IRQn
#define CAR_H7_UART_BAUD_RATE           (115200U)
#define CAR_H7_UART_FREQUENCY           (32000000U)
#define CAR_H7_UART_TX_PORT             PIN_H7_CONTROL_UART_TX_PORT
#define CAR_H7_UART_TX_PIN              PIN_H7_CONTROL_UART_TX
#define CAR_H7_UART_RX_PORT             PIN_H7_CONTROL_UART_RX_PORT
#define CAR_H7_UART_RX_PIN              PIN_H7_CONTROL_UART_RX
#else
#define CAR_H7_UART_REQUIRED \
    (CAR_LIBRARY_H7_IMU_ENABLED || CAR_LIBRARY_H7_LCD_ENABLED)
#define CAR_H7_UART_INST                H7GyroLink_INST
#define CAR_H7_UART_INST_INT_IRQN       H7GyroLink_INST_INT_IRQN
#define CAR_H7_UART_BAUD_RATE           H7GyroLink_BAUD_RATE
#define CAR_H7_UART_TX_PORT             GPIO_LogUart_TX_PORT
#define CAR_H7_UART_TX_PIN              GPIO_LogUart_TX_PIN
#define CAR_H7_UART_RX_PORT             GPIO_LogUart_RX_PORT
#define CAR_H7_UART_RX_PIN              GPIO_LogUart_RX_PIN
#endif

#if (CAR_BLUETOOTH_USES_UART3 && CAR_H7_UART_REQUIRED)
#error "Dimeng Bluetooth and H7 cannot share UART3 PB2/PB3"
#endif

#if (CAR_M0_ATTITUDE_UART_BAUD_RATE != 115200)
#error "M0 attitude UART must use 115200 baud"
#endif

#if (CAR_H7_UART_BAUD_RATE != 115200)
#error "H7 UART must use 115200 baud"
#endif

#endif
