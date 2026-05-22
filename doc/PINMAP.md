# light-car1.1ccs Pin Map

Primary source: 地猛星 MSPM0G3507 最小系统板 H3/H5 排针和 `doc/PIN_ASSIGNMENT.md`。

## Communication

| Module | Peripheral | MCU pins | Baud/Speed |
| --- | --- | --- | --- |
| OLED | I2C0 | PA0 SDA, PA1 SCL | 100 kHz |
| JY61P | UART0 | PA28 TX, PA31 RX | 115200 |
| JQ8400 | UART1 | PB6 TX, PB7 RX | 115200 |
| Link / feedback | UART3 | PB2 TX, PB3 RX | 115200 |

## Stepper Drivers

| Motor | STEP | DIR | Notes |
| --- | --- | --- | --- |
| Chassis left | PA7 | PB18 | Closed-loop stepper driver, no EN |
| Chassis right | PA8 | PA9 | Closed-loop stepper driver, no EN |
| Gimbal 1 | PA12 | PA22 | Closed-loop stepper driver, no EN |
| Gimbal 2 | PA13 | PB24 | Closed-loop stepper driver, no EN |

## Gray Sensors

| Sensor | MCU pin | ADC |
| --- | --- | --- |
| S1 | PA15 | ADC1 MEM0 |
| S2 | PA16 | ADC1 MEM1 |
| S3 | PA17 | ADC1 MEM2 |
| S4 | PA24 | ADC0 MEM0 |
| S5 | PA25 | ADC0 MEM1 |
| S6 | PA26 | ADC0 MEM2 |
| S7 | PA27 | ADC0 MEM3 |

## Keys And Reserved Pins

| Signal | MCU pin |
| --- | --- |
| Key 1 / Key 2 | PB9 / PB8 |
| Status LED | PA14 |
| SWDIO / SWCLK | PA19 / PA20 |
| VREF reserve | PA21 / PA23 |
| BSL reserve | PA10 / PA11 / PA18 |
| SPI Flash reserve | PB14 / PB15 / PB16 / PB17 |
