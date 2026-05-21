# light-car1.0 pin map

Primary source: `D:\激光循迹\Netlist_Schematic1_2026-05-16.tel`.

## Communication

| Module | Peripheral | MCU pins | Baud |
| --- | --- | --- | --- |
| OLED | I2C0 | PA0 SDA, PA1 SCL | 100 kHz |
| JY61P | UART0 | PA28 TX, PA31 RX | 115200 |
| JQ8400 | UART1 | PB6 TX, PB7 RX | 115200 |
| Link / Exchange | UART3 | PB2 TX, PB3 RX | 115200 |

## Motor driver

TB6612 uses two PWM channels and four direction GPIOs.

| Signal | MCU pin | Function |
| --- | --- | --- |
| PWMA | PA7 | TIMA0_C1 |
| PWMB | PA8 | TIMA0_C0 |
| AIN1 | PB19 | GPIO output |
| AIN2 | PB18 | GPIO output |
| BIN1 | PB20 | GPIO output |
| BIN2 | PB24 | GPIO output |
| STBY | VCC | Always enabled |

## Gray sensors

| Sensor | MCU pin | ADC |
| --- | --- | --- |
| S1 | PA14 | ADC0 MEM0 |
| S2 | PA15 | ADC1 MEM0 |
| S3 | PA16 | ADC1 MEM1 |
| S4 | PA17 | ADC1 MEM2 |
| S5 | PA27 | ADC0 MEM3 |
| S6 | PA24 | ADC0 MEM1 |
| S7 | PA25 | ADC0 MEM2 |

## Encoders and keys

| Signal | MCU pin |
| --- | --- |
| Left encoder A/B | PA12 / PA13 |
| Right encoder A/B | PA22 / PA23 |
| Key 1 / Key 2 | PB9 / PB8 |
