# 外设最小测试 demo

外设独立测试工程放在：

```powershell
D:\Ti\mspm0g3507_peripheral_demos
```

这些 demo 用来判断“是工程整体初始化导致异常，还是某个外设/接线导致异常”。每个 demo 成功后 PA14 闪烁，主动判定失败时 PA14 常亮。构建 demo 不会下载或擦除芯片。

构建全部 demo：

```powershell
& "D:\Ti\mspm0g3507_peripheral_demos\build_all.ps1" -Clean
```

ccs1.2 后需要重点使用：

- `00_gpio_pa14_blink`：只测 GPIOA/PA14。
- `01_clock_sysosc_probe`：只测内部 32MHz SYSOSC。
- `02_oled_i2c_probe`：测 OLED/I2C0，必须带超时。
- `03_uart_all_probe`：测 UART0/UART1/UART3。
- `04_gray_adc_probe`：按 ccs1.2 灰度引脚测 PA15/PA16/PA17/PA24/PA25/PA26/PA27。
- `05_stepper_safe_probe`：四路 STEP/DIR 安全脉冲测试。

下载 demo 前仍建议用各自 `tools\jlink_download_halt.jlink`，下载后先 halt，再手动 reset/run。
