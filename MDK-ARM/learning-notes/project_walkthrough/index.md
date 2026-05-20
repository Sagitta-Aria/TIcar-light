# light-car1.0 代码阅读索引

这组文档是给 C 语言基础还不稳、同时又要看懂 TI MSPM0 小车工程的人准备的。它不放在 README 里，因为 README 更适合做项目介绍；这里更像你的“随工程更新的课堂笔记”。

如果你要按整个工程逐文件阅读源码，请先看 `../source_walkthrough/index.md`。`project_walkthrough` 更偏项目总览和关键知识点，`source_walkthrough` 更偏完整源码阅读。

建议阅读顺序：

1. `00_reading_guide.md`：先建立整车代码地图，知道从哪里进、每层负责什么。
2. `04_c_language_map.md`：把项目里反复出现的 C 语法先扫一遍，尤其是结构体、指针、数组、`0U`、`static`、`volatile`。
3. `01_startup_and_system.md`：看上电之后程序怎么初始化外设、怎么进主循环、怎么处理中断。
4. `02_hardware_modules.md`：看电机、按键、编码器、灰度、OLED、串口模块各自做什么。
5. `03_app_logic.md`：看菜单、状态机、循迹、速度闭环、路线外环这些应用逻辑怎么连起来。
6. `05_debug_notes.md`：把你之前问过的 BSL、RESET、OLED、按键、下载失败这些现象和代码逻辑对应起来。

已经有的专题笔记也建议一起看：

- `../gray_adc/gray_adc_walkthrough.md`：灰度 ADC 的详细讲解。
- `../gray_adc/c_mcu_cheatsheet.md`：灰度代码相关 C/单片机速查。
- `../oled_i2c/oled_i2c_walkthrough.md`：OLED I2C、命令/数据模式讲解。

阅读时先抓主线：

```text
main.c
  -> Board_Init()      硬件初始化
  -> App_Init()        应用模块初始化
  -> while (1)
       -> App_Task()   一直循环跑任务
```

单片机程序不像电脑上的普通程序那样“运行完就退出”。它通常是：上电、初始化、进入死循环，在死循环里不断采传感器、处理按键、刷新 OLED、控制电机。
