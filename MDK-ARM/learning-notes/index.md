# light-car1.0 学习笔记索引

这个目录专门放学习笔记，不放到项目 README 里。

## 推荐阅读路线

1. `source_walkthrough/index.md`
   - 按整个 `D:\激光循迹\light-car1.0` 工程源码逐层讲解。
   - 适合你现在系统阅读全部代码。

2. `project_walkthrough/index.md`
   - 项目整体结构、运行主线、调试现象和 C 语言基础地图。
   - 适合先建立大局观。

3. `gray_adc/gray_adc_walkthrough.md`
   - 灰度 ADC 模块专题。
   - 适合重点理解 `GrayAdcSlot`、ADC0/ADC1、MEM、`g_grayRaw`、阈值、循迹误差。

4. `gray_adc/c_mcu_cheatsheet.md`
   - C 语言和单片机概念速查。

5. `oled_i2c/oled_i2c_walkthrough.md`
   - OLED I2C 专题。
   - 适合理解 `OLED_WR_Byte(dat, mode)`、`txData[2]`、命令/数据控制字节。

## 读源码时先记住

```text
generated/  底层外设配置
config/     引脚别名和调参常量
system/     板级初始化、延时、中断
hardware/   电机、按键、编码器、灰度、OLED、串口模块
app/        菜单、状态机、循迹、速度闭环、路线
```

