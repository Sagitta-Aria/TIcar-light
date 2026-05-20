# light-car1.0 源码阅读讲解

这套文档对应整个 `D:\激光循迹\light-car1.0` 工程源码，按适合初学者的阅读顺序来讲。它和 `project_walkthrough` 的区别是：

- `project_walkthrough`：先建立项目整体地图。
- `source_walkthrough`：按文件和模块继续往下读源码。

建议你按这个顺序看：

1. `00_reading_order.md`：先知道整份工程该按什么顺序读。
2. `01_startup_generated_config.md`：启动汇编、TI 生成外设配置、项目配置。
3. `02_system_layer.md`：板级初始化、延时、中断分发。
4. `03_hardware_basic.md`：电机、按键、编码器、Link 串口。
5. `04_hardware_sensors_display_uart.md`：灰度、OLED、JY61P、JQ8400、字库。
6. `05_app_shell_menu_state.md`：main、App 调度、OLED 菜单、状态机。
7. `06_app_motion_tracking_route_speed.md`：运动封装、循迹、异常、路线、速度闭环、电机测试。
8. `07_file_by_file_checklist.md`：整个工程每个源码文件的作用速查。

读的时候不要急着背 C 语法。先问三件事：

```text
这个文件属于哪一层？
它对外提供哪些函数？
它内部保存了哪些状态变量？
```

能回答这三件事，代码就开始变得有形状了。

