# 项目说明

使用 freemodbus 与 SX1278 官方提供的驱动，搭建了一个以 LoRa 为物理层，Modbus 为协议层的通信协议。

# 文档结构说明

```
examples  ---->  使用例子
modbus  ---->  freemodbus 源码
    port  ---->  freemodbus 接口文件
radio  ---->  sx1278 驱动
    sx1276-Hal.c  ---->  硬件接口文件
    sx1276-Hal.h  ---->  硬件接口头文件
tranceive  ----> Lora 收发器通信
```

# 使用说明

## 硬件接口

在 sx1276-HAL.h 和 sx1276-Hal.c 文件中，根据自己实际硬件电路所使用的接口。

给出的代码已经针对 stm32 的 HAL 库写了部分代码。

在使用时，需要自己实现 platform.h 和 platform.c 文件，用于完成对芯片管脚以及所使用的 SPI 模块进行初始化、对特定管脚的输入捕捉等功能，具体请参考 sx1276-Hal.c 文件中注释。

## modbus 寄存器设置说明

在使用 freemodbus 时，需要自己编写相关指令的回调函数，实现不同功能码直接的操作。更详细的信息可以参考 freemodbus 官方提供的例程。
