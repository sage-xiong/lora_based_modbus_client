# Project Introduction

Based the [Freemodbus Client](https://www.embedded-experts.at/en/freemodbus/about/) and the drivers of SX1278.

# Files Tree

```
examples
modbus  ---->  freemodbus source
    port  ---->  freemodbus source interface
radio  ---->  sx1278 Drivers
    sx1276-Hal.c  ---->  Hardware Interface
    sx1276-Hal.h  ---->  Hardware Interface Config file
tranceive  ----> Lora transceiver
```

# How To Use

## Hardware Interface

You should modify the sx1276-HAL.h and sx1276-HAL.c files accroding to use chips.

I've already modifed accroding to my STM32F103.

When using, you need to implement platform.h and platform.c files by yourself to complete the functions of initializing the chip pins and the SPI module used, capturing the input of specific pins, etc., please refer to the comments in the sx1276-Hal.c file for details.

## Modbus Register 

When using FreeModbus, you need to write callback functions with relevant instructions to achieve direct operation of different function codes. For more detailed information, please refer to the official freemodbus routine.