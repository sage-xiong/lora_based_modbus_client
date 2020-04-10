/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) SEMTECH S.A.
 */
/*!
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    1.0.B2 ( PRELIMINARY )
 * \date       Nov 21 2012
 * \author     Miguel Luis
 * \history    2019-09-19 Modify the IO Configration by Sage Xiong.
 */

#include <stdbool.h>

#include "platform.h" //!< TODO - 需要自己完成 platform.h 及 platform.c 文件，用于初始化相应的管脚及模块
#include "radio/sx1276-Hal.h"

/*!
 *  SPI Pins I/O definitions
 */
#define CS_ENABLE          (HAL_GPIO_WritePin(RADIO_CS_GPIO_PORT,RADIO_CS_PIN,GPIO_PIN_RESET))
#define CS_DISABLE         (HAL_GPIO_WritePin(RADIO_CS_GPIO_PORT,RADIO_CS_PIN,GPIO_PIN_SET))

/*!
 * SX1276 Reset
 */
#define RESET_IOPORT            RADIO_RESET_GPIO_PORT
#define RESET_PIN               RADIO_RESET_PIN

uint32_t TickCounter;

/************************************************
函数名称 ： Delay_Ms
功    能 ： 精确的毫秒级延时程序
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void Delay_Ms(__IO uint32_t ms)
{
  HAL_Delay(ms);
}

/************************************************
函数名称 ： SX1276GpioInit
功    能 ： Sx1276所用到的IO口初始化
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276GpioInit(void)
{
  BSP_Radio_Init(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276InitIo  SX127X IO口初始化
功    能 ： NVIC配置
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276InitIo( void )
{
	SX1276GpioInit();  //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276初始化重置
功    能 ： SX1278重启引脚初始化加输出
参    数 ： RESET 输出的电平高低
返 回 值 ： 
作    者 ： sun
*************************************************/
void SX1276SetReset( uint8_t state )
{
  if( state == RADIO_RESET_ON )
  {
    HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin( RESET_IOPORT, RESET_PIN, GPIO_PIN_SET);
  }
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276 SPI写寄存器功能实现
功    能 ： SX1276 SPI写单个寄存器功能实现
参    数 ： addr：寄存器地址  data 待写入值  
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276 SPI读寄存器功能实现
功    能 ： SX1276 SPI读单个寄存器功能实现
参    数 ： addr：寄存器地址  data 待写入值  
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

/************************************************
函数名称 ：
功    能 ： 
参    数 ：
返 回 值 ： 
*************************************************/
uint8_t SPI_WriteReadByte(uint8_t TxData)
{
  return BSP_Radio_WriteReadByte(TxData); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276 SPI写寄存器功能实现
功    能 ： SX1276 SPI写寄存器功能实现
参    数 ： addr：寄存器地址  buffer 待写入值    size ：写入数据的长度
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t i;

  CS_ENABLE;

  SPI_WriteReadByte( addr | 0x80 );
  for( i = 0; i < size; i++ )
  {
      SPI_WriteReadByte( buffer[i] );
  }

  CS_DISABLE;
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276 SPI读寄存器功能实现
功    能 ： SX1276 SPI读寄存器功能实现
参    数 ： addr：所读寄存器地址  buffer 返回值    size ：读取数据长度
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
  uint8_t i;
  
  CS_ENABLE;

  SPI_WriteReadByte( addr & 0x7F );

  for( i = 0; i < size; i++ )
  {
      buffer[i] = SPI_WriteReadByte( 0 );
  }

  CS_DISABLE;
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio0
功    能 ： SX1276 IO_0引脚电平读取
参    数 ： 
返 回 值 ：IO_0引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio0( void )
{
    return BSP_Radio_Read_DIO0(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio1
功    能 ： SX1276 IO_1引脚电平读取
参    数 ： 
返 回 值 ：IO_1引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio1( void )
{
    return BSP_Radio_Read_DIO1(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio2
功    能 ： SX1276 IO_2引脚电平读取
参    数 ： 
返 回 值 ：IO_2引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio2( void )
{
    return BSP_Radio_Read_DIO2(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio3
功    能 ： SX1276 IO_3引脚电平读取
参    数 ： 
返 回 值 ：IO_3引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio3( void )
{
    return BSP_Radio_Read_DIO3(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio4
功    能 ： SX1276 IO_4引脚电平读取
参    数 ： 
返 回 值 ：IO_4引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio4( void )
{
    return BSP_Radio_Read_DIO4(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，SX1276ReadDio5
功    能 ： SX1276 IO_5引脚电平读取
参    数 ： 
返 回 值 ：IO_5引脚电平状态
作    者 ： sun
*************************************************/
uint8_t SX1276ReadDio5( void )
{
    return BSP_Radio_Read_DIO5(); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/************************************************
函数名称 ： SX127x硬件层函数，射频芯片收发切换
功    能 ： 射频芯片收发切换
参    数 ： txEnable 1发送  0接收
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276WriteRxTx( uint8_t txEnable )
{
  BSP_Radio_Write_RxTx(txEnable); //!< TODO - 在 platform.h 中声明，需要自己在 platform.c 中实现
}

/* End of File */
