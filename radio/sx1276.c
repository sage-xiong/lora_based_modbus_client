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
 * \file       sx1276.c
 * \brief      SX1276 RF chip driver
 *
 * \version    2.0.B2
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include "radio\sx1276.h"
#include "radio\sx1276-Hal.h"
#include "radio\sx1276-LoRa.h"

/*!
 * SX1276 registers variable
 */
uint8_t SX1276Regs[0x70];

static bool LoRaOn      = false;
static bool LoRaOnState = false;
/************************************************
函数名称 ： SX127x应用层函数，sx1276重启
功    能 ： sx1276重启
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
uint8_t SX1276Init( void )
{
	uint8_t TempReg;
	if(false == SX1276LoraConfigCheck())  //判断参数是否可用
	{
			return 0;
	}
	SX1276LR = ( tSX1276LR* )SX1276Regs;  //LoRa寄存器指向指针，有利于调用
	SX1276InitIo( );                      //LoRa引脚初始化
	SX1276Reset( );                       //复位芯片
	SX1276Read(0x06,&TempReg);          	//SPI测试
	if(TempReg != 0x6C)                   //判断读取的数据是否正确
	{
		return 0;
	}
	LoRaOn = true;                        //设置LoRa模式标志
	SX1276SetLoRaOn( LoRaOn );
	// Initialize LoRa modem
	SX1276LoRaInit( );                    //LoRa参数配置
  
	return 1;
}


/************************************************
函数名称 ： SX127x应用层函数，sx1276重启
功    能 ： sx1276重启
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276Reset( void )
{
  SX1276SetReset( RADIO_RESET_ON );
  Delay_Ms(5);
  SX1276SetReset( RADIO_RESET_OFF );
  Delay_Ms(1);
}



/************************************************
函数名称 ： SX127x应用层函数，sx127x模式选择，开启
功    能 ： sx1276初始化配置
参    数 ： LoRa工作模式标志
返 回 值 ： 无
作    者 ： sun
*************************************************/

void SX1276SetLoRaOn( bool enable )
{
    if( LoRaOnState == enable )
    {
        return;
    }
    LoRaOnState = enable;
    LoRaOn = enable;
    
    SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
    
    SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
    SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );

    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		
#if defined USE_DIO 
                                                                    // RxDone               RxTimeout                   FhssChangeChannel           CadDone
    SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                                                    // CadDetected          ModeReady
    SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
#endif
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
    
    SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );
}

bool SX1276GetLoRaOn( void )
{
    return LoRaOn;
}

void SX1276SetOpMode( uint8_t opMode )
{
    SX1276LoRaSetOpMode( opMode );
}

uint8_t SX1276GetOpMode( void )
{
    return SX1276LoRaGetOpMode( );
}

double SX1276ReadRssi( void )
{
    return SX1276LoRaReadRssi( );
}

uint8_t SX1276ReadRxGain( void )
{
    return SX1276LoRaReadRxGain( );
}

uint8_t SX1276GetPacketRxGain( void )
{
    return SX1276LoRaGetPacketRxGain(  );
}

int8_t SX1276GetPacketSnr( void )
{
     return SX1276LoRaGetPacketSnr(  );
}

double SX1276GetPacketRssi( void )
{
    return SX1276LoRaGetPacketRssi( );
}

void SX1276StartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1276StartCad( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_CAD_INIT );
}

void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
    SX1276LoRaGetRxPacket( buffer, size );
}

void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
   SX1276LoRaSetTxPacket( buffer, size );
}

uint8_t SX1276GetRFState( void )
{
   return SX1276LoRaGetRFState( );
}

void SX1276SetRFState( uint8_t state )
{
   SX1276LoRaSetRFState( state );
}

uint32_t SX1276Process( void )
{
    return SX1276LoRaProcess( );
}

/* End of File */
