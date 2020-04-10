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
 * \file       sx1276-LoRa.c
 * \brief      SX1276 RF chip driver mode LoRa
 *
 * \version    2.0.B2
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"

#include "sx1276-Hal.h"
#include "sx1276.h"

#include "sx1276-LoRaMisc.h"
#include "sx1276-LoRa.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0

#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0


/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

const double RssiOffsetHF[] =
{   // These values need to be specify in the Lab
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
{
    433920000,        // RFFrequency
    20,               // Power
    8,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    12,               // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             // CrcOn [0: OFF, 1: ON]
    false,            // ImplicitHeaderOn [0: OFF, 1: ON]//0为显示报头模式，1为隐式，显示报头模式下可以读取任意长度数据，数据长度包含在虚头中，隐式只能读取指定长度，且收发必须一致
											//在负载长度以知道的情况下调用隐式可缩短发送时间
    1,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    128,              // PayloadLength (used for implicit header mode)
};

/*!
 * SX1276 LoRa registers variable
 */
tSX1276LR* SX1276LR;

/*!
 * Local RF buffer for communication support
 */
//static uint8_t RFBuffer[RF_BUFFER_SIZE];
uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
static uint32_t TxTimeoutTimer = 0;
/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

/************************************************
函数名称 ： SX1276LoRaInit
功    能 ： Lora初始化
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaInit( void )
{
    RFLRState = RFLR_STATE_IDLE;
    SX1276LoRaSetDefaults( );                                     //读取芯片信息
    SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );  //读取寄存器里的全部数据
    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;                          //增益设置，最大增益

    SX1276WriteBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 ); //写入寄存器

    // set the RF settings
    SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency );         //设置工作频率
    SX1276LoRaSetPa20dBm( true );                                 //开启最大发射功率
    SX1276LoRaSetRFPower( LoRaSettings.Power );                   //设置发射功率
    SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // 设置扩频因子
    SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding );         //设置前向纠错码
    SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn );               //设置是否开启校验
    SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw );        //设置带宽
 
    SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );//序头模式设置
    SX1276LoRaSetSymbTimeout( 0x3FF );                             //设置接收超时时间
    SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength );      //设置负载长度
    SX1276LoRaSetLowDatarateOptimize( true );

    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );                   //工作模式为待机模式
}


/************************************************
函数名称 ： SX1276LoraConfigCheck
功    能 ： Lora 初始化信息检查，防止配置信息出错
参    数 ： 无
返 回 值 ： bool 成功为1  失败为0
作    者 ： sun
*************************************************/

bool SX1276LoraConfigCheck(void)
{
		if((LoRaSettings.RFFrequency <137000000)||(LoRaSettings.RFFrequency >525000000))
        return false;
		if(LoRaSettings.SignalBw > 9)
			return false;
		if((LoRaSettings.SpreadingFactor < 6)||(LoRaSettings.SpreadingFactor >12))
        return false;
		if((LoRaSettings.ErrorCoding < 1)||(LoRaSettings.ErrorCoding >4))
        return false;	
		if(LoRaSettings.PayloadLength  > 128)
			return false;
		return true ;
}


/************************************************
函数名称 ： SX1276LoRaSetDefaults
功    能 ： 读取芯片版本号
参    数 ： 无
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetDefaults( void )
{
    // REMARK: See SX1276 datasheet for modified default values.

    SX1276Read( REG_LR_VERSION, &SX1276LR->RegVersion );
}

/************************************************
函数名称 ： SX1276LoRaSetOpMode
功    能 ： 设置Lora工作模式
参    数 ： opMode  运行模式
						
返 回 值 ： 无
作    者 ： sun
*************************************************/

void SX1276LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            RXTX( antennaSwitchTxOn ); // Antenna switch control
        }
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

		SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
    }
}


/************************************************
函数名称 ： SX1276LoRaGetOpMode
功    能 ： 读取当前Lora工作模式
参    数 ： 无					
返 回 值 ： 当前工作模式
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetOpMode( void )
{
    SX1276Read( REG_LR_OPMODE, &SX1276LR->RegOpMode );

    return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
}
/************************************************
函数名称 ： SX1276LoRaReadRxGain
功    能 ： 读取当前Lora接收增益大小
参    数 ： 无					
返 回 值 ： 当前Lora增益大小
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaReadRxGain( void )
{
    SX1276Read( REG_LR_LNA, &SX1276LR->RegLna );
    return( SX1276LR->RegLna >> 5 ) & 0x07;
}

/************************************************
函数名称 ： SX1276LoRaReadRssi
功    能 ： 读取计算Lora信号强度,环境中信号强度，可用于检测信道占用情况
参    数 ： 无
						
返 回 值 ： 返回值 为信号强度
作    者 ： sun
*************************************************/
double SX1276LoRaReadRssi( void )
{
    // Reads the RSSI value
    SX1276Read( REG_LR_RSSIVALUE, &SX1276LR->RegRssiValue );

    if( LoRaSettings.RFFrequency < 860000000 )  // LF
    {
        return RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegRssiValue;
    }
    else
    {
        return RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegRssiValue;
    }
}
/************************************************
函数名称 ： SX1276LoRaGetPacketSnr
功    能 ： 获取Lora信噪比
参    数 ： 无
						
返 回 值 ： 返回值 信噪比
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1276LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}
/************************************************
函数名称 ： SX1276LoRaGetPacketRssi
功    能 ： 读取最后接收数据包的信号强度
参    数 ： 无
返 回 值 ： 返回值 为信号强度
作    者 ： sun
*************************************************/
double SX1276LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}
/************************************************
函数名称 ： SX1276LoRaStartRx
功    能 ： Lora开始接收
参    数 ： 无
						
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaStartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}
/************************************************
函数名称 ： SX1276LoRaGetRxPacket
功    能 ： Lora获取接收数据
参    数 ： buffer 接收缓存器
						接收到的数据长度
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}
/************************************************
函数名称 ： SX1276LoRaSetTxPacket
功    能 ： Lora开始发送
参    数 ： buffer 发送缓存器
						size   发送的数据长度
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaSettings.FreqHopOn == false )
    {
        TxPacketSize = size;
    }
    else
    {
        TxPacketSize = 255;
    }
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize );

    RFLRState = RFLR_STATE_TX_INIT;
}

/************************************************
函数名称 ： SX1276LoRaGetRFState
功    能 ： 读取工作状态
参    数 ： 无
						
返 回 值 ： 工作状态
						RFLR_STATE_IDLE,
						RFLR_STATE_RX_INIT,
						RFLR_STATE_RX_RUNNING,
						RFLR_STATE_RX_DONE,
						RFLR_STATE_RX_TIMEOUT,
						RFLR_STATE_TX_INIT,
						RFLR_STATE_TX_RUNNING,
						RFLR_STATE_TX_DONE,
						RFLR_STATE_TX_TIMEOUT,
						RFLR_STATE_CAD_INIT,
						RFLR_STATE_CAD_RUNNING
作    者 ： sun
*************************************************/

uint8_t SX1276LoRaGetRFState( void )
{
    return RFLRState;
}

/************************************************
函数名称 ： SX1276LoRaSetRFState
功    能 ： 设置工作状态
参    数 ： 工作状态
						
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}


/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY,
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
  /************************************************
函数名称 ： SX1276LoRaProcess
功    能 ： Lora运行
参    数 ： 无					
返 回 值 ： Lora的工作模式
						RF_IDLE,
						RF_BUSY,
						RF_RX_DONE,
						RF_RX_TIMEOUT,
						RF_RX_ID_ERROR,
						RF_TX_DONE,
						RF_TX_TIMEOUT,
						RF_LEN_ERROR,
						RF_CHANNEL_EMPTY,
						RF_CHANNEL_ACTIVITY_DETECTED
作    者 ： sun
*************************************************/
uint32_t SX1276LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |           //设置中断标志位
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                   // RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
            SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1276LR->RegHopPeriod = 255;
        }
        
        SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
                
                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
		
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
            SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
            
            SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );  //清空RFBuffer

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT();
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
				SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
				if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_VALIDHEADER_MASK ) 
				{
					SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER );						
				}
        SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
				if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_RXDONE_MASK )
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )    //false
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
				if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK )// FHSS Changed Channel
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1276LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )   //计算是否超时
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
        if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )     //CRC校验
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
				}
				
				// If CRC off, then give up the packet.
				// This means it receives wrong header.
				SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
        if( !( SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_PAYLOAD_CRC16_ON ) )
        {
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }
        
        {
            uint8_t rxSnrEstimate;
            SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }
  /**********************************官网源代码
        if( LoRaSettings.RFFrequency < 860000000 )  // LF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
            }
            else
            {    
                SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
                RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
            }
        }
        else                                        // HF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
            }
            else
            {    
                SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
                RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
            }
        }
**********************************************/
/***************************重新计算值************************/
				if( LoRaSettings.RFFrequency < 860000000 )  // LF
				{
							SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
							RxPacketRssiValue = -161.0 + ( double )SX1276LR->RegPktRssiValue;
							if( RxPacketSnrEstimate < 0 && RxPacketRssiValue < -117 )
							{
									RxPacketRssiValue = -119 + RxPacketSnrEstimate;
							}
				}
				else                                        // HF
				{
							SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
							RxPacketRssiValue = -161.0 + ( double )SX1276LR->RegPktRssiValue;
							if( RxPacketSnrEstimate < 0 && RxPacketRssiValue < -117 )
							{
									RxPacketRssiValue = -119 + RxPacketSnrEstimate;
							}
				}

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;       
            SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );    //读取接收数据

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1276LR->RegPayloadLength;
                SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
            }
            else
            {
                SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
                RxPacketSize = SX1276LR->RegNbRxBytes;
                SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1276LR->RegPayloadLength;
                SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
                SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
            }
            else
            {
                SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
                RxPacketSize = SX1276LR->RegNbRxBytes;
                SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
                SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
                SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
            SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1276LR->RegHopPeriod = 0;
        }
        SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1276LR->RegPayloadLength = TxPacketSize;
        SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
        
        SX1276LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );

        SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
        SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
        
        // Write payload buffer to LORA modem
        SX1276WriteFifo( RFBuffer, SX1276LR->RegPayloadLength );
		
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
		
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

        SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

				PacketTimeout = LoRaSettings.TxPacketTimeout;
        TxTimeoutTimer = GET_TICK_COUNT( );


        RFLRState = RFLR_STATE_TX_RUNNING;
				
        break;
    case RFLR_STATE_TX_RUNNING:
		{
			SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
			
			if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_TXDONE_MASK )
      {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;   
      }
			else
			{
				//	if( ( GET_TICK_COUNT( ) - TxTimeoutTimer ) > PacketTimeout )   //发送超时
				if( (TxTimeoutTimer + PacketTimeout ) < GET_TICK_COUNT( ))   //发送超时
					{
							 result = RF_TX_TIMEOUT;
							 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
							 RFLRState = RFLR_STATE_IDLE;
					}
			
			}
      SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
			if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK )// FHSS Changed Channel
      {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
                SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
      }
		}
    break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:    
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
            
        SX1276LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
        //		if( DIO3 == 1 ) // CAD Done interrupt
        if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_CADDONE_MASK )
        { 
            // Clear Irq
           // SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_CADDETECTED_MASK )
            {
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            } 
            else
            {    
                // The device goes in Standby Mode automatically    
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }   
        break;
    
    default:
        break;
    } 
    return result;
}

/************************************************
函数名称 ： SX1276GetLoraTimeOnAir
功    能 ： 获取空中时间
参    数 ： pktLen		有效负载长度			
返 回 值 ： 无
作    者 ： sun
*************************************************/
uint32_t SX1276GetLoraTimeOnAir(uint8_t pktLen )
{
    uint32_t airTime = 0;
		double tPreamble;
    double rs;
		double ts;
		double tmp ;
		double nPayload ;
		double tPayload;
		double tOnAir;
		double bw = 0.0;
		// REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
		switch( LoRaSettings.SignalBw )
		{
			case 0: // 7.8 kHz
			    bw = 7800;
			    break;
			case 1: // 10.4 kHz
			    bw = 10400;
			    break;
			case 2: // 15.6 kHz
			    bw = 15600;
			    break;
			case 3: // 20.8 kHz
			    bw = 20800;
			    break;
			case 4: // 31.2 kHz
			    bw = 31200;
			    break;
			case 5: // 41.4 kHz
			    bw = 41400;
			    break;
			case 6: // 62.5 kHz
			    bw = 62500;
			    break;
			case 7: // 125 kHz
				bw = 125000;
			break;
			case 8: // 250 kHz
				bw = 250000;
			break;
			case 9: // 500 kHz
				bw = 500000;
			break;
		}

		// Symbol rate : time for one symbol (secs)
		 rs = bw / ( 1<< LoRaSettings.SpreadingFactor );
		//	 rs = bw / (pow(2, LoRaSettings.SpreadingFactor));
		 ts = 1 / rs;
		// time of preamble
		 tPreamble = ( SX1276LoRaGetPreambleLength() + 4.25 ) * ts;
		// Symbol length of payload and time
		 tmp = ceil( ( 8 * pktLen - 4 * LoRaSettings.SpreadingFactor +28 + 16 - ( LoRaSettings.ImplicitHeaderOn ? 20 : 0 ) ) /
		( double )( 4 * ( LoRaSettings.SpreadingFactor -( ( SX1276LoRaGetLowDatarateOptimize() > 0 ) ? 1 : 0 ) ) ) ) *( LoRaSettings.ErrorCoding + 4 );
		 nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
		 tPayload = nPayload * ts;
		// Time on air
		 tOnAir = tPreamble + tPayload;
		// return ms secs
		airTime = floor( tOnAir * 933 + 0.999 );
		return airTime;
}

uint32_t SX1276GetLoraDataRate(void)
{
		double bw = 0.0;
		double rs;
		double ts;
		double bps;
		// REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
		switch( LoRaSettings.SignalBw )
		{
			case 0: // 7.8 kHz
			    bw = 7800;
			    break;
			case 1: // 10.4 kHz
			    bw = 10400;
			    break;
			case 2: // 15.6 kHz
			    bw = 15600;
			    break;
			case 3: // 20.8 kHz
			    bw = 20800;
			    break;
			case 4: // 31.2 kHz
			    bw = 31200;
			    break;
			case 5: // 41.4 kHz
			    bw = 41400;
			    break;
			case 6: // 62.5 kHz
			    bw = 62500;
			    break;
			case 7: // 125 kHz
				bw = 125000;
			break;
			case 8: // 250 kHz
				bw = 250000;
			break;
			case 9: // 500 kHz
				bw = 500000;
			break;
		}
		
		// Symbol rate : time for one symbol (secs)
		 rs = bw / ( 1<< LoRaSettings.SpreadingFactor );
		//	 rs = bw / (pow(2, LoRaSettings.SpreadingFactor));
		// ts = 1 / rs;
		
		ts = (double)((4 * LoRaSettings.SpreadingFactor )/( LoRaSettings.ErrorCoding + 4 ));
		bps = rs * ts ;
		return (uint32_t)bps;
}
