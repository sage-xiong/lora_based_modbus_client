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
 * \file       sx1276-LoRaMisc.c
 * \brief      SX1276 RF chip high level functions driver
 *
 * \remark     Optional support functions.
 *             These functions are defined only to easy the change of the
 *             parameters.
 *             For a final firmware the radio parameters will be known so
 *             there is no need to support all possible parameters.
 *             Removing these functions will greatly reduce the final firmware
 *             size.
 *
 * \version    2.0.B2
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */


 /*

LoRa寄存器设置接口

 */

#include "sx1276-Hal.h"
#include "sx1276.h"

#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

extern tLoRaSettings LoRaSettings;



/************************************************
函数名称 ：  LoRa函数，SX1276LoRaSetRFFrequency，实时修改，动态修改LoRa工作频率
功    能 ： 修改LoRa工作频率，请在SX1276Init( ); LoRa初始化之后调用，
						动态修改LoRa工作频率
参    数 ： 	freq 工作频率
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1276LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1276LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1276WriteBuffer( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );

    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );

    if( LoRaSettings.RFFrequency > 860000000 )
    {
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | RFLR_PACONFIG_PASELECT_RFO;
    }
    else
    {
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | RFLR_PACONFIG_PASELECT_PABOOST;
    }
    SX1276Write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
}


/************************************************
函数名称 ： LoRa应用层函数，SX1276LoRaGetRFFrequency，
功    能 ：获取当前LoRa工作频率
						
参    数 ：无
返 回 值 ：当前LoRa工作频率
作    者 ：sun
*************************************************/
uint32_t SX1276LoRaGetRFFrequency( void )
{
    SX1276ReadBuffer( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );
    LoRaSettings.RFFrequency = ( ( uint32_t )SX1276LR->RegFrfMsb << 16 ) | ( ( uint32_t )SX1276LR->RegFrfMid << 8 ) | ( ( uint32_t )SX1276LR->RegFrfLsb );
    LoRaSettings.RFFrequency = ( uint32_t )( ( double )LoRaSettings.RFFrequency * ( double )FREQ_STEP );
    return LoRaSettings.RFFrequency;
}
/************************************************
函数名称 ：  LoRa函数，  SX1276LoRaSetRFPower实时修改
功    能 ： 修改LoRa发射功率，请在SX1276Init( ); LoRa初始化之后调用
参    数 ： Power 发射功率 取值 7-20
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetRFPower( int8_t power )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
    LoRaSettings.Power = power;
}
/************************************************
函数名称 ：LoRa函数，  SX1276LoRaGetRFPower获取LoRa当前发射功率大小
功    能 ：请在SX1276Init( ); LoRa初始化之后调用
参    数 ：无
返 回 值 ：Power 发射功率 取值 7-20
作    者 ：sun
*************************************************/
int8_t SX1276LoRaGetRFPower( void )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 )
        {
            LoRaSettings.Power = 5 + ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            LoRaSettings.Power = 2 + ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        LoRaSettings.Power = -1 + ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return LoRaSettings.Power;
}


/************************************************
函数名称 ：  LoRa函数，  SX1276LoRaSetSignalBandwidth实时修改
功    能 ： 修改LoRa带宽，请在SX1276Init( ); LoRa初始化之后调用
参    数 ： SignalBw
            // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
            // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetSignalBandwidth( uint8_t bw )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}
/************************************************
函数名称 ：  LoRa函数，  SX1276LoRaGetSignalBandwidth获取当前的LoRa工作带宽
功    能 ： 修改LoRa带宽，请在SX1276Init( ); LoRa初始化之后调用
参    数 ： 无
返 回 值 ： SignalBw
            // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
            // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetSignalBandwidth( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    return LoRaSettings.SignalBw;
}
/************************************************
函数名称 ：  LoRa函数， SX1276LoRaSetSpreadingFactor实时修改
功    能 ： 修改LoRa扩频因子，请在SX1276Init( ); LoRa初始化之后调用
参    数 ： SF
 // SpreadingFactor [ 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
        SX1276LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1276LoRaSetNbTrigPeaks( 3 );
    }

    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1276Write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );
    LoRaSettings.SpreadingFactor = factor;
}
/************************************************
函数名称 ：  LoRa函数， SX1276LoRaGetSpreadingFactor获取LoRa当前工作扩频因子
功    能 ： 获取LoRa扩频因子，请在SX1276Init( ); LoRa初始化之后调用
参    数 ： SF
 // SpreadingFactor [ 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
返 回 值 ： 无
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetSpreadingFactor( void )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    LoRaSettings.SpreadingFactor = ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return LoRaSettings.SpreadingFactor;
}
/************************************************
函数名称 ：  LoRa函数， SX1276LoRaSetErrorCoding设置LoRa前向纠错码
功    能 ： 实时动态修改LoRa前向纠错码
参    数 ： value
            ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] 
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetErrorCoding( uint8_t value )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}
/************************************************
函数名称 ： LoRa函数， SX1276LoRaGetErrorCoding获取LoRa前向纠错码
功    能 ： 获取LoRa当前工作的前向纠错码
参    数 ： 无
返 回 值 ： value
            ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] 
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetErrorCoding( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    return LoRaSettings.ErrorCoding;
}
/************************************************
函数名称 ：  LoRa函数， SX1276LoRaSetPacketCrcOn设置LoRa当前是否启用校验
功    能 ： 设置LoRa当前工作是否启用校验
参    数 ： enable 1:开启，0，不开启
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetPacketCrcOn( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    SX1276Write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

/************************************************
函数名称 ：  LoRa函数， SX1276LoRaGetPacketCrcOn  获取LoRa数据包校验是否开启
功    能 ： 获取LoRa当前工作是否启用校验
参    数 ： 无
返 回 值 ： 1:开启，0，不开启
作    者 ： sun
*************************************************/
bool SX1276LoRaGetPacketCrcOn( void )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    return LoRaSettings.CrcOn;
}

/************************************************
函数名称 ：LoRa函数，SX1276LoRaSetPreambleLength，实时修改发送接收数据包前导码长度
功    能 ：修改LoRa前导码，请在SX1276Init( ); LoRa初始化之后调用
           CAD模式时，需要在发送唤醒包之前调用次函数修改前导码长度
参    数 ： value 前导码长度，8-65535，具体数值请根据MCU唤醒间隔，进行设置，具体请根据官网计算软件计算
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetPreambleLength( uint16_t value )
{
    SX1276ReadBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );

    SX1276LR->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    SX1276LR->RegPreambleLsb = value & 0xFF;
    SX1276WriteBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetPreambleLength，获取LoRa工作前导码长度
功    能 ： 获取LoRa前导码，请在SX1276Init( ); LoRa初始化之后调用          
参    数 ： 无
返 回 值 ： 前导码长度，8-65535，具体数值请根据MCU唤醒间隔，进行设置，具体请根据官网计算软件计算
作    者 ： sun
*************************************************/
uint16_t SX1276LoRaGetPreambleLength( void )
{
    SX1276ReadBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
    return ( ( SX1276LR->RegPreambleMsb & 0x00FF ) << 8 ) | SX1276LR->RegPreambleLsb;
}


/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetImplicitHeaderOn，设置LoRa报头显示模式
功    能 ： 设置LoRa报头显示模式。   
参    数 ： 0为显示报头模式，1为隐式，显示报头模式下可以读取任意长度数据，数据长度包含在虚头中，隐式只能读取指定长度，且收发必须一致
					  在负载长度以知道的情况下调用隐式可缩短发送时间
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetImplicitHeaderOn( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetImplicitHeaderOn，读取LoRa报头显示模式
功    能 ： 获取LoRa前导码，请在SX1276Init( ); LoRa初始化之后调用          
参    数 ： 无
返 回 值 ： 0为显示报头模式，1为隐式，显示报头模式下可以读取任意长度数据，数据长度包含在虚头中，隐式只能读取指定长度，且收发必须一致
					 在负载长度以知道的情况下调用隐式可缩短发送时间
作    者 ： sun
*************************************************/
bool SX1276LoRaGetImplicitHeaderOn( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    return LoRaSettings.ImplicitHeaderOn;
}
/************************************************
函数名称 ：  LoRa函数，SX1276LoRaSetRxSingleOn
功    能 ： 设置LoRa接收模式，单次接收还是连续接收          
参    数 ： enable，0:连续接收模式 1;单次接收
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetRxSingleOn( bool enable )
{
    LoRaSettings.RxSingleOn = enable;
}


/************************************************
函数名称 ：  LoRa函数，SX1276LoRaGetRxSingleOn
功    能 ： 读取LoRa接收模式，单次接收还是连续接收          
参    数 ： 无
返 回 值 ： 0:连续接收模式 1;单次接收
作    者 ： sun
*************************************************/
bool SX1276LoRaGetRxSingleOn( void )
{
    return LoRaSettings.RxSingleOn;
}
/************************************************
函数名称 ：  LoRa函数，SX1276LoRaSetFreqHopOn
功    能 ： 设置调频模式         
参    数 ： 无
返 回 值 ： 1：调频，0:不跳频
作    者 ： sun
*************************************************/
void SX1276LoRaSetFreqHopOn( bool enable )
{
    LoRaSettings.FreqHopOn = enable;
}

/************************************************
函数名称 ：  LoRa函数，SX1276LoRaGetFreqHopOn
功    能 ： 获取LoRa调频模式       
参    数 ： 无
返 回 值 ： 1:调频 0;不跳频
作    者 ： sun
*************************************************/
bool SX1276LoRaGetFreqHopOn( void )
{
    return LoRaSettings.FreqHopOn;
}
/************************************************
函数名称 ：  LoRa函数，SX1276LoRaSetHopPeriod
功    能 ： 设置跳频周期设置      
参    数 ： 跳频周期
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetHopPeriod( uint8_t value )
{
    SX1276LR->RegHopPeriod = value;
    SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
    LoRaSettings.HopPeriod = value;
}
/************************************************
函数名称 ：  LoRa函数，SX1276LoRaGetHopPeriod
功    能 ： 读取跳频周期    
参    数 ： 无
返 回 值 ： 调频周期，符号为单位
作    者 ： sun
*************************************************/
uint8_t SX1276LoRaGetHopPeriod( void )
{
    SX1276Read( REG_LR_HOPPERIOD, &SX1276LR->RegHopPeriod );
    LoRaSettings.HopPeriod = SX1276LR->RegHopPeriod;
    return LoRaSettings.HopPeriod;
}

/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetTxPacketTimeout
功    能 ： LoRa数据发送超时时间  
参    数 ： value 单位 ms
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetTxPacketTimeout( uint32_t value )
{
    LoRaSettings.TxPacketTimeout = value;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetTxPacketTimeout
功    能 ： 获取LoRa数据包发送超时时间 
参    数 ： 无
返 回 值 ： value 单位 ms
作    者 ： sun
*************************************************/
uint32_t SX1276LoRaGetTxPacketTimeout( void )
{
    return LoRaSettings.TxPacketTimeout;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetRxPacketTimeout
功    能 ： LoRa数据接收超时时间  
参    数 ： value 单位 ms
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetRxPacketTimeout( uint32_t value )
{
    LoRaSettings.RxPacketTimeout = value;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetRxPacketTimeout
功    能 ： 获取LoRa数据包接收超时时间 ，仅仅单次接收有效
参    数 ： 无
返 回 值 ： value 单位 ms
作    者 ： sun
*************************************************/
uint32_t SX1276LoRaGetRxPacketTimeout( void )
{
    return LoRaSettings.RxPacketTimeout;
}

/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetPayloadLength
功    能 ： 设置发送包数据长度
参    数 ： value 数据包长度
返 回 值 ： 无
作    者 ： sun
*************************************************/
void SX1276LoRaSetPayloadLength( uint8_t value )
{
    SX1276LR->RegPayloadLength = value;
    SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetPayloadLength
功    能 ：获取发送包数据长度
参    数 ：无
返 回 值  数据包长度
作    者 ：sun
*************************************************/
uint8_t SX1276LoRaGetPayloadLength( void )
{
    SX1276Read( REG_LR_PAYLOADLENGTH, &SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = SX1276LR->RegPayloadLength;
    return LoRaSettings.PayloadLength;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetPa20dBm
功    能 ：20dBm 发送启动
参    数 ：1：启用 2：不启用。
返 回 值   无
作    者 ：sun
*************************************************/
void SX1276LoRaSetPa20dBm( bool enale )
{
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( enale == true )
        {
            SX1276LR->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1276LR->RegPaDac = 0x84;
    }
    SX1276Write( REG_LR_PADAC, SX1276LR->RegPaDac );
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetPa20dBm
功    能 ：获取20dBm 发送启动标志
参    数 ：
返 回 值 ： 1：启用 2：不启用。
作    者 ：sun
*************************************************/
bool SX1276LoRaGetPa20dBm( void )
{
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    return ( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetPAOutput
功    能 ： PA引脚输出选择
参    数 ： 0 RFO引脚 （输出不得超过+14dBm） 1 PA_BOOST引脚选择（输出不得超过+20dBm）
返 回 值 ： 无
作    者 ：sun
*************************************************/
void SX1276LoRaSetPAOutput( uint8_t outputPin )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1276Write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetPAOutput
功    能 ： 获取PA引脚输出选择
参    数 ： 无
返 回 值 ：0 RFO引脚 （输出不得超过+14dBm） 1 PA_BOOST引脚选择（输出不得超过+20dBm）
作    者 ：sun
*************************************************/
uint8_t SX1276LoRaGetPAOutput( void )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    return SX1276LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaSetPaRamp
功    能 ：设置LoRa模式下斜升/斜降时间
参    数 ：0 -15 （3.4ms - 10uS）
返 回 值 ：无
作    者 ：sun
*************************************************/
void SX1276LoRaSetPaRamp( uint8_t value )
{
    SX1276Read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    SX1276LR->RegPaRamp = ( SX1276LR->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    SX1276Write( REG_LR_PARAMP, SX1276LR->RegPaRamp );
}
/************************************************
函数名称 ： LoRa函数，SX1276LoRaGetPaRamp
功    能 ：获取LoRa模式下斜升/斜降时间
参    数 ：无
返 回 值 ：0 -15 （3.4ms - 10uS）
作    者 ：sun
*************************************************/
uint8_t SX1276LoRaGetPaRamp( void )
{
    SX1276Read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    return SX1276LR->RegPaRamp & ~RFLR_PARAMP_MASK;
}

/************************************************
函数名称 ：LoRa函数，SX1276LoRaSetSymbTimeout
功    能 ：RX超时时间
参    数 ：超时时间，符号
返 回 值 ：无
作    者 ：sun
*************************************************/
void SX1276LoRaSetSymbTimeout( uint16_t value )
{
    SX1276ReadBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );

    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1276LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1276WriteBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
}

/************************************************
函数名称 ：LoRa函数，SX1276LoRaGetSymbTimeout
功    能 ：获取RX超时时间
参    数 ：无
返 回 值 ：超时时间，符号
作    者 ：sun
*************************************************/
uint16_t SX1276LoRaGetSymbTimeout( void )
{
    SX1276ReadBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
    return ( ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | SX1276LR->RegSymbTimeoutLsb;
}
/************************************************
函数名称 ：LoRa函数，SX1276LoRaSetLowDatarateOptimize
功    能 ：低速率优化，符号长度超过16ms时必须打开
参    数 ：0：关闭 1：启用
返 回 值 ：无
作    者 ：sun
*************************************************/
void SX1276LoRaSetLowDatarateOptimize( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    SX1276LR->RegModemConfig3 = ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    SX1276Write( REG_LR_MODEMCONFIG3, SX1276LR->RegModemConfig3 );
}
/************************************************
函数名称 ：LoRa函数，SX1276LoRaGetLowDatarateOptimize
功    能 ：获取低速率优化，符号长度超过16ms时必须打开
参    数 ：无
返 回 值 ：0：关闭 1：启用
作    者 ：sun
*************************************************/
bool SX1276LoRaGetLowDatarateOptimize( void )
{
    SX1276Read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    return ( ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

void SX1276LoRaSetNbTrigPeaks( uint8_t value )
{
    SX1276Read( 0x31, &SX1276LR->RegTestReserved31 );
    SX1276LR->RegTestReserved31 = ( SX1276LR->RegTestReserved31 & 0xF8 ) | value;
    SX1276Write( 0x31, SX1276LR->RegTestReserved31 );
}

uint8_t SX1276LoRaGetNbTrigPeaks( void )
{
    SX1276Read( 0x31, &SX1276LR->RegTestReserved31 );
    return ( SX1276LR->RegTestReserved31 & 0x07 );
}


