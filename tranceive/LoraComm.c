/*! 
 * \file        LoraComm.c
 * \brief        
 *
 * \version     1.0
 * \date        2019-09-19
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */
 
/* Includes ------------------------------------------------------------------*/
#include "tranceive/LoraComm.h"

#include "radio/radio.h"
#include "stdio.h"	
#include "radio/sx1276-Hal.h"
#include "radio/sx1276.h"
#include "radio/sx1276-LoRaMisc.h"
#include "radio/sx1276-LoRa.h"

#include <string.h>

#include "mb.h"
#include "mbport.h"
#include "ringbuffer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LENGTH_OF_DAT_PAYLOAD		25

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t LoRaBuff[LoRaBuffSize];
uint16_t LoraTxCount=0;
LORACOMM t_LoraComm = LORA_START;

extern RingBuffer  Serial_RX_RingBuffer;  //!< define in portserial.c
extern RingBuffer  Serial_TX_RingBuffer;  //!< define in portserial.c

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/************************************************
函数名称 ： 应用层函数，Lora通信函数
功    能 ： 完成从 LoRa 模块获取数据
参    数 ： buf  接收缓存
						size 缓存长度
返 回 值 ： 无
作    者 ： sage xiong
*************************************************/
void LoraComm(void)
{
	uint32_t result = 0;
	uint16_t bufferLen = 0;
  
  switch(t_LoraComm)
  {
    case LORA_START:
      LoraReadInit(0,UPLINK_FREQUENCY ,100 ,SHORT_PREAMBLE_LENGTH);
      t_LoraComm = LORA_RECEIVE;
      break;
    case LORA_RECEIVE:
      result =  Radio->Process( );
      if(result == RF_RX_DONE)
      {
        Radio->GetRxPacket( LoRaBuff, ( uint16_t* )&bufferLen );
        
        pxMBPortCBTimerExpired();
        for(uint8_t i=0;i<bufferLen;i++)
        {
          rbPush(&Serial_RX_RingBuffer, LoRaBuff[i]);
          pxMBFrameCBByteReceived();
        }
        pxMBPortCBTimerExpired();
        t_LoraComm = LORA_START;
      }
      else if(result == RF_RX_TIMEOUT)
      {
        t_LoraComm = LORA_START;
      }
      break;
    case LORA_GETBUFF:
      break;
    case LORA_SEND:
      {
        eMBEventType eEvent;
        
        if(xMBPortEventGet( &eEvent ) == TRUE)
        {
          if(eEvent == EV_FRAME_SENT)
          {
            LoraWrite(LoRaBuff,LoraTxCount-1,UPLINK_FREQUENCY ,SHORT_PREAMBLE_LENGTH,3000);
            t_LoraComm = LORA_START;
          }
        }
        break;
      }
    case LORA_TRANSMITED:
      break;
    default:
      break;
  }
}

/************************************************
函数名称 ： 应用层函数，Lora接收初始化函数
功    能 ： Lora接收初始化函数
参    数 ： SingleMode  单次接收还是循环接收，1为单次接收，0为循环接收，
						RF_Frequency 接收频率参数
						Timeout 超时时间

返 回 值 ： 无
作    者 ： sun
*************************************************/
void LoraReadInit(uint8_t SingleMode,uint32_t RF_Frequency,uint32_t Timeout, uint16_t preamble )
{
	Radio->LoRaSetRxSingleOn( SingleMode );
	Radio->LoRaSetRFFrequency( RF_Frequency );
	Radio->LoRaSetPreambleLength( preamble );
	Radio->LoRaSetRxPacketTimeout( Timeout );
	Radio->StartRx( );
}

/************************************************
函数名称 ： 应用层函数，LoraWrite
功    能 ： Lora发送   阻塞函数
参    数 ： buffer  发送缓存区，
						bufferLen 发送数据长度
						RF_Frequency 接收频率

返 回 值 ： 无
作    者 ： sun
*************************************************/
uint8_t LoraWrite(uint8_t * buffer,uint16_t bufferLen,uint32_t RF_Frequency,uint16_t preamble,uint32_t Timeout )
{
	uint32_t result = 0;
  
  Radio->LoRaSetRFFrequency( RF_Frequency );	//设置发射频率 		参数来自宏定义
  Radio->LoRaSetPreambleLength( preamble );
  Radio->LoRaSetTxPacketTimeout( Timeout );
  
  if(bufferLen > MAXLOADLEN)
  {
    Radio->SetTxPacket( buffer, MAXLOADLEN );		//开始发送
  }
  else
  {
    Radio->LoRaSetPayloadLength(bufferLen);
    Radio->SetTxPacket( buffer, bufferLen );		//开始发送
  }
  while( 1 )
	{
		result = Radio->Process( );

    if( result == RF_TX_DONE || result == RF_TX_TIMEOUT )
    break;
	}
	if( result == RF_TX_DONE )
	{
		return 1;
	}

		return 0;
}

 /************************************************
函数名称 ： ReceiverProcess
功    能 ： 单信道接收,阻塞函数,接收指定敷在长度的数据
参    数 ： preamble  前导码长度
						timeout  接收超时时间
						snr   	 信噪比
						rssi     信号强度
返 回 值 ： 接收到数据返回数据长度  失败返回-1
作    者 ： sun
*************************************************/
int16_t LoraSpecialRead( void *Buffer ,uint16_t PreambleLength,uint32_t Timeout, int8_t *snr,int16_t *rssi )
{
	uint32_t result;
	uint16_t bufferLen = 0;
  
	Radio->LoRaSetRxSingleOn( 0 );
	Radio->LoRaSetRFFrequency( UPLINK_FREQUENCY );
	
	Radio->LoRaSetRxPacketTimeout( Timeout );
	
	Radio->LoRaSetPreambleLength( PreambleLength );
	
	Radio->LoRaSetPayloadLength( LENGTH_OF_DAT_PAYLOAD );

	Radio->StartRx( );
  
	while( 1 )
	{
		result = Radio->Process( );
		if( result == RF_RX_DONE || result == RF_RX_TIMEOUT )
		break;
	}
	if( result == RF_RX_DONE )
	{
    Radio->GetRxPacket( Buffer, ( uint16_t* )&bufferLen );
    *rssi = ( int16_t )( SX1276LoRaGetPacketRssi( ) + 0.5 );
    *snr = SX1276LoRaGetPacketSnr( );
    
		return bufferLen;
	}
	return -1;
}


/************************************************
函数名称 ： 应用层函数，LoraSpecialWrite
功    能 ： Lora发送 把查询发送完成 请在程序里查询 防止阻塞
参    数 ： buffer  发送缓存区，
						bufferLen 发送数据长度
						RF_Frequency 接收频率

返 回 值 ： 无
作    者 ： sun
*************************************************/
void LoraSpecialWrite(uint8_t * buffer,uint16_t bufferLen,uint32_t RF_Frequency,uint16_t preamble,uint32_t Timeout )
{
		Radio->LoRaSetRFFrequency( RF_Frequency );	//设置发射频率 		参数来自宏定义
		Radio->LoRaSetPreambleLength( preamble );
		Radio->LoRaSetTxPacketTimeout( Timeout );
  
		if(bufferLen > MAXLOADLEN)
		{
			Radio->SetTxPacket( buffer, MAXLOADLEN );		//开始发送
		}
		else
		{
			Radio->LoRaSetPayloadLength(bufferLen);
			Radio->SetTxPacket( buffer, bufferLen );		//开始发送
		}
}

/*  End of File  */
