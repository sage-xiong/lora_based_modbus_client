/*! 
 * \file        portserial.c
 * \brief        
 *
 * \version     1.0
 * \date        2019-10-08
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */
#include "port/port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "tranceive/LoraComm.h"

#include "ringbuffer.h"

#include <string.h>

RingBuffer  Serial_RX_RingBuffer;
uint8_t     Serial_RX_Buffer[256];

RingBuffer  Serial_TX_RingBuffer;
uint8_t     Serial_TX_Buffer[256];

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if(xRxEnable == TRUE && xTxEnable == FALSE)
  {
    rbClear(&Serial_RX_RingBuffer);
  }
  else if(xRxEnable == FALSE && xTxEnable == TRUE)
  {
    eMBEventType eEvent;
  
    LoraTxCount = 0;
    memset(LoRaBuff, 0, sizeof LoRaBuff);
    do
    {
      pxMBFrameCBTransmitterEmpty();
      xMBPortEventGet(&eEvent);
      LoRaBuff[LoraTxCount++] = rbPop(&Serial_TX_RingBuffer);
    }while(eEvent != EV_FRAME_SENT);
    
    t_LoraComm = LORA_SEND;
    xMBPortEventPost(EV_FRAME_SENT);
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  rbInitialize(&Serial_RX_RingBuffer, Serial_RX_Buffer, sizeof Serial_RX_Buffer);
  rbInitialize(&Serial_TX_RingBuffer, Serial_TX_Buffer, sizeof Serial_TX_Buffer);
  
  return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  rbPush(&Serial_TX_RingBuffer, ucByte);
  
  return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  if( !rbIsEmpty(&Serial_RX_RingBuffer) )
  {
    *pucByte = rbPop(&Serial_RX_RingBuffer);
    return TRUE;
  }
  return FALSE;
}

/* End of File */
