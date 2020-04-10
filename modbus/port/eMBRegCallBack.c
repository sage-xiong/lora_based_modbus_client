/*! 
 * \file        eMBRegCallBack.c
 * \brief        
 *
 * \version     1.0
 * \date        2019-10-16
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */

/* ----------------------- Modbus includes ---------------------------------- */
#include "mb.h"
#include "mbport.h"
#include "eMBRegCallBack.h"

#include <stdio.h>
#include <string.h>

/* ----------------------- Variables ---------------------------------------- */
USHORT usRegInputStart;
USHORT usRegInputBuf[REG_INPUT_NREGS];

USHORT usRegHoldingStart;
USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- Start implementation ----------------------------- */

// ########## eMBRegInputCB ####################################################
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;
  
  printf("Enter into eMBRegInputCB!\r\n");

  if( (usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS) )
  {
    iRegIndex = 0;
    while( usNRegs > 0 )
    {
      *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
      *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
      iRegIndex++;
      usNRegs--;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }

  return eStatus;
}

// ########## eMBRegHoldingCB ##################################################
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;
  
  printf("Enter into eMBRegHoldingCB!\r\n");
  
  if( (usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS) )
  {
    iRegIndex = ( int )( usAddress - usRegHoldingStart );
    switch ( eMode )
    {
        /* Pass current register values to the protocol stack. */
    case MB_REG_READ:
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
        break;

    /* Update current register values with new values from the
     * protocol stack. */
    case MB_REG_WRITE:
      while( usNRegs > 0 )
      {
          usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          iRegIndex++;
          usNRegs--;
      }
    }
  }
  else
  {
      eStatus = MB_ENOREG;
  }
  
  return eStatus;
}

// ########## eMBRegCoilsCB ####################################################
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
  printf("Enter into eMBRegCoilsCB!\r\n");
  return MB_ENOREG;
}

// ########## eMBRegDiscreteCB #################################################
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  printf("Enter into eMBRegDiscreteCB!\r\n");
  return MB_ENOREG;
}

/* End of File */
