/*! 
 * \file        porttimer.c
 * \brief        
 *
 * \version     1.0
 * \date        2019-10-08
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port/port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
/* ----------------------- Static variables ---------------------------------*/
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  return TRUE;
}


void
vMBPortTimersEnable(void)
{
}

void
vMBPortTimersDisable(void)
{
}

/* End of File */
