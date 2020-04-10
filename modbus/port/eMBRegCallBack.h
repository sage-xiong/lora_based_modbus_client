/*! 
 * \file        eMBRegCallBack.h
 * \brief        
 *
 * \version     1.0
 * \date        2019-10-16
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */
#include "mbport.h"
 
/* ----------------------- Defines ------------------------------------------ */
#define REG_INPUT_START   0x100
#define REG_INPUT_NREGS   64

#define REG_HOLDING_START 100
#define REG_HOLDING_NREGS 256

/* ----------------------- Static variables --------------------------------- */
extern USHORT usRegInputStart;
extern USHORT usRegInputBuf[REG_INPUT_NREGS];

extern USHORT usRegHoldingStart;
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* End of File */
