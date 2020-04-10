/*! 
 * \file        LoraComm.c
 * \brief        
 *
 * \version     1.0
 * \date        2020-04-10
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "mb.h"
#include "mbport.h"

#include "radio.h"
#include "tranceive/LoraComm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Client_ADDRESS      0x01        /* modbus client id */
#define BAUDRATE            9600        /* ignore */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{   
    /* Init Radio */
    Radio = RadioDriverInit();
    Radio->Init();

    /* Init freemodbus */
    (void)eMBInit(MB_RTU, Client_ADDRESS, 0, BAUDRATE, MB_PAR_NONE);

    while (1)
    {
        /* Lora communication */
        LoraComm();

        /* freemodbus poll */
        (void)eMBPoll();
    }
    
    return 0;
}
}

/*  End of File  */