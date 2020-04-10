/*! 
 * \file        LoraComm.h
 * \brief        
 *
 * \version     1.0
 * \date        2019-09-19
 * \author      Sage Xiong
 * \email       sage.xiong@foxmail.com
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LORACOMM_H
#define _LORACOMM_H

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

/* Exported types ------------------------------------------------------------*/
typedef enum LORAUSART_t
{
	LORA_START,
	LORA_RECEIVE,
	LORA_GETBUFF,
	LORA_SEND,
	LORA_TRANSMITED,
}LORACOMM;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define COMMOM_RF           		433920000
#define UPLINK_FREQUENCY				433920000             //435000000
#define DOWNLINK_FREQUENCY			433920000              //435000000

#define SHORT_PREAMBLE_LENGTH		12
#define LONG_PREAMBLE_LENGTH		60

#define MAXLOADLEN        			128

#define LoRaBuffSize            256U

extern uint8_t LoRaBuff[LoRaBuffSize];
extern LORACOMM t_LoraComm;
extern uint16_t LoraTxCount;

/* Exported functions ------------------------------------------------------- */
void LoraComm(void);

void LoraReadInit(uint8_t SingleMode,uint32_t RF_Frequency,uint32_t Timeout, uint16_t preamble );
uint8_t LoraWrite(uint8_t * buffer,uint16_t bufferLen,uint32_t RF_Frequency,uint16_t preamble,uint32_t Timeout );
int16_t LoraSpecialRead( void *Buffer ,uint16_t PreambleLength,uint32_t Timeout, int8_t *snr,int16_t *rssi );
void LoraSpecialWrite(uint8_t * buffer,uint16_t bufferLen,uint32_t RF_Frequency,uint16_t preamble,uint32_t Timeout );

#endif /* _LORACOMM_H */
