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
 * \file       radio.h
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.B2
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */
#ifndef __RADIO_H__
#define __RADIO_H__

#include "platform.h"

#include <stdbool.h>
#include "radio\sx1276-LoRa.h"
#include <math.h>
/*!
 * SX1272 and SX1276 General parameters definition
 */
#define LORA                                        1         // [0: OFF, 1: ON]

/*!
 * RF process function return codes
 */
//typedef enum
//{
//    RF_IDLE,
//    RF_BUSY,
//		RF_CHANNEL_BUSY,
//    RF_RX_DONE,
//    RF_RX_TIMEOUT,
//    RF_RX_ID_ERROR,
//    RF_TX_DONE,
//    RF_TX_TIMEOUT,
//    RF_LEN_ERROR,
//    RF_CHANNEL_EMPTY,
//    RF_CHANNEL_ACTIVITY_DETECTED
//} tRFProcessReturnCodes;

/*!
 * Radio driver structure defining the different function pointers
 */
typedef struct sRadioDriver
{
  uint8_t ( *Init )( void );
  void ( *Reset )( void );
  void ( *StartCad )( void );
  void ( *StartRx )( void );
  void ( *GetRxPacket )( void *buffer, uint16_t *size );
  void ( *SetTxPacket )( const void *buffer, uint16_t size );
  uint32_t ( *Process )( void );

	void ( *LoRaSetRFFrequency )( uint32_t freq );
	uint32_t ( *LoRaGetRFFrequency )( void );
	void ( *LoRaSetRFPower )( int8_t power );
	int8_t ( *LoRaGetRFPower )( void );
	void ( *LoRaSetSignalBandwidth )( uint8_t bw );
	uint8_t ( *LoRaGetSignalBandwidth )( void );
	void ( *LoRaSetSpreadingFactor )( uint8_t factor );
	uint8_t ( *LoRaGetSpreadingFactor )( void );
	void ( *LoRaSetErrorCoding )( uint8_t value );
	uint8_t ( *LoRaGetErrorCoding )( void );
	void ( *LoRaSetPacketCrcOn )( bool enable );
	bool ( *LoRaGetPacketCrcOn )( void );
	void ( *LoRaSetImplicitHeaderOn )( bool enable );
	bool ( *LoRaGetImplicitHeaderOn )( void );
	void ( *LoRaSetRxSingleOn )( bool enable );
	bool ( *LoRaGetRxSingleOn )( void );
	void ( *LoRaSetFreqHopOn )( bool enable );
	bool ( *LoRaGetFreqHopOn )( void );
	void ( *LoRaSetHopPeriod )( uint8_t value );
	uint8_t ( *LoRaGetHopPeriod )( void );
	void ( *LoRaSetTxPacketTimeout )( uint32_t value );
	uint32_t ( *LoRaGetTxPacketTimeout )( void );
	void ( *LoRaSetRxPacketTimeout )( uint32_t value );
	uint32_t ( *LoRaGetRxPacketTimeout )( void );
	void ( *LoRaSetPayloadLength )( uint8_t value );
	uint8_t ( *LoRaGetPayloadLength )( void );
	void ( *LoRaSetPa20dBm )( bool enale );
	bool ( *LoRaGetPa20dBm )( void );
	void ( *LoRaSetPaRamp )( uint8_t value );
	uint8_t ( *LoRaGetPaRamp )( void );
	void ( *LoRaSetSymbTimeout )( uint16_t value );
	uint16_t ( *LoRaGetSymbTimeout )( void );
	void ( *LoRaSetLowDatarateOptimize )( bool enable );
	bool ( *LoRaGetLowDatarateOptimize )( void );
	uint16_t ( *LoRaGetPreambleLength )( void );
	void ( *LoRaSetPreambleLength )( uint16_t value );
	void ( *LoRaSetNbTrigPeaks )( uint8_t value );

	void ( *LoRaSetOpMode )( uint8_t opMode );
	uint8_t ( *LoRaGetOpMode )( void );

	void ( *Read )( uint8_t addr, uint8_t *data );
	void ( *Write )( uint8_t addr, uint8_t data );
} tRadioDriver;


extern tRadioDriver *Radio;

/*!
 * \brief Initializes the RadioDriver structure with specific radio
 *        functions.
 *
 * \retval radioDriver Pointer to the radio driver variable
 */
tRadioDriver* RadioDriverInit( void );


#endif // __RADIO_H__
