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
 * \file       radio.c
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.B2
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */

#include "radio/radio.h"
#include "platform.h"

#if defined( USE_SX1232_RADIO )
    #include "radio/sx1232.h"
#elif defined( USE_SX1272_RADIO )
    #include "radio/sx1272.h"
	#include "radio/sx1272-LoRaMisc.h"
	#include "radio/sx1272-LoRa.h"
	#include "radio/sx1272-Hal.h"
#elif defined( USE_SX1276_RADIO )
    #include "radio/sx1276.h"
	#include "radio/sx1276-LoRaMisc.h"
	#include "radio/sx1276-LoRa.h"
	#include "radio/sx1276-Hal.h"
#else
	#error "Missing define: USE_XXXXXX_RADIO (ie. USE_SX1272_RADIO)"
#endif


tRadioDriver RadioDriver;

tRadioDriver *Radio = NULL; //定义RF功能指针	

tRadioDriver* RadioDriverInit( void )
{


#if defined( USE_SX1276_RADIO )

  RadioDriver.Init = SX1276Init;
  RadioDriver.Reset = SX1276Reset;
  RadioDriver.StartCad = SX1276StartCad;
  RadioDriver.StartRx = SX1276StartRx;
  RadioDriver.GetRxPacket = SX1276GetRxPacket;
  RadioDriver.SetTxPacket = SX1276SetTxPacket;
  RadioDriver.Process = SX1276Process;

	RadioDriver.LoRaSetRFFrequency = SX1276LoRaSetRFFrequency;
	RadioDriver.LoRaGetRFFrequency = SX1276LoRaGetRFFrequency;
	RadioDriver.LoRaSetRFPower = SX1276LoRaSetRFPower;
	RadioDriver.LoRaGetRFPower = SX1276LoRaGetRFPower;
	RadioDriver.LoRaSetSignalBandwidth = SX1276LoRaSetSignalBandwidth;
	RadioDriver.LoRaGetSignalBandwidth = SX1276LoRaGetSignalBandwidth;
	RadioDriver.LoRaSetSpreadingFactor = SX1276LoRaSetSpreadingFactor;
	RadioDriver.LoRaGetSpreadingFactor = SX1276LoRaGetSpreadingFactor;
	RadioDriver.LoRaSetErrorCoding = SX1276LoRaSetErrorCoding;
	RadioDriver.LoRaGetErrorCoding = SX1276LoRaGetErrorCoding;
	RadioDriver.LoRaSetPacketCrcOn = SX1276LoRaSetPacketCrcOn;
	RadioDriver.LoRaGetPacketCrcOn = SX1276LoRaGetPacketCrcOn;
	RadioDriver.LoRaSetImplicitHeaderOn = SX1276LoRaSetImplicitHeaderOn;
	RadioDriver.LoRaGetImplicitHeaderOn = SX1276LoRaGetImplicitHeaderOn;
	RadioDriver.LoRaSetRxSingleOn = SX1276LoRaSetRxSingleOn;
	RadioDriver.LoRaGetRxSingleOn = SX1276LoRaGetRxSingleOn;
	RadioDriver.LoRaSetFreqHopOn = SX1276LoRaSetFreqHopOn;
	RadioDriver.LoRaGetFreqHopOn = SX1276LoRaGetFreqHopOn;
	RadioDriver.LoRaSetHopPeriod = SX1276LoRaSetHopPeriod;
	RadioDriver.LoRaGetHopPeriod = SX1276LoRaGetHopPeriod;
	RadioDriver.LoRaSetTxPacketTimeout = SX1276LoRaSetTxPacketTimeout;
	RadioDriver.LoRaGetTxPacketTimeout = SX1276LoRaGetTxPacketTimeout;
	RadioDriver.LoRaSetRxPacketTimeout = SX1276LoRaSetRxPacketTimeout;
	RadioDriver.LoRaGetRxPacketTimeout = SX1276LoRaGetRxPacketTimeout;
	RadioDriver.LoRaSetPayloadLength = SX1276LoRaSetPayloadLength;
	RadioDriver.LoRaGetPayloadLength = SX1276LoRaGetPayloadLength;
	RadioDriver.LoRaSetPa20dBm = SX1276LoRaSetPa20dBm;
	RadioDriver.LoRaGetPa20dBm = SX1276LoRaGetPa20dBm;
	RadioDriver.LoRaSetPaRamp = SX1276LoRaSetPaRamp;
	RadioDriver.LoRaGetPaRamp = SX1276LoRaGetPaRamp;
	RadioDriver.LoRaSetSymbTimeout = SX1276LoRaSetSymbTimeout;
	RadioDriver.LoRaGetSymbTimeout = SX1276LoRaGetSymbTimeout;
	RadioDriver.LoRaSetLowDatarateOptimize = SX1276LoRaSetLowDatarateOptimize;
	RadioDriver.LoRaGetLowDatarateOptimize = SX1276LoRaGetLowDatarateOptimize;
	RadioDriver.LoRaGetPreambleLength = SX1276LoRaGetPreambleLength;
	RadioDriver.LoRaSetPreambleLength = SX1276LoRaSetPreambleLength;
	RadioDriver.LoRaSetNbTrigPeaks = SX1276LoRaSetNbTrigPeaks;

	RadioDriver.LoRaSetOpMode = SX1276LoRaSetOpMode;
	RadioDriver.LoRaGetOpMode = SX1276LoRaGetOpMode;
	RadioDriver.Read = SX1276Read;
	RadioDriver.Write = SX1276Write;
#else
	#error "Missing define: USE_XXXXXX_RADIO (ie. USE_SX1272_RADIO)"
#endif

    return &RadioDriver;
}
