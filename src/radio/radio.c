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
 * \version    2.0.0 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */
 
//#include <stdint.h>
 
#include "platform.h"

#include "displayconfigall.h"
#include "displaypal.h"
#include "displaybackend.h"

#include "sx1276.h"
#include "sx1276-LoRa.h"

#include "radio.h"

  
tRadioDriver *Radio = NULL;
const uint8_t MY_TEST_Msg[] = "Hello LoRa";
tRadioDriver RadioDriver;

tRadioDriver* RadioDriverInit( void )
{
    RadioDriver.Init = SX1276Init;
    RadioDriver.Reset = SX1276Reset;
    RadioDriver.StartRx = SX1276StartRx;
    RadioDriver.GetRxPacket = SX1276GetRxPacket;
    RadioDriver.SetTxPacket = SX1276SetTxPacket;
    RadioDriver.Process = SX1276Process;   

    return &RadioDriver;
}

void peripheral_init()
{
	/* Initialize the Platform Abstraction Layer (PAL) interface.  */
	PAL_TimerInit();
	PAL_SpiInit();
	PAL_GpioInit();

	/* Setup GPIOs */
	PAL_GpioPinModeSet(SX1276_PORT_SCK, SX1276_PIN_SCK, palGpioModePushPull, 0);
	PAL_GpioPinModeSet(SX1276_PORT_MOSI, SX1276_PIN_MOSI, palGpioModePushPull, 0);
	PAL_GpioPinModeSet(SX1276_PORT_SS, SX1276_PIN_SS, palGpioModePushPull, 0);
	PAL_GpioPinModeSet(SX1276_PORT_MISO, SX1276_PIN_MISO, palGpioModePushPull, 0);

	PAL_GpioPinModeSet(SX1276_PORT_RST, SX1276_PIN_RST, palGpioModePushPull, 0);

	PAL_GpioPinModeSet(SX1276_PORT_DIO0, SX1276_PIN_DIO0, 2, 0);

	PAL_GpioPinModeSet(SX1276_PORT_RXTX, SX1276_PIN_RXTX, palGpioModePushPull, 0);
}

void RADIO_Init()
{
	peripheral_init();

	Radio = RadioDriverInit();

	Radio->Init();

	Radio->SetTxPacket(MY_TEST_Msg, sizeof(MY_TEST_Msg));
}

void RADIO_Run()
{
	while(1)
	{
		switch(Radio->Process( ))
		{
		case RF_TX_DONE:
			printf("\r\n RF_TX_DONE \r\n");

			Delay(2000);

			break;

		default:
			break;
		}
	}
}

#if 0
void RADIO_Write()
{
	uint8_t cmd = 0x01;

	/* Set SCS */
	PAL_GpioPinOutSet( SX1276_PORT_SS, SX1276_PIN_SS );

	/* SCS setup time: min 6us */
	PAL_TimerMicroSecondsDelay(6);

	/* Send command */
	//cmd = LS013B7DH03_CMD_ALL_CLEAR | lcdPolarity;
	PAL_SpiTransmit ((uint8_t*) &cmd, 1);

	/* SCS hold time: min 2us */
	PAL_TimerMicroSecondsDelay(2);

	/* Clear SCS */
	PAL_GpioPinOutClear( SX1276_PORT_SS, SX1276_PIN_SS );

	return DISPLAY_EMSTATUS_OK;
}
#endif

