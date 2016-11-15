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
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
//#include <stdint.h>
//#include <stdbool.h> 

#include "platform.h"

#include "em_usart.h"
#include "displayconfigall.h"
#include "displaypal.h"
#include "displaybackend.h"


#if 1 //defined( USE_SX1276_RADIO )

//#include "ioe.h"
//#include "spi.h"
#include "sx1276-Hal.h"
#include "displaypal.h"



#if 0
/*!
 * SX1276 RESET I/O definitions
 */
#define RESET_IOPORT                                GPIOA
#define RESET_PIN                                   GPIO_Pin_0

/*!
 * SX1276 SPI NSS I/O definitions
 */
#define NSS_IOPORT                                  GPIOB
#define NSS_PIN                                     GPIO_Pin_6     //鍘烥PIO_Pin_15

/*!
 * SX1276 DIO pins  I/O definitions
 */
#define DIO0_IOPORT                                 GPIOA
#define DIO0_PIN                                    GPIO_Pin_10

#define DIO1_IOPORT                                 GPIOB
#define DIO1_PIN                                    GPIO_Pin_3

#define DIO2_IOPORT                                 GPIOB
#define DIO2_PIN                                    GPIO_Pin_5

#define DIO3_IOPORT                                 GPIOB
#define DIO3_PIN                                    GPIO_Pin_4

#define DIO4_IOPORT                                 GPIOA
#define DIO4_PIN                                    GPIO_Pin_9

#define DIO5_IOPORT                                 GPIOC
#define DIO5_PIN                                    GPIO_Pin_7

#define RXTX_IOPORT                                 
#define RXTX_PIN                                    FEM_CTX_PIN


#define RXE_PORT       			GPIOC
#define RXE_PIN  				GPIO_Pin_1
#define RXE_CLOCK  				RCC_APB2Periph_GPIOA
#define RXE_HIGH()         		GPIO_SetBits(RXE_PORT,RXE_PIN)
#define RXE_LOW()          		GPIO_ResetBits(RXE_PORT,RXE_PIN)
#define RXE_STATE()        		GPIO_ReadOutputDataBit(RXE_PORT,RXE_PIN)

#define TXE_PORT       			GPIOC
#define TXE_PIN  				GPIO_Pin_1
#define TXE_CLOCK  				RCC_APB2Periph_GPIOA
#define TXE_HIGH()         		GPIO_SetBits(TXE_PORT,TXE_PIN)
#define TXE_LOW()          		GPIO_ResetBits(TXE_PORT,TXE_PIN)
#define TXE_STATE()        		GPIO_ReadOutputDataBit(TXE_PORT,TXE_PIN)
#endif


#define RXE_HIGH()         		PAL_GpioPinOutSet(SX1276_PORT_RXTX, SX1276_PIN_RXTX) //GPIO_SetBits(RXE_PORT,RXE_PIN)
#define RXE_LOW()          		PAL_GpioPinOutClear(SX1276_PORT_RXTX, SX1276_PIN_RXTX) //GPIO_ResetBits(RXE_PORT,RXE_PIN)
#define TXE_HIGH()         		PAL_GpioPinOutSet(SX1276_PORT_RXTX, SX1276_PIN_RXTX) //GPIO_SetBits(TXE_PORT,TXE_PIN)
#define TXE_LOW()          		PAL_GpioPinOutClear(SX1276_PORT_RXTX, SX1276_PIN_RXTX) //GPIO_ResetBits(TXE_PORT,TXE_PIN)

void Set_RF_Switch_RX(void)
{
	RXE_HIGH();
	TXE_LOW();
}

void Set_RF_Switch_TX(void)
{
	RXE_LOW();
	TXE_HIGH();
}


void SX1276InitIo( void )
{
	#if 0
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                            RCC_AHBPeriph_GPIOC, ENABLE );
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    
    
     GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
    GPIO_InitStructure.GPIO_Pin = NSS_PIN;
    GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );
           
    GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );
	GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );

	
	//闁板秶鐤嗙亸鍕暥瀵拷閸忓疇濮遍悧鍥ㄥ付閸掑墎顓搁懘锟�   RXE-->CTRL   TXE--> /CTRL
	GPIO_InitStructure.GPIO_Pin = RXE_PIN;
	GPIO_Init(RXE_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = TXE_PIN;
	GPIO_Init(TXE_PORT, &GPIO_InitStructure);	
	//姒涙顓荤拋鎹愵啎缂冾喕璐熼幒銉︽暪閻樿埖锟斤拷
	Set_RF_Switch_RX();	
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

    // Configure DIO0
    GPIO_InitStructure.GPIO_Pin =  DIO0_PIN;
    GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
    
    // Configure DIO1
    GPIO_InitStructure.GPIO_Pin =  DIO1_PIN;
    GPIO_Init( DIO1_IOPORT, &GPIO_InitStructure );
    
    // Configure DIO2
    GPIO_InitStructure.GPIO_Pin =  DIO2_PIN;
    GPIO_Init( DIO2_IOPORT, &GPIO_InitStructure );
    
    // REAMARK: DIO3/4/5 configured are connected to IO expander

    // Configure DIO3 as input
    GPIO_InitStructure.GPIO_Pin =  DIO3_PIN;
    GPIO_Init( DIO3_IOPORT, &GPIO_InitStructure );
    // Configure DIO4 as input
    GPIO_InitStructure.GPIO_Pin =  DIO4_PIN;
    GPIO_Init( DIO4_IOPORT, &GPIO_InitStructure );
    // Configure DIO5 as input
	GPIO_InitStructure.GPIO_Pin =  DIO5_PIN;
    GPIO_Init( DIO5_IOPORT, &GPIO_InitStructure );
	#endif
}

void SX1276SetReset( uint8_t state )
{
	if( state == RADIO_RESET_ON )
	{
		// Set RESET pin to 0
		PAL_GpioPinOutClear(SX1276_PORT_RST, SX1276_PIN_RST);
	}
	else
	{
		// Set RESET pin to 1
		PAL_GpioPinOutSet(SX1276_PORT_RST, SX1276_PIN_RST);
	}

	#if 0
    GPIO_InitTypeDef GPIO_InitStructure;

    if( state == RADIO_RESET_ON )
    {
        // Configure RESET as output
		GPIO_InitStructure.GPIO_Pin = RESET_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
		
		// Set RESET pin to 0
        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_RESET );
    }
    else
    {
		GPIO_InitStructure.GPIO_Pin =  RESET_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
		
		// Set RESET pin to 1
        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );

    }
	#endif
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    uint8_t cmd;

    //NSS = 0;
    //GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
    PAL_GpioPinOutClear(SX1276_PORT_SS, SX1276_PIN_SS);

    //PAL_SpiInOut( addr | 0x80 );
    cmd = addr | 0x80;
    PAL_SpiTransmit(&cmd, 1);
    for( i = 0; i < size; i++ )
    {
        //PAL_SpiInOut( buffer[i] );
    	PAL_SpiTransmit(&buffer[i], 1);
    }

    //NSS = 1;
    //GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
    PAL_GpioPinOutSet(SX1276_PORT_SS, SX1276_PIN_SS);
}


void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    uint8_t cmd;
    uint8_t tmp = 0;

    //NSS = 0;
    //GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );
    PAL_GpioPinOutClear(SX1276_PORT_SS, SX1276_PIN_SS);

	cmd = addr & 0x7F;
	PAL_SpiTransmit(&cmd, 1);
    for( i = 0; i < size; i++ )
    {
        buffer[i] = PAL_SpiTransfer(0); //PAL_Receive( 0 );
    }

    //NSS = 1;
    //GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
    PAL_GpioPinOutSet(SX1276_PORT_SS, SX1276_PIN_SS);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}



#endif // USE_SX1276_RADIO
