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
 * \file       sx1276-Hal.h
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

//#include "ioe.h"
#include "stdint.h"

//#define GET_TICK_COUNT( )                           ( TickCounter )
//#define TICK_RATE_MS( ms )                          ( ms )

#define SX1276_PORT_SCK             (4)  /* = gpioPortE */
#define SX1276_PIN_SCK             (12)

#define SX1276_PORT_MOSI               (4)  /* = gpioPortE */
#define SX1276_PIN_MOSI               (10)

#define SX1276_PORT_MISO               (4)  /* = gpioPortE */
#define SX1276_PIN_MISO               (11)

#define SX1276_PORT_SS              (4)  /* = gpioPortE */
#define SX1276_PIN_SS              (13)

#define SX1276_PORT_RST              (2)  /* = gpioPortC */
#define SX1276_PIN_RST              (0)

#define SX1276_PORT_DIO0            (3) /* = gpioPortD */
#define SX1276_PIN_DIO0             (6)

#define SX1276_PORT_RXTX            (2) /* = gpioPortC */
#define SX1276_PIN_RXTX             (2)

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

/*!
 * \brief Initializes the radio interface I/Os
 */
void SX1276InitIo( void );

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void SX1276SetReset( uint8_t state );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write( uint8_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void SX1276Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );


void Set_RF_Switch_RX(void);
void Set_RF_Switch_TX(void);

#endif //__SX1276_HAL_H__
