/*---------------------------------------------------------------------------*
 * Copyright (C) 2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : hal_spi.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the SPI module.
 *
 * Last Update : June 9, 2013
 *---------------------------------------------------------------------------*/
#ifndef hal_spi_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_spi_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_gpio.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      SPI_INVALID_DEV_TYPE     = -16, /*Invalid spi device type*/
      SPI_NO_DMA               = -15, /*No DMA channels available*/
      SPI_HAL_FAILED           = -14, /*HAL function failed to initialize*/
	   SPI_ONLY_ONE_SLAVE       = -13, /*Only one slave channel per spi*/
	   SPI_BUSY                 = -12, /*Transaction in progress*/
		SPI_MODULE_CON           = -11, /*SPI module already configured*/
	   SPI_INVALID_PORT         = -10,
		SPI_INVALID_PIN          = -9,
		SPI_INVALID_ORDER        = -8,
		SPI_INVALID_OP           = -7,
		SPI_INVALID_MODE         = -6,
      SPI_INVALID_MODULE       = -5,
      SPI_INVALID_BAUD_RATE    = -4,
      SPI_NO_CONFIG            = -3,  /*SPI config hasn't been called*/
      SPI_INVALID_HANDLE       = -2,  /*Handle doesn't map to a SPI*/
      SPI_OUT_OF_HEAP          = -1,  /*No more memory.*/
      SPI_PASSED               = 0    /*Configuration good.*/
   }t_spiError;

   typedef enum
   {
      SPI_1 = 0, /*SPI C*/
      SPI_2 = 1, /*SPI D*/
      SPI_3 = 2, /*SPI E*/
      SPI_4 = 3, /*SPI F*/

   }t_spiChanId;

   typedef enum
   {
      SPI_MODE_0 = 0,
      SPI_MODE_1,
      SPI_MODE_2,
      SPI_MODE_3

   }t_spiMode;

   typedef enum
   {
      SPI_MASTER = 0,
      SPI_SLAVE

   }t_spiOperation;

   typedef enum
   {
      SPI_LSB_FIRST = 0,
      SPI_MSB_FIRST

   }t_spiDataOrder;

   typedef enum
   {
      SPI_CS_EN = 0,
      SPI_CS_DIS

   }t_spiCsCntl;

   typedef struct
   {
      t_spiMode t_spiMd;
		t_spiOperation t_spiOp;
      t_spiDataOrder t_spiOrder;
      uint32_t i_baudRate;
      bool b_enDma;

   }t_spiConfig; /*Configuration for a particular SPI channel*/

   typedef volatile int16_t t_SPIHNDL; /*Handle to a particular SPI*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This function request access to a particular spi channel.
    *------------------------------------------------------------------------*/
   t_SPIHNDL hal_requestSpiChannel( t_spiChanId t_chanId,
	                                 void (*pf_funPtr)( int8_t *pc_data,
												                   uint16_t s_length),
			                           t_gpioPort t_csPort,
		                              uint8_t c_csPin);

   /*------------------------------------------------------------------------*
    * This function releases access to a particular spi channel.
    *------------------------------------------------------------------------*/
   t_spiError hal_releaseSpiChannel( t_SPIHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Creates and configures a spi channel
    *------------------------------------------------------------------------*/
   t_spiError hal_configureSpiChannel( t_spiChanId t_chanId,
                                       t_spiConfig t_conf);

   /*------------------------------------------------------------------------*
    * Write/read a block of data to a particular spi channel.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiReadWrite( t_SPIHNDL t_handle,
                                bool b_enCs,
                                int8_t *pc_txBuffer,
								        int8_t *pc_rxBuffer,
								        uint16_t s_numBytes);

   /*------------------------------------------------------------------------*
    * Read a block of data from a particular spi channel.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiReadBlock( t_SPIHNDL t_handle,
                                bool b_enCs,
								        int8_t *pc_rxBuffer,
								        uint16_t s_numBytes);

   /*------------------------------------------------------------------------*
    * Write a block of data to a particular spi channel.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiWriteBlock( t_SPIHNDL t_handle,
                                 bool b_enCs,
								         int8_t *pc_txBuffer,
								         uint16_t s_numBytes);

   /*------------------------------------------------------------------------*
    * Manually controls the CS pin.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiSetChipSelect( t_SPIHNDL t_handle,
                                    t_spiCsCntl t_csCntl);

   /*------------------------------------------------------------------------*
    * Read a single character from a particular spi device pointed to by
    * 't_handle' - only valid if the device is configured for 'SPI_CHAR'
    * operation.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiReadByte( t_SPIHNDL t_handle,
								       int8_t *pc_rxChar);

   /*------------------------------------------------------------------------*
    * Write a single character to a particular spi device pointed to by
    * 't_handle' - only valid if the device is configured for 'SPI_CHAR'
    * operation.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiWriteByte( t_SPIHNDL t_handle,
								        int8_t c_txChar);

   /*------------------------------------------------------------------------*
    * Sets the baud rate for a particular spi channel pointed to by 't_handle'
    *------------------------------------------------------------------------*/
   t_spiError hal_spiSetBaudRate( t_SPIHNDL t_handle,
                                  uint32_t i_baudRate);

   /*------------------------------------------------------------------------*
    * This functions maps the transaction finished callback function.
    *------------------------------------------------------------------------*/
   t_spiError hal_spiConfCallBack( t_SPIHNDL t_handle,
	                                void (*pf_funPtr)( int8_t *pc_data,
												                  uint16_t s_length));

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_spi_h*/