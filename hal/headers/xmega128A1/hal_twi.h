/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : hal_twi.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the TWI module.
 *
 * Last Update : Jan 30, 2012
 *---------------------------------------------------------------------------*/
#ifndef hal_twi_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_twi_h

   /*------------------------------------------------------------------------*
    * Inlude Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      TWI_OPERATION_FAILED     = -9, /*The read or write operation failed*/
      TWI_CHAN_NOT_MASTER      = -8, /*The twi channel is not a master*/
      TWI_INVALID_SLAVE_ADDR   = -7, /*Slave address can be 7-bits only*/
      TWI_INVALID_MODE         = -6, /*Not master or slave*/
      TWI_INVALID_CHAN         = -5, /*Invalid TWI channel ID*/
      TWI_INVALID_BAUD_RATE    = -4, /*Invalid baud rate*/
      TWI_CHAN_UNAVAILABLE     = -3, /*Channel is already open*/
      TWI_INVALID_HANDLE       = -2, /*Handle doesn't map to a TWI*/
      TWI_OUT_OF_HEAP          = -1, /*No more memory.*/
      TWI_PASSED               = 0   /*Configuration good.*/

   }t_twiError;

   typedef enum
   {
      TWI_1 = 0, /*TWI C*/
      TWI_2,     /*TWI D*/
      TWI_3,     /*TWI E*/
      TWI_4,     /*TWI F*/

   }t_twiChanId;

   typedef enum
   {
      TWI_MASTER = 0,
      TWI_SLAVE

   }t_twiMode;

   typedef enum
   {
      TWI_TRANSACTION_BUSY     = 0, /*Performing a read/write transaction*/
      TWI_NACK_RECEIVED,
      TWI_BUS_ERROR,
      TWI_ARB_LOST,
      TWI_TRANS_COMPLETE,           /*Read/Write operation completed*/
      TWI_IDLE,
      TWI_BUS_BUSY,
      TWI_SLAVE_WRITE,              /*Master writing data to the slave*/
      TWI_SLAVE_READ,               /*Master reading data from slave*/
      TWI_COLLISION,                /*Slave bus collision detected*/
      TWI_BUS_LOCKED,               /*The bus became locked during transaction*/
      TWI_UNKNOWN                   /*Bad TWI handle*/

   }t_twiStatus;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * TWI mode either TWI_MASTER or TWI_SLAVE
       *---------------------------------------------------------------------*/
      t_twiMode t_mode;

      /*---------------------------------------------------------------------*
       * TWI master mode baud rate
       *---------------------------------------------------------------------*/
      uint32_t  i_baud;

      /*---------------------------------------------------------------------*
       * Seven bit slave address
       *---------------------------------------------------------------------*/
      uint8_t c_slaveAddress;

      /*---------------------------------------------------------------------*
       * Pointer to the slave transaction complete function which is
       * used for signaling an end-of-transaction or for reading/writing data
       * to and from a slave device.
       *---------------------------------------------------------------------*/
      int8_t (*pf_transComplete)( t_twiStatus t_status, int8_t c_data);

      /*---------------------------------------------------------------------*
       * If true, the TWI master interface exchanges data by polling the status
       * register, otherwise it is interrupt driven. The polling method
       * should be used when the twi is called from within an interrupt.
       *---------------------------------------------------------------------*/
      //bool b_mastPolling;

   }t_twiConfig; /*Configuration for a particular TWI channel*/

   typedef volatile int16_t t_TWIHNDL; /*Handle to a particular TWI*/

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
    * This function request access to the next available DMA channel.
    *------------------------------------------------------------------------*/
   t_TWIHNDL hal_requestTwiChannel( t_twiChanId t_chanId);

   /*------------------------------------------------------------------------*
    * This function releases access over a particular DMA channel.
    *------------------------------------------------------------------------*/
   t_twiError hal_releaseTwiChannel( t_TWIHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Configures a TWI for the channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_twiError hal_configureTwiChannel( t_TWIHNDL t_handle,
                                       t_twiConfig t_conf);

   /*------------------------------------------------------------------------*
    * Write the data pointed to by 'pc_data' to the DATA register of the
    * particular twi channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_twiError hal_twiMasterWrite( t_TWIHNDL t_handle,
                                  uint8_t *pc_data,
                                  uint16_t s_length,
                                  uint8_t c_slaveAdd,
                                  int8_t c_numRetries);

   /*------------------------------------------------------------------------*
    * Write a single byte to the DATA register of the particular twi
    * channel pointed to by 't_handle'.
    *------------------------------------------------------------------------*/
   t_twiError hal_twiMasterRead( t_TWIHNDL t_handle,
                                 uint8_t *pc_data,
                                 uint16_t s_length,
                                 uint8_t c_slaveAdd,
                                 int8_t c_numRetries);

   /*------------------------------------------------------------------------*
    * Return the current baud rate for a particular twi channel.
    *------------------------------------------------------------------------*/
   int32_t hal_twiGetBaudRate( t_TWIHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Return the current status for an ongoing transaction.
    *------------------------------------------------------------------------*/
   t_twiStatus hal_getTwiStatus( t_TWIHNDL t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_twi_h*/
