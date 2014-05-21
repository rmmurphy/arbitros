/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : drv_arbComm.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for communicating between arbitros 
 *               platforms.
 *
 * Last Update : Jan 19, 2014
 *---------------------------------------------------------------------------*/
#ifndef drv_arbComm_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_arbComm_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "utl_uart.h"

    /*------------------------------------------------------------------------*
     * Global Typedefs
     *------------------------------------------------------------------------*/
   typedef enum
   {
      ARBCOMM_NOT_ENOUGH_DATA = -5, /*Buffer level low*/
      ARBCOMM_INVALID_ARG     = -4, /*Invalid ioctl argument*/
      ARBCOMM_INVALID_CMD     = -3, /*Invalid ioctl command.*/
      ARBCOMM_NULL_PTR        = -2, /*Pointer is not mapped to a valid
                                      address.*/
      ARBCOMM_OUT_OF_HEAP     = -1, /*No more memory.*/
      ARBCOMM_PASSED          = 0   /*Configuration good.*/

   }t_arbCommError; /*Possible error conditions returned by the device's
                      ioctl routine*/

   typedef enum
   {
      ARBCOMM_GET_RX_BUFFER_SIZE = 0,
      ARBCOMM_GET_RX_BUFFER_LEVEL,
      ARBCOMM_SET_BAUD_RATE,
      ARBCOMM_GET_BAUD_RATE
   }t_arbCommCmd;

   typedef struct
   {

      uint16_t s_rxBuffSize;

      /*---------------------------------------------------------------------*
       * Default baud rate for the device being configured.
       *---------------------------------------------------------------------*/
      uint32_t i_baudRate;

      /*---------------------------------------------------------------------*
       * The UART Id for the device being configured.
       *---------------------------------------------------------------------*/
      uint8_t c_uartId;

      /*---------------------------------------------------------------------*
       * This value represents a unique number assigned to this driver and has
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_arbCommSetup;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_arbCommInit( t_arbCommSetup t_setup);

   void drv_arbCommExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_arbComm_h*/