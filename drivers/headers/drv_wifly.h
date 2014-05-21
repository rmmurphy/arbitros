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
 * File Name   : drv_wifly.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for providing a standard API to
 *               the Roving Networks Wifly module.
 *
 * Last Update : July 8, 2013
 *---------------------------------------------------------------------------*/
#ifndef drv_wifly_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_wifly_h
   #define WIFLY_RESET_EN  (1)
   #define WIFLY_RESET_DIS (0)
   #define WIFLY_KEY_SIZE  (27)
   #define WIFLY_SSID_SIZE (25)

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
      WIFLY_NO_CMD_MODE     = -7, /*Trying to send cmd when not in cmd mode*/
      WIFLY_CMD_MODE_FAIL   = -6, /*Device failed to enter/exit cmd mode*/
      WIFLY_FAILED_TO_BOOT  = -5,
      WIFLY_INVALID_ARG     = -4, /*Invalid ioctl argument*/
      WIFLY_INVALID_CMD     = -3, /*Invalid ioctl command.*/
      WIFLY_NULL_PTR        = -2, /*Pointer is not mapped to a valid
                                      address.*/
      WIFLY_OUT_OF_HEAP     = -1, /*No more memory.*/
      WIFLY_PASSED          = 0   /*Configuration good.*/

   }t_wiflyError; /*Possible error conditions returned by the device's
                    ioctl routine*/

   typedef enum
   {
      WIFLY_BEGIN,     /*Reboots, turn off auto connect, and set the local port*/
      WIFLY_ENTER_CMD, /*Put the device into command mode.*/
      WIFLY_EXIT_CMD,  /*Take the device out of command mode.*/
      WIFLY_SEND_CMD   /*Send a command when in cmd mode*/

   }t_wiflyCmd;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Default baud rate for the device being configured.
       *---------------------------------------------------------------------*/
      uint32_t i_baudRate;

      /*---------------------------------------------------------------------*
       * The UART Id for the device being configured.
       *---------------------------------------------------------------------*/
      uint8_t c_uartId;

      /*---------------------------------------------------------------------*
       * Configures the appropriate PORT for reseting the WiFly module.
       *---------------------------------------------------------------------*/
      uint8_t c_resetPort;

      /*---------------------------------------------------------------------*
       * Configures the appropriate pin for reseting the WiFly module.
       *---------------------------------------------------------------------*/
      uint8_t c_resetPin;

      /*---------------------------------------------------------------------*
       * This value represents a unique number assigned to this driver and has
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_wiflySetup;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The secret key need to join the network given by 'ac_ssid'
       *---------------------------------------------------------------------*/
      char ac_key[WIFLY_KEY_SIZE];
      /*---------------------------------------------------------------------*
       * The name of the particular network being joined.
       *---------------------------------------------------------------------*/
      char ac_ssid[WIFLY_SSID_SIZE];
      /*---------------------------------------------------------------------*
       * The channel of the network given by 'ac_ssid'
       *---------------------------------------------------------------------*/
      uint8_t c_channel;
      /*---------------------------------------------------------------------*
       * If true, the network given by 'ac_ssid' is WEP
       *---------------------------------------------------------------------*/
      bool b_isWep;

   }t_wiflyConfig;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_wiflyInit( t_wiflySetup t_setup);

   void drv_wiflyExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_wifly_h*/
