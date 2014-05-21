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
 * File Name   : drv_console.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This driver is responsible for reading and writing to a
 *               specific uart designated for use as a console interface. The
 *               function defined herein are callable from 'thread' space.
 *
 * Last Update : Nov 11, 2011
 *---------------------------------------------------------------------------*/
#ifndef drv_console_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_console_h
   #define CONSOLE_MAX_TOKENS     (7)
   #define CONSOLE_MAX_TOKEN_SIZE (16)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_uart.h"

    /*------------------------------------------------------------------------*
     * Global Typedefs
     *------------------------------------------------------------------------*/
   typedef enum
   {
      CONSOLE_INVALID_COLOR   = -7, /*Invalid color combination*/
      CONSOLE_TOO_MANY_TOKENS = -6, /*Too many tokens entered*/
      CONSOLE_TOKEN_TOO_LARGE = -5, /*An individual token was too large*/
      CONSOLE_INVALID_ARG     = -4, /*Invalid ioctl argument*/
      CONSOLE_INVALID_CMD     = -3, /*Invalid ioctl command.*/
      CONSOLE_NULL_PTR        = -2, /*Pointer is not mapped to a valid
                                      address.*/
      CONSOLE_OUT_OF_HEAP     = -1, /*No more memory.*/
      CONSOLE_PASSED          = 0   /*Configuration good.*/

   }t_consoleError; /*Possible error conditions returned by the device's
                      ioctl routine*/

   typedef enum
   {
      CONSOLE_GET_RX_BUFFER_SIZE = 0,
      CONSOLE_SET_BAUD_RATE,
      CONSOLE_GET_BAUD_RATE,
      CONSOLE_PARSE_CMD_LINE,
      CONSOLE_DISPLAY_PROMPT,
      CONSOLE_SET_PROMPT,
      CONSOLE_RESET_TERMINAL,
      CONSOLE_SET_BG_COLOR,
      CONSOLE_SET_PROMPT_COLOR,
      CONSOLE_SET_FG_COLOR
   }t_consoleCmd;

   typedef enum
   {
      CONSOLE_BLACK   = '0',
      CONSOLE_RED     = '1',
      CONSOLE_GREEN   = '2',
      CONSOLE_YELLOW  = '3',
      CONSOLE_BLUE    = '4',
      CONSOLE_MAGENTA = '5',
      CONSOLE_CYAN    = '6',
      CONSOLE_WHITE   = '7'
   }t_consoleColor;

   /*------------------------------------------------------------------------*
    * This data type defines how data is returned from the device when
    * reading the receive buffer via the command 'CONSOLE_PARSE_RX_BUFFER'.
    *------------------------------------------------------------------------*/
   typedef struct
   {
      int8_t ac_tok[CONSOLE_MAX_TOKENS][CONSOLE_MAX_TOKEN_SIZE];
      uint8_t c_numTokens;
   }t_consoleTokHndl;

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
       * This value represents a unique number assigned to this driver and has
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_consoleSetup;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_consoleInit( t_consoleSetup t_setup);

   void drv_consoleExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_console_h*/
