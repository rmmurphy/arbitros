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
 * File Name   : arb_printf.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for opening handles and providing a
 *               standard interface to all the available debug ports on the
 *               system.
 *
 * Last Update : Feb 25, 2012
 *---------------------------------------------------------------------------*/
#ifndef arb_printf_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_printf_h
   /*------------------------------------------------------------------------*
    * Print low priority debug message to the terminal
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_LOW           (0x1)
   /*------------------------------------------------------------------------*
    * Print medium priority debug message to the terminal
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_MED           (0x2)
   /*------------------------------------------------------------------------*
    * Print high priority debug message to the terminal
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_HIGH          (0x4)
   /*------------------------------------------------------------------------*
    * All terminal messaging is disabled.
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_OFF           (0x20)
   /*------------------------------------------------------------------------*
    * Print the current time the message arrived.
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_SHOW_TIME     (0x40)
   /*------------------------------------------------------------------------*
    * Mask for looking only at the priority bit section
    *------------------------------------------------------------------------*/
   #define PRINTF_DBG_PRIORITY_MASK (0x07)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error arb_printfInit( char *pc_driver,
                           int16_t s_bufSize,
                           char *pc_logFile);

   void arb_printf( uint8_t c_flags, 
                    const char *pc_buff);

   void arb_printfFlushBuf( void);

   t_error arb_setPrintfDbgLevel( uint8_t c_dbg);

   uint8_t arb_getPrintfDbgLevel( void);

   void arb_sysPrintChar( const char c_buff);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_printf_h*/
