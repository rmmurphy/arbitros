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
 * File Name   : usr_platformTest.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : Demonstrates how to use a few of the peripherals available
 *               with the arbitros operating system.
 *
 * References  :
 *
 * Last Update : Jan, 1, 2013
 *---------------------------------------------------------------------------*/
#ifndef usr_platformTest_h

   #ifdef __cplusplus
   extern "C" {
   #endif

    /*------------------------------------------------------------------------*
     * Global Defines
     *------------------------------------------------------------------------*/
    #define usr_platformTest_h

    /*------------------------------------------------------------------------*
     * Include Files
     *------------------------------------------------------------------------*/
    #include "avr_compiler.h"
    #include "arb_mailbox.h"

    /*------------------------------------------------------------------------*
     * Global Typedefs
     *------------------------------------------------------------------------*/
    typedef enum
    {
      PLAT_UART_TEST,
      PLAT_TWI_TEST,
      PLAT_TIMER_GPIO_TEST,
      PLAT_SPI_TEST
    }t_platTestCmd;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void usr_platformTestInit( void);

   t_MAILBOXHNDL usr_getPlatTestInMailbox( void);

   t_MAILBOXHNDL usr_getPlatTestOutMailbox( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef usr_platformTest_h*/
