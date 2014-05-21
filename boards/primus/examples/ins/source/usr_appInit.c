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
 * File Name   : usr_appInit.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for initializing the arbitrOS
 *               scheduler, any drivers needed by the user application, and
 *               all user-space threads.
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include "arb_scheduler.h"
#include "arb_printf.h"
#include "arb_console.h"
#include "usr_console.h"
#include "usr_navigation.h"
#include "drv_signal.h"
#include "drv_console.h"
#include "drv_ins.h"
#include "drv_sd.h"
/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void usr_driverRegistration( void)
{
   t_signalSetup t_sigSetup;
   t_consoleSetup t_conSetup;
   //t_sdSetup t_sdCardSetup;

   /*------------------------------------------------------------------------*
    * Register all drivers needed by the application here...
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Register the console driver with the kernel.
    *------------------------------------------------------------------------*/
   /*------------------------------------------------------------------------*
    * Register the console driver with the kernel.
    *------------------------------------------------------------------------*/
   t_conSetup.c_uartId   = UART_1;
   t_conSetup.i_baudRate = 115200;
   t_conSetup.c_majorNum = 1;
   if( drv_consoleInit( t_conSetup) < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Register the LED signaling driver with the kernel.
    *------------------------------------------------------------------------*/
   t_sigSetup.c_signalPort  = GPIO_PORTK;
   t_sigSetup.ac_ledPins[0] = PIN_0; /*Red LED pin*/
   t_sigSetup.ac_ledPins[1] = PIN_1; /*Yellow LED pin*/
   t_sigSetup.ac_ledPins[2] = PIN_2; /*Green LED pin*/
   t_sigSetup.c_majorNum    = 2;

   if( drv_signalInit( t_sigSetup) < 0)
      exit(0);

   /*------------------------------------------------------------------------*
    * Register the INS driver with the kernel. The driver uses a TWI on
    * PIN_0 and PIN_1 (of PORT D).
    *------------------------------------------------------------------------*/
   if( drv_insInit() < 0)
      exit(0);

   t_conSetup.c_uartId   = UART_1;
   t_conSetup.i_baudRate = 1000000;
   t_conSetup.c_majorNum = 1;

   /*------------------------------------------------------------------------*
    * Register the sd card driver with the kernel.
    *------------------------------------------------------------------------*/
   //t_sdCardSetup.t_csPort   = GPIO_PORTC;
   //t_sdCardSetup.t_csPin    = PIN_4;
   //t_sdCardSetup.t_spiChan  = SPI_1;
   //t_sdCardSetup.c_majorNum = 5;

   //if( drv_sdInit( t_sdCardSetup) < 0)
   //   exit(0);

}/*End usr_driverRegistration*/

static void usr_platformInit( void)
{
   /*------------------------------------------------------------------------*
    * Register drivers with the kernel here...
    *------------------------------------------------------------------------*/
   usr_driverRegistration();

   /*------------------------------------------------------------------------*
    * Initialize the debug interface.
    *------------------------------------------------------------------------*/
   if( arb_printfInit( "consoleDevice0",
                       300,
                       NULL) < 0)//"sdDevice0/logs/dmsg.txt") < 0)
   {
      exit(0);
   }

   /*------------------------------------------------------------------------*
    * Initialize the console interface.
    *------------------------------------------------------------------------*/
   if( arb_consoleInit( "consoleDevice0",
                        "sdDevice0",
                        ARB_STACK_1024B,
                        254,
                        &usr_console) < 0)
   {
      exit(0);
   }

   /*------------------------------------------------------------------------*
    * Initialize the scheduler specifically for this application.
    *------------------------------------------------------------------------*/
   if( arb_schedulerInit( PRIORITY,
                          ARB_STACK_384B,
                          TIMER_7) < 0)
   {
      exit(0);
   }

}/*End usr_platformInit*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void usr_appInit( void)
{

   /*------------------------------------------------------------------------*
    * Enable global interrupts, configure the CPU frequency, and register
    * drivers with the kernel for this particular computing platform.
    *------------------------------------------------------------------------*/
   usr_platformInit();

   usr_navigationInit();

   /*------------------------------------------------------------------------*
    * Once we have exited this function the OS will begin running.
    *------------------------------------------------------------------------*/

}/*End usr_appInit*/
