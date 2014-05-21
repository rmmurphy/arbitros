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
 * File Name   : arb_error.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file defines the set of possible OS errors.
 *
 * Last Update : June, 12, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arb_error.h"
#include "arb_thread.h"
#include "arb_printf.h"
#include "arb_sysTimer.h"
#include "arb_scheduler.h"
#include "hal_pmic.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void arb_stackOverflow( void)
{
   static int8_t ac_buff[30];

   /*------------------------------------------------------------------------*
    * If you have reached this point, a thread's stack has overflowed. This
    * function can never return.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL( );

   arb_setPrintfDbgLevel( PRINTF_DBG_HIGH);

   sprintf( (char *)ac_buff,
            "\n\rStack Overflow Thread = %d\n\r",
            gpt_activeThread->c_id);

   arb_printf( PRINTF_DBG_HIGH | PRINTF_DBG_SHOW_TIME,
               (char *)ac_buff);

   /*------------------------------------------------------------------------*
    * Write the contents of the buffer to the registered devices.
    *------------------------------------------------------------------------*/
   arb_printfFlushBuf();
      
   exit(0);

   HAL_END_CRITICAL( );

}/*End arb_stackOverflow*/
