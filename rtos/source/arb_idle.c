/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : arb_idle.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file contains the idle thread which arbitrates
 *               access to the various background tasks found on its stack.
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "arb_thread.h"
#include "arb_idle.h"
#include "arb_printf.h"
#include "arb_sysTimer.h"
#include "hal_watchDog.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*
 * If the watchdog timer isn't 'pet' within this amount of time the processor
 * is reset.
 *---------------------------------------------------------------------------*/
#define IDLE_WATCHDOG_RESET_PERIOD (8000) /*msec*/
/*---------------------------------------------------------------------------*
 * The minimum rate for 'petting' the watchdog timer.
 *---------------------------------------------------------------------------*/
#define IDLE_WATCHDOG_PET_RATE (2000)/*msec*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Idle thread handle
    *------------------------------------------------------------------------*/
   t_THRDHANDLE t_idleThrdHndl;

   /*------------------------------------------------------------------------*
    * Watchdog timer handle
    *------------------------------------------------------------------------*/
   t_WDHNDL t_wdHndle;

}t_idleObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_idleObject gt_idleObject;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void arb_idle( t_parameters t_param,
               t_arguments  t_args)
{
   uint64_t l_currM;
   uint64_t l_prevM;
   uint64_t l_delta;
   const char ac_msg[100];
   uint32_t i_counter = 0;

   /*------------------------------------------------------------------------*
    * Start the watchdog timer.
    *------------------------------------------------------------------------*/
   hal_wdEnable( gt_idleObject.t_wdHndle);

   l_prevM = 0;

   while( RUN_FOREVER)
   {
      l_currM = arb_sysMsecNow();

      l_delta = arb_sysMsecDelta( l_currM,
                                  l_prevM);

      if( l_delta >= IDLE_WATCHDOG_PET_RATE)
      {
         i_counter++;

         /*------------------------------------------------------------------*
          * Store the previous time...
          *------------------------------------------------------------------*/
         l_prevM = arb_sysMsecNow();

         sprintf( (void *)ac_msg,
                  "Petting watchdog timer %7.0f\r",
                  (double)i_counter);

         arb_printf( PRINTF_DBG_LOW | PRINTF_DBG_SHOW_TIME,
                     ac_msg);

         /*------------------------------------------------------------------*
          * Pet the watchdog timer...
          *------------------------------------------------------------------*/
         HAL_WD_RESET();

      }/*End if( i_delta >= IDLE_WATCHDOG_PET_RATE)*/

      /*---------------------------------------------------------------------*
       * Write the contents of the buffer to the registered devices.
       *---------------------------------------------------------------------*/
      arb_printfFlushBuf();

   }/*End while( RUN_FOREVER)*/

}/*End arb_idle*/

t_THRDHANDLE arb_idleInit( t_stackSize t_idlStack,
                           t_thrdPrio t_pri)
{
   t_wdConfig t_conf;

   gt_idleObject.t_idleThrdHndl = arb_threadCreate( arb_idle,
                                                    0xAA,
                                                    0xBB,
                                                    t_idlStack,
                                                    t_pri);

   if( gt_idleObject.t_idleThrdHndl < 0)
   {
      return (t_THRDHANDLE)gt_idleObject.t_idleThrdHndl;

   }/*End if( gt_idleObject.t_idleThrdHndl < 0)*/

   gt_idleObject.t_wdHndle = hal_requestWdAccess();

   if( gt_idleObject.t_wdHndle < 0)
   {
      return (t_THRDHANDLE)ARB_HAL_ERROR;
   }

   t_conf.i_period = IDLE_WATCHDOG_RESET_PERIOD; /*msec*/

   if( hal_configureWd( gt_idleObject.t_wdHndle, t_conf))
   {
     return (t_THRDHANDLE)ARB_HAL_ERROR;
   }

   return gt_idleObject.t_idleThrdHndl;

}/*End arb_idleInit*/
