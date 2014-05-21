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
 * File Name   : arb_scheduler.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for thread scheduling
 *
 * References  : 1) 'Multitasking on an AVR' Example C Implementation of a
 *                  Multitasking Kernel for the AVR, by Richard Barry
 *               2) http://www.freertos.org/
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/
#ifndef arb_scheduler_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_scheduler_h
   #define ARB_LOAD_EST_Q_FACT ((uint16_t)15)
   #define ARB_LOAD_EST_ONE    ((uint16_t)1<<ARB_LOAD_EST_Q_FACT)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_thread.h"
   #include "arb_error.h"
   #include "hal_timer.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      ROUNDROBIN = 0,
      PRIORITY
   }t_schedulerType;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/
   extern t_tcb *gpt_activeThread; /*The active thread on the system.*/

   extern void (*gpt_scheduler)(void);

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * The naked attribute prevents the compiler from performing a context
    * save since we will be doing that with the call to arb_contextRestore.
    *------------------------------------------------------------------------*/
   t_error arb_schedulerInit( t_schedulerType t_schedType,
                              t_stackSize t_idlStack,
                              t_timerModId t_timerId);

   void arb_schedulerStart( void);

   uint32_t arb_getOneMinLoadingEst( void);

   uint32_t arb_getFiveMinLoadingEst( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_scheduler_h*/
