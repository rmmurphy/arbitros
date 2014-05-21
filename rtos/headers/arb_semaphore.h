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
 * File Name   : arb_semaphore.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for semaphore creation and control
 *
 * References  : 1) 'Multitasking on an AVR' Example C Implementation of a
 *                  Multitasking Kernel for the AVR, by Richard Barry
 *               2) http://www.freertos.org/
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/
#ifndef arb_semaphore_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_semaphore_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"
   #include "arb_thread.h"
   #include "utl_linkedList.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      /*---------------------------------------------------------------------*
       * If the value is less than zero than there are threads being blocked.
       * Each time 'arb_signal' gets called the count is incremented and
       * one of the blocked threads is added back on the active list - of the
       * one being added has higher priority than the current thread, then it
       * is allowed to run immediately.
       *---------------------------------------------------------------------*/
      COUNTING,
      /*---------------------------------------------------------------------*
       * Same concept as 'COUNTING' except that all the threads being blocked
       * are added back onto the active list after an event has been triggered.
       *---------------------------------------------------------------------*/
      SIGNAL,
      /*---------------------------------------------------------------------*
       * Same as a counting semaphore except its value is initialized to 1
       * which represents an unlocked condition.
       *---------------------------------------------------------------------*/
      MUTEX

   }t_semType;

   typedef enum
   {
      /*---------------------------------------------------------------------*
       * The calling process is blocked whenever the semaphore count < 0.
       *---------------------------------------------------------------------*/
      BLOCKING = 0,
      /*---------------------------------------------------------------------*
       * The calling process is never blocked/
       *---------------------------------------------------------------------*/
      NONBLOCKING,

   }t_semMode;

   typedef volatile int16_t t_SEMHANDLE; /*Semaphore handle type*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_SEMHANDLE arb_semaphoreCreate( t_semType t_type);

   t_error arb_semaphoreDestroy( t_SEMHANDLE t_semHandle);

   t_error arb_wait( t_SEMHANDLE t_semHandle,
                     t_semMode t_mode)  __attribute__ ((noinline));

   t_error arb_signal( t_SEMHANDLE t_semHandle)  __attribute__ ((noinline));

   t_error arb_semaphoreInit( t_SEMHANDLE t_semHandle,
                              int16_t s_value);

   int16_t arb_semaphoreGetCount( t_SEMHANDLE t_semHandle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_semaphore_h*/
