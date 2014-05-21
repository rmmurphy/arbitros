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
 * File Name   : arb_sysTimer.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for handling all OS system timer
 *               functionality
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/
#ifndef arb_sysTimer_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_timer.h"
   #include "arb_error.h"
   #include "hal_pmic.h"
   #include "hal_clocks.h"

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_sysTimer_h
   /*------------------------------------------------------------------------*
    * The frequency of the timer used to driver the arbitros system timer.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_TICK_RATE       (500000) /*ticks/sec*/
   /*------------------------------------------------------------------------*
    * The default period (in seconds) of the arbitros system timer.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_PERIOD          (.01f)  /*seconds*/
   /*------------------------------------------------------------------------*
    * The default period (in useconds) of the arbitros system timer.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_PERIOD_IN_USEC  (ARB_SYS_TIMER_PERIOD*1000000)
   /*------------------------------------------------------------------------*
    * The default period (in msec) of the arbitros system timer.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_PERIOD_IN_MSEC  (ARB_SYS_TIMER_PERIOD*1000)
   /*------------------------------------------------------------------------
    * The number of timer clock ticks in one system timer period.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_PERIOD_IN_TICKS (uint16_t)(ARB_SYS_TIMER_PERIOD*\
   ARB_SYS_TIMER_TICK_RATE  - 1)
   /*------------------------------------------------------------------------*
    * The number of useconds in one tick of the timer used to driver the
    * arbitros system timer.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_USEC_PER_TICK   (1000000 / ARB_SYS_TIMER_TICK_RATE)
   /*------------------------------------------------------------------------*
    * The number of arbitros system timer ticks in one second.
    *------------------------------------------------------------------------*/
   #define ARB_TICKS_PER_SECOND (uint16_t)(1.0f / ARB_SYS_TIMER_PERIOD)
   /*------------------------------------------------------------------------*
    * The maximum 'l_msec' is allowed to count before rolling over.
    *------------------------------------------------------------------------*/
   #define ARB_SYS_TIMER_MAX_MSEC ((uint64_t)3600000)

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef struct
   {
      /*---------------------------------------------------------------------*
       * Continuous timers, time to rollover = (2^64-1)/1000 seconds and
       * (2^64-1)/ARB_SYS_TIMER_TICK_RATE seconds
       *---------------------------------------------------------------------*/
      uint64_t l_msec;
      uint64_t l_ticks; /*1 tick represents 1 period of the system timer*/
      /*---------------------------------------------------------------------*
       * TOD timer
       *---------------------------------------------------------------------*/
      uint16_t s_days;  /*Elapsed days*/
      uint8_t  c_hours; /*Elapsed hours*/
      uint8_t  c_min;   /*Elapsed minutes*/
      uint8_t  c_sec;   /*Elapsed seconds*/
      uint32_t i_usec;  /*Elapsed useconds*/

   }t_sysTime;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/
   extern t_sysTime gt_sysTime;

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void arb_sysTimerInit( t_timerModId t_timerId);
   t_sysTime arb_sysTimeNow( void);
   void arb_resetSysTime( void);
   void arb_restartSysTimer( void);
   t_error arb_sysTimerStart( void);
   void arb_sysTimerStop( void);
   void arb_setSysTime( uint8_t c_hours,
                        uint8_t c_min,
                        uint8_t c_sec);
   uint64_t arb_sysMsecNow( void);
   uint64_t arb_sysMsecDelta( uint64_t l_currTime,
                              uint64_t l_prevTime);
   bool arb_sysTimerEnabled( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_sysTimer_h*/
