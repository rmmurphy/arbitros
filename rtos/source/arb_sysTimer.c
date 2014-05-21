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
 * File Name   : arb_sysTimer.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for handling all OS system timer
 *               functionality
 *
 * Last Update : May, 25, 2011
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "arb_sysTimer.h"
#include "arb_thread.h"
#include "arb_scheduler.h"
#include "hal_timer.h"
#include "arb_printf.h"
#include "hal_pmic.h"
#include "hal_gpio.h"
#include "hal_contextSwitch.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Global Variables
 *---------------------------------------------------------------------------*/
t_sysTime gt_sysTime = {0,0,0,0,0};

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void updateSystemTime( void);
static void sysTimerInterrupt( void) __attribute__ ( ( naked, noinline ) );

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_TIMERHNDL t_sysTimerHandle;
static t_tcb *gpt_temp;
static bool gb_sysTimerEnabled = false;
static bool gb_updateSysTime = true;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void updateSystemTime( void)
{
   gt_sysTime.l_ticks += ARB_SYS_TIMER_PERIOD_IN_TICKS;
   gt_sysTime.i_usec += ARB_SYS_TIMER_PERIOD_IN_USEC;
   gt_sysTime.l_msec += ARB_SYS_TIMER_PERIOD_IN_MSEC;

   if( gt_sysTime.l_msec >= ARB_SYS_TIMER_MAX_MSEC)
      gt_sysTime.l_msec -= ARB_SYS_TIMER_MAX_MSEC;

   if( gt_sysTime.i_usec >= ((uint32_t)1000000))
   {
      gt_sysTime.i_usec -= ((uint32_t)1000000);
      gt_sysTime.c_sec++;
      if( gt_sysTime.c_sec == 60)
      {
         gt_sysTime.c_sec = 0;
         gt_sysTime.c_min++;
         if( gt_sysTime.c_min == 60)
         {
            gt_sysTime.c_min = 0;
            gt_sysTime.c_hours++;
            if( gt_sysTime.c_hours == 24)
            {
               gt_sysTime.c_hours = 0;
               gt_sysTime.s_days++;
            }/*End if( gt_sysTime.s_hours == 24)*/

         }/*End if( gt_sysTime.c_min == 60)*/

      }/*End if( gt_sysTime.c_sec == 60)*/

   }/*End if( gt_sysTime.i_usec >= 1000000)*/

}/*End updateSystemTime*/

static void sysTimerInterrupt( void)
{

   /*------------------------------------------------------------------------*
    * Make sure all interrupts are disabled.
    *------------------------------------------------------------------------*/
   HAL_CLI();

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers onto the current
    * thread's TCB. Since we disabled interrupts previously this call will
    * save the SREG with the GIE bit enabled.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOn();

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();
//PORTF.OUTTGL = PIN_3;
   /*------------------------------------------------------------------------*
    * Check to see if this thread overflowed its stack by checking the
    * secret number we inserted at the end of the stack.
    *------------------------------------------------------------------------*/
   gpt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
   if( gpt_temp->ps_stack[0] != ARB_STACK_OVERFLOW_CHECK)
   {
      arb_stackOverflow();
   }

   /*------------------------------------------------------------------------*
    * Change the status of this thread from active to READY.
    *------------------------------------------------------------------------*/
   gpt_temp->t_status = READY;

   /*------------------------------------------------------------------------*
    * All function calls within the ISR must be made WITHOUT the 'naked'
    * syntax so that the use of the Y pointer (or frame pointer) is properly
    * handled.
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Update the system timer tick
    *------------------------------------------------------------------------*/
   if( gb_updateSysTime == true)
      updateSystemTime();

   /*------------------------------------------------------------------------*
    * Make sure the system time is updated on each call unless a timer
    * overflow interrupt occurred during a call to 'arb_sysTimeNow'.
    *------------------------------------------------------------------------*/
   gb_updateSysTime = true;

   /*------------------------------------------------------------------------*
    * Decrement the quantum counter for each of the sleeping threads.
    *------------------------------------------------------------------------*/
   arb_updateSleepingThreads();

   /*------------------------------------------------------------------------*
    * See if there is a higher priority thread ready to run.
    *------------------------------------------------------------------------*/
   gpt_scheduler();

   /*------------------------------------------------------------------------*
    * Estimate the average system loading.
    *------------------------------------------------------------------------*/
   //arb_updateLoadingEst();

   /*------------------------------------------------------------------------*
    * If a higher priority task is ready, restore its stack and CPU registers
    * or restore the stack and CPU registers for the thread that was
    * preempted.
    *------------------------------------------------------------------------*/
   hal_contextRestore();

   /**************************************************************************
    * WARNING - performing any calculation at this point will change the
    * contents of the next thread's CPU registers.
    **************************************************************************/

   /*------------------------------------------------------------------------*
    * After performing the context restore the stack ptr is pointing to the
    * location where the thread's program counter last left off. Issuing a
    * 'ret' or 'reti' cmd will cause this location to be popped off the stack
    * and stored into the CPUs program counter where the last line of code the
    * thread was running at the time of preemption will be executed. Unlike
    * other AVR's using a reti DOES NOT set the GIE bit in the status register
    * (SREG), rather it returns the PMIC to the state it had before enter-
    * ing the interrupt - which means untoggling the '.... Interrupt Executing
    * flag'.
    *------------------------------------------------------------------------*/
   HAL_RETI();

}/*End sysTimerInterrupt*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
uint64_t arb_sysMsecNow( void)
{
   return gt_sysTime.l_msec;
}/*End arb_sysMsecNow*/

uint64_t arb_sysMsecDelta( uint64_t l_currTime,
                           uint64_t l_prevTime)
{
   int64_t l_delta;

   l_delta = (int64_t)l_currTime - (int64_t)l_prevTime;

   if( l_delta < 0)
      l_delta += ((int64_t)ARB_SYS_TIMER_MAX_MSEC);

   return (uint64_t)l_delta;

}/*End arb_sysMsecDelta*/

bool arb_sysTimerEnabled( void)
{
   return gb_sysTimerEnabled;
}/*End arb_sysTimerEnabled*/

t_sysTime arb_sysTimeNow( void)
{
   uint16_t s_curTick   = 0;
   t_sysTime t_currTime = {0,0,0,0,0};
   uint32_t i_temp;

   /*------------------------------------------------------------------------*
    * Momentarily disable interrupts while we read the system time
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * Has the timer period expired?
    *------------------------------------------------------------------------*/
   if( hal_getIntStatus( t_sysTimerHandle, OVERFLOW) > 0)
   {
      /*---------------------------------------------------------------------*
       * Interrupt occurred while reading the system timer, therefore since
       * the system will vector to the interrupt routine only allow one update
       * of the system time.
       *---------------------------------------------------------------------*/
      gb_updateSysTime = false;
      updateSystemTime();
   }

   /*------------------------------------------------------------------------*
    * Read the timer register
    *------------------------------------------------------------------------*/
   s_curTick = hal_getTimerCount( t_sysTimerHandle);

   /*------------------------------------------------------------------------*
    * Get current time snapshot
    *------------------------------------------------------------------------*/
   t_currTime.l_ticks = gt_sysTime.l_ticks + (uint64_t)s_curTick;
   t_currTime.c_sec   = gt_sysTime.c_sec;
   t_currTime.c_min   = gt_sysTime.c_min;
   t_currTime.c_hours = gt_sysTime.c_hours;
   t_currTime.s_days  = gt_sysTime.s_days;
   i_temp = (uint32_t)s_curTick*(uint32_t)ARB_SYS_TIMER_USEC_PER_TICK;
   t_currTime.i_usec  = gt_sysTime.i_usec + i_temp;
   t_currTime.l_msec  = gt_sysTime.l_msec + (uint64_t)(i_temp / 1000);

   /*------------------------------------------------------------------------*
    * Re-enable interrupts
    *------------------------------------------------------------------------*/
   HAL_END_CRITICAL();

   return t_currTime;

}/*End arb_sysTimeNow*/

void arb_setSysTime( uint8_t c_hours,
                     uint8_t c_min,
                     uint8_t c_sec)
{

   /*------------------------------------------------------------------------*
    * Momentarily disable interrupts while we set the system time
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   gt_sysTime.l_ticks = 0;
   gt_sysTime.l_msec  = 0;
   gt_sysTime.c_sec   = c_sec;
   gt_sysTime.c_min   = c_min;
   gt_sysTime.c_hours = c_hours;
   gt_sysTime.s_days  = 0;

   /*------------------------------------------------------------------------*
    * Re-enable interrupts
    *------------------------------------------------------------------------*/
   HAL_END_CRITICAL();

}/*End arb_setSysTime*/

void arb_resetSysTime( void)
{
   gt_sysTime.l_ticks = 0;
   gt_sysTime.l_msec  = 0;
   gt_sysTime.c_sec   = 0;
   gt_sysTime.c_min   = 0;
   gt_sysTime.c_hours = 0;
   gt_sysTime.s_days  = 0;

}/*End arb_resetSysTime*/

void arb_restartSysTimer( void)
{
   arb_resetSysTime();

}/*End arb_restartSysTimer*/

t_error arb_sysTimerStart( void)
{
   t_timerError t_err;

   t_err = hal_startTimer( t_sysTimerHandle);

   if( t_err < 0)
      return ARB_HAL_ERROR;

   gb_sysTimerEnabled = true;
   gb_updateSysTime = true;

   return ARB_PASSED;

}/*End arb_sysTimerStart*/

void arb_sysTimerStop( void)
{

   hal_stopTimer( t_sysTimerHandle);
   gb_sysTimerEnabled = false;

}/*End arb_sysTimerStop*/

void arb_sysTimerInit( t_timerModId t_timerId)
{
   t_timerConfig t_config;

   /*------------------------------------------------------------------------*
    * Reset the system time
    *------------------------------------------------------------------------*/
   arb_resetSysTime();

   t_sysTimerHandle = hal_requestTimer( t_timerId);

   if( t_sysTimerHandle < 0)
   {
      exit(0);
   }/*End if( t_sysTimerHandle < 0)*/

   t_config.t_mode   = NORMAL;
   t_config.t_dir    = DIRECTION_UP;
   t_config.f_period = ARB_SYS_TIMER_PERIOD;

   if( hal_configureTimer( t_sysTimerHandle, t_config) < 0)
   {
      exit(0);
   }

   if( hal_requestTimerInterrupt( t_sysTimerHandle,
                                  OVERFLOW,
                                  &sysTimerInterrupt) < 0)
   {
      exit(0);
   }

}/*End arb_sysTimerInit*/
