/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *        This program is free software: you can redistribute it and/or modify
 *        it under the terms of the GNU General Public License as published by
 *        the Free Software Foundation, either version 3 of the License, or
 *        (at your option) any later version.
 *
 *        This program is distributed in the hope that it will be useful,
 *        but WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *        GNU General Public License for more details.
 *
 *        You should have received a copy of the GNU General Public License
 *        along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : arb_scheduler.c
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

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include "arb_thread.h"
#include "arb_scheduler.h"
#include "arb_idle.h"
#include "arb_sysTimer.h"
#include "arb_printf.h"
#include "utl_linkedlist.h"
#include "hal_pmic.h"
#include "hal_clocks.h"
#include "hal_contextSwitch.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/
#define ARB_LOAD_UPDATE_RATE ((uint16_t)ARB_TICKS_PER_SECOND*(uint16_t)5)
/*---------------------------------------------------------------------------*
 * alpha = 1 - Tupdate/Tavr, where Tavr = 60 sec and Tupdate = 5sec
 *---------------------------------------------------------------------------*/
#define ARB_LOAD_ONE_MIN_ALPHA (uint16_t)((1.0f - 5.0f/60.0f)*\
(float)ARB_LOAD_EST_ONE)
/*---------------------------------------------------------------------------*
 * alpha = 1 - Tupdate/Tavr, where Tavr = 300 sec and Tupdate = 5sec
 *---------------------------------------------------------------------------*/
#define ARB_LOAD_FIVE_MIN_ALPHA (uint16_t)((1.0f - 5.0f/300.0f)*\
(float)ARB_LOAD_EST_ONE)

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * If true, the scheduler is enabled
    *------------------------------------------------------------------------*/
   bool b_enableScheduler;
   /*------------------------------------------------------------------------*
    * When the count reaches ARB_LOAD_UPDATE_RATE, the loading estimate is
    * updated.
    *------------------------------------------------------------------------*/
   uint16_t s_loadUpdateCount;
   /*------------------------------------------------------------------------*
    * The average system loading where index 0 represents a moving average of
    * one minute and index 1 represents a moving average of 5 minutes.
    * Q21.11 number
    *------------------------------------------------------------------------*/
   uint32_t ai_loading[2];
   /*------------------------------------------------------------------------*
    * This variable keeps track of the total number of active threads during
    * the period 'ARB_LOAD_UPDATE_RATE'.
    *------------------------------------------------------------------------*/
   uint32_t i_activeCount;

}t_schedObject;

/*---------------------------------------------------------------------------*
 * Public Global variables
 *---------------------------------------------------------------------------*/
void (*gpt_scheduler)(void);
t_tcb *gpt_activeThread = NULL; /*Current thread running on the system.*/

/*---------------------------------------------------------------------------*
 * Private Global variables
 *---------------------------------------------------------------------------*/
static t_schedObject gt_schedObject;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_roundRobinScheduler( void);

static void arb_priorityScheduler( void);

static inline void __attribute__((always_inline)) arb_updateLoadingEst( void);

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
t_error arb_schedulerInit( t_schedulerType t_schedType,
                           t_stackSize t_idlStack,
                           t_timerModId t_timerId)
{
   t_THRDHANDLE t_idleThrdHndl;

   /*------------------------------------------------------------------------*
    * Scheduler is not enabled until 'arb_schedulerStart' gets called.
    *------------------------------------------------------------------------*/
   gt_schedObject.b_enableScheduler = false;
   gt_schedObject.ai_loading[0] = 0;
   gt_schedObject.ai_loading[1] = 0;
   gt_schedObject.s_loadUpdateCount = 0;
   gt_schedObject.i_activeCount = 0;

   /*------------------------------------------------------------------------*
    * Global Interrupt Disable
    *------------------------------------------------------------------------*/
   HAL_CLI();

   /*------------------------------------------------------------------------*
    * Configure OS system timer for a period of 10msec
    *------------------------------------------------------------------------*/
   arb_sysTimerInit( t_timerId);

   /*------------------------------------------------------------------------*
    * Create the idle thread.
    *------------------------------------------------------------------------*/
   t_idleThrdHndl = arb_idleInit( t_idlStack, ARB_IDLE_PRIORITY);
   if( t_idleThrdHndl < 0)
   {
      arb_printf( PRINTF_DBG_HIGH,
                  "Idle thread failed init.\n\r");

      return (t_error)t_idleThrdHndl;
   }

   /*------------------------------------------------------------------------*
    * Make sure the idle thread is the first thread the scheduler runs.
    *------------------------------------------------------------------------*/
   UTL_SET_CURR_OF_CONT( gt_activeThreads, t_idleThrdHndl);

   /*------------------------------------------------------------------------*
    * Point the function pointer to the correct scheduler
    *------------------------------------------------------------------------*/
   switch( t_schedType)
   {
      case ROUNDROBIN:

         gpt_scheduler = &arb_roundRobinScheduler;

      break;/*End  case ROUNDROBIN:*/

      case PRIORITY:

         gpt_scheduler = &arb_priorityScheduler;

      break;/*End case PRIORITY:*/

      default:

         gpt_scheduler = &arb_roundRobinScheduler;

      break;/*End default:*/

   }/*End switch( t_schedType)*/

   arb_printf( PRINTF_DBG_HIGH,
               "Scheduler initialized\n\r");

   return ARB_PASSED;

}/*End arb_schedulerInit*/

void arb_schedulerStart( void)
{
   static t_tcb *pt_temp;
   t_error t_err;
   char ac_buff[50];

   /*------------------------------------------------------------------------*
    * Enable the scheduler
    *------------------------------------------------------------------------*/
   gt_schedObject.b_enableScheduler = true;

   /*------------------------------------------------------------------------*
    * Start the system timer
    *------------------------------------------------------------------------*/
   t_err = arb_sysTimerStart();
   if( t_err < 0)
   {
      sprintf_P((char *)ac_buff, PSTR("arb_sysTimerStart failed with %d.\n\r"), t_err);

      arb_printf( PRINTF_DBG_HIGH,
                  ac_buff);

      exit(0);

   }/*End if( arb_sysTimerStart() < 0)*/

   /*------------------------------------------------------------------------*
    * Change the status of this thread
    *------------------------------------------------------------------------*/
   pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
   pt_temp->t_status = RUNNING;

   /*------------------------------------------------------------------------*
    * Disable debug...
    *------------------------------------------------------------------------*/
   arb_setPrintfDbgLevel( PRINTF_DBG_OFF);

   /*------------------------------------------------------------------------*
    * Wait one second before launching first thread.
    *------------------------------------------------------------------------*/
   hal_busyDelayMs( 1000, hal_getCpuFreq());

   /*------------------------------------------------------------------------*
    * Get ready to start the IDLE thread - !!!This operation will enable
    * global interrupts!!!
    *------------------------------------------------------------------------*/
   hal_contextRestore();

   /*------------------------------------------------------------------------*
    * After performing the context restore the stack ptr is pointing to the
    * location where the thread's program counter last left off. Issuing a
    * 'ret' or 'reti' cmd will cause this location to be popped off the stack
    * and into the CPUs program counter where the last line of code the
    * thread was running at the time of preemption will be executed.
    *------------------------------------------------------------------------*/
   HAL_RET();

}/*End arb_schedulerStart*/

uint32_t arb_getOneMinLoadingEst( void)
{
   return gt_schedObject.ai_loading[0];
}/*End arb_getOneMinLoadingEst*/

uint32_t arb_getFiveMinLoadingEst( void)
{
   return gt_schedObject.ai_loading[1];
}/*End arb_getFiveMinLoadingEst*/

void arb_updateLoadingEst( void)
{
   t_LINKHNDL t_curr;
   uint16_t s_count;
   t_tcb *pt_temp;
   uint32_t i_temp1 = 0;
   uint32_t i_temp2 = 0;
   uint32_t i_load;

   /*------------------------------------------------------------------------*
    * Estimate the average system loading following the scheme given in
    * the function 'calc_load' defined in the Linux kernel source code. For
    * more information reference
    * http://www.linuxjournal.com/article/9001?page=0,1
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Count all the active and ready threads for loading analysis-except for
    * the idle thread.
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gt_activeThreads, s_count)
   {
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);
      if( ((pt_temp->t_status == READY) ||
           (pt_temp->t_status == RUNNING)) &&
           (pt_temp->t_priority != ARB_IDLE_PRIORITY))
         gt_schedObject.i_activeCount++;
   }

   gt_schedObject.s_loadUpdateCount++;
   if( gt_schedObject.s_loadUpdateCount == ARB_LOAD_UPDATE_RATE)
   {

      i_load = (gt_schedObject.i_activeCount*(uint32_t)ARB_LOAD_EST_ONE) /
      ARB_LOAD_UPDATE_RATE;

      gt_schedObject.i_activeCount = 0;
      gt_schedObject.s_loadUpdateCount = 0;

      i_temp1 = (uint32_t)gt_schedObject.ai_loading[0]*
      (uint32_t)ARB_LOAD_ONE_MIN_ALPHA;
      i_temp2 = ((uint32_t)ARB_LOAD_EST_ONE -
      (uint32_t)ARB_LOAD_ONE_MIN_ALPHA)*i_load;

      gt_schedObject.ai_loading[0] = (uint32_t)(((uint64_t)i_temp1 +
      (uint64_t)i_temp2 + (uint64_t)(ARB_LOAD_EST_ONE >> 1)) >>
      ARB_LOAD_EST_Q_FACT);

      i_temp1 = (uint32_t)gt_schedObject.ai_loading[1]*
      (uint32_t)ARB_LOAD_FIVE_MIN_ALPHA;
      i_temp2 = ((uint32_t)ARB_LOAD_EST_ONE -
      (uint32_t)ARB_LOAD_FIVE_MIN_ALPHA)*i_load;

      gt_schedObject.ai_loading[1] = (uint32_t)(((uint64_t)i_temp1 +
      (uint64_t)i_temp2 + (uint64_t)(ARB_LOAD_EST_ONE >> 1)) >>
       ARB_LOAD_EST_Q_FACT);

   }/*End if( gt_schedObject.s_loadUpdateCount == ARB_LOAD_UPDATE_RATE)*/

}/*End arb_updateLoadingEst*/

/*---------------------------------------------------------------------------*
 * Round-robin scheduler
 *---------------------------------------------------------------------------*/
static void arb_roundRobinScheduler( void)
{
   t_LINKHNDL t_curr = UTL_GET_CURR_OF_CONT( gt_activeThreads);
   t_tcb *pt_temp;

   if( gt_schedObject.b_enableScheduler == true)
   {
      /*---------------------------------------------------------------------*
       * Sequentially search for the next thread that isn't sleeping
       *---------------------------------------------------------------------*/
      do
      {
         t_curr  = UTL_GET_NEXT_LINK( t_curr);
         pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);

      }while( pt_temp->t_status == SLEEPING);

      /*---------------------------------------------------------------------*
       * Set the location of the new 'current' thread
       *---------------------------------------------------------------------*/
      UTL_SET_CURR_OF_CONT( gt_activeThreads, t_curr);

      gpt_activeThread = pt_temp;

      /*---------------------------------------------------------------------*
       * Change the status of this new thread to RUNNING
       *---------------------------------------------------------------------*/
      pt_temp->t_status = RUNNING;

      /*---------------------------------------------------------------------*
       * Estimate the thread loading
       *---------------------------------------------------------------------*/
      arb_updateLoadingEst();

   }/*End if( gt_schedObject.b_enableScheduler == true)*/

}/*End arb_roundRobinScheduler*/

/*---------------------------------------------------------------------------*
 * Priority based scheduler
 *---------------------------------------------------------------------------*/
static void arb_priorityScheduler( void)
{
   t_LINKHNDL t_curr;
   t_LINKHNDL t_highest;
   uint16_t s_highest;
   uint16_t s_count;
   t_tcb *pt_temp;

   if( gt_schedObject.b_enableScheduler == true)
   {

      /*---------------------------------------------------------------------*
       * Find the thread with the highest priority that isn't sleeping.
       *---------------------------------------------------------------------*/
      t_highest = UTL_GET_HEAD_OF_CONT( gt_activeThreads);
      pt_temp   = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
      s_highest = pt_temp->t_priority;

      UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gt_activeThreads, s_count)
      {
         pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);
         if( (pt_temp->t_priority < s_highest) && (pt_temp->t_status !=
         SLEEPING))
         {
            s_highest = pt_temp->t_priority;
            t_highest = t_curr;
         }

      }

      /*---------------------------------------------------------------------*
       * Set the next thread to run to the one with the highest priority
       *---------------------------------------------------------------------*/
      UTL_SET_CURR_OF_CONT( gt_activeThreads, t_highest);

      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);

      gpt_activeThread = pt_temp;

      /*---------------------------------------------------------------------*
       * Change the status of this new thread to RUNNING
       *---------------------------------------------------------------------*/
      pt_temp->t_status = RUNNING;

      /*---------------------------------------------------------------------*
       * Estimate the thread loading
       *---------------------------------------------------------------------*/
      arb_updateLoadingEst();

   }/*End if( gt_schedObject.b_enableScheduler == true)*/

}/*End arb_priorityScheduler*/
