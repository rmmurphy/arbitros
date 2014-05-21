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
 * File Name   : arb_thread.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for thread creation and control
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
#include <stdlib.h>
#include <string.h>
#include "arb_thread.h"
#include "arb_scheduler.h"
#include "utl_linkedlist.h"
#include "hal_pmic.h"
#include "hal_contextSwitch.h"
#include "hal_initThreadStack.h"

/*---------------------------------------------------------------------------*
 * Public Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This container contains a list of all the open threads running on the
 * system.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_activeThreads);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_tcb *gpt_temp;

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_threadInit( t_tcb **pt_newTcb,
                            void *pt_function,
                            t_parameters t_parms,
                            t_arguments t_args,
                            t_stackSize t_stack,
                            t_thrdPrio t_priority);

static void arb_yieldFromSleepIntsOn( void) __attribute__ ( ( naked, noinline ) );

static void arb_yieldFromSleepIntsOff( void) __attribute__ ( ( naked, noinline ) );

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void arb_yieldFromSleepIntsOn( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOn();

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

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
    * Change to a new thread.
    *------------------------------------------------------------------------*/
   gpt_scheduler();

   /*------------------------------------------------------------------------*
    * Restore the next thread's stack and CPU registers. This call must be
    * the last call made in the function since it modifies the CPU stack ptr.
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
    * and into the CPUs program counter where the last line of code the
    * thread was running at the time of preemption will be executed.
    *------------------------------------------------------------------------*/
	HAL_RET();

}/*End arb_yieldFromSleepIntsOn*/

static void arb_yieldFromSleepIntsOff( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOff();

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

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
    * Change to a new thread.
    *------------------------------------------------------------------------*/
   gpt_scheduler();

   /*------------------------------------------------------------------------*
    * Restore the next thread's stack and CPU registers. This call must be
    * the last call made in the function since it modifies the CPU stack ptr.
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
    * and into the CPUs program counter where the last line of code the
    * thread was running at the time of preemption will be executed.
    *------------------------------------------------------------------------*/
	HAL_RET();

}/*End arb_yieldFromSleepIntsOff*/

static void arb_threadInit( t_tcb **pt_newTcb,
                            void *pt_function,
                            t_parameters t_parms,
                            t_arguments t_args,
                            t_stackSize t_stack,
                            t_thrdPrio t_priority)
{
   uint8_t *pc_stackPtr = NULL;

   /*------------------------------------------------------------------------*
    * Initialize the stack ptr to the location of the buffer which starts
    * with the adjacent address location.
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->ps_stack = (uint16_t *)((&(*pt_newTcb)->ps_stack) + 1);

   /*------------------------------------------------------------------------*
    * Insert the overflow checksum at the start of the stack
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->ps_stack[0] = ARB_STACK_OVERFLOW_CHECK;

   /*------------------------------------------------------------------------*
    * Point this thread's stack pointer to the beginning of the memory segment
    *------------------------------------------------------------------------*/
   pc_stackPtr = (uint8_t *)&(*pt_newTcb)->ps_stack[0];

   /*------------------------------------------------------------------------*
    * Save the starting address of the stack
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->s_stackStart = (uint16_t)pc_stackPtr;

   /*------------------------------------------------------------------------*
    * Point to the last byte in the stack
    *------------------------------------------------------------------------*/
   pc_stackPtr = (pc_stackPtr + t_stack + ARB_EXTRA_STACK_BYTES) - 1;

   /*------------------------------------------------------------------------*
    * Save the ending address of the stack
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->s_stackEnd = (uint16_t)pc_stackPtr;

   /*------------------------------------------------------------------------*
    * Store the new thread's priority
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->t_priority = t_priority;

   /*------------------------------------------------------------------------*
    * Make sure the thread is ready to run
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->t_status = INITIALIZED;

   /*------------------------------------------------------------------------*
    * This thread is not sleeping
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->s_quantum = 0;

   /*------------------------------------------------------------------------*
    * Initialize the stack's CPU registers so that a context restore works
    * correctly the first time call the newly allocated thread.
    *------------------------------------------------------------------------*/
   hal_initThreadStack( pt_function,
                        t_parms,
                        t_args,
                        pc_stackPtr,
                        (*pt_newTcb)->s_stackEnd,
                        &(*pt_newTcb)->s_sP);

   /*------------------------------------------------------------------------*
    * Use this number as the thread identification number
    *------------------------------------------------------------------------*/
   (*pt_newTcb)->c_id = UTL_GET_NUM_LINKS_CONT( gt_activeThreads);

}/*End arb_threadInit*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void arb_updateSleepingThreads( void)
{

   t_tcb *pt_temp;
   uint16_t s_count;
   t_LINKHNDL t_linkHndl;

	/*------------------------------------------------------------------------*
	 * Starting with the head of the list, loop through all the threads
    * and update the ones that are sleeping.
	 *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_activeThreads, s_count)
   {
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_temp->t_status == SLEEPING)
      {
         pt_temp->s_quantum--;

         /*------------------------------------------------------------------*
          * Are we done with our nap?
          *------------------------------------------------------------------*/
         if( pt_temp->s_quantum <= 0)
         {
            pt_temp->t_status = READY;

         }/*End if( pt_temp->s_quantum <= 0)*/

      }/*End if( pt_temp->t_status == SLEEPING)*/
   }

}/*End arb_updateSleepingThreads*/

void arb_sleep( const uint16_t s_quantum)
{
   t_tcb *pt_temp;

   if( HAL_ARE_INTS_EN()) /*Interrupts enabled?*/
   {

      /*---------------------------------------------------------------------*
       * Mutual exclusion
       *---------------------------------------------------------------------*/
      HAL_CLI();

      /*---------------------------------------------------------------------*
       * Record the time to sleep. i.e. 1 quantum is 1 timer period.
       *---------------------------------------------------------------------*/
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
      pt_temp->s_quantum = s_quantum;
      pt_temp->t_status  = SLEEPING;

      /*---------------------------------------------------------------------*
       * Calling yield from within the sleep routine will cause everything up
       * to this point to be remembered and pushed onto the thread's stack.
       * This thread will give up its time slice and when it wakes the OS will
       * return to the next instruction after the 'yield' call.
       *---------------------------------------------------------------------*/
      arb_yieldFromSleepIntsOn(); /*Give up the current thread's time slice*/

   }/*End if( HAL_ARE_INTS_EN())*/
   else
   {
      /*---------------------------------------------------------------------*
       * Record the time to sleep. i.e. 1 quantum is 1 timer period.
       *---------------------------------------------------------------------*/
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
      pt_temp->s_quantum = s_quantum;
      pt_temp->t_status  = SLEEPING;

      /*---------------------------------------------------------------------*
       * Calling yield from within the sleep routine will cause everything up
       * to this point to be remembered and pushed onto the thread's stack.
       * This thread will give up its time slice and when it wakes the OS will
       * return to the next instruction after the 'yield' call.
       *---------------------------------------------------------------------*/
      arb_yieldFromSleepIntsOff(); /*Give up the current thread's time slice*/

   }

   /*------------------------------------------------------------------------*
    * If you have returned to this point you are operating on the stack of
    * the thread that initially made the call into sleep.
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Since the call into sleep wasn't 'naked' the previous thread's status
    * will be popped off the stack by the compiler upon exiting this
    * function.
    *------------------------------------------------------------------------*/

}/*End arb_sleep*/

t_THRDHANDLE arb_threadCreate( void *pt_function,
                               t_parameters t_parms,
                               t_arguments t_args,
                               t_stackSize t_stack,
                               t_thrdPrio t_priority)

{
   t_tcb *pt_temp;
   t_LINKHNDL t_linkHndl;
   uint16_t s_count;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   /*------------------------------------------------------------------------*
    * See if this thread has the same priority as another.
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_linkHndl, gt_activeThreads, s_count)
   {
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);
      if( pt_temp->t_priority == t_priority)
      {
         HAL_END_CRITICAL();
         return (t_THRDHANDLE)ARB_INVALID_PRIORITY;
      }
   }

   /*------------------------------------------------------------------------*
    * Create a new link handle with room at the end for the thread's stack...
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_tcb) + (uint16_t)t_stack +
   (uint16_t)ARB_EXTRA_STACK_BYTES);

   if( t_linkHndl < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_THRDHANDLE)ARB_OUT_OF_HEAP;
   }/*End if( t_linkHndl < 0)*/

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the thread is stored.
    *------------------------------------------------------------------------*/
   pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   /*------------------------------------------------------------------------*
    * Initialize information about this thread
    *------------------------------------------------------------------------*/
   arb_threadInit( &pt_temp,
                   pt_function,
                   t_parms,
                   t_args,
                   t_stack,
                   t_priority);

   /*---------------------------------------------------------------------*
    * Add the thread 'link' onto the list containing all the open threads
    * on the system.
    *---------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_activeThreads,
                           t_linkHndl,
                           true);

   HAL_END_CRITICAL();

   /*------------------------------------------------------------------------*
    * Return the 'link' to the allocated thread.
    *------------------------------------------------------------------------*/
   return (t_THRDHANDLE)t_linkHndl;

}/*End arb_threadCreate*/

/*---------------------------------------------------------------------------*
 * This function deletes a thread from memory
 *---------------------------------------------------------------------------*/
t_error arb_threadDestroy( t_THRDHANDLE t_thrdHandle)
{
   t_error t_err;

   /*------------------------------------------------------------------------*
    * Since we are about to act on global variables, protect this region
    * of code against higher priority threads interrupting us while we are
    * trying to register.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL();

   t_err = utl_destroyLink( gt_activeThreads,
                            (t_LINKHNDL)t_thrdHandle);

   if( t_err < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ARB_INVALID_HANDLE;
   }

   HAL_END_CRITICAL();//Enable interrupts
   return ARB_PASSED;

}/*End arb_threadDestroy*/
